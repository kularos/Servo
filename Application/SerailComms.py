from abc import abstractmethod

import serial


class SerialConnectionManager:

    def __init__(self, debug):
        self._debug = debug

    def _connect_serial(self, identifier, ports, baudrate=9600, timeout=1):
        """Scan all available serial ports to find one which passes device validation."""
        self._ser = None

        # Query through all defined ports.
        for port in ports:
            # Attempt to connect to  device at 'port' and validate it.
            try:
                self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

            # If a device exists on the port, but does not respond to the validation attempt:
            except serial.SerialTimeoutException:
                if self._debug:
                    print('Device found on port \'{}\', but timed out after {} seconds'.format(port, timeout))

            # Manage other serial errors:
            except serial.SerialException as err:
                if self._debug:
                    print("Error encountered during connection: {}".format(str(err)))

            # Validation.

            # Open the connected servo and see if it returns the correct identifier:
            try:
                found_identifier = self._send_command("GetID")

                # If a proper connection was made, but the identifier is incorrect:
                if found_identifier != identifier:
                    if self._debug:
                        print('Found device with address {}, not {}'.format(found_identifier, identifier))

                    # Close serial connection and reset ser.
                    self._send_command("GetID")
                    ser.close()
                    ser = None

            # If device timed out during validation:
            except serial.SerialTimeoutException:
                if self._debug:
                    print('Device found on port: {}, but failed to provide an identifier.'.format(port))

                # Reset ser.
                self._ser = None

            # Manage other serial errors:
            except serial.SerialException as err:
                if self._debug:
                    print("Error encountered during validation: {}".format(str(err)))

                # Reset ser.
                self._ser = None

        # Raise an error if device wasn't found
        if self._ser is None:
            raise serial.SerialException("Could not find the desired device on any defined port.")


class SerialServo:
    """Manages communication with a USB slave device. Command sequences are sent, and the response parsed."""

    def __init__(self, port, commands, errors, debug=False):

        # Initialize serial connection on provided port:
        self._ser = serial.Serial(port, baudrate=9600, timeout=1)

        # Set commands
        self._commands = commands

        # Add any user-provided errors and initialize a dict for looking up error byte strings:
        self._errors = errors
        self._error_bytes = {value: key for (key, value) in self._errors.items()}

        # Set flag to enable debugging over the console:
        self._debug = debug

    def _send_command(self, command, data=bytes()):
        """Given a command string and bytes of data, send the command over serial and return the response."""

        # Structure of command from self._commands
        command_byte, n_data_bytes, n_response_bytes = self._commands[command]

        # Debug an anticipated DataIndex error:
        if len(data) != n_data_bytes and self._debug:
            print("Anticipated DataIndexError in command: \"{}\". "
                  "Sending {} bytes of data when {} are expected.".format(command, len(data), n_data_bytes))

        # Format byte string and send over serial:
        outgoing_bytes = command_byte + data
        self._ser.write(outgoing_bytes)

        # Try to read the incoming response, and handle a timeout if it occurs:
        error, response = self._errors["noError"], bytes()  # No error received means no error to debug.
        try:
            incoming_bytes = self._ser.read(n_response_bytes)
            error, response = incoming_bytes[0], incoming_bytes[1:-1]

        except serial.SerialTimeoutException:
            if self._debug:
                print("Response timeout encountered during command: \"{}\"".format(command))

        # Debug the provided error code:
        if error != self._errors["noError"] and self._debug:
            error_string = self._error_bytes[error]
            print("\"{}\" error encountered during command \"{}\".".format(error_string, command))

        # Return the bytes of response:
        return response

    def _receive_response(self):
        pass
