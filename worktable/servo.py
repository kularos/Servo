import serial
import time

# --- SERIAL INTERFACE AND DEFINITIONS --- #
my_ports = ['/dev/cu.usbmodem14101', '/dev/cu.usbmodem14201']

OPEN = 0
CLOSE = 1
RESET = 2

GET_ID = 3
GET_INFO = 4
SET_CAL = 5

GET_SNS = 6
SET_CTL = 7


# --- SERVO CLASS AND INTERFACE DEFINITIONS --- #
class Servo:

    def open(self, *args, **kwargs):
        """Opens communication with downstream nodes."""
        return NotImplemented

    def close(self, *args, **kwargs):
        """Terminates communication with downstream nodes."""
        return NotImplemented

    def get_sense(self, *args, **kwargs):
        """Reads and returns the device's sense signal"""
        return NotImplemented

    def set_control(self, *args, **kwargs):
        """Sets the device's control signal."""
        return NotImplemented


class SerialServo(Servo):
    # Device definitions.
    IDENTIFIER = None

    # Common Command definitions for all serial servos comprise commands [0,7]
    # Specific commands exist for [8, 15].
    # Total 4 bits (15) possible commands for the servo interface

    def __init__(self):
        self.ser = None
        self.t0 = None

    def connect_serial(self, debug=False, baudrate=9600, timeout=1):
        # Query through all defined ports.
        for port in my_ports:
            # Attempt to connect to  device at 'port' and validate it.
            try:
                self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
                self.validate_identifier()

            # If a device exists on the port, but does not respond to the validation attempt:
            except serial.SerialTimeoutException:
                if debug:
                    print('Device found on port \'{}\', but timed out after {} seconds'.format(port, timeout))

            # If no device can be found on the provided port:
            except serial.SerialException:
                if debug:
                    print('No device found on port: {}.'.format(port))

            # If a device is found on the port and responds, but fails the validation:
            except ValueError:
                self.close()
                self.ser = None

        if self.ser is None:
            raise serial.SerialException("Could not find the desired device on any defined port.")

        # Reset local timer.
        self.t0 = time.time()

    def validate_identifier(self):
        """Reads the device's 16-bit identification code, and throws a ValueError if it does not match."""
        if self.IDENTIFIER is None:
            raise NotImplementedError("Device identifier must be specified prior to Identification.")

        # Write the "GET_ID" command to serial, and wait for the response.
        self.write_serial(GET_ID)
        found_identifier = int.from_bytes(self.ser.read(2), byteorder='big')

        # Compare the received identifier to the device's
        if found_identifier != self.IDENTIFIER:
            raise ValueError('Found device with address {}, not {}'.format(found_identifier, self.IDENTIFIER))

    def write_serial(self, command, data=None):
        """Given a 4-bit command, and 12 bits of data, write 16-bit message over serial."""
        output = (command << 12)

        if data is not None:
            output += data

        self.ser.write(output.to_bytes(2, byteorder='big'))

    # --- Interface Compliance ---#
    def open(self):
        """For a servo that connects over serial, opening the servo is equivalent to forming the serial connection."""
        self.connect_serial()
        self.write_serial(OPEN)

    def close(self):
        """To close, terminate the serial connection on both ends."""
        self.write_serial(CLOSE)
        self.ser.close()

    def get_sense(self, *args, **kwargs):
        self.write_serial(GET_SNS)

        newline = self.ser.readline().decode()
        raw_data = newline.split('\t')

        data = [0, 0, 0]
        if len(raw_data) == 3:
            try:
                data = [time.time() - self.t0, float(raw_data[1]), float(raw_data[2])]
            except ValueError:
                pass

        error = raw_data[0]
        if error != '0':
            print("Error code: ", error)

        return data


class rhServo(SerialServo):
    IDENTIFIER = 18756

    def set_control(self, pump_1, pump_2):
        """Formats the 2-byte serial code to set each pump to the desired % activation."""

        # each pump gets a 6-bit control signal, max is (2**6-1) or 63.
        signal_1 = int(pump_1 * 63)
        signal_2 = int(pump_2 * 63)

        # Concatenate both control signals, and write SET_PUMP command.
        self.write_serial(SET_CTL, (signal_1 << 6) + signal_2)


