import serial
import time

DEBUG = True


# --- SERVO CLASS AND INTERFACE DEFINITIONS --- #
class Servo:

    # outline the properties that should be used in each servo
    def __init__(self):
        self._downstream_nodes = list()

        self.control_vec_size = tuple()
        self.sense_vec_size = tuple()

    # Interface for Protocol 1 (Building Servo pipeline)
    def open(self, *args, **kwargs):
        """Opens communication with downstream nodes. Also sets to a defined OPEN state."""
        for node in self._downstream_nodes:
            node.open()
        return NotImplemented

    def close(self, *args, **kwargs):
        """Terminates communication with downstream nodes. Sets to a defined CLOSED state."""
        for node in self._downstream_nodes:
            node.close()
        return NotImplemented

    def reset(self, *args, **kwargs):
        """Reset the internal state, i.e. re-initialize"""
        # This shouldn't affect the pipeline.
        for node in self._downstream_nodes:
            node.open()
        return NotImplemented

    def connect_downstream_node(self, node):
        """Add a new node to the downstream"""
        self._downstream_nodes.append(node)

    # Interface for Protocol 2 (Control loop)
    def get_sense(self, *args, **kwargs):
        """Reads and returns the device's sense signal"""
        return NotImplemented

    def set_control(self, *args, **kwargs):
        """Sets the device's control signal."""
        return NotImplemented

    def _feed_forward(self, *args, **kwargs):
        """Converts the control vector of an upstream node to the downstream node."""
        return NotImplemented

    def _feed_back(self, *args, **kwargs):
        """Converts the sense vector of a downstream node to the upstream node."""
        return NotImplemented

    # Interface 3 (setting & calibrating servo)
        # TODO Implement


# --- SERIAL INTERFACE AND DEFINITIONS --- #
#my_ports = ['/dev/cu.usbmodem14101', '/dev/cu.usbmodem14201']
my_ports = ['COM3', 'COM5']

# Common Command definitions for all serial servos comprise commands [0,7]
# Specific commands exist for [8, 15].
# Total 4 bits (16) possible commands for the servo interface

OPEN = 0
CLOSE = 1
RESET = 2

GET_ID = 3
GET_INFO = 4
SET_CAL = 5

GET_SNS = 6
SET_CTL = 7


def write_serial(ser, command, data=None):
    """Given a 4-bit command, and 12 bits of data, write 16-bit message over serial."""
    output = (command << 12)

    if data is not None:
        output += data

    ser.write(output.to_bytes(2, byteorder='big'))


def validate_identifier(ser, identifier):
    """Reads the device's 16-bit identification code, and throws a ValueError if it does not match."""

    # Write the "GET_ID" command to serial, and wait for the response.
    write_serial(ser, GET_ID)
    found_identifier = int.from_bytes(ser.read(2), byteorder='big')

    if DEBUG:
        print(found_identifier)

    # Compare the received identifier to the device's
    if found_identifier != identifier:
        raise ValueError('Found device with address {}, not {}'.format(found_identifier, identifier))


def connect_serial(identifier, debug=DEBUG, baudrate=9600, timeout=1):
    """Scan all available serial ports to find one which passes device validation."""
    ser = None

    # Query through all defined ports.
    for port in my_ports:
        # Attempt to connect to  device at 'port' and validate it.
        try:
            ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            validate_identifier(ser, identifier)

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
            write_serial(ser, CLOSE)
            ser = None

    if ser is None:
        raise serial.SerialException("Could not find device {0} on any defined port.".format(identifier))


class SerialServo(Servo):
    # Device definitions.
    IDENTIFIER = None
    CTL_BITS = []
    SNS_BITS = []

    def __init__(self):
        self._ser = connect_serial(self.IDENTIFIER)
        super().__init__()

    # --- Interface Compliance ---#
    def open(self):
        """For a servo that connects over serial, opening the servo is equivalent to forming the serial connection."""
        write_serial(self._ser, OPEN)

    def close(self):
        """To close, terminate the serial connection on both ends."""
        write_serial(self._ser, CLOSE)
        self._ser.close()

    def get_sense(self, *args, **kwargs):
        # Propagate command to get sense downstream
        write_serial(self._ser, GET_SNS)

        # load in the response as a bytestream.
        bytestream = self._ser.read()

        newline = self._ser.readline()
        raw_data = newline.decode().split('\t')

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
    IDENTIFIER = int.from_bytes(b"\x49\x44", "big")

    N_CMD_BITS = 4
    N_ERR_BITS = 4

    # Control bytestream consists of each vector element, prepended by the command to set control.
    N_CTL_BITS = [6, 5]
    N_CTL_BYTES = round((N_CMD_BITS + sum(N_CTL_BITS)) / 8 + 0.5)

    # Control bytestream consists of each vector element, prepended by the command to set control.
    SNS_BITS = [6, 6]
    SNS_BYTES = -int(-(4 + sum(SNS_BITS)) / 8)

    def set_control(self, pump_1, pump_2):
        """Formats the 2-byte serial code to set each pump to the desired % activation."""

        # each pump gets a 6-bit control signal, max is (2**6-1) or 63.
        signal_1 = int(pump_1 * 63)
        signal_2 = int(pump_2 * 63)

        # Concatenate both control signals, and write SET_PUMP command.
        self._write_serial(SET_CTL, (signal_1 << 6) + signal_2)

    def _write_serial(self, command, data):
        write_serial(self._ser, command, data)