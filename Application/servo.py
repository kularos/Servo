import serial
import time
from SerailComms import USBDevice


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
my_ports = ['/dev/cu.usbmodem14101', '/dev/cu.usbmodem14201']


# Common Command definitions for all serial servos comprise commands [0,7]
# Specific commands exist for [8, 15].
# Total 4 bits (16) possible commands for the servo interface

OPEN = b"\x3F"  # ?
CLOSE = b"\x40"  # @
RESET = b"\x41"  # A
GET_SENSE = b"\x45"  # E
SET_CONTROL = b"\x46"  # F

NO_ERROR = b"\x47"

# Commands that are specifically used for servos


servo_error_codes = {b"\x47": "noError",  # G
                     b"\x48": "unknownCommandError",  # H
                     b"\x49": "sensorError",  # I
                     b"\x4A": "CommandExecutionError"}  # J


class SerialServo(Servo, USBDevice):
    # Device definitions.
    IDENTIFIER = None





    def __init__(self, sense_bytes, control_bytes, debug=False):

        # Setup sense vector
        self._sense_bytes = sense_bytes
        self.sense_vec = [0 for _ in range(len(sense_bytes))]

        # Setup control vector
        self._control_bytes = control_bytes
        self.control_vec = [0 for _ in range(len(control_bytes))]

        Servo.__init__(self)


    # --- Interface Compliance ---#
    def open(self):
        """For a servo that connects over serial, opening the servo is equivalent to forming the serial connection."""
        self._send_command("Open")

    def get_id(self):
        return self._send_command("GetID")

    def close(self):
        """To close, terminate the serial connection on both ends."""
        self._send_command("Close")
        self._ser.close()

    def reset(self):
        """Reset connected servo to a well-defined base state."""
        self._send_command("Reset")

    def get_sense(self):
        """Request update of sense vector over serial and calculate normalized values."""
        # Propagate command to get sense downstream
        raw_data = self._send_command("GetSense")

        i = 0
        for j in range(len(self.sense_vec)):
            # Read number of bytes in current vector entry:
            num_bytes = self._sense_bytes[j]

            # Slice the bytes that comprise current vector entry:
            sense_i_data = raw_data[i: i + num_bytes]
            i += num_bytes

            # Decode byte array to a normalized float between 0 and 1.
            value_range = (1 << (8 * num_bytes)) - 1
            self.sense_vec[j] = int.from_bytes(sense_i_data, "little") / value_range

        return self.sense_vec

    def set_control(self, control_vec):

        raw_data = bytes()

        for i in range(len(self.control_vec)):
            # Read number of bytes in current vector entry:
            num_bytes = self._sense_bytes[i]

            # Normalize float to unsigned integer range:
            value_range = (1 << (8 * num_bytes)) - 1
            control_i_value = int(control_vec[i] * value_range)

            # append value's bytes to raw_data
            raw_data += control_i_value.to_bytes(num_bytes, "little")

        # Send command to set control on device.
        self._send_command(SET_CONTROL, data=raw_data)


class rhServo(SerialServo):
    IDENTIFIER = b"\x49\x44"

    def __init__(self, debug):
        super().__init__((2, 2), (1, 1), debug)
