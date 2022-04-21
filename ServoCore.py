
class ErrorCode:
    def __init__(self, bytecode, error_name, description, address=None):
        self.bytecode = bytecode
        self.error_name = error_name
        self.description = description
        self.address = address

    def __eq__(self, other):
        """Error codes can evaluate based upon their bytecode or name. """

        if isinstance(other, (bytes, bytearray)):
            return self.bytecode == other

        elif isinstance(other, str):
            return self.error_name == other

    def __repr__(self):
        return self.error_name + " at " + str(self.address.hex())

    def __str__(self):
        return self.__repr__() + ": " + self.description


def recursive_call(func):
    """This function wrapper recursively calls a tree."""

    def wrapper(instance, *args, **kwargs):

        # Error lists for each child comprise the parent error list
        errors = func(instance, *args, **kwargs)
        for child_node in instance.child_nodes:

            # For recursion, call the child's implementation of func.
            child_func = getattr(child_node, func.__name__)
            child_errors = child_func(*args, **kwargs)

            # Prepend the error's child error's address with the parent's to get the global address.
            for error in child_errors:
                error.address = instance.address + error.address

            # Extend parent error list by child errors and return.
            errors.extend(child_errors)

        return errors

    return wrapper


class ServoNode:

    def __repr__(self):
        return type(self).__name__ + " with ID: " + str(self.address)

    def __init__(self, child_nodes, address):

        self.child_nodes = child_nodes
        self._nChildNodes = len(child_nodes)

        self.address = address

    def calc_dims(self):
        """ For the time being, Servos will have the total dimensionality of their child nodes.
        This function recursively calls all child nodes to determine a node's dimensionality.
        """
        sense_dim, control_dim = 0, 0

        for child_node in self.child_nodes:
            child_sense, child_control = child_node.calc_dims()
            sense_dim += child_sense
            control_dim += child_control

        return sense_dim, control_dim

    @recursive_call
    def _open(self):
        """Function which opens (initializes) this servoNode.
        For an abstract node to be opened, it just needs to ensure all of its children are opened."""
        return [ErrorCode(b"x\00", "NODE", "", address=self.address)]

    @recursive_call
    def _close(self):
        """Function which closes (de-initializes) this servoNode.
        For an abstract node to be closed, it just needs to ensure all of its children are opened."""

    @recursive_call
    def _get_sense(self):
        """Function which tells Sensors to read values from the environment and write to the sense vec at the head."""

    @recursive_call
    def _set_control(self):
        """Function which tells Actuators to read values from the environment and write to the sense vec at the head."""


class ServoHead(ServoNode):

    def find_physical_servos(self):

        def rec(node):
            if isinstance(node, Sensor):
                sensors.append(node)

            elif isinstance(node, Actuator):
                actuators.append(node)

            else:
                for child_node in node.child_nodes:
                    rec(child_node)

        sensors, actuators = [], []
        rec(self)

        return sensors, actuators


class Sensor(ServoNode):

    def __init__(self, address, sense_dim=1):
        self.sense_dim = sense_dim

        super().__init__([], address)

    def _open(self):
        return [ErrorCode(b"x\00", "SENSOR", "", address=self.address)]


class Actuator(ServoNode):

    def __init__(self, address, control_dim=1):
        self.control_dim = control_dim

        super().__init__([], address)

    pass


if __name__ == '__main__':
    A = Actuator(b"\x01")
    C = Sensor(b"\x03")
    E = Actuator(b"\x05")
    H = Sensor(b"\x08")

    I = ServoNode([H], b"\x09")
    G = ServoNode([I], b"\x07")
    D = ServoNode([C, E], b"\x04")
    B = ServoNode([A, D], b"\x02")
    F = ServoHead([B, G], b"\x06")

    print(F.find_physical_servos())