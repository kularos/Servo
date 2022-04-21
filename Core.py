import numpy as np
from math import sin
from time import time, sleep
from collections import deque


class BufferedParameter(float):

    def __init__(self, initial_value=None, size=4):
        """Descriptor class for managing a stream of information. """

        self._size = size
        self._buffer = deque([float("nan") for _ in range(size)], maxlen=size)

        if initial_value is not None:
            self.__set__(self, initial_value)

    @property
    def buffer(self):
        return self._buffer

    def __set_name__(self, owner, name):
        setattr(owner, name + "_", self._buffer)
        setattr(owner, "_{}_size".format(name), self._buffer.maxlen)

    def __get__(self, instance, owner):
        return self._buffer[0]

    def __set__(self, instance, value):
        self._buffer.appendleft(value)

    def __str__(self):
        return str(self._buffer[0])


class Terminal:

    def __init__(self):
        """The Terminal is a Base class that allows streaming of data between multiple sources. """

        self._attributes = [attribute for attribute in self.__dict__.keys() if attribute[0] != "_"]
        self._methods = [method for method in dir(self) if callable(getattr(self, method)) and method[0] != "_"]

    def __call__(self, cmd, *args):

        if isinstance(cmd, int):
            cmd = self._methods[cmd]

        if cmd not in self._methods:
            cmd, *args = self._parse_command(cmd)

        return getattr(self, cmd).__call__(*args)

    def _parse_command(self, cmd, delim=","):
        if isinstance(cmd, str):
            args = [arg.strip() for arg in cmd.split(delim)]

        elif isinstance(cmd, bytes):
            return NotImplemented

        else:
            raise TypeError("Unknown type \"{}\" for Terminal command parsing.".format(type(cmd)))

        return args

    def get(self, attribute):
        return getattr(self, attribute)

    def set(self, attribute, value):
        setattr(self, attribute, value)


class LiveTerminal(Terminal):
    buffer_size = 20
    t = BufferedParameter(size=buffer_size)
    dt = BufferedParameter(size=buffer_size)

    def __init__(self):

        # State flag:
        self._old = True
        self._t0 = time()

        super(LiveTerminal, self).__init__()

    def __call__(self, *args, **kwargs):
        if self._old: self._update()
        self._old = True

        return super().__call__(*args, **kwargs)

    def _update(self):

        current_time = time() - self._t0
        self.dt = current_time - self.t
        self.t = current_time

        self._old = False


if __name__ == '__main__':

    term = LiveTerminal()

    while True:
        print(term("get, t_"))
        sleep(0.5)