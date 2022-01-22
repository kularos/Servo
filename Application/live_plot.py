import matplotlib.pyplot as plt
import numpy as np
import collections


class Plotter:
    def __init__(self):
        # set params for internal buffer
        self.buffer_size = 500
        self.dt = 0.05

        # initialize buffers
        self._x = np.linspace(0, self.dt * (self.buffer_size - 1), self.buffer_size)
        self._y = collections.deque(np.NaN * np.zeros(self.buffer_size), maxlen=self.buffer_size)

        # Initialize pyplot figure and axes
        plt.ion()
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111)
        self.line = self._ax.plot(self._x, np.array(self._y))[0]

        # xlims are dynamically set
        plt.xlim([0, self.dt * (self.buffer_size - 1)])
        plt.ylim([0, 100])
        plt.show()

    def _update(self):
        self.line.set_ydata(np.array(self._y))
        plt.pause(self.dt / 2)

    def add_value(self, value):
        self._y.append(value)
        self._update()