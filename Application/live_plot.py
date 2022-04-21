import matplotlib.pyplot as plt
import numpy as np
import collections
from Core import LiveTerminal, BufferedParameter

from time import sleep


class Plotter(LiveTerminal):
    buffer_size = 200
    x = BufferedParameter(size=buffer_size)

    def __init__(self):

        # Initialize pyplot figure and axes
        plt.ion()
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111)
        self.line = self._ax.plot(np.array(self.t_), np.array(self.x_))[0]

        # xlims are dynamically set
        plt.xlim([0, self.dt * (self.buffer_size - 1)])
        plt.ylim([0, 100])
        plt.show()

        super(Plotter, self).__init__()

    def _update(self):
        self.line.set_xdata(np.array(self.t_))
        self.line.set_ydata(np.array(self.x_))
        plt.show()
        super(Plotter, self)._update()

    def add_value(self, value):
        self._y.append(value)
        self._update()


if __name__ == '__main__':
    term = Plotter()

    while True:
        print(term("get, t_"))
        sleep(0.5)