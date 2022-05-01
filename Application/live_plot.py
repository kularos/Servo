import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
from Core import LiveTerminal, BufferedParameter
from time import sleep


class Plotter(LiveTerminal):
    buffer_size = 2000
    t = BufferedParameter(size=buffer_size)
    x = BufferedParameter(dim=1, size=buffer_size)

    def __init__(self):

        # Initialize pyplot figure and axes

        self._fig, self._ax = plt.subplots(1, 1)
        self.line = self._ax.plot(np.array(self.t_), np.array(self.x_))
        # xlims are dynamically set
        plt.xlim([0, 100])
        plt.ylim([-1, 1])

        plt.ion()
        plt.show()
        plt.draw()

        super(Plotter, self).__init__()

    def _update(self):
        self.line[0].set_data(np.array(self.t_, dtype=object), np.array(self.x_, dtype=object))
        plt.pause(0.00001)
        super(Plotter, self)._update()



if __name__ == '__main__':
    term = Plotter()

    while True:
        t = term("get, t")
        #print(t)
        term.set('x', sin(t))
        sleep(0.00005)