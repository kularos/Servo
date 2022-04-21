import matplotlib.pyplot as plt
import numpy as np
from math import sin
from Core import LiveTerminal, BufferedParameter
from time import sleep
import matplotlib
matplotlib.use("GTKAgg")

class Plotter(LiveTerminal):
    buffer_size = 200
    t = BufferedParameter(size=buffer_size)
    x = BufferedParameter(size=buffer_size)

    def __init__(self):

        # Initialize pyplot figure and axes
        plt.ion()
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111)
        self.line = self._ax.plot(np.array(self.t_), np.array(self.x_))
        # xlims are dynamically set
        plt.xlim([0, 100])
        plt.ylim([-1, 1])
        #plt.show()

        super(Plotter, self).__init__()

    def _update(self):

        self.line[0].set_data(np.array(self.t_), np.array(self.x_))
        self._fig.canvas.draw()
        super(Plotter, self)._update()



if __name__ == '__main__':
    term = Plotter()

    while True:
        t = term("get, t")
        term.set('x', sin(t))
        sleep(0.005)