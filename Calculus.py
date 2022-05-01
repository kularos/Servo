import numpy as np
from time import time, sleep
from math import sin, prod

currTime = time()


class Calculussy:

    def __init__(self, n_pts=4):
        self.n_axes = 4
        self.n_pts = n_pts

        self._buffer = np.zeros((self.n_pts, self.n_axes))
        self._ri = 0

    def get_buffer(self, buffer_index, vector_index):
        return self._buffer[(self._ri - buffer_index) % self.n_pts][vector_index]

    def set_buffer(self, buffer_index, vector_index, value):
        self._buffer[(self._ri - buffer_index) % self.n_pts][vector_index] = value

    def updateBuffer(self, x):
        # Store actual values
        self.set_buffer(0, 0, currTime)
        self.set_buffer(0, 1, x)

        # Store finite differences
        self.set_buffer(0, 2, currTime - self.get_buffer(1, 0))
        self.set_buffer(0, 3, x - self.get_buffer(1, 1))

        self._ri = (self._ri + 1) % self.n_pts

    def calc_numerator(self, t):
        """Not the best approach, but O(n)"""
        # Calculate product of all monomials.
        n = prod([t - self.get_buffer(i, 0) for i in range(self.n_pts)])

        # Create a vector which excludes each monomial from the product.
        n_i = [n / (t - self.get_buffer(i, 0)) for i in range(self.n_pts)]

        return n_i

    def calc_denominator(self, t):
        """O(n^2), makes a list of the """
        d_i = []
        for i in range(self.n_pts):
            d = prod([self.get_buffer(i, 0) - self.get_buffer(i + n, 0) for n in range(1, self.n_pts)])
            d_i.append(d)
        return d_i

    def calc_coefs(self, t):
        num = self.calc_numerator(t)
        denom = self.calc_denominator(t)

        l = [num[i]/denom[i] for i in range(self.n_pts)]

        return l

    def calc_deriv(self, t):
        coef = self.calc_coefs(t)
        deriv = 0

        for i in range(self.n_pts):
            d_coef_i = coef[i] * (sum([1/(t - self.get_buffer(n, 0)) for n in range(self.n_pts)]) - 1/(t - self.get_buffer(i, 0)))
            deriv += d_coef_i * self.get_buffer(i, 1)

        return deriv


class CalculussyMk2:

    def __init__(self, n_pts=4, vector_dim=1):
        self._t0 = time()

        self.n_pts = n_pts
        self.n_axes = vector_dim + 1  # One axis is always reserved for time.

        self._buffer = np.zeros((self.n_pts, self.n_axes))  # Ring buffer which stores values
        self._fd_buffer = np.zeros((self.n_pts, self.n_pts))  # Ring buffer which stores all time differences.
        self._ri = 0  # Current index of ring buffer.

    def _roll(self, n):
        """Helper function which modulates an input n by the number of points in the solver."""
        return (self._ri + n) % self.n_pts

    def _update_buffer(self, vec):
        current_time = time() - self._t0
        self._ri = self._roll(1)

        self._buffer[self._roll(0)][0] = current_time
        self._buffer[self._roll(0)][1:] = vec


x = 0
calc = Calculussy()

"""
# Set values to match Desmos workbook
vals = [[0, 1.5, 2.5, 3.5], [6, 4, 2.8, 2]]
for i in range(calc.n_pts):
    calc.set_buffer(i, 0, vals[0][i])
    calc.set_buffer(i, 1, vals[1][i])

for i in np.linspace(0, 5, 51):
    print(i, calc.calc_deriv(i))
"""

while True:
    d = calc.calc_deriv(currTime)

    currTime = time()
    x = sin(currTime)
    calc.updateBuffer(x)

    print(x, d)
    sleep(0.2)