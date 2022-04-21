import matplotlib.pyplot as plt

from servo import rhServo
from live_plot import Plotter
import time

import numpy as np


def test_loop():
    """Most of the control loop exists within this function, I want to partition it to elsewhere."""

    maximum = 1
    minimum = 0
    T = 2
    A = (maximum - minimum) / 2

    signal1 = A * np.sin(np.pi / T * (time.time() - t0)) + minimum + A
    signal2 = -A * np.sin(np.pi / T * (time.time() - t0)) + minimum + A

    return signal1, signal2


if __name__ == '__main__':
    import serial
    ser = serial.Serial('COM3', baudrate=9600, timeout=1)

    t0 = time.time()
    dt = 0.1

    plotter = Plotter()
    plt.show()

    test_servo = rhServo(debug=True)
    # --- Servo lifetime example --- #
    test_servo.open()

    while True:
        try:
            # get set point from test_loop()
            control = test_loop()

            try:
                # propagate to servo.
                test_servo.set_control(control)

                # read sensor output.
                sense = test_servo.get_sense()

            except BaseException as err:
                print(str(err))

            # do something with it.
            plotter.add_value(sense[1] * 100)
            print(sense[0])

        except KeyboardInterrupt:
            test_servo.close()
            break
