from servo import rhServo
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

    test_servo = rhServo()
    # --- Servo lifetime example --- #
    test_servo.open()  # open

    while True:
        try:
            # get set point from test_loop()
            control = test_loop()
            # propagate to servo.
            test_servo.set_control(*control)

            # read sensor output.
            sense = test_servo.get_sense()
            # do something with it.
            print(sense)

            time.sleep(dt)

        except KeyboardInterrupt:
            test_servo.close()
            break
