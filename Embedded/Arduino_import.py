import collections
import serial
from live_plot import Plotter
import matplotlib.pyplot as plt
import numpy as np
import time
import os

DEHYDRATE = b'\x00\x3F'
HYDRATE = b'\x0F\xC0'
STOP = b'\xF0\x00'

calibrate = False
cal_min, cal_max = 40.82, 85.78

n_tau = 2  # here we set to use multiple time constants to help distinguish from latency.

DEBUG = False
PLOT = True  # will display data as a live plot
LOGGING = True  # whether or not we wish to output to a datafile


# open ser as a global for the script
t0 = time.time()
ser = serial.Serial('/dev/cu.usbmodem14201', 9800, timeout=1)


def close():
    ser.write(STOP)
    ser.close()


def parse_input():
    newline = ser.readline()
    raw_data = newline.decode().split('\t')

    data = [0, 0, 0]
    if len(raw_data) == 3:
        data = [time.time() - t0, float(raw_data[1]), float(raw_data[2])]

    error = raw_data[0]
    if error != '0':
        print("Error code: ", error)

    return data


# calibrated maximum and minimum rH from system.
if calibrate:
    eps = 0.1
    frame_len = 25
    rh = 0

    # find maximum achievable humidity
    norm = eps + 1
    buf = collections.deque(np.ones(frame_len) * (eps + 1), maxlen=frame_len)
    ser.write(HYDRATE)

    while norm >= eps:
        try:
            data = parse_input()

            rh_old, rh = rh, data[2]
            drh = rh - rh_old

            buf.append(drh)
            norm = np.linalg.norm(np.asarray(buf))

            if DEBUG:
                print('current offset from eps is: {}'.format(norm - eps))

        except KeyboardInterrupt:
            close()
            break

    cal_max = rh
    print('maximum rh is: {}'.format(cal_max))

    # find minimum achievable humidity
    norm = eps + 1
    buf = collections.deque(np.ones(frame_len) * (eps + 1), maxlen=frame_len)
    ser.write(DEHYDRATE)

    while norm >= eps:
        try:
            data = parse_input()

            rh_old, rh = rh, data[2]
            drh = rh - rh_old

            buf.append(drh)
            norm = np.linalg.norm(np.asarray(buf))

            if DEBUG:
                print('current offset from eps is: {}'.format(norm - eps))

        except KeyboardInterrupt:
            close()
            break

    cal_min = rh
    print('minimum rh is: {}'.format(cal_min))

# We wish to oscillate with half-periods corresponding to each state's time constant.
max_val = cal_max - (cal_max - cal_min) / np.e ** n_tau
min_val = cal_min + (cal_max - cal_min) / np.e ** n_tau

# plotting values
if PLOT:
    rh_plotter = Plotter()
    plt.axhline(y=max_val, color='r', linestyle='-')
    plt.axhline(y=min_val, color='r', linestyle='-')

if LOGGING:
    output_file = open('DataDump/rhTauLatency' + time.strftime("_%d.%m.%Y_%H:%M:%S") + '.txt', 'w')
old_t, t = 0, 0
mode = "Tau"
ser.write(HYDRATE)

while ser.is_open:
    try:
        # read and parse data of relative humidity
        data = parse_input()
        rh = data[2]
        if PLOT:
            rh_plotter.add_value(rh)

        # Simple algorithm for bang-bang control with hysteresis.
        # Here, I'm using it to calculate the system's time constants.
        if min_val <= rh <= max_val and mode == "latency":  # rh is in the desired range
            old_t, t = t, data[0]
            latency = t - old_t

            if DEBUG:
                print("Latency is: {}".format(latency))
            if LOGGING:
                output_file.write("lat, {}".format(latency) + os.linesep)

            mode = "Tau"

        elif rh > max_val and mode == "Tau":  # rh is too high
            # turn PUMP2 on and PUMP1 off
            old_t, t = t, data[0]
            tau = (t - old_t) / n_tau

            if DEBUG:
                print("Time constant for hydration is: {}\n".format(tau))
            if LOGGING:
                output_file.write("hyd, {}".format(tau) + os.linesep)

            mode = "latency"
            ser.write(DEHYDRATE)

        elif rh < min_val and mode == "Tau":  # rh is too low
            # turn PUMP1 on and PUMP2 off
            old_t, t = t, data[0]
            tau = (t - old_t) / n_tau

            if DEBUG:
                print("Time constant for dehydration is: {}\n".format(tau))
            if LOGGING:
                output_file.write("deh, {}".format(tau) + os.linesep)

            mode = "latency"
            ser.write(HYDRATE)

    except KeyboardInterrupt:
        close()
        if LOGGING:
            output_file.close()
        break
