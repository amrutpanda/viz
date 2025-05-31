# import matplotlib
# matplotlib.use('agg')
import csv
import matplotlib.pyplot as plt
import numpy as np
from math import ceil
from scipy import optimize
from scipy import signal


# filename = 'results/data/positionDisturbance01.csv'
def fitActuatorPositionDisturbance(env, systemData, ID=1, filename='results/data/positionDisturbance01.csv'):
    req = env.req

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        data = list(reader)

    data_array = np.array(data[1:], np.float64);
    # get the data
    t = data_array[1:, 0]
    x_a = data_array[1:, 1]
    x_m = data_array[1:, 2]
    T_a = data_array[1:, 3]
    T_ff = data_array[1:, 4]
    T_m = data_array[1:, 5]
    v = data_array[1:, 6]

    T_d = T_a - T_ff - T_m

    gearratio = 101

    M_2PI = 2 * np.pi
    ticks_per_revolution = np.power(2, 19);
    x = M_2PI * x_m / ticks_per_revolution  # in radians

    x = x_a * gearratio

    # Do analysis (FFT)
    from numpy.fft import fft, ifft

    sr = 1000
    X = fft(T_d)
    N = len(X)
    n = np.arange(N)
    T = N / sr
    freq = n / T

    # get peaks
    idx = np.where(np.abs(X) > (100 * np.mean(np.abs(X))))
    print(freq[idx][0:5])

    plt.figure(figsize=(12, 6))
    plt.subplot(121)

    plt.stem(freq, np.abs(X), 'b', \
             markerfmt=" ", basefmt="-b")
    plt.xlabel('Freq (Hz)')
    plt.ylabel('FFT Amplitude |X(freq)|')
    plt.xlim(0, 100)

    plt.subplot(122)
    plt.plot(t, ifft(X), 'r')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.tight_layout()
    plt.show()

    # Do Stuff
    # Wn x sample frequency is the filter frequency.
    b, a = signal.butter(4, 0.02)
    y = signal.filtfilt(b, a, T_d)

    meany = np.mean(T_a)
    minx = np.min(x)
    maxx = np.max(x)
    xrange = maxx - minx
    midx = minx + xrange * 0.5
    delta = 0.01
    idxaroundmin = np.where((x > midx - delta) & (x < midx + delta))
    yaroundmid = T_a[idxaroundmin]
    friction_at_midstroke = np.max(yaroundmid) - np.min(yaroundmid)

    # get single direction:
    print("number of points", x.shape[0])
    t = ceil(x.shape[0] / 2)
    c = 100
    stepsize = 1

    backlash = 0.002

    # forward path
    x1 = x[c:t - c:stepsize] + backlash
    T1 = T_d[c:t - c:stepsize] - np.mean(T_d[c:t - c:stepsize])
    y1 = y[c:t - c:stepsize] - np.mean(y[c:t - c:stepsize])
    v1 = v[c:t - c:stepsize]

    # reverse path
    x2 = x[t + c:2 * t - c:stepsize] - backlash
    T2 = T_d[t + c:2 * t - c:stepsize] - np.mean(T_d[t + c:2 * t - c:stepsize])
    y2 = y[t + c:2 * t - c:stepsize] - np.mean(y[t + c:2 * t - c:stepsize])

    # rough fit of the shape over the complete stroke
    z1 = np.polyfit(x1, y1, 10)
    p1 = np.poly1d(z1)

    # But with curve fitting, python managed to find the correct amplitude and phase or me
    def sinfunc(x, A1, A2, A3, A4, A5, w1, w2, w3, w4, w5, p1, p2, p3, p4, p5, c):
        return (A1 * np.power(np.sin(2 * np.pi * w1 * x + p1), 1) + A2 * np.power(np.sin(2 * np.pi * w2 * x + p2), 1) +
                A3 * np.power(np.sin(2 * np.pi * w3 * x + p3), 1) + A4 * np.power(np.sin(2 * np.pi * w4 * x + p4), 1) +
                A5 * np.power(np.sin(2 * np.pi * w5 * x + p5), 1) + c)

    # def sinfunc(x,A,f,p,c):
    #     output = 0
    #     for i in range(len(A)):
    #         output += A[i] * np.sin(2*np.pi*f[i]*x+p[i])
    #     return output

    # Remove the velocity from the equation:
    f1 = y1 / v1

    print(x1.size)

    i1 = 100
    i2 = x1.size - i1
    rng = range(i1, i2, 1)

    # curve fit
    guess = [0.5, 0.2, 0.1, 0.1, 0.1,  # Amplitudes
             2 / M_2PI, 3 / M_2PI, 4 / M_2PI, 6 / M_2PI, 10 / M_2PI,  # Frequencies
             1, 1, 1, 1, 1,  # Phase
             0]  # Offset

    # curve fit
    # guess = [0.5, 0.2, 0.1, 0.1, 0.1,  # Amplitudes
    #          2/pi2, 3.35/pi2, 4/pi2, 6.76/pi2, 13.53/pi2,   # Frequencies
    #          1, 1, 1, 1, 1,  # Phase
    #          0]  # Offset

    #  Fit curves
    popt, pcov = optimize.curve_fit(sinfunc, x1[rng], y1[rng], p0=guess)

    # A1, A2, A3, w1, w2, w3, p1, p2, p3, c = popt
    print(popt.size)
    print(popt)
    Amplitude = popt[0:5]
    Frequency = popt[5:10]
    Phase = popt[10:15]
    Offset = popt[15]

    print("Amplitude : ", Amplitude)
    print("Frequency : ", Frequency)
    print("Phase : ", Phase)
    print("Offset : ", Offset)
    print(np.linalg.cond(pcov))

    fig = plt.figure(2)
    plt.plot(x1[rng], T1[rng], color='g', linewidth='0.1', label='measurement')  # raw
    plt.plot(x1[rng], v1[rng], color='b', linewidth='0.5', label='vel')  # filtered
    plt.plot(x1[rng], y1[rng], color='r', linewidth='0.5', label='filtfilt')  # filtered
    # plt.plot(x1[rng], sinfunc(x1[rng], A1, A2, A3, w1, w2, w3, p1, p2, p3, c),
    plt.plot(x1[rng], sinfunc(x1[rng], *popt), label='Fitted function')
    plt.legend()
    plt.grid()
    plt.show()
    plt.savefig(env.outputfolder + env.plotfolder + "positionForceCorrection%02d.png" % ID)

    # answer = input("Continue? :")

    # if (answer.lower == 'y') :
    req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionDisturbanceCorrection/amplitude",
                     Amplitude.tolist()).get()
    req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionDisturbanceCorrection/frequency",
                     Frequency.tolist()).get()
    req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionDisturbanceCorrection/phase",
                     Phase.tolist()).get()
    req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionDisturbanceCorrection/offset",
                     Offset.tolist()).get()
    req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionDisturbanceCorrection/order",
                     5).get()

    return
