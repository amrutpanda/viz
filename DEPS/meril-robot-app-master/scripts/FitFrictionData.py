# import matplotlib
# matplotlib.use('agg')
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# this is an example code to do friction fit with. Make only use of the static friction model.
# Needs to be modified. (not working yet)

filename = 'results/data/friction01.csv'

with open(filename, 'r') as f:
    reader = csv.reader(f)
    data = list(reader)

data_array = np.array(data[1:], np.float64);
# get the data
x = data_array[1:, 1]
T_d = data_array[1:, 2]
v = data_array[1:, 3]
ff = data_array[1:, 4]

# remove noise with higher frequencies
# Wn x sample frequency is the filter frequency.
b, a = signal.butter(6, 0.1)
y = signal.filtfilt(b, a, T_d)

b, a = signal.butter(6, 0.1)
v = signal.filtfilt(b, a, v)

meany = np.mean(y)
minx = np.min(x)
maxx = np.max(x)
xrange = maxx - minx
midx = minx + xrange * 0.5
delta = 0.01
idxaroundmin = np.where((x > midx - delta) & (x < midx + delta))
yaroundmid = y[idxaroundmin]
friction_at_midstroke = np.max(yaroundmid) - np.min(yaroundmid)

x1 = x[v > 0]
y1 = y[v > 0] - np.mean(y[v > 0])

x2 = x[v < 0]
y2 = y[v < 0] - np.mean(y[v < 0])

fig = plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(x, y)
plt.plot(x, ff, color='r')
plt.plot(x1, y1, color='k')
plt.plot(x2, y2, color='k')
plt.xlabel("position (m)"), plt.ylabel("torque (Nm)")
plt.subplot(2, 1, 2)
plt.plot(v, y)
plt.plot(v, ff, color='r')
plt.xlabel("velocity (m/s)"), plt.ylabel("torque (Nm)")
ax = plt.gca()
plt.title("friction plot")
plt.show()

# add function to fit the frictioncurve
