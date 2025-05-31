import csv
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, ifft

filename = 'results/data/frequencyAnalysisData02.csv'

# read data from csv file
with open(filename, 'r') as f:
    reader = csv.reader(f)
    data = list(reader)

data_array = np.array(data[1:], np.float64);  # remove the first entry
# get the data
t = data_array[1:, 0]
x_a = data_array[1:, 1]
x_t = data_array[1:, 2]
T_a = data_array[1:, 3]
T_t = data_array[1:, 4]
v_a = data_array[1:, 5]
v_af = data_array[1:, 6]

# Do analysis (FFT)

sr = 1000  # sample time
X = fft(v_a)
X1 = fft(v_af)
# X = fft(x_a)
# X1 = fft(x_t)
# X = fft(T_a)
# X1 = fft(T_t)

N = len(X)
n = np.arange(N)
T = N / sr
freq = n / T

plt.figure(figsize=(12, 6))
plt.subplot(121)

# plt.stem(freq, np.abs(X), 'b', markerfmt=" ", basefmt="-b")
# plt.stem(freq, np.abs(X1), 'g', markerfmt=" ", basefmt="-g")
plt.semilogx(freq, np.abs(X), 'b')
plt.semilogx(freq, np.abs(X1), 'g')
plt.xlabel('Freq (Hz)')
plt.ylabel('FFT Amplitude |X(freq)|')
plt.xlim(0, 500)

plt.subplot(122)
plt.plot(t, ifft(X), 'r')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.tight_layout()
plt.show()
