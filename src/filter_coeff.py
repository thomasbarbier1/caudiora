""" This file can be used to generate coefficient for the anti-aliasing FIR filter for the decimator """
# Usage: $ python filter_coeff.py

import numpy as np
from scipy import signal
from matplotlib import pyplot as plt

N = 79 # number of coeffs, must be odd
fs = 2400000  # sampling frequency
freq = [0, 62500, 140000, fs/2] # frequency array
amp = [1, 1, 0, 0] # amplitude array

# compute the coeff and print them. copy paste in the C program
coeffs = signal.firwin2(N, freq, amp, fs=fs)
print(coeffs)

# in need, check the filter template
filter_a1 = np.array([1])
w,H = signal.freqz(coeffs, filter_a1, 1024, fs=fs)
plt.figure()
plt.plot(w,20*np.log(np.abs(H)))
plt.ylabel('Gain en dB du filtre')
plt.xlabel('Freq(Hz)')
plt.grid()
plt.show()