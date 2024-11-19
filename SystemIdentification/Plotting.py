import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


file_name = "zieglerPonly.csv"
dirname = os.path.dirname(__file__)
path = os.path.join(dirname,file_name)
    
data = pd.read_csv(path)
print(data.shape)
data  = np.array(data)
out1 = data[:,1]
out2 = data[:,5]
plt.plot(data[:,1])
# plt.plot(data[:,3])

plt.show()
dt = 20 * 2000/1e6
fft_x = np.fft.fft(out1 - np.mean(out1))  # Subtract mean to remove DC component
frequencies = np.fft.fftfreq(out1.size, dt)  # Frequency array
plt.plot(frequencies,fft_x)
plt.show()
max_idx = np.argmax(fft_x)
print(1/frequencies[max_idx])
