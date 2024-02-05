import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import csv
import matplotlib.pyplot as plt
import matplotlib
from scipy.signal import butter, lfilter, filtfilt




# -----------------------------------------------------------
# Band Pass Filter
def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

# low pass filter settings
fs1 = 30.
cutoff1 = 5.
order1 = 3
cutoff_relative = cutoff1/(.5*fs1)
b1, a1 = butter(order1, cutoff_relative, analog=False)
lowpass_filt_flag = 1
# -----------------------------------------------------------






with open("./data/mid2_kf_2024_01_23-01_14_03_PM.csv") as fp:
    reader = csv.reader(fp, delimiter=",", quotechar='"')
    # next(reader, None)  # skip the headers
    data_read = [row for row in reader]


time = data_read[0][0:40]
time = [float(row) for row in time]
pos = data_read[1][0:40]
pos = [float(row) for row in pos]
vel = data_read[2][0:40]
vel = [float(row) for row in vel]
acc = data_read[3][0:40]
acc = [float(row) for row in acc]


fpos = np.array([])
fvel = np.array([])
facc = np.array([])

fpos2 = np.array([])
fvel2 = np.array([])
facc2 = np.array([])

w1 = np.array([0.,0.,0.])

i=0
# Finally, run the filter.
while i<40:
    if i>15:    
        w1[0] = filtfilt(b1, a1, pos[0:i])[-2]
        w1[1] = filtfilt(b1, a1, vel[0:i])[-2]
        w1[2] = filtfilt(b1, a1, acc[0:i])[-2]
        fpos = np.append(fpos,w1[0])
        fvel = np.append(fvel,w1[1])
        facc = np.append(facc,w1[2])
    else:
        fpos = np.append(fpos,pos[i])
        fvel = np.append(fvel,vel[i])
        facc = np.append(facc,acc[i])
    i=i+1



plt.plot(time, pos, time, fpos)
plt.legend(['real','lf'])
plt.show()

plt.plot(time, vel, time, fvel)
plt.legend(['real','lf'])
plt.show()

plt.plot(time, acc, time, facc)
plt.legend(['real','lf'])
plt.show()
