import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import csv
import matplotlib.pyplot as plt
import matplotlib

#--------------------------------------------------------------------------
#dropstop_2024_01_31-04_34_40_PM.csv
with open("./data/ballbaordmodel_2024_02_05-01_48_42_PM.csv") as fp:
    reader = csv.reader(fp, delimiter=",", quotechar='"')
    # next(reader, None)  # skip the headers
    data_read = [row for row in reader]





time = data_read[0]
time = [float(row) for row in time]
pos = data_read[1]
pos = [float(row) for row in pos]
vel = data_read[2]
vel = [float(row) for row in vel]
acc = data_read[3]
acc = [float(row) for row in acc]
jerk = data_read[4]
jerk = [float(row) for row in acc]


fpos = np.array([])
fvel = np.array([])

fpos2 = np.array([])
fvel2 = np.array([])



i=0




#--------------------------------------------------------------------------
# Now, create the filter
my_filter = KalmanFilter(dim_x=2, dim_z=1)
# Initialize the filter's matrices.
my_filter.x = np.array([[pos[0]],[vel[0]]])       # initial state (location and velocity)
my_filter.F = np.array([[1., .1/3.],
                        [0.,    1.]])    # state transition matrix
my_filter.B = np.array([[0.],           #
                        [5.886]])      # state transition matrix
my_filter.H = np.array([[1.,0.]])     # Measurement function
my_filter.P *= 0.0                   # covariance matrix
my_filter.R = 0.00000000005             # state uncertainty
my_filter.Q = Q_discrete_white_noise(dim=2, dt=0.1/3., var=0.01) # process uncertainty
#--------------------------------------------------------------------------



#--------------------------------------------------------------------------
# Now, create the filter
my_filter2 = KalmanFilter(dim_x=2, dim_z=1)
# Initialize the filter's matrices.
my_filter2.x = np.array([[acc[0]],[jerk[0]]])       # initial state (location and velocity)
my_filter2.F = np.array([[1., .1/3.],
                        [0.,    1.]])    # state transition matrix
my_filter2.B = np.array([[0.],           #
                        [5.886]])      # state transition matrix
my_filter2.H = np.array([[1.,0.]])     # Measurement function
my_filter2.P *= 0.0                   # covariance matrix
my_filter2.R = 0.0000000005             # state uncertainty
my_filter2.Q = Q_discrete_white_noise(dim=2, dt=0.1/3., var=0.01) # process uncertainty
#--------------------------------------------------------------------------




# print size(pos)
print('size(pos): ', np.size(pos))


# Finally, run the filter.
final = np.size(pos)
while i<final:
    my_filter.predict()
    my_filter.update(pos[i])
    x = my_filter.x
    # print('x: ', x)
    fpos = np.append(fpos,x[0])
    fvel = np.append(fvel,x[1])

    my_filter2.predict()
    my_filter2.update(acc[i])

    x2 = my_filter2.x
    # print('x2: ', x2)
    fpos2 = np.append(fpos2,x2[0])
    fvel2 = np.append(fvel2,x2[1])


    # if i>2:
    #     fpos2 = np.append(fpos2,np.mean(pos[i-3:i]))
    #     fvel2 = np.append(fvel2,np.mean(vel[i-3:i]))
    # else:
    #     fpos2 = np.append(fpos2,pos[i])
    #     fvel2 = np.append(fvel2,vel[i])

    i = i + 1


plt.plot(time[0:i], pos[0:i], time[0:i], fpos[0:i], time[0:i], fpos2[0:i])
plt.legend(['real','kf1000'])
plt.show()

plt.plot(time[0:i], vel[0:i], time[0:i], fvel[0:i], time[0:i], fvel2[0:i])
plt.legend(['real','kf1000','last3mean'])
plt.show()

plt.plot(time[0:i], jerk[0:i], time[0:i], fvel2[0:i])
plt.legend(['real','kf'])
plt.show()
