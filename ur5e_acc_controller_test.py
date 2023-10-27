import numpy as np
import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import copy
from datetime import datetime
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import pdb

# -----------------------------------------------------------
# moving average filter parameters

# -----------------------------------------------------------
# low pass filter parameters
fs = 900.0
cutoff = 10.0
nyq = 0.5 * fs
normal_cutoff = cutoff / nyq
b, a = butter(2, normal_cutoff, btype='low', analog=False)



# ----------------------------------------------------------- 
# Robot setup (RIGHT)
rtde_c = RTDEControl("192.168.5.30")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

# Parameters
acc = 0.0
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ----------------------------------------------------------- 


# -----------------------------------------------------------
# sine wave acceleration command parameters
f = 2.0 # Hz
A = 0.2 # rad/s^2
# -----------------------------------------------------------


# -----------------------------------------------------------
# data logging
# targets
target_acc_data = np.array([])
target_vel_data = np.array([])

# actual
pos_data = np.array([])
vel_data = np.array([])
acc_data = np.array([])

# time
time_data = np.array([])
# -----------------------------------------------------------


# -----------------------------------------------------------
# loop timing parameters 
last_time = time.time();
initial_time = time.time();
test_duration = 2.0
iteration = 0
# -----------------------------------------------------------


# -----------------------------------------------------------
# initial robot state
actual_q = rtde_r.getActualQ()
last_pos = actual_q[4]
last_vel = 0.0
# -----------------------------------------------------------




try:
    while (time.time() - initial_time) < test_duration:

        # command acceleration
        target_acc = A*math.sin(2*math.pi*f*(time.time() - initial_time))
        dt = time.time() - last_time
        joint_speed[4] += target_acc*dt
        rtde_c.speedJ(joint_speed, abs(target_acc), dt)

        
        # get actual robot state
        actual_q = rtde_r.getActualQ()
        pos = actual_q[4]
        dt2 = time.time() - last_time 
        vel = (pos - last_pos)/ dt2
        if iteration == 0:
            acc = 0.0
        else:
            acc = (vel - last_vel)/dt2

        last_pos = copy.deepcopy(pos)
        last_vel = copy.deepcopy(vel)
        last_acc = copy.deepcopy(acc)
        last_time = time.time()

        # log data
        target_acc_data = np.append(target_acc_data, target_acc)
        target_vel_data = np.append(target_vel_data, joint_speed[4])
        pos_data = np.append(pos_data, pos)
        vel_data = np.append(vel_data, vel)
        acc_data = np.append(acc_data, acc)
        time_data = np.append(time_data, time.time() - initial_time)

        time.sleep(0.001)
        iteration += 1

        # updating the value of x and y
        # line1.set_xdata(time_data)
        # line1.set_ydata(acc_data)
        # fig.canvas.draw()
        # fig.canvas.flush_events()
        
finally:

    # stop robot
    rtde_c.speedStop()
    rtde_c.stopScript()



# -----------------------------------------------------------
# filter data
ws_pos = 1
ws_vel = 10
# -----------------------------------------------------------


# plot data
plt.figure(1)
plt.subplot(311)
plt.plot(time_data, target_acc_data, 'r.-', label='target acc')
filtered_acc_data = filtfilt(b, a, acc_data)
plt.plot(time_data, filtered_acc_data, 'b.-', label='actual acc')
plt.legend(shadow=True, fontsize='x-large', loc='lower center')
plt.title('Acceleration', fontsize=20)

plt.subplot(312)
plt.plot(time_data, target_vel_data, 'r.-', label='target vel')
filtered_vel_data = filtfilt(b, a, vel_data)
plt.plot(time_data, filtered_vel_data, 'b.-', label='actual vel')
plt.legend(shadow=True, fontsize='x-large',loc='lower center')
plt.title('Velocity', fontsize=20)

plt.subplot(313)
filtered_pos_data = filtfilt(b, a, pos_data) 
plt.plot(time_data, filtered_pos_data, 'b.-', label='actual pos')
plt.legend(shadow=True, fontsize='x-large',loc='lower center')
plt.title('Position', fontsize=20)
plt.show(True)


# # -----------------------------------------------------------
# # save data to file
# all_data = np.vstack((time_data, target_acc_data, acc_data, target_vel_data, vel_data, pos_data))

# # name file with date and time
# now = datetime.now()
# dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")
# np.savetxt('./data/data_' + dt_string + '.csv', all_data)