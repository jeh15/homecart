import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
import copy
from datetime import datetime
import rtde_receive
import matplotlib.pyplot as plt

# system identification
def armsysid(rtde_c, rtde_r,frequency):

    # design sine wave target arm trajectory
    amplitude = 0.5
    # frequency = 0.1
    t_end = 2.0

    # arm acceleration
    acc = 5.0
    st = 0.0001
    tool_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    # initialize data
    time_data = np.array([])
    vel_data = np.array([])
    targ_vel_data = np.array([])
    
    sleep_time = 0.033

    # while loop
    t_start = time.time()
    t_last = time.time()
    initial_pos = rtde_r.getActualTCPPose()[4]
    while (time.time() - t_start) < t_end:

        # get time
        t = time.time() - t_start

        # update tool speed
        tool_speed[4] = amplitude * np.sin(2*np.pi*frequency*t)

        # speedl to target
        rtde_c.speedL(tool_speed, acc, st)

        # add delay
        time.sleep(sleep_time)

        # record data
        time_data = np.append(time_data, t)
        current_pos = rtde_r.getActualTCPPose()[4]
        current_vel = (current_pos - initial_pos)/(time.time() - t_last)
        vel_data = np.append(vel_data, current_vel)
        targ_vel_data = np.append(targ_vel_data, tool_speed[4])
        t_last = time.time()
        initial_pos = copy.deepcopy(current_pos)

    # stop script
    rtde_c.speedStop(10.0)
    # rtde_c.stopScript()

    # save data
    data = np.vstack((time_data, vel_data, targ_vel_data))

    # add frequency to file name
    # replace . with _ in frequency
    frequency = str(frequency).replace('.', '_')
    filename = './sysiddata/arm_sysid_data4_' + str(frequency) + '.csv'

    # save to file
    # np.savetxt(filename, data, delimiter=",")
    return time_data, vel_data



def read_initial_pos(rtde_r):
    # get initial position
    p0 = rtde_r.getActualTCPPose()
    return p0


def go_to_initial_pos(rtde_c, rtde_r, p0):
    # go to initial position
    time.sleep(2.0)
    rtde_c.moveL(p0, 1.0, 1.0, True)
    time.sleep(2.0)
    rtde_c.stopL(2.0)


# main function
if __name__ == "__main__":

    # initialize rtde
    rtde_c = RTDEControl("192.168.5.30")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

    # get initial position
    p0 = read_initial_pos(rtde_r)
    
    # run system identification function
    td,vd = armsysid(rtde_c, rtde_r,frequency=2.0)

    plt.plot(td , vd)
    plt.show()


    # stop script
    rtde_c.stopScript()