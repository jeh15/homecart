import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
import matlab.engine            #import the matlab engine
import copy
from datetime import datetime
import rtde_receive
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import matplotlib.pyplot as plt

# ----------------------------------------------------------- 
# Robot setup (RIGHT)
rtde_c = RTDEControl("192.168.5.30")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

# Parameters
acceleration = 0.2
tool_speed = [0.0, 0.0, 0.0, 0.0, -0.1, 0.0]
# ----------------------------------------------------------- 
# -----------------------------------------------------------
# main loop parameters
time_data = np.array([])
pos_data = np.array([])
# -----------------------------------------------------------
# -----------------------------------------------------------
robot_pose = rtde_r.getActualTCPPose()
original_pose = copy.deepcopy(robot_pose[4])
print("Original pose: ", original_pose)
# Fixed acceleration
fix_ur5_acc = 0.5
test_duration = 6.0
iteration = 0
initial_time = time.time()
# -----------------------------------------------------------
try:
    while ((time.time() - initial_time) < test_duration):
        # get tool position and velocity
        rtde_c.speedL(tool_speed, fix_ur5_acc, 0.0001)
        time.sleep(0.01)
        board_pos = rtde_r.getActualTCPPose()[4]

        # wrap around the angle
        if board_pos < 0:
            board_pos += 2 * np.pi

        pos_data = np.append(pos_data, board_pos - original_pose)
        time_data = np.append(time_data, time.time() - initial_time)
finally:
    rtde_c.speedStop(10.0)
    rtde_c.stopScript()
    plt.plot(time_data, pos_data)
    plt.show()
