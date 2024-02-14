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




# ----------------------------------------------------------- 
# Robot setup (RIGHT)
rtde_c = RTDEControl("192.168.5.30")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

# Parameters
acceleration = 0.2
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.1, 0.0]
# ----------------------------------------------------------- 

test_duration = 6.0

joint_angles = rtde_r.getActualQ()
joint_angles = [i *180.0/math.pi for i in joint_angles]
angle_range = [90-20,90+20]

initial_time = time.time()

try:
    while ((time.time() - initial_time) < test_duration) and (int(joint_angles[4]) in range(angle_range[0],angle_range[1]) ):
   # find cmd to balance the ball

        # rtde_c.speedJ(joint_speed, min(abs(ax),max_ax), dt)
        rtde_c.speedJ(joint_speed, 0.1,0.0001)

        # record data

        last_time = time.time()
        joint_angles = rtde_r.getActualQ()
        joint_angles = [i *180.0/math.pi for i in joint_angles]
        print("Joint angle 4: ", joint_angles[4])

        time.sleep(0.1)

        
finally:
    rtde_c.speedStop()
    rtde_c.stopScript()



