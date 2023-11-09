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
# Robot setup (RIGHT)
rtde_c = RTDEControl("172.17.0.2")
rtde_r = rtde_receive.RTDEReceiveInterface("172.17.0.2")




try:
    actual_q = rtde_r.getActualQ()
    actual_q = [i *180.0/math.pi for i in actual_q]    
    print("actual_q: ", actual_q)
    # angles in degrees
    # print("joint 1: ", actual_q[0]*180.0/math.pi) 
    # print("joint 2: ", actual_q[1]*180.0/math.pi)
    # print("joint 3: ", actual_q[2]*180.0/math.pi)
    # print("joint 4: ", actual_q[3]*180.0/math.pi)
    # print("joint 5: ", actual_q[4]*180.0/math.pi)
    # print("joint 6: ", actual_q[5]*180.0/math.pi)
    

finally:

    # stop robot
    rtde_c.speedStop()
    rtde_c.stopScript()


