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
# Matlab setup
print("Starting matlab engine...")
eng = matlab.engine.connect_matlab()
print("Matlab engine started.")

# -----------------------------------------------------------


# NMPC
[xout,u_new] = eng.nmpc_new(0.1,0.0,0.2,0.0,0.0,nargout=2)
print(u_new[0][0])
# print(type(np.asarray(out)))
# print(np.asarray(out[-1:]))
# plt.plot(np.asarray(out[0])[0])
# plt.plot([-3.0000988301171114,-2.7500988302650073,-2.500098830456186,-2.250098828036735,-2.0000988279284297,-1.7500988277715634,-1.5000988278660978,-1.2500988286043628,-1.0000988286043628,-0.7500988286043628,-0.5000988286043628])
# plt.show()