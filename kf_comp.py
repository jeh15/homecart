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
# get ball state from camera
def ball_state(frames, last_pos, last_vel, last_acc, last_time, depth_scale, stream_flag,iteration,out, time_now):
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    depth_colormap_dim = depth_colormap.shape
    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
    hsv_image = cv2.cvtColor(resized_color_image, cv2.COLOR_BGR2HSV)
    lower_limit = np.array([0, 153, 221])
    upper_limit = np.array([15, 255, 255])
    mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
    bbox = cv2.boundingRect(mask)
    if bbox is not None:
        x, y, w, h = bbox
        cv2.rectangle(resized_color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.rectangle(depth_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    else:
        print("Object not detected")
    cv2.putText(resized_color_image, f"time : {time_now:.4f} s", (80,80), cv2.FONT_HERSHEY_COMPLEX,0.5, (0,0,0), 1)        
    out.write(resized_color_image)
    depth = depth_image[math.ceil(y+w/2),math.ceil(x+w/2)].astype(float)
    dist = depth * depth_scale
    pos_pixel = np.array([x+w/2, y+w/2, 1.])
    homography = np.array([[-1.91855468e-03,-5.64280788e-05,  1.78707726e-01],[-6.23853288e-06,  1.66568828e-03, -7.74825784e-03],[-2.41772053e-04,  3.13027163e-04,  1.00000000e+00]])
    pos_real = np.dot(homography, pos_pixel)
    pos_real = pos_real/pos_real[2]
    pos_xy = pos_real[0:2]        
    # pos = np.array([pos_xy[0], pos_xy[1], dist])

    # dist along the board
    bx0 = -0.9   # pivot point in x axis
    bz0 =  0.62  # pivot point in z axis
    pw = np.sqrt((pos_xy[0]-bx0)**2 + (dist-bz0)**2)
    pos = np.array([pw, pos_xy[1], dist])

    # print("pos: ", pos)
    # print("last_pos: ", last_pos)
    # print("last_vel: ", last_vel)

    vel = (pos - last_pos) / (time.time() - last_time)
    acc = (vel - last_vel) / (time.time() - last_time)
    jerk = (acc - last_acc) / (time.time() - last_time)

    if iteration == 0:
        vel = np.array([0.0, 0.0, 0.0])
        acc = np.array([0.0, 0.0, 0.0])
        jerk = np.array([0.0, 0.0, 0.0])
    elif iteration == 1:
        acc = np.array([0.0, 0.0, 0.0])
        jerk = np.array([0.0, 0.0, 0.0])
    elif iteration == 2:
        jerk = np.array([0.0, 0.0, 0.0])      

    if stream_flag:
        cv2.putText(resized_color_image, f"Vel: {vel[0]:.4f} {vel[1]:.4f} {vel[2]:.4f} m/s", (80,80), cv2.FONT_HERSHEY_COMPLEX,0.5, (0,0,0), 1)
        cv2.putText(resized_color_image, f"Pos: {pos[0]:.2f} {pos[1]:.2f} {pos[2]:.2f} m", (80,100), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0), 1)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', resized_color_image)
        cv2.waitKey(1)
    return [pos, 1.0*vel, 1.0*acc, jerk]
# -----------------------------------------------------------



# ----------------------------------------------------------- 
# Camera setup
pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
config.enable_device('f1182481')
depth_scale = pipeline_profile.get_device().first_depth_sensor().get_depth_scale()
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
# ----------------------------------------------------------- 


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('balancing_test_kf_mpc.mp4', fourcc, 20.0, (640,  480))



# -----------------------------------------------------------
homography = np.array([[-1.91855468e-03,-5.64280788e-05,  1.78707726e-01],[-6.23853288e-06,  1.66568828e-03, -7.74825784e-03],[-2.41772053e-04,  3.13027163e-04,  1.00000000e+00]])
# -----------------------------------------------------------


# -----------------------------------------------------------
# main loop parameters
max_speed_increase = 0.0001
target_angle_data = np.array([])
ay_data = np.array([])
vx_inc_data = np.array([])
vy_inc_data = np.array([])
time_data = np.array([])
p = np.array([])
v = np.array([])
a = np.array([])

pf = np.array([])
vf = np.array([])
af = np.array([])

ball_jerk_data = np.array([])
ax_array_data = np.array([])

targ_vel_data = np.array([])

arm_angle_array_data = np.array([])
last_pos = np.array([0.5, 0.0, 0.0])
last_vel = np.array([0.0, 0.0, 0.0])
last_acc = np.array([0.0, 0.0, 0.0])
ball_last_time = time.time()
arm_last_time = time.time()
initial_time = time.time()
test_duration = 2.0
ball_threshold = -0.45 # point at which it falls off the cutting board --and (last_pos[0] < ball_threshold)
max_ax = 2.0
iteration = 0
# -----------------------------------------------------------




#--------------------------------------------------------------------------
# Now, create the filter
my_filter = KalmanFilter(dim_x=3, dim_z=1)
# Initialize the filter's matrices.
# my_filter.F = np.array([[1., .1/3., 0.],
#                         [0., 1., .1/3.],
#                         [0., 0., 1.]])    # state transition matrix
# my_filter.B = np.array([[0.],
#                         [0.],
#                         [5.886]])    # state transition matrix
# my_filter.H = np.array([[1.,0.,0.]])    # Measurement function
# my_filter.P *= 1000.                 # covariance matrix
# my_filter.R = 5                      # state uncertainty
# my_filter.Q = Q_discrete_white_noise(dim=3, dt=0.1/3., var=0.1) # process uncertainty
#--------------------------------------------------------------------------



ball_x = -0.5
ball_range = [-7,-3]

# Fixed acceleration
fix_ur5_acc = 5.0
fix_ur5_vel = 0.5

maxvel = 10.0 # previously 10.0

try:
    while ((time.time() - initial_time) < test_duration):

        # print time now
        time_now = time.time() - initial_time
        print("Time: ", time_now)
        # print("j4: ", joint_angles[4])

        # Get ball state
        frames = pipeline.wait_for_frames()
        stream_flag = True
        [pos, vel, acc, jerk] = ball_state(frames, last_pos, last_vel, last_acc, ball_last_time, depth_scale, stream_flag, iteration,out, time_now)
        ball_x = copy.deepcopy(pos[0])

        if iteration == 0:
            xt = pos[0]
            print("Target position: ", xt)
            my_filter.x = np.array([[xt],
                                    [0.],
                                    [0.]])       # initial state (location and velocity)
            my_filter.F = np.array([[1., .1/3., 0.],
                                    [0., 1., .1/3.],
                                    [0., 0., 1.]])    # state transition matrix
            my_filter.B = np.array([[0.],
                                    [0.],
                                    [5.886]])    # state transition matrix
            my_filter.H = np.array([[1.,0.,0.]])    # Measurement function
            my_filter.P *= 1000.                 # covariance matrix
            my_filter.R = 5                      # state uncertainty
            my_filter.Q = Q_discrete_white_noise(dim=3, dt=0.1/3., var=0.1) # process uncertainty
            


        ball_last_time = time.time()

        p = np.append(p,pos[0])
        v = np.append(v,vel[0])
        a = np.append(a,acc[0])


        if iteration > -1:
            my_filter.predict()
            my_filter.update(pos[0])
            xx = my_filter.x
            pf = np.append(pf,xx[0])
            vf = np.append(vf,xx[1])
            af = np.append(af,xx[2])                        
            last_pos = copy.deepcopy(pos[0])
            last_vel = copy.deepcopy(vel[0])
            last_acc = copy.deepcopy(acc[0])    
        else:
            q = [pos[0], vel[0], acc[0], jerk[0]]
            last_pos = copy.deepcopy(pos[0])
            last_vel = copy.deepcopy(vel[0])
            last_acc = copy.deepcopy(acc[0])    
        # af = np.append(af,xx[2]) 

        time_data = np.append(time_data, time.time() - initial_time)

        # record data
        last_time = time.time()
        iteration = iteration + 1
        
finally:
    pipeline.stop()

    # size of variables
    print("p: ", p.shape)
    print("v: ", v.shape)
    print("pf: ", pf.shape)
    print("vf: ", vf.shape)
    print("time: ", time_data.shape)

    plt.plot(time_data, p, time_data, pf)
    plt.legend(['real','f'])
    plt.show()

    plt.plot(time_data, v, time_data, vf)
    plt.legend(['real','f'])
    plt.show()
