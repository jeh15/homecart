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
    pos = np.array([pos_xy[0], pos_xy[1], dist])

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
    return [pos, vel, acc, jerk]
# -----------------------------------------------------------


# -----------------------------------------------------------
# Matlab setup
print("Starting matlab engine...")
eng = matlab.engine.connect_matlab()
print("Matlab engine started.")
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
out = cv2.VideoWriter('balancing_test.mp4', fourcc, 20.0, (640,  480))



# -----------------------------------------------------------
homography = np.array([[-1.91855468e-03,-5.64280788e-05,  1.78707726e-01],[-6.23853288e-06,  1.66568828e-03, -7.74825784e-03],[-2.41772053e-04,  3.13027163e-04,  1.00000000e+00]])
# -----------------------------------------------------------


# ----------------------------------------------------------- 
# Robot setup (RIGHT)
rtde_c = RTDEControl("192.168.5.30")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

# Parameters
acceleration = 0.2
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ----------------------------------------------------------- 


# -----------------------------------------------------------
# main loop parameters
max_speed_increase = 0.0001
ax_data = np.array([])
ay_data = np.array([])
vx_inc_data = np.array([])
vy_inc_data = np.array([])
time_data = np.array([])
ball_pos_data = np.array([])
ball_vel_data = np.array([])
ball_acc_data = np.array([])
ball_jerk_data = np.array([])
ax_array_data = np.array([])
last_pos = np.array([0.0, 0.0, 0.0])
last_vel = np.array([0.0, 0.0, 0.0])
last_acc = np.array([0.0, 0.0, 0.0])
ball_last_time = time.time()
arm_last_time = time.time()
initial_time = time.time()
vel = 0.0
test_duration = 6.0
ball_threshold = -0.45 # point at which it falls off the cutting board --and (last_pos[0] < ball_threshold)
max_ax = 1.0
iteration = 0
# -----------------------------------------------------------


# -----------------------------------------------------------
# setting joint limit
joint_angles = rtde_r.getActualQ()
joint_angles = [i *180.0/math.pi for i in joint_angles]
angle_range = [-104,-80]
# -----------------------------------------------------------


# -----------------------------------------------------------
# convert acc to joint traj
j_up = np.asanyarray([-103.23, -83.26, -98.16, -86.2, 102.03, 347.11])
j_down = np.asanyarray([-80.77, -73.64, -101.6, -94.02, 83.96, 368.62])
j_delta = j_up - j_down
j_ratio = j_delta/max(abs(j_delta))
# -----------------------------------------------------------

ball_x = -0.5
ball_range = [-7,-4]


try:
    while ((time.time() - initial_time) < test_duration) and (int(ball_x*10.0) in range(ball_range[0],ball_range[1]) ):

        # print time now
        time_now = time.time() - initial_time
        print("Time: ", time_now)

        # Get ball state
        frames = pipeline.wait_for_frames()
        stream_flag = True
        [pos, vel, acc, jerk] = ball_state(frames, last_pos, last_vel, last_acc, ball_last_time, depth_scale, stream_flag, iteration,out, time_now)
        ball_x = copy.deepcopy(pos[0])

        if iteration == 0:
            xt = pos[0]
            print("Target position: ", xt)

        ball_last_time = time.time()

        ball_pos_data = np.append(ball_pos_data,pos[0])
        ball_vel_data = np.append(ball_vel_data,vel[0])
        ball_acc_data = np.append(ball_acc_data,acc[0])
        ball_jerk_data = np.append(ball_jerk_data,jerk[0])

        if iteration > 5:
            v_avg = np.mean(ball_vel_data[-5:])
            a_avg = np.mean(ball_acc_data[-5:])
            j_avg = np.mean(ball_jerk_data[-5:])
            q = [pos[0], v_avg, a_avg, j_avg]
            last_pos = copy.deepcopy(pos)
            last_vel = copy.deepcopy([v_avg,0.,0.])
            last_acc = copy.deepcopy([a_avg,0.,0.])            
        else:
            q = [pos[0], vel[0], acc[0], jerk[0]]
            last_pos = copy.deepcopy(pos)
            last_vel = copy.deepcopy(vel)
            last_acc = copy.deepcopy(acc)            


        # convert ball state to matlab double
        posx = matlab.double([q[0]])
        velx = matlab.double([q[1]])
        accx = matlab.double([q[2]])
        jerkx = matlab.double([q[3]])

        # find cmd to balance the ball
        ax = eng.nmpc_test2(posx, velx, accx, jerkx, xt, nargout=1)
        ax_data = np.append(ax_data, ax)
        dt = time.time() - arm_last_time
        arm_last_time = time.time()
        speed_increaseX = -1.*ax*dt
        vx_inc_data = np.append(vx_inc_data, speed_increaseX)
        time_data = np.append(time_data, time.time() - initial_time)
        # joint_speed[4] += speed_increaseX

        js_inc = speed_increaseX * j_ratio
        joint_speed += js_inc
        rtde_c.speedJ(joint_speed, min(abs(ax),max_ax),0.0001)

        # record data

        last_time = time.time()
        iteration = iteration + 1
        joint_angles = rtde_r.getActualQ()
        joint_angles = [i *180.0/math.pi for i in joint_angles]

        
finally:
    pipeline.stop()
    rtde_c.speedStop()
    rtde_c.stopScript()

    # data = np.vstack((ax_data, ay_data, vx_inc_data, vy_inc_data))
    # np.savetxt('data.csv', data, delimiter=',')


    data = np.vstack((time_data,  ball_pos_data, ax_data, ball_vel_data, ball_acc_data, ball_jerk_data ))
    timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
    filename = "./data/data2_" + timestamp + ".csv"
    np.savetxt(filename, data, delimiter=',')
    out.release()
    cv2.destroyAllWindows()

