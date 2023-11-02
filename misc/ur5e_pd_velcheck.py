import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from rtde_control import RTDEControlInterface as RTDEControl
import matlab.engine            #import the matlab engine
import copy

# get ball state from camera
def ball_state(frames, last_pos, last_time, depth_scale, stream_flag):
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
    depth = depth_image[math.ceil(y+w/2),math.ceil(x+w/2)].astype(float)
    dist = depth * depth_scale
    pos_pixel = np.array([x+w/2, y+w/2, 1.])
    homography = np.array([[-1.91855468e-03,-5.64280788e-05,  1.78707726e-01],[-6.23853288e-06,  1.66568828e-03, -7.74825784e-03],[-2.41772053e-04,  3.13027163e-04,  1.00000000e+00]])
    pos_real = np.dot(homography, pos_pixel)
    pos_real = pos_real/pos_real[2]
    pos_xy = pos_real[0:2]        
    pos = np.array([pos_xy[0], pos_xy[1], dist])
    vel = (pos - last_pos) / (time.time() - last_time)
    last_pos = pos
    if stream_flag:
        cv2.putText(resized_color_image, f"Vel: {vel[0]:.4f} {vel[1]:.4f} {vel[2]:.4f} m/s", (80,80), cv2.FONT_HERSHEY_COMPLEX,0.5, (0,0,0), 1)
        cv2.putText(resized_color_image, f"Pos: {pos[0]:.2f} {pos[1]:.2f} {pos[2]:.2f} m", (80,100), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0), 1)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', resized_color_image)
        cv2.waitKey(1)
    return [pos, vel]


# -----------------------------------------------------------
# Matlab setup
print("Starting matlab engine...")
eng = matlab.engine.start_matlab()  
print("Matlab engine started.")


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

# ----------------------------------------------------------- 
# Robot setup (RIGHT)
rtde_c = RTDEControl("192.168.5.30")

# Parameters
acceleration = 2.0
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ----------------------------------------------------------- 

last_pos = np.array([0.0, 0.0, 0.0]);
last_time = time.time();
initial_time = time.time();
vel = 0.0;
homography = np.array([[-1.91855468e-03,-5.64280788e-05,  1.78707726e-01],[-6.23853288e-06,  1.66568828e-03, -7.74825784e-03],[-2.41772053e-04,  3.13027163e-04,  1.00000000e+00]])


test_duration = 10.0

initial = 0

max_speed_increase = 0.0001

ax_data = np.array([])
ay_data = np.array([])

vx_inc_data = np.array([])
vy_inc_data = np.array([])

try:
    while (time.time() - initial_time) < test_duration:
        frames = pipeline.wait_for_frames()
        stream_flag = True
        [pos, vel] = ball_state(frames, last_pos, last_time, depth_scale, stream_flag)

        if initial == 0:
            pos_ix = copy.deepcopy(pos[0])
            pos_iy = copy.deepcopy(pos[1])
            initial = 1
            print("Initial position recorded.")
            print("pos_ix: ", pos_ix)

        posx = matlab.double([pos[0]])
        posy = matlab.double([pos[1]])
        velx = matlab.double([vel[0]])
        vely = matlab.double([vel[1]])

        # if (time.time() - initial_time) > 2.0:
        #     acceleration = 0.0

        # ax = eng.pd_x(posx, velx, pos_ix)
        # ay = eng.pd_y(posy, vely, pos_iy)

        # ax_data = np.append(ax_data, ax)
        # ay_data = np.append(ay_data, ay)

        dt = time.time() - last_time

        # speed_increaseX = ax*dt
        # speed_increaseY = ay*dt

        # vx_inc_data = np.append(vx_inc_data, speed_increaseX)
        # vy_inc_data = np.append(vy_inc_data, speed_increaseY)

        # if abs(speed_increaseX) > max_speed_increase:
        #     speed_increaseX = max_speed_increase

        # if abs(speed_increaseY) > max_speed_increase:
        #     speed_increaseY = max_speed_increase
        kp = 5.0
        kd = 0.0
        joint_speed[4] = kp*(pos[0] - pos_ix) + kd*vel[0]
        # joint_speed[3] = -kp*(pos[1] - pos_iy) - kd*vel[1]
        
        print("joint_speed: ", joint_speed)

        rtde_c.speedJ(joint_speed, acceleration, dt)

        last_pos = copy.deepcopy(pos)
        last_time = time.time()

        
finally:
    pipeline.stop()
    rtde_c.speedStop()
    rtde_c.stopScript()

    data = np.vstack((ax_data, ay_data, vx_inc_data, vy_inc_data))
    np.savetxt('data.csv', data, delimiter=',')





