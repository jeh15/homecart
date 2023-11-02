## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      color based object detection         ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# config_1 = rs.config()
config.enable_device('f1182481')
depth_scale = pipeline_profile.get_device().first_depth_sensor().get_depth_scale()
print('depth scale: ', depth_scale)

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

# Start streaming
pipeline.start(config)

last_pos = np.array([0.0, 0.0, 0.0]);
last_time = time.time();
vel = 0.0;

# array to record position data
pos_data = np.array([0.0, 0.0, 0.0])
vel_data = np.array([0.0, 0.0, 0.0])

homography = np.array([[-1.91855468e-03,-5.64280788e-05,  1.78707726e-01],[-6.23853288e-06,  1.66568828e-03, -7.74825784e-03],[-2.41772053e-04,  3.13027163e-04,  1.00000000e+00]])


try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # if not depth_frame or not color_frame:
        #     continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))


        # Convert the image to HSV
        hsv_image = cv2.cvtColor(resized_color_image, cv2.COLOR_BGR2HSV)
        # lower and upper limits for the white color
        lower_limit = np.array([0, 153, 221])
        upper_limit = np.array([15, 255, 255])

        # create a mask for the specified color range
        mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
        # get the bounding box from the mask image
        bbox = cv2.boundingRect(mask)

        # draw the bounding box on the original image
        if bbox is not None:
            # print("Object detected")
            x, y, w, h = bbox
            cv2.rectangle(resized_color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.rectangle(depth_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            print("Object not detected")        



        depth = depth_image[math.ceil(y+w/2),math.ceil(x+w/2)].astype(float)
        dist = depth * depth_scale
        # dist = cv2.mean(depth)

        # depth = depth * depth_scale
        # dist = cv2.mean(depth)


        # convert pixel coordinates to real world coordinates
        pos_pixel = np.array([x+w/2, y+w/2, 1.])
        pos_real = np.dot(homography, pos_pixel)
        pos_real = pos_real/pos_real[2]
        pos_xy = pos_real[0:2]        

        pos = np.array([pos_xy[0], pos_xy[1], dist])

        vel = (pos - last_pos) / (time.time() - last_time)
        last_pos = pos

        pos_data = np.vstack([pos_data, pos])
        vel_data = np.vstack([vel_data, vel])

        # if pos_data.size > 15:
        #     pos_mean = np.mean(pos_data[:-2,:].reshape(-1,3), axis=0)
        #     vel_mean = np.mean(vel_data[:-2,:].reshape(-1,3), axis=0)
        # else:
        #     pos_mean = np.mean(pos_data.reshape(-1,3), axis=0)
        #     vel_mean = np.mean(vel_data.reshape(-1,3), axis=0)

        cv2.putText(resized_color_image, f"Vel: {vel[0]:.4f} {vel[1]:.4f} {vel[2]:.4f} m/s", (80,80), cv2.FONT_HERSHEY_COMPLEX,0.5, (0,0,0), 1)
        cv2.putText(resized_color_image, f"Pos: {pos[0]:.2f} {pos[1]:.2f} {pos[2]:.2f} m", (80,100), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0), 1)


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', resized_color_image)
        cv2.waitKey(1)
        # input("Press Enter to continue...")


finally:

    # Stop streaming
    pipeline.stop()