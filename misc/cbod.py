## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      color based object detection         ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

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
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        # if depth_colormap_dim != color_colormap_dim:
        #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        #     images = np.hstack((resized_color_image, depth_colormap))
        # else:
        #     images = np.hstack((color_image, depth_colormap))


        # Convert the image to HSV
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
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
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.rectangle(depth_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            print("Object not detected")        



        depth = depth_image[x:x+w, y:y+w].astype(float)

        depth = depth * depth_scale

        dist = cv2.mean(depth)

        cv2.putText(color_image, f"Depth: {dist} m", (200,200), cv2.FONT_HERSHEY_PLAIN, 2.0, (0,0,0), 1)


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)
        # input("Press Enter to continue...")

finally:

    # Stop streaming
    pipeline.stop()