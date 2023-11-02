import cv2
import numpy as np
import os


# Create a function to get the HSV values from the trackbar
def get_hsv_values(val):
    pass


# Create a window to display the image
cv2.namedWindow('image')

# Load the image
image = cv2.imread('/home/orl/Pictures/s3.png')

# Create a trackbar for Hue
cv2.createTrackbar('H min', 'image', 0, 179, get_hsv_values)
cv2.createTrackbar('H max', 'image', 179, 179, get_hsv_values)

# Create a trackbar for Saturation
cv2.createTrackbar('S min', 'image', 0, 255, get_hsv_values)
cv2.createTrackbar('S max', 'image', 255, 255, get_hsv_values)

# Create a trackbar for Value
cv2.createTrackbar('V min', 'image', 0, 255, get_hsv_values)
cv2.createTrackbar('V max', 'image', 255, 255, get_hsv_values)


while True:
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get the current HSV values from the trackbars
    h_min = cv2.getTrackbarPos('H min', 'image')
    h_max = cv2.getTrackbarPos('H max', 'image')
    s_min = cv2.getTrackbarPos('S min', 'image')
    s_max = cv2.getTrackbarPos('S max', 'image')
    v_min = cv2.getTrackbarPos('V min', 'image')
    v_max = cv2.getTrackbarPos('V max', 'image')

    # Set the lower and upper HSV limits
    lower_limit = np.array([h_min, s_min, v_min])
    upper_limit = np.array([h_max, s_max, v_max])

    # Create a mask for the specified color range
    mask = cv2.inRange(hsv_image, lower_limit, upper_limit)

    # Apply the mask to the original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Show the image, the mask, and the result
    cv2.imshow("orginal", image)
    cv2.imshow("mask", mask)
    cv2.imshow('result', result)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Print the current HSV values
    print('H min: {}, H max: {}, S min: {}, S max: {}, V min: {}, V max: {}'.format(
        h_min, h_max, s_min, s_max, v_min, v_max))

# Release the resources and close the window
cv2.destroyAllWindows()