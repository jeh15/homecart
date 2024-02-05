import cv2
import numpy as np
from numpy.linalg import inv

if __name__ == '__main__' :
    # points on pixel plane
    pO=[441, 44]
    pX=[238, 49]
    pY=[238, 164]
    pP=[443,157]

    # Four corners of the book in source image
    pts_src = np.array([pO,pX,pP,pY])

    # points on real plane
    O=[0.0 , 0.065]
    X=[0.29, 0.065]
    Y=[0.29, -0.065]
    P=[0.0 , -0.065]

    pts_dst = np.array([O,X,P,Y])

    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)

    print("homography:",h)
    # print("inv of H:",inv(h))

    pixel_pentip = np.array([[247.0],[172.0],[1.0]])
    real_pentip = np.dot(h,pixel_pentip)
    real_pentip = real_pentip/real_pentip[2]
    print("position of the blob on the ground xy plane:\n",real_pentip[0:2])
    