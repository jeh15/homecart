import cv2
import numpy as np
from numpy.linalg import inv

if __name__ == '__main__' :
    # points on pixel plane
    pO=[93,  5]
    pX=[360, 6]
    pY=[85,  277]
    pP=[482,252]

    # Four corners of the book in source image
    pts_src = np.array([pO,pX,pP,pY])

    # points on real plane
    O=[0.0,0.0]
    X=[-0.56,0.0]
    Y=[0.0,0.425]
    P=[-0.79,0.425]

    pts_dst = np.array([O,X,P,Y])

    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)

    print("homography:",h)
    # print("inv of H:",inv(h))

    pixel_pentip = np.array([[252.0],[89.0],[1.0]])
    real_pentip = np.dot(h,pixel_pentip)
    real_pentip = real_pentip/real_pentip[2]
    print("position of the blob on the ground xy plane:",real_pentip[0:2])
    