import time

import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

import pdb


def main(argv=None):
    # Initialize robot:
    rtde_c = RTDEControl("192.168.5.30")
    rtde_r= rtde_receive.RTDEReceiveInterface("192.168.5.30")

    # Parameters
    speed = 0.1
    acceleration = 0.1

    # Get initial robot state:
    initial_pose = rtde_r.getActualTCPPose()
    initial_pose = np.asarray(initial_pose)

    def rotation(pose, theta, off_set):
        phi = off_set + theta
        R = np.array([
            [np.cos(phi), 0, np.sin(phi)],
            [0, 1, 0],
            [-np.sin(phi), 0, np.cos(phi)],
        ])
        desired_pose = pose @ R 
        return desired_pose
    
    off_set = initial_pose[4]

    for i in range(0, 100):
        theta = np.sin(i / 10)
        desired_pose = rotation(initial_pose[:3], theta, off_set)
        pose = np.concatenate([desired_pose, initial_pose[3:]])
        
        print(f"Pose: {pose}")
        
        rtde_c.moveL(pose, speed, acceleration)
        time.sleep(0.1)


if __name__ == "__main__":
    main()
