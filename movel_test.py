from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import time

rtde_c = RTDEControl("192.168.5.30")

# Parameters
acceleration = 0.5
dt = 1.0/500  # 2ms
# joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]

# x y z rx ry rz
tool_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Move to initial joint position with a regular moveJ
# rtde_c.moveJ(joint_q)

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
for i in range(1000):
    t_start = rtde_c.initPeriod()
    rtde_c.speedL(tool_speed, acceleration, dt)
    tool_speed[4] += 0.0001
    # joint_speed[1] += 0.0005
    rtde_c.waitPeriod(t_start)

rtde_c.speedStop()
rtde_c.stopScript()
