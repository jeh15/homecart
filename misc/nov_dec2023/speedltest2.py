from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import time

rtde_c = RTDEControl("192.168.5.30")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

t_initial = time.time()
acceleration = 0.2
tool_speed = [0.0, 0.0, 0.0, 0.0, 0.1, 0.0]

while time.time() - t_initial < 1.0:
    rtde_c.speedL(tool_speed, acceleration, 0.1)
    bp = rtde_r.getActualTCPPose()[4] - (22.0/7.0)
    if bp < 0.0:
        bp += 2.0*(22.0/7.0)
    print('pos:',bp)
    time.sleep(0.01)

rtde_c.speedStop()
rtde_c.stopScript()
