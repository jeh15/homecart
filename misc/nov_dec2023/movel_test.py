from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import time

rtde_c = RTDEControl("192.168.5.30")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.5.30")

# # Parameters
target = rtde_r.getActualTCPPose()

print('#0:',rtde_r.getActualTCPPose()[4])

target[4] -= 0.1
rtde_c.moveL(target, 2.0, 2.0, True)
time.sleep(0.1)
rtde_c.stopL(2.0)

print('#1:',rtde_r.getActualTCPPose()[4])

target[4] -= 0.1
rtde_c.moveL(target, 2.0, 2.0, True)
time.sleep(0.1)
rtde_c.stopL(2.0)

print('#2:',rtde_r.getActualTCPPose()[4])

target[4] -= 0.1
rtde_c.moveL(target, 2.0, 2.0, True)
time.sleep(0.1)
rtde_c.stopL(2.0)

print('#3:',rtde_r.getActualTCPPose()[4])

target[4] -= 0.1
rtde_c.moveL(target, 2.0, 2.0, True)
time.sleep(0.1)
rtde_c.stopL(2.0)

print('#4:',rtde_r.getActualTCPPose()[4])


# rtde_c.stopScript()
