import rtde_control
import rtde_receive
import rtde_io
import time
import numpy as np
rtde_c = rtde_control.RTDEControlInterface("172.22.22.3")
rtde_r = rtde_receive.RTDEReceiveInterface("172.22.22.3")
print(rtde_r.getActualTCPPose())
cur_pos = rtde_r.getActualTCPPose()
pose = cur_pos.copy()
pose[3:]=[0.5,1.57,1.57]
# pose[3:]=[1.57,0.5,1.57]

rtde_c.moveJ_IK(pose,0.1)
print(rtde_r.getActualTCPPose())
rtde_c.stopScript()
exit()
startpose = rtde_r.getActualTCPPose()
rtde_c.speedL([0,0,-.1,0,0,0],acceleration=10)
time.sleep(.4)
startforce = rtde_r.getActualTCPForce()
while True:
    force = rtde_r.getActualTCPForce()
    if abs(startforce[2]-force[2])>17:
        print("startforce",startforce)
        print("endforce",force)
        break
    time.sleep(0.008)
rtde_c.speedStop()
rtde_c.moveL(startpose,.1,1)
print("DONE")


# exit()

# speed = [0, 0, -0.0100, 0, 0, 0]
# rtde_c.moveUntilContact(speed)
# rtde_c.stopScript()