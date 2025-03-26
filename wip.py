from math import radians
from urbasic import ISCoin, CameraSettings, FocusSettings, Joint6D


iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)


iscoin.gripper.activate()

import ipdb
ipdb.set_trace()

angles = [ 1.26185775, -1.33791804,  1.85079265, -2.08357739, -1.57184052,  2.83136392]

acc = radians(60)

speed = radians(20)

iscoin.robot_control.movej(Joint6D.createFromRadList(angles), a=acc, v=speed)
# TODO: Check if a delay is needed or not -> can display the stream in paralel to see if robot need time for pose to settle