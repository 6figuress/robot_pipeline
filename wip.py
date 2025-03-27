from math import radians
from urbasic import ISCoin, CameraSettings, FocusSettings, Joint6D


iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)


iscoin.gripper.activate()

import ipdb
ipdb.set_trace()

angles = [1.16553354, -1.57083261, 1.50438511, -1.50515127, -1.56908202, 2.73782039]
acc = radians(60)

speed = radians(5)

iscoin.robot_control.movej(Joint6D.createFromRadList(angles), a=acc, v=speed)
# TODO: Check if a delay is needed or not -> can display the stream in paralel to see if robot need time for pose to settle