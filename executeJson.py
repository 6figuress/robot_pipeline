import json
from math import radians
import os
from urbasic import ISCoin, CameraSettings, FocusSettings, Joint6D


iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)


# iscoin.gripper.activate()


def readJson(path):
    points = []
    with open(os.path.join("./trajectories", path), "r") as file:
        data = json.load(file)["modTraj"]
        for i in data:
            points.append(i["positions"])
    return points


# import ipdb
# ipdb.set_trace()

traj = readJson("trajectory_duck_front.json")

acc = radians(60)

speed = radians(5)

for p in traj:
    iscoin.robot_control.movej(Joint6D.createFromRadList(p), a=acc, v=speed)
