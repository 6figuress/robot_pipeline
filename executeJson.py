import json
from math import radians
import os
from urbasic import ISCoin, CameraSettings, FocusSettings, Joint6D


iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)


def executeJson(filepath):
    def readJson(path):
        points = []
        with open(os.path.join("./trajectories", path), "r") as file:
            data = json.load(file)["modTraj"]
            for i in data:
                points.append(i["positions"])
        return points

    traj = readJson("trajectory_duck_front.json")

    acc = radians(60)

    speed = radians(5)

    for p in traj:
        iscoin.robot_control.movej(Joint6D.createFromRadList(p), a=acc, v=speed)


def executeSidedJson(folderPath, order=["front", "left", "back", "right", "top"]):
    for side in order:
        path: str = os.path.join(folderPath, f"{side}")
        if not os.path.exists(path):
            raise Exception(f"Folder {path} does not exist.")

        print(
            f"\nPREPARING TO PAINT SIDE {side} - ENSURE DUCK IS PROPERLY ROTATED AND PRESS ENTER"
        )
        input()
        print(f"Executing {side} side")
        executeColoredJson(path)
    pass


def executeColoredJson(folderPath):
    files = [f for f in os.listdir(folderPath) if f.endswith(".json")]

    for f in files:
        # TODO: Change color here !
        color = f.split(".")[0]

        path = os.path.join(folderPath, f)
        print(f"Executing {f}")
        executeJson(path)
        print(f"Finished executing {f}")
    pass
