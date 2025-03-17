import csv
from math import radians
from URBasic import ISCoin, CameraSettings, Joint6D
from URBasic.devices.camera_settings import FocusSettings

import cv2 as cv
from cv2.typing import Vec6f
import numpy as np

from camera_sync.camera import Camera

import os
import time
import requests
import xmlrpc.client
import json


READING_FOLDER_PATH = "./readings"
PICS_PATH = "./pics"
READING_PATH = os.path.join(READING_FOLDER_PATH, "reading.csv")


# TODO: This method is a dumb copy of a method in the jupyter notebook in ur3e control.
# When read, we need to find a good place for it -> maybe directly in the control library ?
def readJson(path):
    points = []
    with open(path, "r") as file:
        data = json.load(file)["modTraj"]
        for i in data:
            points.append(i["positions"])
    return points


def get_vs(uri) -> tuple[str, dict]:
    response = requests.get("http://10.30.5.158:4242/" + uri)
    response.raise_for_status()
    return response.json()


def startup():
    # rq_xmlrpcserver
    print("rq_xmlrpcserver")
    with xmlrpc.client.ServerProxy("http://10.30.5.158:4250/") as proxy:
        try:
            print(proxy.getserverversion())
        except xmlrpc.client.Fault as err:
            print("A fault occurred")
            print("Fault code: %d" % err.faultCode)
            print("Fault string: %s" % err.faultString)

    # vision_server
    print("\n\nvision_server")
    for cmd in [
        "getcamerasettings",
        # 'setcamerasettings', 'setdefaultcamerasettings',
        # 'status', 'getbarcodelicense', 'hello',
        # 'create', 'resetzoomregion', 'setdisplaysize', 'process',
        # 'trigger', 'triggerinprogress', 'resetserverstate',
        # 'beginbarcode', 'rejectmodule', 'endoperationmode', 'streamoutmodule',
        # 'beginoperationmode', 'endoperationmode', 'streaminmodule',
    ]:
        try:
            print(f"{cmd}:\n{json.dumps(get_vs(cmd), indent=2, sort_keys=True)}")
        except requests.exceptions.HTTPError as err:
            print("HTTP error occurred: %s" % err)
        except Exception as err:
            print("Other error occurred: %s" % err)

    # create readings folder if it does not exist
    if not os.path.exists(READING_FOLDER_PATH):
        os.makedirs(READING_FOLDER_PATH)

    if not os.path.exists(READING_PATH):
        with open(READING_PATH, "w") as f:
            f.write("joint1,joint2,joint3,joint4,joint5,joint6,timestamp\n")


def setFocus(robot: ISCoin, cam: Camera):
    cs = CameraSettings()
    cs.focusSettings.setManualMode(FocusSettings.FocusValue(cam._focus))
    robot.camera.setCameraSettings(cs)


def showStream(robot: ISCoin):
    robot.displayCameraStreamOCVParallel()


def savePose(robot: ISCoin, filename: str):
    # filename = takePic(PICS_PATH)

    joints = robot.robot_control.get_actual_joint_positions()
    j1, j2, j3, j4, j5, j6 = (
        joints.j1,
        joints.j2,
        joints.j3,
        joints.j4,
        joints.j5,
        joints.j6,
    )

    with open(READING_PATH, "a") as f:
        f.write(f"{j1},{j2},{j3},{j4},{j5},{j6},{filename}\n")


def takePic(robot: ISCoin, folder: str) -> str:
    timestamp = int(time.time())

    pic = robot.camera.getImageAsImage()
    pic.save(os.path.join(folder, f"{timestamp}.png"))
    return f"{timestamp}.png"


def takePicCamera(cam: Camera, folder: str) -> str:
    timestamp = int(time.time())

    pic = cam.takePic()
    cv.imwrite(os.path.join(folder, f"{timestamp}.png"), pic)
    return f"{timestamp}.png"


def readPosesFromFile(filepath: str) -> np.ndarray[Vec6f]:
    return np.array([[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]])


def readPosesFromCSV(path: str) -> np.ndarray[Vec6f]:
    with open(path, "r") as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader, None)

        print("Header is : ", header)

        poses = []

        for row in csv_reader:
            angles = np.array(row[:6], dtype=np.float32)
            poses.append(angles)
    return angles


if __name__ == "__main__":
    # TODO: We can try to set a closer focus distance

    cam = Camera("Logitec_robot", 2, focus=10, resolution=(1920, 1080))

    iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)

    time.sleep(1)

    i = 0
    while True:
        ret, frame = cam.captureStream.read()
        if not ret:
            raise Exception("Uh oh ")
        cv.imshow("frame", frame)
        key = cv.waitKey(1)

        if key == ord("q"):
            break
        elif key == ord("s") or key == ord("c"):
            filename = takePicCamera(cam, "./pics")

            savePose(iscoin, filename)
            print(f"Picture n°{i} taken and pose saved")
            i += 1

    # robot_cam = Camera("robot", -1, focus=500, resolution=(640, 480))

    # startup()

    # setFocus(iscoin, robot_cam)

    # time.sleep(2)

    # showStream(iscoin)

    # angles = readPosesFromFile("")

    # acc = radians(120)

    # speed = radians(50)

    # for angle in angles:
    #     iscoin.robot_control.movej(Joint6D.createFromRadList(angle), a=acc, v=speed)
    #     # TODO: Check if a delay is needed or not -> can display the stream in paralel to see if robot need time for pose to settle
    #     time.sleep(0.5)
    #     savePose()
