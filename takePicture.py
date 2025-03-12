from URBasic import ISCoin, CameraSettings
from URBasic.devices.camera_settings import FocusSettings

from ur_ikfast import ur_kinematics

from camera_sync.camera import Camera
from camera_sync.referential import Transform
from camera_sync.plotting import vizPoses

import numpy as np
import os
import time
import requests
import xmlrpc.client
import json


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
    if not os.path.exists("readings"):
        os.makedirs("readings")



def setFocus(robot: ISCoin):
    cs = CameraSettings()
    cs.focusSettings.setManualMode(FocusSettings.FocusValue(500))
    robot.camera.setCameraSettings(cs)

def showStream(robot: ISCoin):
    robot.displayCameraStreamOCVParallel() # 


def fk(angles)-> Transform:
    arm = ur_kinematics.URKinematics("ur3e")
    
    # Copy the 3x4 transformation values into the 4x4 matrix
    mat = arm.forward(angles, rotation_type="matrix")
    return Transform(rot_mat=mat[:3, :3], tvec=mat[:3, 3])
    


def readRobot(robot: ISCoin):
    timestamp = int(time.time())

    pic = robot.camera.getImageAsImage()
    pic.save(f"pics/{timestamp}.png")

    joints = robot.robot_control.get_actual_joint_positions()
    j1, j2, j3, j4, j5, j6 = (
        joints.j1,
        joints.j2,
        joints.j3,
        joints.j4,
        joints.j5,
        joints.j6,
    )

    t = fk(joints.toList())


    with open("readings/reading.csv", "a") as f:
        f.write(f"{j1},{j2},{j3},{j4},{j5},{j6},{t.rvec.ravel()},{t.tvec},{timestamp}\n")

    # vizPoses([t], limits=(-1, 1), length=0.2)


def benchmarkIk():
    robot_name = 'ur3e'
    arm = ur_kinematics.URKinematics(robot_name)
    ikfast_res = 0
    total = 10000

    for i in range(total):
        joint_angles = np.random.uniform(-1*np.pi, 1*np.pi, size=6)
        pose = arm.forward(joint_angles)
        ik_solution = arm.inverse(pose, False, q_guess=joint_angles)
        if ik_solution is not None:
            ikfast_res += 1 if np.allclose(joint_angles, ik_solution, rtol=0.01) else 0
    print("IKFAST success rate %s of %s" % (ikfast_res, total))
    print("percentage %.1f", ikfast_res/float(total)*100.)



robot_cam = Camera("robot", -1, focus=500, resolution=(640, 480))



startup()


iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)

# showStream(iscoin)
# time.sleep(1)

setFocus(iscoin)



time.sleep(2)



# joints = iscoin.robot_control.get_actual_joint_positions().toList()



# t = fk(joints)

readRobot(iscoin)



# T is current robot pose

# vizPoses([t], limits=(-1, 1), length=0.2)

# iscoin.camera.resetCameraSettings()

