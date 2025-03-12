import cv2 as cv
import csv

import numpy as np

from camera_sync.referential import Transform
from camera_sync.camera import Camera
from camera_sync.aruco import processAruco, getArucosFromPaper
from camera_sync.plotting import vizPoses

# Path to the file
file_path = './readings/reading.csv'

picsPath = "./pics"

robot_cam = Camera("robot", -1, focus=500, resolution=(640, 480))

# Open the file and read line by line
with open(file_path, 'r') as file:
    csv_reader = csv.reader(file)
    
    # Read the header
    header = next(csv_reader, None)
    if header:
        print(f"Header: {header}")
    
    robot_poses: list[Transform] = []
    cam_poses: list[Transform] = []

    robot_tvec = []
    robot_rvec = []
    camera_rvec = []
    camera_tvec = []

    i = 0
    # Read remaining lines
    for row in csv_reader:
        # if i > 2:
        #     break
        i+=1
        row[6] = " ".join(row[6].split())
        row[7] = " ".join(row[7].split())
        rvec = np.fromstring(row[6].strip("[]"), sep=" ")
        tvec = np.fromstring(row[7].strip("[]"), sep=" ")
        pose = np.array(row[:6], dtype=np.float32)
        rPose = Transform(rvec=rvec, tvec=tvec)
        robot_tvec.append(rPose.tvec)
        robot_rvec.append(rPose.rvec)
        robot_poses.append(rPose)
        pic = cv.imread(f"{picsPath}/{row[8]}.png")

        rvec, tvec, _, _ = processAruco(getArucosFromPaper().values(), [], robot_cam, pic)

        camera_rvec.append(rvec)
        camera_tvec.append(tvec / 1000)
        cam_poses.append(Transform(rvec=rvec, tvec=tvec/1000).invert)




    print(len(camera_rvec))

    rot_base2world, tvec_base2world, rot_grip2cam, tvec_grip2cam = (
    cv.calibrateRobotWorldHandEye(
        camera_rvec,
        camera_tvec,
        robot_rvec,
        robot_tvec,
        method=cv.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
    )
    )


    base2world = Transform(rot_mat=rot_base2world, tvec=tvec_base2world)

    # vizPoses([base2world.invertq], limits=(-1, 1), length=0.2)


    grip2cam = Transform(rot_mat=rot_grip2cam, tvec=tvec_grip2cam)

    world2base = base2world.invert

    base2gripper = robot_poses[-2]

    cam2world = cam_poses[-2]

    # Eh y'a un monde ou ça marche hein

    origin = np.array([.0, .0, .0])

    total = world2base.combine(base2gripper).combine(grip2cam).combine(cam2world)


    t1 = world2base.apply(origin)

    t2 = base2gripper.apply(t1)




    print(total.transf_mat)

    print("")
        

        