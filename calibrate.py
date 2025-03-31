import math
import os
import cv2 as cv
from cv2.typing import Vec6f, MatLike, Vec3f
import csv
import matplotlib.pyplot as plt

import numpy as np

from ur_ikfast.ur_kinematics import URKinematics

from camera_sync import vizPoses
from camera_sync.referential import Transform
from camera_sync.camera import Camera
from camera_sync.aruco import processAruco, getArucosFromPaper
from takePicture import PICS_PATH, READING_PATH


to_debug = []

CALIBRATION_FOLDER = "calibrations"

# ROBOT_CAM = Camera("robot", -1, focus=500, resolution=(640, 480))

ROBOT_CAM = Camera("Logitec_robot", -1, focus=10, resolution=(1920, 1080))


def readCSV(filePath=READING_PATH) -> list[tuple[Vec6f, MatLike]]:
    with open(filePath, "r") as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader, None)

        print("Header is : ", header)

        poses = []

        for row in csv_reader:
            angles = np.array(row[:6], dtype=np.float32)
            # TODO: Replace with new format
            # picture = cv.imread(os.path.join(PICS_PATH, row[8]) + ".png")
            picture = cv.imread(os.path.join(PICS_PATH, row[6]))
            poses.append((angles, picture))

    return poses


def processPoses(
    poses: list[tuple[Vec6f, MatLike]],
    camera: Camera,
    RMSE_treshold: float = 2.0,
    MAE_treshold: float = 2.0,
    show_graph=True,
    limit=3,
) -> tuple[list[Transform], list[Transform]]:
    robot_poses: list[Transform] = []
    camera_poses: list[Transform] = []

    kinematic = URKinematics("ur3e_pen_final_2")

    arucos = getArucosFromPaper(4).values()

    x_RMSE = []
    y_MAE = []

    i = 0

    for angles, frame in poses:
        i += 1
        if i > limit:
            break
        cam_rvec, cam_tvec, _, metrics = processAruco(
            arucos, [], camera, frame, metrics=True
        )

        to_debug.append((cam_rvec, cam_tvec, frame))

        x_RMSE.append(metrics["PnP"]["RMSE"])
        y_MAE.append(metrics["PnP"]["MAE"])

        if (
            metrics["PnP"]["RMSE"] > RMSE_treshold
            or metrics["PnP"]["MAE"] > MAE_treshold
        ):
            # Error is too big, we won't take the pose for calibration
            continue

        res = kinematic.forward(angles)

        robot_poses.append(
            Transform.fromQuaternion(
                quat=res[3:], tvec=res[:3] * 1000, scalar_first=False
            )
        )
        # TODO: Here is a conversion from mm to M -> don't forget it !!!
        camera_poses.append(Transform.fromRodrigues(rvec=cam_rvec, tvec=cam_tvec))
    if show_graph:
        plt.figure(figsize=(10, 6))
        plt.scatter(x_RMSE, y_MAE, c="blue", alpha=0.6, edgecolors="w")
        for i, (x, y) in enumerate(zip(x_RMSE, y_MAE)):
            plt.text(x - 0.01, y + 0.02, str(i))
        plt.xlabel("Reprojection Error (RMSE)")
        plt.ylabel("Mean Absolute Error (MAE)")
        plt.title("PnP Error Metrics")
        plt.grid(True, alpha=0.3)

        # Add a horizontal and vertical line at the thresholds
        plt.axhline(
            y=MAE_treshold,
            color="r",
            linestyle="--",
            label=f"MAE Threshold ({MAE_treshold})",
        )
        plt.axvline(
            x=RMSE_treshold,
            color="g",
            linestyle="--",
            label=f"RMSE Threshold ({RMSE_treshold})",
        )
        plt.axis("equal")
        plt.legend()
        plt.show()

    return robot_poses, camera_poses


def calibrate(base2gripper: list[Transform], world2camera: list[Transform]):
    robot_rvec, robot_tvec = (
        [p.rvec for p in base2gripper],
        [p.tvec for p in base2gripper],
    )

    camera_rvec, camera_tvec = (
        [p.rvec for p in world2camera],
        [p.tvec for p in world2camera],
    )

    r_base2world, t_base2world, r_grip2cam, t_grip2cam = cv.calibrateRobotWorldHandEye(
        camera_rvec,
        camera_tvec,
        robot_rvec,
        robot_tvec,
        method=cv.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
    )

    base2world: Transform = Transform.fromRotationMatrix(
        rot_mat=r_base2world, tvec=t_base2world
    )
    grip2cam: Transform = Transform.fromRotationMatrix(
        rot_mat=r_grip2cam, tvec=t_grip2cam
    )

    return base2world, grip2cam


def evaluate(
    world2base: Transform,
    base2gripper: list[Transform],
    grip2cam: Transform,
    camera2world: list[Transform],
    report: bool = True,
):
    metrics = {
        "total": [],
        "x": [],
        "y": [],
        "z": [],
    }

    def log(txt):
        if report:
            print(txt)

    mapping = ["x", "y", "z"]

    log("============= REPORT =============")

    for j, (b2g, c2w) in enumerate(zip(base2gripper, camera2world)):
        total: Transform = world2base.combine(b2g).combine(grip2cam).combine(c2w)

        point: Vec3f = np.array([0.0, 0.0, 0.0])

        transformed = total.apply(point)

        rvec = to_debug[j][0]
        tvec = to_debug[j][1]
        frame = to_debug[j][2]

        aruco_corners = []
        transformed_points = []

        for a in getArucosFromPaper(4).values():
            for arucoCorner in a.corners:
                aruco_corners.append(arucoCorner.coords)
                transformed_points.append(total.apply(arucoCorner.coords))

        aruco_corners = np.array(aruco_corners).astype(np.float32)
        transformed_points = np.array(transformed_points).astype(np.float32)

        print("Shape : ", aruco_corners.shape)

        img_points, _ = cv.projectPoints(
            transformed_points, rvec, tvec, ROBOT_CAM.mtx, ROBOT_CAM.dist
        )
        origin, _ = cv.projectPoints(
            aruco_corners, rvec, tvec, ROBOT_CAM.mtx, ROBOT_CAM.dist
        )

        for base, transf in zip(origin, img_points):
            cv.circle(frame, np.rint(transf.ravel()).astype(np.int32), 3, (0, 255, 0))

            cv.circle(frame, np.rint(base.ravel()).astype(np.int32), 6, (0, 0, 255))

        cv.imwrite("./results/" + str(j) + ".png", frame)

        total = 0

        for i, m in enumerate(mapping):
            currErr = abs(transformed[i] - point[i])
            metrics[m].append(currErr)
            total += currErr**2

        metrics["total"].append(math.sqrt(total))
        log(f"Pose n°{j} : ")
        log(f"    Total : {metrics['total'][-1]}")
        log(f"        X : {metrics['x'][-1]}")
        log(f"        Y : {metrics['y'][-1]}")
        log(f"        Z : {metrics['z'][-1]}")

    for k in metrics.keys():
        metrics[k] = np.array(metrics[k])

    log("============= END REPORT =============")

    return metrics


def saveCalibration(base2world: Transform, grip2cam: Transform, filepath: str):
    np.savez(filepath, base2world=base2world.transf_mat, grip2cam=grip2cam.transf_mat)
    pass


def loadCalibration(filepath: str) -> tuple[Transform, Transform]:
    file = np.load(filepath)
    base2world = Transform(transf_mat=file["base2world"])
    grip2cam = Transform(transf_mat=file["grip2cam"])
    return base2world, grip2cam


if __name__ == "__main__":
    poses = readCSV()

    robot_poses, camera_poses = processPoses(
        poses, ROBOT_CAM, show_graph=True, limit=50, RMSE_treshold=4.5, MAE_treshold=4.5
    )

    vizPoses(robot_poses)

    cam2world = [p.invert for p in camera_poses]

    base2world, grip2cam = calibrate(robot_poses, cam2world)

    # TODO: Do not forgetThis is the inversion
    old = Transform(base2world.transf_mat)

    base2world = grip2cam

    grip2cam = old

    # End
    import ipdb

    ipdb.set_trace()

    def showCameraPosition():
        gripInWorld = robot_poses[0].combine(base2world)

        cam_guessed = grip2cam.invert.combine(robot_poses[0]).combine(base2world)

        total = (
            camera_poses[0]
            .combine(grip2cam.invert)
            .combine(robot_poses[0])
            .combine(base2world)
        )

        total2 = (
            base2world.invert.combine(robot_poses[0].invert)
            .combine(grip2cam)
            .combine(camera_poses[0].invert)
        )

        vizPoses([gripInWorld, base2world, cam2world[0], cam_guessed])

    showCameraPosition()

    metrics = evaluate(
        base2world.invert,
        [p.invert for p in robot_poses],
        grip2cam,
        cam2world,
        report=True,
    )

    error = np.mean(metrics["total"])

    print(f"Error is {error}")

    saveCalibration(
        base2world, grip2cam, CALIBRATION_FOLDER + "/calibration_logitec_with_stand.npz"
    )