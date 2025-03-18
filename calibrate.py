import math
import os
import cv2 as cv
from cv2.typing import Vec6f, MatLike, Vec3f
import csv
import matplotlib.pyplot as plt

import numpy as np

from ur_ikfast.ur_kinematics import URKinematics

from camera_sync.referential import Transform
from camera_sync.camera import Camera
from camera_sync.aruco import processAruco, getArucosFromPaper
from camera_sync.plotting import vizPoses
from takePicture import PICS_PATH, READING_PATH


to_debug = []

CALIBRATION_FOLDER = "calibrations"

# ROBOT_CAM = Camera("robot", -1, focus=500, resolution=(640, 480))

ROBOT_CAM = Camera("Logitec_robot", -1, focus=10, resolution=(1920, 1080))


def readCSV() -> list[tuple[Vec6f, MatLike]]:
    with open(READING_PATH, "r") as file:
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


def xzPlaneSymetry(t: Transform) -> Transform:
    s = np.eye(4)
    s[1, 1] = -1
    return Transform(transf_mat=s @ t.transf_mat)


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

    kinematic = URKinematics("ur3e")

    arucos = getArucosFromPaper(2).values()

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

        mat = kinematic.forward(angles, rotation_type="matrix")

        robot_poses.append(
            Transform.fromRotationMatrix(rot_mat=mat[:3, :3], tvec=mat[:3, 3] * 1000)
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

    base2world = Transform.fromRotationMatrix(rot_mat=r_base2world, tvec=t_base2world)
    grip2cam = Transform.fromRotationMatrix(rot_mat=r_grip2cam, tvec=t_grip2cam)

    return base2world, grip2cam


def evaluate(
    base2gripper: list[Transform],
    world2camera: list[Transform],
    base2world: Transform,
    grip2cam: Transform,
    report: bool = True,
):
    world2base = base2world.invert

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

    for j, (r, c) in enumerate(zip(base2gripper, world2camera)):
        total = world2base.combine(r).combine(grip2cam).combine(c)

        point: Vec3f = np.array([0.0, 0.0, 0.0])

        transformed = total.apply(point)

        rvec = to_debug[j][0]
        tvec = to_debug[j][1]
        frame = to_debug[j][2]

        aruco_corners = []
        transformed_points = []

        for a in getArucosFromPaper(2).values():
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

        total_transf = world2base.combine(r).combine(grip2cam).combine(c)

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
        poses, ROBOT_CAM, show_graph=False, limit=50
    )

    cam2world = [p.invert for p in camera_poses]

    base2world, grip2cam = calibrate(robot_poses, cam2world)

    metrics = evaluate(robot_poses, camera_poses, base2world, grip2cam, report=False)

    error = np.mean(metrics["total"])

    print(f"Error is {error}")

    saveCalibration(
        base2world, grip2cam, CALIBRATION_FOLDER + "/calibration_logitec.npz"
    )