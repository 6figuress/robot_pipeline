import json
import math
from camera_sync import Transform, vizPoses
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics


from calibrate import loadCalibration
from mesh_path_stub import MeshPathStub
import os


def generateTrajectoryFromPoses(poses, filename="trajectory.json", graph=False, verbose=False):
    """
    Generate a trajectory.json file from a list of pose

    Parameters:
        - poses A list of pose [x, y, z, qx, qy, qz, qw]
    """

    kine = URKinematics("ur3e_pen_final")

    multi = MultiURKinematics(kine)

    transf = []

    for p in poses:
        transf.append(
            Transform.fromQuaternion(quat=p[3:], tvec=p[:3], scalar_first=False)
        )

    # Those value are written in the technical plan of the duck support
    duck2world: Transform = Transform.fromRodrigues(
        rvec=[0.0, 0.0, 0.0], tvec=[112.57, -147.57, 111.0]
    )

    base2world, grip2cam = loadCalibration(
        "./calibrations/calibration_logitec_with_stand.npz"
    )

    world2base = Transform.fromRodrigues(
        rvec=base2world.invert.rvec,
        tvec=[
            base2world.invert.tvec[0] + 300,
            base2world.invert.tvec[1],
            base2world.invert.tvec[2],
        ],
    )

    test = Transform.fromRodrigues(rvec=[0.0, 0.0, math.pi / 4], tvec=[0.0, 0.0, 0.0])

    world2base = world2base.combine(test)

    duck2robot = duck2world.combine(world2base)

    transformed = []

    for i in range(len(transf)):
        transformed.append(transf[i].combine(duck2robot))

    if graph:
        vizPoses(transformed)

    angles = multi.inverse_optimal([t.kine_pose for t in transformed])

    def generate_trajectory_file(data, filename=f"./trajectories/{filename}.json"):
        modTraj = []
        time_step = 2  # Incrément du temps
        time = 4

        for arr in data:
            positions = [round(float(x), 4) if abs(x) >= 1e-4 else 0.0 for x in arr]
            velocities = [0.0] * 6  # Vélocités à zéro
            modTraj.append(
                {
                    "positions": positions,
                    "velocities": velocities,
                    "time_from_start": [time, 0],
                }
            )
            time += time_step

        with open(filename, "w") as f:
            json.dump({"modTraj": modTraj}, f, indent=4)

        print(f"Trajectory file '{filename}' generated successfully.")

    if verbose:
        number_angles = len(angles.trajectory)
        number_points = len(poses)
        ratio = number_angles / number_points
        print(
            f"Number of poses: {number_points}, Number of angles: {number_angles}, Ratio: {ratio}"
        )
    generate_trajectory_file(angles.trajectory)
