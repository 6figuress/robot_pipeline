import json
import math
from camera_sync import Transform, vizPoses
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics


from calibrate import loadCalibration

def convert_poses_to_transforms(poses):
    return [
        Transform.fromQuaternion(
            quat=p[3:], tvec=np.array(p[:3]) * 1000, scalar_first=False
        )
        for p in poses
    ]

def get_duck_to_robot_transform():
    duck2world = Transform.fromRodrigues(
        rvec=[0.0, 0.0, 0.0], tvec=[112.57, -147.57, 111.0]
    )
    base2world, grip2cam = loadCalibration(
        "./calibrations/calibration_logitec_with_stand.npz"
    )
    world2base = base2world.invert
    return duck2world.combine(world2base)

def apply_transform_chain(transforms, duck2robot):
    return [t.combine(duck2robot) for t in transforms]

def generate_trajectory_file(data, filename):
    filename = f"./trajectories/{filename}"
    modTraj = []
    time_step = 1_000_000_000  # Incrément du temps [us]
    time = 4_000_000_000

    for arr in data:
        positions = [round(float(x), 4) if abs(x) >= 1e-4 else 0.0 for x in arr]
        velocities = [0.0] * 6  # Vélocités à zéro
        ns_time = time % 1_000_000_000
        s_time = int((time - ns_time)/1_000_000_000)
        modTraj.append(
            {
                "positions": positions,
                "velocities": velocities,
                "time_from_start": [s_time, ns_time],
            }
        )
        time += time_step

    with open(filename, "w") as f:
        json.dump({"modTraj": modTraj}, f, indent=4)

    print(f"Trajectory file '{filename}' generated successfully.")


def generateTrajectoryFromPoses(poses, filename="trajectory.json", graph=False, verbose=False):
    """
    Generate a trajectory.json file from a list of pose

    Parameters:
        - poses A list of pose [x, y, z, qx, qy, qz, qw]
    """

    kine = URKinematics("ur3e_pen_final")
    multi = MultiURKinematics(kine)

    transf = convert_poses_to_transforms(poses)
    duck2robot = get_duck_to_robot_transform()
    transformed = apply_transform_chain(transf, duck2robot)

    if graph:
        vizPoses(transformed)

    angles = multi.inverse_optimal([t.kine_pose for t in transformed])

    generate_trajectory_file(angles.trajectory, filename)

    if verbose:
        number_angles = len(angles.trajectory)
        number_points = len(poses)
        ratio = number_angles / number_points
        print(
            f"Number of poses: {number_points}, Number of angles: {number_angles}, Ratio: {ratio}"
        )


def generateTrajectoryFromMultiplePoses(list_poses, filename="trajectory.json", graph=False, verbose=False):
    kine = URKinematics("ur3e_pen_final")
    multi = MultiURKinematics(kine)
    duck2robot = get_duck_to_robot_transform()

    angles = []
    num_poses = 0
    for poses in list_poses:
        num_poses += len(poses)
        transf = convert_poses_to_transforms(poses)
        transformed = apply_transform_chain(transf, duck2robot)
        angles.extend(multi.inverse_optimal([t.kine_pose for t in transformed], logs=False).trajectory)

    if graph:
        for transf in transformed:
            vizPoses(transf)
    
    generate_trajectory_file(angles, filename)

    if verbose:
        number_angles = len(angles)
        ratio = number_angles / num_poses
        print(
            f"Number of poses: {num_poses}, Number of angles: {number_angles}, Ratio: {ratio}"
        )

