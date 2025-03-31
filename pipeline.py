import json
from camera_sync import Transform, vizPoses
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics


from calibrate import loadCalibration
from mesh_path_stub import MeshPathStub
import os

# mesh = load_mesh("./painting_models/cube/cube_8mm.obj")

# res = mesh_to_paths(mesh, n_samples=50000, verbose=True)

res = []


traj_transf = []

# for r in res:
#     for pose in r[1]:
#         traj_transf.append(
#             Transform.fromQuaternion(
#                 quat=pose[1], tvec=np.array(pose[0]) * 1000, scalar_first=False
#             )
#         )


stub: MeshPathStub = MeshPathStub()

trajectories: list[Transform] = traj_transf  # stub.line_on_cube(n_points=10)

base2world, grip2cam = loadCalibration(
    "./calibrations/calibration_logitec_with_stand.npz"
)


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(
    rvec=[0.0, 0.0, 0.0], tvec=[112.57, -147.57, 111.0]
)

duck2robot = duck2world.combine(base2world.invert)


transformed = []

for i in range(len(trajectories)):
    transformed.append(trajectories[i].combine(duck2robot))


kine = URKinematics("ur3e_pen_final")

multi = MultiURKinematics(kine)

angles = multi.inverse_optimal([t.kine_pose for t in transformed])


def generate_trajectory_file(data, filename="./trajectories/trajectory.json"):
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


# generate_trajectory_file(angles.trajectory)

print(json.dumps(angles.trajectory))
