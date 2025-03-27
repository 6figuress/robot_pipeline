from duck_factory.mesh_to_paths import mesh_to_paths, load_mesh

import json
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics

from calibrate import loadCalibration
from camera_sync import Transform, vizPoses

# mesh = load_mesh("./painting_models/cube/cube_8mm.obj")

# res = mesh_to_paths(mesh, n_samples=50000, verbose=True)


# traj = []

# traj_transf = []

# For now, taking only white trajectory
# for r in res:
#     if r[0] == (255, 255, 255, 255):
#         for pose in r[1]:
#             traj_transf.append(
#                 Transform.fromQuaternion(
#                     quat=pose[1], tvec=np.array(pose[0]) * 1000, scalar_first=False
#                 )
#             )


traj_transf: list[Transform] = [
    Transform.fromQuaternion(quat=[0.0, 1.0, 0.0, 0.0], tvec=[-20.0, 0.0, 400 + 80.0]),
    Transform.fromQuaternion(quat=[0.0, 1.0, 0.0, 0.0], tvec=[-10.0, 0.0, 400 + 80.0]),
    Transform.fromQuaternion(quat=[0.0, 1.0, 0.0, 0.0], tvec=[0.0, 0.0, 400 + 80.0]),
    Transform.fromQuaternion(quat=[0.0, 1.0, 0.0, 0.0], tvec=[10.0, 0.0, 400 + 80.0]),
    Transform.fromQuaternion(quat=[0.0, 1.0, 0.0, 0.0], tvec=[20.0, 0.0, 400 + 80.0]),
]

# vizPoses(traj_transf, limits=(-50, 50), length=5)


base2world, grip2cam = loadCalibration(
    "./calibrations/calibration_logitec_with_stand.npz"
)


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(
    rvec=[0.0, 0.0, 0.0], tvec=[112.57, -147.57, 111.0]
)

# TODO: Remove this line !!!!
# duck2world = Transform.fromRodrigues(
#     rvec=[0.0, 0.0, 0.0], tvec=[117.57, -152.57, 250.0]
# )

duck2robot = duck2world.combine(base2world.invert)


transformed = []

for i in range(len(traj_transf)):
    transformed.append(traj_transf[i].combine(duck2robot))


# vizPoses(transformed)

kine = URKinematics("ur3e_with_pen_final")

multi = MultiURKinematics(kine)


import ipdb

ipdb.set_trace()

angles = multi.inverse_optimal([t.kine_pose for t in transformed])

angles.trajectory = np.array(angles.trajectory)

print(json.dumps(angles.trajectory.tolist()))