from duck_factory.mesh_to_paths import mesh_to_paths, load_mesh

import numpy as np
from ur_ikfast.ur_kinematics import URKinematics

from calibrate import loadCalibration
from camera_sync import Transform, vizPoses

import matplotlib.pyplot as plt

plt.ion()

# mesh = load_mesh("./duck_model/DuckComplete.obj")

# res = mesh_to_paths(mesh)

# traj = []


# # For now, taking only white trajectory
# for r in res:
#     if r[0] == (255, 255, 255, 255):
#         traj.append(r[1])


base2world, grip2cam = loadCalibration("./calibrations/calibration_logitec.npz")


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(rvec= [.0, .0, .0], tvec=[117.57, -152.57, 111.0])

duck2robot: Transform = duck2world.combine(base2world.invert)

duck_origin = [0.0, 150.0, 0.0]

duck_origin_robot_frame = duck2robot.apply(duck_origin)

kine = URKinematics("ur3e")


position = [
    duck_origin_robot_frame[0],
    duck_origin_robot_frame[1],
    duck_origin_robot_frame[2],
    1,
    0,
    0,
    0,
]


pose = kine.inverse(position)


vizPoses([duck2world])