from duck_factory.mesh_to_paths import mesh_to_paths, load_mesh

import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics

from calibrate import loadCalibration
from camera_sync import Transform, vizPoses

import matplotlib.pyplot as plt

plt.ion()

mesh = load_mesh("./painting_models/cube/cube_8mm.obj")

res = mesh_to_paths(mesh)

traj = []


traj_transf: list[Transform] = []

# For now, taking only white trajectory
for r in res:
    if r[0] == (255, 255, 255, 255):
        for pose in r[1]:
            traj_transf.append(
                Transform.fromQuaternion(quat=pose[1], tvec=pose[0], scalar_first=False)
            )

base2world, grip2cam = loadCalibration("./calibrations/calibration_logitec_new.npz")


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(rvec= [.0, .0, .0], tvec=[117.57, -152.57, 111.0])

for i in range(len(traj_transf)):
    traj_transf[i] = base2world.combine(traj_transf[i])

duck2robot: Transform = duck2world.combine(base2world.invert)


kine = URKinematics("ur3e")

multi = MultiURKinematics(kine)

import ipdb

ipdb.set_trace()

angles = multi.inverse_optimal([t.kine_pose for t in traj_transf])

import ipdb

ipdb.set_trace()