import numpy as np
from camera_sync import Transform, vizPoses
from pipeline import generateTrajectoryFromPoses

file = np.load("./pre_generated_traj/duck_right_wing.npz")

data = file["poses"]

transf = []

for d in data:
    transf.append(Transform(transf_mat=d))



generateTrajectoryFromPoses([t.kine_pose for t in transf], graph=False, filename="trajectory_duck_right_wing.json")