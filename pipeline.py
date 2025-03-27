from camera_sync import Transform, vizPoses

from calibrate import loadCalibration
from mesh_path_stub import MeshPathStub

# mesh = load_mesh("./painting_models/cube/cube_8mm.obj")

# res = mesh_to_paths(mesh, n_samples=50000)

# traj = []

# traj_transf = []

# # For now, taking only white trajectory
# for r in res:
#     if r[0] == (255, 255, 255, 255):
#         for pose in r[1]:
#             traj_transf.append(
#                 Transform.fromQuaternion(
#                     quat=pose[1], tvec=np.array(pose[0]) * 1000, scalar_first=False
#                 )
#             )


# traj_transf: list[Transform] = [
#     Transform.fromQuaternion(
#         quat=[0.0, 1.0, 0.0, 0.0], tvec=[-20.0, 0.0, 200.0]
#     ),
#     Transform.fromQuaternion(
#         quat=[0.0, 1.0, 0.0, 0.0], tvec=[-10.0, 0.0, 200.0]
#     ),
#     Transform.fromQuaternion(quat=[0.0, 1.0, 0.0, 0.0], tvec=[0.0, 0.0, 200.0]),
#     Transform.fromQuaternion(
#         quat=[0.0, 1.0, 0.0, 0.0], tvec=[10.0, 0.0, 200.0]
#     ),
#     Transform.fromQuaternion(
#         quat=[0.0, 1.0, 0.0, 0.0], tvec=[20.0, 0.0, 200.0]
#     ),
# ]

stub: MeshPathStub = MeshPathStub()
trajectories: list[Transform] = stub.circle_on_cube()

vizPoses(trajectories, limits=[0, 100])


base2world, grip2cam = loadCalibration(
    "./calibrations/calibration_logitec_with_stand.npz"
)


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(
    rvec=[0.0, 0.0, 0.0], tvec=[117.57, -152.57, 50.0]
)

# # TODO: Remove this line !!!!
# duck2world = Transform.fromRodrigues(
#     rvec=[0.0, 0.0, 0.0], tvec=[117.57, -152.57, 250.0]
# )

duck2robot = duck2world.combine(base2world.invert)


transformed = []

for i in range(len(trajectories)):
    transformed.append(trajectories[i].combine(duck2robot))


vizPoses(transformed)

# kine = URKinematics("ur3e")

# multi = MultiURKinematics(kine)

# angles = multi.inverse_optimal([t.kine_pose for t in transformed])

# angles.trajectory = np.array(angles.trajectory)

# print(json.dumps(angles.trajectory.tolist()))
