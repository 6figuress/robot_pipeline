from duck_factory.mesh_to_paths import mesh_to_paths, load_mesh

from duck_factory.dither_class import Dither
from duck_factory.reachable_points import PathAnalyzer
import json
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics, generate_trajectory

from calibrate import loadCalibration
from camera_sync import Transform, vizPoses
import trimesh

def modify_mesh_position(mesh):
    mesh.vertices = np.column_stack(
        (
            -mesh.vertices[:, 0],  # -X
            mesh.vertices[:, 2],  # Z
            mesh.vertices[:, 1],  # Y
        )
    )

    # Transform all vertex normals, if they exist
    if mesh.vertex_normals is not None:
        mesh.vertex_normals = np.column_stack(
            (
                -mesh.vertex_normals[:, 0],
                mesh.vertex_normals[:, 2],
                mesh.vertex_normals[:, 1],
            )
        )

    # mesh.apply_translation([0, 0, 0.05])

    angle_rad = np.radians(180)
    rotation_matrix = trimesh.transformations.rotation_matrix(
        angle_rad,
        [0, 0, 1],
        mesh.centroid,
    )

    # Apply the transformation
    mesh.apply_transform(rotation_matrix)


# mesh = load_mesh("./painting_models/duck/DuckComplete.obj")
mesh = load_mesh("./painting_models/cube/cube_8mm.obj")

modify_mesh_position(mesh)
 
dither = Dither(factor=1.0, algorithm="SimplePalette", nc=2)
path_analyzer = PathAnalyzer(
    tube_length=5e1, diameter=2e-2, cone_height=1e-2, step_angle=36, num_vectors=12
)

res = mesh_to_paths(mesh=mesh, n_samples=50_000, max_dist=0.0024, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.5, nz_threshold=-1.0)

number_points = len(res[0][1])

traj_transf = []

# For now, taking only white trajectory
for r in res:
    if r[0] == (255, 255, 255, 255):
        for pose in r[1]:
            traj_transf.append(
                Transform.fromQuaternion(
                    quat=pose[1], tvec=np.array(pose[0]) * 1000, scalar_first=False
                )
            )

base2world, grip2cam = loadCalibration(
    "./calibrations/calibration_logitec_with_stand.npz"
)


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(
    rvec=[0.0, 0.0, 0.0], tvec=[112.57, -147.57, 116.0]
)

duck2robot = duck2world.combine(base2world.invert)


transformed = []

for i in range(len(traj_transf)):
    transformed.append(traj_transf[i].combine(duck2robot))

kine = URKinematics("ur3e_pen_final_2")

multi = MultiURKinematics(kine)

angles = multi.inverse_optimal([t.kine_pose for t in transformed])

# print(angles)

number_angles = len(angles.trajectory)

print(f"Number of points: {number_points}, Number of angles: {number_angles}, Ratio: {number_angles/number_points}")

generate_trajectory(angles, "duck_trajectory.json")