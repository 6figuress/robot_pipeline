from duck_factory.mesh_to_paths import mesh_to_paths, load_mesh, plot_paths

from duck_factory.dither_class import Dither
from duck_factory.reachable_points import PathAnalyzer
import json
import numpy as np
from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics, generate_trajectory

from calibrate import loadCalibration
from camera_sync import Transform, vizPoses
import trimesh
from datetime import datetime
import time
import os
import re

from multiprocessing import Process


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

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)
    
def get_paths(mesh, name):
    dither = Dither(factor=1.0, algorithm="SimplePalette", nc=2)
    path_analyzer = PathAnalyzer(
        tube_length=5e1, diameter=2e-2, cone_height=1e-2, step_angle=36, num_vectors=12
    )

    # res = mesh_to_paths(mesh=mesh, n_samples=50_000, max_dist=0.0012, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1, nz_threshold=-1.0, thickness=0.0)
    # res = mesh_to_paths(mesh=mesh, n_samples=200_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.0)
    # res = mesh_to_paths(mesh=mesh, n_samples=25_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.004)
    res = mesh_to_paths(mesh=mesh, n_samples=25_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.006)

    with open(f"{name}", "w") as f:
        json.dump(res, f, indent=4, cls=NumpyEncoder)
    print(f"Paths saved to {name}")

def load_latest_timestamped_file(folder_path, prefix):
    """
    Finds the latest file in `folder_path` that starts with `prefix`
    and ends with `.json`, using Swiss timestamp format in the filename.
    """
    pattern = re.compile(rf"{re.escape(prefix)}_(\d{{2}}\.\d{{2}}\.\d{{4}}_\d{{2}}h\d{{2}}m\d{{2}}s)\.json")

    candidates = []
    for filename in os.listdir(folder_path):
        match = pattern.match(filename)
        if match:
            timestamp_str = match.group(1)
            try:
                timestamp = datetime.strptime(timestamp_str, "%d.%m.%Y_%Hh%Mm%Ss")
                candidates.append((timestamp, filename))
            except ValueError:
                continue

    if not candidates:
        return None

    latest = max(candidates)[1]
    return os.path.join(folder_path, latest)

def plot_paths_process(mesh, res):
    from duck_factory.mesh_to_paths import plot_paths
    plot_paths(mesh, res)
# ------------------ MAIN ------------------

# TODO : select the folder to load
# folder_name = "cube"
folder_name = "cube_edge_top"
# folder_name = "cube_isc_top"
# folder_name = "cube_isc_top_filled"
# folder_name = "cube_isc_side"
# folder_name = "duck_isc_filled"
# folder_name = "duck_eyes"
# folder_name = "duck_crown"
# folder_name = "duck_eyes_crown"

timestamp = datetime.now().strftime("%d.%m.%Y_%Hh%Mm%Ss")

folder_path = f"./painting_models/{folder_name}"

#TODO : select the mesh file to load
mesh_file_path = f"{folder_path}/cube_8mm.obj"
# mesh_file_path = f"{folder_path}/duck_isc.obj"

mesh = load_mesh(mesh_file_path)

modify_mesh_position(mesh)
 

paths_file_path = f"{folder_path}/paths_{timestamp}.json"

get_paths(mesh, paths_file_path)

latest_path = load_latest_timestamped_file(folder_path, "paths")
print(f"Loading {latest_path} path file")
with open(latest_path, "r") as f:
    res = json.load(f)

p = Process(target=plot_paths_process, args=(mesh, res))
p.start()

number_points = len(res[0][1])

traj_transf = []

# For now, taking only white trajectory
for r in res:
    if r[0] == [255, 255, 255, 255]: # used when json is loaded
    # if r[0] == (255, 255, 255, 255):
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

start_IK = time.time()
angles = multi.inverse_optimal([t.kine_pose for t in transformed])
print(f"IK took {time.time() - start_IK} seconds")

number_angles = len(angles.trajectory)

angles_file_path = f"{folder_path}/angles_{timestamp}.json"

if number_angles == 0:
    print("No angles found, quitting")
    exit(1)
else:
    with open(angles_file_path, "w") as f:
        json.dump(angles.trajectory, f, indent=4, cls=NumpyEncoder)

latest_angles = load_latest_timestamped_file(folder_path, "angles")
print(f"Loading {latest_angles} angles file")
with open(angles_file_path, "r") as f:
    angles = json.load(f)

ratio = number_angles / number_points
print(f"Number of points: {number_points}, Number of angles: {number_angles}, Ratio: {ratio}")

trajectory_file_path = f"{folder_path}/trajectory_{timestamp}.json"

if ratio >= 0.8 :
    generate_trajectory(angles, trajectory_file_path)
    print(f"Trajectory saved to {trajectory_file_path}")
else:
    print("Ratio is too low, not generating trajectory")

print("Waiting for plot process to finish")
p.join()