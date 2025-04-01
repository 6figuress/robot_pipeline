from pipeline import generateTrajectoryFromPoses

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

def modify_mesh_position(mesh, rotation_angle=180):
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

    angle_rad = np.radians(rotation_angle)
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

def find_obj_files(directory):
    obj_files = []
    pattern = re.compile(r'.*\.obj$', re.IGNORECASE)

    for root, _, files in os.walk(directory):
        for file in files:
            if pattern.match(file):
                full_path = os.path.join(root, file)
                obj_files.append(full_path)

    return obj_files[0]

def plot_paths_process(mesh, res):
    from duck_factory.mesh_to_paths import plot_paths
    plot_paths(mesh, res)

def get_paths(mesh, name):
    # dither = Dither(factor=1.0, algorithm="fs", nc=2)
    dither = Dither(factor=1.0, algorithm="SimplePalette", nc=2)
    path_analyzer = PathAnalyzer(
        tube_length=5e1, diameter=2e-2, cone_height=1e-2, step_angle=36, num_vectors=12
    )

    # res = mesh_to_paths(mesh=mesh, n_samples=50_000, max_dist=0.0012, home_point=((0, 0, 0.11), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1, nz_threshold=-1.0, thickness=0.0006)
    # res = mesh_to_paths(mesh=mesh, n_samples=50_000, max_dist=0.0012, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1, nz_threshold=-1.0, thickness=0.0)


    # res = mesh_to_paths(mesh=mesh, n_samples=200_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.0)
    res = mesh_to_paths(mesh=mesh, n_samples=25_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.004)
    # res = mesh_to_paths(mesh=mesh, n_samples=25_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.006)

    with open(f"{name}", "w") as f:
        json.dump(res, f, indent=4, cls=NumpyEncoder)
    print(f"Paths saved to {name}")


#  ---------------- MAIN ----------------
start_pipeline = time.time()
folder_name = "cube_edge_top"
# folder_name = "cube_isc_top_filled"
# folder_name = "duck_crown"
# folder_name = "duck_eyes"
# folder_name = "duck_one_eye"
# folder_name = "duck_eyes_crown"
# folder_name = "cube_isc_side_filled"
timestamp = datetime.now().strftime("%d.%m.%Y_%Hh%Mm%Ss")

folder_path = f"./painting_models/{folder_name}"

mesh_file_path = find_obj_files(folder_path)

mesh = load_mesh(mesh_file_path)

modify_mesh_position(mesh, rotation_angle=0)
# modify_mesh_position(mesh, rotation_angle=240) # For duck_eyes 270

paths_file_path = f"{folder_path}/paths_{timestamp}.json"

get_paths(mesh, paths_file_path)

end_path = time.time()

latest_path = load_latest_timestamped_file(folder_path, "paths")
print(f"Loading {latest_path} path file")
with open(latest_path, "r") as f:
    res = json.load(f)

p = Process(target=plot_paths_process, args=(mesh, res))
p.start()

poses = [[*path[0], *path[1]] for path in res[0][1]] # res[0] for the first color data, res[0][0] to get the color, res[0][1] to get the path

trajectory_filename = f"trajectory_{folder_name}_{timestamp}"

print(f"Starting IK for {len(poses)} poses")
start_IK = time.time()
generateTrajectoryFromPoses(
    poses=poses,
    filename=trajectory_filename,
    graph=False,
    verbose=True
)
end_IK = time.time()
print(f"Path generation took {end_path - start_pipeline} seconds")
print(f"IK took {end_IK - start_IK} seconds")
print(f"Pipeline took {end_IK - start_pipeline} seconds")
