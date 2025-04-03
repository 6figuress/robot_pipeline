from pipeline import generateTrajectoryFromPoses, generateTrajectoryFromMultiplePoses

from duck_factory.mesh_to_paths import mesh_to_paths, load_mesh

from duck_factory.dither_class import Dither
from duck_factory.reachable_points import PathAnalyzer
import json
import numpy as np

import trimesh
from datetime import datetime
import time
import os
import re

import subprocess
from PIL import Image

from multiprocessing import Process

def modify_mesh_position(mesh, rotation_angle=0):
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

    angle_rad = np.radians(rotation_angle+270) # the 270 are the default orientation of the duck
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

def find_ext_files(directory, extension=".obj"):
    ext_files = []
    escaped_ext = re.escape(extension)
    pattern = re.compile(rf'.*{escaped_ext}$', re.IGNORECASE)

    for root, _, files in os.walk(directory):
        for file in files:
            if pattern.match(file):
                full_path = os.path.join(root, file)
                ext_files.append(full_path)

    if not ext_files:
        print(f"No files with extension {extension} found in {directory}")
        return None
    if len(ext_files) > 1:
        print(f"Multiple files with extension {extension} found in {directory}:")
        for file in ext_files:
            print(f"- {file}")
        print("Returning the first one.")
    return ext_files[0]

def convert_glb_to_obj(glb_file_path):
    result = subprocess.run(["./gltf_obj.sh", glb_file_path])

    if result.returncode == 0:
        new_file_path = glb_file_path.replace(".glb", ".obj")
        if os.path.exists(new_file_path):
            print(f"Converted {glb_file_path} to {new_file_path}")
        else:
            raise FileNotFoundError(f"Converted file {new_file_path} does not exist.")
    else:
        print("Error converting GLB to OBJ:", result.stderr)
        print("Return code:", result.returncode)
        raise RuntimeError("Failed to convert GLTF file to OBJ")


def get_mesh(folder_path):
    mesh_file_path = find_ext_files(folder_path, extension=".obj")
    if not mesh_file_path:
        print(f"No .obj file found in {folder_path}, checking for .glb files")
        glb_file = find_ext_files(folder_path, extension=".glb")
        if glb_file:
            print(f"Converting {glb_file} to .obj")
            convert_glb_to_obj(glb_file)
            mesh_file_path = find_ext_files(folder_path, extension=".obj")
            if not mesh_file_path:
                raise FileNotFoundError(f"Failed to find .obj file after conversion in {folder_path}")
        else:
            raise FileNotFoundError(f"No model file found in {folder_path}")
        
    # Check if the file "nopaint_mask.png" exists in the same directory
    nopaint_mask_path = os.path.join(folder_path, "nopaint_mask.png")
    nopaint_mask = None
    if os.path.exists(nopaint_mask_path):
        print(f"Loading nopaint mask from {nopaint_mask_path}")
        nopaint_mask = Image.open(nopaint_mask_path).convert("L")
        
    mesh = load_mesh(mesh_file_path)
    return mesh, nopaint_mask
    
def plot_paths_process(mesh, res, restricted_face):
    from duck_factory.mesh_to_paths import plot_paths
    restricted_face = [4, 6] 
    plot_paths(mesh, res, restricted_face=restricted_face)

def get_paths(mesh, nopaint_mask, name, restricted_face=None):
    # dither = Dither(factor=1.0, algorithm="fs", nc=2)
    dither = Dither(factor=1.0, algorithm="SimplePalette", nc=2)
    path_analyzer = PathAnalyzer(
        tube_length=5e1, diameter=2e-2, cone_height=1e-2, step_angle=36, num_vectors=12
    )

    res = mesh_to_paths(mesh=mesh, n_samples=50_000, nopaint_mask=nopaint_mask, max_dist=0.004, home_point=((0.04, -0.04, 0.11), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1, nz_threshold=-1.0, thickness=0.002, restricted_face=restricted_face)
    # res = mesh_to_paths(mesh=mesh, n_samples=50_000, max_dist=0.0012, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1, nz_threshold=-1.0, thickness=0.0)


    # res = mesh_to_paths(mesh=mesh, n_samples=200_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.0)
    # res = mesh_to_paths(mesh=mesh, n_samples=25_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.004)
    # res = mesh_to_paths(mesh=mesh, n_samples=25_000, max_dist=0.008, home_point=((0, 0, 0.1), (0, 0, -1)), verbose=True, ditherer=dither, path_analyzer=path_analyzer, bbox_scale=1.1, nz_threshold=-1.0, thickness=0.006)

    with open(f"{name}", "w") as f:
        json.dump(res, f, indent=4, cls=NumpyEncoder)
    print(f"Paths saved to {name}")

#  ---------------- MAIN ----------------
start_pipeline = time.time()
# folder_name = "cube_edge_top"
# folder_name = "cube_isc_top_filled"
# folder_name = "duck_crown"
# folder_name = "duck_eyes"
# folder_name = "duck_eyes_colored"
# folder_name = "duck_line"
# folder_name = "duck_one_eye"
# folder_name = "duck_eyes_crown"
# folder_name = "cube_isc_side_filled"
# folder_name = "duck_spiderman_glb"
folder_name = "duck_isc_filled_right_side"
# folder_name = "cube_isc_side_filled"

# painting_faces = ["top", "bottom", "left", "right","front", "back"]
painting_faces = ["top", "left", "right","front", "back"]
restricted_face = [3, 8] #bottom


timestamp = datetime.now().strftime("%d.%m.%Y_%Hh%Mm%Ss")

folder_path = f"./painting_models/{folder_name}"

mesh, nopaint_mask = get_mesh(folder_path)

modify_mesh_position(mesh, rotation_angle=0)

paths_file_path = f"{folder_path}/paths_{timestamp}.json"

get_paths(mesh, nopaint_mask, paths_file_path, restricted_face=restricted_face)

end_path = time.time()

latest_path = load_latest_timestamped_file(folder_path, "paths")
print(f"Loading {latest_path} path file")
with open(latest_path, "r") as f:
    res = json.load(f)



p = Process(target=plot_paths_process, args=(mesh, res, restricted_face))
p.start()


list_poses = []
number_of_poses = 0
trajectory_folder = f"./trajectories/trajectory_{folder_name}_{timestamp}"
if not os.path.exists(trajectory_folder):
    os.makedirs(trajectory_folder)

for r in res:
    face = r[0] # name of the face
    color = r[1] # color of the face in rgba
    paths = r[2] # path of the face

    trajectory_filename = os.path.join(trajectory_folder, f"trajectory_{face}_{color}.json")

    if face in painting_faces:
        poses = [[*path[0], *path[1]] for path in paths]
        number_of_poses += len(poses)
        if len(poses) > 0:
            print(f"Adding {len(poses)} poses for face {face} with color {color}")
            list_poses.append(poses)

            start_IK = time.time()
            generateTrajectoryFromPoses(
                poses,
                filename=trajectory_filename,
                graph=False,
                verbose=True
            )
            end_IK = time.time()
            print(f"IK for face {face}, color {color} and {len(poses)} poses took {end_IK - start_IK} seconds")
        else:
            print(f"Skipping face {face} with color {color} because it has no poses")
    else:
        print(f"Skipping face {face} with color {color} and {len(paths)} paths because it is not in the painting faces list")

print(f"Pipeline took {time.time() - start_pipeline} seconds")
