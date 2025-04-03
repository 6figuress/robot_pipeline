from pipeline import generateTrajectoryFromPoses

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

import argparse


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

    angle_rad = np.radians(
        rotation_angle + 270
    )  # the 270 are the default orientation of the duck
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
    pattern = re.compile(
        rf"{re.escape(prefix)}_(\d{{2}}\.\d{{2}}\.\d{{4}}_\d{{2}}h\d{{2}}m\d{{2}}s)\.json"
    )

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
    pattern = re.compile(rf".*{escaped_ext}$", re.IGNORECASE)

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
                raise FileNotFoundError(
                    f"Failed to find .obj file after conversion in {folder_path}"
                )
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


def plot_paths_process(mesh, res, restricted_face, display_orientation=True):
    from duck_factory.mesh_to_paths import plot_paths

    restricted_face = [4, 6]
    plot_paths(mesh, res, restricted_face=restricted_face, display_orientation=display_orientation)


def get_paths(
    mesh,
    nopaint_mask,
    name,
    restricted_face=None,
    n_samples=50_000,
    max_dist=0.004,
    thickness=0.002,
    bbox_scale=1.1,
    home_point=((0.04, -0.04, 0.11), (0, 0, -1)),
    verbose=True,
):
    # dither = Dither(factor=1.0, algorithm="fs", nc=2)
    dither = Dither(factor=1.0, algorithm="SimplePalette", nc=2)
    path_analyzer = PathAnalyzer(
        tube_length=5e1, diameter=2e-2, cone_height=1e-2, step_angle=36, num_vectors=12
    )

    res = mesh_to_paths(
        mesh=mesh,
        n_samples=n_samples,
        nopaint_mask=nopaint_mask,
        max_dist=max_dist,
        home_point=home_point,
        verbose=verbose,
        ditherer=dither,
        path_analyzer=path_analyzer,
        bbox_scale=bbox_scale,
        nz_threshold=-1.0,
        thickness=thickness,
        restricted_face=restricted_face,
        point_offset=0.0022,
    )

    with open(f"{name}", "w") as f:
        json.dump(res, f, indent=4, cls=NumpyEncoder)
    print(f"Paths saved to {name}")


def main(
    folder_name,
    output_dir="./trajectories",
    n_samples=100_000,
    display=False,
    display_orientation=False,
    max_dist=0.004,
    thickness=0.002,
    bbox_scale=1.1,
    home_point=((0.04, -0.04, 0.11), (0, 0, -1)),
):
    """
    Main function to run the pipeline"
    Parameters:
        - folder_name: name of the folder under ./painting_models/
        - n_samples: number of samples for point cloud sampling
        - display: display paths
        - max_dist: max distance for path generation
        - thickness: thickness for path generation
        - bbox_scale: bounding box scale for path generation
        - home_point: home point for path generation
    """
    start_pipeline = time.time()
    timestamp = datetime.now().strftime("%d.%m.%Y_%Hh%Mm%Ss")

    folder_path = f"./painting_models/{folder_name}"

    painting_faces = ["top", "left", "right", "front", "back"]
    restricted_face = [3, 8]  # Bottom

    mesh, nopaint_mask = get_mesh(folder_path)
    modify_mesh_position(mesh, rotation_angle=0)

    paths_file_path = f"{folder_path}/paths_{timestamp}.json"
    get_paths(
        mesh,
        nopaint_mask,
        paths_file_path,
        restricted_face=restricted_face,
        n_samples=n_samples,
        max_dist=0.004,
        thickness=0.002,
        bbox_scale=1,
        home_point=((0.04, -0.04, 0.11), (0, 0, -1)),
        verbose=True,
    )

    end_path = time.time()

    latest_path = load_latest_timestamped_file(folder_path, "paths")
    print(f"Loading {latest_path} path file")
    with open(latest_path, "r") as f:
        res = json.load(f)

    if display:
        p = Process(target=plot_paths_process, args=(mesh, res, restricted_face, display_orientation))
        p.start()

    trajectory_folder = f"{output_dir}/trajectory_{folder_name}_{timestamp}"

    if not os.path.exists(trajectory_folder):
        os.makedirs(trajectory_folder)

    for r in res:
        face = r[0]  # name of the face
        color = r[1]  # color of the face in rgba
        paths = r[2]  # path of the face

        trajectory_face_folder = os.path.join(trajectory_folder, face)
        if not os.path.exists(trajectory_face_folder):
            os.makedirs(trajectory_face_folder)

        trajectory_filename = os.path.join(trajectory_face_folder, f"{color}.json")

        if face in painting_faces:
            poses = [[*path[0], *path[1]] for path in paths]
            if len(poses) > 0:
                print(f"Adding {len(poses)} poses for face {face} with color {color}")
                start_IK = time.time()
                generateTrajectoryFromPoses(
                    poses, filename=trajectory_filename, graph=False, verbose=True
                )
                end_IK = time.time()
                print(
                    f"IK for face {face}, color {color} and {len(poses)} poses took {end_IK - start_IK} seconds"
                )
            else:
                print(
                    f"Skipping face {face} with color {color} because it has no poses"
                )
        else:
            print(
                f"Skipping face {face} with color {color} and {len(paths)} paths because it is not in the painting faces list"
            )

    print(f"Pipeline took {time.time() - start_pipeline} seconds")

def display_last(folder_name, display_orientation=False):

    folder_path = f"./painting_models/{folder_name}"

    restricted_face = [3, 8]  # Bottom

    mesh, nopaint_mask = get_mesh(folder_path)
    modify_mesh_position(mesh, rotation_angle=0)

    latest_path = load_latest_timestamped_file(folder_path, "paths")
    print(f"Loading {latest_path} path file")
    with open(latest_path, "r") as f:
        res = json.load(f)

    p = Process(target=plot_paths_process, args=(mesh, res, restricted_face, display_orientation))
    p.start()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mesh to trajectory pipeline")
    parser.add_argument(
        "--folder", type=str, required=True, help="Folder name under ./painting_models/"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="./trajectories",
        help="Output directory for trajectories",
    )
    parser.add_argument(
        "--n_samples",
        type=int,
        default=50000,
        help="Number of samples for point cloud sampling",
    )
    parser.add_argument("--display", action="store_true", help="Display paths")
    parser.add_argument(
        "--max_dist", type=float, default=0.004, help="Max distance for path generation"
    )
    parser.add_argument(
        "--thickness", type=float, default=0.002, help="Thickness for path generation"
    )
    parser.add_argument(
        "--bbox_scale",
        type=float,
        default=1.1,
        help="Bounding box scale for path generation",
    )
    parser.add_argument(
        "--home_point",
        type=tuple,
        default=((0.04, -0.04, 0.11), (0, 0, -1)),
        help="Home point for path generation",
    )
    parser.add_argument("--just_display", action="store_true", help="Display last paths")
    parser.add_argument("--display_orientation", action="store_true", help="Display orientation")
    args = parser.parse_args()

    if args.just_display:
        display_last(args.folder, display_orientation=args.display_orientation)
        exit(0)

    main(
        folder_name=args.folder,
        output_dir=args.output_dir,
        n_samples=args.n_samples,
        display=args.display,
        max_dist=args.max_dist,
        thickness=args.thickness,
        bbox_scale=args.bbox_scale,
        home_point=args.home_point,
    )