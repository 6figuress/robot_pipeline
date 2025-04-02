import numpy as np
import pandas as pd
from camera_sync import Transform, vizPoses

file = "vertices_eye"

# Read the CSV file
csv_path = f"./pre_generated_vertices/{file}.csv"
df = pd.read_csv(csv_path)

# Extract vertex indices
indices = df['Index'].values

# Initialize a dictionary to store matrices by vertex index
transforms = []

# Process each row into a 4x4 transformation matrix
for _, row in df.iterrows():
    idx = int(row['Index'])
    matrix = np.array([
        [row['M00'], row['M01'], row['M02'], row['M03'] * 1000],
        [row['M10'], row['M11'], row['M12'], row['M13'] * 1000],
        [row['M20'], row['M21'], row['M22'], row['M23'] * 1000],
        [row['M30'], row['M31'], row['M32'], row['M33']]
    ])
    t = Transform(transf_mat=matrix)
    transforms.append(t)

poses = np.array([t.transf_mat for t in transforms])

np.savez(f"pre_generated_traj/{file}.npz", poses=poses)
