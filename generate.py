import json
from cv2.typing import Vec6f
import numpy as np
from calibrate import readCSV

def generate_trajectory(data: np.ndarray[Vec6f], filename="trajectory.json"):
    modTraj = []
    time_step = 2  # Incrément du temps
    time = 4
    
    for arr in data:
        positions = [round(float(x), 4) if abs(x) >= 1e-4 else 0.0 for x in arr]
        velocities = [0.0] * 6  # Vélocités à zéro
        modTraj.append({
            "positions": positions,
            "velocities": velocities,
            "time_from_start": [time, 0]
        })
        time += time_step
    
    with open(filename, "w") as f:
        json.dump({"modTraj": modTraj}, f, indent=4)
    
    print(f"Fichier JSON '{filename}' généré avec succès.")


def normalize_angle(angle_rad):
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


poses = readCSV()

angles = np.array([p[0] for p in poses])

sorted_arr = angles[np.lexsort(angles[:, ::-1].T)]

sorted_arr[:, 5] = normalize_angle(sorted_arr[:, 5])


import ipdb
ipdb.set_trace()

generate_trajectory(sorted_arr, "trajectory.json")