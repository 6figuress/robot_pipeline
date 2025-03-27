import numpy as np
from ur_ikfast.ur_kinematics import URKinematics

from calibrate import loadCalibration
from camera_sync import getArucosFromPaper, Transform


arucos = getArucosFromPaper(4)

base2world, grip2cam = loadCalibration(
    "./calibrations/calibration_logitec_with_stand.npz"
)

worldOrigin = arucos[47].corners[0].coords

duck2world: Transform = Transform.fromRodrigues(
    rvec=[0.0, 0.0, 0.0], tvec=[115.0, -294 / 2, 111.0]
)

duck2robot = duck2world.combine(base2world.invert)

worldOrigin_robotFrame = base2world.invert.apply([0, 0, 0])

kine = URKinematics("ur3e")

pose: list[float] = [
    worldOrigin_robotFrame[0] * 0.001,
    worldOrigin_robotFrame[1] * 0.001,
    worldOrigin_robotFrame[2] * 0.001 + 0.2,
    0,
    0.999,
    0.0,
    0.0,
]

import ipdb

ipdb.set_trace()

angles = kine.inverse(pose, all_solutions=True)


print(angles)