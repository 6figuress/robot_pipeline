import numpy as np
from ur_ikfast.ur_kinematics import URKinematics

from calibrate import loadCalibration
from camera_sync import Transform, vizPoses

import matplotlib.pyplot as plt

base2world, grip2cam = loadCalibration("./calibrations/calibration_logitec.npz")


# Those value are written in the technical plan of the duck support
duck2world: Transform = Transform.fromRodrigues(
    rvec=[0.0, 0.0, 0.0], tvec=[117.57, -152.57, 111.0]
)

duck2robot: Transform = duck2world.combine(base2world.invert)

duck_origin = [0.0, 150.0, 0.0]

duck_origin_robot_frame = duck2robot.apply(duck_origin)

kine = URKinematics("ur3e")


position = [
    duck_origin_robot_frame[0],
    duck_origin_robot_frame[1],
    duck_origin_robot_frame[2],
    1,
    0,
    0,
    0,
]


pose = kine.inverse(position)


vizPoses([duck2world])
