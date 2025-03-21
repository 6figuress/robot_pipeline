import numpy as np
from ur_ikfast.ur_kinematics import URKinematics

from calibrate import loadCalibration
from camera_sync import getArucosFromPaper


arucos = getArucosFromPaper(2)

base2world, grip2cam = loadCalibration("./calibrations/calibration_logitec_new.npz")

worldOrigin = arucos[38].corners[0].coords

worldOrigin_robotFrame = base2world.invert.apply(worldOrigin)

kine = URKinematics("ur3e")

pose: list[float] = [worldOrigin_robotFrame[0] * 0.001, worldOrigin_robotFrame[1] * 0.001, worldOrigin_robotFrame[2] * 0.001 + 0.3,  0,  0.999,  0.0, 0.0]

angles = kine.inverse(pose, all_solutions=True)