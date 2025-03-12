from ur_ikfast import ur_kinematics
from camera_sync.referential import Transform
from camera_sync.plotting import vizPoses

pos1 = [0.35230425000190735,-0.7322418850711365,0.5806735197650355,-1.0719731015018006,4.915554523468018,0.31385645270347595]
pos2 = [2.711972236633301,-1.102702186708786,0.9782894293414515,-1.2463486355594178,4.384324073791504,-0.8515551725970667]

def fk(angles)-> Transform:
    arm = ur_kinematics.URKinematics("ur3e")
    
    # Copy the 3x4 transformation values into the 4x4 matrix
    mat = arm.forward(angles, rotation_type="matrix")
    return Transform(rot_mat=mat[:3, :3], tvec=mat[:3, 3])

kine = ur_kinematics.URKinematics("ur3e")

t1 = fk(pos1)

t2 = fk(pos2)

vizPoses([t1, t2], limits=(-1, 1), length=0.2)