from camera_sync import Transform, vizPoses

base_pose = Transform.fromQuaternion(quat=[1, 0, 0, 1], tvec=[5, 5, 0])


ref_change = Transform.fromQuaternion(quat=[1, 0, 0, 0], tvec=[10, 10, 0])


combined = base_pose.combine(ref_change)
combined2 = ref_change.combine(base_pose)
vizPoses([combined2], limits=(-10, 10), length=2)

import ipdb

ipdb.set_trace()
