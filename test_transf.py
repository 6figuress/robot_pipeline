from camera_sync import Transform, vizPoses

tBase = Transform.fromQuaternion(quat=[1.0, .0, .0, .0], tvec=[100, 0, 0])

rot = Transform.fromQuaternion(quat=[1.0, .0,.0, 1.0], tvec=[.0, .0, .0])


vizPoses([tBase.combine(rot)])