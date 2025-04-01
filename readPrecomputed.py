import numpy as np
from camera_sync import Transform, vizPoses
from pipeline import generateTrajectoryFromPoses


def generateForNpz(filename):
    file = np.load(f"./pre_generated_traj/{filename}.npz")
    data = file["poses"]

    transf = []

    order = [2, 1, 0, 3, 4, 5, 6]

    for i in order:
        t = Transform(transf_mat=data[i])
        transf.append(Transform.fromRodrigues(rvec=t.rvec, tvec=t.tvec * 1000))

    generateTrajectoryFromPoses(
        [t.kine_pose for t in transf], graph=False, filename=f"trajectory_{filename}"
    )


if __name__ == "__main__":
    toGen = ["duck_front", "duck_head", "duck_right_wing"]

    # toGen = ["duck_front"]

    # for g in toGen:
    #     try:
    #         generateForNpz(g)
    #     except Exception as e:
    #         if str(e) != "No solutions found":
    #             raise e
    #         else:
    #             print(f"Skipping {g} because no solution was found")

    # x = [i for i in range(-400, 400, 10)]

    # y = [i for i in range(-400, 400, 10)]

    # z = [i for i in range(-50, 50, 10)]

    # rvecs_values = [i for i in range(-320, 320, 10)]

    # tvecs = []

    # rvecs = []

    # for xc in x:
    #     for yc in y:
    #         tvecs.append([xc, yc, 0])

    # for r in rvecs_values:
    #     rvecs.append([0.0, 0.0, r / 100])

    # validPairs = []

    # i = 0

    # total = len(rvecs) * len(tvecs)

    # print("Testing ", total, " combinations")

    # for r in rvecs:
    #     for t in tvecs:
    #         i += 1
    #         if i % 100 == 0:
    #             print(f"Progress : {i}/{total}")
    #         foundAll = True
    #         for g in toGen:
    #             try:
    #                 generateForNpz(g, tvec=t, rvec=r)
    #                 # print("Solutions found with tvec : ", t)
    #             except Exception as e:
    #                 if str(e) != "No solutions found":
    #                     if str(e) == "No secure solutions found":
    #                         foundAll = False

    #                         break
    #                     else:
    #                         raise e
    #                 else:
    #                     foundAll = False
    #                     break
    #                     pass
    #                     # print(f"Skipping {g} because no solution was found with tvec ", t)
    #         if foundAll:
    #             print("========== ALL VALID ============")
    #             print("Tvec : ", t)
    #             print("Rvec : ", r)
    #             print("=================================")
    #             validPairs.append((t, r))

    # print("Valid tvecs founds : ", validPairs)

    # import ipdb

    # ipdb.set_trace()
