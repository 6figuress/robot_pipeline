from dataclasses import dataclass
from functools import reduce
from math import cos, pi, sin
from pipeline import generateTrajectoryFromPoses
from camera_sync import vizPoses


from camera_sync import Transform

from ur_ikfast.ur_kinematics import URKinematics, MultiURKinematics


@dataclass
class Sample:
    """
    The following class represents a sample point in the mesh.

    It is characterized by its position [mm] in a 3-dimensional referential and its angle (quaternion).
    """

    position: list[float]
    angle: list[float]

    @property
    def tvec(self) -> list[float]:
        """Returns the position vector for Transform class"""
        return self.position

    @property
    def quat(self) -> list[float]:
        """Returns the quaternion for Transform class"""
        return self.angle

    def to_transform(self) -> Transform:
        """Convert the Sample to a Transform object"""
        return Transform.fromQuaternion(quat=self.quat, tvec=self.tvec)


class MeshPathStub:
    """The following class serves as a stub for the mesh to path implementation"""

    def __init__(self, cube_size: float = 80.0) -> None:
        """
        Constructor for the MeshPathStub class.

        Parameters
        ----------
                cube_size (float): The size of the cube in millimeters

        Returns
        -------
                None

        """
        self.cube_size: float = cube_size

    def line_on_cube_reach(self, n_points: int = 50) -> list[Transform]:
        # constants used for the line generation
        cube_size: float = self.cube_size
        cube_half_size: float = cube_size / 2.0

        upwards_pen_quaternion: list[float] = [0.0, 1.0, 0.0, 0.0]
        against_face_pen_quaternion: list[float] = [0.0, 1.0, 0.0, -1.0]
        against_other_face_pen_quaternion: list[float] = [0.0, 1.0, 0.0, 1.0]

        quats = [
            against_other_face_pen_quaternion,
            upwards_pen_quaternion,
            against_face_pen_quaternion,
        ]

        points_per_part = n_points // 4

        mapping = {0: 0, 1: 1, 2: 1, 3: 2}

        points_for_parts = [
            [-cube_half_size, -cube_half_size, 0],
            [-cube_half_size, 0, cube_size],
            [0, 0, cube_size],
            [cube_half_size, 0, cube_size],
            [cube_half_size, cube_half_size, 0],
        ]

        pos = []

        for i in range(0, len(points_for_parts) - 1):
            startPoint = points_for_parts[i]
            destPoint = points_for_parts[i + 1]
            xstep = (destPoint[0] - startPoint[0]) / points_per_part
            ystep = (destPoint[1] - startPoint[1]) / points_per_part
            zstep = (destPoint[2] - startPoint[2]) / points_per_part

            for j in range(points_per_part + 1):
                pos.append(
                    Sample(
                        position=[
                            startPoint[0] + xstep * j,
                            startPoint[1] + ystep * j,
                            startPoint[2] + zstep * j,
                        ],
                        angle=quats[mapping[i]],
                    ).to_transform()
                )
        return pos

    def line_on_cube(self, n_points: int = 50) -> list[Transform]:
        """
        "Draws" a straight line on two adjacent faces of a cube.

        The cube is assumed to be centered at its origin (center of downwards face).

        Parameters
        ----------
                n_points (int): The number of points to generate on the line

        Returns
        -------
                list[Transform]: The list of samples

        """
        # constants used for the line generation
        cube_size: float = self.cube_size
        cube_half_size: float = cube_size / 2.0

        upwards_pen_quaternion: list[float] = [0.0, 1.0, 0.0, 0.0]
        against_face_pen_quaternion: list[float] = [0.0, 1.0, 0.0, -1.0]

        half_line_n_points: int = n_points // 2

        origin_position: list[float] = [0.0, 0.0, cube_size]
        halfway_position: list[float] = [cube_half_size, 0.0, cube_size]
        final_position: list[float] = [cube_half_size, 0.0, cube_half_size]

        # start at the center of the upwards face (origin)
        # and interpolate our way to the halfway position
        samples: list[Transform] = reduce(
            lambda res, i: res
            + [
                Sample(
                    position=[
                        origin_position[0]
                        + (halfway_position[0] - origin_position[0])
                        * i
                        / (half_line_n_points - 1),
                        0.0,
                        origin_position[2],
                    ],
                    angle=upwards_pen_quaternion,
                ).to_transform()
            ],
            range(half_line_n_points),
            [],
        )

        # start at the halfway position and
        # interpolate our way to the final position
        samples += reduce(
            lambda res, i: res
            + [
                Sample(
                    position=[
                        cube_half_size,
                        0.0,
                        halfway_position[2]
                        + (final_position[2] - halfway_position[2])
                        * i
                        / (half_line_n_points - 1),
                    ],
                    angle=against_face_pen_quaternion,
                ).to_transform()
            ],
            range(half_line_n_points),
            [],
        )

        return samples

    def circle_on_cube(self, n_points: int = 50) -> list[Transform]:
        """
        "Draws" a circle on the upwards face of a cube.

        The cube of is assumed to be centered at its origin (center of downwards face).

        Parameters
        ----------
                n_points (int): The number of points to generate on the circle

        Returns
        -------
                list[Transform]: The list of samples

        """
        # constants used for the circle generation
        # the circle is centered at the center of
        # the upwards face
        cube_size: float = self.cube_size
        circle_radius: float = cube_size * 0.4
        whole_revolution: float = 2.0 * pi

        upwards_pen_quaternion: list[float] = [0.0, 1.0, 0.0, 0.0]

        # compute the position of the points on the circle
        return reduce(
            lambda res, angle: res
            + [
                Sample(
                    position=[
                        cos(angle) * circle_radius,
                        sin(angle) * circle_radius,
                        cube_size,
                    ],
                    angle=upwards_pen_quaternion,
                ).to_transform()
            ],
            map(lambda i: i * whole_revolution / n_points, range(n_points)),
            [],
        )


def rangeExploration():
    kine = URKinematics("ur3e_pen_final_2")

    multi = MultiURKinematics(kine)

    path = MeshPathStub().line_on_cube_reach(12)

    xRange = range(-400, 400, 80)
    yRange = range(-50, 500, 100)
    zRange = range(-80, 200, 40)

    rotationRange = range(0, 314, 40)

    rvecs = []
    tvecs = []

    for x in xRange:
        for y in yRange:
            for z in zRange:
                tvecs.append([x, y, z])

    for r in rotationRange:
        rvecs.append([0, 0, r / 100])

    total = len(rvecs) * len(tvecs)

    print("Total combinations : ", total)

    valids = []

    i = 0

    for tvec in tvecs:
        for rvec in rvecs:
            i += 1
            if i % 100 == 0:
                print("Progress : ", i, " / ", total)
            t = Transform.fromRodrigues(rvec=rvec, tvec=tvec)
            transformed = [p.combine(t).kine_pose for p in path]
            try:
                multi.inverse_optimal(transformed)
            except Exception as e:
                if str(e) == "No solutions found":
                    continue
                elif str(e) == "No secure solutions found":
                    continue
                else:
                    raise ev
            valids.append((rvec, tvec))

    print("Found ", len(valids), " positions")

    import ipdb

    ipdb.set_trace()

    pass


if __name__ == "__main__":
    rangeExploration()
    # generator = MeshPathStub()

    # samples_circle = generator.circle_on_cube(50)

    # samples_right_line = generator.line_on_cube(50)

    # samples_reach = generator.line_on_cube_reach(12)

    # vizPoses(samples_reach)

    # toGenerate = [("Circle", samples_circle), ("Right face", samples_right_line)]

    # for name, samples in toGenerate:
    #     try:
    #         generateTrajectoryFromPoses([s.kine_pose for s in samples], filename=name)
    #     except Exception as e:
    #         if str(e) == "No solutions found":
    #             print(f"Skipped index {name}, no solution found")
    #         else:
    #             raise e
    pass