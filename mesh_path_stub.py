from dataclasses import dataclass
from functools import reduce
from math import cos, pi, sin

from camera_sync import Transform


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
