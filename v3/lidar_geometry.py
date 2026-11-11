"""LiDAR polar returns expressed at the robot's differential-drive origin."""

import math


def body_clearance_m(x_m: float, y_m: float,
                     bounds_m: tuple[float, float, float, float]) -> float:
    """Euclidean distance to the physical rectangle; points inside return zero.

    Bounds are (rear, front, right, left) in the drive-axle frame.
    """
    rear, front, right, left = bounds_m
    return math.hypot(max(rear - x_m, 0.0, x_m - front),
                      max(right - y_m, 0.0, y_m - left))


def lidar_point_xy(
    angle_rad: float, distance_m: float, x_offset_m: float = 0.0, y_offset_m: float = 0.0,
) -> tuple[float, float]:
    # RPLIDAR angles turn clockwise; robot +X is forward and +Y is left.
    return (distance_m * math.cos(angle_rad) + x_offset_m,
            -distance_m * math.sin(angle_rad) + y_offset_m)
