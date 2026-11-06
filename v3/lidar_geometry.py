"""LiDAR polar returns expressed at the robot's differential-drive origin."""

import math


def lidar_point_xy(
    angle_rad: float, distance_m: float, x_offset_m: float = 0.0, y_offset_m: float = 0.0,
) -> tuple[float, float]:
    # RPLIDAR angles turn clockwise; robot +X is forward and +Y is left.
    return (distance_m * math.cos(angle_rad) + x_offset_m,
            -distance_m * math.sin(angle_rad) + y_offset_m)
