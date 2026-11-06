"""Bounded passive obstacle observations; never a navigation or raw-scan input."""
from __future__ import annotations

import math
from collections.abc import Mapping

from .lidar_geometry import lidar_point_xy

NAVIGATION_MAP_TOPIC = "v3.navigation_map"
NAVIGATION_MAP_TRANSPORT_TOPIC = "v3.navigation_map_transport"
NAVIGATION_MAP_SCHEMA = "R2B4_NAVIGATION_MAP_V1"
NAVIGATION_MAP_MAX_POINTS = 96


def navigation_map_observation(snapshot, points, *, minimum_range_m: float,
                               maximum_range_m: float, x_offset_m: float,
                               y_offset_m: float) -> dict[str, object]:
    """Called only in the LiDAR producer, from its already bounded control scan."""
    if len(points) > NAVIGATION_MAP_MAX_POINTS:
        points = tuple(points[i * len(points) // NAVIGATION_MAP_MAX_POINTS]
                       for i in range(NAVIGATION_MAP_MAX_POINTS))
    cloud = tuple(lidar_point_xy(math.radians(angle), distance, x_offset_m, y_offset_m)
                  for angle, distance, _quality in points)
    value = {
        "schema": NAVIGATION_MAP_SCHEMA,
        "event_type": "navigation_map_observation",
        "measurement_monotonic_ns": snapshot.measurement_monotonic_ns,
        "scan_start_monotonic_ns": snapshot.scan_start_monotonic_ns,
        "scan_end_monotonic_ns": snapshot.scan_end_monotonic_ns,
        "source_sequence": snapshot.raw_scan_id,
        "sensor_health": snapshot.health,
        "frame_id": "ROBOT_BASE",
        "points_xy_m": cloud,
        "source_point_count": len(snapshot.raw_scan),
        "minimum_range_m": minimum_range_m,
        "maximum_range_m": maximum_range_m,
        "lidar_x_offset_m": x_offset_m,
        "lidar_y_offset_m": y_offset_m,
        "motion_authority": False,
    }
    validate_navigation_map(value)
    return value


def validate_navigation_map(value: object) -> Mapping[str, object]:
    if (not isinstance(value, Mapping) or value.get("schema") != NAVIGATION_MAP_SCHEMA
            or value.get("event_type") != "navigation_map_observation"
            or value.get("frame_id") != "ROBOT_BASE" or value.get("motion_authority") is not False
            or value.get("sensor_health") not in {"OK", "STALE", "ERROR"}):
        raise ValueError("invalid passive navigation map observation")
    for name in ("measurement_monotonic_ns", "scan_start_monotonic_ns", "scan_end_monotonic_ns",
                 "source_sequence", "source_point_count"):
        if type(value.get(name)) is not int or value[name] < 0:
            raise ValueError("invalid navigation map lineage")
    if not value["scan_start_monotonic_ns"] <= value["measurement_monotonic_ns"] <= value["scan_end_monotonic_ns"]:
        raise ValueError("invalid navigation map measurement interval")
    for name in ("minimum_range_m", "maximum_range_m", "lidar_x_offset_m", "lidar_y_offset_m"):
        if type(value.get(name)) not in (int, float) or not math.isfinite(value[name]):
            raise ValueError("invalid navigation map geometry")
    if not 0 <= value["minimum_range_m"] < value["maximum_range_m"]:
        raise ValueError("invalid navigation map range")
    points = value.get("points_xy_m")
    if (not isinstance(points, (tuple, list)) or len(points) > NAVIGATION_MAP_MAX_POINTS
            or len(points) > value["source_point_count"]):
        raise ValueError("navigation map point bound exceeded")
    for point in points:
        if (not isinstance(point, (tuple, list)) or len(point) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)):
            raise ValueError("invalid navigation map point")
    return value
