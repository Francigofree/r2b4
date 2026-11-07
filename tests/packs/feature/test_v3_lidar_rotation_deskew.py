"""A rotating physical scan stays sharp in both local and SLAM geometries."""
from dataclasses import replace
import math

import pytest

from v3.adapters.native_lidar_port import (
    NativeRawLidarSnapshot, TimedPoseReference, rotation_deskew_points,
)
from v3.adapters.process_lidar_port import _wire_control_raw, _wire_raw, _unwire_raw
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.adapters.slam_toolbox_transport import encode_scan, decode_packet
from v3.capture_encoding import encode_raw_lidar_snapshot
from v3.lidar_geometry import lidar_point_xy


START, MID, END = 2_000_000_000, 2_050_000_000, 2_100_000_000
OFFSET = (.08, -.03)


def _heading(timestamp):
    # 1.2 rad/s in-place turn, crossing the +/-pi representation boundary.
    yaw = math.pi + 1.2*(timestamp-MID)/1e9
    return TimedPoseReference(timestamp, 0., 0., math.atan2(math.sin(yaw), math.cos(yaw)))


def _spinning_room_scan():
    points = []
    for index in range(72):
        angle = index * 5.
        timestamp = START + index*(END-START)//72
        # Express the static rectangular room in the midpoint robot frame.
        yaw = 1.2*(timestamp-MID)/1e9
        c, s = math.cos(yaw), math.sin(yaw)
        laser_x = c*OFFSET[0]-s*OFFSET[1]
        laser_y = s*OFFSET[0]+c*OFFSET[1]
        direction = yaw-math.radians(angle)
        dx, dy = math.cos(direction), math.sin(direction)
        lengths = [(wall-origin)/velocity
                   for origin, velocity, walls in ((laser_x, dx, (-2., 2.)),
                                                   (laser_y, dy, (-1.5, 1.5)))
                   if abs(velocity) > 1e-12 for wall in walls
                   if (wall-origin)/velocity > 0]
        points.append(RplidarPoint(angle, min(lengths), 15, timestamp))
    return NativeRawLidarSnapshot(7, END/1e9, START, END, MID, "OK", tuple(points), {}, END)


def _deskew(raw, provider=_heading):
    return rotation_deskew_points(raw.raw_scan, scan_start_ns=START, scan_end_ns=END,
        measurement_ns=MID, pose_provider=provider, lidar_x_m=OFFSET[0], lidar_y_m=OFFSET[1])


def _wall_error(points):
    return max(min(abs(abs(x)-2.), abs(abs(y)-1.5)) for x, y in (
        lidar_point_xy(math.radians(point.angle_deg), point.distance_m, *OFFSET)
        for point in points))


def test_rotating_room_wall_error_is_removed_in_local_and_slam_scan_geometry():
    raw = _spinning_room_scan()
    original = raw.raw_scan
    corrected, reason = _deskew(raw)
    assert reason == "APPLIED"
    assert _wall_error(original) > .04
    assert _wall_error(corrected) < 1e-12
    assert raw.raw_scan is original
    geometry = replace(raw, raw_scan=corrected)
    compact = _wire_control_raw(geometry, minimum_range_m=.08, maximum_range_m=6., maximum_points=96)
    local = _unwire_raw(compact)
    assert _wall_error(local.raw_scan) < 1e-12
    assert local.measurement_monotonic_ns == MID
    packet = decode_packet(encode_scan(geometry, session_id=11, source_pid=22,
        minimum_range_m=.08, maximum_range_m=6., lidar_x_m=OFFSET[0], lidar_y_m=OFFSET[1]))
    slam_points = tuple(RplidarPoint((-index*360/len(packet.ranges_m)) % 360, value, 15)
                        for index, value in enumerate(packet.ranges_m) if value)
    # The ROS scan additionally quantizes the bearing into half-degree bins.
    assert _wall_error(slam_points) < .015
    assert packet.measurement_ns == MID


@pytest.mark.parametrize("missing", (START, MID, END-((END-START)//72)))
def test_missing_heading_bracket_keeps_physical_returns_without_extrapolation(missing):
    raw = _spinning_room_scan()
    # Use the actual last return timestamp (integer scan spacing).
    if missing > MID:
        missing = raw.raw_scan[-1].received_monotonic_ns
    points, reason = _deskew(raw, lambda timestamp: None if timestamp == missing else _heading(timestamp))
    assert points is raw.raw_scan
    assert reason == "HEADING_BRACKET_UNAVAILABLE"


def test_legacy_untimed_scan_and_discontinuous_heading_are_explicit_uncompensated():
    raw = _spinning_room_scan()
    untimed = replace(raw, raw_scan=tuple(replace(point, received_monotonic_ns=None) for point in raw.raw_scan))
    assert _deskew(untimed) == (untimed.raw_scan, "POINT_TIMING_UNAVAILABLE")
    points, reason = _deskew(raw, lambda timestamp: TimedPoseReference(
        timestamp, 0., 0., 0. if timestamp == MID else 1.))
    assert points is raw.raw_scan
    assert reason == "HEADING_DISCONTINUITY"


def test_raw_evidence_retains_receipt_times_without_changing_measurement_identity():
    raw = _spinning_room_scan()
    assert _unwire_raw(_wire_raw(raw)) == raw
    encoded = encode_raw_lidar_snapshot(raw, 100)
    assert encoded["point_received_monotonic_ns"] == [point.received_monotonic_ns for point in raw.raw_scan]
    assert encoded["measurement_monotonic_ns"] == MID
    assert encoded["point_encoding"] == "ANGLE_DEG_DISTANCE_M_QUALITY"
