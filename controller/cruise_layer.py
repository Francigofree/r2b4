#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cruise layer: FollowRequest -> navigation intent plus room-cruise fallback."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from controller.follow_types import FollowRequest, TARGET_SOURCE_CAMERA_SEARCH, TARGET_SOURCE_CAMERA_TARGET
from controller.local_planner import _front_scan_gap_analysis
from controller.motion_resolver import ENTRY_TIER_PRIMARY, make_motion_proposal
from controller.motion_schema import execution_mode_for_command
from controller.navigation_intent import NAV_MODE_FOLLOW, NavigationIntent


ROOM_CRUISE_FOLLOW_STYLE = "room_cruise_follow"
ROOM_CRUISE_VALIDATED_CHAIN = "M4_1_room_cruise_quality_validator"

CRUISE_TRIGGER_M = 0.62
CRUISE_STOP_M = 0.50
CRUISE_COLLISION_M = 0.12
CRUISE_PIVOT_FLOOR_M = 0.20
CRUISE_SIDE_REQUIRED_M = 0.50
CRUISE_CAMERA_TARGET_TURN_SIDE_BLOCK_M = CRUISE_SIDE_REQUIRED_M
CRUISE_SIDE_DEADBAND_M = 0.08
CRUISE_WARNING_SIDE_HOLD_MARGIN_M = 0.16
CRUISE_TARGET_REACQUIRE_SIDE_REQUIRED_M = 0.56
CRUISE_TARGET_REACQUIRE_SIDE_MAX_NARROWER_M = 0.24
CRUISE_NEAR_COLLISION_PIVOT_TRACK_MPS = 0.018

CRUISE_FORWARD_TRACK_MPS = 0.034
CRUISE_CLEAR_FORWARD_TRACK_MPS = 0.034
CRUISE_TARGET_CRAWL_TRACK_MPS = 0.012
CRUISE_TARGET_HOLD_M = 0.12
CRUISE_TARGET_HOLD_RELEASE_MARGIN_M = 0.04
CRUISE_MOVING_TARGET_MIN_MPS = 0.010
CRUISE_MOVING_TARGET_INNER_HOLD_M = 0.03
CRUISE_MOVING_TARGET_BUBBLE_TRACK_MPS = 0.028
CRUISE_MOVING_TARGET_HOLD_CREEP_TRACK_MPS = 0.016
CRUISE_MOVING_TARGET_HOLD_CREEP_MIN_RADIAL_MPS = 0.020
CRUISE_MOVING_TARGET_HOLD_CREEP_DISTANCE_MARGIN_M = 0.08
CRUISE_MOVING_TARGET_HOLD_CREEP_MAX_BEARING_RAD = 0.55
CRUISE_BUBBLE_DRIFT_HEADING_DEADBAND_RAD = 0.12
CRUISE_TARGET_SLOW_M = 0.60
CRUISE_TARGET_REAR_X_M = 0.05
CRUISE_PIVOT_TRACK_MPS = 0.030
CRUISE_TARGET_PIVOT_TRACK_MPS = 0.030
CRUISE_TURN_DIFF_MAX_MPS = 0.038
CRUISE_TARGET_TURN_DIFF_MAX_MPS = 0.024
CRUISE_LIDAR_CONFIDENCE_HOLD = 0.30
CRUISE_TARGET_HEADING_ARC_MIN_MPS = 0.018
CRUISE_TARGET_HEADING_ARC_ENTER_RAD = 0.50
CRUISE_TARGET_HEADING_ARC_RELEASE_RAD = 0.36
CRUISE_TARGET_HEADING_ARC_SPEED_SCALE = 2.0
CRUISE_WARNING_HEADING_ARC_MIN_MOVING_TRACK_MPS = 0.034
CRUISE_FRONT_GAP_CONFIDENT_DELTA = 0.16
CRUISE_MAX_TRACK_CAP_MPS = 0.108
CRUISE_SPEED_SCALE_NOMINAL_V_MAX_MPS = 0.08
CRUISE_SPEED_SCALE_MIN = 0.50
CRUISE_SPEED_SCALE_MAX = 1.25
CRUISE_TARGET_DIRECTION_GAP_WINDOW_DEG = 16.0
CRUISE_TARGET_DIRECTION_GAP_MAX_RANGE_M = 2.20
CRUISE_TARGET_DIRECTION_GAP_MIN_POINTS = 2
CRUISE_TARGET_DIRECTION_GAP_RELEASE_MARGIN_M = 0.10
CRUISE_TARGET_DIRECTION_GATE_MIN_DISTANCE_M = 0.22
CRUISE_TARGET_DIRECTION_FORWARD_ARC_DEG = 75.0
CRUISE_FORWARD_HEADING_BIAS_MAX_RAD = 0.24
CRUISE_FORWARD_HEADING_BIAS_MIN_DELTA_M = 0.18
CRUISE_FORWARD_HEADING_BIAS_HOLD_MARGIN_M = 0.16
CRUISE_FORWARD_HEADING_OPPOSITE_TURN_GAIN = 0.35
CRUISE_TARGET_HEADING_DEADBAND_RAD = 0.035
CRUISE_TARGET_HEADING_SIDE_HOLD_RAD = 0.18
CRUISE_TARGET_HOLD_HEADING_ALIGN_DEADBAND_RAD = 0.035
CRUISE_TARGET_HOLD_HEADING_ALIGN_TRACK_MPS = 0.018
CRUISE_TARGET_REACQUIRE_ROTATE_TRACK_MPS = 0.018
CRUISE_TARGET_SEARCH_ROTATE_TRACK_MPS = 0.020
CRUISE_CAMERA_FRONT_HOLD_ALIGN_MIN_CLEARANCE_M = 0.76
CRUISE_TARGET_SEARCH_MIN_FRONT_CLEARANCE_M = 0.62
CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M = 0.70
CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M = 0.60
CRUISE_CAMERA_FRONT_HOLD_RETREAT_REAR_MIN_M = 0.65
CRUISE_CAMERA_FRONT_HOLD_RETREAT_TRACK_MPS = 0.030
CRUISE_CAMERA_RETREAT_MIN_TRACK_MPS = 0.014
CRUISE_CAMERA_URGENT_RETREAT_MIN_TRACK_MPS = 0.035
CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_FRONT_M = CRUISE_TRIGGER_M
CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_TRACK_MPS = 0.018
CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MARGIN_M = 0.08
CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MAX_BEARING_RAD = 0.26
CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MIN_CONFIDENCE = 0.25
CRUISE_CAMERA_TARGET_CLOSE_FRONT_MARGIN_NEAR_M = 0.30
CRUISE_CAMERA_TARGET_CLOSE_FRONT_MARGIN_ROOM_M = 0.03
CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD = 0.050
CRUISE_CAMERA_TARGET_CENTER_ZONE_RAD = 0.140
CRUISE_CAMERA_TARGET_ALIGN_RELEASE_RAD = 0.085
CRUISE_CAMERA_PERSISTED_REACQUIRE_MIN_BEARING_RAD = CRUISE_CAMERA_TARGET_CENTER_ZONE_RAD
CRUISE_CAMERA_TARGET_TURN_MIN_TRACK_MPS = 0.010
CRUISE_CAMERA_TARGET_ONE_TRACK_MPS = 0.048
CRUISE_CAMERA_SIMPLE_TURN_FULL_RAD = 0.34
CRUISE_CAMERA_SIMPLE_FORWARD_MAX_BEARING_RAD = 0.28
CRUISE_CAMERA_SIMPLE_FORWARD_SLOW_BEARING_RAD = 0.13
CRUISE_CAMERA_SIMPLE_FORWARD_MIN_FRONT_CLEARANCE_M = 0.86
CRUISE_CAMERA_SIMPLE_FORWARD_DESIRED_CLEARANCE_MARGIN_M = 0.68
CRUISE_CAMERA_SIMPLE_DISTANCE_DEADBAND_M = 0.055
CRUISE_CAMERA_SIMPLE_DISTANCE_CONTROL_MAX_DESIRED_M = 1.75
CRUISE_CAMERA_SIMPLE_DISTANCE_SLOW_M = 0.42
CRUISE_CAMERA_SIMPLE_FORWARD_CRAWL_TRACK_MPS = 0.010
CRUISE_CAMERA_SIMPLE_FORWARD_FLOOR_MAX_TRACK_MPS = 0.040
CRUISE_CAMERA_SIMPLE_RETREAT_TRACK_MPS = 0.028
CRUISE_CAMERA_SIMPLE_TURN_OMEGA_LIMIT_RATIO = 0.82
CRUISE_CAMERA_LOW_LIDAR_BYPASS_CONFIDENCE = 0.45
CRUISE_CAMERA_LOW_LIDAR_BYPASS_FRONT_MARGIN_M = 0.12


@dataclass(frozen=True)
class CruiseLayerResult:
    proposal: Optional[Dict[str, Any]]
    status: Dict[str, Any]


def _finite_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(out) and out >= 0.0:
        return float(out)
    return None


def _positive_finite_float(value: Any) -> Optional[float]:
    out = _finite_float(value)
    if out is None or float(out) <= 0.0:
        return None
    return float(out)


def _first_finite(data: Dict[str, Any], keys: Tuple[str, ...]) -> Tuple[Optional[float], str]:
    src = dict(data or {})
    for key in keys:
        out = _finite_float(src.get(key))
        if out is not None:
            return out, str(key)
    return None, ""


def _first_positive_finite(data: Dict[str, Any], keys: Tuple[str, ...]) -> Tuple[Optional[float], str]:
    src = dict(data or {})
    for key in keys:
        out = _positive_finite_float(src.get(key))
        if out is not None:
            return out, str(key)
    return None, ""


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def _wrap_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def _extract_pose(ekf_state: Dict[str, Any]) -> Tuple[float, float, float]:
    src = dict(ekf_state or {})
    x = float(src.get("x", 0.0) or 0.0)
    y = float(src.get("y", 0.0) or 0.0)
    theta = src.get("theta")
    if theta is None:
        theta = math.radians(float(src.get("theta_deg", 0.0) or 0.0))
    return float(x), float(y), float(theta)


def _target_geometry(
    *,
    target_pose: Tuple[float, float, float],
    ekf_state: Dict[str, Any],
) -> Dict[str, float]:
    x, y, theta = _extract_pose(ekf_state)
    gx, gy, gtheta = target_pose
    dx = float(gx) - float(x)
    dy = float(gy) - float(y)
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    robot_x = c * dx + s * dy
    robot_y = -s * dx + c * dy
    bearing = _wrap_angle(math.atan2(dy, dx) - float(theta)) if (dx or dy) else 0.0
    return {
        "distance_m": float(math.hypot(dx, dy)),
        "bearing_error_rad": float(bearing),
        "robot_frame_x_m": float(robot_x),
        "robot_frame_y_m": float(robot_y),
        "target_theta_rad": float(gtheta),
    }


def _request_target_speed_mps(request: FollowRequest) -> float:
    def _signed(value: Any) -> Optional[float]:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return None
        if math.isfinite(out):
            return float(out)
        return None

    vx = _signed(getattr(request, "target_vx", None))
    vy = _signed(getattr(request, "target_vy", None))
    if vx is None and vy is None:
        return 0.0
    return float(math.hypot(float(vx or 0.0), float(vy or 0.0)))


def _actual_target_context(
    request: FollowRequest,
    *,
    ekf_state: Dict[str, Any],
) -> Dict[str, float]:
    x, y, theta = _extract_pose(ekf_state)

    def _signed(value: Any) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return 0.0
        return float(out) if math.isfinite(out) else 0.0

    tx = _signed(getattr(request, "target_x", None))
    ty = _signed(getattr(request, "target_y", None))
    vx = _signed(getattr(request, "target_vx", None))
    vy = _signed(getattr(request, "target_vy", None))
    dx = tx - float(x)
    dy = ty - float(y)
    dist = math.hypot(dx, dy)
    radial_velocity_mps = 0.0
    if dist > 1e-6:
        radial_velocity_mps = (vx * dx + vy * dy) / max(1e-6, dist)
    bearing = _wrap_angle(math.atan2(dy, dx) - float(theta)) if dist > 1e-6 else 0.0
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    robot_x = c * dx + s * dy
    robot_y = -s * dx + c * dy
    return {
        "distance_m": float(dist),
        "bearing_error_rad": float(bearing),
        "radial_velocity_mps": float(radial_velocity_mps),
        "robot_frame_x_m": float(robot_x),
        "robot_frame_y_m": float(robot_y),
    }


def _clearances(lidar_summary: Dict[str, Any]) -> Dict[str, Any]:
    src = dict(lidar_summary or {})
    global_min_m, global_min_source = _first_positive_finite(
        src,
        ("min_dist", "min_clearance_m", "clearance_m"),
    )
    front_m, front_source = _first_positive_finite(
        src,
        ("min_dist_narrow", "front_clearance_m", "front_clearance", "min_dist"),
    )
    left_m, left_source = _first_positive_finite(
        src,
        ("left_clearance_m", "left_clearance", "min_left_clearance_m", "avg_left"),
    )
    right_m, right_source = _first_positive_finite(
        src,
        ("right_clearance_m", "right_clearance", "min_right_clearance_m", "avg_right"),
    )
    rear_m, rear_source = _first_positive_finite(
        src,
        ("min_back", "rear_clearance_m", "back_clearance_m", "avg_back"),
    )
    return {
        "global_min_m": global_min_m,
        "global_min_source": global_min_source,
        "front_m": front_m,
        "front_source": front_source,
        "left_m": left_m,
        "left_source": left_source,
        "right_m": right_m,
        "right_source": right_source,
        "rear_m": rear_m,
        "rear_source": rear_source,
        "blocked_front": bool(src.get("blocked_front", False)),
        "blocked_back": bool(src.get("blocked_back", False)),
        "lidar_confidence": _finite_float(src.get("latest_confidence", src.get("lidar_pose_confidence"))),
    }


def _track_limits(request: FollowRequest, track_width_m: float) -> Dict[str, float]:
    req_v = _finite_float(getattr(request, "v_max_mps", None))
    req_omega = _finite_float(getattr(request, "omega_max_rad_s", None))
    max_track = min(CRUISE_MAX_TRACK_CAP_MPS, float(req_v) if req_v and req_v > 0.0 else 0.08)
    scale_source_v = float(req_v) if req_v and req_v > 0.0 else CRUISE_SPEED_SCALE_NOMINAL_V_MAX_MPS
    speed_scale = _clamp(
        scale_source_v / max(1e-6, CRUISE_SPEED_SCALE_NOMINAL_V_MAX_MPS),
        CRUISE_SPEED_SCALE_MIN,
        CRUISE_SPEED_SCALE_MAX,
    )
    omega_limit_track_delta = (
        0.5 * float(req_omega) * max(0.01, float(track_width_m))
        if req_omega and req_omega > 0.0
        else CRUISE_TURN_DIFF_MAX_MPS
    )
    return {
        "max_track_mps": max(0.012, float(max_track)),
        "omega_limit_track_delta_mps": max(0.004, float(omega_limit_track_delta)),
        "max_turn_diff_mps": max(
            0.004,
            min(CRUISE_TURN_DIFF_MAX_MPS * float(speed_scale), float(omega_limit_track_delta)),
        ),
        "target_turn_diff_mps": max(
            0.004,
            min(CRUISE_TARGET_TURN_DIFF_MAX_MPS * float(speed_scale), float(omega_limit_track_delta)),
        ),
        "pivot_track_mps": max(0.010, min(CRUISE_PIVOT_TRACK_MPS * float(speed_scale), float(omega_limit_track_delta))),
        "target_pivot_track_mps": max(
            0.008,
            min(CRUISE_TARGET_PIVOT_TRACK_MPS * float(speed_scale), float(omega_limit_track_delta)),
        ),
        "speed_scale": float(speed_scale),
    }


def _scaled_speed(limits: Dict[str, float], value: float) -> float:
    return float(value) * float(limits.get("speed_scale", 1.0) or 1.0)


def _tracks_to_twist(left_mps: float, right_mps: float, track_width_m: float) -> Tuple[float, float]:
    width = max(0.01, float(track_width_m))
    return 0.5 * (float(left_mps) + float(right_mps)), (float(right_mps) - float(left_mps)) / width


def _side_clearance(clear: Dict[str, Any], side: str) -> Optional[float]:
    if str(side) == "right":
        return _finite_float(clear.get("right_m"))
    if str(side) == "left":
        return _finite_float(clear.get("left_m"))
    return None


def _opposite_side(side: str) -> str:
    return "left" if str(side) == "right" else "right"


def _scan_point_to_bearing_distance(point: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(point, dict):
        return None
    dist_raw = point.get("distance_m", point.get("dist", point.get("dist_mm", None)))
    try:
        dist_value = float(dist_raw)
    except Exception:
        return None
    if not math.isfinite(dist_value) or dist_value <= 0.0:
        return None
    dist_m = dist_value / 1000.0 if dist_value > 20.0 else dist_value
    if not math.isfinite(dist_m) or dist_m <= 0.02:
        return None

    angle_rad_raw = point.get("angle_rad")
    if angle_rad_raw is not None:
        try:
            angle_rad = float(angle_rad_raw)
        except Exception:
            return None
    else:
        try:
            angle_rad = math.radians(float(point.get("angle_deg", point.get("angle", 0.0))))
        except Exception:
            return None
    if not math.isfinite(angle_rad):
        return None

    x = float(math.cos(angle_rad) * dist_m)
    y = float(-math.sin(angle_rad) * dist_m)
    return float(math.atan2(y, x)), float(math.hypot(x, y))


def _bearing_delta_abs_deg(a_deg: float, b_deg: float) -> float:
    return abs(((float(a_deg) - float(b_deg) + 180.0) % 360.0) - 180.0)


def _raw_scan_target_direction_gap(
    raw_scan: Optional[List[Dict[str, Any]]],
    target_bearing_rad: float,
) -> Dict[str, Any]:
    points = list(raw_scan or [])
    target_bearing_deg = math.degrees(float(target_bearing_rad))
    forward_relevant = bool(abs(float(target_bearing_deg)) <= CRUISE_TARGET_DIRECTION_FORWARD_ARC_DEG)
    distances: List[float] = []
    for point in points:
        parsed = _scan_point_to_bearing_distance(point)
        if parsed is None:
            continue
        bearing_rad, dist_m = parsed
        if not math.isfinite(dist_m) or dist_m < 0.05 or dist_m > CRUISE_TARGET_DIRECTION_GAP_MAX_RANGE_M:
            continue
        bearing_deg = math.degrees(float(bearing_rad))
        if _bearing_delta_abs_deg(bearing_deg, target_bearing_deg) <= CRUISE_TARGET_DIRECTION_GAP_WINDOW_DEG:
            distances.append(float(dist_m))

    distances = sorted(distances)
    if len(distances) < CRUISE_TARGET_DIRECTION_GAP_MIN_POINTS:
        return {
            "enabled": True,
            "has_data": False,
            "reason": "insufficient_points" if points else "no_scan",
            "target_bearing_deg": round(float(target_bearing_deg), 2),
            "window_deg": float(CRUISE_TARGET_DIRECTION_GAP_WINDOW_DEG),
            "point_count": int(len(distances)),
            "forward_relevant": bool(forward_relevant),
            "clearance_m": None,
            "blocked_by_scan": False,
            "hard_blocked_by_scan": False,
            "trigger_m": float(CRUISE_TRIGGER_M),
            "stop_m": float(CRUISE_STOP_M),
        }

    idx = int(round((len(distances) - 1) * 0.25))
    clearance_m = float(distances[max(0, min(len(distances) - 1, idx))])
    return {
        "enabled": True,
        "has_data": True,
        "reason": "scored",
        "target_bearing_deg": round(float(target_bearing_deg), 2),
        "window_deg": float(CRUISE_TARGET_DIRECTION_GAP_WINDOW_DEG),
        "point_count": int(len(distances)),
        "forward_relevant": bool(forward_relevant),
        "clearance_m": round(float(clearance_m), 4),
        "blocked_by_scan": bool(forward_relevant and clearance_m <= CRUISE_TRIGGER_M),
        "hard_blocked_by_scan": bool(forward_relevant and clearance_m <= CRUISE_STOP_M),
        "trigger_m": float(CRUISE_TRIGGER_M),
        "stop_m": float(CRUISE_STOP_M),
    }


def _raw_scan_side_clearance(raw_scan: List[Dict[str, Any]], side: str) -> Dict[str, Any]:
    side_norm = str(side or "").strip().lower()
    primary: List[float] = []
    fallback: List[float] = []
    for point in list(raw_scan or []):
        parsed = _scan_point_to_bearing_distance(point)
        if parsed is None:
            continue
        bearing_rad, dist_m = parsed
        if not math.isfinite(dist_m) or dist_m < 0.05 or dist_m > 2.50:
            continue
        bearing_deg = math.degrees(float(bearing_rad))
        if side_norm == "left":
            if 60.0 <= bearing_deg <= 120.0:
                primary.append(float(dist_m))
            elif 35.0 <= bearing_deg <= 145.0:
                fallback.append(float(dist_m))
        elif side_norm == "right":
            if -120.0 <= bearing_deg <= -60.0:
                primary.append(float(dist_m))
            elif -145.0 <= bearing_deg <= -35.0:
                fallback.append(float(dist_m))

    values = primary if len(primary) >= 3 else (primary + fallback)
    values = sorted(float(v) for v in values if math.isfinite(v) and v > 0.0)
    if len(values) < 3:
        return {
            "has_data": False,
            "clearance_m": None,
            "point_count": int(len(values)),
            "source": "raw_scan_insufficient_points",
        }
    idx = int(round((len(values) - 1) * 0.35))
    return {
        "has_data": True,
        "clearance_m": float(values[max(0, min(len(values) - 1, idx))]),
        "point_count": int(len(values)),
        "source": "raw_scan_side_sector",
    }


def _raw_scan_follow_context(raw_scan: Optional[List[Dict[str, Any]]]) -> Dict[str, Any]:
    points = list(raw_scan or [])
    front_gap = _front_scan_gap_analysis(points) if points else {"enabled": True, "has_data": False, "reason": "no_scan"}
    left = _raw_scan_side_clearance(points, "left")
    right = _raw_scan_side_clearance(points, "right")
    return {
        "scan_points": int(len(points)),
        "left": left,
        "right": right,
        "front_gap": dict(front_gap or {}),
    }


def _arc_tracks(side: str, base_mps: float, diff_mps: float, max_track_mps: float) -> Tuple[float, float]:
    base = _clamp(float(base_mps), 0.0, float(max_track_mps))
    diff = max(0.0, min(float(diff_mps), float(max_track_mps)))
    if str(side) == "right":
        return min(float(max_track_mps), base + diff), max(0.0, base - diff)
    return max(0.0, base - diff), min(float(max_track_mps), base + diff)


def _one_track_arc_tracks(side: str, moving_track_mps: float, max_track_mps: float) -> Tuple[float, float]:
    moving = _clamp(float(moving_track_mps), 0.0, float(max_track_mps))
    return _arc_tracks(str(side), moving * 0.5, moving * 0.5, max_track_mps)


def _camera_one_track_direction_tracks(target_side: str, moving_track_mps: float, max_track_mps: float) -> Tuple[float, float]:
    moving = _clamp(float(moving_track_mps), 0.0, float(max_track_mps))
    if str(target_side) == "right":
        return float(moving), 0.0
    if str(target_side) == "left":
        return 0.0, float(moving)
    return 0.0, 0.0


def _in_place_pivot_tracks(side: str, track_mps: float, max_track_mps: float) -> Tuple[float, float]:
    speed = _clamp(float(track_mps), 0.0, float(max_track_mps))
    if str(side) == "right":
        return float(speed), -float(speed)
    return -float(speed), float(speed)


def _camera_retreat_track_mps(
    limits: Dict[str, float],
    *,
    effective_front_m: float,
    desired_distance_m: float,
    base_track_mps: float,
) -> float:
    if float(desired_distance_m) > 0.0:
        urgent_margin_m = 0.14 if float(desired_distance_m) <= 0.70 else 0.10
        urgent_front_m = max(
            CRUISE_STOP_M + 0.08,
            min(
                CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M,
                float(desired_distance_m) + urgent_margin_m,
            ),
        )
    else:
        urgent_front_m = CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_FRONT_M
    min_track = (
        CRUISE_CAMERA_URGENT_RETREAT_MIN_TRACK_MPS
        if float(effective_front_m) <= urgent_front_m
        else CRUISE_CAMERA_RETREAT_MIN_TRACK_MPS
    )
    return min(
        float(limits["max_track_mps"]),
        max(float(min_track), _scaled_speed(limits, float(base_track_mps))),
    )


def _target_aligned_retreat_tracks(
    *,
    retreat_track_mps: float,
    target_bearing_rad: float,
    turn_cap_mps: float,
    max_track_mps: float,
) -> Tuple[float, float, float]:
    retreat = _clamp(float(retreat_track_mps), 0.0, float(max_track_mps))
    left_mps = -float(retreat)
    right_mps = -float(retreat)
    bearing_abs = abs(float(target_bearing_rad))
    turn_track_mps = 0.0
    if bearing_abs > CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD:
        turn_ratio = _clamp(
            (bearing_abs - CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD)
            / max(0.01, CRUISE_CAMERA_SIMPLE_TURN_FULL_RAD - CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD),
            0.0,
            1.0,
        )
        turn_track_mps = min(
            float(turn_cap_mps),
            retreat * 0.85,
            max(0.004, (turn_ratio ** 0.72) * float(turn_cap_mps)),
        )
        if float(target_bearing_rad) >= 0.0:
            left_mps -= float(turn_track_mps)
            right_mps += float(turn_track_mps)
        else:
            left_mps += float(turn_track_mps)
            right_mps -= float(turn_track_mps)
    return (
        _clamp(left_mps, -float(max_track_mps), 0.0),
        _clamp(right_mps, -float(max_track_mps), 0.0),
        float(turn_track_mps),
    )


def _camera_target_turn_track_mps(limits: Dict[str, float], bearing_abs_rad: float) -> float:
    max_track = max(0.0, float(limits.get("max_track_mps", 0.0)))
    if max_track <= 0.0 or float(bearing_abs_rad) <= CRUISE_CAMERA_TARGET_ALIGN_RELEASE_RAD:
        return 0.0
    turn_min = min(max_track, _scaled_speed(limits, CRUISE_CAMERA_TARGET_TURN_MIN_TRACK_MPS))
    turn_max = min(max_track, _scaled_speed(limits, CRUISE_CAMERA_TARGET_ONE_TRACK_MPS))
    span = max(0.01, CRUISE_CAMERA_SIMPLE_TURN_FULL_RAD - CRUISE_CAMERA_TARGET_ALIGN_RELEASE_RAD)
    ratio = _clamp((float(bearing_abs_rad) - CRUISE_CAMERA_TARGET_ALIGN_RELEASE_RAD) / span, 0.0, 1.0)
    shaped = ratio ** 0.85
    return min(max_track, max(turn_min, turn_min + shaped * (turn_max - turn_min)))


def _sharp_heading_arc_tracks(side: str, limits: Dict[str, float]) -> Tuple[float, float]:
    max_track = max(0.0, float(limits.get("max_track_mps", 0.0)))
    heading_arc_min = _scaled_speed(limits, CRUISE_TARGET_HEADING_ARC_MIN_MPS)
    arc_track = min(
        max_track * 0.5,
        CRUISE_TARGET_HEADING_ARC_SPEED_SCALE * max(
            heading_arc_min,
            float(limits.get("target_pivot_track_mps", CRUISE_TARGET_PIVOT_TRACK_MPS)),
        ),
    )
    if max_track <= 0.0:
        return 0.0, 0.0
    return _arc_tracks(str(side), arc_track, arc_track, max_track)


def _warning_heading_arc_tracks(
    side: str,
    limits: Dict[str, float],
    effective_front_m: float,
) -> Tuple[float, float]:
    max_track = max(0.0, float(limits.get("max_track_mps", 0.0)))
    if max_track <= 0.0:
        return 0.0, 0.0
    clearance_ratio = _clamp(
        (float(effective_front_m) - CRUISE_STOP_M) / max(0.01, CRUISE_TRIGGER_M - CRUISE_STOP_M),
        0.0,
        1.0,
    )
    base_arc = min(
        max_track * 0.5,
        max(
            _scaled_speed(limits, CRUISE_TARGET_HEADING_ARC_MIN_MPS),
            float(limits.get("target_pivot_track_mps", CRUISE_TARGET_PIVOT_TRACK_MPS)),
        ),
    )
    arc_track = max(0.004, base_arc * (0.25 + 0.55 * clearance_ratio))
    arc_track = max(
        arc_track,
        min(base_arc, 0.5 * _scaled_speed(limits, CRUISE_WARNING_HEADING_ARC_MIN_MOVING_TRACK_MPS)),
    )
    return _arc_tracks(str(side), arc_track, arc_track, max_track)


def _follow_state_for_decision(
    *,
    confidence_hold: bool,
    target_hold_active: bool,
    distance_m: float,
    target_slow_distance_m: float,
    target_rearward: bool,
    target_outside_forward_arc: bool,
    obstacle_gate: bool,
) -> str:
    if bool(confidence_hold):
        return "hold"
    if bool(target_hold_active):
        return "hold"
    if bool(target_rearward) or bool(target_outside_forward_arc):
        return "reacquire"
    if bool(obstacle_gate):
        return "reacquire"
    if float(distance_m) > float(target_slow_distance_m):
        return "approach"
    return "track"


class CruiseLayer:
    """Motion gate for FOLLOW: produce only room-cruise track primitives."""

    def __init__(self) -> None:
        self._avoidance_side = ""
        self._forward_bias_side = ""
        self._target_heading_side = ""
        self._target_hold_latched = False
        self._heading_reacquire_latched = False
        self._target_direction_blocked_latched = False
        self._camera_target_align_latched = False

    def _reset_follow_latches(self) -> None:
        self._avoidance_side = ""
        self._forward_bias_side = ""
        self._target_heading_side = ""
        self._target_hold_latched = False
        self._heading_reacquire_latched = False
        self._target_direction_blocked_latched = False
        self._camera_target_align_latched = False

    def _preferred_target_side(self, bearing_rad: float) -> str:
        geometric = "left" if float(bearing_rad) >= 0.0 else "right"
        held = str(self._target_heading_side or "")
        if held in {"left", "right"} and abs(float(bearing_rad)) < CRUISE_TARGET_HEADING_SIDE_HOLD_RAD:
            return held
        self._target_heading_side = geometric
        return geometric

    def _select_wider_side(
        self,
        *,
        left_m: Optional[float],
        right_m: Optional[float],
        preferred_side: str,
        obstacle_gate: bool,
    ) -> Tuple[str, str]:
        left_open = bool(left_m is not None and float(left_m) >= CRUISE_SIDE_REQUIRED_M)
        right_open = bool(right_m is not None and float(right_m) >= CRUISE_SIDE_REQUIRED_M)
        if not left_open and not right_open:
            return "", "none_open"
        if left_open and not right_open:
            return "left", "only_left_open"
        if right_open and not left_open:
            return "right", "only_right_open"

        left_val = float(left_m if left_m is not None else 0.0)
        right_val = float(right_m if right_m is not None else 0.0)
        delta = left_val - right_val
        if abs(delta) > CRUISE_SIDE_DEADBAND_M:
            return ("left", "wider_side") if delta > 0.0 else ("right", "wider_side")

        preferred = "right" if str(preferred_side or "").lower() == "right" else "left"
        if bool(obstacle_gate) and self._avoidance_side in {"left", "right"}:
            return self._avoidance_side, "held_tie"
        return preferred, "target_clear_tie"

    def _select_forward_bias_side(
        self,
        *,
        clear: Dict[str, Any],
        preferred_side: str,
        bearing_abs: float,
        target_direction_blocked: bool,
    ) -> Tuple[str, str, bool]:
        if bool(target_direction_blocked) or float(bearing_abs) > CRUISE_FORWARD_HEADING_BIAS_MAX_RAD:
            return "", "", False

        left_m = _side_clearance(clear, "left")
        right_m = _side_clearance(clear, "right")
        left_open = bool(left_m is not None and float(left_m) >= CRUISE_SIDE_REQUIRED_M)
        right_open = bool(right_m is not None and float(right_m) >= CRUISE_SIDE_REQUIRED_M)
        if not left_open and not right_open:
            self._forward_bias_side = ""
            return "", "", False
        if left_open and not right_open:
            self._forward_bias_side = "left"
            return "left", "only_left_open", False
        if right_open and not left_open:
            self._forward_bias_side = "right"
            return "right", "only_right_open", False

        left_val = float(left_m if left_m is not None else 0.0)
        right_val = float(right_m if right_m is not None else 0.0)
        wider_side = "left" if left_val >= right_val else "right"
        narrower_side = _opposite_side(wider_side)
        delta = abs(left_val - right_val)

        if self._forward_bias_side in {"left", "right"}:
            held = self._forward_bias_side
            held_m = left_val if held == "left" else right_val
            other_m = right_val if held == "left" else left_val
            if held_m + CRUISE_FORWARD_HEADING_BIAS_HOLD_MARGIN_M >= other_m:
                return held, "held_forward_space_bias", True

        preferred = "right" if str(preferred_side or "").lower() == "right" else "left"
        if delta >= CRUISE_FORWARD_HEADING_BIAS_MIN_DELTA_M:
            self._forward_bias_side = wider_side
            if preferred == narrower_side:
                return wider_side, "wider_side_small_heading", True
            return wider_side, "wider_side_target_aligned", True

        self._forward_bias_side = preferred
        return preferred, "target_heading", False

    def _hold_warning_side_if_still_open(
        self,
        *,
        clear: Dict[str, Any],
        selected_side: str,
        side_selection: str,
    ) -> Tuple[str, str]:
        held = str(self._avoidance_side or "")
        selected = str(selected_side or "")
        if held not in {"left", "right"} or selected not in {"left", "right"} or held == selected:
            return selected_side, side_selection

        held_m = _side_clearance(clear, held)
        if held_m is None or float(held_m) < CRUISE_SIDE_REQUIRED_M:
            return selected_side, side_selection

        selected_m = _side_clearance(clear, selected)
        if selected_m is None or float(held_m) + CRUISE_WARNING_SIDE_HOLD_MARGIN_M >= float(selected_m):
            return held, "held_warning_side"
        return selected_side, side_selection

    def tick(
        self,
        request: FollowRequest,
        *,
        local_planner: Any = None,
        ekf_state: Dict[str, Any],
        lidar_summary: Dict[str, Any],
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        source: str = "STATE",
        dt: float = 0.02,
        track_width_m: float = 0.175,
    ) -> CruiseLayerResult:
        req_status = request.to_dict() if isinstance(request, FollowRequest) else {}
        if not isinstance(request, FollowRequest) or not request.active:
            self._reset_follow_latches()
            return CruiseLayerResult(
                proposal=None,
                status={
                    "active": False,
                    "reason": str(req_status.get("reason") or "follow_request_inactive"),
                    "follow_request": req_status,
                },
            )

        target_pose = request.target_pose()
        if target_pose is None:
            self._reset_follow_latches()
            return CruiseLayerResult(
                proposal=None,
                status={
                    "active": False,
                    "reason": "follow_request_target_pose_missing",
                    "follow_request": req_status,
                },
            )

        width = max(0.01, float(track_width_m or 0.175))
        limits = _track_limits(request, width)
        geom = _target_geometry(target_pose=target_pose, ekf_state=dict(ekf_state or {}))
        clear = _clearances(dict(lidar_summary or {}))
        raw_context = _raw_scan_follow_context(raw_scan)
        if bool((raw_context.get("left") or {}).get("has_data", False)):
            clear["left_m"] = float((raw_context.get("left") or {}).get("clearance_m"))
            clear["left_source"] = "raw_scan_side_sector"
        if bool((raw_context.get("right") or {}).get("has_data", False)):
            clear["right_m"] = float((raw_context.get("right") or {}).get("clearance_m"))
            clear["right_source"] = "raw_scan_side_sector"
        distance_m = float(geom["distance_m"])
        bearing_abs = abs(float(geom["bearing_error_rad"]))
        robot_frame_x_m = float(geom["robot_frame_x_m"])
        desired_distance_m = max(0.0, _finite_float(getattr(request, "desired_distance_m", 0.0)) or 0.0)
        target_source = str(getattr(request, "target_source", "") or "")
        camera_target_follow = bool(target_source == TARGET_SOURCE_CAMERA_TARGET)
        target_id = str(getattr(request, "target_id", "") or "")
        target_zone = str(getattr(request, "target_zone", "") or "").strip().lower()
        if target_zone not in {"left", "center", "right"}:
            target_zone = ""
        target_search_side = "right" if ("_right" in target_id) else ("left" if ("_left" in target_id) else "left")
        camera_front_lidar_hold = bool(camera_target_follow and target_id.startswith("camera_front_lidar_hold"))
        camera_front_lidar_hold_human_confirmed = bool(target_id == "camera_front_lidar_hold_human_confirmed")
        camera_front_obstacle_arbitrated = bool(camera_target_follow and target_id == "camera_front_obstacle_arbitrated")
        request_front_obstacle_m = _finite_float(getattr(request, "front_obstacle_distance_m", None))
        target_search_active = bool(
            target_source == TARGET_SOURCE_CAMERA_SEARCH or str(getattr(request, "reason", "") or "") == "target_search_scan"
        )
        target_search_direction_only = bool(target_search_active and target_id.endswith("_direction_only"))
        camera_clearance_guard_active = bool(target_source in {TARGET_SOURCE_CAMERA_TARGET, TARGET_SOURCE_CAMERA_SEARCH})
        target_speed_mps = _request_target_speed_mps(request)
        target_moving = bool(target_speed_mps >= CRUISE_MOVING_TARGET_MIN_MPS)
        actual_target = _actual_target_context(request, ekf_state=dict(ekf_state or {}))
        actual_target_distance_m = float(actual_target["distance_m"])
        target_stop_distance_m = float(CRUISE_TARGET_HOLD_M)
        target_slow_distance_m = max(
            CRUISE_TARGET_SLOW_M,
            target_stop_distance_m + 0.20,
        )
        target_hold_release_m = float(target_stop_distance_m) + CRUISE_TARGET_HOLD_RELEASE_MARGIN_M
        target_hold_latch_candidate = False
        if self._target_hold_latched:
            target_hold_latch_candidate = bool(distance_m <= target_hold_release_m)
        else:
            target_hold_latch_candidate = bool(distance_m <= target_stop_distance_m)
        target_bubble_drift = bool(
            target_hold_latch_candidate
            and target_moving
            and distance_m > CRUISE_MOVING_TARGET_INNER_HOLD_M
        )
        target_hold_creep = bool(
            target_hold_latch_candidate
            and not target_bubble_drift
            and target_moving
            and str(request.reason or "") == "inside_follow_standoff"
            and desired_distance_m > 0.0
            and float(actual_target["distance_m"])
            >= max(0.0, desired_distance_m - CRUISE_MOVING_TARGET_HOLD_CREEP_DISTANCE_MARGIN_M)
            and float(actual_target["radial_velocity_mps"]) >= CRUISE_MOVING_TARGET_HOLD_CREEP_MIN_RADIAL_MPS
            and abs(float(actual_target["bearing_error_rad"])) <= CRUISE_MOVING_TARGET_HOLD_CREEP_MAX_BEARING_RAD
        )
        self._target_hold_latched = bool(
            target_hold_latch_candidate
            and not target_bubble_drift
            and not target_hold_creep
        )
        target_hold_active = bool(self._target_hold_latched)
        target_bearing_rad = float(actual_target["bearing_error_rad"])
        actual_target_bearing_abs = abs(float(target_bearing_rad))
        if target_hold_active:
            self._heading_reacquire_latched = False
        elif self._heading_reacquire_latched:
            self._heading_reacquire_latched = bool(actual_target_bearing_abs >= CRUISE_TARGET_HEADING_ARC_RELEASE_RAD)
        else:
            self._heading_reacquire_latched = bool(actual_target_bearing_abs >= CRUISE_TARGET_HEADING_ARC_ENTER_RAD)
        target_rearward = bool(
            (not target_hold_active)
            and float(actual_target["robot_frame_x_m"]) <= CRUISE_TARGET_REAR_X_M
            and distance_m > target_stop_distance_m
        )
        target_outside_forward_arc = bool((not target_hold_active) and self._heading_reacquire_latched)
        preferred_side = self._preferred_target_side(float(target_bearing_rad))
        target_heading_side = preferred_side if actual_target_bearing_abs > CRUISE_TARGET_HEADING_DEADBAND_RAD else ""
        if camera_target_follow and target_zone in {"left", "right"}:
            camera_target_turn_side = str(target_zone)
        elif camera_target_follow and actual_target_bearing_abs > CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD:
            camera_target_turn_side = "left" if float(target_bearing_rad) >= 0.0 else "right"
        else:
            camera_target_turn_side = ""
        camera_target_turn_side_clearance_m = (
            _side_clearance(clear, camera_target_turn_side)
            if camera_target_turn_side in {"left", "right"}
            else None
        )
        camera_target_turn_side_blocked = bool(
            camera_target_turn_side in {"left", "right"}
            and camera_target_turn_side_clearance_m is not None
            and float(camera_target_turn_side_clearance_m) < CRUISE_CAMERA_TARGET_TURN_SIDE_BLOCK_M
        )
        if not camera_target_follow or target_search_active or target_zone == "center":
            self._camera_target_align_latched = False
        elif target_zone in {"left", "right"}:
            self._camera_target_align_latched = True
        elif self._camera_target_align_latched:
            self._camera_target_align_latched = bool(
                actual_target_bearing_abs > CRUISE_CAMERA_TARGET_ALIGN_RELEASE_RAD
            )
        else:
            self._camera_target_align_latched = bool(
                actual_target_bearing_abs > CRUISE_CAMERA_TARGET_CENTER_ZONE_RAD
            )
        camera_target_align_active = bool(self._camera_target_align_latched)
        preferred_side_clearance_m = _side_clearance(clear, preferred_side)
        target_reacquire_side_open = bool(
            preferred_side_clearance_m is not None
            and float(preferred_side_clearance_m) >= CRUISE_TARGET_REACQUIRE_SIDE_REQUIRED_M
        )
        target_reacquire_side_clearance_ok = bool(target_reacquire_side_open)
        front_m = clear["front_m"]
        blocked_front = bool(clear["blocked_front"])
        front_gap = dict(raw_context.get("front_gap") or {})
        target_direction_gap = _raw_scan_target_direction_gap(raw_scan, float(target_bearing_rad))
        raw_front_blocked = bool(front_gap.get("front_blocked_by_scan", False))
        target_direction_clearance_m = _finite_float(target_direction_gap.get("clearance_m"))
        target_direction_gate_relevant = bool(distance_m >= CRUISE_TARGET_DIRECTION_GATE_MIN_DISTANCE_M)
        target_direction_scan_blocked = bool(target_direction_gap.get("blocked_by_scan", False))
        target_direction_scan_hard_blocked = bool(target_direction_gap.get("hard_blocked_by_scan", False))
        target_direction_raw_blocked = bool(target_direction_gate_relevant and target_direction_scan_blocked)
        target_direction_hard_blocked = bool(
            target_direction_gate_relevant and target_direction_scan_hard_blocked
        )
        if target_direction_raw_blocked:
            self._target_direction_blocked_latched = True
        elif bool(self._target_direction_blocked_latched):
            release_m = CRUISE_TRIGGER_M + CRUISE_TARGET_DIRECTION_GAP_RELEASE_MARGIN_M
            if bool(target_direction_gap.get("has_data", False)) and target_direction_clearance_m is not None:
                self._target_direction_blocked_latched = bool(float(target_direction_clearance_m) <= release_m)
            else:
                self._target_direction_blocked_latched = False
        target_direction_blocked = bool(self._target_direction_blocked_latched)
        front_gap_direction = str(front_gap.get("best_direction", "") or "").strip().upper()
        front_gap_side = "left" if front_gap_direction == "LEFT" else ("right" if front_gap_direction == "RIGHT" else "")
        front_gap_score_delta = 0.0
        if front_gap_side:
            left_score = _finite_float(front_gap.get("left_open_score")) or 0.0
            right_score = _finite_float(front_gap.get("right_open_score")) or 0.0
            front_gap_score_delta = (
                float(left_score) - float(right_score)
                if front_gap_side == "left"
                else float(right_score) - float(left_score)
            )
        front_gap_confident = bool(front_gap_side and front_gap_score_delta >= CRUISE_FRONT_GAP_CONFIDENT_DELTA)
        effective_front_m = float(front_m) if front_m is not None else (CRUISE_STOP_M if blocked_front else 9.99)
        if camera_front_obstacle_arbitrated and request_front_obstacle_m is not None:
            effective_front_m = min(float(effective_front_m), float(request_front_obstacle_m))
            if front_m is None or float(request_front_obstacle_m) < float(front_m):
                front_m = float(request_front_obstacle_m)
                clear["front_source"] = "follow_front_obstacle_distance"
        if target_direction_blocked and target_direction_clearance_m is not None:
            effective_front_m = min(float(effective_front_m), float(target_direction_clearance_m))
        global_min_m = _finite_float(clear.get("global_min_m"))
        left_for_global_m = _finite_float(clear.get("left_m"))
        right_for_global_m = _finite_float(clear.get("right_m"))
        side_data_for_global = bool(left_for_global_m is not None or right_for_global_m is not None)
        side_close_for_global = bool(
            (left_for_global_m is not None and float(left_for_global_m) <= CRUISE_SIDE_REQUIRED_M)
            or (right_for_global_m is not None and float(right_for_global_m) <= CRUISE_SIDE_REQUIRED_M)
        )
        front_close_for_global = bool(
            float(effective_front_m) <= CRUISE_TRIGGER_M
            or (
                target_direction_clearance_m is not None
                and float(target_direction_clearance_m) <= CRUISE_TRIGGER_M
            )
        )
        global_min_directionally_relevant = bool(
            not camera_target_follow
            or not side_data_for_global
            or front_close_for_global
            or side_close_for_global
            or (global_min_m is not None and float(global_min_m) >= CRUISE_COLLISION_M)
        )
        camera_target_guard_front_m = float(effective_front_m)
        camera_target_guard_front_source = "effective_front"
        if (
            camera_target_follow
            and global_min_directionally_relevant
            and global_min_m is not None
            and float(global_min_m) > 0.0
        ):
            if float(global_min_m) < float(camera_target_guard_front_m):
                camera_target_guard_front_m = float(global_min_m)
                camera_target_guard_front_source = str(clear.get("global_min_source") or "global_min")
        global_clearance_hard_gate = bool(
            camera_clearance_guard_active
            and global_min_directionally_relevant
            and global_min_m is not None
            and float(global_min_m) <= CRUISE_STOP_M
        )
        global_clearance_warning_gate = bool(
            camera_clearance_guard_active
            and global_min_directionally_relevant
            and global_min_m is not None
            and float(global_min_m) <= CRUISE_TRIGGER_M
        )
        hard_obstacle_gate = bool(
            effective_front_m <= CRUISE_STOP_M
            or blocked_front
            or target_direction_hard_blocked
            or global_clearance_hard_gate
        )
        rear_clearance_m = _finite_float(clear.get("rear_m"))
        rear_clear_for_retreat = bool(
            rear_clearance_m is not None
            and float(rear_clearance_m) >= CRUISE_CAMERA_FRONT_HOLD_RETREAT_REAR_MIN_M
            and not bool(clear.get("blocked_back", False))
        )
        global_front_clear_for_retreat = bool(
            not side_close_for_global
            and (
                front_close_for_global
                or (
                    global_min_m is not None
                    and float(global_min_m) <= CRUISE_TRIGGER_M
                    and float(effective_front_m) <= CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M
                )
            )
        )
        global_clear_for_retreat = bool(
            global_min_m is None
            or not global_min_directionally_relevant
            or float(global_min_m) >= CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M
            or global_front_clear_for_retreat
        )
        retreat_clear_for_retreat = bool(rear_clear_for_retreat and global_clear_for_retreat)
        camera_front_hold_align_min_clearance_m = (
            max(
                CRUISE_STOP_M,
                min(
                    CRUISE_CAMERA_FRONT_HOLD_ALIGN_MIN_CLEARANCE_M,
                    desired_distance_m + 0.10,
                ),
            )
            if desired_distance_m > 0.0
            else CRUISE_CAMERA_FRONT_HOLD_ALIGN_MIN_CLEARANCE_M
        )
        front_hold_align_blocked = bool(
            (camera_front_lidar_hold or camera_front_obstacle_arbitrated)
            and effective_front_m <= camera_front_hold_align_min_clearance_m
        )
        camera_front_hold_retreat_front_m = (
            max(
                CRUISE_STOP_M,
                min(
                    CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M,
                    desired_distance_m + (0.18 if desired_distance_m <= 0.70 else 0.10),
                ),
            )
            if desired_distance_m > 0.0
            else CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M
        )
        front_hold_retreat_candidate = bool(
            (camera_front_lidar_hold or camera_front_obstacle_arbitrated)
            and camera_target_guard_front_m <= camera_front_hold_retreat_front_m
            and retreat_clear_for_retreat
        )
        front_hold_retreat_active = bool(
            front_hold_retreat_candidate and (camera_front_lidar_hold or camera_front_obstacle_arbitrated)
        )
        front_hold_retreat_suppressed = bool(front_hold_retreat_candidate and not front_hold_retreat_active)
        camera_target_warning_retreat_front_m = (
            max(
                CRUISE_TARGET_SEARCH_MIN_FRONT_CLEARANCE_M,
                min(
                    CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M,
                    desired_distance_m + (0.18 if desired_distance_m <= 0.70 else 0.10),
                ),
            )
            if desired_distance_m > 0.0
            else CRUISE_TARGET_SEARCH_MIN_FRONT_CLEARANCE_M
        )
        camera_target_warning_retreat_active = bool(
            camera_target_follow
            and not camera_front_lidar_hold
            and camera_target_guard_front_m <= camera_target_warning_retreat_front_m
            and retreat_clear_for_retreat
        )
        obstacle_gate = bool(
            blocked_front
            or raw_front_blocked
            or target_direction_blocked
            or effective_front_m <= CRUISE_TRIGGER_M
            or global_clearance_warning_gate
        )
        camera_target_persisted = bool(target_id == "camera_target_persisted")
        camera_target_reacquire = bool(target_id == "camera_target_reacquire")
        camera_target_persisted = bool(camera_target_persisted or camera_target_reacquire)
        request_confidence = _finite_float(getattr(request, "confidence", None))
        if request_confidence is None:
            request_confidence = 0.0
        camera_target_close_front_forward_margin_m = (
            CRUISE_CAMERA_TARGET_CLOSE_FRONT_MARGIN_NEAR_M
            if desired_distance_m <= 0.70
            else CRUISE_CAMERA_TARGET_CLOSE_FRONT_MARGIN_ROOM_M
        )
        camera_target_close_front_forward_block_m = min(
            CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M,
            desired_distance_m + camera_target_close_front_forward_margin_m,
        )
        camera_target_close_front_forward_blocked = bool(
            camera_target_follow
            and desired_distance_m > 0.0
            and camera_target_guard_front_m <= camera_target_close_front_forward_block_m
        )
        camera_target_close_retreat_active = bool(
            camera_target_follow
            and target_hold_active
            and not target_search_active
            and not camera_target_persisted
            and not camera_front_lidar_hold
            and not camera_front_obstacle_arbitrated
            and desired_distance_m > 0.0
            and actual_target_distance_m <= max(0.0, desired_distance_m - CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MARGIN_M)
            and actual_target_bearing_abs <= CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MAX_BEARING_RAD
            and float(request_confidence) >= CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MIN_CONFIDENCE
            and retreat_clear_for_retreat
            and not obstacle_gate
        )
        target_hold_heading_align = bool(
            camera_target_follow
            and target_hold_active
            and not obstacle_gate
            and not front_hold_align_blocked
            and actual_target_bearing_abs > CRUISE_TARGET_HOLD_HEADING_ALIGN_DEADBAND_RAD
        )
        forward_bias_side, forward_bias_selection, forward_bias_active = self._select_forward_bias_side(
            clear=clear,
            preferred_side=preferred_side,
            bearing_abs=actual_target_bearing_abs,
            target_direction_blocked=target_direction_blocked,
        )

        selected_side, side_selection = self._select_wider_side(
            left_m=clear["left_m"],
            right_m=clear["right_m"],
            preferred_side=preferred_side,
            obstacle_gate=obstacle_gate,
        )
        if bool(obstacle_gate) and bool(front_gap_confident):
            gap_side_clearance = _side_clearance(clear, front_gap_side)
            if gap_side_clearance is None or float(gap_side_clearance) >= CRUISE_SIDE_REQUIRED_M:
                selected_side = str(front_gap_side)
                side_selection = "front_gap"
        if hard_obstacle_gate and self._avoidance_side in {"left", "right"}:
            held_side_clearance_m = _side_clearance(clear, self._avoidance_side)
            if held_side_clearance_m is not None and float(held_side_clearance_m) >= CRUISE_SIDE_REQUIRED_M:
                selected_side = str(self._avoidance_side)
                side_selection = "held_pivot_escape"
        elif obstacle_gate:
            selected_side, side_selection = self._hold_warning_side_if_still_open(
                clear=clear,
                selected_side=selected_side,
                side_selection=side_selection,
            )
        if obstacle_gate:
            if target_heading_side:
                target_side_clearance_m = _side_clearance(clear, target_heading_side)
                if target_side_clearance_m is None or float(target_side_clearance_m) >= CRUISE_SIDE_REQUIRED_M:
                    selected_side = str(target_heading_side)
                    side_selection = "target_angle_reference"
                else:
                    selected_side = ""
                    side_selection = "target_angle_side_blocked"
            else:
                selected_side = ""
                side_selection = "target_angle_centered_forward_hold"

        phase = "target_arc"
        reason = "target_follow_room_cruise_arc"
        obstacle_avoidance_active = False
        camera_motion_requires_detection = bool(
            target_source in {TARGET_SOURCE_CAMERA_TARGET, TARGET_SOURCE_CAMERA_SEARCH}
        )
        camera_motion_detection_allowed = bool(camera_target_follow and not camera_target_persisted)
        camera_detection_motion_suppressed = False
        camera_detection_reacquire_rotate_allowed = False
        camera_detection_clearance_retreat_active = False
        camera_detection_clearance_retreat_front_m = (
            max(
                CRUISE_STOP_M,
                min(
                    CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_FRONT_M,
                    desired_distance_m + 0.08,
                ),
            )
            if desired_distance_m > 0.0
            else CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_FRONT_M
        )
        camera_detection_clearance_m = float(effective_front_m)
        if target_search_active and global_min_m is not None and float(global_min_m) > 0.0:
            camera_detection_clearance_m = min(float(camera_detection_clearance_m), float(global_min_m))
        lidar_confidence = clear.get("lidar_confidence")
        raw_confidence_hold = bool(
            lidar_confidence is not None and float(lidar_confidence) < CRUISE_LIDAR_CONFIDENCE_HOLD
        )
        camera_detection_clearance_retreat_candidate = bool(
            camera_motion_requires_detection
            and not camera_motion_detection_allowed
            and camera_detection_clearance_m <= camera_detection_clearance_retreat_front_m
            and retreat_clear_for_retreat
            and not raw_confidence_hold
        )
        camera_simple_follow_active = bool(
            camera_target_follow
            and not target_search_active
            and not camera_target_persisted
            and desired_distance_m > 0.0
        )
        camera_direction_only_mode = bool(
            camera_target_follow
            and desired_distance_m > CRUISE_CAMERA_SIMPLE_DISTANCE_CONTROL_MAX_DESIRED_M
        )
        camera_simple_distance_control_active = bool(
            camera_simple_follow_active
            and desired_distance_m <= CRUISE_CAMERA_SIMPLE_DISTANCE_CONTROL_MAX_DESIRED_M
        )
        camera_simple_distance_error_m = float(actual_target_distance_m) - float(desired_distance_m)
        camera_simple_forward_min_front_clearance_m = max(
            CRUISE_CAMERA_SIMPLE_FORWARD_MIN_FRONT_CLEARANCE_M,
            float(desired_distance_m) - CRUISE_CAMERA_SIMPLE_FORWARD_DESIRED_CLEARANCE_MARGIN_M,
        )
        camera_simple_forward_clearance_blocked = bool(
            camera_simple_distance_control_active
            and camera_target_guard_front_m <= camera_simple_forward_min_front_clearance_m
        )
        camera_simple_forward_gate_blocked = bool(
            obstacle_gate
            or hard_obstacle_gate
            or effective_front_m <= CRUISE_TRIGGER_M
            or camera_target_close_front_forward_blocked
            or camera_simple_forward_clearance_blocked
        )
        camera_simple_retreat_gate_blocked = bool(not retreat_clear_for_retreat)
        camera_simple_close_retreat_candidate = bool(
            camera_simple_distance_control_active
            and camera_target_follow
            and not target_search_active
            and not camera_target_persisted
            and desired_distance_m > 0.0
            and camera_simple_distance_error_m < -CRUISE_CAMERA_SIMPLE_DISTANCE_DEADBAND_M
            and retreat_clear_for_retreat
            and not hard_obstacle_gate
        )
        camera_low_lidar_bypass = bool(
            raw_confidence_hold
            and camera_simple_follow_active
            and not camera_simple_forward_gate_blocked
            and float(request_confidence) >= CRUISE_CAMERA_LOW_LIDAR_BYPASS_CONFIDENCE
            and camera_target_guard_front_m
            > max(CRUISE_TRIGGER_M, float(desired_distance_m) + CRUISE_CAMERA_LOW_LIDAR_BYPASS_FRONT_MARGIN_M)
        )
        confidence_hold = bool(raw_confidence_hold and not camera_low_lidar_bypass)
        follow_state = _follow_state_for_decision(
            confidence_hold=confidence_hold,
            target_hold_active=target_hold_active,
            distance_m=distance_m,
            target_slow_distance_m=target_slow_distance_m,
            target_rearward=target_rearward,
            target_outside_forward_arc=target_outside_forward_arc,
            obstacle_gate=obstacle_gate,
        )
        if target_search_active:
            follow_state = "search"
        camera_simple_forward_track_mps = 0.0
        camera_simple_turn_track_mps = 0.0
        camera_simple_turn_cap_mps = 0.0
        camera_simple_forward_cap_mps = 0.0
        camera_simple_forward_crawl_mps = 0.0
        camera_simple_warning_pivot_active = False
        camera_simple_heading_forward_scale = 1.0
        camera_target_turn_cap_mps = min(
            float(limits["max_track_mps"]),
            max(
                float(limits["target_turn_diff_mps"]),
                float(limits.get("omega_limit_track_delta_mps", limits["target_turn_diff_mps"]))
                * CRUISE_CAMERA_SIMPLE_TURN_OMEGA_LIMIT_RATIO,
            ),
        )
        camera_simple_turn_cap_mps = float(camera_target_turn_cap_mps)
        target_reacquire_clearance_m = float(effective_front_m)
        if global_min_m is not None and float(global_min_m) > 0.0:
            target_reacquire_clearance_m = min(float(target_reacquire_clearance_m), float(global_min_m))
        if confidence_hold and not bool(
            front_hold_retreat_active
            or camera_target_warning_retreat_active
            or camera_detection_clearance_retreat_candidate
            or camera_simple_close_retreat_candidate
        ):
            left_mps = 0.0
            right_mps = 0.0
            phase = "lidar_confidence_hold"
            reason = "lidar_confidence_below_cruise_hold"
            obstacle_avoidance_active = False
        elif effective_front_m <= CRUISE_COLLISION_M:
            left_mps = 0.0
            right_mps = 0.0
            phase = "collision_stop"
            reason = "front_inside_absolute_collision_floor"
            obstacle_avoidance_active = True
        elif effective_front_m <= CRUISE_PIVOT_FLOOR_M:
            obstacle_avoidance_active = True
            if selected_side:
                escape_track_mps = min(
                    float(limits["pivot_track_mps"]),
                    CRUISE_NEAR_COLLISION_PIVOT_TRACK_MPS,
                    float(limits["max_track_mps"]),
                )
                left_mps, right_mps = _in_place_pivot_tracks(
                    selected_side,
                    escape_track_mps,
                    limits["max_track_mps"],
                )
                phase = "near_collision_target_angle_pivot"
                reason = "front_inside_pivot_floor_target_angle_reference"
            else:
                left_mps = 0.0
                right_mps = 0.0
                phase = "collision_stop"
                reason = "front_inside_pivot_floor_forward_gate_hold"
        elif front_hold_retreat_active:
            retreat_track_mps = _camera_retreat_track_mps(
                limits,
                effective_front_m=camera_target_guard_front_m,
                desired_distance_m=desired_distance_m,
                base_track_mps=CRUISE_CAMERA_FRONT_HOLD_RETREAT_TRACK_MPS,
            )
            left_mps, right_mps, retreat_turn_track_mps = _target_aligned_retreat_tracks(
                retreat_track_mps=retreat_track_mps,
                target_bearing_rad=target_bearing_rad,
                turn_cap_mps=camera_target_turn_cap_mps,
                max_track_mps=float(limits["max_track_mps"]),
            )
            selected_side = ""
            side_selection = "front_hold_camera_retreat"
            if camera_target_turn_side_blocked:
                left_mps = -float(retreat_track_mps)
                right_mps = -float(retreat_track_mps)
                retreat_turn_track_mps = 0.0
                side_selection = "front_hold_camera_retreat_turn_side_blocked"
            phase = "front_hold_camera_retreat"
            reason = "camera_front_too_close_restore_follow_bubble"
            obstacle_avoidance_active = False
            camera_simple_turn_track_mps = float(retreat_turn_track_mps)
        elif camera_detection_clearance_retreat_candidate:
            retreat_track_mps = _camera_retreat_track_mps(
                limits,
                effective_front_m=effective_front_m,
                desired_distance_m=desired_distance_m,
                base_track_mps=CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_TRACK_MPS,
            )
            left_mps = -float(retreat_track_mps)
            right_mps = -float(retreat_track_mps)
            selected_side = ""
            side_selection = "camera_detection_clearance_retreat"
            phase = "camera_detection_clearance_retreat"
            reason = "front_clearance_restore_without_forward"
            obstacle_avoidance_active = False
            camera_detection_clearance_retreat_active = True
        elif camera_target_warning_retreat_active:
            retreat_track_mps = _camera_retreat_track_mps(
                limits,
                effective_front_m=camera_target_guard_front_m,
                desired_distance_m=desired_distance_m,
                base_track_mps=CRUISE_CAMERA_FRONT_HOLD_RETREAT_TRACK_MPS,
            )
            left_mps, right_mps, retreat_turn_track_mps = _target_aligned_retreat_tracks(
                retreat_track_mps=retreat_track_mps,
                target_bearing_rad=target_bearing_rad,
                turn_cap_mps=camera_target_turn_cap_mps,
                max_track_mps=float(limits["max_track_mps"]),
            )
            selected_side = ""
            side_selection = "camera_follow_front_warning_retreat"
            if camera_target_turn_side_blocked:
                left_mps = -float(retreat_track_mps)
                right_mps = -float(retreat_track_mps)
                retreat_turn_track_mps = 0.0
                side_selection = "camera_follow_front_warning_retreat_turn_side_blocked"
            phase = "front_warning_camera_retreat"
            reason = "front_warning_camera_retreat_restore_clearance"
            obstacle_avoidance_active = False
            camera_simple_turn_track_mps = float(retreat_turn_track_mps)
        elif (
            camera_target_reacquire
            and not hard_obstacle_gate
            and target_reacquire_clearance_m > CRUISE_TARGET_SEARCH_MIN_FRONT_CLEARANCE_M
            and actual_target_bearing_abs <= CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD
        ):
            left_mps = 0.0
            right_mps = 0.0
            selected_side = ""
            side_selection = "target_reacquire_center_hold"
            phase = "target_reacquire_hold"
            reason = "target_short_loss_reacquire_centered_hold"
            obstacle_avoidance_active = False
        elif (
            camera_target_reacquire
            and not hard_obstacle_gate
            and target_reacquire_clearance_m > CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M
        ):
            reacquire_track_mps = min(
                float(limits["max_track_mps"]),
                max(0.006, _scaled_speed(limits, CRUISE_TARGET_REACQUIRE_ROTATE_TRACK_MPS)),
            )
            selected_side = (
                camera_target_turn_side
                if camera_target_turn_side in {"left", "right"}
                else (preferred_side if preferred_side in {"left", "right"} else "left")
            )
            side_selection = "target_reacquire_scan"
            if camera_direction_only_mode:
                left_mps, right_mps = _in_place_pivot_tracks(
                    selected_side,
                    reacquire_track_mps,
                    limits["max_track_mps"],
                )
                phase = "target_reacquire_in_place"
                reason = "target_short_loss_reacquire_in_place_scan"
            else:
                left_mps, right_mps = _camera_one_track_direction_tracks(
                    selected_side,
                    reacquire_track_mps,
                    limits["max_track_mps"],
                )
                phase = "target_reacquire_rotate"
                reason = "target_short_loss_reacquire_scan"
            obstacle_avoidance_active = False
            camera_detection_reacquire_rotate_allowed = True
        elif camera_target_reacquire:
            left_mps = 0.0
            right_mps = 0.0
            selected_side = ""
            side_selection = "target_reacquire_clearance_hold"
            phase = "target_reacquire_hold"
            reason = "target_short_loss_reacquire_blocked_by_clearance"
            obstacle_avoidance_active = False
        elif (
            camera_target_persisted
            and not camera_target_reacquire
            and not hard_obstacle_gate
            and target_reacquire_clearance_m > CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M
            and actual_target_bearing_abs > CRUISE_CAMERA_PERSISTED_REACQUIRE_MIN_BEARING_RAD
            and not camera_target_turn_side_blocked
        ):
            reacquire_track_mps = min(
                float(limits["max_track_mps"]),
                max(0.006, _scaled_speed(limits, CRUISE_TARGET_REACQUIRE_ROTATE_TRACK_MPS)),
            )
            selected_side = (
                camera_target_turn_side
                if camera_target_turn_side in {"left", "right"}
                else (preferred_side if preferred_side in {"left", "right"} else "left")
            )
            side_selection = "camera_target_persisted_reacquire_scan"
            left_mps, right_mps = _in_place_pivot_tracks(
                selected_side,
                reacquire_track_mps,
                limits["max_track_mps"],
            )
            phase = "target_reacquire_in_place"
            reason = "camera_target_persisted_cautious_reacquire_scan"
            obstacle_avoidance_active = False
            camera_detection_reacquire_rotate_allowed = True
        elif target_search_active:
            obstacle_avoidance_active = False
            target_search_clearance_m = float(effective_front_m)
            if global_min_m is not None and float(global_min_m) > 0.0:
                target_search_clearance_m = min(float(target_search_clearance_m), float(global_min_m))
            if hard_obstacle_gate or target_search_clearance_m <= CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M:
                left_mps = 0.0
                right_mps = 0.0
                selected_side = ""
                side_selection = "target_search_blocked"
                phase = "target_search_hold"
                reason = "target_search_blocked_by_front_clearance"
            else:
                selected_side = str(target_search_side)
                side_selection = "target_search_last_seen_side"
                search_track_mps = min(
                    float(limits["max_track_mps"]),
                    max(0.006, _scaled_speed(limits, CRUISE_TARGET_SEARCH_ROTATE_TRACK_MPS)),
                )
                left_mps, right_mps = _camera_one_track_direction_tracks(
                    selected_side,
                    search_track_mps,
                    limits["max_track_mps"],
                )
                phase = "target_search_one_track"
                reason = "target_lost_search_last_seen_side"
                if target_search_direction_only:
                    left_mps, right_mps = _in_place_pivot_tracks(
                        selected_side,
                        search_track_mps,
                        limits["max_track_mps"],
                    )
                    phase = "target_search_in_place"
                    reason = "target_lost_search_last_seen_side_in_place"
        elif camera_simple_follow_active and not hard_obstacle_gate:
            obstacle_avoidance_active = False
            max_track = float(limits["max_track_mps"])
            turn_track_mps = 0.0
            turn_cap_mps = float(camera_target_turn_cap_mps)
            forward_track_mps = 0.0
            forward_crawl_mps = 0.0
            forward_cap_mps = 0.0
            selected_side = ""
            side_selection = "camera_target_center_zone"
            left_mps = 0.0
            right_mps = 0.0
            if camera_simple_close_retreat_candidate:
                retreat_track_mps = _camera_retreat_track_mps(
                    limits,
                    effective_front_m=camera_target_guard_front_m,
                    desired_distance_m=desired_distance_m,
                    base_track_mps=CRUISE_CAMERA_SIMPLE_RETREAT_TRACK_MPS,
                )
                left_mps, right_mps, retreat_turn_track_mps = _target_aligned_retreat_tracks(
                    retreat_track_mps=retreat_track_mps,
                    target_bearing_rad=target_bearing_rad,
                    turn_cap_mps=turn_cap_mps,
                    max_track_mps=max_track,
                )
                selected_side = ""
                side_selection = "camera_simple_close_retreat"
                if camera_target_turn_side_blocked:
                    left_mps = -float(retreat_track_mps)
                    right_mps = -float(retreat_track_mps)
                    retreat_turn_track_mps = 0.0
                    side_selection = "camera_simple_close_retreat_turn_side_blocked"
                phase = "camera_target_close_retreat"
                reason = "camera_target_too_close_restore_follow_distance"
                turn_track_mps = float(retreat_turn_track_mps)
            elif not camera_target_align_active:
                distance_error_for_forward_m = float(camera_simple_distance_error_m)
                if (
                    camera_simple_distance_control_active
                    and
                    distance_error_for_forward_m > CRUISE_CAMERA_SIMPLE_DISTANCE_DEADBAND_M
                    and not camera_simple_forward_gate_blocked
                ):
                    forward_crawl_mps = min(
                        max_track,
                        _scaled_speed(limits, CRUISE_CAMERA_SIMPLE_FORWARD_CRAWL_TRACK_MPS),
                    )
                    forward_cap_mps = min(
                        max_track,
                        _scaled_speed(limits, CRUISE_CAMERA_SIMPLE_FORWARD_FLOOR_MAX_TRACK_MPS),
                    )
                    slow_ratio = _clamp(
                        (
                            distance_error_for_forward_m
                            - CRUISE_CAMERA_SIMPLE_DISTANCE_DEADBAND_M
                        )
                        / max(
                            0.01,
                            CRUISE_CAMERA_SIMPLE_DISTANCE_SLOW_M
                            - CRUISE_CAMERA_SIMPLE_DISTANCE_DEADBAND_M,
                        ),
                        0.0,
                        1.0,
                    )
                    forward_track_mps = forward_crawl_mps + slow_ratio * (
                        forward_cap_mps - forward_crawl_mps
                    )
                    left_mps = float(forward_track_mps)
                    right_mps = float(forward_track_mps)
                    phase = "camera_target_center_forward"
                    reason = "camera_target_center_distance_follow"
                else:
                    phase = "camera_target_center_hold"
                    reason = "camera_target_in_center_third_hold"
            elif camera_target_turn_side_blocked:
                side_selection = "camera_target_angle_side_blocked"
                phase = "camera_target_turn_side_hold"
                reason = "camera_target_turn_side_blocked_by_lidar"
            else:
                selected_side = (
                    str(target_zone)
                    if target_zone in {"left", "right"}
                    else ("left" if float(target_bearing_rad) >= 0.0 else "right")
                )
                side_selection = f"camera_target_one_track_{selected_side}"
                turn_track_mps = float(_camera_target_turn_track_mps(limits, actual_target_bearing_abs))
                if target_zone in {"left", "right"} and turn_track_mps <= 0.0:
                    turn_track_mps = min(
                        max_track,
                        max(0.006, _scaled_speed(limits, CRUISE_CAMERA_TARGET_TURN_MIN_TRACK_MPS)),
                    )
                if obstacle_gate:
                    left_mps, right_mps = _in_place_pivot_tracks(selected_side, turn_track_mps, max_track)
                    phase = "camera_target_pivot_align"
                    reason = "front_warning_camera_target_in_place_align"
                    camera_simple_warning_pivot_active = True
                elif camera_direction_only_mode:
                    left_mps, right_mps = _in_place_pivot_tracks(selected_side, turn_track_mps, max_track)
                    phase = "camera_target_in_place_align"
                    reason = "camera_target_direction_only_in_place_align"
                else:
                    left_mps, right_mps = _camera_one_track_direction_tracks(selected_side, turn_track_mps, max_track)
                    phase = "camera_target_one_track_align"
                    reason = "camera_target_outside_center_third_one_track_align"
            camera_simple_forward_track_mps = float(forward_track_mps)
            camera_simple_turn_track_mps = float(turn_track_mps)
            camera_simple_turn_cap_mps = float(turn_cap_mps)
            camera_simple_forward_cap_mps = float(forward_cap_mps)
            camera_simple_forward_crawl_mps = float(forward_crawl_mps)
        elif target_hold_active:
            if camera_target_close_retreat_active:
                retreat_track_mps = _camera_retreat_track_mps(
                    limits,
                    effective_front_m=effective_front_m,
                    desired_distance_m=desired_distance_m,
                    base_track_mps=CRUISE_CAMERA_FRONT_HOLD_RETREAT_TRACK_MPS,
                )
                left_mps, right_mps, retreat_turn_track_mps = _target_aligned_retreat_tracks(
                    retreat_track_mps=retreat_track_mps,
                    target_bearing_rad=target_bearing_rad,
                    turn_cap_mps=camera_target_turn_cap_mps,
                    max_track_mps=float(limits["max_track_mps"]),
                )
                selected_side = ""
                side_selection = "camera_target_close_retreat"
                phase = "target_close_retreat"
                reason = "camera_target_too_close_restore_follow_distance"
                camera_simple_turn_track_mps = float(retreat_turn_track_mps)
            elif target_hold_heading_align:
                selected_side = "left" if float(actual_target["bearing_error_rad"]) >= 0.0 else "right"
                side_selection = "target_hold_heading"
                if camera_target_turn_side_blocked:
                    selected_side = ""
                    side_selection = "target_hold_heading_side_blocked"
                    left_mps = 0.0
                    right_mps = 0.0
                    phase = "target_hold_heading_blocked"
                    reason = "target_heading_side_blocked_by_lidar"
                else:
                    align_track_mps = min(
                        float(limits["target_pivot_track_mps"]),
                        float(limits["max_track_mps"]),
                        _scaled_speed(limits, CRUISE_TARGET_HOLD_HEADING_ALIGN_TRACK_MPS),
                    )
                    left_mps, right_mps = _in_place_pivot_tracks(
                        selected_side,
                        align_track_mps,
                        limits["max_track_mps"],
                    )
                    phase = "target_hold_heading_align"
                    reason = "follow_goal_reached_heading_align"
            else:
                left_mps = 0.0
                right_mps = 0.0
                phase = "target_hold"
                reason = "follow_goal_reached_stop_radius"
            obstacle_avoidance_active = False
        elif hard_obstacle_gate:
            obstacle_avoidance_active = True
            if selected_side:
                arc_track_mps = min(float(limits["pivot_track_mps"]), float(limits["max_track_mps"]))
                left_mps, right_mps = _in_place_pivot_tracks(
                    selected_side,
                    arc_track_mps,
                    limits["max_track_mps"],
                )
                phase = "obstacle_target_angle_pivot"
                reason = "front_stop_target_angle_reference"
            else:
                left_mps = 0.0
                right_mps = 0.0
                phase = "obstacle_stop_hold"
                reason = "front_stop_forward_gate_hold"
        elif obstacle_gate:
            obstacle_avoidance_active = True
            if camera_target_follow and not target_rearward and not target_outside_forward_arc:
                left_mps = 0.0
                right_mps = 0.0
                selected_side = ""
                side_selection = "camera_follow_front_hold"
                phase = "front_warning_follow_hold"
                reason = "front_warning_hold_for_camera_follow"
            elif selected_side:
                pivot_track_mps = min(float(limits["target_pivot_track_mps"]), float(limits["max_track_mps"]))
                left_mps, right_mps = _in_place_pivot_tracks(selected_side, pivot_track_mps, limits["max_track_mps"])
                phase = "obstacle_target_angle_pivot"
                reason = "front_warning_target_angle_reference"
            else:
                left_mps = 0.0
                right_mps = 0.0
                phase = "obstacle_stop_hold"
                reason = "front_warning_forward_gate_hold"
        else:
            base = min(
                limits["max_track_mps"],
                _scaled_speed(
                    limits,
                    CRUISE_CLEAR_FORWARD_TRACK_MPS if distance_m > 0.35 else CRUISE_FORWARD_TRACK_MPS,
                ),
            )
            if distance_m <= target_slow_distance_m:
                slow_ratio = _clamp(
                    (distance_m - target_stop_distance_m) / max(0.01, target_slow_distance_m - target_stop_distance_m),
                    0.0,
                    1.0,
                )
                base = min(
                    base,
                    _scaled_speed(limits, CRUISE_TARGET_CRAWL_TRACK_MPS)
                    + slow_ratio
                    * (
                        _scaled_speed(limits, CRUISE_FORWARD_TRACK_MPS)
                        - _scaled_speed(limits, CRUISE_TARGET_CRAWL_TRACK_MPS)
                    ),
                )
                reason = "target_follow_room_cruise_arc_slowdown"
            if target_hold_creep:
                base = max(
                    base,
                    min(
                        limits["max_track_mps"],
                        _scaled_speed(limits, CRUISE_MOVING_TARGET_HOLD_CREEP_TRACK_MPS),
                    ),
                )
                selected_side = ""
                side_selection = "hold_creep_straight"
                left_mps = base
                right_mps = base
                phase = "target_hold_creep"
                reason = "moving_target_inside_standoff_forward_creep"
            elif target_bubble_drift:
                base = max(
                    base,
                    min(
                        limits["max_track_mps"],
                        _scaled_speed(limits, CRUISE_MOVING_TARGET_BUBBLE_TRACK_MPS),
                    ),
                )
                phase = "target_bubble_drift"
                reason = "moving_target_inside_goal_bubble_low_speed_track"
            if target_hold_creep:
                pass
            elif target_outside_forward_arc and not target_bubble_drift:
                selected_side = preferred_side
                side_selection = "target_heading"
                left_mps, right_mps = _sharp_heading_arc_tracks(selected_side, limits)
                phase = "target_heading_arc"
                reason = "target_outside_forward_arc_one_track_arc"
            else:
                selected_side = forward_bias_side or preferred_side
                side_selection = forward_bias_selection or "target_heading"
                diff = _clamp(
                    max(0.0, actual_target_bearing_abs / 0.75) * limits["target_turn_diff_mps"],
                    0.0,
                    limits["target_turn_diff_mps"],
                )
                bubble_drift_straight = bool(
                    target_bubble_drift and actual_target_bearing_abs <= CRUISE_BUBBLE_DRIFT_HEADING_DEADBAND_RAD
                )
                if bubble_drift_straight or actual_target_bearing_abs <= CRUISE_TARGET_HEADING_DEADBAND_RAD:
                    diff = 0.0
                elif str(selected_side) != str(preferred_side) and bool(forward_bias_active):
                    diff *= CRUISE_FORWARD_HEADING_OPPOSITE_TURN_GAIN
                left_mps, right_mps = _arc_tracks(selected_side, base, diff, limits["max_track_mps"])
                if bubble_drift_straight:
                    selected_side = ""
                    side_selection = "bubble_drift_straight"

        if (
            camera_motion_requires_detection
            and not camera_motion_detection_allowed
            and (abs(float(left_mps)) > 1e-6 or abs(float(right_mps)) > 1e-6)
            and not camera_detection_clearance_retreat_active
            and not bool(phase in {"target_search_rotate_360", "target_search_one_track", "target_search_in_place", "target_reacquire_rotate", "target_reacquire_in_place"})
        ):
            left_mps = 0.0
            right_mps = 0.0
            selected_side = ""
            side_selection = "camera_detection_required"
            phase = "camera_detection_required_hold"
            reason = "camera_target_detection_required_for_motion"
            obstacle_avoidance_active = False
            camera_detection_motion_suppressed = True
        elif camera_motion_requires_detection and not camera_motion_detection_allowed and phase in {
            "target_search_rotate_360",
            "target_search_one_track",
            "target_search_in_place",
            "target_reacquire_rotate",
            "target_reacquire_in_place",
        }:
            camera_detection_reacquire_rotate_allowed = True

        if obstacle_avoidance_active and selected_side:
            self._avoidance_side = str(selected_side)
        elif not obstacle_avoidance_active:
            self._avoidance_side = ""
        if obstacle_avoidance_active or target_rearward or target_outside_forward_arc:
            self._forward_bias_side = ""
        if phase in {"target_hold", "lidar_confidence_hold"}:
            selected_side = ""
            side_selection = "hold"

        v_target, omega_target = _tracks_to_twist(left_mps, right_mps, width)
        clearance_details = {
            "global_min_clearance_m": None if clear["global_min_m"] is None else round(float(clear["global_min_m"]), 4),
            "global_min_clearance_source": str(clear["global_min_source"] or ""),
            "front_clearance_m": None if front_m is None else round(float(front_m), 4),
            "front_clearance_source": str(clear["front_source"] or ""),
            "left_clearance_m": None if clear["left_m"] is None else round(float(clear["left_m"]), 4),
            "left_clearance_source": str(clear["left_source"] or ""),
            "right_clearance_m": None if clear["right_m"] is None else round(float(clear["right_m"]), 4),
            "right_clearance_source": str(clear["right_source"] or ""),
            "rear_clearance_m": None if clear["rear_m"] is None else round(float(clear["rear_m"]), 4),
            "rear_clearance_source": str(clear["rear_source"] or ""),
            "blocked_back": bool(clear.get("blocked_back", False)),
            "blocked_front": bool(blocked_front),
            "trigger_m": float(CRUISE_TRIGGER_M),
            "stop_m": float(CRUISE_STOP_M),
            "collision_m": float(CRUISE_COLLISION_M),
            "pivot_floor_m": float(CRUISE_PIVOT_FLOOR_M),
            "side_required_m": float(CRUISE_SIDE_REQUIRED_M),
            "lidar_confidence": None if lidar_confidence is None else round(float(lidar_confidence), 4),
            "lidar_confidence_hold": float(CRUISE_LIDAR_CONFIDENCE_HOLD),
            "raw_scan": {
                "scan_points": int(raw_context.get("scan_points", 0)),
                "left_clearance_m": (
                    None
                    if not bool((raw_context.get("left") or {}).get("has_data", False))
                    else round(float((raw_context.get("left") or {}).get("clearance_m")), 4)
                ),
                "right_clearance_m": (
                    None
                    if not bool((raw_context.get("right") or {}).get("has_data", False))
                    else round(float((raw_context.get("right") or {}).get("clearance_m")), 4)
                ),
                "front_gap": dict(front_gap),
                "front_gap_side": str(front_gap_side),
                "front_gap_score_delta": round(float(front_gap_score_delta), 4),
                "front_gap_confident": bool(front_gap_confident),
                "front_blocked_by_scan": bool(raw_front_blocked),
                "target_direction_gap": dict(target_direction_gap),
                "target_direction_gate_relevant": bool(target_direction_gate_relevant),
                "target_direction_scan_blocked": bool(target_direction_scan_blocked),
                "target_direction_scan_hard_blocked": bool(target_direction_scan_hard_blocked),
                "target_direction_raw_blocked": bool(target_direction_raw_blocked),
                "target_direction_latched": bool(target_direction_blocked),
                "target_direction_release_margin_m": float(CRUISE_TARGET_DIRECTION_GAP_RELEASE_MARGIN_M),
                "target_direction_gate_min_distance_m": float(CRUISE_TARGET_DIRECTION_GATE_MIN_DISTANCE_M),
            },
        }
        obstacle_details = {
            "active": bool(obstacle_avoidance_active),
            "side": str(selected_side or ""),
            "side_selection": str(side_selection or ""),
            "reason": str(reason),
            "front_clearance_m": clearance_details["front_clearance_m"],
            "left_clearance_m": clearance_details["left_clearance_m"],
            "right_clearance_m": clearance_details["right_clearance_m"],
            "front_gap_side": str(front_gap_side),
            "front_gap_confident": bool(front_gap_confident),
            "target_direction_blocked": bool(target_direction_blocked),
            "target_direction_clearance_m": (
                None if target_direction_clearance_m is None else round(float(target_direction_clearance_m), 4)
            ),
            "camera_target_turn_side": str(camera_target_turn_side or ""),
            "camera_target_turn_side_clearance_m": (
                None
                if camera_target_turn_side_clearance_m is None
                else round(float(camera_target_turn_side_clearance_m), 4)
            ),
            "camera_target_turn_side_blocked": bool(camera_target_turn_side_blocked),
        }
        room_cruise = {
            "active": True,
            "chain": ROOM_CRUISE_VALIDATED_CHAIN,
            "motion_style": ROOM_CRUISE_FOLLOW_STYLE,
            "follow_state": str(follow_state),
            "phase": str(phase),
            "reason": str(reason),
            "selected_side": str(selected_side or ""),
            "side_selection": str(side_selection or ""),
            "obstacle_avoidance": dict(obstacle_details),
            "clearance": dict(clearance_details),
            "target_geometry": {
                "distance_m": round(float(geom["distance_m"]), 4),
                "bearing_error_rad": round(float(geom["bearing_error_rad"]), 4),
                "robot_frame_x_m": round(float(geom["robot_frame_x_m"]), 4),
                "robot_frame_y_m": round(float(geom["robot_frame_y_m"]), 4),
            },
            "follow_gate": {
                "desired_distance_m": round(float(desired_distance_m), 4),
                "target_stop_distance_m": round(float(target_stop_distance_m), 4),
                "target_hold_release_m": round(float(target_hold_release_m), 4),
                "target_hold_release_margin_m": float(CRUISE_TARGET_HOLD_RELEASE_MARGIN_M),
                "target_hold_latched": bool(target_hold_active),
                "target_hold_heading_align": bool(target_hold_heading_align),
                "target_hold_heading_align_deadband_rad": float(CRUISE_TARGET_HOLD_HEADING_ALIGN_DEADBAND_RAD),
                "target_hold_heading_align_track_mps": float(CRUISE_TARGET_HOLD_HEADING_ALIGN_TRACK_MPS),
                "camera_front_lidar_hold": bool(camera_front_lidar_hold),
                "camera_front_lidar_hold_human_confirmed": bool(camera_front_lidar_hold_human_confirmed),
                "camera_front_obstacle_arbitrated": bool(camera_front_obstacle_arbitrated),
                "request_front_obstacle_m": None if request_front_obstacle_m is None else round(float(request_front_obstacle_m), 4),
                "front_hold_align_blocked": bool(front_hold_align_blocked),
                "front_hold_align_min_clearance_m": round(float(camera_front_hold_align_min_clearance_m), 4),
                "front_hold_retreat_active": bool(front_hold_retreat_active),
                "front_hold_retreat_suppressed": bool(front_hold_retreat_suppressed),
                "front_hold_retreat_front_m": round(float(camera_front_hold_retreat_front_m), 4),
                "front_hold_retreat_front_default_m": float(CRUISE_CAMERA_FRONT_HOLD_RETREAT_FRONT_M),
                "front_hold_retreat_rear_min_m": float(CRUISE_CAMERA_FRONT_HOLD_RETREAT_REAR_MIN_M),
                "front_hold_retreat_track_mps": float(CRUISE_CAMERA_FRONT_HOLD_RETREAT_TRACK_MPS),
                "rear_clear_for_retreat": bool(rear_clear_for_retreat),
                "global_clear_for_retreat": bool(global_clear_for_retreat),
                "retreat_clear_for_retreat": bool(retreat_clear_for_retreat),
                "camera_target_guard_front_m": round(float(camera_target_guard_front_m), 4),
                "camera_target_guard_front_source": str(camera_target_guard_front_source),
                "global_min_directionally_relevant": bool(global_min_directionally_relevant),
                "global_clearance_hard_gate": bool(global_clearance_hard_gate),
                "global_clearance_warning_gate": bool(global_clearance_warning_gate),
                "front_close_for_global": bool(front_close_for_global),
                "side_close_for_global": bool(side_close_for_global),
                "global_front_clear_for_retreat": bool(global_front_clear_for_retreat),
                "camera_target_warning_retreat_active": bool(camera_target_warning_retreat_active),
                "camera_target_warning_retreat_front_m": round(float(camera_target_warning_retreat_front_m), 4),
                "camera_target_close_retreat_active": bool(camera_target_close_retreat_active),
                "camera_target_close_retreat_margin_m": float(CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MARGIN_M),
                "camera_target_close_retreat_max_bearing_rad": float(
                    CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MAX_BEARING_RAD
                ),
                "camera_target_close_retreat_min_confidence": float(
                    CRUISE_CAMERA_TARGET_CLOSE_RETREAT_MIN_CONFIDENCE
                ),
                "camera_target_persisted": bool(camera_target_persisted),
                "camera_target_reacquire": bool(camera_target_reacquire),
                "target_reacquire_clearance_m": round(float(target_reacquire_clearance_m), 4),
                "camera_motion_requires_detection": bool(camera_motion_requires_detection),
                "camera_motion_detection_allowed": bool(camera_motion_detection_allowed),
                "camera_detection_motion_suppressed": bool(camera_detection_motion_suppressed),
                "camera_detection_reacquire_rotate_allowed": bool(camera_detection_reacquire_rotate_allowed),
                "camera_detection_clearance_retreat_active": bool(camera_detection_clearance_retreat_active),
                "camera_detection_clearance_retreat_candidate": bool(camera_detection_clearance_retreat_candidate),
                "camera_detection_clearance_m": round(float(camera_detection_clearance_m), 4),
                "camera_detection_clearance_retreat_front_m": round(
                    float(camera_detection_clearance_retreat_front_m),
                    4,
                ),
                "camera_detection_clearance_retreat_track_mps": float(
                    CRUISE_CAMERA_DETECTION_CLEARANCE_RETREAT_TRACK_MPS
                ),
                "camera_simple_follow_active": bool(camera_simple_follow_active),
                "camera_simple_distance_control_active": bool(camera_simple_distance_control_active),
                "camera_simple_distance_error_m": round(float(camera_simple_distance_error_m), 4),
                "camera_simple_forward_gate_blocked": bool(camera_simple_forward_gate_blocked),
                "camera_simple_forward_clearance_blocked": bool(camera_simple_forward_clearance_blocked),
                "camera_simple_warning_pivot_active": bool(camera_simple_warning_pivot_active),
                "camera_simple_forward_min_front_clearance_m": round(
                    float(camera_simple_forward_min_front_clearance_m),
                    4,
                ),
                "camera_simple_retreat_gate_blocked": bool(camera_simple_retreat_gate_blocked),
                "camera_simple_close_retreat_candidate": bool(camera_simple_close_retreat_candidate),
                "camera_simple_distance_control_max_desired_m": float(
                    CRUISE_CAMERA_SIMPLE_DISTANCE_CONTROL_MAX_DESIRED_M
                ),
                "camera_simple_forward_track_mps": round(float(camera_simple_forward_track_mps), 5),
                "camera_simple_turn_track_mps": round(float(camera_simple_turn_track_mps), 5),
                "camera_simple_turn_cap_mps": round(float(camera_simple_turn_cap_mps), 5),
                "camera_simple_forward_cap_mps": round(float(camera_simple_forward_cap_mps), 5),
                "camera_simple_forward_crawl_mps": round(float(camera_simple_forward_crawl_mps), 5),
                "camera_simple_heading_forward_scale": round(float(camera_simple_heading_forward_scale), 4),
                "camera_low_lidar_bypass": bool(camera_low_lidar_bypass),
                "camera_low_lidar_bypass_confidence_min": float(CRUISE_CAMERA_LOW_LIDAR_BYPASS_CONFIDENCE),
                "camera_low_lidar_bypass_front_margin_m": float(CRUISE_CAMERA_LOW_LIDAR_BYPASS_FRONT_MARGIN_M),
                "raw_lidar_confidence_hold": bool(raw_confidence_hold),
                "effective_lidar_confidence_hold": bool(confidence_hold),
                "camera_simple_heading_deadband_rad": float(CRUISE_CAMERA_SIMPLE_HEADING_DEADBAND_RAD),
                "camera_target_center_zone_rad": float(CRUISE_CAMERA_TARGET_CENTER_ZONE_RAD),
                "camera_target_align_release_rad": float(CRUISE_CAMERA_TARGET_ALIGN_RELEASE_RAD),
                "camera_target_persisted_reacquire_min_bearing_rad": float(
                    CRUISE_CAMERA_PERSISTED_REACQUIRE_MIN_BEARING_RAD
                ),
                "camera_target_align_latched": bool(camera_target_align_active),
                "camera_target_one_track_mps": float(CRUISE_CAMERA_TARGET_ONE_TRACK_MPS),
                "camera_target_turn_min_track_mps": float(CRUISE_CAMERA_TARGET_TURN_MIN_TRACK_MPS),
                "camera_simple_forward_max_bearing_rad": float(CRUISE_CAMERA_SIMPLE_FORWARD_MAX_BEARING_RAD),
                "camera_simple_forward_slow_bearing_rad": float(CRUISE_CAMERA_SIMPLE_FORWARD_SLOW_BEARING_RAD),
                "camera_simple_distance_deadband_m": float(CRUISE_CAMERA_SIMPLE_DISTANCE_DEADBAND_M),
                "camera_target_zone": str(target_zone),
                "camera_target_turn_side": str(camera_target_turn_side or ""),
                "camera_target_turn_side_clearance_m": (
                    None
                    if camera_target_turn_side_clearance_m is None
                    else round(float(camera_target_turn_side_clearance_m), 4)
                ),
                "camera_target_turn_side_blocked": bool(camera_target_turn_side_blocked),
                "camera_target_turn_side_block_m": float(CRUISE_CAMERA_TARGET_TURN_SIDE_BLOCK_M),
                "target_bubble_drift": bool(target_bubble_drift),
                "target_hold_creep": bool(target_hold_creep),
                "target_speed_mps": round(float(target_speed_mps), 4),
                "target_moving": bool(target_moving),
                "actual_target_distance_m": round(float(actual_target["distance_m"]), 4),
                "actual_target_bearing_error_rad": round(float(actual_target["bearing_error_rad"]), 4),
                "actual_target_radial_velocity_mps": round(float(actual_target["radial_velocity_mps"]), 4),
                "moving_target_min_mps": float(CRUISE_MOVING_TARGET_MIN_MPS),
                "moving_target_inner_hold_m": float(CRUISE_MOVING_TARGET_INNER_HOLD_M),
                "moving_target_bubble_track_mps": float(CRUISE_MOVING_TARGET_BUBBLE_TRACK_MPS),
                "moving_target_hold_creep_track_mps": float(CRUISE_MOVING_TARGET_HOLD_CREEP_TRACK_MPS),
                "moving_target_hold_creep_min_radial_mps": float(CRUISE_MOVING_TARGET_HOLD_CREEP_MIN_RADIAL_MPS),
                "moving_target_hold_creep_max_bearing_rad": float(CRUISE_MOVING_TARGET_HOLD_CREEP_MAX_BEARING_RAD),
                "bubble_drift_heading_deadband_rad": float(CRUISE_BUBBLE_DRIFT_HEADING_DEADBAND_RAD),
                "target_slow_distance_m": round(float(target_slow_distance_m), 4),
                "target_rearward": bool(target_rearward),
                "target_outside_forward_arc": bool(target_outside_forward_arc),
                "target_heading_arc_enter_rad": float(CRUISE_TARGET_HEADING_ARC_ENTER_RAD),
                "target_heading_arc_release_rad": float(CRUISE_TARGET_HEADING_ARC_RELEASE_RAD),
                "target_heading_reacquire_latched": bool(self._heading_reacquire_latched),
                "target_heading_side": str(preferred_side),
                "target_heading_side_clearance_m": (
                    None if preferred_side_clearance_m is None else round(float(preferred_side_clearance_m), 4)
                ),
                "target_reacquire_side_required_m": float(CRUISE_TARGET_REACQUIRE_SIDE_REQUIRED_M),
                "target_reacquire_side_open": bool(target_reacquire_side_open),
                "target_reacquire_side_max_narrower_m": float(CRUISE_TARGET_REACQUIRE_SIDE_MAX_NARROWER_M),
                "target_reacquire_side_clearance_ok": bool(target_reacquire_side_clearance_ok),
                "obstacle_gate": bool(obstacle_gate),
                "global_clearance_hard_gate": bool(global_clearance_hard_gate),
                "global_clearance_warning_gate": bool(global_clearance_warning_gate),
                "forward_space_bias_active": bool(forward_bias_active),
                "forward_space_bias_side": str(forward_bias_side or ""),
                "target_heading_side_hold_rad": float(CRUISE_TARGET_HEADING_SIDE_HOLD_RAD),
                "target_direction_blocked": bool(target_direction_blocked),
                "target_direction_gate_relevant": bool(target_direction_gate_relevant),
                "target_direction_raw_blocked": bool(target_direction_raw_blocked),
                "target_direction_latched": bool(target_direction_blocked),
                "target_direction_clearance_m": (
                    None if target_direction_clearance_m is None else round(float(target_direction_clearance_m), 4)
                ),
                "camera_target_close_front_forward_blocked": bool(camera_target_close_front_forward_blocked),
                "camera_target_close_front_forward_block_m": round(
                    float(camera_target_close_front_forward_block_m),
                    4,
                ),
                "camera_target_close_front_forward_margin_m": round(float(camera_target_close_front_forward_margin_m), 4),
                "target_search_active": bool(target_search_active),
                "target_search_clearance_m": round(
                    float(
                        min(
                            float(effective_front_m),
                            float(global_min_m) if global_min_m is not None and float(global_min_m) > 0.0 else float(effective_front_m),
                        )
                    ),
                    4,
                ),
                "target_search_rotate_track_mps": round(
                    float(_scaled_speed(limits, CRUISE_TARGET_SEARCH_ROTATE_TRACK_MPS)),
                    5,
                ),
                "target_search_min_front_clearance_m": float(CRUISE_TARGET_SEARCH_MIN_FRONT_CLEARANCE_M),
                "target_search_hold_front_clearance_m": float(CRUISE_TARGET_SEARCH_HOLD_FRONT_CLEARANCE_M),
                "heading_arc_allowed": bool(
                    (target_rearward or target_outside_forward_arc)
                    and distance_m > target_stop_distance_m
                    and effective_front_m > CRUISE_STOP_M
                    and not blocked_front
                ),
                "forward_arc_allowed": bool(
                    (not target_rearward)
                    and (not target_outside_forward_arc)
                    and distance_m > target_stop_distance_m
                ),
            },
            "track_reference": {
                "left_mps": round(float(left_mps), 5),
                "right_mps": round(float(right_mps), 5),
            },
            "speed_limits": {
                "speed_scale": round(float(limits.get("speed_scale", 1.0)), 4),
                "max_track_mps": round(float(limits.get("max_track_mps", 0.0)), 5),
                "target_turn_diff_mps": round(float(limits.get("target_turn_diff_mps", 0.0)), 5),
                "omega_limit_track_delta_mps": round(float(limits.get("omega_limit_track_delta_mps", 0.0)), 5),
                "target_pivot_track_mps": round(float(limits.get("target_pivot_track_mps", 0.0)), 5),
            },
            "target_lidar_status": {
                "mini_map_used": True,
                "front_gap_has_data": bool(front_gap.get("has_data", False)),
                "target_direction_has_data": bool(target_direction_gap.get("has_data", False)),
                "target_direction_gate_relevant": bool(target_direction_gate_relevant),
                "target_direction_blocked": bool(target_direction_blocked),
                "target_direction_hard_blocked": bool(target_direction_hard_blocked),
                "side_sector_has_data": bool(
                    bool((raw_context.get("left") or {}).get("has_data", False))
                    or bool((raw_context.get("right") or {}).get("has_data", False))
                ),
            },
            "track_width_m": round(float(width), 4),
            "follow_above_cruise": True,
        }
        navigation_intent = NavigationIntent.from_follow_request(
            request,
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            source=str(source or request.source or "STATE"),
            priority=810,
            metadata={
                "room_cruise_phase": str(phase),
                "room_cruise_reason": str(reason),
                "validated_chain": ROOM_CRUISE_VALIDATED_CHAIN,
            },
        )
        navigation_intent_dict = navigation_intent.to_dict()
        local_navigation_diag: Dict[str, Any] = {}
        local_navigation_suppressed_phase = bool(
            phase in {
                "lidar_confidence_hold",
                "camera_detection_required_hold",
                "target_reacquire_hold",
                "target_hold",
                "candidate_hold_zero_track",
            }
        )
        details = {
            "follow_request": req_status,
            "navigation_intent": navigation_intent_dict,
            "cruise_layer": {
                "active": True,
                "primitive_type": "set_track_velocity",
                "motion_style": ROOM_CRUISE_FOLLOW_STYLE,
                "source": str(source or request.source or "STATE"),
                "target_source": str(request.target_source or ""),
                "reason": str(request.reason or ""),
                "room_cruise_chain": True,
                "follow_above_cruise": True,
                "local_planner_bypassed": True,
                "local_navigation_suppressed_phase": bool(local_navigation_suppressed_phase),
            },
            "room_cruise": dict(room_cruise),
            "clearance": dict(clearance_details),
            "obstacle_avoidance": dict(obstacle_details),
            "speed_profile": {"phase": str(phase), "reason": str(reason)},
        }
        if (
            local_planner is not None
            and hasattr(local_planner, "tick_intent")
            and not bool(target_search_active)
            and not bool(local_navigation_suppressed_phase)
        ):
            try:
                local_navigation_result = local_planner.tick_intent(
                    navigation_intent,
                    lidar_summary=dict(lidar_summary or {}),
                    ekf_state=dict(ekf_state or {}),
                    raw_scan=list(raw_scan or []),
                    source=str(source or request.source or "STATE"),
                    dt=float(dt),
                    update_map=False,
                )
                local_navigation_diag = dict(getattr(local_navigation_result, "diagnostics", {}) or {})
                if getattr(local_navigation_result, "proposal", None) is not None:
                    proposal = dict(local_navigation_result.proposal)
                    proposal["name"] = "room_cruise_local_navigation"
                    proposal["source"] = str(source or request.source or "STATE")
                    proposal["priority"] = 810
                    proposal_details = dict(proposal.get("details") or {})
                    local_planner_speed_profile = dict(proposal_details.get("speed_profile") or {})
                    cruise_details = dict(details)
                    cruise_details["cruise_layer"] = {
                        **dict(details["cruise_layer"]),
                        "primitive_type": "local_planner_segment",
                        "local_planner_bypassed": False,
                        "local_navigation_active": True,
                    }
                    cruise_details["cruise_speed_profile"] = dict(cruise_details.get("speed_profile") or {})
                    proposal_details.update(cruise_details)
                    if local_planner_speed_profile:
                        proposal_details["speed_profile"] = dict(local_planner_speed_profile)
                    proposal_details["local_navigation"] = dict(local_navigation_diag)
                    proposal["details"] = proposal_details
                    status = {
                        "active": True,
                        "reason": "room_cruise_local_navigation_ready",
                        "primitive_type": "local_planner_segment",
                        "motion_style": ROOM_CRUISE_FOLLOW_STYLE,
                        "room_cruise_chain": True,
                        "follow_above_cruise": True,
                        "follow_request": req_status,
                        "navigation_intent": navigation_intent_dict,
                        "room_cruise": dict(room_cruise),
                        "local_planner": {
                            **dict(local_navigation_diag),
                            "active": True,
                            "reason": str(local_navigation_diag.get("reason") or "local_navigation_ready"),
                        },
                    }
                    return CruiseLayerResult(proposal=proposal, status=status)
            except Exception as e:
                local_navigation_diag = {
                    "active": False,
                    "reason": f"local_navigation_error:{e}",
                }
        if local_navigation_diag:
            details["local_navigation"] = dict(local_navigation_diag)
        proposal = make_motion_proposal(
            name="room_cruise_follow_gate",
            layer="CRUISE",
            source=str(source or request.source or "STATE"),
            command_type="set_track_velocity",
            execution_mode=execution_mode_for_command("set_track_velocity", "CRUISE"),
            v_target=float(v_target),
            omega_target=float(omega_target),
            priority=810,
            entry_tier=ENTRY_TIER_PRIMARY,
            requested_track_reference={"left_mps": float(left_mps), "right_mps": float(right_mps)},
            details=details,
        )
        status = {
            "active": True,
            "reason": "room_cruise_primitive_ready",
            "primitive_type": "set_track_velocity",
            "motion_style": ROOM_CRUISE_FOLLOW_STYLE,
            "room_cruise_chain": True,
            "follow_above_cruise": True,
            "follow_request": req_status,
            "navigation_intent": navigation_intent_dict,
            "room_cruise": dict(room_cruise),
            "local_planner": {
                "active": False,
                "reason": (
                    str(local_navigation_diag.get("reason"))
                    if local_navigation_diag
                    else "bypassed_by_cruise_motion_gate"
                ),
            },
        }
        return CruiseLayerResult(proposal=proposal, status=status)
