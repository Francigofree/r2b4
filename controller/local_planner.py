#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Minimal local planner — target/pose-based, collision-checked trajectory segments.

Sits between high-level target/waypoint commands and the motion resolver.
Input: target_pose (x, y, theta_rad) + EKF current state + LIDAR summary.
Output: a single resolver-compatible proposal with collision-checked v/omega.

The planner knows WHERE the robot wants to go and checks feasibility along
the actual direction of travel, not just "forward".  This eliminates
blind-forward collision checks when the target is at an angle.

Design:
1. Compute segment geometry: current EKF pose → target_pose.
2. Directional clearance check using LIDAR sector data.
3. Simple proportional v/omega towards target (not a full pose controller —
   that is the job of UnicyclePoseController; the planner focuses on
   feasibility gating and near-obstacle slowdown).
4. Output a resolver-compatible proposal (entry_tier=PRIMARY, priority=795).

Integration:
- Called from cont.py main loop.  Receives target_pose from ctrl.target_pose
  or the active waypoint mission segment.
- Does NOT access motors, PWM, safety_gate.
- Output ONLY through resolver (no direct ctrl attribute writes).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from controller.motion_resolver import (
    ENTRY_TIER_PRIMARY,
    make_motion_proposal,
)
from controller.motion_schema import execution_mode_for_command
from controller.motion_schema import turn_direction_to_omega_sign
from controller.motion_schema import YAW_SIGN_CONVENTION
from core.motion.clearance import dynamic_front_clearance_thresholds


# ---------------------------------------------------------------------------
# Planner configuration (conservative defaults).
# ---------------------------------------------------------------------------

@dataclass
class LocalPlannerConfig:
    """All tunable parameters for the local planner."""

    enabled: bool = True

    # Horizon: how far ahead the planner checks (meters).
    horizon_m: float = 0.40

    # Minimum clearance to allow a segment (meters).
    min_clearance_m: float = 0.30

    # Clearance buffer added to segment length for safety margin.
    clearance_buffer_m: float = 0.15

    # Maximum segment length per tick (meters).
    max_segment_length_m: float = 1.0

    # Optional cap for clearance lookahead distance.
    # Required clearance uses min(segment_length_m, cap) + clearance_buffer_m.
    clearance_lookahead_cap_m: Optional[float] = None

    # Speed reduction when close to min_clearance.
    near_obstacle_v_scale: float = 0.40

    # Blocked-front hard gate: zero output.
    blocked_front_zero: bool = True

    # Priority in the resolver: pose-target navigation should be planner-owned.
    proposal_priority: int = 795

    # Maximum linear speed the planner will propose (m/s).
    max_v: float = 0.30

    # Maximum angular speed the planner will propose (rad/s).
    max_omega: float = 0.60

    # ---- Target/pose proportional gains ----
    # Linear gain: v = k_xy * along-track error (robot frame e_x).
    k_xy: float = 0.50

    # Angular gains: omega = k_theta * heading_error + k_y * lateral_error.
    k_theta: float = 0.80
    k_y: float = 0.25

    # When bearing error exceeds this, rotate before driving.
    # AMR path following should prefer forward arcs for moderate lateral targets.
    rotate_first_threshold_rad: float = 1.20  # ~69°

    # Arrival tolerance.
    tolerance_xy_m: float = 0.03
    tolerance_theta_rad: float = 0.08

    # Room Cruise pivot qualification.  The 0.30 s windows match the frozen
    # M4.1 transition-settle window; three fresh samples are supported by the
    # latest structured M4.1 run (506 samples / 61.69 s).  Cooldown is two
    # confirmation windows, while the 8 s ceiling is the existing heading-turn
    # maximum duration from the active motion-readiness contract.
    room_cruise_stuck_confirm_s: float = 0.30
    room_cruise_stuck_min_fresh_samples: int = 3
    room_cruise_pivot_exit_confirm_s: float = 0.30
    room_cruise_pivot_exit_min_fresh_samples: int = 3
    room_cruise_pivot_cooldown_s: float = 0.60
    room_cruise_pivot_max_s: float = 8.0
    # Room Cruise comfort envelope.  These are navigation thresholds, not
    # safety thresholds: the 0.85 m acquisition point reuses the established
    # wall-follow/soft-clearance trigger, while 0.10 m is the established
    # lateral deadband.  The wall target remains the existing 0.48 m.
    room_cruise_wall_avoid_start_m: float = 0.85
    room_cruise_wall_avoid_asymmetry_m: float = 0.10


PLANNER_YAW_SIGN_CONVENTION = YAW_SIGN_CONVENTION
FOLLOW_CRUISE_MOTION_STYLE = "follow_cruise"
FOLLOW_CRUISE_TRIGGER_M = 1.05
FOLLOW_CRUISE_HARD_FRONT_M = 0.25
FOLLOW_CRUISE_PIVOT_FLOOR_M = 0.20
FOLLOW_CRUISE_SIDE_REQUIRED_M = 0.43
FOLLOW_CRUISE_HEADING_ALIGN_ENTER_RAD = 0.55
FOLLOW_CRUISE_ARC_MIN_V_MPS = 0.15
FOLLOW_CRUISE_ARC_MAX_V_MPS = 0.18
FOLLOW_CRUISE_ARC_CLEARANCE_GAIN = 0.18
FOLLOW_CRUISE_ROTATE_ARC_GAIN = 0.40
FOLLOW_CRUISE_NOMINAL_MIN_V_MPS = 0.15
FOLLOW_CRUISE_ARC_OMEGA_EPS_RAD_S = 0.035
# The legacy CruiseLayer already treats confidence below 0.30 as a motion
# hold.  Room Cruise V2 cannot hold without breaking its continuous-motion
# contract, so it uses the existing in-place PIVOT primitive one observed
# margin earlier, but only with verified surrounding clearance.  The M5
# multi-run audit found no captured matcher result in the 0.35--0.50 band:
# the observed losses crossed both values in the same scan.  Keep the original
# evidence-backed margin; matcher and safety thresholds remain independent.
FOLLOW_CRUISE_LOCALIZATION_PIVOT_ENTER_CONFIDENCE = 0.35
FOLLOW_CRUISE_LOCALIZATION_PIVOT_MIN_CLEARANCE_M = 0.60
FOLLOW_CRUISE_LOCALIZATION_PIVOT_OMEGA_RAD_S = 0.35


def create_from_config(cfg: Dict[str, Any]) -> "LocalPlanner":
    """Create a LocalPlanner from the ``vezerles.local_planner`` config block."""
    raw = dict(cfg or {})
    room_cruise = dict(raw.get("room_cruise") or {})
    lp_cfg = LocalPlannerConfig(
        enabled=bool(raw.get("enabled", True)),
        horizon_m=float(raw.get("horizon_m", 0.60)),
        min_clearance_m=float(raw.get("min_clearance_m", 0.35)),
        clearance_buffer_m=float(raw.get("clearance_buffer_m", 0.20)),
        max_segment_length_m=float(raw.get("max_segment_length_m", 1.0)),
        clearance_lookahead_cap_m=(
            None if raw.get("clearance_lookahead_cap_m") is None
            else float(raw.get("clearance_lookahead_cap_m"))
        ),
        near_obstacle_v_scale=float(raw.get("near_obstacle_v_scale", 0.40)),
        blocked_front_zero=bool(raw.get("blocked_front_zero", True)),
        proposal_priority=int(raw.get("proposal_priority", 795)),
        max_v=float(raw.get("max_v", 0.30)),
        max_omega=float(raw.get("max_omega", 0.60)),
        k_xy=float(raw.get("k_xy", 0.50)),
        k_theta=float(raw.get("k_theta", 0.80)),
        k_y=float(raw.get("k_y", 0.25)),
        rotate_first_threshold_rad=float(raw.get("rotate_first_threshold_rad", 1.20)),
        tolerance_xy_m=float(raw.get("tolerance_xy_m", 0.03)),
        tolerance_theta_rad=float(raw.get("tolerance_theta_rad", 0.08)),
        room_cruise_stuck_confirm_s=max(
            0.0,
            float(room_cruise.get("stuck_confirm_s", 0.30)),
        ),
        room_cruise_stuck_min_fresh_samples=max(
            2,
            int(room_cruise.get("stuck_min_fresh_lidar_samples", 3)),
        ),
        room_cruise_pivot_exit_confirm_s=max(
            0.0,
            float(room_cruise.get("pivot_exit_confirm_s", 0.30)),
        ),
        room_cruise_pivot_exit_min_fresh_samples=max(
            2,
            int(room_cruise.get("pivot_exit_min_fresh_lidar_samples", 3)),
        ),
        room_cruise_pivot_cooldown_s=max(
            0.0,
            float(room_cruise.get("pivot_cooldown_s", 0.60)),
        ),
        room_cruise_pivot_max_s=max(
            0.1,
            float(room_cruise.get("pivot_max_s", 8.0)),
        ),
        room_cruise_wall_avoid_start_m=max(
            0.0,
            float(room_cruise.get("wall_avoid_start_m", 0.85)),
        ),
        room_cruise_wall_avoid_asymmetry_m=max(
            0.0,
            float(room_cruise.get("wall_avoid_asymmetry_m", 0.10)),
        ),
    )
    return LocalPlanner(lp_cfg)


# ---------------------------------------------------------------------------
# Geometry helpers.
# ---------------------------------------------------------------------------

def _wrap_angle(rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _extract_pose(ekf_state: Dict[str, Any]) -> Tuple[float, float, float]:
    """Extract (x, y, theta_rad) from EKF state dict."""
    x = float(ekf_state.get("x", 0.0) or 0.0)
    y = float(ekf_state.get("y", 0.0) or 0.0)
    theta = ekf_state.get("theta")
    if theta is None:
        theta = math.radians(float(ekf_state.get("theta_deg", 0.0) or 0.0))
    else:
        theta = float(theta)
    return x, y, theta


def _first_finite_float(data: Dict[str, Any], keys: Tuple[str, ...]) -> Tuple[Optional[float], str]:
    for key in keys:
        try:
            value = float(data.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value >= 0.0:
            return float(value), str(key)
    return None, ""


def _clearance_channel_for_direction(direction: str) -> Tuple[Tuple[str, ...], str, str]:
    d = str(direction or "forward").strip().lower()
    if d in ("reverse", "back", "backward"):
        return (("min_back", "back_clearance_m", "back_clearance"), "blocked_back", "reverse")
    if d in ("rotate_left", "left", "turn_left"):
        return (("left_clearance_m", "left_clearance", "min_left_clearance_m", "avg_left"), "", "rotate_left")
    if d in ("rotate_right", "right", "turn_right"):
        return (("right_clearance_m", "right_clearance", "min_right_clearance_m", "avg_right"), "", "rotate_right")
    return (
        ("min_dist_narrow", "front_clearance_m", "front_clearance", "min_dist"),
        "blocked_front",
        "forward",
    )


def _finite_values(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _float_from_diag(diag: Dict[str, Any], key: str) -> Optional[float]:
    try:
        value = float(diag.get(key))
    except (TypeError, ValueError):
        return None
    if math.isfinite(value) and value >= 0.0:
        return float(value)
    return None


def _cache_float(value: Any, default: float = -1.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    return float(out) if math.isfinite(out) else float(default)


def _side_clearance_m(lidar_summary: Dict[str, Any], side: str) -> Tuple[Optional[float], str]:
    normalized = str(side or "").strip().lower()
    if normalized == "right":
        return _first_finite_float(
            dict(lidar_summary or {}),
            ("right_clearance_m", "right_clearance", "min_right_clearance_m", "avg_right"),
        )
    return _first_finite_float(
        dict(lidar_summary or {}),
        ("left_clearance_m", "left_clearance", "min_left_clearance_m", "avg_left"),
    )


def _scan_point_to_bearing_distance(point: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(point, dict):
        return None
    dist_raw = point.get("dist", point.get("dist_mm", 0.0))
    try:
        dist_value = float(dist_raw)
    except Exception:
        return None
    if not math.isfinite(dist_value) or dist_value <= 0.0:
        return None
    dist_m = dist_value / 1000.0 if dist_value > 20.0 else dist_value
    if not math.isfinite(dist_m) or dist_m <= 0.01:
        return None

    angle_rad_raw = point.get("angle_rad")
    if angle_rad_raw is not None:
        try:
            angle_rad = float(angle_rad_raw)
        except Exception:
            return None
    else:
        try:
            angle_rad = math.radians(float(point.get("angle", 0.0)))
        except Exception:
            return None
    if not math.isfinite(angle_rad):
        return None

    # LIDAR frame convention used by the runtime: 90 deg = robot right,
    # 270 deg = robot left. Robot-frame bearing is left-positive.
    x = float(math.cos(angle_rad) * dist_m)
    y = float(-math.sin(angle_rad) * dist_m)
    bearing_rad = float(math.atan2(y, x))
    if not math.isfinite(bearing_rad):
        return None
    return bearing_rad, float(math.hypot(x, y))


def _front_scan_gap_analysis(raw_scan: Any, cache: Optional[Dict[Any, Any]] = None) -> Dict[str, Any]:
    status: Dict[str, Any] = {
        "enabled": True,
        "has_data": False,
        "reason": "no_scan",
        "best_direction": "UNKNOWN",
        "best_center_deg": None,
        "best_width_deg": 0.0,
        "best_clearance_m": None,
        "best_score": 0.0,
        "left_open_score": 0.0,
        "right_open_score": 0.0,
        "front_blocked_by_scan": False,
    }
    scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
    cache_key = ("front_gap", id(scan_points), len(scan_points))
    if isinstance(cache, dict) and cache_key in cache:
        return dict(cache[cache_key])
    if not scan_points:
        if isinstance(cache, dict):
            cache[cache_key] = dict(status)
        return status

    bins = 21
    half_angle_deg = 105.0
    bin_width_deg = (2.0 * half_angle_deg) / float(bins)
    max_range_m = 2.20
    free_threshold_m = 0.62
    clearances = [float(max_range_m) for _ in range(bins)]
    valid_points = 0

    for point in scan_points:
        polar = _scan_point_to_bearing_distance(point)
        if polar is None:
            continue
        bearing_rad, dist_m = polar
        if not math.isfinite(dist_m) or dist_m <= 0.02:
            continue
        bearing_deg = float(math.degrees(float(bearing_rad)))
        if bearing_deg < -half_angle_deg or bearing_deg > half_angle_deg:
            continue
        idx = int((bearing_deg + half_angle_deg) / max(1e-6, bin_width_deg))
        idx = max(0, min(bins - 1, idx))
        if dist_m <= max_range_m:
            clearances[idx] = min(float(clearances[idx]), float(dist_m))
        valid_points += 1

    if valid_points <= 0:
        status.update({"reason": "no_front_scan_points", "bin_count": int(bins)})
        if isinstance(cache, dict):
            cache[cache_key] = dict(status)
        return status

    centers = [
        float(-half_angle_deg + ((float(idx) + 0.5) * bin_width_deg))
        for idx in range(bins)
    ]
    free_flags = [float(clearances[idx]) >= free_threshold_m for idx in range(bins)]
    runs: List[Tuple[int, int]] = []
    idx = 0
    while idx < bins:
        if not free_flags[idx]:
            idx += 1
            continue
        start = idx
        while idx + 1 < bins and free_flags[idx + 1]:
            idx += 1
        runs.append((int(start), int(idx)))
        idx += 1

    def _side_score(left_positive: bool) -> float:
        values: List[float] = []
        for i, center in enumerate(centers):
            if (left_positive and float(center) >= 8.0) or ((not left_positive) and float(center) <= -8.0):
                values.append(max(0.0, min(1.0, float(clearances[i]) / max_range_m)))
        return float(sum(values) / max(1, len(values))) if values else 0.0

    left_score = _side_score(True)
    right_score = _side_score(False)
    best_run: Optional[Tuple[int, int]] = None
    best_score = -float("inf")
    best_center_deg = 0.0
    best_width_deg = 0.0
    best_clearance_m = 0.0
    for start, end in runs:
        span = max(1, int(end - start + 1))
        width_deg = float(span) * float(bin_width_deg)
        run_clearances = [float(clearances[i]) for i in range(start, end + 1)]
        avg_clearance = float(sum(run_clearances) / max(1, len(run_clearances)))
        center_deg = float(sum(float(centers[i]) for i in range(start, end + 1)) / float(span))
        width_score = max(0.0, min(1.0, width_deg / max(1e-6, 2.0 * half_angle_deg)))
        clearance_score = max(0.0, min(1.0, avg_clearance / max_range_m))
        center_penalty = max(0.0, min(1.0, abs(center_deg) / max(1e-6, half_angle_deg)))
        score = (0.54 * width_score) + (0.34 * clearance_score) + (0.12 * (1.0 - center_penalty))
        if score > best_score + 1e-9:
            best_score = float(score)
            best_run = (int(start), int(end))
            best_center_deg = float(center_deg)
            best_width_deg = float(width_deg)
            best_clearance_m = float(avg_clearance)

    center_bins = [
        i
        for i, center in enumerate(centers)
        if abs(float(center)) <= max(4.0, float(bin_width_deg) * 0.75)
    ]
    front_blocked_by_scan = any(float(clearances[i]) < free_threshold_m for i in center_bins)
    if best_run is not None:
        if best_center_deg >= 8.0:
            direction = "LEFT"
        elif best_center_deg <= -8.0:
            direction = "RIGHT"
        else:
            direction = "STRAIGHT"
        reason = "scored"
    else:
        direction = "LEFT" if left_score >= right_score else "RIGHT"
        reason = "no_free_run_side_score"
        best_clearance_m = min(clearances, default=float(max_range_m)) if clearances else float(max_range_m)

    status.update(
        {
            "has_data": True,
            "reason": str(reason),
            "best_direction": str(direction),
            "best_center_deg": round(float(best_center_deg), 3),
            "best_width_deg": round(float(best_width_deg), 3),
            "best_clearance_m": round(float(best_clearance_m), 4),
            "best_score": round(max(0.0, min(1.0, float(best_score))), 4),
            "left_open_score": round(float(left_score), 4),
            "right_open_score": round(float(right_score), 4),
            "front_blocked_by_scan": bool(front_blocked_by_scan),
            "bin_count": int(bins),
            "valid_front_points": int(valid_points),
            "free_threshold_m": round(float(free_threshold_m), 4),
            "max_range_m": round(float(max_range_m), 4),
        }
    )
    if isinstance(cache, dict):
        cache[cache_key] = dict(status)
    return status


def _front_clearance_m(
    lidar_summary: Dict[str, Any],
    front_clearance_diag: Dict[str, Any],
) -> Tuple[Optional[float], str]:
    diag_value = _float_from_diag(dict(front_clearance_diag or {}), "min_dist_m")
    if diag_value is not None:
        return diag_value, str((front_clearance_diag or {}).get("min_dist_source") or "front_clearance_diag")
    return _first_finite_float(
        dict(lidar_summary or {}),
        ("min_dist_narrow", "front_clearance_m", "front_clearance", "min_dist"),
    )


def _wide_front_clearance_m(
    lidar_summary: Dict[str, Any],
    *,
    fallback_m: Optional[float],
    fallback_source: str,
) -> Tuple[Optional[float], str, bool]:
    """Resolve the wide front envelope consumed by the final SafetyGate.

    Runtime LIDAR summaries expose this as ``min_dist``.  The explicit fallback
    preserves deterministic replay/unit fixtures that predate the wide channel,
    while diagnostics keep that reduced evidence distinguishable.
    """

    wide_m, wide_source = _first_finite_float(dict(lidar_summary or {}), ("min_dist",))
    if wide_m is not None:
        return float(wide_m), str(wide_source), True
    if fallback_m is not None and math.isfinite(float(fallback_m)) and float(fallback_m) >= 0.0:
        return float(fallback_m), f"fallback:{fallback_source or 'front_clearance_m'}", False
    return None, "", False


def _room_cruise_safe_progress_policy(
    *,
    obstacle_plan: Dict[str, Any],
    blocked_front: bool,
    effective_max_v_mps: float,
) -> Dict[str, Any]:
    """Map aggregate local free-space evidence monotonically to 0.15--0.30 m/s.

    This is a desired-speed policy only.  It deliberately has no temporal
    filter or ramp: physical continuity remains owned by
    ``MotionController.TRACK_REFERENCE_SLEW``.
    """

    plan = dict(obstacle_plan or {})

    def _ratio(value: Optional[float], low: float, high: float) -> Optional[float]:
        if value is None or not math.isfinite(float(value)):
            return None
        return max(0.0, min(1.0, (float(value) - float(low)) / max(1e-6, float(high) - float(low))))

    front_m = _float_from_diag(plan, "front_clearance_m")
    front_hard_m = _float_from_diag(plan, "front_hard_m")
    front_warning_m = _float_from_diag(plan, "front_warning_m")
    side_required_m = _float_from_diag(plan, "side_required_m")
    if front_hard_m is None:
        front_hard_m = float(FOLLOW_CRUISE_HARD_FRONT_M)
    if front_warning_m is None:
        front_warning_m = float(FOLLOW_CRUISE_TRIGGER_M)
    if side_required_m is None:
        side_required_m = float(FOLLOW_CRUISE_SIDE_REQUIRED_M)

    front_score = _ratio(front_m, front_hard_m, front_warning_m)
    left_m = _float_from_diag(plan, "left_clearance_m")
    right_m = _float_from_diag(plan, "right_clearance_m")
    side_scores = [
        value
        for value in (
            _ratio(left_m, side_required_m, front_warning_m),
            _ratio(right_m, side_required_m, front_warning_m),
        )
        if value is not None
    ]
    escape_side_score = max(side_scores) if side_scores else front_score
    nearest_side_score = min(side_scores) if side_scores else front_score

    front_gap = dict(plan.get("front_gap") or {})
    gap_score = (
        _float_from_diag(front_gap, "best_score")
        if bool(front_gap.get("has_data", False))
        else escape_side_score
    )
    if front_score is None:
        front_score = 0.0
    if escape_side_score is None:
        escape_side_score = float(front_score)
    if nearest_side_score is None:
        nearest_side_score = float(front_score)
    if gap_score is None:
        gap_score = float(escape_side_score)

    # Retain the established weighted free-path score, but do not let a wide
    # escape side hide a close wall on the other side.  Both front and nearest
    # lateral clearance are continuous monotonic upper bounds.
    weighted_score = (
        (0.54 * float(front_score))
        + (0.34 * float(escape_side_score))
        + (0.12 * float(gap_score))
    )
    aggregate_score = min(
        float(front_score),
        float(nearest_side_score),
        max(0.0, min(1.0, weighted_score)),
    )
    if bool(blocked_front):
        aggregate_score = 0.0

    runtime_max = max(0.0, float(effective_max_v_mps))
    nominal_min = min(runtime_max, float(FOLLOW_CRUISE_NOMINAL_MIN_V_MPS))
    speed_cap = nominal_min + ((runtime_max - nominal_min) * float(aggregate_score))
    speed_scale = speed_cap / runtime_max if runtime_max > 1e-9 else 0.0
    return {
        "active": True,
        "policy": "room_cruise_safe_progress_v2",
        "score_label": "Biztonsagos haladas",
        "aggregate_score": round(float(aggregate_score), 4),
        "components": {
            "front_clearance": round(float(front_score), 4),
            "escape_side_openness": round(float(escape_side_score), 4),
            "nearest_side_clearance": round(float(nearest_side_score), 4),
            "front_gap_quality": round(float(gap_score), 4),
        },
        "weights": {
            "front_clearance": 0.54,
            "escape_side_openness": 0.34,
            "front_gap_quality": 0.12,
        },
        "front_clearance_m": None if front_m is None else round(float(front_m), 4),
        "front_hard_m": round(float(front_hard_m), 4),
        "front_warning_m": round(float(front_warning_m), 4),
        "left_clearance_m": None if left_m is None else round(float(left_m), 4),
        "right_clearance_m": None if right_m is None else round(float(right_m), 4),
        "side_required_m": round(float(side_required_m), 4),
        "runtime_speed_cap_mps": round(float(runtime_max), 4),
        "nominal_min_mps": round(float(nominal_min), 4),
        "clearance_speed_cap_mps": round(float(speed_cap), 4),
        "speed_scale": round(float(speed_scale), 4),
        "monotonic_mapping": True,
        "temporal_shaping_applied": False,
        "profile_owner": "MotionController.TRACK_REFERENCE_SLEW",
    }


def _room_cruise_front_execution_envelope(
    *,
    lidar_summary: Dict[str, Any],
    obstacle_plan: Dict[str, Any],
    blocked_front: bool,
    effective_max_v_mps: float,
) -> Dict[str, Any]:
    """Prove that a forward Room Cruise request survives the final gate.

    ``clearance_speed_cap_mps`` is an upper bound on the forward command later
    produced by this planner tick.  Checking the SafetyGate brake-start
    threshold at that bound guarantees that any smaller final request is also
    outside the downstream braking zone.
    """

    plan = dict(obstacle_plan or {})
    fallback_m = _float_from_diag(plan, "front_clearance_m")
    wide_m, wide_source, measured_wide = _wide_front_clearance_m(
        lidar_summary,
        fallback_m=fallback_m,
        fallback_source=str(plan.get("front_clearance_source", "") or ""),
    )
    speed_policy = _room_cruise_safe_progress_policy(
        obstacle_plan=plan,
        blocked_front=bool(blocked_front),
        effective_max_v_mps=float(effective_max_v_mps),
    )
    v_upper_bound = max(0.0, float(speed_policy["clearance_speed_cap_mps"]))
    start_m, stop_m = dynamic_front_clearance_thresholds(v_upper_bound)
    available = bool(wide_m is not None)
    clear = bool(
        available
        and not bool(blocked_front)
        and (v_upper_bound <= 0.01 or float(wide_m) >= float(start_m))
    )
    return {
        "active": True,
        "policy": "shared_safety_gate_wide_front_v1",
        "clear": bool(clear),
        "blocked_front": bool(blocked_front),
        "wide_front_available": bool(available),
        "wide_front_measured": bool(measured_wide),
        "wide_front_m": None if wide_m is None else round(float(wide_m), 4),
        "wide_front_source": str(wide_source),
        "v_upper_bound_mps": round(float(v_upper_bound), 4),
        "brake_start_m": round(float(start_m), 4),
        "hard_stop_m": round(float(stop_m), 4),
        "outside_brake_zone": bool(
            available and wide_m is not None and float(wide_m) >= float(start_m)
        ),
    }


def _choose_avoidance_side(
    *,
    left_m: Optional[float],
    right_m: Optional[float],
    side_required_m: float,
    preferred_side: str,
    committed_side: str = "",
    front_m: Optional[float] = None,
    front_gap_side: str = "",
    front_gap_blocked: bool = False,
    front_gap_confident: bool = False,
    motion_style: str = "",
    clearance_escape_side: str = "",
) -> Tuple[str, str]:
    follow_cruise = str(motion_style or "").strip().lower() == FOLLOW_CRUISE_MOTION_STYLE
    preferred = "right" if str(preferred_side or "").strip().lower() == "right" else "left"
    gap_side = "right" if str(front_gap_side or "").strip().lower() == "right" else (
        "left" if str(front_gap_side or "").strip().lower() == "left" else ""
    )
    open_sides: List[Tuple[str, float]] = []
    if left_m is not None and float(left_m) >= float(side_required_m):
        open_sides.append(("left", float(left_m)))
    if right_m is not None and float(right_m) >= float(side_required_m):
        open_sides.append(("right", float(right_m)))
    if not open_sides:
        return "", "none_open"

    committed = "right" if str(committed_side or "").strip().lower() == "right" else (
        "left" if str(committed_side or "").strip().lower() == "left" else ""
    )
    open_clearances = {side: clearance for side, clearance in open_sides}
    clearance_escape = (
        "right" if str(clearance_escape_side or "").strip().lower() == "right"
        else "left" if str(clearance_escape_side or "").strip().lower() == "left"
        else ""
    )
    if gap_side and gap_side in open_clearances:
        front_tight_m = max(float(side_required_m) + 0.12, 0.62)
        front_tight = bool(front_m is not None and float(front_m) <= front_tight_m)
        other = "right" if gap_side == "left" else "left"
        other_clearance = open_clearances.get(other)
        gap_clearance = float(open_clearances.get(gap_side, 0.0))
        gap_not_worse = bool(
            other_clearance is None
            or float(gap_clearance) + (0.20 if bool(follow_cruise) else 0.0) >= float(other_clearance)
        )
        gap_authoritative = bool(
            bool(front_gap_confident)
            or (not bool(follow_cruise))
            or (bool(front_gap_blocked) and bool(gap_not_worse))
        )
        if not committed and bool(gap_authoritative) and (front_tight or bool(front_gap_blocked) or bool(front_gap_confident)):
            return gap_side, "front_gap"
        if committed == gap_side and bool(gap_authoritative) and (front_tight or bool(front_gap_blocked)):
            return gap_side, "front_gap"
        if committed and committed != gap_side and bool(front_gap_confident) and (front_tight or bool(front_gap_blocked)):
            return gap_side, "front_gap_escape"

    if committed and committed in open_clearances:
        other = "right" if committed == "left" else "left"
        if other in open_clearances:
            held_clearance = float(open_clearances[committed])
            other_clearance = float(open_clearances[other])
            switch_margin_m = 0.20 if bool(follow_cruise) else 0.45
            held_comfort_m = max(float(side_required_m) + 0.55, 1.05)
            front_tight_m = max(float(side_required_m) + 0.12, 0.62)
            front_tight = bool(front_m is not None and float(front_m) <= front_tight_m)
            held_uncomfortable = bool(held_clearance <= held_comfort_m)
            if (
                other_clearance - held_clearance >= switch_margin_m
                and (front_tight or held_uncomfortable)
            ):
                return other, "wider_side_escape"
        return committed, "held_side"

    # A newly classified side wall may only choose the initial escape
    # direction.  During an active avoidance episode the committed side is
    # resolved above, including its existing wider-side/front-gap escape
    # rules.  Giving the instantaneous wall classifier priority here used to
    # bypass that hysteresis and reverse a safe tangent arc on a single
    # left/right projection change.
    if clearance_escape and clearance_escape in open_clearances:
        return clearance_escape, "side_clearance_escape"

    if len(open_sides) == 1:
        return open_sides[0][0], "only_open_side"

    left_clearance = float(open_clearances.get("left", 0.0))
    right_clearance = float(open_clearances.get("right", 0.0))
    tie_margin_m = 0.10
    if abs(left_clearance - right_clearance) <= tie_margin_m:
        return preferred, "preferred_tie"
    return ("left", "clearer_side") if left_clearance > right_clearance else ("right", "clearer_side")


# ---------------------------------------------------------------------------
# Feasibility check (standalone, no import from commands.py for isolation).
# ---------------------------------------------------------------------------

def _segment_clearance_ok(
    *,
    segment_length_m: float,
    lidar_summary: Dict[str, Any],
    min_clearance_m: float,
    clearance_buffer_m: float,
    lookahead_cap_m: Optional[float] = None,
    motion_direction: str = "forward",
    clearance_cache: Optional[Dict[Any, Any]] = None,
) -> Tuple[bool, Dict[str, Any]]:
    """Check whether the intended path is clear for ``segment_length_m``."""
    l_sum = dict(lidar_summary or {})
    clearance_keys, blocked_key, clearance_direction = _clearance_channel_for_direction(motion_direction)
    cache_values = tuple(
        round(_cache_float(l_sum.get(key), -1.0), 4)
        for key in clearance_keys
    )
    cache_key = (
        "segment_clearance",
        str(clearance_direction),
        round(float(segment_length_m), 4),
        round(float(min_clearance_m), 4),
        round(float(clearance_buffer_m), 4),
        None if lookahead_cap_m is None else round(float(lookahead_cap_m), 4),
        bool(l_sum.get(blocked_key, False)) if blocked_key else False,
        bool(l_sum.get("blocked_front", False)),
        bool(l_sum.get("blocked_back", False)),
        cache_values,
    )
    if isinstance(clearance_cache, dict) and cache_key in clearance_cache:
        ok, diag = clearance_cache[cache_key]
        return bool(ok), dict(diag)
    min_dist, min_dist_source = _first_finite_float(l_sum, clearance_keys)
    blocked_direction = bool(l_sum.get(blocked_key, False)) if blocked_key else False
    blocked_front = bool(l_sum.get("blocked_front", False))
    blocked_back = bool(l_sum.get("blocked_back", False))
    effective_segment_m = float(segment_length_m)
    if lookahead_cap_m is not None:
        try:
            cap = float(lookahead_cap_m)
        except (TypeError, ValueError):
            cap = 0.0
        if cap > 0.0:
            effective_segment_m = min(effective_segment_m, cap)
    required_clearance = max(min_clearance_m, effective_segment_m + clearance_buffer_m)

    blocked_by_env = bool(
        blocked_direction
        or (min_dist is not None and min_dist < required_clearance)
    )
    feasible = not blocked_by_env

    diag: Dict[str, Any] = {
        "segment_length_m": round(segment_length_m, 4),
        "effective_segment_m": round(effective_segment_m, 4),
        "lookahead_cap_m": (None if lookahead_cap_m is None else round(float(lookahead_cap_m), 4)),
        "required_clearance_m": round(required_clearance, 4),
        "min_dist_m": (None if min_dist is None else round(min_dist, 4)),
        "min_dist_source": str(min_dist_source or ""),
        "clearance_direction": str(clearance_direction),
        "blocked_front": blocked_front,
        "blocked_back": blocked_back,
        "direction_blocked": bool(blocked_direction),
        "feasible": feasible,
    }
    if isinstance(clearance_cache, dict):
        clearance_cache[cache_key] = (bool(feasible), dict(diag))
    return feasible, diag


# ---------------------------------------------------------------------------
# Planner state machine.
# ---------------------------------------------------------------------------

@dataclass
class PlannerTickResult:
    """Output of a single planner tick."""
    proposal: Optional[Dict[str, Any]] = None
    blocked: bool = False
    idle: bool = False
    arrived: bool = False
    diagnostics: Dict[str, Any] = field(default_factory=dict)


class LocalPlanner:
    """Target/pose-based local planner with collision-checked proposals."""

    def __init__(self, cfg: LocalPlannerConfig | None = None):
        self.cfg = cfg or LocalPlannerConfig()
        self._last_tick_mono: float = 0.0
        self._blocked_since: Optional[float] = None
        self._total_blocked_ticks: int = 0
        self._total_passed_ticks: int = 0
        self._avoidance_side: str = ""
        self._room_cruise_pivot_active: bool = False
        self._room_cruise_pivot_side: str = ""
        self._room_cruise_pivot_entered_mono: float = 0.0
        self._room_cruise_pivot_cooldown_until: float = 0.0
        self._room_cruise_exit_started_mono: float = 0.0
        self._room_cruise_exit_fresh_samples: int = 0
        self._room_cruise_attempt_mode: str = ""
        self._room_cruise_attempt_started_mono: float = 0.0
        self._room_cruise_attempt_start_pose: Optional[Tuple[float, float]] = None
        self._room_cruise_attempt_fresh_samples: int = 0
        self._room_cruise_last_lidar_sample: Optional[Tuple[str, int]] = None
        self._room_cruise_arc_failed: bool = False
        self._room_cruise_reverse_active: bool = False
        self._room_cruise_reverse_side: str = ""
        self._room_cruise_reverse_failed: bool = False

    def _reset_room_cruise_escape(self, *, keep_cooldown: bool = False) -> None:
        cooldown = float(self._room_cruise_pivot_cooldown_until) if keep_cooldown else 0.0
        self._room_cruise_pivot_active = False
        self._room_cruise_pivot_side = ""
        self._room_cruise_pivot_entered_mono = 0.0
        self._room_cruise_pivot_cooldown_until = cooldown
        self._room_cruise_exit_started_mono = 0.0
        self._room_cruise_exit_fresh_samples = 0
        self._room_cruise_attempt_mode = ""
        self._room_cruise_attempt_started_mono = 0.0
        self._room_cruise_attempt_start_pose = None
        self._room_cruise_attempt_fresh_samples = 0
        self._room_cruise_last_lidar_sample = None
        self._room_cruise_arc_failed = False
        self._room_cruise_reverse_active = False
        self._room_cruise_reverse_side = ""
        self._room_cruise_reverse_failed = False

    @staticmethod
    def _lidar_sample_identity(lidar_summary: Dict[str, Any]) -> Optional[Tuple[str, int]]:
        summary = dict(lidar_summary or {})
        try:
            raw_scan_id = int(summary.get("raw_scan_id", 0) or 0)
        except (TypeError, ValueError):
            raw_scan_id = 0
        if raw_scan_id > 0:
            return "raw_scan_id", int(raw_scan_id)
        try:
            scan_seq = int(summary.get("scan_seq", 0) or 0)
        except (TypeError, ValueError):
            scan_seq = 0
        if scan_seq > 0:
            return "scan_seq", int(scan_seq)
        try:
            observation_count = int(summary.get("rolling_local_map_observation_count", 0) or 0)
        except (TypeError, ValueError):
            observation_count = 0
        if observation_count > 0:
            return "rolling_observation_count", int(observation_count)
        return None

    def _room_cruise_fresh_lidar_sample(self, lidar_summary: Dict[str, Any]) -> Tuple[bool, str]:
        identity = self._lidar_sample_identity(lidar_summary)
        if identity is None:
            return False, "missing_scan_identity"
        fresh = identity != self._room_cruise_last_lidar_sample
        self._room_cruise_last_lidar_sample = identity
        return bool(fresh), f"{identity[0]}:{identity[1]}"

    def _room_cruise_attempt_evidence(
        self,
        *,
        mode: str,
        pose_xy: Tuple[float, float],
        now_mono: float,
        fresh_lidar_sample: bool,
    ) -> Tuple[bool, Dict[str, Any]]:
        mode_s = str(mode or "")
        if mode_s != self._room_cruise_attempt_mode:
            self._room_cruise_attempt_mode = mode_s
            self._room_cruise_attempt_started_mono = float(now_mono)
            self._room_cruise_attempt_start_pose = (float(pose_xy[0]), float(pose_xy[1]))
            self._room_cruise_attempt_fresh_samples = 0
        if fresh_lidar_sample:
            self._room_cruise_attempt_fresh_samples += 1

        start_pose = self._room_cruise_attempt_start_pose or (float(pose_xy[0]), float(pose_xy[1]))
        progress_m = math.hypot(
            float(pose_xy[0]) - float(start_pose[0]),
            float(pose_xy[1]) - float(start_pose[1]),
        )
        elapsed_s = max(0.0, float(now_mono) - float(self._room_cruise_attempt_started_mono))
        progress_threshold_m = max(0.001, float(self.cfg.tolerance_xy_m))
        if progress_m >= progress_threshold_m:
            self._room_cruise_attempt_started_mono = float(now_mono)
            self._room_cruise_attempt_start_pose = (float(pose_xy[0]), float(pose_xy[1]))
            self._room_cruise_attempt_fresh_samples = 1 if fresh_lidar_sample else 0
            elapsed_s = 0.0
            progress_m = 0.0

        confirmed = bool(
            elapsed_s >= float(self.cfg.room_cruise_stuck_confirm_s)
            and self._room_cruise_attempt_fresh_samples
            >= int(self.cfg.room_cruise_stuck_min_fresh_samples)
            and progress_m < progress_threshold_m
        )
        return confirmed, {
            "attempt_mode": mode_s,
            "elapsed_s": round(float(elapsed_s), 4),
            "fresh_lidar_samples": int(self._room_cruise_attempt_fresh_samples),
            "required_fresh_lidar_samples": int(self.cfg.room_cruise_stuck_min_fresh_samples),
            "confirm_s": round(float(self.cfg.room_cruise_stuck_confirm_s), 4),
            "progress_m": round(float(progress_m), 4),
            "progress_threshold_m": round(float(progress_threshold_m), 4),
            "no_progress_confirmed": bool(confirmed),
        }

    def _room_cruise_escape_policy(
        self,
        *,
        plan: Dict[str, Any],
        lidar_summary: Dict[str, Any],
        pose_xy: Tuple[float, float],
        now_mono: float,
        forward_feasible: bool,
        reverse_feasible: bool,
        reverse_clearance: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Qualify Room Cruise pivot only after failed ARC/reverse execution.

        This state machine chooses a maneuver intent; it does not ramp or shape
        commands.  The unchanged MotionController remains the single physical
        profile owner after proposal resolution.
        """
        original = dict(plan or {})
        out = dict(original)
        fresh_sample, sample_identity = self._room_cruise_fresh_lidar_sample(lidar_summary)
        mode = str(original.get("mode", "") or "")
        side = str(original.get("side", "") or "")
        front_m = _float_from_diag(original, "front_clearance_m")
        front_hard_m = _float_from_diag(original, "front_hard_m")
        side_required_m = _float_from_diag(original, "side_required_m")
        side_clearance_m = _float_from_diag(
            original,
            "left_clearance_m" if side == "left" else "right_clearance_m",
        )
        blocked_front = bool(lidar_summary.get("blocked_front", False))
        front_execution_envelope = dict(original.get("front_execution_envelope") or {})
        front_execution_clear = bool(front_execution_envelope.get("clear", True))
        safe_arc_exit_available = bool(
            original.get("active", False)
            and mode == "tangent_arc"
            and not blocked_front
            and front_execution_clear
            and front_m is not None
            and front_hard_m is not None
            and float(front_m) > float(front_hard_m)
        )
        forward_exit_available = bool(
            not blocked_front
            and front_execution_clear
            and (forward_feasible or safe_arc_exit_available)
            and (front_m is None or front_hard_m is None or float(front_m) > float(front_hard_m))
        )
        pivot_improves_geometry = bool(
            side in {"left", "right"}
            and side_clearance_m is not None
            and side_required_m is not None
            and side_clearance_m >= side_required_m
            and (
                front_m is None
                or side_clearance_m >= float(front_m) + max(0.001, float(self.cfg.tolerance_xy_m))
            )
        )

        evidence: Dict[str, Any] = {
            "policy": "room_cruise_arc_first_stuck_v1",
            "profile_owner": "MotionController.TRACK_REFERENCE_SLEW",
            "fresh_lidar_sample": bool(fresh_sample),
            "lidar_sample_identity": str(sample_identity),
            "forward_feasible": bool(forward_feasible),
            "front_execution_clear": bool(front_execution_clear),
            "front_execution_envelope": dict(front_execution_envelope),
            "safe_arc_exit_available": bool(safe_arc_exit_available),
            "reverse_feasible": bool(reverse_feasible),
            "reverse_clearance": dict(reverse_clearance or {}),
            "pivot_improves_geometry": bool(pivot_improves_geometry),
            "arc_failed": bool(self._room_cruise_arc_failed),
            "reverse_active": bool(self._room_cruise_reverse_active),
            "reverse_failed": bool(self._room_cruise_reverse_failed),
            "pivot_active": bool(self._room_cruise_pivot_active),
            "cooldown_remaining_s": round(
                max(0.0, float(self._room_cruise_pivot_cooldown_until) - float(now_mono)),
                4,
            ),
        }

        if self._room_cruise_pivot_active:
            pivot_elapsed_s = max(0.0, float(now_mono) - float(self._room_cruise_pivot_entered_mono))
            if forward_exit_available:
                if self._room_cruise_exit_started_mono <= 0.0:
                    self._room_cruise_exit_started_mono = float(now_mono)
                    self._room_cruise_exit_fresh_samples = 0
                if fresh_sample:
                    self._room_cruise_exit_fresh_samples += 1
            else:
                self._room_cruise_exit_started_mono = 0.0
                self._room_cruise_exit_fresh_samples = 0
            exit_elapsed_s = (
                max(0.0, float(now_mono) - float(self._room_cruise_exit_started_mono))
                if self._room_cruise_exit_started_mono > 0.0
                else 0.0
            )
            exit_confirmed = bool(
                forward_exit_available
                and exit_elapsed_s >= float(self.cfg.room_cruise_pivot_exit_confirm_s)
                and self._room_cruise_exit_fresh_samples
                >= int(self.cfg.room_cruise_pivot_exit_min_fresh_samples)
            )
            timed_out = pivot_elapsed_s >= float(self.cfg.room_cruise_pivot_max_s)
            evidence.update(
                {
                    "pivot_elapsed_s": round(float(pivot_elapsed_s), 4),
                    "pivot_max_s": round(float(self.cfg.room_cruise_pivot_max_s), 4),
                    "exit_elapsed_s": round(float(exit_elapsed_s), 4),
                    "exit_fresh_lidar_samples": int(self._room_cruise_exit_fresh_samples),
                    "exit_required_fresh_lidar_samples": int(
                        self.cfg.room_cruise_pivot_exit_min_fresh_samples
                    ),
                    "exit_confirmed": bool(exit_confirmed),
                }
            )
            if exit_confirmed:
                self._room_cruise_pivot_cooldown_until = (
                    float(now_mono) + float(self.cfg.room_cruise_pivot_cooldown_s)
                )
                self._reset_room_cruise_escape(keep_cooldown=True)
                out = dict(original)
                evidence.update(
                    {
                        "pivot_active": False,
                        "state": "pivot_exit_to_arc",
                        "cooldown_remaining_s": round(float(self.cfg.room_cruise_pivot_cooldown_s), 4),
                    }
                )
                out["stuck_evidence"] = evidence
                return out
            if timed_out:
                self._room_cruise_pivot_cooldown_until = (
                    float(now_mono) + float(self.cfg.room_cruise_pivot_cooldown_s)
                )
                self._reset_room_cruise_escape(keep_cooldown=True)
                out.update(
                    {
                        "active": False,
                        "mode": "",
                        "reason": "room_cruise_pivot_timeout_hold",
                        "stuck_evidence": {**evidence, "pivot_active": False, "state": "pivot_timeout_hold"},
                    }
                )
                return out
            pivot_side = self._room_cruise_pivot_side or side
            out.update(
                {
                    "active": True,
                    "mode": "room_cruise_stuck_pivot",
                    "side": str(pivot_side),
                    "turn_sign": turn_direction_to_omega_sign(pivot_side),
                    "reason": "room_cruise_stuck_pivot_active",
                    "stuck_evidence": {**evidence, "pivot_active": True, "state": "pivot_active"},
                }
            )
            return out

        geometric_pressure = bool(
            not forward_feasible or blocked_front or not front_execution_clear
        )
        valid_escape_side = bool(
            side in {"left", "right"}
            and turn_direction_to_omega_sign(side) != 0
        )
        reverse_exit_available = bool(
            not blocked_front
            and forward_feasible
            and front_execution_clear
            and (front_m is None or front_hard_m is None or float(front_m) > float(front_hard_m))
        )
        attempt_diag: Dict[str, Any] = {}
        reverse_failed_now = False

        # Once reverse has legitimately started, keep both its direction and
        # sign until a forward path is confirmed across the same evidence
        # window used by pivot exit.  A single clearance-classification tick
        # therefore cannot cause forward/reverse rocking.
        if self._room_cruise_reverse_active:
            reverse_side = self._room_cruise_reverse_side
            reverse_mode = (
                "room_cruise_reverse_arc"
                if reverse_side in {"left", "right"}
                else "room_cruise_reverse_straight"
            )
            if reverse_exit_available:
                if self._room_cruise_exit_started_mono <= 0.0:
                    self._room_cruise_exit_started_mono = float(now_mono)
                    self._room_cruise_exit_fresh_samples = 0
                if fresh_sample:
                    self._room_cruise_exit_fresh_samples += 1
            else:
                self._room_cruise_exit_started_mono = 0.0
                self._room_cruise_exit_fresh_samples = 0
            exit_elapsed_s = (
                max(0.0, float(now_mono) - float(self._room_cruise_exit_started_mono))
                if self._room_cruise_exit_started_mono > 0.0
                else 0.0
            )
            exit_confirmed = bool(
                reverse_exit_available
                and exit_elapsed_s >= float(self.cfg.room_cruise_pivot_exit_confirm_s)
                and self._room_cruise_exit_fresh_samples
                >= int(self.cfg.room_cruise_pivot_exit_min_fresh_samples)
            )
            evidence.update(
                {
                    "reverse_active": True,
                    "reverse_side": str(reverse_side),
                    "reverse_exit_available": bool(reverse_exit_available),
                    "exit_elapsed_s": round(float(exit_elapsed_s), 4),
                    "exit_fresh_lidar_samples": int(self._room_cruise_exit_fresh_samples),
                    "exit_confirmed": bool(exit_confirmed),
                }
            )
            if exit_confirmed:
                self._reset_room_cruise_escape(keep_cooldown=True)
                out = dict(original)
                evidence.update(
                    {
                        "reverse_active": False,
                        "state": "reverse_exit_to_forward",
                    }
                )
                out["stuck_evidence"] = evidence
                return out

            if reverse_exit_available:
                failed_now = False
                attempt_diag = {
                    "attempt_mode": "reverse_exit_confirmation",
                    "elapsed_s": round(float(exit_elapsed_s), 4),
                    "fresh_lidar_samples": int(self._room_cruise_exit_fresh_samples),
                    "confirm_s": round(float(self.cfg.room_cruise_pivot_exit_confirm_s), 4),
                    "no_progress_confirmed": False,
                }
            elif reverse_feasible:
                failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                    mode="reverse_tangent_arc",
                    pose_xy=pose_xy,
                    now_mono=now_mono,
                    fresh_lidar_sample=fresh_sample,
                )
            else:
                failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                    mode="reverse_clearance_unavailable",
                    pose_xy=pose_xy,
                    now_mono=now_mono,
                    fresh_lidar_sample=fresh_sample,
                )
            if failed_now:
                self._room_cruise_reverse_active = False
                self._room_cruise_reverse_failed = True
                reverse_failed_now = True
                self._room_cruise_exit_started_mono = 0.0
                self._room_cruise_exit_fresh_samples = 0
            elif reverse_feasible:
                out.update(
                    {
                        "active": True,
                        "mode": str(reverse_mode),
                        "side": str(reverse_side),
                        "turn_sign": turn_direction_to_omega_sign(reverse_side),
                        "reason": (
                            "room_cruise_reverse_arc_escape_attempt"
                            if reverse_side
                            else "room_cruise_reverse_straight_escape_attempt"
                        ),
                        "stuck_evidence": {
                            **evidence,
                            "attempt": dict(attempt_diag),
                            "state": "reverse_active",
                        },
                    }
                )
                return out
            else:
                out.update(
                    {
                        "active": False,
                        "mode": "",
                        "reason": "room_cruise_reverse_clearance_hold",
                        "stuck_evidence": {
                            **evidence,
                            "attempt": dict(attempt_diag),
                            "state": "reverse_clearance_hold",
                        },
                    }
                )
                return out

        # Clear forward space is never an escape trigger.  This early return
        # is the direct regression guard for the 17:59 live-run rocking root.
        if not geometric_pressure:
            self._reset_room_cruise_escape(keep_cooldown=True)
            evidence.update(
                {
                    "arc_failed": False,
                    "reverse_active": False,
                    "reverse_failed": False,
                    "state": "forward_path_available",
                }
            )
            out["stuck_evidence"] = evidence
            return out

        arc_attempt_available = bool(
            original.get("active", False)
            and mode == "tangent_arc"
            and geometric_pressure
            and not self._room_cruise_arc_failed
        )
        failed_now = False
        if reverse_failed_now:
            pass
        elif arc_attempt_available:
            failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                mode="forward_tangent_arc",
                pose_xy=pose_xy,
                now_mono=now_mono,
                fresh_lidar_sample=fresh_sample,
            )
            if failed_now:
                self._room_cruise_arc_failed = True
        elif mode == "escape_candidate":
            # Geometrically unavailable forward ARC must persist across fresh
            # scans; one blocked classifier sample cannot enable reverse.
            failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                mode="forward_arc_unavailable",
                pose_xy=pose_xy,
                now_mono=now_mono,
                fresh_lidar_sample=fresh_sample,
            )
            if failed_now:
                self._room_cruise_arc_failed = True
        elif geometric_pressure and not valid_escape_side:
            # No side is wide enough for an ARC.  Confirm that geometry over
            # fresh scans, then use the already-supported reverse-straight M3
            # primitive when the rear swept envelope is clear.
            failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                mode="forward_escape_side_unavailable",
                pose_xy=pose_xy,
                now_mono=now_mono,
                fresh_lidar_sample=fresh_sample,
            )
            if failed_now:
                self._room_cruise_arc_failed = True

        if (
            self._room_cruise_arc_failed
            and not self._room_cruise_reverse_failed
            and reverse_feasible
        ):
            self._room_cruise_reverse_active = True
            self._room_cruise_reverse_side = str(side) if valid_escape_side else ""
            self._room_cruise_exit_started_mono = 0.0
            self._room_cruise_exit_fresh_samples = 0
            failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                mode="reverse_tangent_arc",
                pose_xy=pose_xy,
                now_mono=now_mono,
                fresh_lidar_sample=fresh_sample,
            )
            if failed_now:
                self._room_cruise_reverse_active = False
                self._room_cruise_reverse_failed = True
            else:
                reverse_mode = (
                    "room_cruise_reverse_arc"
                    if self._room_cruise_reverse_side
                    else "room_cruise_reverse_straight"
                )
                out.update(
                    {
                        "active": True,
                        "mode": str(reverse_mode),
                        "side": str(self._room_cruise_reverse_side),
                        "turn_sign": turn_direction_to_omega_sign(self._room_cruise_reverse_side),
                        "reason": (
                            "room_cruise_reverse_arc_escape_attempt"
                            if self._room_cruise_reverse_side
                            else "room_cruise_reverse_straight_escape_attempt"
                        ),
                    }
                )
        elif self._room_cruise_arc_failed and not self._room_cruise_reverse_failed:
            failed_now, attempt_diag = self._room_cruise_attempt_evidence(
                mode=(
                    "reverse_clearance_unavailable"
                    if not reverse_feasible
                    else "reverse_arc_direction_unavailable"
                ),
                pose_xy=pose_xy,
                now_mono=now_mono,
                fresh_lidar_sample=fresh_sample,
            )
            if failed_now:
                self._room_cruise_reverse_failed = True

        attempts_exhausted = bool(
            self._room_cruise_arc_failed
            and self._room_cruise_reverse_failed
        )
        prior_valid_attempt_failed = bool(
            self._room_cruise_arc_failed
            and (mode in {"tangent_arc", "escape_candidate"} or self._room_cruise_reverse_failed)
        )
        cooldown_open = float(now_mono) >= float(self._room_cruise_pivot_cooldown_until)
        pivot_entry = bool(
            attempts_exhausted
            and prior_valid_attempt_failed
            and pivot_improves_geometry
            and cooldown_open
        )
        evidence.update(
            {
                "attempt": dict(attempt_diag),
                "arc_failed": bool(self._room_cruise_arc_failed),
                "reverse_active": bool(self._room_cruise_reverse_active),
                "reverse_side": str(self._room_cruise_reverse_side),
                "reverse_failed": bool(self._room_cruise_reverse_failed),
                "attempts_exhausted": bool(attempts_exhausted),
                "prior_valid_attempt_failed": bool(prior_valid_attempt_failed),
                "cooldown_open": bool(cooldown_open),
                "pivot_entry": bool(pivot_entry),
            }
        )
        if pivot_entry:
            pivot_side = side or self._avoidance_side
            self._room_cruise_pivot_active = True
            self._room_cruise_pivot_side = str(pivot_side)
            self._room_cruise_pivot_entered_mono = float(now_mono)
            self._room_cruise_exit_started_mono = 0.0
            self._room_cruise_exit_fresh_samples = 0
            out.update(
                {
                    "active": True,
                    "mode": "room_cruise_stuck_pivot",
                    "side": str(pivot_side),
                    "turn_sign": turn_direction_to_omega_sign(pivot_side),
                    "reason": "room_cruise_stuck_evidence_confirmed",
                }
            )
            evidence.update({"pivot_active": True, "state": "pivot_entry"})
        if str(out.get("mode", "") or "") == "escape_candidate":
            out.update(
                {
                    "active": False,
                    "mode": "",
                    "reason": "room_cruise_escape_hold_without_valid_attempt",
                }
            )
        out["stuck_evidence"] = evidence
        return out

    def _obstacle_tangent_arc_plan(
        self,
        *,
        lidar_summary: Dict[str, Any],
        front_clearance_diag: Dict[str, Any],
        bearing_error: float,
        front_feasible: bool,
        effective_max_v_mps: float,
        raw_scan: Any = None,
        motion_style: str = "",
        clearance_cache: Optional[Dict[Any, Any]] = None,
    ) -> Dict[str, Any]:
        """Return a low-speed tangent arc plan when front clearance is shrinking."""
        follow_cruise = str(motion_style or "").strip().lower() == FOLLOW_CRUISE_MOTION_STYLE
        front_m, front_source = _front_clearance_m(lidar_summary, front_clearance_diag)
        required_m = _float_from_diag(front_clearance_diag, "required_clearance_m")
        if required_m is None:
            required_m = max(self.cfg.min_clearance_m, self.cfg.horizon_m + self.cfg.clearance_buffer_m)
        hard_front_m = (
            max(float(FOLLOW_CRUISE_HARD_FRONT_M), float(self.cfg.min_clearance_m))
            if bool(follow_cruise)
            else max(0.30, float(self.cfg.min_clearance_m))
        )
        warning_front_m = (
            max(float(FOLLOW_CRUISE_TRIGGER_M), float(required_m) + 0.20, hard_front_m + 0.20)
            if bool(follow_cruise)
            else max(float(required_m) + 0.30, hard_front_m + 0.20)
        )
        pivot_warning_m = (
            max(float(FOLLOW_CRUISE_TRIGGER_M), float(required_m) + 0.30)
            if bool(follow_cruise)
            else max(float(required_m) + 0.45, 1.05)
        )
        side_required_m = (
            max(float(FOLLOW_CRUISE_SIDE_REQUIRED_M), float(self.cfg.min_clearance_m) + 0.08)
            if bool(follow_cruise)
            else max(0.45, float(self.cfg.min_clearance_m) + 0.15)
        )
        left_m, left_source = _side_clearance_m(lidar_summary, "left")
        right_m, right_source = _side_clearance_m(lidar_summary, "right")
        wall_avoid_start_m = max(
            float(side_required_m),
            float(self.cfg.room_cruise_wall_avoid_start_m),
        )
        wall_avoid_asymmetry_m = float(self.cfg.room_cruise_wall_avoid_asymmetry_m)
        wall_side = ""
        wall_escape_side = ""
        wall_measured_m: Optional[float] = None
        if bool(follow_cruise) and left_m is not None and right_m is not None:
            if (
                float(left_m) < wall_avoid_start_m
                and float(right_m) >= float(side_required_m)
                and float(right_m) - float(left_m) >= wall_avoid_asymmetry_m
            ):
                wall_side = "left"
                wall_escape_side = "right"
                wall_measured_m = float(left_m)
            elif (
                float(right_m) < wall_avoid_start_m
                and float(left_m) >= float(side_required_m)
                and float(left_m) - float(right_m) >= wall_avoid_asymmetry_m
            ):
                wall_side = "right"
                wall_escape_side = "left"
                wall_measured_m = float(right_m)
        wall_pressure_m = (
            max(0.0, wall_avoid_start_m - float(wall_measured_m))
            if wall_measured_m is not None
            else 0.0
        )
        wall_pressure_ratio = min(
            1.0,
            wall_pressure_m / max(1e-6, wall_avoid_start_m - float(side_required_m)),
        )
        wall_clearance = {
            "active": bool(wall_escape_side),
            "wall_side": str(wall_side),
            "escape_side": str(wall_escape_side),
            "measured_m": (
                None if wall_measured_m is None else round(float(wall_measured_m), 4)
            ),
            "avoid_start_m": round(float(wall_avoid_start_m), 4),
            "side_required_m": round(float(side_required_m), 4),
            "asymmetry_m": round(float(wall_avoid_asymmetry_m), 4),
            "pressure_m": round(float(wall_pressure_m), 4),
            "pressure_ratio": round(float(wall_pressure_ratio), 4),
        }
        preferred_side = "left" if float(bearing_error) >= 0.0 else "right"
        front_gap = _front_scan_gap_analysis(raw_scan, cache=clearance_cache)
        front_gap_direction = str(front_gap.get("best_direction", "") or "").strip().upper()
        front_gap_side = ""
        if bool(front_gap.get("has_data", False)):
            if front_gap_direction == "LEFT":
                front_gap_side = "left"
            elif front_gap_direction == "RIGHT":
                front_gap_side = "right"
        front_gap_score_delta = 0.0
        if front_gap_side:
            left_score = _float_from_diag(front_gap, "left_open_score") or 0.0
            right_score = _float_from_diag(front_gap, "right_open_score") or 0.0
            front_gap_score_delta = (
                float(left_score) - float(right_score)
                if front_gap_side == "left"
                else float(right_score) - float(left_score)
            )
        front_gap_confident = bool(front_gap_side and front_gap_score_delta >= 0.16)

        plan: Dict[str, Any] = {
            "active": False,
            "reason": "",
            "front_clearance_m": (None if front_m is None else round(float(front_m), 4)),
            "front_clearance_source": str(front_source or ""),
            "front_required_clearance_m": round(float(required_m), 4),
            "front_warning_m": round(float(warning_front_m), 4),
            "pivot_warning_m": round(float(pivot_warning_m), 4),
            "front_hard_m": round(float(hard_front_m), 4),
            "left_clearance_m": (None if left_m is None else round(float(left_m), 4)),
            "left_clearance_source": str(left_source or ""),
            "right_clearance_m": (None if right_m is None else round(float(right_m), 4)),
            "right_clearance_source": str(right_source or ""),
            "side_required_m": round(float(side_required_m), 4),
            "preferred_side": str(preferred_side),
            "motion_style": str(motion_style or ""),
            "front_gap": dict(front_gap),
            "front_gap_side": str(front_gap_side),
            "front_gap_score_delta": round(float(front_gap_score_delta), 4),
            "front_gap_confident": bool(front_gap_confident),
            "wall_clearance": dict(wall_clearance),
        }

        front_blocked_flag = bool(front_clearance_diag.get("blocked_front", False)) or bool(lidar_summary.get("blocked_front", False))
        front_execution_envelope = (
            _room_cruise_front_execution_envelope(
                lidar_summary=lidar_summary,
                obstacle_plan=plan,
                blocked_front=front_blocked_flag,
                effective_max_v_mps=float(effective_max_v_mps),
            )
            if bool(follow_cruise)
            else {
                "active": False,
                "policy": "motion_style_not_follow_cruise",
                "clear": True,
            }
        )
        plan["front_execution_envelope"] = dict(front_execution_envelope)
        front_execution_clear = bool(front_execution_envelope.get("clear", True))
        front_inside_hard = bool(
            front_m is not None
            and (
                float(front_m) <= hard_front_m
                if bool(follow_cruise)
                else float(front_m) < hard_front_m
            )
        )
        pivot_escape_floor_m = (
            max(
                float(FOLLOW_CRUISE_PIVOT_FLOOR_M),
                float(self.cfg.min_clearance_m) - 0.05,
            )
            if bool(follow_cruise)
            else max(0.28, hard_front_m - 0.07)
        )
        plan["pivot_escape_floor_m"] = round(float(pivot_escape_floor_m), 4)
        if front_m is None:
            plan["reason"] = "blocked_front_flag" if front_blocked_flag else "front_clearance_unavailable"
            return plan
        if front_inside_hard and float(front_m) <= pivot_escape_floor_m:
            plan["reason"] = "front_inside_hard_clearance"
            return plan
        if front_inside_hard:
            front_feasible = False
        if front_blocked_flag:
            front_feasible = False
        if (
            bool(front_feasible)
            and bool(front_execution_clear)
            and float(front_m) > warning_front_m
            and not bool(wall_clearance.get("active", False))
        ):
            plan["reason"] = "front_clear_enough"
            return plan

        chosen_side, side_selection = _choose_avoidance_side(
            left_m=left_m,
            right_m=right_m,
            side_required_m=side_required_m,
            preferred_side=preferred_side,
            committed_side=self._avoidance_side,
            front_m=front_m,
            front_gap_side=front_gap_side,
            front_gap_blocked=bool(front_gap.get("front_blocked_by_scan", False)),
            front_gap_confident=bool(front_gap_confident),
            motion_style=motion_style,
            clearance_escape_side=wall_escape_side,
        )
        if not chosen_side:
            plan["reason"] = (
                "forward_execution_envelope_no_escape_side"
                if bool(follow_cruise) and not bool(front_execution_clear)
                else "no_open_side_for_tangent_arc"
            )
            plan["side_selection"] = str(side_selection)
            return plan

        self._avoidance_side = str(chosen_side)

        plan.update(
            {
                "active": True,
                "reason": (
                    (
                        "front_inside_hard_escape_candidate"
                        if bool(follow_cruise)
                        else "front_inside_hard_escape_pivot"
                    )
                    if front_inside_hard
                    else (
                        "forward_execution_envelope_unavailable"
                        if bool(follow_cruise) and not bool(front_execution_clear)
                        else (
                            "side_clearance_tangent_escape"
                            if bool(wall_clearance.get("active", False))
                            else (
                                "front_blocked_tangent_escape"
                                if not bool(front_feasible)
                                else "front_warning_tangent_bias"
                            )
                        )
                    )
                ),
                "mode": (
                    "escape_candidate"
                    if bool(follow_cruise)
                    and (front_inside_hard or front_blocked_flag or not bool(front_execution_clear))
                    else (
                        "tangent_arc"
                        if bool(follow_cruise)
                        else (
                            "heading_pivot"
                            if front_inside_hard
                            or (not bool(front_feasible))
                            or front_blocked_flag
                            or (
                                abs(float(bearing_error)) > self.cfg.rotate_first_threshold_rad
                                and float(front_m) <= pivot_warning_m
                            )
                            else "tangent_arc"
                        )
                    )
                ),
                "side": str(chosen_side),
                "side_selection": str(side_selection),
                "turn_sign": turn_direction_to_omega_sign(chosen_side),
            }
        )
        return plan

    # ------------------------------------------------------------------
    # Public tick — called once per control cycle.
    # ------------------------------------------------------------------

    def tick(
        self,
        *,
        target_pose: Tuple[float, float, float] | None,
        local_path_segment: Dict[str, Any] | None = None,
        path_progress_m: Optional[float] = None,
        lidar_summary: Dict[str, Any],
        ekf_state: Dict[str, Any],
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        source: str = "STATE",
        dt: float = 0.02,
        now_s: Optional[float] = None,
        max_v_override: Optional[float] = None,
        max_omega_override: Optional[float] = None,
        motion_style: str = "",
        clearance_cache: Optional[Dict[Any, Any]] = None,
    ) -> PlannerTickResult:
        """
        Evaluate feasibility towards *target_pose* and produce a proposal.

        Args:
            target_pose: (x_m, y_m, theta_rad) goal, or None → idle.
            lidar_summary: current LIDAR obstacle envelope.
            raw_scan: optional raw LIDAR points for front-gap side selection.
            ekf_state: current EKF state dict (x, y, theta / theta_deg).
            source: motion source label for the proposal.
            dt: control timestep (s).

        Returns:
            PlannerTickResult with a resolver-compatible proposal.
        """
        now_mono = time.monotonic() if now_s is None else float(now_s)
        self._last_tick_mono = now_mono
        motion_style = str(motion_style or "").strip().lower()
        if motion_style != FOLLOW_CRUISE_MOTION_STYLE:
            self._reset_room_cruise_escape()

        if not self.cfg.enabled:
            self._reset_room_cruise_escape()
            return PlannerTickResult(idle=True, diagnostics={"reason": "disabled"})

        # No target → idle.
        if target_pose is None:
            self._blocked_since = None
            self._avoidance_side = ""
            self._reset_room_cruise_escape()
            return PlannerTickResult(idle=True, diagnostics={"reason": "no_target"})

        # ---- Geometry: current pose → target ----
        try:
            cx, cy, ctheta = _extract_pose(ekf_state)
            tx, ty, ttheta = float(target_pose[0]), float(target_pose[1]), float(target_pose[2])
        except (TypeError, ValueError, IndexError):
            return self._blocked_zero_result(source=source, reason="invalid_pose_input")
        if not _finite_values(cx, cy, ctheta, tx, ty, ttheta):
            return self._blocked_zero_result(source=source, reason="nonfinite_pose_input")

        dx_global = tx - cx
        dy_global = ty - cy

        # Robot-frame errors.
        cos_c = math.cos(ctheta)
        sin_c = math.sin(ctheta)
        e_x = cos_c * dx_global + sin_c * dy_global   # along-track (forward)
        e_y = -sin_c * dx_global + cos_c * dy_global   # cross-track (left-positive)
        e_theta = _wrap_angle(ttheta - ctheta)

        dist_xy = math.hypot(e_x, e_y)
        bearing_to_target = math.atan2(dy_global, dx_global)
        bearing_error = _wrap_angle(bearing_to_target - ctheta)
        rotate_first = bool(
            abs(bearing_error) > self.cfg.rotate_first_threshold_rad
            and dist_xy > self.cfg.tolerance_xy_m * 2.0
        )
        primitive_hint_active = bool(local_path_segment) if isinstance(local_path_segment, dict) else False
        rotate_first_forward_arc = bool(rotate_first and not primitive_hint_active)
        if rotate_first:
            motion_direction = "rotate_left" if bearing_error >= 0.0 else "rotate_right"
        elif e_x < -0.01:
            motion_direction = "reverse"
        else:
            motion_direction = "forward"

        # ---- Arrival check ----
        if dist_xy < self.cfg.tolerance_xy_m and abs(e_theta) < self.cfg.tolerance_theta_rad:
            self._blocked_since = None
            self._avoidance_side = ""
            self._reset_room_cruise_escape()
            return PlannerTickResult(
                idle=True,
                arrived=True,
                diagnostics={"reason": "arrived", "dist_m": round(dist_xy, 4), "e_theta_rad": round(e_theta, 4)},
            )

        # ---- Segment length for clearance check ----
        # Use the actual distance to target (clamped to horizon).
        segment_length = min(self.cfg.max_segment_length_m, min(self.cfg.horizon_m, dist_xy))

        # ---- Collision check ----
        feasible, clearance_diag = _segment_clearance_ok(
            segment_length_m=segment_length,
            lidar_summary=lidar_summary,
            min_clearance_m=self.cfg.min_clearance_m,
            clearance_buffer_m=self.cfg.clearance_buffer_m,
            lookahead_cap_m=self.cfg.clearance_lookahead_cap_m,
            motion_direction=motion_direction,
            clearance_cache=clearance_cache,
        )

        # Add target geometry to diagnostics.
        clearance_diag["dist_to_target_m"] = round(dist_xy, 4)
        clearance_diag["e_x_m"] = round(e_x, 4)
        clearance_diag["e_y_m"] = round(e_y, 4)
        clearance_diag["e_theta_rad"] = round(e_theta, 4)

        # Active runtime limits bound every Room Cruise request.  Resolve them
        # before planning so swept-front feasibility is checked against the
        # largest command this tick can actually emit.
        effective_max_v_hint = float(self.cfg.max_v)
        if max_v_override is not None:
            try:
                override_v = float(max_v_override)
            except (TypeError, ValueError):
                override_v = 0.0
            if math.isfinite(override_v) and override_v > 0.0:
                effective_max_v_hint = min(effective_max_v_hint, override_v)
        effective_max_omega_hint = float(self.cfg.max_omega)
        if max_omega_override is not None:
            try:
                override_omega = float(max_omega_override)
            except (TypeError, ValueError):
                override_omega = 0.0
            if math.isfinite(override_omega) and override_omega > 0.0:
                effective_max_omega_hint = min(effective_max_omega_hint, override_omega)

        obstacle_tangent_plan: Dict[str, Any] = {}
        forward_path_feasible = bool(feasible and motion_direction == "forward")
        if rotate_first_forward_arc:
            forward_feasible, forward_clearance_diag = _segment_clearance_ok(
                segment_length_m=segment_length,
                lidar_summary=lidar_summary,
                min_clearance_m=self.cfg.min_clearance_m,
                clearance_buffer_m=self.cfg.clearance_buffer_m,
                lookahead_cap_m=self.cfg.clearance_lookahead_cap_m,
                motion_direction="forward",
                clearance_cache=clearance_cache,
            )
            clearance_diag["forward_arc_clearance"] = forward_clearance_diag
            obstacle_tangent_plan = self._obstacle_tangent_arc_plan(
                lidar_summary=lidar_summary,
                front_clearance_diag=forward_clearance_diag,
                bearing_error=bearing_error,
                front_feasible=forward_feasible,
                effective_max_v_mps=effective_max_v_hint,
                raw_scan=raw_scan,
                motion_style=motion_style,
                clearance_cache=clearance_cache,
            )
            forward_path_feasible = bool(forward_feasible)
        elif motion_direction == "forward":
            obstacle_tangent_plan = self._obstacle_tangent_arc_plan(
                lidar_summary=lidar_summary,
                front_clearance_diag=clearance_diag,
                bearing_error=bearing_error,
                front_feasible=feasible,
                effective_max_v_mps=effective_max_v_hint,
                raw_scan=raw_scan,
                motion_style=motion_style,
                clearance_cache=clearance_cache,
            )
            forward_path_feasible = bool(feasible)

        reverse_feasible = False
        reverse_clearance_diag: Dict[str, Any] = {}
        if motion_style == FOLLOW_CRUISE_MOTION_STYLE and obstacle_tangent_plan:
            reverse_ok, reverse_clearance_diag = _segment_clearance_ok(
                segment_length_m=segment_length,
                lidar_summary=lidar_summary,
                min_clearance_m=self.cfg.min_clearance_m,
                clearance_buffer_m=self.cfg.clearance_buffer_m,
                lookahead_cap_m=self.cfg.clearance_lookahead_cap_m,
                motion_direction="reverse",
                clearance_cache=clearance_cache,
            )
            reverse_feasible = bool(
                reverse_ok
                and reverse_clearance_diag.get("min_dist_m") is not None
                and not bool(reverse_clearance_diag.get("blocked_back", False))
            )
            obstacle_tangent_plan = self._room_cruise_escape_policy(
                plan=obstacle_tangent_plan,
                lidar_summary=lidar_summary,
                pose_xy=(cx, cy),
                now_mono=now_mono,
                forward_feasible=bool(forward_path_feasible),
                reverse_feasible=bool(reverse_feasible),
                reverse_clearance=reverse_clearance_diag,
            )
            clearance_diag["reverse_escape_clearance"] = dict(reverse_clearance_diag)

        # Empty/repetitive room geometry can lose scan-match uniqueness while
        # residual, inlier, coverage and obstacle distance all remain sound.
        # The 2026-07-29 M3 trace showed the last safe precursor at 0.3397,
        # followed by 0.2795 -> 0.2196 -> 0.1809 on a newly straightened
        # command.  Use the existing PIVOT primitive in the committed
        # direction while there is verified all-around room.  A forward-only
        # tighter ARC was live-tested first; the M1 forward-track floor
        # reduced it to 0.175 rad/s and LOW_CONF still recurred.  The pivot
        # stays on the normal TRACK route, not a motor or safety bypass.
        # Safety and matcher gates remain authoritative downstream.
        localization_confidence: Optional[float] = None
        localization_confidence_source = ""
        for confidence_key in (
            "measurement_confidence",
            "candidate_measurement_confidence",
            "candidate_confidence",
            "latest_confidence",
            "lidar_pose_confidence",
        ):
            try:
                candidate_value = float(lidar_summary.get(confidence_key))
            except (TypeError, ValueError):
                continue
            # Zero is an observed fail-closed confidence value, not a missing
            # measurement.  Treat it as the strongest richness-recovery
            # precursor while keeping obstacle clearance and downstream
            # localization/safety gates authoritative.
            if math.isfinite(candidate_value) and candidate_value >= 0.0:
                localization_confidence = float(candidate_value)
                localization_confidence_source = str(confidence_key)
                break
        current_obstacle_mode = str(
            obstacle_tangent_plan.get("mode", "") or ""
        )
        matcher_degenerate = bool(lidar_summary.get("matcher_degenerate", False))
        matcher_timed_out = bool(lidar_summary.get("matcher_timed_out", False))
        matcher_degeneracy_reasons = {
            str(reason or "").strip().lower()
            for reason in list(
                lidar_summary.get("matcher_degeneracy_reasons") or []
            )
        }
        matcher_budget_exceeded = bool(
            matcher_timed_out or "budget_exceeded" in matcher_degeneracy_reasons
        )
        tracking_loss_latched = bool(lidar_summary.get("tracking_loss_latched", False))
        localization_status = str(lidar_summary.get("localization_status", "") or "").strip().lower()
        # Degeneracy/latch/status remain diagnostics, not an alternative
        # motion trigger.  Replay showed degeneracy only after confidence had
        # already crossed 0.35, while the LIVE artifacts do not carry a
        # run-bound precursor series proving otherwise.
        richness_warning = bool(
            localization_confidence is not None
            and float(localization_confidence)
            < float(FOLLOW_CRUISE_LOCALIZATION_PIVOT_ENTER_CONFIDENCE)
            and not matcher_budget_exceeded
        )
        confidence_pivot_active = bool(
            motion_style == FOLLOW_CRUISE_MOTION_STYLE
            and richness_warning
            and motion_direction == "forward"
            and not bool(lidar_summary.get("blocked_front", False))
            and current_obstacle_mode
            not in {
                "heading_pivot",
                "room_cruise_reverse_arc",
                "room_cruise_reverse_straight",
                "room_cruise_stuck_pivot",
            }
        )
        if confidence_pivot_active:
            front_m = _float_from_diag(obstacle_tangent_plan, "front_clearance_m")
            front_hard_m = _float_from_diag(obstacle_tangent_plan, "front_hard_m")
            front_execution = dict(
                obstacle_tangent_plan.get("front_execution_envelope") or {}
            )
            global_m, _global_source = _first_finite_float(
                dict(lidar_summary or {}),
                ("min_dist", "min_clearance_m", "clearance_m"),
            )
            left_m = _float_from_diag(obstacle_tangent_plan, "left_clearance_m")
            right_m = _float_from_diag(obstacle_tangent_plan, "right_clearance_m")
            side_required_m = (
                _float_from_diag(obstacle_tangent_plan, "side_required_m")
                or float(FOLLOW_CRUISE_SIDE_REQUIRED_M)
            )
            committed_side = str(
                obstacle_tangent_plan.get("side")
                or self._avoidance_side
                or ""
            )
            if committed_side not in {"left", "right"}:
                committed_side = (
                    "left"
                    if left_m is not None
                    and (right_m is None or float(left_m) >= float(right_m))
                    else "right"
                )
            committed_clearance_m = left_m if committed_side == "left" else right_m
            confidence_pivot_safe = bool(
                front_m is not None
                and front_hard_m is not None
                and float(front_m) > max(
                    float(front_hard_m) + 0.25,
                    float(FOLLOW_CRUISE_LOCALIZATION_PIVOT_MIN_CLEARANCE_M),
                )
                and bool(front_execution.get("clear", False))
                and global_m is not None
                and float(global_m)
                >= float(FOLLOW_CRUISE_LOCALIZATION_PIVOT_MIN_CLEARANCE_M)
                and committed_clearance_m is not None
                and float(committed_clearance_m)
                >= max(
                    float(side_required_m),
                    float(FOLLOW_CRUISE_LOCALIZATION_PIVOT_MIN_CLEARANCE_M),
                )
            )
            if confidence_pivot_safe:
                self._avoidance_side = str(committed_side)
                obstacle_tangent_plan.update(
                    {
                        "active": True,
                        "mode": "localization_confidence_pivot",
                        "reason": "localization_confidence_richness_pivot",
                        "side": str(committed_side),
                        "side_selection": "localization_confidence_committed_side",
                        "turn_sign": turn_direction_to_omega_sign(committed_side),
                        "localization_confidence": round(
                            float(localization_confidence),
                            6,
                        ),
                        "localization_confidence_source": str(
                            localization_confidence_source
                        ),
                        "matcher_degenerate": bool(matcher_degenerate),
                        "matcher_timed_out": bool(matcher_timed_out),
                        "matcher_budget_exceeded": bool(matcher_budget_exceeded),
                        "tracking_loss_latched": bool(tracking_loss_latched),
                        "localization_status": str(localization_status),
                        "localization_confidence_pivot_enter": float(
                            FOLLOW_CRUISE_LOCALIZATION_PIVOT_ENTER_CONFIDENCE
                        ),
                        "localization_confidence_pivot_min_clearance_m": float(
                            FOLLOW_CRUISE_LOCALIZATION_PIVOT_MIN_CLEARANCE_M
                        ),
                    }
                )

        clearance_diag["obstacle_avoidance"] = obstacle_tangent_plan
        obstacle_mode = str(obstacle_tangent_plan.get("mode", "") or "")
        nonforward_escape_active = bool(
            obstacle_tangent_plan.get("active", False)
            and obstacle_mode
            in {
                "heading_pivot",
                "room_cruise_reverse_arc",
                "room_cruise_reverse_straight",
                "room_cruise_stuck_pivot",
            }
        )
        front_execution_envelope = dict(
            obstacle_tangent_plan.get("front_execution_envelope") or {}
        )
        front_execution_clear = bool(front_execution_envelope.get("clear", True))
        if bool(obstacle_tangent_plan.get("active", False)):
            feasible = True
            clearance_diag["feasible"] = True
            clearance_diag["direction_blocked"] = False
        elif rotate_first_forward_arc and not forward_path_feasible:
            feasible = False
            clearance_diag["feasible"] = False
            clearance_diag["direction_blocked"] = True
        if (
            motion_style == FOLLOW_CRUISE_MOTION_STYLE
            and motion_direction in {"forward", "rotate_left", "rotate_right"}
            and not bool(front_execution_clear)
            and not bool(nonforward_escape_active)
        ):
            feasible = False
            clearance_diag["feasible"] = False
            clearance_diag["direction_blocked"] = True
            clearance_diag["blocked_reason"] = "forward_execution_envelope_unavailable"

        # Hard block: blocked_front when target is ahead (e_x > 0).
        if (self.cfg.blocked_front_zero
                and bool(lidar_summary.get("blocked_front", False))
                and motion_direction == "forward"
                and e_x > -0.01
                and not (
                    bool(obstacle_tangent_plan.get("active", False))
                    and str(obstacle_tangent_plan.get("mode", ""))
                    in {
                        "heading_pivot",
                        "room_cruise_reverse_arc",
                        "room_cruise_reverse_straight",
                        "room_cruise_stuck_pivot",
                    }
                )):
            feasible = False
            clearance_diag["feasible"] = False
            clearance_diag["direction_blocked"] = True

        if not feasible:
            self._total_blocked_ticks += 1
            if self._blocked_since is None:
                self._blocked_since = now_mono
            return PlannerTickResult(
                proposal=make_motion_proposal(
                    name="local_planner_blocked",
                    layer="LOCAL_PLANNER",
                    source=source,
                    command_type="local_planner_segment",
                    execution_mode=execution_mode_for_command("local_planner_segment", "LOCAL_PLANNER"),
                    v_target=0.0,
                    omega_target=0.0,
                    priority=self.cfg.proposal_priority,
                    entry_tier=ENTRY_TIER_PRIMARY,
                    details={
                        "planner": "blocked",
                        "clearance": clearance_diag,
                        "bearing_error_rad": round(bearing_error, 4),
                    },
                ),
                blocked=True,
                diagnostics=clearance_diag,
            )

        # ---- Feasible: compute v/omega towards target/path primitive ----
        self._blocked_since = None
        self._total_passed_ticks += 1

        primitive = dict(local_path_segment or {}) if isinstance(local_path_segment, dict) else {}
        primitive_active = bool(primitive)
        primitive_curvature = 0.0
        primitive_length_m = 0.0
        primitive_progress_m = 0.0
        if primitive_active:
            try:
                primitive_curvature = float(primitive.get("curvature", primitive.get("curvature_m_inv", 0.0)) or 0.0)
            except (TypeError, ValueError):
                primitive_curvature = 0.0
            try:
                primitive_length_m = max(0.0, float(primitive.get("length_m", 0.0) or 0.0))
            except (TypeError, ValueError):
                primitive_length_m = 0.0
            try:
                primitive_progress_m = max(0.0, float(path_progress_m if path_progress_m is not None else primitive.get("progress_m", 0.0) or 0.0))
            except (TypeError, ValueError):
                primitive_progress_m = 0.0

        # A local path is a preferred geometric primitive, not an obstacle or
        # localization-recovery override.  Keep its lifecycle/identity alive,
        # but let the existing follow/cruise escape policy temporarily own the
        # physical primitive whenever it is active.
        obstacle_tangent_active = bool(obstacle_tangent_plan.get("active", False))
        follow_cruise_heading_align = bool(
            motion_style == FOLLOW_CRUISE_MOTION_STYLE
            and not primitive_active
            and abs(e_theta) >= FOLLOW_CRUISE_HEADING_ALIGN_ENTER_RAD
        )
        # Keep the Room Cruise avoidance side committed through temporary
        # clear samples.  _choose_avoidance_side may still switch when the
        # opposite side is materially freer; clearing it every tick made
        # left/right decisions sensitive to one-scan classifier changes.
        if (
            motion_style != FOLLOW_CRUISE_MOTION_STYLE
            and not obstacle_tangent_active
            and not bool(lidar_summary.get("blocked_front", False))
        ):
            self._avoidance_side = ""
        # Room Cruise never pivots merely because a refreshed target changes
        # heading.  Normal alignment remains a shortest-direction forward ARC.
        if obstacle_tangent_active:
            turn_sign = turn_direction_to_omega_sign(obstacle_tangent_plan.get("side"))
            obstacle_mode = str(obstacle_tangent_plan.get("mode", "") or "")
            if obstacle_mode == "localization_confidence_pivot":
                v_out = 0.0
                omega_out = turn_sign * min(
                    effective_max_omega_hint,
                    float(FOLLOW_CRUISE_LOCALIZATION_PIVOT_OMEGA_RAD_S),
                )
                profile_phase = "localization_confidence_pivot"
            elif obstacle_mode in {"heading_pivot", "room_cruise_stuck_pivot"}:
                v_out = 0.0
                omega_out = turn_sign * min(effective_max_omega_hint, 0.35)
                profile_phase = (
                    "room_cruise_stuck_pivot"
                    if obstacle_mode == "room_cruise_stuck_pivot"
                    else "obstacle_heading_pivot"
                )
            elif obstacle_mode == "room_cruise_reverse_arc":
                v_out = -min(effective_max_v_hint, FOLLOW_CRUISE_ARC_MIN_V_MPS)
                desired_omega = max(0.22, min(0.35, 0.26))
                radius_guard_m = 0.09
                omega_radius_cap = abs(v_out) / radius_guard_m if abs(v_out) > 1e-9 else 0.0
                omega_cap = min(effective_max_omega_hint, max(0.0, omega_radius_cap))
                omega_out = turn_sign * min(desired_omega, omega_cap) if omega_cap > 0.0 else 0.0
                profile_phase = "room_cruise_reverse_arc"
            elif obstacle_mode == "room_cruise_reverse_straight":
                v_out = -min(effective_max_v_hint, FOLLOW_CRUISE_ARC_MIN_V_MPS)
                omega_out = 0.0
                profile_phase = "room_cruise_reverse_straight"
            else:
                front_m = _float_from_diag(obstacle_tangent_plan, "front_clearance_m")
                hard_m = _float_from_diag(obstacle_tangent_plan, "front_hard_m")
                if front_m is None:
                    front_m = self.cfg.horizon_m
                if hard_m is None:
                    hard_m = max(0.30, self.cfg.min_clearance_m)
                clearance_room_m = max(0.0, float(front_m) - float(hard_m))
                if motion_style == FOLLOW_CRUISE_MOTION_STYLE:
                    desired_arc_v = max(
                        FOLLOW_CRUISE_ARC_MIN_V_MPS,
                        min(
                            FOLLOW_CRUISE_ARC_MAX_V_MPS,
                            FOLLOW_CRUISE_ARC_MIN_V_MPS + (FOLLOW_CRUISE_ARC_CLEARANCE_GAIN * clearance_room_m),
                        ),
                    )
                    wall_clearance = dict(obstacle_tangent_plan.get("wall_clearance") or {})
                    wall_pressure_ratio = _float_from_diag(wall_clearance, "pressure_ratio") or 0.0
                    if bool(wall_clearance.get("active", False)):
                        # Enter wall avoidance without a requested-speed step:
                        # at acquisition keep the score-limited straight
                        # speed, then continuously converge to the normal ARC
                        # speed as lateral pressure grows.
                        desired_arc_v += (
                            max(0.0, float(effective_max_v_hint) - float(desired_arc_v))
                            * (1.0 - max(0.0, min(1.0, float(wall_pressure_ratio))))
                        )
                else:
                    desired_arc_v = max(0.028, min(0.048, 0.028 + (0.060 * clearance_room_m)))
                v_out = min(effective_max_v_hint, desired_arc_v)
                desired_omega = max(0.22, min(0.35, 0.26 + (0.25 * max(0.0, 0.20 - clearance_room_m))))
                if motion_style == FOLLOW_CRUISE_MOTION_STYLE:
                    wall_clearance = dict(obstacle_tangent_plan.get("wall_clearance") or {})
                    wall_pressure_ratio = _float_from_diag(wall_clearance, "pressure_ratio") or 0.0
                    if bool(wall_clearance.get("active", False)):
                        desired_omega = float(FOLLOW_CRUISE_ARC_OMEGA_EPS_RAD_S) + (
                            (0.35 - float(FOLLOW_CRUISE_ARC_OMEGA_EPS_RAD_S))
                            * max(0.0, min(1.0, float(wall_pressure_ratio)))
                        )
                radius_guard_m = 0.09
                omega_radius_cap = abs(v_out) / radius_guard_m if abs(v_out) > 1e-9 else 0.0
                omega_cap = min(effective_max_omega_hint, max(0.0, omega_radius_cap))
                omega_out = turn_sign * min(desired_omega, omega_cap) if omega_cap > 0.0 else 0.0
                profile_phase = "obstacle_tangent_arc"
            primitive_curvature = 0.0
            primitive_progress_m = 0.0
            primitive_length_m = 0.0
        elif (follow_cruise_heading_align or rotate_first) and not primitive_active:
            if motion_style == FOLLOW_CRUISE_MOTION_STYLE:
                desired_arc_v = max(
                    FOLLOW_CRUISE_ARC_MIN_V_MPS,
                    min(FOLLOW_CRUISE_ARC_MAX_V_MPS, FOLLOW_CRUISE_ROTATE_ARC_GAIN * dist_xy),
                )
            else:
                desired_arc_v = max(0.035, min(0.060, 0.20 * dist_xy))
            v_out = min(effective_max_v_hint, desired_arc_v)
            omega_raw = self.cfg.k_theta * bearing_error
            radius_guard_m = 0.09
            omega_radius_cap = abs(v_out) / radius_guard_m if abs(v_out) > 1e-9 else 0.0
            omega_cap = min(effective_max_omega_hint, max(0.0, omega_radius_cap))
            omega_out = math.copysign(min(abs(omega_raw), omega_cap), omega_raw) if omega_cap > 0.0 else 0.0
            profile_phase = (
                "target_heading_arc"
                if follow_cruise_heading_align
                else "rotate_first_forward_arc"
            )
        elif primitive_active:
            primitive_v_max = primitive.get("v_max")
            primitive_omega_max = primitive.get("omega_max")
            primitive_effective_max_v = effective_max_v_hint = self.cfg.max_v
            primitive_effective_max_omega = effective_max_omega_hint = self.cfg.max_omega
            try:
                if primitive_v_max is not None and float(primitive_v_max) > 0.0:
                    primitive_effective_max_v = min(primitive_effective_max_v, float(primitive_v_max))
            except (TypeError, ValueError):
                pass
            try:
                if primitive_omega_max is not None and float(primitive_omega_max) > 0.0:
                    primitive_effective_max_omega = min(primitive_effective_max_omega, float(primitive_omega_max))
            except (TypeError, ValueError):
                pass
            if max_v_override is not None:
                try:
                    override_v = float(max_v_override)
                    if math.isfinite(override_v) and override_v > 0.0:
                        primitive_effective_max_v = min(primitive_effective_max_v, override_v)
                except (TypeError, ValueError):
                    pass
            if max_omega_override is not None:
                try:
                    override_omega = float(max_omega_override)
                    if math.isfinite(override_omega) and override_omega > 0.0:
                        primitive_effective_max_omega = min(primitive_effective_max_omega, override_omega)
                except (TypeError, ValueError):
                    pass

            progress_ratio = (
                min(1.0, primitive_progress_m / max(1e-6, primitive_length_m))
                if primitive_length_m > 1e-6
                else 0.0
            )
            entry_scale = 0.65 if progress_ratio < 0.18 else 1.0
            curvature_scale = 1.0 / (1.0 + 0.55 * abs(primitive_curvature))
            v_mag = primitive_effective_max_v * min(entry_scale, max(0.35, curvature_scale))
            if motion_style == FOLLOW_CRUISE_MOTION_STYLE and e_x < -0.01:
                # A recovery pivot can leave the old coverage waypoint behind
                # the robot.  End that physical primitive with one zero-hold;
                # RoomCruiseV2 will replan on the next tick.  Reverse remains
                # owned exclusively by the evidence-gated recovery lifecycle.
                v_out = 0.0
                omega_out = 0.0
                profile_phase = "room_cruise_goal_behind_replan_hold"
            else:
                if e_x < -0.01:
                    v_mag *= -1.0
                heading_correction = max(-0.20, min(0.20, (self.cfg.k_theta * e_theta + self.cfg.k_y * e_y)))
                omega_out = (v_mag * primitive_curvature) + heading_correction
                omega_out = max(-primitive_effective_max_omega, min(primitive_effective_max_omega, omega_out))
                v_out = max(-primitive_effective_max_v, min(primitive_effective_max_v, v_mag))
                profile_phase = "arc_entry" if progress_ratio < 0.18 else "arc_stable"
            effective_max_v_hint = primitive_effective_max_v
            effective_max_omega_hint = primitive_effective_max_omega
        else:
            # Proportional drive: along-track error → v, cross-track + heading → omega.
            if dist_xy < self.cfg.tolerance_xy_m and abs(e_theta) >= self.cfg.tolerance_theta_rad:
                v_out = 0.0
                omega_out = self.cfg.k_theta * e_theta
                profile_phase = "heading_align"
            else:
                v_out = self.cfg.k_xy * e_x
                omega_out = self.cfg.k_theta * e_theta + self.cfg.k_y * e_y
                profile_phase = "pose_tracking"

        # ---- Near-obstacle slowdown ----
        min_dist = clearance_diag.get("min_dist_m")
        if (
            motion_style != FOLLOW_CRUISE_MOTION_STYLE
            and min_dist is not None
            and min_dist < self.cfg.horizon_m
            and abs(v_out) > 1e-6
        ):
            closeness = max(0.0, 1.0 - (min_dist / self.cfg.horizon_m))
            scale = 1.0 - closeness * (1.0 - self.cfg.near_obstacle_v_scale)
            v_out *= scale

        # ---- Clamp ----
        effective_max_v = float(locals().get("effective_max_v_hint", self.cfg.max_v))
        if max_v_override is not None:
            try:
                override_v = float(max_v_override)
            except (TypeError, ValueError):
                override_v = 0.0
            if math.isfinite(override_v) and override_v > 0.0:
                effective_max_v = min(effective_max_v, override_v)
        effective_max_omega = float(locals().get("effective_max_omega_hint", self.cfg.max_omega))
        if max_omega_override is not None:
            try:
                override_omega = float(max_omega_override)
            except (TypeError, ValueError):
                override_omega = 0.0
            if math.isfinite(override_omega) and override_omega > 0.0:
                effective_max_omega = min(effective_max_omega, override_omega)

        follow_clearance_policy: Dict[str, Any] = {
            "active": False,
            "applied": False,
            "policy": "inactive",
            "profile_owner": "MotionController.TRACK_REFERENCE_SLEW",
        }
        if motion_style == FOLLOW_CRUISE_MOTION_STYLE and v_out > 0.0:
            follow_clearance_policy = _room_cruise_safe_progress_policy(
                obstacle_plan=obstacle_tangent_plan,
                blocked_front=bool(lidar_summary.get("blocked_front", False)),
                effective_max_v_mps=float(effective_max_v),
            )
            before_clearance_v = float(v_out)
            v_out = min(
                float(v_out),
                float(follow_clearance_policy["clearance_speed_cap_mps"]),
            )
            follow_clearance_policy["applied"] = bool(float(v_out) < before_clearance_v - 1e-9)

        arc_semantics: Dict[str, Any] = {
            "requested_arc_label": bool("arc" in str(profile_phase)),
            "omega_eps_rad_s": float(FOLLOW_CRUISE_ARC_OMEGA_EPS_RAD_S),
            "valid": True,
            "action": "unchanged",
        }
        if motion_style == FOLLOW_CRUISE_MOTION_STYLE and "arc" in str(profile_phase):
            if abs(float(omega_out)) < float(FOLLOW_CRUISE_ARC_OMEGA_EPS_RAD_S):
                arc_semantics.update({"valid": False, "action": "semantic_hold_or_relabel"})
                if profile_phase in {"obstacle_tangent_arc", "room_cruise_reverse_arc"}:
                    v_out = 0.0
                    omega_out = 0.0
                    profile_phase = "room_cruise_invalid_arc_hold"
                    arc_semantics["action"] = "zero_hold"
                elif profile_phase == "arc_entry":
                    profile_phase = "straight_entry"
                    arc_semantics.update({"valid": True, "action": "relabel_straight"})
                elif profile_phase == "arc_stable":
                    profile_phase = "straight_stable"
                    arc_semantics.update({"valid": True, "action": "relabel_straight"})
        arc_semantics["final_phase"] = str(profile_phase)
        arc_semantics["final_arc_label"] = bool("arc" in str(profile_phase))
        v_out = max(-effective_max_v, min(effective_max_v, v_out))
        omega_out = max(-effective_max_omega, min(effective_max_omega, omega_out))

        transition_shaping: Dict[str, Any] = {
            "active": False,
            "applied": False,
            "reason": (
                "motion_controller_track_reference_slew_is_single_profile_owner"
                if motion_style == FOLLOW_CRUISE_MOTION_STYLE
                else "motion_style_not_follow_cruise"
            ),
            "owner": "MotionController.TRACK_REFERENCE_SLEW",
        }

        v_scale = (
            1.0
            if obstacle_tangent_active
            else (round(v_out / max(1e-9, self.cfg.k_xy * e_x), 3) if abs(e_x) > 1e-6 else 1.0)
        )

        proposal = make_motion_proposal(
            name="local_planner_segment",
            layer="LOCAL_PLANNER",
            source=source,
            command_type="local_planner_segment",
            execution_mode=execution_mode_for_command("local_planner_segment", "LOCAL_PLANNER"),
            v_target=v_out,
            omega_target=omega_out,
            priority=self.cfg.proposal_priority,
            entry_tier=ENTRY_TIER_PRIMARY,
            details={
                "planner": "pass",
                "clearance": clearance_diag,
                "v_scale": v_scale,
                "effective_max_v": round(effective_max_v, 4),
                "effective_max_omega": round(effective_max_omega, 4),
                "bearing_error_rad": round(bearing_error, 4),
                "rotate_first": bool(rotate_first),
                "motion_style": str(motion_style or ""),
                "local_path_segment": primitive,
                "obstacle_avoidance": dict(obstacle_tangent_plan or {}),
                "follow_clearance_speed_policy": follow_clearance_policy,
                "arc_semantics": arc_semantics,
                "transition_shaping": transition_shaping,
                "speed_profile": {
                    "phase": str(profile_phase),
                    "curvature": round(float(primitive_curvature), 5),
                    "progress_m": round(float(primitive_progress_m), 4),
                    "length_m": round(float(primitive_length_m), 4),
                },
            },
        )
        return PlannerTickResult(proposal=proposal, diagnostics=clearance_diag)

    # ------------------------------------------------------------------
    # Diagnostics.
    # ------------------------------------------------------------------

    def get_diagnostics(self) -> Dict[str, Any]:
        return {
            "enabled": self.cfg.enabled,
            "blocked_since": self._blocked_since,
            "total_blocked_ticks": self._total_blocked_ticks,
            "total_passed_ticks": self._total_passed_ticks,
        }

    def _blocked_zero_result(self, *, source: str, reason: str) -> PlannerTickResult:
        self._total_blocked_ticks += 1
        if self._blocked_since is None:
            self._blocked_since = time.monotonic()
        diagnostics = {
            "reason": str(reason or "invalid_pose_input"),
            "feasible": False,
            "blocked_front": False,
            "blocked_back": False,
            "direction_blocked": True,
        }
        return PlannerTickResult(
            proposal=make_motion_proposal(
                name="local_planner_invalid_pose",
                layer="LOCAL_PLANNER",
                source=source,
                command_type="local_planner_segment",
                execution_mode=execution_mode_for_command("local_planner_segment", "LOCAL_PLANNER"),
                v_target=0.0,
                omega_target=0.0,
                priority=self.cfg.proposal_priority,
                entry_tier=ENTRY_TIER_PRIMARY,
                details={"planner": "invalid_pose", "reason": diagnostics["reason"]},
            ),
            blocked=True,
            diagnostics=diagnostics,
        )
