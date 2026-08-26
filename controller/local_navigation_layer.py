#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Navigation-intent driven wrapper around the local planner."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from controller.local_planner import FOLLOW_CRUISE_MOTION_STYLE
from controller.motion_resolver import ENTRY_TIER_PRIMARY, make_motion_proposal
from controller.motion_schema import EXEC_MODE_TRACK, execution_mode_for_command
from controller.navigation_intent import (
    NAV_MODE_EXPLORE,
    NAV_MODE_FOLLOW,
    NAV_MODE_GOAL,
    NAV_MODE_HOLD,
    NAV_MODE_ROOM_CRUISE,
    NavigationIntent,
)
from controller.rolling_local_map import RollingLocalMap, enhance_lidar_summary


FOLLOW_CLOSE_DISTANCE_DEADBAND_M = 0.055
FOLLOW_CAMERA_CLOSE_DISTANCE_DEADBAND_M = 0.12
FOLLOW_CAMERA_HEADING_DEADBAND_RAD = 0.18
FOLLOW_CAMERA_HEADING_GAIN = 0.62
FOLLOW_CAMERA_NEAR_STANDOFF_OMEGA_MAX_RAD_S = 0.30
FOLLOW_CLOSE_RETREAT_REAR_MIN_M = 0.65
FOLLOW_CLOSE_RETREAT_GLOBAL_MIN_M = 0.50
FOLLOW_CLOSE_RETREAT_TARGET_MATCH_MARGIN_M = 0.20
FOLLOW_CLOSE_RETREAT_MAX_V_MPS = 0.045
FOLLOW_CLOSE_RETREAT_MIN_V_MPS = 0.012
FOLLOW_DIRECTION_ONLY_STANDOFF_M = 2.0
FOLLOW_DIRECTION_ONLY_ZONE_HEADING_RAD = 0.22
TARGET_SEARCH_IN_PLACE_OMEGA_RAD_S = 0.12
TARGET_SEARCH_IN_PLACE_TRACK_WIDTH_M = 0.175
TARGET_SEARCH_IN_PLACE_TRACK_MIN_MPS = 0.007
TARGET_SEARCH_IN_PLACE_TRACK_MAX_MPS = 0.040
TARGET_SEARCH_PIVOT_MIN_GLOBAL_CLEARANCE_M = 0.55
TARGET_SEARCH_PIVOT_MIN_SIDE_CLEARANCE_M = 0.30


@dataclass
class LocalNavigationResult:
    proposal: Optional[Dict[str, Any]] = None
    diagnostics: Dict[str, Any] = field(default_factory=dict)
    snapshot: Dict[str, Any] = field(default_factory=dict)
    enhanced_lidar_summary: Dict[str, Any] = field(default_factory=dict)
    intent: Optional[NavigationIntent] = None


def _safe_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        if value is None:
            return default
        out = float(value)
    except Exception:
        return default
    if not math.isfinite(out):
        return default
    return float(out)


def _extract_pose(ekf_state: Dict[str, Any]) -> Tuple[float, float, float]:
    src = dict(ekf_state or {})
    x = _safe_float(src.get("x"), 0.0) or 0.0
    y = _safe_float(src.get("y"), 0.0) or 0.0
    theta = _safe_float(src.get("theta"), None)
    if theta is None:
        theta = math.radians(_safe_float(src.get("theta_deg"), 0.0) or 0.0)
    return float(x), float(y), float(theta)


def _compact_snapshot(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    src = dict(snapshot or {})
    return {
        "schema": src.get("schema"),
        "enabled": bool(src.get("enabled", False)),
        "has_data": bool(src.get("has_data", False)),
        "ttl_s": src.get("ttl_s"),
        "radius_m": src.get("radius_m"),
        "observation_count": int(src.get("observation_count", 0) or 0),
        "valid_points": int(src.get("valid_points", 0) or 0),
        "min_dist_m": src.get("min_dist_m"),
        "front_clearance_m": src.get("front_clearance_m"),
        "left_clearance_m": src.get("left_clearance_m"),
        "right_clearance_m": src.get("right_clearance_m"),
        "rear_clearance_m": src.get("rear_clearance_m"),
        "blocked_front": bool(src.get("blocked_front", False)),
        "blocked_back": bool(src.get("blocked_back", False)),
        "oldest_age_s": src.get("oldest_age_s"),
        "newest_age_s": src.get("newest_age_s"),
    }


def _clearance(summary: Dict[str, Any], *keys: str) -> Optional[float]:
    for key in keys:
        value = _safe_float((summary or {}).get(key), None)
        if value is not None and value > 0.0:
            return float(value)
    return None


def _wrap_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def _metadata_target_source(intent: NavigationIntent) -> str:
    return str((intent.metadata or {}).get("target_source") or "").strip().upper()


def _metadata_target_id(intent: NavigationIntent) -> str:
    return str((intent.metadata or {}).get("target_id") or "").strip().lower()


def _metadata_search_side(intent: NavigationIntent) -> str:
    meta = dict(intent.metadata or {})
    raw = str(meta.get("search_side") or meta.get("turn_direction") or "").strip().lower()
    if raw in {"left", "right"}:
        return raw
    target_id = _metadata_target_id(intent)
    if "_right" in target_id:
        return "right"
    return "left"


def _is_target_search_intent(intent: NavigationIntent) -> bool:
    return bool(
        _metadata_target_source(intent) == "CAMERA_SEARCH"
        or str(intent.reason or "") == "target_search_navigation"
        or "camera_target_search" in _metadata_target_id(intent)
    )


def _target_search_in_place_track_reference(omega_cmd: float) -> Tuple[Dict[str, float], float, float]:
    if abs(float(omega_cmd)) <= 1e-9:
        return {}, 0.0, 0.0
    track_mps = abs(float(omega_cmd)) * float(TARGET_SEARCH_IN_PLACE_TRACK_WIDTH_M) * 0.5
    track_mps = max(
        float(TARGET_SEARCH_IN_PLACE_TRACK_MIN_MPS),
        min(float(TARGET_SEARCH_IN_PLACE_TRACK_MAX_MPS), float(track_mps)),
    )
    if float(omega_cmd) < 0.0:
        left_mps, right_mps = float(track_mps), -float(track_mps)
    else:
        left_mps, right_mps = -float(track_mps), float(track_mps)
    equivalent_omega = (float(right_mps) - float(left_mps)) / float(TARGET_SEARCH_IN_PLACE_TRACK_WIDTH_M)
    return {"left_mps": float(left_mps), "right_mps": float(right_mps)}, float(track_mps), float(equivalent_omega)


class LocalNavigationLayer:
    """Convert NavigationIntent plus local obstacle memory into one proposal."""

    def __init__(self, *, local_planner: Any, rolling_map: Optional[RollingLocalMap] = None) -> None:
        self.local_planner = local_planner
        self.rolling_map = rolling_map
        self._last_status: Dict[str, Any] = {}

    def get_diagnostics(self) -> Dict[str, Any]:
        return dict(self._last_status)

    def _explore_target(
        self,
        *,
        ekf_state: Dict[str, Any],
        enhanced_lidar_summary: Dict[str, Any],
    ) -> Tuple[float, float, float]:
        x, y, theta = _extract_pose(ekf_state)
        front_m = _clearance(enhanced_lidar_summary, "min_dist_narrow", "front_clearance_m", "min_dist")
        left_m = _clearance(enhanced_lidar_summary, "left_clearance_m", "avg_left")
        right_m = _clearance(enhanced_lidar_summary, "right_clearance_m", "avg_right")
        if front_m is not None and front_m >= 0.75 and not bool(enhanced_lidar_summary.get("blocked_front", False)):
            heading = theta
            distance = 0.65
        else:
            turn_left = (left_m if left_m is not None else 0.0) >= (right_m if right_m is not None else 0.0)
            heading = theta + (0.70 if turn_left else -0.70)
            distance = 0.16
        return (
            float(x + math.cos(heading) * distance),
            float(y + math.sin(heading) * distance),
            float(heading),
        )

    def _hold_result(
        self,
        *,
        intent: NavigationIntent,
        source: str,
        snapshot: Dict[str, Any],
        enhanced: Dict[str, Any],
        reason: str,
    ) -> LocalNavigationResult:
        details = {
            "provider": "local_navigation_layer",
            "navigation_intent": intent.to_dict(),
            "rolling_local_map": _compact_snapshot(snapshot),
            "planner": "hold",
            "reason": str(reason or "hold"),
        }
        proposal = make_motion_proposal(
            name="local_navigation_hold",
            layer="LOCAL_NAVIGATION",
            source=source,
            command_type="local_planner_segment",
            execution_mode=execution_mode_for_command("local_planner_segment", "LOCAL_NAVIGATION"),
            v_target=0.0,
            omega_target=0.0,
            priority=int(intent.priority),
            entry_tier=ENTRY_TIER_PRIMARY,
            details=details,
        )
        diagnostics = {
            "provider": "local_navigation_layer",
            "active": True,
            "reason": str(reason or "hold"),
            "mode": intent.normalized_mode(),
            "feasible": False,
            "rolling_local_map": _compact_snapshot(snapshot),
        }
        self._last_status = dict(diagnostics)
        return LocalNavigationResult(
            proposal=proposal,
            diagnostics=diagnostics,
            snapshot=snapshot,
            enhanced_lidar_summary=enhanced,
            intent=intent,
        )

    def _target_search_in_place_result(
        self,
        *,
        intent: NavigationIntent,
        source: str,
        snapshot: Dict[str, Any],
        enhanced: Dict[str, Any],
        ekf_state: Dict[str, Any],
    ) -> LocalNavigationResult:
        _x, _y, theta = _extract_pose(ekf_state)
        search_side = _metadata_search_side(intent)
        side_sign = -1.0 if search_side == "right" else 1.0
        goal_theta = _safe_float(intent.goal_theta, None)
        heading_error = _wrap_angle(float(goal_theta) - float(theta)) if goal_theta is not None else 0.0
        if abs(float(heading_error)) < 0.04:
            heading_error = side_sign * 0.55
        omega_sign = 1.0 if float(heading_error) >= 0.0 else -1.0
        max_omega = _safe_float(intent.max_omega_rad_s, 0.35) or 0.35
        max_omega = max(0.0, float(max_omega))
        global_m = _clearance(enhanced, "min_dist", "min_clearance_m", "clearance_m")
        side_m = _clearance(
            enhanced,
            "right_clearance_m" if search_side == "right" else "left_clearance_m",
            "avg_right" if search_side == "right" else "avg_left",
        )
        clearance_hold = bool(
            (global_m is not None and float(global_m) <= TARGET_SEARCH_PIVOT_MIN_GLOBAL_CLEARANCE_M)
            or (side_m is not None and float(side_m) <= TARGET_SEARCH_PIVOT_MIN_SIDE_CLEARANCE_M)
        )
        if clearance_hold or max_omega <= 1e-6:
            omega_cmd = 0.0
            phase = "target_search_hold"
            reason = "target_search_pivot_clearance_hold" if clearance_hold else "target_search_omega_limited_zero"
            feasible = False
        else:
            omega_cmd = float(omega_sign) * min(float(max_omega), float(TARGET_SEARCH_IN_PLACE_OMEGA_RAD_S))
            phase = "target_search_in_place"
            reason = "target_search_in_place"
            feasible = True
        requested_track_reference: Dict[str, float] = {}
        track_speed_mps = 0.0
        track_equivalent_omega = float(omega_cmd)
        if feasible:
            requested_track_reference, track_speed_mps, track_equivalent_omega = (
                _target_search_in_place_track_reference(float(omega_cmd))
            )
            omega_cmd = float(track_equivalent_omega)

        details = {
            "provider": "local_navigation_layer",
            "planner": "target_search_in_place",
            "navigation_intent": intent.to_dict(),
            "rolling_local_map": _compact_snapshot(snapshot),
            "clearance": {
                "global_min_clearance_m": None if global_m is None else round(float(global_m), 4),
                "turn_side_clearance_m": None if side_m is None else round(float(side_m), 4),
                "turn_side": str(search_side),
                "pivot_min_global_clearance_m": float(TARGET_SEARCH_PIVOT_MIN_GLOBAL_CLEARANCE_M),
                "pivot_min_side_clearance_m": float(TARGET_SEARCH_PIVOT_MIN_SIDE_CLEARANCE_M),
            },
            "speed_profile": {
                "phase": str(phase),
                "primitive": "in_place_pivot",
                "turn_side": str(search_side),
                "heading_error_rad": round(float(heading_error), 6),
                "local_navigation_omega_cap_rad_s": round(float(TARGET_SEARCH_IN_PLACE_OMEGA_RAD_S), 6),
                "navigation_intent_max_omega_rad_s": round(float(max_omega), 6),
                "local_navigation_omega_rad_s": round(float(omega_cmd), 6),
                "track_reference_source": "in_place_pivot_track_reference",
                "track_width_m": round(float(TARGET_SEARCH_IN_PLACE_TRACK_WIDTH_M), 6),
                "track_speed_mps": round(float(track_speed_mps), 6),
                "track_min_mps": round(float(TARGET_SEARCH_IN_PLACE_TRACK_MIN_MPS), 6),
                "track_max_mps": round(float(TARGET_SEARCH_IN_PLACE_TRACK_MAX_MPS), 6),
                "track_equivalent_omega_rad_s": round(float(track_equivalent_omega), 6),
                "v_forced_zero": True,
            },
            "requested_track_reference": dict(requested_track_reference),
        }
        proposal = make_motion_proposal(
            name="local_navigation_target_search_in_place",
            layer="LOCAL_NAVIGATION",
            source=source,
            command_type="local_planner_segment",
            execution_mode=EXEC_MODE_TRACK if requested_track_reference else execution_mode_for_command(
                "local_planner_segment",
                "LOCAL_NAVIGATION",
            ),
            v_target=0.0,
            omega_target=float(omega_cmd),
            priority=int(intent.priority),
            entry_tier=ENTRY_TIER_PRIMARY,
            requested_track_reference=requested_track_reference,
            details=details,
        )
        diagnostics = {
            "provider": "local_navigation_layer",
            "active": True,
            "mode": intent.normalized_mode(),
            "feasible": bool(feasible),
            "reason": str(reason),
            "target_search_in_place": True,
            "turn_side": str(search_side),
            "heading_error_rad": round(float(heading_error), 6),
            "omega_target_rad_s": round(float(omega_cmd), 6),
            "max_omega_rad_s": round(float(max_omega), 6),
            "track_reference_source": "in_place_pivot_track_reference" if requested_track_reference else "",
            "requested_track_reference": dict(requested_track_reference),
            "rolling_local_map": _compact_snapshot(snapshot),
        }
        self._last_status = dict(diagnostics)
        return LocalNavigationResult(
            proposal=proposal,
            diagnostics=diagnostics,
            snapshot=snapshot,
            enhanced_lidar_summary=enhanced,
            intent=intent,
        )

    def _follow_result(
        self,
        *,
        intent: NavigationIntent,
        source: str,
        snapshot: Dict[str, Any],
        enhanced: Dict[str, Any],
        ekf_state: Dict[str, Any],
    ) -> LocalNavigationResult:
        x, y, theta = _extract_pose(ekf_state)
        goal_theta = _safe_float(intent.goal_theta, theta)
        bearing_error = _wrap_angle(float(goal_theta if goal_theta is not None else theta) - float(theta))
        distance_to_target = _safe_float((intent.metadata or {}).get("distance_to_target_m"), None)
        standoff = _safe_float(intent.standoff_m, None)
        if standoff is None:
            standoff = 1.0
        if distance_to_target is None:
            target = intent.target_pose()
            if target is not None:
                distance_to_target = math.hypot(float(target[0]) - x, float(target[1]) - y) + float(standoff)
            else:
                distance_to_target = float(standoff)

        max_v = _safe_float(intent.max_v_mps, _safe_float(intent.desired_speed_mps, 0.06)) or 0.06
        max_omega = _safe_float(intent.max_omega_rad_s, 0.35) or 0.35
        front_m = _clearance(enhanced, "min_dist_narrow", "front_clearance_m", "min_dist")
        left_m = _clearance(enhanced, "left_clearance_m", "avg_left")
        right_m = _clearance(enhanced, "right_clearance_m", "avg_right")
        rear_m = _clearance(enhanced, "min_back", "back_clearance_m", "back_clearance", "rear_clearance_m", "rear_clearance")
        global_m = _clearance(enhanced, "min_dist", "min_clearance_m", "clearance_m")
        blocked_front = bool(enhanced.get("blocked_front", False))
        blocked_back = bool(enhanced.get("blocked_back", False))

        distance_error = float(distance_to_target) - float(standoff)
        abs_bearing = abs(float(bearing_error))
        target_source = _metadata_target_source(intent)
        target_zone = str((intent.metadata or {}).get("target_zone") or "").strip().lower()
        if target_zone not in {"left", "center", "right"}:
            target_zone = ""
        direction_only_target = bool(
            target_source == "CAMERA_TARGET"
            and float(standoff) >= float(FOLLOW_DIRECTION_ONLY_STANDOFF_M)
        )
        front_caution_m = 0.68
        front_soft_m = 0.48
        front_hard_m = 0.34
        front_turnout_floor_m = 0.30
        front_hard = bool(blocked_front or (front_m is not None and front_m <= front_hard_m))
        front_soft = bool(not front_hard and front_m is not None and front_m <= front_soft_m)
        front_caution = bool(not front_hard and front_m is not None and front_m <= front_caution_m)
        front_speed_scale = 1.0
        if front_caution:
            front_speed_scale = max(
                0.20,
                min(
                    1.0,
                    (float(front_m) - float(front_soft_m)) / max(1e-6, float(front_caution_m) - float(front_soft_m)),
                ),
            )

        camera_target = bool(target_source == "CAMERA_TARGET")
        close_deadband_m = (
            float(FOLLOW_CAMERA_CLOSE_DISTANCE_DEADBAND_M)
            if camera_target
            else float(FOLLOW_CLOSE_DISTANCE_DEADBAND_M)
        )
        heading_deadband_rad = (
            float(FOLLOW_CAMERA_HEADING_DEADBAND_RAD)
            if camera_target
            else 0.12
        )
        heading_gain = float(FOLLOW_CAMERA_HEADING_GAIN if camera_target else 0.85)
        close_error = max(0.0, -float(distance_error))
        close_retreat_requested = bool(close_error > close_deadband_m)
        global_target_like = bool(
            global_m is not None
            and distance_to_target is not None
            and abs(float(global_m) - float(distance_to_target)) <= FOLLOW_CLOSE_RETREAT_TARGET_MATCH_MARGIN_M
        )
        global_clear_for_retreat = bool(
            global_m is None
            or float(global_m) >= FOLLOW_CLOSE_RETREAT_GLOBAL_MIN_M
            or global_target_like
        )
        rear_clear_for_retreat = bool(
            rear_m is not None
            and float(rear_m) >= FOLLOW_CLOSE_RETREAT_REAR_MIN_M
            and not blocked_back
            and global_clear_for_retreat
        )
        effective_max_omega = float(max_omega)
        if camera_target and abs(float(distance_error)) <= 0.22:
            effective_max_omega = min(effective_max_omega, float(FOLLOW_CAMERA_NEAR_STANDOFF_OMEGA_MAX_RAD_S))
        omega_cmd = max(
            -float(effective_max_omega),
            min(float(effective_max_omega), float(heading_gain) * float(bearing_error)),
        )
        v_cmd = 0.0
        phase = "follow_direct_hold"
        feasible = True
        reason = "follow_direct_hold"
        if direction_only_target:
            direction_error = float(bearing_error)
            if target_zone == "left" and abs(direction_error) < 0.04:
                direction_error = float(FOLLOW_DIRECTION_ONLY_ZONE_HEADING_RAD)
            elif target_zone == "right" and abs(direction_error) < 0.04:
                direction_error = -float(FOLLOW_DIRECTION_ONLY_ZONE_HEADING_RAD)
            if target_zone == "center" or abs(float(direction_error)) < 0.10:
                omega_cmd = 0.0
                phase = "camera_target_center_hold"
                reason = "direction_only_center_hold"
            else:
                omega_cmd = max(-float(max_omega), min(float(max_omega), 0.85 * float(direction_error)))
                phase = "camera_target_in_place_align"
                reason = "direction_only_in_place_align"
            v_cmd = 0.0
            feasible = True
        elif close_retreat_requested and rear_clear_for_retreat:
            bearing_scale = max(0.35, min(1.0, math.cos(min(abs_bearing, 1.20))))
            rear_speed_scale = max(
                0.25,
                min(
                    1.0,
                    (float(rear_m) - FOLLOW_CLOSE_RETREAT_REAR_MIN_M)
                    / max(1e-6, 1.0 - FOLLOW_CLOSE_RETREAT_REAR_MIN_M),
                ),
            )
            retreat_cap = min(float(max_v), FOLLOW_CLOSE_RETREAT_MAX_V_MPS)
            retreat_v = min(
                retreat_cap,
                max(
                    FOLLOW_CLOSE_RETREAT_MIN_V_MPS,
                    0.38 * float(close_error) * float(bearing_scale) * float(rear_speed_scale),
                ),
            )
            v_cmd = -float(retreat_v)
            phase = "follow_close_retreat"
            reason = "follow_distance_retreat"
        elif close_retreat_requested:
            v_cmd = 0.0
            if abs_bearing < 0.12:
                omega_cmd = 0.0
                phase = "follow_close_rear_blocked_hold"
                feasible = False
            else:
                align_omega = float(heading_gain) * float(bearing_error)
                omega_cmd = max(
                    -float(effective_max_omega),
                    min(float(effective_max_omega), align_omega),
                )
                if camera_target and abs(float(omega_cmd)) < 0.10 and float(effective_max_omega) > 0.10:
                    omega_cmd = math.copysign(min(float(effective_max_omega), 0.12), float(bearing_error))
                phase = "follow_close_rear_blocked_heading_align"
                feasible = True
            reason = "follow_close_rear_clearance_hold"
        elif front_hard:
            feasible = False
            omega_cmd = 0.0
            reason = "follow_front_hard_hold"
            phase = "follow_front_hard_hold"
            if (
                not blocked_front
                and front_m is not None
                and float(front_m) > float(front_turnout_floor_m)
                and distance_error > 0.04
            ):
                turn_left = (left_m if left_m is not None else 0.0) >= (right_m if right_m is not None else 0.0)
                omega_cmd = (1.0 if turn_left else -1.0) * min(float(max_omega), 0.12)
                feasible = True
                reason = "follow_front_hard_turnout"
                phase = "follow_front_hard_turnout"
        elif abs_bearing >= 0.55:
            phase = "target_heading_align"
            reason = "follow_heading_align"
        elif distance_error > 0.04 and not front_soft:
            bearing_scale = max(0.25, min(1.0, math.cos(min(abs_bearing, 1.20))))
            approach_v = min(float(max_v), max(0.0, 0.42 * float(distance_error)) * bearing_scale)
            v_cmd = float(approach_v) * float(front_speed_scale)
            if front_caution and distance_error > 0.20 and v_cmd > 0.0:
                v_cmd = min(float(max_v), max(0.024, v_cmd))
            phase = "follow_caution_approach" if front_caution else "follow_direct_approach"
            reason = "follow_distance_approach"
        elif distance_error > 0.04:
            if abs_bearing < 0.08:
                turn_left = (left_m if left_m is not None else 0.0) >= (right_m if right_m is not None else 0.0)
                omega_cmd = (1.0 if turn_left else -1.0) * min(float(max_omega), 0.16)
            phase = "follow_front_soft_turnout"
            reason = "follow_front_soft_turnout"
        elif abs_bearing >= float(heading_deadband_rad):
            phase = "heading_align"
            reason = "follow_standoff_heading_align"
        else:
            omega_cmd = 0.0

        details = {
            "provider": "local_navigation_layer",
            "planner": "follow_direct",
            "navigation_intent": intent.to_dict(),
            "rolling_local_map": _compact_snapshot(snapshot),
            "clearance": {
                "front_clearance_m": None if front_m is None else round(float(front_m), 4),
                "left_clearance_m": None if left_m is None else round(float(left_m), 4),
                "right_clearance_m": None if right_m is None else round(float(right_m), 4),
                "rear_clearance_m": None if rear_m is None else round(float(rear_m), 4),
                "global_min_clearance_m": None if global_m is None else round(float(global_m), 4),
                "blocked_front": bool(blocked_front),
                "blocked_back": bool(blocked_back),
                "front_soft_m": float(front_soft_m),
                "front_caution_m": float(front_caution_m),
                "front_hard_m": float(front_hard_m),
                "front_turnout_floor_m": float(front_turnout_floor_m),
                "front_speed_scale": round(float(front_speed_scale), 4),
                "close_distance_deadband_m": round(float(close_deadband_m), 4),
                "close_retreat_rear_min_m": float(FOLLOW_CLOSE_RETREAT_REAR_MIN_M),
                "close_retreat_global_min_m": float(FOLLOW_CLOSE_RETREAT_GLOBAL_MIN_M),
                "rear_clear_for_retreat": bool(rear_clear_for_retreat),
                "global_clear_for_retreat": bool(global_clear_for_retreat),
                "global_target_like": bool(global_target_like),
            },
            "speed_profile": {
                "phase": str(phase),
                "distance_error_m": round(float(distance_error), 4),
                "close_error_m": round(float(close_error), 4),
                "bearing_error_rad": round(float(bearing_error), 4),
                "heading_deadband_rad": round(float(heading_deadband_rad), 4),
                "heading_gain": round(float(heading_gain), 4),
                "effective_max_omega_rad_s": round(float(effective_max_omega), 4),
                "direction_only_target": bool(direction_only_target),
                "target_zone": str(target_zone),
            },
        }
        proposal = make_motion_proposal(
            name="local_navigation_follow_direct",
            layer="LOCAL_NAVIGATION",
            source=source,
            command_type="local_planner_segment",
            execution_mode=execution_mode_for_command("local_planner_segment", "LOCAL_NAVIGATION"),
            v_target=float(v_cmd),
            omega_target=float(omega_cmd),
            priority=int(intent.priority),
            entry_tier=ENTRY_TIER_PRIMARY,
            details=details,
        )
        diagnostics = {
            "provider": "local_navigation_layer",
            "active": True,
            "mode": NAV_MODE_FOLLOW,
            "feasible": bool(feasible),
            "reason": str(reason),
            "distance_to_target_m": round(float(distance_to_target), 4),
            "standoff_m": round(float(standoff), 4),
            "distance_error_m": round(float(distance_error), 4),
            "close_error_m": round(float(close_error), 4),
            "bearing_error_rad": round(float(bearing_error), 4),
            "rear_clear_for_retreat": bool(rear_clear_for_retreat),
            "global_clear_for_retreat": bool(global_clear_for_retreat),
            "rolling_local_map": _compact_snapshot(snapshot),
        }
        self._last_status = dict(diagnostics)
        return LocalNavigationResult(
            proposal=proposal,
            diagnostics=diagnostics,
            snapshot=snapshot,
            enhanced_lidar_summary=enhanced,
            intent=intent,
        )

    def tick_intent(
        self,
        intent: NavigationIntent,
        *,
        lidar_summary: Dict[str, Any],
        ekf_state: Dict[str, Any],
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        source: Optional[str] = None,
        dt: float = 0.02,
        update_map: bool = True,
        now_s: Optional[float] = None,
        local_path_segment: Optional[Dict[str, Any]] = None,
        path_progress_m: Optional[float] = None,
        tick_context: Any = None,
        clearance_cache: Optional[Dict[Any, Any]] = None,
    ) -> LocalNavigationResult:
        if not isinstance(intent, NavigationIntent):
            raise TypeError("intent must be NavigationIntent")

        src = str(source or intent.source or "STATE")
        if not intent.active:
            snapshot = {"enabled": bool(self.rolling_map is not None), "has_data": False, "raw_scan": []}
            compact_map = _compact_snapshot(snapshot)
            diagnostics = {
                "provider": "local_navigation_layer",
                "active": False,
                "reason": "intent_inactive",
                "mode": intent.normalized_mode(),
                "rolling_local_map": compact_map,
            }
            self._last_status = dict(diagnostics)
            enhanced = enhance_lidar_summary(dict(lidar_summary or {}), snapshot)
            return LocalNavigationResult(None, diagnostics, snapshot, enhanced, intent)

        if self.rolling_map is not None and update_map:
            self.rolling_map.update(
                raw_scan=raw_scan,
                lidar_summary=dict(lidar_summary or {}),
                ekf_state=dict(ekf_state or {}),
                now_s=now_s,
            )
        if self.rolling_map is not None:
            snapshot = self.rolling_map.snapshot(dict(ekf_state or {}), now_s=now_s, include_raw_scan=True)
        else:
            snapshot = {"enabled": False, "has_data": False, "raw_scan": []}
        enhanced = enhance_lidar_summary(dict(lidar_summary or {}), snapshot)
        compact_map = _compact_snapshot(snapshot)

        mode = intent.normalized_mode()
        if mode == NAV_MODE_HOLD:
            return self._hold_result(intent=intent, source=src, snapshot=snapshot, enhanced=enhanced, reason="navigation_hold")
        if mode == NAV_MODE_FOLLOW:
            return self._follow_result(
                intent=intent,
                source=src,
                snapshot=snapshot,
                enhanced=enhanced,
                ekf_state=dict(ekf_state or {}),
            )
        if _is_target_search_intent(intent):
            return self._target_search_in_place_result(
                intent=intent,
                source=src,
                snapshot=snapshot,
                enhanced=enhanced,
                ekf_state=dict(ekf_state or {}),
            )

        target_pose = intent.target_pose()
        if target_pose is None and mode in {NAV_MODE_EXPLORE, NAV_MODE_ROOM_CRUISE}:
            target_pose = self._explore_target(ekf_state=dict(ekf_state or {}), enhanced_lidar_summary=enhanced)

        if target_pose is None:
            diagnostics = {
                "provider": "local_navigation_layer",
                "active": False,
                "reason": "target_pose_missing",
                "mode": mode,
                "rolling_local_map": compact_map,
            }
            self._last_status = dict(diagnostics)
            return LocalNavigationResult(None, diagnostics, snapshot, enhanced, intent)

        max_v = _safe_float(intent.max_v_mps, None)
        if max_v is None:
            max_v = _safe_float(intent.desired_speed_mps, None)
        max_omega = _safe_float(intent.max_omega_rad_s, None)
        if mode in {NAV_MODE_ROOM_CRUISE, NAV_MODE_FOLLOW}:
            motion_style = FOLLOW_CRUISE_MOTION_STYLE
        elif mode == NAV_MODE_EXPLORE:
            motion_style = "explore"
        elif mode == NAV_MODE_GOAL:
            motion_style = "goal"
        else:
            motion_style = str(mode).lower()

        snapshot_raw_scan = snapshot.get("raw_scan") if isinstance(snapshot, dict) else None
        planner_raw_scan = (
            snapshot_raw_scan
            if isinstance(snapshot_raw_scan, list) and snapshot_raw_scan
            else (raw_scan if isinstance(raw_scan, list) else list(raw_scan or []))
        )
        planner_result = self.local_planner.tick(
            target_pose=target_pose,
            local_path_segment=local_path_segment,
            path_progress_m=path_progress_m,
            lidar_summary=enhanced,
            ekf_state=dict(ekf_state or {}),
            raw_scan=planner_raw_scan,
            source=src,
            dt=dt,
            now_s=now_s,
            max_v_override=max_v,
            max_omega_override=max_omega,
            motion_style=motion_style,
            clearance_cache=clearance_cache,
        )

        diagnostics = dict(getattr(planner_result, "diagnostics", {}) or {})
        diagnostics.update(
            {
                "provider": "local_navigation_layer",
                "active": bool(getattr(planner_result, "proposal", None) is not None),
                "mode": mode,
                "intent": intent.to_dict(),
                "rolling_local_map": compact_map,
            }
        )

        proposal = getattr(planner_result, "proposal", None)
        if proposal is not None:
            proposal = dict(proposal)
            details = dict(proposal.get("details") or {})
            details.update(
                {
                    "provider": "local_navigation_layer",
                    "navigation_intent": intent.to_dict(),
                    "rolling_local_map": compact_map,
                }
            )
            proposal["layer"] = "LOCAL_NAVIGATION"
            proposal["source"] = src
            proposal["priority"] = int(intent.priority)
            proposal["details"] = details

        self._last_status = dict(diagnostics)
        return LocalNavigationResult(
            proposal=proposal,
            diagnostics=diagnostics,
            snapshot=snapshot,
            enhanced_lidar_summary=enhanced,
            intent=intent,
        )
