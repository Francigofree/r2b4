#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cruise Layer v2: NavigationIntent -> LocalNavigationLayer.

This layer is intentionally narrow. It never creates PWM. Normal cruise/follow
motion must leave this layer as a local_planner_segment proposal owned by
LocalNavigationLayer; target-search pivot may enforce the same in-place track
reference contract as a guard.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from controller.follow_types import (
    FollowRequest,
    TARGET_SOURCE_CAMERA_SEARCH,
    TARGET_SOURCE_CAMERA_TARGET,
    safe_float,
)
from controller.navigation_intent import (
    NAV_MODE_FOLLOW,
    NAV_MODE_GOAL,
    NavigationIntent,
)
from controller.follow_motion_profile import FollowMotionProfile
from controller.motion_schema import EXEC_MODE_TRACK, normalize_execution_mode


CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE = "room_cruise_v2"
CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW = "human_follow_v2"
CRUISE_LAYER_V2_PROVIDER = "cruise_layer_v2"
CRUISE_LAYER_V2_ALLOWED_COMMAND = "local_planner_segment"
CRUISE_LAYER_V2_ALLOWED_LAYER = "LOCAL_NAVIGATION"
CRUISE_LAYER_V2_IN_PLACE_V_EPS_MPS = 0.006
CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S = 0.035
CRUISE_LAYER_V2_SEARCH_HEADING_DELTA_RAD = 0.55
CRUISE_LAYER_V2_SEARCH_PIVOT_OMEGA_FALLBACK_RAD_S = 0.12
CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_WIDTH_M = 0.175
CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_MIN_MPS = 0.150
CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_MAX_MPS = 0.150
CRUISE_LAYER_V2_ROOM_TRACK_WIDTH_M = 0.175
CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS = 0.150
CRUISE_LAYER_V2_ROOM_ARC_TRACK_DIFF_MIN_MPS = 0.036
CRUISE_LAYER_V2_ROOM_TRACK_MAX_MPS = 0.360
CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MIN_MPS = 0.150
CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MAX_MPS = 0.150
CRUISE_LAYER_V2_CAMERA_CENTER_PHASE_RAD = 0.18


@dataclass
class CruiseLayerV2Result:
    proposal: Optional[Dict[str, Any]] = None
    status: Dict[str, Any] = field(default_factory=dict)
    intent: Optional[NavigationIntent] = None


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    if not math.isfinite(out):
        return float(default)
    return float(out)


def _wrap_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2.0 * math.pi) - math.pi


def _extract_pose(ekf_state: Dict[str, Any]) -> tuple[float, float, float]:
    src = dict(ekf_state or {})
    x = _safe_float(src.get("x"), 0.0)
    y = _safe_float(src.get("y"), 0.0)
    theta = src.get("theta")
    if theta is None:
        theta = math.radians(_safe_float(src.get("theta_deg"), 0.0))
    return float(x), float(y), _safe_float(theta, 0.0)


def _compact_map(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    src = dict(snapshot or {})
    return {
        "schema": src.get("schema"),
        "enabled": bool(src.get("enabled", False)),
        "has_data": bool(src.get("has_data", False)),
        "valid_points": int(src.get("valid_points", 0) or 0),
        "observation_count": int(src.get("observation_count", 0) or 0),
        "front_clearance_m": src.get("front_clearance_m"),
        "left_clearance_m": src.get("left_clearance_m"),
        "right_clearance_m": src.get("right_clearance_m"),
        "rear_clearance_m": src.get("rear_clearance_m"),
        "min_dist_m": src.get("min_dist_m"),
        "blocked_front": bool(src.get("blocked_front", False)),
        "blocked_back": bool(src.get("blocked_back", False)),
    }


def _positive_omega_limit(value: Any, default: float) -> float:
    limit = _safe_float(value, default)
    if limit <= 0.0:
        limit = float(default)
    return max(0.01, float(limit))


def _search_direction(request: FollowRequest) -> float:
    target_id = str(getattr(request, "target_id", "") or "").lower()
    if "_right" in target_id:
        return -1.0
    return 1.0


def _search_side(request: FollowRequest) -> str:
    target_id = str(getattr(request, "target_id", "") or "").lower()
    if "_right" in target_id:
        return "right"
    if "_left" in target_id:
        return "left"
    zone = str(getattr(request, "target_zone", "") or "").strip().lower()
    return zone if zone in {"left", "right"} else "left"


def _intent_search_side(intent: NavigationIntent) -> str:
    meta = dict(intent.metadata or {})
    raw = str(meta.get("search_side") or meta.get("turn_direction") or "").strip().lower()
    if raw in {"left", "right"}:
        return raw
    target_id = str(meta.get("target_id") or "").strip().lower()
    if "_right" in target_id:
        return "right"
    return "left"


def _target_search_track_reference_for_omega(
    omega_cmd: float,
    *,
    track_width_m: float = CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_WIDTH_M,
) -> tuple[Dict[str, float], float, float]:
    if abs(float(omega_cmd)) <= 1e-9:
        return {}, 0.0, 0.0
    width = max(0.01, _safe_float(track_width_m, CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_WIDTH_M))
    track_mps = abs(float(omega_cmd)) * width * 0.5
    track_mps = max(
        float(CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_MIN_MPS),
        min(float(CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_MAX_MPS), float(track_mps)),
    )
    if float(omega_cmd) < 0.0:
        left_mps, right_mps = float(track_mps), -float(track_mps)
    else:
        left_mps, right_mps = -float(track_mps), float(track_mps)
    equivalent_omega = (
        float(right_mps) - float(left_mps)
    ) / width
    return {"left_mps": float(left_mps), "right_mps": float(right_mps)}, float(track_mps), float(equivalent_omega)


def _room_cruise_track_reference_for_twist(
    v_cmd: float,
    omega_cmd: float,
    *,
    track_width_m: float = CRUISE_LAYER_V2_ROOM_TRACK_WIDTH_M,
) -> tuple[Dict[str, float], float, float, str, float, Dict[str, Any]]:
    v = _safe_float(v_cmd, 0.0)
    omega = _safe_float(omega_cmd, 0.0)
    adjustment: Dict[str, Any] = {
        "applied": False,
        "reason": "",
        "forward_track_min_mps": float(CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS),
        "arc_track_diff_min_mps": float(CRUISE_LAYER_V2_ROOM_ARC_TRACK_DIFF_MIN_MPS),
        "track_max_mps": float(CRUISE_LAYER_V2_ROOM_TRACK_MAX_MPS),
        "source": "M1_motion_profile:motion_readiness.forward_min_command+heading_turn",
    }
    if abs(v) < 1e-6 and abs(omega) < 1e-6:
        return (
            {"left_mps": 0.0, "right_mps": 0.0},
            0.0,
            0.0,
            "m3_room_cruise_zero_track_hold",
            0.0,
            {
                **adjustment,
                "reason": "room_cruise_zero_track_continuity",
                "source": "normal_room_cruise_hold_keeps_TRACK_EXEC",
            },
        )

    width = max(0.01, _safe_float(track_width_m, CRUISE_LAYER_V2_ROOM_TRACK_WIDTH_M))
    if abs(v) <= CRUISE_LAYER_V2_IN_PLACE_V_EPS_MPS and abs(omega) >= CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S:
        track_mps = abs(float(omega)) * width * 0.5
        track_mps = max(
            float(CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MIN_MPS),
            min(float(CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MAX_MPS), float(track_mps)),
        )
        if omega < 0.0:
            left_mps, right_mps = float(track_mps), -float(track_mps)
        else:
            left_mps, right_mps = -float(track_mps), float(track_mps)
        equivalent_omega = (float(right_mps) - float(left_mps)) / width
        return (
            {"left_mps": float(left_mps), "right_mps": float(right_mps)},
            0.0,
            float(equivalent_omega),
            "m3_room_cruise_in_place_pivot",
            float(track_mps),
            {
                **adjustment,
                "applied": abs(float(track_mps) - abs(float(omega)) * width * 0.5) > 1e-9,
                "reason": "heading_turn_pivot_track_range",
                "source": "vezerles.heading_turn.pivot_track_min_max",
                "pivot_track_min_mps": float(CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MIN_MPS),
                "pivot_track_max_mps": float(CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MAX_MPS),
            },
        )

    left_mps = float(v) - float(omega) * width * 0.5
    right_mps = float(v) + float(omega) * width * 0.5
    original_left_mps = float(left_mps)
    original_right_mps = float(right_mps)

    same_forward_arc = bool(
        float(v) > 0.0
        and left_mps > 0.0
        and right_mps > 0.0
        and abs(float(omega)) >= CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S
    )
    same_forward_straight = bool(
        float(v) > 0.0
        and left_mps > 0.0
        and right_mps > 0.0
        and abs(float(omega)) < CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S
    )
    same_reverse_arc = bool(
        float(v) < 0.0
        and left_mps < 0.0
        and right_mps < 0.0
        and abs(float(omega)) >= CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S
    )
    same_reverse_straight = bool(
        float(v) < 0.0
        and left_mps < 0.0
        and right_mps < 0.0
        and abs(float(omega)) < CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S
    )
    if same_forward_arc:
        turn_left = bool(float(omega) > 0.0)
        floor_mps = float(CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS)
        diff_min = float(CRUISE_LAYER_V2_ROOM_ARC_TRACK_DIFF_MIN_MPS)
        track_max = max(floor_mps + diff_min, float(CRUISE_LAYER_V2_ROOM_TRACK_MAX_MPS))
        inner = float(left_mps if turn_left else right_mps)
        outer = float(right_mps if turn_left else left_mps)
        inner = max(inner, floor_mps)
        outer = max(outer, inner + diff_min)
        if outer > track_max:
            outer = track_max
            inner = min(inner, max(floor_mps, outer - diff_min))
        if turn_left:
            left_mps, right_mps = float(inner), float(outer)
        else:
            left_mps, right_mps = float(outer), float(inner)
        adjustment.update(
            {
                "applied": abs(left_mps - original_left_mps) > 1e-9 or abs(right_mps - original_right_mps) > 1e-9,
                "reason": "m1_forward_arc_track_contract",
                "original_left_mps": float(original_left_mps),
                "original_right_mps": float(original_right_mps),
            }
        )
    elif same_forward_straight:
        floor_mps = float(CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS)
        if 0.0 < min(left_mps, right_mps) < floor_mps:
            left_mps = max(left_mps, floor_mps)
            right_mps = max(right_mps, floor_mps)
            adjustment.update(
                {
                    "applied": True,
                    "reason": "m1_forward_straight_track_contract",
                    "original_left_mps": float(original_left_mps),
                    "original_right_mps": float(original_right_mps),
                }
            )
    elif same_reverse_arc:
        turn_left = bool(float(omega) > 0.0)
        floor_mps = float(CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS)
        diff_min = float(CRUISE_LAYER_V2_ROOM_ARC_TRACK_DIFF_MIN_MPS)
        track_max = max(floor_mps + diff_min, float(CRUISE_LAYER_V2_ROOM_TRACK_MAX_MPS))
        inner_mag = abs(float(right_mps if turn_left else left_mps))
        outer_mag = abs(float(left_mps if turn_left else right_mps))
        inner_mag = max(inner_mag, floor_mps)
        outer_mag = max(outer_mag, inner_mag + diff_min)
        if outer_mag > track_max:
            outer_mag = track_max
            inner_mag = min(inner_mag, max(floor_mps, outer_mag - diff_min))
        if turn_left:
            left_mps, right_mps = -float(outer_mag), -float(inner_mag)
        else:
            left_mps, right_mps = -float(inner_mag), -float(outer_mag)
        adjustment.update(
            {
                "applied": abs(left_mps - original_left_mps) > 1e-9 or abs(right_mps - original_right_mps) > 1e-9,
                "reason": "m1_reverse_arc_track_contract",
                "original_left_mps": float(original_left_mps),
                "original_right_mps": float(original_right_mps),
            }
        )
    elif same_reverse_straight:
        floor_mps = float(CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS)
        if 0.0 < min(abs(left_mps), abs(right_mps)) < floor_mps:
            left_mps = -max(abs(left_mps), floor_mps)
            right_mps = -max(abs(right_mps), floor_mps)
            adjustment.update(
                {
                    "applied": True,
                    "reason": "m1_reverse_straight_track_contract",
                    "original_left_mps": float(original_left_mps),
                    "original_right_mps": float(original_right_mps),
                }
            )
    v_equiv = 0.5 * (float(left_mps) + float(right_mps))
    omega_equiv = (float(right_mps) - float(left_mps)) / width
    return (
        {"left_mps": float(left_mps), "right_mps": float(right_mps)},
        float(v_equiv),
        float(omega_equiv),
        "m3_room_cruise_diff_track",
        max(abs(float(left_mps)), abs(float(right_mps))),
        adjustment,
    )


def _ensure_room_cruise_m3_track_motion(
    proposal: Dict[str, Any],
    *,
    track_width_m: float = CRUISE_LAYER_V2_ROOM_TRACK_WIDTH_M,
) -> Dict[str, Any]:
    out = dict(proposal or {})
    track_ref, v_target, omega_target, source, track_speed_mps, track_adjustment = _room_cruise_track_reference_for_twist(
        _safe_float(out.get("v_target"), 0.0),
        _safe_float(out.get("omega_target"), 0.0),
        track_width_m=track_width_m,
    )
    if not track_ref:
        out["requested_track_reference"] = {}
        out["execution_mode"] = normalize_execution_mode(out.get("execution_mode", ""))
        return out

    out["v_target"] = float(v_target)
    out["omega_target"] = float(omega_target)
    out["execution_mode"] = EXEC_MODE_TRACK
    out["requested_track_reference"] = dict(track_ref)
    details = dict(out.get("details") or {})
    speed_profile = dict(details.get("speed_profile") or {})
    speed_profile.update(
        {
            "track_reference_source": str(source),
            "m3_motion_route": "TRACK_EXEC",
            "track_width_m": round(float(track_width_m), 6),
            "track_speed_mps": round(float(track_speed_mps), 6),
            "track_floor_mps": round(float(CRUISE_LAYER_V2_ROOM_FORWARD_TRACK_MIN_MPS), 6),
            "track_diff_min_mps": round(float(CRUISE_LAYER_V2_ROOM_ARC_TRACK_DIFF_MIN_MPS), 6),
            "track_min_mps": round(float(CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MIN_MPS), 6),
            "track_max_mps": round(float(CRUISE_LAYER_V2_ROOM_PIVOT_TRACK_MAX_MPS), 6),
            "track_equivalent_omega_rad_s": round(float(omega_target), 6),
            "track_reference_adjustment": dict(track_adjustment or {}),
        }
    )
    details["requested_track_reference"] = dict(track_ref)
    details["m3_track_reference_guard"] = {
        "active": True,
        "reason": str(source),
        "execution_mode": EXEC_MODE_TRACK,
        "requested_track_reference": dict(track_ref),
    }
    details["speed_profile"] = speed_profile
    out["details"] = details
    return out


def _ensure_target_search_pivot_motion(
    proposal: Dict[str, Any],
    *,
    intent: NavigationIntent,
    local_diag: Dict[str, Any],
    track_width_m: float = CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_WIDTH_M,
) -> Dict[str, Any]:
    out = dict(proposal or {})
    meta = dict(intent.metadata or {})
    target_source = str(meta.get("target_source") or "").strip().upper()
    is_search = bool(
        target_source == TARGET_SOURCE_CAMERA_SEARCH
        or str(intent.reason or "") == "target_search_navigation"
        or bool(local_diag.get("target_search_in_place", False))
    )
    if not is_search or not bool(local_diag.get("feasible", True)):
        return out

    v_target = _safe_float(out.get("v_target"), 0.0)
    omega_target = _safe_float(out.get("omega_target"), 0.0)
    out["v_target"] = 0.0
    search_side = _intent_search_side(intent)
    zero_omega_recovered = False
    source_omega = float(omega_target)
    if abs(float(source_omega)) < CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S:
        sign = -1.0 if search_side == "right" else 1.0
        max_omega = _safe_float(intent.max_omega_rad_s, CRUISE_LAYER_V2_SEARCH_PIVOT_OMEGA_FALLBACK_RAD_S)
        if max_omega <= CRUISE_LAYER_V2_IN_PLACE_OMEGA_EPS_RAD_S:
            max_omega = CRUISE_LAYER_V2_SEARCH_PIVOT_OMEGA_FALLBACK_RAD_S
        source_omega = sign * min(
            float(max_omega),
            float(CRUISE_LAYER_V2_SEARCH_PIVOT_OMEGA_FALLBACK_RAD_S),
        )
        zero_omega_recovered = True
    else:
        max_omega = abs(float(source_omega))

    track_ref, track_speed_mps, track_equivalent_omega = _target_search_track_reference_for_omega(
        float(source_omega),
        track_width_m=track_width_m,
    )
    if track_ref:
        out["omega_target"] = float(track_equivalent_omega)
        out["execution_mode"] = EXEC_MODE_TRACK
        out["requested_track_reference"] = dict(track_ref)
    else:
        out["omega_target"] = float(source_omega)
    details = dict(out.get("details") or {})
    speed_profile = dict(details.get("speed_profile") or {})
    speed_profile.update(
        {
            "phase": str(speed_profile.get("phase") or "target_search_in_place"),
            "primitive": "in_place_pivot",
            "turn_side": str(search_side),
            "v_forced_zero": True,
            "target_search_zero_omega_recovered": bool(zero_omega_recovered),
            "omega_before_recovery_rad_s": round(float(omega_target), 6),
            "omega_before_track_reference_rad_s": round(float(source_omega), 6),
            "omega_recovery_source": CRUISE_LAYER_V2_PROVIDER,
            "omega_recovered_rad_s": round(float(source_omega), 6),
            "navigation_intent_max_omega_rad_s": round(float(max_omega), 6),
            "track_reference_source": "in_place_pivot_track_reference",
            "track_width_m": round(float(track_width_m), 6),
            "track_speed_mps": round(float(track_speed_mps), 6),
            "track_min_mps": round(float(CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_MIN_MPS), 6),
            "track_max_mps": round(float(CRUISE_LAYER_V2_SEARCH_PIVOT_TRACK_MAX_MPS), 6),
            "track_equivalent_omega_rad_s": round(float(track_equivalent_omega), 6),
        }
    )
    details["requested_track_reference"] = dict(track_ref)
    details["speed_profile"] = speed_profile
    details["target_search_pivot_guard"] = {
        "active": True,
        "reason": (
            "successful_target_search_pivot_had_zero_omega"
            if zero_omega_recovered
            else "target_search_pivot_track_reference_enforced"
        ),
        "v_before_recovery_mps": round(float(v_target), 6),
        "omega_before_recovery_rad_s": round(float(omega_target), 6),
        "omega_recovered_rad_s": round(float(source_omega), 6),
        "track_equivalent_omega_rad_s": round(float(track_equivalent_omega), 6),
        "requested_track_reference": dict(track_ref),
    }
    out["details"] = details
    return out


def _target_geometry(request: Optional[FollowRequest], ekf_state: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(request, FollowRequest):
        return {}
    robot_x, robot_y, robot_theta = _extract_pose(ekf_state)
    tx = safe_float(getattr(request, "target_x", None), None)
    ty = safe_float(getattr(request, "target_y", None), None)
    bearing = None
    if tx is not None and ty is not None:
        bearing = _wrap_angle(math.atan2(float(ty) - robot_y, float(tx) - robot_x) - robot_theta)
    return {
        "distance_m": safe_float(getattr(request, "distance_to_target_m", None), None),
        "desired_distance_m": safe_float(getattr(request, "desired_distance_m", None), None),
        "bearing_error_rad": bearing,
        "target_source": str(getattr(request, "target_source", "") or ""),
        "target_id": str(getattr(request, "target_id", "") or ""),
    }


def _follow_phase(request: Optional[FollowRequest], ekf_state: Dict[str, Any], speed_profile: Dict[str, Any]) -> str:
    if not isinstance(request, FollowRequest):
        return str(speed_profile.get("phase") or "cruise_v2")
    target_source = str(getattr(request, "target_source", "") or "")
    target_id = str(getattr(request, "target_id", "") or "")
    reason = str(getattr(request, "reason", "") or "")
    if target_source == TARGET_SOURCE_CAMERA_SEARCH or reason == "target_search_scan":
        local_phase = str(speed_profile.get("phase") or "")
        if local_phase in {"target_search_hold", "target_search_in_place"}:
            return local_phase
        return "target_search_in_place"
    if target_source == TARGET_SOURCE_CAMERA_TARGET and "reacquire" in target_id:
        return "target_reacquire_hold"
    if target_source == TARGET_SOURCE_CAMERA_TARGET and "persisted" in target_id:
        return "target_hold"

    geometry = _target_geometry(request, ekf_state)
    distance = safe_float(geometry.get("distance_m"), None)
    desired = safe_float(geometry.get("desired_distance_m"), None)
    bearing = safe_float(geometry.get("bearing_error_rad"), 0.0) or 0.0
    distance_error = 0.0 if distance is None or desired is None else float(distance) - float(desired)
    abs_bearing = abs(float(bearing))
    target_zone = str(getattr(request, "target_zone", "") or "").strip().lower()
    direction_only = bool(target_source == TARGET_SOURCE_CAMERA_TARGET and desired is not None and float(desired) >= 2.0)

    if target_source == TARGET_SOURCE_CAMERA_TARGET:
        if direction_only and target_zone == "center":
            return "camera_target_center_hold"
        if direction_only and (target_zone in {"left", "right"} or abs_bearing > 0.10):
            return "camera_target_in_place_align"
        if abs_bearing <= CRUISE_LAYER_V2_CAMERA_CENTER_PHASE_RAD and distance_error > 0.05:
            return "camera_target_center_forward"
        if abs_bearing <= CRUISE_LAYER_V2_CAMERA_CENTER_PHASE_RAD:
            return "camera_target_center_hold"
    if distance_error <= 0.05 and abs_bearing > CRUISE_LAYER_V2_CAMERA_CENTER_PHASE_RAD:
        return "target_hold_heading_align"
    if distance_error <= 0.05:
        return "target_hold"
    return str(speed_profile.get("phase") or "follow_goal_ready")


class CruiseLayerV2:
    def __init__(self, *, track_width_m: float = CRUISE_LAYER_V2_ROOM_TRACK_WIDTH_M) -> None:
        self.track_width_m = max(
            0.01,
            _safe_float(track_width_m, CRUISE_LAYER_V2_ROOM_TRACK_WIDTH_M),
        )
        self.follow_motion_profile = FollowMotionProfile()
        self.last_status: Dict[str, Any] = {
            "active": False,
            "provider": CRUISE_LAYER_V2_PROVIDER,
            "reason": "idle",
            "route": "",
        }

    def status(self) -> Dict[str, Any]:
        return dict(self.last_status)

    def _inactive(self, *, route: str, reason: str, intent: Optional[NavigationIntent] = None) -> CruiseLayerV2Result:
        status = {
            "active": False,
            "provider": CRUISE_LAYER_V2_PROVIDER,
            "reason": str(reason or "inactive"),
            "route": str(route or ""),
            "intent": intent.to_dict() if isinstance(intent, NavigationIntent) else {},
        }
        self.last_status = dict(status)
        return CruiseLayerV2Result(proposal=None, status=status, intent=intent)

    def tick_intent(
        self,
        intent: NavigationIntent,
        *,
        local_navigation_layer: Any,
        lidar_summary: Dict[str, Any],
        ekf_state: Dict[str, Any],
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        source: Optional[str] = None,
        dt: float = 0.02,
        now_s: Optional[float] = None,
        route: str = CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE,
        proposal_name: Optional[str] = None,
        update_map: bool = False,
        follow_request: Optional[FollowRequest] = None,
        tick_context: Any = None,
        clearance_cache: Optional[Dict[Any, Any]] = None,
    ) -> CruiseLayerV2Result:
        if not isinstance(intent, NavigationIntent):
            raise TypeError("intent must be NavigationIntent")
        route_s = str(route or CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE)
        src = str(source or intent.source or "STATE")
        if local_navigation_layer is None or not hasattr(local_navigation_layer, "tick_intent"):
            return self._inactive(route=route_s, reason="local_navigation_missing", intent=intent)
        if not bool(intent.active):
            return self._inactive(route=route_s, reason="intent_inactive", intent=intent)

        intent_metadata = dict(intent.metadata or {})
        local_path_segment = (
            dict(intent_metadata.get("local_path_segment") or {})
            if isinstance(intent_metadata.get("local_path_segment"), dict)
            else None
        )
        try:
            path_progress_m = float(intent_metadata.get("path_progress_m"))
            if not math.isfinite(path_progress_m):
                path_progress_m = None
        except (TypeError, ValueError):
            path_progress_m = None
        local_result = local_navigation_layer.tick_intent(
            intent,
            lidar_summary=lidar_summary,
            ekf_state=ekf_state,
            raw_scan=raw_scan,
            source=src,
            dt=float(dt),
            update_map=bool(update_map),
            now_s=now_s,
            local_path_segment=local_path_segment,
            path_progress_m=path_progress_m,
            tick_context=tick_context,
            clearance_cache=clearance_cache,
        )
        local_diag = dict(getattr(local_result, "diagnostics", {}) or {})
        rolling_map = _compact_map(dict(getattr(local_result, "snapshot", {}) or {}))
        proposal = getattr(local_result, "proposal", None)

        base_status = {
            "active": False,
            "provider": CRUISE_LAYER_V2_PROVIDER,
            "reason": str(local_diag.get("reason") or "local_navigation_no_proposal"),
            "route": route_s,
            "intent": intent.to_dict(),
            "local_navigation": dict(local_diag),
            "rolling_local_map": dict(rolling_map),
        }
        if proposal is None:
            self.last_status = dict(base_status)
            return CruiseLayerV2Result(proposal=None, status=dict(base_status), intent=intent)

        proposal = dict(proposal)
        command_type = str(proposal.get("command_type", "") or "")
        layer = str(proposal.get("layer", "") or "").upper()
        if command_type != CRUISE_LAYER_V2_ALLOWED_COMMAND or layer != CRUISE_LAYER_V2_ALLOWED_LAYER:
            status = {
                **base_status,
                "reason": f"blocked_unexpected_proposal:{layer}:{command_type}",
                "blocked": True,
            }
            self.last_status = dict(status)
            return CruiseLayerV2Result(proposal=None, status=status, intent=intent)

        proposal["name"] = str(proposal_name or f"{route_s}_local_navigation")
        proposal["layer"] = CRUISE_LAYER_V2_ALLOWED_LAYER
        proposal["command_type"] = CRUISE_LAYER_V2_ALLOWED_COMMAND
        proposal["source"] = src
        proposal["priority"] = int(intent.priority)
        target_source = str((intent.metadata or {}).get("target_source") or "").strip().upper()
        is_target_search = bool(
            target_source == TARGET_SOURCE_CAMERA_SEARCH
            or str(intent.reason or "") == "target_search_navigation"
            or bool(local_diag.get("target_search_in_place", False))
        )
        if route_s == CRUISE_LAYER_V2_ROUTE_ROOM_CRUISE and bool(local_diag.get("feasible", True)):
            proposal = _ensure_room_cruise_m3_track_motion(
                proposal,
                track_width_m=self.track_width_m,
            )
        elif not is_target_search or not bool(local_diag.get("feasible", True)):
            proposal["requested_track_reference"] = {}
            proposal["execution_mode"] = normalize_execution_mode(proposal.get("execution_mode", ""))
        proposal = _ensure_target_search_pivot_motion(
            proposal,
            intent=intent,
            local_diag=local_diag,
            track_width_m=self.track_width_m,
        )

        proposal_details = dict(proposal.get("details") or {})
        speed_profile = dict(proposal_details.get("speed_profile") or {})
        follow_status = follow_request.to_dict() if isinstance(follow_request, FollowRequest) else {}
        target_geometry = _target_geometry(follow_request, dict(ekf_state or {}))
        phase = _follow_phase(follow_request, dict(ekf_state or {}), speed_profile)
        reason = str(local_diag.get("reason") or intent.reason or "local_navigation_ready")
        target_source = str(follow_status.get("target_source") or intent.metadata.get("target_source", "") or "")
        omega_limit_chain = dict((intent.metadata or {}).get("omega_limit_chain") or {})
        if omega_limit_chain:
            omega_limit_chain.update(
                {
                    "local_navigation_omega_rad_s": proposal.get("omega_target"),
                    "cruise_layer_v2_output_omega_rad_s": proposal.get("omega_target"),
                    "final_runtime_clamp": "motion_controller_then_speed_limits",
                }
            )

        cruise_status = {
            "active": True,
            "provider": CRUISE_LAYER_V2_PROVIDER,
            "reason": reason,
            "route": route_s,
            "primitive_type": CRUISE_LAYER_V2_ALLOWED_COMMAND,
            "motion_style": route_s,
            "source": src,
            "target_source": target_source,
            "room_cruise_chain": True,
            "follow_above_cruise": bool(route_s == CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW),
            "local_planner_bypassed": False,
            "local_navigation_active": True,
            "local_navigation_suppressed_phase": False,
        }
        room_cruise = {
            "active": True,
            "chain": route_s,
            "motion_style": route_s,
            "phase": phase,
            "reason": reason,
            "route": route_s,
            "follow_above_cruise": bool(route_s == CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW),
            "target_geometry": dict(target_geometry),
            "follow_gate": {
                "actual_target_distance_m": target_geometry.get("distance_m"),
                "desired_distance_m": target_geometry.get("desired_distance_m"),
                "actual_target_bearing_error_rad": target_geometry.get("bearing_error_rad"),
            },
            "clearance": dict(proposal_details.get("clearance") or {}),
        }
        proposal_details.update(
            {
                "provider": CRUISE_LAYER_V2_PROVIDER,
                "navigation_intent": intent.to_dict(),
                "local_navigation": dict(local_diag),
                "rolling_local_map": dict(rolling_map),
                "cruise_layer": dict(cruise_status),
                "room_cruise": room_cruise,
            }
        )
        if omega_limit_chain:
            proposal_details["omega_limit_chain"] = dict(omega_limit_chain)
        if follow_status:
            proposal_details["follow_request"] = dict(follow_status)
        proposal["details"] = proposal_details

        status = {
            **cruise_status,
            "intent": intent.to_dict(),
            "local_navigation": dict(local_diag),
            "rolling_local_map": dict(rolling_map),
            "proposal_active": True,
            "proposal_name": str(proposal.get("name") or ""),
        }
        self.last_status = dict(status)
        return CruiseLayerV2Result(proposal=proposal, status=status, intent=intent)

    def follow_intent(
        self,
        request: FollowRequest,
        *,
        ekf_state: Dict[str, Any],
        source: Optional[str] = None,
        now_s: Optional[float] = None,
        priority: int = 810,
    ) -> NavigationIntent:
        src = str(source or getattr(request, "source", "STATE") or "STATE")
        target_source = str(getattr(request, "target_source", "") or "")
        target_id = str(getattr(request, "target_id", "") or "")
        if target_source == TARGET_SOURCE_CAMERA_TARGET and (
            "persisted" in target_id or "reacquire" in target_id
        ):
            return NavigationIntent.hold(
                source=src,
                behavior="HUMAN_FOLLOW",
                reason=("target_reacquire_hold" if "reacquire" in target_id else "target_persistence_hold"),
                priority=int(priority),
                metadata={
                    "target_source": target_source,
                    "target_id": target_id,
                    "route": CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW,
                    "distance_to_target_m": safe_float(getattr(request, "distance_to_target_m", None), None),
                    "standoff_m": safe_float(getattr(request, "desired_distance_m", None), None),
                },
            )
        if target_source == TARGET_SOURCE_CAMERA_SEARCH:
            x, y, theta = _extract_pose(dict(ekf_state or {}))
            heading = _wrap_angle(theta + (_search_direction(request) * CRUISE_LAYER_V2_SEARCH_HEADING_DELTA_RAD))
            search_side = _search_side(request)
            omega_before_scale = safe_float(getattr(request, "omega_max_rad_s", None), None)
            omega_limit = _positive_omega_limit(omega_before_scale, 0.80)
            return NavigationIntent(
                active=bool(getattr(request, "active", False)),
                source=src,
                behavior="HUMAN_FOLLOW",
                mode=NAV_MODE_GOAL,
                command_type="follow_search_navigation_intent",
                goal_x=float(x),
                goal_y=float(y),
                goal_theta=float(heading),
                desired_speed_mps=0.0,
                max_v_mps=0.0,
                max_omega_rad_s=float(omega_limit),
                standoff_m=0.0,
                priority=int(priority),
                reason="target_search_navigation",
                metadata={
                    "target_source": target_source,
                    "target_id": target_id,
                    "search_side": str(search_side),
                    "turn_direction": str(search_side),
                    "search_heading_delta_rad": round(float(_wrap_angle(float(heading) - float(theta))), 6),
                    "search_motion": "in_place_pivot",
                    "omega_limit_chain": {
                        "follow_request_max_omega_rad_s": omega_before_scale,
                        "navigation_intent_max_omega_rad_s": float(omega_limit),
                    },
                    "route": CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW,
                },
            )
        profile_status: Dict[str, Any] = {}
        goal_x = safe_float(getattr(request, "goal_x", None), None)
        goal_y = safe_float(getattr(request, "goal_y", None), None)
        goal_theta = safe_float(getattr(request, "goal_theta", None), None)
        distance_to_target_m = safe_float(getattr(request, "distance_to_target_m", None), None)
        max_v = safe_float(getattr(request, "v_max_mps", None), None)
        max_w = safe_float(getattr(request, "omega_max_rad_s", None), None)
        use_live_camera_profile = bool(
            target_source == TARGET_SOURCE_CAMERA_TARGET
            and str(src or "").strip().upper() == "ADAPTIVE"
            and not ("persisted" in target_id or "reacquire" in target_id)
        )
        if use_live_camera_profile:
            profiled = self.follow_motion_profile.tick(
                request,
                dict(ekf_state or {}),
                now_s=now_s,
            )
            goal_x = profiled.goal_x
            goal_y = profiled.goal_y
            goal_theta = profiled.goal_theta
            distance_to_target_m = profiled.distance_to_target_m
            max_v = profiled.max_v_mps
            max_w = profiled.max_omega_rad_s
            profile_status = dict(profiled.status or {})
        else:
            self.follow_motion_profile.reset()
        intent = NavigationIntent.from_follow_request(
            request,
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            source=src,
            priority=int(priority),
            metadata={
                "target_source": target_source,
                "target_id": target_id,
                "route": CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW,
                "follow_motion_profile": dict(profile_status),
            },
        )
        if use_live_camera_profile and goal_x is not None and goal_y is not None and goal_theta is not None:
            intent = NavigationIntent(
                active=True,
                source=src,
                behavior="HUMAN_FOLLOW",
                mode=NAV_MODE_FOLLOW,
                command_type="follow_navigation_intent",
                goal_x=float(goal_x),
                goal_y=float(goal_y),
                goal_theta=float(goal_theta),
                desired_speed_mps=max_v,
                max_v_mps=max_v,
                max_omega_rad_s=max_w,
                standoff_m=safe_float(getattr(request, "desired_distance_m", None), None),
                priority=int(priority),
                reason=str(getattr(request, "reason", "") or "follow_goal_ready"),
                metadata={
                    "target_source": target_source,
                    "target_id": target_id,
                    "target_zone": str(getattr(request, "target_zone", "") or ""),
                    "route": CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW,
                    "distance_to_target_m": distance_to_target_m,
                    "follow_motion_profile": dict(profile_status),
                },
            )
        return intent

    def tick_follow_request(
        self,
        request: FollowRequest,
        *,
        local_navigation_layer: Any,
        lidar_summary: Dict[str, Any],
        ekf_state: Dict[str, Any],
        raw_scan: Optional[List[Dict[str, Any]]] = None,
        source: Optional[str] = None,
        dt: float = 0.02,
        now_s: Optional[float] = None,
        update_map: bool = False,
        tick_context: Any = None,
        clearance_cache: Optional[Dict[Any, Any]] = None,
    ) -> CruiseLayerV2Result:
        if not isinstance(request, FollowRequest) or not bool(getattr(request, "active", False)):
            return self._inactive(
                route=CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW,
                reason=str(getattr(request, "reason", "") or "follow_request_inactive"),
            )
        intent = self.follow_intent(
            request,
            ekf_state=dict(ekf_state or {}),
            source=source,
            now_s=now_s,
            priority=810,
        )
        return self.tick_intent(
            intent,
            local_navigation_layer=local_navigation_layer,
            lidar_summary=lidar_summary,
            ekf_state=ekf_state,
            raw_scan=raw_scan,
            source=source,
            dt=float(dt),
            now_s=now_s,
            route=CRUISE_LAYER_V2_ROUTE_HUMAN_FOLLOW,
            proposal_name="human_follow_v2_local_navigation",
            update_map=bool(update_map),
            follow_request=request,
            tick_context=tick_context,
            clearance_cache=clearance_cache,
        )
