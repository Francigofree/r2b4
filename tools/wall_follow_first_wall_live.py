#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Live first-wall wall-follow validation through the normal runtime command bus."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from controller.motion_policy import GlobalMotionPolicy  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_TOKEN,
    LIDAR_SCAN_PATH,
    STATUS_PATH,
    _append_command,
    _get_pose,
    _pose_distance,
    _read_json,
    _safe_float,
    _safe_stop_best_effort,
    _send_command_checked,
    _status_version,
    _wait_for_lidar_scan_progress,
    _wait_for_start_gate,
    _wait_for_status_progress,
    _wait_until_stopped,
)

AGENT_TEST_DIR = test_artifacts_dir()
STATUS_DEBUG_PATH = PROJECT_ROOT / "runtime" / "status_debug.json"
LATEST_RESULT_PATH = AGENT_TEST_DIR / "latest_wall_follow_first_wall_live_result.json"
LATEST_SUMMARY_PATH = AGENT_TEST_DIR / "latest_wall_follow_first_wall_live_summary.json"
HISTORY_PATH = AGENT_TEST_DIR / "wall_follow_first_wall_live_history.jsonl"
NEAR_SINGLE_TRACK_TURN_M = 0.62
SIDE_SINGLE_TRACK_TURN_M = 0.44
FRONT_SINGLE_TRACK_TURN_M = 0.50
FAR_GENTLE_TURN_START_M = 0.80
FAR_GENTLE_TURN_END_M = 1.00
FAR_GENTLE_TURN_DIFF_SCALE = 1.75
FRONT_PRESSURE_MIN_BASE_SCALE = 0.55
SINGLE_TRACK_TURN_STATES = {
    "leave_wall",
    "corner_turn",
    "front_align",
    "front_hard_leave_wall",
    "corner_hard_turn",
    "front_hard_turn",
}
FRONT_TURN_STATES = {
    "corner_turn",
    "front_align",
    "front_hard_leave_wall",
    "corner_hard_turn",
    "front_hard_turn",
}
HARD_FRONT_TURN_STATES = {
    "front_hard_leave_wall",
    "corner_hard_turn",
    "front_hard_turn",
}


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp_path.replace(path)


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def _status_state(status: Dict[str, Any]) -> str:
    return str((status or {}).get("state", "") or "").strip().upper()


def _read_status_with_debug() -> Dict[str, Any]:
    public_status = dict(_read_json(STATUS_PATH) or {})
    debug_status = dict(_read_json(STATUS_DEBUG_PATH) or {})
    if not debug_status:
        return public_status
    merged = dict(public_status or debug_status)
    for key in (
        "global_motion_policy",
        "policy_active_flag",
        "forward_clearance",
        "v_policy_limit",
        "obstacle_avoidance",
        "motion_resolution",
        "motion_semantics",
    ):
        if key in debug_status:
            merged[key] = debug_status.get(key)
    return merged


def _front_clearance(status: Dict[str, Any]) -> float:
    st = dict(status or {})
    policy = dict(st.get("global_motion_policy") or {})
    lidar = dict(st.get("lidar") or {})
    for value in (
        policy.get("forward_clearance_m"),
        st.get("forward_clearance"),
        lidar.get("front_clearance_m"),
        lidar.get("min_dist_narrow"),
        lidar.get("min_dist"),
    ):
        val = _safe_float(value, math.nan)
        if math.isfinite(val) and val >= 0.0:
            return float(val)
    return math.nan


def _read_lidar_scan_points() -> List[Dict[str, Any]]:
    payload = dict(_read_json(LIDAR_SCAN_PATH) or {})
    scan = payload.get("scan")
    return list(scan or []) if isinstance(scan, list) else []


def _scan_side_clearance(raw_scan: Optional[List[Dict[str, Any]]], side: str) -> float:
    return GlobalMotionPolicy._resolve_scan_side_clearance_m(list(raw_scan or []), str(side))


def _scan_point_distance_m(point: Dict[str, Any]) -> float:
    for key in ("dist_m", "distance_m", "range_m"):
        val = _safe_float(dict(point or {}).get(key), math.nan)
        if math.isfinite(val) and val >= 0.0:
            return float(val)
    val = _safe_float(dict(point or {}).get("dist"), math.nan)
    if math.isfinite(val) and val >= 0.0:
        return float(val / 1000.0) if val > 20.0 else float(val)
    return math.nan


def _angle_delta_deg(angle_deg: float, center_deg: float) -> float:
    return ((float(angle_deg) - float(center_deg) + 180.0) % 360.0) - 180.0


def _scan_side_wall_profile(raw_scan: Optional[List[Dict[str, Any]]], side: str) -> Dict[str, Any]:
    center = 270.0 if str(side).strip().lower() == "left" else 90.0
    half_span = 38.0
    values: List[Tuple[float, float]] = []
    for point in list(raw_scan or []):
        item = dict(point or {})
        angle = _safe_float(item.get("angle_deg", item.get("angle")), math.nan)
        dist_m = _scan_point_distance_m(item)
        if not (math.isfinite(angle) and math.isfinite(dist_m) and 0.08 <= float(dist_m) <= 4.0):
            continue
        delta = _angle_delta_deg(float(angle) % 360.0, center)
        if abs(float(delta)) > half_span:
            continue
        perp_m = float(dist_m) * math.cos(math.radians(abs(float(delta))))
        if math.isfinite(perp_m) and 0.10 <= float(perp_m) <= 2.20:
            values.append((float(delta), float(perp_m)))

    if not values:
        return {
            "available": False,
            "confidence": 0.0,
            "count": 0,
            "perpendicular_m": math.nan,
            "mad_m": math.nan,
            "angular_span_deg": 0.0,
        }

    perps = [v for _, v in values]
    median = float(statistics.median(perps))
    mad = float(statistics.median([abs(v - median) for v in perps])) if len(perps) >= 2 else 0.0
    angular_span = max(delta for delta, _ in values) - min(delta for delta, _ in values)
    count_score = _clamp(float(len(values)) / 7.0, 0.0, 1.0)
    spread_score = _clamp(1.0 - (float(mad) / 0.18), 0.0, 1.0)
    span_score = _clamp(float(abs(angular_span)) / 28.0, 0.0, 1.0)
    confidence = float(_clamp((0.55 * count_score) + (0.30 * spread_score) + (0.15 * span_score), 0.0, 1.0))
    return {
        "available": True,
        "confidence": float(confidence),
        "count": int(len(values)),
        "perpendicular_m": float(median),
        "mad_m": float(mad),
        "angular_span_deg": float(abs(angular_span)),
    }


def _side_clearance(
    status: Dict[str, Any],
    side: str,
    *,
    raw_scan: Optional[List[Dict[str, Any]]] = None,
) -> float:
    scan_clearance = _scan_side_clearance(raw_scan, side)
    if math.isfinite(scan_clearance) and scan_clearance >= 0.0:
        return float(scan_clearance)
    lidar = dict((status or {}).get("lidar") or {})
    if side == "left":
        keys = ("left_clearance_m", "left_clearance", "avg_left")
    else:
        keys = ("right_clearance_m", "right_clearance", "avg_right")
    for key in keys:
        val = _safe_float(lidar.get(key), math.nan)
        if math.isfinite(val) and val >= 0.0:
            return float(val)
    return math.nan


def _command_zero(token: str, *, motion_source: str, timeout_s: float) -> Dict[str, Any]:
    return _send_command_checked(
        "set_twist",
        token=str(token),
        timeout_s=float(timeout_s),
        v=0.0,
        omega=0.0,
        motion_source=str(motion_source),
    )


def _command_zero_track(token: str, *, motion_source: str, timeout_s: float) -> Dict[str, Any]:
    return _send_command_checked(
        "set_track_velocity",
        token=str(token),
        timeout_s=float(timeout_s),
        left_mps=0.0,
        right_mps=0.0,
        motion_source=str(motion_source),
        reason="WALL_FOLLOW_FIRST_WALL_ZERO_TRACK",
    )


def _send_track_checked(
    token: str,
    *,
    left_mps: float,
    right_mps: float,
    motion_source: str,
    timeout_s: float,
    reason: str,
) -> Dict[str, Any]:
    return _send_command_checked(
        "set_track_velocity",
        token=str(token),
        timeout_s=float(timeout_s),
        left_mps=float(left_mps),
        right_mps=float(right_mps),
        motion_source=str(motion_source),
        reason=str(reason),
    )


def _append_track(
    token: str,
    *,
    left_mps: float,
    right_mps: float,
    motion_source: str,
    reason: str,
) -> str:
    return str(
        _append_command(
            "set_track_velocity",
            token=str(token),
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            motion_source=str(motion_source),
            reason=str(reason),
        )
    )


def _preferred_direction(value: Any) -> str:
    raw = str(value or "AUTO").strip().upper()
    if raw in ("LEFT", "RIGHT"):
        return raw
    return "AUTO"


def _resolve_follow_direction(
    *,
    preferred_turn_direction: str,
    sample: Dict[str, Any],
    policy: Dict[str, Any],
) -> str:
    preferred = _preferred_direction(preferred_turn_direction)
    if preferred in ("LEFT", "RIGHT"):
        return preferred
    for value in (
        sample.get("chosen_direction"),
        policy.get("chosen_direction"),
        sample.get("gap_direction"),
    ):
        raw = str(value or "").strip().upper()
        if raw in ("LEFT", "RIGHT"):
            return raw
    left = _safe_float(sample.get("left_clearance_m"), math.nan)
    right = _safe_float(sample.get("right_clearance_m"), math.nan)
    if math.isfinite(left) and math.isfinite(right):
        return "LEFT" if left >= right else "RIGHT"
    return "RIGHT"


def _wall_side_from_policy(chosen_direction: str, wall_side: str) -> str:
    side = str(wall_side or "").strip().upper()
    if side in ("LEFT", "RIGHT"):
        return side
    chosen = str(chosen_direction or "").strip().upper()
    if chosen == "RIGHT":
        return "LEFT"
    return "RIGHT"


def _resolve_follow_wall_side(
    *,
    sample: Dict[str, Any],
    policy_wall_side: str,
    chosen_direction: str,
) -> str:
    left = _safe_float(sample.get("left_clearance_m"), math.nan)
    right = _safe_float(sample.get("right_clearance_m"), math.nan)
    left_visible = bool(math.isfinite(left) and 0.16 <= float(left) <= 1.05)
    right_visible = bool(math.isfinite(right) and 0.16 <= float(right) <= 1.05)
    if left_visible and right_visible:
        if abs(float(left) - float(right)) >= 0.12:
            return "LEFT" if float(left) <= float(right) else "RIGHT"
    elif left_visible:
        return "LEFT"
    elif right_visible:
        return "RIGHT"
    return _wall_side_from_policy(chosen_direction, policy_wall_side)


def _turn_sign_from_direction(direction: str) -> float:
    return 1.0 if str(direction or "").strip().upper() == "LEFT" else -1.0


def _wall_push_out_turn_signal(*, side: str, distance_m: float, wall_min_m: float) -> float:
    side_toward_sign = 1.0 if str(side).strip().upper() == "LEFT" else -1.0
    outward_sign = -float(side_toward_sign)
    dist = float(distance_m)
    if dist < 0.40:
        return float(outward_sign * 0.90)
    if dist < float(wall_min_m):
        return float(outward_sign * 0.62)
    return float(outward_sign * 0.22)


def _turn_proximity_for_state(
    *,
    wall_control_state: str,
    front_m: float,
    side_distance_m: float,
    front_pressure: float,
) -> Tuple[float, str]:
    state = str(wall_control_state)
    if state in FRONT_TURN_STATES and math.isfinite(front_m):
        return float(front_m), "front"
    if state in ("leave_wall", "approach_wall", "search_wall") and math.isfinite(side_distance_m):
        return float(side_distance_m), "side"
    if float(front_pressure) > 0.02 and math.isfinite(front_m):
        return float(front_m), "front"
    if math.isfinite(side_distance_m):
        return float(side_distance_m), "side"
    if math.isfinite(front_m):
        return float(front_m), "front"
    return math.nan, "none"


def _build_track_command_from_turn_signal(
    *,
    turn_signal: float,
    wall_control_state: str,
    base_v_mps: float,
    track_diff_mps: float,
    track_diff_min_mps: float,
    track_diff_max_mps: float,
    track_min_inner_mps: float,
    track_max_mps: float,
    front_m: float,
    side_distance_m: float,
    front_pressure: float,
) -> Dict[str, Any]:
    base = _clamp(max(0.0, float(base_v_mps)), 0.0, float(track_max_mps))
    signal = _clamp(float(turn_signal), -1.0, 1.0)
    proximity_m, proximity_source = _turn_proximity_for_state(
        wall_control_state=str(wall_control_state),
        front_m=float(front_m),
        side_distance_m=float(side_distance_m),
        front_pressure=float(front_pressure),
    )
    if base <= 0.0 or abs(signal) <= 0.05:
        return {
            "left_mps": float(base),
            "right_mps": float(base),
            "linear_mps": float(base),
            "track_diff_mps": 0.0,
            "turn_execution_mode": "straight",
            "turn_proximity_m": None if not math.isfinite(proximity_m) else float(proximity_m),
            "turn_proximity_source": str(proximity_source),
        }

    state = str(wall_control_state)
    single_track_arc = False
    if state in SINGLE_TRACK_TURN_STATES and math.isfinite(proximity_m):
        if proximity_source == "side":
            single_track_arc = float(proximity_m) <= SIDE_SINGLE_TRACK_TURN_M
        elif state in HARD_FRONT_TURN_STATES:
            single_track_arc = float(proximity_m) <= NEAR_SINGLE_TRACK_TURN_M
        elif state == "corner_turn":
            single_track_arc = float(proximity_m) <= FRONT_SINGLE_TRACK_TURN_M
    if single_track_arc:
        outer_scale = 1.05 + (0.22 * abs(float(signal)))
        outer = _clamp(
            max(float(base) * float(outer_scale), float(track_diff_min_mps) * 1.50),
            0.0,
            float(track_max_mps),
        )
        inner = 0.0
        mode = "single_track_arc"
    else:
        state_ratio = {
            "track_band": 0.0,
            "search_wall": 0.32,
            "approach_wall": 0.42,
            "leave_wall": 0.78,
            "corner_turn": 0.76,
            "front_align": 0.66,
            "front_hard_leave_wall": 0.82,
            "corner_hard_turn": 0.82,
            "front_hard_turn": 0.78,
        }.get(state, 0.50)
        mode = "progressive_arc"
        if math.isfinite(proximity_m) and float(proximity_m) >= FAR_GENTLE_TURN_START_M:
            far_amount = _clamp(
                (float(proximity_m) - FAR_GENTLE_TURN_START_M)
                / max(0.05, FAR_GENTLE_TURN_END_M - FAR_GENTLE_TURN_START_M),
                0.0,
                1.0,
            )
            state_ratio = min(float(state_ratio), 0.40 - (0.12 * float(far_amount)))
            mode = "gentle_far_arc"
        elif math.isfinite(proximity_m) and float(proximity_m) <= 0.70:
            state_ratio = max(float(state_ratio), 0.72)

        safe_diff_max = min(
            float(track_diff_max_mps),
            max(0.0, 2.0 * max(0.0, float(base) - float(track_min_inner_mps))),
            max(0.0, float(base) * 0.96),
        )
        diff_nominal = _clamp(float(track_diff_mps), float(track_diff_min_mps), float(track_diff_max_mps))
        min_effective_diff = min(float(track_diff_min_mps), max(0.0, float(base) * 0.35))
        diff = min(float(safe_diff_max), float(diff_nominal), float(base) * float(state_ratio))
        diff *= 0.70 + (0.30 * abs(float(signal)))
        if mode == "gentle_far_arc":
            diff = min(float(safe_diff_max), float(diff) * FAR_GENTLE_TURN_DIFF_SCALE)
        if state != "track_band":
            diff = max(float(diff), min(float(min_effective_diff), float(safe_diff_max)))
        inner = float(base) - (0.5 * float(diff))
        outer = float(base) + (0.5 * float(diff))
        inner = _clamp(inner, float(track_min_inner_mps), float(track_max_mps))
        outer = _clamp(outer, float(track_min_inner_mps), float(track_max_mps))

    if signal >= 0.0:
        left = float(inner)
        right = float(outer)
    else:
        left = float(outer)
        right = float(inner)
    return {
        "left_mps": float(left),
        "right_mps": float(right),
        "linear_mps": float(0.5 * (float(left) + float(right))),
        "track_diff_mps": float(abs(float(right) - float(left))),
        "turn_execution_mode": str(mode),
        "turn_proximity_m": None if not math.isfinite(proximity_m) else float(proximity_m),
        "turn_proximity_source": str(proximity_source),
    }


def _compute_policy_track_command(
    status: Dict[str, Any],
    *,
    chosen_direction: str,
    wall_side: str,
    base_v_mps: float,
    wall_target_m: float,
    wall_min_m: float,
    wall_max_m: float,
    track_diff_mps: float,
    track_diff_min_mps: float,
    track_diff_max_mps: float,
    track_min_inner_mps: float,
    track_max_mps: float,
    front_turn_start_m: float,
    front_hard_turn_m: float,
    wall_distance_gain: float,
    corner_turn_active: bool = False,
    raw_scan: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    front = _front_clearance(status)
    side = _wall_side_from_policy(chosen_direction, wall_side)
    side_key = "left" if side == "LEFT" else "right"
    side_profile = _scan_side_wall_profile(raw_scan, side_key)
    side_dist = _side_clearance(status, side_key, raw_scan=raw_scan)
    profile_dist = _safe_float(side_profile.get("perpendicular_m"), math.nan)
    if math.isfinite(profile_dist) and float(side_profile.get("confidence", 0.0)) >= 0.35:
        side_dist = float(profile_dist)
    chosen_sign = _turn_sign_from_direction(chosen_direction)

    if math.isfinite(front):
        denom = max(0.05, float(front_turn_start_m) - float(front_hard_turn_m))
        front_pressure = _clamp((float(front_turn_start_m) - float(front)) / denom, 0.0, 1.0)
    else:
        front_pressure = 0.0

    low_band = min(float(wall_min_m), float(wall_max_m))
    high_band = max(float(wall_min_m), float(wall_max_m))
    wall_lost_m = max(float(high_band) + 0.30, 1.05)
    side_toward_sign = 1.0 if side == "LEFT" else -1.0
    side_turn = 0.0
    wall_control_state = "search_wall"
    side_visible = bool(math.isfinite(side_dist) and 0.12 <= float(side_dist) <= 2.20)
    profile_conf = float(_clamp(_safe_float(side_profile.get("confidence"), 0.0), 0.0, 1.0))
    wall_like = bool(profile_conf >= 0.35 or raw_scan is None)
    wall_lost = bool(
        (not side_visible)
        or (math.isfinite(side_dist) and float(side_dist) > wall_lost_m and not wall_like)
        or (math.isfinite(side_dist) and float(side_dist) > max(1.25, wall_lost_m + 0.18))
    )
    if side_visible:
        if wall_lost:
            wall_control_state = "search_wall"
            side_turn = 0.22 * float(side_toward_sign)
        elif float(side_dist) < low_band:
            wall_control_state = "leave_wall"
            side_turn = _wall_push_out_turn_signal(
                side=str(side),
                distance_m=float(side_dist),
                wall_min_m=float(low_band),
            )
        elif float(side_dist) <= high_band:
            wall_control_state = "track_band"
            side_turn = 0.0
        else:
            wall_control_state = "approach_wall"
            amount = _clamp(
                (float(side_dist) - float(high_band)) / max(0.05, float(wall_lost_m) - float(high_band)),
                0.0,
                1.0,
            )
            side_turn = float(side_toward_sign) * (0.12 + (0.34 * float(amount)))

    corner_turn = bool(corner_turn_active and math.isfinite(front) and float(front) <= float(front_turn_start_m))
    if corner_turn:
        wall_control_state = "corner_turn"
        turn_signal = chosen_sign * (0.62 + (0.18 * float(front_pressure)))
    elif front_pressure > 0.02:
        wall_control_state = "front_align"
        turn_signal = (chosen_sign * (0.24 + 0.36 * float(front_pressure))) + (0.25 * float(side_turn))
    elif side_visible:
        turn_signal = float(side_turn)
    else:
        wall_control_state = "search_wall"
        turn_signal = 0.22 if side == "LEFT" else -0.22
    if math.isfinite(front) and float(front) <= float(front_hard_turn_m):
        if (
            side_visible
            and math.isfinite(side_dist)
            and float(side_dist) < low_band
        ):
            wall_control_state = "front_hard_leave_wall"
            turn_signal = _wall_push_out_turn_signal(
                side=str(side),
                distance_m=float(side_dist),
                wall_min_m=float(low_band),
            )
        elif corner_turn:
            wall_control_state = "corner_hard_turn"
            turn_signal = chosen_sign * 0.82
        else:
            wall_control_state = "front_hard_turn"
            turn_signal = chosen_sign * 0.68
    turn_signal = _clamp(float(turn_signal), -1.0, 1.0)
    if abs(turn_signal) < 0.12 and (front_pressure > 0.02 or not side_visible):
        turn_signal = math.copysign(0.12, chosen_sign)

    if side_visible and front_pressure <= 0.02 and abs(float(turn_signal)) <= 0.05:
        turn_signal = 0.0
    base_for_command = float(base_v_mps)
    if front_pressure > 0.02:
        pressure_scale = 1.0 - ((1.0 - FRONT_PRESSURE_MIN_BASE_SCALE) * float(front_pressure))
        base_for_command = float(base_for_command) * float(
            _clamp(pressure_scale, FRONT_PRESSURE_MIN_BASE_SCALE, 1.0)
        )

    track_targets = _build_track_command_from_turn_signal(
        turn_signal=float(turn_signal),
        wall_control_state=str(wall_control_state),
        base_v_mps=float(base_for_command),
        track_diff_mps=float(track_diff_mps),
        track_diff_min_mps=float(track_diff_min_mps),
        track_diff_max_mps=float(track_diff_max_mps),
        track_min_inner_mps=float(track_min_inner_mps),
        track_max_mps=float(track_max_mps),
        front_m=float(front),
        side_distance_m=float(side_dist),
        front_pressure=float(front_pressure),
    )
    left = float(track_targets["left_mps"])
    right = float(track_targets["right_mps"])
    return {
        "active": True,
        "mode": "policy_assisted_track_velocity",
        "left_mps": float(left),
        "right_mps": float(right),
        "linear_mps": float(track_targets["linear_mps"]),
        "track_diff_mps": float(track_targets["track_diff_mps"]),
        "turn_signal": float(turn_signal),
        "turn_execution_mode": str(track_targets["turn_execution_mode"]),
        "turn_proximity_m": track_targets.get("turn_proximity_m"),
        "turn_proximity_source": str(track_targets.get("turn_proximity_source", "")),
        "chosen_direction": str(chosen_direction or "").strip().upper(),
        "wall_side": str(side),
        "wall_target_m": float(wall_target_m),
        "wall_min_m": float(wall_min_m),
        "wall_max_m": float(wall_max_m),
        "wall_distance_m": None if not math.isfinite(side_dist) else float(side_dist),
        "front_pressure": float(front_pressure),
        "side_visible": bool(side_visible),
        "wall_control_state": str(wall_control_state),
        "corner_turn_active": bool(corner_turn_active),
        "wall_lost": bool(wall_lost),
        "wall_profile_confidence": float(profile_conf),
        "wall_profile_count": int(side_profile.get("count", 0) or 0),
        "wall_profile_mad_m": None
        if not math.isfinite(_safe_float(side_profile.get("mad_m"), math.nan))
        else float(side_profile.get("mad_m")),
    }


def _precheck(token: str, *, motion_source: str, stop_timeout_s: float) -> Dict[str, Any]:
    st = _wait_for_status_progress(min_increments=2, timeout_s=5.0)
    _wait_for_lidar_scan_progress(min_increments=2, timeout_s=3.0)
    if not bool((st.get("startup") or {}).get("ready", False)):
        raise RuntimeError(f"Runtime startup is not READY: {st.get('startup')}")
    if _status_state(st) == "FAILSAFE":
        _send_command_checked("strong_reset", token=str(token), timeout_s=12.0)
        time.sleep(0.3)
        st = _wait_for_status_progress(min_increments=2, timeout_s=4.0)

    stop_cmd = _command_zero(token, motion_source=motion_source, timeout_s=4.0)
    stopped = _wait_until_stopped(timeout_s=float(stop_timeout_s))
    _send_command_checked("reset_pos", token=str(token), timeout_s=4.0)
    time.sleep(0.2)
    gate = _wait_for_start_gate(timeout_s=6.0)
    gate_status = dict(gate.get("status") or {})
    gate_scan = dict(gate.get("lidar_scan") or {})

    if str(gate_status.get("odometry_mode", "")).strip().upper() != "LIDAR_FIRST":
        raise RuntimeError(f"odometry_mode is {gate_status.get('odometry_mode')}, expected LIDAR_FIRST.")
    if not bool(gate_status.get("lidar_enabled", False)):
        raise RuntimeError("LIDAR is not enabled.")
    if str(gate_status.get("lidar_health", "")).strip().upper() != "OK":
        raise RuntimeError(f"LIDAR health is not OK: {gate_status.get('lidar_health')}")
    if not bool(gate_scan.get("scan")):
        raise RuntimeError(f"{LIDAR_SCAN_PATH} does not contain scan points.")

    return {
        "initial_status_version": int(_status_version(st)),
        "stopped_state": _status_state(stopped),
        "stop_cmd": stop_cmd,
        "start_gate": {
            "status_version": int(_safe_float(gate_status.get("status_version"), -1)),
            "front_clearance_m": _front_clearance(gate_status),
            "lidar_scan_ts": _safe_float(gate_scan.get("ts"), math.nan),
            "diagnostics": dict(gate.get("diagnostics") or {}),
        },
    }


def _sample(
    status: Dict[str, Any],
    *,
    elapsed_s: float,
    phase: str,
    track_command: Optional[Dict[str, Any]] = None,
    raw_scan: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    st = dict(status or {})
    policy = dict(st.get("global_motion_policy") or {})
    wall = dict(policy.get("wall_follow") or {})
    gap = dict(policy.get("lidar_gap_analysis") or {})
    lidar = dict(st.get("lidar") or {})
    track = dict(track_command or {})
    pose = _get_pose(st)
    local_track_active = bool(track.get("active", False))
    wall_active = bool(wall.get("active", False) or local_track_active)
    wall_side = str(wall.get("side", "") or track.get("wall_side", ""))
    left_clearance = _side_clearance(st, "left", raw_scan=raw_scan)
    right_clearance = _side_clearance(st, "right", raw_scan=raw_scan)
    if str(wall_side).strip().upper() == "LEFT":
        wall_distance = left_clearance
    elif str(wall_side).strip().upper() == "RIGHT":
        wall_distance = right_clearance
    else:
        wall_distance = math.nan
    return {
        "elapsed_s": round(float(elapsed_s), 3),
        "phase": str(phase),
        "state": _status_state(st),
        "status_version": int(_status_version(st)),
        "front_clearance_m": _front_clearance(st),
        "left_clearance_m": left_clearance,
        "right_clearance_m": right_clearance,
        "policy_state": str(policy.get("policy_state", "")),
        "policy_active": bool(policy.get("active", False)),
        "chosen_direction": str(policy.get("chosen_direction", "")),
        "decision_reason": str(policy.get("decision_reason", "")),
        "policy_actions": list(policy.get("actions", []) or []),
        "clearance_based_limit_mps": _safe_float(policy.get("clearance_based_limit_mps"), 0.0),
        "curvature_based_limit_mps": _safe_float(policy.get("curvature_based_limit_mps"), 0.0),
        "planner_v_target_mps": _safe_float(policy.get("planner_v_target_mps"), 0.0),
        "blocked_front": bool(policy.get("blocked_front", lidar.get("blocked_front", False))),
        "wall_follow_active": bool(wall_active),
        "wall_policy_active": bool(wall.get("active", False)),
        "wall_side": str(wall_side),
        "wall_distance_m": None if not math.isfinite(wall_distance) else float(wall_distance),
        "wall_reason": str(wall.get("reason", "")),
        "wall_kappa_bias": _safe_float(wall.get("kappa_bias"), 0.0),
        "gap_direction": str(gap.get("best_direction", "")),
        "omega_cmd_rad_s": _safe_float(policy.get("omega_cmd_rad_s", st.get("omega_target")), 0.0),
        "v_cmd_mps": _safe_float(policy.get("v_cmd_mps", st.get("v_target")), 0.0),
        "track_wall_controller_active": bool(local_track_active),
        "commanded_left_track_mps": _safe_float(track.get("left_mps"), 0.0),
        "commanded_right_track_mps": _safe_float(track.get("right_mps"), 0.0),
        "commanded_linear_mps": _safe_float(track.get("linear_mps"), 0.0),
        "commanded_track_diff_mps": _safe_float(track.get("track_diff_mps"), 0.0),
        "track_turn_signal": _safe_float(track.get("turn_signal"), 0.0),
        "turn_execution_mode": str(track.get("turn_execution_mode", "")),
        "turn_proximity_m": track.get("turn_proximity_m"),
        "turn_proximity_source": str(track.get("turn_proximity_source", "")),
        "wall_control_state": str(track.get("wall_control_state", "")),
        "corner_turn_active": bool(track.get("corner_turn_active", False)),
        "wall_lost": bool(track.get("wall_lost", False)),
        "wall_profile_confidence": _safe_float(track.get("wall_profile_confidence"), 0.0),
        "pose": pose,
    }


def _sign(value: float, eps: float = 0.015) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def _pose_path_length(samples: List[Dict[str, Any]]) -> float:
    total = 0.0
    prev_pose: Optional[Dict[str, Any]] = None
    for sample in samples:
        pose = dict(sample.get("pose") or {})
        if not all(_is_finite(pose.get(key)) for key in ("x", "y")):
            continue
        if prev_pose is not None:
            step = _pose_distance(prev_pose, pose)
            if math.isfinite(step) and 0.0 <= float(step) <= 0.20:
                total += float(step)
        prev_pose = pose
    return float(total)


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    token = str(args.token)
    motion_source = str(args.motion_source).strip().upper() or "AI"
    started_wall = time.time()
    started_mono = time.monotonic()
    errors: List[str] = []
    samples: List[Dict[str, Any]] = []
    command_ids: List[str] = []
    first_wall_sample: Optional[Dict[str, Any]] = None
    start_cmd: Dict[str, Any] = {}
    stop_cmd: Dict[str, Any] = {}
    final_status: Dict[str, Any] = {}
    active_track_command: Dict[str, Any] = {}
    track_command_started = False
    approach_track_active = False
    follow_chosen_direction = ""
    follow_wall_side = ""
    corner_turn_until_mono: Optional[float] = None

    try:
        precheck = _precheck(token, motion_source=motion_source, stop_timeout_s=float(args.stop_timeout_s))
        st_start = _read_status_with_debug()
        start_pose = _get_pose(st_start)
        approach_v_mps = float(args.v_mps) * float(args.forward_speed_scale)
        track_base_v_mps = float(args.v_mps) * float(args.forward_speed_scale)
        effective_track_diff_mps = float(args.track_diff_mps) * float(args.turn_diff_scale)
        if str(args.approach_control_mode).strip().lower() == "track_velocity":
            start_cmd = _send_track_checked(
                token,
                left_mps=float(approach_v_mps),
                right_mps=float(approach_v_mps),
                motion_source=str(args.track_motion_source),
                timeout_s=4.0,
                reason="WALL_FOLLOW_FIRST_WALL_APPROACH_STRAIGHT_TRACK",
            )
            approach_track_active = True
        else:
            start_cmd = _send_command_checked(
                "set_twist",
                token=token,
                timeout_s=4.0,
                v=float(approach_v_mps),
                omega=0.0,
                motion_source=motion_source,
            )
        command_ids.append(str(start_cmd.get("cmd_id") or ""))
        last_keepalive = time.monotonic()
        phase = "approach_first_wall"
        follow_started_mono: Optional[float] = None
        deadline = time.monotonic() + float(args.approach_timeout_s) + float(args.duration_s) + 8.0

        while time.monotonic() <= deadline:
            now = time.monotonic()
            st = _read_status_with_debug()
            if st:
                final_status = st
                raw_scan = _read_lidar_scan_points()
                front = _front_clearance(st)
                policy = dict(st.get("global_motion_policy") or {})
                wall = dict(policy.get("wall_follow") or {})
                detected_by_front = bool(math.isfinite(front) and front <= float(args.wall_detect_m))
                elapsed = now - started_mono
                sample = _sample(
                    st,
                    elapsed_s=elapsed,
                    phase=phase,
                    track_command=active_track_command,
                    raw_scan=raw_scan,
                )
                samples.append(sample)

                if first_wall_sample is None and detected_by_front:
                    first_wall_sample = dict(sample)
                    follow_started_mono = now
                    corner_turn_until_mono = now + float(args.corner_turn_s)
                    phase = "follow_wall"
                    follow_chosen_direction = _resolve_follow_direction(
                        preferred_turn_direction=str(args.preferred_turn_direction),
                        sample=sample,
                        policy=policy,
                    )
                    first_wall_sample["effective_chosen_direction"] = str(follow_chosen_direction)
                    first_wall_sample["effective_direction_source"] = (
                        "preferred_turn_direction"
                        if _preferred_direction(args.preferred_turn_direction) in ("LEFT", "RIGHT")
                        else "auto_lidar"
                    )
                    follow_wall_side = _resolve_follow_wall_side(
                        sample=first_wall_sample,
                        policy_wall_side=str(sample.get("wall_side") or wall.get("side") or ""),
                        chosen_direction=follow_chosen_direction,
                    )
                    first_wall_sample["effective_wall_side"] = str(follow_wall_side)
                    if str(args.control_mode).strip().lower() == "policy_track":
                        active_track_command = _compute_policy_track_command(
                            st,
                            chosen_direction=follow_chosen_direction,
                            wall_side=follow_wall_side,
                            base_v_mps=float(track_base_v_mps),
                            wall_target_m=float(args.wall_target_m),
                            wall_min_m=float(args.wall_distance_min_m),
                            wall_max_m=float(args.wall_distance_max_m),
                            track_diff_mps=float(effective_track_diff_mps),
                            track_diff_min_mps=float(args.track_diff_min_mps),
                            track_diff_max_mps=float(args.track_diff_max_mps),
                            track_min_inner_mps=float(args.track_min_inner_mps),
                            track_max_mps=float(args.track_max_mps),
                            front_turn_start_m=float(args.front_turn_start_m),
                            front_hard_turn_m=float(args.front_hard_turn_m),
                            wall_distance_gain=float(args.wall_distance_gain),
                            corner_turn_active=bool(
                                corner_turn_until_mono is not None and now <= float(corner_turn_until_mono)
                            ),
                            raw_scan=raw_scan,
                        )
                        track_start = _send_track_checked(
                            token,
                            left_mps=float(active_track_command["left_mps"]),
                            right_mps=float(active_track_command["right_mps"]),
                            motion_source=str(args.track_motion_source),
                            timeout_s=4.0,
                            reason="WALL_FOLLOW_FIRST_WALL_POLICY_TRACK_START",
                        )
                        active_track_command["cmd_status"] = dict(track_start)
                        command_ids.append(str(track_start.get("cmd_id") or ""))
                        track_command_started = True
                        last_keepalive = now

                if follow_started_mono is not None and (now - follow_started_mono) >= float(args.duration_s):
                    break

                if _status_state(st) == "FAILSAFE":
                    errors.append("runtime_entered_failsafe")
                    break

            if now - last_keepalive >= float(args.keepalive_s):
                if track_command_started and str(args.control_mode).strip().lower() == "policy_track":
                    raw_scan = _read_lidar_scan_points()
                    active_track_command = _compute_policy_track_command(
                        st,
                        chosen_direction=str(follow_chosen_direction or "LEFT"),
                        wall_side=str(follow_wall_side or ""),
                        base_v_mps=float(track_base_v_mps),
                        wall_target_m=float(args.wall_target_m),
                        wall_min_m=float(args.wall_distance_min_m),
                        wall_max_m=float(args.wall_distance_max_m),
                        track_diff_mps=float(effective_track_diff_mps),
                        track_diff_min_mps=float(args.track_diff_min_mps),
                        track_diff_max_mps=float(args.track_diff_max_mps),
                        track_min_inner_mps=float(args.track_min_inner_mps),
                        track_max_mps=float(args.track_max_mps),
                        front_turn_start_m=float(args.front_turn_start_m),
                        front_hard_turn_m=float(args.front_hard_turn_m),
                        wall_distance_gain=float(args.wall_distance_gain),
                        corner_turn_active=bool(
                            corner_turn_until_mono is not None and now <= float(corner_turn_until_mono)
                        ),
                        raw_scan=raw_scan,
                    )
                    cmd_id = _append_track(
                        token,
                        left_mps=float(active_track_command["left_mps"]),
                        right_mps=float(active_track_command["right_mps"]),
                        motion_source=str(args.track_motion_source),
                        reason="WALL_FOLLOW_FIRST_WALL_POLICY_TRACK_KEEPALIVE",
                    )
                else:
                    if str(args.approach_control_mode).strip().lower() == "track_velocity":
                        cmd_id = _append_track(
                            token,
                            left_mps=float(approach_v_mps),
                            right_mps=float(approach_v_mps),
                            motion_source=str(args.track_motion_source),
                            reason="WALL_FOLLOW_FIRST_WALL_APPROACH_STRAIGHT_TRACK_KEEPALIVE",
                        )
                    else:
                        cmd_id = _append_command(
                            "set_twist",
                            token=token,
                            v=float(approach_v_mps),
                            omega=0.0,
                            motion_source=motion_source,
                        )
                command_ids.append(str(cmd_id))
                last_keepalive = now

            time.sleep(max(0.02, float(args.poll_s)))

        if first_wall_sample is None:
            errors.append("first_wall_not_detected")
        if first_wall_sample is not None and follow_started_mono is not None:
            followed_s = max(0.0, time.monotonic() - follow_started_mono)
            if followed_s < max(1.0, float(args.duration_s) * 0.95):
                errors.append("wall_follow_duration_short")

        if track_command_started or approach_track_active:
            stop_cmd = _command_zero_track(
                token,
                motion_source=str(args.track_motion_source),
                timeout_s=4.0,
            )
        else:
            stop_cmd = _command_zero(token, motion_source=motion_source, timeout_s=4.0)
        stopped_status = _wait_until_stopped(timeout_s=float(args.stop_timeout_s))
        final_status = _read_status_with_debug() or stopped_status
        end_pose = _get_pose(final_status)
        precheck["start_pose"] = start_pose
        precheck["end_pose"] = end_pose
        precheck["travel_distance_m"] = _pose_distance(start_pose, end_pose)
    except Exception as exc:
        errors.append(str(exc))
        _safe_stop_best_effort(token)
        precheck = locals().get("precheck", {})
        start_pose = _get_pose(_read_status_with_debug())
        end_pose = start_pose

    follow_samples = [s for s in samples if str(s.get("phase")) == "follow_wall"]
    wall_active_samples = [s for s in follow_samples if bool(s.get("wall_follow_active", False))]
    front_values = [float(s["front_clearance_m"]) for s in samples if _is_finite(s.get("front_clearance_m"))]
    wall_distance_values = [
        float(s["wall_distance_m"])
        for s in follow_samples
        if _is_finite(s.get("wall_distance_m"))
    ]
    wall_band_min = min(float(args.wall_distance_min_m), float(args.wall_distance_max_m))
    wall_band_max = max(float(args.wall_distance_min_m), float(args.wall_distance_max_m))
    wall_distance_in_band_samples = [
        value
        for value in wall_distance_values
        if wall_band_min <= float(value) <= wall_band_max
    ]
    wall_below_min_samples = [value for value in wall_distance_values if float(value) < wall_band_min]
    wall_above_max_samples = [value for value in wall_distance_values if float(value) > wall_band_max]
    omega_signs = [_sign(_safe_float(s.get("omega_cmd_rad_s"), 0.0)) for s in follow_samples]
    omega_signs = [s for s in omega_signs if s != 0]
    sign_changes = sum(1 for a, b in zip(omega_signs[:-1], omega_signs[1:]) if a != b)
    wall_sides = [str(s.get("wall_side", "")) for s in wall_active_samples if str(s.get("wall_side", ""))]
    chosen_dirs = [str(s.get("chosen_direction", "")) for s in samples if str(s.get("chosen_direction", ""))]
    wall_control_states = [
        str(s.get("wall_control_state", ""))
        for s in follow_samples
        if str(s.get("wall_control_state", ""))
    ]
    follow_elapsed_s = 0.0
    if first_wall_sample is not None and follow_samples:
        follow_elapsed_s = max(0.0, float(follow_samples[-1].get("elapsed_s", 0.0)) - float(first_wall_sample.get("elapsed_s", 0.0)))

    policy_first_wall_observed = bool(
        first_wall_sample is not None
        and (
            bool(first_wall_sample.get("wall_policy_active", False))
            or bool(first_wall_sample.get("wall_follow_active", False))
            or bool(track_command_started)
        )
    )
    if first_wall_sample is not None and not bool(policy_first_wall_observed):
        errors.append("wall_follow_policy_not_observed")
    follow_forward_cmd_samples = [
        s
        for s in follow_samples
        if max(
            _safe_float(s.get("v_cmd_mps"), 0.0),
            _safe_float(s.get("commanded_linear_mps"), 0.0),
        )
        >= float(args.min_forward_cmd_mps)
    ]
    track_command_samples = [s for s in follow_samples if bool(s.get("track_wall_controller_active", False))]
    turn_execution_modes = [
        str(s.get("turn_execution_mode", ""))
        for s in track_command_samples
        if str(s.get("turn_execution_mode", ""))
    ]
    single_track_arc_samples = [
        s for s in track_command_samples if str(s.get("turn_execution_mode", "")) == "single_track_arc"
    ]
    track_inner_sample_values = [
        (
            s,
            min(
                _safe_float(s.get("commanded_left_track_mps"), 0.0),
                _safe_float(s.get("commanded_right_track_mps"), 0.0),
            ),
        )
        for s in track_command_samples
    ]
    track_inner_values = [v for _, v in track_inner_sample_values]
    low_inner_track_values = [
        v
        for s, v in track_inner_sample_values
        if str(s.get("turn_execution_mode", "")) != "single_track_arc"
        and float(v) < float(args.validation_min_inner_track_mps)
    ]
    low_inner_track_ratio = (
        float(len(low_inner_track_values)) / float(len(track_inner_values)) if track_inner_values else 0.0
    )
    single_track_arc_ratio = (
        float(len(single_track_arc_samples)) / float(len(track_command_samples)) if track_command_samples else 0.0
    )
    tight_turn_values = [
        (
            _safe_float(s.get("commanded_track_diff_mps"), 0.0)
            / max(1e-6, _safe_float(s.get("commanded_linear_mps"), 0.0))
        )
        for s in track_command_samples
        if str(s.get("turn_execution_mode", "")) != "single_track_arc"
        and _safe_float(s.get("commanded_linear_mps"), 0.0) > 0.0
    ]
    tight_turn_samples = [value for value in tight_turn_values if float(value) > float(args.max_tight_turn_ratio)]
    tight_turn_sample_ratio = (
        float(len(tight_turn_samples)) / float(len(tight_turn_values)) if tight_turn_values else 0.0
    )
    saturated_turn_samples = [
        s for s in track_command_samples if abs(_safe_float(s.get("track_turn_signal"), 0.0)) >= 0.95
    ]
    saturated_turn_ratio = (
        float(len(saturated_turn_samples)) / float(len(track_command_samples)) if track_command_samples else 0.0
    )
    follow_forward_cmd_ratio = (
        float(len(follow_forward_cmd_samples)) / float(len(follow_samples)) if follow_samples else 0.0
    )
    follow_distance_m = 0.0
    follow_net_distance_m = 0.0
    follow_path_length_m = _pose_path_length(follow_samples)
    if first_wall_sample is not None and final_status:
        first_pose = dict(first_wall_sample.get("pose") or {})
        end_pose = _get_pose(final_status)
        if all(_is_finite(first_pose.get(k)) and _is_finite(end_pose.get(k)) for k in ("x", "y")):
            follow_net_distance_m = _pose_distance(first_pose, end_pose)
    follow_distance_m = max(float(follow_net_distance_m), float(follow_path_length_m))
    follow_path_net_ratio = (
        float(follow_path_length_m) / max(0.05, float(follow_net_distance_m))
        if follow_path_length_m > 0.0
        else 0.0
    )
    if first_wall_sample is not None and follow_forward_cmd_ratio < float(args.min_forward_cmd_ratio):
        errors.append("forward_cmd_ratio_too_low")
    if first_wall_sample is not None and follow_distance_m < float(args.min_follow_distance_m):
        errors.append("follow_distance_too_short")
    if first_wall_sample is not None and low_inner_track_ratio > float(args.max_low_inner_track_ratio):
        errors.append("inner_track_too_low_spin_risk")
    if first_wall_sample is not None and tight_turn_sample_ratio > float(args.max_tight_turn_sample_ratio):
        errors.append("tight_turn_ratio_too_high")
    if first_wall_sample is not None and saturated_turn_ratio > float(args.max_saturated_turn_ratio):
        errors.append("turn_saturation_ratio_too_high")
    if first_wall_sample is not None and follow_path_net_ratio > float(args.max_follow_path_net_ratio):
        errors.append("follow_path_net_ratio_too_high")
    min_front = min(front_values) if front_values else math.nan
    if math.isfinite(min_front) and min_front < float(args.validation_min_front_clearance_m):
        errors.append("front_clearance_below_validation_min")
    wall_distance_sample_ratio = (
        float(len(wall_distance_values)) / float(len(follow_samples)) if follow_samples else 0.0
    )
    wall_distance_in_band_ratio = (
        float(len(wall_distance_in_band_samples)) / float(len(wall_distance_values)) if wall_distance_values else 0.0
    )
    wall_below_min_ratio = (
        float(len(wall_below_min_samples)) / float(len(wall_distance_values)) if wall_distance_values else 0.0
    )
    wall_above_max_ratio = (
        float(len(wall_above_max_samples)) / float(len(wall_distance_values)) if wall_distance_values else 0.0
    )
    median_wall_distance = statistics.median(wall_distance_values) if wall_distance_values else math.nan
    if first_wall_sample is not None and wall_distance_sample_ratio < float(args.min_wall_distance_sample_ratio):
        errors.append("wall_distance_sample_ratio_too_low")
    if (
        first_wall_sample is not None
        and math.isfinite(median_wall_distance)
        and median_wall_distance > float(args.max_median_wall_distance_m)
    ):
        errors.append("median_wall_distance_too_high")
    if (
        first_wall_sample is not None
        and math.isfinite(median_wall_distance)
        and median_wall_distance < float(args.min_median_wall_distance_m)
    ):
        errors.append("median_wall_distance_too_low")
    if first_wall_sample is not None and wall_distance_in_band_ratio < float(args.min_wall_distance_in_band_ratio):
        errors.append("wall_distance_in_band_ratio_too_low")
    if first_wall_sample is not None and wall_below_min_ratio > float(args.max_wall_below_min_ratio):
        errors.append("wall_distance_below_min_ratio_too_high")
    if first_wall_sample is not None and wall_above_max_ratio > float(args.max_wall_above_max_ratio):
        errors.append("wall_distance_above_max_ratio_too_high")

    success = bool(not errors)
    summary = {
        "success": bool(success),
        "test_name": str(args.test_name),
        "started_at": float(started_wall),
        "duration_s": round(float(time.monotonic() - started_mono), 3),
        "motion_source": motion_source,
        "command_path": (
            "straight set_track_velocity approach via runtime/commands.jsonl, then explicit left/right set_track_velocity wall following"
            if bool(approach_track_active)
            else "set_twist approach via runtime/commands.jsonl, then explicit left/right set_track_velocity wall following"
            if bool(track_command_started)
            else "set_twist via runtime/commands.jsonl, zero-omega forward request, policy-owned wall following"
        ),
        "requested": {
            "start": "straight forward",
            "wall_follow_duration_s": float(args.duration_s),
            "v_mps": float(args.v_mps),
            "effective_forward_v_mps": round(float(args.v_mps) * float(args.forward_speed_scale), 5),
            "forward_speed_scale": float(args.forward_speed_scale),
            "wall_detect_m": float(args.wall_detect_m),
            "corner_turn_s": float(args.corner_turn_s),
            "control_mode": str(args.control_mode),
            "approach_control_mode": str(args.approach_control_mode),
            "preferred_turn_direction": _preferred_direction(args.preferred_turn_direction),
            "track_diff_mps": round(float(args.track_diff_mps) * float(args.turn_diff_scale), 5),
            "turn_diff_scale": float(args.turn_diff_scale),
            "wall_target_m": float(args.wall_target_m),
            "wall_distance_min_m": float(args.wall_distance_min_m),
            "wall_distance_max_m": float(args.wall_distance_max_m),
        },
        "first_wall_detected": bool(first_wall_sample is not None),
        "first_wall_sample": first_wall_sample or {},
        "follow_elapsed_s": round(float(follow_elapsed_s), 3),
        "sample_count": int(len(samples)),
        "follow_sample_count": int(len(follow_samples)),
        "wall_active_sample_count": int(len(wall_active_samples)),
        "wall_active_ratio_in_follow": (
            round(float(len(wall_active_samples)) / float(len(follow_samples)), 4) if follow_samples else 0.0
        ),
        "policy_first_wall_observed": bool(policy_first_wall_observed),
        "track_command_started": bool(track_command_started),
        "forward_cmd_ratio_in_follow": round(float(follow_forward_cmd_ratio), 4),
        "forward_cmd_sample_count": int(len(follow_forward_cmd_samples)),
        "follow_distance_m": round(float(follow_distance_m), 4),
        "follow_net_distance_m": round(float(follow_net_distance_m), 4),
        "follow_path_length_m": round(float(follow_path_length_m), 4),
        "follow_path_net_ratio": round(float(follow_path_net_ratio), 4),
        "min_front_clearance_m": (None if not math.isfinite(min_front) else round(float(min_front), 4)),
        "median_front_clearance_m": (
            round(float(statistics.median(front_values)), 4) if front_values else None
        ),
        "median_wall_distance_m": (
            round(float(median_wall_distance), 4) if math.isfinite(median_wall_distance) else None
        ),
        "wall_distance_sample_ratio": round(float(wall_distance_sample_ratio), 4),
        "wall_distance_in_band_ratio": round(float(wall_distance_in_band_ratio), 4),
        "wall_distance_below_min_ratio": round(float(wall_below_min_ratio), 4),
        "wall_distance_above_max_ratio": round(float(wall_above_max_ratio), 4),
        "wall_distance_min_m": float(args.wall_distance_min_m),
        "wall_distance_max_m": float(args.wall_distance_max_m),
        "min_commanded_inner_track_mps": (
            round(float(min(track_inner_values)), 5) if track_inner_values else None
        ),
        "low_inner_track_ratio": round(float(low_inner_track_ratio), 4),
        "single_track_arc_ratio": round(float(single_track_arc_ratio), 4),
        "tight_turn_sample_ratio": round(float(tight_turn_sample_ratio), 4),
        "max_track_diff_to_linear_ratio": (
            round(float(max(tight_turn_values)), 4) if tight_turn_values else None
        ),
        "saturated_turn_ratio": round(float(saturated_turn_ratio), 4),
        "chosen_direction_counts": {item: chosen_dirs.count(item) for item in sorted(set(chosen_dirs))},
        "wall_side_counts": {item: wall_sides.count(item) for item in sorted(set(wall_sides))},
        "wall_control_state_counts": {
            item: wall_control_states.count(item) for item in sorted(set(wall_control_states))
        },
        "turn_execution_mode_counts": {
            item: turn_execution_modes.count(item) for item in sorted(set(turn_execution_modes))
        },
        "omega_sign_changes_in_follow": int(sign_changes),
        "precheck": precheck,
        "commands": {
            "start": start_cmd,
            "stop": stop_cmd,
            "keepalive_count": max(0, len(command_ids) - 1),
        },
        "final_state": _status_state(final_status),
        "errors": list(errors),
        "artifact_paths": [
            str(LATEST_RESULT_PATH.relative_to(PROJECT_ROOT)),
            str(LATEST_SUMMARY_PATH.relative_to(PROJECT_ROOT)),
        ],
    }
    result = {
        "success": bool(success),
        "summary": summary,
        "samples_tail": samples[-80:],
        "samples": samples if not bool(args.compact) else [],
    }
    _write_json_atomic(LATEST_RESULT_PATH, result)
    _write_json_atomic(LATEST_SUMMARY_PATH, summary)
    _append_jsonl(HISTORY_PATH, summary)
    return result if not bool(args.compact) else summary


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Validate first-wall wall following live.")
    ap.add_argument("--test-name", default="wall_follow_first_wall_live")
    ap.add_argument("--token", default=DEFAULT_TOKEN)
    ap.add_argument("--motion-source", default="AI")
    ap.add_argument("--approach-control-mode", choices=("set_twist", "track_velocity"), default="track_velocity")
    ap.add_argument("--control-mode", choices=("set_twist", "policy_track"), default="policy_track")
    ap.add_argument("--track-motion-source", default="STATE")
    ap.add_argument("--preferred-turn-direction", choices=("AUTO", "LEFT", "RIGHT", "auto", "left", "right"), default="AUTO")
    ap.add_argument("--duration-s", type=float, default=60.0)
    ap.add_argument("--approach-timeout-s", type=float, default=45.0)
    ap.add_argument("--v-mps", type=float, default=0.065)
    ap.add_argument("--forward-speed-scale", type=float, default=0.70)
    ap.add_argument("--wall-detect-m", type=float, default=0.95)
    ap.add_argument("--corner-turn-s", type=float, default=2.8)
    ap.add_argument("--wall-target-m", type=float, default=0.62)
    ap.add_argument("--wall-distance-min-m", type=float, default=0.50)
    ap.add_argument("--wall-distance-max-m", type=float, default=0.75)
    ap.add_argument("--track-diff-mps", type=float, default=0.032)
    ap.add_argument("--turn-diff-scale", type=float, default=1.80)
    ap.add_argument("--track-diff-min-mps", type=float, default=0.026)
    ap.add_argument("--track-diff-max-mps", type=float, default=0.045)
    ap.add_argument("--track-min-inner-mps", type=float, default=0.022)
    ap.add_argument("--track-max-mps", type=float, default=0.080)
    ap.add_argument("--front-turn-start-m", type=float, default=1.00)
    ap.add_argument("--front-hard-turn-m", type=float, default=0.42)
    ap.add_argument("--wall-distance-gain", type=float, default=1.35)
    ap.add_argument("--validation-min-front-clearance-m", type=float, default=0.25)
    ap.add_argument("--min-forward-cmd-mps", type=float, default=0.02)
    ap.add_argument("--min-forward-cmd-ratio", type=float, default=0.35)
    ap.add_argument("--min-follow-distance-m", type=float, default=0.60)
    ap.add_argument("--min-wall-distance-sample-ratio", type=float, default=0.50)
    ap.add_argument("--min-median-wall-distance-m", type=float, default=0.50)
    ap.add_argument("--max-median-wall-distance-m", type=float, default=0.75)
    ap.add_argument(
        "--min-wall-distance-in-band-ratio",
        "--min-wall-target-window-ratio",
        dest="min_wall_distance_in_band_ratio",
        type=float,
        default=0.45,
    )
    ap.add_argument("--max-wall-below-min-ratio", type=float, default=0.20)
    ap.add_argument("--max-wall-above-max-ratio", type=float, default=0.45)
    ap.add_argument("--validation-min-inner-track-mps", type=float, default=0.020)
    ap.add_argument("--max-low-inner-track-ratio", type=float, default=0.20)
    ap.add_argument("--max-tight-turn-ratio", type=float, default=0.95)
    ap.add_argument("--max-tight-turn-sample-ratio", type=float, default=0.25)
    ap.add_argument("--max-saturated-turn-ratio", type=float, default=0.10)
    ap.add_argument("--max-follow-path-net-ratio", type=float, default=8.0)
    ap.add_argument("--keepalive-s", type=float, default=0.18)
    ap.add_argument("--poll-s", type=float, default=0.08)
    ap.add_argument("--stop-timeout-s", type=float, default=5.0)
    ap.add_argument("--compact", action="store_true")
    return ap


def main() -> int:
    args = build_parser().parse_args()
    try:
        payload = run(args)
    except BootstrapGuardError as exc:
        payload = {"success": False, "errors": [str(exc)]}
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    return 0 if bool(payload.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
