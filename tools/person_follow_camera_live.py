#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bounded live person-follow smoke through the camera FOLLOW path."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.command_bus import get_latest_command_status  # noqa: E402
from log.log_paths import test_artifacts_dir  # noqa: E402

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
STATUS_PATH = RUNTIME_DIR / "status.json"
COMMANDS_PATH = RUNTIME_DIR / "commands.jsonl"
RESULT_PATH = AGENT_TESTS_DIR / "latest_person_follow_camera_live.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_person_follow_camera_live_summary.json"
HISTORY_PATH = AGENT_TESTS_DIR / "person_follow_camera_live_samples.jsonl"
FOLLOW_ACCEPT_TARGET_DISTANCE_MIN_M = 1.00
FOLLOW_ACCEPT_TARGET_DISTANCE_MAX_M = 1.45
FOLLOW_ACCEPT_TARGET_DISTANCE_ERROR_P90_M = 0.45
FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P50_DEG = 18.0
FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P90_DEG = 50.0
FOLLOW_ACCEPT_TARGET_LOST_GAP_MAX_S = 2.0
FOLLOW_PREFOLLOW_MIN_FRONT_CLEARANCE_M = 0.55
MOTION_EXPECTED_EXECUTED_V_ERROR_P90_MAX_MPS = 0.006
MOTION_EXPECTED_EXECUTED_OMEGA_ERROR_P90_MAX_RAD_S = 0.060
MOTION_EXPECTED_EXECUTED_TRACK_ERROR_P90_MAX_MPS = 0.006
WALL_STICK_CLOSE_M = 0.56
WALL_STICK_LONGEST_RUN_MAX_S = 0.40
FINAL_TARGET_WINDOW_S = 3.0
FINAL_TARGET_DISTANCE_ERROR_MAX_M = 0.25
HUMAN_FOLLOW_V2_ROUTE = "human_follow_v2"
CANONICAL_CONTROL_MODE = "UNIFIED"
HUMAN_FOLLOW_V2_FRESH_TARGET_OMEGA_MAX_RAD_S = 0.43
HUMAN_FOLLOW_V2_OMEGA_P90_MAX_RAD_S = 0.50
HUMAN_FOLLOW_V2_TARGET_DISTANCE_MIN_M = 0.85
HUMAN_FOLLOW_V2_TARGET_DISTANCE_MAX_M = 1.20
HUMAN_FOLLOW_V2_TARGET_DISTANCE_ERROR_P50_MAX_M = 0.20
HUMAN_FOLLOW_V2_TARGET_DISTANCE_ERROR_P90_MAX_M = 0.35
HUMAN_FOLLOW_V2_SETTLED_DISTANCE_AFTER_FIRST_TARGET_S = 8.0
HUMAN_FOLLOW_V2_SETTLED_DISTANCE_MIN_SAMPLES = 12
HUMAN_FOLLOW_V2_BEARING_ERROR_P50_MAX_DEG = 12.0
HUMAN_FOLLOW_V2_BEARING_ERROR_P90_MAX_DEG = 25.0
HUMAN_FOLLOW_V2_BEARING_MIN_STABLE_SAMPLE_FRACTION = 0.25
HUMAN_FOLLOW_V2_BEARING_STABLE_PHASES = {
    "camera_target_center_forward",
    "camera_target_center_hold",
    "target_hold",
}
HUMAN_FOLLOW_V2_BEARING_STABLE_APPROACH_PHASES = {
    "follow_caution_approach",
    "follow_direct_approach",
}
HUMAN_FOLLOW_V2_APPROACH_STABLE_BEARING_MAX_DEG = 24.0
HUMAN_FOLLOW_V2_ALLOWED_CRUISE_FALLBACK_PHASES = {
    "candidate_hold_zero_track",
    "camera_detection_clearance_retreat",
    "camera_detection_required_hold",
    "collision_stop",
    "front_hold_camera_retreat",
    "front_warning_camera_retreat",
    "front_warning_follow_hold",
    "lidar_confidence_hold",
    "obstacle_stop_hold",
    "target_hold",
    "target_hold_heading_align",
    "target_hold_heading_blocked",
    "target_reacquire_hold",
    "target_reacquire_in_place",
    "target_reacquire_rotate",
    "target_search_hold",
    "target_search_in_place",
    "target_search_one_track",
}
FOLLOW_START_FIRST_LOCK_MAX_S = 12.0
FOLLOW_POST_LOCK_UNHANDLED_MAX_S = 2.0
FOLLOW_POST_LOCK_CANDIDATE_HOLD_MAX_S = 2.0
FOLLOW_TARGET_ACTIVE_ZERO_GAP_MAX_S = 1.20
FOLLOW_COMMAND_DELTA_V_P90_MAX_MPS = 0.080
FOLLOW_COMMAND_DELTA_OMEGA_P90_MAX_RAD_S = 0.220
FOLLOW_COMMAND_DELTA_OMEGA_MAX_RAD_S = 0.520
FOLLOW_REVERSE_LONGEST_RUN_MAX_S = 2.5
FOLLOW_TRANSLATION_LONGEST_RUN_MAX_S = 8.0
FOLLOW_SIDE_FLIP_MAX_COUNT = 12
FOLLOW_MIN_SAMPLE_COVERAGE_FRACTION = 0.75
FOLLOW_SEARCH_PIVOT_OMEGA_MIN_RAD_S = 0.02
FOLLOW_SEARCH_PIVOT_OMEGA_MAX_RAD_S = 0.30
HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO = 0.60
HUMAN_FOLLOW_V2_MAX_SEARCH_RATIO = 0.20
HUMAN_FOLLOW_V2_MAX_CONTIGUOUS_SEARCH_PIVOT_S = 15.0
HUMAN_FOLLOW_V2_MAX_LOCALIZATION_BLOCK_RATIO = 0.05
HUMAN_FOLLOW_V2_MIN_RESPONSE_MOTION_RATIO = 0.30
HUMAN_FOLLOW_V2_MAX_RESPONSE_NO_MOTION_S = 2.0
HUMAN_FOLLOW_V2_RESPONSE_BEARING_REQUIRED_DEG = 10.0
HUMAN_FOLLOW_V2_RESPONSE_DISTANCE_ERROR_REQUIRED_M = 0.10


def _follow_distance_acceptance(follow_distance_m: float) -> Dict[str, float]:
    target = max(0.30, float(follow_distance_m))
    return {
        "min_m": round(max(0.30, target - 0.20), 3),
        "max_m": round(target + 0.25, 3),
        "error_p90_m": round(max(0.30, target * 0.80), 3),
    }


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _append_jsonl(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def _append_command(cmd_type: str, *, token: str, **kwargs: Any) -> str:
    cmd_id = str(uuid.uuid4())[:8]
    entry = {
        "cmd_id": cmd_id,
        "type": str(cmd_type),
        "token": str(token),
        "ts": time.time(),
        **kwargs,
    }
    COMMANDS_PATH.parent.mkdir(parents=True, exist_ok=True)
    with COMMANDS_PATH.open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False, separators=(",", ":")) + "\n")
    return cmd_id


def _wait_command_effective(cmd_id: str, *, timeout_s: float = 4.0) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        row = get_latest_command_status(str(cmd_id), max_lines=80)
        if isinstance(row, dict):
            state = str(row.get("state", "") or "").lower()
            if state in {"effective", "failed"}:
                return row
        time.sleep(0.05)
    return {"cmd_id": str(cmd_id), "state": "failed", "reason": "timeout"}


def _send_command(cmd_type: str, *, token: str, timeout_s: float = 4.0, **kwargs: Any) -> Dict[str, Any]:
    cmd_id = _append_command(cmd_type, token=token, **kwargs)
    status = _wait_command_effective(cmd_id, timeout_s=timeout_s)
    return {
        "cmd_id": cmd_id,
        "cmd_type": str(cmd_type),
        "effective": str(status.get("state", "") or "").lower() == "effective",
        "status": status,
    }


def _normalize_control_mode(value: Any) -> str:
    return str(value or "").strip().upper()


def _ensure_control_mode(
    mode: Any,
    *,
    token: str,
    command_results: List[Dict[str, Any]],
    timeout_s: float = 8.0,
) -> Dict[str, Any]:
    target = _normalize_control_mode(mode)
    status = _read_json(STATUS_PATH)
    applied = _normalize_control_mode(status.get("control_mode"))
    original = applied
    error = ""
    if target != CANONICAL_CONTROL_MODE:
        error = f"unsupported_control_mode:{target or 'MISSING'}"
    elif applied != target:
        error = f"control_mode_not_unified:{applied or 'MISSING'}"
    return {
        "requested": str(mode or ""),
        "target": str(target),
        "original": str(original),
        "applied": str(applied),
        "changed": False,
        "ok": not error,
        "error": error,
    }


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _safe_optional_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        out = float(value)
        if not math.isfinite(out):
            return None
        return out
    except Exception:
        return None


def _percentile(values: List[float], fraction: float) -> float | None:
    vals = sorted(float(v) for v in values if v is not None)
    if not vals:
        return None
    idx = int(round((len(vals) - 1) * max(0.0, min(1.0, float(fraction)))))
    return round(float(vals[idx]), 3)


def _longest_run(samples: List[Dict[str, Any]], predicate) -> int:
    best = 0
    cur = 0
    for sample in samples:
        if predicate(sample):
            cur += 1
            best = max(best, cur)
        else:
            cur = 0
    return int(best)


def _analysis_duration_s(samples: List[Dict[str, Any]], requested_duration_s: Any) -> float:
    duration = _safe_float(requested_duration_s, 0.0)
    max_elapsed = max((_safe_float(s.get("elapsed_s"), 0.0) for s in list(samples or [])), default=0.0)
    return max(0.001, float(duration), float(max_elapsed))


def _sample_interval_s(samples: List[Dict[str, Any]], idx: int, duration_s: float) -> float:
    if idx < 0 or idx >= len(samples):
        return 0.0
    start = max(0.0, _safe_float(samples[idx].get("elapsed_s"), 0.0))
    if idx + 1 < len(samples):
        end = _safe_float(samples[idx + 1].get("elapsed_s"), float(duration_s))
    else:
        end = float(duration_s)
    start = min(float(start), float(duration_s))
    end = min(max(float(end), float(start)), float(duration_s))
    return max(0.0, float(end) - float(start))


def _timed_sum_s(samples: List[Dict[str, Any]], duration_s: float, predicate) -> float:
    total = 0.0
    for idx, sample in enumerate(list(samples or [])):
        if predicate(sample):
            total += _sample_interval_s(samples, idx, duration_s)
    return float(total)


def _timed_longest_run_s(samples: List[Dict[str, Any]], duration_s: float, predicate) -> float:
    best = 0.0
    cur = 0.0
    for idx, sample in enumerate(list(samples or [])):
        if predicate(sample):
            cur += _sample_interval_s(samples, idx, duration_s)
            best = max(best, cur)
        else:
            cur = 0.0
    return float(best)


def _camera_search_or_pivot_sample(sample: Dict[str, Any]) -> bool:
    phase = str(sample.get("room_cruise_phase") or "")
    return bool(
        str(sample.get("follow_target_source") or "") == "CAMERA_SEARCH"
        or bool(sample.get("target_search_active", False))
        or str(sample.get("target_search_state") or "") == "searching"
        or str(sample.get("target_camera_state") or "") == "target_search_scan"
        or phase in {
            "target_search_arc",
            "target_search_rotate_360",
            "target_search_one_track",
            "target_search_in_place",
            "target_search_hold",
        }
    )


def _actual_camera_follow_sample(sample: Dict[str, Any]) -> bool:
    return bool(
        str(sample.get("state") or "").upper() == "FOLLOW"
        and str(sample.get("follow_target_source") or "") == "CAMERA_TARGET"
        and _camera_detection_active(sample)
        and not _camera_search_or_pivot_sample(sample)
        and not bool(sample.get("localization_gate_hard_stop", False))
    )


def _camera_target_response_required_sample(sample: Dict[str, Any]) -> bool:
    if not (
        str(sample.get("follow_target_source") or "") == "CAMERA_TARGET"
        and _camera_detection_active(sample)
        and not _camera_search_or_pivot_sample(sample)
    ):
        return False
    bearing_deg = _angle_error_abs_deg(_safe_float(sample.get("follow_actual_bearing_rad"), 0.0))
    distance_m = _safe_float(sample.get("follow_actual_distance_m"), 0.0)
    if distance_m <= 0.0:
        distance_m = _safe_float(sample.get("target_camera_distance_used_m"), 0.0)
    desired_m = max(0.01, _safe_float(sample.get("follow_desired_distance_m"), 1.0))
    distance_error_m = abs(float(distance_m) - float(desired_m)) if distance_m > 0.0 else 0.0
    phase = str(sample.get("room_cruise_phase") or "")
    return bool(
        float(bearing_deg) >= HUMAN_FOLLOW_V2_RESPONSE_BEARING_REQUIRED_DEG
        or float(distance_error_m) >= HUMAN_FOLLOW_V2_RESPONSE_DISTANCE_ERROR_REQUIRED_M
        or phase
        in {
            "target_heading_align",
            "target_hold_heading_align",
            "camera_target_in_place_align",
            "camera_target_one_track_align",
            "follow_caution_approach",
            "follow_direct_approach",
        }
    )


def _camera_target_source_sample(sample: Dict[str, Any]) -> bool:
    return bool(str(sample.get("follow_target_source") or "") == "CAMERA_TARGET")


def _target_lost_sample(sample: Dict[str, Any]) -> bool:
    return bool(
        _camera_search_or_pivot_sample(sample)
        or str(sample.get("target_camera_gate") or "") == "target_lost_search"
        or not bool(sample.get("target_camera_visible", False))
    )


def _search_follow_search_cycles_per_min(samples: List[Dict[str, Any]], duration_s: float) -> tuple[int, float]:
    sequence: List[str] = []
    for sample in list(samples or []):
        label = (
            "SEARCH"
            if _camera_search_or_pivot_sample(sample)
            else (
                "FOLLOW"
                if str(sample.get("state") or "").upper() == "FOLLOW"
                and _camera_target_source_sample(sample)
                else "OTHER"
            )
        )
        if not sequence or sequence[-1] != label:
            sequence.append(label)
    compact = [label for label in sequence if label in {"SEARCH", "FOLLOW"}]
    cycles = 0
    for idx in range(0, max(0, len(compact) - 2)):
        if compact[idx : idx + 3] == ["SEARCH", "FOLLOW", "SEARCH"]:
            cycles += 1
    per_min = float(cycles) / max(0.001, float(duration_s) / 60.0)
    return int(cycles), round(float(per_min), 3)


def _relock_stable_follow_durations_s(samples: List[Dict[str, Any]], duration_s: float) -> List[float]:
    durations: List[float] = []
    pending_relock = False
    idx = 0
    data = list(samples or [])
    while idx < len(data):
        sample = data[idx]
        if _target_lost_sample(sample):
            pending_relock = True
        if pending_relock and _actual_camera_follow_sample(sample):
            start = _safe_float(sample.get("elapsed_s"), 0.0)
            end_idx = idx
            while end_idx < len(data) and _actual_camera_follow_sample(data[end_idx]):
                end_idx += 1
            end = _safe_float(data[end_idx].get("elapsed_s"), duration_s) if end_idx < len(data) else duration_s
            durations.append(round(max(0.0, float(end) - float(start)), 3))
            pending_relock = False
            idx = end_idx
            continue
        idx += 1
    return durations


def _first_elapsed_s(samples: List[Dict[str, Any]], predicate) -> float | None:
    for sample in list(samples or []):
        if predicate(sample):
            return round(_safe_float(sample.get("elapsed_s"), 0.0), 3)
    return None


def _post_acquire_samples(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    acquired_once = False
    out: List[Dict[str, Any]] = []
    for sample in list(samples or []):
        source = str(sample.get("follow_target_source") or "")
        if source == "CAMERA_TARGET" or _camera_detection_active(sample):
            acquired_once = True
        if bool(acquired_once):
            out.append(sample)
    return out


def _command_delta_samples(samples: List[Dict[str, Any]], key: str) -> List[float]:
    deltas: List[float] = []
    for prev, cur in zip(list(samples or []), list(samples or [])[1:]):
        prev_elapsed = _safe_float(prev.get("elapsed_s"), 0.0)
        cur_elapsed = _safe_float(cur.get("elapsed_s"), 0.0)
        if cur_elapsed > 0.0 and prev_elapsed > 0.0 and cur_elapsed - prev_elapsed > 0.75:
            continue
        deltas.append(abs(_safe_float(cur.get(key), 0.0) - _safe_float(prev.get(key), 0.0)))
    return deltas


def _target_active_zero_gap_sample(sample: Dict[str, Any]) -> bool:
    if bool(sample.get("localization_gate_hard_stop", False)) or not bool(sample.get("localization_gate_allow_motion", True)):
        return False
    if not _camera_detection_active(sample):
        return False
    if str(sample.get("follow_target_source") or "") != "CAMERA_TARGET":
        return False
    desired_m = max(0.01, _safe_float(sample.get("follow_desired_distance_m"), 1.0))
    distance_m = _safe_float(sample.get("target_camera_distance_used_m"), 0.0)
    if distance_m <= 0.0:
        distance_m = _safe_float(sample.get("follow_actual_distance_m"), 0.0)
    if distance_m <= desired_m + 0.18:
        return False
    return bool(
        abs(_safe_float(sample.get("cmd_v"), 0.0)) <= 0.002
        and abs(_safe_float(sample.get("cmd_omega"), 0.0)) <= 0.035
        and abs(_safe_float(sample.get("track_left_mps"), 0.0)) <= 0.004
        and abs(_safe_float(sample.get("track_right_mps"), 0.0)) <= 0.004
    )


def _angle_delta_deg(current: float, previous: float) -> float:
    return (float(current) - float(previous) + 180.0) % 360.0 - 180.0


def _angle_error_abs_deg(rad: float) -> float:
    return abs(math.degrees((float(rad) + math.pi) % (2.0 * math.pi) - math.pi))


def _emergency_count(status: Dict[str, Any]) -> int:
    try:
        return int(((status or {}).get("last_emergency") or {}).get("count", 0) or 0)
    except Exception:
        return 0


def _new_emergency_observed(samples: List[Dict[str, Any]], initial_count: int) -> bool:
    for sample in list(samples or []):
        try:
            if int(sample.get("last_emergency_count", 0) or 0) > int(initial_count):
                return True
        except Exception:
            continue
    return False


def _global_motion_failsafe_count(status: Dict[str, Any]) -> int:
    try:
        policy = dict((status or {}).get("global_motion_policy") or {})
        stats = dict(policy.get("runtime_stats") or {})
        return int(stats.get("failsafe_events", 0) or 0)
    except Exception:
        return 0


def _camera_detection_active(sample: Dict[str, Any]) -> bool:
    return bool(
        bool(sample.get("target_camera_visible", False))
        and bool(sample.get("target_camera_usable", False))
        and not bool(sample.get("target_camera_stale", False))
        and str(sample.get("target_camera_detector") or "") not in {"", "none", "unknown"}
    )


def _allowed_camera_search_motion(sample: Dict[str, Any]) -> bool:
    if str(sample.get("follow_target_source") or "") != "CAMERA_SEARCH":
        return False
    return str(sample.get("room_cruise_phase") or "") in {
        "target_search_in_place",
        "target_reacquire_in_place",
    }


def _sample_has_motion(sample: Dict[str, Any]) -> bool:
    twist_motion = bool(
        abs(_safe_float(sample.get("cmd_v"), 0.0)) >= 0.004
        or abs(_safe_float(sample.get("cmd_omega"), 0.0)) >= 0.035
    )
    pwm_motion = bool(
        abs(_safe_float(sample.get("pwm_left"), 0.0)) >= 0.02
        or abs(_safe_float(sample.get("pwm_right"), 0.0)) >= 0.02
    )
    track_motion = bool(
        abs(_safe_float(sample.get("track_left_mps"), 0.0)) >= 0.004
        or abs(_safe_float(sample.get("track_right_mps"), 0.0)) >= 0.004
    )
    return bool(
        twist_motion
        or pwm_motion
        or (
            track_motion
            and (
                pwm_motion
                or twist_motion
                or str(sample.get("command_type") or "").strip().lower() == "set_track_velocity"
            )
        )
    )


def _direct_motor_bypass_sample(sample: Dict[str, Any]) -> bool:
    command_type = str(sample.get("command_type") or "").strip().lower()
    entry_tier = str(sample.get("entry_tier") or "").strip().upper()
    return bool(
        command_type == "set_motor_pwm"
        or entry_tier == "SERVICE"
        or bool(sample.get("service_motion_active", False))
        or bool(sample.get("actuator_service_active", False))
    )


def _legacy_generic_planner_sample(sample: Dict[str, Any]) -> bool:
    target_source = str(sample.get("follow_target_source") or "")
    command_type = str(sample.get("command_type") or "").strip().lower()
    layer = str(sample.get("resolved_layer") or "").strip().upper()
    entry_tier = str(sample.get("entry_tier") or "").strip().upper()
    phase = str(sample.get("room_cruise_phase") or "")
    if entry_tier == "LEGACY" or layer == "LEGACY_TANK_ADAPTER":
        return True
    if command_type in {"adaptive_direct", "search_person", "search_person_rotate"}:
        return True
    if target_source == "CAMERA_TARGET" and _sample_has_motion(sample):
        if (
            command_type != "local_planner_segment"
            and phase not in HUMAN_FOLLOW_V2_ALLOWED_CRUISE_FALLBACK_PHASES
        ):
            return True
    if target_source == "CAMERA_SEARCH" and _sample_has_motion(sample):
        if command_type != "local_planner_segment" or layer != "LOCAL_NAVIGATION" or phase != "target_search_in_place":
            return True
    return False


def _active_route_for_sample(sample: Dict[str, Any]) -> str:
    target_source = str(sample.get("follow_target_source") or "")
    if target_source not in {"CAMERA_TARGET", "CAMERA_SEARCH"}:
        return ""
    if not bool(sample.get("room_cruise_chain", False)):
        return ""
    if _direct_motor_bypass_sample(sample) or _legacy_generic_planner_sample(sample):
        return ""
    return HUMAN_FOLLOW_V2_ROUTE


def _longest_unhandled_target_lost_run(samples: List[Dict[str, Any]]) -> int:
    acquired_once = False
    post_acquire: List[Dict[str, Any]] = []
    for sample in list(samples or []):
        source = str(sample.get("follow_target_source") or "")
        if source == "CAMERA_TARGET" or _camera_detection_active(sample):
            acquired_once = True
        if bool(acquired_once):
            post_acquire.append(sample)
    return _longest_run(
        post_acquire,
        lambda s: str(s.get("follow_target_source") or "") not in {"CAMERA_TARGET", "CAMERA_SEARCH"}
        and not _camera_detection_active(s),
    )


def _turn_side_from_omega(omega_rad_s: Any, *, eps: float = 0.035) -> str:
    omega = _safe_float(omega_rad_s, 0.0)
    if omega > float(eps):
        return "left"
    if omega < -float(eps):
        return "right"
    return "center"


def _camera_image_side_from_angle(angle_deg: Any, *, eps: float = 2.0) -> str:
    angle = _safe_float(angle_deg, 0.0)
    if angle > float(eps):
        return "right"
    if angle < -float(eps):
        return "left"
    return "center"


def _camera_one_track_motion_side(sample: Dict[str, Any], *, eps: float = 0.004) -> str:
    left = _safe_float(sample.get("expected_track_left_mps"), sample.get("track_left_mps", 0.0))
    right = _safe_float(sample.get("expected_track_right_mps"), sample.get("track_right_mps", 0.0))
    left_forward = bool(float(left) >= float(eps) and abs(float(right)) < float(eps))
    right_forward = bool(float(right) >= float(eps) and abs(float(left)) < float(eps))
    if left_forward == right_forward:
        return "center" if abs(float(left)) < float(eps) and abs(float(right)) < float(eps) else "mixed"
    # Diff-drive yaw is (right-left)/track_width: the opposite-side wheel
    # advances the robot toward the target side.
    return "right" if left_forward else "left"


def _camera_turn_wrong_side_sample(sample: Dict[str, Any]) -> bool:
    if str(sample.get("follow_target_source") or "") != "CAMERA_TARGET":
        return False
    image_side = _camera_image_side_from_angle(sample.get("adaptive_target_angle_deg"))
    motion_side = _camera_one_track_motion_side(sample)
    return bool(image_side in {"left", "right"} and motion_side in {"left", "right"} and image_side != motion_side)


def _front_lidar_human_confirmed(sample: Dict[str, Any]) -> bool:
    detector = str(sample.get("target_camera_detector") or "")
    detector_confidence = _safe_float(sample.get("target_camera_detector_confidence"), 0.0)
    return bool(
        _camera_detection_active(sample)
        and (
            str(sample.get("target_camera_distance_source") or "")
            in {
                "front_lidar_close_bubble_camera_confirmed",
                "front_lidar_room_bubble_camera_confirmed",
            }
            or (
                str(sample.get("target_camera_state") or "") == "front_lidar_hold"
                and str(sample.get("target_camera_gate") or "") == "front_lidar_follow_hold"
                and (
                    detector in {"mediapipe_pose", "onnx_yolov5_person", "opencv_hog"}
                    or (detector == "opencv_template_lock" and float(detector_confidence) >= 0.55)
                    or (detector == "opencv_motion_blob" and float(detector_confidence) >= 0.65)
                )
            )
        )
    )


def _wall_stick_sample(sample: Dict[str, Any]) -> bool:
    close = _safe_float(sample.get("target_front_obstacle_distance_m"), 0.0)
    if close <= 0.0:
        close = _safe_float(sample.get("room_cruise_front_clearance_m"), 0.0)
    if close <= 0.0:
        close = _safe_float(sample.get("lidar_min_narrow_m"), 0.0)
    global_close = _safe_float(sample.get("room_cruise_global_min_clearance_m"), 0.0)
    if global_close <= 0.0:
        global_close = _safe_float(sample.get("lidar_min_dist_m"), 0.0)
    left_close = _safe_float(sample.get("room_cruise_left_clearance_m"), 0.0)
    right_close = _safe_float(sample.get("room_cruise_right_clearance_m"), 0.0)
    side_close = bool(
        (left_close > 0.0 and left_close <= WALL_STICK_CLOSE_M)
        or (right_close > 0.0 and right_close <= WALL_STICK_CLOSE_M)
    )
    if close <= 0.0:
        has_sector_clearance = bool(left_close > 0.0 or right_close > 0.0)
        if side_close or (global_close > 0.0 and not has_sector_clearance):
            close = float(global_close)
    if close <= 0.0 or close > WALL_STICK_CLOSE_M:
        return False
    if _front_lidar_human_confirmed(sample):
        return False
    if bool(sample.get("room_cruise_camera_detection_clearance_retreat_active", False)):
        return False
    if bool(sample.get("room_cruise_camera_target_warning_retreat_active", False)):
        return False
    if bool(sample.get("room_cruise_front_hold_retreat_active", False)):
        return False
    return True


def _front_clearance_m(status: Dict[str, Any]) -> float | None:
    lidar_payload = dict((status or {}).get("lidar") or {})
    lidar_summary = dict(lidar_payload.get("summary") or lidar_payload)
    for key in ("min_dist_narrow", "front_clearance_m", "front_clearance", "min_dist"):
        value = _safe_optional_float(lidar_summary.get(key))
        if value is not None and value > 0.0:
            return float(value)
    return None


def _motion_resolution(status: Dict[str, Any]) -> Dict[str, Any]:
    resolved = dict((status.get("motion_resolution") or {}).get("resolved") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    details = dict(resolved.get("details") or {})
    follow_request = dict(details.get("follow_request") or {})
    cruise_layer = dict(details.get("cruise_layer") or {})
    room_cruise = dict(details.get("room_cruise") or {})
    local_navigation = dict(details.get("local_navigation") or {})
    follow_gate = dict(room_cruise.get("follow_gate") or {})
    clearance = dict(room_cruise.get("clearance") or {})
    target_geometry = dict(room_cruise.get("target_geometry") or {})
    room_track_reference = dict(room_cruise.get("track_reference") or {})
    motion_command = dict(status.get("motion_command") or {})
    limited_intent = dict(motion_command.get("limited_motion_intent") or {})
    track_targets = dict(motion_command.get("track_targets") or {})
    expected_v = _safe_optional_float(final_after_shaping.get("v_target"))
    if expected_v is None:
        expected_v = _safe_optional_float(room_cruise.get("v_target"))
    if expected_v is None:
        expected_v = 0.0
    expected_omega = _safe_optional_float(final_after_shaping.get("omega_target"))
    if expected_omega is None:
        expected_omega = _safe_optional_float(room_cruise.get("omega_target"))
    if expected_omega is None:
        expected_omega = 0.0
    expected_track_left = _safe_optional_float(track_targets.get("left_mps"))
    expected_track_right = _safe_optional_float(track_targets.get("right_mps"))
    if expected_track_left is None:
        expected_track_left = _safe_optional_float(room_track_reference.get("left_mps"))
    if expected_track_right is None:
        expected_track_right = _safe_optional_float(room_track_reference.get("right_mps"))
    if expected_track_left is None:
        expected_track_left = 0.0
    if expected_track_right is None:
        expected_track_right = 0.0
    executed_v = _safe_float(limited_intent.get("v"), 0.0)
    executed_omega = _safe_float(limited_intent.get("omega"), 0.0)
    executed_track_left = _safe_float(track_targets.get("left_mps"), 0.0)
    executed_track_right = _safe_float(track_targets.get("right_mps"), 0.0)
    actual_distance = follow_request.get("distance_to_target_m")
    if actual_distance is None:
        actual_distance = follow_request.get("actual_distance_m")
    if actual_distance is None:
        actual_distance = follow_gate.get("actual_target_distance_m", target_geometry.get("distance_m"))
    desired_distance = follow_request.get("desired_distance_m")
    if desired_distance is None:
        desired_distance = follow_gate.get("desired_distance_m")
    actual_bearing = follow_request.get("actual_bearing_rad")
    if actual_bearing is None:
        actual_bearing = follow_gate.get("actual_target_bearing_error_rad", target_geometry.get("bearing_error_rad"))
    return {
        "resolved_name": str(resolved.get("name") or ""),
        "resolved_source": str(resolved.get("source") or ""),
        "resolved_layer": str(resolved.get("layer") or ""),
        "command_type": str(resolved.get("command_type") or ""),
        "entry_tier": str(resolved.get("entry_tier") or ""),
        "execution_mode": str(resolved.get("execution_mode") or ""),
        "follow_request_active": bool(follow_request.get("active", False)),
        "follow_target_source": str(follow_request.get("target_source") or cruise_layer.get("target_source") or ""),
        "follow_target_id": str(follow_request.get("target_id") or ""),
        "follow_front_obstacle_distance_m": _safe_float(follow_request.get("front_obstacle_distance_m"), 0.0),
        "follow_reason": str(follow_request.get("reason") or ""),
        "follow_actual_distance_m": _safe_float(actual_distance, 0.0),
        "follow_actual_bearing_rad": _safe_float(actual_bearing, 0.0),
        "follow_desired_distance_m": _safe_float(desired_distance, 1.0),
        "cruise_layer_primitive_type": str(cruise_layer.get("primitive_type") or ""),
        "cruise_layer_local_planner_bypassed": bool(cruise_layer.get("local_planner_bypassed", True)),
        "cruise_layer_local_navigation_suppressed_phase": bool(
            cruise_layer.get("local_navigation_suppressed_phase", False)
        ),
        "cruise_layer_local_navigation_active": bool(
            cruise_layer.get("local_navigation_active", False) or local_navigation.get("active", False)
        ),
        "local_navigation_mode": str(local_navigation.get("mode") or ""),
        "local_navigation_reason": str(local_navigation.get("reason") or ""),
        "local_navigation_feasible": bool(local_navigation.get("feasible", False)),
        "local_navigation_rear_clear_for_retreat": bool(local_navigation.get("rear_clear_for_retreat", False)),
        "local_navigation_global_clear_for_retreat": bool(local_navigation.get("global_clear_for_retreat", False)),
        "room_cruise_chain": bool(room_cruise.get("follow_above_cruise", False))
        or str(room_cruise.get("chain") or "") != "",
        "room_cruise_phase": str(room_cruise.get("phase") or ""),
        "room_cruise_reason": str(room_cruise.get("reason") or ""),
        "room_cruise_selected_side": str(room_cruise.get("selected_side") or ""),
        "room_cruise_side_selection": str(room_cruise.get("side_selection") or ""),
        "room_cruise_global_min_clearance_m": _safe_float(clearance.get("global_min_clearance_m"), 0.0),
        "room_cruise_front_clearance_m": _safe_float(clearance.get("front_clearance_m"), 0.0),
        "room_cruise_left_clearance_m": _safe_float(clearance.get("left_clearance_m"), 0.0),
        "room_cruise_right_clearance_m": _safe_float(clearance.get("right_clearance_m"), 0.0),
        "room_cruise_rear_clearance_m": _safe_float(clearance.get("rear_clearance_m"), 0.0),
        "room_cruise_rear_clear_for_retreat": bool(follow_gate.get("rear_clear_for_retreat", False)),
        "room_cruise_camera_target_guard_front_m": _safe_float(
            follow_gate.get("camera_target_guard_front_m"), 0.0
        ),
        "room_cruise_camera_target_guard_front_source": str(
            follow_gate.get("camera_target_guard_front_source") or ""
        ),
        "room_cruise_global_min_directionally_relevant": bool(
            follow_gate.get("global_min_directionally_relevant", True)
        ),
        "room_cruise_global_clearance_hard_gate": bool(follow_gate.get("global_clearance_hard_gate", False)),
        "room_cruise_global_clearance_warning_gate": bool(
            follow_gate.get("global_clearance_warning_gate", False)
        ),
        "room_cruise_global_front_clear_for_retreat": bool(
            follow_gate.get("global_front_clear_for_retreat", False)
        ),
        "room_cruise_front_hold_retreat_active": bool(follow_gate.get("front_hold_retreat_active", False)),
        "room_cruise_camera_target_warning_retreat_active": bool(
            follow_gate.get("camera_target_warning_retreat_active", False)
        ),
        "room_cruise_camera_front_obstacle_arbitrated": bool(
            follow_gate.get("camera_front_obstacle_arbitrated", False)
        ),
        "room_cruise_camera_motion_requires_detection": bool(
            follow_gate.get("camera_motion_requires_detection", False)
        ),
        "room_cruise_camera_motion_detection_allowed": bool(
            follow_gate.get("camera_motion_detection_allowed", False)
        ),
        "room_cruise_camera_detection_motion_suppressed": bool(
            follow_gate.get("camera_detection_motion_suppressed", False)
        ),
        "room_cruise_camera_detection_reacquire_rotate_allowed": bool(
            follow_gate.get("camera_detection_reacquire_rotate_allowed", False)
        ),
        "room_cruise_camera_detection_clearance_retreat_active": bool(
            follow_gate.get("camera_detection_clearance_retreat_active", False)
        ),
        "room_cruise_camera_simple_follow_active": bool(
            follow_gate.get("camera_simple_follow_active", False)
        ),
        "room_cruise_camera_simple_forward_gate_blocked": bool(
            follow_gate.get("camera_simple_forward_gate_blocked", False)
        ),
        "room_cruise_camera_simple_retreat_gate_blocked": bool(
            follow_gate.get("camera_simple_retreat_gate_blocked", False)
        ),
        "room_cruise_camera_simple_distance_error_m": _safe_float(
            follow_gate.get("camera_simple_distance_error_m"), 0.0
        ),
        "room_cruise_camera_simple_forward_track_mps": _safe_float(
            follow_gate.get("camera_simple_forward_track_mps"), 0.0
        ),
        "room_cruise_camera_simple_turn_track_mps": _safe_float(
            follow_gate.get("camera_simple_turn_track_mps"), 0.0
        ),
        "room_cruise_camera_simple_turn_cap_mps": _safe_float(
            follow_gate.get("camera_simple_turn_cap_mps"), 0.0
        ),
        "room_cruise_camera_simple_forward_cap_mps": _safe_float(
            follow_gate.get("camera_simple_forward_cap_mps"), 0.0
        ),
        "room_cruise_camera_simple_heading_forward_scale": _safe_float(
            follow_gate.get("camera_simple_heading_forward_scale"), 1.0
        ),
        "room_cruise_camera_target_turn_side": str(follow_gate.get("camera_target_turn_side") or ""),
        "room_cruise_camera_target_turn_side_clearance_m": _safe_float(
            follow_gate.get("camera_target_turn_side_clearance_m"), 0.0
        ),
        "room_cruise_camera_target_turn_side_blocked": bool(
            follow_gate.get("camera_target_turn_side_blocked", False)
        ),
        "room_cruise_v_target": _safe_float(room_cruise.get("v_target"), 0.0),
        "room_cruise_omega_target": _safe_float(room_cruise.get("omega_target"), 0.0),
        "expected_v": float(expected_v),
        "expected_omega": float(expected_omega),
        "executed_v": float(executed_v),
        "executed_omega": float(executed_omega),
        "expected_turn_side": _turn_side_from_omega(expected_omega),
        "executed_turn_side": _turn_side_from_omega(executed_omega),
        "motion_expected_executed_v_error": abs(float(expected_v) - float(executed_v)),
        "motion_expected_executed_omega_error": abs(float(expected_omega) - float(executed_omega)),
        "expected_track_left_mps": float(expected_track_left),
        "expected_track_right_mps": float(expected_track_right),
        "executed_track_left_mps": float(executed_track_left),
        "executed_track_right_mps": float(executed_track_right),
        "motion_expected_executed_track_left_error_mps": abs(float(expected_track_left) - float(executed_track_left)),
        "motion_expected_executed_track_right_error_mps": abs(float(expected_track_right) - float(executed_track_right)),
        "motion_expected_executed_track_error_mps": max(
            abs(float(expected_track_left) - float(executed_track_left)),
            abs(float(expected_track_right) - float(executed_track_right)),
        ),
        "cmd_v": float(executed_v),
        "cmd_omega": float(executed_omega),
        "track_left_mps": float(executed_track_left),
        "track_right_mps": float(executed_track_right),
    }


def _sample_status(start_mono: float) -> Dict[str, Any]:
    status = _read_json(STATUS_PATH)
    motion = _motion_resolution(status)
    adaptive = dict(status.get("adaptive_motion") or {})
    camera_status = dict(adaptive.get("target_camera_status") or {})
    lidar_status = dict(adaptive.get("target_lidar_status") or {})
    search_status = dict(adaptive.get("target_search_status") or {})
    logger = dict(status.get("logger") or {})
    loop_budget = dict(status.get("loop_budget") or {})
    localization_gate = dict(status.get("localization_gate") or {})
    localization_gate_apply = dict(localization_gate.get("apply") or {})
    lidar_odom_status = dict(status.get("lidar_odom_status") or {})
    lidar_gap_watchdog = dict(lidar_odom_status.get("ekf_applied_gap_watchdog") or {})
    emergency = dict(status.get("last_emergency") or {})
    lidar_payload = dict(status.get("lidar") or {})
    lidar_summary = dict(lidar_payload.get("summary") or lidar_payload)
    pose = dict(status.get("pose") or {})
    motion_public = dict(status.get("motion_public") or {})
    motion_command_status = dict(status.get("motion_command") or {})
    requested_motion = dict(motion_command_status.get("requested_motion_intent") or {})
    limited_motion = dict(motion_command_status.get("limited_motion_intent") or {})
    requested_tracks = dict(motion_command_status.get("requested_track_reference") or {})
    target_tracks = dict(motion_command_status.get("track_targets") or {})
    primitive_contract = dict(motion_command_status.get("primitive_contract") or {})
    actuator_service = dict(motion_command_status.get("actuator_service") or {})
    global_motion_policy = dict(status.get("global_motion_policy") or {})
    global_motion_runtime = dict(global_motion_policy.get("runtime_stats") or {})
    tuning = dict(status.get("tuning") or {})
    follow_tuning = dict(tuning.get("follow") or {})
    pwm = dict(status.get("pwm") or {})
    control_monitor = dict(status.get("control_monitor") or {})
    watchdog = dict(status.get("watchdog") or {})
    loop_slices = dict(loop_budget.get("slices") or {})
    control_loop_slice = dict(loop_slices.get("control_loop_tick") or {})
    write_status_slice = dict(loop_slices.get("write_status") or {})
    encoder_canonical = dict((status.get("encoder") or {}).get("canonical") or {})
    encoder_velocity = dict(encoder_canonical.get("canonical_velocity") or {})
    encoder_pulses = dict(encoder_canonical.get("pulses_delta") or {})
    imu = dict(status.get("imu") or {})
    imu_gyro = list(imu.get("gyro") or [])
    lidar_pose = dict(lidar_odom_status.get("last_lidar_pose") or {})
    stop_status = dict(status.get("stop_status") or {})
    localization_truth = dict(status.get("localization_truth") or {})
    sample = {
        "ts": round(time.time(), 6),
        "elapsed_s": round(time.monotonic() - start_mono, 3),
        "status_version": int(status.get("status_version", 0) or 0),
        "state": str(status.get("state", "") or ""),
        "control_mode": str(status.get("control_mode") or ""),
        "runtime_preset": str(status.get("runtime_preset") or ""),
        "follow_tuning_speed_scale": _safe_float(follow_tuning.get("speed_scale"), 1.0),
        "follow_tuning_target_distance_m": _safe_float(follow_tuning.get("target_distance_m"), 0.0),
        "follow_search_pivot_omega_rad_s": _safe_float(
            follow_tuning.get("search_pivot_omega_rad_s"),
            0.0,
        ),
        "pose_theta_deg": _safe_float(pose.get("theta_deg"), 0.0),
        "ekf_x_m": _safe_float(pose.get("x"), 0.0),
        "ekf_y_m": _safe_float(pose.get("y"), 0.0),
        "ekf_omega_rad_s": _safe_float(pose.get("omega_rad_s"), 0.0),
        "actual_linear_mps": _safe_float(motion_public.get("actual_linear_mps"), 0.0),
        "actual_angular_dps": _safe_float(motion_public.get("actual_angular_dps"), 0.0),
        "actual_omega_rad_s": math.radians(_safe_float(motion_public.get("actual_angular_dps"), 0.0)),
        "cmd_linear_mps": _safe_float(motion_public.get("cmd_linear_mps"), 0.0),
        "cmd_angular_dps": _safe_float(motion_public.get("cmd_angular_dps"), 0.0),
        "requested_v_mps": _safe_float(requested_motion.get("v"), 0.0),
        "requested_omega_rad_s": _safe_float(requested_motion.get("omega"), 0.0),
        "limited_v_mps": _safe_float(limited_motion.get("v"), 0.0),
        "limited_omega_rad_s": _safe_float(limited_motion.get("omega"), 0.0),
        "requested_track_left_mps": _safe_float(requested_tracks.get("left_mps"), 0.0),
        "requested_track_right_mps": _safe_float(requested_tracks.get("right_mps"), 0.0),
        "target_track_left_mps": _safe_float(target_tracks.get("left_mps"), 0.0),
        "target_track_right_mps": _safe_float(target_tracks.get("right_mps"), 0.0),
        "actual_track_left_mps": _safe_float(encoder_velocity.get("left_mps"), 0.0),
        "actual_track_right_mps": _safe_float(encoder_velocity.get("right_mps"), 0.0),
        "actual_track_left_raw_mps": _safe_float(encoder_velocity.get("left_raw_mps"), 0.0),
        "actual_track_right_raw_mps": _safe_float(encoder_velocity.get("right_raw_mps"), 0.0),
        "encoder_yaw_rate_rad_s": _safe_float(encoder_velocity.get("yaw_rate_rad_s"), 0.0),
        "encoder_pulses_left": int(encoder_pulses.get("left_control_window", encoder_pulses.get("left", 0)) or 0),
        "encoder_pulses_right": int(encoder_pulses.get("right_control_window", encoder_pulses.get("right", 0)) or 0),
        "encoder_control_window_s": _safe_float(encoder_pulses.get("dt_control_window_s"), 0.0),
        "imu_heading_deg": _safe_float(imu.get("heading_deg"), math.nan),
        "imu_gyro_z_dps": _safe_float(imu_gyro[2] if len(imu_gyro) > 2 else None, math.nan),
        "imu_gyro_z_rad_s": math.radians(_safe_float(imu_gyro[2] if len(imu_gyro) > 2 else None, math.nan)),
        "lidar_heading_deg": math.degrees(_safe_float(lidar_pose.get("theta"), math.nan)),
        "lidar_odom_confidence": _safe_float(
            lidar_odom_status.get("latest_confidence", lidar_odom_status.get("confidence")),
            0.0,
        ),
        "lidar_odom_latest_age_s": _safe_float(lidar_odom_status.get("latest_age_s"), 0.0),
        "pwm_left": _safe_float(pwm.get("left"), 0.0),
        "pwm_right": _safe_float(pwm.get("right"), 0.0),
        "active_motion_layer": str(motion_command_status.get("active_layer") or ""),
        "active_motion_type": str(motion_command_status.get("command_type") or ""),
        "motion_execution_mode": str(motion_command_status.get("execution_mode") or ""),
        "turn_primitive_requested": str(motion_command_status.get("turn_primitive_requested") or ""),
        "turn_primitive_limited": str(motion_command_status.get("turn_primitive_limited") or ""),
        "turn_primitive_executed": str(motion_command_status.get("turn_primitive_executed") or ""),
        "turn_primitive_actual": str(motion_command_status.get("turn_primitive_actual") or ""),
        "primitive_mismatch_reason": str(motion_command_status.get("mismatch_reason") or ""),
        "primitive_contract_violation": bool(primitive_contract.get("violation", False)),
        "speed_limiting_reason": str(motion_command_status.get("speed_limiting_reason") or ""),
        "safety_limiting_reason": str(motion_command_status.get("safety_limiting_reason") or ""),
        "stop_type": str(stop_status.get("type") or ""),
        "stop_reason": str(stop_status.get("canonical_reason", stop_status.get("reason")) or ""),
        "control_output_reason": str(control_monitor.get("output_reason") or ""),
        "control_execution_mode": str(control_monitor.get("execution_mode") or ""),
        "control_execution_contract_violation": bool(
            control_monitor.get("execution_mode_contract_violation", False)
        ),
        "control_v_cmd_mps": _safe_float(control_monitor.get("v_cmd"), 0.0),
        "control_omega_request_rad_s": _safe_float(control_monitor.get("omega_cmd_request"), 0.0),
        "control_omega_cmd_rad_s": _safe_float(control_monitor.get("omega_cmd"), 0.0),
        "control_heading_correction_owner": str(control_monitor.get("heading_correction_owner") or ""),
        "straight_hold_active": bool(control_monitor.get("straight_hold_active", False)),
        "straight_hold_correction_rad_s": _safe_float(control_monitor.get("straight_hold_correction"), 0.0),
        "straight_hold_heading_error_deg": _safe_float(
            control_monitor.get("straight_hold_heading_error_deg"),
            0.0,
        ),
        "straight_hold_slew_limited": bool(control_monitor.get("straight_hold_slew_limited", False)),
        "wheel_loop_enabled": bool(control_monitor.get("wheel_loop_enabled", False)),
        "wheel_loop_left_ref_mps": _safe_float(control_monitor.get("wheel_loop_left_ref_mps"), 0.0),
        "wheel_loop_right_ref_mps": _safe_float(control_monitor.get("wheel_loop_right_ref_mps"), 0.0),
        "wheel_loop_left_meas_mps": _safe_float(control_monitor.get("wheel_loop_left_meas_mps"), 0.0),
        "wheel_loop_right_meas_mps": _safe_float(control_monitor.get("wheel_loop_right_meas_mps"), 0.0),
        "wheel_loop_left_error_mps": _safe_float(control_monitor.get("wheel_loop_left_error_mps"), 0.0),
        "wheel_loop_right_error_mps": _safe_float(control_monitor.get("wheel_loop_right_error_mps"), 0.0),
        "wheel_loop_left_output_reason": str(control_monitor.get("wheel_loop_left_output_reason") or ""),
        "wheel_loop_right_output_reason": str(control_monitor.get("wheel_loop_right_output_reason") or ""),
        "pwm_raw_left": _safe_float(control_monitor.get("pwm_raw_l"), 0.0),
        "pwm_raw_right": _safe_float(control_monitor.get("pwm_raw_r"), 0.0),
        "watchdog_period_s": _safe_float(watchdog.get("period_sec"), 0.0),
        "watchdog_freq_hz": _safe_float(watchdog.get("freq_hz"), 0.0),
        "watchdog_warn_count": int(watchdog.get("warn_count", 0) or 0),
        "watchdog_stop_triggered": bool(watchdog.get("stop_triggered", False)),
        "loop_total_ema_ms": _safe_float(loop_budget.get("total_ema_ms"), 0.0),
        "control_loop_tick_last_ms": _safe_float(control_loop_slice.get("last_ms"), 0.0),
        "control_loop_tick_ema_ms": _safe_float(control_loop_slice.get("ema_ms"), 0.0),
        "control_loop_tick_max_ms": _safe_float(control_loop_slice.get("max_ms"), 0.0),
        "write_status_last_ms": _safe_float(write_status_slice.get("last_ms"), 0.0),
        "write_status_ema_ms": _safe_float(write_status_slice.get("ema_ms"), 0.0),
        "write_status_max_ms": _safe_float(write_status_slice.get("max_ms"), 0.0),
        "service_motion_active": bool(status.get("service_motion_active", False)),
        "actuator_service_active": bool(actuator_service.get("active", False)),
        "global_motion_failsafe_events": int(global_motion_runtime.get("failsafe_events", 0) or 0),
        "global_motion_degeneracy_events": int(global_motion_runtime.get("degeneracy_events", 0) or 0),
        "camera_enabled": bool(status.get("camera_enabled", False)),
        "safety_allow": bool(status.get("safety_allow", True)),
        "last_emergency_reason": str(emergency.get("reason") or ""),
        "last_emergency_count": _emergency_count(status),
        "logger_queue_depth": int(logger.get("queue_depth", 0) or 0),
        "logger_dropped_messages": int(logger.get("dropped_messages", 0) or 0),
        "loop_watchdog_period_max_ms": _safe_float(
            ((loop_budget.get("watchdog_period") or {}).get("max_ms")),
            0.0,
        ),
        "localization_gate_mode": str(localization_gate.get("mode") or ""),
        "localization_gate_trust": _safe_float(localization_gate.get("trust"), 0.0),
        "localization_gate_allow_motion": bool(localization_gate.get("allow_motion", True)),
        "localization_gate_speed_scale": _safe_float(localization_gate.get("speed_scale"), 1.0),
        "localization_gate_hard_stop": bool(localization_gate.get("hard_stop", False)),
        "localization_gate_reasons": list(localization_gate.get("reasons") or []),
        "localization_gate_apply_reason": str(localization_gate_apply.get("reason") or ""),
        "localization_gate_ekf_applied_gap_s": _safe_float(
            localization_gate.get("ekf_applied_gap_s"),
            0.0,
        ),
        "localization_gate_current_ekf_applied": bool(localization_gate.get("current_ekf_applied", False)),
        "localization_gate_idle_stationary_guard_active": bool(
            localization_gate.get("idle_stationary_guard_active", False)
        ),
        "localization_gate_idle_stationary_gap_recoverable": bool(
            localization_gate.get("idle_stationary_gap_recoverable", False)
        ),
        "localization_gate_idle_resume_bridge_active": bool(
            localization_gate.get("idle_stationary_resume_bridge_active", False)
        ),
        "localization_gate_idle_resume_bridge_recoverable": bool(
            localization_gate.get("idle_stationary_resume_bridge_recoverable", False)
        ),
        "localization_gate_idle_resume_bridge_remaining_s": _safe_float(
            localization_gate.get("idle_stationary_resume_bridge_remaining_s"),
            0.0,
        ),
        "localization_gate_raw_health": str(localization_gate.get("raw_localization_health") or ""),
        "localization_gate_root_cause": str(localization_gate.get("root_cause") or ""),
        "localization_gate_delivery_status": str(localization_gate.get("delivery_status") or ""),
        "localization_truth_state": str(localization_truth.get("state") or ""),
        "localization_truth_trust": _safe_float(localization_truth.get("trust"), 0.0),
        "localization_truth_allow_motion": bool(localization_truth.get("allow_motion", False)),
        "localization_truth_consistent": bool(localization_truth.get("consistent", True)),
        "lidar_odom_status": str(lidar_odom_status.get("status") or ""),
        "lidar_odom_ekf_status": str(lidar_odom_status.get("ekf_status") or ""),
        "lidar_odom_apply_status": str(lidar_odom_status.get("control_loop_lidar_apply_status") or ""),
        "lidar_odom_raw_scan_latest_age_s": _safe_float(lidar_odom_status.get("raw_scan_latest_age_s"), 0.0),
        "lidar_odom_raw_scan_rate_hz": _safe_float(lidar_odom_status.get("raw_scan_rate_hz"), 0.0),
        "lidar_odom_matcher_latency_ms": _safe_float(lidar_odom_status.get("matcher_latency_ms"), 0.0),
        "lidar_odom_matcher_latency_p95_ms": _safe_float(lidar_odom_status.get("matcher_latency_p95_ms"), 0.0),
        "lidar_odom_matcher_queue_depth": int(lidar_odom_status.get("matcher_queue_depth", 0) or 0),
        "lidar_odom_ekf_gap_watchdog_state": str(lidar_gap_watchdog.get("state") or ""),
        "lidar_odom_ekf_gap_watchdog_motion_active": bool(lidar_gap_watchdog.get("motion_active", False)),
        "target_camera_state": str(camera_status.get("state") or ""),
        "target_camera_frame_ok": bool(camera_status.get("frame_ok", False)),
        "target_camera_visible": bool(camera_status.get("target_visible", False)),
        "target_camera_usable": bool(camera_status.get("target_usable", False)),
        "target_camera_stale": bool(camera_status.get("stale", False)),
        "target_camera_age_s": _safe_float(camera_status.get("age_s"), 0.0),
        "target_camera_rotation_deg": int(camera_status.get("rotation_deg", 0) or 0),
        "target_camera_open_failed": bool(camera_status.get("open_failed", False)),
        "target_camera_failed_sessions": int(camera_status.get("failed_sessions", 0) or 0),
        "target_camera_image_width_px": int(camera_status.get("image_width_px", 0) or 0),
        "target_camera_image_height_px": int(camera_status.get("image_height_px", 0) or 0),
        "target_camera_frame_luma_mean": _safe_float(camera_status.get("frame_luma_mean"), 0.0),
        "target_camera_frame_luma_std": _safe_float(camera_status.get("frame_luma_std"), 0.0),
        "target_camera_frame_too_dark": bool(camera_status.get("frame_too_dark", False)),
        "target_camera_frame_too_bright": bool(camera_status.get("frame_too_bright", False)),
        "target_camera_frame_low_contrast": bool(camera_status.get("frame_low_contrast", False)),
        "target_camera_detector": str(camera_status.get("detector") or ""),
        "target_camera_detector_confidence": _safe_float(camera_status.get("detector_confidence"), 0.0),
        "target_camera_detector_error": str(camera_status.get("detector_error") or ""),
        "target_camera_detector_latency_ms": _safe_float(camera_status.get("detector_latency_ms"), 0.0),
        "target_camera_detector_throttled": bool(camera_status.get("detector_throttled", False)),
        "target_camera_capture_pending": bool(camera_status.get("capture_pending", False)),
        "target_camera_capture_status": str(camera_status.get("capture_status") or ""),
        "target_camera_async_worker_active": bool(camera_status.get("async_worker_active", False)),
        "target_camera_async_inference_running": bool(camera_status.get("async_inference_running", False)),
        "target_camera_async_result_age_s": _safe_float(camera_status.get("async_result_age_s"), 0.0),
        "target_camera_async_update_seq": int(camera_status.get("async_update_seq", 0) or 0),
        "target_camera_async_stale_gate": bool(camera_status.get("async_stale_gate", False)),
        "target_camera_stream_seed": bool(camera_status.get("stream_seed", False)),
        "target_camera_stream_seed_age_s": _safe_float(camera_status.get("stream_seed_age_s"), 0.0),
        "target_camera_target_zone": str(camera_status.get("target_zone") or ""),
        "target_camera_lock_state": str(camera_status.get("lock_state") or ""),
        "target_camera_lock_confirmed": bool(camera_status.get("lock_confirmed", False)),
        "target_camera_lock_confirm_count": int(camera_status.get("lock_confirm_count", 0) or 0),
        "target_camera_lock_required_frames": int(camera_status.get("lock_required_frames", 0) or 0),
        "target_camera_lock_reason": str(camera_status.get("lock_reason") or ""),
        "target_camera_lock_id": int(camera_status.get("lock_id", 0) or 0),
        "target_camera_lock_startup_onnx_single_frame_path": bool(
            camera_status.get("lock_startup_onnx_single_frame_path", False)
        ),
        "target_camera_lock_recent_onnx_single_frame_relock_path": bool(
            camera_status.get("lock_recent_onnx_single_frame_relock_path", False)
        ),
        "target_camera_lost_count": int(camera_status.get("lost_count", 0) or 0),
        "target_camera_relock_count": int(camera_status.get("relock_count", 0) or 0),
        "target_camera_last_lock_image_path": str(
            camera_status.get("last_lock_image_path") or camera_status.get("lock_image_path") or ""
        ),
        "target_camera_search_side": str(camera_status.get("search_side") or camera_status.get("last_search_side") or ""),
        "target_camera_onnx_best_score": _safe_float(camera_status.get("onnx_best_score"), 0.0),
        "target_camera_onnx_best_objectness": _safe_float(camera_status.get("onnx_best_objectness"), 0.0),
        "target_camera_onnx_best_person_class_score": _safe_float(
            camera_status.get("onnx_best_person_class_score"),
            0.0,
        ),
        "target_camera_onnx_objectness": _safe_float(camera_status.get("onnx_objectness"), 0.0),
        "target_camera_onnx_person_class_score": _safe_float(camera_status.get("onnx_person_class_score"), 0.0),
        "target_camera_onnx_weak_person_candidate": bool(camera_status.get("onnx_weak_person_candidate", False)),
        "target_camera_onnx_relock_person_candidate": bool(
            camera_status.get("onnx_relock_person_candidate", False)
        ),
        "target_camera_onnx_best_reject_reason": str(camera_status.get("onnx_best_reject_reason") or ""),
        "target_camera_onnx_candidate_count": int(camera_status.get("onnx_candidate_count", 0) or 0),
        "target_camera_onnx_score_reject_count": int(camera_status.get("onnx_score_reject_count", 0) or 0),
        "target_camera_onnx_shape_reject_count": int(camera_status.get("onnx_shape_reject_count", 0) or 0),
        "target_camera_gate": str(camera_status.get("gate") or ""),
        "target_camera_raw_state": str(camera_status.get("raw_state") or ""),
        "target_camera_bbox_width_ratio": _safe_float(camera_status.get("bbox_width_ratio"), 0.0),
        "target_camera_bbox_height_ratio": _safe_float(camera_status.get("bbox_height_ratio"), 0.0),
        "target_camera_bbox_aspect_ratio": _safe_float(camera_status.get("bbox_aspect_ratio"), 0.0),
        "target_camera_bbox_reject_reason": str(camera_status.get("bbox_reject_reason") or ""),
        "target_camera_onnx_best_box_width_ratio": _safe_float(camera_status.get("onnx_best_box_width_ratio"), 0.0),
        "target_camera_onnx_best_box_height_ratio": _safe_float(camera_status.get("onnx_best_box_height_ratio"), 0.0),
        "target_camera_distance_estimate_m": _safe_float(camera_status.get("distance_estimate_m"), 0.0),
        "target_camera_distance_used_m": _safe_float(camera_status.get("distance_used_m"), 0.0),
        "target_camera_distance_confidence": _safe_float(camera_status.get("distance_confidence"), 0.0),
        "target_camera_distance_source": str(camera_status.get("distance_source") or ""),
        "target_camera_lidar_delta_m": _safe_float(camera_status.get("camera_lidar_delta_m"), 0.0),
        "target_front_obstacle_distance_m": _safe_float(camera_status.get("front_obstacle_distance_m"), 0.0),
        "target_front_hold_distance_m": _safe_float(camera_status.get("front_hold_distance_m"), 0.0),
        "adaptive_follow_state": str(adaptive.get("follow_state") or ""),
        "adaptive_target_dist_m": _safe_float(adaptive.get("target_dist_m"), 0.0),
        "adaptive_target_angle_deg": _safe_float(adaptive.get("target_angle_deg"), 0.0),
        "target_lidar_state": str(lidar_status.get("state") or ""),
        "target_lidar_usable_distance": bool(lidar_status.get("usable_distance", False)),
        "target_lidar_source": str(lidar_status.get("source") or ""),
        "target_search_state": str(search_status.get("state") or ""),
        "target_search_active": bool(search_status.get("active", False)),
        "target_search_rotations_completed": int(search_status.get("rotations_completed", 0) or 0),
        "lidar_min_dist_m": _safe_float(lidar_summary.get("min_dist"), 0.0),
        "lidar_min_narrow_m": _safe_float(lidar_summary.get("min_dist_narrow"), 0.0),
        **motion,
    }
    image_side = _camera_image_side_from_angle(sample.get("adaptive_target_angle_deg"))
    motion_side = _camera_one_track_motion_side(sample)
    sample["camera_target_image_side"] = str(image_side)
    sample["camera_target_motion_side"] = str(motion_side)
    sample["camera_turn_alignment_ok"] = bool(
        image_side == "center"
        or motion_side == "center"
        or image_side == str(motion_side)
    )
    sample["camera_turn_wrong_side"] = bool(_camera_turn_wrong_side_sample(sample))
    sample["active_route"] = _active_route_for_sample(sample)
    sample["direct_motor_bypass"] = bool(_direct_motor_bypass_sample(sample))
    sample["used_legacy_generic_planner"] = bool(_legacy_generic_planner_sample(sample))
    return sample


def _wait_status_progress(timeout_s: float = 5.0) -> bool:
    first = int(_read_json(STATUS_PATH).get("status_version", 0) or 0)
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        if int(_read_json(STATUS_PATH).get("status_version", 0) or 0) > first:
            return True
        time.sleep(0.1)
    return False


def run(args: argparse.Namespace) -> Dict[str, Any]:
    token = str(args.token)
    command_results: List[Dict[str, Any]] = []
    samples: List[Dict[str, Any]] = []
    errors: List[str] = []
    speed_scale = _safe_float(getattr(args, "speed_scale", 1.0), 1.0)
    follow_distance_m = _safe_float(getattr(args, "follow_distance_m", 1.0), 1.0)
    search_pivot_omega_rad_s = _safe_optional_float(getattr(args, "search_pivot_omega_rad_s", None))
    control_mode_result: Dict[str, Any] = {}
    motion_quality_tuning = {
        "fresh_target_omega_max_rad_s": _safe_float(
            getattr(args, "fresh_target_omega_max_rad_s", None),
            HUMAN_FOLLOW_V2_FRESH_TARGET_OMEGA_MAX_RAD_S,
        ),
        "omega_p90_max_rad_s": _safe_float(
            getattr(args, "omega_p90_max_rad_s", None),
            HUMAN_FOLLOW_V2_OMEGA_P90_MAX_RAD_S,
        ),
        "command_delta_omega_p90_max_rad_s": _safe_float(
            getattr(args, "command_delta_omega_p90_max_rad_s", None),
            FOLLOW_COMMAND_DELTA_OMEGA_P90_MAX_RAD_S,
        ),
        "command_delta_omega_max_rad_s": _safe_float(
            getattr(args, "command_delta_omega_max_rad_s", None),
            FOLLOW_COMMAND_DELTA_OMEGA_MAX_RAD_S,
        ),
    }
    for key, value in list(motion_quality_tuning.items()):
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            errors.append(f"invalid_{key}")
            if key == "fresh_target_omega_max_rad_s":
                motion_quality_tuning[key] = HUMAN_FOLLOW_V2_FRESH_TARGET_OMEGA_MAX_RAD_S
            elif key == "omega_p90_max_rad_s":
                motion_quality_tuning[key] = HUMAN_FOLLOW_V2_OMEGA_P90_MAX_RAD_S
            elif key == "command_delta_omega_p90_max_rad_s":
                motion_quality_tuning[key] = FOLLOW_COMMAND_DELTA_OMEGA_P90_MAX_RAD_S
            elif key == "command_delta_omega_max_rad_s":
                motion_quality_tuning[key] = FOLLOW_COMMAND_DELTA_OMEGA_MAX_RAD_S
    if not math.isfinite(follow_distance_m) or follow_distance_m < 0.30 or follow_distance_m > 2.50:
        errors.append("invalid_follow_distance")
        follow_distance_m = 1.0
    follow_acceptance = _follow_distance_acceptance(follow_distance_m)
    pre_follow_front_clearance_m: float | None = None
    if not math.isfinite(speed_scale) or speed_scale <= 0.0 or speed_scale > 1.0:
        errors.append("invalid_speed_scale")
        speed_scale = 1.0
    initial_status = _read_json(STATUS_PATH)
    initial_camera_enabled = bool(initial_status.get("camera_enabled", False))
    initial_emergency_count = _emergency_count(initial_status)
    initial_failsafe_count = _global_motion_failsafe_count(initial_status)
    initial_tuning = dict(initial_status.get("tuning") or {})
    initial_follow_tuning = dict(initial_tuning.get("follow") or {})
    initial_search_pivot_omega_rad_s = _safe_optional_float(
        initial_follow_tuning.get("search_pivot_omega_rad_s")
    )
    if initial_search_pivot_omega_rad_s is None:
        initial_search_pivot_omega_rad_s = 0.08
    if search_pivot_omega_rad_s is not None and (
        not math.isfinite(float(search_pivot_omega_rad_s))
        or float(search_pivot_omega_rad_s) < FOLLOW_SEARCH_PIVOT_OMEGA_MIN_RAD_S
        or float(search_pivot_omega_rad_s) > FOLLOW_SEARCH_PIVOT_OMEGA_MAX_RAD_S
    ):
        errors.append("invalid_search_pivot_omega")
        search_pivot_omega_rad_s = None

    if not _wait_status_progress(timeout_s=float(args.status_timeout_s)):
        errors.append("status_not_progressing")
    control_mode_result = _ensure_control_mode(
        getattr(args, "control_mode", CANONICAL_CONTROL_MODE),
        token=token,
        command_results=command_results,
        timeout_s=max(5.0, float(args.status_timeout_s)),
    )
    if not bool(control_mode_result.get("ok", False)):
        errors.append(str(control_mode_result.get("error") or "control_mode_not_unified"))

    try:
        if not bool(_read_json(STATUS_PATH).get("camera_enabled", False)):
            command_results.append(_send_command("toggle_camera", token=token, timeout_s=5.0))
            time.sleep(0.5)

        distance_result = _send_command(
            "set_follow_distance",
            token=token,
            timeout_s=5.0,
            distance_m=follow_distance_m,
        )
        command_results.append(distance_result)
        if not bool(distance_result.get("effective", False)):
            errors.append("follow_distance_command_not_effective")
        if speed_scale < 1.0:
            scale_result = _send_command("set_follow_speed_scale", token=token, timeout_s=5.0, scale=speed_scale)
            command_results.append(scale_result)
            if not bool(scale_result.get("effective", False)):
                errors.append("speed_scale_command_not_effective")
        if search_pivot_omega_rad_s is not None:
            pivot_result = _send_command(
                "set_follow_search_pivot_omega",
                token=token,
                timeout_s=5.0,
                omega_rad_s=float(search_pivot_omega_rad_s),
            )
            command_results.append(pivot_result)
            if not bool(pivot_result.get("effective", False)):
                errors.append("search_pivot_omega_command_not_effective")
        pre_follow_front_clearance_m = _front_clearance_m(_read_json(STATUS_PATH))
        if (
            pre_follow_front_clearance_m is not None
            and float(pre_follow_front_clearance_m) < FOLLOW_PREFOLLOW_MIN_FRONT_CLEARANCE_M
        ):
            errors.append("pre_follow_clearance_too_close")
        if (
            "follow_distance_command_not_effective" not in errors
            and "invalid_follow_distance" not in errors
            and "speed_scale_command_not_effective" not in errors
            and "invalid_speed_scale" not in errors
            and "search_pivot_omega_command_not_effective" not in errors
            and "invalid_search_pivot_omega" not in errors
            and "control_mode_not_applied" not in errors
            and "invalid_control_mode" not in errors
            and "pre_follow_clearance_too_close" not in errors
        ):
            command_results.append(_send_command("toggle_follow", token=token, timeout_s=5.0))
            start_mono = time.monotonic()
            deadline = start_mono + max(1.0, float(args.duration_s))
            sample_period_s = 1.0 / max(1.0, float(args.sample_rate_hz))
            next_sample = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_sample:
                    samples.append(_sample_status(start_mono))
                    next_sample = now + sample_period_s
                time.sleep(0.03)
    except KeyboardInterrupt:
        errors.append("interrupted")
    finally:
        final_status = _read_json(STATUS_PATH)
        if str(final_status.get("state", "") or "").upper() == "FOLLOW":
            command_results.append(_send_command("toggle_follow", token=token, timeout_s=5.0))
        command_results.append(
            _send_command("set_track_velocity", token=token, timeout_s=4.0, left_mps=0.0, right_mps=0.0)
        )
        if speed_scale < 1.0:
            command_results.append(_send_command("set_follow_speed_scale", token=token, timeout_s=5.0, scale=1.0))
        if abs(float(follow_distance_m) - 1.0) > 1e-6:
            command_results.append(_send_command("set_follow_distance", token=token, timeout_s=5.0, distance_m=1.0))
        if (
            search_pivot_omega_rad_s is not None
            and initial_search_pivot_omega_rad_s is not None
            and abs(float(search_pivot_omega_rad_s) - float(initial_search_pivot_omega_rad_s)) > 1e-6
        ):
            command_results.append(
                _send_command(
                    "set_follow_search_pivot_omega",
                    token=token,
                    timeout_s=5.0,
                    omega_rad_s=float(initial_search_pivot_omega_rad_s),
                )
            )
        if (not initial_camera_enabled) and bool(_read_json(STATUS_PATH).get("camera_enabled", False)):
            command_results.append(_send_command("toggle_camera", token=token, timeout_s=5.0))

    if not all(bool(c.get("effective", False)) for c in command_results):
        errors.append("command_not_effective")
    if not any(bool(s.get("camera_enabled", False)) for s in samples):
        errors.append("camera_not_enabled")
    if not any(str(s.get("state", "")).upper() == "FOLLOW" for s in samples):
        errors.append("follow_state_not_observed")
    if not any(bool(s.get("follow_request_active", False)) for s in samples):
        errors.append("follow_request_not_active")
    if not any(str(s.get("follow_target_source", "")) == "CAMERA_TARGET" for s in samples):
        errors.append("camera_target_not_observed")
    if not any(_safe_float(s.get("target_camera_distance_estimate_m"), 0.0) > 0.0 for s in samples):
        errors.append("camera_distance_estimate_not_observed")
    if not any(bool(s.get("room_cruise_chain", False)) for s in samples):
        errors.append("room_cruise_chain_not_observed")
    if _new_emergency_observed(samples, initial_emergency_count):
        errors.append("emergency_observed")
    if any(not bool(s.get("safety_allow", True)) for s in samples):
        errors.append("safety_not_allowing")

    target_samples = [s for s in samples if str(s.get("follow_target_source", "")) == "CAMERA_TARGET"]
    search_samples = [s for s in samples if str(s.get("follow_target_source", "")) == "CAMERA_SEARCH"]
    cruise_samples = [s for s in samples if bool(s.get("room_cruise_chain", False))]
    wall_stick_samples = [s for s in samples if _wall_stick_sample(s)]
    camera_open_failed_samples = [s for s in samples if bool(s.get("target_camera_open_failed", False))]
    camera_dark_samples = [s for s in samples if bool(s.get("target_camera_frame_too_dark", False))]
    camera_bright_samples = [s for s in samples if bool(s.get("target_camera_frame_too_bright", False))]
    camera_low_contrast_samples = [s for s in samples if bool(s.get("target_camera_frame_low_contrast", False))]
    camera_stream_seed_samples = [s for s in samples if bool(s.get("target_camera_stream_seed", False))]
    camera_onnx_relock_samples = [s for s in samples if bool(s.get("target_camera_onnx_relock_person_candidate", False))]
    camera_detector_throttled_samples = [
        s for s in samples if bool(s.get("target_camera_detector_throttled", False))
    ]
    camera_capture_pending_samples = [
        s for s in samples if bool(s.get("target_camera_capture_pending", False))
    ]
    localization_gate_hard_stop_samples = [s for s in samples if bool(s.get("localization_gate_hard_stop", False))]
    localization_gate_idle_resume_samples = [
        s for s in samples if bool(s.get("localization_gate_idle_stationary_gap_recoverable", False))
    ]
    localization_gate_idle_resume_bridge_samples = [
        s for s in samples if bool(s.get("localization_gate_idle_resume_bridge_recoverable", False))
    ]
    localization_gate_current_apply_recovery_samples = [
        s for s in samples if bool(s.get("localization_gate_current_ekf_applied", False))
    ]
    localization_gate_gap_samples = [
        _safe_float(s.get("localization_gate_ekf_applied_gap_s"), 0.0)
        for s in samples
        if _safe_float(s.get("localization_gate_ekf_applied_gap_s"), 0.0) > 0.0
    ]
    lidar_raw_scan_age_samples = [
        _safe_float(s.get("lidar_odom_raw_scan_latest_age_s"), 0.0)
        for s in samples
        if _safe_float(s.get("lidar_odom_raw_scan_latest_age_s"), 0.0) > 0.0
    ]
    lidar_matcher_latency_samples = [
        _safe_float(s.get("lidar_odom_matcher_latency_ms"), 0.0)
        for s in samples
        if _safe_float(s.get("lidar_odom_matcher_latency_ms"), 0.0) > 0.0
    ]
    localization_motion_blocked_ratio = (
        float(len(localization_gate_hard_stop_samples)) / max(1, len(samples))
    )
    localization_motion_blocked_dominant = bool(
        len(localization_gate_hard_stop_samples) >= max(5, int(0.25 * max(1, len(samples))))
    )
    target_camera_failed_session_max = max((int(s.get("target_camera_failed_sessions", 0) or 0) for s in samples), default=0)
    camera_state_counts: Dict[str, int] = {}
    camera_detector_counts: Dict[str, int] = {}
    camera_distance_source_counts: Dict[str, int] = {}
    cruise_phase_counts: Dict[str, int] = {}
    search_state_counts: Dict[str, int] = {}
    localization_gate_mode_counts: Dict[str, int] = {}
    localization_gate_apply_reason_counts: Dict[str, int] = {}
    localization_gate_reason_counts: Dict[str, int] = {}
    for sample in samples:
        state = str(sample.get("target_camera_state") or "unknown")
        camera_state_counts[state] = int(camera_state_counts.get(state, 0)) + 1
        detector = str(sample.get("target_camera_detector") or "unknown")
        camera_detector_counts[detector] = int(camera_detector_counts.get(detector, 0)) + 1
        distance_source = str(sample.get("target_camera_distance_source") or "none")
        camera_distance_source_counts[distance_source] = int(camera_distance_source_counts.get(distance_source, 0)) + 1
        phase = str(sample.get("room_cruise_phase") or "unknown")
        cruise_phase_counts[phase] = int(cruise_phase_counts.get(phase, 0)) + 1
        search_state = str(sample.get("target_search_state") or "unknown")
        search_state_counts[search_state] = int(search_state_counts.get(search_state, 0)) + 1
        localization_mode = str(sample.get("localization_gate_mode") or "unknown")
        localization_gate_mode_counts[localization_mode] = int(localization_gate_mode_counts.get(localization_mode, 0)) + 1
        apply_reason = str(sample.get("localization_gate_apply_reason") or "none")
        localization_gate_apply_reason_counts[apply_reason] = int(
            localization_gate_apply_reason_counts.get(apply_reason, 0)
        ) + 1
        for reason in list(sample.get("localization_gate_reasons") or []):
            reason_key = str(reason or "").strip()
            if not reason_key:
                continue
            localization_gate_reason_counts[reason_key] = int(localization_gate_reason_counts.get(reason_key, 0)) + 1
    moving_samples = [
        s
        for s in samples
        if (
            abs(_safe_float(s.get("cmd_v"), 0.0)) >= 0.004
            or abs(_safe_float(s.get("cmd_omega"), 0.0)) >= 0.035
            or abs(_safe_float(s.get("track_left_mps"), 0.0)) >= 0.004
            or abs(_safe_float(s.get("track_right_mps"), 0.0)) >= 0.004
        )
    ]
    camera_detection_active_samples = [
        s
        for s in samples
        if _camera_detection_active(s)
    ]
    motion_without_camera_detection_samples = [
        s
        for s in moving_samples
        if not _camera_detection_active(s)
        and not _allowed_camera_search_motion(s)
        and not bool(s.get("room_cruise_camera_detection_clearance_retreat_active", False))
        and not bool(s.get("room_cruise_camera_detection_reacquire_rotate_allowed", False))
    ]
    camera_detection_motion_suppressed_count = sum(
        1 for s in samples if bool(s.get("room_cruise_camera_detection_motion_suppressed", False))
    )
    camera_detection_clearance_retreat_count = sum(
        1 for s in samples if bool(s.get("room_cruise_camera_detection_clearance_retreat_active", False))
    )
    camera_detection_reacquire_rotate_count = sum(
        1 for s in samples if bool(s.get("room_cruise_camera_detection_reacquire_rotate_allowed", False))
    )
    camera_simple_follow_count = sum(
        1 for s in samples if bool(s.get("room_cruise_camera_simple_follow_active", False))
    )
    camera_simple_forward_gate_blocked_count = sum(
        1 for s in samples if bool(s.get("room_cruise_camera_simple_forward_gate_blocked", False))
    )
    camera_simple_retreat_gate_blocked_count = sum(
        1 for s in samples if bool(s.get("room_cruise_camera_simple_retreat_gate_blocked", False))
    )
    camera_turn_side_blocked_samples = [
        s for s in samples if bool(s.get("room_cruise_camera_target_turn_side_blocked", False))
    ]
    camera_turn_wrong_side_samples = [s for s in samples if _camera_turn_wrong_side_sample(s)]
    camera_simple_turn_track_samples = [
        abs(_safe_float(s.get("room_cruise_camera_simple_turn_track_mps"), 0.0))
        for s in samples
        if bool(s.get("room_cruise_camera_simple_follow_active", False))
    ]
    camera_simple_forward_scale_samples = [
        _safe_float(s.get("room_cruise_camera_simple_heading_forward_scale"), 1.0)
        for s in samples
        if bool(s.get("room_cruise_camera_simple_follow_active", False))
    ]
    search_phase_names = {
        "target_search_arc",
        "target_search_rotate_360",
        "target_search_one_track",
        "target_search_in_place",
    }
    search_arc_samples = [s for s in samples if str(s.get("room_cruise_phase") or "") in search_phase_names]
    search_rotate_samples = [s for s in samples if str(s.get("room_cruise_phase") or "") == "target_search_rotate_360"]
    target_search_cmd_omega_abs_samples = [
        abs(_safe_float(s.get("cmd_omega"), 0.0))
        for s in search_arc_samples
    ]
    target_search_cmd_angular_abs_dps_samples = [
        abs(_safe_float(s.get("cmd_angular_dps"), 0.0))
        for s in search_arc_samples
    ]
    target_search_tuning_omega_samples = [
        _safe_float(s.get("follow_search_pivot_omega_rad_s"), 0.0)
        for s in search_arc_samples
        if _safe_float(s.get("follow_search_pivot_omega_rad_s"), 0.0) > 0.0
    ]
    camera_distance_estimate_samples = [
        _safe_float(s.get("target_camera_distance_estimate_m"), 0.0)
        for s in target_samples
        if _safe_float(s.get("target_camera_distance_estimate_m"), 0.0) > 0.0
    ]
    camera_distance_used_samples = [
        _safe_float(s.get("target_camera_distance_used_m"), 0.0)
        for s in target_samples
        if _safe_float(s.get("target_camera_distance_used_m"), 0.0) > 0.0
    ]
    camera_luma_samples = [
        _safe_float(s.get("target_camera_frame_luma_mean"), 0.0)
        for s in samples
        if bool(s.get("target_camera_frame_ok", False))
    ]
    camera_contrast_samples = [
        _safe_float(s.get("target_camera_frame_luma_std"), 0.0)
        for s in samples
        if bool(s.get("target_camera_frame_ok", False))
    ]
    follow_distance_samples = [
        _safe_float(s.get("follow_actual_distance_m"), 0.0)
        for s in target_samples
        if _safe_float(s.get("follow_actual_distance_m"), 0.0) > 0.0
    ]
    target_angle_error_samples = [
        _angle_error_abs_deg(_safe_float(s.get("follow_actual_bearing_rad"), 0.0))
        for s in target_samples
    ]
    fresh_camera_target_samples = [s for s in target_samples if _camera_detection_active(s)]
    camera_response_required_samples = [
        s for s in fresh_camera_target_samples if _camera_target_response_required_sample(s)
    ]
    camera_response_required_motion_samples = [
        s
        for s in camera_response_required_samples
        if _sample_has_motion(s) and not bool(s.get("localization_gate_hard_stop", False))
    ]
    fresh_camera_bearing_error_samples = [
        _angle_error_abs_deg(_safe_float(s.get("follow_actual_bearing_rad"), 0.0))
        for s in fresh_camera_target_samples
    ]
    human_follow_v2_bearing_samples: List[float] = []
    human_follow_v2_bearing_phase_counts: Dict[str, int] = {}
    for sample in fresh_camera_target_samples:
        phase = str(sample.get("room_cruise_phase") or "unknown")
        bearing_deg = _angle_error_abs_deg(_safe_float(sample.get("follow_actual_bearing_rad"), 0.0))
        stable_phase = phase in HUMAN_FOLLOW_V2_BEARING_STABLE_PHASES
        stable_approach = bool(
            phase in HUMAN_FOLLOW_V2_BEARING_STABLE_APPROACH_PHASES
            and float(bearing_deg) <= HUMAN_FOLLOW_V2_APPROACH_STABLE_BEARING_MAX_DEG
        )
        if not (stable_phase or stable_approach):
            continue
        human_follow_v2_bearing_samples.append(float(bearing_deg))
        human_follow_v2_bearing_phase_counts[phase] = int(
            human_follow_v2_bearing_phase_counts.get(phase, 0)
        ) + 1
    target_distance_error_samples = [
        abs(
            _safe_float(s.get("follow_actual_distance_m"), 0.0)
            - max(0.01, _safe_float(s.get("follow_desired_distance_m"), 1.0))
        )
        for s in target_samples
        if _safe_float(s.get("follow_actual_distance_m"), 0.0) > 0.0
    ]
    target_translation_samples = [s for s in target_samples if abs(_safe_float(s.get("cmd_v"), 0.0)) > 0.002]
    target_forward_samples = [s for s in target_samples if _safe_float(s.get("cmd_v"), 0.0) > 0.002]
    target_reverse_samples = [s for s in target_samples if _safe_float(s.get("cmd_v"), 0.0) < -0.002]
    wrong_reverse_samples = []
    for sample in target_reverse_samples:
        desired_m = max(0.01, _safe_float(sample.get("follow_desired_distance_m"), follow_distance_m))
        distance_m = _safe_float(sample.get("follow_actual_distance_m"), 0.0)
        if distance_m <= 0.0:
            distance_m = _safe_float(sample.get("target_camera_distance_used_m"), 0.0)
        camera_valid = bool(
            str(sample.get("follow_target_source") or "") == "CAMERA_TARGET"
            and bool(sample.get("target_camera_usable", False))
            and not bool(sample.get("target_camera_stale", False))
        )
        if (not camera_valid) or distance_m <= 0.0 or float(distance_m) > float(desired_m) - 0.04:
            wrong_reverse_samples.append(sample)
    non_target_translation_samples = [
        s
        for s in samples
        if str(s.get("follow_target_source") or "") not in {"CAMERA_TARGET", "CAMERA_SEARCH"}
        and not bool(s.get("target_camera_usable", False))
        and abs(_safe_float(s.get("cmd_v"), 0.0)) > 0.002
    ]
    v2_local_navigation_samples = [
        s
        for s in target_samples
        if str(s.get("command_type") or "") == "local_planner_segment"
        and not bool(s.get("cruise_layer_local_planner_bypassed", True))
        and bool(s.get("cruise_layer_local_navigation_active", False))
    ]
    target_moving_direct_bypass_samples = [
        s
        for s in target_translation_samples
        if bool(s.get("cruise_layer_local_planner_bypassed", True))
        or str(s.get("command_type") or "") != "local_planner_segment"
    ]
    active_route_counts: Dict[str, int] = {}
    for sample in samples:
        route = str(sample.get("active_route") or "none")
        active_route_counts[route] = int(active_route_counts.get(route, 0)) + 1
    camera_route_samples = [
        s
        for s in samples
        if str(s.get("follow_target_source") or "") in {"CAMERA_TARGET", "CAMERA_SEARCH"}
    ]
    human_follow_v2_samples = [
        s for s in camera_route_samples if str(s.get("active_route") or "") == HUMAN_FOLLOW_V2_ROUTE
    ]
    direct_motor_bypass_samples = [s for s in samples if bool(s.get("direct_motor_bypass", False))]
    legacy_generic_planner_samples = [
        s for s in samples if bool(s.get("used_legacy_generic_planner", False))
    ]
    human_follow_v2_route_ok = bool(
        len(camera_route_samples) > 0
        and len(human_follow_v2_samples) == len(camera_route_samples)
        and len(direct_motor_bypass_samples) == 0
        and len(legacy_generic_planner_samples) == 0
    )
    active_route = HUMAN_FOLLOW_V2_ROUTE if human_follow_v2_route_ok else "mixed_or_missing"
    emergency_stop_events = max(
        0,
        max((int(s.get("last_emergency_count", 0) or 0) for s in samples), default=initial_emergency_count)
        - int(initial_emergency_count),
    )
    failsafe_events = max(
        0,
        max((int(s.get("global_motion_failsafe_events", 0) or 0) for s in samples), default=initial_failsafe_count)
        - int(initial_failsafe_count),
    )
    reverse_without_clearance_samples = [
        s
        for s in target_reverse_samples
        if not (
            bool(s.get("local_navigation_rear_clear_for_retreat", False))
            or bool(s.get("room_cruise_rear_clear_for_retreat", False))
        )
    ]
    front_arbitrated_samples = [
        s
        for s in target_samples
        if str(s.get("target_camera_gate") or "") == "front_lidar_obstacle_arbitrated_by_camera_distance"
    ]
    hold_like_phases = {"target_hold", "target_hold_heading_align", "front_hold_camera_retreat", "lidar_confidence_hold"}
    hold_like_target_samples = [
        s for s in target_samples if str(s.get("room_cruise_phase") or "") in hold_like_phases
    ]
    longest_camera_target_run = _longest_run(
        samples,
        lambda s: str(s.get("follow_target_source") or "") == "CAMERA_TARGET",
    )
    longest_target_lost_run = _longest_unhandled_target_lost_run(samples)
    longest_search_run = _longest_run(
        samples,
        lambda s: str(s.get("follow_target_source") or "") == "CAMERA_SEARCH",
    )
    longest_forward_run = _longest_run(
        samples,
        lambda s: _safe_float(s.get("cmd_v"), 0.0) > 0.002,
    )
    longest_reverse_run = _longest_run(
        samples,
        lambda s: _safe_float(s.get("cmd_v"), 0.0) < -0.002,
    )
    longest_translation_run = _longest_run(
        samples,
        lambda s: abs(_safe_float(s.get("cmd_v"), 0.0)) > 0.002,
    )
    longest_wall_stick_run = _longest_run(samples, _wall_stick_sample)
    longest_camera_turn_wrong_side_run = _longest_run(samples, _camera_turn_wrong_side_sample)
    post_acquire_samples = _post_acquire_samples(samples)
    longest_post_lock_candidate_hold_run = _longest_run(
        post_acquire_samples,
        lambda s: str(s.get("follow_target_source") or "") not in {"CAMERA_TARGET", "CAMERA_SEARCH"}
        and bool(s.get("target_camera_visible", False))
        and not bool(s.get("target_camera_usable", False)),
    )
    longest_target_active_zero_gap_run = _longest_run(samples, _target_active_zero_gap_sample)
    command_delta_v_samples = _command_delta_samples(samples, "cmd_v")
    command_delta_omega_samples = _command_delta_samples(samples, "cmd_omega")
    direct_track_reference_samples = [
        s
        for s in samples
        if str(s.get("command_type") or "").strip().lower() == "set_track_velocity"
        and _sample_has_motion(s)
        and str(s.get("follow_target_source") or "") in {"CAMERA_TARGET", "CAMERA_SEARCH"}
    ]
    first_camera_target_elapsed_s = _first_elapsed_s(
        samples,
        lambda s: str(s.get("follow_target_source") or "") == "CAMERA_TARGET",
    )
    first_camera_active_elapsed_s = _first_elapsed_s(samples, _camera_detection_active)
    first_camera_search_elapsed_s = _first_elapsed_s(
        samples,
        lambda s: str(s.get("follow_target_source") or "") == "CAMERA_SEARCH",
    )
    behavior_duration_s = _analysis_duration_s(samples, getattr(args, "duration_s", 0.0))
    follow_state_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: str(s.get("state") or "").upper() == "FOLLOW",
    )
    active_follow_duration_s = _timed_sum_s(samples, behavior_duration_s, _actual_camera_follow_sample)
    target_search_duration_s = _timed_sum_s(samples, behavior_duration_s, _camera_search_or_pivot_sample)
    target_lost_duration_s = _timed_sum_s(samples, behavior_duration_s, _target_lost_sample)
    camera_target_visible_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: bool(s.get("target_camera_visible", False)),
    )
    camera_target_source_duration_s = _timed_sum_s(samples, behavior_duration_s, _camera_target_source_sample)
    usable_camera_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: bool(s.get("target_camera_usable", False)),
    )
    stale_camera_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: bool(s.get("target_camera_stale", False)),
    )
    search_pivot_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: _camera_search_or_pivot_sample(s)
        and (
            abs(_safe_float(s.get("cmd_omega"), 0.0)) >= 0.035
            or abs(_safe_float(s.get("expected_omega"), 0.0)) >= 0.035
            or str(s.get("room_cruise_phase") or "") == "target_search_in_place"
        ),
    )
    pivot_without_fresh_camera_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: _camera_search_or_pivot_sample(s)
        and (
            abs(_safe_float(s.get("cmd_omega"), 0.0)) >= 0.035
            or abs(_safe_float(s.get("expected_omega"), 0.0)) >= 0.035
            or str(s.get("room_cruise_phase") or "") == "target_search_in_place"
        )
        and not _camera_detection_active(s),
    )
    longest_search_or_pivot_s = _timed_longest_run_s(
        samples,
        behavior_duration_s,
        _camera_search_or_pivot_sample,
    )
    relock_stable_follow_durations_s = _relock_stable_follow_durations_s(samples, behavior_duration_s)
    search_follow_search_cycles, search_follow_search_cycles_per_min = _search_follow_search_cycles_per_min(
        samples,
        behavior_duration_s,
    )
    follow_state_ratio = float(follow_state_duration_s) / max(0.001, float(behavior_duration_s))
    active_follow_ratio = float(active_follow_duration_s) / max(0.001, float(behavior_duration_s))
    target_search_ratio = float(target_search_duration_s) / max(0.001, float(behavior_duration_s))
    target_lost_ratio = float(target_lost_duration_s) / max(0.001, float(behavior_duration_s))
    camera_target_visible_ratio = float(camera_target_visible_duration_s) / max(0.001, float(behavior_duration_s))
    camera_target_source_ratio = float(camera_target_source_duration_s) / max(0.001, float(behavior_duration_s))
    usable_camera_ratio = float(usable_camera_duration_s) / max(0.001, float(behavior_duration_s))
    stale_camera_ratio = float(stale_camera_duration_s) / max(0.001, float(behavior_duration_s))
    search_pivot_ratio = float(search_pivot_duration_s) / max(0.001, float(behavior_duration_s))
    pivot_without_fresh_camera_ratio = float(pivot_without_fresh_camera_duration_s) / max(
        0.001,
        float(behavior_duration_s),
    )
    localization_motion_blocked_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: bool(s.get("localization_gate_hard_stop", False)),
    )
    localization_motion_blocked_time_ratio = float(localization_motion_blocked_duration_s) / max(
        0.001,
        float(behavior_duration_s),
    )
    camera_response_required_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        _camera_target_response_required_sample,
    )
    camera_response_required_motion_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: _camera_target_response_required_sample(s)
        and _sample_has_motion(s)
        and not bool(s.get("localization_gate_hard_stop", False)),
    )
    camera_response_required_no_motion_duration_s = _timed_sum_s(
        samples,
        behavior_duration_s,
        lambda s: _camera_target_response_required_sample(s)
        and (
            not _sample_has_motion(s)
            or bool(s.get("localization_gate_hard_stop", False))
        ),
    )
    camera_response_required_no_motion_longest_s = _timed_longest_run_s(
        samples,
        behavior_duration_s,
        lambda s: _camera_target_response_required_sample(s)
        and (
            not _sample_has_motion(s)
            or bool(s.get("localization_gate_hard_stop", False))
        ),
    )
    camera_response_required_motion_ratio = (
        1.0
        if float(camera_response_required_duration_s) < 0.001
        else float(camera_response_required_motion_duration_s)
        / max(0.001, float(camera_response_required_duration_s))
    )
    actual_follow_ratio_ok = bool(active_follow_ratio >= HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO)
    target_search_ratio_ok = bool(target_search_ratio <= HUMAN_FOLLOW_V2_MAX_SEARCH_RATIO)
    search_pivot_duration_ok = bool(
        longest_search_or_pivot_s <= HUMAN_FOLLOW_V2_MAX_CONTIGUOUS_SEARCH_PIVOT_S
    )
    localization_block_ok = bool(
        localization_motion_blocked_time_ratio <= HUMAN_FOLLOW_V2_MAX_LOCALIZATION_BLOCK_RATIO
    )
    camera_target_response_motion_ok = bool(
        float(camera_response_required_duration_s) < 1.0
        or (
            float(camera_response_required_motion_ratio) >= HUMAN_FOLLOW_V2_MIN_RESPONSE_MOTION_RATIO
            and float(camera_response_required_no_motion_longest_s)
            <= HUMAN_FOLLOW_V2_MAX_RESPONSE_NO_MOTION_S
        )
    )
    human_follow_v2_behavior_balance_ok = bool(
        actual_follow_ratio_ok
        and target_search_ratio_ok
        and search_pivot_duration_ok
        and localization_block_ok
        and camera_target_response_motion_ok
    )
    settled_distance_start_s = (
        None
        if first_camera_target_elapsed_s is None
        else float(first_camera_target_elapsed_s) + HUMAN_FOLLOW_V2_SETTLED_DISTANCE_AFTER_FIRST_TARGET_S
    )
    settled_target_samples = [
        s
        for s in target_samples
        if settled_distance_start_s is not None
        and _safe_float(s.get("elapsed_s"), 0.0) >= float(settled_distance_start_s)
        and _safe_float(s.get("follow_actual_distance_m"), 0.0) > 0.0
    ]
    settled_target_distance_error_samples = [
        abs(
            _safe_float(s.get("follow_actual_distance_m"), 0.0)
            - max(0.01, _safe_float(s.get("follow_desired_distance_m"), 1.0))
        )
        for s in settled_target_samples
    ]
    strict_target_distance_error_samples = (
        settled_target_distance_error_samples
        if len(settled_target_distance_error_samples) >= HUMAN_FOLLOW_V2_SETTLED_DISTANCE_MIN_SAMPLES
        else target_distance_error_samples
    )
    max_elapsed_s = max((_safe_float(s.get("elapsed_s"), 0.0) for s in samples), default=0.0)
    final_window_samples = [
        s for s in samples if _safe_float(s.get("elapsed_s"), 0.0) >= max(0.0, max_elapsed_s - FINAL_TARGET_WINDOW_S)
    ]
    final_tail_samples = [
        s for s in samples if _safe_float(s.get("elapsed_s"), 0.0) >= max(0.0, max_elapsed_s - 1.0)
    ]
    final_camera_active_samples = [s for s in final_window_samples if _camera_detection_active(s)]
    final_camera_tail_active_samples = [s for s in final_tail_samples if _camera_detection_active(s)]
    final_localization_hard_stop_samples = [
        s for s in final_window_samples if bool(s.get("localization_gate_hard_stop", False))
    ]
    final_distance_samples: List[float] = []
    final_distance_error_samples: List[float] = []
    for sample in final_camera_active_samples:
        distance_m = _safe_float(sample.get("target_camera_distance_used_m"), 0.0)
        if distance_m <= 0.0:
            distance_m = _safe_float(sample.get("follow_actual_distance_m"), 0.0)
        if distance_m <= 0.0:
            continue
        desired_m = max(0.01, _safe_float(sample.get("follow_desired_distance_m"), follow_distance_m))
        final_distance_samples.append(float(distance_m))
        final_distance_error_samples.append(abs(float(distance_m) - float(desired_m)))

    def _one_track_cmd(sample: Dict[str, Any]) -> bool:
        left = abs(_safe_float(sample.get("track_left_mps"), 0.0))
        right = abs(_safe_float(sample.get("track_right_mps"), 0.0))
        return bool((left <= 1e-6 and right >= 0.004) or (right <= 1e-6 and left >= 0.004))

    def _in_place_rotate_cmd(sample: Dict[str, Any]) -> bool:
        left = _safe_float(sample.get("track_left_mps"), 0.0)
        right = _safe_float(sample.get("track_right_mps"), 0.0)
        return bool(
            abs(left) >= 0.004
            and abs(right) >= 0.004
            and left * right < 0.0
            and abs(left + right) <= 0.004
        )

    target_search_one_track_cmd_count = sum(1 for s in search_arc_samples if _one_track_cmd(s))
    target_search_in_place_rotate_cmd_count = sum(1 for s in search_arc_samples if _in_place_rotate_cmd(s))
    target_search_forward_cmd_count = sum(1 for s in search_arc_samples if _safe_float(s.get("cmd_v"), 0.0) > 0.002)
    camera_angle_side_flip_count = 0
    expected_omega_side_flip_count = 0
    prev_camera_side = "center"
    prev_expected_turn_side = "center"
    for sample in target_samples:
        camera_side = _camera_image_side_from_angle(sample.get("adaptive_target_angle_deg"))
        expected_side = _turn_side_from_omega(sample.get("expected_omega"))
        if camera_side in {"left", "right"}:
            if prev_camera_side in {"left", "right"} and camera_side != prev_camera_side:
                camera_angle_side_flip_count += 1
            prev_camera_side = str(camera_side)
        if expected_side in {"left", "right"}:
            if prev_expected_turn_side in {"left", "right"} and expected_side != prev_expected_turn_side:
                expected_omega_side_flip_count += 1
            prev_expected_turn_side = str(expected_side)
    motion_expected_executed_v_errors = [
        _safe_float(s.get("motion_expected_executed_v_error"), 0.0)
        for s in cruise_samples
    ]
    motion_expected_executed_omega_errors = [
        _safe_float(s.get("motion_expected_executed_omega_error"), 0.0)
        for s in cruise_samples
    ]
    motion_expected_executed_track_errors = [
        _safe_float(s.get("motion_expected_executed_track_error_mps"), 0.0)
        for s in cruise_samples
    ]
    sample_rate_hz = max(1.0, float(args.sample_rate_hz))
    target_lost_gap_max_s = round(float(longest_target_lost_run) / sample_rate_hz, 3)
    post_lock_candidate_hold_longest_run_s = round(float(longest_post_lock_candidate_hold_run) / sample_rate_hz, 3)
    target_active_zero_gap_longest_run_s = round(float(longest_target_active_zero_gap_run) / sample_rate_hz, 3)
    command_delta_v_p90 = _percentile(command_delta_v_samples, 0.90)
    command_delta_omega_p90 = _percentile(command_delta_omega_samples, 0.90)
    command_delta_omega_max = _percentile(command_delta_omega_samples, 1.00)
    wall_stick_longest_run_s = round(float(longest_wall_stick_run) / sample_rate_hz, 3)
    stable_detection_ok = bool(len(target_samples) >= max(8, int(0.20 * max(1, len(samples)))) and longest_camera_target_run >= int(2.0 * sample_rate_hz))
    distance_control_motion_ok = bool(
        len(target_translation_samples) > 0
        and len(non_target_translation_samples) == 0
    )
    v2_local_navigation_ok = bool(
        len(v2_local_navigation_samples) >= max(3, int(0.20 * max(1, len(target_samples))))
        and len(target_moving_direct_bypass_samples) == 0
    )
    reverse_clearance_ok = bool(len(reverse_without_clearance_samples) == 0)
    target_search_in_place_ok = bool(
        len(search_arc_samples) == 0 or target_search_in_place_rotate_cmd_count > 0
    )
    camera_detection_motion_only_ok = bool(len(motion_without_camera_detection_samples) == 0)
    camera_distance_ok = bool(len(camera_distance_estimate_samples) > 0)
    wall_stick_ok = bool(len(wall_stick_samples) == 0 and wall_stick_longest_run_s <= WALL_STICK_LONGEST_RUN_MAX_S)
    camera_turn_wrong_side_ok = bool(len(camera_turn_wrong_side_samples) == 0)
    target_distance_p50 = _percentile(follow_distance_samples, 0.50)
    target_distance_error_p90 = _percentile(target_distance_error_samples, 0.90)
    strict_target_distance_error_p50 = _percentile(strict_target_distance_error_samples, 0.50)
    strict_target_distance_error_p90 = _percentile(strict_target_distance_error_samples, 0.90)
    target_angle_error_p50 = _percentile(target_angle_error_samples, 0.50)
    target_angle_error_p90 = _percentile(target_angle_error_samples, 0.90)
    fresh_bearing_error_p50 = _percentile(fresh_camera_bearing_error_samples, 0.50)
    fresh_bearing_error_p90 = _percentile(fresh_camera_bearing_error_samples, 0.90)
    human_follow_v2_bearing_error_p50 = _percentile(human_follow_v2_bearing_samples, 0.50)
    human_follow_v2_bearing_error_p90 = _percentile(human_follow_v2_bearing_samples, 0.90)
    human_follow_v2_bearing_min_samples = (
        max(
            5,
            int(
                math.ceil(
                    HUMAN_FOLLOW_V2_BEARING_MIN_STABLE_SAMPLE_FRACTION
                    * max(1, len(fresh_camera_target_samples))
                )
            ),
        )
        if len(fresh_camera_target_samples) > 0
        else 0
    )
    human_follow_v2_bearing_sample_count_ok = bool(
        len(human_follow_v2_bearing_samples) >= int(human_follow_v2_bearing_min_samples)
    )
    bearing_error_p50 = (
        human_follow_v2_bearing_error_p50
        if bool(getattr(args, "strict_v2", False))
        else target_angle_error_p50
    )
    bearing_error_p90 = (
        human_follow_v2_bearing_error_p90
        if bool(getattr(args, "strict_v2", False))
        else target_angle_error_p90
    )
    final_target_distance_p50 = _percentile(final_distance_samples, 0.50)
    final_target_distance_error_p50 = _percentile(final_distance_error_samples, 0.50)
    final_camera_target_ok = bool(
        len(final_window_samples) > 0
        and (
            len(final_camera_active_samples) >= max(2, int(math.ceil(0.50 * len(final_window_samples))))
            or (
                len(final_camera_tail_active_samples) > 0
                and float(target_lost_gap_max_s) <= FOLLOW_ACCEPT_TARGET_LOST_GAP_MAX_S
            )
        )
        and len(final_camera_tail_active_samples) > 0
    )
    final_target_distance_ok = bool(
        final_camera_target_ok
        and final_target_distance_p50 is not None
        and final_target_distance_error_p50 is not None
        and float(follow_acceptance["min_m"]) <= float(final_target_distance_p50) <= float(follow_acceptance["max_m"])
        and float(final_target_distance_error_p50) <= FINAL_TARGET_DISTANCE_ERROR_MAX_M
    )
    angular_velocity_fresh_camera_samples = [
        abs(_safe_float(s.get("cmd_omega"), 0.0))
        for s in fresh_camera_target_samples
    ]
    angular_velocity_all_samples = [
        abs(_safe_float(s.get("cmd_omega"), 0.0))
        for s in samples
    ]
    angular_velocity_fresh_camera_max = _percentile(angular_velocity_fresh_camera_samples, 1.0)
    angular_velocity_p90 = _percentile(angular_velocity_all_samples, 0.90)
    blind_forward_samples = [
        s
        for s in samples
        if _safe_float(s.get("cmd_v"), 0.0) > 0.002
        and str(s.get("follow_target_source") or "") != "CAMERA_TARGET"
        and not _allowed_camera_search_motion(s)
        and not bool(s.get("room_cruise_camera_detection_clearance_retreat_active", False))
    ]
    human_follow_v2_strict_ok = True
    if bool(getattr(args, "strict_v2", False)):
        human_follow_v2_strict_ok = bool(
            human_follow_v2_route_ok
            and len(legacy_generic_planner_samples) == 0
            and len(direct_motor_bypass_samples) == 0
            and int(emergency_stop_events) == 0
            and int(failsafe_events) == 0
            and len(blind_forward_samples) == 0
            and len(wall_stick_samples) == 0
            and human_follow_v2_behavior_balance_ok
            and 0.95 <= float(follow_distance_m) <= 1.05
            and target_distance_p50 is not None
            and HUMAN_FOLLOW_V2_TARGET_DISTANCE_MIN_M
            <= float(target_distance_p50)
            <= HUMAN_FOLLOW_V2_TARGET_DISTANCE_MAX_M
            and strict_target_distance_error_p50 is not None
            and float(strict_target_distance_error_p50)
            <= HUMAN_FOLLOW_V2_TARGET_DISTANCE_ERROR_P50_MAX_M
            and strict_target_distance_error_p90 is not None
            and float(strict_target_distance_error_p90) <= HUMAN_FOLLOW_V2_TARGET_DISTANCE_ERROR_P90_MAX_M
            and human_follow_v2_bearing_sample_count_ok
            and bearing_error_p50 is not None
            and float(bearing_error_p50) <= HUMAN_FOLLOW_V2_BEARING_ERROR_P50_MAX_DEG
            and bearing_error_p90 is not None
            and float(bearing_error_p90) <= HUMAN_FOLLOW_V2_BEARING_ERROR_P90_MAX_DEG
            and (
                angular_velocity_fresh_camera_max is None
                or float(angular_velocity_fresh_camera_max)
                <= float(motion_quality_tuning["fresh_target_omega_max_rad_s"])
            )
            and (
                angular_velocity_p90 is None
                or float(angular_velocity_p90) <= float(motion_quality_tuning["omega_p90_max_rad_s"])
            )
        )
    target_distance_ok = bool(
        target_distance_p50 is not None
        and float(follow_acceptance["min_m"]) <= float(target_distance_p50) <= float(follow_acceptance["max_m"])
        and target_distance_error_p90 is not None
        and float(target_distance_error_p90) <= float(follow_acceptance["error_p90_m"])
    )
    target_angle_ok = bool(
        target_angle_error_p50 is not None
        and target_angle_error_p90 is not None
        and float(target_angle_error_p50) <= FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P50_DEG
        and float(target_angle_error_p90) <= FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P90_DEG
    )
    target_lost_gap_ok = bool(float(target_lost_gap_max_s) <= FOLLOW_ACCEPT_TARGET_LOST_GAP_MAX_S)
    motion_expected_executed_v_error_p50 = _percentile(motion_expected_executed_v_errors, 0.50)
    motion_expected_executed_v_error_p90 = _percentile(motion_expected_executed_v_errors, 0.90)
    motion_expected_executed_omega_error_p50 = _percentile(motion_expected_executed_omega_errors, 0.50)
    motion_expected_executed_omega_error_p90 = _percentile(motion_expected_executed_omega_errors, 0.90)
    motion_expected_executed_track_error_p50 = _percentile(motion_expected_executed_track_errors, 0.50)
    motion_expected_executed_track_error_p90 = _percentile(motion_expected_executed_track_errors, 0.90)
    motion_expected_executed_ok = bool(
        len(cruise_samples) > 0
        and motion_expected_executed_v_error_p90 is not None
        and motion_expected_executed_omega_error_p90 is not None
        and motion_expected_executed_track_error_p90 is not None
        and float(motion_expected_executed_v_error_p90) <= MOTION_EXPECTED_EXECUTED_V_ERROR_P90_MAX_MPS
        and float(motion_expected_executed_omega_error_p90) <= MOTION_EXPECTED_EXECUTED_OMEGA_ERROR_P90_MAX_RAD_S
        and float(motion_expected_executed_track_error_p90) <= MOTION_EXPECTED_EXECUTED_TRACK_ERROR_P90_MAX_MPS
    )
    motion_quality_ok = bool(
        distance_control_motion_ok
        and v2_local_navigation_ok
        and reverse_clearance_ok
        and target_distance_ok
        and target_angle_ok
        and target_lost_gap_ok
        and motion_expected_executed_ok
        and wall_stick_ok
        and camera_turn_wrong_side_ok
        and final_camera_target_ok
        and final_target_distance_ok
    )
    startup_lock_ok = bool(
        first_camera_target_elapsed_s is not None
        and float(first_camera_target_elapsed_s) <= FOLLOW_START_FIRST_LOCK_MAX_S
    )
    post_lock_candidate_hold_ok = bool(
        float(post_lock_candidate_hold_longest_run_s) <= FOLLOW_POST_LOCK_CANDIDATE_HOLD_MAX_S
    )
    command_delta_ok = bool(
        (command_delta_v_p90 is None or float(command_delta_v_p90) <= FOLLOW_COMMAND_DELTA_V_P90_MAX_MPS)
        and (
            command_delta_omega_p90 is None
            or float(command_delta_omega_p90)
            <= float(motion_quality_tuning["command_delta_omega_p90_max_rad_s"])
        )
        and (
            command_delta_omega_max is None
            or float(command_delta_omega_max) <= float(motion_quality_tuning["command_delta_omega_max_rad_s"])
        )
    )
    target_active_zero_gap_ok = bool(
        float(target_active_zero_gap_longest_run_s) <= FOLLOW_TARGET_ACTIVE_ZERO_GAP_MAX_S
    )
    robotics_safety_ok = bool(
        not _new_emergency_observed(samples, initial_emergency_count)
        and int(emergency_stop_events) == 0
        and int(failsafe_events) == 0
        and len(blind_forward_samples) == 0
        and wall_stick_ok
        and reverse_clearance_ok
        and int(target_search_forward_cmd_count) == 0
    )
    command_path_integrity_ok = bool(
        human_follow_v2_route_ok
        and len(legacy_generic_planner_samples) == 0
        and len(direct_motor_bypass_samples) == 0
        and len(direct_track_reference_samples) == 0
        and v2_local_navigation_ok
        and motion_expected_executed_ok
    )
    motion_continuity_ok = bool(
        motion_expected_executed_ok
        and target_lost_gap_ok
        and target_active_zero_gap_ok
        and command_delta_ok
        and float(post_lock_candidate_hold_longest_run_s) <= FOLLOW_POST_LOCK_UNHANDLED_MAX_S
    )
    target_alignment_ok = bool(
        target_angle_ok
        and human_follow_v2_bearing_sample_count_ok
        and camera_turn_wrong_side_ok
        and int(camera_angle_side_flip_count) <= FOLLOW_SIDE_FLIP_MAX_COUNT
        and int(expected_omega_side_flip_count) <= FOLLOW_SIDE_FLIP_MAX_COUNT
    )
    distance_behavior_ok = bool(
        target_distance_ok
        and final_target_distance_ok
        and reverse_clearance_ok
        and len(wrong_reverse_samples) == 0
    )
    target_state_ok = bool(
        stable_detection_ok
        and startup_lock_ok
        and final_camera_target_ok
        and camera_detection_motion_only_ok
        and target_search_in_place_ok
    )
    relock_stability_ok = bool(
        target_lost_gap_ok
        and post_lock_candidate_hold_ok
        and camera_turn_wrong_side_ok
        and int(camera_angle_side_flip_count) <= FOLLOW_SIDE_FLIP_MAX_COUNT
        and int(expected_omega_side_flip_count) <= FOLLOW_SIDE_FLIP_MAX_COUNT
    )
    room_cruise_similarity_ok = bool(
        command_path_integrity_ok
        and motion_continuity_ok
        and command_delta_ok
        and angular_velocity_p90 is not None
        and float(angular_velocity_p90) <= float(motion_quality_tuning["omega_p90_max_rad_s"])
        and len(wrong_reverse_samples) == 0
        and len(moving_samples) > 0
    )
    sample_coverage_ok = bool(
        len(samples)
        >= int(
            max(
                1.0,
                float(args.duration_s) * max(1.0, float(args.sample_rate_hz)) * FOLLOW_MIN_SAMPLE_COVERAGE_FRACTION,
            )
        )
    )
    repeatability_ok = bool(
        sample_coverage_ok
        and robotics_safety_ok
        and command_path_integrity_ok
        and target_state_ok
        and relock_stability_ok
        and int(target_camera_failed_session_max) == 0
        and len(camera_open_failed_samples) == 0
    )
    human_follow_criteria_status = "PASS" if all(
        (
            stable_detection_ok,
            motion_quality_ok,
            robotics_safety_ok,
            command_path_integrity_ok,
            motion_continuity_ok,
            target_alignment_ok,
            distance_behavior_ok,
            target_state_ok,
            relock_stability_ok,
            (human_follow_v2_behavior_balance_ok if bool(getattr(args, "strict_v2", False)) else True),
            (localization_block_ok if bool(getattr(args, "strict_v2", False)) else True),
            (camera_target_response_motion_ok if bool(getattr(args, "strict_v2", False)) else True),
            room_cruise_similarity_ok,
            repeatability_ok,
            distance_control_motion_ok,
            v2_local_navigation_ok,
            reverse_clearance_ok,
            target_search_in_place_ok,
            camera_detection_motion_only_ok,
            camera_distance_ok,
            target_distance_ok,
            target_angle_ok,
            target_lost_gap_ok,
            motion_expected_executed_ok,
            wall_stick_ok,
            camera_turn_wrong_side_ok,
            final_camera_target_ok,
            final_target_distance_ok,
            not _new_emergency_observed(samples, initial_emergency_count),
        )
    ) else "FAIL"
    if bool(getattr(args, "strict_v2", False)) and not human_follow_v2_strict_ok:
        human_follow_criteria_status = "FAIL"
    if localization_motion_blocked_dominant:
        errors.append("localization_motion_blocked_dominant")
    if bool(getattr(args, "strict_v2", False)) and not localization_block_ok:
        errors.append("localization_motion_blocked_gate_failed")
    if bool(getattr(args, "strict_v2", False)) and not camera_target_response_motion_ok:
        errors.append("camera_target_response_motion_gate_failed")
    if (
        bool(getattr(args, "strict_v2", False))
        and float(camera_response_required_no_motion_longest_s)
        > HUMAN_FOLLOW_V2_MAX_RESPONSE_NO_MOTION_S
    ):
        errors.append("camera_target_response_no_motion_run_gate_failed")
    if not robotics_safety_ok:
        errors.append("robotics_safety_gate_failed")
    if not command_path_integrity_ok:
        errors.append("command_path_integrity_gate_failed")
    if not motion_continuity_ok:
        errors.append("motion_continuity_gate_failed")
    if not target_alignment_ok:
        errors.append("target_alignment_gate_failed")
    if not distance_behavior_ok:
        errors.append("distance_behavior_gate_failed")
    if not target_state_ok:
        errors.append("target_state_gate_failed")
    if not relock_stability_ok:
        errors.append("relock_stability_gate_failed")
    if bool(getattr(args, "strict_v2", False)) and not human_follow_v2_behavior_balance_ok:
        errors.append("human_follow_v2_behavior_balance_gate_failed")
    if bool(getattr(args, "strict_v2", False)) and not target_search_ratio_ok:
        errors.append("target_search_ratio_gate_failed")
    if bool(getattr(args, "strict_v2", False)) and not search_pivot_duration_ok:
        errors.append("search_pivot_duration_gate_failed")
    if bool(getattr(args, "strict_v2", False)) and not actual_follow_ratio_ok:
        errors.append("actual_follow_ratio_gate_failed")
    if not room_cruise_similarity_ok:
        errors.append("room_cruise_similarity_gate_failed")
    if not repeatability_ok:
        errors.append("repeatability_gate_failed")
    if not stable_detection_ok:
        errors.append("stable_camera_detection_failed")
    if not motion_quality_ok:
        errors.append("motion_quality_gate_failed")
    if not distance_control_motion_ok:
        errors.append("distance_control_motion_to_camera_target_failed")
    if not v2_local_navigation_ok:
        errors.append("v2_local_navigation_gate_failed")
    if not reverse_clearance_ok:
        errors.append("reverse_without_clearance_observed")
    if not target_search_in_place_ok:
        errors.append("target_search_in_place_missing")
    if not camera_detection_motion_only_ok:
        errors.append("motion_without_camera_detection_observed")
    if not camera_distance_ok:
        errors.append("camera_distance_estimate_failed")
    if not target_distance_ok:
        errors.append("target_distance_gate_failed")
    if not target_angle_ok:
        errors.append("target_angle_gate_failed")
    if not target_lost_gap_ok:
        errors.append("target_lost_gap_gate_failed")
    if not motion_expected_executed_ok:
        errors.append("motion_expected_executed_mismatch")
    if not wall_stick_ok:
        errors.append("wall_stick_observed")
    if not camera_turn_wrong_side_ok:
        errors.append("camera_turn_wrong_side_observed")
    if not final_camera_target_ok:
        errors.append("final_camera_target_missing")
    if not final_target_distance_ok:
        errors.append("final_target_distance_gate_failed")
    if bool(getattr(args, "strict_v2", False)):
        if not human_follow_v2_route_ok:
            errors.append("active_route_not_human_follow_v2")
        if len(legacy_generic_planner_samples) > 0:
            errors.append("legacy_generic_planner_observed")
        if len(direct_motor_bypass_samples) > 0:
            errors.append("direct_motor_bypass_observed")
        if int(emergency_stop_events) > 0:
            errors.append("emergency_stop_events_observed")
        if int(failsafe_events) > 0:
            errors.append("failsafe_events_observed")
        if len(blind_forward_samples) > 0:
            errors.append("blind_forward_events_observed")
        if not (0.95 <= float(follow_distance_m) <= 1.05):
            errors.append("desired_distance_not_1m")
        if target_distance_p50 is None or not (
            HUMAN_FOLLOW_V2_TARGET_DISTANCE_MIN_M
            <= float(target_distance_p50)
            <= HUMAN_FOLLOW_V2_TARGET_DISTANCE_MAX_M
        ):
            errors.append("human_follow_v2_target_distance_p50_gate_failed")
        if (
            strict_target_distance_error_p50 is None
            or float(strict_target_distance_error_p50) > HUMAN_FOLLOW_V2_TARGET_DISTANCE_ERROR_P50_MAX_M
        ):
            errors.append("human_follow_v2_target_distance_error_p50_gate_failed")
        if (
            strict_target_distance_error_p90 is None
            or float(strict_target_distance_error_p90) > HUMAN_FOLLOW_V2_TARGET_DISTANCE_ERROR_P90_MAX_M
        ):
            errors.append("human_follow_v2_target_distance_error_p90_gate_failed")
        if not human_follow_v2_bearing_sample_count_ok:
            errors.append("human_follow_v2_bearing_stable_sample_count_failed")
        if (
            bearing_error_p50 is None
            or float(bearing_error_p50) > HUMAN_FOLLOW_V2_BEARING_ERROR_P50_MAX_DEG
        ):
            errors.append("human_follow_v2_bearing_error_p50_gate_failed")
        if (
            bearing_error_p90 is None
            or float(bearing_error_p90) > HUMAN_FOLLOW_V2_BEARING_ERROR_P90_MAX_DEG
        ):
            errors.append("human_follow_v2_bearing_error_p90_gate_failed")
        if (
            angular_velocity_fresh_camera_max is not None
            and float(angular_velocity_fresh_camera_max)
            > float(motion_quality_tuning["fresh_target_omega_max_rad_s"])
        ):
            errors.append("human_follow_v2_fresh_target_omega_gate_failed")
        if (
            angular_velocity_p90 is not None
            and float(angular_velocity_p90) > float(motion_quality_tuning["omega_p90_max_rad_s"])
        ):
            errors.append("human_follow_v2_omega_p90_gate_failed")
        if not human_follow_v2_strict_ok:
            errors.append("human_follow_v2_strict_gate_failed")
    target_search_stationary_pwm_nonzero_count = 0
    for sample in search_arc_samples:
        left_ref = abs(_safe_float(sample.get("track_left_mps"), 0.0))
        right_ref = abs(_safe_float(sample.get("track_right_mps"), 0.0))
        if left_ref <= 1e-6 and right_ref >= 0.004 and abs(_safe_float(sample.get("pwm_left"), 0.0)) > 0.02:
            target_search_stationary_pwm_nonzero_count += 1
        elif right_ref <= 1e-6 and left_ref >= 0.004 and abs(_safe_float(sample.get("pwm_right"), 0.0)) > 0.02:
            target_search_stationary_pwm_nonzero_count += 1

    target_search_yaw_abs_delta_deg = 0.0
    target_search_cmd_yaw_abs_delta_deg = 0.0
    for prev, cur in zip(samples, samples[1:]):
        if str(prev.get("room_cruise_phase") or "") not in search_phase_names:
            continue
        dt_s = max(0.0, _safe_float(cur.get("elapsed_s"), 0.0) - _safe_float(prev.get("elapsed_s"), 0.0))
        target_search_yaw_abs_delta_deg += abs(
            _angle_delta_deg(
                _safe_float(cur.get("pose_theta_deg"), 0.0),
                _safe_float(prev.get("pose_theta_deg"), 0.0),
            )
        )
        target_search_cmd_yaw_abs_delta_deg += abs(_safe_float(prev.get("cmd_angular_dps"), 0.0)) * dt_s
    diagnostic_classification = {
        "controller_motion_quality": (
            "ok"
            if bool(
                motion_expected_executed_ok
                and command_delta_ok
                and len(direct_motor_bypass_samples) == 0
                and (camera_target_response_motion_ok or not localization_block_ok)
            )
            else "problem"
        ),
        "camera_target_loss": (
            "problem"
            if bool(
                float(target_lost_ratio) > HUMAN_FOLLOW_V2_MAX_SEARCH_RATIO
                or float(camera_target_visible_ratio) < HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO
                or float(usable_camera_ratio) < HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO
                or float(stale_camera_ratio) > HUMAN_FOLLOW_V2_MAX_SEARCH_RATIO
            )
            else "ok"
        ),
        "localization_block": (
            "problem"
            if bool(not localization_block_ok)
            else "not_dominant"
        ),
        "test_classification": (
            "follow_state_was_misleading"
            if bool(
                (
                    float(follow_state_ratio) >= HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO
                    or float(camera_target_visible_ratio) >= HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO
                )
                and float(active_follow_ratio) < HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO
            )
            else "ok"
        ),
    }
    result = {
        "status": "PASS" if not errors else "FAIL",
        "test_name": str(args.test_name),
        "duration_s": float(args.duration_s),
        "speed_scale": float(speed_scale),
        "follow_distance_m": float(follow_distance_m),
        "control_mode_target": str(control_mode_result.get("target") or _normalize_control_mode(getattr(args, "control_mode", "")) or ""),
        "control_mode_original": str(control_mode_result.get("original") or ""),
        "control_mode_applied": str(control_mode_result.get("applied") or _normalize_control_mode(_read_json(STATUS_PATH).get("control_mode")) or ""),
        "control_mode_ok": bool(control_mode_result.get("ok", True) if control_mode_result else True),
        "search_pivot_omega_target_rad_s": (
            None if search_pivot_omega_rad_s is None else round(float(search_pivot_omega_rad_s), 6)
        ),
        "search_pivot_omega_initial_rad_s": (
            None if initial_search_pivot_omega_rad_s is None else round(float(initial_search_pivot_omega_rad_s), 6)
        ),
        "search_pivot_omega_observed_p50_rad_s": _percentile(target_search_tuning_omega_samples, 0.50),
        "motion_quality_tuning": dict(motion_quality_tuning),
        "follow_accept_target_distance_min_m": float(follow_acceptance["min_m"]),
        "follow_accept_target_distance_max_m": float(follow_acceptance["max_m"]),
        "follow_accept_target_distance_error_p90_m": float(follow_acceptance["error_p90_m"]),
        "pre_follow_front_clearance_m": pre_follow_front_clearance_m,
        "sample_count": int(len(samples)),
        "behavior_duration_s": round(float(behavior_duration_s), 3),
        "follow_state_duration_s": round(float(follow_state_duration_s), 3),
        "follow_state_ratio": round(float(follow_state_ratio), 4),
        "active_follow_duration_s": round(float(active_follow_duration_s), 3),
        "active_follow_ratio": round(float(active_follow_ratio), 4),
        "target_search_duration_s": round(float(target_search_duration_s), 3),
        "target_search_ratio": round(float(target_search_ratio), 4),
        "target_lost_duration_s": round(float(target_lost_duration_s), 3),
        "target_lost_ratio": round(float(target_lost_ratio), 4),
        "camera_target_visible_duration_s": round(float(camera_target_visible_duration_s), 3),
        "camera_target_visible_ratio": round(float(camera_target_visible_ratio), 4),
        "camera_target_source_duration_s": round(float(camera_target_source_duration_s), 3),
        "camera_target_source_ratio": round(float(camera_target_source_ratio), 4),
        "usable_camera_duration_s": round(float(usable_camera_duration_s), 3),
        "usable_camera_ratio": round(float(usable_camera_ratio), 4),
        "stale_camera_duration_s": round(float(stale_camera_duration_s), 3),
        "stale_camera_ratio": round(float(stale_camera_ratio), 4),
        "search_pivot_duration_s": round(float(search_pivot_duration_s), 3),
        "search_pivot_ratio": round(float(search_pivot_ratio), 4),
        "pivot_without_fresh_camera_duration_s": round(float(pivot_without_fresh_camera_duration_s), 3),
        "pivot_without_fresh_camera_ratio": round(float(pivot_without_fresh_camera_ratio), 4),
        "longest_search_or_pivot_s": round(float(longest_search_or_pivot_s), 3),
        "relock_stable_follow_durations_s": list(relock_stable_follow_durations_s),
        "relock_stable_follow_min_s": _percentile(relock_stable_follow_durations_s, 0.0),
        "relock_stable_follow_p50_s": _percentile(relock_stable_follow_durations_s, 0.50),
        "relock_stable_follow_max_s": _percentile(relock_stable_follow_durations_s, 1.0),
        "search_follow_search_cycles": int(search_follow_search_cycles),
        "search_follow_search_cycles_per_min": float(search_follow_search_cycles_per_min),
        "active_follow_ratio_min": float(HUMAN_FOLLOW_V2_MIN_ACTIVE_FOLLOW_RATIO),
        "target_search_ratio_max": float(HUMAN_FOLLOW_V2_MAX_SEARCH_RATIO),
        "longest_search_or_pivot_max_s": float(HUMAN_FOLLOW_V2_MAX_CONTIGUOUS_SEARCH_PIVOT_S),
        "localization_motion_blocked_duration_s": round(float(localization_motion_blocked_duration_s), 3),
        "localization_motion_blocked_time_ratio": round(float(localization_motion_blocked_time_ratio), 4),
        "localization_motion_blocked_ratio_max": float(HUMAN_FOLLOW_V2_MAX_LOCALIZATION_BLOCK_RATIO),
        "camera_response_required_duration_s": round(float(camera_response_required_duration_s), 3),
        "camera_response_required_motion_duration_s": round(float(camera_response_required_motion_duration_s), 3),
        "camera_response_required_no_motion_duration_s": round(
            float(camera_response_required_no_motion_duration_s),
            3,
        ),
        "camera_response_required_no_motion_longest_s": round(
            float(camera_response_required_no_motion_longest_s),
            3,
        ),
        "camera_response_required_motion_ratio": round(float(camera_response_required_motion_ratio), 4),
        "camera_response_required_motion_ratio_min": float(HUMAN_FOLLOW_V2_MIN_RESPONSE_MOTION_RATIO),
        "camera_response_required_no_motion_max_s": float(HUMAN_FOLLOW_V2_MAX_RESPONSE_NO_MOTION_S),
        "camera_response_bearing_required_deg": float(HUMAN_FOLLOW_V2_RESPONSE_BEARING_REQUIRED_DEG),
        "camera_response_distance_error_required_m": float(
            HUMAN_FOLLOW_V2_RESPONSE_DISTANCE_ERROR_REQUIRED_M
        ),
        "actual_follow_ratio_ok": bool(actual_follow_ratio_ok),
        "target_search_ratio_ok": bool(target_search_ratio_ok),
        "search_pivot_duration_ok": bool(search_pivot_duration_ok),
        "localization_block_ok": bool(localization_block_ok),
        "camera_target_response_motion_ok": bool(camera_target_response_motion_ok),
        "human_follow_v2_behavior_balance_ok": bool(human_follow_v2_behavior_balance_ok),
        "diagnostic_classification": dict(diagnostic_classification),
        "strict_v2": bool(getattr(args, "strict_v2", False)),
        "active_route": str(active_route),
        "active_route_counts": dict(active_route_counts),
        "human_follow_v2_sample_count": int(len(human_follow_v2_samples)),
        "human_follow_v2_route_ok": bool(human_follow_v2_route_ok),
        "human_follow_v2_strict_ok": bool(human_follow_v2_strict_ok),
        "robotics_safety_ok": bool(robotics_safety_ok),
        "command_path_integrity_ok": bool(command_path_integrity_ok),
        "motion_continuity_ok": bool(motion_continuity_ok),
        "target_alignment_ok": bool(target_alignment_ok),
        "distance_behavior_ok": bool(distance_behavior_ok),
        "target_state_ok": bool(target_state_ok),
        "relock_stability_ok": bool(relock_stability_ok),
        "room_cruise_similarity_ok": bool(room_cruise_similarity_ok),
        "repeatability_ok": bool(repeatability_ok),
        "sample_coverage_ok": bool(sample_coverage_ok),
        "used_legacy_generic_planner": bool(len(legacy_generic_planner_samples) > 0),
        "used_legacy_generic_planner_sample_count": int(len(legacy_generic_planner_samples)),
        "legacy_planner_fallback_count": int(len(legacy_generic_planner_samples)),
        "direct_motor_bypass": bool(len(direct_motor_bypass_samples) > 0),
        "direct_motor_bypass_sample_count": int(len(direct_motor_bypass_samples)),
        "direct_track_reference_samples": int(len(direct_track_reference_samples)),
        "direct_track_reference_sample_count": int(len(direct_track_reference_samples)),
        "emergency_stop_events": int(emergency_stop_events),
        "failsafe_events": int(failsafe_events),
        "blind_forward_events": int(len(blind_forward_samples)),
        "camera_target_sample_count": int(len(target_samples)),
        "camera_search_sample_count": int(len(search_samples)),
        "room_cruise_chain_sample_count": int(len(cruise_samples)),
        "target_camera_state_counts": camera_state_counts,
        "target_camera_detector_counts": camera_detector_counts,
        "target_camera_distance_source_counts": camera_distance_source_counts,
        "target_camera_distance_sample_count": int(len(camera_distance_estimate_samples)),
        "target_camera_distance_estimate_p50_m": _percentile(camera_distance_estimate_samples, 0.50),
        "target_camera_distance_estimate_p90_m": _percentile(camera_distance_estimate_samples, 0.90),
        "target_camera_distance_used_p50_m": _percentile(camera_distance_used_samples, 0.50),
        "target_camera_luma_p50": _percentile(camera_luma_samples, 0.50),
        "target_camera_contrast_p50": _percentile(camera_contrast_samples, 0.50),
        "target_camera_dark_sample_count": int(len(camera_dark_samples)),
        "target_camera_bright_sample_count": int(len(camera_bright_samples)),
        "target_camera_low_contrast_sample_count": int(len(camera_low_contrast_samples)),
        "target_camera_stream_seed_sample_count": int(len(camera_stream_seed_samples)),
        "target_camera_onnx_relock_sample_count": int(len(camera_onnx_relock_samples)),
        "target_camera_detector_throttled_sample_count": int(len(camera_detector_throttled_samples)),
        "target_camera_capture_pending_sample_count": int(len(camera_capture_pending_samples)),
        "localization_gate_mode_counts": localization_gate_mode_counts,
        "localization_gate_apply_reason_counts": localization_gate_apply_reason_counts,
        "localization_gate_reason_counts": localization_gate_reason_counts,
        "localization_gate_hard_stop_sample_count": int(len(localization_gate_hard_stop_samples)),
        "localization_motion_blocked_ratio": round(float(localization_motion_blocked_ratio), 4),
        "localization_motion_blocked_dominant": bool(localization_motion_blocked_dominant),
        "localization_gate_idle_resume_sample_count": int(len(localization_gate_idle_resume_samples)),
        "localization_gate_idle_resume_bridge_sample_count": int(
            len(localization_gate_idle_resume_bridge_samples)
        ),
        "localization_gate_current_apply_recovery_sample_count": int(
            len(localization_gate_current_apply_recovery_samples)
        ),
        "localization_gate_ekf_gap_p50_s": _percentile(localization_gate_gap_samples, 0.50),
        "localization_gate_ekf_gap_p90_s": _percentile(localization_gate_gap_samples, 0.90),
        "localization_gate_ekf_gap_max_s": _percentile(localization_gate_gap_samples, 1.00),
        "lidar_raw_scan_age_p90_s": _percentile(lidar_raw_scan_age_samples, 0.90),
        "lidar_raw_scan_age_max_s": _percentile(lidar_raw_scan_age_samples, 1.00),
        "lidar_matcher_latency_p90_ms": _percentile(lidar_matcher_latency_samples, 0.90),
        "lidar_matcher_latency_max_ms": _percentile(lidar_matcher_latency_samples, 1.00),
        "follow_actual_distance_p50_m": _percentile(follow_distance_samples, 0.50),
        "follow_actual_distance_p90_m": _percentile(follow_distance_samples, 0.90),
        "target_distance_p50_m": _percentile(follow_distance_samples, 0.50),
        "target_distance_p90_m": _percentile(follow_distance_samples, 0.90),
        "target_distance_error_p50_m": _percentile(target_distance_error_samples, 0.50),
        "target_distance_error_p90_m": target_distance_error_p90,
        "strict_target_distance_error_p50_m": strict_target_distance_error_p50,
        "strict_target_distance_error_p90_m": strict_target_distance_error_p90,
        "settled_target_distance_start_s": None if settled_distance_start_s is None else round(float(settled_distance_start_s), 3),
        "settled_target_distance_sample_count": int(len(settled_target_distance_error_samples)),
        "target_angle_error_p50_deg": target_angle_error_p50,
        "target_angle_error_p90_deg": target_angle_error_p90,
        "fresh_bearing_error_p50_deg": fresh_bearing_error_p50,
        "fresh_bearing_error_p90_deg": fresh_bearing_error_p90,
        "bearing_error_p50_deg": bearing_error_p50,
        "bearing_error_p90_deg": bearing_error_p90,
        "bearing_error_sample_count": int(len(human_follow_v2_bearing_samples)),
        "bearing_error_min_sample_count": int(human_follow_v2_bearing_min_samples),
        "bearing_error_sample_count_ok": bool(human_follow_v2_bearing_sample_count_ok),
        "bearing_error_sample_fraction": round(
            float(len(human_follow_v2_bearing_samples)) / max(1, len(fresh_camera_target_samples)),
            4,
        ),
        "bearing_error_approach_stable_max_deg": float(HUMAN_FOLLOW_V2_APPROACH_STABLE_BEARING_MAX_DEG),
        "bearing_error_stable_phase_counts": dict(human_follow_v2_bearing_phase_counts),
        "angular_velocity_fresh_camera_max_rad_s": angular_velocity_fresh_camera_max,
        "angular_velocity_p90_rad_s": angular_velocity_p90,
        "motion_expected_executed_sample_count": int(len(cruise_samples)),
        "motion_expected_executed_v_error_p50_mps": motion_expected_executed_v_error_p50,
        "motion_expected_executed_v_error_p90_mps": motion_expected_executed_v_error_p90,
        "motion_expected_executed_omega_error_p50_rad_s": motion_expected_executed_omega_error_p50,
        "motion_expected_executed_omega_error_p90_rad_s": motion_expected_executed_omega_error_p90,
        "motion_expected_executed_track_error_p50_mps": motion_expected_executed_track_error_p50,
        "motion_expected_executed_track_error_p90_mps": motion_expected_executed_track_error_p90,
        "motion_expected_executed_ok": bool(motion_expected_executed_ok),
        "command_delta_v_p90_mps": command_delta_v_p90,
        "command_delta_omega_p90_rad_s": command_delta_omega_p90,
        "command_delta_omega_max_rad_s": command_delta_omega_max,
        "target_active_zero_gap_longest_run_s": target_active_zero_gap_longest_run_s,
        "post_lock_candidate_hold_longest_run_s": post_lock_candidate_hold_longest_run_s,
        "startup_first_camera_target_elapsed_s": first_camera_target_elapsed_s,
        "startup_first_camera_active_elapsed_s": first_camera_active_elapsed_s,
        "startup_first_camera_search_elapsed_s": first_camera_search_elapsed_s,
        "startup_lock_ok": bool(startup_lock_ok),
        "post_lock_candidate_hold_ok": bool(post_lock_candidate_hold_ok),
        "command_delta_ok": bool(command_delta_ok),
        "target_active_zero_gap_ok": bool(target_active_zero_gap_ok),
        "motion_quality_status": "PASS" if motion_quality_ok else "FAIL",
        "motion_quality_ok": bool(motion_quality_ok),
        "target_lost_gap_max_s": target_lost_gap_max_s,
        "wall_stick_sample_count": int(len(wall_stick_samples)),
        "wall_stick_events": int(len(wall_stick_samples)),
        "wall_stick_longest_run_s": wall_stick_longest_run_s,
        "camera_turn_wrong_side_sample_count": int(len(camera_turn_wrong_side_samples)),
        "camera_turn_wrong_side_longest_run_s": round(float(longest_camera_turn_wrong_side_run) / sample_rate_hz, 3),
        "camera_turn_side_blocked_sample_count": int(len(camera_turn_side_blocked_samples)),
        "camera_angle_side_flip_count": int(camera_angle_side_flip_count),
        "expected_omega_side_flip_count": int(expected_omega_side_flip_count),
        "camera_target_longest_run_s": round(float(longest_camera_target_run) / sample_rate_hz, 3),
        "camera_search_longest_run_s": round(float(longest_search_run) / sample_rate_hz, 3),
        "final_window_s": float(FINAL_TARGET_WINDOW_S),
        "final_camera_active_sample_count": int(len(final_camera_active_samples)),
        "final_localization_hard_stop_sample_count": int(len(final_localization_hard_stop_samples)),
        "final_target_distance_p50_m": final_target_distance_p50,
        "final_target_distance_error_p50_m": final_target_distance_error_p50,
        "forward_command_longest_run_s": round(float(longest_forward_run) / sample_rate_hz, 3),
        "reverse_command_longest_run_s": round(float(longest_reverse_run) / sample_rate_hz, 3),
        "translation_command_longest_run_s": round(float(longest_translation_run) / sample_rate_hz, 3),
        "target_translation_sample_count": int(len(target_translation_samples)),
        "target_forward_sample_count": int(len(target_forward_samples)),
        "target_reverse_sample_count": int(len(target_reverse_samples)),
        "camera_response_required_sample_count": int(len(camera_response_required_samples)),
        "camera_response_required_motion_sample_count": int(len(camera_response_required_motion_samples)),
        "non_target_translation_sample_count": int(len(non_target_translation_samples)),
        "v2_local_navigation_sample_count": int(len(v2_local_navigation_samples)),
        "target_moving_direct_bypass_sample_count": int(len(target_moving_direct_bypass_samples)),
        "wrong_reverse_sample_count": int(len(wrong_reverse_samples)),
        "reverse_without_clearance_sample_count": int(len(reverse_without_clearance_samples)),
        "front_lidar_camera_arbitrated_sample_count": int(len(front_arbitrated_samples)),
        "hold_like_target_sample_count": int(len(hold_like_target_samples)),
        "human_follow_criteria": {
            "status": human_follow_criteria_status,
            "stable_camera_detection_ok": bool(stable_detection_ok),
            "robotics_safety_ok": bool(robotics_safety_ok),
            "command_path_integrity_ok": bool(command_path_integrity_ok),
            "motion_continuity_ok": bool(motion_continuity_ok),
            "target_alignment_ok": bool(target_alignment_ok),
            "distance_behavior_ok": bool(distance_behavior_ok),
            "target_state_ok": bool(target_state_ok),
            "relock_stability_ok": bool(relock_stability_ok),
            "behavior_balance_ok": bool(human_follow_v2_behavior_balance_ok),
            "actual_follow_ratio_ok": bool(actual_follow_ratio_ok),
            "target_search_ratio_ok": bool(target_search_ratio_ok),
            "search_pivot_duration_ok": bool(search_pivot_duration_ok),
            "localization_block_ok": bool(localization_block_ok),
            "camera_target_response_motion_ok": bool(camera_target_response_motion_ok),
            "room_cruise_similarity_ok": bool(room_cruise_similarity_ok),
            "repeatability_ok": bool(repeatability_ok),
            "motion_quality_ok": bool(motion_quality_ok),
            "distance_control_motion_to_camera_target_ok": bool(distance_control_motion_ok),
            "v2_local_navigation_ok": bool(v2_local_navigation_ok),
            "reverse_clearance_ok": bool(reverse_clearance_ok),
            "target_search_one_track_ok": bool(target_search_one_track_cmd_count > 0),
            "target_search_in_place_ok": bool(target_search_in_place_ok),
            "camera_detection_motion_only_ok": bool(camera_detection_motion_only_ok),
            "camera_distance_estimate_ok": bool(camera_distance_ok),
            "target_distance_ok": bool(target_distance_ok),
            "target_angle_ok": bool(target_angle_ok),
            "target_lost_gap_ok": bool(target_lost_gap_ok),
            "motion_expected_executed_ok": bool(motion_expected_executed_ok),
            "wall_stick_ok": bool(wall_stick_ok),
            "camera_turn_wrong_side_ok": bool(camera_turn_wrong_side_ok),
            "final_camera_target_ok": bool(final_camera_target_ok),
            "final_target_distance_ok": bool(final_target_distance_ok),
            "emergency_zero_ok": bool(not _new_emergency_observed(samples, initial_emergency_count)),
            "active_route_human_follow_v2_ok": bool(human_follow_v2_route_ok),
            "legacy_generic_planner_ok": bool(len(legacy_generic_planner_samples) == 0),
            "direct_motor_bypass_ok": bool(len(direct_motor_bypass_samples) == 0),
            "direct_track_reference_ok": bool(len(direct_track_reference_samples) == 0),
            "failsafe_zero_ok": bool(int(failsafe_events) == 0),
            "blind_forward_zero_ok": bool(len(blind_forward_samples) == 0),
            "bearing_error_sample_count_ok": bool(human_follow_v2_bearing_sample_count_ok),
            "strict_v2_ok": bool(human_follow_v2_strict_ok),
        },
        "room_cruise_phase_counts": cruise_phase_counts,
        "target_search_state_counts": search_state_counts,
        "motion_command_sample_count": int(len(moving_samples)),
        "camera_detection_active_sample_count": int(len(camera_detection_active_samples)),
        "motion_without_camera_detection_sample_count": int(len(motion_without_camera_detection_samples)),
        "camera_detection_motion_suppressed_count": int(camera_detection_motion_suppressed_count),
        "camera_detection_clearance_retreat_count": int(camera_detection_clearance_retreat_count),
        "camera_detection_reacquire_rotate_count": int(camera_detection_reacquire_rotate_count),
        "camera_simple_follow_sample_count": int(camera_simple_follow_count),
        "camera_simple_forward_gate_blocked_count": int(camera_simple_forward_gate_blocked_count),
        "camera_simple_retreat_gate_blocked_count": int(camera_simple_retreat_gate_blocked_count),
        "camera_simple_turn_track_p50_mps": _percentile(camera_simple_turn_track_samples, 0.50),
        "camera_simple_turn_track_p90_mps": _percentile(camera_simple_turn_track_samples, 0.90),
        "camera_simple_heading_forward_scale_p50": _percentile(camera_simple_forward_scale_samples, 0.50),
        "camera_simple_heading_forward_scale_p90": _percentile(camera_simple_forward_scale_samples, 0.90),
        "target_search_arc_sample_count": int(len(search_arc_samples)),
        "target_search_rotate_sample_count": int(len(search_rotate_samples)),
        "target_search_in_place_ok": bool(target_search_in_place_ok),
        "target_search_one_track_cmd_count": int(target_search_one_track_cmd_count),
        "target_search_in_place_rotate_cmd_count": int(target_search_in_place_rotate_cmd_count),
        "target_search_forward_cmd_count": int(target_search_forward_cmd_count),
        "target_search_stationary_pwm_nonzero_count": int(target_search_stationary_pwm_nonzero_count),
        "target_search_cmd_omega_abs_p50_rad_s": _percentile(target_search_cmd_omega_abs_samples, 0.50),
        "target_search_cmd_omega_abs_p90_rad_s": _percentile(target_search_cmd_omega_abs_samples, 0.90),
        "target_search_cmd_omega_abs_max_rad_s": _percentile(target_search_cmd_omega_abs_samples, 1.00),
        "target_search_cmd_angular_abs_p50_dps": _percentile(target_search_cmd_angular_abs_dps_samples, 0.50),
        "target_search_cmd_angular_abs_p90_dps": _percentile(target_search_cmd_angular_abs_dps_samples, 0.90),
        "target_search_cmd_angular_abs_max_dps": _percentile(target_search_cmd_angular_abs_dps_samples, 1.00),
        "target_search_yaw_abs_delta_deg": round(float(target_search_yaw_abs_delta_deg), 3),
        "target_search_cmd_yaw_abs_delta_deg": round(float(target_search_cmd_yaw_abs_delta_deg), 3),
        "camera_open_failed_sample_count": int(len(camera_open_failed_samples)),
        "target_camera_failed_session_max": int(target_camera_failed_session_max),
        "errors": errors,
        "command_results": command_results,
        "artifact_paths": {
            "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
            "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(HISTORY_PATH.relative_to(PROJECT_ROOT)),
        },
    }
    summary = {
        "status": result["status"],
        "test_name": result["test_name"],
        "speed_scale": result["speed_scale"],
        "follow_distance_m": result["follow_distance_m"],
        "control_mode_target": result["control_mode_target"],
        "control_mode_original": result["control_mode_original"],
        "control_mode_applied": result["control_mode_applied"],
        "control_mode_ok": result["control_mode_ok"],
        "search_pivot_omega_target_rad_s": result["search_pivot_omega_target_rad_s"],
        "search_pivot_omega_initial_rad_s": result["search_pivot_omega_initial_rad_s"],
        "search_pivot_omega_observed_p50_rad_s": result["search_pivot_omega_observed_p50_rad_s"],
        "motion_quality_tuning": dict(result["motion_quality_tuning"]),
        "follow_accept_target_distance_min_m": result["follow_accept_target_distance_min_m"],
        "follow_accept_target_distance_max_m": result["follow_accept_target_distance_max_m"],
        "follow_accept_target_distance_error_p90_m": result["follow_accept_target_distance_error_p90_m"],
        "pre_follow_front_clearance_m": result["pre_follow_front_clearance_m"],
        "sample_count": result["sample_count"],
        "behavior_duration_s": result["behavior_duration_s"],
        "follow_state_duration_s": result["follow_state_duration_s"],
        "follow_state_ratio": result["follow_state_ratio"],
        "active_follow_duration_s": result["active_follow_duration_s"],
        "active_follow_ratio": result["active_follow_ratio"],
        "target_search_duration_s": result["target_search_duration_s"],
        "target_search_ratio": result["target_search_ratio"],
        "target_lost_duration_s": result["target_lost_duration_s"],
        "target_lost_ratio": result["target_lost_ratio"],
        "camera_target_visible_duration_s": result["camera_target_visible_duration_s"],
        "camera_target_visible_ratio": result["camera_target_visible_ratio"],
        "camera_target_source_duration_s": result["camera_target_source_duration_s"],
        "camera_target_source_ratio": result["camera_target_source_ratio"],
        "usable_camera_duration_s": result["usable_camera_duration_s"],
        "usable_camera_ratio": result["usable_camera_ratio"],
        "stale_camera_duration_s": result["stale_camera_duration_s"],
        "stale_camera_ratio": result["stale_camera_ratio"],
        "search_pivot_duration_s": result["search_pivot_duration_s"],
        "search_pivot_ratio": result["search_pivot_ratio"],
        "pivot_without_fresh_camera_duration_s": result["pivot_without_fresh_camera_duration_s"],
        "pivot_without_fresh_camera_ratio": result["pivot_without_fresh_camera_ratio"],
        "longest_search_or_pivot_s": result["longest_search_or_pivot_s"],
        "relock_stable_follow_durations_s": list(result["relock_stable_follow_durations_s"]),
        "relock_stable_follow_min_s": result["relock_stable_follow_min_s"],
        "relock_stable_follow_p50_s": result["relock_stable_follow_p50_s"],
        "relock_stable_follow_max_s": result["relock_stable_follow_max_s"],
        "search_follow_search_cycles": result["search_follow_search_cycles"],
        "search_follow_search_cycles_per_min": result["search_follow_search_cycles_per_min"],
        "active_follow_ratio_min": result["active_follow_ratio_min"],
        "target_search_ratio_max": result["target_search_ratio_max"],
        "longest_search_or_pivot_max_s": result["longest_search_or_pivot_max_s"],
        "localization_motion_blocked_duration_s": result["localization_motion_blocked_duration_s"],
        "localization_motion_blocked_time_ratio": result["localization_motion_blocked_time_ratio"],
        "localization_motion_blocked_ratio_max": result["localization_motion_blocked_ratio_max"],
        "camera_response_required_duration_s": result["camera_response_required_duration_s"],
        "camera_response_required_motion_duration_s": result[
            "camera_response_required_motion_duration_s"
        ],
        "camera_response_required_no_motion_duration_s": result[
            "camera_response_required_no_motion_duration_s"
        ],
        "camera_response_required_no_motion_longest_s": result[
            "camera_response_required_no_motion_longest_s"
        ],
        "camera_response_required_motion_ratio": result["camera_response_required_motion_ratio"],
        "camera_response_required_motion_ratio_min": result[
            "camera_response_required_motion_ratio_min"
        ],
        "camera_response_required_no_motion_max_s": result[
            "camera_response_required_no_motion_max_s"
        ],
        "actual_follow_ratio_ok": result["actual_follow_ratio_ok"],
        "target_search_ratio_ok": result["target_search_ratio_ok"],
        "search_pivot_duration_ok": result["search_pivot_duration_ok"],
        "localization_block_ok": result["localization_block_ok"],
        "camera_target_response_motion_ok": result["camera_target_response_motion_ok"],
        "human_follow_v2_behavior_balance_ok": result["human_follow_v2_behavior_balance_ok"],
        "diagnostic_classification": dict(result["diagnostic_classification"]),
        "strict_v2": result["strict_v2"],
        "active_route": result["active_route"],
        "active_route_counts": dict(result["active_route_counts"]),
        "human_follow_v2_sample_count": result["human_follow_v2_sample_count"],
        "human_follow_v2_route_ok": result["human_follow_v2_route_ok"],
        "human_follow_v2_strict_ok": result["human_follow_v2_strict_ok"],
        "robotics_safety_ok": result["robotics_safety_ok"],
        "command_path_integrity_ok": result["command_path_integrity_ok"],
        "motion_continuity_ok": result["motion_continuity_ok"],
        "target_alignment_ok": result["target_alignment_ok"],
        "distance_behavior_ok": result["distance_behavior_ok"],
        "target_state_ok": result["target_state_ok"],
        "relock_stability_ok": result["relock_stability_ok"],
        "room_cruise_similarity_ok": result["room_cruise_similarity_ok"],
        "repeatability_ok": result["repeatability_ok"],
        "sample_coverage_ok": result["sample_coverage_ok"],
        "used_legacy_generic_planner": result["used_legacy_generic_planner"],
        "used_legacy_generic_planner_sample_count": result["used_legacy_generic_planner_sample_count"],
        "legacy_planner_fallback_count": result["legacy_planner_fallback_count"],
        "direct_motor_bypass": result["direct_motor_bypass"],
        "direct_motor_bypass_sample_count": result["direct_motor_bypass_sample_count"],
        "direct_track_reference_samples": result["direct_track_reference_samples"],
        "direct_track_reference_sample_count": result["direct_track_reference_sample_count"],
        "emergency_stop_events": result["emergency_stop_events"],
        "failsafe_events": result["failsafe_events"],
        "blind_forward_events": result["blind_forward_events"],
        "camera_target_sample_count": result["camera_target_sample_count"],
        "camera_search_sample_count": result["camera_search_sample_count"],
        "room_cruise_chain_sample_count": result["room_cruise_chain_sample_count"],
        "target_camera_state_counts": dict(camera_state_counts),
        "target_camera_detector_counts": dict(camera_detector_counts),
        "target_camera_distance_source_counts": dict(camera_distance_source_counts),
        "target_camera_distance_sample_count": result["target_camera_distance_sample_count"],
        "target_camera_distance_estimate_p50_m": result["target_camera_distance_estimate_p50_m"],
        "target_camera_distance_estimate_p90_m": result["target_camera_distance_estimate_p90_m"],
        "target_camera_distance_used_p50_m": result["target_camera_distance_used_p50_m"],
        "target_camera_luma_p50": result["target_camera_luma_p50"],
        "target_camera_contrast_p50": result["target_camera_contrast_p50"],
        "target_camera_dark_sample_count": result["target_camera_dark_sample_count"],
        "target_camera_bright_sample_count": result["target_camera_bright_sample_count"],
        "target_camera_low_contrast_sample_count": result["target_camera_low_contrast_sample_count"],
        "target_camera_stream_seed_sample_count": result["target_camera_stream_seed_sample_count"],
        "target_camera_onnx_relock_sample_count": result["target_camera_onnx_relock_sample_count"],
        "localization_gate_mode_counts": dict(localization_gate_mode_counts),
        "localization_gate_apply_reason_counts": dict(localization_gate_apply_reason_counts),
        "localization_gate_reason_counts": dict(localization_gate_reason_counts),
        "localization_gate_hard_stop_sample_count": result["localization_gate_hard_stop_sample_count"],
        "localization_motion_blocked_ratio": result["localization_motion_blocked_ratio"],
        "localization_motion_blocked_dominant": result["localization_motion_blocked_dominant"],
        "localization_gate_idle_resume_sample_count": result["localization_gate_idle_resume_sample_count"],
        "localization_gate_idle_resume_bridge_sample_count": result[
            "localization_gate_idle_resume_bridge_sample_count"
        ],
        "localization_gate_current_apply_recovery_sample_count": result[
            "localization_gate_current_apply_recovery_sample_count"
        ],
        "localization_gate_ekf_gap_p50_s": result["localization_gate_ekf_gap_p50_s"],
        "localization_gate_ekf_gap_p90_s": result["localization_gate_ekf_gap_p90_s"],
        "localization_gate_ekf_gap_max_s": result["localization_gate_ekf_gap_max_s"],
        "lidar_raw_scan_age_p90_s": result["lidar_raw_scan_age_p90_s"],
        "lidar_raw_scan_age_max_s": result["lidar_raw_scan_age_max_s"],
        "lidar_matcher_latency_p90_ms": result["lidar_matcher_latency_p90_ms"],
        "lidar_matcher_latency_max_ms": result["lidar_matcher_latency_max_ms"],
        "follow_actual_distance_p50_m": result["follow_actual_distance_p50_m"],
        "follow_actual_distance_p90_m": result["follow_actual_distance_p90_m"],
        "target_distance_p50_m": result["target_distance_p50_m"],
        "target_distance_p90_m": result["target_distance_p90_m"],
        "target_distance_error_p50_m": result["target_distance_error_p50_m"],
        "target_distance_error_p90_m": result["target_distance_error_p90_m"],
        "strict_target_distance_error_p50_m": result["strict_target_distance_error_p50_m"],
        "strict_target_distance_error_p90_m": result["strict_target_distance_error_p90_m"],
        "settled_target_distance_start_s": result["settled_target_distance_start_s"],
        "settled_target_distance_sample_count": result["settled_target_distance_sample_count"],
        "target_angle_error_p50_deg": result["target_angle_error_p50_deg"],
        "target_angle_error_p90_deg": result["target_angle_error_p90_deg"],
        "fresh_bearing_error_p50_deg": result["fresh_bearing_error_p50_deg"],
        "fresh_bearing_error_p90_deg": result["fresh_bearing_error_p90_deg"],
        "bearing_error_p50_deg": result["bearing_error_p50_deg"],
        "bearing_error_p90_deg": result["bearing_error_p90_deg"],
        "bearing_error_sample_count": result["bearing_error_sample_count"],
        "bearing_error_min_sample_count": result["bearing_error_min_sample_count"],
        "bearing_error_sample_count_ok": result["bearing_error_sample_count_ok"],
        "bearing_error_sample_fraction": result["bearing_error_sample_fraction"],
        "bearing_error_approach_stable_max_deg": result["bearing_error_approach_stable_max_deg"],
        "bearing_error_stable_phase_counts": dict(result["bearing_error_stable_phase_counts"]),
        "angular_velocity_fresh_camera_max_rad_s": result["angular_velocity_fresh_camera_max_rad_s"],
        "angular_velocity_p90_rad_s": result["angular_velocity_p90_rad_s"],
        "motion_expected_executed_sample_count": result["motion_expected_executed_sample_count"],
        "motion_expected_executed_v_error_p50_mps": result["motion_expected_executed_v_error_p50_mps"],
        "motion_expected_executed_v_error_p90_mps": result["motion_expected_executed_v_error_p90_mps"],
        "motion_expected_executed_omega_error_p50_rad_s": result["motion_expected_executed_omega_error_p50_rad_s"],
        "motion_expected_executed_omega_error_p90_rad_s": result["motion_expected_executed_omega_error_p90_rad_s"],
        "motion_expected_executed_track_error_p50_mps": result["motion_expected_executed_track_error_p50_mps"],
        "motion_expected_executed_track_error_p90_mps": result["motion_expected_executed_track_error_p90_mps"],
        "motion_expected_executed_ok": result["motion_expected_executed_ok"],
        "command_delta_v_p90_mps": result["command_delta_v_p90_mps"],
        "command_delta_omega_p90_rad_s": result["command_delta_omega_p90_rad_s"],
        "command_delta_omega_max_rad_s": result["command_delta_omega_max_rad_s"],
        "target_active_zero_gap_longest_run_s": result["target_active_zero_gap_longest_run_s"],
        "post_lock_candidate_hold_longest_run_s": result["post_lock_candidate_hold_longest_run_s"],
        "startup_first_camera_target_elapsed_s": result["startup_first_camera_target_elapsed_s"],
        "startup_first_camera_active_elapsed_s": result["startup_first_camera_active_elapsed_s"],
        "startup_first_camera_search_elapsed_s": result["startup_first_camera_search_elapsed_s"],
        "startup_lock_ok": result["startup_lock_ok"],
        "post_lock_candidate_hold_ok": result["post_lock_candidate_hold_ok"],
        "command_delta_ok": result["command_delta_ok"],
        "target_active_zero_gap_ok": result["target_active_zero_gap_ok"],
        "target_lost_gap_max_s": result["target_lost_gap_max_s"],
        "wall_stick_sample_count": result["wall_stick_sample_count"],
        "wall_stick_events": result["wall_stick_events"],
        "wall_stick_longest_run_s": result["wall_stick_longest_run_s"],
        "camera_turn_wrong_side_sample_count": result["camera_turn_wrong_side_sample_count"],
        "camera_turn_wrong_side_longest_run_s": result["camera_turn_wrong_side_longest_run_s"],
        "camera_turn_side_blocked_sample_count": result["camera_turn_side_blocked_sample_count"],
        "camera_angle_side_flip_count": result["camera_angle_side_flip_count"],
        "expected_omega_side_flip_count": result["expected_omega_side_flip_count"],
        "camera_target_longest_run_s": result["camera_target_longest_run_s"],
        "camera_search_longest_run_s": result["camera_search_longest_run_s"],
        "target_camera_detector_throttled_sample_count": result["target_camera_detector_throttled_sample_count"],
        "target_camera_capture_pending_sample_count": result["target_camera_capture_pending_sample_count"],
        "final_window_s": result["final_window_s"],
        "final_camera_active_sample_count": result["final_camera_active_sample_count"],
        "final_localization_hard_stop_sample_count": result["final_localization_hard_stop_sample_count"],
        "final_target_distance_p50_m": result["final_target_distance_p50_m"],
        "final_target_distance_error_p50_m": result["final_target_distance_error_p50_m"],
        "forward_command_longest_run_s": result["forward_command_longest_run_s"],
        "reverse_command_longest_run_s": result["reverse_command_longest_run_s"],
        "translation_command_longest_run_s": result["translation_command_longest_run_s"],
        "target_translation_sample_count": result["target_translation_sample_count"],
        "target_forward_sample_count": result["target_forward_sample_count"],
        "target_reverse_sample_count": result["target_reverse_sample_count"],
        "camera_response_required_sample_count": result["camera_response_required_sample_count"],
        "camera_response_required_motion_sample_count": result[
            "camera_response_required_motion_sample_count"
        ],
        "non_target_translation_sample_count": result["non_target_translation_sample_count"],
        "v2_local_navigation_sample_count": result["v2_local_navigation_sample_count"],
        "wrong_reverse_sample_count": result["wrong_reverse_sample_count"],
        "target_moving_direct_bypass_sample_count": result["target_moving_direct_bypass_sample_count"],
        "reverse_without_clearance_sample_count": result["reverse_without_clearance_sample_count"],
        "front_lidar_camera_arbitrated_sample_count": result["front_lidar_camera_arbitrated_sample_count"],
        "hold_like_target_sample_count": result["hold_like_target_sample_count"],
        "human_follow_criteria": dict(result["human_follow_criteria"]),
        "room_cruise_phase_counts": dict(cruise_phase_counts),
        "target_search_state_counts": dict(search_state_counts),
        "motion_command_sample_count": int(len(moving_samples)),
        "camera_detection_active_sample_count": result["camera_detection_active_sample_count"],
        "motion_without_camera_detection_sample_count": result["motion_without_camera_detection_sample_count"],
        "camera_detection_motion_suppressed_count": result["camera_detection_motion_suppressed_count"],
        "camera_detection_clearance_retreat_count": result["camera_detection_clearance_retreat_count"],
        "camera_detection_reacquire_rotate_count": result["camera_detection_reacquire_rotate_count"],
        "camera_simple_follow_sample_count": result["camera_simple_follow_sample_count"],
        "camera_simple_forward_gate_blocked_count": result["camera_simple_forward_gate_blocked_count"],
        "camera_simple_retreat_gate_blocked_count": result["camera_simple_retreat_gate_blocked_count"],
        "camera_simple_turn_track_p50_mps": result["camera_simple_turn_track_p50_mps"],
        "camera_simple_turn_track_p90_mps": result["camera_simple_turn_track_p90_mps"],
        "camera_simple_heading_forward_scale_p50": result["camera_simple_heading_forward_scale_p50"],
        "camera_simple_heading_forward_scale_p90": result["camera_simple_heading_forward_scale_p90"],
        "target_search_arc_sample_count": result["target_search_arc_sample_count"],
        "target_search_rotate_sample_count": result["target_search_rotate_sample_count"],
        "target_search_in_place_ok": result["target_search_in_place_ok"],
        "target_search_one_track_cmd_count": result["target_search_one_track_cmd_count"],
        "target_search_in_place_rotate_cmd_count": result["target_search_in_place_rotate_cmd_count"],
        "target_search_forward_cmd_count": result["target_search_forward_cmd_count"],
        "target_search_stationary_pwm_nonzero_count": result["target_search_stationary_pwm_nonzero_count"],
        "target_search_cmd_omega_abs_p50_rad_s": result["target_search_cmd_omega_abs_p50_rad_s"],
        "target_search_cmd_omega_abs_p90_rad_s": result["target_search_cmd_omega_abs_p90_rad_s"],
        "target_search_cmd_omega_abs_max_rad_s": result["target_search_cmd_omega_abs_max_rad_s"],
        "target_search_cmd_angular_abs_p50_dps": result["target_search_cmd_angular_abs_p50_dps"],
        "target_search_cmd_angular_abs_p90_dps": result["target_search_cmd_angular_abs_p90_dps"],
        "target_search_cmd_angular_abs_max_dps": result["target_search_cmd_angular_abs_max_dps"],
        "target_search_yaw_abs_delta_deg": result["target_search_yaw_abs_delta_deg"],
        "target_search_cmd_yaw_abs_delta_deg": result["target_search_cmd_yaw_abs_delta_deg"],
        "camera_open_failed_sample_count": result["camera_open_failed_sample_count"],
        "target_camera_failed_session_max": result["target_camera_failed_session_max"],
        "errors": list(errors),
    }
    _append_jsonl(HISTORY_PATH, samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, summary)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Bounded live person-follow smoke through camera detection.")
    parser.add_argument("--test-name", default="person_follow_camera_live")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--sample-rate-hz", type=float, default=5.0)
    parser.add_argument("--speed-scale", type=float, default=0.8)
    parser.add_argument("--follow-distance-m", type=float, default=1.0)
    parser.add_argument("--search-pivot-omega-rad-s", type=float, default=None)
    parser.add_argument("--control-mode", choices=(CANONICAL_CONTROL_MODE,), default=CANONICAL_CONTROL_MODE)
    parser.add_argument("--fresh-target-omega-max-rad-s", type=float, default=None)
    parser.add_argument("--omega-p90-max-rad-s", type=float, default=None)
    parser.add_argument("--command-delta-omega-p90-max-rad-s", type=float, default=None)
    parser.add_argument("--command-delta-omega-max-rad-s", type=float, default=None)
    parser.add_argument("--status-timeout-s", type=float, default=5.0)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--strict-v2", action="store_true")
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    result = run(args)
    print(json.dumps(result if not args.compact else {"status": result["status"], "errors": result["errors"]}, ensure_ascii=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
