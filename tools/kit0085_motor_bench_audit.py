#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Raised-wheel KIT0085 direction and encoder sanity audit."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded
from tools.lidar_1m_step import (
    DEFAULT_MOTION_SOURCE,
    STATUS_PATH,
    _append_command,
    _read_json,
    _send_command_checked,
    _wait_for_status,
    _wait_until_stopped,
)


LATEST_RESULT_PATH = test_artifacts_dir() / "latest_kit0085_motor_bench_audit.json"
CONTROL_MODE_PATH = PROJECT_ROOT / "conf" / "control_mode.json"
LATEST_MANUAL_ENCODER_PATH = latest_artifact_path("latest_kit0085_encoder_manual.json")
EXPECTED_MODEL = "DFROBOT_KIT0085_28PA51G"
EXPECTED_PINS = {"left": (23, 24), "right": (25, 16)}
CANONICAL_CONTROL_MODE = "UNIFIED"
COMMAND_MODES = ("track_velocity", "twist")
MOTION_START_MIN_PWM = 0.05


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2, sort_keys=True)
    os.replace(tmp, path)


def _normalize_control_mode(value: Any) -> str:
    return str(value or "").strip().upper()


def _normalize_command_mode(value: Any) -> str:
    mode = str(value or "").strip().lower().replace("-", "_")
    if mode in ("set_track_velocity", "track"):
        return "track_velocity"
    if mode in ("set_twist",):
        return "twist"
    return mode if mode in COMMAND_MODES else "track_velocity"


def _read_control_mode_file() -> str:
    try:
        data = json.loads(CONTROL_MODE_PATH.read_text(encoding="utf-8"))
        return _normalize_control_mode(data.get("control_mode"))
    except Exception:
        return ""


def _ensure_control_mode(mode: str, *, token: str) -> Dict[str, Any]:
    target = _normalize_control_mode(mode)
    if target != CANONICAL_CONTROL_MODE:
        raise RuntimeError(f"unsupported_control_mode:{target or 'MISSING'}")
    status = _wait_for_status(timeout_s=5.0)
    current = _normalize_control_mode(status.get("control_mode") or _read_control_mode_file())
    if current != target:
        raise RuntimeError(f"control_mode_not_unified:{current or 'MISSING'}")
    return status


def _edge_balance_low(rising: int, falling: int, *, min_ratio: float = 0.35) -> bool:
    if int(rising) <= 0 or int(falling) <= 0:
        return False
    return (min(int(rising), int(falling)) / max(int(rising), int(falling))) < float(min_ratio)


def _load_recent_manual_encoder_result(path: Path, *, max_age_s: float) -> Dict[str, Any]:
    try:
        age_s = max(0.0, time.time() - path.stat().st_mtime)
        if age_s > max(1.0, float(max_age_s)):
            return {"available": False, "path": str(path), "age_s": round(age_s, 3), "reason": "stale"}
        data = json.loads(path.read_text(encoding="utf-8"))
        return {
            "available": True,
            "path": str(path),
            "age_s": round(age_s, 3),
            "success": bool(data.get("success", False)),
            "diagnosis": list(data.get("diagnosis") or []),
            "left": dict(data.get("left") or {}),
            "right": dict(data.get("right") or {}),
        }
    except Exception as exc:
        return {"available": False, "path": str(path), "reason": str(exc)}


def _pid_diagnostics(status: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    pid = dict((status or {}).get("pid_diag") or (status or {}).get("pid") or {})
    monitor = dict((status or {}).get("control_monitor") or pid.get("monitor") or {})
    if "output_reason" not in pid and "output_reason" in monitor:
        pid["output_reason"] = monitor.get("output_reason")
    return pid, monitor


def _encoder_reading(status: Dict[str, Any]) -> Dict[str, Any]:
    encoder = dict((status or {}).get("encoder") or {})
    left = dict(encoder.get("left") or {})
    right = dict(encoder.get("right") or {})
    left_snap = dict(left.get("snapshot") or {})
    right_snap = dict(right.get("snapshot") or {})
    return {
        "left_model": str(left.get("model", "") or ""),
        "right_model": str(right.get("model", "") or ""),
        "left_pins": (_safe_int(left.get("pin_a"), -1), _safe_int(left.get("pin_b"), -1)),
        "right_pins": (_safe_int(right.get("pin_a"), -1), _safe_int(right.get("pin_b"), -1)),
        "left_pulses": _safe_int(left_snap.get("pulses"), 0),
        "right_pulses": _safe_int(right_snap.get("pulses"), 0),
        "left_distance_m": _safe_float(left_snap.get("distance_m"), 0.0),
        "right_distance_m": _safe_float(right_snap.get("distance_m"), 0.0),
        "left_direction_source": str(left_snap.get("direction_source", "") or ""),
        "right_direction_source": str(right_snap.get("direction_source", "") or ""),
        "left_a_rising": _safe_int(left.get("a_rising_count"), 0),
        "left_a_falling": _safe_int(left.get("a_falling_count"), 0),
        "left_b_rising": _safe_int(left.get("b_rising_count"), 0),
        "left_b_falling": _safe_int(left.get("b_falling_count"), 0),
        "left_forward_count": _safe_int(left.get("forward_count"), 0),
        "left_reverse_count": _safe_int(left.get("reverse_count"), 0),
        "right_a_rising": _safe_int(right.get("a_rising_count"), 0),
        "right_a_falling": _safe_int(right.get("a_falling_count"), 0),
        "right_b_rising": _safe_int(right.get("b_rising_count"), 0),
        "right_b_falling": _safe_int(right.get("b_falling_count"), 0),
        "right_forward_count": _safe_int(right.get("forward_count"), 0),
        "right_reverse_count": _safe_int(right.get("reverse_count"), 0),
        "snapshot_health": str(dict(encoder.get("service") or {}).get("snapshot_health", "") or ""),
    }


def _motion_command_reading(status: Dict[str, Any]) -> Dict[str, Any]:
    st = dict(status or {})
    pid, _monitor = _pid_diagnostics(st)
    motion_command = dict(st.get("motion_command") or {})
    requested = dict(motion_command.get("requested_motion_intent") or {})
    requested_track = dict(motion_command.get("requested_track_reference") or st.get("requested_track_reference") or {})
    limited = dict(motion_command.get("limited_motion_intent") or {})
    ssot = dict(motion_command.get("motion_intent_ssot") or {})
    resolved = dict(ssot.get("resolved_intent") or {})
    pwm = dict(st.get("pwm") or {})
    return {
        "source": str(motion_command.get("source") or st.get("motion_command_source") or ""),
        "command_type": str(motion_command.get("command_type") or st.get("active_motion_command_type") or ""),
        "execution_mode": str(motion_command.get("execution_mode") or st.get("motion_execution_mode") or ""),
        "requested_v_mps": _safe_float(requested.get("v"), 0.0),
        "requested_left_mps": _safe_float(requested_track.get("left_mps"), 0.0),
        "requested_right_mps": _safe_float(requested_track.get("right_mps"), 0.0),
        "limited_v_mps": _safe_float(limited.get("v"), 0.0),
        "resolved_v_mps": _safe_float(resolved.get("v_mps"), _safe_float(limited.get("v"), 0.0)),
        "status_v_cmd": _safe_float(st.get("v_cmd"), 0.0),
        "pid_v_cmd": _safe_float(pid.get("v_cmd"), 0.0),
        "pid_v_l_ref": _safe_float(pid.get("v_l_ref"), 0.0),
        "pid_v_r_ref": _safe_float(pid.get("v_r_ref"), 0.0),
        "pwm_left": _safe_float(pwm.get("left"), 0.0),
        "pwm_right": _safe_float(pwm.get("right"), 0.0),
        "arbitration_reason": str(motion_command.get("arbitration_reason", "") or ""),
        "mismatch_reason": str(motion_command.get("mismatch_reason", "") or ""),
    }


def _motion_start_observed(status: Dict[str, Any], *, requested_speed_mps: float) -> bool:
    reading = _motion_command_reading(status)
    min_v = max(0.003, abs(float(requested_speed_mps)) * 0.12)
    v_fields = (
        reading["status_v_cmd"],
        reading["pid_v_cmd"],
        reading["pid_v_l_ref"],
        reading["pid_v_r_ref"],
    )
    pwm_seen = max(abs(reading["pwm_left"]), abs(reading["pwm_right"])) > MOTION_START_MIN_PWM
    positive_motion_seen = any(_safe_float(v, 0.0) >= min_v for v in v_fields)
    command_type = str(reading.get("command_type", "") or "").strip().lower()
    requested_positive = _safe_float(reading.get("requested_v_mps"), 0.0) >= min_v
    requested_tracks_positive = (
        _safe_float(reading.get("requested_left_mps"), 0.0) >= min_v
        and _safe_float(reading.get("requested_right_mps"), 0.0) >= min_v
    )
    command_seen = (
        (command_type in ("set_twist", "set_motion_target") and requested_positive)
        or (command_type == "set_track_velocity" and requested_tracks_positive)
    )
    return bool((command_seen and positive_motion_seen) or pwm_seen)


def _send_motion_command(
    *,
    command_mode: str,
    token: str,
    motion_source: str,
    speed_mps: float,
    timeout_s: float,
) -> Dict[str, Any]:
    mode = _normalize_command_mode(command_mode)
    if mode == "track_velocity":
        return _send_command_checked(
            "set_track_velocity",
            token=str(token),
            timeout_s=float(timeout_s),
            left_mps=float(speed_mps),
            right_mps=float(speed_mps),
            motion_source=str(motion_source),
        )
    return _send_command_checked(
        "set_twist",
        token=str(token),
        timeout_s=float(timeout_s),
        v=float(speed_mps),
        omega=0.0,
        motion_source=str(motion_source),
    )


def _append_motion_keepalive(
    *,
    command_mode: str,
    token: str,
    motion_source: str,
    speed_mps: float,
) -> None:
    mode = _normalize_command_mode(command_mode)
    if mode == "track_velocity":
        _append_command(
            "set_track_velocity",
            token=str(token),
            left_mps=float(speed_mps),
            right_mps=float(speed_mps),
            motion_source=str(motion_source),
        )
        return
    _append_command(
        "set_twist",
        token=str(token),
        v=float(speed_mps),
        omega=0.0,
        motion_source=str(motion_source),
    )


def _runtime_sample(status: Dict[str, Any]) -> Dict[str, Any]:
    pid, monitor = _pid_diagnostics(status)
    encoder = dict((status or {}).get("encoder") or {})
    canonical = dict(encoder.get("canonical") or {})
    can_vel = dict(canonical.get("canonical_velocity") or {})
    raw = dict(canonical.get("raw_measurement") or {})
    raw_vel = dict(raw.get("velocity") or {})
    ekf = dict((status or {}).get("ekf") or {})
    if not ekf:
        ekf = dict((status or {}).get("pose") or {})
    motion_command = _motion_command_reading(status)
    return {
        "control_mode": str((status or {}).get("control_mode", "") or ""),
        "odometry_mode": str((status or {}).get("odometry_mode", "") or ""),
        "motion_execution_mode": str((status or {}).get("motion_execution_mode", "") or ""),
        "motion_command": motion_command,
        "pid": {
            "control_mode": str(pid.get("control_mode", "") or ""),
            "output_reason": str(pid.get("output_reason", "") or ""),
            "v_cmd": _safe_float(pid.get("v_cmd"), 0.0),
            "omega_cmd": _safe_float(pid.get("omega_cmd"), 0.0),
            "v_l": _safe_float(pid.get("v_l"), 0.0),
            "v_r": _safe_float(pid.get("v_r"), 0.0),
            "v_l_ref": _safe_float(pid.get("v_l_ref"), 0.0),
            "v_r_ref": _safe_float(pid.get("v_r_ref"), 0.0),
            "pwm_executor_l": _safe_float(pid.get("pwm_executor_l"), 0.0),
            "pwm_executor_r": _safe_float(pid.get("pwm_executor_r"), 0.0),
            "wheel_loop_enabled": bool(pid.get("wheel_loop_enabled", False)),
            "wheel_loop_feedback_source": str(pid.get("wheel_loop_feedback_source", "") or ""),
            "wheel_loop_left_output_reason": str(pid.get("wheel_loop_left_output_reason", "") or ""),
            "wheel_loop_right_output_reason": str(pid.get("wheel_loop_right_output_reason", "") or ""),
            "straight_hold_active": bool(dict(pid.get("straight_hold") or {}).get("active", False)),
            "motor_compensation_active": bool(dict(pid.get("motor_compensation") or {}).get("active", False)),
            "monitor_mode": str(monitor.get("mode", "") or ""),
        },
        "encoder_canonical": {
            "state": str(canonical.get("canonical_state", "") or ""),
            "combined_trust": _safe_float(canonical.get("combined_trust"), 0.0),
            "ekf_usage_mode": str(canonical.get("ekf_usage_mode", "") or ""),
            "flags": list(canonical.get("flags") or []),
            "left_mps": _safe_float(can_vel.get("left_mps"), 0.0),
            "right_mps": _safe_float(can_vel.get("right_mps"), 0.0),
            "left_raw_mps": _safe_float(raw_vel.get("left_mps"), 0.0),
            "right_raw_mps": _safe_float(raw_vel.get("right_mps"), 0.0),
        },
        "ekf": {
            "v": _safe_float(ekf.get("v"), 0.0),
            "omega_rad_s": _safe_float(ekf.get("omega_rad_s"), 0.0),
            "theta_deg": _safe_float(ekf.get("theta_deg"), 0.0),
        },
    }


def _pwm_chatter_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    active = []
    for row in samples:
        pwm = dict(row.get("pwm") or {})
        left = abs(_safe_float(pwm.get("left"), 0.0))
        right = abs(_safe_float(pwm.get("right"), 0.0))
        if max(left, right) > 0.05:
            active.append((left, right))
    if not active:
        return {
            "active_sample_count": 0,
            "one_side_zero_fraction": 0.0,
            "saturation_fraction": 0.0,
            "max_abs_pwm_delta": 0.0,
            "dominance_flip_count": 0,
        }
    one_side_zero = 0
    saturated = 0
    max_delta = 0.0
    flips = 0
    last_dom = 0
    for left, right in active:
        if (left <= 0.05 and right >= 0.20) or (right <= 0.05 and left >= 0.20):
            one_side_zero += 1
        if left >= 0.95 or right >= 0.95:
            saturated += 1
        max_delta = max(max_delta, abs(left - right))
        dom = 1 if left > right + 0.20 else (-1 if right > left + 0.20 else 0)
        if dom != 0 and last_dom != 0 and dom != last_dom:
            flips += 1
        if dom != 0:
            last_dom = dom
    denom = max(1, len(active))
    return {
        "active_sample_count": len(active),
        "one_side_zero_fraction": round(one_side_zero / denom, 6),
        "saturation_fraction": round(saturated / denom, 6),
        "max_abs_pwm_delta": round(float(max_delta), 6),
        "dominance_flip_count": int(flips),
    }


def evaluate_bench_audit(metrics: Dict[str, Any]) -> List[str]:
    failures: List[str] = []
    start = dict(metrics.get("encoder_start") or {})
    end = dict(metrics.get("encoder_end") or {})
    delta_l_p = _safe_int(metrics.get("left_pulse_delta"), 0)
    delta_r_p = _safe_int(metrics.get("right_pulse_delta"), 0)
    delta_l_m = _safe_float(metrics.get("left_distance_delta_m"), 0.0)
    delta_r_m = _safe_float(metrics.get("right_distance_delta_m"), 0.0)
    min_counts = max(1, _safe_int(metrics.get("min_forward_counts"), 8))
    max_abs_counts = max(min_counts, _safe_int(metrics.get("max_abs_counts"), 7000))
    previous_manual = dict(metrics.get("previous_manual_encoder") or {})
    previous_manual_diag = set(str(item) for item in (previous_manual.get("diagnosis") or []))
    motion_start_observed = bool(metrics.get("motion_start_observed", True))

    if start.get("left_model") != EXPECTED_MODEL or start.get("right_model") != EXPECTED_MODEL:
        failures.append("encoder_model_mismatch")
    if tuple(start.get("left_pins") or ()) != EXPECTED_PINS["left"]:
        failures.append("left_encoder_gpio_mismatch")
    if tuple(start.get("right_pins") or ()) != EXPECTED_PINS["right"]:
        failures.append("right_encoder_gpio_mismatch")
    if str(end.get("snapshot_health", "")).upper() != "OK":
        failures.append("encoder_snapshot_not_ok")
    if _safe_float(metrics.get("max_pwm_left"), 0.0) <= 0.05:
        failures.append("left_motor_pwm_not_observed")
    if _safe_float(metrics.get("max_pwm_right"), 0.0) <= 0.05:
        failures.append("right_motor_pwm_not_observed")
    if not motion_start_observed:
        failures.append("motion_start_not_observed")
    if motion_start_observed:
        if delta_l_p < min_counts:
            failures.append("left_encoder_not_forward")
        if delta_r_p < min_counts:
            failures.append("right_encoder_not_forward")
        if delta_r_p < -min_counts:
            failures.append("right_motor_or_encoder_still_reversed")
        if abs(delta_l_p) > max_abs_counts:
            failures.append("left_encoder_pulse_rate_implausible")
        if abs(delta_r_p) > max_abs_counts:
            failures.append("right_encoder_pulse_rate_implausible")
        if abs(delta_l_m) > _safe_float(metrics.get("max_side_distance_m"), 1.0):
            failures.append("left_encoder_distance_implausible")
        if abs(delta_r_m) > _safe_float(metrics.get("max_side_distance_m"), 1.0):
            failures.append("right_encoder_distance_implausible")
        if max(abs(delta_l_p), abs(delta_r_p)) >= min_counts:
            side_ratio = min(abs(delta_l_p), abs(delta_r_p)) / max(abs(delta_l_p), abs(delta_r_p), 1)
            if side_ratio < 0.20:
                failures.append("encoder_side_asymmetry")

    edge_pairs = (
        ("left", "a"),
        ("left", "b"),
        ("right", "a"),
        ("right", "b"),
    )
    severe_edge_fault_by_side = {"left": False, "right": False}
    if motion_start_observed:
        for side, channel in edge_pairs:
            rising = _safe_int(end.get(f"{side}_{channel}_rising"), 0) - _safe_int(
                start.get(f"{side}_{channel}_rising"), 0
            )
            falling = _safe_int(end.get(f"{side}_{channel}_falling"), 0) - _safe_int(
                start.get(f"{side}_{channel}_falling"), 0
            )
            if rising <= 0:
                failures.append(f"{side}_encoder_{channel}_no_rising_edges")
                severe_edge_fault_by_side[side] = True
            if falling <= 0:
                failures.append(f"{side}_encoder_{channel}_no_falling_edges")
                severe_edge_fault_by_side[side] = True

        if (
            severe_edge_fault_by_side["left"]
            or abs(delta_l_p) > max_abs_counts
            or abs(delta_l_m) > _safe_float(metrics.get("max_side_distance_m"), 1.0)
        ):
            failures.append("left_encoder_signal_unusable")
        if (
            severe_edge_fault_by_side["right"]
            or abs(delta_r_p) > max_abs_counts
            or abs(delta_r_m) > _safe_float(metrics.get("max_side_distance_m"), 1.0)
        ):
            failures.append("right_encoder_signal_unusable")
        if "LEFT:NO_AB_EDGES" in previous_manual_diag and severe_edge_fault_by_side["left"]:
            failures.append("left_encoder_open_or_noise_coupled")
        if "RIGHT:NO_AB_EDGES" in previous_manual_diag and severe_edge_fault_by_side["right"]:
            failures.append("right_encoder_open_or_noise_coupled")

    pwm_chatter = dict(metrics.get("pwm_chatter") or {})
    if _safe_float(pwm_chatter.get("one_side_zero_fraction"), 0.0) > 0.20:
        failures.append("pwm_one_side_chatter_high")
    if _safe_float(pwm_chatter.get("saturation_fraction"), 0.0) > 0.35:
        failures.append("pwm_saturation_high")
    if _safe_int(pwm_chatter.get("dominance_flip_count"), 0) >= 2:
        failures.append("pwm_dominance_flip_high")

    target_mode = str(metrics.get("control_mode_target", "") or "").strip().upper()
    applied_mode = str(metrics.get("control_mode_applied", "") or "").strip().upper()
    restored_mode = str(metrics.get("control_mode_restored", "") or "").strip().upper()
    original_mode = str(metrics.get("control_mode_original", "") or "").strip().upper()
    if target_mode and applied_mode and target_mode != applied_mode:
        failures.append("control_mode_target_not_applied")
    if original_mode and restored_mode and restored_mode != original_mode:
        failures.append("control_mode_restore_failed")

    if bool(metrics.get("failsafe_seen", False)):
        failures.append("failsafe_seen")
    if bool(metrics.get("safety_block_seen", False)):
        failures.append("safety_block_seen")
    if not bool(metrics.get("normal_stop_confirmed", False)):
        failures.append("normal_stop_not_confirmed")
    return list(dict.fromkeys(failures))


def run(args: argparse.Namespace) -> Dict[str, Any]:
    initial_status = _ensure_control_mode(str(args.control_mode), token=str(args.token))
    original_control_mode = _normalize_control_mode(initial_status.get("control_mode") or _read_control_mode_file())
    target_control_mode = CANONICAL_CONTROL_MODE
    control_mode_applied = original_control_mode
    control_mode_restored = original_control_mode
    control_mode_restore_error = ""

    start_status = initial_status
    start_encoder = _encoder_reading(start_status)
    max_pwm_left = 0.0
    max_pwm_right = 0.0
    failsafe_seen = False
    safety_block_seen = False
    command_error = ""
    start_command: Dict[str, Any] = {}
    stop_command: Dict[str, Any] = {}
    motion_start_observed = False
    motion_start_elapsed_s = None
    samples: List[Dict[str, Any]] = []
    end_status = start_status
    motion_source = str(args.motion_source or DEFAULT_MOTION_SOURCE)
    command_mode = _normalize_command_mode(getattr(args, "command_mode", "track_velocity"))
    normal_stop_confirmed = False

    try:
        try:
            start_command = _send_motion_command(
                command_mode=command_mode,
                token=str(args.token),
                motion_source=motion_source,
                speed_mps=float(args.speed_mps),
                timeout_s=4.0,
            )
            started = time.monotonic()
            last_keepalive = started
            while (time.monotonic() - started) <= float(args.duration_s):
                now = time.monotonic()
                if (now - last_keepalive) >= float(args.keepalive_s):
                    _append_motion_keepalive(
                        command_mode=command_mode,
                        token=str(args.token),
                        motion_source=motion_source,
                        speed_mps=float(args.speed_mps),
                    )
                    last_keepalive = now

                status = _read_json(STATUS_PATH)
                if not status:
                    time.sleep(float(args.poll_s))
                    continue
                end_status = status
                state = str(status.get("state", "") or "").upper()
                if state == "FAILSAFE":
                    failsafe_seen = True
                    break
                safety = dict(status.get("safety") or {})
                if safety and not bool(safety.get("allow", True)):
                    safety_block_seen = True
                    break
                if (not motion_start_observed) and _motion_start_observed(
                    status,
                    requested_speed_mps=float(args.speed_mps),
                ):
                    motion_start_observed = True
                    motion_start_elapsed_s = round(now - started, 4)
                pwm = dict(status.get("pwm") or {})
                max_pwm_left = max(max_pwm_left, abs(_safe_float(pwm.get("left"), 0.0)))
                max_pwm_right = max(max_pwm_right, abs(_safe_float(pwm.get("right"), 0.0)))
                if len(samples) < 120:
                    reading = _encoder_reading(status)
                    row = {
                        "elapsed_s": round(now - started, 4),
                        "pwm": {
                            "left": _safe_float(pwm.get("left"), 0.0),
                            "right": _safe_float(pwm.get("right"), 0.0),
                        },
                        "left_pulses": reading["left_pulses"],
                        "right_pulses": reading["right_pulses"],
                        "left_direction_source": reading["left_direction_source"],
                        "right_direction_source": reading["right_direction_source"],
                    }
                    row.update(_runtime_sample(status))
                    samples.append(row)
                time.sleep(float(args.poll_s))
        except Exception as exc:
            command_error = str(exc)
        finally:
            try:
                stop_command = _send_motion_command(
                    command_mode=command_mode,
                    token=str(args.token),
                    motion_source=motion_source,
                    speed_mps=0.0,
                    timeout_s=4.0,
                )
            except Exception as exc:
                command_error = command_error or f"stop_command:{exc}"

        try:
            end_status = _wait_until_stopped(timeout_s=float(args.stop_timeout_s))
            normal_stop_confirmed = True
        except Exception as exc:
            command_error = command_error or f"stop_wait:{exc}"
    finally:
        # UNIFIED is immutable; validation tools never rewrite runtime configuration.
        pass

    end_encoder = _encoder_reading(end_status)
    counts_per_revolution = max(1, int(args.counts_per_revolution))
    max_abs_counts = int(
        counts_per_revolution
        * max(0.5, float(args.max_output_rps))
        * max(0.1, float(args.duration_s))
        * 1.20
    )
    previous_manual = _load_recent_manual_encoder_result(
        Path(args.manual_artifact),
        max_age_s=float(args.manual_max_age_s),
    )
    metrics = {
        "encoder_start": start_encoder,
        "encoder_end": end_encoder,
        "left_pulse_delta": end_encoder["left_pulses"] - start_encoder["left_pulses"],
        "right_pulse_delta": end_encoder["right_pulses"] - start_encoder["right_pulses"],
        "left_distance_delta_m": end_encoder["left_distance_m"] - start_encoder["left_distance_m"],
        "right_distance_delta_m": end_encoder["right_distance_m"] - start_encoder["right_distance_m"],
        "max_pwm_left": max_pwm_left,
        "max_pwm_right": max_pwm_right,
        "min_forward_counts": int(args.min_forward_counts),
        "max_abs_counts": int(max_abs_counts),
        "max_side_distance_m": float(args.max_side_distance_m),
        "pwm_chatter": _pwm_chatter_metrics(samples),
        "previous_manual_encoder": previous_manual,
        "control_mode_original": str(original_control_mode),
        "control_mode_target": str(target_control_mode),
        "control_mode_applied": str(control_mode_applied),
        "control_mode_restored": str(control_mode_restored),
        "control_mode_restore_error": str(control_mode_restore_error),
        "failsafe_seen": failsafe_seen,
        "safety_block_seen": safety_block_seen,
        "normal_stop_confirmed": normal_stop_confirmed,
        "motion_source": str(motion_source),
        "command_mode": str(command_mode),
        "motion_start_observed": bool(motion_start_observed),
        "motion_start_elapsed_s": motion_start_elapsed_s,
        "start_command": dict(start_command),
        "stop_command": dict(stop_command),
        "command_error": command_error,
    }
    failures = evaluate_bench_audit(metrics)
    if command_error:
        failures.append("command_error")
    return {
        "success": not failures,
        "test": "kit0085_motor_bench_direction_audit",
        "hardware": "DFRobot KIT0085",
        "command_path": (
            f"set_track_velocity via runtime/commands.jsonl ({motion_source})"
            if command_mode == "track_velocity"
            else f"set_twist via runtime/commands.jsonl ({motion_source})"
        ),
        "raised_wheels_required": True,
        "control_mode_for_run": str(control_mode_applied),
        "control_mode_restored": str(control_mode_restored),
        "speed_mps": float(args.speed_mps),
        "duration_s": float(args.duration_s),
        "metrics": metrics,
        "samples": samples,
        "failures": list(dict.fromkeys(failures)),
        "artifact": str(LATEST_RESULT_PATH),
    }


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        print(json.dumps({"success": False, "error": str(exc)}, sort_keys=True))
        return 40

    parser = argparse.ArgumentParser(description="Raised-wheel KIT0085 motor direction and encoder audit.")
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--control-mode", choices=(CANONICAL_CONTROL_MODE,), default=CANONICAL_CONTROL_MODE)
    parser.add_argument("--motion-source", default=DEFAULT_MOTION_SOURCE)
    parser.add_argument("--command-mode", choices=COMMAND_MODES, default="track_velocity")
    parser.add_argument("--speed-mps", type=float, default=0.035)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--keepalive-s", type=float, default=0.15)
    parser.add_argument("--poll-s", type=float, default=0.05)
    parser.add_argument("--stop-timeout-s", type=float, default=4.0)
    parser.add_argument("--counts-per-revolution", type=int, default=663)
    parser.add_argument("--max-output-rps", type=float, default=3.5)
    parser.add_argument("--min-forward-counts", type=int, default=8)
    parser.add_argument("--max-side-distance-m", type=float, default=1.0)
    parser.add_argument("--manual-artifact", default=str(LATEST_MANUAL_ENCODER_PATH))
    parser.add_argument("--manual-max-age-s", type=float, default=3600.0)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()

    try:
        result = run(args)
    except Exception as exc:
        result = {
            "success": False,
            "test": "kit0085_motor_bench_direction_audit",
            "error": str(exc),
            "failures": ["bench_audit_exception"],
            "artifact": str(LATEST_RESULT_PATH),
        }
    _write_json_atomic(LATEST_RESULT_PATH, result)
    if args.compact:
        metrics = dict(result.get("metrics") or {})
        print(
            "KIT0085_BENCH "
            f"result={'PASS' if result.get('success') else 'FAIL'} "
            f"mode={metrics.get('control_mode_applied') or result.get('control_mode_for_run') or ''} "
            f"source={metrics.get('motion_source') or ''} "
            f"cmd={metrics.get('command_mode') or ''} "
            f"start={1 if metrics.get('motion_start_observed') else 0} "
            f"pulses={_safe_int(metrics.get('left_pulse_delta'))}/"
            f"{_safe_int(metrics.get('right_pulse_delta'))} "
            f"pwm={_safe_float(metrics.get('max_pwm_left')):.2f}/"
            f"{_safe_float(metrics.get('max_pwm_right')):.2f} "
            f"chatter={_safe_float(dict(metrics.get('pwm_chatter') or {}).get('one_side_zero_fraction')):.2f} "
            f"failures={','.join(result.get('failures') or []) or 'none'}"
        )
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
