#!/usr/bin/env python3
"""Live four-direction motor feed-forward calibration using direct PWM pulses."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import statistics
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded
from middleware.ffp import lookup_wheel_feedforward
from tools.kit0085_motor_bench_audit import _encoder_reading
from tools.lidar_1m_step import (
    STATUS_PATH,
    _read_json,
    _safe_stop_best_effort,
    _send_command_checked,
    _wait_for_status_progress,
    _wait_until_stopped,
)
from tools.live_motion_measurement_validator import (
    _run_pause,
    _wait_measurement_ready_after_reset,
)

SPEED_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
AGENT_TESTS_DIR = test_artifacts_dir()
# Runtime command-authority input, not a session-owned result artifact.  The
# async command reader resolves this same stable path in every Test Hub run.
ARM_PATH = PROJECT_ROOT / "runtime" / "agent_tests" / "feedforward_calibration_arm.json"
LATEST_RESULT = AGENT_TESTS_DIR / "latest_motor_feedforward_calibration.json"
LATEST_SAMPLES = AGENT_TESTS_DIR / "latest_motor_feedforward_calibration_samples.jsonl"
LATEST_BACKUP = AGENT_TESTS_DIR / "speed_map_before_feedforward_calibration.json"
LATEST_CANDIDATE = AGENT_TESTS_DIR / "candidate_wheel_speed_map_feedforward.json"
SHUTTLE_RESULT = AGENT_TESTS_DIR / "latest_speed_map_calibration_acquisition.json"
SHUTTLE_SUPPLEMENT_RESULT = (
    AGENT_TESTS_DIR / "latest_speed_map_calibration_supplement.json"
)
SHUTTLE_SAMPLES = AGENT_TESTS_DIR / "latest_speed_map_calibration_samples.jsonl"
SHUTTLE_BACKUP = AGENT_TESTS_DIR / "speed_map_before_speed_map_calibration.json"
DEFAULT_SPEEDS = (0.15, 0.20, 0.25, 0.30)
STARTUP_PWM_POINTS = (0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.23, 0.26)
MAINTENANCE_PWM_POINTS = (0.26, 0.23, 0.20, 0.18, 0.16, 0.14, 0.12, 0.10, 0.08, 0.06)
STABLE_PWM_POINTS = (0.16, 0.20, 0.24, 0.30, 0.38, 0.48, 0.60, 0.64, 0.88)
NON_BLOCKING_CALIBRATION_ENCODER_FLAGS = frozenset({"SIDE_ASYMMETRY"})
THRESHOLD_OUTCOME_ENCODER_FLAGS = frozenset(
    {
        "SIDE_ASYMMETRY",
        "SIDE_ASYMMETRY_CRITICAL",
        "FORWARD_COHERENCE_LOW",
        "FORWARD_DIRECTION_MISMATCH",
        "BACKWARD_DIRECTION_MISMATCH",
        "PWM_ENCODER_SYMMETRY_VIOLATION",
        "DIRECTION_SWITCH_GRACE",
    }
)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return out if math.isfinite(out) else float(default)


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    os.replace(tmp, path)


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")


def _speed_key(speed: float) -> str:
    return f"{float(speed):.2f}"


def _map_pwm(speed_map: Dict[str, Any], direction: str, speed: float, side: str) -> float:
    sign = 1.0 if str(direction).lower() == "forward" else -1.0
    pwm, _ = lookup_wheel_feedforward(
        speed_map,
        side=side,
        target_mps=sign * abs(float(speed)),
        require_active=False,
    )
    return abs(float(pwm))


def _runtime_ready(status: Dict[str, Any]) -> bool:
    localization = dict(status.get("localization_gate") or status.get("localization_gate_status") or {})
    startup = dict(status.get("startup") or {})
    mode = str(localization.get("mode", "") or "").upper()
    stationary_degraded_ready = bool(
        mode == "DEGRADED"
        and _safe_float(localization.get("trust"), 0.0) >= 0.35
        and not bool(localization.get("hard_stop", False))
        and bool(localization.get("idle_stationary_guard_active", False))
    )
    encoder_reliability = dict((status.get("encoder") or {}).get("canonical") or {})
    encoder_ready = bool(
        str(encoder_reliability.get("snapshot_health", "") or "").upper() == "OK"
        and not bool(encoder_reliability.get("anomaly_active", False))
        and _safe_float(encoder_reliability.get("combined_trust"), 0.0) >= 0.55
    )
    return bool(
        str(status.get("state", "") or "").upper() == "IDLE"
        and bool(startup.get("ready", status.get("startup_ready", False)))
        and (mode == "TRACKING" or stationary_degraded_ready)
        and localization.get("allow_motion", False)
        and str(status.get("lidar_health", "") or "").upper() == "OK"
        and encoder_ready
    )


def _idle_calibration_reanchor_needed(status: Dict[str, Any]) -> bool:
    """Detect an IDLE/manual-reposition localization block safe to reanchor."""

    pwm = dict(status.get("pwm") or {})
    localization = dict(
        status.get("localization_gate")
        or status.get("localization_gate_status")
        or {}
    )
    return bool(
        str(status.get("state", "") or "").upper() == "IDLE"
        and max(
            abs(_safe_float(pwm.get("left"), 0.0)),
            abs(_safe_float(pwm.get("right"), 0.0)),
        )
        <= 0.02
        and (
            bool(localization.get("hard_stop", False))
            or not bool(localization.get("allow_motion", False))
        )
    )


def _wait_calibration_ready(
    timeout_s: float,
    *,
    token: str = "GUI_DEFAULT",
    auto_reanchor_delay_s: float = 3.0,
) -> Dict[str, Any]:
    started = time.monotonic()
    deadline = started + max(1.0, float(timeout_s))
    stable = 0
    last: Dict[str, Any] = {}
    auto_reanchor_attempted = False
    auto_reanchor_error = ""
    while time.monotonic() <= deadline:
        last = _read_json(STATUS_PATH) or {}
        if _runtime_ready(last):
            stable += 1
            if stable >= 3:
                return last
        else:
            stable = 0
            if (
                not auto_reanchor_attempted
                and time.monotonic() - started
                >= max(0.0, float(auto_reanchor_delay_s))
                and _idle_calibration_reanchor_needed(last)
            ):
                auto_reanchor_attempted = True
                print(
                    "SPEED_MAP_AUTO_REANCHOR "
                    "reason=idle_localization_blocked",
                    flush=True,
                )
                try:
                    _send_command_checked(
                        "reset_pos",
                        token=str(token),
                        timeout_s=6.0,
                    )
                    _wait_for_status_progress(min_increments=1, timeout_s=3.0)
                except Exception as exc:
                    auto_reanchor_error = str(exc)
        time.sleep(0.1)
    localization = dict(last.get("localization_gate") or {})
    raise RuntimeError(
        "calibration_runtime_not_ready:"
        f"state={last.get('state')}:mode={localization.get('mode')}:"
        f"allow={localization.get('allow_motion')}:"
        f"auto_reanchor_attempted={auto_reanchor_attempted}:"
        f"auto_reanchor_error={auto_reanchor_error}"
    )


def _distance_pair(status: Dict[str, Any]) -> Tuple[float, float]:
    encoder = _encoder_reading(status)
    return float(encoder["left_distance_m"]), float(encoder["right_distance_m"])


def _velocity_pair(status: Dict[str, Any]) -> Tuple[float, float]:
    encoder = dict(status.get("encoder") or {})
    canonical = dict(encoder.get("canonical") or {})
    velocity = dict(canonical.get("canonical_velocity") or {})
    if velocity:
        return (
            _safe_float(velocity.get("left_mps"), 0.0),
            _safe_float(velocity.get("right_mps"), 0.0),
        )
    left = dict((encoder.get("left") or {}).get("snapshot") or {})
    right = dict((encoder.get("right") or {}).get("snapshot") or {})
    return (
        _safe_float(left.get("velocity_mps"), 0.0),
        _safe_float(right.get("velocity_mps"), 0.0),
    )


def _stability_summary(
    values: List[float],
    *,
    sign: float,
    threshold_mps: float = 0.015,
    elapsed_s: List[float] | None = None,
) -> Dict[str, Any]:
    directed = [float(sign) * float(value) for value in values]
    moving = [value >= float(threshold_mps) for value in directed]
    positive = [max(0.0, value) for value in directed]
    mean = statistics.mean(positive) if positive else 0.0
    stddev = statistics.pstdev(positive) if len(positive) >= 2 else 0.0
    dropout_transitions = 0
    for previous, current in zip(moving, moving[1:]):
        if previous and not current:
            dropout_transitions += 1
    acceleration_slope = 0.0
    time_values = list(elapsed_s or [])
    if len(time_values) == len(positive) and len(positive) >= 3:
        time_mean = statistics.mean(time_values)
        speed_mean = statistics.mean(positive)
        denominator = sum((value - time_mean) ** 2 for value in time_values)
        if denominator > 1e-9:
            acceleration_slope = sum(
                (sample_time - time_mean) * (sample_speed - speed_mean)
                for sample_time, sample_speed in zip(time_values, positive)
            ) / denominator
    return {
        "sample_count": len(values),
        "moving_sample_ratio": (sum(moving) / len(moving)) if moving else 0.0,
        "mean_mps": float(mean),
        "median_mps": statistics.median(positive) if positive else 0.0,
        "stddev_mps": float(stddev),
        "coefficient_of_variation": (stddev / mean) if mean > 0.003 else None,
        "acceleration_slope_mps2": float(acceleration_slope),
        "dropout_transitions": int(dropout_transitions),
        "wrong_direction_samples": sum(1 for value in directed if value < -0.003),
    }


def _safety_fault(status: Dict[str, Any]) -> str:
    stop = dict(status.get("stop_status") or {})
    state = str(status.get("state", "") or "").upper()
    stop_type = str(stop.get("type", "") or "").upper()
    if state == "FAILSAFE" or "EMERGENCY" in stop_type or bool(status.get("emergency_stop_active", False)):
        return stop_type or state
    if status.get("safety_allow") is False:
        return str((status.get("safety") or {}).get("reason", "SAFETY_NOT_ALLOWED") or "SAFETY_NOT_ALLOWED")
    return ""


def _run_pulse(
    *,
    stage: str,
    repeat: int,
    target_speed: float,
    direction: str,
    left_pwm: float,
    right_pwm: float,
    nonce: str,
    token: str,
    stabilization_s: float,
    measurement_s: float,
    poll_s: float,
    max_phase_distance_m: float,
    attempt: int = 1,
    sample_path: Path = LATEST_SAMPLES,
    sample_metadata: Dict[str, Any] | None = None,
    startup_left_pwm: float | None = None,
    startup_right_pwm: float | None = None,
    startup_duration_s: float = 0.0,
    target_distance_m: float | None = None,
    append_sample: bool = True,
) -> Dict[str, Any]:
    sign = 1.0 if direction == "forward" else -1.0
    start = _read_json(STATUS_PATH) or {}
    if not _runtime_ready(start):
        raise RuntimeError(f"runtime_not_ready_before_pulse:{direction}:{target_speed:.2f}")

    startup_duration = max(0.0, float(startup_duration_s))
    startup_left = abs(float(left_pwm)) if startup_left_pwm is None else abs(float(startup_left_pwm))
    startup_right = abs(float(right_pwm)) if startup_right_pwm is None else abs(float(startup_right_pwm))
    duration_s = startup_duration + float(stabilization_s) + float(measurement_s)
    command_start_distance = _distance_pair(start)
    command = _send_command_checked(
        "calibration_pwm_pulse",
        token=str(token),
        timeout_s=5.0,
        left_pwm=sign * abs(float(left_pwm)),
        right_pwm=sign * abs(float(right_pwm)),
        duration_s=duration_s,
        v_hint=sign * abs(float(target_speed)),
        arm_nonce=str(nonce),
        startup_left_pwm=sign * startup_left,
        startup_right_pwm=sign * startup_right,
        startup_duration_s=startup_duration,
        motion_source="SERVICE",
    )
    started = time.monotonic()
    measure_start: Tuple[float, float] | None = None
    measure_start_t: float | None = None
    measure_end: Tuple[float, float] | None = None
    measure_end_t: float | None = None
    pwm_seen: List[Tuple[float, float]] = []
    startup_pwm_seen: List[Tuple[float, float]] = []
    stable_velocity_left: List[float] = []
    stable_velocity_right: List[float] = []
    stable_velocity_elapsed_s: List[float] = []
    onset_left_s: float | None = None
    onset_right_s: float | None = None
    faults: List[str] = []
    encoder_anomaly_seen = False
    encoder_blocking_anomaly_seen = False
    encoder_transient_anomaly_seen = False
    encoder_health_seen: List[str] = []
    encoder_trust_seen: List[float] = []
    encoder_flags_seen: List[str] = []
    encoder_context_seen: List[str] = []
    direct_reason_seen = False
    pi_disabled_seen = False
    pi_violation_seen = False
    controller_distortion = {
        "straight_hold_applied": False,
        "feedforward_map_applied": False,
        "startup_floor_applied": False,
        "maintenance_floor_applied": False,
        "planner_correction_applied": False,
    }
    distance_limit_triggered = False
    distance_target = (
        None
        if target_distance_m is None
        else max(0.02, abs(float(target_distance_m)))
    )
    distance_target_reached = distance_target is None
    max_distance_observed_m = 0.0
    samples = 0
    while time.monotonic() - started <= duration_s + 0.25:
        status = _read_json(STATUS_PATH) or {}
        elapsed = time.monotonic() - started
        if status:
            samples += 1
            fault = _safety_fault(status)
            if fault:
                faults.append(fault)
            encoder_reliability = dict((status.get("encoder") or {}).get("canonical") or {})
            health = str(
                encoder_reliability.get("snapshot_health", status.get("encoder_reliability_health", ""))
                or ""
            )
            trust = _safe_float(
                encoder_reliability.get(
                    "combined_trust",
                    status.get("encoder_reliability_trust", math.nan),
                ),
                math.nan,
            )
            pwm = dict(status.get("pwm") or {})
            current_distance = _distance_pair(status)
            velocity_left, velocity_right = _velocity_pair(status)
            executor_diag = dict(
                status.get("control_monitor")
                or status.get("pid_diag")
                or status.get("pid")
                or {}
            )
            direct_active = bool(
                executor_diag.get("calibration_pwm", False)
                and str(executor_diag.get("output_reason", "") or "") == "CALIBRATION_DIRECT_PWM"
            )
            measurement_active = bool(
                direct_active
                and elapsed >= startup_duration + float(stabilization_s)
                and elapsed <= duration_s
            )
            anomaly_now = bool(
                encoder_reliability.get("anomaly_active", False)
                or status.get("encoder_anomaly_active", False)
            )
            if measurement_active:
                for distortion_key in controller_distortion:
                    controller_distortion[distortion_key] = bool(
                        controller_distortion[distortion_key]
                        or executor_diag.get(distortion_key, False)
                    )
                encoder_anomaly_seen = bool(encoder_anomaly_seen or anomaly_now)
                if health:
                    encoder_health_seen.append(health)
                if math.isfinite(trust):
                    encoder_trust_seen.append(trust)
                encoder_flags_seen.extend(
                    str(flag) for flag in list(encoder_reliability.get("flags") or [])
                )
                current_flags = {
                    str(flag) for flag in list(encoder_reliability.get("flags") or [])
                }
                blocking_flags = current_flags - NON_BLOCKING_CALIBRATION_ENCODER_FLAGS
                encoder_blocking_anomaly_seen = bool(
                    encoder_blocking_anomaly_seen
                    or blocking_flags
                    or (anomaly_now and not current_flags)
                )
                context = str(encoder_reliability.get("observation_context", "") or "")
                if context:
                    encoder_context_seen.append(context)
            else:
                encoder_transient_anomaly_seen = bool(
                    encoder_transient_anomaly_seen or anomaly_now
                )
            if elapsed <= duration_s:
                if direct_active and measurement_active:
                    pwm_seen.append((_safe_float(pwm.get("left")), _safe_float(pwm.get("right"))))
                elif direct_active and elapsed <= startup_duration:
                    startup_pwm_seen.append(
                        (_safe_float(pwm.get("left")), _safe_float(pwm.get("right")))
                    )
                if direct_active and onset_left_s is None and sign * velocity_left >= 0.015:
                    onset_left_s = float(elapsed)
                if direct_active and onset_right_s is None and sign * velocity_right >= 0.015:
                    onset_right_s = float(elapsed)
                max_distance_observed_m = max(
                    max_distance_observed_m,
                    abs(current_distance[0] - command_start_distance[0]),
                    abs(current_distance[1] - command_start_distance[1]),
                )
                directed_left_distance = sign * (
                    current_distance[0] - command_start_distance[0]
                )
                directed_right_distance = sign * (
                    current_distance[1] - command_start_distance[1]
                )
                directed_mean_distance = 0.5 * (
                    directed_left_distance + directed_right_distance
                )
                if (
                    distance_target is not None
                    and directed_mean_distance >= distance_target
                ):
                    distance_target_reached = True
                    if measurement_active and measure_start is not None:
                        measure_end = _distance_pair(status)
                        measure_end_t = time.monotonic()
                    _safe_stop_best_effort(token)
                    break
                if max_distance_observed_m >= max(0.10, float(max_phase_distance_m)):
                    distance_limit_triggered = True
                    faults.append("PHASE_DISTANCE_LIMIT")
                    _safe_stop_best_effort(token)
                    break
                direct_reason_seen = bool(direct_reason_seen or direct_active)
                if executor_diag.get("calibration_pwm", False):
                    pi_disabled = bool(
                        executor_diag.get("wheel_pi_enabled") is False
                        and abs(_safe_float(executor_diag.get("pi_correction_left_pwm"), 0.0)) <= 1e-9
                        and abs(_safe_float(executor_diag.get("pi_correction_right_pwm"), 0.0)) <= 1e-9
                        and executor_diag.get("feedforward_map_applied") is False
                    )
                    pi_disabled_seen = bool(pi_disabled_seen or pi_disabled)
                    pi_violation_seen = bool(pi_violation_seen or not pi_disabled)
                if measurement_active and measure_start is None:
                    measure_start = _distance_pair(status)
                    measure_start_t = time.monotonic()
                if measurement_active and measure_start is not None:
                    measure_end = _distance_pair(status)
                    measure_end_t = time.monotonic()
                    stable_velocity_left.append(float(velocity_left))
                    stable_velocity_right.append(float(velocity_right))
                    stable_velocity_elapsed_s.append(float(elapsed))
        time.sleep(max(0.04, float(poll_s)))

    stopped = _wait_until_stopped(timeout_s=6.0)
    if measure_start is None or measure_end is None or measure_start_t is None or measure_end_t is None:
        raise RuntimeError(f"encoder_measurement_missing:{direction}:{target_speed:.2f}")
    dt = max(1e-3, measure_end_t - measure_start_t)
    left_actual = (measure_end[0] - measure_start[0]) / dt
    right_actual = (measure_end[1] - measure_start[1]) / dt
    command_end_distance = _distance_pair(stopped)
    directed_distance_left = sign * (
        command_end_distance[0] - command_start_distance[0]
    )
    directed_distance_right = sign * (
        command_end_distance[1] - command_start_distance[1]
    )
    directed_distance_mean = 0.5 * (
        directed_distance_left + directed_distance_right
    )
    output = {
        "stage": stage,
        "repeat": int(repeat),
        "attempt": int(attempt),
        "target_speed_mps": float(target_speed),
        "direction": direction,
        "command": command,
        "commanded_pwm": {"left": sign * abs(left_pwm), "right": sign * abs(right_pwm)},
        "startup_commanded_pwm": {
            "left": sign * startup_left,
            "right": sign * startup_right,
        },
        "startup_duration_s": float(startup_duration),
        "actual_mps": {"left": float(left_actual), "right": float(right_actual)},
        "measurement_s": float(dt),
        "samples": int(samples),
        "direct_executor_observed": bool(direct_reason_seen),
        "pi_disabled_observed": bool(pi_disabled_seen and not pi_violation_seen),
        "pi_violation_seen": bool(pi_violation_seen),
        "controller_distortion": controller_distortion,
        "median_output_pwm": {
            "left": statistics.median([item[0] for item in pwm_seen]) if pwm_seen else 0.0,
            "right": statistics.median([item[1] for item in pwm_seen]) if pwm_seen else 0.0,
        },
        "median_startup_output_pwm": {
            "left": statistics.median([item[0] for item in startup_pwm_seen])
            if startup_pwm_seen
            else 0.0,
            "right": statistics.median([item[1] for item in startup_pwm_seen])
            if startup_pwm_seen
            else 0.0,
        },
        "faults": sorted(set(faults)),
        "safety_intervention_seen": any(
            fault != "PHASE_DISTANCE_LIMIT" for fault in set(faults)
        ),
        "encoder_anomaly_seen": bool(encoder_anomaly_seen),
        "encoder_blocking_anomaly_seen": bool(encoder_blocking_anomaly_seen),
        "encoder_transient_anomaly_seen": bool(encoder_transient_anomaly_seen),
        "encoder_reliability_health_seen": sorted(set(encoder_health_seen)),
        "encoder_reliability_trust_min": min(encoder_trust_seen) if encoder_trust_seen else None,
        "encoder_reliability_flags_seen": sorted(set(encoder_flags_seen)),
        "encoder_observation_context_seen": sorted(set(encoder_context_seen)),
        "distance_limit_triggered": bool(distance_limit_triggered),
        "target_distance_m": distance_target,
        "distance_target_reached": bool(distance_target_reached),
        "directed_distance_m": {
            "left": float(directed_distance_left),
            "right": float(directed_distance_right),
            "mean": float(directed_distance_mean),
        },
        "max_distance_observed_m": float(max_distance_observed_m),
        "stability": {
            "left": _stability_summary(
                stable_velocity_left,
                sign=sign,
                elapsed_s=stable_velocity_elapsed_s,
            ),
            "right": _stability_summary(
                stable_velocity_right,
                sign=sign,
                elapsed_s=stable_velocity_elapsed_s,
            ),
            "onset_left_s": onset_left_s,
            "onset_right_s": onset_right_s,
        },
        "end_state": str(stopped.get("state", "") or ""),
    }
    output.update(dict(sample_metadata or {}))
    if append_sample:
        _append_jsonl(sample_path, output)
    return output


def _fit_group(rows: Iterable[Dict[str, Any]], direction: str, side: str, speeds: List[float]) -> Dict[str, float]:
    sign = 1.0 if direction == "forward" else -1.0
    grouped: Dict[float, List[Tuple[float, float]]] = {speed: [] for speed in speeds}
    for row in rows:
        if row.get("direction") != direction:
            continue
        if (
            not row.get("direct_executor_observed", False)
            or not row.get("pi_disabled_observed", False)
            or row.get("pi_violation_seen", False)
            or row.get("faults")
            or row.get("encoder_blocking_anomaly_seen", False)
        ):
            continue
        target = float(row["target_speed_mps"])
        actual = sign * float((row.get("actual_mps") or {}).get(side, 0.0))
        pwm = abs(float((row.get("commanded_pwm") or {}).get(side, 0.0)))
        if target in grouped and actual > 0.002:
            grouped[target].append((actual, pwm))
    pairs: Dict[float, Tuple[float, float]] = {}
    for speed in speeds:
        values = grouped[speed]
        if values:
            pairs[speed] = (
                statistics.median(item[0] for item in values),
                statistics.median(item[1] for item in values),
            )
    if len(pairs) < 3:
        raise RuntimeError(f"insufficient_feedforward_points:{direction}:{side}")
    first_speed = min(pairs)
    proven_motion_floor_pwm = float(pairs[first_speed][1])
    out: Dict[str, float] = {}
    previous = 0.0
    for speed in speeds:
        actual, measured_pwm = pairs.get(speed, pairs[min(pairs, key=lambda key: abs(key - speed))])
        correction_ratio = max(0.80, min(1.25, float(speed) / max(0.005, float(actual))))
        value = float(measured_pwm) * (correction_ratio ** 0.70)
        value = max(proven_motion_floor_pwm, min(0.35, value))
        value = max(value, previous + (0.002 if previous else 0.0))
        out[_speed_key(speed)] = round(value, 4)
        previous = value
    return out


def _candidate_map(old_map: Dict[str, Any], rows: List[Dict[str, Any]], speeds: List[float]) -> Dict[str, Any]:
    candidate = {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "CANDIDATE",
        "hardware": old_map.get("hardware", "DFRobot KIT0085"),
        "calibration_state": "LIVE_DIRECT_PWM_FOUR_DIRECTION_CANDIDATE_2026_07_16",
        "calibration_evidence": "latest_motor_feedforward_calibration.json direct PWM acquisition",
        "interpolation": "linear",
        "curves": {},
    }
    for direction in ("forward", "reverse"):
        left = _fit_group(rows, direction, "left", speeds)
        right = _fit_group(rows, direction, "right", speeds)
        for side, fitted in (("left", left), ("right", right)):
            first_pwm = float(fitted[_speed_key(speeds[0])])
            candidate["curves"][f"{side}_{direction}"] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": first_pwm,
                "maintenance_pwm": first_pwm,
                "dead_zone_pwm": first_pwm,
                "points": [
                    {
                        "speed_mps": float(speed),
                        "pwm": float(fitted[_speed_key(speed)]),
                    }
                    for speed in speeds
                ],
            }
    return candidate


def _score(rows: List[Dict[str, Any]], speeds: List[float]) -> Dict[str, Any]:
    groups: Dict[str, List[float]] = {}
    errors: List[float] = []
    wrong_direction = 0
    for row in rows:
        target = abs(float(row["target_speed_mps"]))
        direction = str(row["direction"])
        sign = 1.0 if direction == "forward" else -1.0
        for side in ("left", "right"):
            actual = sign * float((row.get("actual_mps") or {}).get(side, 0.0))
            if actual <= 0.0:
                wrong_direction += 1
            error = abs(actual - target) / max(0.01, target)
            errors.append(error)
            groups.setdefault(f"{side}_{direction}", []).append(error)
    return {
        "median_relative_error": statistics.median(errors) if errors else math.inf,
        "mean_relative_error": statistics.mean(errors) if errors else math.inf,
        "max_group_median_relative_error": max(
            (statistics.median(values) for values in groups.values()), default=math.inf
        ),
        "group_median_relative_error": {
            key: statistics.median(values) for key, values in sorted(groups.items())
        },
        "wrong_direction_count": int(wrong_direction),
        "fault_count": sum(len(row.get("faults") or []) for row in rows),
        "encoder_anomaly_count": sum(
            1 for row in rows if row.get("encoder_blocking_anomaly_seen", False)
        ),
        "direct_executor_missing_count": sum(1 for row in rows if not row.get("direct_executor_observed", False)),
        "pi_violation_count": sum(
            1
            for row in rows
            if not row.get("pi_disabled_observed", False)
            or row.get("pi_violation_seen", False)
        ),
        "expected_speed_points": len(speeds),
    }


def _run_matrix(
    *,
    stage: str,
    repeats: int,
    speed_map: Dict[str, Any],
    speeds: List[float],
    args: argparse.Namespace,
    nonce: str,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    rows: List[Dict[str, Any]] = []
    pauses: List[Dict[str, Any]] = []
    invalid_attempts: List[Dict[str, Any]] = []
    phase_count = int(repeats) * len(speeds) * 2
    phase_index = 0
    for repeat in range(1, int(repeats) + 1):
        for target in speeds:
            for direction in ("forward", "reverse"):
                phase_index += 1
                left_pwm = _map_pwm(speed_map, direction, target, "left")
                right_pwm = _map_pwm(speed_map, direction, target, "right")
                print(
                    f"FF_CAL phase={phase_index}/{phase_count} stage={stage} repeat={repeat} "
                    f"direction={direction} target={target:.2f} pwm={left_pwm:.4f}/{right_pwm:.4f}",
                    flush=True,
                )
                result: Dict[str, Any] | None = None
                for attempt in (1, 2):
                    measured = _run_pulse(
                        stage=stage,
                        repeat=repeat,
                        target_speed=target,
                        direction=direction,
                        left_pwm=left_pwm,
                        right_pwm=right_pwm,
                        nonce=nonce,
                        token=args.token,
                        stabilization_s=args.stabilization_s,
                        measurement_s=args.measurement_s,
                        poll_s=args.poll_s,
                        max_phase_distance_m=args.max_phase_distance_m,
                        attempt=attempt,
                    )
                    if measured.get("distance_limit_triggered", False):
                        invalid_attempts.append(measured)
                        raise RuntimeError(
                            f"phase_distance_limit:{stage}:{repeat}:{direction}:{target:.2f}"
                        )
                    output_pwm = dict(measured.get("median_output_pwm") or {})
                    direct_output_valid = bool(
                        measured.get("direct_executor_observed", False)
                        and max(abs(float(output_pwm.get("left", 0.0))), abs(float(output_pwm.get("right", 0.0)))) >= 0.03
                    )
                    if direct_output_valid:
                        result = measured
                        break
                    invalid_attempts.append(measured)
                    if attempt >= 2:
                        raise RuntimeError(
                            f"direct_pwm_not_observed_after_retry:{stage}:{repeat}:{direction}:{target:.2f}"
                        )
                    print(
                        f"FF_CAL retry_pause={args.pause_s:.1f}s stage={stage} repeat={repeat} "
                        f"direction={direction} target={target:.2f}",
                        flush=True,
                    )
                    retry_pause = _run_pause(
                        args.pause_s,
                        compact=True,
                        label=f"retry_{stage}_{repeat}_{direction}_{target:.2f}",
                        token=args.token,
                        reset_pos_after_pause=True,
                        post_reset_ready_timeout_s=args.post_reset_ready_timeout_s,
                    )
                    pauses.append(retry_pause)
                    if not retry_pause.get("ok", False):
                        raise RuntimeError("retry_pause_reset_failed")
                    _wait_calibration_ready(
                        args.post_reset_ready_timeout_s,
                        token=args.token,
                    )
                if result is None:
                    raise RuntimeError("direct_pwm_result_missing")
                rows.append(result)
                is_last = phase_index >= phase_count
                if not is_last:
                    print(f"FF_CAL pause={args.pause_s:.1f}s after={direction}_{target:.2f}", flush=True)
                    pause = _run_pause(
                        args.pause_s,
                        compact=True,
                        label=f"{stage}_{repeat}_{direction}_{target:.2f}",
                        token=args.token,
                        reset_pos_after_pause=True,
                        post_reset_ready_timeout_s=args.post_reset_ready_timeout_s,
                    )
                    pauses.append(pause)
                    if not pause.get("ok", False):
                        raise RuntimeError(f"pause_reset_failed:{pause.get('error', '')}")
                    _wait_calibration_ready(
                        args.post_reset_ready_timeout_s,
                        token=args.token,
                    )
    return rows, pauses, invalid_attempts


def _adaptive_leg_distance_m(pwm: float, max_abs_pwm: float) -> float:
    """Use more corridor at faster PWM while staying inside one half of 4 m."""

    ratio = max(0.0, min(1.0, abs(float(pwm)) / max(0.01, float(max_abs_pwm))))
    return round(0.30 + (1.35 * ratio), 3)


def _nominal_speed_hint_mps(pwm: float, max_abs_pwm: float) -> float:
    ratio = max(0.0, min(1.0, abs(float(pwm)) / max(0.01, float(max_abs_pwm))))
    return max(0.08, min(1.0, 0.08 + 0.92 * ratio))


def _planned_shuttle_leg_distance_m(
    *,
    pwm: float,
    max_abs_pwm: float,
    target_distance_m: float | None,
) -> float:
    """Give either direction an adaptive leg when no pair target exists."""

    if target_distance_m is None:
        return _adaptive_leg_distance_m(pwm, max_abs_pwm)
    return max(0.02, abs(float(target_distance_m)))


def _shuttle_quality_rejections(
    row: Dict[str, Any],
    *,
    measurement_kind: str,
) -> List[str]:
    from tools.speed_map_calibration_analyzer import sample_rejection_reasons

    require_stable = measurement_kind == "stable_point"
    checked = dict(row)
    reasons: List[str] = []
    for side in ("left", "right"):
        reasons.extend(
            f"{side}:{reason}"
            for reason in sample_rejection_reasons(
                checked,
                side,
                require_stable=require_stable,
                require_encoder_reliable=require_stable,
            )
        )
    if not require_stable:
        unexpected_encoder_flags = set(
            str(flag)
            for flag in (checked.get("encoder_reliability_flags_seen") or [])
        ) - THRESHOLD_OUTCOME_ENCODER_FLAGS
        reasons.extend(
            f"encoder_flag:{flag}" for flag in sorted(unexpected_encoder_flags)
        )
        sign = 1.0 if str(checked.get("direction")) == "forward" else -1.0
        actual_mps = dict(checked.get("actual_mps") or {})
        directed_distance = dict(checked.get("directed_distance_m") or {})
        for side in ("left", "right"):
            stability = dict((checked.get("stability") or {}).get(side) or {})
            wrong_direction_samples = int(
                stability.get("wrong_direction_samples", 0) or 0
            )
            net_direction_not_wrong = (
                side in actual_mps
                and side in directed_distance
                and sign * _safe_float(actual_mps.get(side), 0.0) >= -0.003
                and _safe_float(directed_distance.get(side), 0.0) >= -0.003
            )
            if wrong_direction_samples != 0 and not net_direction_not_wrong:
                reasons.append(f"{side}:wrong_direction")
    return sorted(set(reasons))


def _recover_invalid_shuttle_attempt(
    *,
    invalid_record: Dict[str, Any],
    args: argparse.Namespace,
    label: str,
) -> Dict[str, Any]:
    """Stop, allow operator repositioning, reset pose, and wait for fresh truth."""

    print(
        "SPEED_MAP_SAMPLE_RETRY "
        f"label={label} attempt={invalid_record.get('attempt', 0)} "
        f"pause_s={float(args.pause_s):.1f}",
        flush=True,
    )
    _safe_stop_best_effort(args.token)
    recovery = _run_pause(
        float(args.pause_s),
        compact=True,
        label=f"invalid_{label}_{invalid_record.get('attempt', 0)}",
        token=args.token,
        reset_pos_after_pause=True,
        post_reset_ready_timeout_s=args.post_reset_ready_timeout_s,
    )
    invalid_record["retry_recovery"] = recovery
    if not bool(recovery.get("ok", False)):
        raise RuntimeError(
            f"sample_retry_recovery_failed:{label}:"
            f"{recovery.get('error', 'unknown_error')}"
        )
    _wait_calibration_ready(
        args.post_reset_ready_timeout_s,
        token=args.token,
    )
    return recovery


def _run_shuttle_leg(
    *,
    measurement_kind: str,
    sweep_direction: str,
    repeat: int,
    pwm: float,
    direction: str,
    target_distance_m: float | None,
    startup_pwm: float,
    startup_duration_s: float,
    args: argparse.Namespace,
    nonce: str,
    calibration_run_id: str,
    invalid_attempts: List[Dict[str, Any]],
) -> Dict[str, Any]:
    nominal_speed = _nominal_speed_hint_mps(pwm, args.max_abs_pwm)
    planned_distance = _planned_shuttle_leg_distance_m(
        pwm=pwm,
        max_abs_pwm=args.max_abs_pwm,
        target_distance_m=target_distance_m,
    )
    measurement_s = min(
        3.10,
        max(
            1.20,
            (
                float(planned_distance) / max(0.08, nominal_speed)
                if planned_distance is not None
                else 1.20
            )
            + 0.35,
        ),
    )
    maximum_distance = min(
        float(args.corridor_leg_max_m),
        max(
            0.25,
            (
                float(planned_distance) * 1.25 + 0.08
                if planned_distance is not None
                else 0.30
            ),
        ),
    )
    for attempt in range(1, int(args.max_sample_attempts) + 1):
        _wait_calibration_ready(
            args.post_reset_ready_timeout_s,
            token=args.token,
        )
        try:
            row = _run_pulse(
                stage="distance_shuttle_acquisition",
                repeat=repeat,
                target_speed=nominal_speed,
                direction=direction,
                left_pwm=pwm,
                right_pwm=pwm,
                nonce=nonce,
                token=args.token,
                stabilization_s=args.shuttle_stabilization_s,
                measurement_s=measurement_s,
                poll_s=args.poll_s,
                max_phase_distance_m=maximum_distance,
                attempt=attempt,
                sample_path=SHUTTLE_SAMPLES,
                sample_metadata={
                    "schema": "R2B4_SPEED_MAP_CALIBRATION_SAMPLE_V1",
                    "calibration_run_id": calibration_run_id,
                    "measurement_kind": measurement_kind,
                    "motion_geometry": "STRAIGHT",
                    "sweep_direction": sweep_direction,
                    "pwm_point": float(pwm),
                    "nominal_speed_hint_mps": float(nominal_speed),
                    "planned_distance_m": planned_distance,
                },
                startup_left_pwm=startup_pwm,
                startup_right_pwm=startup_pwm,
                startup_duration_s=startup_duration_s,
                target_distance_m=planned_distance,
                append_sample=False,
            )
        except RuntimeError as exc:
            invalid_record = {
                "calibration_run_id": calibration_run_id,
                "measurement_kind": measurement_kind,
                "sweep_direction": sweep_direction,
                "repeat": repeat,
                "attempt": attempt,
                "direction": direction,
                "pwm_point": float(pwm),
                "sample_accepted": False,
                "sample_rejection_reasons": [
                    f"measurement_exception:{exc}"
                ],
            }
            invalid_attempts.append(invalid_record)
            if attempt >= int(args.max_sample_attempts):
                raise RuntimeError(
                    "sample_retry_exhausted:"
                    f"{measurement_kind}:{sweep_direction}:{repeat}:"
                    f"{direction}:{pwm:.3f}:measurement_exception:{exc}"
                ) from exc
            _recover_invalid_shuttle_attempt(
                invalid_record=invalid_record,
                args=args,
                label=(
                    f"{measurement_kind}_{sweep_direction}_{repeat}_"
                    f"{direction}_{pwm:.3f}"
                ),
            )
            continue
        rejections = _shuttle_quality_rejections(
            row,
            measurement_kind=measurement_kind,
        )
        row["sample_rejection_reasons"] = rejections
        row["sample_accepted"] = not rejections
        if not rejections:
            return row
        invalid_attempts.append(row)
        if attempt >= int(args.max_sample_attempts):
            raise RuntimeError(
                "sample_retry_exhausted:"
                f"{measurement_kind}:{sweep_direction}:{repeat}:"
                f"{direction}:{pwm:.3f}:{','.join(rejections)}"
            )
        _recover_invalid_shuttle_attempt(
            invalid_record=row,
            args=args,
            label=(
                f"{measurement_kind}_{sweep_direction}_{repeat}_"
                f"{direction}_{pwm:.3f}"
            ),
        )
    raise RuntimeError("sample_retry_loop_unreachable")


def _run_shuttle_pair_once(
    *,
    measurement_kind: str,
    sweep_direction: str,
    repeat: int,
    pwm: float,
    startup_pwm: float,
    startup_duration_s: float,
    args: argparse.Namespace,
    nonce: str,
    calibration_run_id: str,
    invalid_attempts: List[Dict[str, Any]],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    forward = _run_shuttle_leg(
        measurement_kind=measurement_kind,
        sweep_direction=sweep_direction,
        repeat=repeat,
        pwm=pwm,
        direction="forward",
        target_distance_m=None,
        startup_pwm=startup_pwm,
        startup_duration_s=startup_duration_s,
        args=args,
        nonce=nonce,
        calibration_run_id=calibration_run_id,
        invalid_attempts=invalid_attempts,
    )
    forward_distance = max(
        0.0,
        _safe_float((forward.get("directed_distance_m") or {}).get("mean"), 0.0),
    )
    reverse_target = forward_distance if forward_distance >= 0.02 else None
    reverse = _run_shuttle_leg(
        measurement_kind=measurement_kind,
        sweep_direction=sweep_direction,
        repeat=repeat,
        pwm=pwm,
        direction="reverse",
        target_distance_m=reverse_target,
        startup_pwm=startup_pwm,
        startup_duration_s=startup_duration_s,
        args=args,
        nonce=nonce,
        calibration_run_id=calibration_run_id,
        invalid_attempts=invalid_attempts,
    )
    reverse_distance = max(
        0.0,
        _safe_float((reverse.get("directed_distance_m") or {}).get("mean"), 0.0),
    )
    distance_error = abs(reverse_distance - forward_distance)
    distance_error_ratio = (
        distance_error / forward_distance if forward_distance >= 0.02 else 0.0
    )
    pair_id = (
        f"{calibration_run_id}:{measurement_kind}:{sweep_direction}:"
        f"{repeat}:{pwm:.5f}"
    )
    for row, role in ((forward, "outbound"), (reverse, "return")):
        row["shuttle_pair_id"] = pair_id
        row["shuttle_role"] = role
        row["outbound_encoder_distance_m"] = float(forward_distance)
        row["return_encoder_distance_m"] = float(reverse_distance)
        row["return_distance_error_m"] = float(distance_error)
        row["return_distance_error_ratio"] = float(distance_error_ratio)
        row["return_distance_match_required"] = False
        row["return_distance_outside_observation_band"] = bool(
            distance_error > float(args.return_distance_tolerance_m)
            and distance_error_ratio
            > float(args.return_distance_tolerance_ratio)
        )
    _append_jsonl(SHUTTLE_SAMPLES, forward)
    _append_jsonl(SHUTTLE_SAMPLES, reverse)
    return forward, reverse


def _run_shuttle_pair(
    *,
    measurement_kind: str,
    sweep_direction: str,
    repeat: int,
    pwm: float,
    startup_pwm: float,
    startup_duration_s: float,
    args: argparse.Namespace,
    nonce: str,
    calibration_run_id: str,
    invalid_attempts: List[Dict[str, Any]],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    return _run_shuttle_pair_once(
        measurement_kind=measurement_kind,
        sweep_direction=sweep_direction,
        repeat=repeat,
        pwm=pwm,
        startup_pwm=startup_pwm,
        startup_duration_s=startup_duration_s,
        args=args,
        nonce=nonce,
        calibration_run_id=calibration_run_id,
        invalid_attempts=invalid_attempts,
    )


def _shuttle_pair_schedule(
    args: argparse.Namespace,
    *,
    max_abs_pwm: float,
) -> List[Dict[str, Any]]:
    schedule: List[Dict[str, Any]] = []
    stages = (
        (
            "startup_threshold",
            "ascending",
            tuple(point for point in STARTUP_PWM_POINTS if point <= max_abs_pwm),
            int(args.threshold_repeats),
            0.0,
        ),
        (
            "maintenance_threshold",
            "descending",
            tuple(point for point in MAINTENANCE_PWM_POINTS if point <= max_abs_pwm),
            int(args.threshold_repeats),
            float(args.threshold_startup_duration_s),
        ),
    )
    for measurement_kind, sweep, points, repeats, startup_duration in stages:
        for repeat in range(1, repeats + 1):
            for pwm in points:
                schedule.append(
                    {
                        "measurement_kind": measurement_kind,
                        "sweep_direction": sweep,
                        "repeat": int(repeat),
                        "repeat_count": int(repeats),
                        "pwm": float(pwm),
                        "startup_duration_s": float(startup_duration),
                    }
                )

    stable_points = tuple(
        point for point in STABLE_PWM_POINTS if point <= max_abs_pwm
    )
    for sweep, points in (
        ("ascending", stable_points),
        ("descending", tuple(reversed(stable_points))),
    ):
        for repeat in range(1, int(args.stable_repeats) + 1):
            for pwm in points:
                schedule.append(
                    {
                        "measurement_kind": "stable_point",
                        "sweep_direction": sweep,
                        "repeat": int(repeat),
                        "repeat_count": int(args.stable_repeats),
                        "pwm": float(pwm),
                        "startup_duration_s": float(
                            args.threshold_startup_duration_s
                        ),
                    }
                )
    return schedule


def _supplemental_shuttle_schedule(
    args: argparse.Namespace,
) -> List[Dict[str, Any]]:
    """Fixed evidence-only plans selected before each live measurement."""

    schedule: List[Dict[str, Any]] = []
    for repeat in range(4, 7):
        schedule.append(
            {
                "measurement_kind": "maintenance_threshold",
                "sweep_direction": "descending",
                "repeat": int(repeat),
                "repeat_count": 6,
                "pwm": 0.10,
                "startup_duration_s": float(args.threshold_startup_duration_s),
            }
        )
    for sweep in ("ascending", "descending"):
        for repeat in range(3, 5):
            schedule.append(
                {
                    "measurement_kind": "stable_point",
                    "sweep_direction": sweep,
                    "repeat": int(repeat),
                    "repeat_count": 4,
                    "pwm": 0.64,
                    "startup_duration_s": float(
                        args.threshold_startup_duration_s
                    ),
                }
            )
    if str(
        getattr(args, "range_confirmation_analysis", "") or ""
    ).strip():
        for sweep in ("ascending", "descending"):
            for repeat in range(5, 9):
                schedule.append(
                    {
                        "measurement_kind": "stable_point",
                        "sweep_direction": sweep,
                        "repeat": int(repeat),
                        "repeat_count": 8,
                        "pwm": 0.64,
                        "startup_duration_s": float(
                            args.threshold_startup_duration_s
                        ),
                    }
                )
    return schedule


def _validated_resume_prefix(
    *,
    rows: List[Dict[str, Any]],
    result: Dict[str, Any],
    schedule: List[Dict[str, Any]],
) -> Tuple[str, int]:
    if str(result.get("schema", "")) != "R2B4_SPEED_MAP_CALIBRATION_ACQUISITION_V1":
        raise ValueError("resume_result_schema_mismatch")
    calibration_run_id = str(result.get("calibration_run_id", "") or "")
    if not calibration_run_id:
        raise ValueError("resume_calibration_run_id_missing")
    if len(rows) % 2:
        raise ValueError("resume_rows_not_complete_pairs")
    completed_pairs = len(rows) // 2
    if completed_pairs > len(schedule):
        raise ValueError("resume_rows_exceed_schedule")
    for pair_index in range(completed_pairs):
        expected = schedule[pair_index]
        outbound, returned = rows[pair_index * 2 : pair_index * 2 + 2]
        pair_id = str(outbound.get("shuttle_pair_id", "") or "")
        if (
            not pair_id
            or pair_id != str(returned.get("shuttle_pair_id", "") or "")
            or outbound.get("shuttle_role") != "outbound"
            or returned.get("shuttle_role") != "return"
        ):
            raise ValueError(f"resume_pair_contract_mismatch:{pair_index}")
        for row in (outbound, returned):
            if not bool(row.get("sample_accepted", False)):
                raise ValueError(f"resume_unaccepted_row:{pair_index}")
            if str(row.get("calibration_run_id", "") or "") != calibration_run_id:
                raise ValueError(f"resume_run_id_mismatch:{pair_index}")
            if str(row.get("measurement_kind", "")) != str(
                expected["measurement_kind"]
            ):
                raise ValueError(f"resume_measurement_kind_mismatch:{pair_index}")
            if str(row.get("sweep_direction", "")) != str(
                expected["sweep_direction"]
            ):
                raise ValueError(f"resume_sweep_mismatch:{pair_index}")
            if int(row.get("repeat", 0) or 0) != int(expected["repeat"]):
                raise ValueError(f"resume_repeat_mismatch:{pair_index}")
            if abs(
                _safe_float(row.get("pwm_point"), -1.0)
                - float(expected["pwm"])
            ) > 1e-9:
                raise ValueError(f"resume_pwm_mismatch:{pair_index}")
        if str(outbound.get("direction", "")) != "forward":
            raise ValueError(f"resume_outbound_direction_mismatch:{pair_index}")
        if str(returned.get("direction", "")) != "reverse":
            raise ValueError(f"resume_return_direction_mismatch:{pair_index}")
    if int(result.get("accepted_row_count", -1) or 0) != len(rows):
        raise ValueError("resume_result_row_count_mismatch")
    return calibration_run_id, completed_pairs


def _validate_resume_pwm_ceiling(
    *,
    rows: List[Dict[str, Any]],
    previous_max_abs_pwm: float,
    current_max_abs_pwm: float,
    allow_lower: bool,
) -> bool:
    previous = abs(float(previous_max_abs_pwm))
    current = abs(float(current_max_abs_pwm))
    if abs(previous - current) <= 1e-9:
        return False
    if not allow_lower or current >= previous:
        raise ValueError("resume_contract_mismatch:max_abs_pwm")
    for row_index, row in enumerate(rows):
        commanded = dict(row.get("commanded_pwm") or {})
        startup = dict(row.get("startup_commanded_pwm") or {})
        observed = (
            abs(_safe_float(row.get("pwm_point"), 0.0)),
            abs(_safe_float(commanded.get("left"), 0.0)),
            abs(_safe_float(commanded.get("right"), 0.0)),
            abs(_safe_float(startup.get("left"), 0.0)),
            abs(_safe_float(startup.get("right"), 0.0)),
        )
        if max(observed) > current + 1e-9:
            raise ValueError(
                f"resume_rows_exceed_reduced_max_abs_pwm:{row_index}"
            )
    return True


def _load_shuttle_resume(
    args: argparse.Namespace,
    *,
    active_map: Dict[str, Any],
    schedule: List[Dict[str, Any]],
) -> Dict[str, Any]:
    samples_arg = str(getattr(args, "resume_from_samples", "") or "").strip()
    result_arg = str(getattr(args, "resume_from_result", "") or "").strip()
    if not samples_arg and not result_arg:
        return {}
    if not samples_arg or not result_arg:
        raise ValueError("resume_requires_samples_and_result")
    samples_path = Path(samples_arg)
    result_path = Path(result_arg)
    if not samples_path.is_absolute():
        samples_path = PROJECT_ROOT / samples_path
    if not result_path.is_absolute():
        result_path = PROJECT_ROOT / result_path
    if not samples_path.is_file() or not result_path.is_file():
        raise ValueError("resume_artifact_missing")
    backup_path = samples_path.parent / "speed_map_before_speed_map_calibration.json"
    if not backup_path.is_file():
        raise ValueError("resume_map_backup_missing")
    if json.loads(backup_path.read_text(encoding="utf-8")) != active_map:
        raise ValueError("resume_active_map_mismatch")

    rows: List[Dict[str, Any]] = []
    for line_number, raw in enumerate(
        samples_path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        try:
            row = json.loads(raw)
        except Exception as exc:
            raise ValueError(f"resume_sample_json_invalid:{line_number}") from exc
        if not isinstance(row, dict):
            raise ValueError(f"resume_sample_not_object:{line_number}")
        rows.append(row)
    result = json.loads(result_path.read_text(encoding="utf-8"))
    expected_contract = {
        "threshold_repeats": int(args.threshold_repeats),
        "stable_repeats_per_sweep": int(args.stable_repeats),
        "expected_row_count": len(schedule) * 2,
    }
    for key, expected in expected_contract.items():
        if int(result.get(key, -1) or 0) != expected:
            raise ValueError(f"resume_contract_mismatch:{key}")
    previous_max_abs_pwm = _safe_float(result.get("max_abs_pwm"), -1.0)
    ceiling_reduced = _validate_resume_pwm_ceiling(
        rows=rows,
        previous_max_abs_pwm=previous_max_abs_pwm,
        current_max_abs_pwm=abs(float(args.max_abs_pwm)),
        allow_lower=bool(
            getattr(args, "allow_resume_lower_max_abs_pwm", False)
        ),
    )
    if abs(
        _safe_float(result.get("corridor_leg_max_m"), -1.0)
        - float(args.corridor_leg_max_m)
    ) > 1e-9:
        raise ValueError("resume_contract_mismatch:corridor_leg_max_m")
    calibration_run_id, completed_pairs = _validated_resume_prefix(
        rows=rows,
        result=result,
        schedule=schedule,
    )
    return {
        "rows": rows,
        "result": result,
        "calibration_run_id": calibration_run_id,
        "completed_pairs": int(completed_pairs),
        "samples_path": str(samples_path),
        "result_path": str(result_path),
        "previous_max_abs_pwm": float(previous_max_abs_pwm),
        "ceiling_reduced": bool(ceiling_reduced),
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _resolved_input_path(value: str, *, label: str) -> Path:
    path = Path(str(value or "").strip())
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    if not path.is_file():
        raise ValueError(f"{label}_missing")
    return path


def _read_jsonl_rows(path: Path, *, label: str) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for line_number, raw in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        try:
            row = json.loads(raw)
        except Exception as exc:
            raise ValueError(
                f"{label}_json_invalid:{line_number}"
            ) from exc
        if not isinstance(row, dict):
            raise ValueError(f"{label}_row_not_object:{line_number}")
        rows.append(row)
    return rows


def _validate_supplement_tail(
    *,
    rows: List[Dict[str, Any]],
    calibration_run_id: str,
    schedule: List[Dict[str, Any]],
) -> int:
    fake_result = {
        "schema": "R2B4_SPEED_MAP_CALIBRATION_ACQUISITION_V1",
        "calibration_run_id": str(calibration_run_id),
        "accepted_row_count": len(rows),
    }
    _, completed_pairs = _validated_resume_prefix(
        rows=rows,
        result=fake_result,
        schedule=schedule,
    )
    return int(completed_pairs)


def _load_shuttle_supplement(
    args: argparse.Namespace,
    *,
    active_map: Dict[str, Any],
    max_abs_pwm: float,
    supplement_schedule: List[Dict[str, Any]],
) -> Dict[str, Any]:
    base_samples_path = _resolved_input_path(
        args.supplement_base_samples,
        label="supplement_base_samples",
    )
    base_result_path = _resolved_input_path(
        args.supplement_base_result,
        label="supplement_base_result",
    )
    base_backup_path = (
        base_samples_path.parent / "speed_map_before_speed_map_calibration.json"
    )
    if not base_backup_path.is_file():
        raise ValueError("supplement_base_map_backup_missing")
    if json.loads(base_backup_path.read_text(encoding="utf-8")) != active_map:
        raise ValueError("supplement_active_map_mismatch")

    base_rows = _read_jsonl_rows(
        base_samples_path,
        label="supplement_base_samples",
    )
    base_result = json.loads(base_result_path.read_text(encoding="utf-8"))
    base_schedule = _shuttle_pair_schedule(args, max_abs_pwm=max_abs_pwm)
    expected_contract = {
        "threshold_repeats": int(args.threshold_repeats),
        "stable_repeats_per_sweep": int(args.stable_repeats),
        "expected_row_count": len(base_schedule) * 2,
    }
    for key, expected in expected_contract.items():
        if int(base_result.get(key, -1) or 0) != expected:
            raise ValueError(f"supplement_base_contract_mismatch:{key}")
    if (
        str(base_result.get("status", "")) != "PASS"
        or not bool(base_result.get("success", False))
    ):
        raise ValueError("supplement_base_acquisition_not_pass")
    if abs(
        _safe_float(base_result.get("max_abs_pwm"), -1.0) - max_abs_pwm
    ) > 1e-9:
        raise ValueError("supplement_base_contract_mismatch:max_abs_pwm")
    calibration_run_id, base_completed_pairs = _validated_resume_prefix(
        rows=base_rows,
        result=base_result,
        schedule=base_schedule,
    )
    if base_completed_pairs != len(base_schedule):
        raise ValueError("supplement_base_acquisition_incomplete")

    base_samples_sha256 = _sha256(base_samples_path)
    base_result_sha256 = _sha256(base_result_path)
    resume_samples_arg = str(args.resume_from_samples or "").strip()
    resume_result_arg = str(args.resume_from_result or "").strip()
    if not resume_samples_arg and not resume_result_arg:
        return {
            "rows": base_rows,
            "result": {},
            "calibration_run_id": calibration_run_id,
            "completed_pairs": 0,
            "base_row_count": len(base_rows),
            "base_samples_path": str(base_samples_path),
            "base_result_path": str(base_result_path),
            "base_samples_sha256": base_samples_sha256,
            "base_result_sha256": base_result_sha256,
        }
    if not resume_samples_arg or not resume_result_arg:
        raise ValueError("supplement_resume_requires_samples_and_result")

    resume_samples_path = _resolved_input_path(
        resume_samples_arg,
        label="supplement_resume_samples",
    )
    resume_result_path = _resolved_input_path(
        resume_result_arg,
        label="supplement_resume_result",
    )
    resume_backup_path = (
        resume_samples_path.parent / "speed_map_before_speed_map_calibration.json"
    )
    if not resume_backup_path.is_file():
        raise ValueError("supplement_resume_map_backup_missing")
    if json.loads(resume_backup_path.read_text(encoding="utf-8")) != active_map:
        raise ValueError("supplement_resume_active_map_mismatch")
    resume_rows = _read_jsonl_rows(
        resume_samples_path,
        label="supplement_resume_samples",
    )
    resume_result = json.loads(resume_result_path.read_text(encoding="utf-8"))
    if (
        str(resume_result.get("schema", ""))
        != "R2B4_SPEED_MAP_CALIBRATION_SUPPLEMENT_V1"
    ):
        raise ValueError("supplement_resume_result_schema_mismatch")
    if str(resume_result.get("calibration_run_id", "")) != calibration_run_id:
        raise ValueError("supplement_resume_run_id_mismatch")
    if (
        str(resume_result.get("base_samples_sha256", ""))
        != base_samples_sha256
        or str(resume_result.get("base_result_sha256", ""))
        != base_result_sha256
    ):
        raise ValueError("supplement_resume_base_hash_mismatch")
    if resume_rows[: len(base_rows)] != base_rows:
        raise ValueError("supplement_resume_base_rows_mismatch")
    tail = resume_rows[len(base_rows) :]
    completed_pairs = _validate_supplement_tail(
        rows=tail,
        calibration_run_id=calibration_run_id,
        schedule=supplement_schedule,
    )
    if int(resume_result.get("accepted_row_count", -1) or 0) != len(
        resume_rows
    ):
        raise ValueError("supplement_resume_row_count_mismatch")
    return {
        "rows": resume_rows,
        "result": resume_result,
        "calibration_run_id": calibration_run_id,
        "completed_pairs": completed_pairs,
        "base_row_count": len(base_rows),
        "base_samples_path": str(base_samples_path),
        "base_result_path": str(base_result_path),
        "base_samples_sha256": base_samples_sha256,
        "base_result_sha256": base_result_sha256,
        "resume_samples_path": str(resume_samples_path),
        "resume_result_path": str(resume_result_path),
    }


def _validate_range_confirmation_trigger(
    args: argparse.Namespace,
    *,
    supplement: Dict[str, Any],
) -> Dict[str, Any]:
    analysis_value = str(
        getattr(args, "range_confirmation_analysis", "") or ""
    ).strip()
    if not analysis_value:
        return {}
    analysis_path = _resolved_input_path(
        analysis_value,
        label="range_confirmation_analysis",
    )
    analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
    if (
        str(analysis.get("schema", ""))
        != "R2B4_SPEED_MAP_CALIBRATION_ANALYSIS_V1"
    ):
        raise ValueError("range_confirmation_analysis_schema_mismatch")
    if (
        str(analysis.get("status", "")) != "FAIL"
        or bool(analysis.get("success", True))
        or bool(analysis.get("candidate_qualified", True))
    ):
        raise ValueError("range_confirmation_requires_failed_analysis")
    if list(analysis.get("failed_gates") or []) != [
        "speed_range_and_anchor_coverage"
    ]:
        raise ValueError("range_confirmation_analysis_failure_scope_mismatch")
    if str(analysis.get("calibration_run_id", "")) != str(
        supplement.get("calibration_run_id", "")
    ):
        raise ValueError("range_confirmation_analysis_run_id_mismatch")
    common_max = _safe_float(analysis.get("common_max_speed_mps"), math.inf)
    minimum = _safe_float(
        analysis.get("minimum_common_coverage_mps"),
        -math.inf,
    )
    if not common_max < minimum:
        raise ValueError("range_confirmation_analysis_range_not_failed")
    source_value = str(
        (analysis.get("artifacts") or {}).get("source", "") or ""
    ).strip()
    source_path = _resolved_input_path(
        source_value,
        label="range_confirmation_source_samples",
    )
    previous_result = dict(supplement.get("result") or {})
    completed_pairs = int(supplement.get("completed_pairs", 0) or 0)
    initial_plan_pairs = 7
    if completed_pairs < initial_plan_pairs:
        raise ValueError("range_confirmation_initial_supplement_incomplete")
    if completed_pairs == initial_plan_pairs:
        resume_samples_path = Path(
            str(supplement.get("resume_samples_path", "") or "")
        )
        if (
            not resume_samples_path.is_file()
            or _sha256(resume_samples_path) != _sha256(source_path)
        ):
            raise ValueError("range_confirmation_analysis_source_mismatch")
        if (
            str(previous_result.get("status", "")) != "PASS"
            or not bool(previous_result.get("success", False))
            or int(previous_result.get("supplemental_row_count", -1) or 0)
            != initial_plan_pairs * 2
        ):
            raise ValueError("range_confirmation_initial_supplement_not_pass")
    else:
        if (
            str(
                previous_result.get(
                    "range_confirmation_analysis_sha256",
                    "",
                )
            )
            != _sha256(analysis_path)
            or str(
                previous_result.get(
                    "range_confirmation_source_samples_sha256",
                    "",
                )
            )
            != _sha256(source_path)
        ):
            raise ValueError("range_confirmation_resume_trigger_mismatch")
    return {
        "analysis_path": str(analysis_path),
        "analysis_sha256": _sha256(analysis_path),
        "source_samples_path": str(source_path),
        "source_samples_sha256": _sha256(source_path),
        "common_max_speed_mps": common_max,
        "minimum_common_coverage_mps": minimum,
    }


def run_shuttle_supplement(args: argparse.Namespace) -> Dict[str, Any]:
    """Append fixed, gated evidence without mutating the base acquisition."""

    ensure_agent_system_prompt_loaded()
    max_abs_pwm = abs(float(args.max_abs_pwm))
    if abs(max_abs_pwm - 0.64) > 1e-9:
        raise ValueError("supplement_max_abs_pwm_must_be_0.64")
    if float(args.corridor_leg_max_m) > 1.80:
        raise ValueError("max_corridor_leg_exceeds_4m_shuttle_contract")
    active_map = json.loads(SPEED_MAP_PATH.read_text(encoding="utf-8"))
    supplement_schedule = _supplemental_shuttle_schedule(args)
    supplement = _load_shuttle_supplement(
        args,
        active_map=active_map,
        max_abs_pwm=max_abs_pwm,
        supplement_schedule=supplement_schedule,
    )
    range_confirmation = _validate_range_confirmation_trigger(
        args,
        supplement=supplement,
    )
    _write_json_atomic(SHUTTLE_BACKUP, active_map)
    SHUTTLE_SAMPLES.parent.mkdir(parents=True, exist_ok=True)
    accepted_rows = list(supplement["rows"])
    SHUTTLE_SAMPLES.write_text(
        "".join(
            json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n"
            for row in accepted_rows
        ),
        encoding="utf-8",
    )
    previous_result = dict(supplement.get("result") or {})
    invalid_attempts: List[Dict[str, Any]] = list(
        previous_result.get("invalid_attempts") or []
    )
    calibration_run_id = str(supplement["calibration_run_id"])
    base_row_count = int(supplement["base_row_count"])
    completed_pairs = int(supplement.get("completed_pairs", 0) or 0)
    nonce = uuid.uuid4().hex
    _write_json_atomic(
        ARM_PATH,
        {
            "purpose": "motor_feedforward_calibration",
            "nonce": nonce,
            "issued_at": time.time(),
            "expires_at": time.time()
            + max(1800.0, float(args.arm_duration_s)),
            "max_abs_pwm": max_abs_pwm,
        },
    )
    error = ""
    try:
        _safe_stop_best_effort(args.token)
        initial = _wait_measurement_ready_after_reset(
            args.post_reset_ready_timeout_s,
            stable_samples=5,
        )
        if not initial.get("ok", False):
            raise RuntimeError("initial_measurement_not_ready")
        _wait_calibration_ready(
            args.post_reset_ready_timeout_s,
            token=args.token,
        )
        if completed_pairs:
            print(
                "SPEED_MAP_SUPPLEMENT_RESUME "
                f"run_id={calibration_run_id} "
                f"pairs={completed_pairs}/{len(supplement_schedule)}",
                flush=True,
            )
        for pair_index, item in enumerate(supplement_schedule):
            if pair_index < completed_pairs:
                continue
            measurement_kind = str(item["measurement_kind"])
            sweep = str(item["sweep_direction"])
            repeat = int(item["repeat"])
            pwm = float(item["pwm"])
            print(
                "SPEED_MAP_SUPPLEMENT "
                f"kind={measurement_kind} sweep={sweep} "
                f"repeat={repeat}/{item['repeat_count']} pwm={pwm:.3f}",
                flush=True,
            )
            startup_pwm = (
                pwm
                if measurement_kind == "startup_threshold"
                else max(0.30, pwm)
            )
            accepted_rows.extend(
                _run_shuttle_pair(
                    measurement_kind=measurement_kind,
                    sweep_direction=sweep,
                    repeat=repeat,
                    pwm=pwm,
                    startup_pwm=min(max_abs_pwm, startup_pwm),
                    startup_duration_s=float(item["startup_duration_s"]),
                    args=args,
                    nonce=nonce,
                    calibration_run_id=calibration_run_id,
                    invalid_attempts=invalid_attempts,
                )
            )
    except Exception as exc:
        error = str(exc)
    finally:
        _safe_stop_best_effort(args.token)
        try:
            ARM_PATH.unlink()
        except FileNotFoundError:
            pass

    supplemental_row_count = len(accepted_rows) - base_row_count
    expected_supplemental_rows = len(supplement_schedule) * 2
    completed = bool(
        not error and supplemental_row_count == expected_supplemental_rows
    )
    result = {
        "schema": "R2B4_SPEED_MAP_CALIBRATION_SUPPLEMENT_V1",
        "test_name": "speed_map_calibration_supplement_live",
        "status": "PASS" if completed else "FAIL",
        "success": completed,
        "calibration_run_id": calibration_run_id,
        "started_at_epoch_s": _safe_float(
            previous_result.get("started_at_epoch_s"),
            time.time(),
        ),
        "completed_at_epoch_s": time.time(),
        "measurement_role": (
            "TARGETED_RANGE_CONFIRMATION_SUPPLEMENT_ONLY"
            if range_confirmation
            else "TARGETED_EVIDENCE_SUPPLEMENT_ONLY"
        ),
        "candidate_built": False,
        "active_map_mutated": False,
        "base_samples": str(supplement["base_samples_path"]),
        "base_result": str(supplement["base_result_path"]),
        "base_samples_sha256": str(supplement["base_samples_sha256"]),
        "base_result_sha256": str(supplement["base_result_sha256"]),
        "base_row_count": base_row_count,
        "accepted_row_count": len(accepted_rows),
        "supplemental_row_count": supplemental_row_count,
        "expected_supplemental_row_count": expected_supplemental_rows,
        "supplement_plan": supplement_schedule,
        "completed_supplement_pair_count": supplemental_row_count // 2,
        "range_confirmation": bool(range_confirmation),
        "range_confirmation_analysis": str(
            range_confirmation.get("analysis_path", "")
        ),
        "range_confirmation_analysis_sha256": str(
            range_confirmation.get("analysis_sha256", "")
        ),
        "range_confirmation_source_samples": str(
            range_confirmation.get("source_samples_path", "")
        ),
        "range_confirmation_source_samples_sha256": str(
            range_confirmation.get("source_samples_sha256", "")
        ),
        "range_confirmation_trigger_common_max_speed_mps": (
            range_confirmation.get("common_max_speed_mps")
        ),
        "range_confirmation_minimum_common_coverage_mps": (
            range_confirmation.get("minimum_common_coverage_mps")
        ),
        "max_abs_pwm": max_abs_pwm,
        "runtime_pwm_cap_remains_active": True,
        "corridor_leg_max_m": float(args.corridor_leg_max_m),
        "invalid_attempt_count": len(invalid_attempts),
        "invalid_attempts": invalid_attempts,
        "automatic_remeasurement": True,
        "resumed": bool(previous_result),
        "resume_from_samples": str(
            supplement.get("resume_samples_path", "")
        ),
        "resume_from_result": str(
            supplement.get("resume_result_path", "")
        ),
        "error": error,
        "artifacts": {
            "result": str(
                SHUTTLE_SUPPLEMENT_RESULT.relative_to(PROJECT_ROOT)
            ),
            "samples": str(SHUTTLE_SAMPLES.relative_to(PROJECT_ROOT)),
            "rollback_backup": str(SHUTTLE_BACKUP.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(SHUTTLE_SUPPLEMENT_RESULT, result)
    return result


def run_shuttle_acquisition(args: argparse.Namespace) -> Dict[str, Any]:
    """Acquire measurements only; candidate fitting is owned by the analyzer."""

    ensure_agent_system_prompt_loaded()
    max_abs_pwm = abs(float(args.max_abs_pwm))
    if not 0.35 <= max_abs_pwm <= 0.90:
        raise ValueError("max_abs_pwm_out_of_bounds")
    if float(args.corridor_leg_max_m) > 1.80:
        raise ValueError("corridor_leg_max_exceeds_4m_shuttle_contract")

    active_map = json.loads(SPEED_MAP_PATH.read_text(encoding="utf-8"))
    schedule = _shuttle_pair_schedule(args, max_abs_pwm=max_abs_pwm)
    resume = _load_shuttle_resume(
        args,
        active_map=active_map,
        schedule=schedule,
    )
    _write_json_atomic(SHUTTLE_BACKUP, active_map)
    SHUTTLE_SAMPLES.parent.mkdir(parents=True, exist_ok=True)
    resumed_rows = list(resume.get("rows") or [])
    SHUTTLE_SAMPLES.write_text(
        "".join(
            json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n"
            for row in resumed_rows
        ),
        encoding="utf-8",
    )
    previous_result = dict(resume.get("result") or {})
    started_at_epoch_s = _safe_float(
        previous_result.get("started_at_epoch_s"),
        time.time(),
    )
    calibration_run_id = str(
        resume.get("calibration_run_id") or f"speed_map_{uuid.uuid4().hex}"
    )
    nonce = uuid.uuid4().hex
    _write_json_atomic(
        ARM_PATH,
        {
            "purpose": "motor_feedforward_calibration",
            "nonce": nonce,
            "issued_at": time.time(),
            "expires_at": time.time() + max(1800.0, float(args.arm_duration_s)),
            "max_abs_pwm": max_abs_pwm,
        },
    )
    accepted_rows: List[Dict[str, Any]] = resumed_rows
    invalid_attempts: List[Dict[str, Any]] = list(
        previous_result.get("invalid_attempts") or []
    )
    error = ""
    completed_pairs = int(resume.get("completed_pairs", 0) or 0)
    try:
        _safe_stop_best_effort(args.token)
        initial = _wait_measurement_ready_after_reset(
            args.post_reset_ready_timeout_s,
            stable_samples=5,
        )
        if not initial.get("ok", False):
            raise RuntimeError("initial_measurement_not_ready")
        _wait_calibration_ready(
            args.post_reset_ready_timeout_s,
            token=args.token,
        )
        if completed_pairs:
            print(
                "SPEED_MAP_RESUME "
                f"run_id={calibration_run_id} rows={len(accepted_rows)} "
                f"pairs={completed_pairs}/{len(schedule)}",
                flush=True,
            )
        for pair_index, item in enumerate(schedule):
            if pair_index < completed_pairs:
                continue
            measurement_kind = str(item["measurement_kind"])
            sweep = str(item["sweep_direction"])
            repeat = int(item["repeat"])
            pwm = float(item["pwm"])
            print(
                "SPEED_MAP_SHUTTLE "
                f"kind={measurement_kind} sweep={sweep} "
                f"repeat={repeat}/{item['repeat_count']} pwm={pwm:.3f}",
                flush=True,
            )
            startup_pwm = (
                pwm
                if measurement_kind == "startup_threshold"
                else max(0.30, pwm)
            )
            accepted_rows.extend(
                _run_shuttle_pair(
                    measurement_kind=measurement_kind,
                    sweep_direction=sweep,
                    repeat=repeat,
                    pwm=pwm,
                    startup_pwm=min(max_abs_pwm, startup_pwm),
                    startup_duration_s=float(item["startup_duration_s"]),
                    args=args,
                    nonce=nonce,
                    calibration_run_id=calibration_run_id,
                    invalid_attempts=invalid_attempts,
                )
            )
    except Exception as exc:
        error = str(exc)
    finally:
        _safe_stop_best_effort(args.token)
        try:
            ARM_PATH.unlink()
        except FileNotFoundError:
            pass

    expected_rows = (
        len(schedule) * 2
    )
    completed = bool(not error and len(accepted_rows) == expected_rows)
    result = {
        "schema": "R2B4_SPEED_MAP_CALIBRATION_ACQUISITION_V1",
        "test_name": "speed_map_calibration_acquisition_live",
        "status": "PASS" if completed else "FAIL",
        "success": completed,
        "calibration_run_id": calibration_run_id,
        "started_at_epoch_s": started_at_epoch_s,
        "completed_at_epoch_s": time.time(),
        "measurement_role": "ACQUISITION_ONLY",
        "candidate_built": False,
        "active_map_mutated": False,
        "motion_geometry": "STRAIGHT_DISTANCE_SHUTTLE",
        "return_control": "OUTBOUND_ENCODER_DISTANCE",
        "return_distance_mismatch_invalidates_sample": False,
        "time_equality_used": False,
        "threshold_repeats": int(args.threshold_repeats),
        "stable_repeats_per_sweep": int(args.stable_repeats),
        "sweep_directions": ["ascending", "descending"],
        "max_abs_pwm": max_abs_pwm,
        "runtime_pwm_cap_remains_active": True,
        "corridor_leg_max_m": float(args.corridor_leg_max_m),
        "accepted_row_count": len(accepted_rows),
        "expected_row_count": expected_rows,
        "invalid_attempt_count": len(invalid_attempts),
        "invalid_attempts": invalid_attempts,
        "automatic_remeasurement": True,
        "resumed": bool(resume),
        "resume_from_samples": str(resume.get("samples_path", "")),
        "resume_from_result": str(resume.get("result_path", "")),
        "resumed_row_count": len(resumed_rows),
        "resumed_complete_pair_count": int(completed_pairs),
        "resume_previous_max_abs_pwm": (
            float(resume.get("previous_max_abs_pwm"))
            if resume
            else max_abs_pwm
        ),
        "resume_ceiling_reduced": bool(resume.get("ceiling_reduced", False)),
        "max_sample_attempts": int(args.max_sample_attempts),
        "error": error,
        "artifacts": {
            "result": str(SHUTTLE_RESULT.relative_to(PROJECT_ROOT)),
            "samples": str(SHUTTLE_SAMPLES.relative_to(PROJECT_ROOT)),
            "rollback_backup": str(SHUTTLE_BACKUP.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(SHUTTLE_RESULT, result)
    return result


def run(args: argparse.Namespace) -> Dict[str, Any]:
    if bool(getattr(args, "shuttle_supplement", False)):
        return run_shuttle_supplement(args)
    if bool(getattr(args, "shuttle_acquisition", False)):
        return run_shuttle_acquisition(args)
    ensure_agent_system_prompt_loaded()
    speeds = sorted({abs(float(item)) for item in args.speeds if float(item) > 0.0})
    if speeds != list(DEFAULT_SPEEDS):
        raise ValueError("required_speed_points_are_0.15_0.20_0.25_0.30")
    old_map = json.loads(SPEED_MAP_PATH.read_text(encoding="utf-8"))
    _write_json_atomic(LATEST_BACKUP, old_map)
    reused_acquisition: List[Dict[str, Any]] = []
    reusable_samples_path = latest_artifact_path("latest_motor_feedforward_calibration_samples.jsonl")
    if bool(args.reuse_before_artifact) and reusable_samples_path.exists():
        reusable_speed_keys = {round(float(speed), 4) for speed in speeds}
        for raw in reusable_samples_path.read_text(encoding="utf-8").splitlines():
            try:
                row = json.loads(raw)
            except Exception:
                continue
            row_speed = round(_safe_float(row.get("target_speed_mps"), -1.0), 4)
            if (
                row.get("stage") == "before"
                and row.get("direct_executor_observed", False)
                and row_speed in reusable_speed_keys
            ):
                reused_acquisition.append(row)
        expected_before = int(args.repeats) * len(speeds) * 2
        if len(reused_acquisition) != expected_before:
            raise RuntimeError(
                f"reusable_before_sample_count_invalid:{len(reused_acquisition)}:{expected_before}"
            )
    LATEST_SAMPLES.parent.mkdir(parents=True, exist_ok=True)
    LATEST_SAMPLES.write_text("", encoding="utf-8")
    for row in reused_acquisition:
        _append_jsonl(LATEST_SAMPLES, row)

    nonce = uuid.uuid4().hex
    arm = {
        "purpose": "motor_feedforward_calibration",
        "nonce": nonce,
        "issued_at": time.time(),
        "expires_at": time.time() + max(900.0, float(args.arm_duration_s)),
        "max_abs_pwm": 0.35,
    }
    _write_json_atomic(ARM_PATH, arm)
    candidate: Dict[str, Any] = {}
    acquisition: List[Dict[str, Any]] = []
    validation: List[Dict[str, Any]] = []
    pauses: List[Dict[str, Any]] = []
    invalid_attempts: List[Dict[str, Any]] = []
    qualified = False
    error = ""
    try:
        _safe_stop_best_effort(args.token)
        initial = _wait_measurement_ready_after_reset(args.post_reset_ready_timeout_s, stable_samples=5)
        if not initial.get("ok", False):
            raise RuntimeError("initial_measurement_not_ready")
        _wait_calibration_ready(
            args.post_reset_ready_timeout_s,
            token=args.token,
        )
        if reused_acquisition:
            acquisition = list(reused_acquisition)
        else:
            acquisition, acquisition_pauses, acquisition_invalid = _run_matrix(
                stage="before",
                repeats=args.repeats,
                speed_map=old_map,
                speeds=speeds,
                args=args,
                nonce=nonce,
            )
            pauses.extend(acquisition_pauses)
            invalid_attempts.extend(acquisition_invalid)
        before_score = _score(acquisition, speeds)
        candidate = _candidate_map(old_map, acquisition, speeds)
        _write_json_atomic(LATEST_CANDIDATE, candidate)

        if args.pause_s > 0.0:
            print(f"FF_CAL pause={args.pause_s:.1f}s before=validation", flush=True)
            pause = _run_pause(
                args.pause_s,
                compact=True,
                label="candidate_written",
                token=args.token,
                reset_pos_after_pause=True,
                post_reset_ready_timeout_s=args.post_reset_ready_timeout_s,
            )
            pauses.append(pause)
            if not pause.get("ok", False):
                raise RuntimeError("candidate_validation_pause_failed")
            _wait_calibration_ready(
                args.post_reset_ready_timeout_s,
                token=args.token,
            )

        validation, validation_pauses, validation_invalid = _run_matrix(
            stage="after",
            repeats=args.validation_repeats,
            speed_map=candidate,
            speeds=speeds,
            args=args,
            nonce=nonce,
        )
        pauses.extend(validation_pauses)
        invalid_attempts.extend(validation_invalid)
        after_score = _score(validation, speeds)
        qualified = bool(
            after_score["median_relative_error"] + 0.02 < before_score["median_relative_error"]
            and after_score["mean_relative_error"] < before_score["mean_relative_error"]
            and after_score["max_group_median_relative_error"]
            <= before_score["max_group_median_relative_error"] + 0.10
            and after_score["wrong_direction_count"] == 0
            and after_score["fault_count"] == 0
            and after_score["encoder_anomaly_count"] == 0
            and after_score["direct_executor_missing_count"] == 0
            and after_score["pi_violation_count"] == 0
        )
    except Exception as exc:
        error = str(exc)
    finally:
        _safe_stop_best_effort(args.token)
        try:
            ARM_PATH.unlink()
        except FileNotFoundError:
            pass

    before_score = _score(acquisition, speeds)
    after_score = _score(validation, speeds)
    completed = bool(
        not error
        and len(acquisition) == int(args.repeats) * len(speeds) * 2
        and len(validation) == int(args.validation_repeats) * len(speeds) * 2
        and before_score.get("fault_count", 0) == 0
        and after_score.get("fault_count", 0) == 0
    )
    success = bool(completed)
    result = {
        "test_name": "motor_feedforward_calibration_live",
        "success": success,
        "status": "PASS" if success else "FAIL",
        "speed_points_mps": speeds,
        "repeats_before": int(args.repeats),
        "repeats_after": int(args.validation_repeats),
        "reused_before_artifact": bool(reused_acquisition),
        "pause_s": float(args.pause_s),
        "max_abs_pwm": 0.35,
        "max_phase_distance_m": float(args.max_phase_distance_m),
        "direct_pwm_measurement": True,
        "closed_wheel_loop_used_for_calibration": False,
        "before_score": before_score,
        "after_score": after_score,
        "candidate_qualified": bool(qualified),
        "candidate_kept": False,
        "candidate_activation_allowed": False,
        "calibration_outcome": "CANDIDATE_QUALIFIED" if qualified else ("CANDIDATE_REJECTED" if completed else "INCOMPLETE"),
        "candidate_map": candidate,
        "active_speed_map": old_map,
        "error": error,
        "phase_count": len(acquisition) + len(validation),
        "pause_count": len(pauses),
        "pause_failures": [item for item in pauses if not item.get("ok", False)],
        "invalid_attempt_count": len(invalid_attempts),
        "invalid_attempts": invalid_attempts,
        "artifacts": {
            "result": str(LATEST_RESULT.relative_to(PROJECT_ROOT)),
            "samples": str(LATEST_SAMPLES.relative_to(PROJECT_ROOT)),
            "backup": str(LATEST_BACKUP.relative_to(PROJECT_ROOT)),
            "candidate": str(LATEST_CANDIDATE.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(LATEST_RESULT, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shuttle-acquisition", action="store_true")
    parser.add_argument("--shuttle-supplement", action="store_true")
    parser.add_argument("--speeds", nargs="+", type=float, default=list(DEFAULT_SPEEDS))
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--validation-repeats", type=int, default=2)
    parser.add_argument("--stabilization-s", type=float, default=0.8)
    parser.add_argument("--measurement-s", type=float, default=2.0)
    parser.add_argument("--pause-s", type=float, default=10.0)
    parser.add_argument("--poll-s", type=float, default=0.08)
    parser.add_argument("--max-phase-distance-m", type=float, default=1.8)
    parser.add_argument("--max-abs-pwm", type=float, default=0.90)
    parser.add_argument("--corridor-leg-max-m", type=float, default=1.80)
    parser.add_argument("--threshold-repeats", type=int, default=3)
    parser.add_argument("--stable-repeats", type=int, default=2)
    parser.add_argument("--max-sample-attempts", type=int, default=3)
    parser.add_argument("--shuttle-stabilization-s", type=float, default=0.55)
    parser.add_argument("--threshold-startup-duration-s", type=float, default=0.28)
    parser.add_argument("--return-distance-tolerance-m", type=float, default=0.08)
    parser.add_argument("--return-distance-tolerance-ratio", type=float, default=0.08)
    parser.add_argument("--resume-from-samples", default="")
    parser.add_argument("--resume-from-result", default="")
    parser.add_argument("--supplement-base-samples", default="")
    parser.add_argument("--supplement-base-result", default="")
    parser.add_argument("--range-confirmation-analysis", default="")
    parser.add_argument("--allow-resume-lower-max-abs-pwm", action="store_true")
    parser.add_argument("--reuse-before-artifact", action="store_true")
    parser.add_argument("--post-reset-ready-timeout-s", type=float, default=90.0)
    parser.add_argument("--arm-duration-s", type=float, default=1800.0)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    result = run(args)
    if bool(args.shuttle_supplement):
        print(
            "SPEED_MAP_CAL_SUPPLEMENT|"
            f"status={result.get('status')}|"
            f"run_id={result.get('calibration_run_id', '')}|"
            f"supplemental={result.get('supplemental_row_count', 0)}/"
            f"{result.get('expected_supplemental_row_count', 0)}|"
            f"invalid={result.get('invalid_attempt_count', 0)}|"
            f"error={result.get('error', '')}",
            flush=True,
        )
        return 0 if result.get("success", False) else 1
    if bool(args.shuttle_acquisition):
        print(
            "SPEED_MAP_CAL_ACQUISITION|"
            f"status={result.get('status')}|run_id={result.get('calibration_run_id', '')}|"
            f"accepted={result.get('accepted_row_count', 0)}/"
            f"{result.get('expected_row_count', 0)}|"
            f"invalid={result.get('invalid_attempt_count', 0)}|"
            f"error={result.get('error', '')}",
            flush=True,
        )
        return 0 if result.get("success", False) else 1
    print(
        "MOTOR_FF_CAL|"
        f"status={result.get('status')}|qualified={result.get('candidate_qualified')}|"
        f"before={_safe_float((result.get('before_score') or {}).get('median_relative_error'), math.inf):.3f}|"
        f"after={_safe_float((result.get('after_score') or {}).get('median_relative_error'), math.inf):.3f}|"
        f"error={result.get('error', '')}",
        flush=True,
    )
    return 0 if result.get("success", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
