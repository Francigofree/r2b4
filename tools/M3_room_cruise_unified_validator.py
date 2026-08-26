#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""M3 Room Cruise unified full-system validator.

This tool is intentionally a validator/orchestrator, not a motion controller.
It uses the existing Room Cruise v2 command path and validates the UNIFIED
motion stack, peripherals, health, timing, software and hardware runtime
signals with the camera disabled.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
import time
from argparse import Namespace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from middleware.peripheral_usage import read_peripherals, set_peripheral_enabled  # noqa: E402
from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import M3_room_cruise_minoseg as m3_room  # noqa: E402
from tools import room_cruise_v2_live as cruise  # noqa: E402
from tools.M3_emberkovetes_mozgasminoseg import (  # noqa: E402
    _gate,
    _json_safe,
    _percentile,
    _ratio,
    _safe_float,
    _write_json,
    _write_jsonl,
)


RUNTIME_DIR = PROJECT_ROOT / "runtime"
LOGS_DIR = PROJECT_ROOT / "logs"
AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_unified_validator.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_unified_validator_summary.json"
PREFLIGHT_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_unified_validator_preflight.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M3_room_cruise_unified_validator_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_unified_validator_incident.json"

CANONICAL_CONTROL_MODE = "UNIFIED"
CONTINUOUS_ROOM_CRUISE_CONTRACT_ID = "R2B4_M3_CONTINUOUS_ROOM_CRUISE_V1"

DEFAULT_THRESHOLDS: Dict[str, float] = {
    "preflight_duration_s": 4.0,
    "preflight_poll_s": 0.15,
    "status_max_age_s": 2.5,
    "preflight_min_samples": 8,
    "preflight_min_front_m": 0.80,
    "runtime_min_front_m": 0.27,
    "idle_pwm_abs_max": 0.010,
    "idle_cmd_v_abs_max_mps": 0.006,
    "idle_cmd_omega_abs_max_rad_s": 0.020,
    "idle_actual_v_abs_max_mps": 0.040,
    "idle_actual_omega_abs_max_rad_s": 0.080,
    "lidar_confidence_p50_min": 0.35,
    "lidar_raw_scan_rate_p50_min_hz": 8.0,
    "lidar_matcher_latency_p95_max_ms": 110.0,
    "lidar_ekf_gap_p95_max_s": 0.55,
    "encoder_anomaly_persistent_segment_age_min_s": 1.0,
    "logger_queue_depth_max": 256,
    "logger_flush_p95_max_ms": 120.0,
    "loop_frequency_p10_min_hz": 40.0,
    "loop_below_45_ratio_max": 0.10,
    "loop_dt_p95_max_s": 0.030,
    "loop_budget_p95_max_ms": 35.0,
    "slow_tick_ratio_max": 0.25,
    "slow_io_ratio_max": 0.12,
    "cpu_p95_max_percent": 92.0,
    "cpu_temp_max_c": 75.0,
    "sd_latency_p95_max_ms": 150.0,
    "sd_latency_max_ms": 500.0,
    "continuous_active_duration_min_s": 58.0,
    "unjustified_stop_grace_s": 0.45,
    "command_loss_grace_s": 0.35,
    "command_loss_settle_s": 0.30,
}


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _summary(values: Iterable[Any]) -> Dict[str, Any]:
    vals = [float(v) for v in values if _finite(v)]
    if not vals:
        return {"n": 0, "min": None, "p50": None, "p90": None, "p95": None, "max": None}
    return {
        "n": len(vals),
        "min": _percentile(vals, 0.0),
        "p50": _percentile(vals, 0.50),
        "p90": _percentile(vals, 0.90),
        "p95": _percentile(vals, 0.95),
        "max": _percentile(vals, 1.0),
    }


def _max_bool_count(samples: Sequence[Dict[str, Any]], key: str, expected: bool) -> int:
    return sum(1 for sample in samples if bool(sample.get(key, False)) is bool(expected))


def _max_counter_delta(samples: Sequence[Dict[str, Any]], key: str) -> int:
    values = [int(sample.get(key, 0) or 0) for sample in samples]
    return max(0, max(values) - min(values)) if values else 0


def _sample_status(*, force: bool = True, phase: str = "") -> Dict[str, Any]:
    status = cruise._read_status(force=force)
    sample = cruise._sample(status) if status else {"ts": time.time()}
    sample["sample_phase"] = str(phase or "")
    try:
        sample["status_age_s"] = max(0.0, time.time() - (RUNTIME_DIR / "status.json").stat().st_mtime)
    except Exception:
        sample["status_age_s"] = None
    if "peripherals" not in sample:
        sample["peripherals"] = read_peripherals(runtime_dir=RUNTIME_DIR, use_cache=False)
    return sample


def _collect_preflight_samples(duration_s: float, poll_s: float) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    deadline = time.monotonic() + max(0.2, float(duration_s))
    while time.monotonic() <= deadline:
        samples.append(_sample_status(force=True, phase="preflight"))
        time.sleep(max(0.05, float(poll_s)))
    if not samples:
        samples.append(_sample_status(force=True, phase="preflight"))
    return samples


def _all_modes_unified(samples: Sequence[Dict[str, Any]]) -> bool:
    modes = []
    for sample in samples:
        for key in ("control_mode", "motion_state_mode", "motion_state_mode_raw"):
            value = str(sample.get(key, "") or "").strip().upper()
            if value:
                modes.append(value)
    return bool(modes) and all(value == CANONICAL_CONTROL_MODE for value in modes)


def _peripheral_ok(samples: Sequence[Dict[str, Any]], name: str, expected: bool = True) -> bool:
    seen = False
    for sample in samples:
        peripherals = dict(sample.get("peripherals") or {})
        if name in peripherals:
            seen = True
            if bool(peripherals.get(name)) != bool(expected):
                return False
    return seen


def analyze_preflight(samples: Sequence[Dict[str, Any]], thresholds: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})

    sample_list = [dict(sample or {}) for sample in samples]
    pwm_abs = [
        max(abs(_safe_float(s.get("pwm_left"), 0.0)), abs(_safe_float(s.get("pwm_right"), 0.0)))
        for s in sample_list
    ]
    cmd_v_abs = [abs(_safe_float(s.get("resolved_v", s.get("limited_v")), 0.0)) for s in sample_list]
    cmd_w_abs = [abs(_safe_float(s.get("resolved_omega", s.get("limited_omega")), 0.0)) for s in sample_list]
    actual_v_abs = [abs(_safe_float(s.get("actual_v"), 0.0)) for s in sample_list]
    actual_w_abs = [abs(_safe_float(s.get("actual_omega"), 0.0)) for s in sample_list]
    front_values = [_safe_float(s.get("front_m"), math.nan) for s in sample_list]
    lidar_conf = [_safe_float(s.get("lidar_pose_confidence"), math.nan) for s in sample_list]
    status_ages = [_safe_float(s.get("status_age_s"), math.nan) for s in sample_list]
    logger_depths = [_safe_float(s.get("logger_queue_depth"), math.nan) for s in sample_list]
    logger_flush = [_safe_float(s.get("logger_flush_duration_ms"), math.nan) for s in sample_list]

    states = sorted({str(s.get("state", "") or "") for s in sample_list if str(s.get("state", "") or "")})
    health_values = {
        "lidar": sorted({str(s.get("lidar_health", "") or "") for s in sample_list if str(s.get("lidar_health", "") or "")}),
        "encoder": sorted({str(s.get("encoder_reliability_health", "") or "") for s in sample_list if str(s.get("encoder_reliability_health", "") or "")}),
        "imu": sorted({str(s.get("imu_health", "") or "") for s in sample_list if str(s.get("imu_health", "") or "")}),
    }

    status_age_max = _summary(status_ages).get("max")
    front_min = _summary(front_values).get("min")
    lidar_conf_p50 = _summary(lidar_conf).get("p50")
    logger_depth_max = _summary(logger_depths).get("max")
    logger_flush_p95 = _summary(logger_flush).get("p95")

    gates: Dict[str, Dict[str, Any]] = {
        "preflight_samples": _gate(
            "PASS" if len(sample_list) >= int(limits["preflight_min_samples"]) else "FAIL",
            observed={"sample_count": len(sample_list)},
            requirement=f"at least {int(limits['preflight_min_samples'])} no-motion samples",
        ),
        "status_fresh": _gate(
            "PASS" if status_age_max is not None and float(status_age_max) <= limits["status_max_age_s"] else "FAIL",
            observed={"status_age_max_s": status_age_max},
            requirement=f"status age <= {limits['status_max_age_s']} s",
        ),
        "idle_no_motion": _gate(
            "PASS"
            if (
                all(str(s.get("state", "") or "").upper() == "IDLE" for s in sample_list)
                and (_summary(pwm_abs).get("max") or 0.0) <= limits["idle_pwm_abs_max"]
                and (_summary(cmd_v_abs).get("max") or 0.0) <= limits["idle_cmd_v_abs_max_mps"]
                and (_summary(cmd_w_abs).get("max") or 0.0) <= limits["idle_cmd_omega_abs_max_rad_s"]
                and (_summary(actual_v_abs).get("p95") or 0.0) <= limits["idle_actual_v_abs_max_mps"]
                and (_summary(actual_w_abs).get("p95") or 0.0) <= limits["idle_actual_omega_abs_max_rad_s"]
            )
            else "FAIL",
            observed={
                "states": states,
                "pwm_abs": _summary(pwm_abs),
                "cmd_v_abs": _summary(cmd_v_abs),
                "cmd_omega_abs": _summary(cmd_w_abs),
                "actual_v_abs": _summary(actual_v_abs),
                "actual_omega_abs": _summary(actual_w_abs),
            },
            requirement="IDLE state, zero command/PWM, no relevant measured creep",
        ),
        "camera_off": _gate(
            "PASS"
            if _peripheral_ok(sample_list, "camera", expected=False)
            and all(not bool(s.get("camera_enabled", False)) for s in sample_list)
            else "FAIL",
            observed={"camera_enabled_samples": _max_bool_count(sample_list, "camera_enabled", True)},
            requirement="camera disabled before room cruise validation",
        ),
        "required_peripherals": _gate(
            "PASS"
            if all(_peripheral_ok(sample_list, name, expected=True) for name in ("lidar", "encoder", "imu"))
            else "FAIL",
            observed={"last_peripherals": dict((sample_list[-1] if sample_list else {}).get("peripherals") or {})},
            requirement="lidar, encoder and atomic BNO055 imu enabled",
        ),
        "health_ok": _gate(
            "PASS"
            if (
                all(not values or "OK" in values for values in health_values.values())
                and all(bool(s.get("safety_allow", False)) for s in sample_list)
                and not any(bool(s.get("watchdog_stop_triggered", False)) for s in sample_list)
            )
            else "FAIL",
            observed={"health_values": health_values},
            requirement="sensor health OK, safety allow true, watchdog not stopped",
        ),
        "lidar_ready": _gate(
            "PASS"
            if (
                front_min is not None
                and float(front_min) >= limits["preflight_min_front_m"]
                and lidar_conf_p50 is not None
                and float(lidar_conf_p50) >= limits["lidar_confidence_p50_min"]
            )
            else "FAIL",
            observed={"front_min_m": front_min, "lidar_confidence_p50": lidar_conf_p50},
            requirement="front clearance and LIDAR confidence ready for movement",
        ),
        "logger_ready": _gate(
            "PASS"
            if (
                all(int(s.get("logger_dropped_messages", 0) or 0) == 0 for s in sample_list)
                and all(int(s.get("logger_write_errors", 0) or 0) == 0 for s in sample_list)
                and (logger_depth_max is None or float(logger_depth_max) <= limits["logger_queue_depth_max"])
                and (logger_flush_p95 is None or float(logger_flush_p95) <= limits["logger_flush_p95_max_ms"])
            )
            else "FAIL",
            observed={"logger_queue_depth": _summary(logger_depths), "logger_flush_ms": _summary(logger_flush)},
            requirement="logger has no drops/write errors and bounded queue/flush latency",
        ),
        "unified_motion_mode": _gate(
            "PASS" if _all_modes_unified(sample_list) else "FAIL",
            observed={
                "control_modes": sorted({str(s.get("control_mode", "") or "") for s in sample_list}),
                "motion_state_modes": sorted({str(s.get("motion_state_mode", "") or "") for s in sample_list}),
            },
            requirement="single public motion system: UNIFIED",
        ),
    }
    failed = [name for name, gate in gates.items() if gate.get("status") == "FAIL"]
    metrics = {
        "sample_count": len(sample_list),
        "states": states,
        "front_min_m": front_min,
        "lidar_confidence": _summary(lidar_conf),
        "status_age": _summary(status_ages),
        "pwm_abs": _summary(pwm_abs),
        "cmd_v_abs": _summary(cmd_v_abs),
        "cmd_omega_abs": _summary(cmd_w_abs),
        "actual_v_abs": _summary(actual_v_abs),
        "actual_omega_abs": _summary(actual_w_abs),
    }
    return {
        "schema": "M3_ROOM_CRUISE_UNIFIED_PREFLIGHT_V1",
        "status": "PASS" if not failed else "FAIL",
        "success": not failed,
        "gates": gates,
        "failed_gates": failed,
        "metrics": metrics,
        "samples": sample_list,
    }


def _latest_session_dirs() -> List[Path]:
    if not LOGS_DIR.exists():
        return []
    return sorted(
        [path for path in LOGS_DIR.iterdir() if path.is_dir() and path.name.startswith("session_")],
        key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
        reverse=True,
    )


def _parse_ts(row: Dict[str, Any]) -> Optional[float]:
    value = row.get("ts")
    if _finite(value):
        return float(value)
    wall_ts = str(row.get("wall_ts", "") or "").strip()
    if wall_ts:
        try:
            parsed = dt.datetime.fromisoformat(wall_ts.replace("Z", "+00:00"))
            if parsed.tzinfo is None:
                parsed = parsed.replace(tzinfo=dt.timezone.utc)
            return float(parsed.timestamp())
        except (TypeError, ValueError, OverflowError):
            return None
    return None


def _collect_system_metrics(start_wall_s: float, end_wall_s: float) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    for session_dir in _latest_session_dirs()[:4]:
        path = session_dir / "runtime" / "system.jsonl"
        if not path.exists():
            path = session_dir / "system.jsonl"
        if not path.exists():
            continue
        try:
            for line in path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                ts = _parse_ts(row)
                if ts is None or ts < start_wall_s - 2.0 or ts > end_wall_s + 2.0:
                    continue
                data = dict(row.get("data") or {})
                if any(key in data for key in ("cpu_percent", "cpu_temp_c", "sd_write_latency_ms", "throttled")):
                    rows.append({"session": session_dir.name, "ts": ts, **data})
        except Exception:
            continue
    cpu = [_safe_float(row.get("cpu_percent"), math.nan) for row in rows]
    temp = [_safe_float(row.get("cpu_temp_c"), math.nan) for row in rows]
    sd = [_safe_float(row.get("sd_write_latency_ms"), math.nan) for row in rows]
    throttled_bad = [
        row
        for row in rows
        if str(row.get("throttled", "") or "").strip().lower() not in {"", "0x0", "0"}
    ]
    return {
        "sample_count": len(rows),
        "cpu_percent": _summary(cpu),
        "cpu_temp_c": _summary(temp),
        "sd_write_latency_ms": _summary(sd),
        "throttled_bad_count": len(throttled_bad),
        "sessions": sorted({str(row.get("session", "")) for row in rows if row.get("session")}),
    }


def _consecutive_blocks(indices: Sequence[int]) -> List[List[int]]:
    blocks: List[List[int]] = []
    for index in sorted({int(value) for value in indices if int(value) >= 0}):
        if not blocks or index != blocks[-1][-1] + 1:
            blocks.append([index])
        else:
            blocks[-1].append(index)
    return blocks


def _block_duration_s(samples: Sequence[Dict[str, Any]], block: Sequence[int]) -> float:
    duration = sum(max(0.0, _safe_float(samples[index].get("dt_s"), 0.0)) for index in block)
    if duration > 0.0 or len(block) < 2:
        return float(duration)
    first_ts = _safe_float(samples[block[0]].get("ts"), 0.0)
    last_ts = _safe_float(samples[block[-1]].get("ts"), first_ts)
    return max(0.0, float(last_ts) - float(first_ts))


def analyze_continuous_room_cruise(
    live_samples: Sequence[Dict[str, Any]],
    *,
    base_result: Dict[str, Any],
    thresholds: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """Validate only the bounded M3 goal: one uninterrupted 60 s Room Cruise."""

    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})
    samples = [dict(sample or {}) for sample in live_samples]
    active_indices = [
        index for index, sample in enumerate(samples)
        if bool(sample.get("room_cruise_v2_active", False))
    ]
    moving_indices = [
        index for index in active_indices
        if bool(samples[index].get("m3_moving_cmd", False))
    ]
    active_duration_s = 0.0
    active_dropouts: List[int] = []
    if active_indices:
        first_active = active_indices[0]
        last_active = active_indices[-1]
        active_duration_s = max(
            0.0,
            _safe_float(samples[last_active].get("ts"), 0.0)
            - _safe_float(samples[first_active].get("ts"), 0.0),
        )
        active_dropouts = [
            index
            for index in range(first_active, last_active + 1)
            if not bool(samples[index].get("room_cruise_v2_active", False))
        ]

    path_failures = [
        index
        for index in moving_indices
        if (
            not bool(samples[index].get("m3_track_exec", False))
            or bool(samples[index].get("command_owner_conflict", False))
            or int(samples[index].get("active_route_count", 0) or 0) > 1
            or bool(samples[index].get("primitive_contract_violation", False))
            or bool(samples[index].get("control_execution_contract_violation", False))
            or str(samples[index].get("motion_actual_ssot", "") or "")
            != "EKF_POSE_ODOMETRY_SSOT"
        )
    ]
    pose_failures: List[int] = []
    for index in active_indices:
        sample = samples[index]
        pose = dict(sample.get("pose") or {})
        pose_finite = all(_finite(pose.get(key)) for key in ("x", "y", "theta_deg"))
        if (
            not pose_finite
            or str(sample.get("localization_mode", "") or "").upper() != "TRACKING"
            or not bool(sample.get("localization_allow_motion", False))
            or not bool(sample.get("localization_truth_consistent", True))
        ):
            pose_failures.append(index)

    command_loss_indices = [
        index
        for index in moving_indices
        if (
            max(
                abs(_safe_float(samples[index].get("pwm_left"), 0.0)),
                abs(_safe_float(samples[index].get("pwm_right"), 0.0)),
            )
            < 0.015
            and bool(samples[index].get("safety_allow", True))
            and bool(samples[index].get("localization_allow_motion", True))
            and not bool(samples[index].get("encoder_direction_switch_recent", False))
            and _safe_float(samples[index].get("motion_segment_age_s"), math.inf)
            >= limits["command_loss_settle_s"]
        )
    ]
    command_loss_blocks = _consecutive_blocks(command_loss_indices)
    command_loss_durations = [
        _block_duration_s(samples, block) for block in command_loss_blocks
    ]

    stop_blocks = _consecutive_blocks(
        [
            index for index in active_indices
            if bool(samples[index].get("m3_stop_window", False))
        ]
    )
    unjustified_stop_blocks: List[Dict[str, Any]] = []
    obstacle_reason_tokens = (
        "obstacle",
        "blocked",
        "clearance",
        "recovery",
        "pivot",
    )
    for block in stop_blocks:
        has_motion_before = any(index < block[0] for index in moving_indices)
        has_motion_after = any(index > block[-1] for index in moving_indices)
        if not (has_motion_before and has_motion_after):
            continue
        duration_s = _block_duration_s(samples, block)
        justified = any(
            bool(samples[index].get("m3_obstacle_near", False))
            or bool(samples[index].get("blocked_front", False))
            or not bool(samples[index].get("safety_allow", True))
            or not bool(samples[index].get("localization_allow_motion", True))
            or any(
                token in (
                    str(samples[index].get("room_cruise_v2_reason", "") or "")
                    + " "
                    + str(samples[index].get("local_navigation_reason", "") or "")
                ).lower()
                for token in obstacle_reason_tokens
            )
            for index in block
        )
        if duration_s > limits["unjustified_stop_grace_s"] and not justified:
            unjustified_stop_blocks.append(
                {
                    "start_index": block[0],
                    "end_index": block[-1],
                    "duration_s": duration_s,
                }
            )

    safety_failures = [
        index
        for index in active_indices
        if (
            not bool(samples[index].get("safety_allow", True))
            or bool(samples[index].get("watchdog_stop_triggered", False))
            or str(samples[index].get("stop_type", "") or "").upper()
            in {"EMERGENCY_STOP", "FAILSAFE"}
        )
    ]
    base_summary = dict(base_result.get("summary") or {})
    base_duration_s = _safe_float(base_summary.get("duration_s"), 0.0)
    progress_m = _safe_float(base_summary.get("progress_m"), 0.0)

    metrics = {
        "active_sample_count": len(active_indices),
        "moving_sample_count": len(moving_indices),
        "active_duration_s": active_duration_s,
        "base_duration_s": base_duration_s,
        "progress_m": progress_m,
        "active_dropout_indices": active_dropouts,
        "path_failure_indices": path_failures,
        "pose_failure_indices": pose_failures,
        "command_loss_episode_durations_s": command_loss_durations,
        "command_loss_indices": command_loss_indices,
        "unjustified_stop_blocks": unjustified_stop_blocks,
        "safety_failure_indices": safety_failures,
    }
    gates = {
        "single_uninterrupted_run": _gate(
            "PASS"
            if (
                active_indices
                and active_duration_s >= limits["continuous_active_duration_min_s"]
                and base_duration_s >= 60.0
                and progress_m >= 0.45
                and not active_dropouts
            )
            else "FAIL",
            observed={
                "active_sample_count": len(active_indices),
                "active_duration_s": active_duration_s,
                "base_duration_s": base_duration_s,
                "progress_m": progress_m,
                "active_dropout_indices": active_dropouts,
            },
            requirement="one Room Cruise activation stays continuously active for the requested 60 s window and makes physical progress",
        ),
        "normal_command_path_and_pose": _gate(
            "PASS" if moving_indices and not path_failures and not pose_failures else "FAIL",
            observed={
                "moving_sample_count": len(moving_indices),
                "path_failure_indices": path_failures,
                "pose_failure_indices": pose_failures,
            },
            requirement="existing M3 TRACK path only, one owner, valid EKF pose SSOT throughout",
        ),
        "no_unjustified_stop_start": _gate(
            "PASS" if not unjustified_stop_blocks else "FAIL",
            observed={"unjustified_stop_blocks": unjustified_stop_blocks},
            requirement="no internal stop/start longer than the transition grace without obstacle or safety justification",
        ),
        "no_command_or_pwm_loss": _gate(
            "PASS"
            if max(command_loss_durations or [0.0]) <= limits["command_loss_grace_s"]
            else "FAIL",
            observed={
                "episode_durations_s": command_loss_durations,
                "sample_indices": command_loss_indices,
            },
            requirement="no settled moving intent loses normal motor output beyond the bounded transition grace",
        ),
        "safety": _gate(
            "PASS" if not safety_failures else "FAIL",
            observed={"failure_indices": safety_failures},
            requirement="no emergency, failsafe, watchdog stop, or safety-denied active sample",
        ),
    }
    failed = [name for name, gate in gates.items() if gate.get("status") == "FAIL"]
    return {
        "schema": "M3_CONTINUOUS_ROOM_CRUISE_VALIDATION_V1",
        "contract_id": CONTINUOUS_ROOM_CRUISE_CONTRACT_ID,
        "status": "PASS" if not failed else "FAIL",
        "success": not failed,
        "gates": gates,
        "failed_gates": failed,
        "metrics": metrics,
    }


def _runtime_sample_metrics(
    samples: Sequence[Dict[str, Any]],
    encoder_persistent_age_s: float = 1.0,
) -> Dict[str, Any]:
    sample_list = [dict(sample or {}) for sample in samples]
    frequencies = [_safe_float(s.get("watchdog_freq_hz"), math.nan) for s in sample_list]
    periods = [_safe_float(s.get("watchdog_period_s"), math.nan) for s in sample_list]
    loop_budget = [_safe_float(s.get("loop_budget_total_ema_ms"), math.nan) for s in sample_list]
    logger_depth = [_safe_float(s.get("logger_queue_depth"), math.nan) for s in sample_list]
    logger_flush = [_safe_float(s.get("logger_flush_duration_ms"), math.nan) for s in sample_list]
    lidar_conf = [_safe_float(s.get("lidar_pose_confidence"), math.nan) for s in sample_list]
    scan_rate = [_safe_float(s.get("lidar_raw_scan_rate_hz"), math.nan) for s in sample_list]
    matcher = [_safe_float(s.get("lidar_matcher_latency_ms"), math.nan) for s in sample_list]
    ekf_gap = [_safe_float(s.get("lidar_ekf_applied_gap_s"), math.nan) for s in sample_list]
    slow_tick_max = max([int(s.get("slow_tick_count", 0) or 0) for s in sample_list] or [0])
    slow_io_max = max([int(s.get("slow_io_event_count", 0) or 0) for s in sample_list] or [0])
    slow_lidar_max = max([int(s.get("slow_lidar_spike_count", 0) or 0) for s in sample_list] or [0])
    slow_gc_max = max([int(s.get("slow_gc_count", 0) or 0) for s in sample_list] or [0])
    slow_tick_delta = _max_counter_delta(sample_list, "slow_tick_count")
    slow_observed_tick_delta = _max_counter_delta(sample_list, "slow_observed_tick_count")
    slow_io_delta = _max_counter_delta(sample_list, "slow_io_event_count")
    slow_den = max(1, slow_observed_tick_delta)
    encoder_anomaly_samples = [s for s in sample_list if bool(s.get("encoder_anomaly_active", False))]
    encoder_persistent_samples = [
        s
        for s in sample_list
        if bool(s.get("encoder_symmetry_fault_active", False))
        or (
            bool(s.get("encoder_anomaly_active", False))
            and not bool(s.get("encoder_direction_switch_recent", False))
            and (
                not _finite(s.get("motion_segment_age_s"))
                or _safe_float(s.get("motion_segment_age_s"), 0.0) >= float(encoder_persistent_age_s)
            )
        )
    ]
    encoder_anomaly_ages = [
        _safe_float(s.get("motion_segment_age_s"), math.nan)
        for s in encoder_anomaly_samples
    ]
    return {
        "sample_count": len(sample_list),
        "watchdog_frequency_hz": _summary(frequencies),
        "watchdog_period_s": _summary(periods),
        "frequency_below_45_ratio": _ratio(sum(1 for v in frequencies if _finite(v) and float(v) < 45.0), len([v for v in frequencies if _finite(v)])),
        "loop_budget_total_ema_ms": _summary(loop_budget),
        "logger_queue_depth": _summary(logger_depth),
        "logger_flush_duration_ms": _summary(logger_flush),
        "logger_dropped_messages_max": max([int(s.get("logger_dropped_messages", 0) or 0) for s in sample_list] or [0]),
        "logger_write_errors_max": max([int(s.get("logger_write_errors", 0) or 0) for s in sample_list] or [0]),
        "lidar_pose_confidence": _summary(lidar_conf),
        "lidar_raw_scan_rate_hz": _summary(scan_rate),
        "lidar_matcher_latency_ms": _summary(matcher),
        "lidar_ekf_applied_gap_s": _summary(ekf_gap),
        "slow_tick_count_max": slow_tick_max,
        "slow_io_event_count_max": slow_io_max,
        "slow_lidar_spike_count_max": slow_lidar_max,
        "slow_gc_count_max": slow_gc_max,
        "slow_tick_delta": slow_tick_delta,
        "slow_observed_tick_delta": slow_observed_tick_delta,
        "slow_io_event_delta": slow_io_delta,
        "slow_tick_ratio": float(slow_tick_delta) / float(slow_den),
        "slow_io_ratio": float(slow_io_delta) / float(slow_den),
        "encoder_anomaly_samples": len(encoder_anomaly_samples),
        "encoder_transient_anomaly_samples": max(0, len(encoder_anomaly_samples) - len(encoder_persistent_samples)),
        "encoder_persistent_anomaly_samples": len(encoder_persistent_samples),
        "encoder_anomaly_segment_age_s": _summary(encoder_anomaly_ages),
        "encoder_persistent_age_min_s": float(encoder_persistent_age_s),
    }


def analyze_runtime(
    live_samples: Sequence[Dict[str, Any]],
    *,
    base_result: Dict[str, Any],
    m3_result: Dict[str, Any],
    system_metrics: Dict[str, Any],
    thresholds: Optional[Dict[str, float]] = None,
    continuity_result: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})
    samples = [dict(sample or {}) for sample in live_samples]
    continuity = dict(
        continuity_result
        or analyze_continuous_room_cruise(
            samples,
            base_result=base_result,
            thresholds=limits,
        )
    )
    sample_metrics = _runtime_sample_metrics(
        samples,
        encoder_persistent_age_s=limits["encoder_anomaly_persistent_segment_age_min_s"],
    )
    m3_metrics = dict(m3_result.get("metrics") or {})
    m3_integrity = dict(m3_metrics.get("integrity") or {})
    m3_safety = dict(m3_metrics.get("safety") or {})
    m3_timing = dict(m3_metrics.get("timing") or {})
    failed_m3 = list(m3_result.get("failed_gates") or [])

    hard_m3_failures = [
        name
        for name in failed_m3
        if name
        not in {
            "repeatability",
        }
    ]
    system_available = int(system_metrics.get("sample_count", 0) or 0) > 0
    cpu_p95 = (system_metrics.get("cpu_percent") or {}).get("p95")
    temp_max = (system_metrics.get("cpu_temp_c") or {}).get("max")
    sd_p95 = (system_metrics.get("sd_write_latency_ms") or {}).get("p95")
    sd_max = (system_metrics.get("sd_write_latency_ms") or {}).get("max")

    gates: Dict[str, Dict[str, Any]] = {
        "base_room_cruise_v2": _gate(
            "PASS" if str(base_result.get("status") or (base_result.get("summary") or {}).get("status") or "").upper() == "PASS" else "FAIL",
            observed=dict(base_result.get("summary") or {}),
            requirement="underlying room_cruise_v2_live gate passes",
            required=False,
        ),
        "continuous_room_cruise": _gate(
            str(continuity.get("status", "FAIL") or "FAIL").upper(),
            observed={
                "contract_id": continuity.get("contract_id"),
                "failed_gates": list(continuity.get("failed_gates") or []),
                "metrics": dict(continuity.get("metrics") or {}),
            },
            requirement="one uninterrupted 60 s Room Cruise on the normal command path without unjustified stop/start or command loss",
        ),
        "camera_off_runtime": _gate(
            "PASS"
            if samples
            and all(not bool(s.get("camera_enabled", False)) for s in samples)
            and all(not bool((dict(s.get("peripherals") or {})).get("camera", False)) for s in samples)
            else "FAIL",
            observed={"camera_enabled_samples": _max_bool_count(samples, "camera_enabled", True)},
            requirement="camera remains disabled during the whole room cruise run",
        ),
        "unified_motion_mode_runtime": _gate(
            "PASS" if _all_modes_unified(samples) else "FAIL",
            observed={
                "control_modes": sorted({str(s.get("control_mode", "") or "") for s in samples}),
                "motion_state_modes": sorted({str(s.get("motion_state_mode", "") or "") for s in samples}),
            },
            requirement="all runtime samples report UNIFIED motion mode",
        ),
        "ssot_contract": _gate(
            "PASS"
            if (
                int(m3_integrity.get("forbidden_path_samples", 0) or 0) == 0
                and int(m3_integrity.get("owner_conflict_samples", 0) or 0) == 0
                and int(m3_integrity.get("motion_actual_ssot_bad_samples", 0) or 0) == 0
                and int(m3_integrity.get("m3_track_route_bad_samples", 0) or 0) == 0
            )
            else "FAIL",
            observed=m3_integrity,
            requirement="one owner, EKF actual SSOT, no service/legacy/direct path, room cruise uses track route",
        ),
        "safety_runtime": _gate(
            "PASS"
            if (
                int(m3_safety.get("safety_event_samples", 0) or 0) == 0
                and int(m3_safety.get("nonfinite_command_samples", 0) or 0) == 0
                and (
                    m3_safety.get("min_clearance_m") is None
                    or float(m3_safety.get("min_clearance_m") or 0.0) >= limits["runtime_min_front_m"]
                )
            )
            else "FAIL",
            observed=m3_safety,
            requirement="no safety/failsafe/nonfinite event and clearance stays above runtime floor",
        ),
        "control_loop_timing": _gate(
            "PASS"
            if (
                (_safe_float(m3_timing.get("frequency_p10_hz"), 0.0) >= limits["loop_frequency_p10_min_hz"])
                and (_safe_float(m3_timing.get("frequency_below_45_ratio"), 1.0) <= limits["loop_below_45_ratio_max"])
                and (_safe_float(m3_timing.get("dt_p95_s"), math.inf) <= limits["loop_dt_p95_max_s"])
                and (_safe_float(m3_timing.get("loop_budget_total_ema_p95_ms"), math.inf) <= limits["loop_budget_p95_max_ms"])
            )
            else "FAIL",
            observed={"m3_timing": m3_timing, "sample_timing": sample_metrics},
            requirement="50 Hz loop target stays within bounded p10/dt/budget limits",
        ),
        "motion_quality_m3": _gate(
            "PASS" if not hard_m3_failures else "FAIL",
            observed={
                "m3_status": m3_result.get("status"),
                "failed_gates": failed_m3,
                "inconclusive_gates": list(m3_result.get("inconclusive_gates") or []),
                "hard_failures": hard_m3_failures,
            },
            requirement="M3 movement-quality sub-gates have no hard failures for this one-minute run",
            required=False,
        ),
        "software_performance": _gate(
            "PASS"
            if (
                int(sample_metrics.get("logger_dropped_messages_max", 0) or 0) == 0
                and int(sample_metrics.get("logger_write_errors_max", 0) or 0) == 0
                and _safe_float((sample_metrics.get("logger_queue_depth") or {}).get("max"), 0.0) <= limits["logger_queue_depth_max"]
                and _safe_float((sample_metrics.get("logger_flush_duration_ms") or {}).get("p95"), 0.0) <= limits["logger_flush_p95_max_ms"]
                and _safe_float(sample_metrics.get("slow_tick_ratio"), 0.0) <= limits["slow_tick_ratio_max"]
                and _safe_float(sample_metrics.get("slow_io_ratio"), 0.0) <= limits["slow_io_ratio_max"]
            )
            else "FAIL",
            observed=sample_metrics,
            requirement="logger clean, queue bounded, slow tick and I/O ratios bounded",
            required=False,
        ),
        "peripheral_runtime_health": _gate(
            "PASS"
            if (
                _safe_float((sample_metrics.get("lidar_pose_confidence") or {}).get("p50"), 0.0) >= limits["lidar_confidence_p50_min"]
                and _safe_float((sample_metrics.get("lidar_raw_scan_rate_hz") or {}).get("p50"), 0.0) >= limits["lidar_raw_scan_rate_p50_min_hz"]
                and _safe_float((sample_metrics.get("lidar_matcher_latency_ms") or {}).get("p95"), 0.0) <= limits["lidar_matcher_latency_p95_max_ms"]
                and _safe_float((sample_metrics.get("lidar_ekf_applied_gap_s") or {}).get("p95"), 0.0) <= limits["lidar_ekf_gap_p95_max_s"]
                and int(sample_metrics.get("encoder_persistent_anomaly_samples", 0) or 0) == 0
                and all(str(s.get("imu_health", "") or "OK").upper() in {"", "OK"} for s in samples)
            )
            else "FAIL",
            observed=sample_metrics,
            requirement="LIDAR cadence/confidence and IMU health stay usable; encoder transition anomalies are reported, persistent anomalies are forbidden",
            required=False,
        ),
        "hardware_performance": _gate(
            (
                "PASS"
                if (
                    system_available
                    and _safe_float(cpu_p95, 0.0) <= limits["cpu_p95_max_percent"]
                    and _safe_float(temp_max, 0.0) <= limits["cpu_temp_max_c"]
                    and _safe_float(sd_p95, 0.0) <= limits["sd_latency_p95_max_ms"]
                    and _safe_float(sd_max, 0.0) <= limits["sd_latency_max_ms"]
                    and int(system_metrics.get("throttled_bad_count", 0) or 0) == 0
                )
                else ("INCONCLUSIVE" if not system_available else "FAIL")
            ),
            observed=system_metrics,
            requirement="CPU, temperature, throttling and SD write latency remain bounded",
            required=False,
        ),
    }
    failed = [
        name
        for name, gate in gates.items()
        if bool(gate.get("required", True)) and gate.get("status") == "FAIL"
    ]
    inconclusive = [
        name
        for name, gate in gates.items()
        if bool(gate.get("required", True)) and gate.get("status") == "INCONCLUSIVE"
    ]
    diagnostic_failed = [
        name
        for name, gate in gates.items()
        if not bool(gate.get("required", True)) and gate.get("status") == "FAIL"
    ]
    diagnostic_inconclusive = [
        name
        for name, gate in gates.items()
        if not bool(gate.get("required", True)) and gate.get("status") == "INCONCLUSIVE"
    ]
    status = "FAIL" if failed else ("INCONCLUSIVE" if inconclusive else "PASS")
    return {
        "schema": "M3_ROOM_CRUISE_UNIFIED_RUNTIME_V2",
        "contract_id": CONTINUOUS_ROOM_CRUISE_CONTRACT_ID,
        "status": status,
        "success": status == "PASS",
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "diagnostic_failed_gates": diagnostic_failed,
        "diagnostic_inconclusive_gates": diagnostic_inconclusive,
        "metrics": {
            "runtime_samples": sample_metrics,
            "system": system_metrics,
            "m3": m3_metrics,
        },
        "continuous_room_cruise": continuity,
    }


def _incident_payload(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": "M3_ROOM_CRUISE_UNIFIED_VALIDATOR_INCIDENT_V2",
        "contract_id": result.get("contract_id"),
        "needed": result.get("status") != "PASS",
        "status": result.get("status"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "preflight_failed_gates": list(((result.get("preflight") or {}).get("failed_gates")) or []),
        "runtime_failed_gates": list(((result.get("runtime_validation") or {}).get("failed_gates")) or []),
        "continuity_failed_gates": list(
            ((result.get("continuous_room_cruise") or {}).get("failed_gates")) or []
        ),
        "m3_failed_gates": list(((result.get("m3_room_cruise") or {}).get("failed_gates")) or []),
        "artifact_paths": dict(result.get("artifact_paths") or {}),
    }


def build_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": result.get("schema"),
        "contract_id": result.get("contract_id"),
        "status": result.get("status"),
        "success": bool(result.get("success", False)),
        "test_name": result.get("test_name"),
        "plain_summary_hu": result.get("plain_summary_hu"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "preflight": {
            "status": (result.get("preflight") or {}).get("status"),
            "failed_gates": list(((result.get("preflight") or {}).get("failed_gates")) or []),
            "metrics": (result.get("preflight") or {}).get("metrics"),
        },
        "runtime_validation": {
            "status": (result.get("runtime_validation") or {}).get("status"),
            "failed_gates": list(((result.get("runtime_validation") or {}).get("failed_gates")) or []),
            "inconclusive_gates": list(((result.get("runtime_validation") or {}).get("inconclusive_gates")) or []),
        },
        "continuous_room_cruise": {
            "status": (result.get("continuous_room_cruise") or {}).get("status"),
            "failed_gates": list(
                ((result.get("continuous_room_cruise") or {}).get("failed_gates"))
                or []
            ),
            "metrics": (result.get("continuous_room_cruise") or {}).get("metrics"),
        },
        "m3_room_cruise": {
            "status": (result.get("m3_room_cruise") or {}).get("status"),
            "closure_verdict": (result.get("m3_room_cruise") or {}).get("closure_verdict"),
            "failed_gates": list(((result.get("m3_room_cruise") or {}).get("failed_gates")) or []),
            "inconclusive_gates": list(((result.get("m3_room_cruise") or {}).get("inconclusive_gates")) or []),
        },
        "artifact_paths": dict(result.get("artifact_paths") or {}),
    }


def write_artifacts(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    summary = build_summary(result)
    _write_jsonl(SAMPLES_PATH, samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, summary)
    _write_json(PREFLIGHT_PATH, result.get("preflight") or {})
    _write_json(INCIDENT_PATH, _incident_payload(result))
    return summary


def _plain_summary(status: str, preflight: Dict[str, Any], runtime: Optional[Dict[str, Any]]) -> str:
    parts = [f"Eredmeny: {status}."]
    parts.append(f"Foundation preflight: {preflight.get('status')}.")
    if runtime:
        parts.append(f"60s room cruise validalas: {runtime.get('status')}.")
    if preflight.get("failed_gates"):
        parts.append("Preflight hibak: " + ", ".join(preflight.get("failed_gates") or []) + ".")
    if runtime and runtime.get("failed_gates"):
        parts.append("Runtime hibak: " + ", ".join(runtime.get("failed_gates") or []) + ".")
    return " ".join(parts)


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    thresholds = dict(DEFAULT_THRESHOLDS)
    if args.thresholds_json:
        payload = json.loads(Path(args.thresholds_json).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("thresholds JSON must contain an object")
        thresholds.update({key: float(value) for key, value in payload.items() if key in thresholds})

    original_peripherals = read_peripherals(runtime_dir=RUNTIME_DIR, use_cache=False)
    camera_disable_result: Dict[str, Any] = {"requested": bool(args.disable_camera), "original_camera": bool(original_peripherals.get("camera", False))}
    if bool(args.disable_camera):
        camera_disable_result["peripherals_after_disable"] = set_peripheral_enabled("camera", False, runtime_dir=RUNTIME_DIR)
        time.sleep(max(0.0, float(args.camera_settle_s)))

    preflight_samples = _collect_preflight_samples(float(args.preflight_duration_s), float(args.preflight_poll_s))
    preflight = analyze_preflight(preflight_samples, thresholds)
    all_samples: List[Dict[str, Any]] = list(preflight_samples)
    runtime_validation: Optional[Dict[str, Any]] = None
    continuity_result: Dict[str, Any] = {}
    m3_result: Dict[str, Any] = {}
    m3_samples: List[Dict[str, Any]] = []
    base_result: Dict[str, Any] = {}
    system_metrics: Dict[str, Any] = {}

    should_run_live = bool(preflight.get("success", False)) and not bool(args.preflight_only)
    if not should_run_live and not bool(args.preflight_only) and bool(args.run_on_preflight_fail):
        should_run_live = True

    live_started = 0.0
    live_finished = 0.0
    if should_run_live:
        if bool(args.disable_camera):
            camera_disable_result["peripherals_before_live"] = set_peripheral_enabled("camera", False, runtime_dir=RUNTIME_DIR)
            time.sleep(max(0.0, float(args.camera_settle_s)))
        live_started = time.time()
        base_args = Namespace(
            duration_s=float(args.duration_s),
            min_progress_m=float(args.base_min_progress_m),
            min_front_m=float(args.min_front_m),
            v_max_mps=float(args.v_max_mps),
            omega_max_rad_s=float(args.omega_max_rad_s),
            poll_s=float(args.poll_s),
            token=str(args.token),
            compact=False,
        )
        base_result = cruise.run(base_args)
        live_finished = time.time()
        raw_samples = []
        for sample in list(base_result.get("samples") or []):
            row = dict(sample)
            row["sample_phase"] = "room_cruise"
            row["run_index"] = 0
            raw_samples.append(row)
        base_compact = m3_room._base_result_compact(base_result)
        m3_thresholds = {
            "min_run_count_required": 1,
        }
        m3_result, m3_samples = m3_room.analyze_samples(raw_samples, m3_thresholds, base_results=[base_compact])
        continuity_result = analyze_continuous_room_cruise(
            m3_samples,
            base_result=base_compact,
            thresholds=thresholds,
        )
        system_metrics = _collect_system_metrics(live_started, live_finished)
        runtime_validation = analyze_runtime(
            m3_samples,
            base_result=base_compact,
            m3_result=m3_result,
            system_metrics=system_metrics,
            thresholds=thresholds,
            continuity_result=continuity_result,
        )
        all_samples.extend(m3_samples)

    statuses = [str(preflight.get("status", ""))]
    if runtime_validation is not None:
        statuses.append(str(runtime_validation.get("status", "")))
    failed = list(preflight.get("failed_gates") or [])
    inconclusive: List[str] = []
    if runtime_validation is not None:
        failed.extend([f"runtime:{name}" for name in list(runtime_validation.get("failed_gates") or [])])
        inconclusive.extend([f"runtime:{name}" for name in list(runtime_validation.get("inconclusive_gates") or [])])
    if any(status == "FAIL" for status in statuses):
        status = "FAIL"
    elif any(status == "INCONCLUSIVE" for status in statuses) or inconclusive:
        status = "INCONCLUSIVE"
    else:
        status = "PASS"

    result = {
        "schema": "M3_ROOM_CRUISE_UNIFIED_VALIDATOR_V2",
        "contract_id": CONTINUOUS_ROOM_CRUISE_CONTRACT_ID,
        "test_name": str(args.test_name),
        "status": status,
        "success": status == "PASS",
        "generated_ts": time.time(),
        "preflight_only": bool(args.preflight_only),
        "camera_disable": camera_disable_result,
        "thresholds": thresholds,
        "preflight": preflight,
        "runtime_validation": runtime_validation,
        "continuous_room_cruise": continuity_result,
        "m3_room_cruise": m3_result,
        "base_room_cruise": m3_room._base_result_compact(base_result) if base_result else {},
        "system_metrics": system_metrics,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "plain_summary_hu": _plain_summary(status, preflight, runtime_validation),
        "artifact_paths": {
            "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
            "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            "preflight": str(PREFLIGHT_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
            "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
            "base_room_cruise_summary": str(cruise.LATEST_SUMMARY.relative_to(PROJECT_ROOT)),
            "base_room_cruise_result": str(cruise.LATEST_RESULT.relative_to(PROJECT_ROOT)),
        },
        "ssot_summary": {
            "motion_intent": "motion_resolver resolved proposal / motion_command limited intent",
            "pose": "EKF_POSE_ODOMETRY_SSOT via motion_public",
            "motor_output": "MotionExecutor only; emergency stop may clamp",
            "mode": CANONICAL_CONTROL_MODE,
        },
    }
    write_artifacts(result, all_samples)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M3 Room Cruise UNIFIED full-system validator.")
    parser.add_argument("--test-name", default="M3_room_cruise_unified_validator")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--preflight-duration-s", type=float, default=DEFAULT_THRESHOLDS["preflight_duration_s"])
    parser.add_argument("--preflight-poll-s", type=float, default=DEFAULT_THRESHOLDS["preflight_poll_s"])
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--poll-s", type=float, default=0.12)
    parser.add_argument("--v-max-mps", type=float, default=0.30)
    parser.add_argument("--omega-max-rad-s", type=float, default=0.60)
    parser.add_argument("--base-min-progress-m", type=float, default=0.45)
    parser.add_argument("--min-front-m", type=float, default=DEFAULT_THRESHOLDS["runtime_min_front_m"])
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--disable-camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--camera-settle-s", type=float, default=0.4)
    parser.add_argument("--run-on-preflight-fail", action="store_true")
    parser.add_argument("--thresholds-json", default="")
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = run(args)
    output = result
    if bool(args.compact):
        output = {
            "status": result.get("status"),
            "plain_summary_hu": result.get("plain_summary_hu"),
            "failed_gates": result.get("failed_gates"),
            "inconclusive_gates": result.get("inconclusive_gates"),
            "artifact_paths": result.get("artifact_paths"),
        }
    print(json.dumps(_json_safe(output), ensure_ascii=False))
    return 0 if result.get("status") == "PASS" else (2 if result.get("status") == "INCONCLUSIVE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
