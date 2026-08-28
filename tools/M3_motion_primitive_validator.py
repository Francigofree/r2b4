#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""M3 motion primitive full-stack live validator.

This tool validates primitives through the normal public motion path. It is not
a controller and it does not write motor outputs directly.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from middleware.peripheral_usage import read_peripherals, set_peripheral_enabled  # noqa: E402
from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import M3_room_cruise_unified_validator as unified_validator  # noqa: E402
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
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_POLL_S,
    STATUS_PATH,
    _append_command,
    _get_pose,
    _normalize_angle_deg,
    _read_json,
    _safe_stop_best_effort,
    _send_command_checked,
    _status_version,
    _wait_for_status,
    _wait_until_stopped,
)


RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_motion_primitive_validator.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_motion_primitive_validator_summary.json"
PREFLIGHT_PATH = AGENT_TESTS_DIR / "latest_M3_motion_primitive_validator_preflight.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M3_motion_primitive_validator_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M3_motion_primitive_validator_incident.json"
REPLAY_RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_motion_primitive_replay_analysis.json"
REPLAY_SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_motion_primitive_replay_analysis_summary.json"

CANONICAL_CONTROL_MODE = "UNIFIED"
EXPECTED_PIVOT_PRIMITIVE = "IN_PLACE_ROTATE"
EXPECTED_STRAIGHT_PRIMITIVE = "STRAIGHT"
EXPECTED_ARC_PRIMITIVES = {"DIFF_ARC_GENTLE", "DIFF_ARC_SHARP"}
EXPECTED_EXECUTION_MODE = "TRACK_EXEC"
EXPECTED_TWIST_EXECUTION_MODE = "TWIST_EXEC"
EXPECTED_MOTION_ACTUAL_SSOT = "EKF_POSE_ODOMETRY_SSOT"


DEFAULT_THRESHOLDS: Dict[str, float] = {
    "preflight_duration_s": 3.0,
    "preflight_poll_s": 0.15,
    "preflight_min_samples": 7.0,
    "required_clearance_m": 0.35,
    "target_angle_deg": 30.0,
    "angle_tolerance_deg": 10.0,
    "motion_timeout_s": 10.0,
    "stop_timeout_s": 5.0,
    "settle_time_s": 0.60,
    "poll_s": 0.08,
    "no_progress_grace_s": 0.85,
    "no_progress_timeout_s": 2.50,
    "no_progress_min_step_deg": 1.0,
    "pivot_entry_speed_scale": 1.0,
    "pivot_brake_trigger_ratio": 0.78,
    "pivot_brake_speed_scale": 1.0,
    "pivot_stop_window_deg": 2.0,
    "pivot_stop_prediction_horizon_s": 0.22,
    "min_motion_samples": 10.0,
    "min_directional_progress_deg": 12.0,
    "max_linear_leak_m": 0.10,
    "actual_v_abs_p90_max_mps": 0.060,
    "actual_omega_abs_p50_min_rad_s": 0.045,
    "track_exec_ratio_min": 0.55,
    "track_ref_opposite_ratio_min": 0.80,
    "track_ref_symmetry_p90_max_mps": 0.015,
    "actual_wheel_opposite_ratio_min": 0.55,
    "actual_wheel_response_min_ratio": 0.45,
    "primitive_expected_ratio_min": 0.70,
    "actual_primitive_expected_ratio_min": 0.55,
    "actual_primitive_coverage_min": 0.15,
    "actual_primitive_transition_settle_s": 0.30,
    "motion_actual_ssot_ratio_min": 0.85,
    "lidar_confidence_p50_min": 0.30,
    "lidar_ekf_gap_p95_max_s": 0.70,
    "loop_frequency_p10_min_hz": 40.0,
    "loop_below_45_ratio_max": 0.15,
    "loop_budget_p95_max_ms": 40.0,
    "logger_queue_depth_max": 256.0,
    "logger_flush_p95_max_ms": 140.0,
    "slow_tick_ratio_max": 0.35,
    "straight_min_distance_m": 0.025,
    "straight_heading_abs_max_deg": 4.0,
    "arc_min_yaw_deg": 2.5,
    "arc_min_distance_m": 0.010,
    "cpu_p95_max_percent": 94.0,
    "cpu_temp_max_c": 78.0,
    "sd_latency_p95_max_ms": 180.0,
    "sd_latency_max_ms": 650.0,
}


@dataclass(frozen=True)
class PrimitiveCase:
    name: str
    left_mps: float = 0.0
    right_mps: float = 0.0
    kind: str = "pivot_track"
    expected_primitive: str = EXPECTED_PIVOT_PRIMITIVE
    v_mps: float = 0.0
    omega_rad_s: float = 0.0
    duration_s: float = 0.0
    min_distance_m: float = 0.0
    min_yaw_deg: float = 0.0

    @property
    def expected_sign(self) -> float:
        if self.kind == "twist":
            if abs(float(self.omega_rad_s)) > 1e-6:
                return 1.0 if float(self.omega_rad_s) >= 0.0 else -1.0
            return 1.0
        return 1.0 if (float(self.right_mps) - float(self.left_mps)) >= 0.0 else -1.0


def _summary(values: Iterable[Any]) -> Dict[str, Any]:
    finite = [float(v) for v in values if _is_finite(v)]
    if not finite:
        return {"n": 0, "min": None, "p50": None, "p90": None, "p95": None, "max": None}
    return {
        "n": len(finite),
        "min": _percentile(finite, 0.0),
        "p10": _percentile(finite, 0.10),
        "p50": _percentile(finite, 0.50),
        "p90": _percentile(finite, 0.90),
        "p95": _percentile(finite, 0.95),
        "max": _percentile(finite, 1.0),
    }


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _upper(value: Any) -> str:
    return str(value or "").strip().upper()


def _unique(values: Iterable[Any]) -> List[str]:
    out: List[str] = []
    for value in values:
        item = str(value or "").strip()
        if item and item not in out:
            out.append(item)
    return out


def _phase_status_from_gates(gates: Dict[str, Dict[str, Any]]) -> Tuple[str, List[str], List[str]]:
    failed = [name for name, gate in gates.items() if gate.get("status") == "FAIL"]
    inconclusive = [name for name, gate in gates.items() if gate.get("status") == "INCONCLUSIVE"]
    if failed:
        return "FAIL", failed, inconclusive
    if inconclusive:
        return "INCONCLUSIVE", failed, inconclusive
    return "PASS", failed, inconclusive


def _case_definitions(cases: Sequence[str], track_speed_mps: float) -> List[PrimitiveCase]:
    speed = abs(float(track_speed_mps))
    mapping = {
        "pivot_left": PrimitiveCase("pivot_left", left_mps=-speed, right_mps=speed),
        "pivot_right": PrimitiveCase("pivot_right", left_mps=speed, right_mps=-speed),
        "straight_forward": PrimitiveCase(
            "straight_forward",
            kind="twist",
            expected_primitive=EXPECTED_STRAIGHT_PRIMITIVE,
            v_mps=0.150,
            omega_rad_s=0.0,
            duration_s=1.0,
            min_distance_m=DEFAULT_THRESHOLDS["straight_min_distance_m"],
        ),
        "arc_left": PrimitiveCase(
            "arc_left",
            kind="twist",
            expected_primitive="DIFF_ARC_GENTLE",
            v_mps=0.150,
            omega_rad_s=0.20,
            duration_s=0.90,
            min_distance_m=DEFAULT_THRESHOLDS["arc_min_distance_m"],
            min_yaw_deg=DEFAULT_THRESHOLDS["arc_min_yaw_deg"],
        ),
        "arc_right": PrimitiveCase(
            "arc_right",
            kind="twist",
            expected_primitive="DIFF_ARC_GENTLE",
            v_mps=0.150,
            omega_rad_s=-0.20,
            duration_s=0.90,
            min_distance_m=DEFAULT_THRESHOLDS["arc_min_distance_m"],
            min_yaw_deg=DEFAULT_THRESHOLDS["arc_min_yaw_deg"],
        ),
    }
    selected: List[PrimitiveCase] = []
    for raw in cases:
        name = str(raw or "").strip()
        if not name:
            continue
        if name not in mapping:
            raise ValueError(f"Unknown primitive case: {name}. Known cases: {', '.join(sorted(mapping))}")
        selected.append(mapping[name])
    if not selected:
        selected.append(mapping["pivot_left"])
    return selected


def _sample_status(
    status: Dict[str, Any],
    *,
    phase: str,
    start_pose: Dict[str, float],
    expected_sign: float,
    t_rel_s: float,
) -> Dict[str, Any]:
    base = cruise._sample(status) if status else {"ts": time.time()}
    motion_command = dict((status or {}).get("motion_command") or {})
    primitive_contract = dict(motion_command.get("primitive_contract") or (status or {}).get("primitive_contract") or {})
    control_monitor = dict((status or {}).get("control_monitor") or {})
    command_arbitration = dict((motion_command.get("command_arbitration") or {}))
    slow_tick = dict((status or {}).get("slow_tick_diagnostics") or {})
    encoder_status = dict((status or {}).get("encoder") or {})
    encoder_left = dict(encoder_status.get("left") or {})
    encoder_right = dict(encoder_status.get("right") or {})
    encoder_left_snapshot = dict(encoder_left.get("snapshot") or {})
    encoder_right_snapshot = dict(encoder_right.get("snapshot") or {})
    pose = _get_pose(status or {})
    heading_change_deg = float(
        _normalize_angle_deg(float(pose.get("theta_deg", 0.0)) - float(start_pose.get("theta_deg", 0.0)))
    )
    displacement_m = math.hypot(
        float(pose.get("x", 0.0)) - float(start_pose.get("x", 0.0)),
        float(pose.get("y", 0.0)) - float(start_pose.get("y", 0.0)),
    )
    base.update(
        {
            "sample_phase": str(phase),
            "t_rel_s": float(t_rel_s),
            "status_version": int(_status_version(status or {})),
            "pose": pose,
            "odometry_mode": _upper((status or {}).get("odometry_mode")),
            "heading_change_deg": float(heading_change_deg),
            "directional_progress_deg": max(0.0, float(expected_sign) * float(heading_change_deg)),
            "displacement_from_start_m": float(displacement_m),
            "primitive_contract_detail": primitive_contract,
            "control_execution_contract_detail": {
                "execution_mode_contract_violation": bool(
                    control_monitor.get("execution_mode_contract_violation", False)
                ),
                "track_reference_mode": str(control_monitor.get("track_reference_mode", "") or ""),
                "output_reason": str(control_monitor.get("output_reason", "") or ""),
                "wheel_loop_enabled": bool(control_monitor.get("wheel_loop_enabled", False)),
            },
            "command_arbitration_detail": command_arbitration,
            "slow_tick_detail": slow_tick,
            "encoder_driver_onset_trace": {
                "left_enabled": bool(encoder_left.get("edge_trace_enabled", False)),
                "right_enabled": bool(encoder_right.get("edge_trace_enabled", False)),
                "left": list(encoder_left.get("recent_a_rising_events") or []),
                "right": list(encoder_right.get("recent_a_rising_events") or []),
                "left_snapshot_signed_delta": int(
                    encoder_left_snapshot.get("pulse_delta", 0) or 0
                ),
                "right_snapshot_signed_delta": int(
                    encoder_right_snapshot.get("pulse_delta", 0) or 0
                ),
            },
        }
    )
    return base


def _send_track_reference(
    *,
    token: str,
    left_mps: float,
    right_mps: float,
    reason: str,
    timeout_s: float = 4.0,
) -> Dict[str, Any]:
    command = _send_command_checked(
        "set_track_velocity",
        token=str(token),
        timeout_s=float(timeout_s),
        left_mps=float(left_mps),
        right_mps=float(right_mps),
        motion_source="STATE",
    )
    command["primitive_validator_reason"] = str(reason)
    command["left_mps"] = float(left_mps)
    command["right_mps"] = float(right_mps)
    return command


def _stop_track_reference(*, token: str, stop_timeout_s: float) -> Dict[str, Any]:
    payload: Dict[str, Any] = {"ok": False, "stop_command": {}, "stopped_status": {}, "error": ""}
    try:
        stop_command = _send_track_reference(
            token=str(token),
            left_mps=0.0,
            right_mps=0.0,
            reason="zero_track_stop",
            timeout_s=max(4.0, float(stop_timeout_s) + 1.0),
        )
        stopped_status = _wait_until_stopped(timeout_s=float(stop_timeout_s), poll_s=DEFAULT_POLL_S)
        payload.update({"ok": True, "stop_command": stop_command, "stopped_status": stopped_status})
        return payload
    except Exception as exc:
        payload["error"] = str(exc)
        _safe_stop_best_effort(str(token))
        try:
            payload["stopped_status"] = _wait_until_stopped(timeout_s=float(stop_timeout_s), poll_s=DEFAULT_POLL_S)
            payload["ok"] = True
        except Exception as stop_exc:
            payload["error"] = f"{payload['error']}; fallback_stop={stop_exc}"
        return payload


def _wait_for_idle_verify(timeout_s: float, poll_s: float = 0.10) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    last_status: Dict[str, Any] = {}
    while time.monotonic() <= deadline:
        status = _read_json(STATUS_PATH)
        if status:
            last_status = dict(status)
            state = _upper(status.get("state"))
            pwm = dict(status.get("pwm") or {})
            pwm_left = abs(_safe_float(pwm.get("left"), 0.0))
            pwm_right = abs(_safe_float(pwm.get("right"), 0.0))
            if state == "IDLE" and pwm_left <= 0.03 and pwm_right <= 0.03:
                return {"ok": True, "status": status}
        time.sleep(max(0.02, float(poll_s)))
    return {"ok": False, "status": last_status}


def _prepare_run_start_state(*, token: str, stop_timeout_s: float) -> Dict[str, Any]:
    stop = _stop_track_reference(token=str(token), stop_timeout_s=float(stop_timeout_s))
    idle = _wait_for_idle_verify(timeout_s=float(stop_timeout_s))
    return {
        "ok": bool(stop.get("ok", False)) and bool(idle.get("ok", False)),
        "stop": stop,
        "idle_verify": idle,
        "status": dict(idle.get("status") or stop.get("stopped_status") or {}),
    }


def _collect_settle_samples(
    *,
    start_pose: Dict[str, float],
    expected_sign: float,
    duration_s: float,
    poll_s: float,
) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    start_mono = time.monotonic()
    deadline = start_mono + max(0.0, float(duration_s))
    while time.monotonic() <= deadline:
        status = _read_json(STATUS_PATH)
        if status:
            samples.append(
                _sample_status(
                    status,
                    phase="settle",
                    start_pose=start_pose,
                    expected_sign=float(expected_sign),
                    t_rel_s=max(0.0, time.monotonic() - start_mono),
                )
            )
        time.sleep(max(0.02, float(poll_s)))
    return samples


def _predicted_pivot_progress_deg(
    *,
    progress_deg: float,
    actual_omega_rad_s: float,
    expected_sign: float,
    horizon_s: float,
) -> float:
    directional_omega_rad_s = max(
        0.0,
        float(expected_sign) * float(actual_omega_rad_s),
    )
    return float(progress_deg) + math.degrees(
        directional_omega_rad_s * max(0.0, float(horizon_s))
    )


def run_pivot_case(
    case: PrimitiveCase,
    *,
    token: str,
    thresholds: Dict[str, float],
) -> Dict[str, Any]:
    stop_timeout_s = float(thresholds["stop_timeout_s"])
    poll_s = float(thresholds["poll_s"])
    target_abs_deg = abs(float(thresholds["target_angle_deg"]))
    expected_deg = float(case.expected_sign * target_abs_deg)
    start_idle = _wait_for_idle_verify(timeout_s=stop_timeout_s)
    if not bool(start_idle.get("ok", False)):
        start_idle = _prepare_run_start_state(token=str(token), stop_timeout_s=stop_timeout_s)
    start_status = dict(start_idle.get("status") or _wait_for_status(timeout_s=2.0))
    start_pose = _get_pose(start_status)

    entry_scale = max(0.05, min(1.0, float(thresholds["pivot_entry_speed_scale"])))
    brake_trigger_ratio = max(0.20, min(0.98, float(thresholds["pivot_brake_trigger_ratio"])))
    brake_scale = max(0.05, min(1.0, float(thresholds["pivot_brake_speed_scale"])))
    stop_window_deg = max(0.2, min(8.0, float(thresholds["pivot_stop_window_deg"])))
    stop_prediction_horizon_s = max(
        0.0,
        min(0.50, float(thresholds["pivot_stop_prediction_horizon_s"])),
    )
    entry_left = float(case.left_mps * entry_scale)
    entry_right = float(case.right_mps * entry_scale)
    brake_left = float(case.left_mps * brake_scale)
    brake_right = float(case.right_mps * brake_scale)
    envelope_events: List[Dict[str, Any]] = []

    def _push(left_mps: float, right_mps: float, reason: str) -> Dict[str, Any]:
        command = _send_track_reference(
            token=str(token),
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            reason=str(reason),
        )
        envelope_events.append(
            {
                "reason": str(reason),
                "left_mps": float(left_mps),
                "right_mps": float(right_mps),
                "sent_ts_wall": float(_safe_float(command.get("sent_ts_wall"), time.time())),
            }
        )
        return command

    start_cmd = _push(entry_left, entry_right, "entry")
    samples: List[Dict[str, Any]] = []
    last_status = dict(start_status)
    last_version = int(_status_version(start_status))
    last_change = time.monotonic()
    start_mono = time.monotonic()
    deadline = start_mono + max(0.5, float(thresholds["motion_timeout_s"]))
    progress_mark_deg = 0.0
    last_progress_mono = start_mono
    cruise_applied = bool(
        math.isclose(entry_left, float(case.left_mps), abs_tol=1e-9)
        and math.isclose(entry_right, float(case.right_mps), abs_tol=1e-9)
    )
    brake_applied = bool(
        math.isclose(brake_left, float(case.left_mps), abs_tol=1e-9)
        and math.isclose(brake_right, float(case.right_mps), abs_tol=1e-9)
    )
    stop_prediction: Dict[str, Any] = {}
    timeout = False
    no_progress = False
    terminal_reason = "UNKNOWN"

    try:
        while time.monotonic() <= deadline:
            now = time.monotonic()
            status = _read_json(STATUS_PATH)
            if status:
                last_status = dict(status)
                version = int(_status_version(status))
                if version != last_version:
                    last_version = version
                    last_change = now
                row = _sample_status(
                    status,
                    phase=case.name,
                    start_pose=start_pose,
                    expected_sign=float(case.expected_sign),
                    t_rel_s=max(0.0, now - start_mono),
                )
                samples.append(row)
                progress = float(_safe_float(row.get("directional_progress_deg"), 0.0))
                actual_omega_rad_s = float(_safe_float(row.get("actual_omega"), 0.0))
                predicted_progress_deg = _predicted_pivot_progress_deg(
                    progress_deg=progress,
                    actual_omega_rad_s=actual_omega_rad_s,
                    expected_sign=float(case.expected_sign),
                    horizon_s=stop_prediction_horizon_s,
                )
                if (not cruise_applied) and (now - start_mono) >= 0.30:
                    _push(case.left_mps, case.right_mps, "cruise")
                    cruise_applied = True
                if (not brake_applied) and progress >= target_abs_deg * brake_trigger_ratio:
                    _push(brake_left, brake_right, "brake")
                    brake_applied = True
                if predicted_progress_deg >= max(0.0, target_abs_deg - stop_window_deg):
                    stop_prediction = {
                        "observed_progress_deg": float(progress),
                        "actual_omega_rad_s": float(actual_omega_rad_s),
                        "horizon_s": float(stop_prediction_horizon_s),
                        "predicted_progress_deg": float(predicted_progress_deg),
                        "stop_threshold_deg": float(
                            max(0.0, target_abs_deg - stop_window_deg)
                        ),
                    }
                    terminal_reason = "PREDICTED_TARGET_YAW_WINDOW_REACHED"
                    break
                if progress >= float(progress_mark_deg) + float(thresholds["no_progress_min_step_deg"]):
                    progress_mark_deg = progress
                    last_progress_mono = now
                if _upper(row.get("state")) == "FAILSAFE":
                    terminal_reason = "RUNTIME_FAILSAFE"
                    break
                if (
                    (now - start_mono) >= float(thresholds["no_progress_grace_s"])
                    and (now - last_progress_mono) > float(thresholds["no_progress_timeout_s"])
                ):
                    no_progress = True
                    terminal_reason = "NO_PROGRESS"
                    break
            if (time.monotonic() - last_change) > 3.0:
                terminal_reason = "STATUS_STREAM_STALE"
                break
            time.sleep(max(0.02, poll_s))
        else:
            timeout = True
            terminal_reason = "TIMEOUT"
    except Exception:
        _safe_stop_best_effort(str(token))
        raise

    stop = _stop_track_reference(token=str(token), stop_timeout_s=stop_timeout_s)
    if not bool(stop.get("ok", False)):
        timeout = True
        terminal_reason = "STOP_TIMEOUT"

    settle_samples = _collect_settle_samples(
        start_pose=start_pose,
        expected_sign=float(case.expected_sign),
        duration_s=float(thresholds["settle_time_s"]),
        poll_s=poll_s,
    )
    all_samples = list(samples) + list(settle_samples)
    end_pose = dict((all_samples[-1].get("pose") or start_pose) if all_samples else start_pose)
    actual_deg = float(
        _normalize_angle_deg(float(end_pose.get("theta_deg", 0.0)) - float(start_pose.get("theta_deg", 0.0)))
    )
    analysis = analyze_pivot_case(
        case=case,
        expected_deg=expected_deg,
        actual_deg=actual_deg,
        timeout=bool(timeout),
        no_progress=bool(no_progress),
        terminal_reason=str(terminal_reason),
        samples=all_samples,
        thresholds=thresholds,
    )
    analysis.update(
        {
            "raw": {
                "start_command": start_cmd,
                "stop": stop,
                "start_pose": start_pose,
                "end_pose": end_pose,
                "last_status": last_status,
                "motion_samples": samples,
                "settle_samples": settle_samples,
                "envelope_events": envelope_events,
                "stop_prediction": stop_prediction,
            },
            "pivot_envelope": {
                "entry_speed_scale": float(entry_scale),
                "cruise_left_mps": float(case.left_mps),
                "cruise_right_mps": float(case.right_mps),
                "entry_left_mps": float(entry_left),
                "entry_right_mps": float(entry_right),
                "brake_trigger_ratio": float(brake_trigger_ratio),
                "brake_speed_scale": float(brake_scale),
                "brake_left_mps": float(brake_left),
                "brake_right_mps": float(brake_right),
                "stop_window_deg": float(stop_window_deg),
                "stop_prediction_horizon_s": float(stop_prediction_horizon_s),
            },
        }
    )
    return analysis


def _stop_twist(*, token: str, stop_timeout_s: float) -> Dict[str, Any]:
    payload: Dict[str, Any] = {"ok": False, "stop_command": {}, "stopped_status": {}, "error": ""}
    try:
        stop_command = _send_command_checked(
            "set_twist",
            token=str(token),
            timeout_s=max(4.0, float(stop_timeout_s) + 1.0),
            v=0.0,
            omega=0.0,
            motion_source="STATE",
        )
        stopped_status = _wait_until_stopped(timeout_s=float(stop_timeout_s), poll_s=DEFAULT_POLL_S)
        payload.update({"ok": True, "stop_command": stop_command, "stopped_status": stopped_status})
        return payload
    except Exception as exc:
        payload["error"] = str(exc)
        _safe_stop_best_effort(str(token))
        try:
            payload["stopped_status"] = _wait_until_stopped(timeout_s=float(stop_timeout_s), poll_s=DEFAULT_POLL_S)
            payload["ok"] = True
        except Exception as stop_exc:
            payload["error"] = f"{payload['error']}; fallback_stop={stop_exc}"
        return payload


def _primitive_matches(case: PrimitiveCase, value: Any) -> bool:
    primitive = _upper(value)
    if case.expected_primitive in EXPECTED_ARC_PRIMITIVES:
        return primitive in EXPECTED_ARC_PRIMITIVES
    return primitive == _upper(case.expected_primitive)


def _guidance_heading_correction_sample(sample: Dict[str, Any]) -> bool:
    """Identify an L7A guidance correction inside a straight segment.

    This is not a generic classifier tolerance.  The actual instantaneous
    motion remains reported as ``DIFF_ARC_GENTLE``; the validator only treats
    it as belonging to the commanded straight family when all three command
    surfaces stay STRAIGHT and the guidance heading-hold owner is
    explicitly active.  Segment-level physical drift is checked separately.
    """

    src = dict(sample or {})
    return bool(
        _upper(src.get("turn_primitive_actual")) == "DIFF_ARC_GENTLE"
        and bool(src.get("guidance_heading_hold_active", False))
        and _upper(src.get("turn_primitive_requested")) == EXPECTED_STRAIGHT_PRIMITIVE
        and _upper(src.get("turn_primitive_limited")) == EXPECTED_STRAIGHT_PRIMITIVE
        and _upper(src.get("turn_primitive_executed")) == EXPECTED_STRAIGHT_PRIMITIVE
        and not bool(src.get("primitive_contract_violation", False))
        and not bool(src.get("control_execution_contract_violation", False))
    )


def _twist_active_samples(samples: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    active: List[Dict[str, Any]] = []
    for sample in samples:
        if _upper(sample.get("sample_phase")) == "SETTLE":
            continue
        exec_mode = _upper(sample.get("motion_execution_mode") or sample.get("resolved_execution_mode"))
        ref_abs = max(
            abs(_safe_float(sample.get("target_left_mps"), 0.0)),
            abs(_safe_float(sample.get("target_right_mps"), 0.0)),
            abs(_safe_float(sample.get("resolved_v"), 0.0)),
            abs(_safe_float(sample.get("resolved_omega"), 0.0)),
        )
        runtime_moving = _upper(sample.get("state")) not in ("", "IDLE")
        if ref_abs >= 0.005 or (exec_mode == EXPECTED_TWIST_EXECUTION_MODE and runtime_moving):
            active.append(dict(sample))
    return active


def _independent_status_frames(
    samples: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Keep one observation per native runtime status frame.

    The validator may poll faster than ``status.json`` is published. Repeated
    reads of the same ``status_version`` are not independent measurements and
    must not overweight percentile or ratio gates. Legacy/offline fixtures
    without a positive status version remain sample-based.
    """
    independent: List[Dict[str, Any]] = []
    seen_versions = set()
    for sample in samples:
        row = dict(sample or {})
        version = int(row.get("status_version", 0) or 0)
        if version > 0:
            if version in seen_versions:
                continue
            seen_versions.add(version)
        independent.append(row)
    return independent


def analyze_twist_case(
    *,
    case: PrimitiveCase,
    samples: Sequence[Dict[str, Any]],
    timeout: bool,
    terminal_reason: str,
    thresholds: Dict[str, float],
) -> Dict[str, Any]:
    sample_list = [dict(sample or {}) for sample in samples]
    active_observations = _twist_active_samples(sample_list)
    active_count = len(active_observations)
    active = _independent_status_frames(active_observations)
    settled_active = _settled_actual_classifier_window(
        active,
        thresholds["actual_primitive_transition_settle_s"],
    )
    command_chain_samples = settled_active if settled_active else active
    heading_values = [_safe_float(s.get("heading_change_deg"), 0.0) for s in sample_list]
    final_heading_deg = float(heading_values[-1]) if heading_values else 0.0
    signed_progress_max = max([float(case.expected_sign) * float(v) for v in heading_values] or [0.0])
    displacement_max = max([_safe_float(s.get("displacement_from_start_m"), 0.0) for s in sample_list] or [0.0])
    actual_v_abs = [abs(_safe_float(s.get("actual_v"), 0.0)) for s in active]
    actual_omega = [_safe_float(s.get("actual_omega"), 0.0) for s in active]
    actual_omega_abs = [abs(v) for v in actual_omega]
    actual_v_abs_summary = _summary(actual_v_abs)
    actual_omega_abs_summary = _summary(actual_omega_abs)
    watchdog_hz = [_safe_float(s.get("watchdog_freq_hz"), math.nan) for s in active]
    loop_budget = [_safe_float(s.get("loop_budget_total_ema_ms"), math.nan) for s in active]
    logger_depth = [_safe_float(s.get("logger_queue_depth"), math.nan) for s in active]
    logger_flush = [_safe_float(s.get("logger_flush_duration_ms"), math.nan) for s in active]
    sensor_truth_samples = settled_active if settled_active else (active if active else sample_list)
    lidar_conf = [_safe_float(s.get("lidar_pose_confidence"), math.nan) for s in sensor_truth_samples]
    lidar_gap = [_safe_float(s.get("lidar_ekf_applied_gap_s"), math.nan) for s in sensor_truth_samples]
    primitive_req_ratio = _ratio_matching(command_chain_samples, lambda s: _primitive_matches(case, s.get("turn_primitive_requested")))
    primitive_lim_ratio = _ratio_matching(command_chain_samples, lambda s: _primitive_matches(case, s.get("turn_primitive_limited")))
    primitive_exe_ratio = _ratio_matching(command_chain_samples, lambda s: _primitive_matches(case, s.get("turn_primitive_executed")))
    actual_classifier_window = settled_active
    actual_classifier_samples = [
        sample for sample in actual_classifier_window if _actual_classifier_eligible(sample)
    ]
    primitive_act_exact_ratio = _ratio_matching(
        actual_classifier_samples,
        lambda s: _primitive_matches(case, s.get("turn_primitive_actual")),
    )
    straight_physical_candidate = bool(
        case.expected_primitive == EXPECTED_STRAIGHT_PRIMITIVE
        and displacement_max >= max(float(case.min_distance_m), thresholds["straight_min_distance_m"])
        and abs(float(final_heading_deg)) <= thresholds["straight_heading_abs_max_deg"]
        and actual_v_abs_summary.get("p50") is not None
        and float(actual_v_abs_summary.get("p50") or 0.0) >= 0.008
    )
    guidance_heading_correction_samples = [
        sample
        for sample in actual_classifier_samples
        if _guidance_heading_correction_sample(sample)
    ]
    primitive_act_ratio = _ratio_matching(
        actual_classifier_samples,
        lambda s: bool(
            _primitive_matches(case, s.get("turn_primitive_actual"))
            or (
                straight_physical_candidate
                and _guidance_heading_correction_sample(s)
            )
        ),
    )
    actual_classifier_coverage = _ratio(len(actual_classifier_samples), len(actual_classifier_window))
    twist_exec_ratio = _ratio_matching(
        active,
        lambda s: _upper(s.get("motion_execution_mode") or s.get("resolved_execution_mode")) == EXPECTED_TWIST_EXECUTION_MODE,
    )
    motion_actual_ssot_values = [_upper(s.get("motion_actual_ssot")) for s in sample_list if _upper(s.get("motion_actual_ssot"))]
    motion_actual_ssot_ratio = _ratio(
        sum(1 for value in motion_actual_ssot_values if value == EXPECTED_MOTION_ACTUAL_SSOT),
        len(motion_actual_ssot_values),
    )
    odom_modes = _unique(_upper(s.get("odometry_mode")) for s in sample_list)
    command_owner_conflicts = sum(1 for s in active if bool(s.get("command_owner_conflict", False)))
    service_active_samples = sum(1 for s in active if bool(s.get("service_motion_active", False)))
    actual_contract_violations = sum(
        1 for s in actual_classifier_window if bool(s.get("primitive_contract_violation", False))
    )
    transition_contract_violations = sum(
        1
        for s in active
        if s not in actual_classifier_window and bool(s.get("primitive_contract_violation", False))
    )
    execution_contract_violations = sum(1 for s in active if bool(s.get("control_execution_contract_violation", False)))
    slow_tick_delta, slow_observed_tick_delta, slow_tick_ratio = _slow_tick_window(active)
    slow_io_delta = _max_counter_delta(active, "slow_io_event_count")
    slow_lidar_delta = _max_counter_delta(active, "slow_lidar_spike_count")
    slow_resolver_delta = _max_counter_delta(active, "slow_resolver_spike_count")
    slow_gc_delta = _max_counter_delta(active, "slow_gc_count")
    slow_none_delta = _max_counter_delta(active, "slow_none_count")
    finite_watchdog = [v for v in watchdog_hz if math.isfinite(float(v))]
    loop_below_ratio = _ratio(sum(1 for v in finite_watchdog if float(v) < 45.0), len(finite_watchdog))
    actual_omega_expected_sign_ratio = _ratio(
        sum(1 for v in actual_omega if abs(float(v)) >= 0.020 and float(case.expected_sign) * float(v) > 0.0),
        len([v for v in actual_omega if abs(float(v)) >= 0.020]),
    )

    metrics = {
        "sample_count": len(sample_list),
        "active_sample_count": active_count,
        "active_unique_status_frame_count": len(active),
        "command_chain_settled_frame_count": len(command_chain_samples),
        "expected_primitive": str(case.expected_primitive),
        "final_heading_deg": round(float(final_heading_deg), 3),
        "signed_progress_max_deg": round(float(signed_progress_max), 3),
        "displacement_max_m": round(float(displacement_max), 4),
        "actual_v_abs": actual_v_abs_summary,
        "actual_omega_abs": actual_omega_abs_summary,
        "actual_omega_expected_sign_ratio": actual_omega_expected_sign_ratio,
        "watchdog_frequency_hz": _summary(watchdog_hz),
        "loop_budget_total_ema_ms": _summary(loop_budget),
        "logger_queue_depth": _summary(logger_depth),
        "logger_flush_duration_ms": _summary(logger_flush),
        "lidar_pose_confidence": _summary(lidar_conf),
        "lidar_ekf_applied_gap_s": _summary(lidar_gap),
        "twist_exec_ratio": twist_exec_ratio,
        "primitive_requested_expected_ratio": primitive_req_ratio,
        "primitive_limited_expected_ratio": primitive_lim_ratio,
        "primitive_executed_expected_ratio": primitive_exe_ratio,
        "primitive_actual_expected_ratio": primitive_act_ratio,
        "primitive_actual_exact_ratio": primitive_act_exact_ratio,
        "guidance_heading_correction_accepted_samples": (
            len(guidance_heading_correction_samples)
            if straight_physical_candidate
            else 0
        ),
        "guidance_heading_correction_observed_samples": len(guidance_heading_correction_samples),
        "actual_classifier_semantics": "exact_or_guidance_heading_hold_gentle_with_physical_straight_pass",
        "actual_classifier_eligible_sample_count": len(actual_classifier_samples),
        "actual_classifier_settled_window_sample_count": len(actual_classifier_window),
        "actual_classifier_transition_sample_count": max(0, len(active) - len(actual_classifier_window)),
        "actual_classifier_transition_settle_s": thresholds["actual_primitive_transition_settle_s"],
        "actual_classifier_coverage_ratio": actual_classifier_coverage,
        "motion_actual_ssot_ratio": motion_actual_ssot_ratio,
        "motion_actual_ssot_values_seen": _unique(motion_actual_ssot_values),
        "control_modes_seen": _unique(s.get("control_mode") for s in sample_list),
        "motion_state_modes_seen": _unique(s.get("motion_state_mode") for s in sample_list),
        "odometry_modes_seen": odom_modes,
        "execution_modes_seen": _unique(
            _upper(s.get("motion_execution_mode") or s.get("resolved_execution_mode")) for s in active
        ),
        "turn_primitives_seen": {
            "requested": _unique(_upper(s.get("turn_primitive_requested")) for s in active),
            "limited": _unique(_upper(s.get("turn_primitive_limited")) for s in active),
            "executed": _unique(_upper(s.get("turn_primitive_executed")) for s in active),
            "actual": _unique(_upper(s.get("turn_primitive_actual")) for s in active),
        },
        "turn_primitives_settled_seen": {
            "requested": _unique(_upper(s.get("turn_primitive_requested")) for s in command_chain_samples),
            "limited": _unique(_upper(s.get("turn_primitive_limited")) for s in command_chain_samples),
            "executed": _unique(_upper(s.get("turn_primitive_executed")) for s in command_chain_samples),
            "actual": _unique(_upper(s.get("turn_primitive_actual")) for s in command_chain_samples),
        },
        "command_owner_conflict_samples": int(command_owner_conflicts),
        "service_motion_active_samples": int(service_active_samples),
        "actual_contract_violation_samples": int(actual_contract_violations),
        "transition_contract_violation_samples": int(transition_contract_violations),
        "execution_contract_violation_samples": int(execution_contract_violations),
        "slow_tick_delta": int(slow_tick_delta),
        "slow_observed_tick_delta": int(slow_observed_tick_delta),
        "slow_tick_ratio": float(slow_tick_ratio),
        "slow_io_event_delta": int(slow_io_delta),
        "slow_lidar_spike_delta": int(slow_lidar_delta),
        "slow_resolver_spike_delta": int(slow_resolver_delta),
        "slow_gc_delta": int(slow_gc_delta),
        "slow_none_delta": int(slow_none_delta),
        "loop_below_45_ratio": loop_below_ratio,
        "logger_dropped_messages_max": max([int(s.get("logger_dropped_messages", 0) or 0) for s in active] or [0]),
        "logger_write_errors_max": max([int(s.get("logger_write_errors", 0) or 0) for s in active] or [0]),
    }

    watchdog_p10 = metrics["watchdog_frequency_hz"].get("p10")
    loop_budget_p95 = metrics["loop_budget_total_ema_ms"].get("p95")
    logger_depth_max = metrics["logger_queue_depth"].get("max")
    logger_flush_p95 = metrics["logger_flush_duration_ms"].get("p95")
    lidar_conf_p50 = metrics["lidar_pose_confidence"].get("p50")
    lidar_gap_p95 = metrics["lidar_ekf_applied_gap_s"].get("p95")
    straight_physical_ok = bool(straight_physical_candidate)
    arc_physical_ok = bool(
        case.expected_primitive in EXPECTED_ARC_PRIMITIVES
        and signed_progress_max >= max(float(case.min_yaw_deg), thresholds["arc_min_yaw_deg"])
        and displacement_max >= max(float(case.min_distance_m), thresholds["arc_min_distance_m"])
        and (actual_omega_expected_sign_ratio is None or float(actual_omega_expected_sign_ratio) >= 0.55)
    )
    camera_off_ok = bool(sample_list) and all(not bool(s.get("camera_enabled", False)) for s in sample_list)
    unified_ok = bool(sample_list) and all(_upper(s.get("control_mode")) == CANONICAL_CONTROL_MODE for s in sample_list)
    safety_ok = bool(sample_list) and all(
        bool(s.get("safety_allow", False))
        and _upper(s.get("state")) != "FAILSAFE"
        and not bool(s.get("watchdog_stop_triggered", False))
        for s in sample_list
    )

    gates: Dict[str, Dict[str, Any]] = {
        "motion_evidence": _gate(
            "PASS" if active_count >= int(thresholds["min_motion_samples"]) and not bool(timeout) else "FAIL",
            observed={
                "active_sample_count": active_count,
                "active_unique_status_frame_count": len(active),
                "timeout": bool(timeout),
                "terminal_reason": str(terminal_reason),
            },
            requirement="bounded live twist primitive produces enough active samples without timeout",
        ),
        "unified_single_motion_system": _gate(
            "PASS" if unified_ok and camera_off_ok else "FAIL",
            observed={
                "control_modes_seen": metrics["control_modes_seen"],
                "camera_enabled_samples": sum(1 for s in sample_list if bool(s.get("camera_enabled", False))),
            },
            requirement="UNIFIED motion mode only, camera disabled during primitive validation",
        ),
        "twist_exec_ssot_path": _gate(
            "PASS"
            if (
                twist_exec_ratio is not None
                and float(twist_exec_ratio) >= thresholds["track_exec_ratio_min"]
                and command_owner_conflicts == 0
                and service_active_samples == 0
                and len(motion_actual_ssot_values) > 0
                and motion_actual_ssot_ratio is not None
                and float(motion_actual_ssot_ratio) >= thresholds["motion_actual_ssot_ratio_min"]
                and (not odom_modes or all(mode in ("", "LIDAR_FIRST") for mode in odom_modes))
            )
            else "FAIL",
            observed={
                "twist_exec_ratio": twist_exec_ratio,
                "execution_modes_seen": metrics["execution_modes_seen"],
                "motion_actual_ssot_ratio": motion_actual_ssot_ratio,
                "odometry_modes_seen": odom_modes,
                "command_owner_conflict_samples": command_owner_conflicts,
                "service_motion_active_samples": service_active_samples,
            },
            requirement="one routed TWIST_EXEC owner, EKF motion actual SSOT, no service/direct/legacy path evidence",
        ),
        "primitive_command_chain": _gate(
            "PASS"
            if (
                execution_contract_violations == 0
                and primitive_req_ratio is not None
                and primitive_lim_ratio is not None
                and primitive_exe_ratio is not None
                and float(primitive_req_ratio) >= thresholds["primitive_expected_ratio_min"]
                and float(primitive_lim_ratio) >= thresholds["primitive_expected_ratio_min"]
                and float(primitive_exe_ratio) >= thresholds["primitive_expected_ratio_min"]
            )
            else "FAIL",
            observed={
                "turn_primitives_seen": metrics["turn_primitives_settled_seen"],
                "settled_frame_count": len(command_chain_samples),
                "primitive_requested_expected_ratio": primitive_req_ratio,
                "primitive_limited_expected_ratio": primitive_lim_ratio,
                "primitive_executed_expected_ratio": primitive_exe_ratio,
                "execution_contract_violation_samples": execution_contract_violations,
            },
            requirement="requested, limited and executed primitive match the expected straight/arc family",
        ),
        "actual_primitive_classifier": _gate(
            "PASS"
            if primitive_act_ratio is not None
            and float(primitive_act_ratio) >= thresholds["actual_primitive_expected_ratio_min"]
            and actual_classifier_coverage is not None
            and float(actual_classifier_coverage) >= thresholds["actual_primitive_coverage_min"]
            and actual_contract_violations == 0
            else (
                "INCONCLUSIVE"
                if actual_classifier_coverage is None
                or float(actual_classifier_coverage) < thresholds["actual_primitive_coverage_min"]
                else "FAIL"
            ),
            observed={
                "turn_primitives_seen": metrics["turn_primitives_seen"],
                "primitive_actual_expected_ratio": primitive_act_ratio,
                "primitive_actual_exact_ratio": primitive_act_exact_ratio,
                "guidance_heading_correction_accepted_samples": metrics[
                    "guidance_heading_correction_accepted_samples"
                ],
                "guidance_heading_correction_observed_samples": metrics[
                    "guidance_heading_correction_observed_samples"
                ],
                "actual_classifier_semantics": metrics["actual_classifier_semantics"],
                "actual_classifier_eligible_sample_count": len(actual_classifier_samples),
                "actual_classifier_settled_window_sample_count": len(actual_classifier_window),
                "actual_classifier_transition_sample_count": max(0, len(active) - len(actual_classifier_window)),
                "actual_classifier_coverage_ratio": actual_classifier_coverage,
                "actual_contract_violation_samples": actual_contract_violations,
                "transition_contract_violation_samples": transition_contract_violations,
            },
            requirement="enough ready/reliable settled measurements match the exact primitive, or a physically passing STRAIGHT segment contains only executor-owned gentle straight-hold corrections; settled contract violations are forbidden",
        ),
        "physical_twist_quality": _gate(
            "PASS" if straight_physical_ok or arc_physical_ok else "FAIL",
            observed={
                "expected_primitive": str(case.expected_primitive),
                "final_heading_deg": metrics["final_heading_deg"],
                "signed_progress_max_deg": metrics["signed_progress_max_deg"],
                "displacement_max_m": metrics["displacement_max_m"],
                "actual_v_abs": metrics["actual_v_abs"],
                "actual_omega_abs": metrics["actual_omega_abs"],
            },
            requirement="straight moves forward with low heading drift; arc moves forward while turning in the requested direction",
        ),
        "sensor_truth_runtime": _gate(
            "PASS"
            if (
                (lidar_conf_p50 is None or float(lidar_conf_p50) >= thresholds["lidar_confidence_p50_min"])
                and (lidar_gap_p95 is None or float(lidar_gap_p95) <= thresholds["lidar_ekf_gap_p95_max_s"])
            )
            else "FAIL",
            observed={
                "lidar_pose_confidence": metrics["lidar_pose_confidence"],
                "lidar_ekf_applied_gap_s": metrics["lidar_ekf_applied_gap_s"],
                "settled_unique_status_frame_count": len(sensor_truth_samples),
            },
            requirement="EKF/LIDAR truth surface remains fresh enough for live primitive measurement",
        ),
        "safety_runtime": _gate(
            "PASS" if safety_ok else "FAIL",
            observed={"safety_ok": bool(safety_ok), "front_min_m": _summary([s.get("front_m") for s in sample_list]).get("min")},
            requirement="no failsafe/watchdog stop during twist primitive",
        ),
        "software_loop_runtime": _gate(
            "PASS"
            if (
                (watchdog_p10 is None or float(watchdog_p10) >= thresholds["loop_frequency_p10_min_hz"])
                and (loop_below_ratio is None or float(loop_below_ratio) <= thresholds["loop_below_45_ratio_max"])
                and (loop_budget_p95 is None or float(loop_budget_p95) <= thresholds["loop_budget_p95_max_ms"])
                and int(metrics["logger_dropped_messages_max"]) == 0
                and int(metrics["logger_write_errors_max"]) == 0
                and (logger_depth_max is None or float(logger_depth_max) <= thresholds["logger_queue_depth_max"])
                and (logger_flush_p95 is None or float(logger_flush_p95) <= thresholds["logger_flush_p95_max_ms"])
                and float(slow_tick_ratio) <= thresholds["slow_tick_ratio_max"]
            )
            else "FAIL",
            observed={
                "watchdog_frequency_hz": metrics["watchdog_frequency_hz"],
                "loop_below_45_ratio": loop_below_ratio,
                "loop_budget_total_ema_ms": metrics["loop_budget_total_ema_ms"],
                "slow_tick_delta": slow_tick_delta,
                "slow_observed_tick_delta": slow_observed_tick_delta,
                "slow_tick_ratio": slow_tick_ratio,
                "slow_io_event_delta": slow_io_delta,
                "slow_lidar_spike_delta": slow_lidar_delta,
                "slow_resolver_spike_delta": slow_resolver_delta,
                "slow_gc_delta": slow_gc_delta,
                "slow_none_delta": slow_none_delta,
            },
            requirement="control loop, logger and slow-tick surfaces stay bounded during twist primitive",
        ),
    }
    status, failed, inconclusive = _phase_status_from_gates(gates)
    return {
        "schema": "M3_MOTION_PRIMITIVE_TWIST_CASE_V1",
        "case_name": str(case.name),
        "command": {
            "type": "set_twist",
            "motion_source": "STATE",
            "v_mps": float(case.v_mps),
            "omega_rad_s": float(case.omega_rad_s),
            "duration_s": float(case.duration_s),
        },
        "status": status,
        "success": status == "PASS",
        "pass": status == "PASS",
        "expected_deg": None,
        "actual_deg": round(float(final_heading_deg), 3),
        "angle_error_deg": None,
        "timeout": bool(timeout),
        "no_progress": False,
        "terminal_reason": str(terminal_reason),
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "metrics": metrics,
        "sample_count": len(sample_list),
    }


def run_twist_case(
    case: PrimitiveCase,
    *,
    token: str,
    thresholds: Dict[str, float],
) -> Dict[str, Any]:
    stop_timeout_s = float(thresholds["stop_timeout_s"])
    poll_s = float(thresholds["poll_s"])
    start_idle = _wait_for_idle_verify(timeout_s=stop_timeout_s)
    if not bool(start_idle.get("ok", False)):
        start_idle = _prepare_run_start_state(token=str(token), stop_timeout_s=stop_timeout_s)
    start_status = dict(start_idle.get("status") or _wait_for_status(timeout_s=2.0))
    start_pose = _get_pose(start_status)
    start_cmd = _send_command_checked(
        "set_twist",
        token=str(token),
        timeout_s=4.0,
        v=float(case.v_mps),
        omega=float(case.omega_rad_s),
        motion_source="STATE",
    )
    samples: List[Dict[str, Any]] = []
    start_mono = time.monotonic()
    deadline = start_mono + max(0.2, float(case.duration_s))
    last_keepalive = start_mono
    last_status = dict(start_status)
    last_version = int(_status_version(start_status))
    last_change = time.monotonic()
    timeout = False
    terminal_reason = "DURATION_REACHED"
    try:
        while time.monotonic() <= deadline:
            now = time.monotonic()
            if now - last_keepalive >= 0.25:
                _append_command(
                    "set_twist",
                    token=str(token),
                    v=float(case.v_mps),
                    omega=float(case.omega_rad_s),
                    motion_source="STATE",
                )
                last_keepalive = now
            status = _read_json(STATUS_PATH)
            if status:
                last_status = dict(status)
                version = int(_status_version(status))
                if version != last_version:
                    last_version = version
                    last_change = now
                row = _sample_status(
                    status,
                    phase=case.name,
                    start_pose=start_pose,
                    expected_sign=float(case.expected_sign),
                    t_rel_s=max(0.0, now - start_mono),
                )
                samples.append(row)
                if _upper(row.get("state")) == "FAILSAFE":
                    terminal_reason = "RUNTIME_FAILSAFE"
                    break
            if time.monotonic() - last_change > 3.0:
                timeout = True
                terminal_reason = "STATUS_STREAM_STALE"
                break
            time.sleep(max(0.02, poll_s))
    except Exception:
        _safe_stop_best_effort(str(token))
        raise
    stop = _stop_twist(token=str(token), stop_timeout_s=stop_timeout_s)
    if not bool(stop.get("ok", False)):
        timeout = True
        terminal_reason = "STOP_TIMEOUT"
    settle_samples = _collect_settle_samples(
        start_pose=start_pose,
        expected_sign=float(case.expected_sign),
        duration_s=float(thresholds["settle_time_s"]),
        poll_s=poll_s,
    )
    all_samples = list(samples) + list(settle_samples)
    analysis = analyze_twist_case(
        case=case,
        samples=all_samples,
        timeout=bool(timeout),
        terminal_reason=str(terminal_reason),
        thresholds=thresholds,
    )
    analysis.update(
        {
            "raw": {
                "start_command": start_cmd,
                "stop": stop,
                "start_pose": start_pose,
                "end_pose": dict((all_samples[-1].get("pose") or start_pose) if all_samples else start_pose),
                "last_status": last_status,
                "motion_samples": samples,
                "settle_samples": settle_samples,
                "envelope_events": [
                    {
                        "reason": "twist_duration",
                        "v_mps": float(case.v_mps),
                        "omega_rad_s": float(case.omega_rad_s),
                        "duration_s": float(case.duration_s),
                        "sent_ts_wall": float(_safe_float(start_cmd.get("sent_ts_wall"), time.time())),
                    }
                ],
            }
        }
    )
    return analysis


def _active_motion_samples(samples: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    active: List[Dict[str, Any]] = []
    for sample in samples:
        ref_abs = max(
            abs(_safe_float(sample.get("target_left_mps"), 0.0)),
            abs(_safe_float(sample.get("target_right_mps"), 0.0)),
            abs(_safe_float(sample.get("requested_track_left_mps"), 0.0)),
            abs(_safe_float(sample.get("requested_track_right_mps"), 0.0)),
        )
        runtime_moving = _upper(sample.get("state")) not in ("", "IDLE")
        if _upper(sample.get("sample_phase")) != "SETTLE" and (
            ref_abs >= 0.005
            or (
                _upper(sample.get("motion_execution_mode")) == EXPECTED_EXECUTION_MODE
                and runtime_moving
            )
            or abs(_safe_float(sample.get("actual_omega"), 0.0)) >= 0.030
        ):
            active.append(dict(sample))
    return active


def _ratio_matching(samples: Sequence[Dict[str, Any]], predicate) -> Optional[float]:
    return _ratio(sum(1 for sample in samples if predicate(sample)), len(samples))


def _actual_classifier_eligible(sample: Dict[str, Any]) -> bool:
    src = dict(sample or {})
    explicit_keys = {
        "actual_primitive_measurement_available",
        "actual_primitive_measurement_ready",
        "actual_primitive_measurement_reliable",
    }
    if not any(key in src for key in explicit_keys):
        return _upper(src.get("turn_primitive_actual")) not in ("", "UNKNOWN")
    return bool(
        src.get("actual_primitive_measurement_available", False)
        and src.get("actual_primitive_measurement_ready", False)
        and src.get("actual_primitive_measurement_reliable", False)
        and _upper(src.get("turn_primitive_actual")) not in ("", "UNKNOWN")
    )


def _settled_actual_classifier_window(
    samples: Sequence[Dict[str, Any]],
    settle_s: float,
) -> List[Dict[str, Any]]:
    """Exclude only command-transition frames from actual primitive gating.

    Missing segment age remains eligible for backward-compatible offline data;
    live samples with a native segment age must reach the declared settle time.
    """
    return [
        dict(sample)
        for sample in samples
        if not _is_finite(sample.get("motion_segment_age_s"))
        or _safe_float(sample.get("motion_segment_age_s"), 0.0) >= float(settle_s)
    ]


def _slow_tick_window(samples: Sequence[Dict[str, Any]]) -> tuple[int, int, float]:
    sample_list = [dict(sample or {}) for sample in samples]
    slow_delta = _max_counter_delta(sample_list, "slow_tick_count")
    observed_delta = _max_counter_delta(sample_list, "slow_observed_tick_count")
    if observed_delta <= 0 and len(sample_list) >= 2:
        times = [_safe_float(sample.get("ts"), math.nan) for sample in sample_list]
        finite_times = [value for value in times if math.isfinite(float(value))]
        if len(finite_times) >= 2:
            observed_delta = max(1, int(round((max(finite_times) - min(finite_times)) * 50.0)))
    ratio = float(slow_delta) / float(max(1, observed_delta))
    return int(slow_delta), int(observed_delta), float(ratio)


def _observation_metrics(samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    sample_list = [dict(sample or {}) for sample in samples]
    versions = [
        int(sample.get("status_version", 0) or 0)
        for sample in sample_list
        if int(sample.get("status_version", 0) or 0) > 0
    ]
    unique_status_frames = len(set(versions)) if versions else len(sample_list)
    times = [
        _safe_float(sample.get("t_rel_s"), math.nan)
        for sample in sample_list
        if math.isfinite(_safe_float(sample.get("t_rel_s"), math.nan))
    ]
    span_s = max(0.0, max(times) - min(times)) if len(times) >= 2 else 0.0
    return {
        "unique_status_frame_count": int(unique_status_frames),
        "observation_span_s": float(span_s),
    }


def _onset_measurements(
    samples: Sequence[Dict[str, Any]],
    *,
    expected_primitive: str,
    onset_window_s: float = 0.35,
) -> List[Dict[str, Any]]:
    unique: List[Dict[str, Any]] = []
    seen_versions = set()
    for sample in samples:
        status_version = int(sample.get("status_version", 0) or 0)
        key = ("version", status_version) if status_version > 0 else (
            "sample",
            len(unique),
        )
        if key in seen_versions:
            continue
        seen_versions.add(key)
        segment_age_s = _safe_float(sample.get("motion_segment_age_s"), math.nan)
        if not math.isfinite(segment_age_s) or segment_age_s > float(onset_window_s):
            continue
        actual = _upper(sample.get("turn_primitive_actual"))
        unique.append(
            {
                "status_version": status_version,
                "t_rel_s": _safe_float(sample.get("t_rel_s"), math.nan),
                "segment_age_s": segment_age_s,
                "ekf_v_mps": _safe_float(sample.get("ekf_v_mps"), math.nan),
                "pose_v_mps": _safe_float(sample.get("pose_v_mps"), math.nan),
                "actual_left_mps": _safe_float(sample.get("actual_left_mps"), math.nan),
                "actual_right_mps": _safe_float(sample.get("actual_right_mps"), math.nan),
                "imu_omega_rad_s": _safe_float(sample.get("imu_gyro_z_rad_s"), math.nan),
                "commanded_v_mps": _safe_float(sample.get("commanded_v_mps"), math.nan),
                "commanded_omega_rad_s": _safe_float(
                    sample.get("commanded_omega_rad_s"),
                    math.nan,
                ),
                "actual_v_mps": _safe_float(sample.get("actual_v"), math.nan),
                "actual_omega_rad_s": _safe_float(sample.get("actual_omega"), math.nan),
                "turn_primitive_requested": _upper(
                    sample.get("turn_primitive_requested")
                ),
                "turn_primitive_executed": _upper(
                    sample.get("turn_primitive_executed")
                ),
                "turn_primitive_actual": actual,
                "measurement_ready": bool(
                    sample.get("actual_primitive_measurement_ready", False)
                ),
                "measurement_reliable": bool(
                    sample.get("actual_primitive_measurement_reliable", False)
                ),
                "measurement_gate_reasons": list(
                    sample.get("actual_measurement_gate_reasons") or []
                ),
                "actual_primitive_corroboration": dict(
                    sample.get("actual_primitive_corroboration") or {}
                ),
                "primitive_mismatch": actual != _upper(expected_primitive),
                "primitive_contract_violation": bool(
                    sample.get("primitive_contract_violation", False)
                ),
            }
        )
    return unique


def analyze_pivot_case(
    *,
    case: PrimitiveCase,
    expected_deg: float,
    actual_deg: float,
    timeout: bool,
    no_progress: bool,
    terminal_reason: str,
    samples: Sequence[Dict[str, Any]],
    thresholds: Dict[str, float],
) -> Dict[str, Any]:
    sample_list = [dict(sample or {}) for sample in samples]
    active_observations = _active_motion_samples(sample_list)
    active = _independent_status_frames(active_observations)
    settled_active = _settled_actual_classifier_window(
        active,
        thresholds["actual_primitive_transition_settle_s"],
    )
    command_chain_samples = settled_active if settled_active else active
    angle_error_deg = abs(float(_normalize_angle_deg(float(actual_deg) - float(expected_deg))))
    directional_progress_max = max([_safe_float(s.get("directional_progress_deg"), 0.0) for s in sample_list] or [0.0])
    displacement_max = max([_safe_float(s.get("displacement_from_start_m"), 0.0) for s in sample_list] or [0.0])
    actual_v_abs = [abs(_safe_float(s.get("actual_v"), 0.0)) for s in active]
    actual_omega_abs = [abs(_safe_float(s.get("actual_omega"), 0.0)) for s in active]
    watchdog_hz = [_safe_float(s.get("watchdog_freq_hz"), math.nan) for s in active]
    loop_budget = [_safe_float(s.get("loop_budget_total_ema_ms"), math.nan) for s in active]
    logger_depth = [_safe_float(s.get("logger_queue_depth"), math.nan) for s in active]
    logger_flush = [_safe_float(s.get("logger_flush_duration_ms"), math.nan) for s in active]
    sensor_truth_samples = settled_active if settled_active else (active if active else sample_list)
    lidar_conf = [_safe_float(s.get("lidar_pose_confidence"), math.nan) for s in sensor_truth_samples]
    lidar_gap = [_safe_float(s.get("lidar_ekf_applied_gap_s"), math.nan) for s in sensor_truth_samples]
    target_pairs = [
        (_safe_float(s.get("target_left_mps"), math.nan), _safe_float(s.get("target_right_mps"), math.nan))
        for s in active
    ]
    target_pairs = [(l, r) for l, r in target_pairs if math.isfinite(l) and math.isfinite(r)]
    actual_pairs = [
        (_safe_float(s.get("actual_left_mps"), math.nan), _safe_float(s.get("actual_right_mps"), math.nan))
        for s in active
    ]
    actual_pairs = [(l, r) for l, r in actual_pairs if math.isfinite(l) and math.isfinite(r)]
    target_opposite = sum(1 for left, right in target_pairs if left * right < 0.0)
    target_symmetry = [abs(abs(left) - abs(right)) for left, right in target_pairs]
    actual_response_pairs = [
        (left, right)
        for left, right in actual_pairs
        if max(abs(left), abs(right)) >= 0.006
    ]
    actual_opposite = sum(1 for left, right in actual_response_pairs if left * right < 0.0)
    actual_expected_sign = sum(
        1
        for sample in active
        if float(case.expected_sign) * _safe_float(sample.get("actual_omega"), 0.0) > 0.020
    )
    active_count = len(active_observations)
    primitive_req_ratio = _ratio_matching(
        command_chain_samples,
        lambda s: _upper(s.get("turn_primitive_requested")) == EXPECTED_PIVOT_PRIMITIVE,
    )
    primitive_lim_ratio = _ratio_matching(
        command_chain_samples,
        lambda s: _upper(s.get("turn_primitive_limited")) == EXPECTED_PIVOT_PRIMITIVE,
    )
    primitive_exe_ratio = _ratio_matching(
        command_chain_samples,
        lambda s: _upper(s.get("turn_primitive_executed")) == EXPECTED_PIVOT_PRIMITIVE,
    )
    actual_classifier_window = settled_active
    actual_classifier_samples = [
        sample for sample in actual_classifier_window if _actual_classifier_eligible(sample)
    ]
    primitive_act_ratio = _ratio_matching(
        actual_classifier_samples,
        lambda s: _upper(s.get("turn_primitive_actual")) == EXPECTED_PIVOT_PRIMITIVE,
    )
    actual_classifier_coverage = _ratio(len(actual_classifier_samples), len(actual_classifier_window))
    track_exec_ratio = _ratio_matching(
        active,
        lambda s: _upper(s.get("motion_execution_mode") or s.get("resolved_execution_mode")) == EXPECTED_EXECUTION_MODE,
    )
    motion_actual_ssot_values = [_upper(s.get("motion_actual_ssot")) for s in sample_list if _upper(s.get("motion_actual_ssot"))]
    motion_actual_ssot_ratio = _ratio(
        sum(1 for value in motion_actual_ssot_values if value == EXPECTED_MOTION_ACTUAL_SSOT),
        len(motion_actual_ssot_values),
    )
    odom_modes = _unique(_upper(s.get("odometry_mode")) for s in sample_list)
    command_owner_conflicts = sum(1 for s in active if bool(s.get("command_owner_conflict", False)))
    service_active_samples = sum(1 for s in active if bool(s.get("service_motion_active", False)))
    actual_contract_violations = sum(
        1
        for s in actual_classifier_window
        if bool(s.get("primitive_contract_violation", False))
    )
    transition_contract_violations = sum(
        1
        for s in active
        if s not in actual_classifier_window and bool(s.get("primitive_contract_violation", False))
    )
    active_observation = _observation_metrics(active_observations)
    classifier_observation = _observation_metrics(actual_classifier_samples)
    violation_observation = _observation_metrics(
        [s for s in active if bool(s.get("primitive_contract_violation", False))]
    )
    onset_measurements = _onset_measurements(
        active,
        expected_primitive=EXPECTED_PIVOT_PRIMITIVE,
    )
    execution_contract_violations = sum(
        1
        for s in active
        if bool(s.get("control_execution_contract_violation", False))
    )
    slow_tick_delta, slow_observed_tick_delta, slow_tick_ratio = _slow_tick_window(active)
    slow_io_delta = _max_counter_delta(active, "slow_io_event_count")
    slow_lidar_delta = _max_counter_delta(active, "slow_lidar_spike_count")
    slow_resolver_delta = _max_counter_delta(active, "slow_resolver_spike_count")
    slow_gc_delta = _max_counter_delta(active, "slow_gc_count")
    slow_none_delta = _max_counter_delta(active, "slow_none_count")
    finite_watchdog = [v for v in watchdog_hz if math.isfinite(float(v))]
    loop_below_ratio = _ratio(sum(1 for v in finite_watchdog if float(v) < 45.0), len(finite_watchdog))

    metrics = {
        "sample_count": len(sample_list),
        "active_sample_count": active_count,
        "command_chain_settled_frame_count": len(command_chain_samples),
        "active_unique_status_frame_count": active_observation["unique_status_frame_count"],
        "active_observation_span_s": active_observation["observation_span_s"],
        "expected_deg": round(float(expected_deg), 3),
        "actual_deg": round(float(actual_deg), 3),
        "angle_error_deg": round(float(angle_error_deg), 3),
        "directional_progress_max_deg": round(float(directional_progress_max), 3),
        "displacement_max_m": round(float(displacement_max), 4),
        "actual_v_abs": _summary(actual_v_abs),
        "actual_omega_abs": _summary(actual_omega_abs),
        "watchdog_frequency_hz": _summary(watchdog_hz),
        "loop_budget_total_ema_ms": _summary(loop_budget),
        "logger_queue_depth": _summary(logger_depth),
        "logger_flush_duration_ms": _summary(logger_flush),
        "lidar_pose_confidence": _summary(lidar_conf),
        "lidar_ekf_applied_gap_s": _summary(lidar_gap),
        "track_ref_opposite_ratio": _ratio(target_opposite, len(target_pairs)),
        "track_ref_symmetry_abs_mps": _summary(target_symmetry),
        "actual_wheel_response_ratio": _ratio(len(actual_response_pairs), len(actual_pairs)),
        "actual_wheel_opposite_ratio": _ratio(actual_opposite, len(actual_response_pairs)),
        "actual_omega_expected_sign_ratio": _ratio(actual_expected_sign, len(active)),
        "track_exec_ratio": track_exec_ratio,
        "primitive_requested_expected_ratio": primitive_req_ratio,
        "primitive_limited_expected_ratio": primitive_lim_ratio,
        "primitive_executed_expected_ratio": primitive_exe_ratio,
        "primitive_actual_expected_ratio": primitive_act_ratio,
        "actual_classifier_eligible_sample_count": len(actual_classifier_samples),
        "actual_classifier_settled_window_sample_count": len(actual_classifier_window),
        "actual_classifier_transition_sample_count": max(0, len(active) - len(actual_classifier_window)),
        "actual_classifier_transition_settle_s": thresholds["actual_primitive_transition_settle_s"],
        "actual_classifier_unique_status_frame_count": classifier_observation[
            "unique_status_frame_count"
        ],
        "actual_classifier_coverage_ratio": actual_classifier_coverage,
        "motion_actual_ssot_ratio": motion_actual_ssot_ratio,
        "motion_actual_ssot_values_seen": _unique(motion_actual_ssot_values),
        "control_modes_seen": _unique(s.get("control_mode") for s in sample_list),
        "motion_state_modes_seen": _unique(s.get("motion_state_mode") for s in sample_list),
        "odometry_modes_seen": odom_modes,
        "execution_modes_seen": _unique(
            _upper(s.get("motion_execution_mode") or s.get("resolved_execution_mode")) for s in active
        ),
        "turn_primitives_seen": {
            "requested": _unique(_upper(s.get("turn_primitive_requested")) for s in active),
            "limited": _unique(_upper(s.get("turn_primitive_limited")) for s in active),
            "executed": _unique(_upper(s.get("turn_primitive_executed")) for s in active),
            "actual": _unique(_upper(s.get("turn_primitive_actual")) for s in active),
        },
        "turn_primitives_settled_seen": {
            "requested": _unique(_upper(s.get("turn_primitive_requested")) for s in command_chain_samples),
            "limited": _unique(_upper(s.get("turn_primitive_limited")) for s in command_chain_samples),
            "executed": _unique(_upper(s.get("turn_primitive_executed")) for s in command_chain_samples),
            "actual": _unique(_upper(s.get("turn_primitive_actual")) for s in command_chain_samples),
        },
        "command_owner_conflict_samples": int(command_owner_conflicts),
        "service_motion_active_samples": int(service_active_samples),
        "actual_contract_violation_samples": int(actual_contract_violations),
        "transition_contract_violation_samples": int(transition_contract_violations),
        "actual_contract_violation_unique_status_frame_count": violation_observation[
            "unique_status_frame_count"
        ],
        "execution_contract_violation_samples": int(execution_contract_violations),
        "slow_tick_delta": int(slow_tick_delta),
        "slow_observed_tick_delta": int(slow_observed_tick_delta),
        "slow_tick_ratio": float(slow_tick_ratio),
        "slow_io_event_delta": int(slow_io_delta),
        "slow_lidar_spike_delta": int(slow_lidar_delta),
        "slow_resolver_spike_delta": int(slow_resolver_delta),
        "slow_gc_delta": int(slow_gc_delta),
        "slow_none_delta": int(slow_none_delta),
        "loop_below_45_ratio": loop_below_ratio,
        "logger_dropped_messages_max": max([int(s.get("logger_dropped_messages", 0) or 0) for s in active] or [0]),
        "logger_write_errors_max": max([int(s.get("logger_write_errors", 0) or 0) for s in active] or [0]),
    }
    target_symmetry_p90 = metrics["track_ref_symmetry_abs_mps"].get("p90")
    actual_v_p90 = metrics["actual_v_abs"].get("p90")
    actual_omega_p50 = metrics["actual_omega_abs"].get("p50")
    lidar_conf_p50 = metrics["lidar_pose_confidence"].get("p50")
    lidar_gap_p95 = metrics["lidar_ekf_applied_gap_s"].get("p95")
    watchdog_p10 = metrics["watchdog_frequency_hz"].get("p10")
    loop_budget_p95 = metrics["loop_budget_total_ema_ms"].get("p95")
    logger_depth_max = metrics["logger_queue_depth"].get("max")
    logger_flush_p95 = metrics["logger_flush_duration_ms"].get("p95")

    camera_off_ok = bool(sample_list) and all(not bool(s.get("camera_enabled", False)) for s in sample_list)
    unified_ok = bool(sample_list) and all(_upper(s.get("control_mode")) == CANONICAL_CONTROL_MODE for s in sample_list)
    motion_state_unified_ok = bool(sample_list) and all(
        _upper(s.get("motion_state_mode") or s.get("motion_state_mode_raw")) == CANONICAL_CONTROL_MODE
        for s in sample_list
        if _upper(s.get("motion_state_mode") or s.get("motion_state_mode_raw"))
    )
    safety_ok = bool(sample_list) and all(
        bool(s.get("safety_allow", False))
        and _upper(s.get("state")) != "FAILSAFE"
        and not bool(s.get("watchdog_stop_triggered", False))
        for s in sample_list
    )
    clearance_ok = all(
        (not _is_finite(s.get("front_m"))) or float(_safe_float(s.get("front_m"), math.inf)) >= float(thresholds["required_clearance_m"])
        for s in sample_list
    )

    gates: Dict[str, Dict[str, Any]] = {
        "motion_evidence": _gate(
            "PASS"
            if active_count >= int(thresholds["min_motion_samples"]) and not bool(timeout) and not bool(no_progress)
            else "FAIL",
            observed={
                "active_sample_count": active_count,
                "active_unique_status_frame_count": active_observation[
                    "unique_status_frame_count"
                ],
                "active_observation_span_s": active_observation["observation_span_s"],
                "timeout": bool(timeout),
                "no_progress": bool(no_progress),
                "terminal_reason": str(terminal_reason),
            },
            requirement="bounded live pivot produces enough measured active samples without timeout/no-progress",
        ),
        "unified_single_motion_system": _gate(
            "PASS" if unified_ok and motion_state_unified_ok and camera_off_ok else "FAIL",
            observed={
                "control_modes_seen": metrics["control_modes_seen"],
                "motion_state_modes_seen": metrics["motion_state_modes_seen"],
                "camera_enabled_samples": sum(1 for s in sample_list if bool(s.get("camera_enabled", False))),
            },
            requirement="UNIFIED motion mode only, camera disabled during primitive validation",
        ),
        "track_exec_ssot_path": _gate(
            "PASS"
            if (
                (track_exec_ratio is not None and float(track_exec_ratio) >= thresholds["track_exec_ratio_min"])
                and command_owner_conflicts == 0
                and service_active_samples == 0
                and len(motion_actual_ssot_values) > 0
                and (motion_actual_ssot_ratio is not None and float(motion_actual_ssot_ratio) >= thresholds["motion_actual_ssot_ratio_min"])
                and (not odom_modes or all(mode in ("", "LIDAR_FIRST") for mode in odom_modes))
            )
            else "FAIL",
            observed={
                "track_exec_ratio": track_exec_ratio,
                "execution_modes_seen": metrics["execution_modes_seen"],
                "motion_actual_ssot_ratio": motion_actual_ssot_ratio,
                "motion_actual_ssot_values_seen": metrics["motion_actual_ssot_values_seen"],
                "odometry_modes_seen": odom_modes,
                "command_owner_conflict_samples": command_owner_conflicts,
                "service_motion_active_samples": service_active_samples,
            },
            requirement="one routed TRACK_EXEC owner, EKF motion actual SSOT, no service/direct/legacy path evidence",
        ),
        "primitive_command_chain": _gate(
            "PASS"
            if (
                execution_contract_violations == 0
                and primitive_req_ratio is not None
                and primitive_lim_ratio is not None
                and primitive_exe_ratio is not None
                and float(primitive_req_ratio) >= thresholds["primitive_expected_ratio_min"]
                and float(primitive_lim_ratio) >= thresholds["primitive_expected_ratio_min"]
                and float(primitive_exe_ratio) >= thresholds["primitive_expected_ratio_min"]
            )
            else "FAIL",
            observed={
                "turn_primitives_seen": metrics["turn_primitives_settled_seen"],
                "settled_frame_count": len(command_chain_samples),
                "primitive_requested_expected_ratio": primitive_req_ratio,
                "primitive_limited_expected_ratio": primitive_lim_ratio,
                "primitive_executed_expected_ratio": primitive_exe_ratio,
                "execution_contract_violation_samples": execution_contract_violations,
            },
            requirement="requested, limited and executed primitive stay IN_PLACE_ROTATE on the commanded path",
        ),
        "actual_primitive_classifier": _gate(
            "PASS"
            if primitive_act_ratio is not None
            and float(primitive_act_ratio) >= thresholds["actual_primitive_expected_ratio_min"]
            and actual_classifier_coverage is not None
            and float(actual_classifier_coverage) >= thresholds["actual_primitive_coverage_min"]
            and actual_contract_violations == 0
            else (
                "INCONCLUSIVE"
                if actual_classifier_coverage is None
                or float(actual_classifier_coverage) < thresholds["actual_primitive_coverage_min"]
                else "FAIL"
            ),
            observed={
                "turn_primitives_seen": metrics["turn_primitives_seen"],
                "primitive_actual_expected_ratio": primitive_act_ratio,
                "actual_classifier_eligible_sample_count": len(actual_classifier_samples),
                "actual_classifier_settled_window_sample_count": len(actual_classifier_window),
                "actual_classifier_transition_sample_count": max(0, len(active) - len(actual_classifier_window)),
                "actual_classifier_unique_status_frame_count": classifier_observation[
                    "unique_status_frame_count"
                ],
                "actual_classifier_coverage_ratio": actual_classifier_coverage,
                "actual_contract_violation_samples": actual_contract_violations,
                "transition_contract_violation_samples": transition_contract_violations,
                "actual_contract_violation_unique_status_frame_count": violation_observation[
                    "unique_status_frame_count"
                ],
            },
            requirement="enough ready/reliable measurements after the declared transition settle window report IN_PLACE_ROTATE; settled contract violations are forbidden",
        ),
        "track_reference_shape": _gate(
            "PASS"
            if (
                metrics["track_ref_opposite_ratio"] is not None
                and float(metrics["track_ref_opposite_ratio"]) >= thresholds["track_ref_opposite_ratio_min"]
                and target_symmetry_p90 is not None
                and float(target_symmetry_p90) <= thresholds["track_ref_symmetry_p90_max_mps"]
            )
            else "FAIL",
            observed={
                "track_ref_opposite_ratio": metrics["track_ref_opposite_ratio"],
                "track_ref_symmetry_abs_mps": metrics["track_ref_symmetry_abs_mps"],
            },
            requirement="pivot command reaches opposite, near-symmetric left/right track references",
        ),
        "physical_pivot_quality": _gate(
            "PASS"
            if (
                float(case.expected_sign) * float(actual_deg) > 0.0
                and directional_progress_max >= thresholds["min_directional_progress_deg"]
                and angle_error_deg <= thresholds["angle_tolerance_deg"]
                and displacement_max <= thresholds["max_linear_leak_m"]
                and actual_v_p90 is not None
                and float(actual_v_p90) <= thresholds["actual_v_abs_p90_max_mps"]
                and actual_omega_p50 is not None
                and float(actual_omega_p50) >= thresholds["actual_omega_abs_p50_min_rad_s"]
            )
            else "FAIL",
            observed={
                "expected_deg": metrics["expected_deg"],
                "actual_deg": metrics["actual_deg"],
                "angle_error_deg": metrics["angle_error_deg"],
                "directional_progress_max_deg": metrics["directional_progress_max_deg"],
                "displacement_max_m": metrics["displacement_max_m"],
                "actual_v_abs": metrics["actual_v_abs"],
                "actual_omega_abs": metrics["actual_omega_abs"],
            },
            requirement="EKF-measured yaw turns in the requested direction with bounded angle error and low linear leakage",
        ),
        "wheel_encoder_response": _gate(
            "PASS"
            if (
                metrics["actual_wheel_response_ratio"] is not None
                and float(metrics["actual_wheel_response_ratio"]) >= thresholds["actual_wheel_response_min_ratio"]
                and metrics["actual_wheel_opposite_ratio"] is not None
                and float(metrics["actual_wheel_opposite_ratio"]) >= thresholds["actual_wheel_opposite_ratio_min"]
                and metrics["actual_omega_expected_sign_ratio"] is not None
                and float(metrics["actual_omega_expected_sign_ratio"]) >= 0.55
            )
            else "FAIL",
            observed={
                "actual_wheel_response_ratio": metrics["actual_wheel_response_ratio"],
                "actual_wheel_opposite_ratio": metrics["actual_wheel_opposite_ratio"],
                "actual_omega_expected_sign_ratio": metrics["actual_omega_expected_sign_ratio"],
            },
            requirement="encoder wheel speeds and EKF angular velocity confirm the requested in-place pivot",
        ),
        "sensor_truth_runtime": _gate(
            "PASS"
            if (
                (lidar_conf_p50 is None or float(lidar_conf_p50) >= thresholds["lidar_confidence_p50_min"])
                and (lidar_gap_p95 is None or float(lidar_gap_p95) <= thresholds["lidar_ekf_gap_p95_max_s"])
            )
            else "FAIL",
            observed={
                "lidar_pose_confidence": metrics["lidar_pose_confidence"],
                "lidar_ekf_applied_gap_s": metrics["lidar_ekf_applied_gap_s"],
                "settled_unique_status_frame_count": len(sensor_truth_samples),
                "motion_actual_ssot_ratio": motion_actual_ssot_ratio,
            },
            requirement="EKF/LIDAR truth surface remains fresh enough for live primitive measurement",
        ),
        "safety_runtime": _gate(
            "PASS" if safety_ok and clearance_ok else "FAIL",
            observed={
                "safety_ok": bool(safety_ok),
                "clearance_ok": bool(clearance_ok),
                "front_min_m": _summary([s.get("front_m") for s in sample_list]).get("min"),
            },
            requirement="no failsafe/watchdog stop and clearance remains above primitive floor",
        ),
        "software_loop_runtime": _gate(
            "PASS"
            if (
                (watchdog_p10 is None or float(watchdog_p10) >= thresholds["loop_frequency_p10_min_hz"])
                and (loop_below_ratio is None or float(loop_below_ratio) <= thresholds["loop_below_45_ratio_max"])
                and (loop_budget_p95 is None or float(loop_budget_p95) <= thresholds["loop_budget_p95_max_ms"])
                and int(metrics["logger_dropped_messages_max"]) == 0
                and int(metrics["logger_write_errors_max"]) == 0
                and (logger_depth_max is None or float(logger_depth_max) <= thresholds["logger_queue_depth_max"])
                and (logger_flush_p95 is None or float(logger_flush_p95) <= thresholds["logger_flush_p95_max_ms"])
                and float(slow_tick_ratio) <= thresholds["slow_tick_ratio_max"]
            )
            else "FAIL",
            observed={
                "watchdog_frequency_hz": metrics["watchdog_frequency_hz"],
                "loop_below_45_ratio": loop_below_ratio,
                "loop_budget_total_ema_ms": metrics["loop_budget_total_ema_ms"],
                "logger_queue_depth": metrics["logger_queue_depth"],
                "logger_flush_duration_ms": metrics["logger_flush_duration_ms"],
                "logger_dropped_messages_max": metrics["logger_dropped_messages_max"],
                "logger_write_errors_max": metrics["logger_write_errors_max"],
                "slow_tick_delta": slow_tick_delta,
                "slow_observed_tick_delta": slow_observed_tick_delta,
                "slow_tick_ratio": slow_tick_ratio,
                "slow_io_event_delta": slow_io_delta,
                "slow_lidar_spike_delta": slow_lidar_delta,
                "slow_resolver_spike_delta": slow_resolver_delta,
                "slow_gc_delta": slow_gc_delta,
                "slow_none_delta": slow_none_delta,
            },
            requirement="control loop, logger and slow-tick surfaces stay bounded during primitive motion",
        ),
    }
    status, failed, inconclusive = _phase_status_from_gates(gates)
    return {
        "schema": "M3_MOTION_PRIMITIVE_PIVOT_CASE_V1",
        "case_name": str(case.name),
        "command": {
            "type": "set_track_velocity",
            "motion_source": "STATE",
            "left_mps": float(case.left_mps),
            "right_mps": float(case.right_mps),
        },
        "status": status,
        "success": status == "PASS",
        "pass": status == "PASS",
        "expected_deg": round(float(expected_deg), 3),
        "actual_deg": round(float(actual_deg), 3),
        "angle_error_deg": round(float(angle_error_deg), 3),
        "timeout": bool(timeout),
        "no_progress": bool(no_progress),
        "terminal_reason": str(terminal_reason),
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "metrics": metrics,
        "onset_measurements": onset_measurements,
        "sample_count": len(sample_list),
    }


def _max_counter_delta(samples: Sequence[Dict[str, Any]], key: str) -> int:
    values = [int(sample.get(key, 0) or 0) for sample in samples]
    return max(0, max(values) - min(values)) if values else 0


def analyze_system_metrics(system_metrics: Dict[str, Any], thresholds: Dict[str, float]) -> Dict[str, Any]:
    sample_count = int(system_metrics.get("sample_count", 0) or 0)
    cpu_p95 = (system_metrics.get("cpu_percent") or {}).get("p95")
    temp_max = (system_metrics.get("cpu_temp_c") or {}).get("max")
    sd_p95 = (system_metrics.get("sd_write_latency_ms") or {}).get("p95")
    sd_max = (system_metrics.get("sd_write_latency_ms") or {}).get("max")
    status = "INCONCLUSIVE"
    if sample_count > 0:
        status = (
            "PASS"
            if (
                _safe_float(cpu_p95, 0.0) <= thresholds["cpu_p95_max_percent"]
                and _safe_float(temp_max, 0.0) <= thresholds["cpu_temp_max_c"]
                and _safe_float(sd_p95, 0.0) <= thresholds["sd_latency_p95_max_ms"]
                and _safe_float(sd_max, 0.0) <= thresholds["sd_latency_max_ms"]
                and int(system_metrics.get("throttled_bad_count", 0) or 0) == 0
            )
            else "FAIL"
        )
    gate = _gate(
        status,
        observed=system_metrics,
        requirement="CPU, temperature, throttling and SD write latency stay bounded during primitive validation",
        required=False,
    )
    return {
        "schema": "M3_MOTION_PRIMITIVE_SYSTEM_METRICS_V1",
        "status": status,
        "success": status == "PASS",
        "gates": {"hardware_performance": gate},
        "failed_gates": ["hardware_performance"] if status == "FAIL" else [],
        "inconclusive_gates": ["hardware_performance"] if status == "INCONCLUSIVE" else [],
        "metrics": system_metrics,
    }


def build_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": result.get("schema"),
        "status": result.get("status"),
        "success": bool(result.get("success", False)),
        "test_name": result.get("test_name"),
        "plain_summary_hu": result.get("plain_summary_hu"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "preflight": {
            "status": (result.get("preflight") or {}).get("status"),
            "failed_gates": list(((result.get("preflight") or {}).get("failed_gates")) or []),
        },
        "primitive_cases": [
            {
                "case_name": case.get("case_name"),
                "case_id": case.get("case_id"),
                "run_index": case.get("run_index"),
                "status": case.get("status"),
                "failed_gates": list(case.get("failed_gates") or []),
                "expected_deg": case.get("expected_deg"),
                "actual_deg": case.get("actual_deg"),
                "angle_error_deg": case.get("angle_error_deg"),
                "terminal_reason": case.get("terminal_reason"),
                "metrics": {
                    "directional_progress_max_deg": (case.get("metrics") or {}).get("directional_progress_max_deg"),
                    "displacement_max_m": (case.get("metrics") or {}).get("displacement_max_m"),
                    "track_exec_ratio": (case.get("metrics") or {}).get("track_exec_ratio"),
                    "primitive_executed_expected_ratio": (case.get("metrics") or {}).get("primitive_executed_expected_ratio"),
                    "primitive_actual_expected_ratio": (case.get("metrics") or {}).get("primitive_actual_expected_ratio"),
                    "actual_wheel_opposite_ratio": (case.get("metrics") or {}).get("actual_wheel_opposite_ratio"),
                },
                "onset_measurements": list(case.get("onset_measurements") or []),
            }
            for case in list(result.get("primitive_cases") or [])
        ],
        "system_validation": {
            "status": (result.get("system_validation") or {}).get("status"),
            "failed_gates": list(((result.get("system_validation") or {}).get("failed_gates")) or []),
            "inconclusive_gates": list(((result.get("system_validation") or {}).get("inconclusive_gates")) or []),
        },
        "artifact_paths": dict(result.get("artifact_paths") or {}),
    }


def _incident_payload(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": "M3_MOTION_PRIMITIVE_VALIDATOR_INCIDENT_V1",
        "needed": result.get("status") != "PASS",
        "status": result.get("status"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "preflight_failed_gates": list(((result.get("preflight") or {}).get("failed_gates")) or []),
        "primitive_failed_gates": {
            str(case.get("case_name")): list(case.get("failed_gates") or [])
            for case in list(result.get("primitive_cases") or [])
            if case.get("failed_gates")
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


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for raw in handle:
                try:
                    row = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if isinstance(row, dict):
                    rows.append(row)
    except FileNotFoundError:
        pass
    return rows


def _classifier_replay_study(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    samples = [dict(row or {}) for row in rows if row.get("case_name")]
    by_case: Dict[str, List[Dict[str, Any]]] = {}
    for row in samples:
        by_case.setdefault(str(row.get("case_name")), []).append(row)
    case_studies: Dict[str, Any] = {}
    for case_name, case_rows in sorted(by_case.items()):
        active = _active_motion_samples(case_rows) if str(case_name).startswith("pivot") else _twist_active_samples(case_rows)
        actual_counts: Dict[str, int] = {}
        command_counts: Dict[str, int] = {}
        violation_examples: List[Dict[str, Any]] = []
        for row in active:
            actual = _upper(row.get("turn_primitive_actual"))
            executed = _upper(row.get("turn_primitive_executed"))
            if actual:
                actual_counts[actual] = actual_counts.get(actual, 0) + 1
            if executed:
                command_counts[executed] = command_counts.get(executed, 0) + 1
            if bool(row.get("primitive_contract_violation", False)) and len(violation_examples) < 8:
                violation_examples.append(
                    {
                        "t_rel_s": row.get("t_rel_s"),
                        "actual": actual,
                        "executed": executed,
                        "actual_v": row.get("actual_v"),
                        "actual_omega": row.get("actual_omega"),
                        "displacement_from_start_m": row.get("displacement_from_start_m"),
                        "directional_progress_deg": row.get("directional_progress_deg"),
                        "contract": row.get("primitive_contract_detail"),
                    }
                )
        total = len(active)
        inplace_or_arc = int(actual_counts.get(EXPECTED_PIVOT_PRIMITIVE, 0)) + int(
            sum(actual_counts.get(name, 0) for name in EXPECTED_ARC_PRIMITIVES)
        )
        case_studies[case_name] = {
            "active_sample_count": total,
            "actual_counts": actual_counts,
            "executed_counts": command_counts,
            "actual_in_place_ratio": _ratio(actual_counts.get(EXPECTED_PIVOT_PRIMITIVE, 0), total),
            "actual_arc_family_ratio": _ratio(int(sum(actual_counts.get(name, 0) for name in EXPECTED_ARC_PRIMITIVES)), total),
            "actual_in_place_or_arc_family_ratio": _ratio(inplace_or_arc, total),
            "contract_violation_samples": sum(1 for row in active if bool(row.get("primitive_contract_violation", False))),
            "violation_examples": violation_examples,
            "interpretation_hu": (
                "A parancsolt primitive es a fizikai mozgas szetvalasztva ertekelendo: "
                "ha az executed stabil, de az actual valtogat, akkor classifier/szerzodes oldali bizonytalansag valoszinu."
            ),
        }
    return {"schema": "M3_MOTION_PRIMITIVE_CLASSIFIER_REPLAY_STUDY_V1", "cases": case_studies}


def run_replay(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    thresholds = dict(DEFAULT_THRESHOLDS)
    if args.thresholds_json:
        payload = json.loads(Path(args.thresholds_json).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("thresholds JSON must contain an object")
        thresholds.update({key: float(value) for key, value in payload.items() if key in thresholds})
    samples_path = Path(args.replay_samples)
    if not samples_path.is_absolute():
        samples_path = PROJECT_ROOT / samples_path
    rows = _read_jsonl(samples_path)
    by_case: Dict[str, List[Dict[str, Any]]] = {}
    for row in rows:
        case_name = str(row.get("case_name", "") or "")
        if case_name:
            by_case.setdefault(case_name, []).append(dict(row))
    case_results: List[Dict[str, Any]] = []
    known_cases = {case.name: case for case in _case_definitions(["pivot_left", "pivot_right", "straight_forward", "arc_left", "arc_right"], float(args.track_speed_mps))}
    for case_name, case_rows in sorted(by_case.items()):
        case = known_cases.get(case_name)
        if case is None:
            continue
        if case.kind == "twist":
            case_result = analyze_twist_case(
                case=case,
                samples=case_rows,
                timeout=False,
                terminal_reason="REPLAY",
                thresholds=thresholds,
            )
        else:
            final_heading = float(_safe_float((case_rows[-1] if case_rows else {}).get("heading_change_deg"), 0.0))
            case_result = analyze_pivot_case(
                case=case,
                expected_deg=float(case.expected_sign) * float(thresholds["target_angle_deg"]),
                actual_deg=final_heading,
                timeout=False,
                no_progress=False,
                terminal_reason="REPLAY",
                samples=case_rows,
                thresholds=thresholds,
            )
        case_results.append(case_result)
    failed = [f"{case.get('case_name')}:{name}" for case in case_results for name in list(case.get("failed_gates") or [])]
    inconclusive = [
        f"{case.get('case_name')}:{name}"
        for case in case_results
        for name in list(case.get("inconclusive_gates") or [])
    ]
    status = "FAIL" if failed else ("INCONCLUSIVE" if inconclusive else "PASS")
    result = {
        "schema": "M3_MOTION_PRIMITIVE_REPLAY_ANALYSIS_V1",
        "test_name": str(args.test_name),
        "status": status,
        "success": status == "PASS",
        "generated_ts": time.time(),
        "source_samples": str(samples_path),
        "sample_count": len(rows),
        "primitive_cases": case_results,
        "classifier_study": _classifier_replay_study(rows),
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "plain_summary_hu": (
            f"Replay elemzes: {status}. "
            f"Mintak: {len(rows)}. Hibak: {', '.join(failed) if failed else 'nincs'}."
        ),
        "artifact_paths": {
            "result": str(REPLAY_RESULT_PATH.relative_to(PROJECT_ROOT)),
            "summary": str(REPLAY_SUMMARY_PATH.relative_to(PROJECT_ROOT)),
        },
    }
    summary = build_summary({"schema": result["schema"], **result, "preflight": {}, "system_validation": {}})
    _write_json(REPLAY_RESULT_PATH, result)
    _write_json(REPLAY_SUMMARY_PATH, summary)
    return result


def _plain_summary(status: str, preflight: Dict[str, Any], cases: Sequence[Dict[str, Any]]) -> str:
    parts = [f"Eredmeny: {status}.", f"Foundation preflight: {preflight.get('status')}."]
    if cases:
        case_parts = [
            f"{case.get('case_id') or case.get('case_name')}={case.get('status')} actual={case.get('actual_deg')}deg err={case.get('angle_error_deg')}deg"
            for case in cases
        ]
        parts.append("Primitive: " + "; ".join(case_parts) + ".")
    if preflight.get("failed_gates"):
        parts.append("Preflight hibak: " + ", ".join(preflight.get("failed_gates") or []) + ".")
    failed_cases = [
        f"{case.get('case_id') or case.get('case_name')}:{','.join(case.get('failed_gates') or [])}"
        for case in cases
        if case.get("failed_gates")
    ]
    if failed_cases:
        parts.append("Primitive hibak: " + "; ".join(failed_cases) + ".")
    return " ".join(parts)


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    if getattr(args, "replay_samples", ""):
        return run_replay(args)
    thresholds = dict(DEFAULT_THRESHOLDS)
    if args.thresholds_json:
        payload = json.loads(Path(args.thresholds_json).read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("thresholds JSON must contain an object")
        thresholds.update({key: float(value) for key, value in payload.items() if key in thresholds})
    thresholds.update(
        {
            "preflight_duration_s": float(args.preflight_duration_s),
            "preflight_poll_s": float(args.preflight_poll_s),
            "required_clearance_m": float(args.required_clearance_m),
            "target_angle_deg": float(args.target_angle_deg),
            "angle_tolerance_deg": float(args.angle_tolerance_deg),
            "motion_timeout_s": float(args.motion_timeout_s),
            "stop_timeout_s": float(args.stop_timeout_s),
            "poll_s": float(args.poll_s),
        }
    )

    original_peripherals = read_peripherals(runtime_dir=RUNTIME_DIR, use_cache=False)
    camera_disable: Dict[str, Any] = {
        "requested": bool(args.disable_camera),
        "original_camera": bool(original_peripherals.get("camera", False)),
    }
    if bool(args.disable_camera):
        camera_disable["peripherals_after_disable"] = set_peripheral_enabled("camera", False, runtime_dir=RUNTIME_DIR)
        time.sleep(max(0.0, float(args.camera_settle_s)))

    preflight_thresholds = {
        "preflight_min_front_m": float(thresholds["required_clearance_m"]),
        "preflight_min_samples": float(thresholds["preflight_min_samples"]),
        "preflight_duration_s": float(thresholds["preflight_duration_s"]),
        "preflight_poll_s": float(thresholds["preflight_poll_s"]),
    }
    preflight_samples = unified_validator._collect_preflight_samples(
        float(thresholds["preflight_duration_s"]),
        float(thresholds["preflight_poll_s"]),
    )
    preflight = unified_validator.analyze_preflight(preflight_samples, preflight_thresholds)
    all_samples: List[Dict[str, Any]] = list(preflight_samples)
    primitive_cases: List[Dict[str, Any]] = []
    system_metrics: Dict[str, Any] = {}
    system_validation: Dict[str, Any] = {}

    should_run_live = bool(preflight.get("success", False)) and not bool(args.preflight_only)
    if not should_run_live and not bool(args.preflight_only) and bool(args.run_on_preflight_fail):
        should_run_live = True

    if should_run_live:
        if bool(args.disable_camera):
            camera_disable["peripherals_before_live"] = set_peripheral_enabled("camera", False, runtime_dir=RUNTIME_DIR)
            time.sleep(max(0.0, float(args.camera_settle_s)))
        prepare = _prepare_run_start_state(
            token=str(args.token),
            stop_timeout_s=float(thresholds["stop_timeout_s"]),
        )
        if not bool(prepare.get("ok", False)):
            primitive_cases.append(
                {
                    "schema": "M3_MOTION_PRIMITIVE_PIVOT_CASE_V1",
                    "case_name": "prepare",
                    "status": "FAIL",
                    "success": False,
                    "pass": False,
                    "failed_gates": ["prepare_start_state"],
                    "inconclusive_gates": [],
                    "gates": {
                        "prepare_start_state": _gate(
                            "FAIL",
                            observed=prepare,
                            requirement="robot reaches idle/stopped start state before primitive validation",
                        )
                    },
                    "metrics": {},
                    "terminal_reason": "PREPARE_FAILED",
                }
            )
        else:
            cases = _case_definitions(str(args.cases).split(","), float(args.track_speed_mps))
            live_started = time.time()
            case_occurrences: Dict[str, int] = {}
            for idx, case in enumerate(cases):
                case_occurrences[case.name] = case_occurrences.get(case.name, 0) + 1
                case_id = f"{case.name}_{case_occurrences[case.name]}"
                try:
                    if str(case.kind) == "twist":
                        case_result = run_twist_case(case, token=str(args.token), thresholds=thresholds)
                    else:
                        case_result = run_pivot_case(case, token=str(args.token), thresholds=thresholds)
                except Exception as exc:
                    _safe_stop_best_effort(str(args.token))
                    case_result = {
                        "schema": "M3_MOTION_PRIMITIVE_CASE_EXCEPTION_V1",
                        "case_name": str(case.name),
                        "status": "FAIL",
                        "success": False,
                        "pass": False,
                        "command": {
                            "type": "set_twist" if str(case.kind) == "twist" else "set_track_velocity",
                            "motion_source": "STATE",
                            "left_mps": float(case.left_mps),
                            "right_mps": float(case.right_mps),
                            "v_mps": float(case.v_mps),
                            "omega_rad_s": float(case.omega_rad_s),
                        },
                        "failed_gates": ["case_exception"],
                        "inconclusive_gates": [],
                        "gates": {
                            "case_exception": _gate(
                                "FAIL",
                                observed={"error": str(exc)},
                                requirement="primitive case completes without tool/runtime exception",
                            )
                        },
                        "metrics": {},
                        "terminal_reason": "CASE_EXCEPTION",
                    }
                case_result["case_id"] = str(case_id)
                case_result["run_index"] = int(idx)
                primitive_cases.append(case_result)
                raw = dict(case_result.get("raw") or {})
                for sample in list(raw.get("motion_samples") or []) + list(raw.get("settle_samples") or []):
                    row = dict(sample)
                    row["case_name"] = str(case.name)
                    row["case_id"] = str(case_id)
                    row["run_index"] = int(idx)
                    all_samples.append(row)
                if idx + 1 < len(cases):
                    time.sleep(max(0.0, float(args.case_gap_s)))
            live_finished = time.time()
            system_metrics = unified_validator._collect_system_metrics(live_started, live_finished)
            system_validation = analyze_system_metrics(system_metrics, thresholds)

    statuses = [str(preflight.get("status", ""))]
    failed = list(preflight.get("failed_gates") or [])
    inconclusive: List[str] = []
    for case in primitive_cases:
        statuses.append(str(case.get("status", "")))
        case_label = str(case.get("case_id") or case.get("case_name"))
        failed.extend([f"{case_label}:{name}" for name in list(case.get("failed_gates") or [])])
        inconclusive.extend(
            [f"{case_label}:{name}" for name in list(case.get("inconclusive_gates") or [])]
        )
    if system_validation:
        statuses.append(str(system_validation.get("status", "")))
        failed.extend([f"system:{name}" for name in list(system_validation.get("failed_gates") or [])])
        inconclusive.extend([f"system:{name}" for name in list(system_validation.get("inconclusive_gates") or [])])
    if any(status == "FAIL" for status in statuses):
        status = "FAIL"
    elif any(status == "INCONCLUSIVE" for status in statuses) or inconclusive:
        status = "INCONCLUSIVE"
    else:
        status = "PASS"

    result = {
        "schema": "M3_MOTION_PRIMITIVE_VALIDATOR_V1",
        "test_name": str(args.test_name),
        "status": status,
        "success": status == "PASS",
        "generated_ts": time.time(),
        "preflight_only": bool(args.preflight_only),
        "camera_disable": camera_disable,
        "thresholds": thresholds,
        "preflight": preflight,
        "primitive_cases": primitive_cases,
        "system_validation": system_validation,
        "system_metrics": system_metrics,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "plain_summary_hu": _plain_summary(status, preflight, primitive_cases),
        "artifact_paths": {
            "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
            "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
            "preflight": str(PREFLIGHT_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
            "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
        },
        "ssot_summary": {
            "motion_intent": "single set_track_velocity intent routed through motion_command/motion_resolver",
            "pose": "EKF pose only for physical yaw/displacement verdict",
            "motor_output": "MotionExecutor only; tool never writes PWM directly",
            "primitive": EXPECTED_PIVOT_PRIMITIVE,
            "execution_mode": EXPECTED_EXECUTION_MODE,
            "mode": CANONICAL_CONTROL_MODE,
        },
        "layer_contracts": {
            "tool": "validator/orchestrator only",
            "command_bus": "append set_track_velocity commands with GUI_DEFAULT token",
            "arbitration": "one active route, no owner conflict",
            "primitive_semantics": "requested/limited/executed IN_PLACE_ROTATE",
            "executor": "TRACK_EXEC track references only",
            "measurement": "EKF pose, encoder wheel speeds, IMU/LIDAR health surfaces",
        },
    }
    write_artifacts(result, all_samples)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M3 motion primitive full-stack live validator.")
    parser.add_argument("--test-name", default="M3_motion_primitive_validator")
    parser.add_argument("--cases", default="pivot_left")
    parser.add_argument("--track-speed-mps", type=float, default=0.150)
    parser.add_argument("--target-angle-deg", type=float, default=DEFAULT_THRESHOLDS["target_angle_deg"])
    parser.add_argument("--angle-tolerance-deg", type=float, default=DEFAULT_THRESHOLDS["angle_tolerance_deg"])
    parser.add_argument("--required-clearance-m", type=float, default=DEFAULT_THRESHOLDS["required_clearance_m"])
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--preflight-duration-s", type=float, default=DEFAULT_THRESHOLDS["preflight_duration_s"])
    parser.add_argument("--preflight-poll-s", type=float, default=DEFAULT_THRESHOLDS["preflight_poll_s"])
    parser.add_argument("--motion-timeout-s", type=float, default=DEFAULT_THRESHOLDS["motion_timeout_s"])
    parser.add_argument("--stop-timeout-s", type=float, default=DEFAULT_THRESHOLDS["stop_timeout_s"])
    parser.add_argument("--poll-s", type=float, default=DEFAULT_THRESHOLDS["poll_s"])
    parser.add_argument("--case-gap-s", type=float, default=0.4)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--disable-camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--camera-settle-s", type=float, default=0.4)
    parser.add_argument("--run-on-preflight-fail", action="store_true")
    parser.add_argument("--thresholds-json", default="")
    parser.add_argument("--replay-samples", default="")
    parser.add_argument("--replay-only", action="store_true")
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
