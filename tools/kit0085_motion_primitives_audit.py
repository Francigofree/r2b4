#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bounded KIT0085 reverse and turn primitive audit through the normal twist path."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.kit0085_live_audit import (  # noqa: E402
    _encoder_reading,
    _ensure_control_mode,
    _pid_diagnostics,
    _safe_float,
    _safe_int,
    _write_json_atomic,
)
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_MOTION_SOURCE,
    DEFAULT_TOKEN,
    STATUS_PATH,
    _append_command,
    _get_pose,
    _normalize_angle_deg,
    _pose_distance,
    _read_json,
    _safe_stop_best_effort,
    _send_command_checked,
    _wait_for_status_progress,
    _wait_until_stopped,
)


LATEST_RESULT_PATH = test_artifacts_dir() / "latest_kit0085_motion_primitives.json"


@dataclass(frozen=True)
class PrimitiveCase:
    name: str
    kind: str
    v_mps: float
    omega_rad_s: float
    duration_s: float
    target_distance_m: float = 0.0
    min_yaw_deg: float = 0.0


DEFAULT_CASES: Dict[str, PrimitiveCase] = {
    "reverse_0p3m": PrimitiveCase("reverse_0p3m", "reverse", -0.035, 0.0, 18.0, 0.30, 0.0),
    "arc_left": PrimitiveCase("arc_left", "arc_left", 0.040, 0.100, 3.0, 0.0, 5.0),
    "arc_right": PrimitiveCase("arc_right", "arc_right", 0.040, -0.115, 3.0, 0.0, 5.0),
}


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _front_clearance_m(status: Dict[str, Any]) -> float:
    lidar = dict((status or {}).get("lidar") or {})
    vals = [
        _safe_float(lidar.get("front_min_m"), math.nan),
        _safe_float(lidar.get("min_dist"), math.nan),
        _safe_float((status or {}).get("front_clearance_m"), math.nan),
    ]
    finite = [float(v) for v in vals if math.isfinite(float(v)) and float(v) > 0.0]
    return min(finite) if finite else math.nan


def _back_clearance_m(status: Dict[str, Any]) -> float:
    lidar = dict((status or {}).get("lidar") or {})
    vals = [
        _safe_float(lidar.get("back_min_m"), math.nan),
        _safe_float(lidar.get("min_back"), math.nan),
        _safe_float((status or {}).get("back_clearance_m"), math.nan),
    ]
    finite = [float(v) for v in vals if math.isfinite(float(v)) and float(v) > 0.0]
    return min(finite) if finite else math.nan


def _case_failures(metrics: Dict[str, Any]) -> List[str]:
    failures: List[str] = []
    kind = str(metrics.get("kind", "") or "")
    left = _safe_float(metrics.get("left_distance_delta_m"), 0.0)
    right = _safe_float(metrics.get("right_distance_delta_m"), 0.0)
    enc_avg = _safe_float(metrics.get("encoder_average_delta_m"), 0.0)
    yaw = _safe_float(metrics.get("signed_yaw_deg"), 0.0)
    chord = _safe_float(metrics.get("ekf_chord_m"), 0.0)
    target = abs(_safe_float(metrics.get("target_distance_m"), 0.0))

    if bool(metrics.get("failsafe_seen", False)):
        failures.append("failsafe_seen")
    if bool(metrics.get("safety_block_seen", False)):
        failures.append("safety_block_seen")
    if not bool(metrics.get("normal_stop_confirmed", False)):
        failures.append("normal_stop_not_confirmed")
    if _safe_float(metrics.get("max_pwm_left"), 0.0) <= 0.05:
        failures.append("left_motor_pwm_not_observed")
    if _safe_float(metrics.get("max_pwm_right"), 0.0) <= 0.05:
        failures.append("right_motor_pwm_not_observed")

    if kind == "reverse":
        if not (left < -0.08 and right < -0.08):
            failures.append("encoder_reverse_sign_missing")
        if abs(enc_avg) < max(0.16, 0.55 * target):
            failures.append("encoder_reverse_progress_low")
        if target > 0.0 and abs(enc_avg) > 1.55 * target:
            failures.append("encoder_reverse_progress_high")
        if chord < max(0.12, 0.40 * target):
            failures.append("ekf_reverse_progress_low")
        if abs(yaw) > 18.0:
            failures.append("reverse_heading_deviation_high")
    elif kind in ("arc_left", "arc_right"):
        expected_sign = 1.0 if kind == "arc_left" else -1.0
        if expected_sign * yaw < _safe_float(metrics.get("min_yaw_deg"), 0.0):
            failures.append("arc_yaw_progress_low")
        if abs(enc_avg) < 0.035:
            failures.append("arc_encoder_progress_low")
        side_delta = right - left
        if kind == "arc_left" and side_delta < 0.004:
            failures.append("left_arc_side_delta_low")
        if kind == "arc_right" and side_delta > -0.004:
            failures.append("right_arc_side_delta_low")
        if abs(yaw) > 55.0:
            failures.append("arc_yaw_too_high")
    else:
        failures.append("unknown_case_kind")
    return failures


def _sample_counts(status: Dict[str, Any], feedback_counts: Dict[str, int], output_counts: Dict[str, int]) -> None:
    pid, monitor = _pid_diagnostics(status)
    feedback = str(
        pid.get("feedback_velocity_source")
        or monitor.get("feedback_velocity_source")
        or "UNKNOWN"
    ).strip().upper()
    output = str(pid.get("output_reason") or monitor.get("output_reason") or "NONE").strip().upper()
    feedback_counts[feedback] = feedback_counts.get(feedback, 0) + 1
    output_counts[output] = output_counts.get(output, 0) + 1


def _run_case(case: PrimitiveCase, *, token: str, poll_s: float, keepalive_s: float, stop_timeout_s: float) -> Dict[str, Any]:
    start_status = _wait_for_status_progress(min_increments=1, timeout_s=5.0)
    start_pose = _get_pose(start_status)
    start_encoder = _encoder_reading(start_status)
    feedback_counts: Dict[str, int] = {}
    output_counts: Dict[str, int] = {}
    max_pwm_left = 0.0
    max_pwm_right = 0.0
    failsafe_seen = False
    safety_block_seen = False
    end_status = start_status

    _send_command_checked(
        "set_twist",
        token=str(token),
        timeout_s=4.0,
        v=float(case.v_mps),
        omega=float(case.omega_rad_s),
        motion_source=DEFAULT_MOTION_SOURCE,
    )
    started = time.monotonic()
    last_keepalive = started
    sample_count = 0
    try:
        while (time.monotonic() - started) <= float(case.duration_s):
            now = time.monotonic()
            if (now - last_keepalive) >= float(keepalive_s):
                _append_command(
                    "set_twist",
                    token=str(token),
                    v=float(case.v_mps),
                    omega=float(case.omega_rad_s),
                    motion_source=DEFAULT_MOTION_SOURCE,
                )
                last_keepalive = now
            status = _read_json(STATUS_PATH)
            if not status:
                time.sleep(float(poll_s))
                continue
            end_status = status
            state = str(status.get("state", "") or "").upper()
            failsafe_seen = bool(failsafe_seen or state == "FAILSAFE")
            safety = dict(status.get("safety") or {})
            safety_block_seen = bool(safety_block_seen or (safety and not bool(safety.get("allow", True))))
            pwm = dict(status.get("pwm") or {})
            max_pwm_left = max(max_pwm_left, abs(_safe_float(pwm.get("left"), 0.0)))
            max_pwm_right = max(max_pwm_right, abs(_safe_float(pwm.get("right"), 0.0)))
            _sample_counts(status, feedback_counts, output_counts)
            sample_count += 1
            if failsafe_seen or safety_block_seen:
                break
            if case.kind == "reverse" and case.target_distance_m > 0.0:
                reading = _encoder_reading(status)
                enc_avg = 0.5 * (
                    (reading["left_distance_m"] - start_encoder["left_distance_m"])
                    + (reading["right_distance_m"] - start_encoder["right_distance_m"])
                )
                if abs(float(enc_avg)) >= abs(float(case.target_distance_m)):
                    break
            time.sleep(float(poll_s))
    finally:
        _safe_stop_best_effort(token=str(token))

    normal_stop_confirmed = False
    try:
        end_status = _wait_until_stopped(timeout_s=float(stop_timeout_s))
        normal_stop_confirmed = True
    except Exception:
        end_status = _read_json(STATUS_PATH) or end_status

    end_pose = _get_pose(end_status)
    end_encoder = _encoder_reading(end_status)
    left_delta = end_encoder["left_distance_m"] - start_encoder["left_distance_m"]
    right_delta = end_encoder["right_distance_m"] - start_encoder["right_distance_m"]
    metrics = {
        "case": str(case.name),
        "kind": str(case.kind),
        "v_mps": float(case.v_mps),
        "omega_rad_s": float(case.omega_rad_s),
        "duration_s": round(float(time.monotonic() - started), 3),
        "target_distance_m": float(case.target_distance_m),
        "min_yaw_deg": float(case.min_yaw_deg),
        "left_pulse_delta": end_encoder["left_pulses"] - start_encoder["left_pulses"],
        "right_pulse_delta": end_encoder["right_pulses"] - start_encoder["right_pulses"],
        "left_distance_delta_m": float(left_delta),
        "right_distance_delta_m": float(right_delta),
        "encoder_average_delta_m": 0.5 * (float(left_delta) + float(right_delta)),
        "ekf_chord_m": _pose_distance(start_pose, end_pose),
        "signed_yaw_deg": _normalize_angle_deg(
            float(_safe_float(end_pose.get("theta_deg"), 0.0))
            - float(_safe_float(start_pose.get("theta_deg"), 0.0))
        ),
        "max_pwm_left": float(max_pwm_left),
        "max_pwm_right": float(max_pwm_right),
        "feedback_source_counts": dict(sorted(feedback_counts.items())),
        "output_reason_counts": dict(sorted(output_counts.items())),
        "sample_count": int(sample_count),
        "failsafe_seen": bool(failsafe_seen),
        "safety_block_seen": bool(safety_block_seen),
        "normal_stop_confirmed": bool(normal_stop_confirmed),
    }
    failures = _case_failures(metrics)
    return {
        "case": str(case.name),
        "success": not failures,
        "failures": list(dict.fromkeys(failures)),
        "metrics": metrics,
    }


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    token = str(args.token)
    cases = [DEFAULT_CASES[name.strip()] for name in str(args.cases).split(",") if name.strip()]
    status = _ensure_control_mode(str(args.control_mode), token=token)
    front = _front_clearance_m(status)
    back = _back_clearance_m(status)
    preflight_failures: List[str] = []
    if math.isfinite(front) and float(front) < float(args.required_front_clearance_m):
        preflight_failures.append(f"front_clearance_low:{front:.3f}<{float(args.required_front_clearance_m):.3f}")
    if math.isfinite(back) and float(back) < float(args.required_back_clearance_m):
        preflight_failures.append(f"back_clearance_low:{back:.3f}<{float(args.required_back_clearance_m):.3f}")

    results: List[Dict[str, Any]] = []
    if not preflight_failures:
        try:
            _safe_stop_best_effort(token=token)
            for case in cases:
                try:
                    results.append(
                        _run_case(
                            case,
                            token=token,
                            poll_s=float(args.poll_s),
                            keepalive_s=float(args.keepalive_s),
                            stop_timeout_s=float(args.stop_timeout_s),
                        )
                    )
                except Exception as exc:
                    _safe_stop_best_effort(token=token)
                    results.append(
                        {
                            "case": str(case.name),
                            "success": False,
                            "failures": ["case_exception"],
                            "metrics": {
                                "case": str(case.name),
                                "kind": str(case.kind),
                                "v_mps": float(case.v_mps),
                                "omega_rad_s": float(case.omega_rad_s),
                                "error": str(exc),
                            },
                        }
                    )
                    break
                time.sleep(max(0.05, float(args.inter_case_pause_s)))
        finally:
            _safe_stop_best_effort(token=token)

    failures: List[str] = list(preflight_failures)
    for result in results:
        for failure in result.get("failures") or []:
            failures.append(f"{result.get('case')}:{failure}")

    result = {
        "success": not failures and len(results) == len(cases),
        "test": "kit0085_motion_primitives_audit",
        "started_at_utc": _now_iso_utc(),
        "hardware": "DFRobot KIT0085",
        "control_mode": str(args.control_mode).strip().upper(),
        "cases_requested": [case.name for case in cases],
        "cases": results,
        "preflight": {
            "front_clearance_m": None if not math.isfinite(front) else float(front),
            "back_clearance_m": None if not math.isfinite(back) else float(back),
            "required_front_clearance_m": float(args.required_front_clearance_m),
            "required_back_clearance_m": float(args.required_back_clearance_m),
        },
        "failures": list(dict.fromkeys(failures)),
        "artifact": str(LATEST_RESULT_PATH),
    }
    _write_json_atomic(LATEST_RESULT_PATH, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit KIT0085 reverse and normal turning primitives.")
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--control-mode", default="UNIFIED")
    parser.add_argument("--cases", default="reverse_0p3m,arc_left,arc_right")
    parser.add_argument("--required-front-clearance-m", type=float, default=0.55)
    parser.add_argument("--required-back-clearance-m", type=float, default=0.45)
    parser.add_argument("--poll-s", type=float, default=0.05)
    parser.add_argument("--keepalive-s", type=float, default=0.18)
    parser.add_argument("--stop-timeout-s", type=float, default=5.0)
    parser.add_argument("--inter-case-pause-s", type=float, default=0.50)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    try:
        result = run(args)
    except BootstrapGuardError as exc:
        print(json.dumps({"success": False, "error": f"bootstrap_guard_failed:{exc}"}, sort_keys=True))
        return 40
    except Exception as exc:
        _safe_stop_best_effort(token=str(getattr(args, "token", DEFAULT_TOKEN)))
        result = {
            "success": False,
            "test": "kit0085_motion_primitives_audit",
            "error": str(exc),
            "failures": ["audit_exception"],
            "artifact": str(LATEST_RESULT_PATH),
        }
        _write_json_atomic(LATEST_RESULT_PATH, result)
    if bool(args.compact):
        cases = result.get("cases") or []
        case_bits = [
            f"{item.get('case')}:{'PASS' if item.get('success') else 'FAIL'}"
            for item in cases
        ]
        print(
            "KIT0085_PRIMITIVES "
            f"result={'PASS' if result.get('success') else 'FAIL'} "
            f"cases={','.join(case_bits) or 'none'} "
            f"failures={','.join(result.get('failures') or []) or 'none'}"
        )
    else:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if bool(result.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
