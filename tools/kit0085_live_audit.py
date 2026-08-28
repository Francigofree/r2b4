#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bounded live audit for the DFRobot KIT0085 drivetrain."""

from __future__ import annotations

import argparse
import json
import math
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
    _get_pose,
    _normalize_angle_deg,
    _pose_distance,
    _precheck,
    _read_json,
    _send_command_checked,
    _wait_for_status,
    _wait_until_stopped,
)


LATEST_RESULT_PATH = test_artifacts_dir() / "latest_kit0085_audit.json"
CONTROL_MODE_PATH = PROJECT_ROOT / "conf" / "control_mode.json"
EXPECTED_MODEL = "DFROBOT_KIT0085_28PA51G"
EXPECTED_PINS = {"left": (23, 24), "right": (25, 16)}
MAX_PLAUSIBLE_SIDE_DISTANCE_M = 1.20
CANONICAL_CONTROL_MODE = "UNIFIED"


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


def _edge_balance_low(rising: int, falling: int, *, min_ratio: float = 0.35) -> bool:
    if int(rising) <= 0 or int(falling) <= 0:
        return False
    return (min(int(rising), int(falling)) / max(int(rising), int(falling))) < float(min_ratio)


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2, sort_keys=True)
    os.replace(tmp, path)


def _normalize_control_mode(value: Any) -> str:
    return str(value or "").strip().upper()


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
        "left_unresolved": _safe_int(left_snap.get("unresolved_pulses"), 0),
        "right_unresolved": _safe_int(right_snap.get("unresolved_pulses"), 0),
        "left_level_a": _safe_int(left.get("level_a"), -1),
        "left_level_b": _safe_int(left.get("level_b"), -1),
        "right_level_a": _safe_int(right.get("level_a"), -1),
        "right_level_b": _safe_int(right.get("level_b"), -1),
        "left_a_edges": _safe_int(left.get("a_edge_count"), 0),
        "left_a_rising": _safe_int(left.get("a_rising_count"), 0),
        "left_a_falling": _safe_int(left.get("a_falling_count"), 0),
        "left_b_edges": _safe_int(left.get("b_edge_count"), 0),
        "left_b_rising": _safe_int(left.get("b_rising_count"), 0),
        "left_b_falling": _safe_int(left.get("b_falling_count"), 0),
        "left_forward_count": _safe_int(left.get("forward_count"), 0),
        "left_reverse_count": _safe_int(left.get("reverse_count"), 0),
        "right_a_edges": _safe_int(right.get("a_edge_count"), 0),
        "right_a_rising": _safe_int(right.get("a_rising_count"), 0),
        "right_a_falling": _safe_int(right.get("a_falling_count"), 0),
        "right_b_edges": _safe_int(right.get("b_edge_count"), 0),
        "right_b_rising": _safe_int(right.get("b_rising_count"), 0),
        "right_b_falling": _safe_int(right.get("b_falling_count"), 0),
        "right_forward_count": _safe_int(right.get("forward_count"), 0),
        "right_reverse_count": _safe_int(right.get("reverse_count"), 0),
        "snapshot_health": str(dict(encoder.get("service") or {}).get("snapshot_health", "") or ""),
    }


def _start_lidar_confidence(precheck: Dict[str, Any], status: Dict[str, Any]) -> float:
    start_gate = dict((precheck or {}).get("start_gate") or {})
    diagnostics = dict(start_gate.get("diagnostics") or {})
    for key in ("lidar_latest_confidence", "latest_confidence", "confidence"):
        value = diagnostics.get(key)
        conf = _safe_float(value, math.nan)
        if math.isfinite(conf):
            return float(conf)
    for section_name in ("lidar_odom_runtime_status", "lidar_odom_status", "lidar_odom"):
        section = dict((status or {}).get(section_name) or {})
        for key in ("latest_confidence", "candidate_confidence", "confidence"):
            conf = _safe_float(section.get(key), math.nan)
            if math.isfinite(conf):
                return float(conf)
    return float("nan")


def _lidar_confidence_warning(confidence: float, minimum: float) -> str:
    if not math.isfinite(confidence):
        return "start_lidar_confidence_missing"
    if confidence < float(minimum):
        return f"start_lidar_confidence_low:{confidence:.3f}<{float(minimum):.3f}"
    return ""


def _pid_diagnostics(status: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    pid_diag = dict((status or {}).get("pid_diag") or (status or {}).get("pid") or {})
    monitor = dict((status or {}).get("control_monitor") or pid_diag.get("monitor") or {})
    motion_semantics = dict((status or {}).get("motion_semantics") or {})
    if motion_semantics:
        active = bool(motion_semantics.get("heading_hold_applied", False))
        pid_diag["guidance_heading_hold"] = {
            "active": active,
            "owner": str(motion_semantics.get("heading_hold_owner", "") or ""),
            "mode": str(motion_semantics.get("heading_hold_mode", "") or ""),
            "heading_error_deg": motion_semantics.get("heading_error_deg", 0.0),
            "omega_correction_rad_s": (
                motion_semantics.get("omega_target", 0.0) if active else 0.0
            ),
        }
    if "feedback_velocity_source" not in pid_diag and "feedback_velocity_source" in monitor:
        pid_diag["feedback_velocity_source"] = monitor.get("feedback_velocity_source")
    if "output_reason" not in pid_diag and "output_reason" in monitor:
        pid_diag["output_reason"] = monitor.get("output_reason")
    for key in (
        "v_cmd",
        "omega_cmd",
        "omega_cmd_request",
        "v_l",
        "v_r",
        "v_l_ref",
        "v_r_ref",
        "pwm_executor_l",
        "pwm_executor_r",
    ):
        if key not in pid_diag and key in monitor:
            pid_diag[key] = monitor.get(key)
    return pid_diag, monitor


def evaluate_audit(metrics: Dict[str, Any], *, target_distance_m: float = 0.30) -> List[str]:
    failures: List[str] = []
    start = dict(metrics.get("encoder_start") or {})
    end = dict(metrics.get("encoder_end") or {})
    delta_l_p = _safe_int(metrics.get("left_pulse_delta"), 0)
    delta_r_p = _safe_int(metrics.get("right_pulse_delta"), 0)
    delta_l_m = _safe_float(metrics.get("left_distance_delta_m"), 0.0)
    delta_r_m = _safe_float(metrics.get("right_distance_delta_m"), 0.0)
    ekf_progress = _safe_float(metrics.get("ekf_progress_m"), 0.0)
    target_distance = max(0.05, abs(float(target_distance_m)))
    min_side_progress = 0.60 * target_distance
    min_ekf_progress = 0.80 * target_distance
    max_ekf_progress = 1.40 * target_distance

    if start.get("left_model") != EXPECTED_MODEL or start.get("right_model") != EXPECTED_MODEL:
        failures.append("encoder_model_mismatch")
    if tuple(start.get("left_pins") or ()) != EXPECTED_PINS["left"]:
        failures.append("left_encoder_gpio_mismatch")
    if tuple(start.get("right_pins") or ()) != EXPECTED_PINS["right"]:
        failures.append("right_encoder_gpio_mismatch")
    if str(end.get("snapshot_health", "")).upper() != "OK":
        failures.append("encoder_snapshot_not_ok")
    if delta_l_p <= 0:
        failures.append("left_encoder_not_forward")
    if delta_r_p <= 0:
        failures.append("right_encoder_not_forward")
    if delta_l_m < min_side_progress:
        failures.append("left_encoder_progress_low")
    if delta_r_m < min_side_progress:
        failures.append("right_encoder_progress_low")
    if abs(delta_l_m) > MAX_PLAUSIBLE_SIDE_DISTANCE_M:
        failures.append("left_encoder_distance_implausible")
    if abs(delta_r_m) > MAX_PLAUSIBLE_SIDE_DISTANCE_M:
        failures.append("right_encoder_distance_implausible")
    left_a_rising_delta = _safe_int(end.get("left_a_rising"), 0) - _safe_int(start.get("left_a_rising"), 0)
    left_a_falling_delta = _safe_int(end.get("left_a_falling"), 0) - _safe_int(start.get("left_a_falling"), 0)
    left_b_rising_delta = _safe_int(end.get("left_b_rising"), 0) - _safe_int(start.get("left_b_rising"), 0)
    left_b_falling_delta = _safe_int(end.get("left_b_falling"), 0) - _safe_int(start.get("left_b_falling"), 0)
    right_a_rising_delta = _safe_int(end.get("right_a_rising"), 0) - _safe_int(start.get("right_a_rising"), 0)
    right_a_falling_delta = _safe_int(end.get("right_a_falling"), 0) - _safe_int(start.get("right_a_falling"), 0)
    right_b_rising_delta = _safe_int(end.get("right_b_rising"), 0) - _safe_int(start.get("right_b_rising"), 0)
    right_b_falling_delta = _safe_int(end.get("right_b_falling"), 0) - _safe_int(start.get("right_b_falling"), 0)
    if max(abs(delta_l_m), abs(delta_r_m)) > 1e-6:
        side_ratio = min(abs(delta_l_m), abs(delta_r_m)) / max(abs(delta_l_m), abs(delta_r_m))
        if side_ratio < 0.55:
            failures.append("encoder_side_asymmetry")
    if not (min_ekf_progress <= ekf_progress <= max_ekf_progress):
        failures.append(
            "ekf_0p3m_progress_out_of_range"
            if abs(target_distance - 0.30) < 1e-6
            else "ekf_progress_out_of_range"
        )
    encoder_avg = 0.5 * (delta_l_m + delta_r_m)
    if ekf_progress > 1e-6 and not (0.45 <= encoder_avg / ekf_progress <= 1.65):
        failures.append("encoder_vs_ekf_distance_mismatch")
    if _safe_float(metrics.get("max_pwm_left"), 0.0) <= 0.05:
        failures.append("left_motor_pwm_not_observed")
    if _safe_float(metrics.get("max_pwm_right"), 0.0) <= 0.05:
        failures.append("right_motor_pwm_not_observed")
    if not bool(metrics.get("left_quadrature_direction_seen", False)):
        failures.append("left_quadrature_direction_missing")
    if not bool(metrics.get("right_quadrature_direction_seen", False)):
        failures.append("right_quadrature_direction_missing")
    if _safe_int(end.get("left_unresolved"), 0) > 0 or _safe_int(end.get("right_unresolved"), 0) > 0:
        failures.append("unresolved_encoder_pulses")
    if abs(_safe_float(metrics.get("max_heading_delta_deg"), 0.0)) > 15.0:
        failures.append("heading_deviation_high")
    if bool(metrics.get("failsafe_seen", False)):
        failures.append("failsafe_seen")
    if bool(metrics.get("safety_block_seen", False)):
        failures.append("safety_block_seen")
    if not bool(metrics.get("normal_stop_confirmed", False)):
        failures.append("normal_stop_not_confirmed")
    return failures


def run(args: argparse.Namespace) -> Dict[str, Any]:
    token = str(args.token)
    target_control_mode = _normalize_control_mode(args.control_mode) if str(args.control_mode or "").strip() else ""
    control_mode_applied = ""
    precheck = _precheck(
        token,
        target_distance_m=float(args.target_distance_m),
        required_clearance_m=float(args.required_clearance_m),
        stop_timeout_s=float(args.stop_timeout_s),
    )
    start_status = (
        _ensure_control_mode(target_control_mode, token=token)
        if target_control_mode
        else _wait_for_status(timeout_s=3.0)
    )
    control_mode_applied = _normalize_control_mode(start_status.get("control_mode") or _read_control_mode_file())
    if str(start_status.get("odometry_mode", "") or "").upper() != "LIDAR_FIRST":
        raise RuntimeError("odometry_mode_not_lidar_first")
    if not bool(start_status.get("encoder_pose_fusion_active", False)):
        raise RuntimeError("encoder_pose_fusion_inactive")
    start_lidar_confidence = _start_lidar_confidence(precheck, start_status)
    warnings: List[str] = []
    lidar_confidence_warning = _lidar_confidence_warning(
        start_lidar_confidence,
        float(args.min_start_lidar_confidence),
    )
    if lidar_confidence_warning:
        warnings.append(lidar_confidence_warning)

    start_pose = _get_pose(start_status)
    start_encoder = _encoder_reading(start_status)
    start_heading = float(start_pose["theta_deg"])
    max_pwm_left = 0.0
    max_pwm_right = 0.0
    max_heading_delta_deg = 0.0
    left_quadrature_direction_seen = False
    right_quadrature_direction_seen = False
    failsafe_seen = False
    safety_block_seen = False
    end_status = start_status
    command_error = ""
    sample_count = 0
    pwm_saturation_samples = 0
    feedback_source_counts: Dict[str, int] = {}
    output_reason_counts: Dict[str, int] = {}
    max_feedback_v_l_mps = 0.0
    max_feedback_v_r_mps = 0.0
    max_ref_v_l_mps = 0.0
    max_ref_v_r_mps = 0.0
    max_v_cmd_mps = 0.0
    guidance_heading_hold_active_samples = 0
    guidance_heading_hold_mode_counts: Dict[str, int] = {}
    guidance_heading_hold_peak_heading_error_deg = 0.0
    guidance_heading_hold_peak_correction_rad_s = 0.0

    _send_command_checked(
        "set_twist",
        token=token,
        timeout_s=4.0,
        v=float(args.speed_mps),
        omega=0.0,
        motion_source=DEFAULT_MOTION_SOURCE,
    )
    started = time.monotonic()
    last_keepalive = started
    try:
        while (time.monotonic() - started) <= float(args.move_timeout_s):
            now = time.monotonic()
            if (now - last_keepalive) >= float(args.keepalive_s):
                _append_command(
                    "set_twist",
                    token=token,
                    v=float(args.speed_mps),
                    omega=0.0,
                    motion_source=DEFAULT_MOTION_SOURCE,
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

            pwm = dict(status.get("pwm") or {})
            pwm_left_abs = abs(_safe_float(pwm.get("left"), 0.0))
            pwm_right_abs = abs(_safe_float(pwm.get("right"), 0.0))
            max_pwm_left = max(max_pwm_left, pwm_left_abs)
            max_pwm_right = max(max_pwm_right, pwm_right_abs)
            sample_count += 1
            if pwm_left_abs >= 0.99 or pwm_right_abs >= 0.99:
                pwm_saturation_samples += 1
            pid_diag, monitor = _pid_diagnostics(status)
            feedback_source = str(
                pid_diag.get("feedback_velocity_source")
                or monitor.get("feedback_velocity_source")
                or "UNKNOWN"
            ).strip().upper()
            feedback_source_counts[feedback_source] = feedback_source_counts.get(feedback_source, 0) + 1
            output_reason = str(
                pid_diag.get("output_reason")
                or monitor.get("output_reason")
                or "NONE"
            ).strip().upper()
            output_reason_counts[output_reason] = output_reason_counts.get(output_reason, 0) + 1
            max_feedback_v_l_mps = max(max_feedback_v_l_mps, abs(_safe_float(pid_diag.get("v_l"), 0.0)))
            max_feedback_v_r_mps = max(max_feedback_v_r_mps, abs(_safe_float(pid_diag.get("v_r"), 0.0)))
            max_ref_v_l_mps = max(max_ref_v_l_mps, abs(_safe_float(pid_diag.get("v_l_ref"), 0.0)))
            max_ref_v_r_mps = max(max_ref_v_r_mps, abs(_safe_float(pid_diag.get("v_r_ref"), 0.0)))
            max_v_cmd_mps = max(max_v_cmd_mps, abs(_safe_float(pid_diag.get("v_cmd"), 0.0)))
            guidance_heading_hold = dict(pid_diag.get("guidance_heading_hold") or {})
            if bool(guidance_heading_hold.get("active", False)):
                guidance_heading_hold_active_samples += 1
            guidance_mode = (
                str(guidance_heading_hold.get("mode", "") or "").strip() or "UNKNOWN"
            )
            guidance_heading_hold_mode_counts[guidance_mode] = (
                guidance_heading_hold_mode_counts.get(guidance_mode, 0) + 1
            )
            guidance_heading_hold_peak_heading_error_deg = max(
                guidance_heading_hold_peak_heading_error_deg,
                abs(_safe_float(guidance_heading_hold.get("heading_error_deg"), 0.0)),
            )
            guidance_heading_hold_peak_correction_rad_s = max(
                guidance_heading_hold_peak_correction_rad_s,
                abs(_safe_float(guidance_heading_hold.get("omega_correction_rad_s"), 0.0)),
            )
            reading = _encoder_reading(status)
            left_quadrature_direction_seen |= reading["left_direction_source"] == "QUADRATURE_AB"
            right_quadrature_direction_seen |= reading["right_direction_source"] == "QUADRATURE_AB"
            encoder_progress_m = 0.5 * (
                (reading["left_distance_m"] - start_encoder["left_distance_m"])
                + (reading["right_distance_m"] - start_encoder["right_distance_m"])
            )

            pose = _get_pose(status)
            progress = _pose_distance(start_pose, pose)
            heading_delta = abs(_normalize_angle_deg(float(pose["theta_deg"]) - start_heading))
            max_heading_delta_deg = max(max_heading_delta_deg, heading_delta)
            if heading_delta > 15.0:
                break
            if encoder_progress_m >= float(args.target_distance_m):
                break
            if progress >= float(args.target_distance_m):
                break
            time.sleep(float(args.poll_s))
    except Exception as exc:
        command_error = str(exc)
    finally:
        try:
            _send_command_checked(
                "set_twist",
                token=token,
                timeout_s=4.0,
                v=0.0,
                omega=0.0,
                motion_source=DEFAULT_MOTION_SOURCE,
            )
        except Exception as exc:
            command_error = command_error or f"stop_command:{exc}"

    normal_stop_confirmed = False
    try:
        end_status = _wait_until_stopped(timeout_s=float(args.stop_timeout_s))
        normal_stop_confirmed = True
    except Exception as exc:
        command_error = command_error or f"stop_wait:{exc}"

    end_pose = _get_pose(end_status)
    end_encoder = _encoder_reading(end_status)
    metrics = {
        "encoder_start": start_encoder,
        "encoder_end": end_encoder,
        "left_pulse_delta": end_encoder["left_pulses"] - start_encoder["left_pulses"],
        "right_pulse_delta": end_encoder["right_pulses"] - start_encoder["right_pulses"],
        "left_distance_delta_m": end_encoder["left_distance_m"] - start_encoder["left_distance_m"],
        "right_distance_delta_m": end_encoder["right_distance_m"] - start_encoder["right_distance_m"],
        "encoder_progress_m": 0.5 * (
            (end_encoder["left_distance_m"] - start_encoder["left_distance_m"])
            + (end_encoder["right_distance_m"] - start_encoder["right_distance_m"])
        ),
        "ekf_progress_m": _pose_distance(start_pose, end_pose),
        "max_heading_delta_deg": max_heading_delta_deg,
        "max_pwm_left": max_pwm_left,
        "max_pwm_right": max_pwm_right,
        "sample_count": sample_count,
        "pwm_saturation_samples": pwm_saturation_samples,
        "pwm_saturation_fraction": (
            float(pwm_saturation_samples) / float(sample_count) if sample_count > 0 else 0.0
        ),
        "feedback_source_counts": dict(sorted(feedback_source_counts.items())),
        "output_reason_counts": dict(sorted(output_reason_counts.items())),
        "max_feedback_v_l_mps": max_feedback_v_l_mps,
        "max_feedback_v_r_mps": max_feedback_v_r_mps,
        "max_ref_v_l_mps": max_ref_v_l_mps,
        "max_ref_v_r_mps": max_ref_v_r_mps,
        "max_v_cmd_mps": max_v_cmd_mps,
        "guidance_heading_hold_active_samples": guidance_heading_hold_active_samples,
        "guidance_heading_hold_active_fraction": (
            float(guidance_heading_hold_active_samples) / float(sample_count)
            if sample_count > 0
            else 0.0
        ),
        "guidance_heading_hold_mode_counts": dict(
            sorted(guidance_heading_hold_mode_counts.items())
        ),
        "guidance_heading_hold_peak_heading_error_deg": (
            guidance_heading_hold_peak_heading_error_deg
        ),
        "guidance_heading_hold_peak_correction_rad_s": (
            guidance_heading_hold_peak_correction_rad_s
        ),
        "left_quadrature_direction_seen": left_quadrature_direction_seen,
        "right_quadrature_direction_seen": right_quadrature_direction_seen,
        "failsafe_seen": failsafe_seen,
        "safety_block_seen": safety_block_seen,
        "normal_stop_confirmed": normal_stop_confirmed,
        "command_error": command_error,
    }
    failures = evaluate_audit(metrics, target_distance_m=float(args.target_distance_m))
    if command_error:
        failures.append("command_error")
    return {
        "success": not failures,
        "test": "kit0085_encoder_motor_0p3m_audit",
        "hardware": "DFRobot KIT0085",
        "command_path": "set_twist via runtime/commands.jsonl",
        "target_distance_m": float(args.target_distance_m),
        "speed_mps": float(args.speed_mps),
        "control_mode_target": str(target_control_mode),
        "control_mode_applied": str(control_mode_applied),
        "precheck": precheck,
        "start_lidar_confidence": (
            None if not math.isfinite(start_lidar_confidence) else float(start_lidar_confidence)
        ),
        "min_start_lidar_confidence": float(args.min_start_lidar_confidence),
        "warnings": list(dict.fromkeys(warnings)),
        "metrics": metrics,
        "failures": list(dict.fromkeys(failures)),
        "artifact": str(LATEST_RESULT_PATH),
    }


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        print(json.dumps({"success": False, "error": str(exc)}, sort_keys=True))
        return 40

    parser = argparse.ArgumentParser(description="Audit KIT0085 motors, A/B encoders, and slow 0.3m forward motion.")
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--target-distance-m", type=float, default=0.30)
    parser.add_argument("--speed-mps", type=float, default=0.06)
    parser.add_argument("--move-timeout-s", type=float, default=12.0)
    parser.add_argument("--stop-timeout-s", type=float, default=6.0)
    parser.add_argument("--required-clearance-m", type=float, default=0.65)
    parser.add_argument("--min-start-lidar-confidence", type=float, default=0.75)
    parser.add_argument("--control-mode", choices=(CANONICAL_CONTROL_MODE,), default=CANONICAL_CONTROL_MODE)
    parser.add_argument("--keepalive-s", type=float, default=0.15)
    parser.add_argument("--poll-s", type=float, default=0.05)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()

    try:
        result = run(args)
    except Exception as exc:
        error_text = str(exc)
        failure = "audit_exception"
        if error_text.startswith("start_lidar_confidence_low"):
            failure = "start_lidar_confidence_low"
        elif error_text == "start_lidar_confidence_missing":
            failure = "start_lidar_confidence_missing"
        result = {
            "success": False,
            "test": "kit0085_encoder_motor_0p3m_audit",
            "error": error_text,
            "failures": [failure],
            "artifact": str(LATEST_RESULT_PATH),
        }
    _write_json_atomic(LATEST_RESULT_PATH, result)
    if args.compact:
        metrics = dict(result.get("metrics") or {})
        warnings = result.get("warnings") or []
        print(
            "KIT0085_AUDIT "
            f"result={'PASS' if result.get('success') else 'FAIL'} "
            f"mode={result.get('control_mode_applied') or ''} "
            f"ekf={_safe_float(metrics.get('ekf_progress_m')):.3f}m "
            f"enc_l={_safe_float(metrics.get('left_distance_delta_m')):.3f}m "
            f"enc_r={_safe_float(metrics.get('right_distance_delta_m')):.3f}m "
            f"pulses={_safe_int(metrics.get('left_pulse_delta'))}/"
            f"{_safe_int(metrics.get('right_pulse_delta'))} "
            f"failures={','.join(result.get('failures') or []) or 'none'} "
            f"warnings={','.join(warnings) or 'none'}"
        )
    else:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
