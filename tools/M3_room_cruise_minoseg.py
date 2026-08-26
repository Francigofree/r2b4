#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Live Room Cruise v2 behavior and movement-quality validation."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from argparse import Namespace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools import room_cruise_v2_live as cruise  # noqa: E402
from tools.M3_emberkovetes_mozgasminoseg import (  # noqa: E402
    _angle_delta_deg,
    _episode_count,
    _gate,
    _json_safe,
    _mean,
    _oscillation_metrics,
    _percentile,
    _ratio,
    _safe_float,
    _std,
    _write_json,
    _write_jsonl,
)


AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_minoseg.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_minoseg_summary.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "M3_room_cruise_minoseg_samples.jsonl"
INCIDENT_PATH = AGENT_TESTS_DIR / "latest_M3_room_cruise_minoseg_incident.json"


def _configured_track_width_m() -> float:
    try:
        payload = json.loads((PROJECT_ROOT / "conf" / "fizika.json").read_text(encoding="utf-8"))
        return max(0.05, float(payload["nyomtav_szelesseg_m"]))
    except Exception:
        return 0.185


DEFAULT_THRESHOLDS: Dict[str, float] = {
    "min_total_samples": 160,
    "min_run_count_required": 2,
    "min_run_duration_s": 50.0,
    "min_moving_ratio": 0.18,
    "min_progress_per_run_m": 0.45,
    "min_straight_samples": 18,
    "min_left_arc_samples": 12,
    "min_right_arc_samples": 12,
    "min_pivot_samples": 8,
    "min_stop_samples": 8,
    "min_start_episodes": 1,
    "min_obstacle_near_samples": 8,
    "min_obstacle_avoidance_samples": 8,
    "min_recovery_continue_samples": 3,
    "min_wheel_direction_samples": 8,
    "wheel_direction_target_min_mps": 0.010,
    "collision_clearance_min_m": 0.25,
    "front_warning_m": 0.70,
    "front_close_m": 0.48,
    "close_forward_v_max_mps": 0.035,
    "loop_expected_hz": 50.0,
    "loop_frequency_p10_min_hz": 40.0,
    "loop_frequency_below_45_ratio_max": 0.10,
    "loop_dt_p95_max_s": 0.030,
    "loop_dt_p99_max_s": 0.045,
    "loop_jitter_p95_max_s": 0.012,
    "loop_budget_total_ema_p95_max_ms": 35.0,
    "moving_v_min_mps": 0.018,
    "moving_omega_min_rad_s": 0.055,
    "straight_omega_max_rad_s": 0.040,
    "arc_omega_min_rad_s": 0.055,
    "pivot_v_max_mps": 0.014,
    "pivot_omega_min_rad_s": 0.12,
    "linear_tracking_abs_p90_max_mps": 0.035,
    "linear_tracking_rel_p90_max": 0.75,
    "omega_tracking_abs_p90_max_rad_s": 0.20,
    "omega_tracking_rel_p90_max": 0.80,
    "wheel_tracking_abs_p90_max_mps": 0.035,
    "wheel_wrong_sign_ratio_max": 0.05,
    "wheel_pwm_reversal_max": 0,
    "wheel_transition_settle_s": 0.30,
    "straight_oscillation_amplitude_p90_max_rad_s": 0.10,
    "straight_oscillation_frequency_max_hz": 1.50,
    "arc_residual_amplitude_p90_max_rad_s": 0.16,
    "arc_wrong_direction_ratio_max": 0.05,
    "arc_outer_wheel_ratio_min": 0.90,
    "arc_same_direction_ratio_min": 0.90,
    "pivot_opposite_track_ratio_min": 0.90,
    "pivot_wrong_direction_ratio_max": 0.05,
    "pwm_step_p95_max": 0.12,
    "velocity_step_p95_max_mps": 0.055,
    "omega_step_p95_max_rad_s": 0.24,
    "unexplained_pwm_zero_ratio_max": 0.04,
    "idle_creep_ratio_max": 0.03,
    "stop_settle_time_max_s": 1.0,
    "sensor_rate_disagreement_p90_max_rad_s": 0.22,
    "sensor_endpoint_heading_disagreement_max_deg": 15.0,
    "min_heading_change_for_endpoint_gate_deg": 8.0,
    "planner_phase_switch_rate_max_hz": 0.35,
    "small_area_radius_max_m": 0.22,
    "small_area_progress_min_m": 0.50,
    "repeat_progress_cv_max": 0.65,
    "repeat_start_pose_delta_min_m": 0.15,
    "repeat_start_heading_delta_min_deg": 12.0,
    "repeat_start_clearance_delta_min_m": 0.12,
    "min_m3_track_exec_moving_ratio": 0.95,
    "track_width_m": _configured_track_width_m(),
}


EXPECTED_LIVE_MOTION_HU = (
    "A GUI profil ket egymast koveto, kb. 60 masodperces Room Cruise v2 futast indit. "
    "A robotnak elore dominansan kell cirkalnia, kb. 0.30 m/s celmaximum mellett siman gyorsitva/lassitva, "
    "akadaly elott lassitva vagy megallva, majd bal es jobb ivvel, szuk helyzetben rovid helyben "
    "fordulassal vagy recovery utan tovabbhaladva. Minden futas vegen determinisztikusan meg kell allnia. "
    "Kozvetlen PWM, legacy vagy szolgaltatasi mozgasi ut nem megengedett."
)


def _load_thresholds(path: str) -> Dict[str, float]:
    if not path:
        return {}
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("thresholds JSON must contain an object")
    unknown = sorted(set(payload) - set(DEFAULT_THRESHOLDS))
    if unknown:
        raise ValueError(f"unknown threshold keys: {unknown}")
    return {str(key): float(value) for key, value in payload.items()}


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _compact_number(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 6) if math.isfinite(value) else None
    return value


def _evidence(samples: Sequence[Dict[str, Any]], indices: Iterable[int], limit: int = 8) -> List[Dict[str, Any]]:
    fields = (
        "sample_index",
        "run_index",
        "elapsed_s",
        "run_elapsed_s",
        "m3_class",
        "m3_causes",
        "room_cruise_v2_reason",
        "local_navigation_mode",
        "resolved_name",
        "resolved_layer",
        "resolved_command_type",
        "resolved_execution_mode",
        "motion_execution_mode",
        "resolved_v",
        "resolved_omega",
        "requested_track_reference_source",
        "requested_track_left_mps",
        "requested_track_right_mps",
        "control_track_reference_mode",
        "control_output_reason",
        "local_nav_pivot_track_required",
        "actual_v",
        "actual_omega",
        "target_left_mps",
        "target_right_mps",
        "actual_left_mps",
        "actual_right_mps",
        "pwm_left",
        "pwm_right",
        "front_m",
        "left_clearance_m",
        "right_clearance_m",
        "watchdog_freq_hz",
        "watchdog_period_s",
        "loop_budget_total_ema_ms",
        "localization_mode",
        "safety_reason",
        "stop_type",
        "stop_reason",
    )
    output: List[Dict[str, Any]] = []
    seen = set()
    for index in indices:
        idx = int(index)
        if idx in seen or idx < 0 or idx >= len(samples):
            continue
        seen.add(idx)
        sample = samples[idx]
        output.append({key: _compact_number(sample.get(key)) for key in fields if key in sample})
        if len(output) >= int(limit):
            break
    return output


def _run_groups(samples: Sequence[Dict[str, Any]]) -> Dict[int, List[int]]:
    groups: Dict[int, List[int]] = {}
    for index, sample in enumerate(samples):
        run_index = int(sample.get("run_index", 0) or 0)
        groups.setdefault(run_index, []).append(index)
    return groups


def _pose_progress(samples: Sequence[Dict[str, Any]], indices: Sequence[int]) -> float:
    total = 0.0
    previous: Optional[Dict[str, Any]] = None
    for index in indices:
        pose = dict(samples[index].get("pose") or {})
        if previous is not None:
            total += math.hypot(
                _safe_float(pose.get("x"), 0.0) - _safe_float(previous.get("x"), 0.0),
                _safe_float(pose.get("y"), 0.0) - _safe_float(previous.get("y"), 0.0),
            )
        previous = pose
    return float(total)


def _run_duration(samples: Sequence[Dict[str, Any]], indices: Sequence[int]) -> float:
    if len(indices) < 2:
        return 0.0
    return max(
        0.0,
        _safe_float(samples[indices[-1]].get("run_elapsed_s"), 0.0)
        - _safe_float(samples[indices[0]].get("run_elapsed_s"), 0.0),
    )


def _phase_switch_rate(samples: Sequence[Dict[str, Any]], indices: Sequence[int]) -> Optional[float]:
    if len(indices) < 2:
        return None
    last = ""
    switches = 0
    for index in indices:
        sample = samples[index]
        phase = str(
            sample.get("room_cruise_v2_reason")
            or sample.get("local_navigation_mode")
            or sample.get("resolved_command_type")
            or ""
        )
        if last and phase and phase != last:
            switches += 1
        if phase:
            last = phase
    duration = _run_duration(samples, indices)
    return float(switches / duration) if duration > 0.0 else None


def _start_pose_diversity(samples: Sequence[Dict[str, Any]], groups: Dict[int, List[int]], limits: Dict[str, float]) -> Dict[str, Any]:
    starts = []
    for run_index in sorted(groups):
        first = samples[groups[run_index][0]]
        pose = dict(first.get("pose") or {})
        clearances = [
            _safe_float(first.get("front_m"), math.nan),
            _safe_float(first.get("left_clearance_m"), math.nan),
            _safe_float(first.get("right_clearance_m"), math.nan),
        ]
        starts.append(
            {
                "run_index": run_index,
                "x": _safe_float(pose.get("x"), math.nan),
                "y": _safe_float(pose.get("y"), math.nan),
                "theta_deg": _safe_float(pose.get("theta_deg"), math.nan),
                "clearances": clearances,
            }
        )
    max_position_delta = 0.0
    max_heading_delta = 0.0
    max_clearance_delta = 0.0
    for left_idx, left in enumerate(starts):
        for right in starts[left_idx + 1 :]:
            if all(_finite(left.get(key)) and _finite(right.get(key)) for key in ("x", "y")):
                max_position_delta = max(
                    max_position_delta,
                    math.hypot(float(left["x"]) - float(right["x"]), float(left["y"]) - float(right["y"])),
                )
            if _finite(left.get("theta_deg")) and _finite(right.get("theta_deg")):
                max_heading_delta = max(
                    max_heading_delta,
                    abs(_angle_delta_deg(float(left["theta_deg"]), float(right["theta_deg"]))),
                )
            for a, b in zip(left.get("clearances") or [], right.get("clearances") or []):
                if _finite(a) and _finite(b):
                    max_clearance_delta = max(max_clearance_delta, abs(float(a) - float(b)))
    ok = bool(
        max_position_delta >= limits["repeat_start_pose_delta_min_m"]
        or max_heading_delta >= limits["repeat_start_heading_delta_min_deg"]
        or max_clearance_delta >= limits["repeat_start_clearance_delta_min_m"]
    )
    return {
        "start_count": len(starts),
        "max_position_delta_m": max_position_delta,
        "max_heading_delta_deg": max_heading_delta,
        "max_clearance_delta_m": max_clearance_delta,
        "diverse_enough": ok,
        "starts": starts,
    }


def _classify_samples(raw_samples: Sequence[Dict[str, Any]], thresholds: Dict[str, float]) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    first_ts_by_run: Dict[int, float] = {}
    previous_by_run: Dict[int, Dict[str, Any]] = {}
    rate_previous_by_run: Dict[int, Dict[str, Any]] = {}
    lidar_previous_by_run: Dict[int, Dict[str, Any]] = {}
    global_first_ts: Optional[float] = None
    expected_dt = 1.0 / max(1.0, thresholds["loop_expected_hz"])

    for index, original in enumerate(raw_samples):
        sample = dict(original)
        track_width = max(
            0.05,
            _safe_float(sample.get("track_width_m"), thresholds["track_width_m"]),
        )
        run_index = int(sample.get("run_index", 0) or 0)
        ts = _safe_float(sample.get("ts"), time.time())
        if global_first_ts is None:
            global_first_ts = ts
        first_ts_by_run.setdefault(run_index, ts)
        previous = previous_by_run.get(run_index)
        rate_previous = rate_previous_by_run.get(run_index)
        lidar_previous = lidar_previous_by_run.get(run_index)
        dt_s = _safe_float(sample.get("dt_s"), 0.0)
        if dt_s <= 0.0 and previous is not None:
            dt_s = max(0.0, ts - _safe_float(previous.get("ts"), ts))
        elapsed_s = _safe_float(sample.get("elapsed_s"), ts - float(global_first_ts))
        run_elapsed_s = _safe_float(sample.get("run_elapsed_s"), ts - first_ts_by_run[run_index])

        resolved_v = _safe_float(sample.get("resolved_v", sample.get("limited_v")), 0.0)
        resolved_omega = _safe_float(sample.get("resolved_omega", sample.get("limited_omega")), 0.0)
        actual_v = _safe_float(sample.get("actual_v"), 0.0)
        actual_omega = _safe_float(sample.get("actual_omega"), 0.0)
        target_left = _safe_float(sample.get("target_left_mps"), 0.0)
        target_right = _safe_float(sample.get("target_right_mps"), 0.0)
        actual_left = _safe_float(sample.get("actual_left_mps"), 0.0)
        actual_right = _safe_float(sample.get("actual_right_mps"), 0.0)
        pwm_left = _safe_float(sample.get("pwm_left"), 0.0)
        pwm_right = _safe_float(sample.get("pwm_right"), 0.0)
        front_m = _safe_float(sample.get("front_m"), math.nan)
        min_clearance = _safe_float(sample.get("min_clearance_m"), front_m)

        moving_cmd = bool(
            abs(resolved_v) >= thresholds["moving_v_min_mps"]
            or abs(resolved_omega) >= thresholds["moving_omega_min_rad_s"]
            or max(abs(target_left), abs(target_right)) >= thresholds["moving_v_min_mps"]
        )
        physical_motion = bool(
            abs(actual_v) >= 0.012
            or abs(actual_omega) >= 0.040
            or max(abs(actual_left), abs(actual_right)) >= 0.012
        )
        straight = bool(moving_cmd and abs(resolved_v) >= thresholds["moving_v_min_mps"] and abs(resolved_omega) <= thresholds["straight_omega_max_rad_s"])
        left_arc = bool(moving_cmd and resolved_v >= thresholds["moving_v_min_mps"] and resolved_omega >= thresholds["arc_omega_min_rad_s"])
        right_arc = bool(moving_cmd and resolved_v >= thresholds["moving_v_min_mps"] and resolved_omega <= -thresholds["arc_omega_min_rad_s"])
        pivot = bool(abs(resolved_v) <= thresholds["pivot_v_max_mps"] and abs(resolved_omega) >= thresholds["pivot_omega_min_rad_s"])
        stop_window = bool(
            abs(resolved_v) < thresholds["moving_v_min_mps"]
            and abs(resolved_omega) < thresholds["moving_omega_min_rad_s"]
            and max(abs(target_left), abs(target_right)) < thresholds["moving_v_min_mps"]
        )
        obstacle_near = bool(math.isfinite(front_m) and front_m <= thresholds["front_warning_m"])
        obstacle_close = bool(math.isfinite(front_m) and front_m <= thresholds["front_close_m"])
        obstacle_avoid = bool(
            obstacle_near
            and (
                left_arc
                or right_arc
                or pivot
                or bool(sample.get("blocked_front", False))
                or "obstacle" in str(sample.get("room_cruise_v2_reason", "") or "").lower()
            )
        )
        previous_hold = bool(
            previous is not None
            and (
                bool(previous.get("m3_stop_window", False))
                or bool(previous.get("m3_obstacle_near", False))
                or bool(previous.get("blocked_front", False))
            )
        )
        recovery_continue = bool(moving_cmd and previous_hold)

        pwm_step = 0.0
        v_step = 0.0
        omega_step = 0.0
        linear_accel = None
        omega_accel = None
        if previous is not None:
            pwm_step = max(
                abs(pwm_left - _safe_float(previous.get("pwm_left"), 0.0)),
                abs(pwm_right - _safe_float(previous.get("pwm_right"), 0.0)),
            )
            v_step = abs(resolved_v - _safe_float(previous.get("resolved_v"), 0.0))
            omega_step = abs(resolved_omega - _safe_float(previous.get("resolved_omega"), 0.0))
            if dt_s > 1e-4:
                linear_accel = (actual_v - _safe_float(previous.get("actual_v"), 0.0)) / dt_s
                omega_accel = (actual_omega - _safe_float(previous.get("actual_omega"), 0.0)) / dt_s

        status_time_s = _safe_float(sample.get("status_time_s"), ts)
        status_version = sample.get("status_version")
        previous_status_version = rate_previous.get("status_version") if rate_previous is not None else None
        status_rate_fresh = bool(
            rate_previous is None
            or status_version is None
            or previous_status_version is None
            or str(status_version) != str(previous_status_version)
        )

        pose = dict(sample.get("pose") or {})
        pose_theta = _safe_float(pose.get("theta_deg"), math.nan)
        lidar_heading = _safe_float(sample.get("lidar_heading_deg"), math.nan)
        lidar_yaw_rate = math.nan
        pose_yaw_rate = math.nan
        if rate_previous is not None and status_rate_fresh:
            rate_dt_s = status_time_s - _safe_float(
                rate_previous.get("status_time_s", rate_previous.get("ts")),
                status_time_s,
            )
            prev_pose = dict(rate_previous.get("pose") or {})
            prev_theta = _safe_float(prev_pose.get("theta_deg"), math.nan)
            if rate_dt_s > 1e-4 and math.isfinite(pose_theta) and math.isfinite(prev_theta):
                pose_yaw_rate = math.radians(_angle_delta_deg(pose_theta, prev_theta)) / rate_dt_s

        lidar_scan_seq = sample.get("lidar_scan_seq")
        previous_lidar_scan_seq = lidar_previous.get("lidar_scan_seq") if lidar_previous is not None else None
        lidar_rate_fresh = bool(
            lidar_previous is None
            or lidar_scan_seq is None
            or previous_lidar_scan_seq is None
            or str(lidar_scan_seq) != str(previous_lidar_scan_seq)
        )
        if lidar_previous is not None and lidar_rate_fresh:
            lidar_time_s = _safe_float(sample.get("lidar_pose_time_s"), status_time_s)
            previous_lidar_time_s = _safe_float(
                lidar_previous.get("lidar_pose_time_s", lidar_previous.get("status_time_s", lidar_previous.get("ts"))),
                lidar_time_s,
            )
            lidar_dt_s = lidar_time_s - previous_lidar_time_s
            previous_lidar_heading = _safe_float(lidar_previous.get("lidar_heading_deg"), math.nan)
            if lidar_dt_s > 1e-4 and math.isfinite(lidar_heading) and math.isfinite(previous_lidar_heading):
                lidar_yaw_rate = math.radians(_angle_delta_deg(lidar_heading, previous_lidar_heading)) / lidar_dt_s

        encoder_yaw_rate = (actual_right - actual_left) / track_width
        imu_gyro_z = _safe_float(sample.get("imu_gyro_z_rad_s"), math.nan)
        if not math.isfinite(imu_gyro_z):
            imu_gyro_z = actual_omega

        loop_period = _safe_float(sample.get("watchdog_period_s"), math.nan)
        loop_freq = _safe_float(sample.get("watchdog_freq_hz"), math.nan)
        loop_slow = bool(
            (math.isfinite(loop_freq) and loop_freq < thresholds["loop_frequency_p10_min_hz"])
            or (math.isfinite(loop_period) and loop_period > 1.0 / thresholds["loop_frequency_p10_min_hz"])
        )
        finite_command = all(
            math.isfinite(value)
            for value in (
                resolved_v,
                resolved_omega,
                target_left,
                target_right,
                pwm_left,
                pwm_right,
            )
        )
        motion_execution_mode = str(
            sample.get("motion_execution_mode")
            or sample.get("resolved_execution_mode")
            or sample.get("execution_mode")
            or ""
        ).upper()
        control_track_reference_mode = str(sample.get("control_track_reference_mode", "") or "").upper()
        requested_track_left = _safe_float(sample.get("requested_track_left_mps"), math.nan)
        requested_track_right = _safe_float(sample.get("requested_track_right_mps"), math.nan)
        requested_track_present = bool(
            math.isfinite(requested_track_left)
            and math.isfinite(requested_track_right)
            and (
                abs(requested_track_left) > 1e-6
                or abs(requested_track_right) > 1e-6
                or stop_window
            )
        )
        m3_track_exec = bool(
            motion_execution_mode == "TRACK_EXEC"
            or control_track_reference_mode in {"TRACK_EXEC", "HEADING_PIVOT"}
        )

        m3_class = "idle"
        if pivot:
            m3_class = "pivot"
        elif left_arc:
            m3_class = "left_arc"
        elif right_arc:
            m3_class = "right_arc"
        elif straight:
            m3_class = "straight"
        elif moving_cmd:
            m3_class = "transition"

        causes: List[str] = []
        if obstacle_near:
            causes.append("OBSTACLE_NEAR")
        if recovery_continue:
            causes.append("RECOVERY_CONTINUE")
        if loop_slow:
            causes.append("CONTROL_LOOP_SLOW")
        if not finite_command:
            causes.append("NONFINITE_COMMAND")
        if abs(actual_omega - resolved_omega) > thresholds["omega_tracking_abs_p90_max_rad_s"]:
            causes.append("OMEGA_TRACKING_ERROR")
        if pwm_step > thresholds["pwm_step_p95_max"]:
            causes.append("PWM_STEP")

        sample.update(
            {
                "sample_index": index,
                "run_index": run_index,
                "elapsed_s": elapsed_s,
                "run_elapsed_s": run_elapsed_s,
                "dt_s": dt_s,
                "resolved_v": resolved_v,
                "resolved_omega": resolved_omega,
                "actual_v": actual_v,
                "actual_omega": actual_omega,
                "target_left_mps": target_left,
                "target_right_mps": target_right,
                "actual_left_mps": actual_left,
                "actual_right_mps": actual_right,
                "pwm_left": pwm_left,
                "pwm_right": pwm_right,
                "front_m": front_m,
                "min_clearance_m": min_clearance,
                "m3_moving_cmd": moving_cmd,
                "m3_physical_motion": physical_motion,
                "m3_straight_window": straight,
                "m3_left_arc_window": left_arc,
                "m3_right_arc_window": right_arc,
                "m3_arc_window": bool(left_arc or right_arc),
                "m3_pivot_window": pivot,
                "m3_stop_window": stop_window,
                "m3_obstacle_near": obstacle_near,
                "m3_obstacle_close": obstacle_close,
                "m3_obstacle_avoidance": obstacle_avoid,
                "m3_recovery_continue": recovery_continue,
                "m3_residual_omega_rad_s": actual_omega - resolved_omega,
                "m3_pwm_step": pwm_step,
                "m3_v_step_mps": v_step,
                "m3_omega_step_rad_s": omega_step,
                "m3_linear_accel_mps2": linear_accel,
                "m3_omega_accel_rad_s2": omega_accel,
                "m3_loop_jitter_s": abs(loop_period - expected_dt) if math.isfinite(loop_period) else None,
                "m3_encoder_yaw_rate_rad_s": encoder_yaw_rate,
                "m3_track_width_m": track_width,
                "m3_pose_yaw_rate_rad_s": pose_yaw_rate,
                "m3_lidar_yaw_rate_rad_s": lidar_yaw_rate,
                "m3_imu_yaw_rate_rad_s": imu_gyro_z,
                "m3_status_rate_fresh": status_rate_fresh,
                "m3_lidar_rate_fresh": lidar_rate_fresh,
                "m3_loop_slow": loop_slow,
                "m3_finite_command": finite_command,
                "m3_motion_execution_mode": motion_execution_mode,
                "m3_track_exec": m3_track_exec,
                "m3_requested_track_reference_present": requested_track_present,
                "m3_class": m3_class,
                "m3_causes": causes,
            }
        )
        samples.append(sample)
        previous_by_run[run_index] = sample
        if status_rate_fresh:
            rate_previous_by_run[run_index] = sample
        if lidar_rate_fresh and math.isfinite(lidar_heading):
            lidar_previous_by_run[run_index] = sample
    return samples


def _wheel_direction_metrics(
    samples: Sequence[Dict[str, Any]],
    indices: Sequence[int],
    *,
    side: str,
    direction: int,
    limits: Dict[str, float],
) -> Dict[str, Any]:
    target_key = f"target_{side}_mps"
    actual_key = f"actual_{side}_mps"
    pwm_key = f"pwm_{side}"
    raw_dir_indices = [
        idx
        for idx in indices
        if direction * _safe_float(samples[idx].get(target_key), 0.0) >= limits["wheel_direction_target_min_mps"]
    ]
    transition_indices = [
        idx
        for idx in raw_dir_indices
        if _finite(samples[idx].get("motion_segment_age_s"))
        and _safe_float(samples[idx].get("motion_segment_age_s"), 0.0) < limits["wheel_transition_settle_s"]
    ]
    dir_indices = [idx for idx in raw_dir_indices if idx not in set(transition_indices)]
    errors = [
        abs(_safe_float(samples[idx].get(actual_key), 0.0) - _safe_float(samples[idx].get(target_key), 0.0))
        for idx in dir_indices
    ]
    wrong_sign = [
        idx
        for idx in dir_indices
        if _safe_float(samples[idx].get(actual_key), 0.0) * _safe_float(samples[idx].get(target_key), 0.0) < 0.0
    ]
    reversals: List[int] = []
    for idx in dir_indices:
        if idx <= 0:
            continue
        if int(samples[idx - 1].get("run_index", 0) or 0) != int(samples[idx].get("run_index", 0) or 0):
            continue
        prev_pwm = _safe_float(samples[idx - 1].get(pwm_key), 0.0)
        cur_pwm = _safe_float(samples[idx].get(pwm_key), 0.0)
        cur_target = _safe_float(samples[idx].get(target_key), 0.0)
        prev_target = _safe_float(samples[idx - 1].get(target_key), 0.0)
        if cur_target * prev_target > 0.0 and abs(cur_target) >= limits["moving_v_min_mps"] and prev_pwm * cur_pwm < 0.0:
            reversals.append(idx)
    return {
        "sample_count": len(dir_indices),
        "raw_sample_count": len(raw_dir_indices),
        "transition_sample_count": len(transition_indices),
        "transition_wrong_sign_count": sum(
            1
            for idx in transition_indices
            if _safe_float(samples[idx].get(actual_key), 0.0) * _safe_float(samples[idx].get(target_key), 0.0) < 0.0
        ),
        "settle_window_s": limits["wheel_transition_settle_s"],
        "error_abs_p90_mps": _percentile(errors, 0.90),
        "wrong_sign_ratio": _ratio(len(wrong_sign), len(dir_indices)),
        "pwm_sign_reversal_count": len(reversals),
        "wrong_sign_indices": wrong_sign,
        "reversal_indices": reversals,
        "indices": dir_indices,
    }


def _stop_settle_metrics(
    samples: Sequence[Dict[str, Any]],
    stop_indices: Sequence[int],
    limits: Dict[str, float],
) -> Dict[str, Any]:
    episodes: List[List[int]] = []
    for idx in stop_indices:
        if (
            not episodes
            or idx != episodes[-1][-1] + 1
            or int(samples[idx].get("run_index", 0) or 0)
            != int(samples[episodes[-1][-1]].get("run_index", 0) or 0)
        ):
            episodes.append([idx])
        else:
            episodes[-1].append(idx)

    settle_limit_s = float(limits["stop_settle_time_max_s"])
    eligible: List[Dict[str, Any]] = []
    short_count = 0
    idle_evaluation_indices: List[int] = []
    for episode in episodes:
        first_idx = episode[0]
        preceded_by_motion = bool(
            first_idx > 0
            and int(samples[first_idx - 1].get("run_index", 0) or 0)
            == int(samples[first_idx].get("run_index", 0) or 0)
            and bool(samples[first_idx - 1].get("m3_moving_cmd", False))
        )
        explicit_post_stop = any(bool(samples[idx].get("post_stop_sample", False)) for idx in episode)
        if not preceded_by_motion and not explicit_post_stop:
            continue
        first_ts = _safe_float(samples[first_idx].get("status_time_s", samples[first_idx].get("ts")), math.nan)
        ages: List[float] = []
        for idx in episode:
            age = _safe_float(samples[idx].get("motion_segment_age_s"), math.nan)
            if not math.isfinite(age):
                current_ts = _safe_float(samples[idx].get("status_time_s", samples[idx].get("ts")), math.nan)
                age = current_ts - first_ts if math.isfinite(current_ts) and math.isfinite(first_ts) else math.nan
            ages.append(age)
        finite_ages = [age for age in ages if math.isfinite(age) and age >= 0.0]
        coverage_s = max(finite_ages) if finite_ages else 0.0
        if coverage_s < settle_limit_s:
            short_count += 1
            continue

        quiet = [
            max(
                abs(_safe_float(samples[idx].get("actual_v"), 0.0)),
                abs(_safe_float(samples[idx].get("actual_omega"), 0.0)),
            )
            <= 0.040
            for idx in episode
        ]
        settle_time_s = None
        for position, age in enumerate(ages):
            if math.isfinite(age) and quiet[position] and all(quiet[position:]):
                settle_time_s = max(0.0, float(age))
                break
        after_settle_limit = [
            idx
            for idx, age in zip(episode, ages)
            if math.isfinite(age) and age >= settle_limit_s
        ]
        idle_evaluation_indices.extend(after_settle_limit)
        eligible.append(
            {
                "start_sample_index": first_idx,
                "end_sample_index": episode[-1],
                "coverage_s": coverage_s,
                "settle_time_s": settle_time_s,
                "settled_within_limit": bool(settle_time_s is not None and settle_time_s <= settle_limit_s),
            }
        )

    settle_times = [float(item["settle_time_s"]) for item in eligible if item.get("settle_time_s") is not None]
    unsettled = [item for item in eligible if not bool(item.get("settled_within_limit", False))]
    return {
        "settle_limit_s": settle_limit_s,
        "eligible_episode_count": len(eligible),
        "short_episode_count": short_count,
        "unsettled_episode_count": len(unsettled),
        "settle_time_max_s": max(settle_times) if settle_times else None,
        "episodes": eligible,
        "idle_evaluation_indices": idle_evaluation_indices,
    }


def _arc_metrics(
    samples: Sequence[Dict[str, Any]],
    indices: Sequence[int],
    *,
    side: str,
    limits: Dict[str, float],
) -> Dict[str, Any]:
    sign = 1 if side == "left" else -1
    arc_indices = [idx for idx in indices if sign * _safe_float(samples[idx].get("resolved_omega"), 0.0) > 0.0]
    wrong_direction = [
        idx
        for idx in arc_indices
        if _safe_float(samples[idx].get("resolved_omega"), 0.0) * _safe_float(samples[idx].get("actual_omega"), 0.0) < 0.0
    ]
    same_direction = []
    outer_ok = []
    for idx in arc_indices:
        left_target = _safe_float(samples[idx].get("target_left_mps"), 0.0)
        right_target = _safe_float(samples[idx].get("target_right_mps"), 0.0)
        if left_target * right_target > 0.0:
            same_direction.append(idx)
        if sign > 0:
            if right_target > left_target:
                outer_ok.append(idx)
        elif left_target > right_target:
            outer_ok.append(idx)
    return {
        "sample_count": len(arc_indices),
        "wrong_direction_ratio": _ratio(len(wrong_direction), len(arc_indices)),
        "same_direction_ratio": _ratio(len(same_direction), len(arc_indices)),
        "outer_wheel_ratio": _ratio(len(outer_ok), len(arc_indices)),
        "oscillation": _oscillation_metrics(samples, arc_indices, limits["straight_omega_max_rad_s"]),
        "wrong_direction_indices": wrong_direction,
        "indices": arc_indices,
    }


def _sensor_endpoint_metrics(samples: Sequence[Dict[str, Any]], moving_indices: Sequence[int]) -> Dict[str, Any]:
    if len(moving_indices) < 2:
        return {"available": False, "reason": "insufficient_moving_samples"}

    rows: List[Dict[str, Any]] = []
    seen_versions = set()
    for idx in range(moving_indices[0], moving_indices[-1] + 1):
        sample = samples[idx]
        version = sample.get("status_version")
        if version is not None:
            version_key = str(version)
            if version_key in seen_versions:
                continue
            seen_versions.add(version_key)
        rows.append(sample)

    def measurement_time(row: Dict[str, Any], key: str) -> float:
        return _safe_float(
            row.get(key, row.get("status_time_s", row.get("ts"))),
            math.nan,
        )

    def integrated_heading_deg(field_getter, *, time_key: str, sequence_key: str = "") -> Optional[float]:
        total = 0.0
        used = 0
        previous_value = None
        previous_row = None
        previous_sequence = None
        for row in rows:
            value = field_getter(row)
            if not _finite(value):
                continue
            sequence = row.get(sequence_key) if sequence_key else None
            if sequence_key and sequence is not None and sequence == previous_sequence:
                continue
            current = float(value)
            dt_s = (
                measurement_time(row, time_key) - measurement_time(previous_row, time_key)
                if previous_row is not None
                else math.nan
            )
            same_run = bool(
                previous_row is not None
                and row.get("run_index") == previous_row.get("run_index")
            )
            if previous_value is not None and same_run and math.isfinite(dt_s) and 0.0 < dt_s <= 1.0:
                total += _angle_delta_deg(current, previous_value)
                used += 1
            previous_value = current
            previous_row = row
            previous_sequence = sequence
        return float(total) if used else None

    def integrated_rate_deg(rate_getter) -> Optional[float]:
        total_rad = 0.0
        used = 0
        for previous, current in zip(rows, rows[1:]):
            dt_s = measurement_time(current, "status_time_s") - measurement_time(previous, "status_time_s")
            rate_prev = rate_getter(previous)
            rate_current = rate_getter(current)
            if not (
                math.isfinite(dt_s)
                and 0.0 < dt_s <= 1.0
                and _finite(rate_prev)
                and _finite(rate_current)
            ):
                continue
            total_rad += 0.5 * (float(rate_prev) + float(rate_current)) * dt_s
            used += 1
        return math.degrees(total_rad) if used else None

    changes = {
        "ekf_pose": integrated_heading_deg(
            lambda row: (row.get("pose") or {}).get("theta_deg"),
            time_key="status_time_s",
        ),
        "imu_gyro": integrated_rate_deg(lambda row: row.get("imu_gyro_z_rad_s")),
        "encoder": integrated_rate_deg(lambda row: row.get("m3_encoder_yaw_rate_rad_s")),
        "lidar": integrated_heading_deg(
            lambda row: row.get("lidar_heading_deg"),
            time_key="lidar_pose_time_s",
            sequence_key="lidar_scan_seq",
        ),
    }
    changes = {
        name: float(value)
        for name, value in changes.items()
        if value is not None and math.isfinite(value)
    }
    pairs: Dict[str, float] = {}
    names = sorted(changes)
    for left_index, left in enumerate(names):
        for right in names[left_index + 1 :]:
            pairs[f"{left}_vs_{right}"] = abs(changes[left] - changes[right])
    return {
        "available": len(changes) >= 2,
        "heading_change_deg": changes,
        "pair_disagreement_deg": pairs,
        "max_pair_disagreement_deg": max(pairs.values()) if pairs else None,
        "max_observed_heading_change_deg": max((abs(value) for value in changes.values()), default=0.0),
        "frame_contract": "R2B4_BOOT_ROBOT_MAP:+x_forward,+y_left,+yaw_ccw",
        "imu_source": "gyro_z_integrated",
        "deduplicated_sample_count": len(rows),
        "start_sample_index": moving_indices[0],
        "end_sample_index": moving_indices[-1],
    }


def analyze_samples(
    raw_samples: Sequence[Dict[str, Any]],
    thresholds: Optional[Dict[str, float]] = None,
    *,
    base_results: Optional[Sequence[Dict[str, Any]]] = None,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        limits.update({key: float(value) for key, value in thresholds.items() if key in limits})
    samples = _classify_samples(raw_samples, limits)
    total = len(samples)
    groups = _run_groups(samples)
    run_summaries = []
    for run_index in sorted(groups):
        indices = groups[run_index]
        progress = _pose_progress(samples, indices)
        duration = _run_duration(samples, indices)
        moving_count = sum(1 for idx in indices if bool(samples[idx].get("m3_moving_cmd", False)))
        run_summaries.append(
            {
                "run_index": run_index,
                "sample_count": len(indices),
                "duration_s": duration,
                "progress_m": progress,
                "moving_ratio": _ratio(moving_count, len(indices)),
                "phase_switch_rate_hz": _phase_switch_rate(samples, indices),
            }
        )

    moving_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_moving_cmd", False))]
    physical_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_physical_motion", False))]
    straight_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_straight_window", False))]
    left_arc_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_left_arc_window", False))]
    right_arc_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_right_arc_window", False))]
    arc_indices = sorted(left_arc_indices + right_arc_indices)
    pivot_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_pivot_window", False))]
    stop_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_stop_window", False))]
    obstacle_near_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_obstacle_near", False))]
    obstacle_close_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_obstacle_close", False))]
    obstacle_avoidance_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_obstacle_avoidance", False))]
    recovery_continue_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_recovery_continue", False))]

    linear_indices = [idx for idx in moving_indices if abs(_safe_float(samples[idx].get("resolved_v"), 0.0)) >= limits["moving_v_min_mps"]]
    omega_indices = [idx for idx in moving_indices if abs(_safe_float(samples[idx].get("resolved_omega"), 0.0)) >= limits["moving_omega_min_rad_s"]]
    linear_abs_errors = [
        abs(_safe_float(samples[idx].get("actual_v"), 0.0) - _safe_float(samples[idx].get("resolved_v"), 0.0))
        for idx in linear_indices
    ]
    linear_rel_errors = [
        err / max(0.01, abs(_safe_float(samples[idx].get("resolved_v"), 0.0)))
        for err, idx in zip(linear_abs_errors, linear_indices)
    ]
    omega_abs_errors = [
        abs(_safe_float(samples[idx].get("actual_omega"), 0.0) - _safe_float(samples[idx].get("resolved_omega"), 0.0))
        for idx in omega_indices
    ]
    omega_rel_errors = [
        err / max(0.03, abs(_safe_float(samples[idx].get("resolved_omega"), 0.0)))
        for err, idx in zip(omega_abs_errors, omega_indices)
    ]

    wheel_direction = {
        "left_forward": _wheel_direction_metrics(samples, moving_indices, side="left", direction=1, limits=limits),
        "right_forward": _wheel_direction_metrics(samples, moving_indices, side="right", direction=1, limits=limits),
        "left_reverse": _wheel_direction_metrics(samples, moving_indices, side="left", direction=-1, limits=limits),
        "right_reverse": _wheel_direction_metrics(samples, moving_indices, side="right", direction=-1, limits=limits),
    }
    left_arc = _arc_metrics(samples, left_arc_indices, side="left", limits=limits)
    right_arc = _arc_metrics(samples, right_arc_indices, side="right", limits=limits)
    straight_oscillation = _oscillation_metrics(samples, straight_indices, limits["straight_omega_max_rad_s"])

    pivot_wrong = [
        idx
        for idx in pivot_indices
        if _safe_float(samples[idx].get("resolved_omega"), 0.0) * _safe_float(samples[idx].get("actual_omega"), 0.0) < 0.0
    ]
    pivot_opposite = [
        idx
        for idx in pivot_indices
        if _safe_float(samples[idx].get("target_left_mps"), 0.0) * _safe_float(samples[idx].get("target_right_mps"), 0.0) < 0.0
    ]
    pivot_metrics = {
        "sample_count": len(pivot_indices),
        "opposite_track_ratio": _ratio(len(pivot_opposite), len(pivot_indices)),
        "wrong_direction_ratio": _ratio(len(pivot_wrong), len(pivot_indices)),
        "wrong_direction_indices": pivot_wrong,
    }

    front_values = [_safe_float(sample.get("front_m"), math.nan) for sample in samples]
    front_values = [value for value in front_values if math.isfinite(value)]
    min_front = min(front_values) if front_values else None
    min_clearance_values = [_safe_float(sample.get("min_clearance_m"), math.nan) for sample in samples]
    min_clearance_values = [value for value in min_clearance_values if math.isfinite(value)]
    min_clearance = min(min_clearance_values) if min_clearance_values else min_front
    open_v_values = [
        abs(_safe_float(samples[idx].get("resolved_v"), 0.0))
        for idx in moving_indices
        if _safe_float(samples[idx].get("front_m"), 0.0) >= max(1.0, limits["front_warning_m"] + 0.20)
    ]
    near_v_values = [abs(_safe_float(samples[idx].get("resolved_v"), 0.0)) for idx in obstacle_near_indices]
    close_v_values = [abs(_safe_float(samples[idx].get("resolved_v"), 0.0)) for idx in obstacle_close_indices]
    obstacle_metrics = {
        "near_sample_count": len(obstacle_near_indices),
        "close_sample_count": len(obstacle_close_indices),
        "avoidance_sample_count": len(obstacle_avoidance_indices),
        "recovery_continue_sample_count": len(recovery_continue_indices),
        "open_v_p50_mps": _percentile(open_v_values, 0.50),
        "near_v_p50_mps": _percentile(near_v_values, 0.50),
        "close_v_max_mps": _percentile(close_v_values, 1.0),
        "min_front_clearance_m": min_front,
        "min_clearance_m": min_clearance,
    }

    periods = [_safe_float(sample.get("watchdog_period_s"), math.nan) for sample in samples]
    periods = [value for value in periods if math.isfinite(value) and value > 0.0]
    frequencies = [_safe_float(sample.get("watchdog_freq_hz"), math.nan) for sample in samples]
    frequencies = [value for value in frequencies if math.isfinite(value) and value > 0.0]
    expected_dt = 1.0 / max(1.0, limits["loop_expected_hz"])
    jitters = [abs(value - expected_dt) for value in periods]
    loop_budget_values = [_safe_float(sample.get("loop_budget_total_ema_ms"), math.nan) for sample in samples]
    loop_budget_values = [value for value in loop_budget_values if math.isfinite(value)]
    below_45_ratio = _ratio(sum(1 for value in frequencies if value < 45.0), len(frequencies))
    loop_slow_indices = [idx for idx, sample in enumerate(samples) if bool(sample.get("m3_loop_slow", False))]
    loop_budget_slices: Dict[str, float] = {}
    for sample in samples:
        slices = dict((dict(sample.get("loop_budget") or {})).get("slices") or {})
        for name, payload in slices.items():
            val = _safe_float((payload or {}).get("ema_ms"), math.nan)
            if math.isfinite(val):
                loop_budget_slices[str(name)] = max(loop_budget_slices.get(str(name), 0.0), val)
    timing_metrics = {
        "expected_hz": limits["loop_expected_hz"],
        "frequency_p10_hz": _percentile(frequencies, 0.10),
        "frequency_p50_hz": _percentile(frequencies, 0.50),
        "frequency_below_45_ratio": below_45_ratio,
        "dt_p50_s": _percentile(periods, 0.50),
        "dt_p95_s": _percentile(periods, 0.95),
        "dt_p99_s": _percentile(periods, 0.99),
        "dt_max_s": _percentile(periods, 1.0),
        "jitter_p95_s": _percentile(jitters, 0.95),
        "loop_budget_total_ema_p95_ms": _percentile(loop_budget_values, 0.95),
        "loop_budget_component_ema_max_ms": dict(sorted(loop_budget_slices.items(), key=lambda item: item[1], reverse=True)[:8]),
        "slow_sample_count": len(loop_slow_indices),
    }

    pwm_steps = [_safe_float(sample.get("m3_pwm_step"), 0.0) for sample in samples if _safe_float(sample.get("dt_s"), 0.0) > 0.0]
    v_steps = [_safe_float(sample.get("m3_v_step_mps"), 0.0) for sample in samples if _safe_float(sample.get("dt_s"), 0.0) > 0.0]
    omega_steps = [_safe_float(sample.get("m3_omega_step_rad_s"), 0.0) for sample in samples if _safe_float(sample.get("dt_s"), 0.0) > 0.0]
    unexplained_pwm_zero = [
        idx
        for idx in moving_indices
        if max(abs(_safe_float(samples[idx].get("pwm_left"), 0.0)), abs(_safe_float(samples[idx].get("pwm_right"), 0.0))) < 0.015
        and bool(samples[idx].get("safety_allow", True))
    ]
    stop_settle = _stop_settle_metrics(samples, stop_indices, limits)
    idle_evaluation_indices = list(stop_settle.get("idle_evaluation_indices") or [])
    idle_creep = [
        idx
        for idx in idle_evaluation_indices
        if max(
            abs(_safe_float(samples[idx].get("actual_v"), 0.0)),
            abs(_safe_float(samples[idx].get("actual_omega"), 0.0)),
        )
        > 0.040
    ]
    start_indices = []
    for idx in moving_indices:
        if idx <= 0 or int(samples[idx - 1].get("run_index", 0) or 0) != int(samples[idx].get("run_index", 0) or 0):
            start_indices.append(idx)
        elif not bool(samples[idx - 1].get("m3_moving_cmd", False)):
            start_indices.append(idx)
    start_stop_metrics = {
        "start_episode_count": _episode_count(start_indices),
        "stop_sample_count": len(stop_indices),
        "stop_episode_count": _episode_count(stop_indices),
        "unexplained_pwm_zero_ratio": _ratio(len(unexplained_pwm_zero), len(moving_indices)),
        "idle_creep_ratio": _ratio(len(idle_creep), len(idle_evaluation_indices)),
        "unexplained_pwm_zero_samples": len(unexplained_pwm_zero),
        "idle_creep_samples": len(idle_creep),
        "idle_evaluation_samples": len(idle_evaluation_indices),
        "settle_limit_s": stop_settle["settle_limit_s"],
        "settle_coverage_episode_count": stop_settle["eligible_episode_count"],
        "short_stop_episode_count": stop_settle["short_episode_count"],
        "unsettled_episode_count": stop_settle["unsettled_episode_count"],
        "settle_time_max_s": stop_settle["settle_time_max_s"],
        "settle_episodes": stop_settle["episodes"],
    }
    smoothness_metrics = {
        "pwm_step_p95": _percentile(pwm_steps, 0.95),
        "velocity_step_p95_mps": _percentile(v_steps, 0.95),
        "omega_step_p95_rad_s": _percentile(omega_steps, 0.95),
        "pwm_step_peak": _percentile(pwm_steps, 1.0),
    }

    route_bad = [
        idx
        for idx in moving_indices
        if not (
            bool(samples[idx].get("room_cruise_v2_active", False))
            or bool(samples[idx].get("resolved_has_room_cruise_v2_details", False))
            or str(samples[idx].get("resolved_name", "") or "") == "room_cruise_v2_local_navigation"
        )
    ]
    forbidden = [
        idx
        for idx, sample in enumerate(samples)
        if bool(sample.get("service_motion_active", False))
        or "LEGACY" in str(sample.get("active_motion_layer", "") or "").upper()
        or str(sample.get("active_motion_type", "") or "").lower() in {"set_motor_pwm", "set_tank", "step_tank"}
        or str(sample.get("resolved_command_type", "") or "").lower() in {"set_motor_pwm", "set_tank", "step_tank"}
        or bool(sample.get("direct_motor_bypass", False))
    ]
    owner_conflicts = [
        idx
        for idx, sample in enumerate(samples)
        if bool(sample.get("command_owner_conflict", False)) or int(sample.get("active_route_count", 0) or 0) > 1
    ]
    contract_violations = [
        idx
        for idx, sample in enumerate(samples)
        if bool(sample.get("primitive_contract_violation", False))
        or bool(sample.get("control_execution_contract_violation", False))
    ]
    nonfinite_commands = [idx for idx, sample in enumerate(samples) if not bool(sample.get("m3_finite_command", True))]
    safety_events = [
        idx
        for idx, sample in enumerate(samples)
        if not bool(sample.get("safety_allow", True))
        or bool(sample.get("watchdog_stop_triggered", False))
        or str(sample.get("stop_type", "") or "").upper() in {"EMERGENCY_STOP", "FAILSAFE"}
    ]
    localization_contradictions = [
        idx
        for idx, sample in enumerate(samples)
        if (
            str(sample.get("localization_mode", "") or "").upper() == "LOST"
            and bool(sample.get("localization_allow_motion", False))
        )
        or not bool(sample.get("localization_truth_consistent", True))
        or (
            str(sample.get("localization_truth_state", "") or "").upper() == "LOST"
            and bool(sample.get("localization_truth_allow_motion", False))
        )
    ]
    motion_actual_ssot_bad = [
        idx
        for idx in moving_indices
        if str(samples[idx].get("motion_actual_ssot", "EKF_POSE_ODOMETRY_SSOT") or "EKF_POSE_ODOMETRY_SSOT")
        != "EKF_POSE_ODOMETRY_SSOT"
    ]
    m3_track_exec_indices = [idx for idx in moving_indices if bool(samples[idx].get("m3_track_exec", False))]
    m3_track_route_bad = [
        idx
        for idx in moving_indices
        if (
            bool(samples[idx].get("room_cruise_v2_active", False))
            or bool(samples[idx].get("resolved_has_room_cruise_v2_details", False))
            or str(samples[idx].get("resolved_name", "") or "") == "room_cruise_v2_local_navigation"
        )
        and not bool(samples[idx].get("m3_track_exec", False))
    ]
    pivot_legacy_path = [
        idx
        for idx in pivot_indices
        if (
            not bool(samples[idx].get("m3_track_exec", False))
            or not bool(samples[idx].get("m3_requested_track_reference_present", False))
            or bool(samples[idx].get("local_nav_pivot_track_required", False))
        )
    ]

    rate_disagreements: List[float] = []
    rate_disagreement_indices: List[int] = []
    pose_lidar_rate_disagreements: List[float] = []
    for idx in moving_indices:
        imu_rate = _safe_float(samples[idx].get("m3_imu_yaw_rate_rad_s"), math.nan)
        encoder_rate = _safe_float(samples[idx].get("m3_encoder_yaw_rate_rad_s"), math.nan)
        if math.isfinite(imu_rate) and math.isfinite(encoder_rate):
            disagreement = abs(imu_rate - encoder_rate)
            rate_disagreements.append(disagreement)
            if disagreement > limits["sensor_rate_disagreement_p90_max_rad_s"]:
                rate_disagreement_indices.append(idx)
        pose_rate = _safe_float(samples[idx].get("m3_pose_yaw_rate_rad_s"), math.nan)
        lidar_rate = _safe_float(samples[idx].get("m3_lidar_yaw_rate_rad_s"), math.nan)
        if math.isfinite(pose_rate) and math.isfinite(lidar_rate):
            pose_lidar_rate_disagreements.append(abs(pose_rate - lidar_rate))
    endpoint = _sensor_endpoint_metrics(samples, moving_indices)
    localization_metrics = {
        "contradiction_samples": len(localization_contradictions),
        "sensor_rate_disagreement_p90_rad_s": _percentile(rate_disagreements, 0.90),
        "sensor_rate_pair": "encoder_vs_imu_same_status_snapshot",
        "asynchronous_pose_lidar_rate_disagreement_p90_rad_s": _percentile(
            pose_lidar_rate_disagreements,
            0.90,
        ),
        "asynchronous_pose_lidar_rate_is_gate_input": False,
        "endpoint": endpoint,
    }

    progress_values = [float(item.get("progress_m", 0.0)) for item in run_summaries]
    progress_cv = None
    if len(progress_values) > 1 and _mean(progress_values) and _mean(progress_values) > 1e-6:
        progress_cv = float(_std(progress_values) or 0.0) / float(_mean(progress_values) or 1.0)
    start_diversity = _start_pose_diversity(samples, groups, limits)
    behavior_metrics = {
        "run_count": len(groups),
        "total_progress_m": sum(progress_values),
        "progress_per_run_m": progress_values,
        "progress_cv": progress_cv,
        "moving_ratio": _ratio(len(moving_indices), total),
        "physical_motion_ratio": _ratio(len(physical_indices), total),
        "phase_switch_rate_max_hz": max(
            [
                float(item["phase_switch_rate_hz"])
                for item in run_summaries
                if item.get("phase_switch_rate_hz") is not None
            ],
            default=None,
        ),
        "start_pose_diversity": start_diversity,
    }

    base_results_list = [dict(item or {}) for item in (base_results or [])]
    base_failures = [
        idx
        for idx, item in enumerate(base_results_list)
        if str(item.get("status") or (item.get("summary") or {}).get("status") or "").upper() != "PASS"
    ]

    metrics: Dict[str, Any] = {
        "coverage": {
            "total_samples": total,
            "run_count": len(groups),
            "run_summaries": run_summaries,
            "moving_samples": len(moving_indices),
            "straight_samples": len(straight_indices),
            "left_arc_samples": len(left_arc_indices),
            "right_arc_samples": len(right_arc_indices),
            "pivot_samples": len(pivot_indices),
            "stop_samples": len(stop_indices),
            "obstacle_near_samples": len(obstacle_near_indices),
            "obstacle_avoidance_samples": len(obstacle_avoidance_indices),
            "recovery_continue_samples": len(recovery_continue_indices),
        },
        "safety": {
            "min_front_clearance_m": min_front,
            "min_clearance_m": min_clearance,
            "safety_event_samples": len(safety_events),
            "nonfinite_command_samples": len(nonfinite_commands),
        },
        "integrity": {
            "forbidden_path_samples": len(forbidden),
            "wrong_route_moving_samples": len(route_bad),
            "owner_conflict_samples": len(owner_conflicts),
            "execution_contract_violation_samples": len(contract_violations),
            "motion_actual_ssot_bad_samples": len(motion_actual_ssot_bad),
            "m3_track_exec_moving_samples": len(m3_track_exec_indices),
            "m3_track_exec_moving_ratio": _ratio(len(m3_track_exec_indices), len(moving_indices)),
            "m3_track_route_bad_samples": len(m3_track_route_bad),
            "pivot_legacy_path_samples": len(pivot_legacy_path),
        },
        "timing": timing_metrics,
        "motion_tracking": {
            "linear_samples": len(linear_indices),
            "linear_abs_error_p90_mps": _percentile(linear_abs_errors, 0.90),
            "linear_rel_error_p90": _percentile(linear_rel_errors, 0.90),
            "omega_samples": len(omega_indices),
            "omega_abs_error_p90_rad_s": _percentile(omega_abs_errors, 0.90),
            "omega_rel_error_p90": _percentile(omega_rel_errors, 0.90),
        },
        "wheel_direction": {
            key: {inner_key: value for inner_key, value in data.items() if not inner_key.endswith("_indices") and inner_key != "indices"}
            for key, data in wheel_direction.items()
        },
        "straight": straight_oscillation,
        "left_arc": {key: value for key, value in left_arc.items() if key not in {"indices", "wrong_direction_indices"}},
        "right_arc": {key: value for key, value in right_arc.items() if key not in {"indices", "wrong_direction_indices"}},
        "pivot": pivot_metrics,
        "start_stop": start_stop_metrics,
        "smoothness": smoothness_metrics,
        "localization": localization_metrics,
        "obstacle": obstacle_metrics,
        "behavior": behavior_metrics,
        "base_room_cruise_v2": {
            "run_count": len(base_results_list),
            "failed_run_indices": base_failures,
            "statuses": [
                str(item.get("status") or (item.get("summary") or {}).get("status") or "MISSING")
                for item in base_results_list
            ],
        },
    }

    gates: Dict[str, Dict[str, Any]] = {}

    suff_missing: List[str] = []
    if total < int(limits["min_total_samples"]):
        suff_missing.append("total_samples")
    if len(groups) < int(limits["min_run_count_required"]):
        suff_missing.append("independent_runs")
    if len(straight_indices) < int(limits["min_straight_samples"]):
        suff_missing.append("straight_motion")
    if len(left_arc_indices) < int(limits["min_left_arc_samples"]):
        suff_missing.append("left_arc")
    if len(right_arc_indices) < int(limits["min_right_arc_samples"]):
        suff_missing.append("right_arc")
    if len(pivot_indices) < int(limits["min_pivot_samples"]):
        suff_missing.append("in_place_pivot")
    if len(stop_indices) < int(limits["min_stop_samples"]):
        suff_missing.append("stop_hold")
    if len(obstacle_near_indices) < int(limits["min_obstacle_near_samples"]):
        suff_missing.append("obstacle_slowdown")
    if len(obstacle_avoidance_indices) < int(limits["min_obstacle_avoidance_samples"]):
        suff_missing.append("obstacle_avoidance")
    if len(recovery_continue_indices) < int(limits["min_recovery_continue_samples"]):
        suff_missing.append("recovery_continue")
    gates["measurement_sufficiency"] = _gate(
        "PASS" if not suff_missing else "INCONCLUSIVE",
        observed=metrics["coverage"],
        requirement="required Room Cruise movement forms and at least two independent live runs are observed",
        reason="missing:" + ",".join(suff_missing) if suff_missing else "",
    )

    safety_fail_indices = sorted(set(safety_events + nonfinite_commands))
    safety_status = "INCONCLUSIVE" if min_clearance is None else "PASS"
    if safety_fail_indices or (min_clearance is not None and min_clearance < limits["collision_clearance_min_m"]):
        safety_status = "FAIL"
    gates["safety"] = _gate(
        safety_status,
        observed=metrics["safety"],
        requirement=f"no safety/failsafe/non-finite event and clearance >= {limits['collision_clearance_min_m']} m",
        evidence=_evidence(samples, safety_fail_indices),
    )

    integrity_fail = sorted(
        set(
            forbidden
            + route_bad
            + owner_conflicts
            + contract_violations
            + motion_actual_ssot_bad
            + m3_track_route_bad
            + pivot_legacy_path
        )
    )
    m3_track_ratio_ok = _safe_float(metrics["integrity"]["m3_track_exec_moving_ratio"], 0.0) >= limits[
        "min_m3_track_exec_moving_ratio"
    ]
    gates["control_chain_integrity"] = _gate(
        "PASS" if not integrity_fail and moving_indices and m3_track_ratio_ok else ("INCONCLUSIVE" if not moving_indices else "FAIL"),
        observed=metrics["integrity"],
        requirement="Room Cruise v2 -> local navigation -> M3 TRACK executor, no legacy/service/direct path, EKF actual SSOT",
        evidence=_evidence(samples, integrity_fail),
    )

    loop_available = bool(periods and frequencies)
    loop_ok = bool(
        loop_available
        and _safe_float(timing_metrics["frequency_p10_hz"], 0.0) >= limits["loop_frequency_p10_min_hz"]
        and _safe_float(timing_metrics["frequency_below_45_ratio"], 1.0) <= limits["loop_frequency_below_45_ratio_max"]
        and _safe_float(timing_metrics["dt_p95_s"], math.inf) <= limits["loop_dt_p95_max_s"]
        and _safe_float(timing_metrics["dt_p99_s"], math.inf) <= limits["loop_dt_p99_max_s"]
        and _safe_float(timing_metrics["jitter_p95_s"], math.inf) <= limits["loop_jitter_p95_max_s"]
        and _safe_float(timing_metrics["loop_budget_total_ema_p95_ms"], 0.0) <= limits["loop_budget_total_ema_p95_max_ms"]
    )
    gates["control_loop_timing"] = _gate(
        "PASS" if loop_ok else ("INCONCLUSIVE" if not loop_available else "FAIL"),
        observed=timing_metrics,
        requirement="50 Hz target: p10 >= 40 Hz, dt/jitter p95/p99 bounded, loop-budget p95 bounded",
        evidence=_evidence(samples, loop_slow_indices),
    )

    linear_ok = bool(
        len(linear_indices) >= int(limits["min_straight_samples"])
        and _safe_float(metrics["motion_tracking"]["linear_abs_error_p90_mps"], math.inf) <= limits["linear_tracking_abs_p90_max_mps"]
        and _safe_float(metrics["motion_tracking"]["linear_rel_error_p90"], math.inf) <= limits["linear_tracking_rel_p90_max"]
    )
    omega_ok = bool(
        len(omega_indices) >= int(limits["min_left_arc_samples"])
        and _safe_float(metrics["motion_tracking"]["omega_abs_error_p90_rad_s"], math.inf) <= limits["omega_tracking_abs_p90_max_rad_s"]
        and _safe_float(metrics["motion_tracking"]["omega_rel_error_p90"], math.inf) <= limits["omega_tracking_rel_p90_max"]
    )
    if len(linear_indices) < int(limits["min_straight_samples"]) or len(omega_indices) < int(limits["min_left_arc_samples"]):
        motion_status = "INCONCLUSIVE"
    else:
        motion_status = "PASS" if linear_ok and omega_ok else "FAIL"
    gates["motion_command_fidelity"] = _gate(
        motion_status,
        observed=metrics["motion_tracking"],
        requirement="actual linear/angular motion follows the limited resolved command within low-speed robot tolerance",
        evidence=_evidence(samples, sorted(linear_indices + omega_indices, key=lambda idx: abs(_safe_float(samples[idx].get("m3_residual_omega_rad_s"), 0.0)), reverse=True)),
    )

    for name, data in wheel_direction.items():
        if int(data["sample_count"]) < int(limits["min_wheel_direction_samples"]):
            status = "INCONCLUSIVE"
            reason = "wheel_direction_not_exercised"
        else:
            status = "PASS" if (
                _safe_float(data["error_abs_p90_mps"], math.inf) <= limits["wheel_tracking_abs_p90_max_mps"]
                and _safe_float(data["wrong_sign_ratio"], 1.0) <= limits["wheel_wrong_sign_ratio_max"]
                and int(data["pwm_sign_reversal_count"]) <= int(limits["wheel_pwm_reversal_max"])
            ) else "FAIL"
            reason = ""
        gates[f"wheel_{name}"] = _gate(
            status,
            observed={key: value for key, value in data.items() if key not in {"indices", "wrong_sign_indices", "reversal_indices"}},
            requirement="wheel target is tracked in this direction, no wrong sign, no PWM sign reversal",
            reason=reason,
            evidence=_evidence(samples, list(data.get("wrong_sign_indices") or []) + list(data.get("reversal_indices") or [])),
        )

    if len(straight_indices) < int(limits["min_straight_samples"]):
        gates["straight_motion_quality"] = _gate(
            "INCONCLUSIVE",
            observed=straight_oscillation,
            requirement=f"straight samples >= {int(limits['min_straight_samples'])}",
            reason="straight_motion_not_exercised",
        )
    else:
        straight_ok = bool(
            _safe_float(straight_oscillation.get("amplitude_p90_rad_s"), math.inf)
            <= limits["straight_oscillation_amplitude_p90_max_rad_s"]
            and _safe_float(straight_oscillation.get("approx_frequency_hz"), 0.0)
            <= limits["straight_oscillation_frequency_max_hz"]
        )
        gates["straight_motion_quality"] = _gate(
            "PASS" if straight_ok else "FAIL",
            observed=straight_oscillation,
            requirement="low-amplitude, low-frequency straight residual yaw oscillation",
            evidence=_evidence(samples, sorted(straight_indices, key=lambda idx: abs(_safe_float(samples[idx].get("m3_residual_omega_rad_s"), 0.0)), reverse=True)),
        )

    for side, data, min_key in (
        ("left", left_arc, "min_left_arc_samples"),
        ("right", right_arc, "min_right_arc_samples"),
    ):
        if int(data["sample_count"]) < int(limits[min_key]):
            status = "INCONCLUSIVE"
            reason = f"{side}_arc_not_exercised"
        else:
            status = "PASS" if (
                _safe_float(data["wrong_direction_ratio"], 1.0) <= limits["arc_wrong_direction_ratio_max"]
                and _safe_float(data["same_direction_ratio"], 0.0) >= limits["arc_same_direction_ratio_min"]
                and _safe_float(data["outer_wheel_ratio"], 0.0) >= limits["arc_outer_wheel_ratio_min"]
                and _safe_float((data["oscillation"] or {}).get("amplitude_p90_rad_s"), math.inf)
                <= limits["arc_residual_amplitude_p90_max_rad_s"]
            ) else "FAIL"
            reason = ""
        gates[f"{side}_arc_quality"] = _gate(
            status,
            observed={key: value for key, value in data.items() if key not in {"indices", "wrong_direction_indices"}},
            requirement="arc turns requested direction, outer wheel faster, both wheels same direction, residual smooth",
            reason=reason,
            evidence=_evidence(samples, list(data.get("wrong_direction_indices") or [])),
        )

    if len(pivot_indices) < int(limits["min_pivot_samples"]):
        gates["in_place_pivot_quality"] = _gate(
            "INCONCLUSIVE",
            observed=pivot_metrics,
            requirement=f"pivot samples >= {int(limits['min_pivot_samples'])}",
            reason="in_place_pivot_not_exercised",
        )
    else:
        pivot_ok = bool(
            _safe_float(pivot_metrics["opposite_track_ratio"], 0.0) >= limits["pivot_opposite_track_ratio_min"]
            and _safe_float(pivot_metrics["wrong_direction_ratio"], 1.0) <= limits["pivot_wrong_direction_ratio_max"]
        )
        gates["in_place_pivot_quality"] = _gate(
            "PASS" if pivot_ok else "FAIL",
            observed=pivot_metrics,
            requirement="pivot uses opposite wheel directions and rotates in requested yaw direction",
            evidence=_evidence(samples, pivot_wrong),
        )

    start_stop_ok = bool(
        int(start_stop_metrics["start_episode_count"]) >= int(limits["min_start_episodes"])
        and int(start_stop_metrics["stop_sample_count"]) >= int(limits["min_stop_samples"])
        and int(start_stop_metrics["settle_coverage_episode_count"]) >= 1
        and int(start_stop_metrics["unsettled_episode_count"]) == 0
        and _safe_float(start_stop_metrics["settle_time_max_s"], math.inf) <= limits["stop_settle_time_max_s"]
        and _safe_float(start_stop_metrics["unexplained_pwm_zero_ratio"], 1.0) <= limits["unexplained_pwm_zero_ratio_max"]
        and _safe_float(start_stop_metrics["idle_creep_ratio"], 1.0) <= limits["idle_creep_ratio_max"]
    )
    if (
        int(start_stop_metrics["start_episode_count"]) < int(limits["min_start_episodes"])
        or int(start_stop_metrics["stop_sample_count"]) < int(limits["min_stop_samples"])
        or int(start_stop_metrics["settle_coverage_episode_count"]) < 1
    ):
        start_stop_status = "INCONCLUSIVE"
    else:
        start_stop_status = "PASS" if start_stop_ok else "FAIL"
    gates["start_stop_quality"] = _gate(
        start_stop_status,
        observed=start_stop_metrics,
        requirement="start and stop are observed; measured motion settles within the declared 1 s window, then idle holds without unexplained PWM zero or creep",
        evidence=_evidence(samples, unexplained_pwm_zero + idle_creep),
    )

    smooth_available = bool(pwm_steps and v_steps and omega_steps)
    smooth_ok = bool(
        smooth_available
        and _safe_float(smoothness_metrics["pwm_step_p95"], math.inf) <= limits["pwm_step_p95_max"]
        and _safe_float(smoothness_metrics["velocity_step_p95_mps"], math.inf) <= limits["velocity_step_p95_max_mps"]
        and _safe_float(smoothness_metrics["omega_step_p95_rad_s"], math.inf) <= limits["omega_step_p95_max_rad_s"]
    )
    gates["motion_smoothness_comfort"] = _gate(
        "PASS" if smooth_ok else ("INCONCLUSIVE" if not smooth_available else "FAIL"),
        observed=smoothness_metrics,
        requirement="PWM, linear command, and angular command steps stay within calm indoor motion limits",
        evidence=_evidence(samples, sorted(range(len(samples)), key=lambda idx: _safe_float(samples[idx].get("m3_pwm_step"), 0.0), reverse=True)),
    )

    endpoint_change = _safe_float(endpoint.get("max_observed_heading_change_deg"), 0.0)
    endpoint_disagreement = endpoint.get("max_pair_disagreement_deg")
    loc_available = bool(rate_disagreements) and bool(endpoint.get("available")) and endpoint_change >= limits["min_heading_change_for_endpoint_gate_deg"]
    loc_ok = bool(
        loc_available
        and not localization_contradictions
        and _safe_float(localization_metrics["sensor_rate_disagreement_p90_rad_s"], math.inf) <= limits["sensor_rate_disagreement_p90_max_rad_s"]
        and _safe_float(endpoint_disagreement, math.inf) <= limits["sensor_endpoint_heading_disagreement_max_deg"]
    )
    gates["localization_sensor_consistency"] = _gate(
        "PASS" if loc_ok else ("INCONCLUSIVE" if not loc_available else "FAIL"),
        observed=localization_metrics,
        requirement="same-snapshot encoder/IMU yaw rates agree and timestamp-correct EKF/LIDAR/IMU/encoder endpoint changes agree; no LOST/allow contradiction",
        reason="" if loc_available else "insufficient_heading_or_rate_evidence",
        evidence=_evidence(samples, localization_contradictions + rate_disagreement_indices + [endpoint.get("start_sample_index", -1), endpoint.get("end_sample_index", -1)]),
    )

    slowdown_available = len(obstacle_near_indices) >= int(limits["min_obstacle_near_samples"])
    near_v = obstacle_metrics["near_v_p50_mps"]
    open_v = obstacle_metrics["open_v_p50_mps"]
    close_v = obstacle_metrics["close_v_max_mps"]
    slowdown_ok = bool(
        slowdown_available
        and (
            (near_v is not None and open_v is not None and float(near_v) <= max(0.015, 0.90 * float(open_v)))
            or (close_v is not None and float(close_v) <= limits["close_forward_v_max_mps"])
        )
    )
    gates["obstacle_slowdown"] = _gate(
        "PASS" if slowdown_ok else ("INCONCLUSIVE" if not slowdown_available else "FAIL"),
        observed=obstacle_metrics,
        requirement="front obstacle exposure is observed and command speed decreases before close range",
        reason="" if slowdown_available else "obstacle_not_exercised",
        evidence=_evidence(samples, obstacle_near_indices),
    )

    avoidance_available = len(obstacle_avoidance_indices) >= int(limits["min_obstacle_avoidance_samples"])
    avoidance_ok = bool(avoidance_available and len(recovery_continue_indices) >= int(limits["min_recovery_continue_samples"]))
    gates["obstacle_avoidance_recovery"] = _gate(
        "PASS" if avoidance_ok else ("INCONCLUSIVE" if not avoidance_available else "FAIL"),
        observed=obstacle_metrics,
        requirement="obstacle avoidance and recovery/continue after blocking are observed",
        reason="" if avoidance_available else "avoidance_not_exercised",
        evidence=_evidence(samples, obstacle_avoidance_indices + recovery_continue_indices),
    )

    progress_ok = all(float(item.get("progress_m", 0.0)) >= limits["min_progress_per_run_m"] for item in run_summaries) if run_summaries else False
    duration_ok = all(float(item.get("duration_s", 0.0)) >= limits["min_run_duration_s"] for item in run_summaries) if run_summaries else False
    moving_ratio_ok = _safe_float(behavior_metrics["moving_ratio"], 0.0) >= limits["min_moving_ratio"]
    phase_switch = behavior_metrics["phase_switch_rate_max_hz"]
    phase_ok = phase_switch is None or float(phase_switch) <= limits["planner_phase_switch_rate_max_hz"]
    behavior_ok = bool(progress_ok and duration_ok and moving_ratio_ok and phase_ok)
    gates["room_cruise_behavior"] = _gate(
        "PASS" if behavior_ok else ("INCONCLUSIVE" if not run_summaries else "FAIL"),
        observed=behavior_metrics,
        requirement="sustained progress, no unnecessary standing, no rapid planner thrashing, no small-area patrol",
        evidence=_evidence(samples, moving_indices[:4] + moving_indices[-4:]),
    )

    repeat_available = len(groups) >= int(limits["min_run_count_required"])
    repeat_ok = bool(
        repeat_available
        and (behavior_metrics["progress_cv"] is None or float(behavior_metrics["progress_cv"]) <= limits["repeat_progress_cv_max"])
        and bool(start_diversity.get("diverse_enough", False))
    )
    gates["repeatability"] = _gate(
        "PASS" if repeat_ok else ("INCONCLUSIVE" if not repeat_available or not bool(start_diversity.get("diverse_enough", False)) else "FAIL"),
        observed={"progress_cv": behavior_metrics["progress_cv"], "start_pose_diversity": start_diversity},
        requirement="at least two runs with different start pose/heading/clearance and no large quality degradation",
        reason="" if repeat_available else "need_multiple_live_runs",
    )

    if not base_results_list:
        gates["base_room_cruise_v2_gate"] = _gate(
            "INCONCLUSIVE",
            observed=metrics["base_room_cruise_v2"],
            requirement="underlying room_cruise_v2_live gate result is available for every run",
            reason="missing_base_result",
        )
    else:
        gates["base_room_cruise_v2_gate"] = _gate(
            "PASS" if not base_failures else "FAIL",
            observed=metrics["base_room_cruise_v2"],
            requirement="existing Room Cruise v2 live gate passes for every run",
        )

    required_gates = [gate for gate in gates.values() if bool(gate.get("required", True))]
    if any(gate.get("status") == "FAIL" for gate in required_gates):
        status = "FAIL"
    elif any(gate.get("status") == "INCONCLUSIVE" for gate in required_gates):
        status = "INCONCLUSIVE"
    else:
        status = "PASS"
    failed = [name for name, gate in gates.items() if gate.get("status") == "FAIL"]
    inconclusive = [name for name, gate in gates.items() if gate.get("status") == "INCONCLUSIVE"]
    closure = (
        "LOW_MID_LEVELS_CLOSED"
        if status == "PASS"
        else ("LOW_MID_LEVELS_NOT_CLOSED" if status == "FAIL" else "INSUFFICIENT_EVIDENCE")
    )

    result = {
        "schema": "M3_ROOM_CRUISE_MINOSEG_V1",
        "status": status,
        "success": status == "PASS",
        "closure_verdict": closure,
        "generated_ts": time.time(),
        "thresholds": limits,
        "expected_live_motion_hu": EXPECTED_LIVE_MOTION_HU,
        "mandatory_gates": list(gates.keys()),
        "inconclusive_evidence_missing": inconclusive,
        "immediate_fail_conditions": [
            "collision_or_clearance_below_collision_min",
            "safety_or_failsafe_event",
            "non_finite_or_wrong_sign_motion_command",
            "legacy_service_or_direct_pwm_motion_path",
            "room_cruise_motion_not_on_m3_track_executor_path",
            "command_owner_conflict_or_execution_contract_violation",
            "localization_lost_but_motion_allowed_or_sensor_contradiction",
            "wheel_or_pivot_kinematic_sign_error",
            "room_cruise_v2_base_gate_failure",
        ],
        "required_live_runs": {
            "minimum_count": int(limits["min_run_count_required"]),
            "required_variation": [
                "different start pose or heading",
                "different observed clearance/obstacle exposure",
                "left and right dominant route evidence",
            ],
            "implemented_default": "2 runs, with a pause window between runs",
        },
        "metrics": metrics,
        "gates": gates,
        "failed_gates": failed,
        "inconclusive_gates": inconclusive,
        "diagnostic_cause_groups": _diagnostic_cause_groups(failed, inconclusive),
        "plain_summary_hu": _plain_summary(status, closure, metrics, failed, inconclusive),
    }
    return result, samples


def _diagnostic_cause_groups(failed: Sequence[str], inconclusive: Sequence[str]) -> Dict[str, List[str]]:
    cause_map = {
        "loop_timing": {"control_loop_timing"},
        "wheel_speed_loop": {"wheel_left_forward", "wheel_right_forward", "wheel_left_reverse", "wheel_right_reverse"},
        "feedforward_or_deadzone": {"start_stop_quality", "motion_smoothness_comfort"},
        "heading_hold": {"straight_motion_quality"},
        "localization": {"localization_sensor_consistency"},
        "planner_decision_thrashing": {"room_cruise_behavior"},
        "obstacle_avoidance": {"obstacle_slowdown", "obstacle_avoidance_recovery"},
        "kinematic_or_sign_error": {"left_arc_quality", "right_arc_quality", "in_place_pivot_quality"},
        "command_path_overwrite": {"control_chain_integrity"},
        "motor_or_wheel_asymmetry": {"wheel_left_forward", "wheel_right_forward", "wheel_left_reverse", "wheel_right_reverse"},
        "measurement_evidence": {"measurement_sufficiency", "repeatability"},
    }
    all_names = set(failed) | set(inconclusive)
    grouped: Dict[str, List[str]] = {}
    matched = set()
    for cause, gate_names in cause_map.items():
        hits = sorted(all_names & gate_names)
        if hits:
            grouped[cause] = hits
            matched.update(hits)
    unknown = sorted(all_names - matched)
    if unknown:
        grouped["unknown_or_correlated"] = unknown
    return grouped


def _plain_summary(
    status: str,
    closure: str,
    metrics: Dict[str, Any],
    failed: Sequence[str],
    inconclusive: Sequence[str],
) -> str:
    coverage = dict(metrics.get("coverage") or {})
    behavior = dict(metrics.get("behavior") or {})
    safety = dict(metrics.get("safety") or {})
    parts = [f"Eredmeny: {status}. Lezarasi minosites: {closure}."]
    parts.append(
        "Bizonyitek: {runs} futas, {samples} minta, {moving} mozgasi minta.".format(
            runs=coverage.get("run_count", 0),
            samples=coverage.get("total_samples", 0),
            moving=coverage.get("moving_samples", 0),
        )
    )
    if behavior.get("total_progress_m") is not None:
        parts.append(f"Osszes mért haladas: {float(behavior.get('total_progress_m') or 0.0):.2f} m.")
    if safety.get("min_clearance_m") is not None:
        parts.append(f"Minimum clearance: {float(safety.get('min_clearance_m')):.2f} m.")
    if failed:
        parts.append("Biztosan hibas kapuk: " + ", ".join(failed) + ".")
    if inconclusive:
        parts.append("Hianyzo bizonyitek: " + ", ".join(inconclusive) + ".")
    return " ".join(parts)


def _incident_payload(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    relevant_names = list(result.get("failed_gates") or []) + list(result.get("inconclusive_gates") or [])
    relevant_samples: List[Dict[str, Any]] = []
    seen = set()
    for name in relevant_names:
        gate = dict((result.get("gates") or {}).get(name) or {})
        for sample in list(gate.get("evidence") or []):
            index = sample.get("sample_index")
            if index in seen:
                continue
            seen.add(index)
            relevant_samples.append(sample)
            if len(relevant_samples) >= 40:
                break
    return {
        "schema": "M3_ROOM_CRUISE_MINOSEG_INCIDENT_V1",
        "needed": result.get("status") != "PASS",
        "status": result.get("status"),
        "closure_verdict": result.get("closure_verdict"),
        "failed_gates": list(result.get("failed_gates") or []),
        "inconclusive_gates": list(result.get("inconclusive_gates") or []),
        "diagnostic_cause_groups": dict(result.get("diagnostic_cause_groups") or {}),
        "relevant_samples": relevant_samples,
        "sample_artifact": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
        "plain_summary_hu": result.get("plain_summary_hu"),
        "total_samples": len(samples),
    }


def _base_result_compact(payload: Dict[str, Any]) -> Dict[str, Any]:
    summary = dict(payload.get("summary") or {})
    return {
        "status": str(payload.get("status") or summary.get("status") or ""),
        "success": bool(payload.get("success", str(summary.get("status", "")).upper() == "PASS")),
        "summary": summary,
        "artifact_paths": dict(payload.get("artifact_paths") or {}),
    }


def build_summary(result: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "schema": result["schema"],
        "status": result["status"],
        "success": result["success"],
        "closure_verdict": result["closure_verdict"],
        "test_name": result["test_name"],
        "plain_summary_hu": result["plain_summary_hu"],
        "expected_live_motion_hu": result["expected_live_motion_hu"],
        "failed_gates": result["failed_gates"],
        "inconclusive_gates": result["inconclusive_gates"],
        "coverage": result["metrics"]["coverage"],
        "safety": result["metrics"]["safety"],
        "timing": result["metrics"]["timing"],
        "behavior": result["metrics"]["behavior"],
        "base_room_cruise_v2": result["metrics"]["base_room_cruise_v2"],
        "artifact_paths": result["artifact_paths"],
    }


def write_artifacts(result: Dict[str, Any], samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    summary = build_summary(result)
    _write_jsonl(SAMPLES_PATH, samples)
    _write_json(RESULT_PATH, result)
    _write_json(SUMMARY_PATH, summary)
    _write_json(INCIDENT_PATH, _incident_payload(result, samples))
    return summary


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    threshold_overrides = _load_thresholds(str(args.thresholds_json or ""))
    all_samples: List[Dict[str, Any]] = []
    base_results: List[Dict[str, Any]] = []
    repeat_count = max(1, int(args.repeat_count))
    for run_index in range(repeat_count):
        if run_index > 0:
            time.sleep(max(0.0, float(args.inter_run_pause_s)))
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
        base = cruise.run(base_args)
        base_results.append(_base_result_compact(base))
        for sample in list(base.get("samples") or []):
            row = dict(sample)
            row["run_index"] = run_index
            row["base_room_cruise_status"] = str(base.get("status") or "")
            row["base_room_cruise_progress_m"] = _safe_float((base.get("summary") or {}).get("progress_m"), 0.0)
            all_samples.append(row)

    result, samples = analyze_samples(all_samples, threshold_overrides, base_results=base_results)
    result.update(
        {
            "test_name": str(args.test_name),
            "duration_s_per_run": float(args.duration_s),
            "repeat_count": int(repeat_count),
            "poll_s": float(args.poll_s),
            "artifact_paths": {
                "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
                "summary": str(SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                "samples": str(SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                "incident": str(INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                "base_room_cruise_summary": str(cruise.LATEST_SUMMARY.relative_to(PROJECT_ROOT)),
                "base_room_cruise_result": str(cruise.LATEST_RESULT.relative_to(PROJECT_ROOT)),
            },
            "base_room_cruise_runs": base_results,
        }
    )
    write_artifacts(result, samples)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M3 live Room Cruise v2 behavior and movement-quality validation.")
    parser.add_argument("--test-name", default="M3_room_cruise_minoseg")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--repeat-count", type=int, default=2)
    parser.add_argument("--inter-run-pause-s", type=float, default=12.0)
    parser.add_argument("--poll-s", type=float, default=0.12)
    parser.add_argument("--v-max-mps", type=float, default=0.30)
    parser.add_argument("--omega-max-rad-s", type=float, default=0.60)
    parser.add_argument("--base-min-progress-m", type=float, default=0.45)
    parser.add_argument("--min-front-m", type=float, default=0.27)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--thresholds-json", default="")
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = run(args)
    output = result if not args.compact else {
        "status": result["status"],
        "closure_verdict": result["closure_verdict"],
        "plain_summary_hu": result["plain_summary_hu"],
        "failed_gates": result["failed_gates"],
        "inconclusive_gates": result["inconclusive_gates"],
        "artifact_paths": result["artifact_paths"],
    }
    print(json.dumps(_json_safe(output), ensure_ascii=False))
    return 0 if result.get("status") == "PASS" else (2 if result.get("status") == "INCONCLUSIVE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
