#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Iterative live turning validator/tuner for AIR2B / R2B4.

Goals:
- validate turning behavior on the normal runtime path (follow_arc primitive)
- enforce hard constraints in normal mode:
  - no negative track references
  - no in-place (opposite-sign tracks)
- run short, small-space-safe cycles with measurable metrics
- support explicit operator reposition as a controlled (non-failure) interrupt
- produce compact artifacts for agent/human triage
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.agent_motion_probe import _run_preflight as _strict_preflight  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_POLL_S,
    DEFAULT_TOKEN,
    STATUS_PATH,
    _extract_truth_basis,
    _append_jsonl,
    _get_pose,
    _normalize_angle_deg,
    _read_json,
    _safe_float,
    _safe_stop_best_effort,
    _sample_forward_clearance,
    _send_command_checked,
    _status_version,
    _summarize_lidar_observation_rows,
    _wait_for_status,
    _wait_until_stopped,
)

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_RESULT_PATH = AGENT_TESTS_DIR / "latest_turning_iterative_result.json"
LATEST_SUMMARY_PATH = AGENT_TESTS_DIR / "latest_turning_iterative_summary.json"
HISTORY_PATH = AGENT_TESTS_DIR / "turning_iterative_history.jsonl"

MOTION_EXEC_TERMINAL = {"succeeded", "blocked", "cancelled", "failed"}


@dataclass(frozen=True)
class TurnCase:
    name: str
    angle_deg: float
    radius_m: float
    speed_mps: float
    max_duration_s: float
    required_clearance_m: float


@dataclass
class TuneProfile:
    speed_scale: float = 1.0
    radius_scale: float = 1.0


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _ts_tag_utc() -> str:
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve()))
    except Exception:
        return str(path)


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _as_dict(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


def _pose_distance_m(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(float(b.get("x", 0.0)) - float(a.get("x", 0.0)), float(b.get("y", 0.0)) - float(a.get("y", 0.0)))


def _extract_motion_task(status: Dict[str, Any]) -> Dict[str, Any]:
    task = _as_dict((status or {}).get("motion_task"))
    details_raw = task.get("details")
    details = dict(details_raw) if isinstance(details_raw, dict) else {}
    return {
        "command_type": str(task.get("command_type", "") or ""),
        "execution_state": str(task.get("execution_state", "") or "").strip().lower(),
        "terminal_reason": str(task.get("terminal_reason", "") or ""),
        "updated_ts": float(_safe_float(task.get("updated_ts"), 0.0)),
        "details": details,
    }


def _extract_min_clearance_m(status: Dict[str, Any]) -> Optional[float]:
    vals: List[float] = []
    lidar = _as_dict((status or {}).get("lidar"))
    forward_clearance = _as_dict((status or {}).get("forward_clearance"))
    for value in (
        lidar.get("min_dist"),
        forward_clearance.get("min_dist_min_m"),
        forward_clearance.get("min_dist_median_m"),
    ):
        candidate = float(_safe_float(value, math.nan))
        if math.isfinite(candidate) and candidate > 0.0:
            vals.append(candidate)
    if not vals:
        return None
    return float(min(vals))


def _extract_track_refs(status: Dict[str, Any]) -> Tuple[Optional[float], Optional[float]]:
    left = status.get("motion_ref_v_l", None)
    right = status.get("motion_ref_v_r", None)
    if left is None or right is None:
        motion_controller = _as_dict((status or {}).get("motion_controller"))
        if left is None:
            left = motion_controller.get("v_l_ref", None)
        if right is None:
            right = motion_controller.get("v_r_ref", None)
    left_f = float(_safe_float(left, math.nan))
    right_f = float(_safe_float(right, math.nan))
    out_left = None if not math.isfinite(left_f) else float(left_f)
    out_right = None if not math.isfinite(right_f) else float(right_f)
    return out_left, out_right


def _extract_gyro_z_rad_s(status: Dict[str, Any]) -> Optional[float]:
    imu = _as_dict((status or {}).get("imu"))
    gyro = _as_dict(imu.get("gyro"))
    for key in ("z_rad_s", "gz_rad_s", "z", "omega_z"):
        value = float(_safe_float(gyro.get(key), math.nan))
        if math.isfinite(value):
            # Heuristic: if very large, assume deg/s and convert.
            if abs(value) > 20.0:
                return float(value * (math.pi / 180.0))
            return float(value)
    top_dps = float(_safe_float((status or {}).get("actual_angular_dps"), math.nan))
    if math.isfinite(top_dps):
        return float(top_dps * (math.pi / 180.0))
    pose = dict((status or {}).get("pose") or {})
    pose_omega = float(_safe_float(pose.get("omega_rad_s"), math.nan))
    if math.isfinite(pose_omega):
        return float(pose_omega)
    return None


def _encoder_pose_active(status: Dict[str, Any]) -> bool:
    if "encoder_pose_fusion_active" in (status or {}):
        return bool(status.get("encoder_pose_fusion_active", False))
    pose = _as_dict((status or {}).get("pose"))
    mode = str(pose.get("encoder_trust_mode", "") or "").strip().upper()
    return bool(pose.get("encoder_enabled", False)) or (mode not in ("", "DISABLED"))


def _sample_status_row(
    status: Dict[str, Any],
    *,
    start_pose: Dict[str, float],
    target_heading_deg: float,
    t_rel_s: float,
) -> Dict[str, Any]:
    pose = _get_pose(status)
    heading_change_deg = float(_normalize_angle_deg(float(pose.get("theta_deg", 0.0)) - float(start_pose.get("theta_deg", 0.0))))
    heading_error_deg = float(_normalize_angle_deg(float(target_heading_deg) - float(pose.get("theta_deg", 0.0))))
    left_ref, right_ref = _extract_track_refs(status)
    task = _extract_motion_task(status)
    gyro_z = _extract_gyro_z_rad_s(status)
    min_clearance_m = _extract_min_clearance_m(status)
    lidar = _as_dict((status or {}).get("lidar"))
    truth_surface = _extract_truth_basis(status)
    truth_basis = _as_dict(truth_surface.get("truth_basis"))
    return {
        "t_rel_s": float(t_rel_s),
        "status_version": int(_status_version(status)),
        "state": str((status or {}).get("state", "") or "").strip().upper(),
        "execution_mode": str(truth_surface.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "pose": {
            "x": float(_safe_float(pose.get("x"), 0.0)),
            "y": float(_safe_float(pose.get("y"), 0.0)),
            "theta_deg": float(_safe_float(pose.get("theta_deg"), 0.0)),
            "v": float(_safe_float(pose.get("v"), 0.0)),
        },
        "heading_change_deg": float(heading_change_deg),
        "heading_error_deg": float(heading_error_deg),
        "gyro_z_rad_s": (None if gyro_z is None else float(gyro_z)),
        "motion_ref_v_l": (None if left_ref is None else float(left_ref)),
        "motion_ref_v_r": (None if right_ref is None else float(right_ref)),
        "task": task,
        "blocked_front": bool(lidar.get("blocked_front", False)),
        "blocked_back": bool(lidar.get("blocked_back", False)),
        "min_clearance_m": (None if min_clearance_m is None else float(min_clearance_m)),
        "recovery_mode": bool((status or {}).get("recovery_mobility_mode", False)),
        "encoder_pose_active": bool(_encoder_pose_active(status)),
        "encoder_pose_active_samples": int(_safe_float(truth_surface.get("encoder_pose_active_samples"), 0.0)),
        "motion_actual_ssot": str(
            truth_surface.get(
                "motion_actual_ssot",
                truth_basis.get("motion_actual_ssot", (status or {}).get("motion_actual_ssot", "")),
            )
            or ""
        ).strip().upper(),
        "truth_basis": dict(truth_basis),
        "lidar_odom_applied": bool(truth_surface.get("lidar_odom_applied", False)),
        "lidar_odom_latest_age_s": truth_surface.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": truth_surface.get("lidar_odom_latest_confidence"),
        "lidar_observation": dict(truth_surface.get("lidar_observation") or {}),
        "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(truth_surface.get("turn_primitives") or {}),
    }


def _settle_time_s(
    samples: List[Dict[str, Any]],
    *,
    tolerance_deg: float,
    stable_window_samples: int,
) -> Optional[float]:
    if not samples:
        return None
    n = max(1, int(stable_window_samples))
    tol = max(0.1, float(tolerance_deg))
    errors = [abs(float(_safe_float(row.get("heading_error_deg"), math.inf))) for row in samples]
    for i in range(0, max(0, len(errors) - n + 1)):
        window = errors[i : i + n]
        if all(math.isfinite(e) and e <= tol for e in window):
            return float(_safe_float(samples[i].get("t_rel_s"), 0.0))
    return None


def _oscillation_crossings(
    samples: List[Dict[str, Any]],
    *,
    deadband_deg: float,
) -> int:
    db = max(0.0, float(deadband_deg))
    signs: List[int] = []
    for row in samples:
        err = float(_safe_float(row.get("heading_error_deg"), 0.0))
        if abs(err) <= db:
            continue
        signs.append(1 if err > 0.0 else -1)
    if len(signs) <= 1:
        return 0
    crossings = 0
    prev = signs[0]
    for cur in signs[1:]:
        if cur != prev:
            crossings += 1
            prev = cur
    return int(crossings)


def _forward_only_violations(
    samples: List[Dict[str, Any]],
    *,
    speed_eps: float,
) -> Dict[str, Any]:
    eps = max(0.0, float(speed_eps))
    negative_samples = 0
    in_place_samples = 0
    first_negative: Optional[Dict[str, Any]] = None
    first_in_place: Optional[Dict[str, Any]] = None

    for row in samples:
        if bool(row.get("recovery_mode", False)):
            continue
        left = row.get("motion_ref_v_l")
        right = row.get("motion_ref_v_r")
        if left is None or right is None:
            continue
        lv = float(left)
        rv = float(right)
        if (lv < -eps) or (rv < -eps):
            negative_samples += 1
            if first_negative is None:
                first_negative = {
                    "t_rel_s": float(_safe_float(row.get("t_rel_s"), 0.0)),
                    "left_mps": float(lv),
                    "right_mps": float(rv),
                }
        if lv * rv < -(eps * eps):
            in_place_samples += 1
            if first_in_place is None:
                first_in_place = {
                    "t_rel_s": float(_safe_float(row.get("t_rel_s"), 0.0)),
                    "left_mps": float(lv),
                    "right_mps": float(rv),
                }
    return {
        "negative_track_ref_samples": int(negative_samples),
        "in_place_samples": int(in_place_samples),
        "first_negative_track_ref": first_negative,
        "first_in_place": first_in_place,
        "ok": bool(negative_samples == 0 and in_place_samples == 0),
    }


def _evaluate_case(
    case: TurnCase,
    *,
    start_pose: Dict[str, float],
    end_pose: Dict[str, float],
    target_heading_deg: float,
    motion_samples: List[Dict[str, Any]],
    settle_samples: List[Dict[str, Any]],
    stopped_pose: Dict[str, float],
    terminal_task: Dict[str, Any],
    timed_out: bool,
    stale_stream: bool,
    thresholds: Dict[str, float],
) -> Dict[str, Any]:
    expected_heading_delta_deg = float(case.angle_deg)
    all_samples = list(motion_samples) + list(settle_samples)
    heading_change_deg = float(_normalize_angle_deg(float(end_pose.get("theta_deg", 0.0)) - float(start_pose.get("theta_deg", 0.0))))
    heading_error_deg = float(_normalize_angle_deg(float(end_pose.get("theta_deg", 0.0)) - float(target_heading_deg)))
    target_abs_deg = abs(float(expected_heading_delta_deg))
    progress_sign = 1.0 if float(expected_heading_delta_deg) >= 0.0 else -1.0
    expected_direction = 1 if float(expected_heading_delta_deg) >= 0.0 else -1
    observed_direction = 0
    if abs(float(heading_change_deg)) >= 1.0:
        observed_direction = 1 if float(heading_change_deg) > 0.0 else -1
    wrap_ambiguous = bool(target_abs_deg < 170.0 and abs(float(heading_change_deg)) > 170.0)
    direction_match = bool(wrap_ambiguous or observed_direction == 0 or observed_direction == expected_direction)
    progress_values = [progress_sign * float(_safe_float(s.get("heading_change_deg"), 0.0)) for s in all_samples]
    max_progress_deg = float(max(progress_values)) if progress_values else max(0.0, progress_sign * heading_change_deg)
    overshoot_deg = max(0.0, float(max_progress_deg) - float(target_abs_deg))

    settle_time = _settle_time_s(
        all_samples,
        tolerance_deg=float(thresholds["settle_tolerance_deg"]),
        stable_window_samples=int(thresholds["settle_stable_samples"]),
    )
    osc_crossings = _oscillation_crossings(
        motion_samples,
        deadband_deg=float(thresholds["oscillation_deadband_deg"]),
    )
    oscillation = bool(osc_crossings > int(thresholds["oscillation_crossings_max"]))

    settle_end_pose = _as_dict((all_samples[-1].get("pose") if all_samples else end_pose) or end_pose)
    post_stop_heading_drift_deg = abs(
        float(
            _normalize_angle_deg(
                float(settle_end_pose.get("theta_deg", 0.0)) - float(stopped_pose.get("theta_deg", 0.0))
            )
        )
    )
    post_stop_distance_drift_m = _pose_distance_m(stopped_pose, settle_end_pose)

    gyro_abs_rad_s = [
        abs(float(_safe_float(row.get("gyro_z_rad_s"), math.nan)))
        for row in motion_samples
        if math.isfinite(float(_safe_float(row.get("gyro_z_rad_s"), math.nan)))
    ]
    max_gyro_dps = float(max(gyro_abs_rad_s) * (180.0 / math.pi)) if gyro_abs_rad_s else 0.0
    avg_gyro_dps = (
        float(sum(gyro_abs_rad_s) / len(gyro_abs_rad_s) * (180.0 / math.pi))
        if gyro_abs_rad_s
        else 0.0
    )

    min_clearance_m = None
    clearance_vals = [
        float(_safe_float(row.get("min_clearance_m"), math.nan))
        for row in all_samples
        if math.isfinite(float(_safe_float(row.get("min_clearance_m"), math.nan)))
    ]
    if clearance_vals:
        min_clearance_m = float(min(clearance_vals))
    blocked_any = any(bool(row.get("blocked_front", False) or row.get("blocked_back", False)) for row in all_samples)

    forward_only = _forward_only_violations(
        all_samples,
        speed_eps=float(thresholds["track_speed_eps_mps"]),
    )
    encoder_pose_active_samples = int(sum(1 for row in all_samples if bool(row.get("encoder_pose_active", False))))
    execution_modes_seen = [
        str(row.get("execution_mode", "") or "").strip().upper()
        for row in all_samples
        if str(row.get("execution_mode", "") or "").strip()
    ]
    motion_actual_ssot_values = [
        str(row.get("motion_actual_ssot", "") or "").strip().upper()
        for row in all_samples
        if str(row.get("motion_actual_ssot", "") or "").strip()
    ]
    motion_actual_ssot_consistent = bool(motion_actual_ssot_values) and len(set(motion_actual_ssot_values)) == 1
    motion_actual_ssot = (
        motion_actual_ssot_values[0]
        if motion_actual_ssot_consistent
        else ("MIXED" if motion_actual_ssot_values else "")
    )
    lidar_observation_summary = _summarize_lidar_observation_rows(all_samples)
    lidar_applied_samples = int(lidar_observation_summary.get("applied_samples", 0))
    lidar_age_values = list(lidar_observation_summary.get("age_values") or [])
    lidar_conf_values = list(lidar_observation_summary.get("confidence_values") or [])
    lidar_contract_errors = list(
        lidar_observation_summary.get("observation_contract_errors") or []
    )
    p_req_values: List[str] = []
    p_lim_values: List[str] = []
    p_exe_values: List[str] = []
    p_act_values: List[str] = []
    primitive_req_lim_total = 0
    primitive_req_lim_ok = 0
    primitive_lim_exe_total = 0
    primitive_lim_exe_ok = 0
    primitive_exe_act_total = 0
    primitive_exe_act_ok = 0
    primitive_req_exe_total = 0
    primitive_req_exe_ok = 0
    for row in all_samples:
        p_req = str(row.get("turn_primitive_requested", "") or "").strip().upper()
        p_lim = str(row.get("turn_primitive_limited", "") or "").strip().upper()
        p_exe = str(row.get("turn_primitive_executed", "") or "").strip().upper()
        p_act = str(row.get("turn_primitive_actual", "") or "").strip().upper()
        if p_req:
            p_req_values.append(p_req)
        if p_lim:
            p_lim_values.append(p_lim)
        if p_exe:
            p_exe_values.append(p_exe)
        if p_act:
            p_act_values.append(p_act)
        if p_req and p_lim:
            primitive_req_lim_total += 1
            if p_req == p_lim:
                primitive_req_lim_ok += 1
        if p_lim and p_exe:
            primitive_lim_exe_total += 1
            if p_lim == p_exe:
                primitive_lim_exe_ok += 1
        if p_exe and p_act:
            primitive_exe_act_total += 1
            if p_exe == p_act:
                primitive_exe_act_ok += 1
        if p_req and p_exe:
            primitive_req_exe_total += 1
            if p_req == p_exe:
                primitive_req_exe_ok += 1

    truth_basis = {
        "sample_count": int(len(all_samples)),
        "execution_modes_seen": list(dict.fromkeys(execution_modes_seen)),
        "motion_actual_ssot": str(motion_actual_ssot),
        "motion_actual_ssot_consistent": bool(motion_actual_ssot_consistent),
        "encoder_pose_active_samples": int(encoder_pose_active_samples),
        "lidar_odom_applied_samples": int(lidar_applied_samples),
        "lidar_odom_applied_status_samples": int(
            lidar_observation_summary.get("applied_status_samples", 0)
        ),
        "lidar_odom_unique_measurement_samples": int(
            lidar_observation_summary.get("unique_lidar_odometry_measurements", 0)
        ),
        "lidar_odom_applied_measurement_ids": list(
            lidar_observation_summary.get("applied_measurement_ids") or []
        ),
        "lidar_odom_applied_missing_measurement_id_samples": int(
            lidar_observation_summary.get("applied_missing_measurement_id_samples", 0)
        ),
        "lidar_observation_contract_errors": lidar_contract_errors,
        "lidar_odom_latest_age_s_median": (
            None
            if not lidar_age_values
            else round(float(sorted(lidar_age_values)[len(lidar_age_values) // 2]), 4)
        ),
        "lidar_odom_confidence_median": (
            None
            if not lidar_conf_values
            else round(float(sorted(lidar_conf_values)[len(lidar_conf_values) // 2]), 4)
        ),
        "turn_primitive_requested_vs_limited_match_ratio": (
            None
            if primitive_req_lim_total <= 0
            else round(float(primitive_req_lim_ok / float(primitive_req_lim_total)), 4)
        ),
        "turn_primitive_limited_vs_executed_match_ratio": (
            None
            if primitive_lim_exe_total <= 0
            else round(float(primitive_lim_exe_ok / float(primitive_lim_exe_total)), 4)
        ),
        "turn_primitive_requested_vs_executed_match_ratio": (
            None
            if primitive_req_exe_total <= 0
            else round(float(primitive_req_exe_ok / float(primitive_req_exe_total)), 4)
        ),
        "turn_primitive_executed_vs_actual_match_ratio": (
            None
            if primitive_exe_act_total <= 0
            else round(float(primitive_exe_act_ok / float(primitive_exe_act_total)), 4)
        ),
        "turn_primitives_seen": {
            "requested": list(dict.fromkeys(p_req_values)),
            "limited": list(dict.fromkeys(p_lim_values)),
            "executed": list(dict.fromkeys(p_exe_values)),
            "actual": list(dict.fromkeys(p_act_values)),
        },
    }

    allowed_heading_error = (
        float(thresholds["heading_error_deg_max_180"])
        if target_abs_deg >= 170.0
        else float(thresholds["heading_error_deg_max"])
    )

    fail_reasons: List[str] = []
    if int(lidar_observation_summary.get("applied_missing_measurement_id_samples", 0)) > 0:
        fail_reasons.append("lidar_applied_measurement_id_missing")
    if lidar_contract_errors:
        fail_reasons.append("lidar_observation_contract_violation")
    terminal_state = str((terminal_task or {}).get("execution_state", "") or "").strip().lower()
    terminal_reason = str((terminal_task or {}).get("terminal_reason", "") or "").strip()
    if timed_out:
        fail_reasons.append("motion_timeout")
    if stale_stream:
        fail_reasons.append("status_stream_stale")
    if terminal_state != "succeeded":
        fail_reasons.append(f"terminal_state_{terminal_state or 'unknown'}")
    if not direction_match:
        fail_reasons.append(
            f"direction_mismatch_expected_{expected_direction:+d}_observed_{observed_direction:+d}"
        )
    if abs(float(heading_error_deg)) > float(allowed_heading_error):
        fail_reasons.append(f"heading_error_{heading_error_deg:.2f}deg")
    if float(overshoot_deg) > float(thresholds["overshoot_deg_max"]):
        fail_reasons.append(f"overshoot_{overshoot_deg:.2f}deg")
    if settle_time is None:
        fail_reasons.append("no_settle_window")
    elif float(settle_time) > float(thresholds["settle_time_s_max"]):
        fail_reasons.append(f"settle_time_{float(settle_time):.2f}s")
    if oscillation:
        fail_reasons.append(f"oscillation_crossings_{int(osc_crossings)}")
    if float(post_stop_heading_drift_deg) > float(thresholds["post_stop_heading_drift_deg_max"]):
        fail_reasons.append(f"post_stop_heading_drift_{post_stop_heading_drift_deg:.2f}deg")
    if float(post_stop_distance_drift_m) > float(thresholds["post_stop_distance_drift_m_max"]):
        fail_reasons.append(f"post_stop_distance_drift_{post_stop_distance_drift_m:.3f}m")
    if min_clearance_m is not None and float(min_clearance_m) < float(thresholds["min_clearance_m"]):
        fail_reasons.append(f"clearance_low_{float(min_clearance_m):.3f}m")
    if bool(thresholds["fail_on_blocked"]) and blocked_any:
        fail_reasons.append("blocked_state_detected")
    if not bool(forward_only.get("ok", False)):
        fail_reasons.append("forward_only_constraint_violated")
    if execution_modes_seen and any(mode != "ARC_EXEC" for mode in execution_modes_seen):
        fail_reasons.append("execution_mode_not_arc_exec")
    if motion_actual_ssot not in ("", "EKF_POSE_ODOMETRY_SSOT"):
        fail_reasons.append(f"motion_actual_ssot_{motion_actual_ssot.lower()}")
    if motion_actual_ssot and not motion_actual_ssot_consistent:
        fail_reasons.append("motion_actual_ssot_inconsistent")
    req_lim_ratio = truth_basis.get("turn_primitive_requested_vs_limited_match_ratio")
    if isinstance(req_lim_ratio, (int, float)) and float(req_lim_ratio) < 0.999:
        fail_reasons.append("requested_limited_primitive_mismatch")
    lim_exe_ratio = truth_basis.get("turn_primitive_limited_vs_executed_match_ratio")
    if isinstance(lim_exe_ratio, (int, float)) and float(lim_exe_ratio) < 0.999:
        fail_reasons.append("limited_executed_primitive_mismatch")
    req_exe_ratio = truth_basis.get("turn_primitive_requested_vs_executed_match_ratio")
    if isinstance(req_exe_ratio, (int, float)) and float(req_exe_ratio) < 0.999:
        fail_reasons.append("requested_executed_primitive_mismatch")
    exe_act_ratio = truth_basis.get("turn_primitive_executed_vs_actual_match_ratio")
    if isinstance(exe_act_ratio, (int, float)) and float(exe_act_ratio) < 0.999:
        fail_reasons.append("executed_actual_primitive_mismatch")

    stop_error_deg = abs(
        float(
            _normalize_angle_deg(
                float(stopped_pose.get("theta_deg", 0.0)) - float(target_heading_deg)
            )
        )
    )
    path_drift_m = float(post_stop_distance_drift_m)

    success = len(fail_reasons) == 0

    def _single_or_mixed(values: List[str]) -> str:
        unique = list(dict.fromkeys(str(v or "").strip().upper() for v in values if str(v or "").strip()))
        if not unique:
            return "UNKNOWN"
        if len(unique) == 1:
            return str(unique[0])
        return "MIXED"

    execution_mode = _single_or_mixed(execution_modes_seen)
    turn_primitives_seen = _as_dict(truth_basis.get("turn_primitives_seen"))
    return {
        "success": bool(success),
        "fail_reasons": list(fail_reasons),
        "target_heading_deg": round(float(target_heading_deg), 3),
        "expected_heading_delta_deg": round(float(expected_heading_delta_deg), 3),
        "final_heading_deg": round(float(end_pose.get("theta_deg", 0.0)), 3),
        "heading_error_deg": round(float(heading_error_deg), 3),
        "heading_change_deg": round(float(heading_change_deg), 3),
        "expected_direction": int(expected_direction),
        "observed_direction": int(observed_direction),
        "direction_match": bool(direction_match),
        "direction_wrap_ambiguous": bool(wrap_ambiguous),
        "overshoot_deg": round(float(overshoot_deg), 3),
        "stop_error_deg": round(float(stop_error_deg), 3),
        "path_drift_m": round(float(path_drift_m), 4),
        "settle_time_s": (None if settle_time is None else round(float(settle_time), 3)),
        "oscillation": bool(oscillation),
        "oscillation_crossings": int(osc_crossings),
        "post_stop_heading_drift_deg": round(float(post_stop_heading_drift_deg), 3),
        "post_stop_distance_drift_m": round(float(post_stop_distance_drift_m), 4),
        "max_gyro_dps": round(float(max_gyro_dps), 3),
        "avg_gyro_dps": round(float(avg_gyro_dps), 3),
        "min_clearance_m": (None if min_clearance_m is None else round(float(min_clearance_m), 4)),
        "blocked_any": bool(blocked_any),
        "motion_task_lifecycle": [
            str(item)
            for item in dict.fromkeys(
                str((row.get("task") or {}).get("execution_state", "") or "").strip().lower()
                for row in motion_samples
                if str((row.get("task") or {}).get("execution_state", "") or "").strip()
            )
        ],
        "motion_task_terminal_state": str(terminal_state),
        "motion_task_terminal_reason": str(terminal_reason),
        "forward_only_constraints": dict(forward_only),
        "encoder_pose_active_samples": int(encoder_pose_active_samples),
        "execution_mode": str(execution_mode),
        "motion_actual_ssot": str(motion_actual_ssot),
        "truth_basis": dict(truth_basis),
        "lidar_odom_latest_age_s": truth_basis.get("lidar_odom_latest_age_s_median"),
        "lidar_odom_latest_confidence": truth_basis.get("lidar_odom_confidence_median"),
        "turn_primitive_requested": _single_or_mixed(list(turn_primitives_seen.get("requested") or [])),
        "turn_primitive_limited": _single_or_mixed(list(turn_primitives_seen.get("limited") or [])),
        "turn_primitive_executed": _single_or_mixed(list(turn_primitives_seen.get("executed") or [])),
        "turn_primitive_actual": _single_or_mixed(list(turn_primitives_seen.get("actual") or [])),
        "turn_primitives": {
            "requested": list(turn_primitives_seen.get("requested") or []),
            "limited": list(turn_primitives_seen.get("limited") or []),
            "executed": list(turn_primitives_seen.get("executed") or []),
            "actual": list(turn_primitives_seen.get("actual") or []),
        },
    }


def _wait_motion_terminal(
    *,
    command_type: str,
    sent_ts_wall: float,
    timeout_s: float,
    start_pose: Dict[str, float],
    target_heading_deg: float,
    poll_s: float,
    stale_status_s: float,
) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    last_status: Dict[str, Any] = {}
    last_version = -1
    last_change = time.monotonic()
    start_mono = time.monotonic()
    samples: List[Dict[str, Any]] = []
    terminal_task: Dict[str, Any] = {}
    timed_out = False
    stale_stream = False

    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            last_status = st
            version = _status_version(st)
            if version != last_version:
                last_version = version
                last_change = time.monotonic()

            row = _sample_status_row(
                st,
                start_pose=start_pose,
                target_heading_deg=target_heading_deg,
                t_rel_s=max(0.0, time.monotonic() - start_mono),
            )
            samples.append(row)

            task = _extract_motion_task(st)
            if (
                str(task.get("command_type", "") or "") == str(command_type)
                and float(_safe_float(task.get("updated_ts"), 0.0)) >= float(sent_ts_wall)
                and str(task.get("execution_state", "") or "").strip().lower() in MOTION_EXEC_TERMINAL
            ):
                terminal_task = _as_dict(task)
                break
        if time.monotonic() - last_change > float(stale_status_s):
            stale_stream = True
            break
        time.sleep(max(0.01, float(poll_s)))
    else:
        timed_out = True

    return {
        "samples": samples,
        "last_status": last_status,
        "terminal_task": terminal_task,
        "timed_out": bool(timed_out),
        "stale_stream": bool(stale_stream),
    }


def _collect_settle_samples(
    *,
    start_pose: Dict[str, float],
    target_heading_deg: float,
    duration_s: float,
    poll_s: float,
) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    start_mono = time.monotonic()
    deadline = start_mono + max(0.0, float(duration_s))
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            row = _sample_status_row(
                st,
                start_pose=start_pose,
                target_heading_deg=target_heading_deg,
                t_rel_s=max(0.0, time.monotonic() - start_mono),
            )
            samples.append(row)
        time.sleep(max(0.01, float(poll_s)))
    return samples


def _clear_active_motion_best_effort(token: str, *, timeout_s: float = 4.0) -> None:
    try:
        _send_command_checked(
            "cancel_motion",
            token=str(token),
            timeout_s=4.0,
            reason="turning_validator_prepare",
            motion_source="STATE",
        )
    except Exception:
        pass
    try:
        _send_command_checked(
            "stop",
            token=str(token),
            timeout_s=4.0,
            reason="turning_validator_prepare",
            motion_source="STATE",
        )
    except Exception:
        pass
    try:
        _wait_until_stopped(timeout_s=float(timeout_s))
    except Exception:
        pass


def _run_turn_case(
    case: TurnCase,
    *,
    token: str,
    tune: TuneProfile,
    thresholds: Dict[str, float],
    poll_s: float,
    stop_timeout_s: float,
    settle_time_s: float,
    stale_status_s: float,
) -> Dict[str, Any]:
    _clear_active_motion_best_effort(str(token), timeout_s=float(stop_timeout_s))
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    # follow_arc input uses the same signed convention as pose heading delta.
    expected_heading_delta_deg = float(case.angle_deg)
    target_heading_deg = float(
        _normalize_angle_deg(float(start_pose.get("theta_deg", 0.0)) + float(expected_heading_delta_deg))
    )
    speed_mps = max(0.03, float(case.speed_mps) * float(tune.speed_scale))
    radius_m = max(0.08, float(case.radius_m) * float(tune.radius_scale))
    arc_angle_rad = math.radians(float(case.angle_deg))
    expected_runtime_s = abs(float(radius_m) * float(arc_angle_rad)) / max(0.03, float(speed_mps))
    max_duration_s = max(float(case.max_duration_s), expected_runtime_s * 2.0 + 1.2)

    start_cmd = _send_command_checked(
        "follow_arc",
        token=str(token),
        timeout_s=4.0,
        radius_m=float(radius_m),
        arc_angle_rad=float(arc_angle_rad),
        speed_mps=float(speed_mps),
        max_duration_s=float(max_duration_s),
        motion_source="STATE",
    )

    wait_out = _wait_motion_terminal(
        command_type="follow_arc",
        sent_ts_wall=float(start_cmd.get("sent_ts_wall", time.time())),
        timeout_s=float(max_duration_s) + 2.0,
        start_pose=start_pose,
        target_heading_deg=target_heading_deg,
        poll_s=float(poll_s),
        stale_status_s=float(stale_status_s),
    )

    stop_cmd = _send_command_checked(
        "stop",
        token=str(token),
        timeout_s=4.0,
        reason=f"{case.name}_stop",
        motion_source="STATE",
    )
    stopped_status = _wait_until_stopped(timeout_s=float(stop_timeout_s))
    stopped_pose = _get_pose(stopped_status)

    settle_samples = _collect_settle_samples(
        start_pose=start_pose,
        target_heading_deg=target_heading_deg,
        duration_s=float(settle_time_s),
        poll_s=float(poll_s),
    )

    end_status = _as_dict((settle_samples[-1].get("pose") if settle_samples else {}) or {})
    end_pose = _get_pose(_read_json(STATUS_PATH))
    if not end_pose:
        end_pose = _as_dict(stopped_pose)

    evaluation = _evaluate_case(
        case=case,
        start_pose=start_pose,
        end_pose=end_pose,
        target_heading_deg=target_heading_deg,
        motion_samples=list(wait_out.get("samples") or []),
        settle_samples=list(settle_samples),
        stopped_pose=stopped_pose,
        terminal_task=_as_dict(wait_out.get("terminal_task")),
        timed_out=bool(wait_out.get("timed_out", False)),
        stale_stream=bool(wait_out.get("stale_stream", False)),
        thresholds=thresholds,
    )

    return {
        "case_name": str(case.name),
        "command": {
            "type": "follow_arc",
            "radius_m": float(radius_m),
            "angle_deg": float(case.angle_deg),
            "speed_mps": float(speed_mps),
            "max_duration_s": float(max_duration_s),
        },
        "start_pose": start_pose,
        "stopped_pose": stopped_pose,
        "end_pose": end_pose,
        "raw": {
            "start_command": start_cmd,
            "stop_command": stop_cmd,
            "motion_samples": list(wait_out.get("samples") or []),
            "settle_samples": list(settle_samples),
            "terminal_task": _as_dict(wait_out.get("terminal_task")),
            "timed_out": bool(wait_out.get("timed_out", False)),
            "stale_stream": bool(wait_out.get("stale_stream", False)),
            "last_status_pose": _as_dict(end_status),
        },
        "metrics": evaluation,
        "execution_mode": str(evaluation.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "motion_actual_ssot": str(evaluation.get("motion_actual_ssot", "") or ""),
        "truth_basis": dict(evaluation.get("truth_basis") or {}),
        "lidar_odom_latest_age_s": evaluation.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": evaluation.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(_safe_float(evaluation.get("encoder_pose_active_samples"), 0.0)),
        "turn_primitive_requested": str(evaluation.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(evaluation.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(evaluation.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(evaluation.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(evaluation.get("turn_primitives") or {}),
    }


def _asymmetry_report(left_case: Dict[str, Any], right_case: Dict[str, Any], *, thresholds: Dict[str, float]) -> Dict[str, Any]:
    left_m = _as_dict((left_case or {}).get("metrics"))
    right_m = _as_dict((right_case or {}).get("metrics"))
    left_head = abs(float(_safe_float(left_m.get("heading_error_deg"), 0.0)))
    right_head = abs(float(_safe_float(right_m.get("heading_error_deg"), 0.0)))
    left_stop = abs(float(_safe_float(left_m.get("stop_error_deg"), 0.0)))
    right_stop = abs(float(_safe_float(right_m.get("stop_error_deg"), 0.0)))
    left_drift = abs(float(_safe_float(left_m.get("path_drift_m"), 0.0)))
    right_drift = abs(float(_safe_float(right_m.get("path_drift_m"), 0.0)))
    left_over = abs(float(_safe_float(left_m.get("overshoot_deg"), 0.0)))
    right_over = abs(float(_safe_float(right_m.get("overshoot_deg"), 0.0)))
    left_gyro = float(_safe_float(left_m.get("max_gyro_dps"), 0.0))
    right_gyro = float(_safe_float(right_m.get("max_gyro_dps"), 0.0))

    heading_delta = abs(left_head - right_head)
    stop_delta = abs(left_stop - right_stop)
    drift_delta = abs(left_drift - right_drift)
    overshoot_delta = abs(left_over - right_over)
    gyro_delta = abs(left_gyro - right_gyro)
    gyro_ratio = gyro_delta / max(1.0, left_gyro, right_gyro)

    fail_reasons: List[str] = []
    if heading_delta > float(thresholds["asymmetry_heading_error_deg_max"]):
        fail_reasons.append(f"asym_heading_error_delta_{heading_delta:.2f}deg")
    if overshoot_delta > float(thresholds["asymmetry_overshoot_deg_max"]):
        fail_reasons.append(f"asym_overshoot_delta_{overshoot_delta:.2f}deg")
    if stop_delta > float(thresholds["asymmetry_stop_error_deg_max"]):
        fail_reasons.append(f"asym_stop_error_delta_{stop_delta:.2f}deg")
    if drift_delta > float(thresholds["asymmetry_path_drift_m_max"]):
        fail_reasons.append(f"asym_path_drift_delta_{drift_delta:.4f}m")
    if gyro_ratio > float(thresholds["asymmetry_omega_ratio_max"]):
        fail_reasons.append(f"asym_omega_ratio_{gyro_ratio:.3f}")

    return {
        "left_case": str((left_case or {}).get("case_name", "")),
        "right_case": str((right_case or {}).get("case_name", "")),
        "yaw_error_deg_left": round(float(left_head), 3),
        "yaw_error_deg_right": round(float(right_head), 3),
        "stop_error_deg_left": round(float(left_stop), 3),
        "stop_error_deg_right": round(float(right_stop), 3),
        "path_drift_m_left": round(float(left_drift), 4),
        "path_drift_m_right": round(float(right_drift), 4),
        "overshoot_deg_left": round(float(left_over), 3),
        "overshoot_deg_right": round(float(right_over), 3),
        "heading_error_abs_delta_deg": round(float(heading_delta), 3),
        "stop_error_abs_delta_deg": round(float(stop_delta), 3),
        "path_drift_abs_delta_m": round(float(drift_delta), 4),
        "overshoot_abs_delta_deg": round(float(overshoot_delta), 3),
        "max_gyro_abs_delta_dps": round(float(gyro_delta), 3),
        "max_gyro_delta_ratio": round(float(gyro_ratio), 4),
        "success": len(fail_reasons) == 0,
        "fail_reasons": fail_reasons,
    }


def _iteration_summary(
    *,
    iteration_index: int,
    case_results: List[Dict[str, Any]],
    asymmetry: Dict[str, Any],
    tune: TuneProfile,
) -> Dict[str, Any]:
    case_pass = sum(1 for c in case_results if bool(((c.get("metrics") or {}).get("success", False))))
    fail_reasons: List[str] = []
    for c in case_results:
        for reason in list(((c.get("metrics") or {}).get("fail_reasons") or [])):
            fail_reasons.append(f"{c.get('case_name')}:{reason}")
    for reason in list((asymmetry or {}).get("fail_reasons") or []):
        fail_reasons.append(f"left_right_asymmetry:{reason}")
    success = (case_pass == len(case_results)) and bool((asymmetry or {}).get("success", True))
    return {
        "iteration": int(iteration_index),
        "success": bool(success),
        "cases_passed": int(case_pass),
        "cases_total": int(len(case_results)),
        "fail_reasons": fail_reasons,
        "tune_profile": {
            "speed_scale": float(tune.speed_scale),
            "radius_scale": float(tune.radius_scale),
        },
    }


def _suggest_tune_adjustment(
    *,
    case_results: List[Dict[str, Any]],
    current: TuneProfile,
) -> Dict[str, Any]:
    metrics = [_as_dict(c.get("metrics")) for c in case_results]
    hard_violation = any(
        any(
            str(r).startswith("forward_only_constraint_violated")
            for r in list(m.get("fail_reasons") or [])
        )
        for m in metrics
    )
    if hard_violation:
        return {
            "applied": False,
            "reason": "hard_constraint_violation_requires_logic_fix",
            "next_tune_profile": {
                "speed_scale": float(current.speed_scale),
                "radius_scale": float(current.radius_scale),
            },
        }

    overshoots = [float(_safe_float(m.get("overshoot_deg"), 0.0)) for m in metrics]
    settle_times = [
        float(_safe_float(m.get("settle_time_s"), math.nan))
        for m in metrics
        if m.get("settle_time_s") is not None
    ]
    oscillating = any(bool(m.get("oscillation", False)) for m in metrics)
    heading_errors = [abs(float(_safe_float(m.get("heading_error_deg"), 0.0))) for m in metrics]

    if overshoots and max(overshoots) > 8.0:
        next_profile = TuneProfile(
            speed_scale=max(0.65, float(current.speed_scale) * 0.90),
            radius_scale=min(1.25, float(current.radius_scale) * 1.06),
        )
        return {
            "applied": True,
            "reason": "reduce_overshoot",
            "change": {"speed_scale_factor": 0.90, "radius_scale_factor": 1.06},
            "next_tune_profile": {
                "speed_scale": float(next_profile.speed_scale),
                "radius_scale": float(next_profile.radius_scale),
            },
        }

    if oscillating or (settle_times and max(settle_times) > 2.8):
        next_profile = TuneProfile(
            speed_scale=max(0.65, float(current.speed_scale) * 0.92),
            radius_scale=min(1.22, float(current.radius_scale) * 1.04),
        )
        return {
            "applied": True,
            "reason": "reduce_oscillation_or_settle_time",
            "change": {"speed_scale_factor": 0.92, "radius_scale_factor": 1.04},
            "next_tune_profile": {
                "speed_scale": float(next_profile.speed_scale),
                "radius_scale": float(next_profile.radius_scale),
            },
        }

    if heading_errors and statistics.mean(heading_errors) > 6.0:
        next_profile = TuneProfile(
            speed_scale=min(1.15, float(current.speed_scale) * 1.03),
            radius_scale=max(0.90, float(current.radius_scale) * 0.97),
        )
        return {
            "applied": True,
            "reason": "improve_heading_completion",
            "change": {"speed_scale_factor": 1.03, "radius_scale_factor": 0.97},
            "next_tune_profile": {
                "speed_scale": float(next_profile.speed_scale),
                "radius_scale": float(next_profile.radius_scale),
            },
        }

    return {
        "applied": False,
        "reason": "no_safe_small_adjustment_identified",
        "next_tune_profile": {
            "speed_scale": float(current.speed_scale),
            "radius_scale": float(current.radius_scale),
        },
    }


def _operator_reposition_gate(
    *,
    mode: str,
    token: str,
    case_name: str,
    reason: str,
    reset_pos_after_reposition: bool,
) -> Dict[str, Any]:
    event = {
        "timestamp": _now_iso_utc(),
        "case_name": str(case_name),
        "mode": str(mode),
        "reason": str(reason),
        "required": True,
        "completed": False,
        "controlled_interrupt": False,
    }
    print(
        f"OPERATOR_REPOSITION_REQUIRED|case={case_name}|reason={reason}|mode={mode}",
        flush=True,
    )
    if mode == "none":
        return event
    if mode == "prompt":
        _safe_stop_best_effort(str(token))
        print(
            "Please reposition robot to safe start pose, then press ENTER.",
            flush=True,
        )
        try:
            input()
            if bool(reset_pos_after_reposition):
                _send_command_checked("reset_pos", token=str(token), timeout_s=4.0)
            event["completed"] = True
        except Exception as exc:
            event["reason"] = f"{event['reason']};prompt_error={exc}"
        return event
    event["controlled_interrupt"] = True
    return event


def _build_thresholds(args: argparse.Namespace) -> Dict[str, float]:
    return {
        "heading_error_deg_max": float(args.heading_error_deg_max),
        "heading_error_deg_max_180": float(args.heading_error_deg_max_180),
        "overshoot_deg_max": float(args.overshoot_deg_max),
        "settle_time_s_max": float(args.settle_time_s_max),
        "settle_tolerance_deg": float(args.settle_tolerance_deg),
        "settle_stable_samples": float(args.settle_stable_samples),
        "oscillation_deadband_deg": float(args.oscillation_deadband_deg),
        "oscillation_crossings_max": float(args.oscillation_crossings_max),
        "post_stop_heading_drift_deg_max": float(args.post_stop_heading_drift_deg_max),
        "post_stop_distance_drift_m_max": float(args.post_stop_distance_drift_m_max),
        "track_speed_eps_mps": float(args.track_speed_eps_mps),
        "min_clearance_m": float(args.min_clearance_m),
        "fail_on_blocked": 1.0 if bool(args.fail_on_blocked) else 0.0,
        "asymmetry_heading_error_deg_max": float(args.asymmetry_heading_error_deg_max),
        "asymmetry_overshoot_deg_max": float(args.asymmetry_overshoot_deg_max),
        "asymmetry_stop_error_deg_max": float(args.asymmetry_stop_error_deg_max),
        "asymmetry_path_drift_m_max": float(args.asymmetry_path_drift_m_max),
        "asymmetry_omega_ratio_max": float(args.asymmetry_omega_ratio_max),
    }


def _default_cases(args: argparse.Namespace) -> List[TurnCase]:
    return [
        TurnCase(
            name="turn_left_90",
            angle_deg=90.0,
            radius_m=float(args.turn_radius_m),
            speed_mps=float(args.turn_speed_mps),
            max_duration_s=float(args.turn_max_duration_s),
            required_clearance_m=float(args.required_clearance_m),
        ),
        TurnCase(
            name="turn_right_90",
            angle_deg=-90.0,
            radius_m=float(args.turn_radius_m),
            speed_mps=float(args.turn_speed_mps),
            max_duration_s=float(args.turn_max_duration_s),
            required_clearance_m=float(args.required_clearance_m),
        ),
        TurnCase(
            name="turn_180",
            angle_deg=180.0,
            radius_m=float(args.turn180_radius_m),
            speed_mps=float(args.turn180_speed_mps),
            max_duration_s=float(args.turn180_max_duration_s),
            required_clearance_m=float(args.required_clearance_m),
        ),
        TurnCase(
            name="forward_arc_short",
            angle_deg=float(args.short_arc_angle_deg),
            radius_m=float(args.short_arc_radius_m),
            speed_mps=float(args.short_arc_speed_mps),
            max_duration_s=float(args.short_arc_max_duration_s),
            required_clearance_m=float(args.required_clearance_m),
        ),
    ]


def _compact_case_line(case_result: Dict[str, Any]) -> str:
    metrics = _as_dict((case_result or {}).get("metrics"))
    return (
        "CASE|name={name}|result={res}|heading_err={he:.2f}|overshoot={ov:.2f}|settle={st}|"
        "osc={osc}|drift_h={dh:.2f}|drift_p={dp:.3f}|term={term}:{reason}|viol={viol}".format(
            name=case_result.get("case_name", ""),
            res=("PASS" if metrics.get("success") else "FAIL"),
            he=float(_safe_float(metrics.get("heading_error_deg"), 0.0)),
            ov=float(_safe_float(metrics.get("overshoot_deg"), 0.0)),
            st=("n/a" if metrics.get("settle_time_s") is None else f"{float(metrics.get('settle_time_s')):.2f}s"),
            osc=("yes" if metrics.get("oscillation") else "no"),
            dh=float(_safe_float(metrics.get("post_stop_heading_drift_deg"), 0.0)),
            dp=float(_safe_float(metrics.get("post_stop_distance_drift_m"), 0.0)),
            term=str(metrics.get("motion_task_terminal_state", "")),
            reason=str(metrics.get("motion_task_terminal_reason", "")),
            viol=(
                f"neg={int((metrics.get('forward_only_constraints') or {}).get('negative_track_ref_samples', 0))},"
                f"inplace={int((metrics.get('forward_only_constraints') or {}).get('in_place_samples', 0))}"
            ),
        )
    )


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Iterative live turning validator (forward-only normal mode).")
    ap.add_argument("--test-name", default="turning_live_iterative")
    ap.add_argument("--token", default=DEFAULT_TOKEN)
    ap.add_argument("--iterations", type=int, default=2)
    ap.add_argument("--stop-on-pass", action="store_true")
    ap.add_argument("--poll-s", type=float, default=DEFAULT_POLL_S)
    ap.add_argument("--stale-status-s", type=float, default=3.0)
    ap.add_argument("--stop-timeout-s", type=float, default=5.0)
    ap.add_argument("--settle-time-s", type=float, default=1.0)
    ap.add_argument("--required-clearance-m", type=float, default=0.35)
    ap.add_argument("--clearance-sample-s", type=float, default=0.8)
    ap.add_argument("--max-displacement-before-reposition-m", type=float, default=0.85)
    ap.add_argument("--force-reposition-between-cases", action="store_true")
    ap.add_argument("--reposition-mode", choices=("none", "prompt", "control"), default="control")
    ap.add_argument("--reset-pos-after-reposition", action="store_true")

    ap.add_argument("--turn-radius-m", type=float, default=0.14)
    ap.add_argument("--turn-speed-mps", type=float, default=0.09)
    ap.add_argument("--turn-max-duration-s", type=float, default=8.0)

    ap.add_argument("--turn180-radius-m", type=float, default=0.14)
    ap.add_argument("--turn180-speed-mps", type=float, default=0.08)
    ap.add_argument("--turn180-max-duration-s", type=float, default=12.0)

    ap.add_argument("--short-arc-angle-deg", type=float, default=35.0)
    ap.add_argument("--short-arc-radius-m", type=float, default=0.20)
    ap.add_argument("--short-arc-speed-mps", type=float, default=0.10)
    ap.add_argument("--short-arc-max-duration-s", type=float, default=8.0)

    ap.add_argument("--heading-error-deg-max", type=float, default=8.0)
    ap.add_argument("--heading-error-deg-max-180", type=float, default=12.0)
    ap.add_argument("--overshoot-deg-max", type=float, default=10.0)
    ap.add_argument("--settle-time-s-max", type=float, default=3.0)
    ap.add_argument("--settle-tolerance-deg", type=float, default=3.0)
    ap.add_argument("--settle-stable-samples", type=int, default=5)
    ap.add_argument("--oscillation-deadband-deg", type=float, default=1.0)
    ap.add_argument("--oscillation-crossings-max", type=int, default=1)
    ap.add_argument("--post-stop-heading-drift-deg-max", type=float, default=2.5)
    ap.add_argument("--post-stop-distance-drift-m-max", type=float, default=0.03)
    ap.add_argument("--track-speed-eps-mps", type=float, default=0.01)
    ap.add_argument("--min-clearance-m", type=float, default=0.18)
    ap.add_argument("--fail-on-blocked", action="store_true")
    ap.add_argument("--asymmetry-heading-error-deg-max", type=float, default=5.0)
    ap.add_argument("--asymmetry-overshoot-deg-max", type=float, default=4.0)
    ap.add_argument("--asymmetry-stop-error-deg-max", type=float, default=5.0)
    ap.add_argument("--asymmetry-path-drift-m-max", type=float, default=0.03)
    ap.add_argument("--asymmetry-omega-ratio-max", type=float, default=0.45)

    ap.add_argument("--compact", action="store_true")
    return ap


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        payload = {
            "status": "FAIL",
            "success": False,
            "error": str(exc),
            "bootstrap_guard": {
                "loaded": False,
                "required_path": "project_rules/agent_system_prompt.txt",
            },
        }
        print(json.dumps(payload, ensure_ascii=False), flush=True)
        return 40

    args = build_parser().parse_args()
    thresholds = _build_thresholds(args)
    cases = _default_cases(args)
    run_tag = f"{str(args.test_name)}_{_ts_tag_utc()}"
    run_dir = AGENT_TESTS_DIR / run_tag
    run_dir.mkdir(parents=True, exist_ok=True)

    start_iso = _now_iso_utc()
    started_mono = time.monotonic()
    tune = TuneProfile()
    iterations: List[Dict[str, Any]] = []
    operator_events: List[Dict[str, Any]] = []
    final_status = "FAIL"
    final_success = False
    controlled_interrupt = False
    final_reason = "no_iteration_result"
    ready_for_higher_level_motion = False

    try:
        _wait_for_status(timeout_s=5.0)
    except Exception as exc:
        payload = {
            "status": "FAIL",
            "success": False,
            "error": f"status_not_available:{exc}",
        }
        _write_json_atomic(run_dir / "result.json", payload)
        _write_json_atomic(LATEST_RESULT_PATH, payload)
        _write_json_atomic(LATEST_SUMMARY_PATH, payload)
        print(json.dumps(payload, ensure_ascii=False), flush=True)
        return 1

    anchor_pose = _get_pose(_read_json(STATUS_PATH))

    for idx in range(max(1, int(args.iterations))):
        iter_name = idx + 1
        try:
            preflight = _strict_preflight(
                str(args.token),
                stop_timeout_s=float(args.stop_timeout_s),
                required_clearance_m=float(args.required_clearance_m),
            )
        except Exception as exc:
            preflight = {
                "ready": False,
                "can_move_now": False,
                "blocking_issues": [str(exc)],
            }

        if not bool(preflight.get("ready", False)):
            final_status = "FAIL"
            final_success = False
            final_reason = ";".join(list(preflight.get("blocking_issues") or [])) or "preflight_failed"
            iterations.append(
                {
                    "iteration": int(iter_name),
                    "preflight": preflight,
                    "cases": [],
                    "asymmetry": {},
                    "summary": {
                        "iteration": int(iter_name),
                        "success": False,
                        "cases_passed": 0,
                        "cases_total": 0,
                        "fail_reasons": [str(final_reason)],
                        "tune_profile": {
                            "speed_scale": float(tune.speed_scale),
                            "radius_scale": float(tune.radius_scale),
                        },
                    },
                    "tune_action": {
                        "applied": False,
                        "reason": "preflight_failed",
                    },
                }
            )
            break

        if bool(args.compact):
            print(
                f"ITERATION|index={iter_name}|speed_scale={tune.speed_scale:.3f}|radius_scale={tune.radius_scale:.3f}",
                flush=True,
            )

        case_results: List[Dict[str, Any]] = []
        reposition_break = False
        for cidx, case in enumerate(cases):
            current_status = _read_json(STATUS_PATH)
            current_pose = _get_pose(current_status)
            displacement = _pose_distance_m(anchor_pose, current_pose)
            clearance = _sample_forward_clearance(sample_s=float(args.clearance_sample_s), poll_s=float(args.poll_s))
            med_clearance = clearance.get("min_dist_median_m")
            med_clearance_f = float(_safe_float(med_clearance, math.nan))
            low_clearance = math.isfinite(med_clearance_f) and med_clearance_f < float(case.required_clearance_m)
            reposition_needed = False
            reposition_reason = ""
            if bool(args.force_reposition_between_cases) and cidx > 0:
                reposition_needed = True
                reposition_reason = "forced_between_cases"
            elif displacement > float(args.max_displacement_before_reposition_m):
                reposition_needed = True
                reposition_reason = f"displacement_{displacement:.3f}m"
            elif low_clearance:
                reposition_needed = True
                reposition_reason = f"clearance_{med_clearance_f:.3f}m"

            if reposition_needed:
                event = _operator_reposition_gate(
                    mode=str(args.reposition_mode),
                    token=str(args.token),
                    case_name=str(case.name),
                    reason=str(reposition_reason),
                    reset_pos_after_reposition=bool(args.reset_pos_after_reposition),
                )
                operator_events.append(event)
                if bool(event.get("controlled_interrupt", False)):
                    controlled_interrupt = True
                    final_status = "REPOSITION_REQUIRED"
                    final_success = True
                    final_reason = f"operator_reposition_required_before_{case.name}"
                    reposition_break = True
                    break

            try:
                case_result = _run_turn_case(
                    case,
                    token=str(args.token),
                    tune=tune,
                    thresholds=thresholds,
                    poll_s=float(args.poll_s),
                    stop_timeout_s=float(args.stop_timeout_s),
                    settle_time_s=float(args.settle_time_s),
                    stale_status_s=float(args.stale_status_s),
                )
            except Exception as exc:
                _clear_active_motion_best_effort(str(args.token), timeout_s=float(args.stop_timeout_s))
                _safe_stop_best_effort(str(args.token))
                pose_now = _get_pose(_read_json(STATUS_PATH))
                truth_stub = {
                    "sample_count": 0,
                    "execution_modes_seen": [],
                    "motion_actual_ssot": "",
                    "motion_actual_ssot_consistent": False,
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 0,
                    "lidar_odom_latest_age_s_median": None,
                    "lidar_odom_confidence_median": None,
                    "turn_primitive_requested_vs_limited_match_ratio": None,
                    "turn_primitive_limited_vs_executed_match_ratio": None,
                    "turn_primitive_requested_vs_executed_match_ratio": None,
                    "turn_primitive_executed_vs_actual_match_ratio": None,
                    "turn_primitives_seen": {
                        "requested": [],
                        "limited": [],
                        "executed": [],
                        "actual": [],
                    },
                }
                metrics_stub = {
                    "success": False,
                    "fail_reasons": [f"exception:{exc}"],
                    "target_heading_deg": None,
                    "final_heading_deg": None,
                    "heading_error_deg": 0.0,
                    "heading_change_deg": 0.0,
                    "overshoot_deg": 0.0,
                    "stop_error_deg": 0.0,
                    "path_drift_m": 0.0,
                    "settle_time_s": None,
                    "oscillation": False,
                    "oscillation_crossings": 0,
                    "post_stop_heading_drift_deg": 0.0,
                    "post_stop_distance_drift_m": 0.0,
                    "max_gyro_dps": 0.0,
                    "avg_gyro_dps": 0.0,
                    "min_clearance_m": None,
                    "blocked_any": False,
                    "motion_task_lifecycle": [],
                    "motion_task_terminal_state": "",
                    "motion_task_terminal_reason": "",
                    "forward_only_constraints": {
                        "ok": True,
                        "negative_track_ref_samples": 0,
                        "in_place_samples": 0,
                    },
                    "encoder_pose_active_samples": 0,
                    "execution_mode": "UNKNOWN",
                    "motion_actual_ssot": "",
                    "truth_basis": dict(truth_stub),
                    "lidar_odom_latest_age_s": None,
                    "lidar_odom_latest_confidence": None,
                    "turn_primitive_requested": "UNKNOWN",
                    "turn_primitive_limited": "UNKNOWN",
                    "turn_primitive_executed": "UNKNOWN",
                    "turn_primitive_actual": "UNKNOWN",
                    "turn_primitives": {
                        "requested": [],
                        "limited": [],
                        "executed": [],
                        "actual": [],
                    },
                }
                case_result = {
                    "case_name": str(case.name),
                    "command": {},
                    "start_pose": dict(pose_now),
                    "stopped_pose": dict(pose_now),
                    "end_pose": dict(pose_now),
                    "raw": {
                        "start_command": {},
                        "stop_command": {},
                        "motion_samples": [],
                        "settle_samples": [],
                        "terminal_task": {},
                        "timed_out": False,
                        "stale_stream": False,
                        "exception": str(exc),
                    },
                    "metrics": metrics_stub,
                    "execution_mode": "UNKNOWN",
                    "motion_actual_ssot": "",
                    "truth_basis": dict(truth_stub),
                    "lidar_odom_latest_age_s": None,
                    "lidar_odom_latest_confidence": None,
                    "encoder_pose_active_samples": 0,
                    "turn_primitive_requested": "UNKNOWN",
                    "turn_primitive_limited": "UNKNOWN",
                    "turn_primitive_executed": "UNKNOWN",
                    "turn_primitive_actual": "UNKNOWN",
                    "turn_primitives": {
                        "requested": [],
                        "limited": [],
                        "executed": [],
                        "actual": [],
                    },
                }
            case_results.append(case_result)
            if bool(args.compact):
                print(_compact_case_line(case_result), flush=True)

        if reposition_break:
            iterations.append(
                {
                    "iteration": int(iter_name),
                    "preflight": preflight,
                    "cases": case_results,
                    "asymmetry": {},
                    "summary": {
                        "iteration": int(iter_name),
                        "success": False,
                        "cases_passed": int(sum(1 for c in case_results if bool((c.get("metrics") or {}).get("success", False)))),
                        "cases_total": int(len(cases)),
                        "fail_reasons": [str(final_reason)],
                        "tune_profile": {
                            "speed_scale": float(tune.speed_scale),
                            "radius_scale": float(tune.radius_scale),
                        },
                    },
                    "tune_action": {
                        "applied": False,
                        "reason": "controlled_interrupt_reposition_required",
                    },
                }
            )
            break

        left = next((c for c in case_results if str(c.get("case_name")) == "turn_left_90"), None)
        right = next((c for c in case_results if str(c.get("case_name")) == "turn_right_90"), None)
        asymmetry = (
            _asymmetry_report(left, right, thresholds=thresholds)
            if (left is not None and right is not None)
            else {"success": False, "fail_reasons": ["missing_left_or_right_case"]}
        )
        summary = _iteration_summary(
            iteration_index=iter_name,
            case_results=case_results,
            asymmetry=asymmetry,
            tune=tune,
        )
        tune_action = _suggest_tune_adjustment(
            case_results=case_results,
            current=tune,
        )

        next_profile = _as_dict(tune_action.get("next_tune_profile"))
        tune = TuneProfile(
            speed_scale=float(_safe_float(next_profile.get("speed_scale"), tune.speed_scale)),
            radius_scale=float(_safe_float(next_profile.get("radius_scale"), tune.radius_scale)),
        )

        iterations.append(
            {
                "iteration": int(iter_name),
                "preflight": preflight,
                "cases": case_results,
                "asymmetry": asymmetry,
                "summary": summary,
                "tune_action": tune_action,
            }
        )

        if bool(summary.get("success", False)):
            final_status = "PASS"
            final_success = True
            ready_for_higher_level_motion = True
            final_reason = "turning_suite_passed"
            break
        if not bool(tune_action.get("applied", False)):
            final_status = "FAIL"
            final_success = False
            final_reason = str(tune_action.get("reason", "no_more_safe_tuning"))
            break
        final_status = "FAIL"
        final_success = False
        final_reason = "iteration_failed_retrying_with_small_tune"

    if controlled_interrupt:
        ready_for_higher_level_motion = False
    elif final_status != "PASS":
        ready_for_higher_level_motion = False

    ended_iso = _now_iso_utc()
    duration_s = round(time.monotonic() - started_mono, 3)
    last_iteration = iterations[-1] if iterations else {}
    last_summary = _as_dict(last_iteration.get("summary"))
    last_asymmetry = _as_dict(last_iteration.get("asymmetry"))
    truth_anchor: Dict[str, Any] = {}
    for case in reversed(list(last_iteration.get("cases") or [])):
        if isinstance(case, dict) and case.get("truth_basis"):
            truth_anchor = dict(case)
            break
    if not truth_anchor and isinstance(last_iteration, dict):
        truth_anchor = _as_dict((last_iteration.get("cases") or [{}])[-1] if (last_iteration.get("cases") or []) else {})
    payload = {
        "status": str(final_status),
        "success": bool(final_success),
        "classification": (
            "TURN_READY"
            if final_status == "PASS"
            else ("OPERATOR_REPOSITION_REQUIRED" if final_status == "REPOSITION_REQUIRED" else "TURN_NOT_READY")
        ),
        "test_name": str(args.test_name),
        "started_at_utc": start_iso,
        "ended_at_utc": ended_iso,
        "duration_s": float(duration_s),
        "ready_for_higher_level_motion": bool(ready_for_higher_level_motion),
        "final_reason": str(final_reason),
        "iterations_executed": int(len(iterations)),
        "last_iteration_summary": last_summary,
        "last_asymmetry": last_asymmetry,
        "operator_events": operator_events,
        "thresholds": thresholds,
        "execution_mode": str(truth_anchor.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "motion_actual_ssot": str(truth_anchor.get("motion_actual_ssot", "") or ""),
        "truth_basis": dict(truth_anchor.get("truth_basis") or {}),
        "lidar_odom_latest_age_s": truth_anchor.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": truth_anchor.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(_safe_float(truth_anchor.get("encoder_pose_active_samples"), 0.0)),
        "turn_primitive_requested": str(truth_anchor.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(truth_anchor.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(truth_anchor.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(truth_anchor.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(truth_anchor.get("turn_primitives") or {}),
        "iterations": iterations,
        "artifact_paths": {
            "run_dir": _rel(run_dir),
            "result_json": _rel(run_dir / "result.json"),
            "summary_json": _rel(run_dir / "summary.json"),
            "latest_result": _rel(LATEST_RESULT_PATH),
            "latest_summary": _rel(LATEST_SUMMARY_PATH),
            "history": _rel(HISTORY_PATH),
        },
    }
    summary_payload = {
        "status": str(payload.get("status")),
        "success": bool(payload.get("success")),
        "classification": str(payload.get("classification")),
        "test_name": str(payload.get("test_name")),
        "duration_s": float(payload.get("duration_s", 0.0)),
        "ready_for_higher_level_motion": bool(payload.get("ready_for_higher_level_motion", False)),
        "final_reason": str(payload.get("final_reason", "")),
        "iterations_executed": int(payload.get("iterations_executed", 0)),
        "cases_passed": int(_safe_float(last_summary.get("cases_passed"), 0.0)),
        "cases_total": int(_safe_float(last_summary.get("cases_total"), 0.0)),
        "fail_reasons": list(last_summary.get("fail_reasons") or []),
        "asymmetry_success": bool(last_asymmetry.get("success", False)),
        "execution_mode": str(payload.get("execution_mode", "UNKNOWN") or "UNKNOWN"),
        "motion_actual_ssot": str(payload.get("motion_actual_ssot", "") or ""),
        "truth_basis": dict(payload.get("truth_basis") or {}),
        "lidar_odom_latest_age_s": payload.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": payload.get("lidar_odom_latest_confidence"),
        "encoder_pose_active_samples": int(_safe_float(payload.get("encoder_pose_active_samples"), 0.0)),
        "turn_primitive_requested": str(payload.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(payload.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(payload.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(payload.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(payload.get("turn_primitives") or {}),
        "operator_reposition_required": any(bool(e.get("controlled_interrupt", False)) for e in operator_events),
        "artifact_paths": _as_dict(payload.get("artifact_paths")),
    }

    _write_json_atomic(run_dir / "result.json", payload)
    _write_json_atomic(run_dir / "summary.json", summary_payload)
    _write_json_atomic(LATEST_RESULT_PATH, payload)
    _write_json_atomic(LATEST_SUMMARY_PATH, summary_payload)
    _append_jsonl(HISTORY_PATH, payload)

    if bool(args.compact):
        print(
            "TURN_SUITE|status={status}|class={classification}|iters={iters}|cases={passed}/{total}|reason={reason}".format(
                status=summary_payload.get("status"),
                classification=summary_payload.get("classification"),
                iters=summary_payload.get("iterations_executed"),
                passed=summary_payload.get("cases_passed"),
                total=summary_payload.get("cases_total"),
                reason=summary_payload.get("final_reason"),
            ),
            flush=True,
        )
        print(f"Artifact={_rel(run_dir / 'result.json')}", flush=True)
        print(f"Summary={_rel(run_dir / 'summary.json')}", flush=True)

    print(json.dumps(payload, ensure_ascii=False), flush=True)
    if str(final_status) in ("PASS", "REPOSITION_REQUIRED"):
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
