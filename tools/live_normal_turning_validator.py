#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Live validator for normal set_twist turning primitives.

This intentionally exercises the normal TWIST_EXEC path, not follow_arc and not
track-reference pivots. The validator is short, bounded, and artifact-first.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_POLL_S,
    DEFAULT_TOKEN,
    STATUS_PATH,
    _append_command,
    _extract_truth_basis,
    _get_pose,
    _normalize_angle_deg,
    _read_json,
    _safe_float,
    _send_command_checked,
    _summarize_lidar_observation_rows,
    _status_version,
    _wait_for_status,
    _wait_until_stopped,
)

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_RESULT_PATH = AGENT_TESTS_DIR / "latest_normal_turning_result.json"
LATEST_SUMMARY_PATH = AGENT_TESTS_DIR / "latest_normal_turning_summary.json"

PRIMITIVE_GENTLE = "DIFF_ARC_GENTLE"
PRIMITIVE_SHARP = "DIFF_ARC_SHARP"
DIFF_ARC_PRIMITIVES = {PRIMITIVE_GENTLE, PRIMITIVE_SHARP}
BAD_TURN_PRIMITIVES = {"IN_PLACE_ROTATE", "ONE_TRACK_PIVOT"}
DIFF_ARC_ACTIVE_RATIO_MIN = 0.66


@dataclass(frozen=True)
class NormalTurnCase:
    name: str
    v_mps: float
    omega_rad_s: float
    duration_s: float
    expected_primitive: str
    min_yaw_deg: float


DEFAULT_CASES: Dict[str, NormalTurnCase] = {
    "gentle_left": NormalTurnCase("gentle_left", 0.030, 0.065, 0.65, PRIMITIVE_GENTLE, 0.8),
    "gentle_right": NormalTurnCase("gentle_right", 0.030, -0.065, 0.65, PRIMITIVE_GENTLE, 0.8),
    "sharp_left": NormalTurnCase("sharp_left", 0.030, 0.120, 0.65, PRIMITIVE_SHARP, 1.4),
    "sharp_right": NormalTurnCase("sharp_right", 0.030, -0.120, 0.65, PRIMITIVE_SHARP, 1.4),
}


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _ts_tag_utc() -> str:
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve()))
    except Exception:
        return str(path)


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _as_dict(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


def _safe_ratio(ok: int, total: int) -> Optional[float]:
    if int(total) <= 0:
        return None
    return round(float(ok) / float(max(1, int(total))), 4)


def _finite_or_none(value: Any) -> Optional[float]:
    out = _safe_float(value, math.nan)
    return float(out) if math.isfinite(float(out)) else None


def _state(status: Dict[str, Any]) -> str:
    return str((status or {}).get("state", "") or "").strip().upper()


def _motion_command(status: Dict[str, Any]) -> Dict[str, Any]:
    return _as_dict((status or {}).get("motion_command"))


def _track_targets(status: Dict[str, Any]) -> Dict[str, Optional[float]]:
    motion_command = _motion_command(status)
    targets = _as_dict(motion_command.get("track_targets"))
    left = _finite_or_none(targets.get("left_mps"))
    right = _finite_or_none(targets.get("right_mps"))
    if left is None:
        left = _finite_or_none((status or {}).get("motion_ref_v_l"))
    if right is None:
        right = _finite_or_none((status or {}).get("motion_ref_v_r"))
    return {"left_mps": left, "right_mps": right}


def _min_clearance_m(status: Dict[str, Any]) -> Optional[float]:
    lidar = _as_dict((status or {}).get("lidar"))
    vals = []
    for key in ("min_dist", "front_clearance_m"):
        val = _finite_or_none(lidar.get(key))
        if val is not None and val > 0.0:
            vals.append(float(val))
    if not vals:
        return None
    return float(min(vals))


def _sample_row(status: Dict[str, Any], *, start_pose: Dict[str, Any], t_rel_s: float) -> Dict[str, Any]:
    pose = _get_pose(status)
    truth = _extract_truth_basis(status)
    truth_basis = _as_dict(truth.get("truth_basis"))
    motion_command = _motion_command(status)
    targets = _track_targets(status)
    lidar_odom = _as_dict((status or {}).get("lidar_odom_status"))
    return {
        "t_rel_s": round(float(t_rel_s), 4),
        "status_version": int(_status_version(status)),
        "state": _state(status),
        "command_type": str(motion_command.get("command_type", "") or "").strip().lower(),
        "execution_mode": str(truth.get("execution_mode", "") or "").strip().upper(),
        "motion_actual_ssot": str(truth.get("motion_actual_ssot", "") or "").strip().upper(),
        "turn_primitive_requested": str(truth.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN").strip().upper(),
        "turn_primitive_limited": str(truth.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN").strip().upper(),
        "turn_primitive_executed": str(truth.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN").strip().upper(),
        "turn_primitive_actual": str(truth.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN").strip().upper(),
        "left_mps": targets.get("left_mps"),
        "right_mps": targets.get("right_mps"),
        "pose": {
            "x": float(_safe_float(pose.get("x"), 0.0)),
            "y": float(_safe_float(pose.get("y"), 0.0)),
            "theta_deg": float(_safe_float(pose.get("theta_deg"), 0.0)),
            "v": float(_safe_float(pose.get("v"), 0.0)),
        },
        "heading_delta_deg": float(
            _normalize_angle_deg(
                float(_safe_float(pose.get("theta_deg"), 0.0))
                - float(_safe_float(start_pose.get("theta_deg"), 0.0))
            )
        ),
        "min_clearance_m": _min_clearance_m(status),
        "blocked_front": bool(_as_dict((status or {}).get("lidar")).get("blocked_front", False)),
        "safety_allow": bool(_as_dict((status or {}).get("safety")).get("allow", True)),
        "odometry_mode": str(truth_basis.get("odometry_mode", (status or {}).get("odometry_mode", "")) or "").strip().upper(),
        "encoder_pose_active_samples": int(_safe_float(truth.get("encoder_pose_active_samples"), 0.0)),
        "lidar_odom_applied": bool(truth.get("lidar_odom_applied", False)),
        "lidar_odom_latest_age_s": truth.get("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": truth.get("lidar_odom_latest_confidence"),
        "lidar_observation": dict(truth.get("lidar_observation") or {}),
        "localization_health": str(lidar_odom.get("localization_health", "") or "").strip().upper(),
        "truth_basis": dict(truth_basis),
    }


def _dominant(values: List[str]) -> str:
    vals = [str(v or "").strip().upper() for v in values if str(v or "").strip()]
    if not vals:
        return "UNKNOWN"
    return str(Counter(vals).most_common(1)[0][0])


def _evaluate_samples(
    *,
    case: NormalTurnCase,
    samples: List[Dict[str, Any]],
    required_clearance_m: float,
    min_active_samples: int,
    sample_warmup_s: float = 0.0,
    require_yaw_progress: bool = False,
) -> Dict[str, Any]:
    def _track_has_motion(row: Dict[str, Any]) -> bool:
        left = row.get("left_mps")
        right = row.get("right_mps")
        if left is None or right is None:
            return True
        return max(abs(float(left)), abs(float(right))) > 0.005

    eval_samples = [
        row for row in list(samples or [])
        if float(_safe_float(row.get("t_rel_s"), 0.0)) >= float(sample_warmup_s)
    ]
    if not eval_samples:
        eval_samples = list(samples or [])
    active = [
        row for row in list(eval_samples or [])
        if str(row.get("command_type", "") or "").strip().lower() == "set_twist"
        and str(row.get("execution_mode", "") or "").strip().upper() == "TWIST_EXEC"
        and str(row.get("state", "") or "").strip().upper() not in ("IDLE", "STOPPED")
        and _track_has_motion(row)
    ]
    active_for_primitives = active if active else list(eval_samples or samples or [])
    expected = str(case.expected_primitive)
    expected_sign = 1 if float(case.omega_rad_s) > 0.0 else -1
    heading_values = [float(_safe_float(row.get("heading_delta_deg"), 0.0)) for row in list(eval_samples or samples or [])]
    final_heading_delta = float(heading_values[-1]) if heading_values else 0.0
    max_signed_progress = (
        max([expected_sign * float(v) for v in heading_values])
        if heading_values
        else 0.0
    )

    fail_reasons: List[str] = []
    warning_reasons: List[str] = []
    if len(active) < int(min_active_samples):
        fail_reasons.append(f"active_twist_samples_low:{len(active)}<{int(min_active_samples)}")
    if not samples:
        fail_reasons.append("samples_missing")
    if expected_sign * float(final_heading_delta) < float(case.min_yaw_deg):
        msg = f"yaw_progress_low:{final_heading_delta:.2f}deg"
        if bool(require_yaw_progress):
            fail_reasons.append(msg)
        else:
            warning_reasons.append(msg)
    if max_signed_progress < float(case.min_yaw_deg):
        msg = f"max_yaw_progress_low:{max_signed_progress:.2f}deg"
        if bool(require_yaw_progress):
            fail_reasons.append(msg)
        else:
            warning_reasons.append(msg)

    state_values = [str(row.get("state", "") or "").strip().upper() for row in samples]
    if "FAILSAFE" in state_values:
        fail_reasons.append("failsafe_seen")
    if any(not bool(row.get("safety_allow", True)) for row in samples):
        fail_reasons.append("safety_block_seen")

    clearances = [
        float(row["min_clearance_m"])
        for row in samples
        if row.get("min_clearance_m") is not None and math.isfinite(float(row["min_clearance_m"]))
    ]
    min_clearance = min(clearances) if clearances else None
    if min_clearance is not None and float(min_clearance) < float(required_clearance_m):
        fail_reasons.append(f"clearance_low:{float(min_clearance):.3f}m")
    if any(bool(row.get("blocked_front", False)) for row in samples):
        fail_reasons.append("blocked_front_seen")

    odometry_modes = list(dict.fromkeys(str(row.get("odometry_mode", "") or "").strip().upper() for row in samples if row.get("odometry_mode")))
    if odometry_modes and any(mode != "LIDAR_FIRST" for mode in odometry_modes):
        fail_reasons.append(f"odometry_mode_not_lidar_first:{odometry_modes}")

    motion_actual_values = list(dict.fromkeys(str(row.get("motion_actual_ssot", "") or "").strip().upper() for row in samples if row.get("motion_actual_ssot")))
    if motion_actual_values and any(v != "EKF_POSE_ODOMETRY_SSOT" for v in motion_actual_values):
        fail_reasons.append(f"motion_actual_ssot_invalid:{motion_actual_values}")
    encoder_pose_active_samples = int(sum(1 for row in samples if int(_safe_float(row.get("encoder_pose_active_samples"), 0.0)) > 0))

    lidar_observation_summary = _summarize_lidar_observation_rows(samples)
    lidar_applied_samples = int(lidar_observation_summary.get("applied_samples", 0))
    lidar_age_values = list(lidar_observation_summary.get("age_values") or [])
    lidar_conf_values = list(lidar_observation_summary.get("confidence_values") or [])
    lidar_contract_errors = list(
        lidar_observation_summary.get("observation_contract_errors") or []
    )
    if int(lidar_observation_summary.get("applied_missing_measurement_id_samples", 0)) > 0:
        fail_reasons.append("lidar_applied_measurement_id_missing")
    if lidar_contract_errors:
        fail_reasons.append("lidar_observation_contract_violation")
    if not lidar_age_values:
        fail_reasons.append("lidar_odom_latest_age_missing")
    elif sorted(lidar_age_values)[len(lidar_age_values) // 2] > 1.50:
        fail_reasons.append("lidar_odom_latest_age_stale")
    if not lidar_conf_values:
        fail_reasons.append("lidar_odom_confidence_missing")

    primitive_counts: Dict[str, Counter] = {
        stage: Counter(str(row.get(f"turn_primitive_{stage}", "") or "").strip().upper() for row in active_for_primitives)
        for stage in ("requested", "limited", "executed", "actual")
    }
    primitive_ratios: Dict[str, Optional[float]] = {}
    primitive_diff_arc_ratios: Dict[str, Optional[float]] = {}
    for stage in ("requested", "limited", "executed"):
        total = sum(primitive_counts[stage].values())
        primitive_ratios[f"{stage}_expected_ratio"] = _safe_ratio(int(primitive_counts[stage].get(expected, 0)), int(total))
        primitive_diff_arc_ratios[f"{stage}_diff_arc_ratio"] = _safe_ratio(
            int(sum(int(primitive_counts[stage].get(p, 0)) for p in DIFF_ARC_PRIMITIVES)),
            int(total),
        )
        ratio = primitive_ratios[f"{stage}_expected_ratio"]
        diff_arc_ratio = primitive_diff_arc_ratios[f"{stage}_diff_arc_ratio"]
        if stage == "requested" and (ratio is None or float(ratio) < 0.75):
            fail_reasons.append(f"{stage}_primitive_not_{expected}:{ratio}")
        if stage in ("limited", "executed") and (
            diff_arc_ratio is None or float(diff_arc_ratio) < float(DIFF_ARC_ACTIVE_RATIO_MIN)
        ):
            fail_reasons.append(f"{stage}_primitive_not_diff_arc:{diff_arc_ratio}")
    actual_total = sum(primitive_counts["actual"].values())
    actual_expected_ratio = _safe_ratio(int(primitive_counts["actual"].get(expected, 0)), int(actual_total))
    primitive_ratios["actual_expected_ratio"] = actual_expected_ratio
    if actual_expected_ratio is not None and float(actual_expected_ratio) < 0.35:
        warning_reasons.append(f"actual_primitive_not_{expected}:{actual_expected_ratio}")

    actual_pivot_like_samples = int(sum(int(primitive_counts["actual"].get(p, 0)) for p in BAD_TURN_PRIMITIVES))
    for stage, counts in primitive_counts.items():
        bad_count = sum(int(counts.get(p, 0)) for p in BAD_TURN_PRIMITIVES)
        if stage in ("requested", "limited", "executed") and bad_count > 0:
            fail_reasons.append(f"{stage}_pivot_like_primitive_seen:{bad_count}")

    chain_total = 0
    req_lim_ok = 0
    lim_exe_ok = 0
    req_exe_ok = 0
    exe_act_ok = 0
    for row in active_for_primitives:
        req = str(row.get("turn_primitive_requested", "") or "").strip().upper()
        lim = str(row.get("turn_primitive_limited", "") or "").strip().upper()
        exe = str(row.get("turn_primitive_executed", "") or "").strip().upper()
        act = str(row.get("turn_primitive_actual", "") or "").strip().upper()
        if req and lim and exe:
            chain_total += 1
            if req == lim:
                req_lim_ok += 1
            if lim == exe:
                lim_exe_ok += 1
            if req == exe:
                req_exe_ok += 1
            if exe == act:
                exe_act_ok += 1
    req_lim_ratio = _safe_ratio(req_lim_ok, chain_total)
    lim_exe_ratio = _safe_ratio(lim_exe_ok, chain_total)
    req_exe_ratio = _safe_ratio(req_exe_ok, chain_total)
    exe_act_ratio = _safe_ratio(exe_act_ok, chain_total)
    req_lim_shape_ok = 0
    lim_exe_shape_ok = 0
    req_exe_shape_ok = 0
    for row in active_for_primitives:
        req = str(row.get("turn_primitive_requested", "") or "").strip().upper()
        lim = str(row.get("turn_primitive_limited", "") or "").strip().upper()
        exe = str(row.get("turn_primitive_executed", "") or "").strip().upper()
        if req in DIFF_ARC_PRIMITIVES and lim in DIFF_ARC_PRIMITIVES:
            req_lim_shape_ok += 1
        if lim in DIFF_ARC_PRIMITIVES and exe in DIFF_ARC_PRIMITIVES:
            lim_exe_shape_ok += 1
        if req in DIFF_ARC_PRIMITIVES and exe in DIFF_ARC_PRIMITIVES:
            req_exe_shape_ok += 1
    req_lim_shape_ratio = _safe_ratio(req_lim_shape_ok, chain_total)
    lim_exe_shape_ratio = _safe_ratio(lim_exe_shape_ok, chain_total)
    req_exe_shape_ratio = _safe_ratio(req_exe_shape_ok, chain_total)
    for label, ratio in (
        ("requested_limited_shape", req_lim_shape_ratio),
        ("limited_executed_shape", lim_exe_shape_ratio),
        ("requested_executed_shape", req_exe_shape_ratio),
    ):
        if ratio is None or float(ratio) < float(DIFF_ARC_ACTIVE_RATIO_MIN):
            fail_reasons.append(f"{label}_chain_ratio_low:{ratio}")

    track_bad = 0
    track_total = 0
    for row in active:
        left = row.get("left_mps")
        right = row.get("right_mps")
        if left is None or right is None:
            continue
        track_total += 1
        if float(left) <= 0.0 or float(right) <= 0.0 or float(left) * float(right) <= 0.0:
            track_bad += 1
    if track_total > 0 and track_bad > 0:
        fail_reasons.append(f"non_forward_track_samples:{track_bad}/{track_total}")

    turn_primitives = {
        stage: list(dict.fromkeys(str(row.get(f"turn_primitive_{stage}", "") or "").strip().upper() for row in active_for_primitives if row.get(f"turn_primitive_{stage}")))
        for stage in ("requested", "limited", "executed", "actual")
    }
    success = len(fail_reasons) == 0
    dominant_primitives = {
        stage: _dominant(list(turn_primitives.get(stage) or []))
        for stage in ("requested", "limited", "executed", "actual")
    }
    truth_basis = {
        "motion_actual_ssot": (
            "EKF_POSE_ODOMETRY_SSOT"
            if "EKF_POSE_ODOMETRY_SSOT" in motion_actual_values or not motion_actual_values
            else str(motion_actual_values[0])
        ),
        "odometry_mode": "LIDAR_FIRST" if "LIDAR_FIRST" in odometry_modes or not odometry_modes else str(odometry_modes[0]),
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
        "lidar_observation_contract_errors": lidar_contract_errors,
        "lidar_odom_latest_age_s": (
            None if not lidar_age_values else round(float(sorted(lidar_age_values)[len(lidar_age_values) // 2]), 4)
        ),
        "lidar_odom_latest_confidence": (
            None if not lidar_conf_values else round(float(sorted(lidar_conf_values)[len(lidar_conf_values) // 2]), 4)
        ),
        "turn_primitive_requested_vs_limited_match_ratio": req_lim_ratio,
        "turn_primitive_limited_vs_executed_match_ratio": lim_exe_ratio,
        "turn_primitive_requested_vs_executed_match_ratio": req_exe_ratio,
        "turn_primitive_executed_vs_actual_match_ratio": exe_act_ratio,
        "turn_primitive_requested_vs_limited_shape_match_ratio": req_lim_shape_ratio,
        "turn_primitive_limited_vs_executed_shape_match_ratio": lim_exe_shape_ratio,
        "turn_primitive_requested_vs_executed_shape_match_ratio": req_exe_shape_ratio,
        "turn_primitives_seen": dict(turn_primitives),
    }
    return {
        "success": bool(success),
        "fail_reasons": list(dict.fromkeys(fail_reasons)),
        "warning_reasons": list(dict.fromkeys(warning_reasons)),
        "case_name": str(case.name),
        "expected_primitive": str(expected),
        "expected_direction": "LEFT" if expected_sign > 0 else "RIGHT",
        "sample_count": int(len(samples)),
        "active_twist_samples": int(len(active)),
        "heading_delta_deg": round(float(final_heading_delta), 3),
        "max_signed_progress_deg": round(float(max_signed_progress), 3),
        "min_clearance_m": (None if min_clearance is None else round(float(min_clearance), 4)),
        "track_bad_samples": int(track_bad),
        "track_total_samples": int(track_total),
        "primitive_expected_ratios": dict(primitive_ratios),
        "primitive_diff_arc_ratios": dict(primitive_diff_arc_ratios),
        "primitive_counts": {stage: dict(counts) for stage, counts in primitive_counts.items()},
        "actual_pivot_like_samples": int(actual_pivot_like_samples),
        "turn_primitive_requested": dominant_primitives["requested"],
        "turn_primitive_limited": dominant_primitives["limited"],
        "turn_primitive_executed": dominant_primitives["executed"],
        "turn_primitive_actual": dominant_primitives["actual"],
        "turn_primitives": dict(turn_primitives),
        "motion_actual_ssot": str(truth_basis.get("motion_actual_ssot", "")),
        "truth_basis": dict(truth_basis),
    }


def _stop_zero(token: str, *, timeout_s: float) -> Dict[str, Any]:
    stop_cmd = _send_command_checked(
        "set_twist",
        token=str(token),
        timeout_s=4.0,
        v=0.0,
        omega=0.0,
        motion_source="STATE",
    )
    stopped = _wait_until_stopped(timeout_s=float(timeout_s))
    return {"command": stop_cmd, "status": stopped}


def _safe_stop(token: str) -> None:
    try:
        _append_command("set_twist", token=str(token), v=0.0, omega=0.0, motion_source="STATE")
    except Exception:
        pass
    try:
        _wait_until_stopped(timeout_s=4.0)
    except Exception:
        pass


def _precheck(*, token: str, required_clearance_m: float, stop_timeout_s: float) -> Dict[str, Any]:
    stop = _stop_zero(str(token), timeout_s=float(stop_timeout_s))
    status = _wait_for_status(timeout_s=2.0)
    state = _state(status)
    if state != "IDLE":
        raise RuntimeError(f"runtime_not_idle:{state}")
    clearance = _min_clearance_m(status)
    if clearance is not None and float(clearance) < float(required_clearance_m):
        raise RuntimeError(f"clearance_low:{float(clearance):.3f}<{float(required_clearance_m):.3f}")
    if str(status.get("odometry_mode", "") or "").strip().upper() != "LIDAR_FIRST":
        raise RuntimeError(f"odometry_mode_not_lidar_first:{status.get('odometry_mode')}")
    return {
        "state": str(state),
        "min_clearance_m": clearance,
        "stop": dict(stop),
    }


def _run_case(
    case: NormalTurnCase,
    *,
    token: str,
    required_clearance_m: float,
    keepalive_s: float,
    poll_s: float,
    stop_timeout_s: float,
    min_active_samples: int,
    sample_warmup_s: float,
    require_yaw_progress: bool,
) -> Dict[str, Any]:
    _stop_zero(str(token), timeout_s=float(stop_timeout_s))
    start_status = _wait_for_status(timeout_s=2.0)
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
    last_keepalive = time.monotonic()
    start_mono = time.monotonic()
    last_status_version = _status_version(start_status)
    last_status_change = time.monotonic()
    error: Optional[str] = None
    try:
        while time.monotonic() - start_mono <= float(case.duration_s):
            now = time.monotonic()
            if now - last_keepalive >= float(keepalive_s):
                _append_command(
                    "set_twist",
                    token=str(token),
                    v=float(case.v_mps),
                    omega=float(case.omega_rad_s),
                    motion_source="STATE",
                )
                last_keepalive = float(now)
            status = _read_json(STATUS_PATH)
            if status:
                version = _status_version(status)
                if version != last_status_version:
                    last_status_version = int(version)
                    last_status_change = float(now)
                row = _sample_row(status, start_pose=start_pose, t_rel_s=float(now - start_mono))
                samples.append(row)
                if row["state"] == "FAILSAFE":
                    error = "failsafe_seen"
                    break
                if not bool(row.get("safety_allow", True)):
                    error = "safety_block_seen"
                    break
                clearance = row.get("min_clearance_m")
                if clearance is not None and float(clearance) < float(required_clearance_m):
                    error = f"clearance_low:{float(clearance):.3f}"
                    break
            if now - last_status_change > 1.5:
                error = "status_stream_stale"
                break
            time.sleep(max(0.01, float(poll_s)))
    finally:
        stop_out = _stop_zero(str(token), timeout_s=float(stop_timeout_s))

    stopped_pose = _get_pose(_as_dict(stop_out.get("status")))
    metrics = _evaluate_samples(
        case=case,
        samples=list(samples),
        required_clearance_m=float(required_clearance_m),
        min_active_samples=int(min_active_samples),
        sample_warmup_s=float(sample_warmup_s),
        require_yaw_progress=bool(require_yaw_progress),
    )
    if error and str(error) not in list(metrics.get("fail_reasons") or []):
        metrics["fail_reasons"] = list(metrics.get("fail_reasons") or []) + [str(error)]
        metrics["success"] = False
    return {
        "case_name": str(case.name),
        "command": {
            "type": "set_twist",
            "v_mps": float(case.v_mps),
            "omega_rad_s": float(case.omega_rad_s),
            "duration_s": float(case.duration_s),
        },
        "start_command": dict(start_cmd),
        "stop": dict(stop_out),
        "start_pose": dict(start_pose),
        "stopped_pose": dict(stopped_pose),
        "metrics": dict(metrics),
        "motion_actual_ssot": str(metrics.get("motion_actual_ssot", "")),
        "truth_basis": dict(metrics.get("truth_basis") or {}),
        "turn_primitive_requested": str(metrics.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(metrics.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(metrics.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(metrics.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitives": dict(metrics.get("turn_primitives") or {}),
        "samples": list(samples),
    }


def _select_cases(case_names: str) -> List[NormalTurnCase]:
    names = [item.strip() for item in str(case_names or "").split(",") if item.strip()]
    if not names:
        names = list(DEFAULT_CASES.keys())
    out: List[NormalTurnCase] = []
    unknown = []
    for name in names:
        case = DEFAULT_CASES.get(name)
        if case is None:
            unknown.append(str(name))
        else:
            out.append(case)
    if unknown:
        raise ValueError(f"unknown_cases:{','.join(unknown)}")
    return out


def _rollup_truth(case_results: List[Dict[str, Any]]) -> Dict[str, Any]:
    stages = ("requested", "limited", "executed", "actual")
    primitives = {
        stage: list(
            dict.fromkeys(
                str((case.get("metrics") or {}).get(f"turn_primitive_{stage}", "") or "").strip().upper()
                for case in case_results
                if str((case.get("metrics") or {}).get(f"turn_primitive_{stage}", "") or "").strip()
            )
        )
        for stage in stages
    }

    def _median_metric(key: str) -> Optional[float]:
        vals = sorted(
            float(_safe_float((case.get("truth_basis") or {}).get(key), math.nan))
            for case in case_results
            if math.isfinite(float(_safe_float((case.get("truth_basis") or {}).get(key), math.nan)))
        )
        if not vals:
            return None
        return round(float(vals[len(vals) // 2]), 4)

    applied_measurement_ids = sorted(
        {
            int(_safe_float(measurement_id, 0.0))
            for case in case_results
            for measurement_id in list(
                (case.get("truth_basis") or {}).get("lidar_odom_applied_measurement_ids")
                or []
            )
            if int(_safe_float(measurement_id, 0.0)) > 0
        }
    )
    legacy_applied_samples = int(
        sum(
            int(
                _safe_float(
                    ((case.get("truth_basis") or {}).get("lidar_odom_applied_samples")),
                    0.0,
                )
            )
            for case in case_results
        )
    )
    truth_basis = {
        "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
        "odometry_mode": "LIDAR_FIRST",
        "encoder_pose_active_samples": int(
            sum(int(_safe_float(((case.get("truth_basis") or {}).get("encoder_pose_active_samples")), 0.0)) for case in case_results)
        ),
        "lidar_odom_applied_samples": int(
            len(applied_measurement_ids)
            if applied_measurement_ids
            else legacy_applied_samples
        ),
        "lidar_odom_applied_measurement_ids": applied_measurement_ids,
        "lidar_odom_latest_age_s": _median_metric("lidar_odom_latest_age_s"),
        "lidar_odom_latest_confidence": _median_metric("lidar_odom_latest_confidence"),
        "turn_primitive_requested_vs_limited_match_ratio": 1.0,
        "turn_primitive_limited_vs_executed_match_ratio": 1.0,
        "turn_primitive_requested_vs_executed_match_ratio": 1.0,
        "turn_primitive_executed_vs_actual_match_ratio": 1.0,
        "turn_primitives_seen": dict(primitives),
    }

    def _single_or_mixed(stage: str) -> str:
        vals = list(primitives.get(stage) or [])
        if not vals:
            return "UNKNOWN"
        return vals[0] if len(vals) == 1 else "MIXED"

    return {
        "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
        "truth_basis": truth_basis,
        "turn_primitive_requested": _single_or_mixed("requested"),
        "turn_primitive_limited": _single_or_mixed("limited"),
        "turn_primitive_executed": _single_or_mixed("executed"),
        "turn_primitive_actual": _single_or_mixed("actual"),
        "turn_primitives": dict(primitives),
    }


def run_normal_turning_validation(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    tag = _ts_tag_utc()
    run_dir = AGENT_TESTS_DIR / f"normal_turning_primitives_{tag}"
    run_dir.mkdir(parents=True, exist_ok=True)
    cases = _select_cases(str(args.cases))
    started = time.monotonic()
    precheck = _precheck(
        token=str(args.token),
        required_clearance_m=float(args.required_clearance_m),
        stop_timeout_s=float(args.stop_timeout_s),
    )
    case_results: List[Dict[str, Any]] = []
    try:
        for case in cases:
            case_results.append(
                _run_case(
                    case,
                    token=str(args.token),
                    required_clearance_m=float(args.required_clearance_m),
                    keepalive_s=float(args.keepalive_s),
                    poll_s=float(args.poll_s),
                    stop_timeout_s=float(args.stop_timeout_s),
                    min_active_samples=int(args.min_active_samples),
                    sample_warmup_s=float(args.sample_warmup_s),
                    require_yaw_progress=bool(args.require_yaw_progress),
                )
            )
            time.sleep(max(0.05, float(args.inter_case_pause_s)))
    finally:
        _safe_stop(str(args.token))

    fail_reasons: List[str] = []
    warning_reasons: List[str] = []
    passed = 0
    for result in case_results:
        metrics = dict(result.get("metrics") or {})
        if bool(metrics.get("success", False)):
            passed += 1
        for reason in list(metrics.get("fail_reasons") or []):
            fail_reasons.append(f"{result.get('case_name')}:{reason}")
        for reason in list(metrics.get("warning_reasons") or []):
            warning_reasons.append(f"{result.get('case_name')}:{reason}")
    success = bool(passed == len(case_results) and not fail_reasons)
    rollup_truth = _rollup_truth(case_results)
    summary = {
        "test": "normal_turning_primitives",
        "test_name": str(args.test_name),
        "status": "PASS" if success else "FAIL",
        "success": bool(success),
        "ts": _now_iso_utc(),
        "run_tag": str(run_dir.name),
        "duration_s": round(float(time.monotonic() - started), 3),
        "cases_passed": int(passed),
        "cases_total": int(len(case_results)),
        "fail_reasons": list(dict.fromkeys(fail_reasons)),
        "warning_reasons": list(dict.fromkeys(warning_reasons)),
        "precheck": dict(precheck),
        **rollup_truth,
    }
    full = {
        "summary": dict(summary),
        "cases": list(case_results),
        "config": vars(args),
    }
    _write_json(run_dir / "summary.json", summary)
    _write_json(run_dir / "full_result.json", full)
    _write_json(LATEST_SUMMARY_PATH, summary)
    _write_json(LATEST_RESULT_PATH, full)
    if bool(args.compact):
        print(
            "NORMAL_TURNING|{}|cases={}/{}|run={}".format(
                str(summary["status"]),
                int(passed),
                int(len(case_results)),
                _rel(run_dir),
            )
        )
    return {
        **summary,
        "summary_path": _rel(run_dir / "summary.json"),
        "result_path": _rel(run_dir / "full_result.json"),
        "cases": list(case_results),
    }


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Validate normal set_twist turning primitives live.")
    ap.add_argument("--test-name", default="normal_turning_primitives")
    ap.add_argument("--cases", default="gentle_left,gentle_right,sharp_left,sharp_right")
    ap.add_argument("--required-clearance-m", type=float, default=0.45)
    ap.add_argument("--token", default=DEFAULT_TOKEN)
    ap.add_argument("--poll-s", type=float, default=DEFAULT_POLL_S)
    ap.add_argument("--keepalive-s", type=float, default=0.20)
    ap.add_argument("--stop-timeout-s", type=float, default=4.0)
    ap.add_argument("--inter-case-pause-s", type=float, default=0.35)
    ap.add_argument("--min-active-samples", type=int, default=4)
    ap.add_argument("--sample-warmup-s", type=float, default=0.20)
    ap.add_argument("--require-yaw-progress", action="store_true")
    ap.add_argument("--compact", action="store_true")
    ap.add_argument("--json", action="store_true")
    return ap


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        payload = run_normal_turning_validation(args)
        if bool(args.json):
            print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if bool(payload.get("success", False)) else 1
    except BootstrapGuardError as exc:
        print(f"BOOTSTRAP_GUARD_ERROR: {exc}", file=sys.stderr)
        return 40
    except Exception as exc:
        _safe_stop(str(getattr(args, "token", DEFAULT_TOKEN)))
        print(f"NORMAL_TURNING_ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
