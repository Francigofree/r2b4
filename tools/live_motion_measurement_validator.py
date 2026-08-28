#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Live measurement-trust and baseline motion validation through normal twist commands."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from controller.motion_kinematics import twist_to_track_velocity  # noqa: E402
from tools.kit0085_live_audit import _encoder_reading, _safe_float, _safe_int  # noqa: E402
from tools.kit0085_motor_bench_audit import _ensure_control_mode  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_MOTION_SOURCE,
    DEFAULT_TOKEN,
    STATUS_PATH,
    _append_command,
    _get_pose,
    _latest_command_status,
    _normalize_angle_deg,
    _pose_distance,
    _read_json,
    _safe_stop_best_effort,
    _send_command_checked,
    _status_version,
    _wait_command_terminal,
    _wait_for_status_progress,
    _wait_until_stopped,
)


AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_TRUST_RESULT_PATH = AGENT_TESTS_DIR / "latest_M0_measurement_trust_live.json"
LATEST_MINI_TRUST_RESULT_PATH = AGENT_TESTS_DIR / "latest_M0_mini_measurement_trust_live.json"
LATEST_BASELINE_RESULT_PATH = AGENT_TESTS_DIR / "latest_M1_motion_baseline_live.json"
LATEST_TRUST_SAMPLES_PATH = AGENT_TESTS_DIR / "latest_M0_measurement_trust_live_samples.jsonl"
LATEST_BASELINE_SAMPLES_PATH = AGENT_TESTS_DIR / "latest_M1_motion_baseline_live_samples.jsonl"
M0_MINI_CONTRACT_ID = "R2B4_M0_MINI_FIRST_MOTION_V2"
M0_MINI_CASE_NAME = "m0_mini"
M0_MINI_CASTER_PAIR = "m0_mini_first_forward"
M0_MINI_CASTER_ORIENTATION = "uncontrolled_initial"
M1_SPEED_MAP_EXECUTION_CONTRACT_ID = "R2B4_M1_SPEED_MAP_EXECUTION_V1"
M1_CHASSIS_DYNAMICS_VALIDATOR = "M2_chassis_motion_dynamics_live"
M1_CASTER_INFLUENCE_CONTRACT_ID = "R2B4_M1_PASSIVE_FRONT_CASTER_V1"
M1_CASTER_ORIENTATION = "uncontrolled_case_start"
SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID = (
    "R2B4_SENSOR_ENDPOINT_SHARED_SOURCE_TIME_V1"
)

RAD_TO_DEG = 57.29577951308232
MIN_PWM_OBSERVED = 0.05
SETTLED_PHASE_START_S = 0.30
WHEEL_SPEED_TRACKING_MAE_MAX_MPS = 0.015
CASTER_TRANSIENT_ALLOWANCE_S = 1.0
CASTER_FULL_SETTLED_WHEEL_MAE_MAX_MPS = 0.030
CASTER_MIN_TRANSIENT_FEEDBACK_WINDOWS = 3
CASTER_MIN_POST_TRANSIENT_FEEDBACK_WINDOWS = 3
M1_CASTER_WHEEL_MAE_RELATIVE_MAX = 0.20
M1_CASTER_WHEEL_MAE_ABSOLUTE_MAX_MPS = 0.050
M1_CASTER_ARC_ANGULAR_RATIO_MIN = 0.75
M1_CASTER_ARC_ANGULAR_RATIO_MAX = 1.25
MIN_TRANSLATION_PROGRESS_M = 0.004
MIN_ROTATION_PROGRESS_DEG = 1.0
MAX_LIDAR_LATEST_AGE_S = 1.5
MIN_LIDAR_CONFIDENCE = 0.20
MAX_SHARED_ENDPOINT_ENCODER_BRACKET_S = 0.30
FALLBACK_STABLE_PWM_FLOOR = 0.08
STABLE_PWM_FLOOR_MARGIN = 0.005
MAX_NEAR_STABLE_PWM_RATIO = 0.65
MAX_BELOW_STABLE_PWM_RATIO = 0.20
LATEST_DEADZONE_RESULT_PATH = latest_artifact_path("latest_motor_deadzone_calibration.json")
VALIDATION_WHEEL_MAX_MPS = 0.30
# The canonical gear contract scales the physical profile as level / 9.
# Expose the complete active 0.30 m/s wheel range during validation; the
# individual cases remain bounded by their own physical track references.
VALIDATION_SPEED_LEVEL = 9


@dataclass(frozen=True)
class MeasurementCase:
    name: str
    kind: str
    v_mps: float
    omega_rad_s: float
    duration_s: float
    expected_linear_sign: int = 0
    expected_yaw_sign: int = 0
    command_motion: bool = True
    quality_gate: bool = True
    command_type: str = "set_twist"
    target_angle_deg: Optional[float] = None
    heading_speed_level: int = 0
    caster_pair: str = ""
    caster_orientation: str = ""
    caster_transient_s: float = 0.0
    operator_instruction_hu: str = ""
    chassis_dynamics_verdict: bool = True


M0_CASES = (
    MeasurementCase("idle_static", "idle", 0.0, 0.0, 1.2, command_motion=False, quality_gate=False),
    MeasurementCase("trust_forward_pulse", "forward", 0.150, 0.0, 2.16, expected_linear_sign=1, quality_gate=False),
    MeasurementCase("trust_arc_left", "arc_left", 0.225, 0.20, 1.44, expected_linear_sign=1, expected_yaw_sign=1, quality_gate=False),
    MeasurementCase("trust_arc_right", "arc_right", 0.225, -0.20, 1.44, expected_linear_sign=1, expected_yaw_sign=-1, quality_gate=False),
)

M1_CASES = (
    MeasurementCase(
        M0_MINI_CASE_NAME,
        "start",
        0.150,
        0.0,
        2.16,
        expected_linear_sign=1,
        caster_pair=M0_MINI_CASTER_PAIR,
        caster_orientation=M0_MINI_CASTER_ORIENTATION,
        caster_transient_s=CASTER_TRANSIENT_ALLOWANCE_S,
    ),
    MeasurementCase(
        "forward",
        "forward",
        0.150,
        0.0,
        3.6,
        expected_linear_sign=1,
        caster_pair="m1_forward_after_first_forward",
        caster_orientation=M1_CASTER_ORIENTATION,
        caster_transient_s=CASTER_TRANSIENT_ALLOWANCE_S,
        chassis_dynamics_verdict=False,
    ),
    MeasurementCase(
        "backward",
        "backward",
        -0.150,
        0.0,
        3.24,
        expected_linear_sign=-1,
        caster_pair="m1_reverse_after_forward",
        caster_orientation=M1_CASTER_ORIENTATION,
        caster_transient_s=CASTER_TRANSIENT_ALLOWANCE_S,
        chassis_dynamics_verdict=False,
    ),
    MeasurementCase(
        "arc_left",
        "arc_left",
        0.225,
        0.20,
        2.4,
        expected_linear_sign=1,
        expected_yaw_sign=1,
        caster_pair="m1_arc_left_after_reverse",
        caster_orientation=M1_CASTER_ORIENTATION,
        caster_transient_s=CASTER_TRANSIENT_ALLOWANCE_S,
        chassis_dynamics_verdict=False,
    ),
    MeasurementCase(
        "arc_right",
        "arc_right",
        0.225,
        -0.20,
        2.4,
        expected_linear_sign=1,
        expected_yaw_sign=-1,
        caster_pair="m1_arc_right_after_arc_left",
        caster_orientation=M1_CASTER_ORIENTATION,
        caster_transient_s=CASTER_TRANSIENT_ALLOWANCE_S,
        chassis_dynamics_verdict=False,
    ),
    MeasurementCase("rotate_left", "rotate_left", 0.0, 0.0, 8.0, expected_yaw_sign=1, command_type="rotate_to_heading", target_angle_deg=45.0, heading_speed_level=1, chassis_dynamics_verdict=False),
    MeasurementCase("rotate_right", "rotate_right", 0.0, 0.0, 8.0, expected_yaw_sign=-1, command_type="rotate_to_heading", target_angle_deg=-45.0, heading_speed_level=1, chassis_dynamics_verdict=False),
    MeasurementCase("stop_hold", "stop", 0.0, 0.0, 1.2, command_motion=False, chassis_dynamics_verdict=False),
)

M1_CASTER_CASE_CONTRACTS = {
    case.name: case.caster_pair
    for case in M1_CASES
    if case.name != M0_MINI_CASE_NAME
    and bool(case.expected_linear_sign)
    and bool(case.caster_pair)
}


def _m1_speed_map_execution_contract(
    *,
    required: bool,
) -> Dict[str, Any]:
    return {
        "required": bool(required),
        "contract_id": M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
        "promotion_blocking": bool(required),
        "chassis_dynamics_verdict": not bool(required),
        "delegated_validator": M1_CHASSIS_DYNAMICS_VALIDATOR,
        "blocking_scope": [
            "embedded_m0_mini",
            "executed_command",
            "wheel_reference_tracking",
            "linear_speed",
            "whole_phase_distance",
            "integrated_distance",
            "safety",
            "sensor_truth",
            "timing_contract",
            "endpoint_consistency",
            "stop_start",
            "normal_stop",
        ],
        "delegated_scope": [
            "passive_front_caster",
            "effective_track_width",
            "ground_slip",
            "physical_arc_yaw_and_curvature",
            "pivot_accuracy_and_symmetry",
        ],
    }


def _m1_caster_wheel_mae_limit_mps(linear_mps: Any) -> float:
    speed = abs(_safe_float(linear_mps, 0.0))
    return min(
        float(M1_CASTER_WHEEL_MAE_ABSOLUTE_MAX_MPS),
        max(
            float(WHEEL_SPEED_TRACKING_MAE_MAX_MPS),
            float(M1_CASTER_WHEEL_MAE_RELATIVE_MAX) * float(speed),
        ),
    )


def _m1_caster_contract_matches(
    case_name: str,
    command: Dict[str, Any],
) -> bool:
    expected_pair = M1_CASTER_CASE_CONTRACTS.get(str(case_name))
    caster_transient_s = _finite(command.get("caster_transient_s"))
    return bool(
        expected_pair
        and str(command.get("caster_pair", "") or "") == str(expected_pair)
        and str(command.get("caster_orientation", "") or "")
        == M1_CASTER_ORIENTATION
        and caster_transient_s is not None
        and abs(
            float(caster_transient_s) - float(CASTER_TRANSIENT_ALLOWANCE_S)
        )
        <= 1e-9
    )


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _validation_motion_contract(
    cases: Iterable[MeasurementCase],
    *,
    track_width_m: float,
    wheel_min_mps: float,
    wheel_max_mps: float,
) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    failures: List[str] = []
    minimum = max(0.0, float(wheel_min_mps))
    maximum = max(minimum, float(wheel_max_mps))
    for case in cases:
        if not bool(case.command_motion) or str(case.command_type).strip().lower() != "set_twist":
            continue
        left_mps, right_mps = twist_to_track_velocity(
            float(case.v_mps),
            float(case.omega_rad_s),
            float(track_width_m),
        )
        nonzero_tracks = [abs(value) for value in (left_mps, right_mps) if abs(value) > 1e-9]
        feasible = bool(
            nonzero_tracks
            and all(minimum - 1e-9 <= value <= maximum + 1e-9 for value in nonzero_tracks)
        )
        row = {
            "case": str(case.name),
            "v_mps": float(case.v_mps),
            "omega_rad_s": float(case.omega_rad_s),
            "left_mps": float(left_mps),
            "right_mps": float(right_mps),
            "feasible": bool(feasible),
        }
        rows.append(row)
        if not feasible:
            failures.append(f"{case.name}:wheel_speed_range_infeasible")
    return {
        "ok": not failures,
        "track_width_m": float(track_width_m),
        "wheel_min_mps": float(minimum),
        "wheel_max_mps": float(maximum),
        "cases": rows,
        "failures": failures,
    }


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
    os.replace(tmp, path)


def _reset_jsonl(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")


def _finite(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out):
        return None
    return float(out)


def _canonical_encoder_count_window_identity(
    pulses: Dict[str, Any],
) -> Optional[tuple]:
    """Return the immutable identity of one canonical encoder count window."""
    window_start = _finite(pulses.get("window_start_ts"))
    window_end = _finite(pulses.get("window_end_ts"))
    if window_start is None or window_end is None:
        return None
    counts: List[int] = []
    for key in (
        "left_count_start",
        "left_count_end",
        "right_count_start",
        "right_count_end",
    ):
        value = pulses.get(key)
        if value is None or isinstance(value, bool):
            return None
        try:
            integer = int(value)
            if float(value) != float(integer):
                return None
        except Exception:
            return None
        counts.append(integer)
    return (float(window_start), float(window_end), *counts)


def _mean(values: Iterable[float]) -> Optional[float]:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return None
    return float(sum(vals) / len(vals))


def _median(values: Iterable[float]) -> Optional[float]:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return None
    return float(statistics.median(vals))


def _percentile(values: Iterable[Any], fraction: float) -> Optional[float]:
    vals = sorted(float(value) for value in values if _finite(value) is not None)
    if not vals:
        return None
    index = int(round(max(0.0, min(1.0, float(fraction))) * float(len(vals) - 1)))
    return float(vals[index])


def _moving_median(values: Iterable[Any], window: int = 5) -> List[float]:
    vals = [float(value) for value in values if _finite(value) is not None]
    if not vals:
        return []
    radius = max(0, int(window) // 2)
    return [
        float(statistics.median(vals[max(0, idx - radius):min(len(vals), idx + radius + 1)]))
        for idx in range(len(vals))
    ]


def _integrate_timed_samples(
    samples: List[Dict[str, Any]],
    indices: Iterable[int],
    value_getter,
) -> Optional[float]:
    points: List[tuple[float, float]] = []
    for idx in indices:
        ts = _finite(samples[idx].get("ts"))
        value = _finite(value_getter(samples[idx]))
        if ts is not None and value is not None:
            points.append((float(ts), float(value)))
    if len(points) < 2:
        return None
    area = 0.0
    for (ts0, value0), (ts1, value1) in zip(points, points[1:]):
        dt = max(0.0, min(0.5, float(ts1) - float(ts0)))
        area += 0.5 * (float(value0) + float(value1)) * dt
    return float(area)


def _sign_ok(value: Optional[float], sign: int, minimum_abs: float) -> bool:
    if int(sign) == 0:
        return True
    if value is None:
        return False
    return (float(value) * float(sign)) >= float(minimum_abs)


def _status_state(status: Dict[str, Any]) -> str:
    return str((status or {}).get("state", "") or "").strip().upper()


def _extract_imu_yaw_deg(status: Dict[str, Any]) -> Dict[str, Any]:
    imu = dict((status or {}).get("imu") or {})
    euler = dict(imu.get("euler") or {})
    gyro = list(imu.get("gyro") or [])
    gyro_z_dps = _finite(gyro[2]) if len(gyro) >= 3 else None
    candidates = (
        ("yaw_deg", False),
        ("heading_deg", False),
        ("theta_deg", False),
        ("yaw_rad", True),
        ("heading_rad", True),
        ("yaw", False),
        ("heading", False),
        ("z", False),
    )
    for key, is_rad in candidates:
        val = _finite(euler.get(key))
        if val is None:
            continue
        yaw = float(val) * RAD_TO_DEG if bool(is_rad) else float(val)
        if key in ("yaw", "heading", "z") and abs(yaw) <= (2.0 * math.pi + 0.01):
            yaw *= RAD_TO_DEG
        return {
            "available": True,
            "yaw_deg": float(yaw),
            "source": f"imu.euler.{key}",
            "health": str(imu.get("health", "") or ""),
            "gyro_z_dps": gyro_z_dps,
        }
    return {
        "available": gyro_z_dps is not None,
        "yaw_deg": None,
        "source": "",
        "health": str(imu.get("health", "") or ""),
        "gyro_z_dps": gyro_z_dps,
    }


def _pose_like(value: Any) -> Dict[str, Optional[float]]:
    if isinstance(value, dict):
        theta = _finite(value.get("theta", value.get("theta_rad")))
        theta_deg = _finite(value.get("theta_deg"))
        if theta is None and theta_deg is not None:
            theta = math.radians(float(theta_deg))
        if theta_deg is None and theta is not None:
            theta_deg = math.degrees(float(theta))
        return {
            "x": _finite(value.get("x")),
            "y": _finite(value.get("y")),
            "theta": theta,
            "theta_deg": theta_deg,
        }
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        theta = _finite(value[2])
        return {
            "x": _finite(value[0]),
            "y": _finite(value[1]),
            "theta": theta,
            "theta_deg": None if theta is None else math.degrees(float(theta)),
        }
    return {"x": None, "y": None, "theta": None, "theta_deg": None}


def _extract_lidar_pose(status: Dict[str, Any]) -> Dict[str, Any]:
    lidar_odom = dict((status or {}).get("lidar_odom_status") or {})
    for key in ("last_lidar_pose", "pose_ref_current", "ekf_pose_after"):
        pose = _pose_like(lidar_odom.get(key))
        if pose.get("x") is not None and pose.get("y") is not None:
            return {"available": True, "source": f"lidar_odom_status.{key}", **pose}
    return {"available": False, "source": "", "x": None, "y": None, "theta": None, "theta_deg": None}


def _positive_int_or_none(value: Any) -> Optional[int]:
    try:
        parsed = int(value or 0)
    except (TypeError, ValueError):
        return None
    return int(parsed) if parsed > 0 else None


def _extract_lidar_status(status: Dict[str, Any]) -> Dict[str, Any]:
    lidar = dict((status or {}).get("lidar") or {})
    lidar_odom = dict((status or {}).get("lidar_odom_status") or {})
    conf = _finite(lidar_odom.get("latest_confidence"))
    if conf is None:
        conf = _finite(lidar.get("latest_confidence", lidar.get("final_confidence")))
    latest_age = _finite(lidar_odom.get("latest_age_s"))
    raw_scan_id = _positive_int_or_none(
        lidar_odom.get("raw_scan_id", lidar.get("raw_scan_id"))
    )
    matcher_result_id = _positive_int_or_none(lidar_odom.get("matcher_result_id"))
    candidate_id = _positive_int_or_none(lidar_odom.get("candidate_id"))
    measurement_id = _positive_int_or_none(
        lidar_odom.get("lidar_odometry_measurement_id")
    )
    measurement_timestamp = _finite(
        lidar_odom.get("lidar_odometry_measurement_timestamp")
    )
    ekf_input_measurement_id = _positive_int_or_none(
        lidar_odom.get("ekf_input_lidar_odometry_measurement_id")
    )
    ekf_processed_measurement_id = _positive_int_or_none(
        lidar_odom.get("ekf_last_processed_lidar_odometry_measurement_id")
    )
    ekf_applied_measurement_id = _positive_int_or_none(
        lidar_odom.get("ekf_last_applied_lidar_odometry_measurement_id")
    )
    source_matcher_result_id = _positive_int_or_none(
        lidar_odom.get("measurement_source_matcher_result_id")
    )
    source_raw_scan_id = _positive_int_or_none(
        lidar_odom.get("measurement_source_raw_scan_id")
    )
    source_raw_scan_timestamp = _finite(
        lidar_odom.get("measurement_source_raw_scan_timestamp")
    )
    candidate_source_raw_scan_id = _positive_int_or_none(
        lidar_odom.get("candidate_source_raw_scan_id")
    )
    applied = bool(lidar_odom.get("applied", False))
    lineage_errors: List[str] = []
    if (
        matcher_result_id is not None
        and candidate_id is not None
        and matcher_result_id != candidate_id
    ):
        lineage_errors.append("candidate_matcher_result_id_mismatch")
    if measurement_id is not None and source_matcher_result_id is None:
        lineage_errors.append("measurement_source_matcher_result_id_missing")
    if measurement_id is not None and source_raw_scan_id is None:
        lineage_errors.append("measurement_source_raw_scan_id_missing")
    if applied:
        if measurement_id is None:
            lineage_errors.append("applied_measurement_id_missing")
        if ekf_input_measurement_id is None:
            lineage_errors.append("applied_ekf_input_measurement_id_missing")
        if ekf_applied_measurement_id is None:
            lineage_errors.append("applied_ekf_measurement_id_missing")
        if (
            measurement_id is not None
            and ekf_input_measurement_id is not None
            and measurement_id != ekf_input_measurement_id
        ):
            lineage_errors.append("applied_input_measurement_id_mismatch")
        if (
            ekf_input_measurement_id is not None
            and ekf_applied_measurement_id is not None
            and ekf_input_measurement_id != ekf_applied_measurement_id
        ):
            lineage_errors.append("applied_result_measurement_id_mismatch")
    return {
        "health": str((status or {}).get("lidar_health", "") or ""),
        "enabled": bool((status or {}).get("lidar_enabled", False)),
        "latest_age_s": latest_age,
        "latest_confidence": conf,
        "accepted": _safe_int(lidar_odom.get("accepted"), 0),
        "rejected_total": sum(
            _safe_int(value, 0)
            for key, value in lidar_odom.items()
            if str(key).startswith("rejected_")
        ),
        "applied": bool(applied),
        "observation": {
            "raw_scan_id": raw_scan_id,
            "matcher_result_id": matcher_result_id,
            "candidate_id": candidate_id,
            "lidar_odometry_measurement_id": measurement_id,
            "lidar_odometry_measurement_timestamp_s": measurement_timestamp,
            "candidate_source_raw_scan_id": candidate_source_raw_scan_id,
            "measurement_source_matcher_result_id": source_matcher_result_id,
            "measurement_source_raw_scan_id": source_raw_scan_id,
            "measurement_source_raw_scan_timestamp_s": source_raw_scan_timestamp,
            "ekf_input_lidar_odometry_measurement_id": ekf_input_measurement_id,
            "ekf_last_processed_lidar_odometry_measurement_id": ekf_processed_measurement_id,
            "ekf_last_applied_lidar_odometry_measurement_id": ekf_applied_measurement_id,
            "ekf_duplicate_measurement_rejected_total": _safe_int(
                lidar_odom.get("ekf_duplicate_measurement_rejected_total"), 0
            ),
            "ekf_missing_measurement_id_rejected_total": _safe_int(
                lidar_odom.get("ekf_missing_measurement_id_rejected_total"), 0
            ),
            "rejected_duplicate_matcher_result": _safe_int(
                lidar_odom.get("rejected_duplicate_matcher_result"), 0
            ),
            "rejected_duplicate_raw_scan": _safe_int(
                lidar_odom.get("rejected_duplicate_raw_scan"), 0
            ),
            "lineage_errors": list(lineage_errors),
        },
        "scan_count_filtered": _safe_int(
            lidar_odom.get("scan_count_filtered", lidar.get("scan_count_filtered")),
            0,
        ),
        "tracking_direction": {
            "checked": bool(lidar_odom.get("tracking_direction_checked", False)),
            "consistent": bool(lidar_odom.get("tracking_direction_consistent", True)),
            "rejected": bool(lidar_odom.get("tracking_direction_rejected", False)),
            "rejected_total": _safe_int(lidar_odom.get("tracking_direction_rejected_total"), 0),
            "reference_delta_m": _finite(lidar_odom.get("tracking_reference_delta_m")),
            "reference_linear_mps": _finite(lidar_odom.get("tracking_reference_linear_mps")),
            "candidate_projection_m": _finite(lidar_odom.get("tracking_candidate_projection_m")),
            "backtrack_debt_m": _finite(lidar_odom.get("tracking_backtrack_debt_m")),
            "reference_source": str(
                lidar_odom.get("tracking_direction_reference_source", "") or ""
            ),
            "backtrack_tolerance_m": _finite(
                lidar_odom.get("tracking_direction_backtrack_tolerance_m")
            ),
        },
        "pose": _extract_lidar_pose(status),
    }


def _extract_motion_command(status: Dict[str, Any]) -> Dict[str, Any]:
    command = dict((status or {}).get("motion_command") or {})
    requested = dict(command.get("requested_motion_intent") or {})
    limited = dict(command.get("limited_motion_intent") or {})
    tracks = dict(command.get("track_targets") or {})
    return {
        "command_type": str(command.get("command_type", "") or ""),
        "execution_mode": str(command.get("execution_mode", "") or ""),
        "source": str(command.get("source", (status or {}).get("motion_command_source", "")) or ""),
        "active_layer": str(command.get("active_layer", "") or ""),
        "requested_v": _finite(requested.get("v")),
        "requested_omega": _finite(requested.get("omega")),
        "limited_v": _finite(limited.get("v")),
        "limited_omega": _finite(limited.get("omega")),
        "track_left_mps": _finite(tracks.get("left_mps")),
        "track_right_mps": _finite(tracks.get("right_mps")),
        "safety_limiting_reason": str(command.get("safety_limiting_reason", "") or ""),
        "speed_limiting_reason": str(command.get("speed_limiting_reason", "") or ""),
    }


def _extract_runtime_diagnostics(status: Dict[str, Any]) -> Dict[str, Any]:
    status = dict(status or {})
    command = dict(status.get("motion_command") or {})
    motion_controller = dict(status.get("motion_controller") or {})
    motion_semantics = dict(status.get("motion_semantics") or {})
    control_monitor = dict(status.get("control_monitor") or {})
    pid_diag = dict(status.get("pid_diag") or {})
    global_policy = dict(status.get("global_motion_policy") or {})
    if not global_policy:
        global_policy = dict(command.get("global_motion_policy") or {})
    localization_gate = dict(status.get("localization_gate") or {})
    localization_apply = dict(localization_gate.get("apply") or {})
    primitive_contract = dict(status.get("primitive_contract") or {})
    left_feedforward = dict(pid_diag.get("left_feedforward") or {})
    right_feedforward = dict(pid_diag.get("right_feedforward") or {})
    left_startup = dict(pid_diag.get("left_startup") or {})
    right_startup = dict(pid_diag.get("right_startup") or {})
    return {
        "motion_controller": {
            "v_in": _finite(motion_controller.get("v_in")),
            "omega_in": _finite(motion_controller.get("omega_in")),
            "v_pre_limit": _finite(motion_controller.get("v_pre_limit")),
            "omega_pre_limit": _finite(motion_controller.get("omega_pre_limit")),
            "v_out": _finite(motion_controller.get("v_out")),
            "omega_out": _finite(motion_controller.get("omega_out")),
            "v_l_ref": _finite(motion_controller.get("v_l_ref")),
            "v_r_ref": _finite(motion_controller.get("v_r_ref")),
            "clamped": bool(motion_controller.get("clamped", False)),
            "limiter": str(motion_controller.get("limiter", "") or ""),
            "forward_dominant_policy_mode": str(motion_controller.get("forward_dominant_policy_mode", "") or ""),
            "forward_dominant_no_reverse": bool(motion_controller.get("forward_dominant_no_reverse", False)),
            "forward_dominant_policy_applied": bool(motion_controller.get("forward_dominant_policy_applied", False)),
            "forward_dominant_policy_actions": list(motion_controller.get("forward_dominant_policy_actions") or []),
            "reverse_guard_applied": bool(motion_controller.get("reverse_guard_applied", False)),
            "reverse_guard_reason": str(motion_controller.get("reverse_guard_reason", "") or ""),
            "explicit_motion_target_pivot": bool(motion_controller.get("explicit_motion_target_pivot", False)),
            "explicit_motion_target_arc": bool(motion_controller.get("explicit_motion_target_arc", False)),
            "explicit_arc_omega_scale": _finite(motion_controller.get("explicit_arc_omega_scale")),
        },
        "guidance": {
            "heading_correction_owner": str(motion_semantics.get("heading_hold_owner", "") or ""),
            "straight_hold_active": bool(motion_semantics.get("heading_hold_applied", False)),
            "straight_hold_correction_rad_s": _finite(
                motion_semantics.get("omega_target")
                if motion_semantics.get("heading_hold_applied", False)
                else 0.0
            ),
            "straight_hold_heading_error_deg": _finite(motion_semantics.get("heading_error_deg")),
            "straight_hold_mode": str(motion_semantics.get("heading_hold_mode", "") or ""),
        },
        "executor": {
            "mode": str(control_monitor.get("mode", pid_diag.get("control_mode", "")) or ""),
            "output_reason": str(control_monitor.get("output_reason", pid_diag.get("output_reason", "")) or ""),
            "feedforward_left": _finite(left_feedforward.get("feedforward_pwm")),
            "feedforward_right": _finite(right_feedforward.get("feedforward_pwm")),
            "pwm_executor_left": _finite(pid_diag.get("pwm_executor_l", control_monitor.get("pwm_executor_l"))),
            "pwm_executor_right": _finite(pid_diag.get("pwm_executor_r", control_monitor.get("pwm_executor_r"))),
            "wheel_loop_enabled": bool(pid_diag.get("wheel_pi_enabled", False)),
            "wheel_loop_effective_kp": _finite(
                pid_diag.get("wheel_pi_effective_kp", control_monitor.get("wheel_loop_effective_kp"))
            ),
            "wheel_loop_left_p": _finite(pid_diag.get("left_p_pwm", control_monitor.get("wheel_loop_left_p"))),
            "wheel_loop_right_p": _finite(pid_diag.get("right_p_pwm", control_monitor.get("wheel_loop_right_p"))),
            "wheel_loop_feedback_source": "encoder_canonical",
            "wheel_loop_left_output_reason": str(
                pid_diag.get("left_output_reason", "") or ""
            ),
            "wheel_loop_right_output_reason": str(
                pid_diag.get("right_output_reason", "") or ""
            ),
            "wheel_loop_left_maintenance_floor_pwm": _finite(left_startup.get("maintenance_pwm")),
            "wheel_loop_right_maintenance_floor_pwm": _finite(right_startup.get("maintenance_pwm")),
            "wheel_loop_left_maintenance_floor_applied": bool(left_startup.get("startup_floor_applied", False)),
            "wheel_loop_right_maintenance_floor_applied": bool(right_startup.get("startup_floor_applied", False)),
            "wheel_loop_left_ref_mps": _finite(pid_diag.get("left_reference_mps")),
            "wheel_loop_right_ref_mps": _finite(pid_diag.get("right_reference_mps")),
            "wheel_loop_left_meas_mps": _finite(pid_diag.get("left_measured_mps")),
            "wheel_loop_right_meas_mps": _finite(pid_diag.get("right_measured_mps")),
            "wheel_loop_left_error_mps": _finite(pid_diag.get("left_control_error_mps")),
            "wheel_loop_right_error_mps": _finite(pid_diag.get("right_control_error_mps")),
        },
        "global_motion_policy": {
            "active": bool(global_policy.get("active", False)),
            "bypassed": bool(global_policy.get("bypassed", False)),
            "bypass_reason": str(global_policy.get("bypass_reason", "") or ""),
            "actions": list(global_policy.get("actions") or []),
            "policy_state": str(global_policy.get("policy_state", "") or ""),
            "state_transition_reason": str(global_policy.get("state_transition_reason", "") or ""),
            "decision_reason": str(global_policy.get("decision_reason", "") or ""),
            "chosen_direction": str(global_policy.get("chosen_direction", "") or ""),
            "forward_clearance_m": _finite(global_policy.get("forward_clearance_m")),
            "predicted_clearance_m": _finite(global_policy.get("predicted_clearance_m")),
            "blocked_front": bool(global_policy.get("blocked_front", False)),
            "safety_stop_applied": bool(global_policy.get("safety_stop_applied", False)),
            "justified_reverse_allowed": bool(global_policy.get("justified_reverse_allowed", False)),
            "reverse_policy_exception": str(global_policy.get("reverse_policy_exception", "") or ""),
            "v_limit_mps": _finite(global_policy.get("v_limit_mps")),
            "omega_in": _finite(global_policy.get("omega_in")),
            "omega_out": _finite(global_policy.get("omega_out")),
        },
        "localization_gate": {
            "applied": bool(localization_apply.get("applied", False)),
            "reason": str(localization_apply.get("reason", "") or ""),
            "v_scale": _finite(localization_apply.get("v_scale")),
        },
        "primitive_contract": {
            "violation": bool(primitive_contract.get("violation", False)),
            "chain_match": bool(primitive_contract.get("chain_match", False)),
            "context": list(primitive_contract.get("context") or []),
        },
    }


def _sample_status(case_name: str, status: Dict[str, Any]) -> Dict[str, Any]:
    pose = _get_pose(status)
    encoder = _encoder_reading(status)
    encoder_status = dict((status or {}).get("encoder") or {})
    encoder_service = dict(encoder_status.get("service") or {})
    encoder_computed = dict(encoder_status.get("computed") or {})
    left_status = dict(encoder_status.get("left") or {})
    right_status = dict(encoder_status.get("right") or {})
    left_snapshot = dict(left_status.get("snapshot") or {})
    right_snapshot = dict(right_status.get("snapshot") or {})
    encoder_canonical = dict(encoder_status.get("canonical") or {})
    canonical_velocity = dict(encoder_canonical.get("canonical_velocity") or {})
    canonical_pulses = dict(encoder_canonical.get("pulses_delta") or {})
    gc_runtime = dict((status or {}).get("gc_runtime") or {})
    pwm = dict((status or {}).get("pwm") or {})
    safety = dict((status or {}).get("safety") or {})
    stop_status = dict((status or {}).get("stop_status") or {})
    last_emergency = dict((status or {}).get("last_emergency") or {})
    return {
        "ts": time.time(),
        "case": str(case_name),
        "status_version": _status_version(status),
        "state": _status_state(status),
        "pose": pose,
        "encoder": {
            "left_distance_m": float(encoder.get("left_distance_m", 0.0)),
            "right_distance_m": float(encoder.get("right_distance_m", 0.0)),
            "left_pulses": int(encoder.get("left_pulses", 0)),
            "right_pulses": int(encoder.get("right_pulses", 0)),
            "raw_counter_difference_right_minus_left": int(
                encoder.get("right_pulses", 0)
            )
            - int(encoder.get("left_pulses", 0)),
            "snapshot_pulses_delta": {
                "left": _safe_int(left_snapshot.get("pulse_delta"), 0),
                "right": _safe_int(right_snapshot.get("pulse_delta"), 0),
            },
            "driver_a_rising_trace": {
                "left_enabled": bool(left_status.get("edge_trace_enabled", False)),
                "right_enabled": bool(right_status.get("edge_trace_enabled", False)),
                "left": list(left_status.get("recent_a_rising_events") or []),
                "right": list(right_status.get("recent_a_rising_events") or []),
            },
            "step_distance_m": {
                "left": _finite(encoder_computed.get("step_distance_left_m")),
                "right": _finite(encoder_computed.get("step_distance_right_m")),
            },
            "measurement_timestamp_s": _finite(
                encoder_service.get("snapshot_ts_perf")
            ),
            "publication_timestamp_s": _finite(
                encoder_service.get("snapshot_published_ts_perf")
            ),
            "measurement_freshness_s": (
                None
                if _finite(encoder_service.get("snapshot_age_ms")) is None
                else float(encoder_service.get("snapshot_age_ms")) / 1000.0
            ),
            "publication_delay_s": (
                None
                if _finite(encoder_service.get("snapshot_publish_latency_ms")) is None
                else float(encoder_service.get("snapshot_publish_latency_ms")) / 1000.0
            ),
            "snapshot_health": str(encoder.get("snapshot_health", "") or ""),
            "canonical_velocity": {
                "left_mps": _finite(canonical_velocity.get("left_mps")),
                "right_mps": _finite(canonical_velocity.get("right_mps")),
            },
            "canonical_pulses_delta": {
                "left": _safe_int(canonical_pulses.get("left"), 0),
                "right": _safe_int(canonical_pulses.get("right"), 0),
                "left_control_window": _safe_int(
                    canonical_pulses.get("left_control_window"), 0
                ),
                "right_control_window": _safe_int(
                    canonical_pulses.get("right_control_window"), 0
                ),
                "left_instant": _safe_int(canonical_pulses.get("left_instant"), 0),
                "right_instant": _safe_int(canonical_pulses.get("right_instant"), 0),
                "dt_control_window_s": _finite(
                    canonical_pulses.get("dt_control_window_s")
                ),
                "dt_aggregation_window_s": _finite(
                    canonical_pulses.get("dt_aggregation_window_s")
                ),
                "window_start_ts": _finite(canonical_pulses.get("window_start_ts")),
                "window_end_ts": _finite(canonical_pulses.get("window_end_ts")),
                "left_count_start": _safe_int(
                    canonical_pulses.get("left_count_start"), 0
                ),
                "left_count_end": _safe_int(
                    canonical_pulses.get("left_count_end"), 0
                ),
                "right_count_start": _safe_int(
                    canonical_pulses.get("right_count_start"), 0
                ),
                "right_count_end": _safe_int(
                    canonical_pulses.get("right_count_end"), 0
                ),
            },
            "canonical_state": str(encoder_canonical.get("canonical_state", "") or ""),
            "canonical_flags": list(encoder_canonical.get("flags") or []),
            "canonical_trust": _finite(encoder_canonical.get("combined_trust")),
            "canonical_timing_valid": bool(encoder_canonical.get("timing_valid", True)),
            "canonical_timing_contract_present": bool(
                "timing_valid" in encoder_canonical
                and "timing_gap_count" in encoder_canonical
                and "motion_timing_gap_count" in encoder_canonical
                and "timing_gap_threshold_s" in encoder_canonical
            ),
            "canonical_timing_error": str(encoder_canonical.get("timing_error", "") or ""),
            "canonical_timing_gap_s": _finite(encoder_canonical.get("timing_gap_s")),
            "canonical_timing_gap_threshold_s": _finite(
                encoder_canonical.get("timing_gap_threshold_s")
            ),
            "canonical_timing_gap_count": _safe_int(
                encoder_canonical.get("timing_gap_count"), 0
            ),
            "canonical_motion_timing_gap_count": _safe_int(
                encoder_canonical.get("motion_timing_gap_count"), 0
            ),
            "canonical_idle_timing_gap_count": _safe_int(
                encoder_canonical.get("idle_timing_gap_count"), 0
            ),
            "canonical_last_timing_gap": dict(
                encoder_canonical.get("last_timing_gap") or {}
            ),
        },
        "gc_runtime": {
            "policy": str(gc_runtime.get("policy", "") or ""),
            "policy_source": str(gc_runtime.get("policy_source", "") or ""),
            "automatic_enabled": bool(gc_runtime.get("automatic_enabled", False)),
            "automatic_disabled_contract_ok": bool(
                gc_runtime.get("automatic_disabled_contract_ok", False)
            ),
            "collection_count": _safe_int(gc_runtime.get("collection_count"), 0),
            "authorized_collection_count": _safe_int(
                gc_runtime.get("authorized_collection_count"), 0
            ),
            "unowned_collection_count": _safe_int(
                gc_runtime.get("unowned_collection_count"), 0
            ),
            "motion_collection_count": _safe_int(
                gc_runtime.get("motion_collection_count"), 0
            ),
            "contract_violation_count": _safe_int(
                gc_runtime.get("contract_violation_count"), 0
            ),
            "fail_closed_active": bool(gc_runtime.get("fail_closed_active", False)),
            "last_collection": dict(gc_runtime.get("last_collection") or {}),
            "last_violation": dict(gc_runtime.get("last_violation") or {}),
        },
        "imu": _extract_imu_yaw_deg(status),
        "lidar": _extract_lidar_status(status),
        "motion_command": _extract_motion_command(status),
        "diagnostics": _extract_runtime_diagnostics(status),
        "pwm": {
            "left": _safe_float(pwm.get("left"), 0.0),
            "right": _safe_float(pwm.get("right"), 0.0),
        },
        "raw_velocity": {
            "left_mps": _safe_float((status or {}).get("v_l_raw"), 0.0),
            "right_mps": _safe_float((status or {}).get("v_r_raw"), 0.0),
        },
        "safety": {
            "allow": bool(safety.get("allow", True)),
            "reason": str(safety.get("reason", "") or ""),
        },
        "stop_status": {
            "type": str(stop_status.get("type", "") or ""),
            "reason": str(stop_status.get("reason", "") or ""),
        },
        "last_emergency": {
            "count": _safe_int(last_emergency.get("count"), 0),
            "reason": str(last_emergency.get("reason", "") or ""),
        },
    }


def _active_islands(flags: List[bool]) -> int:
    islands = 0
    previous = False
    for flag in flags:
        current = bool(flag)
        if current and not previous:
            islands += 1
        previous = current
    return int(islands)


def _stable_pwm_floors(case: MeasurementCase) -> Dict[str, Any]:
    directions = {
        "left": "forward" if case.v_mps >= 0.0 else "reverse",
        "right": "forward" if case.v_mps >= 0.0 else "reverse",
    }
    if not case.expected_linear_sign and case.expected_yaw_sign:
        directions = {
            "left": "reverse" if case.expected_yaw_sign > 0 else "forward",
            "right": "forward" if case.expected_yaw_sign > 0 else "reverse",
        }
    result = _read_json(LATEST_DEADZONE_RESULT_PATH) or {}
    curves = dict(result.get("curves") or {})
    floors: Dict[str, float] = {}
    sources: Dict[str, str] = {}
    for side in ("left", "right"):
        key = f"{side}_{directions[side]}"
        floor = _finite((curves.get(key) or {}).get("min_stable_pwm"))
        floors[side] = float(floor) if floor is not None else FALLBACK_STABLE_PWM_FLOOR
        sources[side] = "live_deadzone_calibration" if floor is not None else "fallback"
    return {"left": floors["left"], "right": floors["right"], "directions": directions, "sources": sources}


def _direction_changes(values: List[Any], epsilon: float) -> int:
    signs: List[int] = []
    for value in values:
        finite = _finite(value)
        if finite is None or abs(float(finite)) <= float(epsilon):
            continue
        signs.append(1 if float(finite) > 0.0 else -1)
    return sum(1 for previous, current in zip(signs, signs[1:]) if previous != current)


def _total_variation(values: List[Any]) -> float:
    finite_values = [float(value) for value in values if _finite(value) is not None]
    return sum(abs(current - previous) for previous, current in zip(finite_values, finite_values[1:]))


def _mean_finite(values: Iterable[Any]) -> Optional[float]:
    finite_values = [float(value) for value in values if _finite(value) is not None]
    return None if not finite_values else float(sum(finite_values) / len(finite_values))


def _measured_linear_speed_abs(sample: Dict[str, Any]) -> float:
    """Return only the persisted canonical count-window wheel-speed mean."""
    canonical = dict(((sample.get("encoder") or {}).get("canonical_velocity") or {}))
    left = _finite(canonical.get("left_mps"))
    right = _finite(canonical.get("right_mps"))
    if left is None or right is None:
        return math.nan
    return abs(0.5 * (float(left) + float(right)))


def _ratio(actual: Any, reference: Any) -> Optional[float]:
    actual_f = _finite(actual)
    reference_f = _finite(reference)
    if actual_f is None or reference_f is None or abs(float(reference_f)) <= 1e-6:
        return None
    return float(actual_f) / float(reference_f)


def _integrated_gyro_yaw_delta_deg(samples: List[Dict[str, Any]]) -> Optional[float]:
    if len(samples) < 2:
        return None
    total = 0.0
    previous_ts: Optional[float] = None
    previous_gyro: Optional[float] = None
    used = 0
    for sample in samples:
        ts = _finite(sample.get("ts"))
        gyro = _finite((sample.get("imu") or {}).get("gyro_z_dps"))
        if ts is None or gyro is None:
            continue
        if previous_ts is not None and previous_gyro is not None and ts >= previous_ts:
            total += float(previous_gyro) * max(0.0, float(ts) - float(previous_ts))
            used += 1
        previous_ts = float(ts)
        previous_gyro = float(gyro)
    if used <= 0:
        return None
    return float(total)


def _unwrapped_yaw_delta_deg(values: List[Any]) -> Optional[float]:
    finite_values = [float(value) for value in values if _finite(value) is not None]
    if len(finite_values) < 2:
        return None
    total = 0.0
    previous = float(finite_values[0])
    for current in finite_values[1:]:
        total += _normalize_angle_deg(float(current) - float(previous))
        previous = float(current)
    return float(total)


def _repeatability_pose_angle_abs(metrics: Dict[str, Any]) -> Optional[float]:
    """Use the EKF pose SSOT for cross-run and left/right angle comparison."""
    value = _finite((metrics.get("ekf") or {}).get("yaw_delta_deg"))
    return None if value is None else abs(float(value))


def _integrated_pose_forward_delta_m(samples: List[Dict[str, Any]]) -> Optional[float]:
    poses = [dict(sample.get("pose") or {}) for sample in samples]
    valid = [
        pose
        for pose in poses
        if _finite(pose.get("x")) is not None
        and _finite(pose.get("y")) is not None
        and _finite(pose.get("theta_deg")) is not None
    ]
    if len(valid) < 2:
        return None
    total = 0.0
    previous = valid[0]
    for current in valid[1:]:
        heading = math.radians(_safe_float(previous.get("theta_deg"), 0.0))
        dx = _safe_float(current.get("x"), 0.0) - _safe_float(previous.get("x"), 0.0)
        dy = _safe_float(current.get("y"), 0.0) - _safe_float(previous.get("y"), 0.0)
        total += float(dx) * math.cos(heading) + float(dy) * math.sin(heading)
        previous = current
    return float(total)


def _interpolate_endpoint_timeline(
    timeline: List[Dict[str, Any]],
    target_ts: float,
) -> Optional[Dict[str, Any]]:
    """Interpolate encoder/EKF state at one monotonic sensor timestamp."""

    if len(timeline) < 2:
        return None
    target = float(target_ts)
    for left, right in zip(timeline, timeline[1:]):
        left_ts = _finite(left.get("timestamp_s"))
        right_ts = _finite(right.get("timestamp_s"))
        if left_ts is None or right_ts is None:
            continue
        if target < float(left_ts) - 1e-9 or target > float(right_ts) + 1e-9:
            continue
        span_s = float(right_ts) - float(left_ts)
        if span_s <= 0.0 or span_s > MAX_SHARED_ENDPOINT_ENCODER_BRACKET_S:
            return None
        fraction = min(
            1.0,
            max(0.0, (target - float(left_ts)) / float(span_s)),
        )

        def lerp(key: str) -> Optional[float]:
            start = _finite(left.get(key))
            end = _finite(right.get(key))
            if start is None or end is None:
                return None
            return float(start) + fraction * (float(end) - float(start))

        theta_start = _finite(left.get("ekf_theta_deg"))
        theta_end = _finite(right.get("ekf_theta_deg"))
        theta_deg = None
        if theta_start is not None and theta_end is not None:
            theta_deg = float(theta_start) + fraction * _normalize_angle_deg(
                float(theta_end) - float(theta_start)
            )
        return {
            "timestamp_s": float(target),
            "bracket_start_timestamp_s": float(left_ts),
            "bracket_end_timestamp_s": float(right_ts),
            "bracket_span_s": float(span_s),
            "fraction": float(fraction),
            "left_pulses": lerp("left_pulses"),
            "right_pulses": lerp("right_pulses"),
            "left_distance_m": lerp("left_distance_m"),
            "right_distance_m": lerp("right_distance_m"),
            "left_step_distance_m": lerp("left_step_distance_m"),
            "right_step_distance_m": lerp("right_step_distance_m"),
            "ekf_pose": {
                "x": lerp("ekf_x"),
                "y": lerp("ekf_y"),
                "theta_deg": theta_deg,
                "theta": (
                    None if theta_deg is None else math.radians(float(theta_deg))
                ),
            },
        }
    return None


def _shared_sensor_endpoint_window(
    samples: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Build one physical endpoint interval for encoder, LiDAR and EKF.

    A LiDAR pose describes the source raw scan time, not the later validator
    poll or measurement-delivery time.  Accepted LiDAR source timestamps are
    therefore the interval boundaries; cumulative encoder counts and the EKF
    control pose are interpolated onto those exact monotonic timestamps.
    """

    unavailable = {
        "contract_id": SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID,
        "available": False,
        "failure_reason": "",
        "window_basis": "lidar.measurement_source_raw_scan_timestamp_s",
        "encoder_timeline_basis": "encoder.measurement_timestamp_s",
    }
    timeline_by_ts: Dict[float, Dict[str, Any]] = {}
    for sample in samples:
        encoder = dict(sample.get("encoder") or {})
        timestamp_s = _finite(encoder.get("measurement_timestamp_s"))
        left_pulses = _finite(encoder.get("left_pulses"))
        right_pulses = _finite(encoder.get("right_pulses"))
        step_distance = dict(encoder.get("step_distance_m") or {})
        left_step = _finite(step_distance.get("left"))
        right_step = _finite(step_distance.get("right"))
        if (
            timestamp_s is None
            or left_pulses is None
            or right_pulses is None
            or left_step is None
            or right_step is None
            or float(left_step) <= 0.0
            or float(right_step) <= 0.0
        ):
            continue
        pose = dict(sample.get("pose") or {})
        timeline_by_ts[float(timestamp_s)] = {
            "timestamp_s": float(timestamp_s),
            "left_pulses": float(left_pulses),
            "right_pulses": float(right_pulses),
            "left_distance_m": _finite(encoder.get("left_distance_m")),
            "right_distance_m": _finite(encoder.get("right_distance_m")),
            "left_step_distance_m": float(left_step),
            "right_step_distance_m": float(right_step),
            "ekf_x": _finite(pose.get("x")),
            "ekf_y": _finite(pose.get("y")),
            "ekf_theta_deg": _finite(pose.get("theta_deg")),
        }
    timeline = sorted(timeline_by_ts.values(), key=lambda item: item["timestamp_s"])
    if len(timeline) < 2:
        return {
            **unavailable,
            "failure_reason": "encoder_timeline_insufficient",
            "encoder_timeline_points": int(len(timeline)),
        }

    anchors_by_id: Dict[int, Dict[str, Any]] = {}
    anchor_contract_errors: List[str] = []
    for sample in samples:
        lidar = dict(sample.get("lidar") or {})
        observation = dict(lidar.get("observation") or {})
        measurement_id = _positive_int_or_none(
            observation.get("lidar_odometry_measurement_id")
        )
        source_timestamp_s = _finite(
            observation.get("measurement_source_raw_scan_timestamp_s")
        )
        pose = dict(lidar.get("pose") or {})
        if measurement_id is None or source_timestamp_s is None:
            continue
        if (
            not bool(pose.get("available", False))
            or _finite(pose.get("x")) is None
            or _finite(pose.get("y")) is None
        ):
            continue
        anchor = {
            "measurement_id": int(measurement_id),
            "measurement_timestamp_s": _finite(
                observation.get("lidar_odometry_measurement_timestamp_s")
            ),
            "source_raw_scan_id": _positive_int_or_none(
                observation.get("measurement_source_raw_scan_id")
            ),
            "source_timestamp_s": float(source_timestamp_s),
            "pose": {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "theta": _finite(pose.get("theta")),
                "theta_deg": _finite(pose.get("theta_deg")),
                "source": str(pose.get("source", "") or ""),
            },
        }
        previous = anchors_by_id.get(int(measurement_id))
        if previous is not None:
            if previous != anchor:
                anchor_contract_errors.append(
                    f"measurement_id_reused_with_changed_endpoint:{measurement_id}"
                )
            continue
        anchors_by_id[int(measurement_id)] = anchor
    if anchor_contract_errors:
        return {
            **unavailable,
            "failure_reason": "lidar_endpoint_contract_error",
            "contract_errors": list(dict.fromkeys(anchor_contract_errors)),
        }

    aligned_anchors: List[Dict[str, Any]] = []
    for anchor in sorted(
        anchors_by_id.values(),
        key=lambda item: (item["source_timestamp_s"], item["measurement_id"]),
    ):
        interpolated = _interpolate_endpoint_timeline(
            timeline,
            float(anchor["source_timestamp_s"]),
        )
        if interpolated is not None:
            aligned_anchors.append({**anchor, "interpolated": interpolated})
    if len(aligned_anchors) < 2:
        return {
            **unavailable,
            "failure_reason": "shared_lidar_encoder_interval_insufficient",
            "accepted_lidar_anchor_count": int(len(anchors_by_id)),
            "aligned_lidar_anchor_count": int(len(aligned_anchors)),
            "encoder_timeline_points": int(len(timeline)),
            "encoder_timeline_start_s": float(timeline[0]["timestamp_s"]),
            "encoder_timeline_end_s": float(timeline[-1]["timestamp_s"]),
        }

    start_anchor = aligned_anchors[0]
    end_anchor = aligned_anchors[-1]
    start_ts = float(start_anchor["source_timestamp_s"])
    end_ts = float(end_anchor["source_timestamp_s"])
    if (
        int(end_anchor["measurement_id"]) <= int(start_anchor["measurement_id"])
        or end_ts <= start_ts
    ):
        return {
            **unavailable,
            "failure_reason": "shared_interval_not_progressing",
        }

    encoder_start = dict(start_anchor["interpolated"])
    encoder_end = dict(end_anchor["interpolated"])
    left_pulse_delta = _safe_float(encoder_end.get("left_pulses"), 0.0) - _safe_float(
        encoder_start.get("left_pulses"), 0.0
    )
    right_pulse_delta = _safe_float(
        encoder_end.get("right_pulses"), 0.0
    ) - _safe_float(encoder_start.get("right_pulses"), 0.0)
    left_step = 0.5 * (
        _safe_float(encoder_start.get("left_step_distance_m"), 0.0)
        + _safe_float(encoder_end.get("left_step_distance_m"), 0.0)
    )
    right_step = 0.5 * (
        _safe_float(encoder_start.get("right_step_distance_m"), 0.0)
        + _safe_float(encoder_end.get("right_step_distance_m"), 0.0)
    )
    left_delta_m = float(left_pulse_delta) * float(left_step)
    right_delta_m = float(right_pulse_delta) * float(right_step)
    encoder_average_delta_m = 0.5 * (float(left_delta_m) + float(right_delta_m))

    lidar_start_pose = dict(start_anchor["pose"])
    lidar_end_pose = dict(end_anchor["pose"])
    lidar_dx = _safe_float(lidar_end_pose.get("x"), 0.0) - _safe_float(
        lidar_start_pose.get("x"), 0.0
    )
    lidar_dy = _safe_float(lidar_end_pose.get("y"), 0.0) - _safe_float(
        lidar_start_pose.get("y"), 0.0
    )
    lidar_chord_m = math.hypot(float(lidar_dx), float(lidar_dy))

    ekf_start_pose = dict(encoder_start.get("ekf_pose") or {})
    ekf_end_pose = dict(encoder_end.get("ekf_pose") or {})
    ekf_dx = (
        None
        if _finite(ekf_start_pose.get("x")) is None
        or _finite(ekf_end_pose.get("x")) is None
        else float(ekf_end_pose["x"]) - float(ekf_start_pose["x"])
    )
    ekf_dy = (
        None
        if _finite(ekf_start_pose.get("y")) is None
        or _finite(ekf_end_pose.get("y")) is None
        else float(ekf_end_pose["y"]) - float(ekf_start_pose["y"])
    )
    ekf_chord_m = (
        None
        if ekf_dx is None or ekf_dy is None
        else math.hypot(float(ekf_dx), float(ekf_dy))
    )
    ekf_forward_delta_m = None
    start_heading_deg = _finite(ekf_start_pose.get("theta_deg"))
    if ekf_dx is not None and ekf_dy is not None and start_heading_deg is not None:
        start_heading = math.radians(float(start_heading_deg))
        ekf_forward_delta_m = (
            float(ekf_dx) * math.cos(start_heading)
            + float(ekf_dy) * math.sin(start_heading)
        )

    interpolated_left_distance_delta_m = (
        None
        if _finite(encoder_start.get("left_distance_m")) is None
        or _finite(encoder_end.get("left_distance_m")) is None
        else float(encoder_end["left_distance_m"])
        - float(encoder_start["left_distance_m"])
    )
    interpolated_right_distance_delta_m = (
        None
        if _finite(encoder_start.get("right_distance_m")) is None
        or _finite(encoder_end.get("right_distance_m")) is None
        else float(encoder_end["right_distance_m"])
        - float(encoder_start["right_distance_m"])
    )
    return {
        "contract_id": SENSOR_ENDPOINT_SHARED_WINDOW_CONTRACT_ID,
        "available": True,
        "failure_reason": "",
        "window_basis": "lidar.measurement_source_raw_scan_timestamp_s",
        "encoder_timeline_basis": "encoder.measurement_timestamp_s",
        "start_timestamp_s": float(start_ts),
        "end_timestamp_s": float(end_ts),
        "duration_s": float(end_ts - start_ts),
        "accepted_lidar_anchor_count": int(len(anchors_by_id)),
        "aligned_lidar_anchor_count": int(len(aligned_anchors)),
        "encoder_timeline_points": int(len(timeline)),
        "lidar": {
            "accepted_measurement_id_start": int(start_anchor["measurement_id"]),
            "accepted_measurement_id_end": int(end_anchor["measurement_id"]),
            "source_raw_scan_id_start": start_anchor.get("source_raw_scan_id"),
            "source_raw_scan_id_end": end_anchor.get("source_raw_scan_id"),
            "measurement_timestamp_start_s": start_anchor.get(
                "measurement_timestamp_s"
            ),
            "measurement_timestamp_end_s": end_anchor.get(
                "measurement_timestamp_s"
            ),
            "pose_start": lidar_start_pose,
            "pose_end": lidar_end_pose,
            "pose_dx_m": float(lidar_dx),
            "pose_dy_m": float(lidar_dy),
            "pose_chord_m": float(lidar_chord_m),
        },
        "encoder": {
            "timestamp_start_s": float(start_ts),
            "timestamp_end_s": float(end_ts),
            "left_pulses_start_interpolated": encoder_start.get("left_pulses"),
            "left_pulses_end_interpolated": encoder_end.get("left_pulses"),
            "right_pulses_start_interpolated": encoder_start.get("right_pulses"),
            "right_pulses_end_interpolated": encoder_end.get("right_pulses"),
            "left_signed_pulse_delta": float(left_pulse_delta),
            "right_signed_pulse_delta": float(right_pulse_delta),
            "step_distance_m": {
                "left": float(left_step),
                "right": float(right_step),
            },
            "left_delta_m": float(left_delta_m),
            "right_delta_m": float(right_delta_m),
            "average_delta_m": float(encoder_average_delta_m),
            "interpolated_distance_delta_m": {
                "left": interpolated_left_distance_delta_m,
                "right": interpolated_right_distance_delta_m,
            },
            "meter_conversion_residual_m": {
                "left": (
                    None
                    if interpolated_left_distance_delta_m is None
                    else float(interpolated_left_distance_delta_m)
                    - float(left_delta_m)
                ),
                "right": (
                    None
                    if interpolated_right_distance_delta_m is None
                    else float(interpolated_right_distance_delta_m)
                    - float(right_delta_m)
                ),
            },
            "start_interpolation": {
                key: encoder_start.get(key)
                for key in (
                    "bracket_start_timestamp_s",
                    "bracket_end_timestamp_s",
                    "bracket_span_s",
                    "fraction",
                )
            },
            "end_interpolation": {
                key: encoder_end.get(key)
                for key in (
                    "bracket_start_timestamp_s",
                    "bracket_end_timestamp_s",
                    "bracket_span_s",
                    "fraction",
                )
            },
        },
        "ekf_control": {
            "timestamp_start_s": float(start_ts),
            "timestamp_end_s": float(end_ts),
            "pose_start": ekf_start_pose,
            "pose_end": ekf_end_pose,
            "chord_m": ekf_chord_m,
            "forward_delta_m": ekf_forward_delta_m,
        },
        "ratios": {
            "lidar_chord_vs_encoder": (
                None
                if abs(float(encoder_average_delta_m)) <= 1e-9
                else abs(float(lidar_chord_m))
                / abs(float(encoder_average_delta_m))
            ),
            "ekf_forward_vs_encoder": (
                None
                if ekf_forward_delta_m is None
                or abs(float(encoder_average_delta_m)) <= 1e-9
                else abs(float(ekf_forward_delta_m))
                / abs(float(encoder_average_delta_m))
            ),
        },
    }


def _summarize_samples(case: MeasurementCase, samples: List[Dict[str, Any]], stop_outcome: Dict[str, Any]) -> Dict[str, Any]:
    start = dict(samples[0]) if samples else {}
    end = dict(samples[-1]) if samples else start
    start_pose = dict(start.get("pose") or {})
    end_pose = dict(end.get("pose") or {})
    start_encoder = dict(start.get("encoder") or {})
    end_encoder = dict(end.get("encoder") or {})
    start_imu = dict(start.get("imu") or {})
    end_imu = dict(end.get("imu") or {})
    start_lidar = dict(start.get("lidar") or {})
    end_lidar = dict(end.get("lidar") or {})
    start_lidar_pose = dict(start_lidar.get("pose") or {})
    end_lidar_pose = dict(end_lidar.get("pose") or {})
    shared_sensor_endpoint = _shared_sensor_endpoint_window(samples)

    left_delta = _safe_float(end_encoder.get("left_distance_m"), 0.0) - _safe_float(start_encoder.get("left_distance_m"), 0.0)
    right_delta = _safe_float(end_encoder.get("right_distance_m"), 0.0) - _safe_float(start_encoder.get("right_distance_m"), 0.0)
    encoder_avg = 0.5 * (float(left_delta) + float(right_delta))
    encoder_diff = float(right_delta) - float(left_delta)

    heading0 = math.radians(_safe_float(start_pose.get("theta_deg"), 0.0))
    dx = _safe_float(end_pose.get("x"), 0.0) - _safe_float(start_pose.get("x"), 0.0)
    dy = _safe_float(end_pose.get("y"), 0.0) - _safe_float(start_pose.get("y"), 0.0)
    ekf_forward_endpoint_delta = float(dx) * math.cos(heading0) + float(dy) * math.sin(heading0)
    ekf_forward_delta = _integrated_pose_forward_delta_m(samples)
    if ekf_forward_delta is None:
        ekf_forward_delta = float(ekf_forward_endpoint_delta)
    ekf_yaw_endpoint_delta = _normalize_angle_deg(
        _safe_float(end_pose.get("theta_deg"), 0.0) - _safe_float(start_pose.get("theta_deg"), 0.0)
    )
    ekf_yaw_delta = _unwrapped_yaw_delta_deg(
        [(sample.get("pose") or {}).get("theta_deg") for sample in samples]
    )
    if ekf_yaw_delta is None:
        ekf_yaw_delta = float(ekf_yaw_endpoint_delta)

    imu_yaw_delta = None
    if bool(start_imu.get("available", False)) and bool(end_imu.get("available", False)):
        imu_yaw_delta = _normalize_angle_deg(
            _safe_float(end_imu.get("yaw_deg"), 0.0) - _safe_float(start_imu.get("yaw_deg"), 0.0)
        )
    gyro_yaw_delta = _integrated_gyro_yaw_delta_deg(samples)
    selected_imu_yaw_delta = gyro_yaw_delta if gyro_yaw_delta is not None else imu_yaw_delta

    lidar_dx = None
    lidar_dy = None
    lidar_chord = None
    lidar_yaw_delta = None
    if bool(start_lidar_pose.get("available", False)) and bool(end_lidar_pose.get("available", False)):
        lidar_dx = _safe_float(end_lidar_pose.get("x"), 0.0) - _safe_float(start_lidar_pose.get("x"), 0.0)
        lidar_dy = _safe_float(end_lidar_pose.get("y"), 0.0) - _safe_float(start_lidar_pose.get("y"), 0.0)
        lidar_chord = math.hypot(float(lidar_dx), float(lidar_dy))
        start_lidar_theta = _finite(start_lidar_pose.get("theta_deg"))
        end_lidar_theta = _finite(end_lidar_pose.get("theta_deg"))
        if start_lidar_theta is not None and end_lidar_theta is not None:
            lidar_yaw_delta = _unwrapped_yaw_delta_deg(
                [((sample.get("lidar") or {}).get("pose") or {}).get("theta_deg") for sample in samples]
            )
            if lidar_yaw_delta is None:
                lidar_yaw_delta = _normalize_angle_deg(float(end_lidar_theta) - float(start_lidar_theta))

    pwm_left = [_safe_float((sample.get("pwm") or {}).get("left"), 0.0) for sample in samples]
    pwm_right = [_safe_float((sample.get("pwm") or {}).get("right"), 0.0) for sample in samples]
    pwm_asymmetry = [abs(float(left) - float(right)) for left, right in zip(pwm_left, pwm_right)]
    pwm_active = [
        max(abs(float(left)), abs(float(right))) >= MIN_PWM_OBSERVED
        for left, right in zip(pwm_left, pwm_right)
    ]
    stable_pwm_floors = _stable_pwm_floors(case)
    pwm_side_active_count = 0
    pwm_near_stable_floor_count = 0
    pwm_below_stable_floor_count = 0
    for left, right in zip(pwm_left, pwm_right):
        for side, value in (("left", left), ("right", right)):
            magnitude = abs(float(value))
            if magnitude < MIN_PWM_OBSERVED:
                continue
            pwm_side_active_count += 1
            floor = float(stable_pwm_floors[side])
            if magnitude <= floor + STABLE_PWM_FLOOR_MARGIN:
                pwm_near_stable_floor_count += 1
            if magnitude < max(MIN_PWM_OBSERVED, floor - STABLE_PWM_FLOOR_MARGIN):
                pwm_below_stable_floor_count += 1
    one_side_pwm_active = [
        (abs(float(left)) >= MIN_PWM_OBSERVED) != (abs(float(right)) >= MIN_PWM_OBSERVED)
        for left, right in zip(pwm_left, pwm_right)
    ]
    opposing_pwm_active = [
        (float(left) * float(right)) < 0.0
        and max(abs(float(left)), abs(float(right))) >= MIN_PWM_OBSERVED
        for left, right in zip(pwm_left, pwm_right)
    ]
    velocity_active = [
        max(
            abs(_safe_float((sample.get("raw_velocity") or {}).get("left_mps"), 0.0)),
            abs(_safe_float((sample.get("raw_velocity") or {}).get("right_mps"), 0.0)),
        )
        >= 0.02
        for sample in samples
    ]
    safety_reasons = [
        str((sample.get("safety") or {}).get("reason", "") or "")
        for sample in samples
        if not bool((sample.get("safety") or {}).get("allow", True))
    ]
    stop_reasons = [
        str((sample.get("stop_status") or {}).get("reason", "") or "")
        for sample in samples
        if str((sample.get("stop_status") or {}).get("reason", "") or "")
    ]
    command_types = Counter(
        str((sample.get("motion_command") or {}).get("command_type", "") or "").strip().lower()
        for sample in samples
    )
    policy_modes = Counter(
        str(
            (((sample.get("diagnostics") or {}).get("motion_controller") or {}).get("forward_dominant_policy_mode", ""))
            or ""
        ).strip()
        for sample in samples
    )
    executor_reasons = Counter(
        str((((sample.get("diagnostics") or {}).get("executor") or {}).get("output_reason", "")) or "").strip()
        for sample in samples
    )
    heading_owners = Counter(
        str((((sample.get("diagnostics") or {}).get("guidance") or {}).get("heading_correction_owner", "")) or "").strip()
        for idx, sample in enumerate(samples)
        if bool(pwm_active[idx])
    )
    wheel_left_reasons = Counter(
        str((((sample.get("diagnostics") or {}).get("executor") or {}).get("wheel_loop_left_output_reason", "")) or "").strip()
        for sample in samples
    )
    wheel_right_reasons = Counter(
        str((((sample.get("diagnostics") or {}).get("executor") or {}).get("wheel_loop_right_output_reason", "")) or "").strip()
        for sample in samples
    )
    policy_null_samples = 0
    executor_zero_cmd_samples = 0
    trailing_zero_grace_samples = 0
    first_pwm_active_idx = next((idx for idx, active in enumerate(pwm_active) if bool(active)), None)
    first_sample_ts = _finite((samples[0] if samples else {}).get("ts"))
    trailing_grace_s = 0.35
    for idx, sample in enumerate(samples):
        motion_command = dict(sample.get("motion_command") or {})
        diagnostics = dict(sample.get("diagnostics") or {})
        controller_diag = dict(diagnostics.get("motion_controller") or {})
        executor_diag = dict(diagnostics.get("executor") or {})
        steady_after_start = bool(
            (idx >= 3)
            if first_pwm_active_idx is None
            else (idx > int(first_pwm_active_idx) + 1)
        )
        sample_ts = _finite(sample.get("ts"))
        elapsed_s = None
        if first_sample_ts is not None and sample_ts is not None:
            elapsed_s = max(0.0, float(sample_ts) - float(first_sample_ts))
        executor_zero_reason = str(executor_diag.get("output_reason", "") or "").upper()
        safety_limiting_reason = str(motion_command.get("safety_limiting_reason", "") or "")
        trailing_zero_grace = bool(
            elapsed_s is not None
            and elapsed_s >= max(0.0, float(case.duration_s) - trailing_grace_s)
            and not safety_limiting_reason
            and executor_zero_reason == "ZERO_TARGET"
        )
        if trailing_zero_grace:
            trailing_zero_grace_samples += 1
        requested_motion_abs = max(
            abs(_safe_float(motion_command.get("requested_v"), 0.0)),
            abs(_safe_float(motion_command.get("requested_omega"), 0.0)),
        )
        controller_out_abs = max(
            abs(_safe_float(controller_diag.get("v_out"), _safe_float(motion_command.get("limited_v"), 0.0))),
            abs(_safe_float(controller_diag.get("omega_out"), _safe_float(motion_command.get("limited_omega"), 0.0))),
        )
        if (
            bool(case.command_motion)
            and steady_after_start
            and not trailing_zero_grace
            and requested_motion_abs > 1e-6
            and controller_out_abs <= 1e-6
        ):
            policy_null_samples += 1
        if (
            bool(case.command_motion)
            and steady_after_start
            and not trailing_zero_grace
            and requested_motion_abs > 1e-6
            and executor_zero_reason == "ZERO_TARGET"
        ):
            executor_zero_cmd_samples += 1
    lidar_ages = [
        float((sample.get("lidar") or {}).get("latest_age_s"))
        for sample in samples
        if _finite((sample.get("lidar") or {}).get("latest_age_s")) is not None
    ]
    lidar_conf_poll = [
        float((sample.get("lidar") or {}).get("latest_confidence"))
        for sample in samples
        if _finite((sample.get("lidar") or {}).get("latest_confidence")) is not None
    ]
    measurement_observations: Dict[int, Dict[str, Any]] = {}
    applied_measurement_ids = set()
    applied_status_samples = 0
    applied_missing_measurement_id_samples = 0
    raw_scan_ids = set()
    matcher_result_ids = set()
    observation_contract_errors: List[str] = []
    for sample in samples:
        lidar_sample = dict(sample.get("lidar") or {})
        observation = dict(lidar_sample.get("observation") or {})
        raw_scan_id = _positive_int_or_none(observation.get("raw_scan_id"))
        matcher_result_id = _positive_int_or_none(observation.get("matcher_result_id"))
        measurement_id = _positive_int_or_none(
            observation.get("lidar_odometry_measurement_id")
        )
        if raw_scan_id is not None:
            raw_scan_ids.add(int(raw_scan_id))
        if matcher_result_id is not None:
            matcher_result_ids.add(int(matcher_result_id))
        observation_contract_errors.extend(
            str(error)
            for error in list(observation.get("lineage_errors") or [])
            if str(error)
        )
        if measurement_id is not None:
            signature = {
                "measurement_source_matcher_result_id": _positive_int_or_none(
                    observation.get("measurement_source_matcher_result_id")
                ),
                "measurement_source_raw_scan_id": _positive_int_or_none(
                    observation.get("measurement_source_raw_scan_id")
                ),
                "measurement_source_raw_scan_timestamp_s": _finite(
                    observation.get("measurement_source_raw_scan_timestamp_s")
                ),
                "lidar_odometry_measurement_timestamp_s": _finite(
                    observation.get("lidar_odometry_measurement_timestamp_s")
                ),
                "pose": dict(lidar_sample.get("pose") or {}),
                "latest_confidence": _finite(lidar_sample.get("latest_confidence")),
            }
            previous = measurement_observations.get(int(measurement_id))
            if previous is not None and previous != signature:
                observation_contract_errors.append(
                    f"measurement_id_reused_with_changed_payload:{measurement_id}"
                )
            else:
                measurement_observations[int(measurement_id)] = signature
        if bool(lidar_sample.get("applied", False)):
            applied_status_samples += 1
            ekf_input_id = _positive_int_or_none(
                observation.get("ekf_input_lidar_odometry_measurement_id")
            )
            applied_id = ekf_input_id or measurement_id
            if applied_id is None:
                applied_missing_measurement_id_samples += 1
            else:
                applied_measurement_ids.add(int(applied_id))
    lidar_conf = [
        float(signature["latest_confidence"])
        for signature in measurement_observations.values()
        if _finite(signature.get("latest_confidence")) is not None
    ]

    emergency_start = _safe_int((start.get("last_emergency") or {}).get("count"), 0)
    emergency_end = _safe_int((end.get("last_emergency") or {}).get("count"), emergency_start)
    stop_status = dict((stop_outcome or {}).get("status") or {})
    stop_encoder_canonical = dict(
        (((stop_status.get("encoder") or {}).get("canonical")) or {})
    )
    stop_gc_runtime = dict(stop_status.get("gc_runtime") or {})
    timing_gap_start = _safe_int(start_encoder.get("canonical_timing_gap_count"), 0)
    timing_gap_end = max(
        _safe_int(end_encoder.get("canonical_timing_gap_count"), timing_gap_start),
        _safe_int(stop_encoder_canonical.get("timing_gap_count"), timing_gap_start),
    )
    motion_timing_gap_start = _safe_int(
        start_encoder.get("canonical_motion_timing_gap_count"), 0
    )
    motion_timing_gap_end = max(
        _safe_int(
            end_encoder.get("canonical_motion_timing_gap_count"),
            motion_timing_gap_start,
        ),
        _safe_int(
            stop_encoder_canonical.get("motion_timing_gap_count"),
            motion_timing_gap_start,
        ),
    )
    idle_timing_gap_start = _safe_int(
        start_encoder.get("canonical_idle_timing_gap_count"), 0
    )
    idle_timing_gap_end = max(
        _safe_int(
            end_encoder.get("canonical_idle_timing_gap_count"),
            idle_timing_gap_start,
        ),
        _safe_int(
            stop_encoder_canonical.get("idle_timing_gap_count"),
            idle_timing_gap_start,
        ),
    )
    timing_gap_samples = [
        sample
        for sample in samples
        if not bool((sample.get("encoder") or {}).get("canonical_timing_valid", True))
        and str((sample.get("encoder") or {}).get("canonical_timing_error", "") or "")
        == "TIMING_GAP"
    ]
    start_gc_runtime = dict(start.get("gc_runtime") or {})
    end_gc_runtime = dict(end.get("gc_runtime") or {})
    gc_motion_start = _safe_int(start_gc_runtime.get("motion_collection_count"), 0)
    gc_motion_end = max(
        _safe_int(end_gc_runtime.get("motion_collection_count"), gc_motion_start),
        _safe_int(stop_gc_runtime.get("motion_collection_count"), gc_motion_start),
    )
    gc_unowned_start = _safe_int(start_gc_runtime.get("unowned_collection_count"), 0)
    gc_unowned_end = max(
        _safe_int(end_gc_runtime.get("unowned_collection_count"), gc_unowned_start),
        _safe_int(stop_gc_runtime.get("unowned_collection_count"), gc_unowned_start),
    )
    final_gc_runtime = (
        stop_gc_runtime
        if _safe_int(stop_gc_runtime.get("collection_count"), 0)
        >= _safe_int(end_gc_runtime.get("collection_count"), 0)
        else end_gc_runtime
    )
    normal_stop_confirmed = bool(stop_outcome.get("normal_stop_confirmed", False))

    active_indices = [idx for idx, active in enumerate(pwm_active) if bool(active)]
    guidance_omega_active = [
        _finite((((samples[idx].get("diagnostics") or {}).get("guidance") or {}).get("straight_hold_correction_rad_s")))
        for idx in active_indices
    ]
    straight_hold_correction_active = [
        _finite(
            (((samples[idx].get("diagnostics") or {}).get("guidance") or {}).get("straight_hold_correction_rad_s"))
        )
        for idx in active_indices
    ]
    pwm_difference_active = [
        float(pwm_right[idx]) - float(pwm_left[idx])
        for idx in active_indices
    ]
    active_duration_s = 0.0
    if len(active_indices) >= 2:
        first_active_ts = _finite(samples[active_indices[0]].get("ts"))
        last_active_ts = _finite(samples[active_indices[-1]].get("ts"))
        if first_active_ts is not None and last_active_ts is not None:
            active_duration_s = max(0.0, float(last_active_ts) - float(first_active_ts))
    guard_intervention_samples = sum(
        1
        for idx in active_indices
        if bool(
            (((samples[idx].get("diagnostics") or {}).get("executor") or {}).get("forward_dominant_guard_applied", False))
            or (((samples[idx].get("diagnostics") or {}).get("executor") or {}).get("track_direction_guard_applied", False))
        )
    )
    active_executor = [
        dict(((samples[idx].get("diagnostics") or {}).get("executor") or {}))
        for idx in active_indices
    ]
    executed_v_mean = _mean_finite(item.get("v_cmd") for item in active_executor)
    executed_omega_mean = _mean_finite(item.get("omega_cmd") for item in active_executor)
    requested_distance_m = float(case.v_mps) * float(case.duration_s)
    executed_distance_m = _integrate_timed_samples(
        samples,
        active_indices,
        lambda sample: ((sample.get("diagnostics") or {}).get("executor") or {}).get("v_cmd"),
    )
    requested_angle_rad = (
        math.radians(float(case.target_angle_deg))
        if case.target_angle_deg is not None
        else float(case.omega_rad_s) * float(case.duration_s)
    )
    executed_angle_rad = _integrate_timed_samples(
        samples,
        active_indices,
        lambda sample: ((sample.get("diagnostics") or {}).get("executor") or {}).get("omega_cmd"),
    )
    duration_for_rate = max(1e-6, float(active_duration_s or case.duration_s))
    encoder_linear_mps = float(encoder_avg) / duration_for_rate
    whole_phase_duration_s = max(1e-6, float(case.duration_s))
    whole_phase_encoder_linear_mps = float(encoder_avg) / whole_phase_duration_s
    imu_omega_rad_s = (
        None
        if selected_imu_yaw_delta is None
        else math.radians(float(selected_imu_yaw_delta)) / duration_for_rate
    )
    ekf_omega_rad_s = math.radians(float(ekf_yaw_delta)) / duration_for_rate
    lidar_omega_rad_s = (
        None
        if lidar_yaw_delta is None
        else math.radians(float(lidar_yaw_delta)) / duration_for_rate
    )
    wheel_errors: List[float] = []
    for idx in active_indices:
        executor = dict(((samples[idx].get("diagnostics") or {}).get("executor") or {}))
        canonical = dict(((samples[idx].get("encoder") or {}).get("canonical_velocity") or {}))
        for reference_key, measured_key in (
            ("wheel_loop_left_ref_mps", "left_mps"),
            ("wheel_loop_right_ref_mps", "right_mps"),
        ):
            reference = _finite(executor.get(reference_key))
            measured = _finite(canonical.get(measured_key))
            if reference is not None and measured is not None:
                wheel_errors.append(abs(float(reference) - float(measured)))
    first_active_ts = (
        _finite(samples[active_indices[0]].get("ts"))
        if active_indices
        else None
    )
    transition_indices: List[int] = []
    settled_indices: List[int] = []
    caster_transient_indices: List[int] = []
    post_caster_transient_indices: List[int] = []
    stop_phase_indices: List[int] = []
    onset_indices = list(range(0, (active_indices[0] + 1) if active_indices else len(samples)))
    caster_transient_s = max(0.0, float(case.caster_transient_s))
    if first_active_ts is not None:
        for idx, sample in enumerate(samples):
            sample_ts = _finite(sample.get("ts"))
            if sample_ts is None:
                continue
            elapsed_from_motion_start_s = float(sample_ts) - float(first_active_ts)
            if bool(pwm_active[idx]):
                if elapsed_from_motion_start_s < float(SETTLED_PHASE_START_S):
                    transition_indices.append(idx)
                else:
                    settled_indices.append(idx)
                if caster_transient_s > 0.0:
                    if elapsed_from_motion_start_s < caster_transient_s:
                        caster_transient_indices.append(idx)
                    else:
                        post_caster_transient_indices.append(idx)
            elif elapsed_from_motion_start_s >= 0.0:
                stop_phase_indices.append(idx)

    def _wheel_phase_metrics(indices: List[int]) -> Dict[str, Any]:
        left_errors: List[float] = []
        right_errors: List[float] = []
        independent_left_errors: List[float] = []
        independent_right_errors: List[float] = []
        seen_feedback_windows = set()
        for idx in indices:
            executor = dict(((samples[idx].get("diagnostics") or {}).get("executor") or {}))
            encoder = dict(samples[idx].get("encoder") or {})
            canonical = dict(encoder.get("canonical_velocity") or {})
            pulses = dict(encoder.get("canonical_pulses_delta") or {})
            left_ref = _finite(executor.get("wheel_loop_left_ref_mps"))
            right_ref = _finite(executor.get("wheel_loop_right_ref_mps"))
            left_meas = _finite(canonical.get("left_mps"))
            right_meas = _finite(canonical.get("right_mps"))
            left_error = (
                None
                if left_ref is None or left_meas is None
                else abs(float(left_ref) - float(left_meas))
            )
            right_error = (
                None
                if right_ref is None or right_meas is None
                else abs(float(right_ref) - float(right_meas))
            )
            if left_error is not None:
                left_errors.append(float(left_error))
            if right_error is not None:
                right_errors.append(float(right_error))
            count_window_identity = _canonical_encoder_count_window_identity(pulses)
            feedback_identity = (
                ("canonical_count_window", *count_window_identity)
                if count_window_identity is not None
                else ("missing_count_window_identity",)
            )
            if feedback_identity in seen_feedback_windows:
                continue
            seen_feedback_windows.add(feedback_identity)
            if left_error is not None:
                independent_left_errors.append(float(left_error))
            if right_error is not None:
                independent_right_errors.append(float(right_error))
        return {
            "sample_count": int(len(indices)),
            "independent_feedback_windows": int(
                max(len(independent_left_errors), len(independent_right_errors))
            ),
            "wheel_error_sample_count": int(
                len(independent_left_errors) + len(independent_right_errors)
            ),
            "wheel_speed_tracking_mae_mps": _mean_finite(
                independent_left_errors + independent_right_errors
            ),
            "left_wheel_speed_tracking_mae_mps": _mean_finite(independent_left_errors),
            "right_wheel_speed_tracking_mae_mps": _mean_finite(independent_right_errors),
            "poll_weighted_wheel_speed_tracking_mae_mps": _mean_finite(
                left_errors + right_errors
            ),
        }

    m0_caster_case = str(case.name) == M0_MINI_CASE_NAME
    m1_caster_case = str(case.name) in M1_CASTER_CASE_CONTRACTS
    m1_caster_wheel_mae_limit = (
        _m1_caster_wheel_mae_limit_mps(case.v_mps)
        if m1_caster_case
        else None
    )
    phase_tracking = {
        "contract": {
            "anchor": "first_observed_pwm",
            "settled_start_after_first_pwm_s": float(SETTLED_PHASE_START_S),
            "caster_transient_after_first_pwm_s": float(caster_transient_s),
            "full_phase_metric_retained": True,
            "wheel_tracking_gate_phase": (
                "post_caster_transient"
                if m0_caster_case
                else (
                    "settled_and_post_caster_bounded"
                    if m1_caster_case
                    else "settled"
                )
            ),
            "wheel_tracking_mae_max_mps": float(
                WHEEL_SPEED_TRACKING_MAE_MAX_MPS
            ),
            "caster_full_settled_wheel_mae_max_mps": (
                float(CASTER_FULL_SETTLED_WHEEL_MAE_MAX_MPS)
                if m0_caster_case
                else None
            ),
            "m1_caster_influence_contract_id": (
                M1_CASTER_INFLUENCE_CONTRACT_ID
                if m1_caster_case
                else ""
            ),
            "m1_caster_wheel_mae_relative_max": (
                float(M1_CASTER_WHEEL_MAE_RELATIVE_MAX)
                if m1_caster_case
                else None
            ),
            "m1_caster_wheel_mae_absolute_max_mps": (
                float(M1_CASTER_WHEEL_MAE_ABSOLUTE_MAX_MPS)
                if m1_caster_case
                else None
            ),
            "m1_caster_wheel_mae_case_limit_mps": (
                m1_caster_wheel_mae_limit
                if m1_caster_case
                else None
            ),
            "m1_caster_arc_angular_ratio_range": (
                [
                    float(M1_CASTER_ARC_ANGULAR_RATIO_MIN),
                    float(M1_CASTER_ARC_ANGULAR_RATIO_MAX),
                ]
                if m1_caster_case and bool(case.expected_yaw_sign)
                else None
            ),
            "caster_min_transient_feedback_windows": (
                int(CASTER_MIN_TRANSIENT_FEEDBACK_WINDOWS)
                if m0_caster_case or m1_caster_case
                else 0
            ),
            "caster_min_post_transient_feedback_windows": (
                int(CASTER_MIN_POST_TRANSIENT_FEEDBACK_WINDOWS)
                if m0_caster_case or m1_caster_case
                else 0
            ),
        },
        "onset": _wheel_phase_metrics(onset_indices),
        "transition": _wheel_phase_metrics(transition_indices),
        "settled": _wheel_phase_metrics(settled_indices),
        "caster_transient": _wheel_phase_metrics(caster_transient_indices),
        "post_caster_transient": _wheel_phase_metrics(
            post_caster_transient_indices
        ),
        "stop": {
            **_wheel_phase_metrics(stop_phase_indices),
            "max_abs_pwm": max(
                [
                    max(abs(float(pwm_left[idx])), abs(float(pwm_right[idx])))
                    for idx in stop_phase_indices
                ]
                or [0.0]
            ),
        },
    }
    if m1_caster_case:
        settled_mae = phase_tracking["settled"][
            "wheel_speed_tracking_mae_mps"
        ]
        post_caster_mae = phase_tracking["post_caster_transient"][
            "wheel_speed_tracking_mae_mps"
        ]
        phase_tracking["caster_influence"] = {
            "contract_id": M1_CASTER_INFLUENCE_CONTRACT_ID,
            "case_limit_mps": m1_caster_wheel_mae_limit,
            "nominal_limit_mps": float(WHEEL_SPEED_TRACKING_MAE_MAX_MPS),
            "settled_wheel_mae_mps": settled_mae,
            "post_caster_wheel_mae_mps": post_caster_mae,
            "allowance_used": bool(
                (
                    settled_mae is not None
                    and float(settled_mae)
                    > float(WHEEL_SPEED_TRACKING_MAE_MAX_MPS)
                )
                or (
                    post_caster_mae is not None
                    and float(post_caster_mae)
                    > float(WHEEL_SPEED_TRACKING_MAE_MAX_MPS)
                )
            ),
        }
    poll_weighted_actual_speeds = _moving_median(
        [_measured_linear_speed_abs(sample) for sample in samples],
        window=5,
    )
    independent_feedback_observations: List[Dict[str, float]] = []
    seen_feedback_windows = set()
    for idx in active_indices:
        executor = dict(((samples[idx].get("diagnostics") or {}).get("executor") or {}))
        encoder = dict(samples[idx].get("encoder") or {})
        canonical = dict(encoder.get("canonical_velocity") or {})
        pulses = dict(encoder.get("canonical_pulses_delta") or {})
        left_meas = _finite(canonical.get("left_mps"))
        right_meas = _finite(canonical.get("right_mps"))
        if left_meas is None or right_meas is None:
            continue
        count_window_identity = _canonical_encoder_count_window_identity(pulses)
        feedback_identity = (
            ("canonical_count_window", *count_window_identity)
            if count_window_identity is not None
            else ("missing_count_window_identity",)
        )
        if feedback_identity in seen_feedback_windows:
            continue
        seen_feedback_windows.add(feedback_identity)
        sample_ts = _finite(samples[idx].get("ts"))
        if sample_ts is None:
            continue
        independent_feedback_observations.append(
            {
                "ts": float(sample_ts),
                "speed_abs_mps": abs(0.5 * (float(left_meas) + float(right_meas))),
            }
        )
    independent_actual_speeds = _moving_median(
        [item["speed_abs_mps"] for item in independent_feedback_observations],
        window=5,
    )
    settling_time_s = None
    if active_indices and executed_v_mean is not None and abs(float(executed_v_mean)) > 1e-6:
        ref_abs = abs(float(executed_v_mean))
        first_active_ts = _finite(samples[active_indices[0]].get("ts"))
        stable = 0
        for observation, actual_v in zip(
            independent_feedback_observations,
            independent_actual_speeds,
        ):
            if abs(abs(float(actual_v)) - ref_abs) <= ref_abs * 0.20:
                stable += 1
                if stable >= 3 and first_active_ts is not None:
                    settling_time_s = max(
                        0.0,
                        float(observation["ts"]) - float(first_active_ts),
                    )
                    break
            else:
                stable = 0
    endpoint_yaws = [
        float(value)
        for value in (selected_imu_yaw_delta, ekf_yaw_delta, lidar_yaw_delta)
        if _finite(value) is not None
    ]
    endpoint_yaw_spread_deg = (
        max(endpoint_yaws) - min(endpoint_yaws) if len(endpoint_yaws) >= 2 else None
    )
    imu_yaw_series = [
        _finite((sample.get("imu") or {}).get("yaw_deg"))
        for sample in samples
    ]
    imu_yaw_series = [float(value) for value in imu_yaw_series if value is not None]
    signed_yaw_progress = [
        float(case.expected_yaw_sign) * _unwrapped_yaw_delta_deg(imu_yaw_series[:idx])
        for idx in range(2, len(imu_yaw_series) + 1)
    ]
    pivot_overshoot_deg = None
    if case.target_angle_deg is not None and signed_yaw_progress:
        pivot_overshoot_deg = max(
            0.0,
            max(signed_yaw_progress) - abs(float(case.target_angle_deg)),
        )

    return {
        "case": str(case.name),
        "kind": str(case.kind),
        "command": {
            "command_motion": bool(case.command_motion),
            "v_mps": float(case.v_mps),
            "omega_rad_s": float(case.omega_rad_s),
            "duration_s": float(case.duration_s),
            "expected_linear_sign": int(case.expected_linear_sign),
            "expected_yaw_sign": int(case.expected_yaw_sign),
            "quality_gate": bool(case.quality_gate),
            "command_type": str(case.command_type),
            "target_angle_deg": case.target_angle_deg,
            "heading_speed_level": int(case.heading_speed_level),
            "caster_pair": str(case.caster_pair),
            "caster_orientation": str(case.caster_orientation),
            "caster_transient_s": float(case.caster_transient_s),
            "operator_instruction_hu": str(case.operator_instruction_hu),
            "chassis_dynamics_verdict": bool(
                case.chassis_dynamics_verdict
            ),
            "command_types_seen": dict(sorted(command_types.items())),
        },
        "samples": {
            "count": int(len(samples)),
            "status_version_start": _safe_int(start.get("status_version"), -1),
            "status_version_end": _safe_int(end.get("status_version"), -1),
            "status_progressed": _safe_int(end.get("status_version"), -1) > _safe_int(start.get("status_version"), -1),
        },
        "sensor_endpoint_shared_window": shared_sensor_endpoint,
        "runtime_timing": {
            "encoder_timing_contract_missing_samples": sum(
                1
                for sample in samples
                if not bool(
                    (sample.get("encoder") or {}).get(
                        "canonical_timing_contract_present", False
                    )
                )
            ),
            "encoder_timing_gap_count_delta": max(0, int(timing_gap_end - timing_gap_start)),
            "encoder_motion_timing_gap_count_delta": max(
                0, int(motion_timing_gap_end - motion_timing_gap_start)
            ),
            "encoder_idle_timing_gap_count_delta": max(
                0, int(idle_timing_gap_end - idle_timing_gap_start)
            ),
            "encoder_timing_gap_observed_samples": int(len(timing_gap_samples)),
            "encoder_timing_gap_max_s": max(
                [
                    _safe_float(
                        (sample.get("encoder") or {}).get("canonical_timing_gap_s"),
                        0.0,
                    )
                    for sample in timing_gap_samples
                ]
                or [0.0]
            ),
            "gc_policy": str(final_gc_runtime.get("policy", start_gc_runtime.get("policy", "")) or ""),
            "gc_automatic_enabled": bool(
                final_gc_runtime.get("automatic_enabled", start_gc_runtime.get("automatic_enabled", False))
            ),
            "gc_motion_collection_count_delta": max(0, int(gc_motion_end - gc_motion_start)),
            "gc_unowned_collection_count_delta": max(0, int(gc_unowned_end - gc_unowned_start)),
            "gc_contract_violation_count_end": _safe_int(
                final_gc_runtime.get("contract_violation_count"), 0
            ),
            "gc_last_collection": dict(final_gc_runtime.get("last_collection") or {}),
            "gc_last_violation": dict(final_gc_runtime.get("last_violation") or {}),
        },
        "motor_pwm": {
            "max_abs_left": max([abs(float(v)) for v in pwm_left] or [0.0]),
            "max_abs_right": max([abs(float(v)) for v in pwm_right] or [0.0]),
            "active_samples": sum(1 for flag in pwm_active if flag),
            "active_islands": _active_islands(pwm_active),
            "mean_abs_left": _mean([abs(float(v)) for v in pwm_left]),
            "mean_abs_right": _mean([abs(float(v)) for v in pwm_right]),
            "mean_abs_delta": _mean(pwm_asymmetry),
            "max_abs_delta": max(pwm_asymmetry or [0.0]),
            "one_side_active_samples": sum(1 for flag in one_side_pwm_active if flag),
            "opposing_active_samples": sum(1 for flag in opposing_pwm_active if flag),
            "stable_floor": stable_pwm_floors,
            "active_side_samples": int(pwm_side_active_count),
            "near_stable_floor_samples": int(pwm_near_stable_floor_count),
            "near_stable_floor_ratio": (
                float(pwm_near_stable_floor_count) / float(pwm_side_active_count)
                if pwm_side_active_count
                else 0.0
            ),
            "below_stable_floor_samples": int(pwm_below_stable_floor_count),
            "below_stable_floor_ratio": (
                float(pwm_below_stable_floor_count) / float(pwm_side_active_count)
                if pwm_side_active_count
                else 0.0
            ),
        },
        "encoder": {
            "available": bool(samples),
            "left_delta_m": float(left_delta),
            "right_delta_m": float(right_delta),
            "average_delta_m": float(encoder_avg),
            "differential_delta_m": float(encoder_diff),
            "left_pulse_delta": _safe_int(end_encoder.get("left_pulses"), 0) - _safe_int(start_encoder.get("left_pulses"), 0),
            "right_pulse_delta": _safe_int(end_encoder.get("right_pulses"), 0) - _safe_int(start_encoder.get("right_pulses"), 0),
            "snapshot_health_start": str(start_encoder.get("snapshot_health", "") or ""),
            "snapshot_health_end": str(end_encoder.get("snapshot_health", "") or ""),
        },
        "imu": {
            "available": bool(start_imu.get("available", False) and end_imu.get("available", False)),
            "yaw_start_deg": start_imu.get("yaw_deg"),
            "yaw_end_deg": end_imu.get("yaw_deg"),
            "euler_yaw_delta_deg": imu_yaw_delta,
            "gyro_integrated_yaw_delta_deg": gyro_yaw_delta,
            "yaw_delta_deg": selected_imu_yaw_delta,
            "source": (
                "imu.gyro_z_integrated"
                if gyro_yaw_delta is not None
                else str(end_imu.get("source") or start_imu.get("source") or "")
            ),
            "euler_source": str(end_imu.get("source") or start_imu.get("source") or ""),
            "health": str(end_imu.get("health") or start_imu.get("health") or ""),
        },
        "ekf": {
            "pose_start": start_pose,
            "pose_end": end_pose,
            "chord_m": _pose_distance(start_pose, end_pose) if start_pose and end_pose else 0.0,
            "forward_delta_m": float(ekf_forward_delta),
            "forward_endpoint_projection_m": float(ekf_forward_endpoint_delta),
            "yaw_delta_deg": float(ekf_yaw_delta),
            "yaw_endpoint_wrapped_delta_deg": float(ekf_yaw_endpoint_delta),
        },
        "lidar": {
            "enabled_seen": any(bool((sample.get("lidar") or {}).get("enabled", False)) for sample in samples),
            "health_values": sorted(
                {
                    str((sample.get("lidar") or {}).get("health", "") or "")
                    for sample in samples
                    if str((sample.get("lidar") or {}).get("health", "") or "")
                }
            ),
            "accepted_delta": _safe_int(end_lidar.get("accepted"), 0) - _safe_int(start_lidar.get("accepted"), 0),
            "rejected_delta": _safe_int(end_lidar.get("rejected_total"), 0) - _safe_int(start_lidar.get("rejected_total"), 0),
            "applied_samples": int(len(applied_measurement_ids)),
            "applied_status_samples": int(applied_status_samples),
            "applied_missing_measurement_id_samples": int(
                applied_missing_measurement_id_samples
            ),
            "unique_raw_scan_observations": int(len(raw_scan_ids)),
            "unique_matcher_result_observations": int(len(matcher_result_ids)),
            "unique_lidar_odometry_measurements": int(len(measurement_observations)),
            "latest_age_s_max": max(lidar_ages) if lidar_ages else None,
            "latest_age_s_median": _median(lidar_ages),
            "latest_confidence_min": min(lidar_conf) if lidar_conf else None,
            "latest_confidence_median": _median(lidar_conf),
            "latest_confidence_poll_median": _median(lidar_conf_poll),
            "observation_contract_errors": list(
                dict.fromkeys(observation_contract_errors)
            ),
            "pose_source": str(end_lidar_pose.get("source") or start_lidar_pose.get("source") or ""),
            "pose_chord_m": lidar_chord,
            "pose_dx_m": lidar_dx,
            "pose_dy_m": lidar_dy,
            "yaw_delta_deg": lidar_yaw_delta,
        },
        "safety": {
            "failsafe_seen": any(str(sample.get("state", "")).upper() == "FAILSAFE" for sample in samples),
            "safety_block_seen": bool(safety_reasons),
            "safety_stop_reasons": list(dict.fromkeys(reason for reason in safety_reasons if reason)),
            "stop_reasons": list(dict.fromkeys(reason for reason in stop_reasons if reason)),
            "emergency_count_delta": int(emergency_end - emergency_start),
            "normal_stop_confirmed": bool(normal_stop_confirmed),
            "stop_status": {
                "state": _status_state(stop_status),
                "type": str((stop_status.get("stop_status") or {}).get("type", "") or ""),
                "reason": str((stop_status.get("stop_status") or {}).get("reason", "") or ""),
            },
        },
        "stop_start": {
            "pwm_active_islands": _active_islands(pwm_active),
            "velocity_active_islands": _active_islands(velocity_active),
            "command_drop_samples": sum(
                1
                for idx, active in enumerate(pwm_active)
                if bool(case.command_motion) and idx >= 3 and not bool(active) and not bool(velocity_active[idx])
            ),
            "stop_start_suspect": bool(case.command_motion and _active_islands(pwm_active) > 1),
        },
        "runtime_diagnostics": {
            "motion_controller_policy_modes": dict(sorted(policy_modes.items())),
            "executor_output_reasons": dict(sorted(executor_reasons.items())),
            "wheel_loop_left_output_reasons": dict(sorted(wheel_left_reasons.items())),
            "wheel_loop_right_output_reasons": dict(sorted(wheel_right_reasons.items())),
            "policy_null_samples": int(policy_null_samples),
            "executor_zero_cmd_while_commanded_samples": int(executor_zero_cmd_samples),
            "trailing_zero_grace_samples": int(trailing_zero_grace_samples),
            "forward_dominant_policy_applied_samples": sum(
                1
                for sample in samples
                if bool((((sample.get("diagnostics") or {}).get("motion_controller") or {}).get("forward_dominant_policy_applied", False)))
            ),
            "heading_correction_owners": dict(sorted(heading_owners.items())),
            "guidance_straight_hold_samples": sum(
                1
                for idx in active_indices
                if bool((((samples[idx].get("diagnostics") or {}).get("guidance") or {}).get("straight_hold_active", False)))
            ),
            "guard_intervention_samples": int(guard_intervention_samples),
            "guard_intervention_ratio": (
                float(guard_intervention_samples) / float(len(active_indices))
                if active_indices
                else 0.0
            ),
        },
        "correction_dynamics": {
            "active_duration_s": float(active_duration_s),
            "guidance_omega_direction_changes": _direction_changes(guidance_omega_active, 0.003),
            "straight_hold_direction_changes": _direction_changes(straight_hold_correction_active, 0.003),
            "pwm_difference_direction_changes": _direction_changes(pwm_difference_active, 0.01),
            "guidance_omega_total_variation": _total_variation(guidance_omega_active),
            "straight_hold_total_variation": _total_variation(straight_hold_correction_active),
            "pwm_difference_total_variation": _total_variation(pwm_difference_active),
            "guidance_omega_total_variation_per_s": (
                _total_variation(guidance_omega_active) / float(active_duration_s)
                if active_duration_s > 1e-6
                else 0.0
            ),
            "pwm_difference_total_variation_per_s": (
                _total_variation(pwm_difference_active) / float(active_duration_s)
                if active_duration_s > 1e-6
                else 0.0
            ),
        },
        "phase_tracking": phase_tracking,
        "command_fidelity": {
            "requested": {
                "linear_mps": float(case.v_mps),
                "angular_rad_s": float(case.omega_rad_s),
                "distance_m": float(requested_distance_m),
                "angle_rad": float(requested_angle_rad),
            },
            "executed": {
                "linear_mps_mean": executed_v_mean,
                "angular_rad_s_mean": executed_omega_mean,
                "distance_m": executed_distance_m,
                "angle_rad": executed_angle_rad,
            },
            "actual": {
                "encoder_linear_mps": float(encoder_linear_mps),
                "whole_phase_encoder_linear_mps": float(whole_phase_encoder_linear_mps),
                "imu_angular_rad_s": imu_omega_rad_s,
                "ekf_angular_rad_s": float(ekf_omega_rad_s),
                "lidar_angular_rad_s": lidar_omega_rad_s,
                "encoder_distance_m": float(encoder_avg),
                "imu_angle_rad": (
                    None if selected_imu_yaw_delta is None else math.radians(float(selected_imu_yaw_delta))
                ),
                "ekf_angle_rad": math.radians(float(ekf_yaw_delta)),
                "lidar_angle_rad": (
                    None if lidar_yaw_delta is None else math.radians(float(lidar_yaw_delta))
                ),
            },
            "errors": {
                "executed_linear_ratio_vs_requested": _ratio(executed_v_mean, case.v_mps),
                "executed_angular_ratio_vs_requested": _ratio(executed_omega_mean, case.omega_rad_s),
                "linear_speed_ratio_vs_requested": _ratio(encoder_linear_mps, case.v_mps),
                "linear_speed_ratio_vs_executed": _ratio(encoder_linear_mps, executed_v_mean),
                "whole_phase_linear_speed_ratio_vs_requested": _ratio(
                    whole_phase_encoder_linear_mps, case.v_mps
                ),
                "whole_phase_linear_speed_ratio_vs_executed": _ratio(
                    encoder_avg, executed_distance_m
                ),
                "imu_angular_speed_ratio_vs_requested": _ratio(imu_omega_rad_s, case.omega_rad_s),
                "imu_angular_speed_ratio_vs_executed": _ratio(imu_omega_rad_s, executed_omega_mean),
                "encoder_distance_error_vs_requested_m": float(encoder_avg - requested_distance_m),
                "encoder_distance_error_vs_executed_m": (
                    None if executed_distance_m is None else float(encoder_avg - executed_distance_m)
                ),
                "imu_angle_error_vs_requested_deg": (
                    None
                    if selected_imu_yaw_delta is None
                    else float(selected_imu_yaw_delta - math.degrees(requested_angle_rad))
                ),
                "imu_angle_error_vs_executed_deg": (
                    None
                    if selected_imu_yaw_delta is None or executed_angle_rad is None
                    else float(selected_imu_yaw_delta - math.degrees(executed_angle_rad))
                ),
                "wheel_speed_tracking_mae_mps": _mean_finite(wheel_errors),
                "settled_wheel_speed_tracking_mae_mps": (
                    phase_tracking["settled"]["wheel_speed_tracking_mae_mps"]
                ),
                "post_caster_wheel_speed_tracking_mae_mps": (
                    phase_tracking["post_caster_transient"][
                        "wheel_speed_tracking_mae_mps"
                    ]
                ),
                "endpoint_yaw_spread_deg": endpoint_yaw_spread_deg,
            },
            "transient": {
                "settling_time_s": settling_time_s,
                "pivot_settling_time_s": (
                    float(active_duration_s)
                    if case.target_angle_deg is not None and active_duration_s > 0.0
                    else None
                ),
                "pivot_overshoot_deg": pivot_overshoot_deg,
                "independent_encoder_feedback_windows": int(
                    len(independent_feedback_observations)
                ),
                "poll_weighted_linear_speed_overshoot_ratio": (
                    None
                    if executed_v_mean is None
                    or abs(float(executed_v_mean)) <= 1e-6
                    or not poll_weighted_actual_speeds
                    else _safe_float(
                        _percentile(poll_weighted_actual_speeds, 0.95),
                        0.0,
                    )
                    / abs(float(executed_v_mean))
                ),
                "linear_speed_overshoot_ratio": (
                    None
                    if executed_v_mean is None
                    or abs(float(executed_v_mean)) <= 1e-6
                    or not independent_actual_speeds
                    else _safe_float(
                        _percentile(independent_actual_speeds, 0.95),
                        0.0,
                    )
                    / abs(float(executed_v_mean))
                ),
            },
        },
        "start_sample": start,
        "end_sample": end,
        "stop_outcome": stop_outcome,
    }


def _case_failures(metrics: Dict[str, Any]) -> List[str]:
    failures: List[str] = []
    command = dict(metrics.get("command") or {})
    samples = dict(metrics.get("samples") or {})
    pwm = dict(metrics.get("motor_pwm") or {})
    encoder = dict(metrics.get("encoder") or {})
    imu = dict(metrics.get("imu") or {})
    ekf = dict(metrics.get("ekf") or {})
    lidar = dict(metrics.get("lidar") or {})
    safety = dict(metrics.get("safety") or {})
    stop_start = dict(metrics.get("stop_start") or {})
    runtime_diag = dict(metrics.get("runtime_diagnostics") or {})
    runtime_timing = dict(metrics.get("runtime_timing") or {})
    fidelity = dict(metrics.get("command_fidelity") or {})
    fidelity_errors = dict(fidelity.get("errors") or {})
    fidelity_transient = dict(fidelity.get("transient") or {})
    command_motion = bool(command.get("command_motion", True))
    quality_gate = bool(command.get("quality_gate", True))
    linear_sign = int(_safe_int(command.get("expected_linear_sign"), 0))
    yaw_sign = int(_safe_int(command.get("expected_yaw_sign"), 0))
    chassis_dynamics_verdict = bool(
        command.get("chassis_dynamics_verdict", True)
    )
    case_name = str(metrics.get("case", "") or "")
    m1_caster_case = case_name in M1_CASTER_CASE_CONTRACTS
    m1_caster_contract_ok = _m1_caster_contract_matches(
        case_name,
        command,
    )

    if _safe_int(samples.get("count"), 0) < 3:
        failures.append("sample_count_low")
    if not bool(samples.get("status_progressed", False)):
        failures.append("status_stream_not_progressing")
    if bool(safety.get("failsafe_seen", False)):
        failures.append("failsafe_seen")
    if bool(safety.get("safety_block_seen", False)):
        failures.append("safety_block_seen")
    if bool(command_motion) and not bool(safety.get("normal_stop_confirmed", False)):
        failures.append("normal_stop_not_confirmed")
    # A canonical timing gap mintát továbbra sem használhatja a PI vagy az EKF,
    # de önmagában nem bizonyít mozgásminőségi vagy safety hibát. Az esemény a
    # _case_warnings() útján, számlálóval marad látható; a tényleges minőségi,
    # sensor-truth és safety kapuk külön, változatlanul FAIL-t adnak.
    if _safe_int(runtime_timing.get("encoder_timing_contract_missing_samples"), 0) > 0:
        failures.append("encoder_timing_contract_missing")
    if quality_gate and bool(command_motion) and _safe_int(
        runtime_timing.get("gc_motion_collection_count_delta"), 0
    ) > 0:
        failures.append("gc_forbidden_while_motion_active")
    if bool(command_motion):
        command_types_seen = {str(key).strip().lower() for key in dict(command.get("command_types_seen") or {}).keys()}
        expected_command_type = str(command.get("command_type", "set_twist") or "set_twist").strip().lower()
        if expected_command_type not in command_types_seen:
            failures.append(f"command_path_not_{expected_command_type}")
    if bool(command_motion) and max(
        _safe_float(pwm.get("max_abs_left"), 0.0),
        _safe_float(pwm.get("max_abs_right"), 0.0),
    ) < MIN_PWM_OBSERVED:
        failures.append("motor_pwm_not_observed")
    if quality_gate and bool(command_motion) and _safe_int(runtime_diag.get("policy_null_samples"), 0) > 0:
        failures.append("policy_nulling_detected")
    if quality_gate and bool(command_motion) and _safe_int(runtime_diag.get("executor_zero_cmd_while_commanded_samples"), 0) > 0:
        failures.append("executor_zero_cmd_while_commanded")

    if not bool(encoder.get("available", False)):
        failures.append("encoder_missing")
    if not bool(imu.get("available", False)):
        failures.append("imu_yaw_missing")
    if not bool(lidar.get("enabled_seen", False)):
        failures.append("lidar_not_enabled")
    if "OK" not in {str(v).upper() for v in list(lidar.get("health_values") or [])}:
        failures.append("lidar_health_not_ok")
    if (
        list(lidar.get("observation_contract_errors") or [])
        or _safe_int(lidar.get("applied_missing_measurement_id_samples"), 0) > 0
    ):
        failures.append("lidar_observation_contract_violation")
    latest_age = _finite(lidar.get("latest_age_s_max"))
    if latest_age is None or latest_age > MAX_LIDAR_LATEST_AGE_S:
        failures.append("lidar_latest_age_high")
    latest_conf = _finite(lidar.get("latest_confidence_median"))
    if latest_conf is None or latest_conf < MIN_LIDAR_CONFIDENCE:
        failures.append("lidar_confidence_low")

    if linear_sign:
        if not _sign_ok(_finite(encoder.get("average_delta_m")), linear_sign, MIN_TRANSLATION_PROGRESS_M):
            failures.append("encoder_linear_sign_or_progress_bad")
        if not _sign_ok(_finite(ekf.get("forward_delta_m")), linear_sign, MIN_TRANSLATION_PROGRESS_M):
            failures.append("ekf_linear_sign_or_progress_bad")
        shared_endpoint = metrics.get("sensor_endpoint_shared_window")
        if isinstance(shared_endpoint, dict):
            if not bool(shared_endpoint.get("available", False)):
                failures.append("sensor_endpoint_shared_window_unavailable")
                aligned_encoder = {}
                aligned_ekf = {}
                aligned_lidar = {}
            else:
                aligned_encoder = dict(shared_endpoint.get("encoder") or {})
                aligned_ekf = dict(shared_endpoint.get("ekf_control") or {})
                aligned_lidar = dict(shared_endpoint.get("lidar") or {})
            encoder_distance = _finite(aligned_encoder.get("average_delta_m"))
            ekf_distance = _finite(aligned_ekf.get("forward_delta_m"))
            lidar_distance = _finite(aligned_lidar.get("pose_chord_m"))
        else:
            # Compatibility for historical/offline hand-built metric payloads.
            # Every newly sampled live result contains the shared-window
            # contract and fails closed above when it cannot be constructed.
            encoder_distance = _finite(encoder.get("average_delta_m"))
            ekf_distance = _finite(ekf.get("forward_delta_m"))
            lidar_distance = _finite(lidar.get("pose_chord_m"))
        if encoder_distance is not None:
            distance_tolerance = max(0.04, abs(float(encoder_distance)) * 0.35)
            if ekf_distance is None or abs(abs(float(ekf_distance)) - abs(float(encoder_distance))) > distance_tolerance:
                failures.append("encoder_ekf_endpoint_distance_mismatch")
            if lidar_distance is None or abs(abs(float(lidar_distance)) - abs(float(encoder_distance))) > distance_tolerance:
                failures.append("encoder_lidar_endpoint_distance_mismatch")
        elif isinstance(shared_endpoint, dict):
            failures.append("sensor_endpoint_shared_window_encoder_missing")
        if quality_gate and yaw_sign == 0:
            active_samples = max(1, _safe_int(pwm.get("active_samples"), 0))
            one_side_samples = _safe_int(pwm.get("one_side_active_samples"), 0)
            if one_side_samples / float(active_samples) > 0.25:
                failures.append("motor_pwm_asymmetry_high")
            straight_yaw = abs(_safe_float(imu.get("yaw_delta_deg"), 0.0))
            if chassis_dynamics_verdict and straight_yaw > 12.0:
                failures.append("straight_yaw_drift_high")
    if yaw_sign:
        if linear_sign == 0 and _safe_int(pwm.get("opposing_active_samples"), 0) < 2:
            failures.append("pivot_countertrack_pwm_not_observed")
        if not _sign_ok(_finite(encoder.get("differential_delta_m")), yaw_sign, 0.01):
            failures.append("encoder_yaw_sign_or_progress_bad")
        if not _sign_ok(_finite(imu.get("yaw_delta_deg")), yaw_sign, MIN_ROTATION_PROGRESS_DEG):
            failures.append("imu_yaw_sign_or_progress_bad")
        if not _sign_ok(_finite(ekf.get("yaw_delta_deg")), yaw_sign, MIN_ROTATION_PROGRESS_DEG):
            failures.append("ekf_yaw_sign_or_progress_bad")
        if not _sign_ok(_finite(lidar.get("yaw_delta_deg")), yaw_sign, MIN_ROTATION_PROGRESS_DEG):
            failures.append("lidar_yaw_sign_or_progress_bad")
        if quality_gate and chassis_dynamics_verdict:
            expected_yaw_deg = abs(
                math.degrees(
                    math.radians(_safe_float(command.get("target_angle_deg"), 0.0))
                    if command.get("target_angle_deg") is not None
                    else _safe_float(command.get("omega_rad_s"), 0.0)
                    * _safe_float(command.get("duration_s"), 0.0)
                )
            )
            imu_progress = abs(_safe_float(imu.get("yaw_delta_deg"), 0.0))
            if expected_yaw_deg > MIN_ROTATION_PROGRESS_DEG:
                if imu_progress < max(MIN_ROTATION_PROGRESS_DEG, expected_yaw_deg * 0.35):
                    failures.append("yaw_undertravel_high")
                if imu_progress > max(expected_yaw_deg * 2.25, expected_yaw_deg + 30.0):
                    failures.append("yaw_overtravel_high")
        if not quality_gate:
            endpoint_yaws = [
                float(value)
                for value in (
                    _finite(imu.get("yaw_delta_deg")),
                    _finite(ekf.get("yaw_delta_deg")),
                    _finite(lidar.get("yaw_delta_deg")),
                )
                if value is not None
            ]
            if len(endpoint_yaws) < 3 or max(endpoint_yaws) - min(endpoint_yaws) > 15.0:
                failures.append("sensor_endpoint_yaw_spread_high")

    if quality_gate and bool(command_motion):
        executed_linear_ratio = _finite(fidelity_errors.get("executed_linear_ratio_vs_requested"))
        if linear_sign and (executed_linear_ratio is None or not 0.80 <= abs(float(executed_linear_ratio)) <= 1.20):
            failures.append("executed_linear_command_error_high")

        linear_ratio = _finite(fidelity_errors.get("linear_speed_ratio_vs_executed"))
        if linear_sign and (linear_ratio is None or not 0.80 <= abs(float(linear_ratio)) <= 1.20):
            failures.append("linear_speed_error_high")
        whole_phase_ratio = _finite(fidelity_errors.get("whole_phase_linear_speed_ratio_vs_executed"))
        if linear_sign and (
            whole_phase_ratio is None or not 0.80 <= abs(float(whole_phase_ratio)) <= 1.20
        ):
            failures.append("whole_phase_linear_speed_error_high")
        executed_distance = _finite((fidelity.get("executed") or {}).get("distance_m"))
        distance_error = _finite(fidelity_errors.get("encoder_distance_error_vs_executed_m"))
        if linear_sign and (
            executed_distance is None
            or distance_error is None
            or abs(float(distance_error)) > max(0.02, abs(float(executed_distance)) * 0.20)
        ):
            failures.append("integrated_distance_error_high")

        if linear_sign and case_name == M0_MINI_CASE_NAME:
            phase_tracking = dict(metrics.get("phase_tracking") or {})
            caster_transient = dict(phase_tracking.get("caster_transient") or {})
            post_caster = dict(phase_tracking.get("post_caster_transient") or {})
            settled_wheel_mae = _finite(
                fidelity_errors.get("settled_wheel_speed_tracking_mae_mps")
            )
            post_caster_wheel_mae = _finite(
                post_caster.get("wheel_speed_tracking_mae_mps")
            )
            caster_transient_s = _finite(command.get("caster_transient_s"))
            transient_windows = _safe_int(
                caster_transient.get("independent_feedback_windows"), 0
            )
            post_caster_windows = _safe_int(
                post_caster.get("independent_feedback_windows"), 0
            )
            if (
                str(command.get("caster_pair", "") or "")
                != M0_MINI_CASTER_PAIR
                or str(command.get("caster_orientation", "") or "")
                != M0_MINI_CASTER_ORIENTATION
                or caster_transient_s is None
                or abs(
                    float(caster_transient_s)
                    - float(CASTER_TRANSIENT_ALLOWANCE_S)
                )
                > 1e-9
            ):
                failures.append("m0_mini_caster_transient_contract_mismatch")
            if transient_windows < CASTER_MIN_TRANSIENT_FEEDBACK_WINDOWS:
                failures.append("m0_mini_caster_transient_feedback_windows_low")
            if post_caster_windows < CASTER_MIN_POST_TRANSIENT_FEEDBACK_WINDOWS:
                failures.append("m0_mini_post_caster_feedback_windows_low")
            if settled_wheel_mae is None:
                failures.append("settled_wheel_speed_tracking_missing")
            elif settled_wheel_mae > CASTER_FULL_SETTLED_WHEEL_MAE_MAX_MPS:
                failures.append("m0_mini_full_settled_wheel_speed_unbounded")
            if post_caster_wheel_mae is None:
                failures.append("post_caster_wheel_speed_tracking_missing")
            elif post_caster_wheel_mae > WHEEL_SPEED_TRACKING_MAE_MAX_MPS:
                failures.append("post_caster_wheel_speed_tracking_error_high")
            settling_time = _finite(fidelity_transient.get("settling_time_s"))
            if (
                settling_time is None
                or settling_time > CASTER_TRANSIENT_ALLOWANCE_S
            ):
                failures.append("m0_mini_caster_reorientation_not_settled_by_1s")
        elif linear_sign and m1_caster_case:
            phase_tracking = dict(metrics.get("phase_tracking") or {})
            caster_transient = dict(
                phase_tracking.get("caster_transient") or {}
            )
            post_caster = dict(
                phase_tracking.get("post_caster_transient") or {}
            )
            settled_wheel_mae = _finite(
                fidelity_errors.get(
                    "settled_wheel_speed_tracking_mae_mps"
                )
            )
            post_caster_wheel_mae = _finite(
                post_caster.get("wheel_speed_tracking_mae_mps")
            )
            transient_windows = _safe_int(
                caster_transient.get("independent_feedback_windows"), 0
            )
            post_caster_windows = _safe_int(
                post_caster.get("independent_feedback_windows"), 0
            )
            caster_limit_mps = _m1_caster_wheel_mae_limit_mps(
                command.get("v_mps")
            )
            if not m1_caster_contract_ok:
                failures.append("m1_caster_influence_contract_mismatch")
            if transient_windows < CASTER_MIN_TRANSIENT_FEEDBACK_WINDOWS:
                failures.append(
                    "m1_caster_transient_feedback_windows_low"
                )
            if (
                post_caster_windows
                < CASTER_MIN_POST_TRANSIENT_FEEDBACK_WINDOWS
            ):
                failures.append(
                    "m1_post_caster_feedback_windows_low"
                )
            if settled_wheel_mae is None:
                failures.append("settled_wheel_speed_tracking_missing")
            elif settled_wheel_mae > caster_limit_mps:
                failures.append(
                    "m1_caster_settled_wheel_speed_tracking_error_high"
                )
            if post_caster_wheel_mae is None:
                failures.append(
                    "post_caster_wheel_speed_tracking_missing"
                )
            elif post_caster_wheel_mae > caster_limit_mps:
                failures.append(
                    "m1_post_caster_wheel_speed_tracking_error_high"
                )
        elif linear_sign:
            settled_wheel_mae = _finite(
                fidelity_errors.get("settled_wheel_speed_tracking_mae_mps")
            )
            if settled_wheel_mae is None:
                failures.append("settled_wheel_speed_tracking_missing")
            elif settled_wheel_mae > WHEEL_SPEED_TRACKING_MAE_MAX_MPS:
                failures.append("settled_wheel_speed_tracking_error_high")

        if linear_sign and yaw_sign:
            executed_angular_ratio = _finite(fidelity_errors.get("executed_angular_ratio_vs_requested"))
            if executed_angular_ratio is None or not 0.80 <= abs(float(executed_angular_ratio)) <= 1.20:
                failures.append("executed_arc_angular_command_error_high")
            if chassis_dynamics_verdict:
                angular_ratio = _finite(
                    fidelity_errors.get(
                        "imu_angular_speed_ratio_vs_executed"
                    )
                )
                angular_ratio_min = (
                    M1_CASTER_ARC_ANGULAR_RATIO_MIN
                    if m1_caster_case and m1_caster_contract_ok
                    else 0.80
                )
                angular_ratio_max = (
                    M1_CASTER_ARC_ANGULAR_RATIO_MAX
                    if m1_caster_case and m1_caster_contract_ok
                    else 1.20
                )
                if (
                    angular_ratio is None
                    or not float(angular_ratio_min)
                    <= abs(float(angular_ratio))
                    <= float(angular_ratio_max)
                ):
                    failures.append("arc_angular_speed_error_high")

        if yaw_sign and not linear_sign and chassis_dynamics_verdict:
            pivot_angle_error = _finite(fidelity_errors.get("imu_angle_error_vs_requested_deg"))
            if pivot_angle_error is None or abs(float(pivot_angle_error)) > 10.0:
                failures.append("pivot_final_angle_error_high")
            pivot_settling = _finite(fidelity_transient.get("pivot_settling_time_s"))
            if pivot_settling is None or pivot_settling > _safe_float(command.get("duration_s"), 0.0):
                failures.append("pivot_settling_time_high")
            pivot_overshoot = _finite(fidelity_transient.get("pivot_overshoot_deg"))
            if pivot_overshoot is None or pivot_overshoot > 10.0:
                failures.append("pivot_overshoot_high")

        if yaw_sign:
            endpoint_spread = _finite(fidelity_errors.get("endpoint_yaw_spread_deg"))
            if endpoint_spread is None or endpoint_spread > 15.0:
                failures.append("sensor_endpoint_yaw_spread_high")

        settling_time = _finite(fidelity_transient.get("settling_time_s"))
        if linear_sign and (settling_time is None or settling_time > min(2.0, float(command.get("duration_s", 0.0)) * 0.50)):
            failures.append("settling_time_high")
        overshoot_ratio = _finite(fidelity_transient.get("linear_speed_overshoot_ratio"))
        if linear_sign and overshoot_ratio is not None and overshoot_ratio > 1.35:
            failures.append("linear_speed_overshoot_high")

        if _safe_int(pwm.get("active_side_samples"), 0) >= 6:
            if (
                _safe_float(pwm.get("near_stable_floor_ratio"), 0.0) > MAX_NEAR_STABLE_PWM_RATIO
                and bool(stop_start.get("stop_start_suspect", False))
            ):
                failures.append("near_deadzone_pwm_occupancy_high")
            if _safe_float(pwm.get("below_stable_floor_ratio"), 0.0) > MAX_BELOW_STABLE_PWM_RATIO:
                failures.append("below_stable_pwm_floor_occupancy_high")

    if not bool(command_motion):
        if max(
            _safe_float(pwm.get("max_abs_left"), 0.0),
            _safe_float(pwm.get("max_abs_right"), 0.0),
        ) > 0.04:
            failures.append("idle_pwm_nonzero")
        if abs(_safe_float(encoder.get("average_delta_m"), 0.0)) > 0.012:
            failures.append("idle_encoder_drift_high")

    if quality_gate and bool(stop_start.get("stop_start_suspect", False)):
        failures.append("stop_start_suspect")
    return list(dict.fromkeys(failures))


def _case_warnings(metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Return non-blocking, countable timing warnings for a measured case.

    The underlying encoder timing contract remains mandatory. A missing
    contract is still a failure in _case_failures(); an observed canonical
    TIMING_GAP is a warning because canonical speed samples are excluded and
    the independent motion-quality and safety gates decide the verdict.
    """
    command = dict(metrics.get("command") or {})
    runtime_timing = dict(metrics.get("runtime_timing") or {})
    if not bool(command.get("command_motion", True)):
        return []

    motion_gap_count = max(
        0,
        _safe_int(runtime_timing.get("encoder_motion_timing_gap_count_delta"), 0),
    )
    observed_samples = max(
        0,
        _safe_int(runtime_timing.get("encoder_timing_gap_observed_samples"), 0),
    )
    if motion_gap_count <= 0 and observed_samples <= 0:
        return []

    return [
        {
            "code": "encoder_timing_gap",
            "severity": "WARNING",
            "count": int(max(motion_gap_count, observed_samples)),
            "motion_timing_gap_count_delta": int(motion_gap_count),
            "timing_gap_count_delta": max(
                0,
                _safe_int(runtime_timing.get("encoder_timing_gap_count_delta"), 0),
            ),
            "observed_samples": int(observed_samples),
            "max_gap_s": max(
                0.0,
                _safe_float(runtime_timing.get("encoder_timing_gap_max_s"), 0.0),
            ),
            "message_hu": (
                "Encoder timing-gap megfigyelve: a canonical sebességminta "
                "kizárt, ezért az esemény önmagában WARNING; a safety és "
                "mozgásminőségi kapuk változatlanul kötelezőek."
            ),
        }
    ]


def _normal_zero_stop(token: str, *, timeout_s: float) -> Dict[str, Any]:
    outcome: Dict[str, Any] = {
        "normal_stop_confirmed": False,
        "command": {},
        "status": {},
        "error": "",
    }
    try:
        cmd = _send_command_checked(
            "set_twist",
            token=str(token),
            timeout_s=4.0,
            v=0.0,
            omega=0.0,
            motion_source=DEFAULT_MOTION_SOURCE,
        )
        stopped = _wait_until_stopped(timeout_s=max(0.5, float(timeout_s)))
        outcome.update(
            {
                "normal_stop_confirmed": True,
                "command": dict(cmd or {}),
                "status": dict(stopped or {}),
            }
        )
    except Exception as exc:
        outcome["error"] = str(exc)
        _safe_stop_best_effort(token=str(token))
        outcome["status"] = dict(_read_json(STATUS_PATH) or {})
    return outcome


def _run_case(
    case: MeasurementCase,
    *,
    token: str,
    poll_s: float,
    keepalive_s: float,
    stop_timeout_s: float,
    sample_path: Path,
    case_attempt: int = 1,
) -> Dict[str, Any]:
    start_status = _wait_for_status_progress(min_increments=1, timeout_s=5.0)
    pre_command_sample = _sample_status(str(case.name), start_status)
    pre_command_sample["case_attempt"] = int(case_attempt)
    samples: List[Dict[str, Any]] = [pre_command_sample]
    _append_jsonl(sample_path, pre_command_sample)
    start_command: Dict[str, Any] = {}
    command_id = ""
    command_sent_mono = None
    command_status_terminal = False
    error = ""

    if bool(case.command_motion):
        command_sent_wall = time.time()
        command_sent_mono = time.monotonic()
        if str(case.command_type).strip().lower() == "rotate_to_heading":
            command_id = _append_command(
                "rotate_to_heading",
                token=str(token),
                relative_deg=float(case.target_angle_deg or 0.0),
                tolerance_deg=2.0,
                settle_time_s=0.25,
                max_duration_s=float(case.duration_s),
                speed_level=int(case.heading_speed_level),
                motion_source=DEFAULT_MOTION_SOURCE,
            )
        else:
            command_id = _append_command(
                "set_twist",
                token=str(token),
                v=float(case.v_mps),
                omega=float(case.omega_rad_s),
                motion_source=DEFAULT_MOTION_SOURCE,
            )
        start_command = {
            "cmd_id": str(command_id),
            "cmd_type": str(case.command_type),
            "sent_ts_wall": float(command_sent_wall),
            "duration_s": None,
            "status": {"state": "submitted"},
        }

    started = float(command_sent_mono) if command_sent_mono is not None else time.monotonic()
    last_keepalive = started
    try:
        while (time.monotonic() - started) <= float(case.duration_s):
            now = time.monotonic()
            if (
                bool(case.command_motion)
                and str(case.command_type).strip().lower() == "set_twist"
                and (now - last_keepalive) >= float(keepalive_s)
            ):
                _append_command(
                    "set_twist",
                    token=str(token),
                    v=float(case.v_mps),
                    omega=float(case.omega_rad_s),
                    motion_source=DEFAULT_MOTION_SOURCE,
                )
                last_keepalive = now
            status = _read_json(STATUS_PATH) or start_status
            sample = _sample_status(str(case.name), status)
            sample["case_attempt"] = int(case_attempt)
            samples.append(sample)
            _append_jsonl(sample_path, sample)
            if command_id and not command_status_terminal:
                lifecycle = _latest_command_status(str(command_id))
                lifecycle_state = str((lifecycle or {}).get("state", "") or "").strip().lower()
                if lifecycle_state in {"effective", "failed"}:
                    command_status_terminal = True
                    start_command["status"] = dict(lifecycle or {})
                    start_command["duration_s"] = max(
                        0.0,
                        time.monotonic() - float(command_sent_mono or started),
                    )
                    if lifecycle_state == "failed":
                        error = str(
                            (lifecycle or {}).get("reason")
                            or (lifecycle or {}).get("error_code")
                            or "command_failed"
                        )
                        break
            state = str(sample.get("state", "") or "").upper()
            safety = dict(sample.get("safety") or {})
            if state == "FAILSAFE" or not bool(safety.get("allow", True)):
                break
            if (
                str(case.command_type).strip().lower() == "rotate_to_heading"
                and (now - started) >= 0.5
                and state == "IDLE"
                and max(
                    abs(float((sample.get("pwm") or {}).get("left", 0.0))),
                    abs(float((sample.get("pwm") or {}).get("right", 0.0))),
                ) < 0.02
            ):
                break
            time.sleep(max(0.01, float(poll_s)))
    except Exception as exc:
        error = str(exc)
    finally:
        if command_id and not command_status_terminal:
            lifecycle = _wait_command_terminal(str(command_id), timeout_s=0.5)
            lifecycle_state = str((lifecycle or {}).get("state", "") or "").strip().lower()
            start_command["status"] = dict(lifecycle or {})
            start_command["duration_s"] = max(
                0.0,
                time.monotonic() - float(command_sent_mono or started),
            )
            if lifecycle_state != "effective" and not error:
                error = str(
                    (lifecycle or {}).get("reason")
                    or (lifecycle or {}).get("error_code")
                    or "command_status_timeout"
                )
        stop_outcome = _normal_zero_stop(str(token), timeout_s=float(stop_timeout_s))

    if not samples:
        sample = _sample_status(str(case.name), _read_json(STATUS_PATH) or start_status)
        sample["case_attempt"] = int(case_attempt)
        samples.append(sample)
        _append_jsonl(sample_path, sample)

    metrics = _summarize_samples(case, samples, stop_outcome)
    metrics["start_command"] = dict(start_command or {})
    metrics["runtime_error"] = str(error)
    failures = _case_failures(metrics)
    warnings = _case_warnings(metrics)
    if error:
        failures.append("case_runtime_error")
    return {
        "case": str(case.name),
        "attempt": int(case_attempt),
        "success": not failures,
        "failures": list(dict.fromkeys(failures)),
        "warnings": warnings,
        "metrics": metrics,
    }


def _case_retry_reason(
    result: Dict[str, Any],
    *,
    retry_all_measurement_failures: bool,
) -> str:
    """Classify an invalid case for remeasurement without accepting its data."""

    if bool(result.get("success", False)):
        return ""
    failures = {
        str(item)
        for item in list(result.get("failures") or [])
        if str(item)
    }
    if retry_all_measurement_failures and failures:
        return "measurement_gate_failure"
    metrics = dict(result.get("metrics") or {})
    safety = dict(metrics.get("safety") or {})
    if (
        bool(safety.get("failsafe_seen", False))
        or bool(safety.get("safety_block_seen", False))
        or _safe_int(safety.get("emergency_count_delta"), 0) > 0
    ):
        return "safety_intervention"
    stop_status = dict((metrics.get("stop_outcome") or {}).get("status") or {})
    if bool(_manual_reposition_suspected(stop_status).get("suspected", False)):
        return "manual_reposition"
    return ""


def _invalid_case_attempt_record(
    result: Dict[str, Any],
    *,
    retry_reason: str,
) -> Dict[str, Any]:
    metrics = dict(result.get("metrics") or {})
    return {
        "case": str(result.get("case", "") or ""),
        "attempt": _safe_int(result.get("attempt"), 0),
        "sample_accepted": False,
        "retry_reason": str(retry_reason),
        "failures": list(result.get("failures") or []),
        "warnings": list(result.get("warnings") or []),
        "safety": dict(metrics.get("safety") or {}),
        "encoder": dict(metrics.get("encoder") or {}),
        "lidar": dict(metrics.get("lidar") or {}),
        "ekf": dict(metrics.get("ekf") or {}),
    }


def _measurement_trust_errors(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    phase = str(payload.get("phase", "") or "").strip().upper()
    trust = dict(payload.get("measurement_trust") or {})
    if phase == "M0_MINI":
        if str(payload.get("contract_id", "") or "") != M0_MINI_CONTRACT_ID:
            errors.append("measurement_trust_m0_mini_contract_mismatch")
        if not bool(trust.get("equivalent_to_full_m0", False)):
            errors.append("measurement_trust_m0_mini_not_equivalent")
    elif phase != "M0":
        errors.append("measurement_trust_phase_not_m0")
    if not bool(payload.get("success", False)):
        errors.append("measurement_trust_failed")
    if not bool(trust.get("ok", False)):
        errors.append("measurement_trust_not_ok")
    surface = dict(trust.get("sensor_surface") or {})
    for key in ("encoder_cases", "imu_cases", "lidar_cases", "ekf_cases", "motor_pwm_cases"):
        if _safe_int(surface.get(key), 0) <= 0:
            errors.append(f"measurement_trust_{key}_missing")
    return errors


def _latest_measurement_trust_path() -> Optional[Path]:
    candidates = [
        path
        for path in (LATEST_TRUST_RESULT_PATH, LATEST_MINI_TRUST_RESULT_PATH)
        if path.exists()
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: float(path.stat().st_mtime))


def _m0_mini_result(
    case_result: Dict[str, Any],
    *,
    source_profile: str = "M1_motion_baseline_live",
) -> Dict[str, Any]:
    metrics = dict(case_result.get("metrics") or {})
    pwm = dict(metrics.get("motor_pwm") or {})
    sensor_surface = {
        "encoder_cases": int(bool((metrics.get("encoder") or {}).get("available", False))),
        "imu_cases": int(bool((metrics.get("imu") or {}).get("available", False))),
        "lidar_cases": int(bool((metrics.get("lidar") or {}).get("enabled_seen", False))),
        "ekf_cases": int(bool(metrics.get("ekf"))),
        "motor_pwm_cases": int(
            max(
                _safe_float(pwm.get("max_abs_left"), 0.0),
                _safe_float(pwm.get("max_abs_right"), 0.0),
            )
            >= MIN_PWM_OBSERVED
        ),
    }
    failures = list(case_result.get("failures") or [])
    warnings = [
        dict(item)
        for item in list(case_result.get("warnings") or [])
        if isinstance(item, dict)
    ]
    if str(case_result.get("case", "") or "") != M0_MINI_CASE_NAME:
        failures.append("m0_mini_case_identity_mismatch")
    for key, count in sensor_surface.items():
        if int(count) <= 0:
            failures.append(f"m0_mini_{key}_missing")
    failures = list(dict.fromkeys(failures))
    success = not failures
    return {
        "schema": "R2B4_M0_MINI_TRUST_V2",
        "contract_id": M0_MINI_CONTRACT_ID,
        "phase": "M0_MINI",
        "test": "M0_mini_measurement_trust_live",
        "source_profile": str(source_profile),
        "started_at_utc": _now_iso_utc(),
        "success": bool(success),
        "cases": [dict(case_result)],
        "failures": failures,
        "warnings": warnings,
        "measurement_trust": {
            "ok": bool(success),
            "equivalent_to_full_m0": bool(success),
            "sensor_surface": sensor_surface,
            "failures": failures,
            "warnings": warnings,
        },
        "artifact": str(LATEST_MINI_TRUST_RESULT_PATH),
    }


def _m0_mini_allows_continuation(payload: Dict[str, Any]) -> bool:
    trust = dict(payload.get("measurement_trust") or {})
    return bool(
        str(payload.get("phase", "") or "") == "M0_MINI"
        and str(payload.get("contract_id", "") or "") == M0_MINI_CONTRACT_ID
        and bool(payload.get("success", False))
        and bool(trust.get("ok", False))
        and bool(trust.get("equivalent_to_full_m0", False))
        and not list(payload.get("failures") or [])
    )


def _timing_gap_warning_summary(results: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Aggregate case warnings for future trend monitoring without gating PASS."""
    cases: List[Dict[str, Any]] = []
    warning_count = 0
    motion_count = 0
    total_count = 0
    observed_samples = 0
    max_gap_s = 0.0
    for result in results:
        case_name = str(result.get("case", "") or "")
        metrics = dict(result.get("metrics") or {})
        runtime_timing = dict(metrics.get("runtime_timing") or {})
        case_warnings = [
            dict(item)
            for item in list(result.get("warnings") or [])
            if isinstance(item, dict)
            and str(item.get("code", "") or "") == "encoder_timing_gap"
        ]
        if not case_warnings:
            continue
        case_motion_count = max(
            0,
            _safe_int(runtime_timing.get("encoder_motion_timing_gap_count_delta"), 0),
        )
        case_total_count = max(
            0,
            _safe_int(runtime_timing.get("encoder_timing_gap_count_delta"), 0),
        )
        case_observed = max(
            0,
            _safe_int(runtime_timing.get("encoder_timing_gap_observed_samples"), 0),
        )
        case_max_gap_s = max(
            0.0,
            _safe_float(runtime_timing.get("encoder_timing_gap_max_s"), 0.0),
        )
        case_warning_count = sum(
            max(0, _safe_int(warning.get("count"), 0))
            for warning in case_warnings
        )
        cases.append(
            {
                "case": case_name,
                "warning_count": int(case_warning_count),
                "motion_timing_gap_count_delta": int(case_motion_count),
                "timing_gap_count_delta": int(case_total_count),
                "observed_samples": int(case_observed),
                "max_gap_s": float(case_max_gap_s),
            }
        )
        warning_count += int(case_warning_count)
        motion_count += int(case_motion_count)
        total_count += int(case_total_count)
        observed_samples += int(case_observed)
        max_gap_s = max(float(max_gap_s), float(case_max_gap_s))

    return {
        "schema": "R2B4_TIMING_GAP_WARNING_V1",
        "code": "encoder_timing_gap",
        "severity": "WARNING" if cases else "NONE",
        "case_count": int(len(cases)),
        "warning_count": int(warning_count),
        "motion_timing_gap_count_delta": int(motion_count),
        "timing_gap_count_delta": int(total_count),
        "observed_samples": int(observed_samples),
        "max_gap_s": float(max_gap_s),
        "future_trend_monitoring_required": bool(cases),
        "pass_policy": (
            "non_blocking_when_safety_sensor_truth_and_motion_quality_gates_pass"
        ),
        "cases": cases,
    }


def _dependency_status(max_age_s: float) -> Dict[str, Any]:
    trust_path = _latest_measurement_trust_path()
    result = {
        "required": True,
        "ok": False,
        "path": str(trust_path or LATEST_TRUST_RESULT_PATH),
        "age_s": None,
        "errors": [],
        "payload_summary": {},
    }
    if trust_path is None:
        result["errors"] = ["measurement_trust_missing"]
        return result
    try:
        payload = json.loads(trust_path.read_text(encoding="utf-8"))
    except Exception as exc:
        result["errors"] = [f"measurement_trust_unreadable:{exc}"]
        return result
    try:
        age_s = max(0.0, time.time() - float(trust_path.stat().st_mtime))
        result["age_s"] = round(age_s, 3)
    except Exception:
        age_s = float("inf")
    errors = _measurement_trust_errors(payload)
    if age_s > max(60.0, float(max_age_s)):
        errors.append("measurement_trust_stale")
    result["errors"] = errors
    result["ok"] = not errors
    result["payload_summary"] = {
        "success": bool(payload.get("success", False)),
        "phase": str(payload.get("phase", "") or ""),
        "contract_id": str(payload.get("contract_id", "") or ""),
        "case_count": len(payload.get("cases") or []),
        "failures": list(payload.get("failures") or []),
    }
    return result


def _case_catalog(mode: str) -> List[MeasurementCase]:
    return list(M0_CASES if str(mode).strip().lower() == "trust" else M1_CASES)


def _measurement_ready_snapshot(status: Dict[str, Any]) -> Dict[str, Any]:
    lidar_odom = dict(status.get("lidar_odom_status") or {})
    lidar = dict(status.get("lidar") or {})
    latest_age = _finite(lidar_odom.get("latest_age_s"))
    if latest_age is None:
        latest_age = _finite(lidar.get("latest_age_s"))
    confidence = _finite(lidar_odom.get("latest_confidence"))
    if confidence is None:
        confidence = _finite(lidar_odom.get("confidence"))
    if confidence is None:
        confidence = _finite(lidar.get("latest_confidence"))
    localization_health = str(lidar_odom.get("localization_health", "") or "").strip().upper()
    localization_health_reason = str(lidar_odom.get("localization_health_reason", "") or "").strip()
    apply_status = str(lidar_odom.get("control_loop_lidar_apply_status", "") or "").strip()
    lidar_health = str(status.get("lidar_health", lidar_odom.get("lidar_health", "")) or "").strip().upper()
    state = str(status.get("state", "") or "").strip().upper()
    idle_stationary_hold = bool(
        localization_health == "DEGRADED"
        and localization_health_reason == "delivery_missing_idle_stationary_guard"
        and apply_status.lower() in {"", "not_called"}
    )
    ready = bool(
        state == "IDLE"
        and lidar_health == "OK"
        and (localization_health in {"TRACKING", "LOCALIZED"} or idle_stationary_hold)
        and latest_age is not None
        and latest_age <= 0.35
        and confidence is not None
        and confidence >= MIN_LIDAR_CONFIDENCE
    )
    return {
        "ready": bool(ready),
        "state": state,
        "lidar_health": lidar_health,
        "localization_health": localization_health,
        "localization_health_reason": localization_health_reason,
        "idle_stationary_hold": idle_stationary_hold,
        "latest_age_s": latest_age,
        "latest_confidence": confidence,
        "control_loop_lidar_apply_status": apply_status,
    }


def _wait_measurement_ready_after_reset(timeout_s: float, *, stable_samples: int = 10) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    stable = 0
    last: Dict[str, Any] = {}
    while time.monotonic() <= deadline:
        status = _read_json(STATUS_PATH) or {}
        snap = _measurement_ready_snapshot(status)
        last = snap
        if bool(snap.get("ready", False)):
            stable += 1
            if stable >= max(1, int(stable_samples)):
                out = dict(snap)
                out["stable_samples"] = int(stable)
                out["ok"] = True
                return out
        else:
            stable = 0
        time.sleep(0.1)
    out = dict(last)
    out["stable_samples"] = int(stable)
    out["ok"] = False
    out["timeout_s"] = float(timeout_s)
    return out


def _manual_reposition_suspected(status: Dict[str, Any]) -> Dict[str, Any]:
    lidar_odom = dict(status.get("lidar_odom_status") or {})
    guard = dict(lidar_odom.get("idle_stationary_guard") or {})
    delta_m = _finite(guard.get("delta_m"))
    delta_rad = _finite(guard.get("delta_rad"))
    apply_status = str(lidar_odom.get("control_loop_lidar_apply_status", "") or "")
    reason = str(lidar_odom.get("localization_health_reason", "") or "")
    suspected = bool(
        (delta_m is not None and delta_m > 0.06)
        or (delta_rad is not None and abs(delta_rad) > 0.12)
        or apply_status.strip().lower() == "rejected_idle_stationary_guard"
    )
    return {
        "suspected": bool(suspected),
        "delta_m": delta_m,
        "delta_rad": delta_rad,
        "control_loop_lidar_apply_status": apply_status,
        "localization_health_reason": reason,
    }


def _runtime_limit_state_matches(
    status: Dict[str, Any],
    *,
    expected_level: int,
    expected_gear_ratio: float,
    expected_v_max_mps: Optional[float],
) -> bool:
    motion_state = dict(status.get("motion_state") or {})
    pwm = dict(status.get("pwm") or {})
    level = _safe_int(
        status.get("speed_level"),
        _safe_int(motion_state.get("gear_level"), -1),
    )
    ratio = _safe_float(motion_state.get("gear_ratio"), -1.0)
    active_v_max = _safe_float(motion_state.get("v_max_active"), -1.0)
    return bool(
        level == int(expected_level)
        and abs(ratio - float(expected_gear_ratio)) <= 0.002
        and (
            expected_v_max_mps is None
            or abs(active_v_max - float(expected_v_max_mps)) <= 0.002
        )
        and str(status.get("state", "") or "").strip().upper() == "IDLE"
        and abs(_safe_float(status.get("v_target"), 0.0)) <= 0.002
        and abs(_safe_float(status.get("omega_target"), 0.0)) <= 0.01
        and max(
            abs(_safe_float(pwm.get("left"), 0.0)),
            abs(_safe_float(pwm.get("right"), 0.0)),
        )
        <= 0.02
    )


def _wait_runtime_limit_state(
    *,
    expected_level: int,
    expected_gear_ratio: float,
    expected_v_max_mps: Optional[float],
    timeout_s: float,
) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last: Dict[str, Any] = {}
    while time.monotonic() < deadline:
        last = dict(_read_json(STATUS_PATH) or {})
        if _runtime_limit_state_matches(
            last,
            expected_level=int(expected_level),
            expected_gear_ratio=float(expected_gear_ratio),
            expected_v_max_mps=expected_v_max_mps,
        ):
            return last
        time.sleep(0.05)
    return last


def _run_pause(
    seconds: float,
    *,
    compact: bool,
    label: str,
    token: str,
    reset_pos_after_pause: bool,
    post_reset_ready_timeout_s: float,
) -> Dict[str, Any]:
    pause_s = max(0.0, float(seconds))
    event: Dict[str, Any] = {
        "after_case": str(label),
        "pause_s": float(pause_s),
        "reset_pos_after_pause": bool(reset_pos_after_pause),
        "reset_pos_ok": None,
        "ok": True,
        "error": "",
    }
    if pause_s <= 0.0:
        return event
    if not bool(compact):
        print(f"pause_before_next_case_s={pause_s:.1f} after={label}", flush=True)
    deadline = time.monotonic() + pause_s
    while time.monotonic() < deadline:
        time.sleep(min(0.5, max(0.01, deadline - time.monotonic())))
    pre_reset_status = _read_json(STATUS_PATH) or {}
    reposition = _manual_reposition_suspected(pre_reset_status)
    event["manual_reposition_suspected"] = reposition
    if bool(reset_pos_after_pause):
        try:
            reset_cmd = _send_command_checked("reset_pos", token=str(token), timeout_s=6.0)
            event["reset_pos_ok"] = True
            event["reset_pos_command"] = dict(reset_cmd)
            _wait_for_status_progress(min_increments=1, timeout_s=3.0)
        except Exception as exc:
            event["ok"] = False
            event["reset_pos_ok"] = False
            event["error"] = str(exc)
    if bool(event.get("ok", True)):
        ready = _wait_measurement_ready_after_reset(timeout_s=float(post_reset_ready_timeout_s))
        event["post_pause_measurement_ready"] = ready
        if not bool(ready.get("ok", False)):
            event["ok"] = False
            event["error"] = "post_pause_measurement_not_ready"
    return event


def run(
    args: argparse.Namespace,
    *,
    run_spec: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    spec = dict(run_spec or {})
    mode = str(args.mode).strip().lower()
    if mode not in ("trust", "baseline"):
        raise ValueError(f"unknown_mode:{mode}")

    phase = str(spec.get("phase") or ("M0" if mode == "trust" else "M1"))
    test_name = str(
        spec.get("test_name")
        or ("M0_measurement_trust_live" if mode == "trust" else "M1_motion_baseline_live")
    )
    result_path = Path(
        spec.get("result_path")
        or (LATEST_TRUST_RESULT_PATH if mode == "trust" else LATEST_BASELINE_RESULT_PATH)
    )
    sample_path = Path(
        spec.get("sample_path")
        or (LATEST_TRUST_SAMPLES_PATH if mode == "trust" else LATEST_BASELINE_SAMPLES_PATH)
    )
    cases = list(spec.get("cases") or _case_catalog(mode))
    if args.cases:
        requested = {item.strip() for item in str(args.cases).split(",") if item.strip()}
        cases = [case for case in cases if case.name in requested]
    base_cases = list(cases)
    cases = [case for _ in range(max(1, int(args.repeat_count))) for case in base_cases]
    embedded_m0_mini = bool(mode == "baseline" and args.embedded_m0_mini)
    expected_embedded_case_names = list(
        spec.get("embedded_m0_mini_case_names")
        or [case.name for case in M1_CASES]
    )
    m0_mini_source_profile = str(
        spec.get("m0_mini_source_profile") or "M1_motion_baseline_live"
    )
    phase_state_path = (
        None
        if not str(spec.get("phase_state_path") or "").strip()
        else Path(str(spec.get("phase_state_path")))
    )

    def publish_phase_state(**payload: Any) -> None:
        if phase_state_path is None:
            return
        _write_json_atomic(
            phase_state_path,
            {
                "schema": "R2B4_LIVE_VALIDATOR_PHASE_STATE_V1",
                "test": str(test_name),
                "phase": str(phase),
                "updated_at_utc": _now_iso_utc(),
                **payload,
            },
        )

    _reset_jsonl(sample_path)
    publish_phase_state(status="preflight", case_index=0, case_count=len(cases))
    dependency = {"required": False, "ok": True, "errors": []}
    if embedded_m0_mini:
        dependency = {
            "required": True,
            "ok": False,
            "source": "embedded_m0_mini",
            "path": str(LATEST_MINI_TRUST_RESULT_PATH),
            "errors": ["m0_mini_pending"],
            "payload_summary": {},
        }
    elif mode == "baseline" and bool(args.require_measurement_trust):
        dependency = _dependency_status(max_age_s=float(args.measurement_trust_max_age_s))
        if not bool(dependency.get("ok", False)):
            result = {
                "success": False,
                "status": "FAIL",
                "phase": phase,
                "test": test_name,
                "started_at_utc": _now_iso_utc(),
                "cases_requested": [case.name for case in cases],
                "cases": [],
                "measurement_trust_dependency": dependency,
                "m1_speed_map_execution_contract": (
                    _m1_speed_map_execution_contract(
                        required=bool(
                            mode == "baseline" and str(phase) == "M1"
                        )
                    )
                ),
                "failures": list(dependency.get("errors") or []),
                "artifact": str(result_path),
                "samples_artifact": str(sample_path),
            }
            _write_json_atomic(result_path, result)
            return result

    control_mode_error = ""
    try:
        start_status = _ensure_control_mode(str(args.control_mode), token=str(args.token))
    except Exception as exc:
        control_mode_error = str(exc)
        start_status = _wait_for_status_progress(min_increments=1, timeout_s=5.0)
    start_status_summary = {
        "state": str(start_status.get("state", "") or ""),
        "startup_ready": bool((start_status.get("startup") or {}).get("ready", False)),
        "odometry_mode": str(start_status.get("odometry_mode", "") or ""),
        "lidar_health": str(start_status.get("lidar_health", "") or ""),
        "control_mode": str(start_status.get("control_mode", "") or ""),
    }
    start_gc_runtime = dict(start_status.get("gc_runtime") or {})
    start_encoder_canonical = dict(
        (((start_status.get("encoder") or {}).get("canonical")) or {})
    )
    start_status_summary["gc_runtime"] = {
        "schema": str(start_gc_runtime.get("schema", "") or ""),
        "policy": str(start_gc_runtime.get("policy", "") or ""),
        "automatic_enabled": bool(start_gc_runtime.get("automatic_enabled", False)),
        "automatic_disabled_contract_ok": bool(
            start_gc_runtime.get("automatic_disabled_contract_ok", False)
        ),
    }
    start_motion_state = dict(start_status.get("motion_state") or {})
    start_profile = dict(start_motion_state.get("profile") or {})
    profile_v_max = _safe_float(start_profile.get("v_max"), 0.0)
    wheel_min_mps = _safe_float(start_profile.get("v_min"), 0.0)
    track_width_m = _safe_float(
        ((start_status.get("motion_command") or {}).get("track_width_m")),
        0.3557,
    )
    if track_width_m <= 0.0:
        track_width_m = 0.3557
    initial_gear_ratio = _safe_float(start_motion_state.get("gear_ratio"), 0.0)
    initial_speed_level = _safe_int(
        start_status.get("speed_level"),
        _safe_int(start_motion_state.get("gear_level"), 0),
    )
    validation_gear_ratio = max(0.1, min(1.0, float(VALIDATION_SPEED_LEVEL) / 9.0))
    validation_wheel_max_mps = max(
        float(wheel_min_mps),
        float(profile_v_max) * float(validation_gear_ratio),
    )
    runtime_limit_event: Dict[str, Any] = {
        "required": True,
        "ok": False,
        "profile_v_max_mps": float(profile_v_max),
        "wheel_min_mps": float(wheel_min_mps),
        "minimum_required_wheel_max_mps": float(VALIDATION_WHEEL_MAX_MPS),
        "requested_speed_level": int(VALIDATION_SPEED_LEVEL),
        "expected_wheel_max_mps": float(validation_wheel_max_mps),
        "initial_gear_ratio": float(initial_gear_ratio),
        "initial_speed_level": int(initial_speed_level),
    }
    motion_contract = _validation_motion_contract(
        cases,
        track_width_m=float(track_width_m),
        wheel_min_mps=float(wheel_min_mps),
        wheel_max_mps=float(validation_wheel_max_mps),
    )

    preflight_failures: List[str] = []
    if control_mode_error:
        preflight_failures.append("control_mode_apply_failed")
    if str(start_status_summary.get("control_mode", "")).strip().upper() != "UNIFIED":
        preflight_failures.append("control_mode_target_not_applied")
    if str(start_status_summary.get("odometry_mode", "")).strip().upper() != "LIDAR_FIRST":
        preflight_failures.append("odometry_mode_not_lidar_first")
    if str(start_status_summary.get("lidar_health", "")).strip().upper() != "OK":
        preflight_failures.append("lidar_health_not_ok_at_start")
    if str(start_gc_runtime.get("schema", "") or "") != "MOTION_GC_CONTRACT_V1":
        preflight_failures.append("gc_runtime_contract_missing")
    if str(start_gc_runtime.get("policy", "") or "") not in {"automatic", "motion_safe"}:
        preflight_failures.append("gc_runtime_policy_invalid")
    if (
        str(start_gc_runtime.get("policy", "") or "") == "motion_safe"
        and not bool(start_gc_runtime.get("automatic_disabled_contract_ok", False))
    ):
        preflight_failures.append("gc_automatic_collector_not_disabled")
    if not {
        "timing_valid",
        "timing_gap_count",
        "motion_timing_gap_count",
        "timing_gap_threshold_s",
    }.issubset(start_encoder_canonical):
        preflight_failures.append("encoder_timing_contract_missing")
    if validation_wheel_max_mps + 1e-9 < float(VALIDATION_WHEEL_MAX_MPS):
        preflight_failures.append("validation_wheel_max_exceeds_profile")
    if wheel_min_mps <= 0.0:
        preflight_failures.append("validation_wheel_min_missing")
    if not bool(motion_contract.get("ok", False)):
        preflight_failures.extend(list(motion_contract.get("failures") or []))
    if embedded_m0_mini and [case.name for case in cases] != expected_embedded_case_names:
        preflight_failures.append("m0_mini_m1_case_contract_mismatch")

    if not preflight_failures:
        try:
            _safe_stop_best_effort(token=str(args.token))
            runtime_limit_event["apply_command"] = _send_command_checked(
                "set_speed",
                token=str(args.token),
                timeout_s=4.0,
                level=int(VALIDATION_SPEED_LEVEL),
                apply_state=False,
                motion_source=DEFAULT_MOTION_SOURCE,
            )
            applied_status = _wait_runtime_limit_state(
                expected_level=int(VALIDATION_SPEED_LEVEL),
                expected_gear_ratio=float(validation_gear_ratio),
                expected_v_max_mps=float(validation_wheel_max_mps),
                timeout_s=3.0,
            )
            applied_motion_state = dict(applied_status.get("motion_state") or {})
            applied_v_max = _safe_float(applied_motion_state.get("v_max_active"), 0.0)
            applied_pwm = dict(applied_status.get("pwm") or {})
            runtime_limit_event["applied_v_max_mps"] = float(applied_v_max)
            runtime_limit_event["applied_speed_level"] = _safe_int(
                applied_status.get("speed_level"),
                -1,
            )
            runtime_limit_event["applied_gear_ratio"] = _safe_float(
                applied_motion_state.get("gear_ratio"),
                0.0,
            )
            runtime_limit_event["applied_state"] = str(applied_status.get("state", "") or "")
            runtime_limit_event["applied_v_target_mps"] = _safe_float(applied_status.get("v_target"), 0.0)
            runtime_limit_event["applied_omega_target_rad_s"] = _safe_float(
                applied_status.get("omega_target"),
                0.0,
            )
            runtime_limit_event["applied_pwm"] = {
                "left": _safe_float(applied_pwm.get("left"), 0.0),
                "right": _safe_float(applied_pwm.get("right"), 0.0),
            }
            runtime_limit_event["ok"] = bool(
                abs(float(applied_v_max) - float(validation_wheel_max_mps)) <= 0.002
                and int(runtime_limit_event["applied_speed_level"]) == int(VALIDATION_SPEED_LEVEL)
                and str(runtime_limit_event["applied_state"]).strip().upper() == "IDLE"
                and abs(float(runtime_limit_event["applied_v_target_mps"])) <= 0.002
                and abs(float(runtime_limit_event["applied_omega_target_rad_s"])) <= 0.01
                and max(
                    abs(float(runtime_limit_event["applied_pwm"]["left"])),
                    abs(float(runtime_limit_event["applied_pwm"]["right"])),
                ) <= 0.02
            )
            if not bool(runtime_limit_event["ok"]):
                preflight_failures.append("validation_runtime_wheel_max_not_applied")
        except Exception as exc:
            runtime_limit_event["error"] = str(exc)
            preflight_failures.append("validation_runtime_limit_apply_failed")

    initial_reset_event: Dict[str, Any] = {
        "required": bool(mode == "baseline" and args.reset_pos_after_pause),
        "ok": True,
    }
    if not preflight_failures and bool(initial_reset_event["required"]):
        try:
            _safe_stop_best_effort(token=str(args.token))
            reset_cmd = _send_command_checked("reset_pos", token=str(args.token), timeout_s=6.0)
            initial_reset_event["reset_pos_command"] = dict(reset_cmd)
            _wait_for_status_progress(min_increments=1, timeout_s=3.0)
            ready = _wait_measurement_ready_after_reset(
                timeout_s=float(args.post_reset_ready_timeout_s)
            )
            initial_reset_event["measurement_ready"] = ready
            if not bool(ready.get("ok", False)):
                initial_reset_event["ok"] = False
                initial_reset_event["error"] = "initial_reset_measurement_not_ready"
                preflight_failures.append("initial_reset_measurement_not_ready")
        except Exception as exc:
            initial_reset_event["ok"] = False
            initial_reset_event["error"] = str(exc)
            preflight_failures.append("initial_reset_pos_failed")

    results: List[Dict[str, Any]] = []
    invalid_case_attempts: List[Dict[str, Any]] = []
    pause_events: List[Dict[str, Any]] = []
    pause_failures: List[str] = []
    m0_mini_block: Dict[str, Any] = {
        "required": bool(embedded_m0_mini),
        "ok": not bool(embedded_m0_mini),
        "contract_id": M0_MINI_CONTRACT_ID,
        "artifact": str(LATEST_MINI_TRUST_RESULT_PATH),
        "failures": [],
    }
    if not preflight_failures:
        try:
            _safe_stop_best_effort(token=str(args.token))
            for idx, case in enumerate(cases):
                result: Dict[str, Any] = {}
                retry_recovery_failed = False
                max_case_attempts = max(1, int(args.max_case_attempts))
                for case_attempt in range(1, max_case_attempts + 1):
                    publish_phase_state(
                        status="moving",
                        case_index=int(idx + 1),
                        case_count=int(len(cases)),
                        case=str(case.name),
                        case_attempt=int(case_attempt),
                        max_case_attempts=int(max_case_attempts),
                        caster_pair=str(case.caster_pair),
                        caster_orientation=str(case.caster_orientation),
                        operator_instruction_hu=str(case.operator_instruction_hu),
                    )
                    result = _run_case(
                        case,
                        token=str(args.token),
                        poll_s=float(args.poll_s),
                        keepalive_s=float(args.keepalive_s),
                        stop_timeout_s=float(args.stop_timeout_s),
                        sample_path=sample_path,
                        case_attempt=int(case_attempt),
                    )
                    retry_reason = _case_retry_reason(
                        result,
                        retry_all_measurement_failures=bool(
                            mode == "trust" and args.retry_all_trust_failures
                        ),
                    )
                    if (
                        bool(result.get("success", False))
                        or not retry_reason
                        or case_attempt >= max_case_attempts
                    ):
                        break
                    invalid_case_attempts.append(
                        _invalid_case_attempt_record(
                            result,
                            retry_reason=retry_reason,
                        )
                    )
                    print(
                        "LIVE_MEASUREMENT_RETRY "
                        f"case={case.name} attempt={case_attempt}/{max_case_attempts} "
                        f"reason={retry_reason} pause_s={float(args.inter_case_pause_s):.1f}",
                        flush=True,
                    )
                    publish_phase_state(
                        status="invalid_attempt_recovery",
                        case_index=int(idx + 1),
                        case_count=int(len(cases)),
                        case=str(case.name),
                        case_attempt=int(case_attempt),
                        max_case_attempts=int(max_case_attempts),
                        retry_reason=str(retry_reason),
                        pause_s=float(args.inter_case_pause_s),
                    )
                    retry_pause = _run_pause(
                        float(args.inter_case_pause_s),
                        compact=bool(args.compact),
                        label=f"retry_{case.name}_{case_attempt}",
                        token=str(args.token),
                        reset_pos_after_pause=True,
                        post_reset_ready_timeout_s=float(
                            args.post_reset_ready_timeout_s
                        ),
                    )
                    retry_pause["event"] = "invalid_case_attempt_recovery"
                    retry_pause["case"] = str(case.name)
                    retry_pause["case_attempt"] = int(case_attempt)
                    retry_pause["retry_reason"] = str(retry_reason)
                    pause_events.append(retry_pause)
                    if not bool(retry_pause.get("ok", False)):
                        pause_failures.append(
                            f"retry_after_{case.name}:measurement_recovery_failed"
                        )
                        retry_recovery_failed = True
                        break
                results.append(result)
                if retry_recovery_failed:
                    break
                if embedded_m0_mini and idx == 0:
                    mini_result = _m0_mini_result(
                        result,
                        source_profile=m0_mini_source_profile,
                    )
                    _write_json_atomic(LATEST_MINI_TRUST_RESULT_PATH, mini_result)
                    mini_allows_continuation = _m0_mini_allows_continuation(
                        mini_result
                    )
                    m0_mini_block = {
                        "required": True,
                        "ok": bool(mini_allows_continuation),
                        "contract_id": M0_MINI_CONTRACT_ID,
                        "artifact": str(LATEST_MINI_TRUST_RESULT_PATH),
                        "failures": list(mini_result.get("failures") or []),
                        "warnings": [
                            dict(item)
                            for item in list(mini_result.get("warnings") or [])
                            if isinstance(item, dict)
                        ],
                        "sensor_surface": dict(
                            (mini_result.get("measurement_trust") or {}).get(
                                "sensor_surface"
                            )
                            or {}
                        ),
                    }
                    dependency = {
                        "required": True,
                        "ok": bool(mini_allows_continuation),
                        "source": "embedded_m0_mini",
                        "path": str(LATEST_MINI_TRUST_RESULT_PATH),
                        "errors": (
                            []
                            if bool(mini_allows_continuation)
                            else ["m0_mini_failed"]
                        ),
                        "payload_summary": {
                            "success": bool(mini_allows_continuation),
                            "phase": "M0_MINI",
                            "contract_id": M0_MINI_CONTRACT_ID,
                            "case_count": 1,
                            "failures": list(mini_result.get("failures") or []),
                            "warnings": [
                                dict(item)
                                for item in list(mini_result.get("warnings") or [])
                                if isinstance(item, dict)
                            ],
                        },
                    }
                    if not bool(mini_allows_continuation):
                        pause_failures.append("m0_mini_gate_failed")
                        break
                if idx < len(cases) - 1:
                    next_case = cases[idx + 1]
                    publish_phase_state(
                        status="operator_pause",
                        case_index=int(idx + 1),
                        case_count=int(len(cases)),
                        completed_case=str(case.name),
                        next_case=str(next_case.name),
                        next_caster_pair=str(next_case.caster_pair),
                        next_caster_orientation=str(next_case.caster_orientation),
                        operator_instruction_hu=str(
                            next_case.operator_instruction_hu
                        ),
                        pause_s=float(args.inter_case_pause_s),
                    )
                    pause_event = _run_pause(
                        float(args.inter_case_pause_s),
                        compact=bool(args.compact),
                        label=str(case.name),
                        token=str(args.token),
                        reset_pos_after_pause=bool(args.reset_pos_after_pause),
                        post_reset_ready_timeout_s=float(args.post_reset_ready_timeout_s),
                    )
                    pause_events.append(pause_event)
                    if not bool(pause_event.get("ok", False)):
                        pause_failures.append(f"pause_after_{case.name}:reset_pos_after_pause_failed")
                        break
        finally:
            _safe_stop_best_effort(token=str(args.token))

    if "apply_command" in runtime_limit_event:
        try:
            _safe_stop_best_effort(token=str(args.token))
            runtime_limit_event["restore_command"] = _send_command_checked(
                "set_speed",
                token=str(args.token),
                timeout_s=4.0,
                level=int(initial_speed_level),
                apply_state=False,
                motion_source=DEFAULT_MOTION_SOURCE,
            )
            restored_status = _wait_runtime_limit_state(
                expected_level=int(initial_speed_level),
                expected_gear_ratio=float(initial_gear_ratio),
                expected_v_max_mps=None,
                timeout_s=3.0,
            )
            restored_state = dict(restored_status.get("motion_state") or {})
            restored_ratio = _safe_float(restored_state.get("gear_ratio"), -1.0)
            restored_level = _safe_int(restored_status.get("speed_level"), -1)
            runtime_limit_event["restored_gear_ratio"] = float(restored_ratio)
            runtime_limit_event["restored_speed_level"] = int(restored_level)
            runtime_limit_event["restored"] = bool(
                abs(float(restored_ratio) - float(initial_gear_ratio)) <= 0.002
                and int(restored_level) == int(initial_speed_level)
                and str(restored_status.get("state", "") or "").strip().upper() == "IDLE"
            )
            if not bool(runtime_limit_event["restored"]):
                pause_failures.append("validation_runtime_limit_restore_failed")
        except Exception as exc:
            runtime_limit_event["restore_error"] = str(exc)
            runtime_limit_event["restored"] = False
            pause_failures.append("validation_runtime_limit_restore_failed")

    failures: List[str] = list(preflight_failures) + list(pause_failures)
    for result in results:
        for failure in list(result.get("failures") or []):
            failures.append(f"{result.get('case')}:{failure}")

    repeatability: Dict[str, Any] = {"repeat_count": int(args.repeat_count), "groups": {}, "failures": []}
    grouped: Dict[str, List[Dict[str, Any]]] = {}
    for item in results:
        metrics = dict(item.get("metrics") or {})
        grouped.setdefault(str(metrics.get("kind", item.get("case", "")) or ""), []).append(metrics)
    for kind, rows in grouped.items():
        speed_ratios = [
            _finite((((row.get("command_fidelity") or {}).get("errors") or {}).get("linear_speed_ratio_vs_executed")))
            for row in rows
        ]
        speed_ratios = [float(value) for value in speed_ratios if value is not None]
        angle_values = [_repeatability_pose_angle_abs(row) for row in rows]
        angle_values = [float(value) for value in angle_values if value is not None]
        metric_values = speed_ratios if speed_ratios else angle_values
        mean_value = _mean_finite(metric_values)
        stddev = statistics.pstdev(metric_values) if len(metric_values) >= 2 else None
        cv = (
            None
            if stddev is None or mean_value is None or abs(float(mean_value)) <= 1e-6
            else float(stddev) / abs(float(mean_value))
        )
        repeatability["groups"][kind] = {
            "samples": len(metric_values),
            "mean": mean_value,
            "stddev": stddev,
            "coefficient_of_variation": cv,
            "angle_source": "EKF_POSE_ODOMETRY_SSOT" if angle_values and not speed_ratios else None,
        }
        if int(args.repeat_count) >= 2 and (cv is None or float(cv) > 0.10):
            repeatability["failures"].append(f"{kind}:repeatability_variation_high")

    left_angles = [
        value for value in (
            _repeatability_pose_angle_abs(row) for row in grouped.get("rotate_left", [])
        ) if value is not None
    ]
    right_angles = [
        value for value in (
            _repeatability_pose_angle_abs(row) for row in grouped.get("rotate_right", [])
        ) if value is not None
    ]
    left_mean = _mean_finite(left_angles)
    right_mean = _mean_finite(right_angles)
    pivot_lr_difference_ratio = None
    if left_mean is not None and right_mean is not None:
        pivot_lr_difference_ratio = abs(float(left_mean) - float(right_mean)) / max(
            abs(float(left_mean)), abs(float(right_mean)), 1e-6
        )
        if pivot_lr_difference_ratio > 0.10 and str(phase) != "M1":
            repeatability["failures"].append("pivot_left_right_difference_high")
    repeatability["pivot_left_right"] = {
        "source": "EKF_POSE_ODOMETRY_SSOT",
        "left_mean_abs_deg": left_mean,
        "right_mean_abs_deg": right_mean,
        "difference_ratio": pivot_lr_difference_ratio,
    }
    failures.extend(list(repeatability["failures"]))

    timing_gap_warning_summary = _timing_gap_warning_summary(results)
    warnings: List[Dict[str, Any]] = []
    if int(timing_gap_warning_summary.get("case_count", 0)) > 0:
        warnings.append(
            {
                "code": "encoder_timing_gap",
                "severity": "WARNING",
                "count": int(
                    timing_gap_warning_summary.get(
                        "warning_count", 0
                    )
                ),
                "summary": timing_gap_warning_summary,
                "message_hu": (
                    "Encoder timing-gap WARNING: a futási minőség, safety és "
                    "sensor-truth kapuk PASS esetén nem rontja a verdictet; "
                    "a számláló jövőbeni trendfigyelést igényel."
                ),
            }
        )

    sensor_surface = {
        "encoder_cases": sum(1 for item in results if bool(((item.get("metrics") or {}).get("encoder") or {}).get("available", False))),
        "imu_cases": sum(1 for item in results if bool(((item.get("metrics") or {}).get("imu") or {}).get("available", False))),
        "lidar_cases": sum(1 for item in results if bool(((item.get("metrics") or {}).get("lidar") or {}).get("enabled_seen", False))),
        "ekf_cases": sum(1 for item in results if bool((item.get("metrics") or {}).get("ekf"))),
        "motor_pwm_cases": sum(
            1
            for item in results
            if max(
                _safe_float((((item.get("metrics") or {}).get("motor_pwm") or {}).get("max_abs_left")), 0.0),
                _safe_float((((item.get("metrics") or {}).get("motor_pwm") or {}).get("max_abs_right")), 0.0),
            )
            >= MIN_PWM_OBSERVED
        ),
    }
    cases_ok = len(results) == len(cases) and not failures
    m1_speed_map_scope = bool(
        mode == "baseline" and str(phase) == "M1"
    )
    trust_block = {
        "ok": bool(cases_ok),
        "sensor_surface": sensor_surface,
        "failures": list(dict.fromkeys(failures)),
    }
    result = {
        "success": bool(cases_ok),
        "status": "PASS" if cases_ok else "FAIL",
        "phase": phase,
        "test": test_name,
        "started_at_utc": _now_iso_utc(),
        "mode": mode,
        "command_path": (
            "set_twist and rotate_to_heading via runtime/commands.jsonl "
            f"({DEFAULT_MOTION_SOURCE}, normal v2 executor, zero-twist stop)"
        ),
        "start_status": start_status_summary,
        "control_mode_target": "UNIFIED",
        "control_mode_error": str(control_mode_error),
        "initial_reset_event": initial_reset_event,
        "runtime_limit_event": runtime_limit_event,
        "validation_motion_contract": motion_contract,
        "m1_speed_map_execution_contract": (
            _m1_speed_map_execution_contract(
                required=m1_speed_map_scope
            )
        ),
        "m1_caster_influence_contract": {
            "required": bool(mode == "baseline"),
            "contract_id": M1_CASTER_INFLUENCE_CONTRACT_ID,
            "orientation_truth": M1_CASTER_ORIENTATION,
            "case_pairs": dict(M1_CASTER_CASE_CONTRACTS),
            "transient_observation_s": float(
                CASTER_TRANSIENT_ALLOWANCE_S
            ),
            "nominal_wheel_mae_mps": float(
                WHEEL_SPEED_TRACKING_MAE_MAX_MPS
            ),
            "wheel_mae_relative_max": float(
                M1_CASTER_WHEEL_MAE_RELATIVE_MAX
            ),
            "wheel_mae_absolute_max_mps": float(
                M1_CASTER_WHEEL_MAE_ABSOLUTE_MAX_MPS
            ),
            "arc_angular_ratio_range": [
                float(M1_CASTER_ARC_ANGULAR_RATIO_MIN),
                float(M1_CASTER_ARC_ANGULAR_RATIO_MAX),
            ],
            "arc_angular_ratio_verdict": (
                "delegated_to_M2"
                if m1_speed_map_scope
                else "local"
            ),
            "unchanged_gates": [
                "executed_command",
                "linear_speed",
                "whole_phase_distance",
                "integrated_distance",
                "safety",
                "sensor_truth",
                "timing",
                "endpoint",
                "stop_start",
                "normal_stop",
            ],
        },
        "measurement_trust_dependency": dependency,
        "m0_mini": m0_mini_block,
        "cases_requested": [case.name for case in cases],
        "cases": results,
        "invalid_case_attempts": invalid_case_attempts,
        "invalid_case_attempt_count": len(invalid_case_attempts),
        "automatic_case_remeasurement": bool(int(args.max_case_attempts) > 1),
        "max_case_attempts": max(1, int(args.max_case_attempts)),
        "repeatability": repeatability,
        "warnings": warnings,
        "warning_summary": timing_gap_warning_summary,
        "pause_events": pause_events,
        "reset_pos_after_pause": bool(args.reset_pos_after_pause),
        "measurement_trust": trust_block if mode == "trust" else {},
        "baseline": {
            "ok": bool(cases_ok),
            "movement_quality_not_tuned": True,
            "manual_reposition_pause_s": float(args.inter_case_pause_s),
            "reset_pos_after_pause": bool(args.reset_pos_after_pause),
            "failures": list(dict.fromkeys(failures)),
        } if mode == "baseline" else {},
        "failures": list(dict.fromkeys(failures)),
        "artifact": str(result_path),
        "samples_artifact": str(sample_path),
    }
    _write_json_atomic(result_path, result)
    publish_phase_state(
        status="complete",
        success=bool(result.get("success", False)),
        failures=list(result.get("failures") or []),
        warnings=list(result.get("warnings") or []),
        artifact=str(result_path),
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live M0/M1 measurement trust and motion baseline validator.")
    parser.add_argument("--mode", choices=("trust", "baseline"), default="trust")
    parser.add_argument("--control-mode", choices=("UNIFIED",), default="UNIFIED")
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--cases", default="")
    parser.add_argument("--repeat-count", type=int, default=1)
    parser.add_argument("--max-case-attempts", type=int, default=1)
    parser.add_argument(
        "--retry-all-trust-failures",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--poll-s", type=float, default=0.05)
    parser.add_argument("--keepalive-s", type=float, default=0.18)
    parser.add_argument("--stop-timeout-s", type=float, default=5.0)
    parser.add_argument("--inter-case-pause-s", type=float, default=10.0)
    parser.add_argument("--reset-pos-after-pause", action="store_true")
    parser.add_argument("--post-reset-ready-timeout-s", type=float, default=8.0)
    parser.add_argument("--require-measurement-trust", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--measurement-trust-max-age-s", type=float, default=3600.0)
    parser.add_argument("--embedded-m0-mini", action="store_true")
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        result = run(args)
    except BootstrapGuardError as exc:
        result = {
            "success": False,
            "status": "FAIL",
            "error": f"bootstrap_guard_failed:{exc}",
            "failures": ["bootstrap_guard_failed"],
        }
    except Exception as exc:
        _safe_stop_best_effort(token=str(getattr(args, "token", DEFAULT_TOKEN)))
        mode = str(getattr(args, "mode", "trust"))
        path = LATEST_TRUST_RESULT_PATH if mode == "trust" else LATEST_BASELINE_RESULT_PATH
        result = {
            "success": False,
            "status": "FAIL",
            "phase": "M0" if mode == "trust" else "M1",
            "test": "M0_measurement_trust_live" if mode == "trust" else "M1_motion_baseline_live",
            "m1_speed_map_execution_contract": (
                _m1_speed_map_execution_contract(
                    required=bool(mode == "baseline")
                )
            ),
            "error": str(exc),
            "failures": ["validator_exception"],
            "artifact": str(path),
        }
        _write_json_atomic(path, result)

    if bool(getattr(args, "compact", False)):
        print(
            "LIVE_MEASUREMENT_VALIDATOR "
            f"test={result.get('test', '')} "
            f"phase={result.get('phase', '')} "
            f"result={'PASS' if result.get('success') else 'FAIL'} "
            f"cases={len(result.get('cases') or [])}/{len(result.get('cases_requested') or [])} "
            f"failures={','.join(result.get('failures') or []) or 'none'} "
            f"artifact={result.get('artifact', '')}"
        )
    else:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if bool(result.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
