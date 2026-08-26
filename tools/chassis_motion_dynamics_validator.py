#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Independent M2 chassis-motion dynamics validator.

M1 remains the promotion-blocking speed-map execution contract.  M2 consumes
the same canonical M1 measurements and owns the passive-caster, effective
track-width, slip, ARC curvature and pivot-dynamics verdicts.  An M2 result is
fail-closed inside its own scope, but is never an input to speed-map
acceptance or promotion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import test_artifacts_dir  # noqa: E402
from project_rules.bootstrap_guard import (  # noqa: E402
    BootstrapGuardError,
    ensure_agent_system_prompt_loaded,
)
from tools import live_motion_measurement_validator as m1  # noqa: E402


CONTRACT_ID = "R2B4_M2_CHASSIS_MOTION_DYNAMICS_V1"
PROFILE_NAME = "M2_chassis_motion_dynamics_live"
RESULT_PATH = (
    test_artifacts_dir()
    / "latest_M2_chassis_motion_dynamics_live.json"
)
SOURCE_M1_PATH = m1.LATEST_BASELINE_RESULT_PATH

ARC_PHYSICAL_ANGULAR_RATIO_MIN = 0.75
ARC_PHYSICAL_ANGULAR_RATIO_MAX = 1.25
ARC_CURVATURE_RATIO_MIN = 0.70
ARC_CURVATURE_RATIO_MAX = 1.30
EFFECTIVE_TRACK_WIDTH_RELATIVE_ERROR_MAX = 0.34
GROUND_MOTION_RATIO_MIN = 0.65
GROUND_MOTION_RATIO_MAX = 1.35
ARC_LEFT_RIGHT_RATIO_DIFFERENCE_MAX = 0.20
STRAIGHT_YAW_DRIFT_MAX_DEG = 12.0
PIVOT_FINAL_ANGLE_ERROR_MAX_DEG = 10.0
PIVOT_OVERSHOOT_MAX_DEG = 10.0
PIVOT_LEFT_RIGHT_DIFFERENCE_MAX = 0.10

ACTIVE_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
PID_CONFIG_PATH = PROJECT_ROOT / "conf" / "vezerles.json"


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _ratio(numerator: Any, denominator: Any) -> Optional[float]:
    left = _finite(numerator)
    right = _finite(denominator)
    if left is None or right is None or abs(float(right)) <= 1e-9:
        return None
    return float(left) / float(right)


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_json(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"json_object_required:{path}")
    return payload


def _case_map(raw: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {
        str(item.get("case", "") or ""): dict(item)
        for item in list(raw.get("cases") or [])
        if isinstance(item, dict) and str(item.get("case", "") or "")
    }


def _between(value: Optional[float], minimum: float, maximum: float) -> bool:
    return bool(
        value is not None
        and float(minimum) <= abs(float(value)) <= float(maximum)
    )


def _gate(
    gates: Dict[str, Dict[str, Any]],
    failures: List[str],
    *,
    name: str,
    passed: bool,
    value: Any,
    requirement: str,
    failure: str,
) -> None:
    gates[name] = {
        "status": "PASS" if passed else "FAIL",
        "value": value,
        "requirement": requirement,
    }
    if not passed:
        failures.append(failure)


def _source_failures(raw: Dict[str, Any]) -> List[str]:
    failures: List[str] = []
    contract = dict(raw.get("m1_speed_map_execution_contract") or {})
    expected_cases = [case.name for case in m1.M1_CASES]
    requested = [str(item) for item in list(raw.get("cases_requested") or [])]
    rows = _case_map(raw)

    if str(raw.get("phase", "") or "") != "M1":
        failures.append("source_m1_phase_invalid")
    if (
        str(contract.get("contract_id", "") or "")
        != m1.M1_SPEED_MAP_EXECUTION_CONTRACT_ID
    ):
        failures.append("source_m1_contract_invalid")
    if not bool(contract.get("required", False)):
        failures.append("source_m1_contract_not_required")
    if not bool(contract.get("promotion_blocking", False)):
        failures.append("source_m1_promotion_scope_missing")
    if bool(contract.get("chassis_dynamics_verdict", True)):
        failures.append("source_m1_dynamics_not_delegated")
    if requested != expected_cases:
        failures.append("source_m1_case_order_invalid")
    if set(rows) != set(expected_cases):
        failures.append("source_m1_cases_incomplete")
    if str(raw.get("status", "") or "").upper() != "PASS" or not bool(
        raw.get("success", False)
    ):
        failures.append("source_m1_execution_not_pass")
    if any(
        not bool((rows.get(case_name) or {}).get("success", False))
        or list((rows.get(case_name) or {}).get("failures") or [])
        for case_name in expected_cases
    ):
        failures.append("source_m1_case_not_pass")
    if not bool((raw.get("m0_mini") or {}).get("ok", False)):
        failures.append("source_m0_mini_not_pass")
    return list(dict.fromkeys(failures))


def _ground_motion_ratios(metrics: Dict[str, Any]) -> Dict[str, Optional[float]]:
    shared = metrics.get("sensor_endpoint_shared_window")
    if isinstance(shared, dict):
        if bool(shared.get("available", False)):
            encoder_distance = _finite(
                (shared.get("encoder") or {}).get("average_delta_m")
            )
            ekf_distance = _finite(
                (shared.get("ekf_control") or {}).get("forward_delta_m")
            )
            lidar_chord = _finite(
                (shared.get("lidar") or {}).get("pose_chord_m")
            )
        else:
            encoder_distance = None
            ekf_distance = None
            lidar_chord = None
    else:
        # Historical M1 payload compatibility. New live M1 artifacts always
        # carry the fail-closed shared source-time contract.
        encoder_distance = _finite(
            (metrics.get("encoder") or {}).get("average_delta_m")
        )
        ekf_distance = _finite(
            (metrics.get("ekf") or {}).get("forward_delta_m")
        )
        lidar_chord = _finite(
            (metrics.get("lidar") or {}).get("pose_chord_m")
        )
    return {
        "ekf_vs_encoder": (
            None
            if encoder_distance is None or ekf_distance is None
            else _ratio(abs(float(ekf_distance)), abs(float(encoder_distance)))
        ),
        "lidar_chord_vs_encoder": (
            None
            if encoder_distance is None or lidar_chord is None
            else _ratio(abs(float(lidar_chord)), abs(float(encoder_distance)))
        ),
    }


def _arc_analysis(
    case_name: str,
    row: Dict[str, Any],
    *,
    nominal_track_width_m: float,
) -> Tuple[Dict[str, Any], List[str]]:
    failures: List[str] = []
    gates: Dict[str, Dict[str, Any]] = {}
    metrics = dict(row.get("metrics") or {})
    errors = dict(
        ((metrics.get("command_fidelity") or {}).get("errors") or {})
    )
    command = dict(metrics.get("command") or {})
    angular_ratio = _finite(
        errors.get("imu_angular_speed_ratio_vs_executed")
    )
    linear_ratio = _finite(errors.get("linear_speed_ratio_vs_executed"))
    curvature_ratio = (
        None
        if angular_ratio is None
        or linear_ratio is None
        or abs(float(linear_ratio)) <= 1e-9
        else abs(float(angular_ratio)) / abs(float(linear_ratio))
    )
    effective_track_width_m = (
        None
        if angular_ratio is None or abs(float(angular_ratio)) <= 1e-9
        else float(nominal_track_width_m) / abs(float(angular_ratio))
    )
    effective_track_width_relative_error = (
        None
        if effective_track_width_m is None
        or float(nominal_track_width_m) <= 1e-9
        else abs(
            float(effective_track_width_m)
            - float(nominal_track_width_m)
        )
        / float(nominal_track_width_m)
    )
    ground_motion_ratios = _ground_motion_ratios(metrics)
    slip_estimates = {
        source: (
            None if value is None else 1.0 - float(value)
        )
        for source, value in ground_motion_ratios.items()
    }

    _gate(
        gates,
        failures,
        name="physical_angular_response",
        passed=_between(
            angular_ratio,
            ARC_PHYSICAL_ANGULAR_RATIO_MIN,
            ARC_PHYSICAL_ANGULAR_RATIO_MAX,
        ),
        value=angular_ratio,
        requirement=(
            f"{ARC_PHYSICAL_ANGULAR_RATIO_MIN:.2f} <= "
            "|IMU omega / executed omega| <= "
            f"{ARC_PHYSICAL_ANGULAR_RATIO_MAX:.2f}"
        ),
        failure=f"{case_name}:physical_angular_response_out_of_range",
    )
    _gate(
        gates,
        failures,
        name="physical_curvature_response",
        passed=_between(
            curvature_ratio,
            ARC_CURVATURE_RATIO_MIN,
            ARC_CURVATURE_RATIO_MAX,
        ),
        value=curvature_ratio,
        requirement=(
            f"{ARC_CURVATURE_RATIO_MIN:.2f} <= "
            "(physical angular ratio / wheel-linear ratio) <= "
            f"{ARC_CURVATURE_RATIO_MAX:.2f}"
        ),
        failure=f"{case_name}:physical_curvature_response_out_of_range",
    )
    _gate(
        gates,
        failures,
        name="effective_track_width",
        passed=bool(
            effective_track_width_relative_error is not None
            and effective_track_width_relative_error
            <= EFFECTIVE_TRACK_WIDTH_RELATIVE_ERROR_MAX
        ),
        value={
            "nominal_m": float(nominal_track_width_m),
            "effective_m": effective_track_width_m,
            "relative_error": effective_track_width_relative_error,
        },
        requirement=(
            "kinematic effective track-width relative error <= "
            f"{EFFECTIVE_TRACK_WIDTH_RELATIVE_ERROR_MAX:.2f}"
        ),
        failure=f"{case_name}:effective_track_width_error_high",
    )
    for source, value in ground_motion_ratios.items():
        _gate(
            gates,
            failures,
            name=f"ground_motion_{source}",
            passed=_between(
                value,
                GROUND_MOTION_RATIO_MIN,
                GROUND_MOTION_RATIO_MAX,
            ),
            value=value,
            requirement=(
                f"{GROUND_MOTION_RATIO_MIN:.2f} <= "
                "|ground displacement / encoder displacement| <= "
                f"{GROUND_MOTION_RATIO_MAX:.2f}"
            ),
            failure=f"{case_name}:ground_motion_{source}_out_of_range",
        )

    phase_tracking = dict(metrics.get("phase_tracking") or {})
    caster = dict(phase_tracking.get("caster_influence") or {})
    return (
        {
            "case": case_name,
            "caster_orientation": command.get("caster_orientation"),
            "caster_pair": command.get("caster_pair"),
            "caster_influence": caster,
            "physical_angular_ratio": angular_ratio,
            "wheel_linear_ratio": linear_ratio,
            "curvature_ratio": curvature_ratio,
            "effective_track_width_m": effective_track_width_m,
            "effective_track_width_relative_error": (
                effective_track_width_relative_error
            ),
            "ground_motion_ratios": ground_motion_ratios,
            "slip_estimates": slip_estimates,
            "gates": gates,
            "status": "PASS" if not failures else "FAIL",
            "failures": failures,
        },
        failures,
    )


def _straight_analysis(
    case_name: str,
    row: Dict[str, Any],
) -> Tuple[Dict[str, Any], List[str]]:
    metrics = dict(row.get("metrics") or {})
    yaw_delta_deg = _finite((metrics.get("imu") or {}).get("yaw_delta_deg"))
    passed = bool(
        yaw_delta_deg is not None
        and abs(float(yaw_delta_deg)) <= STRAIGHT_YAW_DRIFT_MAX_DEG
    )
    failures = (
        [] if passed else [f"{case_name}:straight_yaw_drift_high"]
    )
    return (
        {
            "case": case_name,
            "imu_yaw_delta_deg": yaw_delta_deg,
            "requirement": (
                f"|IMU yaw delta| <= {STRAIGHT_YAW_DRIFT_MAX_DEG:.1f} deg"
            ),
            "status": "PASS" if passed else "FAIL",
            "failures": failures,
        },
        failures,
    )


def _pivot_analysis(
    case_name: str,
    row: Dict[str, Any],
) -> Tuple[Dict[str, Any], List[str]]:
    metrics = dict(row.get("metrics") or {})
    fidelity = dict(metrics.get("command_fidelity") or {})
    errors = dict(fidelity.get("errors") or {})
    transient = dict(fidelity.get("transient") or {})
    command = dict(metrics.get("command") or {})
    angle_error = _finite(errors.get("imu_angle_error_vs_requested_deg"))
    settling_s = _finite(transient.get("pivot_settling_time_s"))
    overshoot_deg = _finite(transient.get("pivot_overshoot_deg"))
    duration_s = _finite(command.get("duration_s"))
    gates: Dict[str, Dict[str, Any]] = {}
    failures: List[str] = []

    _gate(
        gates,
        failures,
        name="final_angle",
        passed=bool(
            angle_error is not None
            and abs(float(angle_error))
            <= PIVOT_FINAL_ANGLE_ERROR_MAX_DEG
        ),
        value=angle_error,
        requirement=(
            f"|IMU requested-angle error| <= "
            f"{PIVOT_FINAL_ANGLE_ERROR_MAX_DEG:.1f} deg"
        ),
        failure=f"{case_name}:pivot_final_angle_error_high",
    )
    _gate(
        gates,
        failures,
        name="settling",
        passed=bool(
            settling_s is not None
            and duration_s is not None
            and settling_s <= duration_s
        ),
        value=settling_s,
        requirement="pivot settling time <= commanded case duration",
        failure=f"{case_name}:pivot_settling_time_high",
    )
    _gate(
        gates,
        failures,
        name="overshoot",
        passed=bool(
            overshoot_deg is not None
            and overshoot_deg <= PIVOT_OVERSHOOT_MAX_DEG
        ),
        value=overshoot_deg,
        requirement=(
            f"pivot overshoot <= {PIVOT_OVERSHOOT_MAX_DEG:.1f} deg"
        ),
        failure=f"{case_name}:pivot_overshoot_high",
    )
    return (
        {
            "case": case_name,
            "imu_yaw_delta_deg": _finite(
                (metrics.get("imu") or {}).get("yaw_delta_deg")
            ),
            "gates": gates,
            "status": "PASS" if not failures else "FAIL",
            "failures": failures,
        },
        failures,
    )


def _symmetry_analysis(
    arcs: Iterable[Dict[str, Any]],
    pivots: Iterable[Dict[str, Any]],
) -> Tuple[Dict[str, Any], List[str]]:
    arc_map = {str(item.get("case")): item for item in arcs}
    pivot_map = {str(item.get("case")): item for item in pivots}
    failures: List[str] = []
    gates: Dict[str, Dict[str, Any]] = {}

    left_arc = _finite(
        (arc_map.get("arc_left") or {}).get("physical_angular_ratio")
    )
    right_arc = _finite(
        (arc_map.get("arc_right") or {}).get("physical_angular_ratio")
    )
    arc_difference = (
        None
        if left_arc is None or right_arc is None
        else abs(abs(float(left_arc)) - abs(float(right_arc)))
    )
    _gate(
        gates,
        failures,
        name="arc_left_right",
        passed=bool(
            arc_difference is not None
            and arc_difference <= ARC_LEFT_RIGHT_RATIO_DIFFERENCE_MAX
        ),
        value=arc_difference,
        requirement=(
            "left/right physical angular-ratio absolute difference <= "
            f"{ARC_LEFT_RIGHT_RATIO_DIFFERENCE_MAX:.2f}"
        ),
        failure="arc_left_right_physical_asymmetry_high",
    )

    left_pivot = _finite(
        (pivot_map.get("rotate_left") or {}).get("imu_yaw_delta_deg")
    )
    right_pivot = _finite(
        (pivot_map.get("rotate_right") or {}).get("imu_yaw_delta_deg")
    )
    pivot_difference = (
        None
        if left_pivot is None or right_pivot is None
        else abs(abs(float(left_pivot)) - abs(float(right_pivot)))
        / max(abs(float(left_pivot)), abs(float(right_pivot)), 1e-9)
    )
    _gate(
        gates,
        failures,
        name="pivot_left_right",
        passed=bool(
            pivot_difference is not None
            and pivot_difference <= PIVOT_LEFT_RIGHT_DIFFERENCE_MAX
        ),
        value=pivot_difference,
        requirement=(
            "left/right pivot absolute-angle relative difference <= "
            f"{PIVOT_LEFT_RIGHT_DIFFERENCE_MAX:.2f}"
        ),
        failure="pivot_left_right_difference_high",
    )
    return (
        {
            "gates": gates,
            "status": "PASS" if not failures else "FAIL",
            "failures": failures,
        },
        failures,
    )


def analyze_m1_result(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Build the deterministic M2 verdict from one versioned M1 artifact."""

    source_failures = _source_failures(raw)
    rows = _case_map(raw)
    motion_contract = dict(raw.get("validation_motion_contract") or {})
    nominal_track_width_m = _finite(motion_contract.get("track_width_m"))
    if nominal_track_width_m is None or nominal_track_width_m <= 0.0:
        nominal_track_width_m = 0.3557

    failures: List[str] = list(source_failures)
    straight_rows: List[Dict[str, Any]] = []
    arc_rows: List[Dict[str, Any]] = []
    pivot_rows: List[Dict[str, Any]] = []
    if not source_failures:
        for case_name in ("forward", "backward"):
            analysis, case_failures = _straight_analysis(
                case_name,
                rows[case_name],
            )
            straight_rows.append(analysis)
            failures.extend(case_failures)
        for case_name in ("arc_left", "arc_right"):
            analysis, case_failures = _arc_analysis(
                case_name,
                rows[case_name],
                nominal_track_width_m=float(nominal_track_width_m),
            )
            arc_rows.append(analysis)
            failures.extend(case_failures)
        for case_name in ("rotate_left", "rotate_right"):
            analysis, case_failures = _pivot_analysis(
                case_name,
                rows[case_name],
            )
            pivot_rows.append(analysis)
            failures.extend(case_failures)

    symmetry, symmetry_failures = _symmetry_analysis(arc_rows, pivot_rows)
    if not source_failures:
        failures.extend(symmetry_failures)
    else:
        symmetry = {
            "status": "NOT_RUN",
            "gates": {},
            "failures": [],
        }

    failures = list(dict.fromkeys(failures))
    success = not failures
    source_contract = dict(
        raw.get("m1_speed_map_execution_contract") or {}
    )
    return {
        "schema": "R2B4_M2_CHASSIS_MOTION_DYNAMICS_RESULT_V1",
        "contract_id": CONTRACT_ID,
        "test": PROFILE_NAME,
        "phase": "M2",
        "status": "PASS" if success else "FAIL",
        "success": success,
        "speed_map_promotion_blocking": False,
        "promotion_contract": {
            "included_in_speed_map_decision": False,
            "may_block_speed_map_acceptance": False,
            "may_block_speed_map_promotion": False,
        },
        "mutation_contract": {
            "speed_map_write_allowed": False,
            "pid_write_allowed": False,
            "motion_control_logic_write_allowed": False,
            "active_runtime_map_only": True,
        },
        "source_m1": {
            "contract_id": source_contract.get("contract_id"),
            "status": raw.get("status"),
            "success": bool(raw.get("success", False)),
            "case_count": len(rows),
            "failures": source_failures,
        },
        "thresholds": {
            "arc_physical_angular_ratio": [
                ARC_PHYSICAL_ANGULAR_RATIO_MIN,
                ARC_PHYSICAL_ANGULAR_RATIO_MAX,
            ],
            "arc_curvature_ratio": [
                ARC_CURVATURE_RATIO_MIN,
                ARC_CURVATURE_RATIO_MAX,
            ],
            "effective_track_width_relative_error_max": (
                EFFECTIVE_TRACK_WIDTH_RELATIVE_ERROR_MAX
            ),
            "ground_motion_ratio": [
                GROUND_MOTION_RATIO_MIN,
                GROUND_MOTION_RATIO_MAX,
            ],
            "arc_left_right_ratio_difference_max": (
                ARC_LEFT_RIGHT_RATIO_DIFFERENCE_MAX
            ),
            "straight_yaw_drift_max_deg": STRAIGHT_YAW_DRIFT_MAX_DEG,
            "pivot_final_angle_error_max_deg": (
                PIVOT_FINAL_ANGLE_ERROR_MAX_DEG
            ),
            "pivot_overshoot_max_deg": PIVOT_OVERSHOOT_MAX_DEG,
            "pivot_left_right_difference_max": (
                PIVOT_LEFT_RIGHT_DIFFERENCE_MAX
            ),
        },
        "nominal_track_width_m": float(nominal_track_width_m),
        "straight": straight_rows,
        "arcs": arc_rows,
        "pivots": pivot_rows,
        "symmetry": symmetry,
        "failures": failures,
    }


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    source_path = Path(args.source_m1)
    result_path = Path(args.result_path)
    live_execution: Dict[str, Any] = {
        "requested": not bool(args.analyze_only),
        "return_code": None,
        "source_artifact_fresh": bool(args.analyze_only),
        "error": "",
    }
    started_at_epoch_s = time.time()
    map_hash_before = _sha256_file(ACTIVE_MAP_PATH)
    pid_hash_before = _sha256_file(PID_CONFIG_PATH)

    if not bool(args.analyze_only):
        try:
            completed = subprocess.run(
                [
                    sys.executable,
                    "tools/live_motion_measurement_validator.py",
                    "--mode",
                    "baseline",
                    "--embedded-m0-mini",
                    "--inter-case-pause-s",
                    str(float(args.inter_case_pause_s)),
                    "--reset-pos-after-pause",
                    "--post-reset-ready-timeout-s",
                    str(float(args.post_reset_ready_timeout_s)),
                    "--max-case-attempts",
                    str(int(args.max_case_attempts)),
                    "--retry-all-trust-failures",
                    "--compact",
                ],
                cwd=PROJECT_ROOT,
                timeout=float(args.m1_timeout_s),
                check=False,
            )
            live_execution["return_code"] = int(completed.returncode)
        except Exception as exc:
            live_execution["error"] = str(exc)
        live_execution["source_artifact_fresh"] = bool(
            source_path.exists()
            and source_path.stat().st_mtime >= started_at_epoch_s - 1.0
        )

    if not source_path.exists():
        result = {
            "schema": "R2B4_M2_CHASSIS_MOTION_DYNAMICS_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "test": PROFILE_NAME,
            "phase": "M2",
            "status": "FAIL",
            "success": False,
            "speed_map_promotion_blocking": False,
            "failures": ["source_m1_artifact_missing"],
        }
    elif not bool(live_execution["source_artifact_fresh"]):
        result = {
            "schema": "R2B4_M2_CHASSIS_MOTION_DYNAMICS_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "test": PROFILE_NAME,
            "phase": "M2",
            "status": "FAIL",
            "success": False,
            "speed_map_promotion_blocking": False,
            "failures": ["source_m1_artifact_not_fresh"],
        }
    else:
        result = analyze_m1_result(_read_json(source_path))

    map_hash_after = _sha256_file(ACTIVE_MAP_PATH)
    pid_hash_after = _sha256_file(PID_CONFIG_PATH)
    mutation_guard = {
        "active_map_sha256_before": map_hash_before,
        "active_map_sha256_after": map_hash_after,
        "active_map_unchanged": map_hash_before == map_hash_after,
        "pid_config_sha256_before": pid_hash_before,
        "pid_config_sha256_after": pid_hash_after,
        "pid_config_unchanged": pid_hash_before == pid_hash_after,
    }
    if not (
        bool(mutation_guard["active_map_unchanged"])
        and bool(mutation_guard["pid_config_unchanged"])
    ):
        result.setdefault("failures", []).append(
            "forbidden_map_or_pid_mutation"
        )
        result["failures"] = list(dict.fromkeys(result["failures"]))
        result["success"] = False
        result["status"] = "FAIL"

    result.update(
        {
            "started_at_epoch_s": started_at_epoch_s,
            "completed_at_epoch_s": time.time(),
            "source_artifact": str(source_path),
            "artifact": str(result_path),
            "live_execution": live_execution,
            "mutation_guard": mutation_guard,
        }
    )
    m1._write_json_atomic(result_path, result)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Independent, non-speed-map-blocking M2 chassis dynamics "
            "validator."
        )
    )
    parser.add_argument("--analyze-only", action="store_true")
    parser.add_argument("--source-m1", type=Path, default=SOURCE_M1_PATH)
    parser.add_argument("--result-path", type=Path, default=RESULT_PATH)
    parser.add_argument("--inter-case-pause-s", type=float, default=10.0)
    parser.add_argument(
        "--post-reset-ready-timeout-s",
        type=float,
        default=90.0,
    )
    parser.add_argument("--max-case-attempts", type=int, default=3)
    parser.add_argument("--m1-timeout-s", type=float, default=900.0)
    parser.add_argument("--compact", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        result = run(args)
    except BootstrapGuardError as exc:
        result = {
            "schema": "R2B4_M2_CHASSIS_MOTION_DYNAMICS_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "test": PROFILE_NAME,
            "phase": "M2",
            "status": "FAIL",
            "success": False,
            "speed_map_promotion_blocking": False,
            "error": f"bootstrap_guard_failed:{exc}",
            "failures": ["bootstrap_guard_failed"],
        }
    except Exception as exc:
        result = {
            "schema": "R2B4_M2_CHASSIS_MOTION_DYNAMICS_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "test": PROFILE_NAME,
            "phase": "M2",
            "status": "FAIL",
            "success": False,
            "speed_map_promotion_blocking": False,
            "error": str(exc),
            "failures": ["validator_exception"],
        }

    if bool(getattr(args, "compact", False)):
        print(
            "M2_CHASSIS_MOTION_DYNAMICS "
            f"result={result.get('status', 'FAIL')} "
            f"promotion_blocking="
            f"{str(bool(result.get('speed_map_promotion_blocking', False))).lower()} "
            f"failures={','.join(result.get('failures') or []) or 'none'} "
            f"artifact={result.get('artifact', RESULT_PATH)}"
        )
    else:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if bool(result.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
