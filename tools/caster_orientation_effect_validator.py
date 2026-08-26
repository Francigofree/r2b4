#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""M1.1 paired live validator for the passive front caster orientation effect."""

from __future__ import annotations

import argparse
import json
import math
import sys
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
from tools import live_motion_measurement_validator as base  # noqa: E402


AGENT_TESTS_DIR = test_artifacts_dir()
RESULT_PATH = AGENT_TESTS_DIR / "latest_M1_1_caster_orientation_live.json"
SAMPLES_PATH = AGENT_TESTS_DIR / "latest_M1_1_caster_orientation_live_samples.jsonl"
PHASE_STATE_PATH = AGENT_TESTS_DIR / "latest_M1_1_caster_phase_state.json"

CONTRACT_ID = "R2B4_M1_1_CASTER_ORIENTATION_V1"
TRANSIENT_ALLOWANCE_S = base.CASTER_TRANSIENT_ALLOWANCE_S
POST_TRANSIENT_WHEEL_MAE_MAX_MPS = base.WHEEL_SPEED_TRACKING_MAE_MAX_MPS
FULL_SETTLED_WHEEL_MAE_MAX_MPS = base.CASTER_FULL_SETTLED_WHEEL_MAE_MAX_MPS
MIN_TRANSIENT_FEEDBACK_WINDOWS = base.CASTER_MIN_TRANSIENT_FEEDBACK_WINDOWS
MIN_POST_TRANSIENT_FEEDBACK_WINDOWS = (
    base.CASTER_MIN_POST_TRANSIENT_FEEDBACK_WINDOWS
)
ALLOWED_REVERSED_FAILURES = frozenset(
    {"settled_wheel_speed_tracking_error_high"}
)


def _case(
    name: str,
    kind: str,
    v_mps: float,
    omega_rad_s: float,
    duration_s: float,
    *,
    pair: str,
    orientation: str,
    instruction: str,
    expected_linear_sign: int = 0,
    expected_yaw_sign: int = 0,
    command_type: str = "set_twist",
    target_angle_deg: Optional[float] = None,
    heading_speed_level: int = 0,
) -> base.MeasurementCase:
    return base.MeasurementCase(
        name=name,
        kind=kind,
        v_mps=v_mps,
        omega_rad_s=omega_rad_s,
        duration_s=duration_s,
        expected_linear_sign=expected_linear_sign,
        expected_yaw_sign=expected_yaw_sign,
        command_type=command_type,
        target_angle_deg=target_angle_deg,
        heading_speed_level=heading_speed_level,
        caster_pair=pair,
        caster_orientation=orientation,
        caster_transient_s=TRANSIENT_ALLOWANCE_S,
        operator_instruction_hu=instruction,
    )


def _pair_cases(
    *,
    pair: str,
    kind: str,
    v_mps: float,
    omega_rad_s: float,
    duration_s: float,
    travel_direction_hu: str,
    expected_linear_sign: int = 0,
    expected_yaw_sign: int = 0,
    command_type: str = "set_twist",
    target_angle_deg: Optional[float] = None,
    heading_speed_level: int = 0,
) -> Tuple[base.MeasurementCase, base.MeasurementCase]:
    common = dict(
        pair=pair,
        kind=kind,
        v_mps=v_mps,
        omega_rad_s=omega_rad_s,
        duration_s=duration_s,
        expected_linear_sign=expected_linear_sign,
        expected_yaw_sign=expected_yaw_sign,
        command_type=command_type,
        target_angle_deg=target_angle_deg,
        heading_speed_level=heading_speed_level,
    )
    return (
        _case(
            f"{pair}_aligned",
            orientation="aligned",
            instruction=(
                f"Állítsd a bolygókereket a várható helyi menetirányba "
                f"({travel_direction_hu})."
            ),
            **common,
        ),
        _case(
            f"{pair}_reversed",
            orientation="reversed_180",
            instruction=(
                f"Fordítsd a bolygókereket pontosan 180 fokkal a várható "
                f"helyi menetiránnyal szembe ({travel_direction_hu})."
            ),
            **common,
        ),
    )


_M0_MINI = base.M1_CASES[0]
_STOP_HOLD = base.M1_CASES[-1]
M1_1_PAIRS: Tuple[Tuple[base.MeasurementCase, base.MeasurementCase], ...] = (
    _pair_cases(
        pair="forward",
        kind="forward",
        v_mps=0.150,
        omega_rad_s=0.0,
        duration_s=3.6,
        travel_direction_hu="robot-előre",
        expected_linear_sign=1,
    ),
    _pair_cases(
        pair="backward",
        kind="backward",
        v_mps=-0.150,
        omega_rad_s=0.0,
        duration_s=3.24,
        travel_direction_hu="robot-hátra",
        expected_linear_sign=-1,
    ),
    _pair_cases(
        pair="arc_left",
        kind="arc_left",
        v_mps=0.225,
        omega_rad_s=0.20,
        duration_s=2.4,
        travel_direction_hu="előre-balra ív érintője",
        expected_linear_sign=1,
        expected_yaw_sign=1,
    ),
    _pair_cases(
        pair="arc_right",
        kind="arc_right",
        v_mps=0.225,
        omega_rad_s=-0.20,
        duration_s=2.4,
        travel_direction_hu="előre-jobbra ív érintője",
        expected_linear_sign=1,
        expected_yaw_sign=-1,
    ),
    _pair_cases(
        pair="rotate_left",
        kind="rotate_left",
        v_mps=0.0,
        omega_rad_s=0.0,
        duration_s=8.0,
        travel_direction_hu="a robot elején oldalirányban balra",
        expected_yaw_sign=1,
        command_type="rotate_to_heading",
        target_angle_deg=45.0,
        heading_speed_level=1,
    ),
    _pair_cases(
        pair="rotate_right",
        kind="rotate_right",
        v_mps=0.0,
        omega_rad_s=0.0,
        duration_s=8.0,
        travel_direction_hu="a robot elején oldalirányban jobbra",
        expected_yaw_sign=-1,
        command_type="rotate_to_heading",
        target_angle_deg=-45.0,
        heading_speed_level=1,
    ),
)
M1_1_CASES: Tuple[base.MeasurementCase, ...] = (
    _M0_MINI,
    *(case for pair in M1_1_PAIRS for case in pair),
    _STOP_HOLD,
)


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _phase_metric(
    result: Dict[str, Any],
    phase_name: str,
    key: str,
) -> Optional[float]:
    metrics = dict(result.get("metrics") or {})
    phase = dict((metrics.get("phase_tracking") or {}).get(phase_name) or {})
    return _finite(phase.get(key))


def _command_metric(result: Dict[str, Any], section: str, key: str) -> Optional[float]:
    fidelity = dict((result.get("metrics") or {}).get("command_fidelity") or {})
    return _finite((fidelity.get(section) or {}).get(key))


def _metric(result: Dict[str, Any], *path: str) -> Optional[float]:
    value: Any = dict(result.get("metrics") or {})
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return _finite(value)


def _result_map(raw: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {
        str(item.get("case", "") or ""): dict(item)
        for item in list(raw.get("cases") or [])
        if str(item.get("case", "") or "")
    }


def _propagate_warnings(raw: Dict[str, Any], rows: Dict[str, Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """Preserve non-blocking M0/M1 warnings in the M1.1 aggregate artifact."""
    raw_warnings = [
        dict(item)
        for item in list(raw.get("warnings") or [])
        if isinstance(item, dict)
    ]
    raw_summary = dict(raw.get("warning_summary") or {})
    if raw_warnings:
        return raw_warnings, raw_summary

    warnings: List[Dict[str, Any]] = []
    warning_cases = set()
    warning_count = 0
    for case_name, row in rows.items():
        for item in list(row.get("warnings") or []):
            if not isinstance(item, dict):
                continue
            warning = dict(item)
            warning.setdefault("case", str(case_name))
            warnings.append(warning)
            warning_cases.add(str(case_name))
            count = _finite(warning.get("count"))
            warning_count += max(1, int(count or 0))
    return warnings, {
        "schema": "R2B4_TIMING_GAP_WARNING_V1",
        "severity": "WARNING" if warnings else "NONE",
        "case_count": int(len(warning_cases)),
        "motion_timing_gap_count_delta": int(warning_count),
        "future_trend_monitoring_required": bool(warnings),
        "pass_policy": (
            "non_blocking_when_safety_sensor_truth_and_motion_quality_gates_pass"
        ),
    }


def _remove_case_failure(
    failures: Iterable[str],
    *,
    case_name: str,
    failure: str,
) -> List[str]:
    target = f"{case_name}:{failure}"
    return [str(item) for item in failures if str(item) != target]


def analyze_result(
    raw: Dict[str, Any],
    *,
    operator_protocol_armed: bool,
    operator_id: str,
) -> Dict[str, Any]:
    expected_names = [case.name for case in M1_1_CASES]
    rows = _result_map(raw)
    warnings, warning_summary = _propagate_warnings(raw, rows)
    final_failures = [str(item) for item in list(raw.get("failures") or [])]
    analysis_failures: List[str] = []
    pair_results: List[Dict[str, Any]] = []
    allowance_used_count = 0
    detected_pair_count = 0

    if list(raw.get("cases_requested") or []) != expected_names:
        analysis_failures.append("m1_1_case_order_contract_mismatch")
    if not bool(operator_protocol_armed):
        analysis_failures.append("operator_caster_protocol_not_armed")
    if abs(float((raw.get("baseline") or {}).get("manual_reposition_pause_s", 0.0)) - 10.0) > 1e-9:
        analysis_failures.append("m1_1_pause_not_exactly_10s")
    if not bool(raw.get("reset_pos_after_pause", False)):
        analysis_failures.append("m1_1_pause_reanchor_missing")

    for aligned_case, reversed_case in M1_1_PAIRS:
        aligned = rows.get(aligned_case.name)
        reversed_row = rows.get(reversed_case.name)
        pair_failures: List[str] = []
        if aligned is None:
            pair_failures.append("aligned_case_missing")
        if reversed_row is None:
            pair_failures.append("reversed_case_missing")
        if aligned is None or reversed_row is None:
            analysis_failures.extend(
                f"{aligned_case.caster_pair}:{failure}" for failure in pair_failures
            )
            pair_results.append(
                {
                    "pair": aligned_case.caster_pair,
                    "status": "FAIL",
                    "failures": pair_failures,
                }
            )
            continue

        aligned_failures = set(str(item) for item in list(aligned.get("failures") or []))
        reversed_failures = set(
            str(item) for item in list(reversed_row.get("failures") or [])
        )
        if aligned_failures:
            pair_failures.extend(
                f"aligned:{failure}" for failure in sorted(aligned_failures)
            )

        aligned_transient_mae = _phase_metric(
            aligned, "caster_transient", "wheel_speed_tracking_mae_mps"
        )
        reversed_transient_mae = _phase_metric(
            reversed_row, "caster_transient", "wheel_speed_tracking_mae_mps"
        )
        aligned_post_mae = _phase_metric(
            aligned, "post_caster_transient", "wheel_speed_tracking_mae_mps"
        )
        reversed_post_mae = _phase_metric(
            reversed_row, "post_caster_transient", "wheel_speed_tracking_mae_mps"
        )
        aligned_transient_windows = int(
            _phase_metric(
                aligned, "caster_transient", "independent_feedback_windows"
            )
            or 0
        )
        reversed_transient_windows = int(
            _phase_metric(
                reversed_row, "caster_transient", "independent_feedback_windows"
            )
            or 0
        )
        aligned_post_windows = int(
            _phase_metric(
                aligned, "post_caster_transient", "independent_feedback_windows"
            )
            or 0
        )
        reversed_post_windows = int(
            _phase_metric(
                reversed_row, "post_caster_transient", "independent_feedback_windows"
            )
            or 0
        )
        linear_case = int(aligned_case.expected_linear_sign) != 0
        required_windows = [
            ("aligned_transient", aligned_transient_windows, MIN_TRANSIENT_FEEDBACK_WINDOWS),
            ("reversed_transient", reversed_transient_windows, MIN_TRANSIENT_FEEDBACK_WINDOWS),
        ]
        if linear_case:
            required_windows.extend(
                [
                    (
                        "aligned_post",
                        aligned_post_windows,
                        MIN_POST_TRANSIENT_FEEDBACK_WINDOWS,
                    ),
                    (
                        "reversed_post",
                        reversed_post_windows,
                        MIN_POST_TRANSIENT_FEEDBACK_WINDOWS,
                    ),
                ]
            )
        for label, count, minimum in required_windows:
            if int(count) < int(minimum):
                pair_failures.append(f"{label}_feedback_windows_low")

        reversed_full_settled_mae = _command_metric(
            reversed_row,
            "errors",
            "settled_wheel_speed_tracking_mae_mps",
        )
        settling_time_s = _command_metric(
            reversed_row,
            "transient",
            "settling_time_s",
        )
        allowed_failures = reversed_failures & ALLOWED_REVERSED_FAILURES
        non_allowed_failures = reversed_failures - ALLOWED_REVERSED_FAILURES
        pair_failures.extend(
            f"reversed:{failure}" for failure in sorted(non_allowed_failures)
        )

        transient_waiver_ok = False
        if linear_case:
            if reversed_post_mae is None:
                pair_failures.append("reversed_post_transient_mae_missing")
            elif reversed_post_mae > POST_TRANSIENT_WHEEL_MAE_MAX_MPS:
                pair_failures.append("reversed_post_transient_mae_high")
            if (
                reversed_full_settled_mae is None
                or reversed_full_settled_mae > FULL_SETTLED_WHEEL_MAE_MAX_MPS
            ):
                pair_failures.append("reversed_full_settled_mae_unbounded")
            if settling_time_s is None or settling_time_s > TRANSIENT_ALLOWANCE_S:
                pair_failures.append("caster_reorientation_not_settled_by_1s")
            transient_waiver_ok = bool(
                allowed_failures
                and not non_allowed_failures
                and reversed_post_mae is not None
                and reversed_post_mae <= POST_TRANSIENT_WHEEL_MAE_MAX_MPS
                and reversed_full_settled_mae is not None
                and reversed_full_settled_mae <= FULL_SETTLED_WHEEL_MAE_MAX_MPS
                and settling_time_s is not None
                and settling_time_s <= TRANSIENT_ALLOWANCE_S
                and reversed_post_windows >= MIN_POST_TRANSIENT_FEEDBACK_WINDOWS
                and reversed_transient_windows >= MIN_TRANSIENT_FEEDBACK_WINDOWS
            )
            if transient_waiver_ok:
                allowance_used_count += 1
                for failure in allowed_failures:
                    final_failures = _remove_case_failure(
                        final_failures,
                        case_name=reversed_case.name,
                        failure=failure,
                    )
            elif allowed_failures:
                pair_failures.extend(
                    f"reversed_unaccepted:{failure}"
                    for failure in sorted(allowed_failures)
                )
        elif allowed_failures:
            pair_failures.extend(
                f"pivot_failure_not_waivable:{failure}"
                for failure in sorted(allowed_failures)
            )

        transient_delta = (
            None
            if aligned_transient_mae is None or reversed_transient_mae is None
            else float(reversed_transient_mae) - float(aligned_transient_mae)
        )
        post_delta = (
            None
            if aligned_post_mae is None or reversed_post_mae is None
            else float(reversed_post_mae) - float(aligned_post_mae)
        )
        effect_detected = bool(
            transient_delta is not None and abs(float(transient_delta)) >= 0.003
        )
        if effect_detected:
            detected_pair_count += 1

        motion_impact = {
            "linear_speed_ratio_vs_executed": {
                "aligned": _metric(
                    aligned,
                    "command_fidelity",
                    "errors",
                    "linear_speed_ratio_vs_executed",
                ),
                "reversed": _metric(
                    reversed_row,
                    "command_fidelity",
                    "errors",
                    "linear_speed_ratio_vs_executed",
                ),
            },
            "encoder_distance_m": {
                "aligned": _metric(aligned, "encoder", "average_delta_m"),
                "reversed": _metric(
                    reversed_row, "encoder", "average_delta_m"
                ),
            },
            "lidar_chord_m": {
                "aligned": _metric(aligned, "lidar", "pose_chord_m"),
                "reversed": _metric(reversed_row, "lidar", "pose_chord_m"),
            },
            "ekf_forward_delta_m": {
                "aligned": _metric(aligned, "ekf", "forward_delta_m"),
                "reversed": _metric(
                    reversed_row, "ekf", "forward_delta_m"
                ),
            },
            "ekf_yaw_delta_deg": {
                "aligned": _metric(aligned, "ekf", "yaw_delta_deg"),
                "reversed": _metric(reversed_row, "ekf", "yaw_delta_deg"),
            },
            "imu_yaw_delta_deg": {
                "aligned": _metric(aligned, "imu", "yaw_delta_deg"),
                "reversed": _metric(reversed_row, "imu", "yaw_delta_deg"),
            },
            "pwm_difference_total_variation": {
                "aligned": _metric(
                    aligned,
                    "correction_dynamics",
                    "pwm_difference_total_variation",
                ),
                "reversed": _metric(
                    reversed_row,
                    "correction_dynamics",
                    "pwm_difference_total_variation",
                ),
            },
        }
        pair_status = "PASS" if not pair_failures else "FAIL"
        analysis_failures.extend(
            f"{aligned_case.caster_pair}:{failure}" for failure in pair_failures
        )
        pair_results.append(
            {
                "pair": str(aligned_case.caster_pair),
                "status": pair_status,
                "aligned_case": str(aligned_case.name),
                "reversed_case": str(reversed_case.name),
                "effect_detected": bool(effect_detected),
                "transient_allowance_used": bool(transient_waiver_ok),
                "raw_m1_failures": {
                    "aligned": sorted(aligned_failures),
                    "reversed": sorted(reversed_failures),
                },
                "transient": {
                    "aligned_mae_mps": aligned_transient_mae,
                    "reversed_mae_mps": reversed_transient_mae,
                    "reversed_minus_aligned_mae_mps": transient_delta,
                    "aligned_feedback_windows": aligned_transient_windows,
                    "reversed_feedback_windows": reversed_transient_windows,
                },
                "post_transient": {
                    "aligned_mae_mps": aligned_post_mae,
                    "reversed_mae_mps": reversed_post_mae,
                    "reversed_minus_aligned_mae_mps": post_delta,
                    "aligned_feedback_windows": aligned_post_windows,
                    "reversed_feedback_windows": reversed_post_windows,
                },
                "reversed_full_settled_mae_mps": reversed_full_settled_mae,
                "reversed_settling_time_s": settling_time_s,
                "motion_impact": motion_impact,
                "failures": list(pair_failures),
            }
        )

    final_failures.extend(analysis_failures)
    final_failures = list(dict.fromkeys(final_failures))
    success = bool(
        not final_failures
        and len(rows) == len(M1_1_CASES)
        and bool((raw.get("m0_mini") or {}).get("ok", False))
    )
    status = "PASS" if success else "FAIL"
    result = dict(raw)
    result.update(
        {
            "schema": "R2B4_M1_1_CASTER_ORIENTATION_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "phase": "M1_1",
            "test": "M1_1_caster_orientation_live",
            "status": status,
            "success": bool(success),
            "proof_verdict": (
                "M1_1_CASTER_EFFECT_BOUNDED"
                if success
                else "M1_1_CASTER_EFFECT_NOT_PROVEN"
            ),
            "operator_protocol": {
                "armed": bool(operator_protocol_armed),
                "operator_id": str(operator_id),
                "orientation_sensor_available": False,
                "evidence_basis": "run_bound_operator_controlled_pair_order",
                "pair_order": [
                    {
                        "pair": pair[0].caster_pair,
                        "first": "aligned",
                        "second": "reversed_180",
                    }
                    for pair in M1_1_PAIRS
                ],
            },
            "caster_orientation_analysis": {
                "ok": not analysis_failures,
                "transient_allowance_s": TRANSIENT_ALLOWANCE_S,
                "post_transient_wheel_mae_max_mps": (
                    POST_TRANSIENT_WHEEL_MAE_MAX_MPS
                ),
                "full_settled_wheel_mae_max_mps": (
                    FULL_SETTLED_WHEEL_MAE_MAX_MPS
                ),
                "waivable_failure_names": sorted(ALLOWED_REVERSED_FAILURES),
                "allowance_used_count": int(allowance_used_count),
                "effect_detected_pair_count": int(detected_pair_count),
                "pair_count": int(len(M1_1_PAIRS)),
                "pairs": pair_results,
                "failures": list(dict.fromkeys(analysis_failures)),
                "permissiveness_guard": {
                    "only_reversed_orientation": True,
                    "only_first_s": TRANSIENT_ALLOWANCE_S,
                    "safety_timing_sensor_stop_and_endpoint_gates_waivable": False,
                    "post_transient_original_m1_wheel_mae_gate_retained": True,
                    "full_settled_mae_has_absolute_cap": True,
                },
            },
            "raw_baseline_failures": list(raw.get("failures") or []),
            "failures": final_failures,
            "warnings": warnings,
            "warning_summary": warning_summary,
            "artifact": str(RESULT_PATH),
            "samples_artifact": str(SAMPLES_PATH),
        }
    )
    result["baseline"] = {
        **dict(result.get("baseline") or {}),
        "ok": bool(success),
        "m1_1_caster_transient_policy": CONTRACT_ID,
        "failures": list(final_failures),
        "warnings": warnings,
    }
    return result


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    raw = base.run(
        args,
        run_spec={
            "phase": "M1_1",
            "test_name": "M1_1_caster_orientation_live",
            "result_path": RESULT_PATH,
            "sample_path": SAMPLES_PATH,
            "cases": M1_1_CASES,
            "embedded_m0_mini_case_names": [case.name for case in M1_1_CASES],
            "m0_mini_source_profile": "M1_1_caster_orientation_live",
            "phase_state_path": PHASE_STATE_PATH,
        },
    )
    result = analyze_result(
        raw,
        operator_protocol_armed=bool(args.operator_protocol_armed),
        operator_id=str(args.operator_id),
    )
    base._write_json_atomic(RESULT_PATH, result)
    base._write_json_atomic(
        PHASE_STATE_PATH,
        {
            "schema": "R2B4_LIVE_VALIDATOR_PHASE_STATE_V1",
            "test": "M1_1_caster_orientation_live",
            "phase": "M1_1",
            "status": "complete",
            "success": bool(result.get("success", False)),
            "failures": list(result.get("failures") or []),
            "warnings": list(result.get("warnings") or []),
            "artifact": str(RESULT_PATH),
        },
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = base.build_parser()
    parser.description = (
        "M1.1 paired live front-caster orientation effect validator."
    )
    parser.set_defaults(
        mode="baseline",
        repeat_count=1,
        inter_case_pause_s=10.0,
        reset_pos_after_pause=True,
        embedded_m0_mini=True,
        post_reset_ready_timeout_s=90.0,
    )
    parser.add_argument("--operator-protocol-armed", action="store_true")
    parser.add_argument("--operator-id", default="interactive_operator")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        result = run(args)
    except BootstrapGuardError as exc:
        result = {
            "schema": "R2B4_M1_1_CASTER_ORIENTATION_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "phase": "M1_1",
            "test": "M1_1_caster_orientation_live",
            "status": "FAIL",
            "success": False,
            "failures": [f"bootstrap_guard_failed:{exc}"],
            "artifact": str(RESULT_PATH),
        }
        base._write_json_atomic(RESULT_PATH, result)
    except Exception as exc:
        base._safe_stop_best_effort(token=str(getattr(args, "token", base.DEFAULT_TOKEN)))
        result = {
            "schema": "R2B4_M1_1_CASTER_ORIENTATION_RESULT_V1",
            "contract_id": CONTRACT_ID,
            "phase": "M1_1",
            "test": "M1_1_caster_orientation_live",
            "status": "FAIL",
            "success": False,
            "error": str(exc),
            "failures": ["validator_exception"],
            "artifact": str(RESULT_PATH),
        }
        base._write_json_atomic(RESULT_PATH, result)

    if bool(getattr(args, "compact", False)):
        print(
            "CASTER_ORIENTATION_VALIDATOR "
            f"result={result.get('status', 'FAIL')} "
            f"pairs={len((result.get('caster_orientation_analysis') or {}).get('pairs') or [])}/"
            f"{len(M1_1_PAIRS)} "
            f"allowance_used={(result.get('caster_orientation_analysis') or {}).get('allowance_used_count', 0)} "
            f"failures={','.join(result.get('failures') or []) or 'none'} "
            f"artifact={result.get('artifact', '')}"
        )
    else:
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if bool(result.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
