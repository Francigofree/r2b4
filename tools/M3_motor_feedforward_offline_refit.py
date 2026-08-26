#!/usr/bin/env python3
"""Offline four-direction feed-forward refit from completed live measurements.

This tool never sends a runtime command and never mutates the active speed map.
It fits monotonic PWM->speed response models from the saved acquisition and
candidate-validation rows, inverts the combined model, and publishes an
offline-only candidate plus explicit evidence gates.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded
from middleware.ffp import lookup_wheel_feedforward
from tools.live_motor_deadzone_calibrator import (
    DEFAULT_SPEEDS,
    _build_candidate,
    _build_curve,
    _is_stable_side,
    _model_score,
)

ACTIVE_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
SOURCE_RESULT_PATH = (
    latest_artifact_path("latest_motor_feedforward_calibration.json")
)
SOURCE_SAMPLES_PATH = (
    latest_artifact_path("latest_motor_feedforward_calibration_samples.jsonl")
)
AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_RESULT_PATH = (
    AGENT_TESTS_DIR / "latest_motor_feedforward_offline_refit.json"
)
LATEST_CANDIDATE_PATH = (
    AGENT_TESTS_DIR / "candidate_wheel_speed_map_offline_refit.json"
)

EXPECTED_STAGE_COUNTS = {"before": 36, "after": 24}
MODEL_NAMES = ("acquisition", "validation", "combined")
MAP_NAMES = ("active", "rejected_live_candidate", "offline_refit")
PIVOT_TRACK_SPEED_MPS = 0.150
REPEATABLE_SPEED_TOLERANCE_MPS = 0.002


def _read_json(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"json_object_required:{path}")
    return payload


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise RuntimeError(f"jsonl_object_required:{path}:{line_number}")
        rows.append(payload)
    return rows


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    os.replace(tmp, path)


def _finite(value: Any, default: float = math.inf) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    return number if math.isfinite(number) else float(default)


def _curve_set(rows: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    materialized = list(rows)
    curves: Dict[str, Dict[str, Any]] = {}
    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            key = f"{side}_{direction}"
            curves[key] = _build_curve(materialized, direction, side)
    return curves


def _curve_summary(curves: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for key, curve in sorted(curves.items()):
        stable_points = list(curve.get("stable_points") or [])
        out[key] = {
            "stable_point_count": len(stable_points),
            "min_stable_pwm": _finite(curve.get("min_stable_pwm"), 0.0),
            "min_stable_speed_mps": _finite(curve.get("min_stable_speed_mps"), 0.0),
            "max_stable_pwm": _finite(curve.get("max_stable_pwm"), 0.0),
            "max_stable_speed_mps": _finite(curve.get("max_stable_speed_mps"), 0.0),
            "stable_points": [
                {
                    "pwm": _finite(point.get("pwm"), 0.0),
                    "median_speed_mps": _finite(point.get("median_speed_mps"), 0.0),
                    "isotonic_speed_mps": _finite(point.get("isotonic_speed_mps"), 0.0),
                    "stable_repeats": int(point.get("stable_repeats", 0) or 0),
                }
                for point in stable_points
            ],
        }
    return out


def _integrity_counts(rows: Iterable[Dict[str, Any]]) -> Dict[str, int]:
    materialized = list(rows)
    return {
        "row_count": len(materialized),
        "fault_row_count": sum(bool(row.get("faults")) for row in materialized),
        "blocking_encoder_row_count": sum(
            bool(row.get("encoder_blocking_anomaly_seen", False)) for row in materialized
        ),
        "direct_executor_missing_count": sum(
            not bool(row.get("direct_executor_observed", False)) for row in materialized
        ),
        "pid_integrity_failure_count": sum(
            (not bool(row.get("pi_disabled_observed", False)))
            or bool(row.get("pi_violation_seen", False))
            for row in materialized
        ),
        "unstable_side_count": sum(
            not _is_stable_side(row, side)
            for row in materialized
            for side in ("left", "right")
        ),
    }


def _score_matrix(
    maps: Dict[str, Dict[str, Any]],
    models: Dict[str, Dict[str, Dict[str, Any]]],
) -> Dict[str, Dict[str, Any]]:
    return {
        model_name: {
            map_name: _model_score(speed_map, curves)
            for map_name, speed_map in maps.items()
        }
        for model_name, curves in models.items()
    }


def _pivot_operating_points(
    maps: Dict[str, Dict[str, Any]],
    combined_curves: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    cases = {
        "pivot_left": {"left": -PIVOT_TRACK_SPEED_MPS, "right": PIVOT_TRACK_SPEED_MPS},
        "pivot_right": {"left": PIVOT_TRACK_SPEED_MPS, "right": -PIVOT_TRACK_SPEED_MPS},
    }
    diagnostics: Dict[str, Any] = {}
    for case_name, targets in cases.items():
        wheels: Dict[str, Any] = {}
        for side, target in targets.items():
            direction = "forward" if target >= 0.0 else "reverse"
            curve_key = f"{side}_{direction}"
            minimum = _finite(combined_curves[curve_key].get("min_stable_speed_mps"), 0.0)
            margin = abs(float(target)) - minimum
            pwm_by_map = {
                map_name: abs(
                    lookup_wheel_feedforward(
                        speed_map,
                        side=side,
                        target_mps=target,
                        require_active=False,
                    )[0]
                )
                for map_name, speed_map in maps.items()
            }
            wheels[side] = {
                "curve": curve_key,
                "target_mps": float(target),
                "minimum_stable_speed_mps": minimum,
                "repeatable_speed_margin_mps": margin,
                "repeatable_speed_tolerance_mps": REPEATABLE_SPEED_TOLERANCE_MPS,
                "within_repeatable_speed_range": bool(
                    margin >= -REPEATABLE_SPEED_TOLERANCE_MPS
                ),
                "feedforward_pwm_abs_by_map": pwm_by_map,
            }
        diagnostics[case_name] = {
            "track_speed_mps": PIVOT_TRACK_SPEED_MPS,
            "repeatable_speed_range_supported": all(
                bool(wheel["within_repeatable_speed_range"])
                for wheel in wheels.values()
            ),
            "wheels": wheels,
        }
    return diagnostics


def _strictly_better(candidate: Dict[str, Any], baseline: Dict[str, Any]) -> bool:
    return bool(
        _finite(candidate.get("median_relative_error")) + 0.01
        < _finite(baseline.get("median_relative_error"))
        and _finite(candidate.get("mean_relative_error"))
        < _finite(baseline.get("mean_relative_error"))
        and _finite(candidate.get("max_group_median_relative_error"))
        <= _finite(baseline.get("max_group_median_relative_error")) + 0.05
    )


def _apply_unreachable_active_fallback(
    candidate: Dict[str, Any],
    active_map: Dict[str, Any],
    unreachable_targets: Iterable[str],
) -> None:
    by_curve: Dict[str, List[float]] = {}
    for raw in unreachable_targets:
        try:
            curve_key, speed_raw = str(raw).rsplit(":", 1)
            by_curve.setdefault(curve_key, []).append(float(speed_raw))
        except (TypeError, ValueError):
            continue

    candidate_curves = dict(candidate.get("curves") or {})
    active_curves = dict(active_map.get("curves") or {})
    for curve_key, speeds in by_curve.items():
        curve = dict(candidate_curves.get(curve_key) or {})
        active_curve = dict(active_curves.get(curve_key) or {})
        active_points = {
            round(float(point.get("speed_mps", 0.0)), 4): float(point.get("pwm", 0.0))
            for point in list(active_curve.get("points") or [])
        }
        fallback_speeds = {round(float(speed), 4) for speed in speeds}
        points = []
        for point in list(curve.get("points") or []):
            updated = dict(point)
            speed_key = round(float(updated.get("speed_mps", 0.0)), 4)
            if speed_key in fallback_speeds and speed_key in active_points:
                updated["pwm"] = round(float(active_points[speed_key]), 4)
                updated["offline_fallback"] = "active_map_unreachable_target"
            points.append(updated)
        curve["points"] = points
        curve["offline_unreachable_fallback_targets"] = sorted(fallback_speeds)
        if round(0.05, 4) in fallback_speeds:
            curve["dead_zone_pwm"] = float(
                active_curve.get("dead_zone_pwm", active_points.get(round(0.05, 4), 0.0))
            )
            curve["startup_pwm"] = float(
                active_curve.get("startup_pwm", curve["dead_zone_pwm"])
            )
        candidate_curves[curve_key] = curve
    candidate["curves"] = candidate_curves


def recompute(
    *,
    active_map: Dict[str, Any],
    source_result: Dict[str, Any],
    source_rows: List[Dict[str, Any]],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    stage_rows = {
        stage: [row for row in source_rows if str(row.get("stage", "")) == stage]
        for stage in EXPECTED_STAGE_COUNTS
    }
    stage_counts = {stage: len(rows) for stage, rows in stage_rows.items()}
    source_complete = bool(
        all(stage_counts[stage] == expected for stage, expected in EXPECTED_STAGE_COUNTS.items())
        and bool(source_result.get("success", False))
        and int(source_result.get("phase_count", 0) or 0) == sum(EXPECTED_STAGE_COUNTS.values())
    )
    if not source_complete:
        raise RuntimeError(f"feedforward_source_incomplete:{stage_counts}")

    acquisition_rows = stage_rows["before"]
    validation_rows = stage_rows["after"]
    combined_rows = list(acquisition_rows) + list(validation_rows)
    models = {
        "acquisition": _curve_set(acquisition_rows),
        "validation": _curve_set(validation_rows),
        "combined": _curve_set(combined_rows),
    }

    candidate, unreachable_targets = _build_candidate(active_map, models["combined"])
    _apply_unreachable_active_fallback(candidate, active_map, unreachable_targets)
    candidate["calibration_state"] = "OFFLINE_ROBUST_ISOTONIC_REFIT_CANDIDATE_2026_07_18"
    candidate["calibration_evidence"] = (
        "saved 3x acquisition and 2x rejected-candidate direct-PWM validation; "
        "anomalous rows excluded by stable-side gates"
    )
    candidate["offline_only"] = True
    candidate["activation_allowed"] = False
    candidate["requires_live_validation"] = True

    rejected_candidate = dict(source_result.get("candidate_map") or {})
    if not rejected_candidate.get("curves"):
        raise RuntimeError("rejected_live_candidate_missing")
    maps = {
        "active": active_map,
        "rejected_live_candidate": rejected_candidate,
        "offline_refit": candidate,
    }
    scores = _score_matrix(maps, models)
    pivot_operating_points = _pivot_operating_points(maps, models["combined"])
    integrity = {
        "acquisition": _integrity_counts(acquisition_rows),
        "validation": _integrity_counts(validation_rows),
        "combined": _integrity_counts(combined_rows),
    }
    operating_min_mps = min(float(speed) for speed in DEFAULT_SPEEDS)
    operating_rows = {
        "acquisition": [
            row
            for row in acquisition_rows
            if abs(_finite(row.get("target_speed_mps"), 0.0)) >= operating_min_mps - 1e-9
        ],
        "validation": [
            row
            for row in validation_rows
            if abs(_finite(row.get("target_speed_mps"), 0.0)) >= operating_min_mps - 1e-9
        ],
    }
    operating_rows["combined"] = list(operating_rows["acquisition"]) + list(
        operating_rows["validation"]
    )
    operating_integrity = {
        name: _integrity_counts(rows) for name, rows in operating_rows.items()
    }
    stable_curve_coverage = bool(
        all(
            len(curve.get("stable_points") or []) >= 3
            for model in models.values()
            for curve in model.values()
        )
    )
    executor_integrity = bool(
        integrity["combined"]["fault_row_count"] == 0
        and integrity["combined"]["direct_executor_missing_count"] == 0
        and integrity["combined"]["pid_integrity_failure_count"] == 0
    )
    anomaly_free_source = (
        operating_integrity["combined"]["blocking_encoder_row_count"] == 0
    )
    all_targets_reachable = not unreachable_targets
    improvement_by_model = {
        name: _strictly_better(scores[name]["offline_refit"], scores[name]["active"])
        for name in MODEL_NAMES
    }
    robust_model_improvement = all(improvement_by_model.values())
    pivot_operating_range_supported = all(
        bool(case["repeatable_speed_range_supported"])
        for case in pivot_operating_points.values()
    )
    offline_model_supported = bool(
        executor_integrity
        and stable_curve_coverage
        and all_targets_reachable
        and robust_model_improvement
        and pivot_operating_range_supported
    )
    candidate_qualified = bool(offline_model_supported and anomaly_free_source)

    gates = {
        "source_complete": "PASS",
        "executor_pid_integrity": "PASS" if executor_integrity else "FAIL",
        "stable_curve_coverage": "PASS" if stable_curve_coverage else "FAIL",
        "all_targets_reachable": "PASS" if all_targets_reachable else "FAIL",
        "robust_model_improvement": "PASS" if robust_model_improvement else "FAIL",
        "pivot_operating_range": (
            "PASS" if pivot_operating_range_supported else "FAIL"
        ),
        "anomaly_free_source": "PASS" if anomaly_free_source else "FAIL",
        "live_candidate_validation": "INCONCLUSIVE",
    }
    failed_gates = [name for name, status in gates.items() if status == "FAIL"]
    inconclusive_gates = [name for name, status in gates.items() if status == "INCONCLUSIVE"]
    result = {
        "schema": "M3_MOTOR_FEEDFORWARD_OFFLINE_REFIT_V1",
        "test_name": "M3_motor_feedforward_offline_refit",
        "status": "PASS",
        "success": True,
        "offline_only": True,
        "robot_motion_performed": False,
        "active_map_mutated": False,
        "candidate_activation_allowed": False,
        "candidate_qualified": candidate_qualified,
        "offline_model_supported": offline_model_supported,
        "calibration_outcome": (
            "OFFLINE_CANDIDATE_PROVISIONAL"
            if candidate_qualified
            else "OFFLINE_CANDIDATE_REJECTED"
        ),
        "source_phase_counts": stage_counts,
        "integrity": integrity,
        "operating_range_min_mps": operating_min_mps,
        "operating_range_integrity": operating_integrity,
        "unreachable_targets": sorted(unreachable_targets),
        "improvement_by_model": improvement_by_model,
        "scores": scores,
        "pivot_operating_points": pivot_operating_points,
        "curve_models": {
            name: _curve_summary(curves) for name, curves in models.items()
        },
        "gates": gates,
        "failed_gates": failed_gates,
        "inconclusive_gates": inconclusive_gates,
        "candidate_map": candidate,
        "active_speed_map": active_map,
        "rejected_live_candidate": rejected_candidate,
        "notes": [
            "Offline model fit cannot replace a live candidate validation.",
            "The active speed map remains the runtime SSOT.",
            "In-range blocking encoder rows are excluded from curve fitting and remain a qualification failure.",
            "Qualification anomaly counts cover only the configured speed-map operating range; historical lower-speed rows remain in the full-source audit.",
        ],
    }
    return result, candidate


def run(
    *,
    active_map_path: Path = ACTIVE_MAP_PATH,
    source_result_path: Path = SOURCE_RESULT_PATH,
    source_samples_path: Path = SOURCE_SAMPLES_PATH,
    result_path: Path = LATEST_RESULT_PATH,
    candidate_path: Path = LATEST_CANDIDATE_PATH,
) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    result, candidate = recompute(
        active_map=_read_json(active_map_path),
        source_result=_read_json(source_result_path),
        source_rows=_read_jsonl(source_samples_path),
    )
    result["artifacts"] = {
        "result": str(result_path.relative_to(PROJECT_ROOT)),
        "candidate": str(candidate_path.relative_to(PROJECT_ROOT)),
        "source_result": str(source_result_path.relative_to(PROJECT_ROOT)),
        "source_samples": str(source_samples_path.relative_to(PROJECT_ROOT)),
        "active_map": str(active_map_path.relative_to(PROJECT_ROOT)),
    }
    _write_json_atomic(candidate_path, candidate)
    _write_json_atomic(result_path, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    try:
        result = run()
    except Exception as exc:
        result = {
            "schema": "M3_MOTOR_FEEDFORWARD_OFFLINE_REFIT_V1",
            "test_name": "M3_motor_feedforward_offline_refit",
            "status": "FAIL",
            "success": False,
            "offline_only": True,
            "robot_motion_performed": False,
            "error": str(exc),
        }
    if args.compact:
        print(
            json.dumps(
                {
                    "status": result.get("status"),
                    "success": result.get("success"),
                    "calibration_outcome": result.get("calibration_outcome"),
                    "candidate_qualified": result.get("candidate_qualified", False),
                    "failed_gates": result.get("failed_gates", []),
                    "inconclusive_gates": result.get("inconclusive_gates", []),
                    "artifacts": result.get("artifacts", {}),
                    "error": result.get("error", ""),
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
            flush=True,
        )
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True), flush=True)
    return 0 if result.get("success", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
