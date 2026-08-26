#!/usr/bin/env python3
"""Aggregate multiple Test Hub Room Cruise runs into the M5 physical proof."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    return float(out) if math.isfinite(out) else float(default)


def _pose(result: Dict[str, Any], key: str) -> Dict[str, float]:
    src = dict(result.get(key) or {})
    return {
        "x": _finite(src.get("x"), 0.0),
        "y": _finite(src.get("y"), 0.0),
        "theta_deg": _finite(src.get("theta_deg"), 0.0),
    }


def _heading_delta_deg(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _environment(result: Dict[str, Any]) -> Dict[str, Any]:
    return dict((dict(result.get("summary") or {})).get("m5_start_environment") or {})


def _environment_delta_m(left: Dict[str, Any], right: Dict[str, Any]) -> float:
    deltas: List[float] = []
    for key in (
        "front_m",
        "min_clearance_m",
        "left_clearance_m",
        "right_clearance_m",
        "rear_clearance_m",
    ):
        left_value = left.get(key)
        right_value = right.get(key)
        try:
            left_number = float(left_value)
            right_number = float(right_value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(left_number) and math.isfinite(right_number):
            deltas.append(abs(left_number - right_number))
    return max(deltas, default=0.0)


def _distinct_start_pairs(
    starts: List[Dict[str, float]],
    environments: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    pairs: List[Dict[str, Any]] = []
    for left_idx in range(len(starts)):
        for right_idx in range(left_idx + 1, len(starts)):
            left = starts[left_idx]
            right = starts[right_idx]
            distance = math.hypot(left["x"] - right["x"], left["y"] - right["y"])
            heading = _heading_delta_deg(left["theta_deg"], right["theta_deg"])
            environment_delta = _environment_delta_m(
                environments[left_idx],
                environments[right_idx],
            )
            pairs.append(
                {
                    "left_run": int(left_idx + 1),
                    "right_run": int(right_idx + 1),
                    "distance_m": round(float(distance), 4),
                    "heading_delta_deg": round(float(heading), 3),
                    "environment_clearance_delta_m": round(float(environment_delta), 4),
                    "distinct": bool(
                        distance >= 0.25
                        or heading >= 20.0
                        or environment_delta >= 0.25
                    ),
                }
            )
    return pairs


def validate_results(paths: List[Path], *, minimum_runs: int = 3) -> Dict[str, Any]:
    results = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    run_reports: List[Dict[str, Any]] = []
    starts = [_pose(result, "start_pose") for result in results]
    environments = [_environment(result) for result in results]
    for index, (path, result) in enumerate(zip(paths, results), start=1):
        summary = dict(result.get("summary") or {})
        checks = dict(result.get("checks") or {})
        sample_count = int(summary.get("sample_count", 0) or 0)
        duration_s = _finite(summary.get("duration_s"), 0.0)
        m5_samples = int(summary.get("m5_full_stack_samples", 0) or 0)
        route_samples = int(summary.get("m5_route_selector_samples", 0) or 0)
        path_samples = int(summary.get("m5_local_path_segment_samples", 0) or 0)
        replans = int(summary.get("m5_replan_count", 0) or 0)
        progress_m = _finite(summary.get("progress_m"), 0.0)
        gates = {
            "base_room_cruise_pass": bool(result.get("success", False)) and str(result.get("status", "")).upper() == "PASS",
            "m5_status_observed": bool(m5_samples >= max(3, int(sample_count * 0.20))),
            "existing_route_selector_used": bool(route_samples >= max(1, int(m5_samples * 0.20))),
            "waypoint_local_path_used": bool(path_samples >= max(1, int(m5_samples * 0.20))),
            "persistent_goal_not_tick_bounce": bool(
                int(summary.get("m5_unique_goal_count", 0) or 0) >= 1
                and _finite(summary.get("m5_goal_switch_ratio"), 1.0) <= 0.10
            ),
            "world_coverage_memory_used": bool(int(summary.get("m5_visited_cell_count", 0) or 0) >= 2),
            "purposeful_progress": bool(
                progress_m >= 0.35
                and (
                    int(summary.get("m5_completed_waypoints", 0) or 0) >= 1
                    or progress_m >= 0.60
                )
            ),
            "bounded_replanning": bool(replans <= max(3, int(duration_s / 3.0) + 1)),
            "no_uncontrolled_reverse": bool(
                int(summary.get("m5_uncontrolled_reverse_samples", 0) or 0) == 0
            ),
            "separate_fail_closed_safety": bool(
                int(summary.get("emergency_stop_events", 0) or 0) == 0
                and int(summary.get("localization_contradiction_samples", 0) or 0) == 0
                and bool(summary.get("room_cruise_idle_confirmed", False))
                and bool(checks.get("collision_margin_ok", False))
            ),
            "m3_executor_contract_retained": bool(
                int(summary.get("m3_track_execution_samples", 0) or 0) >= max(3, int(sample_count * 0.20))
                and int(summary.get("forbidden_path_samples", 0) or 0) == 0
            ),
        }
        run_reports.append(
            {
                "run": int(index),
                "path": str(path),
                "start_pose": starts[index - 1],
                "start_environment": environments[index - 1],
                "end_pose": _pose(result, "end_pose"),
                "progress_m": float(progress_m),
                "m5_unique_goal_count": int(summary.get("m5_unique_goal_count", 0) or 0),
                "m5_completed_waypoints": int(summary.get("m5_completed_waypoints", 0) or 0),
                "m5_replan_count": int(replans),
                "m5_visited_cell_count": int(summary.get("m5_visited_cell_count", 0) or 0),
                "gates": gates,
                "status": "PASS" if all(gates.values()) else "FAIL",
            }
        )

    start_pairs = _distinct_start_pairs(starts, environments)
    distinct_run_indices = {1} if starts else set()
    for pair in start_pairs:
        if bool(pair["distinct"]):
            distinct_run_indices.add(int(pair["left_run"]))
            distinct_run_indices.add(int(pair["right_run"]))
    aggregate_gates = {
        "minimum_three_runs": bool(len(results) >= int(minimum_runs)),
        "all_runs_pass": bool(run_reports and all(row["status"] == "PASS" for row in run_reports)),
        "different_start_conditions": bool(len(distinct_run_indices) >= int(minimum_runs)),
        "repeatable_full_stack_behavior": bool(
            len(run_reports) >= int(minimum_runs)
            and all(int(row["m5_visited_cell_count"]) >= 2 for row in run_reports)
            and all(float(row["progress_m"]) >= 0.35 for row in run_reports)
        ),
    }
    return {
        "schema": "M5_FULL_STACK_ROOM_CRUISE_MULTI_START_V1",
        "status": "PASS" if all(aggregate_gates.values()) else "FAIL",
        "run_count": int(len(results)),
        "distinct_start_run_count": int(len(distinct_run_indices)),
        "aggregate_gates": aggregate_gates,
        "start_pairs": start_pairs,
        "runs": run_reports,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", nargs="+", type=Path, help="Run-bound room_cruise_v2_live result.json files")
    parser.add_argument("--minimum-runs", type=int, default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = validate_results([path.resolve() for path in args.results], minimum_runs=args.minimum_runs)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
