#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path

from tools.m5_full_stack_room_cruise_live import validate_results


def _result(start_x: float, start_heading: float, front_m: float = 1.0) -> dict:
    sample_count = 400
    summary = {
        "sample_count": sample_count,
        "duration_s": 45.0,
        "progress_m": 1.4,
        "emergency_stop_events": 0,
        "localization_contradiction_samples": 0,
        "room_cruise_idle_confirmed": True,
        "m3_track_execution_samples": 360,
        "forbidden_path_samples": 0,
        "m5_full_stack_samples": 360,
        "m5_route_selector_samples": 360,
        "m5_local_path_segment_samples": 360,
        "m5_unique_goal_count": 4,
        "m5_goal_switch_ratio": 0.01,
        "m5_completed_waypoints": 3,
        "m5_replan_count": 1,
        "m5_visited_cell_count": 9,
        "m5_uncontrolled_reverse_samples": 0,
        "m5_start_environment": {
            "front_m": front_m,
            "min_clearance_m": min(front_m, 0.8),
            "left_clearance_m": 0.7,
            "right_clearance_m": 0.9,
            "rear_clearance_m": 1.2,
        },
    }
    return {
        "status": "PASS",
        "success": True,
        "summary": summary,
        "checks": {"collision_margin_ok": True},
        "start_pose": {"x": start_x, "y": 0.0, "theta_deg": start_heading},
        "end_pose": {"x": start_x + 1.2, "y": 0.4, "theta_deg": start_heading + 30.0},
    }


def test_multi_start_m5_requires_and_accepts_three_distinct_hub_runs(tmp_path: Path):
    paths = []
    for index, result in enumerate((_result(0.0, 0.0), _result(0.4, 5.0), _result(0.8, 35.0)), start=1):
        path = tmp_path / f"run_{index}.json"
        path.write_text(json.dumps(result), encoding="utf-8")
        paths.append(path)

    report = validate_results(paths)

    assert report["status"] == "PASS"
    assert report["run_count"] == 3
    assert report["distinct_start_run_count"] == 3
    assert report["aggregate_gates"]["all_runs_pass"]
    assert all(run["status"] == "PASS" for run in report["runs"])


def test_multi_start_m5_rejects_same_start_pose_repetition(tmp_path: Path):
    paths = []
    for index in range(3):
        path = tmp_path / f"same_{index}.json"
        path.write_text(json.dumps(_result(0.0, 0.0)), encoding="utf-8")
        paths.append(path)

    report = validate_results(paths)

    assert report["status"] == "FAIL"
    assert not report["aggregate_gates"]["different_start_conditions"]


def test_multi_start_m5_accepts_distinct_environment_when_boot_pose_resets(tmp_path: Path):
    paths = []
    for index, front_m in enumerate((0.7, 1.0, 1.3), start=1):
        path = tmp_path / f"environment_{index}.json"
        path.write_text(json.dumps(_result(0.0, 0.0, front_m)), encoding="utf-8")
        paths.append(path)

    report = validate_results(paths)

    assert report["status"] == "PASS"
    assert report["distinct_start_run_count"] == 3
    assert all(pair["environment_clearance_delta_m"] >= 0.25 for pair in report["start_pairs"])
