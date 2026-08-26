#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path

from tools.m5_room_cruise_replay import replay_and_validate


def _frame(
    seq: int,
    x: float,
    *,
    localization_confidence: float = 1.0,
    matcher_degenerate: bool = False,
) -> dict:
    proposal = {
        "name": "room_cruise_v2_local_navigation",
        "details": {
            "room_cruise_v2": {"active": True},
            "navigation_intent": {
                "active": True,
                "mode": "ROOM_CRUISE",
                "goal": None,
            },
            "clearance": {
                "blocked_front": False,
                "obstacle_avoidance": {
                    "front_execution_envelope": {"wide_front_m": 2.0},
                },
                "reverse_escape_clearance": {
                    "blocked_back": False,
                    "min_dist_m": 2.0,
                },
            },
        },
    }
    return {
        "capture_seq": int(seq),
        "monotonic_ns": int((100.0 + (seq * 0.10)) * 1_000_000_000),
        "pipeline": {
            "stages": {
                "requested_motion": {"input": {"proposals": [proposal]}},
                "resolver": {
                    "input": {
                        "motion_tick_context": {
                            "lidar_seq": int(seq),
                            "pose": {"x": float(x), "y": 0.0, "theta_rad": 0.0},
                            "front_clearance_m": 2.0,
                            "left_clearance_m": 2.0,
                            "right_clearance_m": 2.0,
                        }
                    }
                },
                "localization_gate": {
                    "input": {
                        "lidar_odom_status": {
                            "candidate_confidence": float(localization_confidence),
                            "candidate_measurement_confidence": float(
                                localization_confidence
                            ),
                            "matcher_degenerate": bool(matcher_degenerate),
                            "matcher_degeneracy_reasons": (
                                ["weak_observability"]
                                if matcher_degenerate
                                else []
                            ),
                            "matcher_quality": {
                                "measurement_confidence": float(
                                    localization_confidence
                                )
                            },
                        }
                    }
                },
            }
        },
    }


def test_m5_replay_finds_goal_missing_root_cause_and_is_deterministic(tmp_path: Path):
    frames = tmp_path / "frames.jsonl"
    rows = [_frame(idx + 1, min(0.84, idx * 0.024)) for idx in range(40)]
    frames.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")

    result = replay_and_validate(frames)

    assert result["status"] == "PASS"
    assert result["gates"]["deterministic_replay"]
    assert result["gates"]["recorded_reactive_root_cause"]
    assert result["gates"]["persistent_goal_lifecycle"]
    assert result["gates"]["world_coverage_memory_used"]
    assert result["decision_digest"] == result["second_decision_digest"]
    assert result["metrics"]["unique_goal_count"] >= 1
    assert result["metrics"]["track_execution_samples"] >= 1


def test_m5_replay_keeps_035_to_050_confidence_as_diagnostic_only(tmp_path: Path):
    frames = tmp_path / "frames.jsonl"
    rows = [
        _frame(
            idx + 1,
            min(0.84, idx * 0.024),
            localization_confidence=0.40,
            matcher_degenerate=True,
        )
        for idx in range(40)
    ]
    frames.write_text(
        "".join(json.dumps(row) + "\n" for row in rows),
        encoding="utf-8",
    )

    result = replay_and_validate(frames)

    assert result["status"] == "PASS"
    assert result["metrics"]["confidence_below_035_samples"] == 0
    assert result["metrics"]["confidence_035_to_050_samples"] == 40
    assert result["metrics"]["matcher_degenerate_samples"] == 40
    assert all(
        row["motion_phase"] != "localization_confidence_pivot"
        for row in result["rows"]
    )
