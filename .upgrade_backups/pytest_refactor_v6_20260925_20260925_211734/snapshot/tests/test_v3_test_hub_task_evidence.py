"""Mode-aware task and tuning evidence stays descriptive and sampling-aware."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from v3.test_hub_motion_tuning import build_motion_tuning_evidence
from v3.test_hub_task_evidence import build_task_evidence


class FakeReader:
    def __init__(self, ticks, *, navigation=None, hz=10):
        self.ticks = tuple(ticks)
        self.navigation = navigation or {}
        self.hz = hz

    def iter_json_messages(self, *, topics):
        del topics
        for tick in self.ticks:
            yield SimpleNamespace(sequence=tick["tick_id"], log_time_ns=tick["monotonic_ns"]), tick

    def first_json(self, topic):
        del topic
        return (
            SimpleNamespace(),
            {"configuration": {"resolved_control": {"v3_navigation": self.navigation}}},
        )

    def latest_metadata(self, name):
        del name
        return {"tick_sample_hz": str(self.hz)}


def _candidate(candidate_id, *, collision=False, viable=True, total=0.8, potential=0.4, clearance=0.5):
    return {
        "candidate_id": candidate_id,
        "v_mps": 0.15,
        "omega_rad_s": 0.0,
        "collision": collision,
        "min_clearance_m": clearance,
        "progress_score": 0.5,
        "progress_potential_score": potential,
        "progress_viable": viable,
        "smoothness_score": 0.8,
        "novelty_score": 0.4,
        "total_score": total,
    }


def _tick(
    tick_id,
    *,
    mode="EXPLORE",
    x=0.0,
    y=0.0,
    yaw=0.0,
    progress=0.0,
    local_goal=None,
    tracks=(),
    nav_status="ACTIVE",
    nav_reason=None,
    target_pose=None,
    requested_v=0.2,
    allowed_v=0.18,
    actual_v=0.17,
    actual_omega=0.0,
    candidates=(),
):
    ns = 1_000_000_000 + tick_id * 100_000_000
    if local_goal is not None:
        local_goal = {"x_m": local_goal[0], "y_m": local_goal[1], "yaw_rad": None}
    if target_pose is not None:
        target_pose = {"x_m": target_pose[0], "y_m": target_pose[1], "yaw_rad": target_pose[2]}
    selected = candidates[0] if candidates else None
    return {
        "tick_id": tick_id,
        "monotonic_ns": ns,
        "inputs": {
            "raw_devices": {
                "samples": [
                    {
                        "__type__": "DeviceSample",
                        "device_id": "ENC",
                        "kind": "wheel_velocity",
                        "sequence": tick_id + 1,
                        "captured_monotonic_ns": ns,
                        "values": [
                            {"key": "left_mps", "value": actual_v + 0.01},
                            {"key": "right_mps", "value": actual_v + 0.02},
                        ],
                    }
                ]
            }
        },
        "expected": {
            "layers": {
                "L3": {"x_m": x, "y_m": y, "yaw_rad": yaw, "v_mps": actual_v, "omega_rad_s": actual_omega},
                "L4": {
                    "map_revision": tick_id,
                    "freshness_ns": 20_000_000,
                    "obstacle_tracks": list(tracks),
                    "local_costmap": {
                        "occupied_cells": [{"grid_x": 0, "grid_y": 0, "observation_count": 1}],
                    },
                },
                "L5": {
                    "mission_id": "mission-1",
                    "mode": mode,
                    "lifecycle": "ACTIVE",
                    "target_pose": target_pose,
                    "constraints": {
                        "goal_tolerance_m": 0.08,
                        "yaw_tolerance_rad": 0.10,
                    },
                },
                "L6": {
                    "mission_id": "mission-1",
                    "status": nav_status,
                    "reason": nav_reason,
                    "progress": progress,
                    "local_goal": local_goal,
                    "trajectory_candidates": list(candidates),
                },
                "L7": {"trajectory": selected},
                "L8": {"requested_v_mps": requested_v, "requested_omega_rad_s": 0.0},
                "L9": {"allowed_v_mps": allowed_v, "allowed_omega_rad_s": 0.0, "active_constraints": []},
                "L10": {"left_mps": allowed_v, "right_mps": allowed_v},
                "L11": {"left_normalized": 0.40, "right_normalized": 0.42, "saturated": False},
                "L12": {"safety_decision": "ALLOW", "enabled": True},
            }
        },
    }


def _episodes(path: Path, mode: str, start: int, end: int, path_length=0.5):
    row = {
        "episode_id": "behavior-1",
        "command_id": "cmd-1",
        "mission_id": "mission-1",
        "mode": mode,
        "start_tick": start,
        "end_tick": end,
        "duration_s": (end - start) * 0.1,
        "path_length_m": path_length,
        "incident_ids": [],
    }
    path.write_text(json.dumps(row) + "\n", encoding="utf-8")
    return path


def _lines(path: Path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def test_explore_exports_coverage_cells_and_local_goal_measurements_without_verdict(tmp_path):
    ticks = [
        _tick(0, x=0.00, progress=0.10, local_goal=(0.6, 0.0)),
        _tick(1, x=0.10, progress=0.10, local_goal=(0.6, 0.0)),
        _tick(2, x=0.30, progress=0.12, local_goal=(0.6, 0.0)),
        _tick(3, x=0.55, progress=0.14, local_goal=(1.1, 0.0)),
    ]
    navigation = {"exploration": {"coverage_cell_size_m": 0.25, "coverage_max_cells": 512, "local_goal_max_age_ns": 8_000_000_000}}
    episode_path = _episodes(tmp_path / "episodes.ndjson", "EXPLORE", 0, 3)
    result = build_task_evidence(FakeReader(ticks, navigation=navigation), tmp_path, behavior_episodes_path=episode_path)
    row = _lines(tmp_path / result["episodes"])[0]
    metrics = row["mode_metrics"]
    assert metrics["coverage_progress_fraction"]["delta"] == pytest.approx(0.04)
    assert metrics["sampled_pose_cells"]["unique_cell_count"] == 3
    assert metrics["sampled_pose_cells"]["revisit_sample_count"] == 1
    assert metrics["local_goal"]["observed_goal_count"] == 2
    assert row["policy"].startswith("DESCRIPTIVE_EVIDENCE_ONLY")
    assert "quality" not in json.dumps(row).lower()
    assert "recommendation" not in json.dumps(row).lower()


def test_follow_person_reconstructs_observed_lock_loss_and_reacquire_from_capture_config(tmp_path):
    person_a = {"track_id": "person-a", "x_m": 1.5, "y_m": 0.0, "radius_m": 0.3, "vx_mps": 0.0, "vy_mps": 0.0, "confidence": 0.8}
    person_a_low = {**person_a, "x_m": 1.4, "confidence": 0.50}
    person_b = {"track_id": "person-b", "x_m": 1.0, "y_m": 0.2, "radius_m": 0.3, "vx_mps": 0.0, "vy_mps": 0.0, "confidence": 0.99}
    ticks = [
        _tick(0, mode="FOLLOW_PERSON", tracks=(person_a,)),
        _tick(1, mode="FOLLOW_PERSON", tracks=()),
        _tick(2, mode="FOLLOW_PERSON", tracks=(person_a_low, person_b)),
        _tick(3, mode="FOLLOW_PERSON", tracks=(person_a_low, person_b)),
    ]
    navigation = {"follow_person": {"minimum_confidence": 0.6, "retention_minimum_confidence": 0.45, "stand_off_m": 1.05, "distance_deadband_m": 0.15, "min_safe_distance_m": 0.75}}
    episode_path = _episodes(tmp_path / "episodes.ndjson", "FOLLOW_PERSON", 0, 3)
    result = build_task_evidence(FakeReader(ticks, navigation=navigation), tmp_path, behavior_episodes_path=episode_path)
    row = _lines(tmp_path / result["episodes"])[0]
    metrics = row["mode_metrics"]
    assert metrics["target_identity"]["locked_track_id"] == "person-a"
    assert metrics["target_identity"]["observed_target_switch_count"] == 0
    assert metrics["visibility"]["observed_loss_count"] == 1
    assert metrics["visibility"]["observed_reacquisition_count"] == 1
    events = _lines(tmp_path / result["timeline"])
    assert {event["event_type"] for event in events} >= {
        "FOLLOW_TARGET_ACQUIRED_OBSERVED",
        "FOLLOW_TARGET_LOST_OBSERVED",
        "FOLLOW_TARGET_REACQUIRED_OBSERVED",
    }


@pytest.mark.parametrize("return_uid, reacquisitions, switches", [("person-a", 2, 0), ("person-b", 1, 1)])
def test_captured_follow_visibility_transitions_do_not_repeat_during_acquire(
    tmp_path, return_uid, reacquisitions, switches,
):
    observations = ((None, False), ("person-a", True), ("person-a", False),
                    ("person-a", True), (None, False), (None, False),
                    (None, False), (return_uid, True), (return_uid, True))
    ticks = []
    for tick_id, (uid, visible) in enumerate(observations):
        tick = _tick(tick_id, mode="FOLLOW_PERSON")
        tick["tick_evidence"] = [{"__type__": "FollowPersonEvidence",
                                 "locked_target_uid": uid, "target_visible": visible,
                                 "state": "ACQUIRE" if uid is None else "FOLLOW"}]
        ticks.append(tick)
    episode_path = _episodes(tmp_path / "episodes.ndjson", "FOLLOW_PERSON", 0, len(ticks) - 1)
    result = build_task_evidence(FakeReader(ticks), tmp_path, behavior_episodes_path=episode_path)
    metrics = _lines(tmp_path / result["episodes"])[0]["mode_metrics"]
    assert metrics["visibility"]["observed_loss_count"] == 2
    assert metrics["visibility"]["observed_reacquisition_count"] == reacquisitions
    assert metrics["visibility"]["visible_sample_count"] == 4
    assert metrics["visibility"]["missing_sample_count"] == 1
    assert metrics["target_identity"]["observed_target_switch_count"] == switches
    loss_events = [event for event in _lines(tmp_path / result["timeline"])
                   if event["event_type"] == "FOLLOW_TARGET_LOST_OBSERVED"]
    assert len(loss_events) == 2
    assert all(event["track_id"] == "person-a" for event in loss_events)


def test_navigate_exports_goal_errors_and_complete_observation_not_success_verdict(tmp_path):
    ticks = [
        _tick(0, mode="NAVIGATE", x=0.0, target_pose=(1.0, 0.0, 0.0)),
        _tick(1, mode="NAVIGATE", x=0.8, target_pose=(1.0, 0.0, 0.0)),
        _tick(2, mode="NAVIGATE", x=0.95, target_pose=(1.0, 0.0, 0.0), nav_status="COMPLETE"),
    ]
    episode_path = _episodes(tmp_path / "episodes.ndjson", "NAVIGATE", 0, 2, path_length=0.98)
    result = build_task_evidence(FakeReader(ticks), tmp_path, behavior_episodes_path=episode_path)
    row = _lines(tmp_path / result["episodes"])[0]
    metrics = row["mode_metrics"]
    assert metrics["distance_to_goal_m"]["start"] == pytest.approx(1.0)
    assert metrics["distance_to_goal_m"]["final"] == pytest.approx(0.05)
    assert metrics["complete_observation_count"] == 1
    rendered = json.dumps(row).lower()
    assert '"success"' not in rendered
    assert '"failure"' not in rendered
    assert '"verdict"' not in rendered


def test_motion_tuning_exports_cross_layer_wheel_and_planner_measurements_only(tmp_path):
    candidates = (
        _candidate("a", total=0.9, viable=True, potential=0.5, clearance=0.6),
        _candidate("b", total=0.7, viable=False, potential=0.0, collision=True, clearance=0.1),
    )
    ticks = [
        _tick(0, mode="EXPLORE", candidates=candidates),
        _tick(1, mode="EXPLORE", candidates=candidates),
        _tick(2, mode="EXPLORE", candidates=candidates),
    ]
    episode_path = _episodes(tmp_path / "episodes.ndjson", "EXPLORE", 0, 2)
    result = build_motion_tuning_evidence(FakeReader(ticks), tmp_path, behavior_episodes_path=episode_path)
    summary = json.loads((tmp_path / result["summary"]).read_text(encoding="utf-8"))
    assert summary["capture_tick_sample_hz"] == 10
    assert summary["motion_segment_count"] == 1
    assert summary["all_samples"]["wheel_tracking_error"]["left_measured_minus_target_mps"]["count"] == 3
    assert summary["all_samples"]["planner_candidates"]["progress_viable_fraction"]["mean"] == pytest.approx(0.5)
    assert summary["all_samples"]["planner_candidates"]["top_score_margin"]["mean"] == pytest.approx(0.2)
    rendered = json.dumps(summary).lower()
    assert '"diagnosis"' not in rendered
    assert '"finding"' not in rendered
    assert '"recommendation"' not in rendered
    assert '"verdict"' not in rendered
