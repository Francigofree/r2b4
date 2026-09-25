"""Generic behavior evidence tests for the offline Test Hub."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import v3.test_hub_next as test_hub_next
import v3.test_hub_portable as portable
from v3.test_hub_behavior import BEHAVIOR_SCHEMA, build_behavior_evidence
from v3.test_hub_next import run_default



class FakeReader:
    def __init__(self, ticks):
        self._ticks = tuple(ticks)

    def iter_json_messages(self, *, topics):
        del topics
        for payload in self._ticks:
            tick_id = payload["tick_id"]
            monotonic_ns = payload["monotonic_ns"]
            yield SimpleNamespace(sequence=tick_id, log_time_ns=monotonic_ns), payload

    def first_json(self, topic):
        del topic
        return None

    def sha256(self):
        return "0" * 64


def _tick(
    tick_id: int,
    *,
    command_id: str = "cmd-1",
    mission_id: str = "mission-cmd-1",
    l6_mission_id: str | None = None,
    mode: str = "TELEOP",
    lifecycle: str = "ACTIVE",
    progress: float = 0.0,
    navigation_status: str = "ACTIVE",
    requested_v: float = 0.15,
    requested_w: float = 0.0,
    allowed_v: float | None = None,
    allowed_w: float | None = None,
    safety: str = "ALLOW",
    x_m: float | None = None,
    y_m: float = 0.0,
):
    if allowed_v is None:
        allowed_v = requested_v
    if allowed_w is None:
        allowed_w = requested_w
    if l6_mission_id is None:
        l6_mission_id = mission_id
    if x_m is None:
        x_m = tick_id * 0.1
    monotonic_ns = 1_000_000_000 + tick_id * 20_000_000
    return {
        "record_type": "closed_input_tick",
        "tick_id": tick_id,
        "monotonic_ns": monotonic_ns,
        "inputs": {
            "command": {
                "__type__": "CommandRequest",
                "command_id": command_id,
                "mode": mode,
                "goal": [
                    {"__type__": "DataField", "key": "opaque_goal", "value": "value"}
                ],
                "expiry_tick": tick_id + 20,
            }
        },
        "expected": {
            "fault_layer": None,
            "layers": {
                "L3": {"x_m": x_m, "y_m": y_m, "yaw_rad": 0.0},
                "L5": {
                    "__type__": "MissionIntent",
                    "mission_id": mission_id,
                    "mode": mode,
                    "lifecycle": lifecycle,
                    "stop_reason": None if lifecycle == "ACTIVE" else "COMMAND_STOP",
                },
                "L6": {
                    "__type__": "NavigationPlan",
                    "mission_id": l6_mission_id,
                    "progress": progress,
                    "status": navigation_status,
                    "reason": None,
                },
                "L8": {"v_mps": requested_v, "omega_rad_s": requested_w},
                "L9": {
                    "allowed_v_mps": allowed_v,
                    "allowed_omega_rad_s": allowed_w,
                    "constraints": [],
                },
                "L12": {"safety_decision": safety, "reason": safety},
            },
        },
    }


def _inactive(tick_id: int, *, command_id: str = "stop", mission_id: str = "mission-stop"):
    return _tick(
        tick_id,
        command_id=command_id,
        mission_id=mission_id,
        mode="STOP",
        lifecycle="IDLE",
        requested_v=0.0,
        allowed_v=0.0,
        safety="STOP",
        navigation_status="IDLE",
    )


def _load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _load_lines(path: Path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _analyze(tmp_path: Path, ticks, triage=None):
    result = build_behavior_evidence(FakeReader(ticks), tmp_path, triage=triage or {})
    summary = _load_json(tmp_path / "behavior_summary.json")
    episodes = _load_lines(tmp_path / "behavior_episodes.ndjson")
    timeline = _load_lines(tmp_path / "behavior_timeline.ndjson")
    return result, summary, episodes, timeline


def test_no_active_mission_produces_empty_behavior_evidence(tmp_path):
    result, summary, episodes, timeline = _analyze(tmp_path, [_inactive(0), _inactive(1)])
    assert result["schema"] == BEHAVIOR_SCHEMA
    assert result["episode_count"] == 0
    assert summary["episode_count"] == 0
    assert episodes == []
    assert timeline == []


def test_teleop_is_one_generic_behavior_episode(tmp_path):
    _result, summary, episodes, _timeline = _analyze(
        tmp_path, [_inactive(0), _tick(1, progress=0.1), _tick(2, progress=0.4), _inactive(3)]
    )
    assert summary["episode_count"] == 1
    episode = episodes[0]
    assert episode["mode"] == "TELEOP"
    assert episode["command_id"] == "cmd-1"
    assert episode["mission_id"] == "mission-cmd-1"
    assert episode["command"]["goal"][0]["key"] == "opaque_goal"
    assert episode["navigation"]["progress_delta"] == pytest.approx(0.3)
    assert episode["end_reason"] == "MISSION_LIFECYCLE_CHANGED"


def test_explore_follow_person_and_future_mode_use_the_same_generic_path(tmp_path):
    for index, mode in enumerate(("EXPLORE", "FOLLOW_PERSON", "FUTURE_MODE"), 1):
        destination = tmp_path / str(index)
        _result, summary, episodes, _timeline = _analyze(
            destination,
            [
                _tick(
                    1,
                    command_id=f"cmd-{index}",
                    mission_id=f"mission-cmd-{index}",
                    mode=mode,
                )
            ],
        )
        assert summary["modes"] == {mode: 1}
        assert episodes[0]["mode"] == mode
        assert episodes[0]["end_reason"] == "CAPTURE_END"


def test_two_missions_create_two_episodes_and_explicit_correlation(tmp_path):
    ticks = [
        _tick(1, command_id="a", mission_id="mission-a", progress=0.1),
        _tick(2, command_id="a", mission_id="mission-a", progress=0.2),
        _tick(3, command_id="b", mission_id="mission-b", progress=0.3),
        _tick(4, command_id="b", mission_id="mission-b", progress=0.5),
    ]
    _result, summary, episodes, timeline = _analyze(tmp_path, ticks)
    assert summary["episode_count"] == 2
    assert summary["unique_command_count"] == 2
    assert summary["unique_mission_count"] == 2
    assert [episode["mission_id"] for episode in episodes] == ["mission-a", "mission-b"]
    assert episodes[0]["correlation"] == {
        "command_id": "a",
        "expected_mission_id": "mission-a",
        "mission_id": "mission-a",
        "command_mission_match": True,
        "navigation_mission_ids": ["mission-a"],
        "mission_plan_match": True,
    }
    assert episodes[0]["end_reason"] == "MISSION_CHANGED"
    assert any(row["event_type"] == "MISSION_CHANGE" for row in timeline)


def test_l5_l6_mission_mismatch_becomes_behavior_finding_not_new_incident(tmp_path):
    _result, summary, episodes, _timeline = _analyze(
        tmp_path,
        [
            _tick(1, mission_id="mission-a", l6_mission_id="mission-wrong"),
            _tick(2, mission_id="mission-a", l6_mission_id="mission-wrong"),
        ],
    )
    findings = episodes[0]["findings"]
    assert findings[0]["code"] == "MISSION_PLAN_ID_MISMATCH"
    assert findings[0]["count"] == 2
    assert findings[0]["first_tick"] == 1
    assert findings[0]["last_tick"] == 2
    assert episodes[0]["incident_ids"] == []
    assert summary["finding_counts"] == {"MISSION_PLAN_ID_MISMATCH": 1}


def test_motion_safety_path_and_incident_counts_are_episode_scoped(tmp_path):
    triage = {
        "incidents": [
            {"id": "before", "tick_id": 0},
            {"id": "motion-blocked-l9-2", "tick_id": 2},
            {"id": "after", "tick_id": 9},
        ]
    }
    ticks = [
        _tick(1, x_m=0.0, progress=0.1, safety="ALLOW"),
        _tick(2, x_m=1.0, progress=0.2, allowed_v=0.0, allowed_w=0.0, safety="ALLOW"),
        _tick(3, x_m=2.0, progress=0.3, safety="STOP"),
        _tick(4, x_m=2.5, progress=0.4, requested_v=0.0, allowed_v=0.0, safety="FAULT"),
    ]
    _result, summary, episodes, timeline = _analyze(tmp_path, ticks, triage)
    episode = episodes[0]
    assert episode["motion"] == {"requested_tick_count": 3, "constrained_zero_tick_count": 1}
    assert episode["safety"] == {"ALLOW": 2, "STOP": 1, "FAULT": 1}
    assert episode["path_length_m"] == 2.5
    assert episode["incident_ids"] == ["motion-blocked-l9-2"]
    assert summary["episodes_with_incidents"] == 1
    assert summary["total_requested_motion_ticks"] == 3
    assert summary["total_constrained_zero_ticks"] == 1
    assert summary["safety"] == {"ALLOW": 2, "STOP": 1, "FAULT": 1}
    event_types = {row["event_type"] for row in timeline}
    assert "MOTION_BLOCKED" in event_types
    assert "SAFETY_CHANGE" in event_types
    assert "MOTION_STOPPED" in event_types


def test_behavior_timeline_is_event_only_and_compact(tmp_path):
    ticks = [_tick(tick_id, progress=tick_id / 10.0) for tick_id in range(1, 8)]
    _result, _summary, _episodes, timeline = _analyze(tmp_path, ticks)
    assert len(timeline) < len(ticks)
    assert all(row["row_type"] == "event" for row in timeline)
    assert {row["event_type"] for row in timeline} <= {
        "BEHAVIOR_START",
        "MISSION_CHANGE",
        "MISSION_LIFECYCLE_CHANGE",
        "NAVIGATION_STATUS_CHANGE",
        "MOTION_STARTED",
        "MOTION_STOPPED",
        "MOTION_BLOCKED",
        "SAFETY_CHANGE",
        "BEHAVIOR_END",
    }


def test_run_default_wires_behavior_artifacts_agent_view_and_manifest_without_runtime_fixture(
    monkeypatch, tmp_path
):
    ticks = [
        _inactive(0),
        _tick(1, progress=0.1),
        _tick(2, progress=0.4),
        _inactive(3),
    ]
    reader = FakeReader(ticks)
    capture_path = tmp_path / "behavior-test.mcap"
    capture_path.write_bytes(b"offline-test-hub-fixture")
    destination = tmp_path / "behavior.evidence"

    def fake_diagnose_once(_capture, output_dir, _replay_mode):
        output_dir.mkdir(parents=True, exist_ok=False)
        return {
            "status": "PASS",
            "diagnosis_status": "PASS",
            "evidence_status": "PASS",
            "behavior_status": "PASS",
            "replay_status": "MATCH",
        }

    def fake_build_run_view(_capture, *, hz, output_path, triage):
        del hz, triage
        output_path.write_text(
            json.dumps({"row_type": "header", "effective_incidents": []}) + "\n",
            encoding="utf-8",
        )
        return {
            "effective_incidents": [],
            "data_coverage": {},
            "phases": [],
            "suppressed_agent_noise": [],
        }

    def fake_write_lidar_summary(_capture, output_path, *, reader):
        del reader
        output_path.write_text("", encoding="utf-8")
        return {"path": str(output_path), "scan_count": 0}, {}

    def fake_write_manifest(destination, _capture, **_kwargs):
        artifacts = sorted(path.name for path in destination.iterdir() if path.is_file())
        path = destination / "portable_manifest.json"
        path.write_text(
            json.dumps({"artifacts": artifacts}, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return path

    monkeypatch.setattr(test_hub_next, "McapReader", lambda _capture: reader)
    monkeypatch.setattr(test_hub_next, "_diagnose_once", fake_diagnose_once)
    monkeypatch.setattr(test_hub_next, "analyze_capture", lambda _reader: {})
    monkeypatch.setattr(test_hub_next, "build_run_view", fake_build_run_view)
    monkeypatch.setattr(test_hub_next, "write_incident_slices", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(
        test_hub_next, "write_lidar_summary", fake_write_lidar_summary
    )
    monkeypatch.setattr(
        test_hub_next, "write_raw_lidar_incident_slices", lambda *_args, **_kwargs: []
    )
    monkeypatch.setattr(test_hub_next, "write_portable_manifest", fake_write_manifest)

    result = run_default(
        capture_path,
        output_dir=destination,
        replay_mode="incident",
        replay_sweep_enabled=False,
    )

    # Replay remains an input from the canonical diagnosis path.  The Behavior
    # analyzer only derives additional offline evidence and does not own replay.
    assert result["replay_status"] == "MATCH"
    for name in (
        "behavior_summary.json",
        "behavior_episodes.ndjson",
        "behavior_timeline.ndjson",
    ):
        assert (destination / name).is_file()

    agent = _load_json(destination / "agent_view.json")
    assert agent["behavior"]["episode_count"] > 0
    assert agent["behavior"] == {
        "schema": BEHAVIOR_SCHEMA,
        "summary": "behavior_summary.json",
        "episodes": "behavior_episodes.ndjson",
        "timeline": "behavior_timeline.ndjson",
        "episode_count": agent["behavior"]["episode_count"],
        "correlation_keys": ["command_id", "mission_id"],
    }
    assert "behavior_status" in agent

    manifest = _load_json(destination / "portable_manifest.json")
    assert "behavior_summary.json" in manifest["artifacts"]
    assert "behavior_episodes.ndjson" in manifest["artifacts"]
    assert "behavior_timeline.ndjson" in manifest["artifacts"]


def test_testhub_pytest_scope_contains_behavior_tests(monkeypatch, tmp_path):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout="pass", stderr="")

    monkeypatch.setattr(portable.subprocess, "run", fake_run)
    result = portable.run_pytest(Path(__file__).resolve().parents[1], scope="testhub")
    assert result["status"] == "PASS"
    assert "tests/test_v3_test_hub_behavior.py" in result["command"]
    assert calls
