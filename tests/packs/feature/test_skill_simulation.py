"""Motor-free semantic traces cross real Brain/SDK/workers and passive evidence."""
from __future__ import annotations

from collections import deque
import json
from pathlib import Path

import pytest

from rig import ROOT
from r2b4_orchestration.learning_tools import measure_skill_trial
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_simulation import MotorlessSkillSimulator
from tools.mcap_evidence.query import query
from v3.mcap_capture import EVENT_TOPIC
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC
from v3.mcap_reader import McapReader


def test_semantic_trace_report_has_exact_source_identity_and_complete_capture(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("observer", '''async def run(robot):
    sample = await robot.read("simulation.observation")
    await robot.call("hri.report", text=sample["label"])
    return {"observed": sample}
''')
    sample = {"label": "door-open", "measurement_time_ns": 17, "sequence": 4,
              "revision": "recorded-v1", "lineage": {"message_id": "original-source-4"}}
    scenario = {"kind": "semantic_trace", "resources": {"simulation.observation": [sample]},
                "expected_result": {"observed": sample}, "expected_reports": ["door-open"]}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        simulator.goal_text = "Report the recorded semantic observation exactly once."
        trial = measure_skill_trial(ROOT, library, simulator, "observer", {}, scenario, timeout_s=5)
        assert trial["runtime"]["state"] == "SUCCEEDED", trial
        assert trial["assessment"]["status"] == "PASSED", trial
        assert trial["brain_goal"]["text"] == simulator.goal_text
        assert trial["observations"] == [{"resource": "simulation.observation", "sample": sample}]
        assert trial["physical_output"] is False and trial["motor_writes"] == 2
        evidence = trial["evidence"]
        assert evidence["replay"]["status"] == "MATCH"
        assert evidence["feedback"]["capture"]["raw_evidence_complete"] is True
        assert evidence["feedback"]["replay"]["verified_match"] is True
        rows = list(query(Path(trial["capture"]).with_suffix(".evidence"), topic=EVENT_TOPIC, limit=100))
        host = [row["payload"]["payload"] for row in rows
                if row["payload"].get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
        observed = [row for row in host if row["kind"] == "simulation_observation"]
        assert len(observed) == 1 and observed[0]["value"] == sample
        reports = [row for row in host if row["kind"] == "report"]
        assert len(reports) == 1 and reports[0]["value"]["text"] == "door-open"
        assert reports[0]["producer_id"] == observed[0]["producer_id"]
        assert reports[0]["event_sequence"] > observed[0]["event_sequence"]
        assessments = [row["value"] for row in host if row["kind"] == "simulation_trial_assessment"]
        assert len(assessments) == 1
        assert assessments[0]["run_id"] == trial["runtime"]["run_id"]
        assert assessments[0]["source_hash"] == library.load("observer").source_hash
        assert not (simulator.root / "runtime" / "v3_command.json").exists()


def test_false_success_cannot_pass_trace_observation_or_report_checks(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("false_success", "async def run(robot): return {'success': True}\n")
    scenario = {"kind": "semantic_trace", "resources": {"simulation.observation": [{"value": 1}]},
                "expected_result": {"success": True}, "expected_reports": ["observed"]}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("false_success", {}, scenario, timeout_s=5)
        assert trial["runtime"]["state"] == "SUCCEEDED"
        assert trial["assessment"]["status"] == "FAILED"
        checks = {row["name"]: row["passed"] for row in trial["assessment"]["checks"]}
        assert checks["independent_result"] and not checks["all_trace_samples_observed"]
        assert not checks["observed_reports"]


def test_motion_is_unavailable_and_has_no_device_or_command_transport(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("motion", '''async def run(robot):
    return await robot.call("v3.command.move_relative", forward_m=1)
''')
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        capability = simulator.interface.capabilities()["capabilities"]["v3.command.move_relative"]
        assert capability["ready"] is False and capability["available"] is False
        trial = simulator.trial("motion", {}, {"kind": "threshold_detection"}, timeout_s=5)
        assert trial["runtime"]["state"] == "FAILED"
        assert "MOTORLESS_SIMULATION" in trial["runtime"]["error"]
        assert trial["runtime"]["finalizing"] is False and trial["physical_output"] is False
        assert not (simulator.root / "runtime" / "v3_command.json").exists()


def test_waiting_for_worker_drains_status_poll_evidence_before_bounded_queue_overflows(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("delayed_observer", '''async def run(robot):
    import asyncio
    await asyncio.sleep(.35)
    sample = await robot.read("simulation.observation")
    return {"observed": sample}
''')
    scenario = {"kind": "semantic_trace", "resources": {"simulation.observation": [{"value": 1}]},
                "expected_result": {"observed": {"value": 1}}}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        # A small bound exposes repeated Brain status polls deterministically;
        # the production queue remains unchanged at 256 entries.
        simulator.runtime._evidence_queue = deque(maxlen=32)
        trial = simulator.trial("delayed_observer", {}, scenario, timeout_s=15)
        assert trial["runtime"]["state"] == "SUCCEEDED", trial
        assert trial["assessment"]["status"] == "PASSED", trial
        assert simulator.runtime._evidence_dropped == 0
        journal = simulator.root / "runtime/public_world/events.ndjson"
    reader = McapReader(trial["capture"])
    integrity = reader.capture_integrity()["integrity"]
    assert integrity["sample_complete"] is True
    assert not integrity["integrity_reasons"], integrity
    events = [record.json() for record in reader.iter_messages(topics={EVENT_TOPIC})]
    captured = [row["payload"] for row in events if row.get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
    original = {row["event_sequence"]: row for row in map(json.loads, journal.read_text().splitlines())}
    assert captured and all(row == original[row["event_sequence"]] for row in captured)
    assert all(row["evidence_dropped"] == 0 for row in captured)
    assert all(b["event_sequence"] == a["event_sequence"]+1 for a, b in zip(captured, captured[1:]))


@pytest.mark.parametrize("scenario", [
    {"kind": "semantic_trace", "resources": {"v3.status": [{}]}, "expected_result": {}},
    {"kind": "semantic_trace", "resources": {"simulation.observation": [{}]}},
    {"kind": "threshold_detection", "values": [float("nan")]},
])
def test_invalid_scenario_cannot_supply_robot_authority_or_unchecked_success(scenario):
    with pytest.raises(ValueError):
        MotorlessSkillSimulator.scenario_descriptor(scenario)
