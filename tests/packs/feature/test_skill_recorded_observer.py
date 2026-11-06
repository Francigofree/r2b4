"""Unmodified production read/observe SDK skills consume bounded recorded inputs."""
from __future__ import annotations

from pathlib import Path
import threading
import time

import pytest

from rig import ROOT
from r2b4_orchestration.learning_tools import measure_skill_trial
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_simulation import MotorlessSkillSimulator
from tools.mcap_evidence.query import query
from v3.mcap_capture import EVENT_TOPIC
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC


SOURCE = '''async def run(robot):
    camera = await robot.read("camera.status")
    async with robot.observe(object_kind="person") as observer:
        event = await robot.events.wait(observer, timeout_s=0.2)
        await robot.call("hri.report", text="observed:" + str(event.source_sequence))
        return {"camera": camera, "event": dict(event)}
'''


def scenario(*, complete=True, event_kind="qualified_presence", event_sequence=1):
    event = {"kind": event_kind, "sequence": event_sequence, "event_id": "original-observer:1",
             "observer": "original-observer", "measurement_monotonic_ns": 1_020_000_000,
             "source_sequence": 37, "source_frame_sequence": 41, "owner_generation": "recorded-owner-7",
             "revision": "detector-revision-2", "lineage": {"message_id": "original-evi-message-8"}}
    camera = {"camera_state": "ON", "measurement_monotonic_ns": 1_010_000_000,
              "owner_generation": "recorded-owner-7"}
    return {"kind": "semantic_trace", "resources": {"camera.status": [camera]},
            "observer": {"parameters": {"region": None, "object_kind": "person"}, "events": [event],
                         "start_monotonic_ns": 1_000_000_000, "end_monotonic_ns": 1_300_000_000,
                         "complete": complete},
            "expected_result": {"camera": camera, "event": event}, "expected_reports": ["observed:37"]}


def test_production_sdk_observer_preserves_recorded_identity_and_capture_lineage(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("recorded_observer", SOURCE)
    supplied = scenario()
    # The production owner stores normalized full-frame regions; a skill's
    # default region=None denotes that same region rather than a new binding.
    supplied["observer"]["parameters"]["region"] = {"xmin": 0., "ymin": 0., "xmax": 1., "ymax": 1.}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = measure_skill_trial(ROOT, library, simulator, "recorded_observer", {}, supplied, timeout_s=5)
        assert trial["runtime"]["state"] == "SUCCEEDED", trial
        assert trial["runtime"]["result"] == supplied["expected_result"]
        assert trial["assessment"]["status"] == "PASSED", trial
        assert trial["evidence"]["feedback"]["replay"]["verified_match"] is True
        assert trial["evidence"]["feedback"]["capture"]["raw_evidence_complete"] is True
        assert trial["physical_output"] is False
        assert not (simulator.root / "runtime" / "v3_command.json").exists()
        assert not simulator.recorded_observation._sessions
        rows = list(query(Path(trial["capture"]).with_suffix(".evidence"), topic=EVENT_TOPIC, limit=100))
        host = [row["payload"]["payload"] for row in rows
                if row["payload"].get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
        observations = [row["value"] for row in host if row["kind"] == "simulation_observation"]
        assert observations == [supplied["resources"]["camera.status"][0], supplied["observer"]["events"][0]]
        assert [row["value"]["text"] for row in host if row["kind"] == "report"] == ["observed:37"]


def test_sdk_scenario_inputs_do_not_expose_independent_result_or_report_oracles(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("inspect_inputs", '''async def run(robot):
    scenario = await robot.read("simulation.scenario")
    await robot.read("simulation.observation")
    return {"expected_result_visible": "expected_result" in scenario,
            "expected_reports_visible": "expected_reports" in scenario}
''')
    supplied = {"kind": "semantic_trace", "resources": {"simulation.observation": [{"value": 4}]},
                "expected_result": {"expected_result_visible": False, "expected_reports_visible": False},
                "expected_reports": []}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("inspect_inputs", {}, supplied, timeout_s=5)
        assert trial["runtime"]["state"] == "SUCCEEDED"
        assert trial["assessment"]["status"] == "PASSED"


@pytest.mark.parametrize("change", ["declared_partial", "gap", "sequence_loss"])
def test_missing_recorded_coverage_blocks_successful_program_return(tmp_path, change):
    library = SkillLibrary(tmp_path / "robot_skills")
    # The same program may return its expected value, while the input admission
    # still rejects incomplete recorded evidence independently.
    supplied = scenario(complete=change != "declared_partial",
                        event_kind="coverage_gap" if change == "gap" else "qualified_presence",
                        event_sequence=2 if change == "sequence_loss" else 1)
    library.create("recorded_observer", SOURCE if change != "sequence_loss" else SOURCE.replace(
        'event = await robot.events.wait(observer, timeout_s=0.2)',
        'event = await robot.events.wait(observer, timeout_s=0.2)\n'
        '        if event.kind == "coverage_gap":\n'
        '            event = await robot.events.wait(observer, timeout_s=0.2)'))
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("recorded_observer", {}, supplied, timeout_s=5)
        assert trial["runtime"]["state"] == "SUCCEEDED", trial
        assert trial["runtime"]["result"] == supplied["expected_result"]
        assert trial["assessment"]["status"] == "INCOMPLETE"
        assert trial["assessment"]["reason"] == "RECORDED_OBSERVER_COVERAGE_INCOMPLETE"


def test_finish_outside_recorded_interval_is_partial_and_never_live_freshness(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("finish_recorded", '''import asyncio
async def run(robot):
    async with robot.observe() as observer:
        return [dict(event) async for event in observer.finish_events(
            until=asyncio.get_running_loop().time() + 1, timeout_s=0.2)]
''')
    supplied = scenario()
    supplied["resources"] = {}
    supplied["expected_result"] = [supplied["observer"]["events"][0],
        {"kind": "coverage_gap", "reason": "INTERVAL_CLOSURE_PARTIAL"}]
    supplied["expected_reports"] = []
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("finish_recorded", {}, supplied, timeout_s=5)
        assert trial["runtime"]["state"] == "SUCCEEDED", trial
        assert trial["runtime"]["result"] == supplied["expected_result"]
        assert trial["assessment"]["status"] == "INCOMPLETE"


def test_observer_binding_and_owning_session_revocation_fail_closed(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("wrong_binding", '''async def run(robot):
    async with robot.observe(region={"xmin":0.2,"ymin":0.2,"xmax":0.8,"ymax":0.8}):
        return {"success": True}
''')
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("wrong_binding", {}, scenario(), timeout_s=5)
        assert trial["runtime"]["state"] == "FAILED"
        assert "RECORDED_OBSERVER_PARAMETERS_MISMATCH" in trial["runtime"]["error"]
        assert not simulator.recorded_observation._sessions

    marker = tmp_path / "opened"
    library.create("blocked_recorded", f'''from pathlib import Path
async def run(robot):
    async with robot.observe():
        Path({str(marker)!r}).touch()
        while True:
            pass
''')
    cancelled = threading.Event()
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        def cancel_after_open():
            deadline = time.monotonic() + 3
            while not marker.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            cancelled.set()
        canceller = threading.Thread(target=cancel_after_open)
        canceller.start()
        trial = simulator.trial("blocked_recorded", {}, scenario(), timeout_s=5, cancel_event=cancelled)
        canceller.join(timeout=1)
        assert marker.exists()
        assert trial["runtime"]["state"] == "CANCELLED"
        assert not trial["runtime"]["finalizing"]
        assert not simulator.recorded_observation._sessions
        assert trial["physical_output"] is False


@pytest.mark.parametrize("resource", ["v3.status", "operator.status", "system.status", "world.snapshot",
                                      "capture.status", "config.patch", "unpublished.observation"])
def test_recorded_read_cannot_override_owners_or_publish_actions(tmp_path, resource):
    supplied = {"kind": "semantic_trace", "resources": {resource: [{}]}, "expected_result": {}}
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("unused", "async def run(robot): return {}")
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        with pytest.raises(ValueError):
            simulator.trial("unused", {}, supplied, timeout_s=5)


def test_recorded_observer_does_not_compose_media_or_live_camera_owner(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("needs_media", '''async def run(robot):
    async with robot.observe() as observer:
        async with robot.media.event_recorder(observer):
            return {"success":True}
''')
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("needs_media", {}, scenario(), timeout_s=5)
        assert trial["runtime"]["state"] == "FAILED"
        assert "media.recorder.open" in trial["runtime"]["error"]
        assert trial["physical_output"] is False
        assert not simulator.recorded_observation._sessions
