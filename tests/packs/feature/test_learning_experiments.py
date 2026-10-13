"""Measured learning stays scoped to environments and explicit physical permission."""
from __future__ import annotations

import asyncio
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

from r2b4_orchestration import learning_tools
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.learning_tools import build_learning_tools, measure_skill_suite
from r2b4_orchestration.skill_learning import SkillLearningLoop
from r2b4_orchestration.skill_library import SkillLibrary
from v3.config import ConfigResolver

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def learning_root(tmp_path):
    shutil.copytree(ROOT / "conf", tmp_path / "conf")
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("baseline", "async def run(robot): return {'success': True}\n")
    return tmp_path, library


def handlers(root, **kwargs):
    return {spec.name: handler for spec, handler in build_learning_tools(root, **kwargs)}


def test_live_plan_is_realizable_asks_permission_and_never_calls_robot(learning_root):
    root, library = learning_root
    calls, events = [], []

    class Interface:
        def capabilities(self):
            return {"capabilities": {name: {"supported": True, "available": True, "ready": True}
                for name in ("v3.command.move_relative", "v3.command.turn_by")}}

        def execute(self, *args, **kwargs):
            calls.append((args, kwargs))
            pytest.fail("a permission request must never execute robot motion")

    planner = handlers(root, interface=Interface(), event_sink=lambda event, fields: events.append((event, fields)))["learning.live_plan"]
    result = planner({"name": "baseline", "steps": [
        {"action": "v3.command.move_relative", "parameters": {"forward_m": .2}},
        {"action": "v3.command.turn_by", "parameters": {"angle_deg": 30}}]})
    wheels = ConfigResolver.for_project(root).resolve().runtime.composition.live_control.control.navigation.wheel_limits
    assert result["status"] == "APPROVAL_REQUIRED" and result["execution_started"] is False and not calls
    assert "Engedélyezed" in result["approval_question"]
    assert result["source_hash"] == library.load("baseline").source_hash
    assert result["original_skill_execution"] is False
    assert result["bounded_execution"]["source_sha256"] == hashlib.sha256(
        result["bounded_execution"]["source"].encode()).hexdigest()
    move, turn = result["proposal"]["steps"]
    assert move["parameters"]["max_v_mps"] >= wheels.target_floor_mps
    assert turn["parameters"]["max_omega_rad_s"] >= wheels.target_center_spin_rad_s
    assert result["proposal"]["constraints"] == {}  # Finite motion is not a behavior duration constraint.
    assert result["capture"]["mode"] == "full" and result["capture"]["tick_sample_hz"] == 10
    assert events[0][0] == "learning_live_approval_requested" and events[0][1]["measurement_time_ns"] > 0
    compile(result["bounded_execution"]["source"], "reviewed_live_trial", "exec")


def test_live_reviewed_sdk_timeout_stops_and_does_not_retry(learning_root):
    root, _ = learning_root
    planner = handlers(root)["learning.live_plan"]
    result = planner({"name": "baseline", "max_duration_s": 1, "steps": [
        {"action": "v3.command.move_relative", "parameters": {"forward_m": .2}},
        {"action": "v3.command.turn_by", "parameters": {"angle_deg": 30}}]})
    namespace = {}
    exec(result["bounded_execution"]["source"], namespace)
    calls, stopped = [], []

    class Robot:
        async def call(self, action, **parameters):
            calls.append((action, parameters))
            await asyncio.Event().wait()

        async def stop(self):
            stopped.append(True)

    with pytest.raises(TimeoutError):
        asyncio.run(namespace["run"](Robot()))
    assert len(calls) == 1 and stopped == [True]
    assert calls[0][1]["capture_mode"] == "full" and calls[0][1]["capture_hz"] == 10


@pytest.mark.parametrize("first_result", [
    {"status": "INTERRUPTED", "reason": "SAFETY_STOP"},
    {"status": "INTERRUPTED", "reason": "RUNTIME_FAULT:HEALTH_FAULT"},
    {"status": "INTERRUPTED", "reason": "TIMEOUT"},
    None,
])
def test_live_reviewed_sdk_interruption_stops_without_second_motion(learning_root, first_result):
    root, _ = learning_root
    result = handlers(root)["learning.live_plan"]({"name": "baseline", "steps": [
        {"action": "v3.command.move_relative", "parameters": {"forward_m": .2}},
        {"action": "v3.command.turn_by", "parameters": {"angle_deg": 30}}]})
    namespace, calls, stopped = {}, [], []
    exec(result["bounded_execution"]["source"], namespace)

    class Robot:
        async def call(self, action, **parameters):
            calls.append(action)
            return first_result

        async def stop(self):
            stopped.append(True)

    with pytest.raises(RuntimeError, match="LIVE_LEARNING_ACTION_INTERRUPTED"):
        asyncio.run(namespace["run"](Robot()))
    assert calls == ["v3.command.move_relative"] and stopped == [True]


@pytest.mark.parametrize("change", [
    {"approved": True},
    {"steps": [{"action": "v3.command.move_relative", "parameters": {"forward_m": .2, "max_v_mps": .12}}]},
    {"steps": [{"action": "v3.command.turn_by", "parameters": {"angle_deg": 30, "max_omega_rad_s": .3}}]},
    {"steps": [{"action": "v3.command.teleop", "parameters": {"v_mps": .2, "omega_rad_s": 0}}]},
])
def test_live_plan_rejects_boolean_approval_impossible_speed_and_unbounded_motion(learning_root, change):
    root, _ = learning_root
    request = {"name": "baseline", "steps": [{"action": "v3.command.move_relative", "parameters": {"forward_m": .2}}], **change}
    with pytest.raises(ValueError):
        handlers(root)["learning.live_plan"](request)


def test_explicit_suite_measures_all_environments_and_stops_on_unexpected_fault(learning_root, monkeypatch, tmp_path):
    root, library = learning_root
    directory = tmp_path / "experiment"
    directory.mkdir()
    simulator = SimpleNamespace(root=directory)
    requested = []

    def measure(_root, _library, _simulator, name, parameters, scenario, **kwargs):
        requested.append(scenario["environment_id"])
        failed = scenario["environment_id"] == "fault"
        return {"schema": "R2B4_SKILL_TRIAL_V1", "runtime": {
            "source_hash": library.load(name).source_hash, "state": "FAILED" if failed else "SUCCEEDED",
            "error": "HEALTH_FAULT" if failed else None},
            "scenario": scenario, "physical_output": False,
            "assessment": {"status": "FAILED" if failed else "PASSED", "checks": [
                {"name": "goal", "passed": not failed}]}}

    monkeypatch.setattr(learning_tools, "measure_skill_trial", measure)
    scenario = {"kind": "scenario_suite", "scenarios": [{"environment_id": name} for name in ("open", "tight")]}
    measured = measure_skill_suite(root, library, simulator, "baseline", {}, scenario)
    assert requested == ["open", "tight"] and measured["assessment"]["status"] == "PASSED"
    assert {row["environment_id"] for row in measured["assessment"]["checks"]} == {"open", "tight"}
    path = Path(measured["feedback_path"])
    assert measured["feedback_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    selected = handlers(root)["learning.read"]({"feedback_path": str(path), "sha256": measured["feedback_sha256"], "pointer": "/trials"})
    assert selected["item_count"] == 2
    requested.clear()
    scenario["scenarios"] = [{"environment_id": name} for name in ("open", "fault", "never")]
    measured = measure_skill_suite(root, library, simulator, "baseline", {}, scenario)
    assert requested == ["open", "fault"] and measured["assessment"]["status"] == "INCOMPLETE"
    assert "HEALTH_FAULT" in measured["assessment"]["reason"]


def test_simulated_physical_success_requires_live_validation_and_uses_zero_llm_calls(learning_root):
    _root, library = learning_root

    class Provider:
        def complete_agent_step(self, *args, **kwargs):
            pytest.fail("a measured passing baseline must use zero model calls")

    def trial(name, *_args, **_kwargs):
        return {"runtime": {"state": "SUCCEEDED", "source_hash": library.load(name).source_hash},
            "assessment": {"status": "PASSED", "checks": [{"name": "independent_truth", "passed": True}]}}

    events = []
    result = SkillLearningLoop(AgentCore(Provider(), AgentToolBroker(())), library, trial).run(
        "Improve real robot motion", skill="baseline", scenario={"kind": "differential_drive"},
        event_sink=lambda event, fields: events.append(event))
    assert result["status"] == "LIVE_VALIDATION_REQUIRED" and result["physical_validation"] == "NOT_PERFORMED"
    assert result["goal_satisfaction"] == "ASSESSED_IN_SIMULATION" and result["provider_calls"] == 0
    assert events[0] == "learning_started" and events[-1] == "learning_completed"


def test_comparison_cannot_recommend_change_from_unverified_artifacts(learning_root):
    root, _ = learning_root
    paths = []
    for name, error in (("baseline", .3), ("variant", .02)):
        path = root / (name + ".json")
        path.write_text(json.dumps({"schema": "R2B4_SKILL_TRIAL_V1",
            "scenario": {"kind": "differential_drive", "environment_id": "open", "seed": 7},
            "simulation_config": {"base_sha256": "a" * 64},
            "assessment": {"status": "PASSED" if name == "variant" else "FAILED"},
            "performance": {"goal_reached": name == "variant", "metrics": {"goal_position_error_m": error}}}))
        paths.append(path)
    compare = handlers(root)["learning.compare"]
    result = compare({"baseline": str(paths[0]), "variant": str(paths[1])})
    assert result["status"] == "INCONCLUSIVE" and result["change_recommended"] is False
    assert "TRIAL_EVIDENCE_NOT_VERIFIED" in {row["reason"] for row in result["blockers"]}
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        compare({"baseline": str(paths[0]), "variant": str(paths[1]), "variant_sha256": "0" * 64})


def test_actual_simulated_capture_cannot_validate_live_robot_motion(learning_root):
    from r2b4_orchestration.skill_simulation import MotorlessSkillSimulator
    root, library = learning_root
    library.create("observe", '''async def run(robot):
    sample = await robot.read("simulation.observation")
    return {"observed": sample}
''')
    scenario = {"kind": "semantic_trace", "resources": {"simulation.observation": [{"value": 1}]},
                "expected_result": {"observed": {"value": 1}}}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        measured = learning_tools.measure_skill_trial(ROOT, library, simulator, "observe", {}, scenario, timeout_s=5)
    runtime = measured["runtime"]
    result = handlers(ROOT)["learning.live_feedback"]({"evidence": str(Path(measured["capture"]).with_suffix(".evidence")),
        "capture": measured["capture"], "run_id": runtime["run_id"], "source_hash": runtime["source_hash"],
        "conditions": [{"metric": "tracking/linear_rmse_mps", "maximum": 1}]})
    assert result["status"] == "EVIDENCE_BLOCKED" and result["goal_satisfaction"] == "UNASSESSED"
    assert result["measurement_scope"]["status"] != "BOUND_TO_SKILL_RUN"
    assert len(json.dumps(result).encode()) <= 12_000
    path = Path(result["feedback_path"])
    assert result["feedback_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    full = handlers(ROOT)["learning.read"]({"feedback_path": str(path), "sha256": result["feedback_sha256"],
                                           "pointer": "/evidence/trial_binding/status"})
    assert full["data"] == "VERIFIED"


@pytest.mark.parametrize("terminal_state,safety_decision,safety_reason,action_status,expected", [
    ("SUCCEEDED", "ALLOW", None, "COMPLETED", "PASSED"),
    ("SUCCEEDED", "STOP", "NOT_ACTIVE", "COMPLETED", "PASSED"),
    ("FAILED", "ALLOW", None, "COMPLETED", "FAILED"),
    ("CANCELLED", "ALLOW", None, "COMPLETED", "FAILED"),
    ("SUCCEEDED", "FAULT", "CRITICAL_DEVICE_FAILED", "COMPLETED", "FAILED"),
    ("SUCCEEDED", "STOP", "LIDAR_CLEARANCE_LOW", "COMPLETED", "FAILED"),
    ("SUCCEEDED", "ALLOW", None, "INTERRUPTED", "FAILED"),
])
def test_passing_live_metric_cannot_hide_terminal_fault_or_interrupted_action(
        learning_root, monkeypatch, terminal_state, safety_decision, safety_reason, action_status, expected):
    from r2b4_orchestration import agent_evidence_tools
    from tools.mcap_evidence.reader import EvidenceBundle
    from v3 import mcap_reader
    from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC
    root, _ = learning_root
    scope = {"status": "BOUND_TO_SKILL_RUN", "started_ns": 100, "finished_ns": 300,
             "clock_epoch": "source-boot", "run_id": "run", "source_hash": "source"}
    feedback = {"evidence": {"verification_status": "PASS", "source_integrity": "COMPLETE"},
        "capture": {"status": "VERIFIED"}, "measurement_scope": scope,
        "trial_binding": {"status": "VERIFIED", "terminal_events": [{
            "state": terminal_state, "started_ns": 100, "finished_ns": 300,
            "evidence_ref": {"clock_epoch": "source-boot"}}]},
        "analyzers": [{"analyzer_id": "motion_performance", "admission": {"state": "APPLICABLE"},
                       "metrics": {"performance": {"allowed_motion_s": .1},
                                   "tracking": {"active_command_pairs": 1, "linear_rmse_mps": .001}}}],
        "replay": {"status": "NOT_REQUESTED"}}
    monkeypatch.setattr(agent_evidence_tools, "collect_evidence_feedback", lambda *args, **kwargs: feedback)

    class Reader:
        def __init__(self, *_args):
            pass

        def iter_messages(self, **_kwargs):
            yield SimpleNamespace(json=lambda: {"metadata": {"runtime": "v3_process_runtime",
                "command_gateway": "AtomicResidentCommandGateway", "monotonic_boot_id": "source-boot"}})

    monkeypatch.setattr(mcap_reader, "McapReader", Reader)

    class Bundle:
        def iter_view(self, view, **_kwargs):
            if view == "ticks":
                # Actual motion first, then a failed boundary. A small tracking
                # error on the earlier motion must not turn that run successful.
                for decision, reason in (("ALLOW", None), (safety_decision, safety_reason)):
                    yield SimpleNamespace(payload={"inputs": {"context": {"monotonic_ns": 200}},
                        "expected": {"layers": {"L12": {"left_output": .3, "right_output": .3,
                            "safety_decision": decision, "reason": reason}, "L3": {"local_v_mps": .2}}}})
            elif view == "events":
                yield SimpleNamespace(payload={"source_topic": PUBLIC_RUNTIME_EVENT_TOPIC, "payload": {
                    "kind": "interface", "clock_epoch": "source-boot", "value": {
                        "kind": "ACTION_RESULT", "action": "v3.command.move_relative", "measurement_time_ns": 200,
                        "status": action_status, "reason": "SAFETY_STOP" if action_status == "INTERRUPTED" else None}}})

    monkeypatch.setattr(EvidenceBundle, "open", lambda *args, **kwargs: Bundle())
    result = handlers(root)["learning.live_feedback"]({"evidence": "explicit.evidence", "capture": "explicit.mcap",
        "run_id": "run", "source_hash": "source", "conditions": [{"metric": "tracking/linear_rmse_mps", "maximum": .01}]})
    assert result["checks"][0]["passed"] is True
    assert result["status"] == expected
    assert result["goal_satisfaction"] == ("ASSESSED_BY_LIVE_METRICS" if expected == "PASSED" else "UNASSESSED")


def test_actual_learning_cli_records_model_and_final_events_in_full_10hz_capture(learning_root):
    from r2b4_orchestration.agent_contracts import AgentModelReply
    from r2b4_orchestration.learning_cli import run_learning
    from v3.mcap_capture import EVENT_TOPIC
    from v3.mcap_reader import McapReader
    from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC
    root, library = learning_root

    class Provider:
        model = "scripted-local-test"

        def complete_agent_step(self, *_args, **_kwargs):
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill", "name": "learned_observer",
                "source": '''async def run(robot):
    sample = await robot.read("simulation.observation")
    return {"observed": sample}
'''})

    scenario = {"kind": "semantic_trace", "resources": {"simulation.observation": [{"value": 1}]},
                "expected_result": {"observed": {"value": 1}}}
    result = run_learning(ROOT, "Observe the independent semantic sample", model=Provider(), library=library,
                          scenario=scenario, timeout_s=30)
    assert result["status"] == "LEARNED" and result["provider_calls"] == 1, result
    reader = McapReader(result["learning_capture"])
    assert reader.latest_metadata("r2b4.capture")["tick_sample_hz"] == "10"
    host_events = [message.json()["payload"] for message in reader.iter_messages(topics=(EVENT_TOPIC,))
                   if message.json().get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
    observed = {(event["kind"], event["value"].get("event_type")) for event in host_events
                if isinstance(event.get("value"), dict)}
    assert ("learning_agent_event", "agent_llm_started") in observed
    assert ("learning_completed", "learning_completed") in observed
    assert Path(result["summary_path"]).is_file()

