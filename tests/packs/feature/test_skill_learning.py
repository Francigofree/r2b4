"""Measured repair, offline reuse, and honest learning evidence boundaries."""
from functools import partial
import json
import threading
import time

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolSpec
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.skill_learning import SkillLearningLoop
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_runtime import SkillRuntime
from r2b4_orchestration.software_interface import test_skill as run_skill_test


FAULTY = """async def run(robot, samples, threshold=4):
    matches = []
    for value in samples:
        if value > threshold:
            matches.append(value)
    return {"matches": matches, "samples_checked": len(samples)}
"""
FIXED = FAULTY.replace("value > threshold", "value >= threshold")
SAVED_TEST = """import asyncio
from measured_threshold import run

def test_empty_input_has_no_matches():
    assert asyncio.run(run(None, [], 4)) == {"matches": [], "samples_checked": 0}
"""


class RepairProvider:
    model = "scripted-development-provider"

    def __init__(self):
        self.calls = 0

    def complete_agent_step(self, messages, *_catalogs, **_control):
        self.calls += 1
        assert messages[-1] == {"role": "user", "content": "Count every sample at or above threshold."}
        if self.calls == 1:
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill",
                "name": "measured_threshold", "source": FAULTY, "test_source": SAVED_TEST})
        context = next(item["content"] for item in messages if "R2B4_LEARNING_CONTEXT_JSON=" in item["content"])
        feedback = json.loads(context.split("R2B4_LEARNING_CONTEXT_JSON=", 1)[1])
        assert feedback["previous_iteration"]["assessment"]["checks"][0]["actual"] == [5]
        assert feedback["previous_iteration"]["assessment"]["checks"][0]["expected"] == [4, 5]
        assert feedback["previous_iteration"]["trial"]["runtime"]["result"]["matches"] == [5]
        assert feedback["selected_skill"]["source_hash"]
        assert feedback["selected_skill"]["source"] == FAULTY
        return AgentModelReply(self.model, skill_proposal={"kind": "update_skill",
            "name": "measured_threshold", "source": FIXED})


def core(library, model):
    tools = []
    for action in ("create", "update"):
        tools.append((AgentToolSpec("skill." + action, "Save Python", "READ"),
            lambda args, action=action: getattr(library, action)(**args)))
    tools.append((AgentToolSpec("skill.test", "Test Python", "READ"),
                  lambda args: run_skill_test(library, **args)))
    return AgentCore(model, AgentToolBroker(tools))


class WorkerTrials:
    """The independent oracle executes the actual saved file in a child interpreter."""
    def __init__(self, library, tmp_path):
        self.library = library
        self.socket = tmp_path / "no-device-service.sock"
        self.runtime_ids = []

    def __call__(self, name, parameters, scenario, *, timeout_s, cancel_event):
        with_runtime = SkillRuntime(self.library, self.socket)
        try:
            started = with_runtime.start(name, parameters)
            self.runtime_ids.append(started["run_id"])
            deadline = time.monotonic() + min(5, timeout_s)
            while with_runtime.active:
                if time.monotonic() >= deadline or cancel_event.is_set():
                    with_runtime.stop(started["run_id"], "TEST_TIMEOUT")
                    break
                time.sleep(.01)
            runtime = with_runtime.status(started["run_id"])
            actual = (runtime.get("result") or {}).get("matches")
            expected = [value for value in scenario["samples"] if value >= scenario["threshold"]]
            passed = actual == expected
            return {"runtime": runtime, "assessment": {"status": "PASSED" if passed else "FAILED",
                "scope": "motorless_parameter_oracle", "checks": [
                    {"name": "inclusive_threshold", "passed": passed, "actual": actual, "expected": expected}]}}
        finally:
            with_runtime.close()


def test_actual_worker_failure_measurement_drives_repair_then_reuse_needs_zero_model_calls(tmp_path):
    library = SkillLibrary(tmp_path / "robot_skills")
    provider = RepairProvider()
    trials = WorkerTrials(library, tmp_path)
    loop = SkillLearningLoop(core(library, provider), library, trials,
                             test_runner=partial(run_skill_test, library))
    parameters = {"samples": [1, 4, 5], "threshold": 4}
    scenario = dict(parameters)
    learned = loop.run("Count every sample at or above threshold.", parameters=parameters, scenario=scenario)
    assert learned["status"] == "LEARNED", learned
    assert learned["provider_calls"] == provider.calls == 2
    assert len(learned["iterations"]) == 2
    assert learned["iterations"][0]["assessment"]["status"] == "FAILED"
    assert learned["iterations"][1]["assessment"]["status"] == "PASSED"
    assert all(item["tests"]["status"] == "PASSED" for item in learned["iterations"])
    assert all(item["tests"]["executor"] == "AgentCore" for item in learned["iterations"])
    assert learned["source_hash"] != learned["current_hash"] == library.load("measured_threshold").source_hash
    assert all(item["trial"]["runtime"]["pid"] for item in learned["iterations"])
    reused = loop.run("Count every sample at or above threshold.", skill="measured_threshold",
                      parameters=parameters, scenario=scenario)
    assert reused["status"] == "LEARNED" and reused["provider_calls"] == 0
    assert provider.calls == 2 and len(trials.runtime_ids) == 3
    assert len(set(trials.runtime_ids)) == 3


class UnexpectedProvider:
    model = "must-not-be-called"
    def complete_agent_step(self, *_args, **_kwargs):
        raise AssertionError("an interrupted trial must never start provider repair")


def ready_loop(tmp_path, trial):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("baseline", "async def run(robot): return {'success': True}\n")
    def runner(name, _parameters, _scenario, **_control):
        runtime = {"state": "SUCCEEDED", "source_hash": library.load(name).source_hash,
                   "result": {"success": True}}
        return {"runtime": runtime, "assessment": {"status": "PASSED", "checks": [
            {"name": "independent", "passed": True, "expected": 1, "actual": 1}]}, **trial(runtime)}
    return library, SkillLearningLoop(core(library, UnexpectedProvider()), library, runner)


@pytest.mark.parametrize("error", ["WORKER_EXIT:-9", "WORKER_ERROR:pipe failed", "TIMEOUT", "SAFETY_FAULT"])
def test_unexpected_process_or_safety_evidence_stops_without_retry(tmp_path, error):
    def failed(runtime):
        return {"runtime": {**runtime, "state": "FAILED", "error": error}}
    _library, loop = ready_loop(tmp_path, failed)
    result = loop.run("Measure safely", skill="baseline")
    assert result["status"] == "FAILED" and result["reason"].startswith("TRIAL_INTERRUPTED:")
    assert len(result["iterations"]) == 1 and result["provider_calls"] == 0


@pytest.mark.parametrize("evidence", [
    {"status": "ERROR", "error": "broken capture"},
    {"replay": {"status": "MISMATCH"}},
    {"feedback": {"evidence": {"verification_status": "FAIL"}}},
    {"feedback": {"analyzers": [{"admission": {"state": "INSUFFICIENT_EVIDENCE"}}]}},
    {"feedback": {"replay": {"status": "MATCH", "verified_match": False}}},
    {"feedback": {"trial_binding": {"status": "EVIDENCE_BLOCKED", "reasons": ["SKILL_SOURCE_HASH_MISMATCH"]}}},
])
def test_successful_skill_and_scenario_cannot_override_broken_evidence(tmp_path, evidence):
    _library, loop = ready_loop(tmp_path, lambda _: {"evidence": evidence})
    result = loop.run("Measure safely", skill="baseline")
    assert result["status"] == "INCOMPLETE" and result["goal_satisfaction"] == "UNASSESSED"
    assert len(result["iterations"]) == 1 and result["provider_calls"] == 0


def test_self_report_without_independent_goal_predicate_is_incomplete(tmp_path):
    _library, loop = ready_loop(tmp_path, lambda _: {"assessment": {"status": "INCOMPLETE",
        "reason": "INDEPENDENT_GOAL_CHECK_REQUIRED", "checks": []}})
    result = loop.run("An arbitrary human goal", skill="baseline")
    assert result["status"] == "INCOMPLETE" and result["goal_satisfaction"] == "UNASSESSED"
    assert result["reason"] == "INDEPENDENT_GOAL_CHECK_REQUIRED"


def test_source_edit_during_trial_cannot_be_credited_to_previous_worker(tmp_path):
    library = None
    def changed(_runtime):
        library.update("baseline", "async def run(robot): return {'success': False}\n")
        return {}
    library, loop = ready_loop(tmp_path, changed)
    result = loop.run("Measure safely", skill="baseline")
    assert result["status"] == "FAILED" and result["reason"] == "SKILL_SOURCE_CHANGED_DURING_TRIAL"
    assert result["source_hash"] != result["current_hash"]


def test_cancelled_request_starts_no_trial_or_provider(tmp_path):
    _library, loop = ready_loop(tmp_path, lambda _: {})
    cancel = threading.Event()
    cancel.set()
    result = loop.run("Measure safely", skill="baseline", cancel_event=cancel)
    assert result["status"] == "CANCELLED" and not result["iterations"] and result["provider_calls"] == 0
