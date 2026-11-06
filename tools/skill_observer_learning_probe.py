"""Motor-free recorded observer interpretation, measured repair and skill reuse.

Run: python3 -m tools.skill_observer_learning_probe --json
The provider is an explicit SCRIPTED_DOUBLE, not a language-model demonstration.
The unchanged production observe/events SDK runs in real Python skill workers.
All new skill, scenario and native capture evidence artifacts stay under /tmp.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

from r2b4_orchestration.agent_contracts import AgentModelReply
from r2b4_orchestration.learning_cli import run_learning
from r2b4_orchestration.skill_library import SkillLibrary
from tools.mcap_evidence.query import query
from tools.mcap_evidence.reader import EvidenceBundle
from v3.mcap_reader import EVENT_TOPIC
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC


NAME = "learned_observer_probe"
GOAL = "Observe the recorded person event and report its original source frame sequence; preserve camera and event measurement lineage."
CAMERA = {"camera_state": "ON", "measurement_monotonic_ns": 1_010_000_000,
          "owner_generation": "recorded-owner-7"}
EVENT = {"kind": "qualified_presence", "sequence": 1, "event_id": "original-observer:1",
         "observer": "original-observer", "measurement_monotonic_ns": 1_020_000_000,
         "source_sequence": 37, "source_frame_sequence": 41, "owner_generation": "recorded-owner-7",
         "revision": "detector-revision-2", "lineage": {"message_id": "original-evi-message-8"}}
# These expectations are experiment data established before program generation.
# They are supplied only to the independent trial owner, not to the skill SDK.
EXPECTED_REPORTS = ["person frame:41"]
SCENARIO = {"kind": "semantic_trace", "resources": {"camera.status": [CAMERA]},
            "observer": {"parameters": {"region": None, "object_kind": "person"}, "events": [EVENT],
                         "start_monotonic_ns": 1_000_000_000, "end_monotonic_ns": 1_300_000_000,
                         "complete": True},
            "expected_result": {"camera": CAMERA, "event": EVENT}, "expected_reports": EXPECTED_REPORTS}
FAULTY = '''async def run(robot):
    camera = await robot.read("camera.status")
    async with robot.observe(object_kind="person") as observer:
        event = await robot.events.wait(observer, timeout_s=0.2)
        if event is None:
            return {"status": "INCOMPLETE", "reason": "NO_RECORDED_EVENT"}
        await robot.call("hri.report", text="person frame:" + str(event.sequence))
        return {"camera": camera, "event": dict(event)}
'''
FIXED = FAULTY.replace("str(event.sequence)", "str(event.source_frame_sequence)")
TEST = '''import asyncio
from learned_observer_probe import run
from r2b4_orchestration.robot_sdk import RobotEvent

class Observer:
    async def __aenter__(self): return self
    async def __aexit__(self, *_): return None

class Robot:
    def __init__(self): self.events = self
    async def read(self, resource):
        assert resource == "camera.status"
        return {"owner_generation": "camera-7", "measurement_monotonic_ns": 10}
    def observe(self, **parameters):
        assert parameters == {"object_kind": "person"}
        return Observer()
    async def wait(self, observer, **parameters):
        return RobotEvent(kind="qualified_presence", sequence=2, source_sequence=17,
                          source_frame_sequence=19, measurement_monotonic_ns=12,
                          owner_generation="camera-7", lineage={"message_id":"source-17"})
    async def call(self, action, **parameters):
        assert action == "hri.report"

def test_observation_result_retains_original_measurement_and_frame_identity():
    result = asyncio.run(run(Robot()))
    assert result == {"camera": {"owner_generation":"camera-7", "measurement_monotonic_ns":10},
                      "event": {"kind":"qualified_presence", "sequence":2, "source_sequence":17,
                                "source_frame_sequence":19, "measurement_monotonic_ns":12,
                                "owner_generation":"camera-7", "lineage":{"message_id":"source-17"}}}
'''


def _require(condition, description):
    if not condition:
        raise RuntimeError(description)


class ScriptedObserverRepairProvider:
    """Exercise the real agent loop while keeping provider evidence explicit."""
    model = "scripted-recorded-observer-proof"

    def __init__(self):
        self.calls = 0

    def complete_agent_step(self, messages, *_catalogs, **_controls):
        self.calls += 1
        if self.calls == 1:
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill",
                "name": NAME, "source": FAULTY, "test_source": TEST})
        _require(self.calls == 2, "saved passing observer skill requested another provider call")
        context = next(item["content"].split("R2B4_LEARNING_CONTEXT_JSON=", 1)[1]
                       for item in messages if "R2B4_LEARNING_CONTEXT_JSON=" in item["content"])
        previous = json.loads(context)
        trial = previous["previous_iteration"]["trial"]
        _require(trial["runtime"]["state"] == "SUCCEEDED", "faulty baseline must return successfully")
        _require(trial["assessment"]["status"] == "FAILED", "independent report oracle did not reject baseline")
        report = next(item for item in trial["assessment"]["checks"] if item["name"] == "observed_reports")
        _require(report["actual"] == ["person frame:1"] and report["expected"] == EXPECTED_REPORTS,
                 "repair prompt lost the exact observer interpretation counterexample")
        feedback = trial["evidence"]["feedback"]
        _require(feedback["replay"]["verified_match"] is True, "baseline has no verified canonical replay")
        _require(feedback["trial_binding"]["status"] == "VERIFIED", "baseline is not bound to its skill version")
        _require(feedback["measurements"], "repair prompt lost canonical measurements")
        _require(previous["selected_skill"]["source"] == FAULTY, "repair is not based on the measured program")
        return AgentModelReply(self.model, skill_proposal={"kind": "update_skill", "name": NAME, "source": FIXED})


def _trial_proof(iteration, expected_reports):
    trial = iteration["trial"]
    runtime = trial["runtime"]
    feedback = trial["evidence"]["feedback"]
    _require(trial["physical_output"] is False, "trial claimed physical output")
    _require(runtime["source_hash"] == iteration["source_hash"], "skill source changed during the measured trial")
    _require(feedback["capture"]["status"] == "VERIFIED"
             and feedback["capture"]["complete"] is True
             and feedback["capture"]["raw_evidence_complete"] is True,
             "native capture requested scope is incomplete")
    _require(feedback["evidence"]["verification_source_checked"] is True, "authority capture was not checked")
    binding = feedback["trial_binding"]
    _require(binding["status"] == "VERIFIED"
             and binding["expected"]["run_id"] == runtime["run_id"]
             and binding["expected"]["source_hash"] == iteration["source_hash"],
             "evidence belongs to a different execution or program version")
    _require(feedback["replay"]["status"] == "MATCH"
             and feedback["replay"]["verified_match"] is True, "native replay did not MATCH")
    bundle = EvidenceBundle.open(Path(trial["capture"]).with_suffix(".evidence"), verification="manifest")
    outputs = list(bundle.iter_view("layers/L12"))
    _require(len(outputs) == bundle.view_count("ticks") == 2, "capture omitted native final outputs")
    _require(all(row.payload["left_output"] == row.payload["right_output"] == 0. for row in outputs),
             "a captured native final output is nonzero")
    host = [row["payload"]["payload"] for row in query(bundle.root, topic=EVENT_TOPIC, limit=100)
            if row["payload"].get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
    _require([row["value"] for row in host if row["kind"] == "simulation_observation"] == [CAMERA, EVENT],
             "recorded observer measurement/sequence/revision/lineage changed")
    _require([row["value"]["text"] for row in host if row["kind"] == "report"] == expected_reports,
             "capture reports differ from the independently checked output")
    return {"run_id": runtime["run_id"], "source_hash": runtime["source_hash"],
            "assessment": trial["assessment"]["status"], "capture": trial["capture"],
            "evidence": str(bundle.root), "bundle_manifest_sha256": feedback["evidence"]["bundle_manifest_sha256"],
            "source_sha256": feedback["evidence"]["source_sha256"], "replay": feedback["replay"]["status"],
            "trial_binding": binding["status"], "captured_final_output_rows": len(outputs),
            "nonzero_final_output_rows": 0, "reports": expected_reports}


def prove(project_root):
    root = Path(project_root).resolve()
    artifacts = Path(tempfile.mkdtemp(prefix="r2b4-observer-learning-proof-", dir="/tmp"))
    with (artifacts / "scenario.json").open("x") as stream:
        json.dump(SCENARIO, stream, ensure_ascii=False, indent=2, allow_nan=False)
    library = SkillLibrary(artifacts / "robot_skills")
    provider = ScriptedObserverRepairProvider()
    learned = run_learning(root, GOAL, model=provider, scenario=SCENARIO, library=library)
    _require(learned["status"] == "LEARNED", f"observer learning failed: {learned['reason']}; {learned['summary_path']}")
    _require(len(learned["iterations"]) == 2 and learned["provider_calls"] == provider.calls == 2,
             "observer repair did not use two measured skill versions")
    _require(learned["iterations"][0]["source_hash"] != learned["iterations"][1]["source_hash"],
             "repair did not change the program version")
    reused = run_learning(root, GOAL, model=provider, skill=NAME, scenario=SCENARIO, library=library)
    _require(reused["status"] == "LEARNED" and reused["provider_calls"] == 0 and provider.calls == 2,
             "passing observer skill could not run without provider intervention")
    trials = [_trial_proof(learned["iterations"][0], ["person frame:1"]),
              _trial_proof(learned["iterations"][1], EXPECTED_REPORTS),
              _trial_proof(reused["iterations"][0], EXPECTED_REPORTS)]
    _require(trials[0]["assessment"] == "FAILED"
             and all(item["assessment"] == "PASSED" for item in trials[1:]),
             "independent measured report failure was not repaired")
    _require(len({item["run_id"] for item in trials}) == 3, "reuse repeated a previous run identity")
    result = {"status": "PASSED", "provider_kind": "SCRIPTED_DOUBLE", "real_model_calls": 0,
              "physical_hardware": False, "observation_source": "RECORDED_SEMANTIC_TRACE",
              "provider_calls": provider.calls, "generation_repair_iterations": len(learned["iterations"]),
              "reused_provider_calls": reused["provider_calls"], "library": str(library.root),
              "scenario": str(artifacts / "scenario.json"), "learning_summary": learned["summary_path"],
              "reuse_summary": reused["summary_path"], "trials": trials,
              "proof_path": str(artifacts / "proof.json")}
    with Path(result["proof_path"]).open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    result = prove(Path(__file__).resolve().parents[1])
    print(json.dumps(result, ensure_ascii=False, indent=None if args.json else 2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
