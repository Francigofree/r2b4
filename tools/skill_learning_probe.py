"""Motor-free generate/measure/repair/reuse proof, with explicitly scripted reasoning.

Run: python3 -m tools.skill_learning_probe --json
Every source/evidence artifact is new and kept under /tmp. No device owner starts.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

from r2b4_orchestration.agent_contracts import AgentModelReply
from r2b4_orchestration.learning_cli import run_learning
from r2b4_orchestration.skill_library import SkillLibrary


GOAL = "Observe every supplied semantic sample; report detections at or above threshold 4 with their original measurement lineage."
SCENARIO = {"kind": "threshold_detection", "values": [1, 4, 5, 3], "threshold": 4}
FAULTY = '''async def run(robot, threshold=4):
    scenario = await robot.read("simulation.scenario")
    detections = []
    for _ in scenario["values"]:
        sample = await robot.read("simulation.sample")
        if sample["value"] > threshold:
            detections.append(sample)
    return {"detections": detections, "samples_checked": len(scenario["values"])}
'''
FIXED = FAULTY.replace('sample["value"] > threshold', 'sample["value"] >= threshold')
TEST = '''import asyncio
from learned_threshold_probe import run

class Robot:
    async def read(self, resource):
        return {"values": []}

def test_empty_observation_has_no_detection():
    assert asyncio.run(run(Robot())) == {"detections": [], "samples_checked": 0}
'''


class ScriptedRepairProvider:
    """A deterministic provider double; it makes no language-model proof claim."""
    model = "scripted-motorless-proof"

    def __init__(self):
        self.calls = 0

    def complete_agent_step(self, messages, *_catalogs, **_controls):
        self.calls += 1
        if self.calls == 1:
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill",
                "name": "learned_threshold_probe", "source": FAULTY, "test_source": TEST})
        if self.calls != 2:
            raise AssertionError("a saved passing skill must need no further inference")
        context = next(item["content"].split("R2B4_LEARNING_CONTEXT_JSON=", 1)[1]
                       for item in messages if "R2B4_LEARNING_CONTEXT_JSON=" in item["content"])
        feedback = json.loads(context)
        trial = feedback["previous_iteration"]["trial"]
        assert trial["assessment"]["status"] == "FAILED"
        assert trial["evidence"]["feedback"]["replay"]["verified_match"] is True
        assert feedback["selected_skill"]["source"] == FAULTY
        return AgentModelReply(self.model, skill_proposal={"kind": "update_skill",
            "name": "learned_threshold_probe", "source": FIXED})


def prove(project_root):
    root = Path(project_root).resolve()
    artifacts = Path(tempfile.mkdtemp(prefix="r2b4-learning-proof-", dir="/tmp"))
    library = SkillLibrary(artifacts / "robot_skills")
    provider = ScriptedRepairProvider()
    learned = run_learning(root, GOAL, model=provider, scenario=SCENARIO, library=library)
    if learned["status"] != "LEARNED":
        raise RuntimeError(json.dumps({"status": learned["status"], "reason": learned["reason"],
                                       "summary": learned["summary_path"]}))
    reused = run_learning(root, GOAL, model=provider, skill="learned_threshold_probe",
                          scenario=SCENARIO, library=library)
    if reused["status"] != "LEARNED" or reused["provider_calls"] != 0:
        raise RuntimeError("LLM-free reuse failed: " + str(reused["reason"]))
    result = {"status": "PASSED", "provider_kind": "SCRIPTED_DOUBLE", "physical_hardware": False,
              "provider_calls": provider.calls, "generation_repair_iterations": len(learned["iterations"]),
              "reused_provider_calls": reused["provider_calls"], "library": str(library.root),
              "learning_summary": learned["summary_path"], "reuse_summary": reused["summary_path"],
              "replay_statuses": [item["trial"]["evidence"]["replay"]["status"]
                                  for item in (*learned["iterations"], *reused["iterations"])],
              "proof_path": str(artifacts / "proof.json")}
    with Path(result["proof_path"]).open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    result = prove(Path(__file__).resolve().parents[1])
    print(json.dumps(result, ensure_ascii=False, indent=None if args.json else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
