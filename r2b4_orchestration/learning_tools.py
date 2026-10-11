"""Explicit offline skill experiments using the existing host and evidence tools."""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import json

from .agent_contracts import AgentToolSpec


def measure_skill_trial(project_root, library, simulator, name, parameters, scenario, *,
                        timeout_s=30.0, cancel_event=None):
    """The developer experiment requests capture compilation/replay explicitly.

    None of this runs at capture finalization or in a resident control process.
    Compilation, diagnostics and replay retain their separate owner verdicts.
    """
    from v3.host_cli import hardware_guard
    from .agent_evidence_tools import build_evidence_tools, collect_evidence_feedback

    root = Path(project_root).resolve()
    with hardware_guard(root):
        trial = simulator.trial(name, parameters, scenario, timeout_s=min(120., timeout_s),
                                cancel_event=cancel_event)
    capture = trial.get("capture")
    if not capture or cancel_event is not None and cancel_event.is_set():
        return trial
    capture = Path(capture)
    tools = {spec.name: handler for spec, handler in build_evidence_tools(root)}
    # A fresh trial has fresh output names; compiler/replay cannot overwrite
    # previous local evidence. A pipeline failure is an evidence blocker.
    try:
        compiled = tools["evi.compile"]({"capture": str(capture),
                                         "output": str(capture.with_suffix(".evidence"))})
        replay = tools["replay.run"]({"capture": str(capture),
                                     "output": str(capture.with_suffix(".replay.json"))})
        feedback = collect_evidence_feedback(root, evidence=str(capture.with_suffix(".evidence")),
                                             capture=str(capture),
                                             replay_result=str(capture.with_suffix(".replay.json")))
        trial["evidence"] = {"compiled": compiled, "replay": replay, "feedback": feedback}
    except Exception as exc:
        trial["evidence"] = {"status": "ERROR", "error": f"{type(exc).__name__}:{exc}"[:1024]}
        trial["assessment"] = {**trial.get("assessment", {}), "status": "INCOMPLETE",
                                "reason": "EVIDENCE_PIPELINE_FAILED"}
    trial["feedback_path"] = str(capture.parent / "measured.json")
    with Path(trial["feedback_path"]).open("x") as stream:
        json.dump(trial, stream, ensure_ascii=False, indent=2, allow_nan=False)
    return trial


def build_learning_tools(project_root: Path):
    root = Path(project_root).resolve()

    def trial(arguments: Mapping[str, object]):
        from .skill_library import SkillLibrary
        from .skill_simulation import MotorlessSkillSimulator

        args = dict(arguments)
        if set(args) - {"name", "parameters", "scenario", "timeout_s"}:
            raise ValueError("unknown learning.trial arguments")
        timeout = args.get("timeout_s", 10.0)
        if type(timeout) not in {int, float} or not 0 < timeout <= 20:
            raise ValueError("trial timeout must be within (0, 20]")
        library = SkillLibrary(root / "robot_skills")
        library.load(args["name"])
        with MotorlessSkillSimulator(root, library) as simulator:
            return measure_skill_trial(root, library, simulator, args["name"], args.get("parameters", {}),
                                       args.get("scenario"), timeout_s=timeout)

    return ((AgentToolSpec(
        "learning.trial",
        "Run one explicit motor-free scenario through the shared SDK/host/Python worker, then compile its fresh capture, replay and measure DIAG. Independent scenario checks assess only simulated behavior.",
        "READ", {"name": "required saved skill name", "parameters": "optional object",
                  "scenario": "optional independent simulation scenario object; absent means INCOMPLETE",
                  "timeout_s": "optional seconds in (0,20], default 10"}), trial),)


__all__ = ["build_learning_tools", "measure_skill_trial"]
