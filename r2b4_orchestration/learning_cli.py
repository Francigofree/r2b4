"""Bounded, measured LCE/LLM skill learning in an explicit motor-free experiment."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import threading


def parser():
    result = argparse.ArgumentParser(prog="r learn", description="Python-skill generálás, mérés és javítás motor nélküli LCE/LLM ciklusban.")
    result.add_argument("goal", nargs="+", help="a megtanulandó teljes cél szövege")
    result.add_argument("--skill", help="meglévő skill: először mért baseline, majd szükség szerinti javítás")
    result.add_argument("--parameters", default="{}", help="skillparaméterek JSON-objektumként")
    result.add_argument("--scenario", type=Path, help="független motor nélküli mérési feltételek JSON-fájlja")
    result.add_argument("--max-iterations", type=int, default=3)
    result.add_argument("--timeout-s", type=float, default=120.0)
    result.add_argument("--evidence", type=Path, help="explicit meglévő EVI háttérevidence, a friss próba külön adat")
    result.add_argument("--capture", type=Path, help="a háttérevidence eredeti MCAP authorityja")
    result.add_argument("--provider", help="a meglévő provider-választó beállítása")
    result.add_argument("--model", help="a meglévő provider modellbeállítása")
    result.add_argument("--json", action="store_true")
    result.epilog = "A siker a megadott szimulációs feltételekre vonatkozik. Forgatókönyv nélkül nincs bizonyított célteljesülés. Új evidence: egyedi /tmp könyvtár; a skill a robot_skills könyvtárban marad."
    return result


def _object(text, label):
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def run_learning(project_root, goal, *, model=None, provider=None, skill=None, parameters=None,
                 scenario=None, max_iterations=3, timeout_s=120.0, evidence=None, capture=None,
                 cancel_event=None, event_sink=None, library=None):
    from .agent_core import AgentCore, AgentToolBroker
    from .agent_tools import build_default_agent_tools
    from .learning_tools import measure_skill_trial
    from .skill_library import SkillLibrary
    from .skill_learning import SkillLearningLoop
    from .skill_simulation import MotorlessSkillSimulator
    from .software_interface import test_skill
    from r2b4_voice.conversation_interface import _OnDemandLLM
    from r2b4_voice.llm_provider import load_project_env, model_for, resolve_llm_provider
    from r2b4_voice.prompting import PromptAssembler

    root = Path(project_root).resolve()
    library = library or SkillLibrary(root / "robot_skills")
    # This is the same provider/authentication/failover path as conversation;
    # provider initialization remains on demand, so passing baseline uses zero calls.
    selected_model = model if callable(getattr(model, "complete_agent_step", None)) else _OnDemandLLM(
        provider=provider, model=model or (model_for(provider, selected_provider=resolve_llm_provider(),
            explicit_model=None, project_env=load_project_env(root)) if provider else None), project_root=root)
    background = None
    if evidence is not None:
        from .agent_evidence_tools import collect_evidence_feedback
        background = collect_evidence_feedback(root, evidence=str(evidence),
                                               capture=str(capture) if capture is not None else None)
    elif capture is not None:
        raise ValueError("--capture requires --evidence")
    with MotorlessSkillSimulator(root, library) as simulator:
        simulator.goal_text = goal
        if scenario is not None:
            simulator.scenario_descriptor(scenario)
            simulator.source.prepare(scenario, "simulation-discovery")
        # A simulation has no production-config restart capability. The same
        # owning read/evidence/skill tools remain available in this experiment.
        tools = tuple((spec, handler) for spec, handler in build_default_agent_tools(root, interface=simulator.interface)
                      if spec.name != "config.patch")
        agent = AgentCore(selected_model, AgentToolBroker(tools))

        def trial(name, parameters, scenario, *, timeout_s, cancel_event):
            return measure_skill_trial(root, library, simulator, name, parameters, scenario,
                                       timeout_s=timeout_s, cancel_event=cancel_event)

        loop = SkillLearningLoop(agent, library, trial,
                                 test_runner=lambda name, **kwargs: test_skill(library, name, **kwargs),
                                 system_prompt=PromptAssembler(root / "conf" / "r2b4_agent_system.md").system_prompt)
        summary = loop.run(goal, skill=skill, parameters=parameters, scenario=scenario,
                           max_iterations=max_iterations, timeout_s=timeout_s,
                           evidence=background, cancel_event=cancel_event, event_sink=event_sink)
        summary["artifact_directory"] = str(simulator.root)
        summary["execution_scope"] = "MOTORLESS_SIMULATION"
        summary["summary_path"] = str(simulator.root / "learning.json")
        with Path(summary["summary_path"]).open("x") as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
        return summary


def main(argv=None, *, project_root=None):
    args = parser().parse_args(argv)
    root = Path(project_root or Path(__file__).resolve().parents[1]).resolve()
    cancelled = threading.Event()
    try:
        scenario = None
        if args.scenario is not None:
            if args.scenario.stat().st_size > 65_536:
                raise ValueError("scenario exceeded 65536 bytes")
            scenario = _object(args.scenario.read_text(), "scenario")
        parameters = _object(args.parameters, "parameters")

        def report(event, fields):
            if event in {"learning_iteration_started", "learning_iteration_completed", "agent_llm_started"}:
                print(f"{event}: {fields.get('iteration', fields.get('round', ''))}", file=sys.stderr, flush=True)

        summary = run_learning(root, " ".join(args.goal), model=args.model, provider=args.provider,
                               skill=args.skill, parameters=parameters, scenario=scenario,
                               max_iterations=args.max_iterations, timeout_s=args.timeout_s,
                               evidence=args.evidence, capture=args.capture,
                               cancel_event=cancelled, event_sink=report)
        if args.json:
            print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        else:
            print(f"{summary['status']}: {summary.get('skill') or '(nincs skill)'}; "
                  f"LLM-hívások: {summary.get('provider_calls', 0)}")
            print(f"Evidence: {summary['artifact_directory']}")
            if summary.get("reason"):
                print(summary["reason"])
        return 0 if summary["status"] == "LEARNED" else 1
    except KeyboardInterrupt:
        cancelled.set()
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


__all__ = ["main", "parser", "run_learning"]
