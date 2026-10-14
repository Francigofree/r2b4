"""Explicit offline skill experiments using the existing host and evidence tools."""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import hashlib
import json
import math
import re
import tempfile
import time

from .agent_contracts import AgentToolSpec


def bounded_feedback(value, *, max_bytes=4096, pointer=""):
    """Keep small facts intact; explicitly identify omitted artifact values."""
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    if len(encoded) <= max_bytes:
        return value
    result = {"omitted": True, "serialized_bytes": len(encoded),
              "sha256": hashlib.sha256(encoded).hexdigest(), "json_pointer": pointer}
    if isinstance(value, Mapping):
        result["key_count"] = len(value)
        if max_bytes > 1024:
            result["preview"] = {str(key): bounded_feedback(item, max_bytes=max(512, max_bytes // 16),
                pointer=pointer + "/" + str(key).replace("~", "~0").replace("/", "~1"))
                for key, item in list(value.items())[:8]}
    elif isinstance(value, list):
        result["item_count"] = len(value)
        if max_bytes > 1024:
            result["preview"] = [bounded_feedback(item, max_bytes=max(512, max_bytes // 8),
                pointer=pointer + "/" + str(index)) for index, item in enumerate(value[:3])]
    elif isinstance(value, str):
        result["preview"] = value[:max_bytes // 8]
    if len(json.dumps(result, ensure_ascii=False).encode()) > max_bytes:
        result.pop("preview", None)
    return result


def compact_test_feedback(tests):
    result = {key: bounded_feedback(value, max_bytes=1024) for key, value in tests.items() if key != "output"}
    output = tests.get("output")
    result["output"] = bounded_feedback(output)
    if isinstance(output, str) and result["output"] != output:
        # Pytest's assertion and exception details occur after its progress
        # prefix. Preserve the measured failure tail in the repair context.
        result["output_tail"] = output[-2048:]
    return result


def compact_assessment(assessment):
    if not isinstance(assessment, Mapping):
        return assessment
    checks = assessment.get("checks", [])
    if not isinstance(checks, list):
        return bounded_feedback(assessment, pointer="/assessment")
    # Failed checks lead repair even when a long successful prefix precedes them.
    indices = [i for i, row in enumerate(checks) if isinstance(row, Mapping) and row.get("passed") is False]
    indices += [i for i in range(len(checks)) if i not in indices]
    selected = sorted(indices[:4])
    result = {key: bounded_feedback(value, max_bytes=1024, pointer="/assessment/" + key)
              for key, value in assessment.items() if key != "checks"}
    result["checks"] = [{key: bounded_feedback(value, max_bytes=1024,
        pointer=f"/assessment/checks/{i}/" + key) for key, value in checks[i].items()}
        if isinstance(checks[i], Mapping) else checks[i] for i in selected]
    result.update(check_count=len(checks), failed_check_count=sum(
        isinstance(row, Mapping) and row.get("passed") is False for row in checks),
        checks_truncated=len(selected) < len(checks), check_indices=selected)
    return result


def compact_trial_feedback(trial):
    """One repair/tool view; the complete owner result stays in measured.json."""
    result = {key: trial.get(key) for key in ("schema", "capture", "learning_capture", "feedback_path", "feedback_sha256",
                                             "physical_output", "artifact_directory")}
    for key in ("performance", "comparison", "next_experiment"):
        if key in trial:
            result[key] = bounded_feedback(trial[key], max_bytes=4096, pointer="/" + key)
    if "trials" in trial:
        result["trials"] = [{key: item.get(key) for key in ("feedback_path", "feedback_sha256", "capture", "learning_capture")}
            | {"summary": bounded_feedback({"runtime": {key: item.get("runtime", {}).get(key) for key in (
                "run_id", "source_hash", "state", "error")}, "assessment": compact_assessment(item.get("assessment")),
                "performance": item.get("performance")}, max_bytes=3072, pointer=f"/trials/{index}")}
            for index, item in enumerate(trial["trials"])]
    runtime = trial.get("runtime", {})
    result["runtime"] = {key: runtime.get(key) for key in (
        "run_id", "source_hash", "state", "error", "pid", "returncode", "callback_error")}
    result["runtime"]["result"] = bounded_feedback(runtime.get("result"), pointer="/runtime/result")
    result["assessment"] = compact_assessment(trial.get("assessment"))
    evidence = trial.get("evidence", {})
    feedback = evidence.get("feedback", {})
    selected = {key: feedback.get(key) for key in ("schema", "root_cause_inferred")}
    for key in ("evidence", "capture", "replay", "trial_binding", "measurement_scope"):
        selected[key] = bounded_feedback(feedback.get(key), pointer="/evidence/feedback/" + key)
    selected["measurements"] = [{key: bounded_feedback(value, max_bytes=1024,
        pointer=f"/evidence/feedback/measurements/{i}/" + key) for key, value in item.items()}
        for i, item in enumerate(feedback.get("measurements", []))]
    selected["analyzers"] = [{key: bounded_feedback(value, max_bytes=2048,
        pointer=f"/evidence/feedback/analyzers/{i}/" + key) for key, value in item.items()
        if key in {"analyzer_id", "admission", "summary", "metrics", "observations", "episodes",
                   "observations_count", "observations_truncated", "episodes_count", "episodes_truncated"}}
        for i, item in enumerate(feedback.get("analyzers", []))]
    result["evidence"] = {"status": evidence.get("status"), "error": evidence.get("error"), "feedback": selected}
    return result


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
                                             analyzers=("evidence_health", "safety", "motion_performance") if
                                                 trial.get("scenario", {}).get("kind") == "differential_drive" else
                                                 ("evidence_health", "safety"),
                                             replay_result=str(capture.with_suffix(".replay.json")),
                                             binding={key: trial["runtime"].get(key)
                                                      for key in ("run_id", "source_hash")})
        binding = feedback["trial_binding"]
        returned = json.dumps(trial["runtime"].get("result"), sort_keys=True,
                              separators=(",", ":"), allow_nan=False).encode()
        digest = hashlib.sha256(returned).hexdigest()
        for terminal in binding["terminal_events"]:
            if terminal.get("result_sha256") != digest:
                binding["reasons"].append("SKILL_RESULT_SHA256_MISMATCH")
            if terminal["state"] != trial["runtime"].get("state"):
                binding["reasons"].append("TERMINAL_SKILL_STATE_MISMATCH")
        assessment_digest = hashlib.sha256(json.dumps(trial.get("assessment"), sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        if not binding["assessment_event_count"]:
            binding["reasons"].append("TRIAL_ASSESSMENT_EVENT_NOT_FOUND")
        for assessment in binding["assessment_events"]:
            if assessment.get("assessment_sha256") != assessment_digest:
                binding["reasons"].append("TRIAL_ASSESSMENT_SHA256_MISMATCH")
        if binding["reasons"]:
            binding.update(status="EVIDENCE_BLOCKED", reasons=list(dict.fromkeys(binding["reasons"])))
        trial["evidence"] = {"compiled": compiled, "replay": replay, "feedback": feedback}
        if feedback["trial_binding"]["status"] != "VERIFIED":
            trial["assessment"] = {**trial.get("assessment", {}), "status": "INCOMPLETE",
                                    "reason": "TRIAL_EVIDENCE_BINDING_FAILED"}
    except Exception as exc:
        trial["evidence"] = {"status": "ERROR", "error": f"{type(exc).__name__}:{exc}"[:1024]}
        trial["assessment"] = {**trial.get("assessment", {}), "status": "INCOMPLETE",
                                "reason": "EVIDENCE_PIPELINE_FAILED"}
    trial["schema"] = "R2B4_SKILL_TRIAL_V1"
    trial["feedback_path"] = str(capture.parent / "measured.json")
    with Path(trial["feedback_path"]).open("x") as stream:
        json.dump(trial, stream, ensure_ascii=False, indent=2, allow_nan=False)
    trial["feedback_sha256"] = hashlib.sha256(Path(trial["feedback_path"]).read_bytes()).hexdigest()
    return trial


def scenario_suite(value):
    """Only an explicit bounded suite broadens the experiment environment."""
    if not isinstance(value, Mapping) or value.get("kind") != "scenario_suite":
        return None
    if set(value) != {"kind", "scenarios"} or not isinstance(value["scenarios"], list) or not 1 <= len(value["scenarios"]) <= 8:
        raise ValueError("scenario_suite requires 1..8 explicit scenarios")
    if any(not isinstance(item, Mapping) for item in value["scenarios"]):
        raise ValueError("suite scenarios must be objects")
    return list(value["scenarios"])


def measure_skill_suite(project_root, library, simulator, name, parameters, scenario, *,
                        timeout_s=30.0, cancel_event=None):
    scenarios = scenario_suite(scenario)
    if scenarios is None:
        return measure_skill_trial(project_root, library, simulator, name, parameters, scenario,
                                   timeout_s=timeout_s, cancel_event=cancel_event)
    from .skill_learning import SkillLearningLoop
    deadline = time.monotonic() + timeout_s
    trials, checks = [], []
    loaded = library.load(name)
    reason = None
    for index, item in enumerate(scenarios):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or cancel_event is not None and cancel_event.is_set():
            reason = "SUITE_TIMEOUT_OR_CANCELLED"
            break
        measured = measure_skill_trial(project_root, library, simulator, name, parameters, item,
                                       timeout_s=remaining, cancel_event=cancel_event)
        trials.append(measured)
        runtime, assessment = measured.get("runtime", {}), measured.get("assessment", {})
        if runtime.get("source_hash") != loaded.source_hash or library.load(name).source_hash != loaded.source_hash:
            reason = "SKILL_SOURCE_CHANGED_DURING_SUITE"
            break
        reason = SkillLearningLoop._interruption(runtime, assessment) or SkillLearningLoop._evidence_blocker(measured.get("evidence"))
        checks.extend({**check, "name": f"scenario_{index + 1}:" + str(check.get("name")),
                       "environment_id": measured.get("scenario", {}).get("environment_id", str(index + 1))}
                      for check in assessment.get("checks", []))
        if reason or assessment.get("status") == "INCOMPLETE":
            reason = reason or assessment.get("reason") or "SUITE_SCENARIO_INCOMPLETE"
            break
    complete = len(trials) == len(scenarios) and reason is None
    passed = complete and bool(checks) and all(row.get("passed") is True for row in checks)
    runtime = dict(trials[-1]["runtime"]) if trials else {"source_hash": loaded.source_hash, "state": "FAILED"}
    if reason:
        runtime.update(state="FAILED", error=reason)
    result = {"schema": "R2B4_SKILL_SUITE_V1", "runtime": runtime,
              "scenario": dict(scenario), "source": "simulation", "physical_output": False,
              "assessment": {"status": "PASSED" if passed else "FAILED" if complete else "INCOMPLETE",
                  "reason": reason, "scope": "independent_simulation_suite", "checks": checks},
              "trials": trials, "artifact_directory": str(simulator.root)}
    path = Path(simulator.root) / f"suite-{len(list(Path(simulator.root).glob('suite-*.json'))) + 1}.json"
    result["feedback_path"] = str(path)
    with path.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    result["feedback_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def _measured_artifact(root, value, expected_sha256=None):
    path = Path(value)
    if not path.is_absolute():
        path = root / path
    if path.is_symlink() or not path.is_file():
        raise ValueError("feedback must be an explicit regular measured artifact")
    with path.open("rb") as stream:
        raw = stream.read(8_388_609)
    if len(raw) > 8_388_608:
        raise ValueError("measured artifact exceeds 8 MiB")
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and expected_sha256 != digest:
        raise ValueError("measured artifact SHA256 mismatch")
    result = json.loads(raw)
    if not isinstance(result, Mapping) or result.get("schema") not in {"R2B4_SKILL_TRIAL_V1", "R2B4_SKILL_SUITE_V1", "R2B4_LIVE_LEARNING_FEEDBACK_V1", "R2B4_SKILL_LEARNING_V1"}:
        raise ValueError("expected R2B4_SKILL_TRIAL_V1, R2B4_SKILL_SUITE_V1, R2B4_SKILL_LEARNING_V1 or R2B4_LIVE_LEARNING_FEEDBACK_V1 measured artifact")
    return path, digest, result


def build_learning_tools(project_root: Path, *, interface=None, event_sink=None):
    root = Path(project_root).resolve()

    def emit(event, fields):
        if event_sink is not None:
            event_sink(event, {"measurement_time_ns": time.monotonic_ns(), "clock_domain": "host_monotonic",
                               **bounded_feedback(fields, max_bytes=6144)})

    def trial(arguments: Mapping[str, object]):
        from .skill_library import SkillLibrary
        from .skill_simulation import MotorlessSkillSimulator

        args = dict(arguments)
        if set(args) - {"name", "parameters", "scenario", "timeout_s"}:
            raise ValueError("unknown learning.trial arguments")
        timeout = args.get("timeout_s", 10.0)
        if type(timeout) not in {int, float} or not math.isfinite(timeout) or not 0 < timeout <= 20:
            raise ValueError("trial timeout must be within (0, 20]")
        library = SkillLibrary(root / "robot_skills")
        library.load(args["name"])
        with MotorlessSkillSimulator(root, library) as simulator:
            emit("learning_trial_started", {"skill": args["name"], "source_hash": library.load(args["name"]).source_hash})
            measured = measure_skill_suite(root, library, simulator, args["name"],
                args.get("parameters", {}), args.get("scenario"), timeout_s=timeout)
            emit("learning_trial_completed", {"skill": args["name"], "assessment": compact_assessment(measured.get("assessment")),
                "feedback_path": measured.get("feedback_path"), "feedback_sha256": measured.get("feedback_sha256")})
            return compact_trial_feedback(measured)

    def read(arguments: Mapping[str, object]):
        args = dict(arguments)
        if set(args) - {"feedback_path", "sha256", "pointer", "offset", "limit"}:
            raise ValueError("unknown learning.read arguments")
        path, digest, value = _measured_artifact(root, args["feedback_path"], args.get("sha256"))
        pointer = args.get("pointer", "")
        if not isinstance(pointer, str) or len(pointer) > 1024 or pointer and not pointer.startswith("/"):
            raise ValueError("pointer must be a bounded JSON pointer")
        for token in pointer.split("/")[1:] if pointer else ():
            if re.search(r"~(?![01])", token):
                raise ValueError("invalid JSON pointer escape")
            token = token.replace("~1", "/").replace("~0", "~")
            if isinstance(value, list):
                if not re.fullmatch(r"0|[1-9][0-9]*", token):
                    raise ValueError("invalid JSON pointer array index")
                value = value[int(token)]
            else:
                value = value[token]
        text = isinstance(value, str)
        offset, limit = args.get("offset", 0), args.get("limit", 2048 if text else 8)
        maximum = 4096 if text else 32
        if type(offset) is not int or offset < 0 or type(limit) is not int or not 1 <= limit <= maximum:
            raise ValueError(f"offset must be nonnegative and limit within [1,{maximum}]")
        result = {"feedback_path": str(path.resolve()), "sha256": digest, "pointer": pointer}
        if isinstance(value, list):
            rows = value[offset:offset + limit]
            result.update(item_count=len(value), offset=offset,
                next_offset=offset + len(rows) if offset + len(rows) < len(value) else None,
                data=[bounded_feedback(row, max_bytes=max(512, 8192 // limit),
                    pointer=pointer + "/" + str(offset + i)) for i, row in enumerate(rows)])
        elif text:
            segment = value[offset:offset + limit]
            result.update(character_count=len(value), offset=offset, offset_units="unicode_code_points",
                next_offset=offset + len(segment) if offset + len(segment) < len(value) else None, data=segment)
        elif isinstance(value, Mapping):
            items = list(value.items())[offset:offset + limit]
            result.update(key_count=len(value), offset=offset, offset_units="object_keys",
                next_offset=offset + len(items) if offset + len(items) < len(value) else None,
                data={key: bounded_feedback(item, max_bytes=max(512, 8192 // limit),
                    pointer=pointer + "/" + key.replace("~", "~0").replace("/", "~1")) for key, item in items})
        else:
            if offset:
                raise ValueError("offset requires a list, object or string selection")
            result["data"] = bounded_feedback(value, max_bytes=8192, pointer=pointer)
        return result

    def compare(arguments):
        from .skill_simulation import compare_trials
        args = dict(arguments)
        if set(args) - {"baseline", "variant", "baseline_sha256", "variant_sha256"}:
            raise ValueError("unknown learning.compare arguments")
        _, baseline_hash, baseline = _measured_artifact(root, args["baseline"], args.get("baseline_sha256"))
        _, variant_hash, variant = _measured_artifact(root, args["variant"], args.get("variant_sha256"))
        result = compare_trials(baseline.get("trials", [baseline]), variant.get("trials", [variant]))
        result = {**result, "baseline_sha256": baseline_hash, "variant_sha256": variant_hash}
        emit("learning_comparison", result)
        return bounded_feedback(result, max_bytes=8192)

    def live_plan(arguments):
        """Prepare a reviewable experiment; this tool has no execution call."""
        from .skill_library import SkillLibrary
        from v3.config import ConfigResolver
        args = dict(arguments)
        if set(args) - {"name", "parameters", "steps", "max_duration_s", "purpose"}:
            raise ValueError("unknown learning.live_plan arguments; approval is supplied by the user conversation")
        loaded = SkillLibrary(root / "robot_skills").load(args["name"])
        parameters = args.get("parameters", {})
        if not isinstance(parameters, Mapping):
            raise ValueError("parameters must be an object")
        steps = args.get("steps")
        if not isinstance(steps, list) or not 1 <= len(steps) <= 4:
            raise ValueError("live experiment requires 1..4 explicit finite canonical steps")
        duration = args.get("max_duration_s", 20.0)
        if type(duration) not in {int, float} or not math.isfinite(duration) or not 1 <= duration <= 30:
            raise ValueError("live experiment duration must be within [1,30] seconds")
        purpose = args.get("purpose", "A skill robotikai teljesítményének mérése.")
        if not isinstance(purpose, str) or not 0 < len(purpose) <= 1024:
            raise ValueError("purpose must be bounded nonempty text")
        resolved = ConfigResolver.for_project(root).resolve()
        control = resolved.runtime.composition.live_control.control
        wheels = control.motion_realization.wheel_limits
        linear_floor = wheels.target_floor_mps
        angular_floor = wheels.target_center_spin_rad_s
        linear_cap = min(.25, wheels.maximum_mps, control.operational_constraints.max_v_mps,
                         control.mission.default_constraints.max_v_mps, resolved.edges.command_ingress.maximum_linear_speed_mps)
        angular_cap = min(1.2, control.operational_constraints.max_omega_rad_s,
                          control.mission.default_constraints.max_omega_rad_s, resolved.edges.command_ingress.maximum_angular_speed_rad_s)
        if linear_floor > linear_cap or angular_floor > angular_cap:
            raise ValueError("calibrated robot motion cannot fit the bounded learning envelope")
        reviewed = []
        distance, angle = 0., 0.
        for step in steps:
            if not isinstance(step, Mapping) or set(step) != {"action", "parameters"} or not isinstance(step["parameters"], Mapping):
                raise ValueError("each live step requires action and parameters")
            action, params = step["action"], dict(step["parameters"])
            if action == "v3.command.move_relative":
                if set(params) - {"forward_m", "left_m", "final_yaw_rad", "max_v_mps", "max_omega_rad_s"} or "forward_m" not in params:
                    raise ValueError("live movement requires finite relative distance")
                params.setdefault("left_m", 0.)
                params.setdefault("max_v_mps", linear_floor)
                params.setdefault("max_omega_rad_s", angular_floor)
                if any(type(value) not in {int, float} or not math.isfinite(value) for value in params.values()):
                    raise ValueError("live motion parameters must be finite numbers")
                length = math.hypot(params["forward_m"], params["left_m"])
                if length <= 0 or length > .5 or not linear_floor <= params["max_v_mps"] <= linear_cap or not angular_floor <= params["max_omega_rad_s"] <= angular_cap:
                    raise ValueError("live movement exceeds bounded learning envelope")
                distance += length
                if "final_yaw_rad" in params:
                    if abs(params["final_yaw_rad"]) > math.pi / 2:
                        raise ValueError("live final yaw exceeds 90 degrees")
                    angle += abs(math.degrees(params["final_yaw_rad"]))
            elif action == "v3.command.turn_by":
                if set(params) - {"angle_deg", "max_omega_rad_s"} or "angle_deg" not in params:
                    raise ValueError("live turn requires finite relative angle")
                params.setdefault("max_omega_rad_s", angular_floor)
                if any(type(value) not in {int, float} or not math.isfinite(value) for value in params.values()):
                    raise ValueError("live turn parameters must be finite numbers")
                if abs(params["angle_deg"]) <= 0 or abs(params["angle_deg"]) > 90 or not angular_floor <= params["max_omega_rad_s"] <= angular_cap:
                    raise ValueError("live turn exceeds bounded learning envelope")
                angle += abs(params["angle_deg"])
            else:
                raise ValueError("live learning plan supports move_relative and turn_by only")
            reviewed.append({"action": action, "parameters": params, "completion": "mission", "max_retries": 0})
        if distance > .5 or angle > 90:
            raise ValueError("combined live experiment exceeds 0.5 m or 90 degrees")
        # These are proposed limits. Arbitrary Python source is not claimed to
        # obey them; the operator approves the concrete canonical steps.
        readiness = "UNVERIFIED"
        if interface is not None:
            catalog = interface.capabilities().get("capabilities", {})
            readiness = "READY" if all(isinstance(catalog.get(row["action"]), Mapping)
                and catalog[row["action"]].get("supported") is True
                and catalog[row["action"]].get("available") is True
                and catalog[row["action"]].get("ready") is True for row in reviewed) else "NOT_READY"
        max_speed = max((row["parameters"].get("max_v_mps", 0) for row in reviewed), default=0)
        execution_steps = [{"action": row["action"], "parameters": {**row["parameters"],
                           "capture_mode": "full", "capture_hz": 10, "finite_timeout_s": duration}}
                           for row in reviewed]
        # Brain plan duration_s is a behavior-duration constraint, not a finite
        # motion timeout. Keep that valid plan separate from the reviewable SDK
        # experiment that enforces the requested wall-time and canonical STOP.
        trial_source = ("import asyncio\n\nasync def run(robot):\n    try:\n"
            f"        async with asyncio.timeout({duration!r}):\n"
            f"            for step in {execution_steps!r}:\n"
            "                completed = await robot.call(step['action'], **step['parameters'])\n"
            "                if not isinstance(completed, dict) or completed.get('status') != 'COMPLETED':\n"
            "                    reason = str(completed.get('reason') or completed.get('status'))[:384] if isinstance(completed, dict) else 'INVALID_ACTION_RESULT'\n"
            "                    raise RuntimeError('LIVE_LEARNING_ACTION_INTERRUPTED:' + reason)\n"
            "        return {'status': 'COMPLETED', 'goal_satisfaction': 'UNASSESSED'}\n"
            "    finally:\n        await robot.stop()\n")
        result = {"schema": "R2B4_LIVE_LEARNING_PLAN_V1", "status": "APPROVAL_REQUIRED",
            "skill": loaded.name, "source_hash": loaded.source_hash, "parameters": bounded_feedback(parameters),
            "purpose": purpose, "runtime_readiness": readiness, "execution_started": False,
            "original_skill_execution": False,
            "approval_question": f"Engedélyezed az élő canonical mérőpróbát (hivatkozott skill: {loaded.name}): "
                f"összesen {distance:.2f} m relatív cél-eltolás, "
                f"{angle:.0f}° kért fordulás, {duration:g} s, legfeljebb {max_speed:.2f} m/s lineáris és "
                f"{max(row['parameters']['max_omega_rad_s'] for row in reviewed):.2f} rad/s szögsebesség? {purpose}",
            "proposal": {"steps": reviewed, "constraints": {}},
            "bounded_execution": {"max_duration_s": duration, "source": trial_source,
                "source_sha256": hashlib.sha256(trial_source.encode()).hexdigest(),
                "required": True, "scope": "Execute the reviewed finite SDK trial after user approval; the Brain plan alone has no total wall-time bound."},
            "calibrated_envelope": {"snapshot_id": resolved.snapshot_id, "wheel_minimum_mps": wheels.minimum_mps,
                "target_linear_floor_mps": linear_floor, "target_center_spin_rad_s": angular_floor,
                "maximum_linear_mps": linear_cap, "maximum_angular_rad_s": angular_cap},
            "capture": {"mode": "full", "tick_sample_hz": 10, "learning_events": True,
                "exact_replay": "Use a separate full 50 Hz capture when exact native replay is required."},
            "stop_policy": "Canonical STOP on completion, timeout, cancellation, safety/health/timing fault or process death; no automatic retry.",
            "measurement": "Compile the original MCAP explicitly; run motion_performance; compare paired environments and preserved run/source identity.",
            "scope": "Reviewable canonical target experiment. Target displacement is not travelled path length or guaranteed physical displacement; steering, detours and stopping affect actual motion. The referenced original skill is not executed."}
        emit("learning_live_approval_requested", {key: result[key] for key in ("skill", "source_hash", "status", "approval_question", "proposal")})
        return result

    def live_feedback(arguments):
        from .agent_evidence_tools import collect_evidence_feedback
        from .skill_learning import SkillLearningLoop
        from v3.mcap_reader import McapReader
        from v3.mcap_capture import RUNTIME_TOPIC
        from tools.mcap_evidence.reader import EvidenceBundle
        args = dict(arguments)
        if set(args) - {"evidence", "capture", "run_id", "source_hash", "replay_result", "conditions"}:
            raise ValueError("unknown learning.live_feedback arguments")
        if not isinstance(args.get("run_id"), str) or not args["run_id"] or not isinstance(args.get("source_hash"), str):
            raise ValueError("live feedback requires expected run_id and source_hash")
        feedback = collect_evidence_feedback(root, evidence=args["evidence"], capture=args["capture"],
            analyzers=("evidence_health", "safety", "motion_performance"), replay_result=args.get("replay_result"),
            binding={key: args[key] for key in ("run_id", "source_hash")})
        capture = Path(args["capture"])
        capture = capture if capture.is_absolute() else root / capture
        runtimes = []
        for message in McapReader(capture).iter_messages(topics=(RUNTIME_TOPIC,)):
            runtimes.append(message.json())
            if len(runtimes) > 1:
                break
        metadata = runtimes[0].get("metadata", {}) if len(runtimes) == 1 and isinstance(runtimes[0], Mapping) else {}
        live = isinstance(metadata, Mapping) and metadata.get("runtime") == "v3_process_runtime"
        live = live and metadata.get("command_gateway") == "AtomicResidentCommandGateway" and metadata.get("source") != "simulation"
        motion = next((item for item in feedback["analyzers"] if item["analyzer_id"] == "motion_performance"), {})
        metrics = motion.get("metrics", {})
        performance = metrics.get("performance", {})
        tracking = metrics.get("tracking", {})
        qualified = (type(performance.get("allowed_motion_s")) in {int, float} and performance["allowed_motion_s"] > 0
                     and type(tracking.get("active_command_pairs")) is int and tracking["active_command_pairs"] > 0)
        blocker = SkillLearningLoop._evidence_blocker(feedback)
        # The owner scopes the motion analyzer to the verified worker interval.
        # Startup/shutdown STOP evidence remains useful in the original capture.
        terminals = feedback.get("trial_binding", {}).get("terminal_events", [])
        execution_failure = ("LIVE_TRIAL_DID_NOT_SUCCEED" if not terminals or any(
            row.get("state") != "SUCCEEDED" or row.get("reported_success") is False
            or str(row.get("reported_status") or "").upper() in {"FAILED", "PARTIAL", "INTERRUPTED", "CANCELLED"}
            for row in terminals) else None)
        interval = {(row.get("started_ns"), row.get("finished_ns")) for row in terminals}
        timing_bound = len(interval) == 1
        start, finish = next(iter(interval)) if timing_bound else (None, None)
        timing_bound = timing_bound and type(start) is int and type(finish) is int and 0 <= start < finish
        epoch = metadata.get("monotonic_boot_id") if isinstance(metadata, Mapping) else None
        if not epoch or any(row.get("evidence_ref", {}).get("clock_epoch") != epoch for row in terminals):
            timing_bound = False
        scope = feedback.get("measurement_scope", {})
        if (not isinstance(scope, Mapping) or scope.get("status") != "BOUND_TO_SKILL_RUN"
                or scope.get("started_ns") != start or scope.get("finished_ns") != finish
                or scope.get("clock_epoch") != epoch
                or any(scope.get(key) != args[key] for key in ("run_id", "source_hash"))):
            timing_bound = False
        path = Path(args["evidence"])
        path = path if path.is_absolute() else root / path
        bundle = EvidenceBundle.open(path, verification="manifest")
        observed_motion = False
        native_failure = None
        scoped_ticks = scanned_ticks = 0
        for row in bundle.iter_view("ticks", limit=100_001):
            scanned_ticks += 1
            payload = row.payload
            context = payload.get("inputs", {}).get("context", {}) if isinstance(payload, Mapping) else {}
            timestamp = context.get("monotonic_ns")
            if scanned_ticks > 100_000 or not timing_bound or type(timestamp) is not int:
                timing_bound = False
                break
            if not start <= timestamp <= finish:
                continue
            scoped_ticks += 1
            layers = payload.get("expected", {}).get("layers", {})
            final, estimate = layers.get("L12", {}), layers.get("L3", {})
            if native_failure is None and final.get("safety_decision") == "FAULT":
                native_failure = "LIVE_TRIAL_NATIVE_FAULT:" + str(final.get("reason") or "FAULT")[:384]
            elif native_failure is None and final.get("safety_decision") == "STOP" and final.get("reason") != "NOT_ACTIVE":
                native_failure = "LIVE_TRIAL_SAFETY_STOP:" + str(final.get("reason") or "UNKNOWN")[:384]
            actuation = any(type(final.get(key)) in {int, float} and abs(final[key]) > 1e-9
                            for key in ("left_output", "right_output"))
            estimated_motion = any(type(estimate.get(key)) in {int, float} and abs(estimate[key]) > 1e-6
                                   for key in ("local_v_mps", "local_omega_rad_s"))
            observed_motion |= actuation and estimated_motion
        qualified = qualified and observed_motion and scoped_ticks > 0
        execution_failure = native_failure or execution_failure
        if not blocker and not timing_bound:
            blocker = "DIAG_SCOPE_NOT_BOUND_TO_SKILL_RUN"
        if timing_bound and bundle.view_count("events") > 4096:
            blocker = blocker or "LIVE_EVENT_SCAN_INCOMPLETE"
        elif timing_bound:
            from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC
            for row in bundle.iter_view("events", limit=4096):
                envelope = row.payload
                if not isinstance(envelope, Mapping) or envelope.get("source_topic") != PUBLIC_RUNTIME_EVENT_TOPIC:
                    continue
                event = envelope.get("payload", {})
                if not isinstance(event, Mapping):
                    continue
                value = event.get("value", {}) if isinstance(event, Mapping) else {}
                timestamp = value.get("measurement_time_ns") if isinstance(value, Mapping) else None
                if (execution_failure is None and isinstance(value, Mapping) and event.get("kind") == "skill"
                        and all(value.get(key) == args[key] for key in ("run_id", "source_hash"))
                        and (value.get("cleanup_status") == "FAILED" or value.get("cleanup_error"))):
                    execution_failure = "LIVE_TRIAL_CLEANUP_FAILED"
                if (execution_failure is None and event.get("kind") == "interface" and event.get("clock_epoch") == epoch
                        and type(timestamp) is int and start <= timestamp <= finish
                        and isinstance(value.get("action"), str) and value["action"].startswith("v3.command.")
                        and (value.get("kind") == "ACTION_ERROR" or value.get("status") == "INTERRUPTED")):
                    execution_failure = "LIVE_TRIAL_ACTION_INTERRUPTED:" + str(value.get("reason") or value.get("error_type") or "ACTION_ERROR")[:384]
        conditions = args.get("conditions", [])
        if not isinstance(conditions, list) or len(conditions) > 8:
            raise ValueError("live conditions must be 0..8 metric predicates")
        checks = []
        for condition in conditions:
            if not isinstance(condition, Mapping) or set(condition) != {"metric", "maximum"}:
                raise ValueError("live condition requires metric group/key and maximum")
            metric, maximum = condition["metric"], condition["maximum"]
            if not isinstance(metric, str) or not re.fullmatch(r"[a-z_]+/[a-z_]+", metric):
                raise ValueError("metric must be a scalar DIAG group/key")
            if type(maximum) not in {int, float} or not math.isfinite(maximum) or maximum < 0:
                raise ValueError("condition maximum must be a finite nonnegative number")
            group, key = metric.split("/")
            actual = metrics.get(group, {}).get(key)
            valid = type(actual) in {int, float} and math.isfinite(actual)
            checks.append({"name": metric, "passed": valid and actual <= maximum, "actual": actual, "maximum": maximum})
        status = ("EVIDENCE_BLOCKED" if blocker or not live else "FAILED" if execution_failure else
                  "EVIDENCE_BLOCKED" if not qualified else "PASSED" if checks and all(row["passed"] for row in checks) else "FAILED" if checks else "MEASURED")
        reason = blocker
        if reason is None:
            reason = ("PRODUCTION_LIVE_CAPTURE_REQUIRED" if not live else execution_failure or
                      ("QUALIFIED_LIVE_MOTION_REQUIRED" if not qualified else None))
        result = {"schema": "R2B4_LIVE_LEARNING_FEEDBACK_V1", "status": status,
            "reason": reason,
            "run_id": args["run_id"], "source_hash": args["source_hash"],
            "measurement_scope": scope,
            "goal_satisfaction": "ASSESSED_BY_LIVE_METRICS" if status == "PASSED" else "UNASSESSED",
            "checks": checks, "motion_performance": motion, "evidence": feedback,
            "scope": "Recorded production motion and estimator/control measurements; independent physical ground truth and general improvement require separate evidence."}
        emit("learning_live_measurement", {"status": status, "reason": result["reason"], "run_id": args["run_id"],
            "source_hash": args["source_hash"], "capture": str(capture), "checks": checks})
        directory = Path(tempfile.mkdtemp(prefix="r2b4-live-learning-", dir="/tmp"))
        path = directory / "measured.json"
        result["feedback_path"] = str(path)
        with path.open("x") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        compact = {key: result[key] for key in ("schema", "status", "reason", "run_id", "source_hash",
                    "measurement_scope", "goal_satisfaction", "checks", "scope", "feedback_path")}
        compact["feedback_sha256"] = digest
        compact["motion_performance"] = {
            "analyzer_id": motion.get("analyzer_id"), "admission": motion.get("admission"),
            "summary": bounded_feedback(motion.get("summary"), max_bytes=2048, pointer="/motion_performance/summary"),
            "metrics": {group: bounded_feedback(value, max_bytes=1024, pointer="/motion_performance/metrics/" + group)
                        for group, value in metrics.items()}}
        compact["evidence"] = {"capture": feedback.get("capture"),
            "integrity": {key: feedback.get("evidence", {}).get(key) for key in (
                "verification_status", "source_integrity", "source_sha256", "verification_source_checked")},
            "trial_binding": bounded_feedback(feedback.get("trial_binding"), max_bytes=2048, pointer="/evidence/trial_binding"),
            "replay": feedback.get("replay")}
        if len(json.dumps(compact, ensure_ascii=False, allow_nan=False).encode()) > 12_000:
            compact = {key: result[key] for key in ("schema", "status", "reason", "run_id", "source_hash", "feedback_path")}
            compact.update(feedback_sha256=digest, details=bounded_feedback(result, max_bytes=10_000))
        return compact

    return ((AgentToolSpec(
        "learning.trial",
        "Run one explicit motor-free scenario through the shared SDK/host/Python worker, then compile its fresh capture, replay and measure DIAG. Independent scenario checks assess only simulated behavior.",
        "READ", {"name": "required saved skill name", "parameters": "optional object",
                  "scenario": "optional independent scenario or {kind:scenario_suite,scenarios:[1..8]}; absent means INCOMPLETE",
                  "timeout_s": "optional seconds in (0,20], default10. Offline compiler/replay complete before the next budget check."}), trial),
        (AgentToolSpec("learning.read",
            "Read full trial measurement details on demand by JSON pointer, with bounded list/object/string paging. Use the measured artifact path and SHA256 from learning.trial or prior iteration; omitted previews are not full values.",
            "READ", {"feedback_path": "required explicit measured.json path", "sha256": "optional expected artifact SHA256",
                     "pointer": "optional JSON pointer, default root", "offset": "optional item/key/character offset, default 0",
                     "limit": "optional items/keys 1..32 (default 8); string characters 1..4096 (default 2048)"}), read),
        (AgentToolSpec("learning.compare", "Compare paired measured physics trials or suites across environments. Check regressions, bounded evidence and metric deltas before deciding whether a change is worthwhile.", "READ",
            {"baseline": "required explicit baseline measured artifact", "variant": "required explicit changed measured artifact",
             "baseline_sha256": "optional expected SHA256", "variant_sha256": "optional expected SHA256"}), compare),
        (AgentToolSpec("learning.live_plan", "Prepare a bounded live experiment and explicit user approval question. Return APPROVAL_REQUIRED; ask the user before any motion. This tool never executes the saved skill or proposed steps.", "READ",
            {"name": "required saved skill", "parameters": "optional skill parameters", "steps": "required 1..4 canonical move_relative/turn_by steps",
             "max_duration_s": "optional total duration 1..30, default 20", "purpose": "optional measured learning question"}), live_plan),
        (AgentToolSpec("learning.live_feedback", "Measure an explicit original production capture through EVI and motion_performance with expected skill run/source identity. Full details remain in feedback_path/SHA256 for learning.read. Simulation cannot satisfy live validation. Estimator evidence does not establish independent physical accuracy.", "READ",
            {"evidence": "required compiled EVI path", "capture": "required original MCAP path", "run_id": "required expected skill run identity",
             "source_hash": "required expected skill source hash", "replay_result": "optional canonical full-rate replay JSON",
             "conditions": "optional 0..8 independent {metric:group/key,maximum:number} predicates"}), live_feedback))


__all__ = ["build_learning_tools", "measure_skill_trial", "measure_skill_suite", "scenario_suite", "compact_trial_feedback"]
