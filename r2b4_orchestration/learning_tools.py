"""Explicit offline skill experiments using the existing host and evidence tools."""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import hashlib
import json
import re

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
    result = {key: trial.get(key) for key in ("schema", "capture", "feedback_path", "feedback_sha256",
                                             "physical_output", "artifact_directory")}
    runtime = trial.get("runtime", {})
    result["runtime"] = {key: runtime.get(key) for key in (
        "run_id", "source_hash", "state", "error", "pid", "returncode", "callback_error")}
    result["runtime"]["result"] = bounded_feedback(runtime.get("result"), pointer="/runtime/result")
    result["assessment"] = compact_assessment(trial.get("assessment"))
    evidence = trial.get("evidence", {})
    feedback = evidence.get("feedback", {})
    selected = {key: feedback.get(key) for key in ("schema", "root_cause_inferred")}
    for key in ("evidence", "capture", "replay", "trial_binding"):
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
            return compact_trial_feedback(measure_skill_trial(root, library, simulator, args["name"],
                args.get("parameters", {}), args.get("scenario"), timeout_s=timeout))

    def read(arguments: Mapping[str, object]):
        args = dict(arguments)
        if set(args) - {"feedback_path", "sha256", "pointer", "offset", "limit"}:
            raise ValueError("unknown learning.read arguments")
        path = Path(args["feedback_path"])
        if not path.is_absolute():
            path = root / path
        if path.is_symlink() or not path.is_file():
            raise ValueError("feedback must be an explicit regular measured artifact")
        with path.open("rb") as stream:
            raw = stream.read(8_388_609)
        if len(raw) > 8_388_608:
            raise ValueError("measured artifact exceeds 8 MiB")
        digest = hashlib.sha256(raw).hexdigest()
        if args.get("sha256", digest) != digest:
            raise ValueError("measured artifact SHA256 mismatch")
        value = json.loads(raw)
        if not isinstance(value, Mapping) or value.get("schema") != "R2B4_SKILL_TRIAL_V1":
            raise ValueError("expected R2B4_SKILL_TRIAL_V1 measured artifact")
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

    return ((AgentToolSpec(
        "learning.trial",
        "Run one explicit motor-free scenario through the shared SDK/host/Python worker, then compile its fresh capture, replay and measure DIAG. Independent scenario checks assess only simulated behavior.",
        "READ", {"name": "required saved skill name", "parameters": "optional object",
                  "scenario": "optional independent simulation scenario object; absent means INCOMPLETE",
                  "timeout_s": "optional seconds in (0,20], default 10"}), trial),
        (AgentToolSpec("learning.read",
            "Read full trial measurement details on demand by JSON pointer, with bounded list/object/string paging. Use the measured artifact path and SHA256 from learning.trial or prior iteration; omitted previews are not full values.",
            "READ", {"feedback_path": "required explicit measured.json path", "sha256": "optional expected artifact SHA256",
                     "pointer": "optional JSON pointer, default root", "offset": "optional item/key/character offset, default 0",
                     "limit": "optional items/keys 1..32 (default 8); string characters 1..4096 (default 2048)"}), read))


__all__ = ["build_learning_tools", "measure_skill_trial", "compact_trial_feedback"]
