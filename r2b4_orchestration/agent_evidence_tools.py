"""EVI and DIAG adapters for AgentCore; no duplicated evidence logic."""
from __future__ import annotations

import json
import hashlib
from dataclasses import replace
from pathlib import Path
from collections.abc import Mapping

from .agent_contracts import AgentToolSpec


def _strict_args(value: Mapping[str, object], allowed: set[str]) -> dict[str, object]:
    args = dict(value)
    unknown = sorted(set(args) - allowed)
    if unknown:
        raise ValueError("unknown arguments: " + ", ".join(unknown))
    return args


def _bundle(root: Path, value: object):
    from tools.diag.context import resolve_evidence
    from tools.mcap_evidence.reader import EvidenceBundle

    selected = "latest" if value is None else value
    if not isinstance(selected, (str, Path)):
        raise ValueError("evidence must be a path or 'latest'")
    path = resolve_evidence(root, selected)
    return EvidenceBundle.open(path, verification="manifest")


def _summary(root: Path, value: Mapping[str, object]) -> object:
    args = _strict_args(value, {"evidence"})
    bundle = _bundle(root, args.get("evidence"))
    start, end = bundle.message_time_bounds()
    return {
        "path": str(bundle.root),
        "verification": bundle.verification,
        "manifest": {
            "compiler_status": bundle.manifest.get("compiler_status"),
            "source_integrity": bundle.manifest.get("source_integrity"),
            "source": bundle.manifest.get("source"),
            "compiler": bundle.manifest.get("compiler"),
        },
        "message_count": bundle.message_count(),
        "message_start_time_ns": start,
        "message_end_time_ns": end,
        "topic_counts": bundle.topic_counts(),
        "view_counts": bundle.view_counts(),
    }


def _query(root: Path, value: Mapping[str, object]) -> object:
    from tools.diag.context import resolve_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.reader import EvidenceBundle

    allowed = {
        "evidence", "topic", "message_id", "tick_id", "layer", "sensor", "field",
        "channel", "sequence", "source_offset", "start_ns", "end_ns", "limit",
    }
    args = _strict_args(value, allowed)
    evidence = args.pop("evidence", "latest")
    path = resolve_evidence(root, evidence)
    # Verify the sealed manifest before indexed reading. Full source verification
    # remains available through DIAG when that stronger scope is needed.
    EvidenceBundle.open(path, verification="manifest")
    limit = args.get("limit", 20)
    if not isinstance(limit, int) or isinstance(limit, bool) or not 1 <= limit <= 100:
        raise ValueError("limit must be an integer within [1, 100]")
    args["limit"] = limit
    rows = list(query(path, **args))
    return {"evidence": str(path), "count": len(rows), "rows": rows}


def _source_message(root: Path, value: Mapping[str, object]) -> object:
    args = _strict_args(value, {"evidence", "message_id"})
    message_id = args.get("message_id")
    if not isinstance(message_id, str) or not message_id:
        raise ValueError("message_id is required")
    bundle = _bundle(root, args.get("evidence"))
    return bundle.source_message(message_id)


def _diag_list(_root: Path, value: Mapping[str, object]) -> object:
    _strict_args(value, set())
    from tools.diag.registry import build_default_registry
    return {"analyzers": build_default_registry().describe()}


def _diag_run(root: Path, value: Mapping[str, object]) -> object:
    args = _strict_args(value, {"analyzer", "evidence"})
    analyzer = args.get("analyzer")
    if not isinstance(analyzer, str) or not analyzer:
        raise ValueError("analyzer is required")
    from tools.diag.context import open_context
    from tools.diag.registry import build_default_registry
    from v3.host_cli import hardware_guard

    registry = build_default_registry()
    if analyzer not in registry.ids():
        raise ValueError(f"unknown DIAG analyzer: {analyzer}")
    # Preserve the canonical robot-host policy: heavy DIAG is offline and may
    # not race a resident V3 runtime.
    with hardware_guard(root):
        context = open_context(root, args.get("evidence", "latest"))
        return registry.run(analyzer, context).as_dict()


def _explicit_path(root: Path, value: object, name: str) -> Path:
    if not isinstance(value, (str, Path)) or not str(value).strip() or str(value) == "latest":
        raise ValueError(f"{name} requires an explicit path")
    path = Path(value).expanduser()
    return path if path.is_absolute() else root / path


def _new_output(root: Path, value: object) -> Path:
    path = _explicit_path(root, value, "output")
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"output already exists: {path}")
    return path


def _compile(root: Path, value: Mapping[str, object]) -> object:
    args = _strict_args(value, {"capture", "output", "workers"})
    capture = _explicit_path(root, args.get("capture"), "capture")
    output = _new_output(root, args.get("output"))
    workers = args.get("workers", 1)
    if type(workers) is not int or not 1 <= workers <= 4:
        raise ValueError("workers must be an integer within [1, 4]")
    from tools.mcap_evidence.compiler import compile_evidence
    from v3.host_cli import hardware_guard

    with hardware_guard(root):
        return compile_evidence(capture, output, workers=workers, overwrite=False,
                                command=["agent", "evi.compile", str(capture), str(output)])


def _replay_summary(result: Mapping[str, object]) -> dict[str, object]:
    capture = result.get("capture")
    determinism = result.get("determinism")
    divergence = result.get("first_divergence")
    if isinstance(divergence, Mapping):
        divergence = {key: item for key, item in divergence.items() if key != "evidence"}
        for key in ("expected", "actual"):
            if key in divergence and len(json.dumps(divergence[key], ensure_ascii=False)) > 2_000:
                divergence[key] = {"omitted": "large value retained in canonical replay result"}
    return {
        "status": result.get("status"),
        "capture": dict(capture) if isinstance(capture, Mapping) else {},
        "scope": result.get("scope"),
        "repeated_trace_match": determinism.get("repeated_trace_match") if isinstance(determinism, Mapping) else None,
        "first_divergence": divergence,
        "result_sha256": result.get("result_sha256"),
    }


def _replay(root: Path, value: Mapping[str, object]) -> object:
    args = _strict_args(value, {"capture", "output", "start_tick_id", "end_tick_id"})
    capture = _explicit_path(root, args.pop("capture", None), "capture")
    output = _new_output(root, args.pop("output", None))
    from v3.host_cli import hardware_guard
    from v3.replay import ReplaySelection, replay_capture, write_replay_result

    selection = ReplaySelection(**args)
    with hardware_guard(root):
        result = replay_capture(capture, selection=selection, project_root=root)
        # The complete canonical result stays in an explicitly selected artifact;
        # the agent receives only source binding, scope and the first divergence.
        if output.exists() or output.is_symlink():
            raise FileExistsError(f"output already exists: {output}")
        write_replay_result(result, output)
    return {**_replay_summary(result), "output": str(output.resolve())}


def _capture_facts(path: Path, expected_sha256: str | None,
                   expected_size: int | None) -> dict[str, object]:
    from v3.mcap_reader import McapReader, McapReadError

    if path.is_symlink() or not path.is_file():
        raise ValueError("capture must be a regular, non-symlink MCAP file")
    if path.stat().st_size != expected_size:
        raise ValueError("capture size does not match the sealed evidence snapshot")
    reader = McapReader(path)
    digest = reader.sha256()
    if not expected_sha256 or digest != expected_sha256:
        raise ValueError("capture hash does not match the sealed evidence snapshot")
    try:
        structure = reader.inspect(verify_chunks=True)
        if not structure.valid:
            raise McapReadError("MCAP structural/deep CRC preflight failed")
        final = reader.capture_integrity()
        integrity = final["integrity"]
        return {
            "path": str(path.resolve()), "sha256": digest, "source_checked": True,
            "status": "VERIFIED", "production_status": final.get("status"),
            "complete": integrity.get("complete"),
            "sample_complete": integrity.get("sample_complete"),
            "replay_complete": integrity.get("replay_complete"),
            "raw_evidence_requested": integrity.get("raw_evidence_requested"),
            "raw_evidence_complete": integrity.get("raw_evidence_complete"),
            "tick_sample_hz": integrity.get("tick_sample_hz"),
            "message_stream_sha256": integrity.get("message_stream_sha256"),
        }
    except McapReadError as exc:
        # A sealed salvage bundle is useful diagnostic evidence even when the
        # source fails its separate canonical capture-completeness gate.
        return {"path": str(path.resolve()), "sha256": digest, "source_checked": True,
                "status": "EVIDENCE_BLOCKED", "complete": False, "error": str(exc)}


def _feedback_replay(root: Path, value: object, source_sha256: str | None) -> dict[str, object]:
    if value is None:
        return {"status": "NOT_REQUESTED", "verified_match": False}
    from v3.replay import verify_replay_result

    path = _explicit_path(root, value, "replay_result")
    verification = verify_replay_result(path)
    result = json.loads(path.read_text(encoding="utf-8"))
    summary = _replay_summary(result)
    capture = summary["capture"]
    source_matches = bool(source_sha256 and capture.get("sha256") == source_sha256)
    return {
        **summary, "result_path": str(path.resolve()),
        "checksum_ok": verification["checksum_ok"],
        "source_matches_evidence": source_matches,
        "verified_match": verification["status"] == "PASS" and source_matches,
    }


def _binding_expectation(value: object) -> dict[str, object] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("binding must be an object")
    expected = _strict_args(value, {"run_id", "source_hash", "clock_epoch", "started_ns", "finished_ns"})
    for name in ("run_id", "source_hash"):
        if not isinstance(expected.get(name), str) or not 1 <= len(expected[name]) <= 256:
            raise ValueError(f"binding requires bounded {name}")
    for name in ("started_ns", "finished_ns"):
        if name in expected and (type(expected[name]) is not int or expected[name] < 0):
            raise ValueError(f"binding {name} must be a non-negative integer")
    if "clock_epoch" in expected and (
            not isinstance(expected["clock_epoch"], str) or not 1 <= len(expected["clock_epoch"]) <= 256):
        raise ValueError("binding clock_epoch must be bounded text")
    return expected


def _trial_binding(bundle, expected: dict[str, object] | None,
                   capture_facts: Mapping[str, object], limit: int) -> dict[str, object]:
    if expected is None:
        return {"status": "NOT_REQUESTED"}
    from tools.diag.profiling import evidence_ref
    from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC

    reasons = []
    if capture_facts.get("status") != "VERIFIED" or capture_facts.get("source_checked") is not True:
        reasons.append("SOURCE_CAPTURE_NOT_VERIFIED")
    if any(name in expected for name in ("started_ns", "finished_ns")) and "clock_epoch" not in expected:
        reasons.append("TIME_BINDING_REQUIRES_CLOCK_EPOCH")
    terminal = []
    assessments = []
    terminal_claims = set()
    assessment_claims = set()
    matching_events = terminal_count = assessment_count = scanned = 0
    # A fresh trial is small. An older bundle with an unbounded event history
    # cannot acquire a positive binding from a silently truncated search.
    for row in bundle.iter_view("events", limit=4097):
        scanned += 1
        if scanned > 4096:
            reasons.append("EVENT_SCAN_LIMIT")
            break
        payload = row.payload
        if not isinstance(payload, Mapping):
            continue
        source_topic = payload.get("source_topic")
        envelope = payload.get("payload")
        if not isinstance(envelope, Mapping):
            continue
        if source_topic == PUBLIC_RUNTIME_EVENT_TOPIC:
            kind, value, pointer = envelope.get("kind"), envelope.get("value"), "/payload/value"
        elif source_topic in {"r2b4.skill", "r2b4.simulation_trial_assessment"}:
            kind, value, pointer = source_topic.removeprefix("r2b4."), envelope, "/payload"
        else:
            continue
        if kind not in {"skill", "simulation_trial_assessment"} or not isinstance(value, Mapping):
            continue
        if value.get("run_id") != expected["run_id"]:
            continue
        matching_events += 1
        if value.get("source_hash") != expected["source_hash"]:
            reasons.append("SKILL_SOURCE_HASH_MISMATCH")
            continue
        if "clock_epoch" in expected and envelope.get("clock_epoch") != expected["clock_epoch"]:
            reasons.append("CLOCK_EPOCH_MISMATCH")
            continue
        ref = {
            **evidence_ref(row, pointer).as_dict(), "source_topic": source_topic,
            "producer_id": envelope.get("producer_id"), "event_sequence": envelope.get("event_sequence"),
            "clock_epoch": envelope.get("clock_epoch"),
            "publication_time_ns": envelope.get("publication_time_ns", payload.get("monotonic_ns")),
        }
        if kind == "skill":
            if value.get("state") not in {"SUCCEEDED", "FAILED", "CANCELLED"}:
                continue
            if any(name in expected and (
                    type(value.get(name)) is not int or value[name] != expected[name])
                    for name in ("started_ns", "finished_ns")):
                reasons.append("SKILL_TIME_MISMATCH")
                continue
            terminal_count += 1
            terminal_claims.add(json.dumps({key: value.get(key) for key in (
                "state", "started_ns", "finished_ns", "result_sha256")}, sort_keys=True, allow_nan=False))
            if len(terminal) < limit:
                terminal.append({"run_id": value["run_id"], "source_hash": value["source_hash"],
                                 "state": value["state"], "started_ns": value.get("started_ns"),
                                 "finished_ns": value.get("finished_ns"), "evidence_ref": ref,
                                 **{key: value[key] for key in ("result_sha256", "result_size_bytes", "result_inline",
                                     "result_scope", "reported_status", "reported_reason", "reported_success",
                                     "goal_satisfaction") if key in value}})
        else:
            assessment_count += 1
            assessment = value.get("assessment")
            digest = hashlib.sha256(json.dumps(assessment, sort_keys=True,
                separators=(",", ":"), allow_nan=False).encode()).hexdigest()
            assessment_claims.add(digest)
            if len(assessments) < limit:
                assessments.append({"goal_id": value.get("goal_id"),
                                    "status": assessment.get("status") if isinstance(assessment, Mapping) else None,
                                    "assessment_sha256": digest,
                                    "evidence_ref": ref})
    if not terminal_count:
        reasons.append("TERMINAL_SKILL_EVENT_NOT_FOUND")
    if len(terminal_claims) > 1:
        reasons.append("TERMINAL_SKILL_EVENT_CONFLICT")
    if len(assessment_claims) > 1:
        reasons.append("TRIAL_ASSESSMENT_EVENT_CONFLICT")
    return {
        "status": "EVIDENCE_BLOCKED" if reasons else "VERIFIED", "expected": expected,
        "reasons": list(dict.fromkeys(reasons)), "matching_event_count": matching_events,
        "terminal_event_count": terminal_count, "assessment_event_count": assessment_count,
        "terminal_events": terminal, "assessment_events": assessments,
        "refs_truncated": terminal_count > limit or assessment_count > limit,
    }


def _measurement_slice(analyzer: str, metrics: Mapping[str, object]) -> list[dict[str, object]]:
    """Select eight concrete control measurements from existing DIAG profiles."""
    profiles = metrics.get("profiles", {})
    selections = (
        ("layers/L12", "/left_output"), ("layers/L12", "/right_output"),
        ("layers/L12", "/safety_decision"), ("layers/L12", "/reason"),
        ("layers/L9", "/requested_v_mps"), ("layers/L9", "/allowed_v_mps"),
        ("layers/L9", "/requested_omega_rad_s"), ("layers/L9", "/allowed_omega_rad_s"),
    )
    result = []
    for view, field in selections:
        profile = profiles.get(view) if isinstance(profiles, Mapping) else None
        fields = profile.get("fields", {}) if isinstance(profile, Mapping) else {}
        statistics = fields.get(field) if isinstance(fields, Mapping) else None
        if not isinstance(statistics, Mapping):
            continue
        result.append({
            "analyzer_id": analyzer, "view": view, "field_path": field,
            "row_count": profile["row_count"],
            "first_log_time_ns": profile["first_log_time_ns"],
            "last_log_time_ns": profile["last_log_time_ns"],
            "statistics": dict(statistics),
        })
    return result


def _motion_measurement_scope(bundle, binding: Mapping[str, object]) -> dict[str, object]:
    """Qualify a live run interval without reinterpreting a synthetic clock."""
    if binding.get("status") == "NOT_REQUESTED":
        return {"status": "CAPTURE_WIDE"}
    if binding.get("status") != "VERIFIED":
        return {"status": "RUN_BINDING_UNVERIFIED"}
    runtimes = list(bundle.iter_view("runtime", limit=2)) if bundle.has_view("runtime") else []
    if len(runtimes) != 1 or not isinstance(runtimes[0].payload, Mapping):
        return {"status": "RUNTIME_CLOCK_UNVERIFIED"}
    runtime = runtimes[0].payload
    metadata = runtime.get("metadata", {})
    if not isinstance(metadata, Mapping):
        return {"status": "RUNTIME_CLOCK_UNVERIFIED"}
    if metadata.get("source") == "simulation" or metadata.get("physical_hardware") is False:
        return {"status": "SIMULATION_CLOCK", "scope": "Full synthetic trial; worker wall time does not bound virtual physics time."}
    if (metadata.get("runtime") != "v3_process_runtime"
            or metadata.get("command_gateway") != "AtomicResidentCommandGateway"):
        return {"status": "RUNTIME_CLOCK_UNVERIFIED"}
    epoch = metadata.get("monotonic_boot_id")
    terminal = binding.get("terminal_events", [])
    intervals = {(row.get("started_ns"), row.get("finished_ns")) for row in terminal}
    if (not isinstance(epoch, str) or not epoch or not terminal
            or runtime.get("clock_epoch") != "linux_monotonic_boot"
            or any(row.get("evidence_ref", {}).get("clock_epoch") != epoch for row in terminal)):
        return {"status": "CLOCK_EPOCH_UNVERIFIED"}
    if len(intervals) != 1:
        return {"status": "RUN_INTERVAL_UNVERIFIED"}
    start, finish = next(iter(intervals))
    if (type(start) is not int or type(finish) is not int or not 0 <= start < finish
            or any(type(row.get("evidence_ref", {}).get("publication_time_ns")) is not int
                   or finish > row["evidence_ref"]["publication_time_ns"] for row in terminal)):
        return {"status": "RUN_INTERVAL_UNVERIFIED"}
    from tools.diag.profiling import evidence_ref
    return {"status": "BOUND_TO_SKILL_RUN", "started_ns": start, "finished_ns": finish,
            "clock_epoch": epoch, **{key: binding["expected"][key] for key in ("run_id", "source_hash")},
            "runtime_ref": evidence_ref(runtimes[0], "/metadata/monotonic_boot_id").as_dict(),
            "terminal_ref": terminal[0]["evidence_ref"]}


def read_evidence_feedback(project_root: Path, value: Mapping[str, object]) -> dict[str, object]:
    """Read explicit offline trial evidence, preserving independent proof scopes.

    Message time bounds describe the selected artifact; they are never a claim
    that an older capture is the current skill run. Optional binding verifies
    the expected run/source against a recorded terminal skill event.
    """
    root = Path(project_root).resolve()
    args = _strict_args(value, {"evidence", "capture", "analyzers", "replay_result", "limit", "binding"})
    evidence = _explicit_path(root, args.get("evidence"), "evidence")
    binding = _binding_expectation(args.get("binding"))
    analyzers = args.get("analyzers", ("evidence_health", "safety"))
    if (not isinstance(analyzers, (list, tuple)) or not 1 <= len(analyzers) <= 4
            or any(not isinstance(item, str) or not item for item in analyzers)
            or len(set(analyzers)) != len(analyzers)):
        raise ValueError("analyzers must contain 1..4 unique analyzer ids")
    limit = args.get("limit", 6)
    if type(limit) is not int or not 1 <= limit <= 10:
        raise ValueError("limit must be an integer within [1, 10]")
    from tools.diag.context import open_context
    from tools.diag.registry import build_default_registry
    from v3.host_cli import hardware_guard

    registry = build_default_registry()
    for analyzer in analyzers:
        if analyzer not in registry.ids():
            raise ValueError(f"unknown DIAG analyzer: {analyzer}")
    with hardware_guard(root):
        context = open_context(root, evidence, verification="full")
        capture_facts = {"status": "NOT_CHECKED", "source_checked": False}
        if args.get("capture") is not None:
            capture_facts = _capture_facts(
                _explicit_path(root, args["capture"], "capture"),
                context.facts.source_sha256, context.facts.source_snapshot_size)
            context = replace(context, facts=replace(context.facts, verification_source_checked=True))
        facts = context.facts.as_dict()
        facts["bundle_manifest_sha256"] = context.bundle.manifest["self_sha256"]
        trial_binding = _trial_binding(context.bundle, binding, capture_facts, limit)
        motion_scope = _motion_measurement_scope(context.bundle, trial_binding)
        results = []
        measurements = {}
        for analyzer in analyzers:
            selected_context = context
            if analyzer == "motion_performance" and motion_scope["status"] == "BOUND_TO_SKILL_RUN":
                selected_context = replace(context, measurement_time_interval_ns=(
                    motion_scope["started_ns"], motion_scope["finished_ns"]))
            result = registry.run(analyzer, selected_context).as_dict(include_evidence=False, include_producer=False)
            metrics = result["metrics"]
            for measurement in _measurement_slice(analyzer, metrics):
                measurements.setdefault((measurement["view"], measurement["field_path"]), measurement)
            # Per-field profiles remain available through diag.run; scalar and
            # aggregate measurements are sufficient for the learning prompt.
            result["metrics"] = {key: item for key, item in metrics.items() if key != "profiles"}
            result["omitted_metric_groups"] = ["profiles"] if "profiles" in metrics else []
            for name in ("observations", "episodes"):
                values = result[name]
                result[name + "_count"] = len(values)
                result[name + "_truncated"] = len(values) > limit
                result[name] = values[:limit]
            results.append(result)
        feedback = {
            "schema": "R2B4_AGENT_EVIDENCE_FEEDBACK_V1",
            "evidence": facts, "capture": capture_facts,
            "producer": context.producer.as_dict(), "analyzers": results,
            "measurements": list(measurements.values()),
            "trial_binding": trial_binding,
            "measurement_scope": motion_scope,
            "replay": _feedback_replay(root, args.get("replay_result"), context.facts.source_sha256),
            "root_cause_inferred": False,
        }
    if len(json.dumps(feedback, ensure_ascii=False, allow_nan=False)) > 32_000:
        raise ValueError("feedback exceeds 32000 characters; select fewer analyzers and a smaller limit")
    return feedback


def collect_evidence_feedback(project_root: Path, *, evidence: str | Path,
                              capture: str | Path | None = None,
                              analyzers: tuple[str, ...] = ("evidence_health", "safety"),
                              replay_result: str | Path | None = None,
                              limit: int = 6,
                              binding: Mapping[str, object] | None = None) -> dict[str, object]:
    """Convenience API for an explicit host-side learning evidence reader."""
    return read_evidence_feedback(project_root, {
        "evidence": evidence, "capture": capture, "analyzers": analyzers,
        "replay_result": replay_result, "limit": limit, "binding": binding,
    })


def build_evidence_tools(project_root: Path):
    root = Path(project_root).resolve()
    return (
        (
            AgentToolSpec(
                "evi.summary",
                "Read bounded integrity/coverage/topic/view facts from a sealed EVI bundle.",
                "READ",
                {"evidence": "optional bundle path or latest"},
            ),
            lambda args: _summary(root, args),
        ),
        (
            AgentToolSpec(
                "evi.query",
                "Run the canonical indexed EVI query API against a sealed bundle; use narrow filters and small limits.",
                "READ",
                {
                    "evidence": "optional bundle path or latest", "topic": "optional string",
                    "message_id": "optional string", "tick_id": "optional id", "layer": "optional layer id",
                    "sensor": "optional sensor id", "field": "optional field glob", "channel": "optional channel",
                    "sequence": "optional sequence", "source_offset": "optional source offset",
                    "start_ns": "optional timestamp", "end_ns": "optional timestamp", "limit": "optional integer 1..100",
                },
            ),
            lambda args: _query(root, args),
        ),
        (
            AgentToolSpec(
                "evi.source_message",
                "Read one original message preserved in a sealed EVI bundle by message_id.",
                "READ",
                {"evidence": "optional bundle path or latest", "message_id": "required string"},
            ),
            lambda args: _source_message(root, args),
        ),
        (
            AgentToolSpec("diag.list", "List the canonical evidence-native DIAG analyzers.", "READ", {}),
            lambda args: _diag_list(root, args),
        ),
        (
            AgentToolSpec(
                "diag.run",
                "Run one canonical DIAG analyzer on verified EVI evidence. Current host policy requires resident V3 to be stopped.",
                "READ",
                {"analyzer": "required analyzer id", "evidence": "optional bundle path or latest"},
            ),
            lambda args: _diag_run(root, args),
        ),
        (
            AgentToolSpec(
                "evi.compile", "Explicit offline MCAP compilation into a new sealed EVI output; resident V3 must be stopped.",
                "READ", {"capture": "required explicit MCAP path", "output": "required new bundle path",
                         "workers": "optional integer 1..4, default 1"}),
            lambda args: _compile(root, args),
        ),
        (
            AgentToolSpec(
                "replay.run", "Explicit offline canonical replay, retaining the complete result at a new output path; resident V3 must be stopped.",
                "READ", {"capture": "required explicit capture path", "output": "required new result JSON path",
                         "start_tick_id": "optional inclusive tick id", "end_tick_id": "optional inclusive tick id"}),
            lambda args: _replay(root, args),
        ),
        (
            AgentToolSpec(
                "evidence.feedback", "Read compact full-verified trial measurements with source lineage; capture integrity, replay and production outcome remain separate.",
                "READ", {"evidence": "required explicit sealed bundle path; latest is not trial evidence",
                         "capture": "optional explicit source MCAP to verify source binding and capture completeness",
                         "analyzers": "optional list of 1..4 unique DIAG analyzer ids",
                         "replay_result": "optional canonical replay JSON path, verified against the same source hash",
                         "binding": "optional expected run_id/source_hash; clock_epoch required when started_ns/finished_ns are checked",
                         "limit": "optional observation/episode limit 1..10, default 6"}),
            lambda args: read_evidence_feedback(root, args),
        ),
    )


__all__ = ["build_evidence_tools", "collect_evidence_feedback", "read_evidence_feedback"]
