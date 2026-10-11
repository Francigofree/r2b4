"""EVI and DIAG adapters for AgentCore; no duplicated evidence logic."""
from __future__ import annotations

import json
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


def read_evidence_feedback(project_root: Path, value: Mapping[str, object]) -> dict[str, object]:
    """Read explicit offline trial evidence, preserving independent proof scopes.

    Message time bounds describe the selected artifact; they are never a claim
    that an older capture is the current skill run. The caller must bind the
    selected capture to its trial's run/source identity.
    """
    root = Path(project_root).resolve()
    args = _strict_args(value, {"evidence", "capture", "analyzers", "replay_result", "limit"})
    evidence = _explicit_path(root, args.get("evidence"), "evidence")
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
        results = []
        for analyzer in analyzers:
            result = registry.run(analyzer, context).as_dict(include_evidence=False, include_producer=False)
            metrics = result["metrics"]
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
                              limit: int = 6) -> dict[str, object]:
    """Convenience API for an explicit host-side learning evidence reader."""
    return read_evidence_feedback(project_root, {
        "evidence": evidence, "capture": capture, "analyzers": analyzers,
        "replay_result": replay_result, "limit": limit,
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
                         "limit": "optional observation/episode limit 1..10, default 6"}),
            lambda args: read_evidence_feedback(root, args),
        ),
    )


__all__ = ["build_evidence_tools", "collect_evidence_feedback", "read_evidence_feedback"]
