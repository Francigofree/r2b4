"""EVI and DIAG adapters for AgentCore; no duplicated evidence logic."""
from __future__ import annotations

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
    )


__all__ = ["build_evidence_tools"]
