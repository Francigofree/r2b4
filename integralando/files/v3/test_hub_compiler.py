"""Canonical fact-only Test Hub compiler for finished R2B4 MCAP captures.

Test Hub is an offline diagnostic compiler. It owns no robot command, mission,
safety, motor or runtime authority and emits no diagnosis, priority, severity,
finding or root-cause claim.
"""
from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from pathlib import Path

from .mcap_reader import (
    CHECKPOINT_TOPIC, EVENT_TOPIC, RAW_LIDAR_TOPIC, RUNTIME_TOPIC, TICK_TOPIC,
    McapReader,
)
from .mcap_replay_bridge import ReplayWindow, replay_mcap
from .test_hub_diagnostic_coverage import write_diagnostic_coverage
from .test_hub_facts import (
    analyze_ticks, extract_control_state, read_checkpoints, read_ticks, write_ndjson,
)

COMPILER_SCHEMA = "R2B4_TEST_HUB_COMPILER_V1"
EVIDENCE_INDEX_SCHEMA = "R2B4_TEST_HUB_EVIDENCE_INDEX_V3"
AGENT_EVIDENCE_SCHEMA = "R2B4_TEST_HUB_AGENT_EVIDENCE_V1"
FACT_ONLY_POLICY = "MCAP_FACTS_ONLY_NO_DIAGNOSIS_NO_CAUSAL_INFERENCE"


class TestHubCompilerError(RuntimeError):
    pass


def _write_json(path: Path, payload: Mapping[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    tmp.replace(path)
    return path


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _final_event(reader: McapReader) -> Mapping[str, object] | None:
    found: Mapping[str, object] | None = None
    for _message, payload in reader.iter_json_messages(topics=(EVENT_TOPIC,)):
        if isinstance(payload, Mapping) and payload.get("event_type") == "capture_finalized":
            found = payload
    return found


def _topic_counts(reader: McapReader) -> dict[str, int | None]:
    channel_counts = reader.statistics.get("channel_message_counts")
    result: dict[str, int | None] = {}
    for channel_id, channel in reader.channels.items():
        count = None
        if isinstance(channel_counts, Mapping):
            raw = channel_counts.get(channel_id)
            if raw is None:
                raw = channel_counts.get(str(channel_id))
            if isinstance(raw, int):
                count = raw
        result[channel.topic] = count
    return dict(sorted(result.items()))


def inspect_mcap(capture_path: str | Path, *, deep: bool = True) -> dict[str, object]:
    reader = McapReader(capture_path)
    structure = reader.inspect(verify_chunks=deep)
    return {
        "schema": COMPILER_SCHEMA,
        "policy": FACT_ONLY_POLICY,
        "capture_path": str(reader.path.resolve()),
        "capture_sha256": reader.sha256(),
        "structure": {
            "valid": structure.valid,
            "file_size": structure.file_size,
            "profile": structure.profile,
            "library": structure.library,
            "data_crc_ok": structure.data_crc_ok,
            "summary_crc_ok": structure.summary_crc_ok,
            "chunk_crc_ok": structure.chunk_crc_ok,
            "chunk_count": structure.chunk_count,
            "channel_count": structure.channel_count,
            "metadata_count": structure.metadata_count,
            "message_count": structure.message_count,
            "message_start_time": structure.message_start_time,
            "message_end_time": structure.message_end_time,
            "errors": list(structure.errors),
        },
        "topic_message_counts": _topic_counts(reader),
        "capture_metadata": reader.latest_metadata("r2b4.capture"),
        "final_metadata": reader.latest_metadata("r2b4.capture.final"),
        "final_event": _final_event(reader),
    }


def _runtime_payload(reader: McapReader) -> Mapping[str, object]:
    pair = reader.first_json(RUNTIME_TOPIC)
    return pair[1] if pair and isinstance(pair[1], Mapping) else {}


def _tick_sample_hz(reader: McapReader) -> int:
    metadata = reader.latest_metadata("r2b4.capture") or {}
    try:
        return int(metadata.get("tick_sample_hz", "50"))
    except (TypeError, ValueError):
        return 50


def _replay_facts(capture: Path, project_root: Path, mode: str,
                  tick_sample_hz: int) -> dict[str, object]:
    requested = mode.strip().lower()
    if requested not in {"off", "auto", "full"}:
        raise ValueError("replay mode must be off, auto or full")
    if requested == "off":
        return {"requested": requested, "applicable": tick_sample_hz == 50, "executed": False}
    if tick_sample_hz != 50:
        return {
            "requested": requested, "applicable": False, "executed": False,
            "reason": "SAMPLED_TICK_STREAM",
        }
    raw = replay_mcap(capture, window=ReplayWindow(), project_root=project_root)
    return {
        "requested": requested, "applicable": True, "executed": True,
        "status": raw.get("status"),
        "capture": raw.get("capture"),
        "scope": raw.get("scope"),
        "execution": raw.get("execution"),
        "reconstruction_contract": raw.get("reconstruction_contract"),
        "determinism": raw.get("determinism"),
        "first_divergence": raw.get("first_divergence"),
        "source_first": raw.get("source_first"),
        "mcap_bridge": raw.get("mcap_bridge"),
    }


def _other_json_events(reader: McapReader) -> list[dict[str, object]]:
    excluded = {TICK_TOPIC, RAW_LIDAR_TOPIC, CHECKPOINT_TOPIC, RUNTIME_TOPIC}
    rows: list[dict[str, object]] = []
    for message, payload in reader.iter_json_messages():
        if message.topic in excluded:
            continue
        rows.append({
            "topic": message.topic, "sequence": message.sequence,
            "monotonic_ns": message.log_time_ns, "payload": payload,
        })
    return rows


def _prepare_output(destination: Path) -> None:
    if destination.is_symlink():
        raise TestHubCompilerError("output directory must not be a symlink")
    if destination.exists():
        if not destination.is_dir() or any(destination.iterdir()):
            raise TestHubCompilerError("output directory must be absent or empty")
    else:
        destination.mkdir(parents=True)


def compile_evidence(
    capture_path: str | Path,
    output_dir: str | Path,
    *,
    replay_mode: str = "auto",
    project_root: str | Path | None = None,
) -> dict[str, object]:
    capture = Path(capture_path).resolve()
    if capture.suffix.lower() != ".mcap" or not capture.is_file() or capture.is_symlink():
        raise TestHubCompilerError("Test Hub accepts one regular finished .mcap file")
    destination = Path(output_dir).resolve()
    _prepare_output(destination)
    root = Path(project_root).resolve() if project_root is not None else Path.cwd().resolve()
    reader = McapReader(capture)

    artifacts: list[Path] = []
    capture_facts = inspect_mcap(capture)
    artifacts.append(_write_json(destination / "capture_facts.json", capture_facts))

    artifacts.append(_write_json(destination / "runtime_config.json", {
        "schema": "R2B4_TEST_HUB_RUNTIME_CONFIG_V1",
        "policy": FACT_ONLY_POLICY,
        "runtime": _runtime_payload(reader),
    }))

    ticks = read_ticks(reader)
    checkpoints = read_checkpoints(reader)
    run_facts = analyze_ticks(ticks)
    artifacts.append(_write_json(destination / "run_facts.json", run_facts))
    artifacts.append(write_ndjson(destination / "production_events.ndjson", run_facts["production_events"]))
    artifacts.append(_write_json(destination / "encoder_evidence.json", run_facts["encoder"]))
    artifacts.append(_write_json(destination / "motion_metrics.json", run_facts["motion"]))
    artifacts.append(_write_json(destination / "localization_metrics.json", run_facts["localization"]))
    artifacts.append(_write_json(
        destination / "control_feedback_evidence.json",
        extract_control_state(checkpoints, ticks),
    ))
    artifacts.append(write_ndjson(destination / "captured_events.ndjson", _other_json_events(reader)))

    coverage_path = destination / "diagnostic_schema_coverage.json"
    write_diagnostic_coverage(reader, coverage_path)
    artifacts.append(coverage_path)

    replay = _replay_facts(capture, root, replay_mode, _tick_sample_hz(reader))
    artifacts.append(_write_json(destination / "replay_facts.json", replay))

    final_event = capture_facts.get("final_event")
    integrity = final_event.get("integrity") if isinstance(final_event, Mapping) else None
    agent = {
        "schema": AGENT_EVIDENCE_SCHEMA,
        "policy": FACT_ONLY_POLICY,
        "compiler_status": "COMPLETE",
        "authority": {
            "capture_path": str(capture),
            "capture_sha256": reader.sha256(),
            "mcap_is_authority": True,
        },
        "captured_execution_status": (
            final_event.get("status") if isinstance(final_event, Mapping) else None
        ),
        "capture_integrity": integrity,
        "tick_sample_hz": _tick_sample_hz(reader),
        "tick_count": len(ticks),
        "replay": replay,
        "artifacts": {
            "capture_facts": "capture_facts.json",
            "runtime_config": "runtime_config.json",
            "run_facts": "run_facts.json",
            "production_events": "production_events.ndjson",
            "encoder_evidence": "encoder_evidence.json",
            "control_feedback_evidence": "control_feedback_evidence.json",
            "motion_metrics": "motion_metrics.json",
            "localization_metrics": "localization_metrics.json",
            "diagnostic_schema_coverage": "diagnostic_schema_coverage.json",
            "captured_events": "captured_events.ndjson",
            "replay_facts": "replay_facts.json",
        },
        "semantics": {
            "diagnosis_emitted": False,
            "priority_emitted": False,
            "severity_emitted": False,
            "causal_inference_emitted": False,
            "root_cause_emitted": False,
        },
    }
    agent_path = _write_json(destination / "agent_evidence.json", agent)
    artifacts.append(agent_path)

    index_payload: dict[str, object] = {
        "schema": EVIDENCE_INDEX_SCHEMA,
        "policy": FACT_ONLY_POLICY,
        "compiler_status": "COMPLETE",
        "authority_capture": {"path": str(capture), "sha256": reader.sha256()},
        "artifacts": {
            path.name: {"sha256": _sha256_file(path), "size_bytes": path.stat().st_size}
            for path in artifacts
        },
    }
    unsigned = json.dumps(index_payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    index_payload["evidence_sha256"] = hashlib.sha256(unsigned).hexdigest()
    index_path = _write_json(destination / "evidence_index.json", index_payload)

    return {
        "schema": COMPILER_SCHEMA,
        "policy": FACT_ONLY_POLICY,
        "status": "COMPLETE",
        "compiler_status": "COMPLETE",
        "capture": str(capture), "output_dir": str(destination),
        "agent_evidence": str(agent_path), "evidence_index": str(index_path),
        "captured_execution_status": agent["captured_execution_status"],
        "capture_integrity_complete": (
            integrity.get("complete") if isinstance(integrity, Mapping) else None
        ),
        "replay_status": replay.get("status"),
    }


def verify_evidence(index_path: str | Path) -> dict[str, object]:
    path = Path(index_path).resolve()
    if not path.is_file() or path.is_symlink():
        raise TestHubCompilerError("evidence index must be a regular file")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping) or value.get("schema") != EVIDENCE_INDEX_SCHEMA:
        return {"status": "INVALID", "index_path": str(path), "index_schema_valid": False}
    unsigned = dict(value)
    expected = unsigned.pop("evidence_sha256", None)
    actual = hashlib.sha256(
        json.dumps(unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()
    artifacts = value.get("artifacts")
    all_match = isinstance(artifacts, Mapping)
    rows: dict[str, object] = {}
    if isinstance(artifacts, Mapping):
        for name, spec in artifacts.items():
            artifact = path.parent / str(name)
            spec_map = spec if isinstance(spec, Mapping) else {}
            safe = Path(str(name)).name == str(name)
            match = bool(
                safe and artifact.is_file() and not artifact.is_symlink()
                and _sha256_file(artifact) == spec_map.get("sha256")
                and artifact.stat().st_size == spec_map.get("size_bytes")
            )
            rows[str(name)] = {"matches_index": match}
            all_match = bool(all_match and match)
    valid = expected == actual and bool(all_match)
    return {
        "status": "VALID" if valid else "INVALID",
        "index_path": str(path), "index_checksum_valid": expected == actual,
        "artifacts_valid": bool(all_match), "artifacts": rows,
    }


def latest_capture(capture_dir: Path = Path("runtime/captures")) -> Path:
    rows = [p for p in capture_dir.glob("*.mcap") if p.is_file() and not p.is_symlink()]
    if not rows:
        raise FileNotFoundError(f"no MCAP capture under {capture_dir}")
    return max(rows, key=lambda p: p.stat().st_mtime_ns)


def run_pending(capture_dir: Path, *, replay_mode: str = "auto") -> dict[str, object]:
    results: list[dict[str, object]] = []
    for capture in sorted(capture_dir.glob("*.mcap")):
        destination = capture.with_suffix(".evidence")
        if destination.exists():
            continue
        results.append(compile_evidence(
            capture, destination, replay_mode=replay_mode,
            project_root=Path(__file__).resolve().parents[1],
        ))
    return {"status": "COMPLETE", "compiled_count": len(results), "results": results}


def query_capture(
    capture_path: str | Path, *, topic: str,
    start_ns: int | None = None, end_ns: int | None = None, max_rows: int = 200,
) -> dict[str, object]:
    reader = McapReader(capture_path)
    rows: list[dict[str, object]] = []
    for message, payload in reader.iter_json_messages(
        topics=(topic,), start_ns=start_ns, end_ns=end_ns
    ):
        rows.append({
            "topic": topic, "sequence": message.sequence,
            "monotonic_ns": message.log_time_ns, "payload": payload,
        })
        if len(rows) >= max_rows:
            break
    return {"topic": topic, "rows": rows, "row_count": len(rows), "max_rows": max_rows}
