"""Bounded MCAP -> current Replayer V3 bridge.

MCAP remains the authority capture. For compatibility with the current public
``v3.replay.replay_capture(path)`` API, this bridge materializes only the
required replay window into a temporary JSON file under /tmp and deletes it
immediately after replay. No persistent JSON capture is produced.

The bridge prefers a checkpoint before the requested incident. That keeps
agent-driven replay small while preserving current production state semantics.
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from .mcap_reader import CHECKPOINT_TOPIC, EVENT_TOPIC, McapReader, McapReadError, RUNTIME_TOPIC, TICK_TOPIC


class McapReplayBridgeError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class ReplayWindow:
    requested_start_tick_id: int | None = None
    requested_end_tick_id: int | None = None
    requested_start_ns: int | None = None
    requested_end_ns: int | None = None
    start_layer: str = "L1"
    end_layer: str = "L12"
    max_materialized_ticks: int = 4096
    max_materialized_bytes: int = 128 * 1024 * 1024

    def __post_init__(self) -> None:
        if self.max_materialized_ticks <= 0 or self.max_materialized_bytes <= 0:
            raise ValueError("replay materialization bounds must be positive")


def replay_mcap(
    capture_path: str | Path,
    *,
    window: ReplayWindow | None = None,
    project_root: str | Path | None = None,
    capture_source_manifest_path: str | Path | None = None,
) -> dict[str, object]:
    """Replay a bounded MCAP slice through the current canonical Replayer V3."""

    # Imports stay local so inspect/agent-only usage has no dependency on the
    # legacy JSON capture path and can run even while replay is being refactored.
    from .capture import V3_CAPTURE_SCHEMA, payload_sha256
    from .replay import ReplaySelection, replay_capture, _payload_sha256

    try:
        reader = McapReader(capture_path)
        structure = reader.inspect(verify_chunks=True)
        if not structure.valid:
            raise McapReplayBridgeError("MCAP structural/deep CRC preflight failed")
        final = reader.capture_integrity()
    except McapReadError as exc:
        raise McapReplayBridgeError("MCAP preflight failed: " + str(exc)) from exc
    authority_sha256 = reader.sha256()
    requested = window or ReplayWindow()
    runtime_pair = reader.first_json(RUNTIME_TOPIC)
    if runtime_pair is None or not isinstance(runtime_pair[1], Mapping):
        raise McapReplayBridgeError("MCAP capture lacks /r2b4/runtime configuration")
    runtime = runtime_pair[1]

    first_target_msg = None
    last_target_msg = None
    for message in reader.iter_messages(topics=(TICK_TOPIC,)):
        if _selected_message(message.sequence, message.log_time_ns, requested):
            if first_target_msg is None:
                first_target_msg = message
            last_target_msg = message
    if first_target_msg is None or last_target_msg is None:
        raise McapReplayBridgeError("requested replay selection contains no ticks")

    checkpoint_message = None
    checkpoint_payload: Mapping[str, object] | None = None
    for message, payload in reader.iter_json_messages(
        topics=(CHECKPOINT_TOPIC,),
        end_ns=first_target_msg.log_time_ns,
    ):
        if not isinstance(payload, Mapping):
            continue
        state = payload.get("state")
        if not isinstance(state, Mapping):
            continue
        tick_id = payload.get("tick_id")
        if isinstance(tick_id, int) and tick_id >= first_target_msg.sequence:
            continue
        checkpoint_message = message
        checkpoint_payload = payload

    decoded_ticks: list[dict[str, object]] = []
    materialized_bytes = 0
    for message in reader.iter_messages(
        topics=(TICK_TOPIC,),
        start_ns=checkpoint_message.log_time_ns if checkpoint_message else None,
        end_ns=last_target_msg.log_time_ns,
    ):
        if checkpoint_message is not None and message.sequence <= checkpoint_message.sequence:
            continue
        materialized_bytes += len(message.data)
        if len(decoded_ticks) >= requested.max_materialized_ticks or materialized_bytes > requested.max_materialized_bytes:
            raise McapReplayBridgeError("replay window exceeds bounded materialization budget; select a shorter checkpoint window")
        payload = message.json()
        if not isinstance(payload, dict):
            raise McapReplayBridgeError("/r2b4/tick payload must be a JSON object")
        decoded_ticks.append(payload)
    if not decoded_ticks:
        raise McapReplayBridgeError("checkpoint leaves no replay ticks")
    first_tick_id = decoded_ticks[0]["tick_id"]
    state_available = checkpoint_payload is not None or first_tick_id == 0
    if not state_available:
        raise McapReplayBridgeError("CAPTURE_INCOMPLETE: no initial state checkpoint or tick-zero prefix")
    if checkpoint_payload is not None and checkpoint_payload.get("tick_id") != first_tick_id - 1:
        raise McapReplayBridgeError("CAPTURE_INCOMPLETE: checkpoint does not precede first replay tick")
    original_complete = replay_eligible = True

    configuration = runtime.get("configuration")
    metadata = runtime.get("metadata")
    if not isinstance(configuration, Mapping):
        raise McapReplayBridgeError("/r2b4/runtime configuration is not an object")
    if not isinstance(metadata, Mapping):
        metadata = {}

    status = final.get("status") if isinstance(final, Mapping) else None
    if status not in {"PASS", "FAIL", "FAULT"}:
        status = "FAIL"

    document: dict[str, object] = {
        "schema": V3_CAPTURE_SCHEMA,
        "capture_id": str(runtime.get("capture_id") or Path(capture_path).stem),
        "status": status,
        "created_at_utc": "1970-01-01T00:00:00Z",
        "configuration": dict(configuration),
        "metadata": {
            **dict(metadata),
            "mcap_authority_path": str(Path(capture_path).resolve()),
            "mcap_replay_bridge": True,
        },
        "tick_count": len(decoded_ticks),
        "ticks": decoded_ticks,
        "capture_integrity": {
            "complete": replay_eligible,
            "replay_match_eligible": replay_eligible,
            "source_mcap_complete": original_complete,
            "state_checkpoint_complete": state_available,
            "bridge_window_tick_count": len(decoded_ticks),
        },
    }
    if checkpoint_payload is not None:
        state = checkpoint_payload.get("state")
        if isinstance(state, Mapping):
            document["initial_state_checkpoint"] = dict(state)

    document["capture_sha256"] = payload_sha256(document)
    # Include config, state and metadata in the disk budget, not only tick bytes.
    encoded_document = json.dumps(document, ensure_ascii=False, sort_keys=True,
                                  separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
    if len(encoded_document) > requested.max_materialized_bytes:
        raise McapReplayBridgeError("temporary JSON exceeds bounded materialization budget")

    selection = ReplaySelection(
        start_tick_id=requested.requested_start_tick_id,
        end_tick_id=requested.requested_end_tick_id,
        start_monotonic_ns=requested.requested_start_ns,
        end_monotonic_ns=requested.requested_end_ns,
        start_layer=requested.start_layer,
        end_layer=requested.end_layer,
    )

    temporary_path: Path | None = None
    try:
        fd, name = tempfile.mkstemp(prefix="r2b4-mcap-replay-", suffix=".json", dir="/tmp")
        os.close(fd)
        temporary_path = Path(name)
        temporary_path.write_bytes(encoded_document)
        if reader.sha256() != authority_sha256:
            raise McapReplayBridgeError("authority MCAP changed during preflight")
        result = replay_capture(
            temporary_path,
            selection=selection,
            project_root=project_root,
            capture_source_manifest_path=capture_source_manifest_path,
        )
        if reader.sha256() != authority_sha256:
            raise McapReplayBridgeError("authority MCAP changed during replay")
        result = dict(result)
        result["mcap_bridge"] = {
            "authority_capture": str(Path(capture_path).resolve()),
            "authority_sha256": authority_sha256,
            "preflight": "PASS",
            "materialized_bytes": len(encoded_document),
            "temporary_json_persisted": False,
            "materialized_tick_count": len(decoded_ticks),
            "checkpoint_used": checkpoint_payload is not None,
            "replay_eligible": replay_eligible,
        }
        result["capture"] = {
            **result["capture"], "path": str(reader.path.resolve()),
            "sha256": authority_sha256, "schema": "R2B4_MCAP_CAPTURE_V1",
            "payload_sha256": final["integrity"]["message_stream_sha256"],
        }
        result.pop("result_sha256", None)
        result["result_sha256"] = _payload_sha256(result)
        return result
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def _selected_message(sequence: int, monotonic_ns: int, window: ReplayWindow) -> bool:
    return bool(
        (window.requested_start_tick_id is None or sequence >= window.requested_start_tick_id)
        and (window.requested_end_tick_id is None or sequence <= window.requested_end_tick_id)
        and (window.requested_start_ns is None or monotonic_ns >= window.requested_start_ns)
        and (window.requested_end_ns is None or monotonic_ns <= window.requested_end_ns)
    )


def _final_event(reader: McapReader) -> dict[str, object]:
    result: dict[str, object] = {}
    for _message, payload in reader.iter_json_messages(topics=(EVENT_TOPIC,)):
        if isinstance(payload, Mapping) and payload.get("event_type") == "capture_finalized":
            result = dict(payload)
    return result


__all__ = ["McapReplayBridgeError", "ReplayWindow", "replay_mcap"]
