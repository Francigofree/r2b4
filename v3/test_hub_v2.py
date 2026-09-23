"""R2B4 Test Hub V2: MCAP-native triage, bounded replay and agent evidence.

One engine serves CLI, GUI and coding agents. MCAP remains the authority.
Infrastructure validity/replay determinism and robot-behaviour findings are
reported separately so a perfectly replayable failure is never presented as
an unqualified PASS.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Mapping, Sequence

from .mcap_reader import (
    EVENT_TOPIC,
    McapReadError,
    McapReader,
    RAW_LIDAR_TOPIC,
    RUNTIME_TOPIC,
    TICK_TOPIC,
)
from .mcap_replay_bridge import McapReplayBridgeError, ReplayWindow, replay_mcap
from .test_hub_analysis import LAYER_ORDER, analyze_capture
from .test_hub_profiles import BEHAVIORAL, capture_analysis_profile

TEST_HUB_SCHEMA = "R2B4_TEST_HUB_V2"
AGENT_BRIEF_SCHEMA = "R2B4_AGENT_BRIEF_V2"
GUI_MANIFEST_SCHEMA = "R2B4_TEST_HUB_GUI_V2"
EVIDENCE_INDEX_SCHEMA = "R2B4_EVIDENCE_INDEX_V2"

DEFAULT_AGENT_MAX_BYTES = 48 * 1024
DEFAULT_SLICE_MAX_BYTES = 512 * 1024

LAYER_SOURCE = {
    "L1": "v3/layers/l1_acquisition.py",
    "L2": "v3/layers/l2_admission.py",
    "L3": "v3/layers/l3_state_estimation.py",
    "L4": "v3/layers/l4_world_model.py",
    "L5": "v3/layers/l5_command_mission.py",
    "L6": "v3/layers/l6_navigation.py",
    "L7": "v3/layers/l7_motion_selection.py",
    "L8": "v3/layers/l8_motion_realization.py",
    "L9": "v3/layers/l9_operational_constraints.py",
    "L10": "v3/layers/l10_chassis_control.py",
    "L11": "v3/layers/l11_actuator_control.py",
    "L12": "v3/layers/l12_safety_final.py",
}


class TestHubV2Error(RuntimeError):
    pass


def inspect_mcap(
    capture_path: str | Path, *, deep: bool = False
) -> dict[str, object]:
    reader = McapReader(capture_path)
    structure = reader.inspect(verify_chunks=deep)
    final_meta = reader.latest_metadata("r2b4.capture.final")
    capture_meta = reader.latest_metadata("r2b4.capture")
    final_event = _final_event(reader)
    integrity_error = None
    if deep and structure.valid:
        try:
            reader.capture_integrity()
        except McapReadError as exc:
            integrity_error = str(exc)
    runtime = reader.first_json(RUNTIME_TOPIC)
    runtime_payload = (
        runtime[1] if runtime and isinstance(runtime[1], Mapping) else {}
    )
    return {
        "schema": TEST_HUB_SCHEMA,
        "status": (
            "PASS"
            if structure.valid and integrity_error is None
            else "FAIL"
        ),
        "integrity_error": integrity_error,
        "capture_path": str(reader.path.resolve()),
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
        "topics": {
            channel.topic: {
                "channel_id": channel.channel_id,
                "encoding": channel.message_encoding,
                "metadata": dict(channel.metadata),
                "message_count": _channel_count(reader, channel.channel_id),
            }
            for channel in reader.channels.values()
        },
        "capture_metadata": capture_meta,
        "analysis_profile": capture_analysis_profile(reader),
        "final_metadata": final_meta,
        "runtime": {
            "capture_id": runtime_payload.get("capture_id"),
            "capture_format_version": runtime_payload.get(
                "capture_format_version"
            ),
            "clock_epoch": runtime_payload.get("clock_epoch"),
        },
        "final_event": final_event,
    }


def diagnose_run(
    capture_path: str | Path,
    output_dir: str | Path,
    *,
    replay_mode: str = "incident",
    project_root: str | Path | None = None,
    capture_source_manifest_path: str | Path | None = None,
    agent_max_bytes: int = DEFAULT_AGENT_MAX_BYTES,
    slice_max_bytes: int = DEFAULT_SLICE_MAX_BYTES,
    replay_window: ReplayWindow | None = None,
) -> dict[str, object]:
    """Create one run-bound evidence directory from an MCAP authority capture."""

    destination = _prepare_output_dir(output_dir)
    reader = McapReader(capture_path)

    inspect = inspect_mcap(capture_path, deep=True)
    inspect_path = _write_json(inspect, destination / "inspect.json")

    timeline_path = destination / "timeline.ndjson"
    triage = analyze_capture(reader, timeline_path=timeline_path)
    triage_path = _write_json(triage, destination / "triage.json")

    replay: dict[str, object] | None = None
    replay_error: str | None = None
    mode = replay_mode.lower().strip()
    if mode not in {"off", "incident", "full"}:
        raise ValueError("replay_mode must be off, incident or full")

    replay_applicable = inspect["analysis_profile"]["exact_replay_applicable"]
    if mode != "off" and replay_applicable:
        try:
            if replay_window is None:
                replay_window = (
                    _incident_replay_window(triage)
                    if mode == "incident"
                    else ReplayWindow()
                )
            replay = replay_mcap(
                capture_path,
                window=replay_window,
                project_root=project_root,
                capture_source_manifest_path=capture_source_manifest_path,
            )
        except (
            McapReplayBridgeError,
            OSError,
            TypeError,
            ValueError,
            RuntimeError,
        ) as exc:
            replay_error = str(exc)

    replay_path: Path | None = None
    if replay is not None:
        replay_path = _write_json(
            replay, destination / "replay_result.json"
        )

    diagnosis = _build_diagnosis(
        inspect,
        triage,
        replay,
        replay_error,
        replay_requested=mode != "off" and replay_applicable,
    )
    diagnosis["replay_mode_requested"] = mode
    diagnosis_path = _write_json(
        diagnosis, destination / "diagnosis.json"
    )

    root = diagnosis.get("root_cause")
    root_tick = (
        root.get("tick_id") if isinstance(root, Mapping) else None
    )
    root_layer = (
        root.get("layer") if isinstance(root, Mapping) else None
    )
    slice_path = destination / "interesting_slice.ndjson"
    slice_info = write_interesting_slice(
        reader,
        slice_path,
        around_tick=root_tick if isinstance(root_tick, int) else None,
        root_layer=(
            str(root_layer) if isinstance(root_layer, str) else None
        ),
        max_bytes=slice_max_bytes,
    )

    agent_brief = _build_agent_brief(
        capture_path,
        inspect,
        triage,
        replay,
        diagnosis,
        slice_info,
        max_bytes=agent_max_bytes,
    )
    agent_path = _write_json(
        agent_brief, destination / "agent_brief.json"
    )

    gui_manifest = _build_gui_manifest(
        inspect, triage, diagnosis, timeline_path, slice_path
    )
    gui_path = _write_json(
        gui_manifest, destination / "gui_manifest.json"
    )

    artifact_paths = [
        inspect_path,
        triage_path,
        timeline_path,
        diagnosis_path,
        slice_path,
        agent_path,
        gui_path,
    ]
    if replay_path is not None:
        artifact_paths.append(replay_path)

    evidence = _build_evidence_index(
        reader, destination, artifact_paths, diagnosis
    )
    evidence_path = _write_json(
        evidence, destination / "evidence_index.json"
    )

    return {
        "schema": TEST_HUB_SCHEMA,
        "status": diagnosis["status"],
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get("diagnosis_status"),
        "evidence_status": diagnosis.get("evidence_status"),
        "behavior_status": diagnosis.get("behavior_status"),
        "capture_status": diagnosis.get("capture_status"),
        "capture_integrity": diagnosis.get("capture_integrity"),
        "replay_status": diagnosis.get("replay_status"),
        "root_cause": diagnosis.get("root_cause"),
        "incident_count": triage.get("incident_count"),
        "output_dir": str(destination.resolve()),
        "agent_brief": str(agent_path.resolve()),
        "gui_manifest": str(gui_path.resolve()),
        "evidence_index": str(evidence_path.resolve()),
    }


def build_agent_brief_only(
    capture_path: str | Path,
    *,
    replay: bool = True,
    project_root: str | Path | None = None,
    max_bytes: int = DEFAULT_AGENT_MAX_BYTES,
) -> dict[str, object]:
    """Fast agent-facing answer without persistent evidence files."""

    reader = McapReader(capture_path)
    inspect = inspect_mcap(capture_path, deep=True)
    triage = analyze_capture(reader)
    replay_result = None
    replay_error = None

    replay_applicable = inspect["analysis_profile"]["exact_replay_applicable"]
    if replay and replay_applicable:
        try:
            replay_result = replay_mcap(
                capture_path,
                window=_incident_replay_window(triage),
                project_root=project_root,
            )
        except (
            McapReplayBridgeError,
            OSError,
            TypeError,
            ValueError,
            RuntimeError,
        ) as exc:
            replay_error = str(exc)

    diagnosis = _build_diagnosis(
        inspect,
        triage,
        replay_result,
        replay_error,
        replay_requested=replay and replay_applicable,
    )
    return _build_agent_brief(
        capture_path,
        inspect,
        triage,
        replay_result,
        diagnosis,
        {"written": False, "reason": "agent-only mode"},
        max_bytes=max_bytes,
    )


def write_interesting_slice(
    reader: McapReader,
    output_path: Path,
    *,
    around_tick: int | None,
    root_layer: str | None,
    before: int = 5,
    after: int = 5,
    max_bytes: int = DEFAULT_SLICE_MAX_BYTES,
) -> dict[str, object]:
    """Write a bounded, human/agent-readable exact-evidence slice."""

    layers = _layer_neighborhood(root_layer)
    start_tick = (
        max(0, around_tick - before)
        if isinstance(around_tick, int)
        else None
    )
    end_tick = (
        around_tick + after
        if isinstance(around_tick, int)
        else None
    )
    written = 0
    rows = 0
    truncated = False
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as handle:
        for message, payload in reader.iter_json_messages(
            topics=(TICK_TOPIC,)
        ):
            if (
                start_tick is not None
                and message.sequence < start_tick
            ):
                continue
            if end_tick is not None and message.sequence > end_tick:
                continue
            if not isinstance(payload, Mapping):
                continue
            reduced = _reduce_tick(
                payload,
                layers=layers,
                include_inputs=(message.sequence == around_tick),
            )
            line = (
                json.dumps(
                    {
                        "topic": TICK_TOPIC,
                        "sequence": message.sequence,
                        "monotonic_ns": message.log_time_ns,
                        "payload": _prune_large(reduced),
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                + "\n"
            )
            encoded = line.encode("utf-8")
            if written + len(encoded) > max_bytes:
                truncated = True
                break
            handle.write(line)
            written += len(encoded)
            rows += 1

        if around_tick is not None:
            final_line = (
                json.dumps(
                    {
                        "note": (
                            "raw lidar point arrays omitted by default; "
                            "use query --topic /r2b4/raw_lidar --full"
                        ),
                        "root_tick": around_tick,
                        "layers": list(layers),
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
            if (
                written + len(final_line.encode("utf-8"))
                <= max_bytes
            ):
                handle.write(final_line)
                written += len(final_line.encode("utf-8"))

    return {
        "path": str(output_path.resolve()),
        "rows": rows,
        "size_bytes": written,
        "truncated": truncated,
        "tick_range": [start_tick, end_tick],
        "layers": list(layers),
    }


def query_capture(
    capture_path: str | Path,
    *,
    topics: Sequence[str],
    start_tick_id: int | None = None,
    end_tick_id: int | None = None,
    start_ns: int | None = None,
    end_ns: int | None = None,
    layers: Sequence[str] = (),
    include_inputs: bool = False,
    full: bool = False,
    max_bytes: int = 2 * 1024 * 1024,
) -> dict[str, object]:
    """Return precise selected evidence with a hard size ceiling."""

    reader = McapReader(capture_path)
    result_rows: list[dict[str, object]] = []
    current_bytes = 2
    truncated = False

    for message, payload in reader.iter_json_messages(
        topics=topics,
        start_ns=start_ns,
        end_ns=end_ns,
    ):
        if (
            start_tick_id is not None
            and message.topic == TICK_TOPIC
            and message.sequence < start_tick_id
        ):
            continue
        if (
            end_tick_id is not None
            and message.topic == TICK_TOPIC
            and message.sequence > end_tick_id
        ):
            continue

        selected_payload: object = payload
        if not full:
            if (
                message.topic == TICK_TOPIC
                and isinstance(payload, Mapping)
            ):
                selected_payload = _reduce_tick(
                    payload,
                    layers=(
                        tuple(layers) if layers else LAYER_ORDER
                    ),
                    include_inputs=include_inputs,
                )
            elif (
                message.topic == RAW_LIDAR_TOPIC
                and isinstance(payload, Mapping)
            ):
                selected_payload = {
                    key: value
                    for key, value in payload.items()
                    if key != "points"
                }
                points = payload.get("points")
                if isinstance(points, list):
                    selected_payload["points_omitted"] = len(points)
            else:
                selected_payload = _prune_large(payload)

        row = {
            "topic": message.topic,
            "sequence": message.sequence,
            "monotonic_ns": message.log_time_ns,
            "payload": selected_payload,
        }
        encoded = json.dumps(
            row,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        if current_bytes + len(encoded) + 1 > max_bytes:
            truncated = True
            break
        result_rows.append(row)
        current_bytes += len(encoded) + 1

    return {
        "schema": "R2B4_TEST_HUB_QUERY_V2",
        "status": "PASS",
        "capture": str(Path(capture_path).resolve()),
        "row_count": len(result_rows),
        "truncated": truncated,
        "max_bytes": max_bytes,
        "rows": result_rows,
    }


def verify_evidence(index_path: str | Path) -> dict[str, object]:
    path = Path(index_path)
    if path.is_symlink() or not path.is_file():
        raise TestHubV2Error(
            "evidence index must be a regular file"
        )
    payload = _read_json_object(path)
    expected = payload.get("evidence_sha256")
    checksum_ok = (
        isinstance(expected, str)
        and expected
        == _payload_sha256(payload, "evidence_sha256")
    )

    artifacts = payload.get("artifacts")
    artifact_results: dict[str, object] = {}
    all_ok = True
    if not isinstance(artifacts, Mapping):
        all_ok = False
    else:
        for name, row in artifacts.items():
            safe = Path(str(name)).name == str(name)
            artifact_path = path.parent / str(name)
            regular = (
                safe
                and not artifact_path.is_symlink()
                and artifact_path.is_file()
            )
            actual_hash = (
                _sha256_file(artifact_path) if regular else None
            )
            actual_size = (
                artifact_path.stat().st_size if regular else None
            )
            expected_row = (
                row if isinstance(row, Mapping) else {}
            )
            matches = bool(
                regular
                and expected_row.get("sha256") == actual_hash
                and expected_row.get("size_bytes") == actual_size
            )
            all_ok = all_ok and matches
            artifact_results[str(name)] = {
                "regular_file": regular,
                "matches_index": matches,
                "sha256": actual_hash,
                "size_bytes": actual_size,
            }

    authority = payload.get("authority_capture")
    authority_path = (
        Path(str(authority.get("path", "")))
        if isinstance(authority, Mapping)
        else None
    )
    authority_hash = None
    if (
        authority_path is not None
        and authority_path.is_absolute()
        and not authority_path.is_symlink()
        and authority_path.is_file()
    ):
        try:
            authority_hash = _sha256_file(authority_path)
        except OSError:
            pass

    authority_ok = bool(
        authority_hash
        and isinstance(authority, Mapping)
        and authority_hash == authority.get("sha256")
    )
    return {
        "status": (
            "PASS"
            if checksum_ok and all_ok and authority_ok
            else "FAIL"
        ),
        "index_checksum_ok": checksum_ok,
        "authority_capture_ok": authority_ok,
        "authority_sha256": authority_hash,
        "artifacts_ok": all_ok,
        "artifacts": artifact_results,
    }


def _build_diagnosis(
    inspect: Mapping[str, object],
    triage: Mapping[str, object],
    replay: Mapping[str, object] | None,
    replay_error: str | None,
    *,
    replay_requested: bool = True,
) -> dict[str, object]:
    """Build diagnosis without conflating evidence validity with behaviour."""

    profile = inspect.get("analysis_profile") or triage.get("analysis_profile")
    behavioral = isinstance(profile, Mapping) and profile.get("name") == BEHAVIORAL
    root = (
        dict(triage.get("root_cause_candidate") or {})
        if isinstance(triage.get("root_cause_candidate"), Mapping)
        else {}
    )
    replay_status = (
        replay.get("status") if isinstance(replay, Mapping) else None
    )
    first_divergence = (
        replay.get("first_divergence")
        if isinstance(replay, Mapping)
        else None
    )
    first_live_incident = (
        replay.get("first_live_incident")
        if isinstance(replay, Mapping)
        else None
    )
    physical_root = (
        replay.get("physical_root_cause")
        if isinstance(replay, Mapping)
        else None
    )

    if (
        isinstance(first_divergence, Mapping)
        and first_divergence.get("reason")
        != "CAPTURE_INCOMPLETE"
        and replay_status == "MISMATCH"
        and not inspect.get("integrity_error")
    ):
        root = {
            "confidence": "PROVEN",
            "reason": (
                first_divergence.get("reason")
                or "REPLAY_DIVERGENCE"
            ),
            "tick_id": first_divergence.get("tick_id"),
            "layer": first_divergence.get("layer"),
            "kind": "SOFTWARE_REPLAY_DIVERGENCE",
            "evidence": _prune_large(first_divergence),
        }
    elif (
        isinstance(physical_root, Mapping)
        and physical_root.get("status")
        in {"PROVEN", "INDICATED"}
    ):
        root = {
            **root,
            "replayer_physical_root_cause": _prune_large(
                physical_root
            ),
        }

    final_event = inspect.get("final_event")
    integrity = (
        final_event.get("integrity")
        if isinstance(final_event, Mapping)
        else None
    )
    structure = inspect.get("structure")
    structure_ok = bool(
        isinstance(structure, Mapping)
        and structure.get("valid") is True
    )
    capture_complete = bool(
        isinstance(integrity, Mapping)
        and integrity.get("complete") is True
    )
    capture_complete = (
        capture_complete and not inspect.get("integrity_error")
    )

    if (
        not capture_complete
        or (
            isinstance(first_divergence, Mapping)
            and first_divergence.get("reason")
            == "CAPTURE_INCOMPLETE"
        )
    ):
        root = {
            "kind": "CAPTURE_INCOMPLETE",
            "confidence": "EVIDENCE_BLOCKED",
            "reason": (
                inspect.get("integrity_error")
                or "CAPTURE_INCOMPLETE"
            ),
        }

    replay_gate_ok = (
        (not replay_requested and replay_error is None)
        or replay_status == "MATCH"
    )
    if replay_requested and replay_status is None:
        replay_status = (
            "ERROR" if replay_error else "NOT_RUN"
        )

    evidence_status = (
        "PASS"
        if structure_ok and capture_complete and replay_gate_ok
        else "FAIL"
    )
    behavior_status = _triage_behavior_status(triage)
    actionable = _has_actionable_finding(triage)

    if evidence_status != "PASS":
        diagnosis_status = "FAIL"
    elif actionable:
        diagnosis_status = "FINDING"
    elif behavioral and triage.get("warning_count", 0):
        diagnosis_status = "WARNING"
    else:
        diagnosis_status = "PASS"

    return {
        "schema": "R2B4_TEST_HUB_DIAGNOSIS_V2",
        **({"analysis_profile": profile} if profile is not None else {}),
        # Deliberately no longer an unqualified infrastructure-only PASS.
        "status": diagnosis_status,
        "diagnosis_status": diagnosis_status,
        "evidence_status": evidence_status,
        "behavior_status": behavior_status,
        "capture_status": (
            final_event.get("status")
            if isinstance(final_event, Mapping)
            else None
        ),
        "capture_integrity": integrity,
        "structure_valid": structure_ok,
        "replay_status": "NOT_APPLICABLE" if behavioral and not replay_requested else replay_status,
        **({"replay_skip_reason": "SAMPLED_TICK_STREAM", "warning_count": triage.get("warning_count", 0)} if behavioral else {}),
        "replay_error": replay_error,
        "replay_requested": replay_requested,
        "root_cause": root,
        "first_divergence": first_divergence,
        "first_live_incident": first_live_incident,
        "physical_root_cause": physical_root,
        "metrics": {
            "ticks": triage.get("ticks"),
            "timing": triage.get("timing"),
            "motion": triage.get("motion"),
            "safety": triage.get("safety"),
            "sensors": triage.get("sensors"),
            "localization": triage.get("localization"),
            "navigation": triage.get("navigation"),
            **({"behavioral_trends": triage.get("behavioral_trends")} if behavioral else {}),
        },
        "incident_count": triage.get("incident_count"),
    }


def _triage_behavior_status(
    triage: Mapping[str, object],
) -> str:
    direct = triage.get("behavior_status")
    if isinstance(direct, str) and direct:
        return direct
    motion = triage.get("motion")
    if isinstance(motion, Mapping):
        nested = motion.get("behavior_status")
        if isinstance(nested, str) and nested:
            return nested
        requested = motion.get("requested_motion_tick_count")
        zero = motion.get("constrained_to_zero_tick_count")
        if isinstance(requested, int) and requested > 0:
            if isinstance(zero, int) and zero >= requested:
                return "BLOCKED"
            if isinstance(zero, int) and zero > 0:
                return "DEGRADED"
            return "NOMINAL"
    return "NO_MOTION_REQUEST"


def _has_actionable_finding(
    triage: Mapping[str, object],
) -> bool:
    if _triage_behavior_status(triage) in {
        "BLOCKED",
        "DEGRADED",
        "FAULT",
    }:
        return True

    incidents = triage.get("incidents")
    if not isinstance(incidents, list):
        return False
    for item in incidents:
        if not isinstance(item, Mapping):
            continue
        if str(item.get("severity")) in {
            "CRITICAL",
            "HIGH",
        }:
            return True
        if triage.get("analysis_profile") and str(item.get("severity")) == "MEDIUM":
            return True
    return False


def _incident_replay_window(
    triage: Mapping[str, object],
) -> ReplayWindow:
    root = triage.get("root_cause_candidate")
    tick_id = (
        root.get("tick_id")
        if isinstance(root, Mapping)
        else None
    )
    first_tick = _nested_int(
        triage, "ticks", "first_tick_id"
    )
    last_tick = _nested_int(
        triage, "ticks", "last_tick_id"
    )

    if not isinstance(tick_id, int):
        return ReplayWindow(
            requested_start_tick_id=first_tick,
            requested_end_tick_id=(
                min(last_tick, first_tick + 10)
                if first_tick is not None
                and last_tick is not None
                else None
            ),
        )

    layer = (
        root.get("layer")
        if isinstance(root, Mapping)
        else None
    )
    kind = (
        root.get("kind")
        if isinstance(root, Mapping)
        else None
    )
    reason = (
        str(root.get("reason") or "")
        if isinstance(root, Mapping)
        else ""
    )

    # Stateful localization failure needs history, not only +/-3 ticks.
    if (
        kind == "MOTION_BLOCKED"
        and (
            layer == "L9"
            or "LOCALIZATION" in reason.upper()
        )
    ):
        start = max(0, tick_id - 50)
        if first_tick is not None:
            start = max(first_tick, start)
        end = tick_id + 10
        if last_tick is not None:
            end = min(last_tick, end)
        return ReplayWindow(
            requested_start_tick_id=start,
            requested_end_tick_id=end,
            start_layer="L1",
            end_layer="L12",
        )

    return ReplayWindow(
        requested_start_tick_id=max(0, tick_id - 3),
        requested_end_tick_id=tick_id + 3,
        start_layer="L1",
        end_layer="L12",
    )


def _build_agent_brief(
    capture_path: str | Path,
    inspect: Mapping[str, object],
    triage: Mapping[str, object],
    replay: Mapping[str, object] | None,
    diagnosis: Mapping[str, object],
    slice_info: Mapping[str, object],
    *,
    max_bytes: int,
) -> dict[str, object]:
    root = (
        diagnosis.get("root_cause")
        if isinstance(diagnosis.get("root_cause"), Mapping)
        else {}
    )
    incidents = (
        triage.get("incidents")
        if isinstance(triage.get("incidents"), list)
        else []
    )
    root_layer = (
        root.get("layer") if isinstance(root, Mapping) else None
    )
    source_files = _source_hints(
        str(root_layer)
        if isinstance(root_layer, str)
        else None,
        replay,
        reason=(
            str(root.get("reason") or "")
            if isinstance(root, Mapping)
            else ""
        ),
    )
    brief: dict[str, object] = {
        "schema": AGENT_BRIEF_SCHEMA,
        "status": diagnosis.get("status"),
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get(
            "diagnosis_status"
        ),
        "evidence_status": diagnosis.get(
            "evidence_status"
        ),
        "behavior_status": diagnosis.get(
            "behavior_status"
        ),
        "capture": {
            "path": str(Path(capture_path).resolve()),
            "capture_status": diagnosis.get("capture_status"),
            "integrity_complete": (
                diagnosis.get(
                    "capture_integrity", {}
                ).get("complete")
                if isinstance(
                    diagnosis.get("capture_integrity"),
                    Mapping,
                )
                else None
            ),
        },
        "root_cause": root,
        "replay": {
            "status": diagnosis.get("replay_status"),
            "first_divergence": _prune_large(
                diagnosis.get("first_divergence")
            ),
            "bridge": (
                replay.get("mcap_bridge")
                if isinstance(replay, Mapping)
                else None
            ),
        },
        "top_incidents": [
            _prune_large(item) for item in incidents[:8]
        ],
        "metrics": diagnosis.get("metrics"),
        "source_files": source_files,
        "evidence_slice": dict(slice_info),
        "next_queries": _suggest_queries(root),
        "contract": {
            "claim_policy": (
                "PROVEN only for direct capture/replay evidence; "
                "upstream hypotheses stay INDICATED/NOT_PROVEN"
            ),
            "status_semantics": (
                "PASS=no actionable finding; "
                "FINDING=evidence valid but behaviour/problem found; "
                "WARNING=sampled low-level observation only; "
                "FAIL=evidence/replay gate failed"
            ),
            "mcap_is_authority": True,
            "agent_output_is_derived": True,
        },
    }
    return _fit_json_budget(brief, max_bytes)


def _build_gui_manifest(
    inspect: Mapping[str, object],
    triage: Mapping[str, object],
    diagnosis: Mapping[str, object],
    timeline_path: Path,
    slice_path: Path,
) -> dict[str, object]:
    return {
        "schema": GUI_MANIFEST_SCHEMA,
        "status": diagnosis.get("status"),
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get(
            "diagnosis_status"
        ),
        "evidence_status": diagnosis.get(
            "evidence_status"
        ),
        "behavior_status": diagnosis.get(
            "behavior_status"
        ),
        "capture": {
            "status": diagnosis.get("capture_status"),
            "integrity": diagnosis.get(
                "capture_integrity"
            ),
            "topics": inspect.get("topics"),
        },
        "root_cause": diagnosis.get("root_cause"),
        "incidents": triage.get("incidents"),
        "tracks": [
            "pose.x_m",
            "pose.y_m",
            "pose.yaw_rad",
            "pose.covariance_trace",
            "navigation_progress",
            "requested_motion.v_mps",
            "constrained_motion.v_mps",
            "safety_decision",
            "l9_constraints",
        ],
        "artifacts": {
            "timeline_ndjson": timeline_path.name,
            "interesting_slice_ndjson": slice_path.name,
            "diagnosis_json": "diagnosis.json",
            "replay_json": "replay_result.json" if diagnosis.get("replay_status") != "NOT_APPLICABLE" else None,
        },
    }


def _build_evidence_index(
    reader: McapReader,
    destination: Path,
    artifact_paths: Sequence[Path],
    diagnosis: Mapping[str, object],
) -> dict[str, object]:
    artifacts = {
        path.name: {
            "sha256": _sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        for path in artifact_paths
    }
    payload: dict[str, object] = {
        "schema": EVIDENCE_INDEX_SCHEMA,
        "run_id": destination.name,
        "created_at_utc": time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
        ),
        "status": diagnosis.get("status"),
        "analysis_profile": diagnosis.get("analysis_profile"),
        "diagnosis_status": diagnosis.get(
            "diagnosis_status"
        ),
        "evidence_status": diagnosis.get(
            "evidence_status"
        ),
        "behavior_status": diagnosis.get(
            "behavior_status"
        ),
        "authority_capture": {
            "path": str(reader.path.resolve()),
            "sha256": reader.sha256(),
            "container": "MCAP",
        },
        "root_cause": diagnosis.get("root_cause"),
        "artifacts": artifacts,
    }
    payload["evidence_sha256"] = _payload_sha256(
        payload, "evidence_sha256"
    )
    return payload


def _source_hints(
    layer: str | None,
    replay: Mapping[str, object] | None,
    *,
    reason: str = "",
) -> list[dict[str, object]]:
    ordered: list[str] = []

    if "LOCALIZATION_DEGRADED" in reason.upper():
        ordered.extend(
            [
                "v3/layers/l3_state_estimation.py",
                "v3/layers/l9_operational_constraints.py",
                "v3/adapters/latest_lidar.py",
                "v3/adapters/live_lidar.py",
                "v3/lidar_estimator.py",
            ]
        )
    elif layer in LAYER_SOURCE:
        index = LAYER_ORDER.index(layer)
        for candidate in LAYER_ORDER[
            max(0, index - 1) :
            min(len(LAYER_ORDER), index + 2)
        ]:
            ordered.append(LAYER_SOURCE[candidate])

    ordered.extend(
        [
            "v3/engine.py",
            "v3/execution.py",
            "STRUKTURALIS_RETEGEK_V3.md",
        ]
    )

    source_first = (
        replay.get("source_first")
        if isinstance(replay, Mapping)
        else None
    )
    files = (
        source_first.get("files")
        if isinstance(source_first, Mapping)
        else None
    )
    result: list[dict[str, object]] = []
    seen: set[str] = set()

    for path in ordered:
        if path in seen:
            continue
        seen.add(path)
        row = (
            files.get(path)
            if isinstance(files, Mapping)
            else None
        )
        result.append(
            {
                "path": path,
                "sha256": (
                    row.get("sha256")
                    if isinstance(row, Mapping)
                    else None
                ),
                "capture_baseline_match": (
                    row.get("capture_baseline_match")
                    if isinstance(row, Mapping)
                    else None
                ),
            }
        )
    return result


def _suggest_queries(
    root: Mapping[str, object],
) -> list[str]:
    tick = root.get("tick_id")
    layer = root.get("layer")
    result = []

    if isinstance(tick, int):
        layers = ",".join(
            _layer_neighborhood(
                str(layer)
                if isinstance(layer, str)
                else None
            )
        )
        before = 50 if layer == "L9" else 3
        result.append(
            "python3 -m v3.test_hub_v2 query "
            f"<capture.mcap> --ticks {max(0,tick-before)}:{tick+10} "
            f"--layers {layers}"
        )
        result.append(
            "python3 -m v3.test_hub_v2 query "
            f"<capture.mcap> --ticks {tick}:{tick} "
            "--include-inputs --full"
        )

    result.append(
        "python3 -m v3.test_hub_v2 query "
        "<capture.mcap> --topic /r2b4/raw_lidar "
        "--full --max-bytes 2097152"
    )
    return result


def _reduce_tick(
    tick: Mapping[str, object],
    *,
    layers: Sequence[str],
    include_inputs: bool,
) -> dict[str, object]:
    result: dict[str, object] = {
        "record_type": tick.get("record_type"),
        "tick_id": tick.get("tick_id"),
        "monotonic_ns": tick.get("monotonic_ns"),
    }
    expected = tick.get("expected")
    if isinstance(expected, Mapping):
        raw_layers = expected.get("layers")
        selected_layers = {}
        if isinstance(raw_layers, Mapping):
            for layer in layers:
                if layer in raw_layers:
                    selected_layers[layer] = raw_layers[layer]
        result["expected"] = {
            "fault_layer": expected.get("fault_layer"),
            "layers": selected_layers,
            **(
                {
                    "writer_failure": expected.get(
                        "writer_failure"
                    )
                }
                if expected.get("writer_failure") is not None
                else {}
            ),
        }
    if tick.get("edge_fault") is not None:
        result["edge_fault"] = tick.get("edge_fault")
    if include_inputs and tick.get("inputs") is not None:
        result["inputs"] = tick.get("inputs")
    if tick.get("tick_evidence") is not None:
        result["tick_evidence"] = tick.get(
            "tick_evidence"
        )
    return result


def _prune_large(
    value: object, *, depth: int = 0
) -> object:
    if depth > 7:
        return "<depth-truncated>"
    if isinstance(value, Mapping):
        return {
            str(key): _prune_large(
                item, depth=depth + 1
            )
            for key, item in value.items()
        }
    if isinstance(value, list):
        if len(value) > 64:
            head = [
                _prune_large(item, depth=depth + 1)
                for item in value[:24]
            ]
            tail = [
                _prune_large(item, depth=depth + 1)
                for item in value[-8:]
            ]
            return (
                head
                + [{"__omitted_items__": len(value) - 32}]
                + tail
            )
        return [
            _prune_large(item, depth=depth + 1)
            for item in value
        ]
    if isinstance(value, tuple):
        return _prune_large(
            list(value), depth=depth
        )
    return value


def _fit_json_budget(
    payload: dict[str, object], max_bytes: int
) -> dict[str, object]:
    if max_bytes < 4096:
        raise ValueError(
            "agent max_bytes must be at least 4096"
        )
    result = dict(payload)
    result["budget"] = {
        "max_bytes": max_bytes,
        "truncated": False,
    }

    def size() -> int:
        return len(
            json.dumps(
                result,
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        )

    if size() <= max_bytes:
        result["budget"]["actual_bytes"] = size()  # type: ignore[index]
        return result

    incidents = result.get("top_incidents")
    if isinstance(incidents, list):
        while (
            len(incidents) > 2
            and size() > max_bytes
        ):
            incidents.pop()

    metrics = result.get("metrics")
    if (
        isinstance(metrics, Mapping)
        and size() > max_bytes
    ):
        result["metrics"] = {
            "ticks": metrics.get("ticks"),
            "timing": metrics.get("timing"),
            "motion": metrics.get("motion"),
            "safety": metrics.get("safety"),
        }

    if size() > max_bytes:
        result["source_files"] = list(
            result.get("source_files", [])
        )[:3]

    if size() > max_bytes:
        replay = result.get("replay")
        result["replay"] = {
            "status": (
                replay.get("status")
                if isinstance(replay, Mapping)
                else None
            ),
            "first_divergence": (
                replay.get("first_divergence")
                if isinstance(replay, Mapping)
                else None
            ),
        }

    result["budget"] = {
        "max_bytes": max_bytes,
        "truncated": True,
    }
    result["budget"]["actual_bytes"] = size()  # type: ignore[index]

    if size() > max_bytes:
        result = {
            "schema": AGENT_BRIEF_SCHEMA,
            "status": payload.get("status"),
            "diagnosis_status": payload.get(
                "diagnosis_status"
            ),
            "evidence_status": payload.get(
                "evidence_status"
            ),
            "behavior_status": payload.get(
                "behavior_status"
            ),
            "capture": payload.get("capture"),
            "root_cause": payload.get("root_cause"),
            "top_incidents": list(
                payload.get("top_incidents", [])
            )[:1],
            "next_queries": payload.get("next_queries"),
            "budget": {
                "max_bytes": max_bytes,
                "truncated": True,
                "hard_compaction": True,
            },
        }
        result["budget"]["actual_bytes"] = len(  # type: ignore[index]
            json.dumps(
                result, separators=(",", ":")
            ).encode("utf-8")
        )
    return result


def _layer_neighborhood(
    layer: str | None,
) -> tuple[str, ...]:
    if layer not in LAYER_ORDER:
        return (
            "L2",
            "L3",
            "L5",
            "L6",
            "L8",
            "L9",
            "L12",
        )
    index = LAYER_ORDER.index(layer)
    values = list(
        LAYER_ORDER[
            max(0, index - 2) :
            min(len(LAYER_ORDER), index + 2)
        ]
    )
    if "L12" not in values:
        values.append("L12")
    return tuple(values)


def _channel_count(
    reader: McapReader, channel_id: int
) -> int | None:
    counts = reader.statistics.get(
        "channel_message_counts"
    )
    if isinstance(counts, Mapping):
        value = counts.get(channel_id)
        return int(value) if isinstance(value, int) else None
    return None


def _final_event(
    reader: McapReader,
) -> dict[str, object]:
    result: dict[str, object] = {}
    for _message, payload in reader.iter_json_messages(
        topics=(EVENT_TOPIC,)
    ):
        if (
            isinstance(payload, Mapping)
            and payload.get("event_type")
            == "capture_finalized"
        ):
            result = dict(payload)
    return result


def _prepare_output_dir(
    path_value: str | Path,
) -> Path:
    path = Path(path_value)
    if path.is_symlink():
        raise TestHubV2Error(
            "output directory must not be a symlink"
        )
    if (
        path.exists()
        and (
            not path.is_dir()
            or any(path.iterdir())
        )
    ):
        raise TestHubV2Error(
            "output directory must be absent or empty"
        )
    path.mkdir(parents=True, exist_ok=True)
    return path


def _write_json(
    payload: Mapping[str, object], path: Path
) -> Path:
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.tmp"
    )
    temporary.write_text(
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _read_json_object(
    path: Path,
) -> dict[str, object]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise TestHubV2Error(
            f"invalid JSON: {path}"
        ) from exc
    if not isinstance(value, dict):
        raise TestHubV2Error(
            "JSON root must be an object"
        )
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(
            lambda: handle.read(1024 * 1024), b""
        ):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(
    payload: Mapping[str, object], key: str
) -> str:
    unsigned = dict(payload)
    unsigned.pop(key, None)
    return hashlib.sha256(
        json.dumps(
            unsigned,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def _nested_int(
    value: Mapping[str, object],
    parent: str,
    child: str,
) -> int | None:
    row = value.get(parent)
    item = (
        row.get(child)
        if isinstance(row, Mapping)
        else None
    )
    return int(item) if isinstance(item, int) else None


def _parse_tick_range(
    value: str,
) -> tuple[int | None, int | None]:
    if ":" not in value:
        number = int(value)
        return number, number
    start_text, end_text = value.split(":", 1)
    return (
        int(start_text) if start_text else None,
        int(end_text) if end_text else None,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__
    )
    commands = parser.add_subparsers(
        dest="command", required=True
    )

    inspect_parser = commands.add_parser(
        "inspect",
        help="cheap MCAP/integrity summary",
    )
    inspect_parser.add_argument("capture_path")
    inspect_parser.add_argument(
        "--deep",
        action="store_true",
        help="verify every chunk CRC",
    )

    diagnose_parser = commands.add_parser(
        "diagnose",
        help="build full run-bound evidence",
    )
    diagnose_parser.add_argument("capture_path")
    diagnose_parser.add_argument(
        "--output-dir", required=True
    )
    diagnose_parser.add_argument(
        "--replay",
        choices=("off", "incident", "full"),
        default="incident",
    )
    diagnose_parser.add_argument(
        "--project-root", default="."
    )
    diagnose_parser.add_argument(
        "--capture-source-manifest"
    )
    diagnose_parser.add_argument(
        "--agent-max-bytes",
        type=int,
        default=DEFAULT_AGENT_MAX_BYTES,
    )
    diagnose_parser.add_argument(
        "--slice-max-bytes",
        type=int,
        default=DEFAULT_SLICE_MAX_BYTES,
    )

    agent_parser = commands.add_parser(
        "agent",
        help="small token-budgeted diagnosis on stdout",
    )
    agent_parser.add_argument("capture_path")
    agent_parser.add_argument(
        "--no-replay", action="store_true"
    )
    agent_parser.add_argument(
        "--project-root", default="."
    )
    agent_parser.add_argument(
        "--max-bytes",
        type=int,
        default=DEFAULT_AGENT_MAX_BYTES,
    )

    query_parser = commands.add_parser(
        "query",
        help="precise bounded evidence extraction",
    )
    query_parser.add_argument("capture_path")
    query_parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
    )
    query_parser.add_argument("--ticks")
    query_parser.add_argument(
        "--start-ns", type=int
    )
    query_parser.add_argument(
        "--end-ns", type=int
    )
    query_parser.add_argument(
        "--layers", default=""
    )
    query_parser.add_argument(
        "--include-inputs",
        action="store_true",
    )
    query_parser.add_argument(
        "--full", action="store_true"
    )
    query_parser.add_argument(
        "--max-bytes",
        type=int,
        default=2 * 1024 * 1024,
    )

    verify_parser = commands.add_parser(
        "verify-evidence"
    )
    verify_parser.add_argument("index_path")
    return parser


def main(
    argv: Sequence[str] | None = None,
) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "inspect":
            output = inspect_mcap(
                args.capture_path, deep=args.deep
            )
        elif args.command == "diagnose":
            output = diagnose_run(
                args.capture_path,
                args.output_dir,
                replay_mode=args.replay,
                project_root=args.project_root,
                capture_source_manifest_path=(
                    args.capture_source_manifest
                ),
                agent_max_bytes=args.agent_max_bytes,
                slice_max_bytes=args.slice_max_bytes,
            )
        elif args.command == "agent":
            output = build_agent_brief_only(
                args.capture_path,
                replay=not args.no_replay,
                project_root=args.project_root,
                max_bytes=args.max_bytes,
            )
        elif args.command == "query":
            start_tick = end_tick = None
            if args.ticks:
                start_tick, end_tick = (
                    _parse_tick_range(args.ticks)
                )
            topics = args.topics or [TICK_TOPIC]
            layers = tuple(
                item
                for item in args.layers.split(",")
                if item
            )
            output = query_capture(
                args.capture_path,
                topics=topics,
                start_tick_id=start_tick,
                end_tick_id=end_tick,
                start_ns=args.start_ns,
                end_ns=args.end_ns,
                layers=layers,
                include_inputs=args.include_inputs,
                full=args.full,
                max_bytes=args.max_bytes,
            )
        else:
            output = verify_evidence(
                args.index_path
            )

        print(
            json.dumps(
                output,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
        )
        # FINDING means the diagnostic tool worked and found a robot
        # behaviour problem. It is not a Test Hub execution failure.
        return (
            0
            if output.get("status")
            in {"PASS", "FINDING"}
            else 2
        )
    except (
        McapReadError,
        McapReplayBridgeError,
        TestHubV2Error,
        OSError,
        TypeError,
        ValueError,
        RuntimeError,
    ) as exc:
        print(
            json.dumps(
                {
                    "status": "ERROR",
                    "error": str(exc),
                },
                ensure_ascii=False,
                indent=2,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "AGENT_BRIEF_SCHEMA",
    "EVIDENCE_INDEX_SCHEMA",
    "GUI_MANIFEST_SCHEMA",
    "TEST_HUB_SCHEMA",
    "TestHubV2Error",
    "build_agent_brief_only",
    "diagnose_run",
    "inspect_mcap",
    "query_capture",
    "verify_evidence",
    "write_interesting_slice",
]
