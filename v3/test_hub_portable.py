"""Portable, agent-facing evidence export for R2B4 MCAP captures.

This module is deliberately offline and read-only.  It never owns robot control,
hardware, lifecycle or safety.  The MCAP stays the authority; this module creates
small derived artifacts that are useful when the MCAP is too large to upload.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
import sys
import time
from collections import Counter, defaultdict
from collections.abc import Mapping, Sequence
from pathlib import Path

from .pytest_profiles import get_pytest_profile, resolve_pytest_files, resolve_pytest_targets
from .mcap_reader import McapReadError, McapReader, RAW_LIDAR_TOPIC, TICK_TOPIC
from .mcap_replay_bridge import McapReplayBridgeError, ReplayWindow, replay_mcap
from .test_hub_v2 import write_interesting_slice
from .test_hub_runtime_correlation import slow_tick_correlation_from_inspect

PORTABLE_SCHEMA = "R2B4_TEST_HUB_PORTABLE_V1"
REPLAY_SWEEP_SCHEMA = "R2B4_REPLAY_SWEEP_V1"
PYTEST_SCHEMA = "R2B4_TEST_HUB_PYTEST_V1"
DEFAULT_REPLAY_WINDOW_TICKS = 3_000
DEFAULT_MAX_INCIDENT_GROUPS = 24
DEFAULT_RAW_INCIDENT_BYTES = 8 * 1024 * 1024
DEFAULT_INCIDENT_SLICE_BYTES = 768 * 1024

_SEVERITY_RANK = {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3}
_LIDAR_PHYSICAL_KINDS = {
    "lidar_health",
    "lidar_safety_clearance",
    "lidar_local_points",
}


def _mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _sequence(value: object) -> Sequence[object]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return value
    return ()


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _field_values(value: object) -> dict[str, object]:
    result: dict[str, object] = {}
    for item in _sequence(value):
        if not isinstance(item, Mapping):
            continue
        key = item.get("key")
        if isinstance(key, str):
            result[key] = item.get("value")
    return result


def _small_scalars(value: Mapping[str, object], *, max_items: int = 48) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, raw in value.items():
        if len(result) >= max_items:
            break
        if raw is None or isinstance(raw, (str, bool, int, float)):
            result[str(key)] = raw
        elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)) and len(raw) <= 8:
            if all(item is None or isinstance(item, (str, bool, int, float)) for item in raw):
                result[str(key)] = list(raw)
    return result


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, payload: Mapping[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _safe_name(value: object) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "incident")).strip("_.")
    return text[:96] or "incident"


def _tick_lidar_samples(tick: Mapping[str, object]) -> tuple[set[int], list[dict[str, object]]]:
    refs: set[int] = set()
    rows: list[dict[str, object]] = []
    inputs = _mapping(tick.get("inputs"))
    raw = _mapping(inputs.get("raw_devices"))
    for sample in _sequence(raw.get("samples")):
        if not isinstance(sample, Mapping):
            continue
        kind = sample.get("kind")
        sequence = sample.get("sequence")
        fields = _field_values(sample.get("values"))
        source_revision = fields.get("source_raw_scan_id")
        revision: int | None = None
        if kind in _LIDAR_PHYSICAL_KINDS and isinstance(sequence, int) and sequence > 0:
            revision = sequence
        elif kind == "lidar_matcher_diagnostics" and isinstance(source_revision, int) and source_revision > 0:
            revision = source_revision
        if revision is None:
            continue
        refs.add(revision)
        rows.append({
            "revision": revision,
            "kind": kind,
            "device_id": sample.get("device_id"),
            "captured_monotonic_ns": sample.get("captured_monotonic_ns"),
            "fields": _small_scalars(fields),
        })
    return refs, rows


def build_lidar_tick_index(reader: McapReader) -> tuple[dict[int, set[int]], dict[int, dict[str, object]]]:
    """Return tick->raw revisions and compact per-revision tick diagnostics."""
    tick_refs: dict[int, set[int]] = {}
    revisions: dict[int, dict[str, object]] = {}
    for message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if not isinstance(payload, Mapping):
            continue
        tick_id = payload.get("tick_id")
        if not isinstance(tick_id, int):
            tick_id = message.sequence
        refs, samples = _tick_lidar_samples(payload)
        tick_refs[tick_id] = refs
        for revision in refs:
            row = revisions.setdefault(revision, {
                "first_tick": tick_id,
                "last_tick": tick_id,
                "referenced_tick_count": 0,
                "samples": {},
            })
            row["first_tick"] = min(int(row["first_tick"]), tick_id)
            row["last_tick"] = max(int(row["last_tick"]), tick_id)
            row["referenced_tick_count"] = int(row["referenced_tick_count"]) + 1
        for sample in samples:
            revision = int(sample["revision"])
            row = revisions.setdefault(revision, {
                "first_tick": tick_id,
                "last_tick": tick_id,
                "referenced_tick_count": 0,
                "samples": {},
            })
            kind = str(sample.get("kind") or "unknown")
            sample_map = _mapping(row.get("samples"))
            mutable = dict(sample_map)
            mutable[kind] = {
                "device_id": sample.get("device_id"),
                "captured_monotonic_ns": sample.get("captured_monotonic_ns"),
                "fields": sample.get("fields"),
            }
            row["samples"] = mutable
    return tick_refs, revisions


def write_lidar_summary(
    capture_path: str | Path,
    output_path: str | Path,
    *,
    reader: McapReader | None = None,
) -> tuple[dict[str, object], dict[int, set[int]]]:
    """Write one compact line per raw scan; never copy point arrays."""
    source = reader or McapReader(capture_path)
    tick_refs, revision_info = build_lidar_tick_index(source)
    rows: list[dict[str, object]] = []
    point_counts: list[int] = []
    health_counts: Counter[str] = Counter()
    truncated = 0

    for message, payload in source.iter_json_messages(topics=(RAW_LIDAR_TOPIC,)):
        if not isinstance(payload, Mapping):
            continue
        revision = payload.get("revision")
        if not isinstance(revision, int):
            revision = message.sequence
        point_count = payload.get("source_point_count")
        if isinstance(point_count, int):
            point_counts.append(point_count)
        health = str(payload.get("health") or "UNKNOWN")
        health_counts[health] += 1
        is_truncated = bool(payload.get("points_truncated"))
        truncated += int(is_truncated)
        scan_start = payload.get("scan_start_monotonic_ns")
        scan_end = payload.get("scan_end_monotonic_ns")
        duration_ms = None
        if isinstance(scan_start, int) and isinstance(scan_end, int) and scan_end >= scan_start:
            duration_ms = (scan_end - scan_start) / 1_000_000.0
        info = revision_info.get(revision, {})
        rows.append({
            "row_type": "scan",
            "revision": revision,
            "measurement_monotonic_ns": payload.get("measurement_monotonic_ns"),
            "scan_start_monotonic_ns": scan_start,
            "scan_end_monotonic_ns": scan_end,
            "scan_duration_ms": duration_ms,
            "health": health,
            "source_point_count": point_count,
            "points_truncated": is_truncated,
            "summary": payload.get("summary") if isinstance(payload.get("summary"), Mapping) else {},
            "first_tick": info.get("first_tick"),
            "last_tick": info.get("last_tick"),
            "referenced_tick_count": info.get("referenced_tick_count", 0),
            "tick_diagnostics": info.get("samples", {}),
        })

    header = {
        "row_type": "header",
        "schema": PORTABLE_SCHEMA,
        "capture": Path(capture_path).name,
        "scan_count": len(rows),
        "point_count_min": min(point_counts) if point_counts else None,
        "point_count_max": max(point_counts) if point_counts else None,
        "point_count_mean": (sum(point_counts) / len(point_counts)) if point_counts else None,
        "health_counts": dict(sorted(health_counts.items())),
        "truncated_scan_count": truncated,
        "point_arrays_in_file": False,
        "note": "Full raw point arrays stay in MCAP except bounded incident slices.",
    }
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(header, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")
    return {
        "path": str(path.resolve()),
        "scan_count": len(rows),
        "size_bytes": path.stat().st_size,
        "point_arrays_in_file": False,
    }, tick_refs


def group_incidents(
    incidents: Sequence[object], *, max_groups: int = DEFAULT_MAX_INCIDENT_GROUPS
) -> list[dict[str, object]]:
    """Collapse repeated identical incidents without hiding their count/span."""
    grouped: dict[tuple[str, str, str], dict[str, object]] = {}
    for raw in incidents:
        if not isinstance(raw, Mapping):
            continue
        key = (
            str(raw.get("category") or "UNKNOWN"),
            str(raw.get("reason") or "UNKNOWN"),
            str(raw.get("layer") or "UNKNOWN"),
        )
        tick = raw.get("tick_id")
        item = grouped.get(key)
        if item is None:
            grouped[key] = {
                "category": key[0],
                "reason": key[1],
                "layer": key[2],
                "severity": str(raw.get("severity") or "LOW"),
                "count": 1,
                "first_tick": tick if isinstance(tick, int) else None,
                "last_tick": tick if isinstance(tick, int) else None,
                "representative": dict(raw),
            }
            continue
        item["count"] = int(item["count"]) + 1
        if isinstance(tick, int):
            if item.get("first_tick") is None:
                item["first_tick"] = tick
            item["last_tick"] = tick
        severity = str(raw.get("severity") or "LOW")
        if _SEVERITY_RANK.get(severity, 9) < _SEVERITY_RANK.get(str(item.get("severity")), 9):
            item["severity"] = severity
            item["representative"] = dict(raw)

    rows = list(grouped.values())
    rows.sort(key=lambda row: (
        _SEVERITY_RANK.get(str(row.get("severity")), 9),
        int(row.get("first_tick")) if isinstance(row.get("first_tick"), int) else 2**63 - 1,
        str(row.get("reason")),
    ))
    return rows[:max_groups]


def write_incident_slices(
    reader: McapReader,
    output_dir: str | Path,
    incident_groups: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    result: list[dict[str, object]] = []
    for index, group in enumerate(incident_groups, 1):
        tick = group.get("first_tick")
        if not isinstance(tick, int):
            continue
        layer = group.get("layer")
        path = destination / f"{index:02d}_{_safe_name(group.get('reason'))}.ndjson"
        info = write_interesting_slice(
            reader,
            path,
            around_tick=tick,
            root_layer=str(layer) if isinstance(layer, str) and layer.startswith("L") else None,
            before=10,
            after=10,
            max_bytes=DEFAULT_INCIDENT_SLICE_BYTES,
        )
        result.append({
            "group_index": index,
            "category": group.get("category"),
            "reason": group.get("reason"),
            "count": group.get("count"),
            "first_tick": group.get("first_tick"),
            "last_tick": group.get("last_tick"),
            "path": path.name,
            "rows": info.get("rows"),
            "truncated": info.get("truncated"),
        })
    return result


def _incident_target_revisions(
    tick_refs: Mapping[int, set[int]], tick: int, *, before_ticks: int = 5, after_ticks: int = 5
) -> set[int]:
    result: set[int] = set()
    for tick_id in range(max(0, tick - before_ticks), tick + after_ticks + 1):
        result.update(tick_refs.get(tick_id, set()))
    expanded: set[int] = set(result)
    for revision in tuple(result):
        expanded.update(candidate for candidate in range(max(1, revision - 2), revision + 3))
    return expanded


def write_raw_lidar_incident_slices(
    reader: McapReader,
    output_dir: str | Path,
    incident_groups: Sequence[Mapping[str, object]],
    tick_refs: Mapping[int, set[int]],
    *,
    max_groups: int = 12,
    max_bytes_per_group: int = DEFAULT_RAW_INCIDENT_BYTES,
) -> list[dict[str, object]]:
    """Preserve full point arrays only near representative incidents."""
    selected: list[tuple[int, Mapping[str, object], set[int]]] = []
    all_revisions: set[int] = set()
    for index, group in enumerate(incident_groups, 1):
        if len(selected) >= max_groups:
            break
        tick = group.get("first_tick")
        if not isinstance(tick, int):
            continue
        revisions = _incident_target_revisions(tick_refs, tick)
        if not revisions:
            continue
        selected.append((index, group, revisions))
        all_revisions.update(revisions)

    payloads: dict[int, tuple[int, object]] = {}
    if all_revisions:
        for message, payload in reader.iter_json_messages(topics=(RAW_LIDAR_TOPIC,)):
            revision = payload.get("revision") if isinstance(payload, Mapping) else None
            if not isinstance(revision, int):
                revision = message.sequence
            if revision in all_revisions:
                payloads[revision] = (message.log_time_ns, payload)

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    result: list[dict[str, object]] = []
    for index, group, revisions in selected:
        path = destination / f"{index:02d}_{_safe_name(group.get('reason'))}.ndjson"
        written = 0
        rows = 0
        truncated = False
        with path.open("x", encoding="utf-8") as handle:
            header = {
                "row_type": "header",
                "schema": PORTABLE_SCHEMA,
                "incident_reason": group.get("reason"),
                "incident_tick": group.get("first_tick"),
                "requested_revisions": sorted(revisions),
                "point_arrays_in_file": True,
            }
            first = json.dumps(header, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n"
            handle.write(first)
            written += len(first.encode("utf-8"))
            for revision in sorted(revisions):
                pair = payloads.get(revision)
                if pair is None:
                    continue
                line = json.dumps({
                    "row_type": "raw_lidar",
                    "revision": revision,
                    "monotonic_ns": pair[0],
                    "payload": pair[1],
                }, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n"
                size = len(line.encode("utf-8"))
                if written + size > max_bytes_per_group:
                    truncated = True
                    break
                handle.write(line)
                written += size
                rows += 1
        result.append({
            "group_index": index,
            "reason": group.get("reason"),
            "path": path.name,
            "raw_scan_count": rows,
            "truncated": truncated,
            "size_bytes": written,
        })
    return result


def replay_sweep(
    capture_path: str | Path,
    *,
    project_root: str | Path | None = None,
    window_ticks: int = DEFAULT_REPLAY_WINDOW_TICKS,
    authority_sha256: str | None = None,
    verified_final_event: Mapping[str, object] | None = None,
    structure_already_verified: bool = False,
) -> dict[str, object]:
    """Replay a long capture in bounded windows while hashing authority only twice."""
    if window_ticks <= 0 or window_ticks > 3_500:
        raise ValueError("window_ticks must be in 1..3500")
    path = Path(capture_path)
    reader = McapReader(path)
    if not structure_already_verified:
        structure = reader.inspect(verify_chunks=True)
        if not structure.valid:
            return {
                "schema": REPLAY_SWEEP_SCHEMA,
                "status": "ERROR",
                "error": "MCAP structural/deep CRC preflight failed",
                "windows": [],
            }
    if verified_final_event is None:
        try:
            verified_final = reader.capture_integrity()
        except (McapReadError, OSError, TypeError, ValueError) as exc:
            return {
                "schema": REPLAY_SWEEP_SCHEMA,
                "status": "ERROR",
                "error": f"capture integrity preflight failed: {exc}",
                "windows": [],
            }
    else:
        verified_final = dict(verified_final_event)
    initial_sha = authority_sha256 or _sha256_file(path)
    tick_times: dict[int, int] = {}
    first_tick: int | None = None
    last_tick: int | None = None
    tick_count = 0
    for message in reader.iter_messages(topics=(TICK_TOPIC,)):
        first_tick = message.sequence if first_tick is None else first_tick
        last_tick = message.sequence
        tick_times[message.sequence] = message.log_time_ns
        tick_count += 1
    if first_tick is None or last_tick is None:
        return {
            "schema": REPLAY_SWEEP_SCHEMA,
            "status": "ERROR",
            "error": "capture contains no ticks",
            "windows": [],
        }

    windows: list[dict[str, object]] = []
    first_divergence: object | None = None
    any_error = False
    any_mismatch = False
    start = first_tick
    while start <= last_tick:
        end = min(last_tick, start + window_ticks - 1)
        start_ns = tick_times.get(start)
        end_ns = tick_times.get(end)
        row: dict[str, object] = {
            "start_tick": start,
            "end_tick": end,
            "start_monotonic_ns": start_ns,
            "end_monotonic_ns": end_ns,
        }
        try:
            replay = replay_mcap(
                path,
                window=ReplayWindow(
                    requested_start_tick_id=start,
                    requested_end_tick_id=end,
                    requested_start_ns=start_ns,
                    requested_end_ns=end_ns,
                    max_materialized_ticks=4_096,
                    max_materialized_bytes=128 * 1024 * 1024,
                ),
                project_root=project_root,
                authority_sha256=initial_sha,
                verify_authority_unchanged=False,
                verify_structure=False,
                verified_final_event=verified_final,
            )
            status = str(replay.get("status") or "ERROR")
            row["status"] = status
            bridge = _mapping(replay.get("mcap_bridge"))
            row["materialized_tick_count"] = bridge.get("materialized_tick_count")
            divergence = replay.get("first_divergence")
            if divergence is not None:
                row["first_divergence"] = divergence
                if first_divergence is None:
                    first_divergence = divergence
            if status == "MISMATCH":
                any_mismatch = True
            elif status != "MATCH":
                any_error = True
        except (McapReplayBridgeError, OSError, TypeError, ValueError, RuntimeError) as exc:
            row["status"] = "ERROR"
            row["error"] = str(exc)
            any_error = True
        windows.append(row)
        start = end + 1

    final_sha = _sha256_file(path)
    authority_unchanged = final_sha == initial_sha
    if not authority_unchanged:
        any_error = True
    status = "MISMATCH" if any_mismatch else "ERROR" if any_error else "MATCH"
    return {
        "schema": REPLAY_SWEEP_SCHEMA,
        "status": status,
        "capture": path.name,
        "capture_sha256": initial_sha,
        "authority_unchanged": authority_unchanged,
        "tick_count": tick_count,
        "first_tick": first_tick,
        "last_tick": last_tick,
        "window_ticks": window_ticks,
        "window_count": len(windows),
        "covered_tick_span": [first_tick, last_tick],
        "first_divergence": first_divergence,
        "windows": windows,
    }


def runtime_performance_summary(inspect_payload: Mapping[str, object]) -> dict[str, object]:
    final = _mapping(inspect_payload.get("final_event"))
    metrics = _mapping(final.get("metrics"))
    timing = _mapping(metrics.get("runtime_tick_timing"))
    elapsed_ns = metrics.get("elapsed_ns")
    process_cpu_ns = metrics.get("process_cpu_ns")
    period_count = timing.get("period_count")
    period_sum_ns = timing.get("period_sum_ns")
    average_period_ms = None
    average_hz = None
    if isinstance(period_count, int) and period_count > 0 and isinstance(period_sum_ns, int):
        average_period_ms = period_sum_ns / period_count / 1_000_000.0
        average_hz = 1_000.0 / average_period_ms if average_period_ms > 0 else None
    cpu_core_equivalent = None
    if isinstance(elapsed_ns, int) and elapsed_ns > 0 and isinstance(process_cpu_ns, int):
        cpu_core_equivalent = process_cpu_ns / elapsed_ns
    return {
        "schema": PORTABLE_SCHEMA,
        "parent_process": {
            "elapsed_ns": elapsed_ns,
            "process_cpu_ns": process_cpu_ns,
            "cpu_core_equivalent": cpu_core_equivalent,
            "max_rss_bytes": metrics.get("process_max_rss_bytes"),
            "write_duration_ns": metrics.get("write_duration_ns"),
        },
        "runtime_tick": {
            **dict(timing),
            "average_period_ms": average_period_ms,
            "average_hz": average_hz,
        },
        "per_core_available": False,
        "slow_tick_correlation": slow_tick_correlation_from_inspect(inspect_payload),
        "note": "Capture-derived evidence only. Slow-tick correlation is descriptive association, not a causal layer-cost claim.",
    }


def run_pytest(
    project_root: str | Path,
    *,
    scope: str = "testhub",
    timeout_s: int | None = None,
) -> dict[str, object]:
    """Run one named R2B4 pytest profile under the offline Test Hub umbrella."""
    root = Path(project_root).resolve()
    profile = get_pytest_profile(scope)
    files = resolve_pytest_files(root, scope)
    targets = resolve_pytest_targets(root, scope)
    command = [sys.executable, "-m", "pytest", "-q", *targets]
    started = time.monotonic()
    metadata = {
        "schema": PYTEST_SCHEMA,
        "scope": scope,
        "profile_marker": profile.marker,
        "profile_description": profile.description,
        "test_file_count": len(files),
    }
    try:
        completed = subprocess.run(
            command,
            cwd=root,
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
        status = "PASS" if completed.returncode == 0 else "FAIL"
        return {
            **metadata,
            "status": status,
            "exit_code": completed.returncode,
            "duration_s": time.monotonic() - started,
            "command": command,
            "stdout_tail": completed.stdout[-50_000:],
            "stderr_tail": completed.stderr[-50_000:],
        }
    except subprocess.TimeoutExpired as exc:
        return {
            **metadata,
            "status": "ERROR",
            "exit_code": None,
            "duration_s": time.monotonic() - started,
            "command": command,
            "error": f"pytest timeout after {timeout_s}s",
            "stdout_tail": (exc.stdout or "")[-50_000:] if isinstance(exc.stdout, str) else "",
            "stderr_tail": (exc.stderr or "")[-50_000:] if isinstance(exc.stderr, str) else "",
        }

def write_portable_manifest(
    destination: str | Path,
    capture_path: str | Path,
    *,
    capture_sha256: str,
    replay_sweep_payload: Mapping[str, object] | None,
    pytest_payload: Mapping[str, object] | None,
) -> Path:
    root = Path(destination)
    artifacts: dict[str, dict[str, object]] = {}
    manifest_path = root / "portable_manifest.json"
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path == manifest_path:
            continue
        relative = path.relative_to(root).as_posix()
        artifacts[relative] = {
            "size_bytes": path.stat().st_size,
            "sha256": _sha256_file(path),
        }
    payload = {
        "schema": PORTABLE_SCHEMA,
        "capture": {
            "name": Path(capture_path).name,
            "sha256": capture_sha256,
            "local_path": str(Path(capture_path).resolve()),
            "mcap_required_for_normal_remote_analysis": False,
            "mcap_remains_authority": True,
        },
        "replay_sweep_status": replay_sweep_payload.get("status") if replay_sweep_payload else "OFF",
        "pytest_status": pytest_payload.get("status") if pytest_payload else "OFF",
        "artifacts": artifacts,
        "limitations": [
            "Full raw LiDAR point arrays are exported only around representative incidents.",
            "Exact evidence outside exported incident slices remains available from the local MCAP.",
            "Per-core scheduler telemetry is not invented when the capture did not record it.",
        ],
    }
    return _write_json(manifest_path, payload)


__all__ = [
    "DEFAULT_MAX_INCIDENT_GROUPS",
    "DEFAULT_REPLAY_WINDOW_TICKS",
    "PORTABLE_SCHEMA",
    "build_lidar_tick_index",
    "group_incidents",
    "replay_sweep",
    "run_pytest",
    "runtime_performance_summary",
    "write_incident_slices",
    "write_lidar_summary",
    "write_portable_manifest",
    "write_raw_lidar_incident_slices",
]
