#!/usr/bin/env python3
"""
R2B4 live LiDAR -> L4 end-to-end latency probe.

Purpose
-------
Measure, during a real production live run, the timing chain:

    scan measurement
           ↓ A
    scan complete
           ↓ B
    NativeLidarPort snapshot update
           ↓ C
    MultiRate publication
           ↓ D
    L2 admission
           ↓ E
    L4 accepted

The tool does NOT modify repository files.  It launches the requested normal
R2B4 command with a temporary Python sitecustomize hook inherited by the
resident runtime process.  The hook passively timestamps production objects at
the relevant boundaries and appends compact NDJSON events.

Important
---------
For exact live instrumentation the resident runtime must be STARTED under this
probe.  Attaching to an already-running Python runtime cannot install these
hooks safely without modifying production code, so the tool refuses to start
if a live resident runtime is already present.

Typical usage from /home/alba/project_r2b4:

    python3 r2b4_lidar_l4_latency_probe.py -- r rc 30 c full

or after copying into tools/:

    python3 tools/r2b4_lidar_l4_latency_probe.py -- r rc 30 c full

Outputs:
    runtime/diagnostics/lidar_l4_latency_<timestamp>.events.ndjson
    runtime/diagnostics/lidar_l4_latency_<timestamp>.csv
    runtime/diagnostics/lidar_l4_latency_<timestamp>.json

The A-E labels mean:
    A = measurement -> scan complete
    B = scan complete -> NativeLidarPort snapshot construction/update boundary
    C = snapshot update -> MultiRate publication
    D = MultiRate publication -> L2 accepted
    E = L2 accepted -> L4 entry
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

SCHEMA = "R2B4_LIDAR_L4_LATENCY_PROBE_V1"
TOOL_VERSION = 1

ENV_ENABLE = "R2B4_LIDAR_L4_PROBE_ENABLE"
ENV_LOG = "R2B4_LIDAR_L4_PROBE_LOG"
ENV_ROOT = "R2B4_ROOT"

_HOOKS_INSTALLED = False


# ---------------------------------------------------------------------------
# Shared small helpers
# ---------------------------------------------------------------------------

def _json_line_write(path: str, payload: dict[str, Any]) -> None:
    """Append one compact event with O_APPEND; keep one event within one write()."""
    try:
        raw = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        try:
            os.write(fd, raw)
        finally:
            os.close(fd)
    except Exception:
        # Probe must never become control authority or break production execution.
        pass


def _probe_event(kind: str, **fields: Any) -> None:
    path = os.environ.get(ENV_LOG)
    if not path:
        return
    payload: dict[str, Any] = {
        "schema": SCHEMA,
        "tool_version": TOOL_VERSION,
        "kind": kind,
        "event_monotonic_ns": time.monotonic_ns(),
        "pid": os.getpid(),
        "process_argv0": sys.argv[0] if sys.argv else "",
    }
    payload.update(fields)
    _json_line_write(path, payload)


def _field_map(values: Iterable[Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for item in values:
        key = getattr(item, "key", None)
        if isinstance(key, str):
            result[key] = getattr(item, "value", None)
    return result


def _lidar_health_from_samples(samples: Iterable[Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for sample in samples:
        if getattr(sample, "device_id", None) != "RPLIDAR_C1":
            continue
        if getattr(sample, "kind", None) != "lidar_health":
            continue
        fields = _field_map(getattr(sample, "values", ()))
        out.append(
            {
                "revision": getattr(sample, "sequence", None),
                "captured_monotonic_ns": getattr(sample, "captured_monotonic_ns", None),
                "measurement_monotonic_ns": fields.get("measurement_monotonic_ns"),
                "scan_start_monotonic_ns": fields.get("scan_start_monotonic_ns"),
                "scan_end_monotonic_ns": fields.get("scan_end_monotonic_ns"),
                "age_ns": fields.get("age_ns"),
            }
        )
    return out


def _lidar_health_from_observations(observations: Iterable[Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for obs in observations:
        if getattr(obs, "source_device_id", None) != "RPLIDAR_C1":
            continue
        if getattr(obs, "kind", None) != "lidar_health":
            continue
        fields = _field_map(getattr(obs, "values", ()))
        out.append(
            {
                "revision": getattr(obs, "source_sequence", None),
                "captured_monotonic_ns": getattr(obs, "captured_monotonic_ns", None),
                "measurement_monotonic_ns": fields.get("measurement_monotonic_ns"),
                "scan_start_monotonic_ns": fields.get("scan_start_monotonic_ns"),
                "scan_end_monotonic_ns": fields.get("scan_end_monotonic_ns"),
                "age_ns": fields.get("age_ns"),
            }
        )
    return out


def _runtime_process() -> bool:
    text = " ".join(str(x) for x in sys.argv)
    return "v3_process_runtime.py" in text


# ---------------------------------------------------------------------------
# Hooks installed by temporary sitecustomize inside the resident runtime
# ---------------------------------------------------------------------------

def install_hooks_from_env() -> None:
    """Install passive hooks in the live resident runtime only."""
    global _HOOKS_INSTALLED
    if _HOOKS_INSTALLED:
        return
    if os.environ.get(ENV_ENABLE) != "1":
        return
    if not os.environ.get(ENV_LOG):
        return

    # Avoid replay/test-hub/CLI processes: their timing is not live-path latency.
    if not _runtime_process():
        return

    _HOOKS_INSTALLED = True
    _probe_event("probe_runtime_started")

    # --- C boundary: exact NativeLidarPort._raw_snapshot assignment.
    #
    # NativeLidarPort has normal object attribute assignment.  Intercepting only
    # the _raw_snapshot attribute gives an exact timestamp immediately AFTER the
    # production owner has installed the new immutable snapshot.  No production
    # logic is replaced; all other assignments go straight through unchanged.
    try:
        from v3.adapters.native_lidar_port import NativeLidarPort, NativeRawLidarSnapshot

        original_port_setattr = NativeLidarPort.__setattr__

        def lidar_port_setattr_probe(self: Any, name: str, value: Any) -> None:
            original_port_setattr(self, name, value)
            if name != "_raw_snapshot" or not isinstance(value, NativeRawLidarSnapshot):
                return
            try:
                revision = int(getattr(value, "raw_scan_id"))
                if revision <= 0:
                    return
                _probe_event(
                    "lidar_snapshot_update",
                    revision=revision,
                    measurement_monotonic_ns=int(getattr(value, "measurement_monotonic_ns")),
                    scan_start_monotonic_ns=int(getattr(value, "scan_start_monotonic_ns")),
                    scan_end_monotonic_ns=int(getattr(value, "scan_end_monotonic_ns")),
                )
            except Exception:
                pass

        NativeLidarPort.__setattr__ = lidar_port_setattr_probe  # type: ignore[method-assign]
    except Exception as exc:
        _probe_event("probe_hook_error", hook="NativeLidarPort.__setattr__", error=repr(exc))

    # --- D boundary: exact timestamp used by MultiRate publication.
    #
    # _PublishedSnapshot.visible_monotonic_ns is the production publication
    # boundary used when the control tick selects what is visible.
    try:
        from v3.adapters.multirate_inputs import _PublishedSnapshot

        original_published_init = _PublishedSnapshot.__init__

        def published_init_probe(self: Any, *args: Any, **kwargs: Any) -> None:
            original_published_init(self, *args, **kwargs)
            try:
                visible_ns = int(getattr(self, "visible_monotonic_ns"))
                snapshot = getattr(self, "snapshot")
                for info in _lidar_health_from_samples(getattr(snapshot, "samples", ())):
                    revision = info.get("revision")
                    if isinstance(revision, int) and revision > 0:
                        _probe_event(
                            "multirate_publication",
                            revision=revision,
                            publication_monotonic_ns=visible_ns,
                            captured_monotonic_ns=info.get("captured_monotonic_ns"),
                            measurement_monotonic_ns=info.get("measurement_monotonic_ns"),
                            scan_end_monotonic_ns=info.get("scan_end_monotonic_ns"),
                        )
            except Exception:
                pass

        _PublishedSnapshot.__init__ = published_init_probe  # type: ignore[method-assign]
    except Exception as exc:
        _probe_event("probe_hook_error", hook="_PublishedSnapshot", error=repr(exc))

    # --- L2 boundary: accepted/rejected lidar_health.
    try:
        from v3.layers.l2_admission import InputAdmission

        original_l2_call = InputAdmission.__call__

        def l2_call_probe(self: Any, frame: Any) -> Any:
            result = original_l2_call(self, frame)
            event_ns = time.monotonic_ns()
            try:
                tick_id = int(getattr(getattr(frame, "context"), "tick_id"))
                tick_ns = int(getattr(getattr(frame, "context"), "monotonic_ns"))
                for info in _lidar_health_from_observations(getattr(result, "accepted", ())):
                    revision = info.get("revision")
                    if isinstance(revision, int) and revision > 0:
                        _probe_event(
                            "l2_admitted",
                            event_override_monotonic_ns=event_ns,
                            tick_id=tick_id,
                            tick_monotonic_ns=tick_ns,
                            revision=revision,
                            captured_monotonic_ns=info.get("captured_monotonic_ns"),
                            measurement_monotonic_ns=info.get("measurement_monotonic_ns"),
                            scan_end_monotonic_ns=info.get("scan_end_monotonic_ns"),
                        )
                for rejected in getattr(result, "rejected", ()):
                    if getattr(rejected, "source_device_id", None) != "RPLIDAR_C1":
                        continue
                    revision = getattr(rejected, "source_sequence", None)
                    if not isinstance(revision, int) or revision <= 0:
                        continue
                    reason = getattr(getattr(rejected, "reason", None), "value", None)
                    if reason is None:
                        reason = str(getattr(rejected, "reason", ""))
                    _probe_event(
                        "l2_rejected",
                        event_override_monotonic_ns=event_ns,
                        tick_id=tick_id,
                        tick_monotonic_ns=tick_ns,
                        revision=revision,
                        reason=str(reason),
                        age_ns=getattr(rejected, "age_ns", None),
                    )
            except Exception:
                pass
            return result

        InputAdmission.__call__ = l2_call_probe  # type: ignore[method-assign]
    except Exception as exc:
        _probe_event("probe_hook_error", hook="InputAdmission", error=repr(exc))

    # --- L4 boundary: accepted admitted lidar_health reaches WorldModel.
    try:
        from v3.layers.l4_world_model import ShadowWorldModel

        original_l4_call = ShadowWorldModel.__call__

        def l4_call_probe(self: Any, frame: Any, estimate: Any) -> Any:
            entry_ns = time.monotonic_ns()
            infos: list[dict[str, Any]] = []
            try:
                infos = _lidar_health_from_observations(getattr(frame, "accepted", ()))
                tick_id = int(getattr(getattr(frame, "context"), "tick_id"))
                tick_ns = int(getattr(getattr(frame, "context"), "monotonic_ns"))
                for info in infos:
                    revision = info.get("revision")
                    if isinstance(revision, int) and revision > 0:
                        _probe_event(
                            "l4_accepted",
                            event_override_monotonic_ns=entry_ns,
                            tick_id=tick_id,
                            tick_monotonic_ns=tick_ns,
                            revision=revision,
                            captured_monotonic_ns=info.get("captured_monotonic_ns"),
                            measurement_monotonic_ns=info.get("measurement_monotonic_ns"),
                            scan_end_monotonic_ns=info.get("scan_end_monotonic_ns"),
                        )
            except Exception:
                pass

            result = original_l4_call(self, frame, estimate)

            try:
                if infos:
                    _probe_event(
                        "l4_output",
                        tick_id=int(getattr(getattr(frame, "context"), "tick_id")),
                        world_freshness_ns=int(getattr(result, "freshness_ns")),
                        map_revision=int(getattr(result, "map_revision")),
                        local_costmap_freshness_ns=(
                            int(getattr(getattr(result, "local_costmap"), "freshness_ns"))
                            if getattr(result, "local_costmap", None) is not None
                            else None
                        ),
                    )
            except Exception:
                pass
            return result

        ShadowWorldModel.__call__ = l4_call_probe  # type: ignore[method-assign]
    except Exception as exc:
        _probe_event("probe_hook_error", hook="ShadowWorldModel", error=repr(exc))


# ---------------------------------------------------------------------------
# Offline reduction of live probe events
# ---------------------------------------------------------------------------

def _effective_event_ns(event: dict[str, Any]) -> int | None:
    override = event.get("event_override_monotonic_ns")
    if isinstance(override, int):
        return override
    value = event.get("event_monotonic_ns")
    return value if isinstance(value, int) else None


def _percentile(values: list[int], q: float) -> int | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * q) - 1))
    return ordered[idx]


def _stats(values: list[int]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean_ms": None,
            "p50_ms": None,
            "p95_ms": None,
            "p99_ms": None,
            "max_ms": None,
        }
    return {
        "count": len(values),
        "mean_ms": (sum(values) / len(values)) / 1e6,
        "p50_ms": _percentile(values, 0.50) / 1e6,
        "p95_ms": _percentile(values, 0.95) / 1e6,
        "p99_ms": _percentile(values, 0.99) / 1e6,
        "max_ms": max(values) / 1e6,
    }


def _read_events(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    if not path.exists():
        return events
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict) and item.get("schema") == SCHEMA:
            events.append(item)
    return events


def _build_rows(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_revision: dict[int, dict[str, list[dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    for event in events:
        revision = event.get("revision")
        if isinstance(revision, int) and revision > 0:
            by_revision[revision][str(event.get("kind"))].append(event)

    rows: list[dict[str, Any]] = []
    for revision in sorted(by_revision):
        kinds = by_revision[revision]

        snapshots = sorted(kinds.get("lidar_snapshot_update", []), key=lambda x: _effective_event_ns(x) or -1)
        admissions = sorted(kinds.get("l2_admitted", []), key=lambda x: _effective_event_ns(x) or -1)
        l4s = sorted(kinds.get("l4_accepted", []), key=lambda x: _effective_event_ns(x) or -1)
        pubs = sorted(kinds.get("multirate_publication", []), key=lambda x: x.get("publication_monotonic_ns", -1))

        snapshot = snapshots[0] if snapshots else None
        admission = admissions[0] if admissions else None
        l4 = l4s[0] if l4s else None

        measurement_ns = None
        scan_end_ns = None
        for source in (snapshot, admission, l4, pubs[0] if pubs else None):
            if not source:
                continue
            if measurement_ns is None and isinstance(source.get("measurement_monotonic_ns"), int):
                measurement_ns = source["measurement_monotonic_ns"]
            if scan_end_ns is None:
                value = source.get("scan_end_monotonic_ns")
                if not isinstance(value, int):
                    value = source.get("captured_monotonic_ns")
                if isinstance(value, int):
                    scan_end_ns = value

        snapshot_ns = _effective_event_ns(snapshot) if snapshot else None
        admission_ns = _effective_event_ns(admission) if admission else None
        l4_ns = _effective_event_ns(l4) if l4 else None

        # Pick the publication that could actually have been visible to this
        # control tick. L0 closes against TickContext.monotonic_ns, not against
        # the later wall-clock time at which L2 happened to execute. A duplicate
        # publication after the tick cutoff cannot have fed this L2 admission.
        publication = None
        if pubs:
            cutoff_ns = (
                admission.get("tick_monotonic_ns")
                if admission and isinstance(admission.get("tick_monotonic_ns"), int)
                else None
            )
            candidates = [
                p for p in pubs
                if isinstance(p.get("publication_monotonic_ns"), int)
                and (cutoff_ns is None or p["publication_monotonic_ns"] <= cutoff_ns)
            ]
            publication = candidates[-1] if candidates else None
        publication_ns = (
            int(publication["publication_monotonic_ns"])
            if publication and isinstance(publication.get("publication_monotonic_ns"), int)
            else None
        )

        def delta(a: int | None, b: int | None) -> int | None:
            if a is None or b is None:
                return None
            return b - a

        a_ns = delta(measurement_ns, scan_end_ns)
        b_ns = delta(scan_end_ns, snapshot_ns)
        c_ns = delta(snapshot_ns, publication_ns)
        d_ns = delta(publication_ns, admission_ns)
        e_ns = delta(admission_ns, l4_ns)
        total_ns = delta(measurement_ns, l4_ns)

        reject_reasons = [
            str(item.get("reason"))
            for item in kinds.get("l2_rejected", [])
            if item.get("reason") is not None
        ]

        rows.append(
            {
                "revision": revision,
                "measurement_monotonic_ns": measurement_ns,
                "scan_end_monotonic_ns": scan_end_ns,
                "snapshot_update_monotonic_ns": snapshot_ns,
                "publication_monotonic_ns": publication_ns,
                "l2_admission_monotonic_ns": admission_ns,
                "l4_accepted_monotonic_ns": l4_ns,
                "l2_tick_id": admission.get("tick_id") if admission else None,
                "l4_tick_id": l4.get("tick_id") if l4 else None,
                "A_measurement_to_scan_complete_ns": a_ns,
                "B_scan_complete_to_snapshot_ns": b_ns,
                "C_snapshot_to_publication_ns": c_ns,
                "D_publication_to_l2_ns": d_ns,
                "E_l2_to_l4_ns": e_ns,
                "total_measurement_to_l4_ns": total_ns,
                "A_ms": None if a_ns is None else a_ns / 1e6,
                "B_ms": None if b_ns is None else b_ns / 1e6,
                "C_ms": None if c_ns is None else c_ns / 1e6,
                "D_ms": None if d_ns is None else d_ns / 1e6,
                "E_ms": None if e_ns is None else e_ns / 1e6,
                "total_ms": None if total_ns is None else total_ns / 1e6,
                "publication_count_for_revision": len(pubs),
                "l2_reject_reasons": "|".join(reject_reasons),
                "complete_A_to_E": all(
                    value is not None
                    for value in (
                        measurement_ns,
                        scan_end_ns,
                        snapshot_ns,
                        publication_ns,
                        admission_ns,
                        l4_ns,
                    )
                ),
            }
        )
    return rows


def _summary(rows: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[str, Any]:
    def vals(name: str) -> list[int]:
        return [
            int(row[name])
            for row in rows
            if isinstance(row.get(name), int) and row[name] >= 0
        ]

    complete = [row for row in rows if row.get("complete_A_to_E") is True]
    totals = vals("total_measurement_to_l4_ns")
    return {
        "schema": SCHEMA,
        "tool_version": TOOL_VERSION,
        "row_count": len(rows),
        "complete_A_to_E_count": len(complete),
        "event_count": len(events),
        "segments": {
            "A_measurement_to_scan_complete": _stats(vals("A_measurement_to_scan_complete_ns")),
            "B_scan_complete_to_snapshot": _stats(vals("B_scan_complete_to_snapshot_ns")),
            "C_snapshot_to_publication": _stats(vals("C_snapshot_to_publication_ns")),
            "D_publication_to_l2": _stats(vals("D_publication_to_l2_ns")),
            "E_l2_to_l4": _stats(vals("E_l2_to_l4_ns")),
            "TOTAL_measurement_to_l4": _stats(totals),
        },
        "thresholds": {
            "total_over_200ms_count": sum(v > 200_000_000 for v in totals),
            "total_over_250ms_count": sum(v > 250_000_000 for v in totals),
            "total_over_300ms_count": sum(v > 300_000_000 for v in totals),
        },
        "probe_hook_errors": [
            event for event in events if event.get("kind") == "probe_hook_error"
        ],
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    columns = [
        "revision",
        "l2_tick_id",
        "l4_tick_id",
        "A_ms",
        "B_ms",
        "C_ms",
        "D_ms",
        "E_ms",
        "total_ms",
        "publication_count_for_revision",
        "l2_reject_reasons",
        "complete_A_to_E",
        "measurement_monotonic_ns",
        "scan_end_monotonic_ns",
        "snapshot_update_monotonic_ns",
        "publication_monotonic_ns",
        "l2_admission_monotonic_ns",
        "l4_accepted_monotonic_ns",
    ]
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _fmt_ms(value: Any) -> str:
    return "-" if value is None else f"{float(value):8.3f}"


def _print_summary(summary: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    print()
    print("R2B4 LiDAR -> L4 latency probe")
    print(f"scans: {summary['row_count']}  complete A-E: {summary['complete_A_to_E_count']}")
    print()
    print("segment                              p50 ms    p95 ms    p99 ms    max ms")
    print("----------------------------------  --------  --------  --------  --------")
    names = [
        ("A measurement -> scan complete", "A_measurement_to_scan_complete"),
        ("B scan complete -> snapshot", "B_scan_complete_to_snapshot"),
        ("C snapshot -> MultiRate publish", "C_snapshot_to_publication"),
        ("D publish -> L2 accepted", "D_publication_to_l2"),
        ("E L2 accepted -> L4", "E_l2_to_l4"),
        ("TOTAL measurement -> L4", "TOTAL_measurement_to_l4"),
    ]
    for label, key in names:
        item = summary["segments"][key]
        print(
            f"{label:34s}  "
            f"{_fmt_ms(item['p50_ms'])}  "
            f"{_fmt_ms(item['p95_ms'])}  "
            f"{_fmt_ms(item['p99_ms'])}  "
            f"{_fmt_ms(item['max_ms'])}"
        )
    t = summary["thresholds"]
    print()
    print(
        "TOTAL threshold counts: "
        f">200ms={t['total_over_200ms_count']}  "
        f">250ms={t['total_over_250ms_count']}  "
        f">300ms={t['total_over_300ms_count']}"
    )

    complete = [r for r in rows if isinstance(r.get("total_ms"), (int, float))]
    if complete:
        worst = sorted(complete, key=lambda r: float(r["total_ms"]), reverse=True)[:10]
        print()
        print("Worst scans:")
        print("rev   tick   A(ms)   B(ms)   C(ms)   D(ms)   E(ms)   TOTAL(ms)")
        for r in worst:
            print(
                f"{int(r['revision']):4d} "
                f"{str(r.get('l4_tick_id') or '-'):>6s} "
                f"{_fmt_ms(r.get('A_ms'))} "
                f"{_fmt_ms(r.get('B_ms'))} "
                f"{_fmt_ms(r.get('C_ms'))} "
                f"{_fmt_ms(r.get('D_ms'))} "
                f"{_fmt_ms(r.get('E_ms'))} "
                f"{_fmt_ms(r.get('total_ms'))}"
            )

    errors = summary.get("probe_hook_errors", [])
    if errors:
        print()
        print(f"WARNING: {len(errors)} probe hook error(s) recorded; inspect JSON.")


# ---------------------------------------------------------------------------
# Launcher
# ---------------------------------------------------------------------------

def _repo_root(explicit: str | None) -> Path:
    if explicit:
        root = Path(explicit).expanduser().resolve()
    elif os.environ.get(ENV_ROOT):
        root = Path(os.environ[ENV_ROOT]).expanduser().resolve()
    else:
        cwd = Path.cwd().resolve()
        if (cwd / "v3").is_dir() and (cwd / "pytest.ini").is_file():
            root = cwd
        else:
            root = Path("/home/alba/project_r2b4")
    if not (root / "v3").is_dir() or not (root / "pytest.ini").is_file():
        raise SystemExit(f"ERROR: invalid R2B4 root: {root}")
    return root


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _existing_runtime_pid(root: Path) -> int | None:
    path = root / "runtime" / ".r2b4_runtime_pid"
    try:
        pid = int(path.read_text(encoding="ascii").strip())
    except (OSError, UnicodeError, ValueError):
        return None
    return pid if _pid_alive(pid) else None


def _sitecustomize_text(tool_dir: Path) -> str:
    # tool_dir is also placed in PYTHONPATH, so importing the tool is stable.
    return (
        "try:\n"
        "    from r2b4_lidar_l4_latency_probe import install_hooks_from_env\n"
        "    install_hooks_from_env()\n"
        "except Exception:\n"
        "    pass\n"
    )


def _default_command(root: Path) -> list[str]:
    launcher = root / "r"
    if launcher.exists():
        return [str(launcher), "rc", "30", "c", "full"]
    return ["python3", "-m", "v3.interface_cli", "rc", "30", "c", "full"]


def _self_test() -> int:
    now = 10_000_000_000
    events: list[dict[str, Any]] = []
    for revision in (1, 2):
        measurement = now + revision * 1_000_000_000
        end = measurement + 35_000_000
        snap = end + 2_000_000
        pub = snap + 3_000_000
        l2 = pub + 4_000_000
        l4 = l2 + 1_000_000
        base = {
            "schema": SCHEMA,
            "tool_version": TOOL_VERSION,
            "revision": revision,
            "pid": 1,
            "process_argv0": "v3_process_runtime.py",
        }
        events += [
            dict(base, kind="lidar_snapshot_update", event_monotonic_ns=snap,
                 measurement_monotonic_ns=measurement, scan_end_monotonic_ns=end),
            dict(base, kind="multirate_publication", event_monotonic_ns=pub,
                 publication_monotonic_ns=pub, measurement_monotonic_ns=measurement,
                 scan_end_monotonic_ns=end),
            dict(base, kind="l2_admitted", event_monotonic_ns=l2,
                 event_override_monotonic_ns=l2, tick_id=revision,
                 measurement_monotonic_ns=measurement, scan_end_monotonic_ns=end),
            dict(base, kind="l4_accepted", event_monotonic_ns=l4,
                 event_override_monotonic_ns=l4, tick_id=revision,
                 measurement_monotonic_ns=measurement, scan_end_monotonic_ns=end),
        ]
    rows = _build_rows(events)
    assert len(rows) == 2
    for row in rows:
        assert row["A_measurement_to_scan_complete_ns"] == 35_000_000
        assert row["B_scan_complete_to_snapshot_ns"] == 2_000_000
        assert row["C_snapshot_to_publication_ns"] == 3_000_000
        assert row["D_publication_to_l2_ns"] == 4_000_000
        assert row["E_l2_to_l4_ns"] == 1_000_000
        assert row["total_measurement_to_l4_ns"] == 45_000_000
    summary = _summary(rows, events)
    assert summary["complete_A_to_E_count"] == 2
    assert summary["segments"]["TOTAL_measurement_to_l4"]["p99_ms"] == 45.0
    print("SELF-TEST PASS")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Measure production live LiDAR measurement -> L4 latency without modifying repo files.",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 r2b4_lidar_l4_latency_probe.py -- r rc 30 c full\n"
            "  python3 r2b4_lidar_l4_latency_probe.py --root /home/alba/project_r2b4 -- r rc 60 c full\n"
            "  python3 r2b4_lidar_l4_latency_probe.py --self-test\n"
        ),
    )
    parser.add_argument("--root", help="R2B4 repository root")
    parser.add_argument("--output-dir", help="output directory; default runtime/diagnostics")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="normal live command after --")
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    root = _repo_root(args.root)
    existing = _existing_runtime_pid(root)
    if existing is not None:
        print(
            f"REFUSED: resident runtime PID {existing} is already running.\n"
            "Exact B-mode instrumentation must start the runtime under the probe.\n"
            "Stop it first with: r sd",
            file=sys.stderr,
        )
        return 3

    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        command = _default_command(root)

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else root / "runtime" / "diagnostics"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = output_dir / f"lidar_l4_latency_{stamp}"
    event_path = Path(str(stem) + ".events.ndjson")
    csv_path = Path(str(stem) + ".csv")
    json_path = Path(str(stem) + ".json")

    # Ensure a fresh event stream.
    try:
        event_path.unlink()
    except FileNotFoundError:
        pass

    tool_dir = Path(__file__).resolve().parent

    with tempfile.TemporaryDirectory(prefix="r2b4-l4-latency-site-") as temp:
        temp_path = Path(temp)
        (temp_path / "sitecustomize.py").write_text(
            _sitecustomize_text(tool_dir),
            encoding="utf-8",
        )

        env = os.environ.copy()
        env[ENV_ENABLE] = "1"
        env[ENV_LOG] = str(event_path)
        env[ENV_ROOT] = str(root)

        existing_pythonpath = env.get("PYTHONPATH", "")
        pieces = [str(temp_path), str(tool_dir), str(root)]
        if existing_pythonpath:
            pieces.append(existing_pythonpath)
        env["PYTHONPATH"] = os.pathsep.join(pieces)

        print("R2B4 live LiDAR -> L4 latency probe")
        print(f"repo:    {root}")
        print(f"events:  {event_path}")
        print(f"command: {shlex.join(command)}")
        print()
        print("The normal production run is starting under passive timing hooks.")

        try:
            completed = subprocess.run(command, cwd=root, env=env)
            command_rc = int(completed.returncode)
        except KeyboardInterrupt:
            print("\nInterrupted.", file=sys.stderr)
            command_rc = 130
        except OSError as exc:
            print(f"ERROR: cannot start command: {exc}", file=sys.stderr)
            return 2

    events = _read_events(event_path)
    rows = _build_rows(events)
    summary = _summary(rows, events)
    summary["generated_at"] = datetime.now().isoformat(timespec="seconds")
    summary["repo_root"] = str(root)
    summary["command"] = command
    summary["command_returncode"] = command_rc
    summary["event_file"] = str(event_path)
    summary["csv_file"] = str(csv_path)

    _write_csv(csv_path, rows)
    json_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    _print_summary(summary, rows)
    print()
    print(f"raw events: {event_path}")
    print(f"per-scan CSV: {csv_path}")
    print(f"summary JSON: {json_path}")

    if not events:
        print(
            "\nERROR: no live probe events were recorded. "
            "Verify that the command actually started v3_process_runtime.py.",
            file=sys.stderr,
        )
        return 4 if command_rc == 0 else command_rc

    # Preserve underlying live-command failure if it failed.
    return command_rc


if __name__ == "__main__":
    raise SystemExit(main())
