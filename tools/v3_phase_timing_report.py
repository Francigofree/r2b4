#!/usr/bin/env python3
"""Print the latest resident phase-timing evidence without inferring root cause."""

from __future__ import annotations

import json
from pathlib import Path
import sys


def ms(value: object) -> float:
    return float(value or 0) / 1_000_000.0


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    root = Path.cwd()
    path = Path(args[0]) if args else root / "runtime" / "v3_status.json"
    if not path.is_absolute():
        path = root / path
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: cannot read {path}: {exc}", file=sys.stderr)
        return 2
    report = payload.get("report") if isinstance(payload, dict) else None
    timing = report.get("timing") if isinstance(report, dict) else None
    phase = timing.get("control_phase_timing") if isinstance(timing, dict) else None
    phases = phase.get("phases") if isinstance(phase, dict) else None
    if not isinstance(phases, dict) or not phases:
        print("No R2B4_RUNTIME_PHASE_TIMING_V1 evidence in status file.")
        return 1

    print(f"source: {path}")
    print("scope: normal ticks only; elapsed wall-clock, not CPU time")
    print("note: PIPELINE_TOTAL overlaps L1-L12; no causal root cause is inferred")
    print()
    print(f"{'phase':<18} {'n':>6} {'mean':>9} {'p50':>9} {'p95':>9} {'p99':>9} {'max':>9} {'>20ms':>7}")
    rows = []
    for name, values in phases.items():
        if not isinstance(values, dict):
            continue
        rows.append((name, values))
    # Diagnostic display: slowest p99 first, then name. This is not a causal ranking.
    rows.sort(key=lambda item: (-int(item[1].get("p99_ns", 0)), item[0]))
    for name, values in rows:
        print(
            f"{name:<18} {int(values.get('count', 0)):>6d} "
            f"{ms(values.get('mean_ns')):>8.3f} "
            f"{ms(values.get('p50_ns')):>8.3f} "
            f"{ms(values.get('p95_ns')):>8.3f} "
            f"{ms(values.get('p99_ns')):>8.3f} "
            f"{ms(values.get('max_ns')):>8.3f} "
            f"{int(values.get('over_20ms_count', 0)):>7d}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
