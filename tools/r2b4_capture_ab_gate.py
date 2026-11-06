#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

MEAN_PERIOD_LIMIT_PCT = 0.5
CPU_CORE_EQUIVALENT_LIMIT = 0.03


def _load(path: str | Path) -> dict[str, object]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: root must be an object")
    return value


def _num(mapping: dict[str, object], *path: str) -> float:
    current: object = mapping
    for key in path:
        if not isinstance(current, dict):
            raise KeyError(".".join(path))
        current = current[key]
    if isinstance(current, bool) or not isinstance(current, (int, float)):
        raise TypeError(".".join(path))
    return float(current)


def _static(project: Path) -> int:
    sidecars = (project / "v3/process_sidecars.py").read_text(encoding="utf-8")
    runtime = (project / "v3_runtime.py").read_text(encoding="utf-8")
    session = sidecars.split("class ProcessMcapCaptureSession:", 1)[1]

    failures: list[str] = []
    for token in (
        "CaptureCoreProjector",
        "CaptureIpcProjectionError",
        "self._projector",
        ".project(record)",
    ):
        if token in session:
            failures.append(f"production capture ingress still contains {token}")
    if 'self._enqueue("record", record)' not in session:
        failures.append("production capture ingress is not direct typed-record handoff")
    if '"CAPTURE_TAP"' not in runtime:
        failures.append("CAPTURE_TAP timing is missing")
    if '"CAPTURE_CHECKPOINT"' not in runtime:
        failures.append("CAPTURE_CHECKPOINT timing is missing")

    process_runtime = (project / "v3_process_runtime.py").read_text(encoding="utf-8")
    if "capture_session.raw_lidar_queue" not in process_runtime:
        failures.append("direct raw LiDAR evidence lane is missing")

    if failures:
        print("STATIC GATE: FAIL")
        for item in failures:
            print(" -", item)
        return 1
    print("STATIC GATE: PASS")
    return 0


def _ab(off_path: Path, on_path: Path) -> int:
    off = _load(off_path)
    on = _load(on_path)

    off_period = _num(off, "runtime_tick", "average_period_ms")
    on_period = _num(on, "runtime_tick", "average_period_ms")
    off_cpu = _num(off, "parent_process", "cpu_core_equivalent")
    on_cpu = _num(on, "parent_process", "cpu_core_equivalent")

    period_delta_pct = 100.0 * (on_period - off_period) / off_period
    cpu_delta = on_cpu - off_cpu

    print(f"capture OFF average period: {off_period:.6f} ms")
    print(f"capture ON  average period: {on_period:.6f} ms")
    print(f"period delta: {period_delta_pct:+.3f}%")
    print(f"parent CPU-core-equivalent delta: {cpu_delta:+.4f}")

    failures: list[str] = []
    if period_delta_pct > MEAN_PERIOD_LIMIT_PCT:
        failures.append(
            f"average period regression {period_delta_pct:.3f}% > "
            f"{MEAN_PERIOD_LIMIT_PCT:.3f}%"
        )
    if cpu_delta > CPU_CORE_EQUIVALENT_LIMIT:
        failures.append(
            f"parent CPU regression {cpu_delta:.4f} > "
            f"{CPU_CORE_EQUIVALENT_LIMIT:.4f} core"
        )

    if failures:
        print("A/B GATE: FAIL")
        for item in failures:
            print(" -", item)
        print(
            "P2 decision: profile multiprocessing feeder/pickle. "
            "Only a measured residual there justifies a capture-specific "
            "fixed-slot shared-memory transport."
        )
        return 1

    print("A/B GATE: PASS")
    print(
        "P2 decision: current evidence does not justify replacing the bounded "
        "multiprocessing transport with shared memory."
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="R2B4 capture async-refactor static and capture OFF/ON A/B gate"
    )
    parser.add_argument("--static", dest="static_root")
    parser.add_argument("--off")
    parser.add_argument("--on")
    args = parser.parse_args()

    if args.static_root:
        return _static(Path(args.static_root).resolve())
    if bool(args.off) != bool(args.on):
        parser.error("--off and --on must be provided together")
    if args.off and args.on:
        return _ab(Path(args.off), Path(args.on))
    parser.error("use --static PROJECT_ROOT or --off FILE --on FILE")


if __name__ == "__main__":
    raise SystemExit(main())
