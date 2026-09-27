#!/usr/bin/env python3
"""Audit R2B4 CPU affinity while live and timing evidence after shutdown."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


def _resolve_pid(args: argparse.Namespace) -> int:
    if args.pid is not None:
        return args.pid
    value = Path(args.pid_file).read_text().strip()
    return int(value)


def live(args: argparse.Namespace) -> int:
    pid = _resolve_pid(args)
    root = Path(__file__).resolve().parents[1]
    import sys
    sys.path.insert(0, str(root))
    from v3.runtime_performance import load_runtime_affinity_config, apply_host_affinity
    from v3.affinity_diagnostics import audit_affinity
    apply_host_affinity(root, "diagnostics")
    result = audit_affinity(load_runtime_affinity_config(root / "conf" / "vezerles.json"),
                            pid, project_root=root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


def report(args: argparse.Namespace) -> int:
    payload = json.loads(Path(args.status).read_text(encoding="utf-8"))
    report_value = payload.get("report") if isinstance(payload, dict) else None
    if not isinstance(report_value, dict):
        raise SystemExit("status does not contain a terminal report")
    timing = report_value.get("timing")
    if not isinstance(timing, dict):
        raise SystemExit("terminal report does not contain timing evidence")

    def ms(key: str) -> float:
        return float(timing.get(key, 0)) / 1_000_000.0

    checks = {
        "normal_stop": report_value.get("termination_class") == "SHUTDOWN_SAFE_LOW",
        "no_fault_layer": report_value.get("fault_layer") is None,
        "period_p99_le_25ms": int(timing.get("period_p99_ns", 0)) <= args.p99_ns,
        "period_max_le_40ms": int(timing.get("period_max_ns", 0)) <= args.max_ns,
        "no_period_over_40ms": int(timing.get("period_over_40ms_count", 0)) == 0,
        "work_p99_below_period": int(timing.get("work_p99_ns", 0))
        <= int(timing.get("target_period_ns", 0)),
    }
    print(
        "timing_ms "
        f"period_mean={ms('period_mean_ns'):.3f} "
        f"p50={ms('period_p50_ns'):.3f} "
        f"p95={ms('period_p95_ns'):.3f} "
        f"p99={ms('period_p99_ns'):.3f} "
        f"max={ms('period_max_ns'):.3f}"
    )
    print(
        "work_ms "
        f"control_p99={ms('control_p99_ns'):.3f} "
        f"observer_p99={ms('observer_p99_ns'):.3f} "
        f"work_p99={ms('work_p99_ns'):.3f} "
        f"lateness_p99={ms('lateness_p99_ns'):.3f}"
    )
    for name, passed in checks.items():
        print(f"{name}: {'PASS' if passed else 'FAIL'}")
    ok = all(checks.values())
    print("TIMING_AUDIT=" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    p_live = sub.add_parser("live")
    p_live.add_argument("--pid", type=int)
    p_live.add_argument("--pid-file", default="runtime/.r2b4_runtime_pid")
    p_live.set_defaults(func=live)
    p_report = sub.add_parser("report")
    p_report.add_argument("--status", default="runtime/v3_status.json")
    p_report.add_argument("--p99-ns", type=int, default=25_000_000)
    p_report.add_argument("--max-ns", type=int, default=40_000_000)
    p_report.set_defaults(func=report)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
