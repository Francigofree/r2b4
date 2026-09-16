#!/usr/bin/env python3
"""Audit R2B4 CPU affinity while live and timing evidence after shutdown."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


EXPECTED = {
    "r2b4-runtime": 3,
    "r2b4-io-start": 0,
    "r2b4-vision": 1,
    "r2b4-lidar": 2,
}


def _cpu_list(text: str) -> set[int]:
    cpus: set[int] = set()
    for part in text.strip().split(","):
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-", 1)
            cpus.update(range(int(lo), int(hi) + 1))
        else:
            cpus.add(int(part))
    return cpus


def _task_info(pid: int) -> list[tuple[int, str, set[int]]]:
    root = Path(f"/proc/{pid}/task")
    result = []
    for task in sorted(root.iterdir(), key=lambda p: int(p.name)):
        tid = int(task.name)
        try:
            name = (task / "comm").read_text().strip()
            status = (task / "status").read_text()
        except OSError:
            continue
        allowed = set()
        for line in status.splitlines():
            if line.startswith("Cpus_allowed_list:"):
                allowed = _cpu_list(line.split(":", 1)[1])
                break
        result.append((tid, name, allowed))
    return result


def _children(pid: int) -> list[int]:
    """Return direct child PIDs without relying on task/children availability."""

    result: list[int] = []
    for item in Path("/proc").iterdir():
        if not item.name.isdigit():
            continue
        try:
            status = (item / "status").read_text()
        except OSError:
            continue
        parent = None
        for line in status.splitlines():
            if line.startswith("PPid:"):
                parent = int(line.split(":", 1)[1].strip())
                break
        if parent == pid:
            result.append(int(item.name))
    return sorted(result)


def _resolve_pid(args: argparse.Namespace) -> int:
    if args.pid is not None:
        return args.pid
    value = Path(args.pid_file).read_text().strip()
    return int(value)


def live(args: argparse.Namespace) -> int:
    pid = _resolve_pid(args)
    parent_rows = _task_info(pid)
    child_pids = _children(pid)
    child_rows: list[tuple[int, str, set[int]]] = []
    for child in child_pids:
        child_rows.extend(_task_info(child))
    print(f"runtime_pid={pid} children={child_pids}")
    for tid, name, allowed in parent_rows:
        print(f"parent tid={tid:<7} name={name:<16} allowed={','.join(map(str, sorted(allowed)))}")
    for tid, name, allowed in child_rows:
        print(f"child  tid={tid:<7} name={name:<16} allowed={','.join(map(str, sorted(allowed)))}")

    main_mask = next((mask for tid, _, mask in parent_rows if tid == pid), set())
    runtime_ok = main_mask == {3}
    vision_ok = any(mask == {1} for tid, _, mask in parent_rows if tid != pid)
    io_ok = any(mask == {0} for tid, _, mask in parent_rows if tid != pid)
    lidar_ok = bool(child_rows) and all(mask == {2} for _, _, mask in child_rows)
    parent_workers = tuple(mask for tid, _, mask in parent_rows if tid != pid)
    parent_single_cpu = bool(parent_workers) and all(len(mask) == 1 for mask in parent_workers)
    parent_cpu_roles_only = all(mask <= {0, 1, 3} for mask in parent_workers)
    checks = {
        "runtime_main_cpu3": runtime_ok,
        "vision_worker_cpu1_present": vision_ok,
        "io_worker_cpu0_present": io_ok,
        "all_parent_tasks_single_cpu": parent_single_cpu,
        "no_parent_task_leaks_to_lidar_cpu2": parent_cpu_roles_only,
        "lidar_child_cpu2": lidar_ok,
    }
    for name, passed in checks.items():
        print(f"{name}: {'PASS' if passed else 'FAIL'}")
    ok = all(checks.values())
    print("LIVE_AFFINITY_AUDIT=" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


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
