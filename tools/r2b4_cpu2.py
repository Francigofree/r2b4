#!/usr/bin/env python3
"""R2B4 CPU2: passive, whole-host Linux CPU and scheduler profiler.

Drop-in replacement for tools/r2b4_cpu2.py (source af9537f, 2026-11-18).

No motor, navigation, sensor or ROS authority. Reads /proc, supervisor JSON,
CPU clock/thermal metadata. Preserves the original positional CLI and core
sample/summary keys; adds independently launched Nav2 and SLAM groups and a
whole-system reconciliation. Never treats residual kernel/IRQ/short-lived CPU
as if it belonged to a named process. No root-cause claims.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Mapping

SAMPLE_SCHEMA = "R2B4_CPU2_SAMPLE_V1"
SUMMARY_SCHEMA = "R2B4_CPU2_SUMMARY_V1"
EXTENSION_SCHEMA = "R2B4_CPU2_FULL_SYSTEM_P0_V1"
PROC = Path("/proc")


def _read(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError):
        return None


def _json_file(path: Path) -> dict[str, Any]:
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError, UnicodeError):
        return {}


def parse_proc_stat(text: str) -> dict[str, int | str]:
    left, right = text.find("("), text.rfind(")")
    if left <= 0 or right <= left:
        raise ValueError("invalid /proc stat row")
    pid = int(text[:left].strip())
    rest = text[right + 1:].strip().split()
    if len(rest) < 37:
        raise ValueError("truncated /proc stat row")
    return {
        "pid": pid, "comm": text[left + 1:right], "state": rest[0],
        "ppid": int(rest[1]), "pgrp": int(rest[2]), "session": int(rest[3]),
        "utime": int(rest[11]), "stime": int(rest[12]),
        "starttime": int(rest[19]), "processor": int(rest[36]),
    }


def parse_schedstat(text: str) -> tuple[int, int, int]:
    a = text.split()
    if len(a) < 3:
        raise ValueError("invalid schedstat")
    return int(a[0]), int(a[1]), int(a[2])


def decode_throttled(value: int) -> list[str]:
    bits = {0: "UNDER_VOLTAGE_NOW", 1: "ARM_FREQUENCY_CAPPED_NOW",
            2: "THROTTLED_NOW", 3: "SOFT_TEMPERATURE_LIMIT_NOW",
            16: "UNDER_VOLTAGE_OCCURRED", 17: "ARM_FREQUENCY_CAPPED_OCCURRED",
            18: "THROTTLED_OCCURRED", 19: "SOFT_TEMPERATURE_LIMIT_OCCURRED"}
    return [name for n, name in bits.items() if value & (1 << n)]


def throttled() -> dict[str, Any]:
    try:
        cp = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                            text=True, timeout=1.0, check=False)
        raw = cp.stdout.strip().split("=", 1)[1]
        value = int(raw, 16)
        return {"available": True, "raw": raw, "value": value,
                "flags": decode_throttled(value)}
    except (OSError, IndexError, ValueError, subprocess.SubprocessError):
        return {"available": False, "raw": None, "flags": []}


def load_affinity(root: Path) -> dict[str, Any]:
    conf = _json_file(root / "conf/vezerles.json")
    obj = conf.get("runtime_affinity", {})
    return obj if isinstance(obj, dict) else {}


def parse_cpu_list(value: str) -> set[int]:
    ans: set[int] = set()
    for component in value.split(","):
        if not component:
            continue
        a = component.split("-")
        ans.update(range(int(a[0]), int(a[-1]) + 1))
    return ans


def pin_monitor_to_diagnostics_cpus(root: Path) -> dict[str, Any]:
    try:
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from v3.runtime_performance import apply_host_affinity
        evidence = apply_host_affinity(root, "diagnostics")
        return {"attempted": bool(evidence), "tasks": [v.as_dict() for v in evidence],
                "applied": bool(evidence) and all(v.applied for v in evidence)}
    except (ImportError, OSError, ValueError, RuntimeError) as exc:
        return {"attempted": False, "tasks": [], "applied": False,
                "error": f"{type(exc).__name__}:{exc}"[:256]}


def read_cores() -> dict[int, tuple[int, int]]:
    """Return (total, idle+iowait) jiffies per core, excluding guest overlap."""
    out: dict[int, tuple[int, int]] = {}
    data = _read(PROC / "stat") or ""
    for line in data.splitlines():
        a = line.split()
        if not a or not a[0].startswith("cpu") or not a[0][3:].isdigit():
            continue
        try:
            v = [int(x) for x in a[1:8 + 1]]  # user..steal; guest duplicates user/nice
            if len(v) < 4:
                continue
            out[int(a[0][3:])] = (sum(v), v[3] + (v[4] if len(v) > 4 else 0))
        except ValueError:
            continue
    return out


def core_busy(before: Mapping[int, tuple[int, int]],
              after: Mapping[int, tuple[int, int]]) -> dict[int, float]:
    out: dict[int, float] = {}
    for cpu, current in after.items():
        old = before.get(cpu)
        if old is None:
            continue
        total, idle = current[0] - old[0], current[1] - old[1]
        if total > 0:
            out[cpu] = min(100.0, max(0.0, 100.0 * (total - idle) / total))
    return out


def frequencies_mhz() -> dict[int, float]:
    out: dict[int, float] = {}
    for base in Path("/sys/devices/system/cpu").glob("cpu[0-9]*"):
        value = _read(base / "cpufreq/scaling_cur_freq")
        if value and base.name[3:].isdigit():
            try:
                out[int(base.name[3:])] = float(value.strip()) / 1000.0
            except ValueError:
                pass
    return out


def temperature_c() -> float | None:
    value = _read(Path("/sys/class/thermal/thermal_zone0/temp"))
    try:
        n = float(value)  # type: ignore[arg-type]
        return n / 1000 if n > 1000 else n
    except (TypeError, ValueError):
        return None


def read_argv(pid: int) -> tuple[str, ...]:
    try:
        raw = (PROC / str(pid) / "cmdline").read_bytes()
        return tuple(x.decode("utf-8", "replace") for x in raw.split(b"\x00") if x)
    except OSError:
        return ()


def _command(argv: tuple[str, ...], limit: int = 512) -> str:
    return " ".join(argv)[:limit]


def read_process(pid: int) -> dict[str, Any] | None:
    value = _read(PROC / str(pid) / "stat")
    if not value:
        return None
    try:
        row = parse_proc_stat(value)
        if row["state"] == "Z":
            return None
        return {"pid": pid, "ppid": row["ppid"], "pgrp": row["pgrp"],
                "session": row["session"], "comm": row["comm"],
                "ticks": int(row["utime"]) + int(row["stime"]),
                "start_ticks": row["starttime"], "processor": row["processor"]}
    except (TypeError, ValueError):
        return None


def all_processes() -> dict[int, dict[str, Any]]:
    try:
        entries = list(PROC.iterdir())
    except OSError:
        return {}
    return {pid: row for entry in entries if entry.name.isdigit()
            if (pid := int(entry.name)) > 0
            if (row := read_process(pid)) is not None}


def _proc_identity(process: Mapping[str, Any]) -> tuple[int, int]:
    return (int(process["pid"]), int(process["start_ticks"]))


def _same_proc(old: Mapping[str, Any] | None, current: Mapping[str, Any]) -> bool:
    return old is not None and old.get("start_ticks") == current["start_ticks"]


def read_process_ticks(pid: int) -> tuple[int, int] | None:
    x = read_process(pid)
    return (int(x["ticks"]), int(x["start_ticks"])) if x else None


def task_ids(pid: int) -> list[int]:
    try:
        return sorted(int(p.name) for p in (PROC / str(pid) / "task").iterdir() if p.name.isdigit())
    except OSError:
        return []


def descendants(pid: int) -> list[int]:
    processes = all_processes()
    return sorted(_child_set(pid, processes) - {pid})


def _child_set(pid: int, processes: Mapping[int, Mapping[str, Any]]) -> set[int]:
    children: dict[int, list[int]] = {}
    for p, v in processes.items():
        children.setdefault(int(v["ppid"]), []).append(p)
    queue, found = [pid], {pid}
    while queue:
        parent = queue.pop()
        for child in children.get(parent, []):
            if child not in found:
                found.add(child)
                queue.append(child)
    return found


def _status(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for line in (_read(path) or "").splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            out[k.strip()] = v.strip()
    return out


def _status_int(values: Mapping[str, str], key: str) -> int | None:
    try:
        return int(values[key].split()[0])
    except (KeyError, IndexError, ValueError):
        return None


def read_task(pid: int, tid: int | None = None) -> dict[str, Any] | None:
    tid = pid if tid is None else tid
    base = PROC / str(pid) / "task" / str(tid)
    raw = _read(base / "stat")
    if raw is None:
        return None
    try:
        stat = parse_proc_stat(raw)
    except ValueError:
        return None
    sched = _read(base / "schedstat")
    ns = (None, None, None)
    if sched:
        try:
            ns = parse_schedstat(sched)
        except ValueError:
            pass
    status = _status(base / "status")
    return {"tid": tid, "comm": stat["comm"], "ppid": stat["ppid"],
            "cpu_ticks": int(stat["utime"]) + int(stat["stime"]),
            "processor": stat["processor"], "start_ticks": stat["starttime"],
            "sched_runtime_ns": ns[0], "sched_wait_ns": ns[1],
            "sched_slices": ns[2],
            "voluntary_ctxt": _status_int(status, "voluntary_ctxt_switches"),
            "nonvoluntary_ctxt": _status_int(status, "nonvoluntary_ctxt_switches"),
            "allowed_cpus": status.get("Cpus_allowed_list")}


def _delta(now: object, old: object) -> int | None:
    return now - old if type(now) is int and type(old) is int and now >= old else None


def task_delta(before: Mapping[str, Any], after: Mapping[str, Any],
               elapsed: float, clk_tck: int) -> dict[str, Any] | None:
    if before.get("start_ticks") != after.get("start_ticks") or elapsed <= 0:
        return None
    ticks = _delta(after.get("cpu_ticks"), before.get("cpu_ticks"))
    if ticks is None:
        return None
    run = _delta(after.get("sched_runtime_ns"), before.get("sched_runtime_ns"))
    wait = _delta(after.get("sched_wait_ns"), before.get("sched_wait_ns"))
    return {"tid": after["tid"], "comm": after["comm"],
            "cpu_pct_one_core": 100 * ticks / clk_tck / elapsed,
            "processor": after["processor"], "allowed_cpus": after["allowed_cpus"],
            "sched_runtime_ns": run, "runqueue_wait_ns": wait,
            "runqueue_wait_share": (wait / (run + wait) if run is not None and wait is not None
                                    and run + wait else None),
            "timeslices": _delta(after.get("sched_slices"), before.get("sched_slices")),
            "voluntary_ctxt_switches": _delta(after.get("voluntary_ctxt"), before.get("voluntary_ctxt")),
            "nonvoluntary_ctxt_switches": _delta(after.get("nonvoluntary_ctxt"), before.get("nonvoluntary_ctxt"))}


def process_cpu_delta(before: tuple[int, int] | None,
                      after: tuple[int, int] | None, elapsed: float, clk_tck: int) -> float | None:
    if not before or not after or before[1] != after[1] or elapsed <= 0:
        return None
    ticks = after[0] - before[0]
    return 100 * ticks / clk_tck / elapsed if ticks >= 0 else None


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def find_runtime_pid(root: Path) -> tuple[int | None, str]:
    """Trust the PID file only after inspecting the live process command."""
    value = _read(root / "runtime/.r2b4_runtime_pid")
    try:
        p = int(value) if value else -1
    except ValueError:
        p = -1
    if p > 1 and read_process(p) and _is_runtime_command(read_argv(p)):
        return p, "runtime/.r2b4_runtime_pid:cmdline_verified"
    matches = [p for p in all_processes()
               if _is_runtime_command(read_argv(p))]
    return ((matches[0], "proc_cmdline_fallback") if len(matches) == 1 else
            (None, "not_found" if not matches else "ambiguous_proc_cmdline"))


def _is_runtime_command(argv: tuple[str, ...]) -> bool:
    return any(token == "v3_process_runtime" or token.endswith("/v3_process_runtime.py")
               or token == "v3_process_runtime.py" for token in argv)


def _supervisor_command(argv: tuple[str, ...], module: str) -> bool:
    return module in argv and "__supervise" in argv


def _verified_supervisor(root: Path, group: str,
                         processes: Mapping[int, Mapping[str, Any]]) -> dict[str, Any]:
    folder, module = (("nav2_planner", "tools.nav2_planner") if group == "nav2" else
                      ("slam_toolbox", "tools.slam_toolbox"))
    file = root / "runtime" / folder / "supervisor.json"
    state = _json_file(file)
    result: dict[str, Any] = {"group": group, "supervisor_file": str(file),
                              "state": state.get("state"), "verified": False,
                              "pid": state.get("pid"), "reason": "missing_or_stale"}
    pid, recorded = state.get("pid"), state.get("process_start")
    if state.get("root") != str(root) or type(pid) is not int or pid < 2:
        result["reason"] = "root_or_pid_invalid"
        return result
    proc = processes.get(pid)
    if proc is None or str(proc["start_ticks"]) != str(recorded):
        result["reason"] = "pid_starttime_mismatch_or_exited"
        return result
    argv = read_argv(pid)
    if not _supervisor_command(argv, module):
        result["reason"] = "supervisor_cmdline_mismatch"
        return result
    result.update(verified=True, reason="pid_starttime_cmdline_verified",
                  command=_command(argv), start_ticks=proc["start_ticks"])
    return result


def _recognize_role(group: str, argv: tuple[str, ...], pid: int, root_pid: int) -> str:
    if pid == root_pid:
        return group + ".supervisor" if group != "v3" else "v3.control"
    joined = " ".join(argv)
    if group == "nav2":
        for match, role in (("v3.nav2_navigation_bridge", "bridge"),
                            ("v3.nav2_planner_bridge", "bridge"),
                            ("controller_server", "controller_mppi"),
                            ("planner_server", "planner"),
                            ("behavior_server", "behavior"),
                            ("bt_navigator", "bt_navigator"),
                            ("lifecycle_manager", "lifecycle_manager"),
                            ("explore_lite/explore", "frontier_explorer")):
            if match in joined:
                return "nav2." + role
    elif group == "slam":
        if "v3.slam_toolbox_bridge" in joined:
            return "slam.bridge"
        if "async_slam_toolbox_node" in joined or "sync_slam_toolbox_node" in joined:
            return "slam.node"
        if "slam_toolbox" in joined:
            return "slam.node_or_helper"
    elif group == "v3":
        if "multiprocessing" in joined:
            return "v3.python_worker_unidentified"
        for match, role in (("resource_tracker", "resource_tracker"),
                            ("v3_process_runtime", "control")):
            if match in joined:
                return "v3." + role
    if group == "other":
        return "other.unattributed_program"
    return group + ".unidentified_child"


def classify_processes(root: Path, processes: Mapping[int, Mapping[str, Any]],
                       runtime_pid: int,
                       trusted: dict[tuple[int, int], str]) -> tuple[dict[int, str], dict[str, Any]]:
    """Classification is identity-bound; no name-only adoption into ROS groups."""
    supervisors = {name: _verified_supervisor(root, name, processes)
                   for name in ("nav2", "slam")}
    memberships: dict[int, str] = {}
    if runtime_pid in processes and _is_runtime_command(read_argv(runtime_pid)):
        members = _child_set(runtime_pid, processes)
        for p in members:
            memberships[p] = "v3"
    for group, owner in supervisors.items():
        if not owner["verified"]:
            continue
        pid = int(owner["pid"])
        members = _child_set(pid, processes)
        # Validate named children even if /proc children enumeration missed them.
        state = _json_file(Path(owner["supervisor_file"]))
        if group == "nav2":
            starts = state.get("child_process_starts", {})
            for rawpid, rawstart in (starts.items() if isinstance(starts, dict) else []):
                try:
                    childpid = int(rawpid)
                except (TypeError, ValueError):
                    continue
                child = processes.get(childpid)
                if (child is not None and str(child["start_ticks"]) == str(rawstart)
                        and child["pgrp"] == processes[pid]["pgrp"]):
                    members.add(childpid)
        else:
            children = state.get("children", {})
            if isinstance(children, dict):
                for label, childpid in children.items():
                    child = processes.get(childpid) if type(childpid) is int else None
                    if (child and child["pgrp"] == processes[pid]["pgrp"]
                            and (childpid in members or _slam_child_command(read_argv(childpid), label))):
                        members.add(childpid)
        for p in members:
            if p not in memberships:
                memberships[p] = group
    # Verified earlier in this measurement, same PID and /proc starttime.
    for pid, proc in processes.items():
        key = _proc_identity(proc)
        if pid not in memberships and key in trusted:
            memberships[pid] = trusted[key]
        if pid in memberships:
            trusted[key] = memberships[pid]
    return memberships, supervisors


def _slam_child_command(argv: tuple[str, ...], label: str) -> bool:
    joined = " ".join(argv)
    if label == "bridge":
        return "v3.slam_toolbox_bridge" in argv
    if label == "node":
        return "slam_toolbox" in joined
    return False


def _safe_command(argv: tuple[str, ...], group: str) -> str:
    """Display verified R2B4 launch commands, but never leak other app arguments."""
    if group == "other":
        return Path(argv[0]).name if argv else "[kernel/no_cmdline]"
    hidden = ("password", "secret", "token", "api-key", "apikey", "authorization",
              "credential", "passwd", "private-key")
    clean: list[str] = []
    mask_next = False
    for word in argv:
        if mask_next:
            clean.append("[REDACTED]")
            mask_next = False
        elif any(key in word.lower() for key in hidden):
            if "=" in word:
                clean.append(word.split("=", 1)[0] + "=[REDACTED]")
            else:
                clean.append(word)
                mask_next = True
        else:
            clean.append(word)
    return " ".join(clean)[:2048]


def _threads_of(pids: list[int]) -> dict[tuple[int, int], dict[str, Any]]:
    ans: dict[tuple[int, int], dict[str, Any]] = {}
    for pid in pids:
        for tid in task_ids(pid):
            row = read_task(pid, tid)
            if row:
                ans[(pid, tid)] = row
    return ans


def _identity_row(pid: int, proc: Mapping[str, Any], group: str, root_pid: int,
                  argv_cache: dict[tuple[int, int], tuple[str, ...]]) -> dict[str, Any]:
    identity = _proc_identity(proc)
    if identity not in argv_cache:
        argv_cache[identity] = read_argv(pid)
    argv = argv_cache[identity]
    return {"pid": pid, "start_ticks": proc["start_ticks"], "ppid": proc["ppid"],
            "pgrp": proc["pgrp"], "session": proc["session"],
            "comm": proc["comm"], "role": _recognize_role(group, argv, pid, root_pid),
            "group": group, "command": _safe_command(argv, group),
            "command_available": bool(argv),
            "attribution": "verified_owner_or_parent_chain" if group != "other" else
                           "outside_verified_R2B4_process_groups"}


def _process_update(aggregate: dict[tuple[int, int], dict[str, Any]],
                    proc: Mapping[str, Any], group: str, root_pid: int,
                    argv_cache: dict[tuple[int, int], tuple[str, ...]],
                    elapsed: float, ticks_delta: int, hz: int, total_s: float) -> dict[str, Any]:
    identity = _proc_identity(proc)
    row = aggregate.get(identity)
    if row is None:
        row = _identity_row(int(proc["pid"]), proc, group, root_pid, argv_cache)
        row.update(cpu_ticks_delta=0, samples=0, cpu_pct_max_one_core=0.0,
                   processors_observed=set(), groups_seen=set(),
                   first_observed_s=total_s)
        aggregate[identity] = row
    if row["group"] != group and group != "other":
        # A verified group may appear after a supervisor finishes starting.
        corrected = _identity_row(int(proc["pid"]), proc, group, root_pid, argv_cache)
        row.update({k: corrected[k] for k in ("group", "role", "command", "attribution")})
    row["samples"] += 1
    row["cpu_ticks_delta"] += ticks_delta
    row["cpu_pct_max_one_core"] = max(row["cpu_pct_max_one_core"],
                                     100.0 * ticks_delta / hz / elapsed)
    row["processors_observed"].add(proc["processor"])
    row["groups_seen"].add(group)
    return row


def _new_thread_aggregate(pid: int, t: Mapping[str, Any], group: str,
                          process_start_ticks: int) -> dict[str, Any]:
    return {"pid": pid, "tid": t["tid"], "start_ticks": t["start_ticks"],
            "process_start_ticks": process_start_ticks,
            "comm": t["comm"], "group": group, "cpu_ticks_delta": 0,
            "cpu_pct_max_one_core": 0.0, "sched_runtime_ns": 0,
            "runqueue_wait_ns": 0, "timeslices": 0, "voluntary_ctxt_switches": 0,
            "nonvoluntary_ctxt_switches": 0, "processors_observed": set(),
            "allowed_cpu_sets_observed": set(), "samples": 0,
            "schedstat_samples": 0}


def _update_thread(aggregate: dict[tuple[int, int, int], dict[str, Any]],
                   pid: int, after: Mapping[str, Any], row: Mapping[str, Any],
                   group: str, ticks_delta: int, elapsed: float, hz: int,
                   process_start_ticks: int) -> None:
    key = (pid, process_start_ticks, int(after["tid"]), int(after["start_ticks"]))
    target = aggregate.setdefault(key, _new_thread_aggregate(pid, after, group, process_start_ticks))
    target["samples"] += 1
    target["schedstat_samples"] += int(isinstance(row.get("sched_runtime_ns"), int)
                                       and isinstance(row.get("runqueue_wait_ns"), int))
    target["cpu_ticks_delta"] += ticks_delta
    target["cpu_pct_max_one_core"] = max(target["cpu_pct_max_one_core"],
                                         100 * ticks_delta / hz / elapsed)
    for old, new in (("sched_runtime_ns", "sched_runtime_ns"),
                     ("runqueue_wait_ns", "runqueue_wait_ns"),
                     ("timeslices", "timeslices"),
                     ("voluntary_ctxt_switches", "voluntary_ctxt_switches"),
                     ("nonvoluntary_ctxt_switches", "nonvoluntary_ctxt_switches")):
        if isinstance(row.get(old), int):
            target[new] += row[old]
    target["processors_observed"].add(after["processor"])
    if after.get("allowed_cpus"):
        target["allowed_cpu_sets_observed"].add(after["allowed_cpus"])


def _mean_all(sums: Mapping[str, float], samples: int) -> dict[str, float]:
    return {k: (v / samples if samples else 0.0) for k, v in sums.items()}


def _pct(ticks: int, duration: float, hz: int) -> float:
    return 100 * ticks / hz / duration if duration > 0 else 0.0


def _thread_report(data: Mapping[str, Any], seconds: float, hz: int) -> dict[str, Any]:
    a = dict(data)
    for key in ("processors_observed", "allowed_cpu_sets_observed"):
        a[key] = sorted(a[key])
    a["cpu_pct_mean_one_core"] = _pct(a.pop("cpu_ticks_delta"), seconds, hz)
    sched_available = a["schedstat_samples"] > 0
    a["sched_runtime_ms"] = a.pop("sched_runtime_ns") / 1e6 if sched_available else None
    a["runqueue_wait_ms"] = a.pop("runqueue_wait_ns") / 1e6 if sched_available else None
    total = (a["runqueue_wait_ms"] + a["sched_runtime_ms"] if sched_available else 0)
    a["runqueue_wait_share"] = a["runqueue_wait_ms"] / total if total > 0 else None
    return a


def _process_report(data: Mapping[str, Any], seconds: float, hz: int) -> dict[str, Any]:
    a = dict(data)
    a["cpu_pct_mean_one_core"] = _pct(a["cpu_ticks_delta"], seconds, hz)
    a["processors_observed"] = sorted(a.pop("processors_observed"))
    a["groups_seen"] = sorted(a.pop("groups_seen"))
    return a


def _p95(values: list[float]) -> float | None:
    if not values:
        return None
    sorted_values = sorted(values)
    return sorted_values[max(0, math.ceil(.95 * len(sorted_values)) - 1)]


def _rate(v: float | None) -> str:
    return "n/a" if v is None else f"{v:6.2f}%"


def make_selftest_stat() -> str:
    rest = ["0"] * 37
    rest[0] = "S"
    rest[1] = "1"
    rest[2] = "123"
    rest[3] = "123"
    rest[11] = "120"
    rest[12] = "30"
    rest[19] = "999"
    rest[36] = "3"
    return "123 (r2b4 runtime) " + " ".join(rest)


def selftest() -> None:
    x = parse_proc_stat(make_selftest_stat())
    assert (x["pid"], x["comm"], x["utime"], x["stime"], x["starttime"],
            x["processor"], x["pgrp"]) == (123, "r2b4 runtime", 120, 30, 999, 3, 123)
    assert parse_schedstat("100 25 7\n") == (100, 25, 7)
    assert decode_throttled((1 << 2) | (1 << 18)) == ["THROTTLED_NOW", "THROTTLED_OCCURRED"]
    assert process_cpu_delta((100, 9), (200, 9), 1, 100) == 100.0
    assert process_cpu_delta((100, 9), (200, 10), 1, 100) is None
    assert _child_set(2, {2: {"ppid": 1}, 3: {"ppid": 2}, 4: {"ppid": 3}}) == {2, 3, 4}
    assert _is_runtime_command(("python3", "-m", "v3_process_runtime"))
    assert _supervisor_command(("python3", "-m", "tools.nav2_planner", "__supervise"), "tools.nav2_planner")
    assert _safe_command(("program", "--token", "supersecret"), "nav2") == "program --token [REDACTED]"
    assert parse_cpu_list("0-2,3") == {0, 1, 2, 3}
    print("cpu2 selftest: PASS (full-system P0)")


def run(root: Path, seconds: float, interval: float) -> int:
    runtime_pid, pid_source = find_runtime_pid(root)
    if runtime_pid is None:
        print(f"ERROR: resident runtime PID unavailable ({pid_source})")
        return 3

    affinity = load_affinity(root)
    monitor_affinity = pin_monitor_to_diagnostics_cpus(root)
    expected_control = (set(affinity["control_cpus"])
                        if affinity.get("enabled") is True and "control_cpus" in affinity else None)
    out_dir = root / "runtime/cpu"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    ndjson_path = out_dir / f"cpu2_{stamp}.ndjson"
    summary_path = out_dir / f"cpu2_{stamp}_summary.json"
    clk = int(os.sysconf("SC_CLK_TCK"))
    throttle_start = throttled()

    prev_processes = all_processes()
    if runtime_pid not in prev_processes:
        print("ERROR: runtime disappeared before measurement start")
        return 3
    runtime_identity = _proc_identity(prev_processes[runtime_pid])
    trusted: dict[tuple[int, int], str] = {}
    memberships, owners = classify_processes(root, prev_processes, runtime_pid, trusted)
    prev_threads = _threads_of([p for p in prev_processes if p in memberships])
    prev_cores = read_cores()
    started = prev_time = time.monotonic()
    argv_cache: dict[tuple[int, int], tuple[str, ...]] = {}
    proc_agg: dict[tuple[int, int], dict[str, Any]] = {}
    thread_agg: dict[tuple[int, int, int, int], dict[str, Any]] = {}
    total_group_ticks = {g: 0 for g in ("v3", "nav2", "slam", "other")}
    group_sums = {g: 0.0 for g in total_group_ticks}
    group_samples: dict[str, list[float]] = {g: [] for g in total_group_ticks}
    core_values: dict[int, list[float]] = {}
    freq_values: dict[int, list[float]] = {}
    temp_values: list[float] = []
    busy_values: list[float] = []
    other_values: list[float] = []
    residual_values: list[float] = []
    runtime_values: list[float] = []
    main_processors: set[int] = set()
    main_allowed: set[str] = set()
    main_runtime_ns = main_wait_ns = main_voluntary = main_involuntary = 0
    affinity_mismatches = 0
    missed_intervals = 0
    failed_proc_reads = 0
    reuses = 0
    observed_supervisors = {"nav2": False, "slam": False}
    supervisor_states: dict[str, Any] = owners
    sample_count = 0
    last_duration = 0.0
    next_print = 0.0
    runtime_exited = False

    print(f"R2B4 CPU2 full-system P0: {seconds:g}s interval={interval:g}s runtime_pid={runtime_pid}")
    print(f"runtime identity: pid={runtime_identity[0]} start_ticks={runtime_identity[1]} via {pid_source}")
    print("Groups: V3 subtree, verified Nav2 supervisor, verified SLAM supervisor, other processes.")
    print("Residual CPU = IRQ/kernel/short-lived/unreadable processes + sampling mismatch; not invented attribution.")
    print(f"Monitor affinity: {monitor_affinity}")

    with ndjson_path.open("w", encoding="utf-8", buffering=1) as log:
        while True:
            elapsed_total = time.monotonic() - started
            if elapsed_total >= seconds:
                break
            time.sleep(min(interval, seconds - elapsed_total))
            current_time = time.monotonic()
            duration = current_time - prev_time
            if duration <= 0:
                continue
            last_duration = current_time - started
            curr_cores = read_cores()
            busy = core_busy(prev_cores, curr_cores)
            curr_processes = all_processes()
            curr_runtime = curr_processes.get(runtime_pid)
            if curr_runtime is None or _proc_identity(curr_runtime) != runtime_identity:
                runtime_exited = True
                print("runtime process exited or PID was reused; safely ending CPU2 measurement")
                break

            curr_memberships, supervisors = classify_processes(root, curr_processes,
                                                                runtime_pid, trusted)
            for g in observed_supervisors:
                observed_supervisors[g] |= bool(supervisors[g]["verified"])
            supervisor_states = supervisors
            group_pcts = {g: 0.0 for g in total_group_ticks}
            process_rows = []
            matched_pids: set[int] = set()
            for pid, curr in curr_processes.items():
                prev = prev_processes.get(pid)
                if not _same_proc(prev, curr):
                    if prev is not None:
                        reuses += 1
                    continue
                tickdiff = _delta(curr["ticks"], prev["ticks"])
                if tickdiff is None:
                    failed_proc_reads += 1
                    continue
                matched_pids.add(pid)
                group = curr_memberships.get(pid, "other")
                # All processes are measured exactly once; groups never overlap.
                group_pcts[group] += 100.0 * tickdiff / clk / duration
                total_group_ticks[group] += tickdiff
                owner_pid = (runtime_pid if group == "v3" else
                             supervisors[group]["pid"] if group in supervisors and
                             supervisors[group]["verified"] else -1)
                agg = _process_update(proc_agg, curr, group, owner_pid,
                                      argv_cache, duration, tickdiff, clk, last_duration)
                if tickdiff > 0 or group != "other":
                    process_rows.append({"pid": pid, "start_ticks": curr["start_ticks"],
                                         "group": group, "role": agg["role"],
                                         "comm": curr["comm"], "processor": curr["processor"],
                                         "cpu_pct_one_core": 100 * tickdiff / clk / duration})
            # Snapshot per-thread scheduler and context-switch evidence only in
            # verified R2B4 groups, not every unrelated desktop process.
            curr_threads = _threads_of([pid for pid in curr_processes if pid in curr_memberships])
            active_thread_rows = []
            for (pid, tid), after in curr_threads.items():
                before = prev_threads.get((pid, tid))
                if before is None or pid not in matched_pids:
                    continue
                row = task_delta(before, after, duration, clk)
                if row is None:
                    continue
                tickdiff = _delta(after["cpu_ticks"], before["cpu_ticks"])
                if tickdiff is None:
                    continue
                group = curr_memberships.get(pid)
                if group is None:
                    continue
                _update_thread(thread_agg, pid, after, row, group, tickdiff, duration, clk,
                               int(curr_processes[pid]["start_ticks"]))
                active_thread_rows.append({"pid": pid, "group": group, **row})

            main = next((v for v in active_thread_rows if v["pid"] == runtime_pid
                         and v["tid"] == runtime_pid), None)
            if main:
                main_processors.add(main["processor"])
                if main["allowed_cpus"]:
                    main_allowed.add(main["allowed_cpus"])
                main_runtime_ns += main["sched_runtime_ns"] or 0
                main_wait_ns += main["runqueue_wait_ns"] or 0
                main_voluntary += main["voluntary_ctxt_switches"] or 0
                main_involuntary += main["nonvoluntary_ctxt_switches"] or 0
                if (expected_control is not None and main["allowed_cpus"] and
                        parse_cpu_list(main["allowed_cpus"]) != expected_control):
                    affinity_mismatches += 1

            core_count = len(busy)
            system_busy = sum(busy.values()) / core_count if core_count else None
            measured_process_busy = (sum(group_pcts.values()) / core_count
                                     if core_count else None)
            residual = (system_busy - measured_process_busy
                        if system_busy is not None and measured_process_busy is not None else None)
            measured_pct = (group_pcts["v3"] / core_count if core_count else None)
            external_pct = (group_pcts["other"] / core_count if core_count else None)
            if system_busy is not None:
                busy_values.append(system_busy)
            if residual is not None:
                residual_values.append(residual)
            if external_pct is not None:
                other_values.append(external_pct)
            for g, pct in group_pcts.items():
                group_sums[g] += pct
                group_samples[g].append(pct)
            for cpu, val in busy.items():
                core_values.setdefault(cpu, []).append(val)
            freq = frequencies_mhz()
            for cpu, val in freq.items():
                freq_values.setdefault(cpu, []).append(val)
            temp = temperature_c()
            if temp is not None:
                temp_values.append(temp)
            runtime_pct = group_pcts["v3"]
            runtime_control_pct = (100 * (curr_runtime["ticks"] - prev_processes[runtime_pid]["ticks"])
                                   / clk / duration)
            runtime_values.append(runtime_control_pct)
            if duration > max(1.5 * interval, interval + 0.05):
                missed_intervals += 1

            process_rows.sort(key=lambda x: x["cpu_pct_one_core"], reverse=True)
            active_thread_rows.sort(key=lambda x: x["cpu_pct_one_core"], reverse=True)
            v3_children = [{"pid": p["pid"], "comm": p["comm"],
                            "cpu_pct_one_core": p["cpu_pct_one_core"],
                            "processor": p["processor"]}
                           for p in process_rows if p["group"] == "v3" and p["pid"] != runtime_pid]
            sample = {
                "schema": SAMPLE_SCHEMA, "extension_schema": EXTENSION_SCHEMA,
                "timestamp": dt.datetime.now().isoformat(),
                "t_s": last_duration, "runtime_pid": runtime_pid,
                "core_busy_pct": {str(k): v for k, v in sorted(busy.items())},
                "core_frequency_mhz": {str(k): v for k, v in sorted(freq.items())},
                "temperature_c": temp,
                "runtime_process_cpu_pct_one_core": runtime_control_pct,
                "runtime_main_task": main,
                "runtime_thread_count": sum(p == runtime_pid for p, t in curr_threads),
                "top_runtime_threads": [x for x in active_thread_rows if x["pid"] == runtime_pid][:12],
                "descendant_processes": v3_children,
                "process_groups_cpu_pct_one_core": group_pcts,
                "process_groups_cpu_pct_entire_host":
                    ({k: val / core_count for k, val in group_pcts.items()} if core_count else {}),
                "system_cpu_busy_pct": system_busy,
                "observed_userspace_cpu_pct": measured_process_busy,
                "residual_unattributed_cpu_pct": residual,
                "idle_cpu_pct": (100 - system_busy if system_busy is not None else None),
                "observed_process_count": len(curr_processes),
                "sampled_process_delta_count": len(matched_pids),
                "supervisor_identity": supervisors,
                "top_system_processes": process_rows[:20],
                "top_group_threads": active_thread_rows[:30],
            }
            log.write(json.dumps(sample, allow_nan=False, sort_keys=True) + "\n")
            sample_count += 1
            if current_time - started >= next_print:
                parts = (f"{g}={_rate(group_pcts[g] / core_count if core_count else None)}"
                         for g in ("v3", "nav2", "slam", "other"))
                print(f"t={last_duration:5.1f}s host={_rate(system_busy)} "
                      + " ".join(parts)
                      + f" residual={_rate(residual)} temp={temp if temp is not None else 'n/a'}C")
                next_print = current_time - started + 1.0
            prev_processes = curr_processes
            prev_threads = curr_threads
            prev_cores = curr_cores
            prev_time = current_time

    # All means below use the same total observed wall-time denominator. Process
    # churn never inflates the mean by dropping quiet/missing intervals.
    actual_s = last_duration if sample_count else max(0.0, time.monotonic() - started)
    core_count = len(core_values)
    throttle_end = throttled()
    process_details = sorted((_process_report(v, actual_s, clk) for v in proc_agg.values()),
                             key=lambda x: x["cpu_pct_mean_one_core"], reverse=True)
    thread_details = sorted((_thread_report(v, actual_s, clk) for v in thread_agg.values()),
                            key=lambda x: x["cpu_pct_mean_one_core"], reverse=True)
    runtime_threads = [v for v in thread_details if v["pid"] == runtime_pid]
    # Also publish scheduler evidence per program (not just per thread/group).
    # An absent /proc/<pid>/task measurement is reported as unknown, not zero.
    process_thread_stats: dict[tuple[int, int], dict[str, Any]] = {}
    for thread in thread_details:
        key = (int(thread["pid"]), int(thread["process_start_ticks"]))
        a = process_thread_stats.setdefault(key, {
            "thread_identities_observed": 0, "sched_runtime_ms": 0.0,
            "runqueue_wait_ms": 0.0, "schedstat_samples": 0,
            "voluntary_ctxt_switches": 0,
            "nonvoluntary_ctxt_switches": 0,
            "allowed_cpu_sets_observed": set(),
        })
        a["thread_identities_observed"] += 1
        a["schedstat_samples"] += thread["schedstat_samples"]
        a["sched_runtime_ms"] += thread["sched_runtime_ms"] or 0.0
        a["runqueue_wait_ms"] += thread["runqueue_wait_ms"] or 0.0
        a["voluntary_ctxt_switches"] += thread["voluntary_ctxt_switches"]
        a["nonvoluntary_ctxt_switches"] += thread["nonvoluntary_ctxt_switches"]
        a["allowed_cpu_sets_observed"].update(thread["allowed_cpu_sets_observed"])
    for program in process_details:
        stats = process_thread_stats.get((program["pid"], program["start_ticks"])) if program["group"] != "other" else None
        if stats is None:
            program.update(thread_metrics_available=False,
                           thread_identities_observed=None,
                           sched_runtime_ms=None, runqueue_wait_ms=None,
                           runqueue_wait_share=None,
                           voluntary_ctxt_switches=None,
                           nonvoluntary_ctxt_switches=None,
                           allowed_cpu_sets_observed=[])
        else:
            program.update({k: (sorted(v) if isinstance(v, set) else v)
                            for k, v in stats.items()})
            program["thread_metrics_available"] = stats["schedstat_samples"] > 0
            duration = stats["sched_runtime_ms"] + stats["runqueue_wait_ms"]
            program["runqueue_wait_share"] = (stats["runqueue_wait_ms"] / duration
                                                  if duration > 0 else None)
            if not program["thread_metrics_available"]:
                program["sched_runtime_ms"] = None
                program["runqueue_wait_ms"] = None
    descendants_summary = [{"pid": p["pid"],
                            "cpu_pct_mean_one_core": p["cpu_pct_mean_one_core"],
                            "cpu_pct_max_one_core": p["cpu_pct_max_one_core"]}
                           for p in process_details if p["group"] == "v3" and p["pid"] != runtime_pid]
    group_report = {}
    for group, total in total_group_ticks.items():
        one_core_mean = _pct(total, actual_s, clk)
        values = group_samples[group]
        group_report[group] = {
            "cpu_pct_mean_one_core": one_core_mean,
            "cpu_pct_mean_entire_host": (one_core_mean / core_count if core_count else None),
            "cpu_pct_max_one_core": max(values) if values else None,
            "cpu_pct_p95_one_core": _p95(values),
            "process_identities_observed": sum(1 for p in process_details if p["group"] == group),
            "scheduler_runqueue_wait_ms": sum(t["runqueue_wait_ms"] or 0.0 for t in thread_details
                                              if t["group"] == group),
            "schedstat_samples": sum(t["schedstat_samples"] for t in thread_details
                                     if t["group"] == group),
            "voluntary_ctxt_switches": sum(t["voluntary_ctxt_switches"] for t in thread_details
                                            if t["group"] == group),
            "nonvoluntary_ctxt_switches": sum(t["nonvoluntary_ctxt_switches"] for t in thread_details
                                               if t["group"] == group),
        }
    main_wait_share = (main_wait_ns / (main_runtime_ns + main_wait_ns)
                       if main_runtime_ns + main_wait_ns else None)
    host_mean = _mean(busy_values)
    group_mean_sum = sum(g["cpu_pct_mean_entire_host"] or 0.0 for g in group_report.values())
    residual_mean = (host_mean - group_mean_sum) if host_mean is not None else None
    summary = {
        "schema": SUMMARY_SCHEMA, "extension_schema": EXTENSION_SCHEMA,
        "claim_policy": {
            "root_cause_inferred": False,
            "statement": "Linux /proc and scheduler evidence; no V3/ROS algorithm root-cause inference.",
        },
        "measurement": {
            "requested_duration_s": seconds, "actual_duration_s": actual_s,
            "interval_s": interval, "sample_count": sample_count,
            "runtime_pid": runtime_pid, "runtime_pid_source": pid_source,
            "runtime_start_ticks": runtime_identity[1],
            "clock_ticks_per_second": clk, "monitor_affinity": monitor_affinity,
            "configured_runtime_affinity": affinity,
            "runtime_exited_or_reused": runtime_exited,
            "delayed_sampling_intervals": missed_intervals,
            "pid_reuses_observed": reuses, "failed_process_delta_count": failed_proc_reads,
        },
        "core_busy": {str(cpu): {"mean_pct": _mean(vals), "max_pct": max(vals)}
                      for cpu, vals in sorted(core_values.items())},
        "core_frequency": {str(cpu): {"min_mhz": min(vals),
                                      "mean_mhz": _mean(vals), "max_mhz": max(vals)}
                           for cpu, vals in sorted(freq_values.items())},
        "runtime_process": {"cpu_pct_mean_one_core": _mean(runtime_values),
                            "cpu_pct_max_one_core": max(runtime_values) if runtime_values else None},
        "runtime_main_task": {
            "expected_control_cpus": sorted(expected_control) if expected_control is not None else None,
            "processors_observed": sorted(main_processors),
            "allowed_cpu_sets_observed": sorted(main_allowed),
            "affinity_mismatch_sample_count": affinity_mismatches,
            "sched_runtime_ms": main_runtime_ns / 1e6,
            "runqueue_wait_ms": main_wait_ns / 1e6,
            "runqueue_wait_share": main_wait_share,
            "voluntary_ctxt_switches": main_voluntary,
            "nonvoluntary_ctxt_switches": main_involuntary,
        },
        "runtime_threads": runtime_threads,
        "descendant_processes": descendants_summary,
        "whole_system_cpu": {
            "busy_mean_pct": host_mean, "busy_p95_pct": _p95(busy_values),
            "busy_max_pct": max(busy_values) if busy_values else None,
            "idle_mean_pct": 100 - host_mean if host_mean is not None else None,
            "named_process_groups_mean_pct":
                (sum(group_report[g]["cpu_pct_mean_entire_host"] or 0
                     for g in ("v3", "nav2", "slam")) if core_count else None),
            "other_userspace_mean_pct": group_report["other"]["cpu_pct_mean_entire_host"],
            "residual_unattributed_mean_pct": residual_mean,
            "reconciliation": "busy = V3 + Nav2 + SLAM + other /proc processes + residual",
            "residual_definition": "IRQ/kernel activity, short-lived/unreadable processes and sampling mismatch",
            "residual_is_not_per_process_attribution": True,
            "residual_may_be_negative_due_to_sampling": True,
            "sample_residual_min_pct": min(residual_values) if residual_values else None,
            "sample_residual_max_pct": max(residual_values) if residual_values else None,
        },
        "process_groups": group_report,
        "supervisors": {**supervisor_states, "ever_verified": observed_supervisors},
        "system_processes": process_details,
        "system_threads": thread_details,
        "thermal": {"temperature_mean_c": _mean(temp_values),
                    "temperature_max_c": max(temp_values) if temp_values else None,
                    "throttled_start": throttle_start, "throttled_end": throttle_end},
        "limitations": [
            "Process CPU is sampled from /proc; tasks shorter than a sampling interval can disappear.",
            "Threads of verified R2B4 groups are measured; unrelated process threads are not.",
            "A Python multiprocessing spawn command cannot prove an individual V3 worker role.",
            "Per-core process attribution is not available from last-CPU snapshots; core busy is measured.",
            "Residual host CPU includes IRQ/kernel and measurement/permission differences.",
            "Runqueue wait is scheduler evidence, not an algorithmic cause verdict.",
            "This tool performs read-only diagnostics and may itself add small CPU load.",
        ],
        "artifacts": {"samples_ndjson": str(ndjson_path), "summary_json": str(summary_path)},
    }
    summary_path.write_text(json.dumps(summary, sort_keys=True, indent=2,
                                       ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")

    print("\nCPU2 FULL-SYSTEM SUMMARY (passive observation)")
    print(f"host busy (4-core-normalized): {_rate(host_mean)}")
    for g in ("v3", "nav2", "slam", "other"):
        row = group_report[g]
        print(f"  {g:6} {_rate(row['cpu_pct_mean_entire_host'])} of host"
              f"  ({_rate(row['cpu_pct_mean_one_core'])} of one core)")
    print(f"residual, not attributed: {_rate(residual_mean)}")
    print("verified supervisor observed: " + ", ".join(
        f"{name}={observed_supervisors[name]}" for name in ("nav2", "slam")))
    print("\nTop processes (percent of ONE core; incl. external processes):")
    for row in [p for p in process_details if p["cpu_pct_mean_one_core"] > 0][:20]:
        print(f"  {_rate(row['cpu_pct_mean_one_core'])} PID={row['pid']:<7}"
              f" {row['role']:<30} {row['command'][:78]}")
    print("\nTop R2B4 threads (percent of ONE core, scheduler waiting measured):")
    for row in thread_details[:12]:
        print(f"  {_rate(row['cpu_pct_mean_one_core'])} PID={row['pid']:<7}"
              f" TID={row['tid']:<7} {row['group']:<5}"
              f" wait={row['runqueue_wait_ms'] if row['runqueue_wait_ms'] is not None else 'n/a'}ms"
              f" voluntary={row['voluntary_ctxt_switches']}"
              f" involuntary={row['nonvoluntary_ctxt_switches']}")
    print(f"max temperature: {max(temp_values) if temp_values else None} C")
    print(f"throttle start: {throttle_start} | end: {throttle_end}")
    print("No root-cause claim was inferred.")
    print(f"cpu2 samples saved: {ndjson_path}")
    print(f"cpu2 summary saved: {summary_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="R2B4 whole-system /proc CPU2 P0 diagnostics")
    parser.add_argument("seconds", nargs="?", type=float, default=30.0)
    parser.add_argument("interval", nargs="?", type=float, default=0.5)
    parser.add_argument("--root", type=Path, default=Path("/home/alba/project_r2b4"))
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args(argv)
    if args.selftest:
        selftest()
        return 0
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds must be finite and > 0")
    if not math.isfinite(args.interval) or args.interval <= 0:
        parser.error("interval must be finite and > 0")
    return run(args.root.resolve(), args.seconds, args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
