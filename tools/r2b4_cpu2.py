#!/usr/bin/env python3
"""Deep factual Linux /proc diagnostics for the R2B4 runtime.

No V3 control/hardware authority is used.  The tool reports scheduler/process
facts only and intentionally emits no root-cause verdict.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import Mapping

SAMPLE_SCHEMA = "R2B4_CPU2_SAMPLE_V1"
SUMMARY_SCHEMA = "R2B4_CPU2_SUMMARY_V1"


def parse_proc_stat(text: str) -> dict[str, int | str]:
    left = text.find("(")
    right = text.rfind(")")
    if left <= 0 or right <= left:
        raise ValueError("invalid /proc stat row")
    pid = int(text[:left].strip())
    comm = text[left + 1:right]
    rest = text[right + 1:].strip().split()  # rest[0] == field 3
    if len(rest) < 37:
        raise ValueError("truncated /proc stat row")
    return {
        "pid": pid,
        "comm": comm,
        "ppid": int(rest[1]),        # field 4
        "utime": int(rest[11]),      # field 14
        "stime": int(rest[12]),      # field 15
        "starttime": int(rest[19]),  # field 22
        "processor": int(rest[36]),  # field 39
    }


def parse_schedstat(text: str) -> tuple[int, int, int]:
    parts = text.split()
    if len(parts) < 3:
        raise ValueError("invalid schedstat")
    return int(parts[0]), int(parts[1]), int(parts[2])


def decode_throttled(value: int) -> list[str]:
    names = {
        0: "UNDER_VOLTAGE_NOW",
        1: "ARM_FREQUENCY_CAPPED_NOW",
        2: "THROTTLED_NOW",
        3: "SOFT_TEMPERATURE_LIMIT_NOW",
        16: "UNDER_VOLTAGE_OCCURRED",
        17: "ARM_FREQUENCY_CAPPED_OCCURRED",
        18: "THROTTLED_OCCURRED",
        19: "SOFT_TEMPERATURE_LIMIT_OCCURRED",
    }
    return [name for bit, name in names.items() if value & (1 << bit)]


def _read(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError):
        return None


def _status(path: Path) -> dict[str, str]:
    text = _read(path)
    out: dict[str, str] = {}
    if text:
        for line in text.splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                out[k.strip()] = v.strip()
    return out


def _status_int(values: Mapping[str, str], key: str) -> int | None:
    raw = values.get(key)
    if raw is None:
        return None
    try:
        return int(raw.split()[0])
    except (ValueError, IndexError):
        return None


def read_task(pid: int, tid: int | None = None) -> dict[str, object] | None:
    tid = pid if tid is None else tid
    base = Path(f"/proc/{pid}/task/{tid}")
    stat_text = _read(base / "stat")
    if stat_text is None:
        return None
    try:
        parsed = parse_proc_stat(stat_text)
    except (TypeError, ValueError):
        return None
    sched_runtime = sched_wait = sched_slices = None
    sched_text = _read(base / "schedstat")
    if sched_text:
        try:
            sched_runtime, sched_wait, sched_slices = parse_schedstat(sched_text)
        except (TypeError, ValueError):
            pass
    status = _status(base / "status")
    return {
        "tid": tid,
        "comm": str(parsed["comm"]),
        "ppid": int(parsed["ppid"]),
        "cpu_ticks": int(parsed["utime"]) + int(parsed["stime"]),
        "processor": int(parsed["processor"]),
        "start_ticks": int(parsed["starttime"]),
        "sched_runtime_ns": sched_runtime,
        "sched_wait_ns": sched_wait,
        "sched_slices": sched_slices,
        "voluntary_ctxt": _status_int(status, "voluntary_ctxt_switches"),
        "nonvoluntary_ctxt": _status_int(status, "nonvoluntary_ctxt_switches"),
        "allowed_cpus": status.get("Cpus_allowed_list"),
    }


def read_process_ticks(pid: int) -> tuple[int, int] | None:
    text = _read(Path(f"/proc/{pid}/stat"))
    if text is None:
        return None
    try:
        row = parse_proc_stat(text)
    except (TypeError, ValueError):
        return None
    return int(row["utime"]) + int(row["stime"]), int(row["starttime"])


def task_ids(pid: int) -> list[int]:
    try:
        return sorted(
            int(p.name) for p in Path(f"/proc/{pid}/task").iterdir() if p.name.isdigit()
        )
    except OSError:
        return []


def descendants(pid: int) -> list[int]:
    seen: set[int] = set()
    queue = [pid]
    while queue:
        parent = queue.pop(0)
        text = _read(Path(f"/proc/{parent}/task/{parent}/children"))
        if not text:
            continue
        for raw in text.split():
            try:
                child = int(raw)
            except ValueError:
                continue
            if child not in seen:
                seen.add(child)
                queue.append(child)
    return sorted(seen)


def read_cores() -> dict[int, tuple[int, int]]:
    out: dict[int, tuple[int, int]] = {}
    text = _read(Path("/proc/stat"))
    if not text:
        return out
    for line in text.splitlines():
        parts = line.split()
        if not parts or not parts[0].startswith("cpu") or not parts[0][3:].isdigit():
            continue
        values = [int(v) for v in parts[1:]]
        total = sum(values)
        idle = (values[3] if len(values) > 3 else 0) + (values[4] if len(values) > 4 else 0)
        out[int(parts[0][3:])] = (total, idle)
    return out


def core_busy(before: Mapping[int, tuple[int, int]], after: Mapping[int, tuple[int, int]]) -> dict[int, float]:
    out: dict[int, float] = {}
    for cpu, now in after.items():
        old = before.get(cpu)
        if old is None:
            continue
        total = now[0] - old[0]
        idle = now[1] - old[1]
        if total > 0:
            out[cpu] = max(0.0, min(100.0, 100.0 * (total - idle) / total))
    return out


def frequencies_mhz() -> dict[int, float]:
    out: dict[int, float] = {}
    for base in Path("/sys/devices/system/cpu").glob("cpu[0-9]*"):
        suffix = base.name[3:]
        text = _read(base / "cpufreq/scaling_cur_freq")
        if suffix.isdigit() and text:
            try:
                out[int(suffix)] = float(text.strip()) / 1000.0
            except ValueError:
                pass
    return out


def temperature_c() -> float | None:
    text = _read(Path("/sys/class/thermal/thermal_zone0/temp"))
    if not text:
        return None
    try:
        value = float(text.strip())
    except ValueError:
        return None
    return value / 1000.0 if value > 1000.0 else value


def throttled() -> dict[str, object]:
    try:
        cp = subprocess.run(
            ["vcgencmd", "get_throttled"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.SubprocessError):
        return {"available": False, "raw": None, "flags": []}
    text = cp.stdout.strip()
    if "=" not in text:
        return {"available": False, "raw": text or None, "flags": []}
    raw = text.split("=", 1)[1].strip()
    try:
        value = int(raw, 16)
    except ValueError:
        return {"available": False, "raw": raw, "flags": []}
    return {"available": True, "raw": raw, "value": value, "flags": decode_throttled(value)}


def load_affinity(root: Path) -> dict[str, object]:
    try:
        value = json.loads((root / "conf/vezerles.json").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return {}
    raw = value.get("runtime_affinity") if isinstance(value, dict) else None
    return dict(raw) if isinstance(raw, dict) else {}


def find_runtime_pid(root: Path) -> tuple[int | None, str]:
    text = _read(root / "runtime/.r2b4_runtime_pid")
    if text:
        try:
            pid = int(text.strip())
        except ValueError:
            pid = -1
        if pid > 1 and Path(f"/proc/{pid}").is_dir():
            return pid, "runtime/.r2b4_runtime_pid"
    matches: list[int] = []
    try:
        proc_entries = list(Path("/proc").iterdir())
    except OSError:
        proc_entries = []
    for entry in proc_entries:
        if not entry.name.isdigit():
            continue
        cmdline = _read(entry / "cmdline")
        if cmdline and "v3_process_runtime" in cmdline.replace("\x00", " "):
            matches.append(int(entry.name))
    if len(matches) == 1:
        return matches[0], "proc_cmdline_fallback"
    return None, "not_found" if not matches else "ambiguous_proc_cmdline"


def _delta(now: object, old: object) -> int | None:
    if isinstance(now, int) and isinstance(old, int) and now >= old:
        return now - old
    return None


def task_delta(before: Mapping[str, object], after: Mapping[str, object], elapsed: float, clk_tck: int) -> dict[str, object] | None:
    if before.get("start_ticks") != after.get("start_ticks") or elapsed <= 0:
        return None
    cpu_now = after.get("cpu_ticks")
    cpu_old = before.get("cpu_ticks")
    if not isinstance(cpu_now, int) or not isinstance(cpu_old, int) or cpu_now < cpu_old:
        return None
    runtime_ns = _delta(after.get("sched_runtime_ns"), before.get("sched_runtime_ns"))
    wait_ns = _delta(after.get("sched_wait_ns"), before.get("sched_wait_ns"))
    slices = _delta(after.get("sched_slices"), before.get("sched_slices"))
    voluntary = _delta(after.get("voluntary_ctxt"), before.get("voluntary_ctxt"))
    involuntary = _delta(after.get("nonvoluntary_ctxt"), before.get("nonvoluntary_ctxt"))
    cpu_pct = 100.0 * ((cpu_now - cpu_old) / clk_tck) / elapsed
    wait_share = (
        wait_ns / (runtime_ns + wait_ns)
        if isinstance(runtime_ns, int) and isinstance(wait_ns, int) and runtime_ns + wait_ns > 0
        else None
    )
    return {
        "tid": after.get("tid"),
        "comm": after.get("comm"),
        "cpu_pct_one_core": cpu_pct,
        "processor": after.get("processor"),
        "allowed_cpus": after.get("allowed_cpus"),
        "sched_runtime_ns": runtime_ns,
        "runqueue_wait_ns": wait_ns,
        "runqueue_wait_share": wait_share,
        "timeslices": slices,
        "voluntary_ctxt_switches": voluntary,
        "nonvoluntary_ctxt_switches": involuntary,
    }


def process_cpu_delta(before: tuple[int, int] | None, after: tuple[int, int] | None, elapsed: float, clk_tck: int) -> float | None:
    if before is None or after is None or before[1] != after[1] or elapsed <= 0:
        return None
    diff = after[0] - before[0]
    return None if diff < 0 else 100.0 * (diff / clk_tck) / elapsed


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def pin_monitor_to_io_cpu(affinity: Mapping[str, object]) -> dict[str, object]:
    cpu = affinity.get("io_cpu")
    if affinity.get("enabled") is not True or not isinstance(cpu, int):
        return {"attempted": False, "applied": False, "cpu": None}
    setter = getattr(os, "sched_setaffinity", None)
    getter = getattr(os, "sched_getaffinity", None)
    if not callable(setter) or not callable(getter):
        return {"attempted": True, "applied": False, "cpu": cpu, "error": "unavailable"}
    try:
        setter(0, {cpu})
        allowed = sorted(int(v) for v in getter(0))
        return {"attempted": True, "applied": allowed == [cpu], "cpu": cpu, "allowed_cpus": allowed}
    except OSError as exc:
        return {"attempted": True, "applied": False, "cpu": cpu, "error": f"{type(exc).__name__}:{exc}"}


def make_selftest_stat() -> str:
    rest = ["0"] * 37
    rest[0] = "S"      # field 3
    rest[1] = "1"      # ppid field 4
    rest[11] = "120"   # utime field 14
    rest[12] = "30"    # stime field 15
    rest[19] = "999"   # starttime field 22
    rest[36] = "3"     # processor field 39
    return "123 (r2b4 runtime) " + " ".join(rest)


def selftest() -> None:
    row = parse_proc_stat(make_selftest_stat())
    assert row["pid"] == 123
    assert row["comm"] == "r2b4 runtime"
    assert row["utime"] == 120
    assert row["stime"] == 30
    assert row["starttime"] == 999
    assert row["processor"] == 3
    assert parse_schedstat("100 25 7\n") == (100, 25, 7)
    assert decode_throttled((1 << 2) | (1 << 18)) == ["THROTTLED_NOW", "THROTTLED_OCCURRED"]
    print("cpu2 selftest: PASS")


def run(root: Path, seconds: float, interval: float) -> int:
    pid, pid_source = find_runtime_pid(root)
    if pid is None:
        print(f"ERROR: resident runtime PID unavailable ({pid_source})")
        return 3

    affinity = load_affinity(root)
    monitor_affinity = pin_monitor_to_io_cpu(affinity)
    expected_runtime_cpu = (
        affinity.get("runtime_cpu")
        if affinity.get("enabled") is True and isinstance(affinity.get("runtime_cpu"), int)
        else None
    )

    out_dir = root / "runtime/cpu"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    ndjson_path = out_dir / f"cpu2_{stamp}.ndjson"
    summary_path = out_dir / f"cpu2_{stamp}_summary.json"

    clk_tck = int(os.sysconf("SC_CLK_TCK"))
    prev_core = read_cores()
    prev_proc = read_process_ticks(pid)
    prev_tasks = {tid: row for tid in task_ids(pid) if (row := read_task(pid, tid)) is not None}
    prev_children = {child: read_process_ticks(child) for child in descendants(pid)}
    prev_time = time.monotonic()
    started = prev_time
    throttle_start = throttled()

    core_values: dict[int, list[float]] = {}
    freq_values: dict[int, list[float]] = {}
    proc_values: list[float] = []
    temp_values: list[float] = []
    main_runtime_ns = 0
    main_wait_ns = 0
    main_voluntary = 0
    main_involuntary = 0
    main_processors: set[int] = set()
    main_allowed: set[str] = set()
    affinity_mismatches = 0
    thread_agg: dict[str, dict[str, object]] = {}
    child_values: dict[int, list[float]] = {}
    samples = 0
    next_print = 0.0

    print(f"R2B4 cpu2: {seconds:g}s interval={interval:g}s runtime_pid={pid}")
    print(f"PID source: {pid_source}")
    print(f"monitor affinity: {monitor_affinity}")
    print("Policy: factual Linux observations only; no root-cause verdict.")

    with ndjson_path.open("w", encoding="utf-8", buffering=1) as log:
        while True:
            total_elapsed = time.monotonic() - started
            if total_elapsed >= seconds:
                break
            time.sleep(min(interval, seconds - total_elapsed))
            now = time.monotonic()
            elapsed = now - prev_time
            if elapsed <= 0:
                continue

            cur_core = read_cores()
            busy = core_busy(prev_core, cur_core)
            cur_proc = read_process_ticks(pid)
            if cur_proc is None:
                print("runtime process exited during cpu2 measurement")
                break
            proc_pct = process_cpu_delta(prev_proc, cur_proc, elapsed, clk_tck)

            cur_tasks = {tid: row for tid in task_ids(pid) if (row := read_task(pid, tid)) is not None}
            task_rows: list[dict[str, object]] = []
            for tid, after in cur_tasks.items():
                before = prev_tasks.get(tid)
                if before is None:
                    continue
                row = task_delta(before, after, elapsed, clk_tck)
                if row is None:
                    continue
                task_rows.append(row)
                key = f"{tid}:{after['start_ticks']}:{after['comm']}"
                agg = thread_agg.setdefault(
                    key,
                    {
                        "tid": tid,
                        "comm": after["comm"],
                        "cpu": [],
                        "runtime_ns": 0,
                        "wait_ns": 0,
                        "voluntary": 0,
                        "involuntary": 0,
                        "processors": set(),
                        "allowed": set(),
                    },
                )
                agg["cpu"].append(float(row["cpu_pct_one_core"]))
                if isinstance(row["sched_runtime_ns"], int):
                    agg["runtime_ns"] += row["sched_runtime_ns"]
                if isinstance(row["runqueue_wait_ns"], int):
                    agg["wait_ns"] += row["runqueue_wait_ns"]
                if isinstance(row["voluntary_ctxt_switches"], int):
                    agg["voluntary"] += row["voluntary_ctxt_switches"]
                if isinstance(row["nonvoluntary_ctxt_switches"], int):
                    agg["involuntary"] += row["nonvoluntary_ctxt_switches"]
                if isinstance(row["processor"], int):
                    agg["processors"].add(row["processor"])
                if isinstance(row["allowed_cpus"], str):
                    agg["allowed"].add(row["allowed_cpus"])

            main = next((row for row in task_rows if row["tid"] == pid), None)
            if main:
                if isinstance(main["processor"], int):
                    main_processors.add(main["processor"])
                if isinstance(main["allowed_cpus"], str):
                    main_allowed.add(main["allowed_cpus"])
                if isinstance(main["sched_runtime_ns"], int):
                    main_runtime_ns += main["sched_runtime_ns"]
                if isinstance(main["runqueue_wait_ns"], int):
                    main_wait_ns += main["runqueue_wait_ns"]
                if isinstance(main["voluntary_ctxt_switches"], int):
                    main_voluntary += main["voluntary_ctxt_switches"]
                if isinstance(main["nonvoluntary_ctxt_switches"], int):
                    main_involuntary += main["nonvoluntary_ctxt_switches"]
                if expected_runtime_cpu is not None and str(main["allowed_cpus"]) != str(expected_runtime_cpu):
                    affinity_mismatches += 1

            cur_children = {child: read_process_ticks(child) for child in descendants(pid)}
            child_rows = []
            for child, after in cur_children.items():
                pct = process_cpu_delta(prev_children.get(child), after, elapsed, clk_tck)
                task = read_task(child)
                if pct is not None:
                    child_values.setdefault(child, []).append(pct)
                child_rows.append(
                    {
                        "pid": child,
                        "comm": task.get("comm") if task else None,
                        "cpu_pct_one_core": pct,
                        "processor": task.get("processor") if task else None,
                        "allowed_cpus": task.get("allowed_cpus") if task else None,
                    }
                )

            freq = frequencies_mhz()
            temp = temperature_c()
            for cpu, value in busy.items():
                core_values.setdefault(cpu, []).append(value)
            for cpu, value in freq.items():
                freq_values.setdefault(cpu, []).append(value)
            if proc_pct is not None:
                proc_values.append(proc_pct)
            if temp is not None:
                temp_values.append(temp)

            task_rows.sort(key=lambda row: float(row.get("cpu_pct_one_core") or 0.0), reverse=True)
            sample = {
                "schema": SAMPLE_SCHEMA,
                "timestamp": dt.datetime.now().isoformat(),
                "t_s": now - started,
                "runtime_pid": pid,
                "core_busy_pct": {str(k): v for k, v in sorted(busy.items())},
                "core_frequency_mhz": {str(k): v for k, v in sorted(freq.items())},
                "temperature_c": temp,
                "runtime_process_cpu_pct_one_core": proc_pct,
                "runtime_main_task": main,
                "runtime_thread_count": len(cur_tasks),
                "top_runtime_threads": task_rows[:12],
                "descendant_processes": child_rows,
            }
            log.write(json.dumps(sample, sort_keys=True, allow_nan=False) + "\n")
            samples += 1

            if now - started >= next_print:
                cores = " ".join(f"CPU{cpu}={val:5.1f}%" for cpu, val in sorted(busy.items()))
                main_cpu = "n/a" if not main else f"{float(main['cpu_pct_one_core']):5.1f}%"
                wait = "n/a" if not main or main.get("runqueue_wait_share") is None else f"{100*float(main['runqueue_wait_share']):4.1f}%"
                psr = "n/a" if not main else str(main.get("processor"))
                temp_text = "n/a" if temp is None else f"{temp:.1f}C"
                proc_text = "n/a" if proc_pct is None else f"{proc_pct:.1f}%"
                print(
                    f"t={now-started:5.1f}s {cores} | runtime={proc_text} "
                    f"main={main_cpu} PSR={psr} runqueue_wait_share={wait} temp={temp_text}"
                )
                next_print = now - started + 1.0

            prev_core = cur_core
            prev_proc = cur_proc
            prev_tasks = cur_tasks
            prev_children = cur_children
            prev_time = now

    throttle_end = throttled()

    thread_summary = []
    for agg in thread_agg.values():
        runtime_ns = int(agg["runtime_ns"])
        wait_ns = int(agg["wait_ns"])
        thread_summary.append(
            {
                "tid": agg["tid"],
                "comm": agg["comm"],
                "cpu_pct_mean_one_core": _mean(list(agg["cpu"])),
                "cpu_pct_max_one_core": max(agg["cpu"]) if agg["cpu"] else None,
                "sched_runtime_ms": runtime_ns / 1_000_000.0,
                "runqueue_wait_ms": wait_ns / 1_000_000.0,
                "runqueue_wait_share": wait_ns / (runtime_ns + wait_ns) if runtime_ns + wait_ns else None,
                "voluntary_ctxt_switches": agg["voluntary"],
                "nonvoluntary_ctxt_switches": agg["involuntary"],
                "processors_observed": sorted(agg["processors"]),
                "allowed_cpu_sets_observed": sorted(agg["allowed"]),
            }
        )
    thread_summary.sort(key=lambda row: float(row.get("cpu_pct_mean_one_core") or 0.0), reverse=True)

    main_wait_share = main_wait_ns / (main_runtime_ns + main_wait_ns) if main_runtime_ns + main_wait_ns else None
    summary = {
        "schema": SUMMARY_SCHEMA,
        "claim_policy": {
            "root_cause_inferred": False,
            "statement": (
                "cpu2 reports Linux scheduler/process/thread facts only. These observations "
                "do not by themselves identify a V3 layer as the runtime-loop root cause."
            ),
        },
        "measurement": {
            "requested_duration_s": seconds,
            "actual_duration_s": max(0.0, time.monotonic() - started),
            "interval_s": interval,
            "sample_count": samples,
            "runtime_pid": pid,
            "runtime_pid_source": pid_source,
            "clock_ticks_per_second": clk_tck,
            "monitor_affinity": monitor_affinity,
            "configured_runtime_affinity": affinity,
        },
        "core_busy": {
            str(cpu): {"mean_pct": _mean(values), "max_pct": max(values) if values else None}
            for cpu, values in sorted(core_values.items())
        },
        "core_frequency": {
            str(cpu): {
                "min_mhz": min(values) if values else None,
                "mean_mhz": _mean(values),
                "max_mhz": max(values) if values else None,
            }
            for cpu, values in sorted(freq_values.items())
        },
        "runtime_process": {
            "cpu_pct_mean_one_core": _mean(proc_values),
            "cpu_pct_max_one_core": max(proc_values) if proc_values else None,
        },
        "runtime_main_task": {
            "expected_runtime_cpu": expected_runtime_cpu,
            "processors_observed": sorted(main_processors),
            "allowed_cpu_sets_observed": sorted(main_allowed),
            "affinity_mismatch_sample_count": affinity_mismatches,
            "sched_runtime_ms": main_runtime_ns / 1_000_000.0,
            "runqueue_wait_ms": main_wait_ns / 1_000_000.0,
            "runqueue_wait_share": main_wait_share,
            "voluntary_ctxt_switches": main_voluntary,
            "nonvoluntary_ctxt_switches": main_involuntary,
        },
        "runtime_threads": thread_summary,
        "descendant_processes": [
            {
                "pid": child,
                "cpu_pct_mean_one_core": _mean(values),
                "cpu_pct_max_one_core": max(values) if values else None,
            }
            for child, values in sorted(child_values.items())
        ],
        "thermal": {
            "temperature_mean_c": _mean(temp_values),
            "temperature_max_c": max(temp_values) if temp_values else None,
            "throttled_start": throttle_start,
            "throttled_end": throttle_end,
        },
        "limitations": [
            "No Python/V3 layer execution duration is measured.",
            "No causal root-cause classification is emitted.",
            "Runqueue wait is Linux scheduler evidence, not layer attribution.",
            "CPU percentages are interval deltas and may vary with sampling.",
        ],
        "artifacts": {
            "samples_ndjson": str(ndjson_path),
            "summary_json": str(summary_path),
        },
    }
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    print()
    print("CPU2 SUMMARY (direct observations only)")
    print(f"runtime process CPU mean: {_mean(proc_values)} % of one core")
    print(f"runtime main runqueue wait: {main_wait_ns/1_000_000.0:.3f} ms (share={main_wait_share})")
    print(f"runtime main processors observed: {sorted(main_processors)}")
    print(f"runtime main allowed CPU sets: {sorted(main_allowed)}")
    print(f"affinity mismatch samples: {affinity_mismatches}")
    print(f"max temperature: {max(temp_values) if temp_values else None} C")
    print(f"throttle start: {throttle_start}")
    print(f"throttle end:   {throttle_end}")
    print("No root-cause claim was inferred.")
    print(f"cpu2 summary saved: {summary_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="R2B4 deep /proc runtime diagnostics")
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
