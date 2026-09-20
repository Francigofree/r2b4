#!/usr/bin/env python3
"""
R2B4 V3 LIVE control-loop root-cause diagnostic.

This tool starts the NORMAL production resident runtime through OperatorController,
with real motors, real encoders, real IMU/LiDAR/camera/person, real L0-L12,
real capture and normal command/status/PID files.

Instrumentation is injected only into the v3_process_runtime.py child via a
temporary sitecustomize.py inherited through PYTHONPATH. No repository source
file is modified.

Primary measurement is deliberately low-perturbation:
  - one perf_counter_ns() pair around the original NativeLiveInputReader.read()
  - one perf_counter_ns() pair around each original production source.read()
  - the existing RuntimeTimingAccumulator values are copied with list.append()
  - command mode is recorded per tick
  - optional detailed thread-CPU / Linux schedstat sampling is sparse; those
    sampled ticks are excluded from the primary latency statistics

Use:
  Terminal 1:
    python3 tools/r2b4_live_control_rootcause_diag.py --capture-mode full

  When READY appears, Terminal 2:
    r rc 30 c full

The diagnostic detects the first ACTIVE motion interval. After it returns to
idle for a short tail, it safely shuts down the resident runtime, writes the
evidence JSON and prints the concrete dominant root-cause path.

Ctrl+C also performs a normal operator runtime shutdown.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

SCHEMA = "R2B4_V3_LIVE_CONTROL_ROOTCAUSE_DIAG_V1"

SOURCE_TARGETS = {
    "ENCODER": [
        "v3/adapters/gpio_encoder.py",
        "v3/adapters/counter_encoder.py",
        "v3/adapters/gpio_counter.py",
    ],
    "IMU": [
        "v3/adapters/bno055_device.py",
        "v3/adapters/bno055_imu.py",
        "v3/adapters/live_imu.py",
    ],
    "LIDAR": [
        "v3/adapters/latest_lidar.py",
        "v3/adapters/live_lidar.py",
        "v3/adapters/native_lidar_port.py",
    ],
    "CAMERA": [
        "v3/adapters/live_camera.py",
        "v3/adapters/picamera2_camera.py",
    ],
    "PERSON": [
        "v3/adapters/live_person_detection.py",
        "v3/adapters/person_detection.py",
        "v3/adapters/litert_person_detector.py",
    ],
    "READER_NON_SOURCE": ["v3/adapters/live_inputs.py"],
}

LAYER_TARGETS = {
    "L1": ["v3/layers/l1_acquisition.py"],
    "L2": ["v3/layers/l2_admission.py"],
    "L3": ["v3/layers/l3_state_estimation.py"],
    "L4": ["v3/layers/l4_world_model.py"],
    "L5": ["v3/layers/l5_command_mission.py"],
    "L6": ["v3/layers/l6_navigation.py", "v3/adapters/l6_planner_process.py"],
    "L7": ["v3/layers/l7_motion_selection.py"],
    "L8": ["v3/layers/l8_motion_realization.py"],
    "L9": ["v3/layers/l9_operational_constraints.py"],
    "L10": ["v3/layers/l10_chassis_control.py"],
    "L11": ["v3/layers/l11_actuator_control.py"],
    "L12": [
        "v3/layers/l12_safety_final.py",
        "v3/adapters/motor_writer.py",
        "v3/adapters/gpio_motor.py",
    ],
}

TOP_TARGETS = {
    "L0_READ": ["v3/adapters/live_inputs.py"],
    "COMMAND_SNAPSHOT": ["v3/adapters/resident_command.py"],
    "PIPELINE_TOTAL": ["v3/engine.py"],
    "POST_CONTROL": ["v3/composition/resident_live_control.py"],
    "OBSERVER_TOTAL": ["v3_runtime.py", "v3_process_runtime.py", "v3/observation.py"],
    "PERIOD_SCHEDULER_RESIDUAL": ["v3_runtime.py", "v3/runtime_performance.py"],
}

ACTIVE_MODES = {"EXPLORE", "TELEOP", "FACE_PERSON", "FOLLOW_PERSON"}


SITE_CUSTOMIZE = r"""
from __future__ import annotations

import atexit
import json
import os
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

if any(arg == "v3_process_runtime.py" or arg.endswith("/v3_process_runtime.py") for arg in sys.argv):
    RAW_PATH = Path(os.environ["R2B4_LIVE_DIAG_RAW"])
    MECHANISM_EVERY = int(os.environ.get("R2B4_LIVE_DIAG_MECHANISM_EVERY", "50"))
    SCHEMA = "R2B4_V3_LIVE_CONTROL_ROOTCAUSE_RAW_V1"

    source_rows = []
    l0_rows = []
    command_rows = []
    phase_rows = []
    runtime_rows = []
    observer_rows = []
    mechanism_rows = []
    profiled_ticks = set()
    current_tick_id = None
    started_ns = time.monotonic_ns()
    patches = []

    def _schedstat():
        tid = threading.get_native_id()
        try:
            p = Path(f"/proc/self/task/{tid}/schedstat")
            parts = p.read_text(encoding="ascii").split()
            if len(parts) >= 3:
                return int(parts[0]), int(parts[1]), int(parts[2])
        except Exception:
            return None
        return None

    def _set(obj, name, value):
        patches.append((obj, name, getattr(obj, name)))
        setattr(obj, name, value)

    def _tick_id(context):
        value = getattr(context, "tick_id", None)
        return int(value) if isinstance(value, int) and not isinstance(value, bool) else None

    def _wrap_source(cls, role):
        original = cls.read
        def wrapped(self, context):
            tick_id = _tick_id(context)
            detailed = (
                tick_id is not None
                and MECHANISM_EVERY > 0
                and tick_id % MECHANISM_EVERY == 0
            )
            if not detailed:
                start = time.perf_counter_ns()
                try:
                    return original(self, context)
                finally:
                    source_rows.append((tick_id, role, max(0, time.perf_counter_ns() - start)))
            profiled_ticks.add(tick_id)
            sched0 = _schedstat()
            process0 = time.process_time_ns()
            cpu0 = time.thread_time_ns()
            wall0 = time.perf_counter_ns()
            try:
                return original(self, context)
            finally:
                wall1 = time.perf_counter_ns()
                cpu1 = time.thread_time_ns()
                process1 = time.process_time_ns()
                sched1 = _schedstat()
                wall = max(0, wall1 - wall0)
                thread_cpu = max(0, cpu1 - cpu0)
                process_cpu = max(0, process1 - process0)
                runqueue = 0
                if sched0 is not None and sched1 is not None:
                    runqueue = max(0, sched1[1] - sched0[1])
                source_rows.append((tick_id, role, wall))
                mechanism_rows.append(
                    (
                        tick_id,
                        role,
                        wall,
                        thread_cpu,
                        process_cpu,
                        runqueue,
                        max(0, wall - thread_cpu - runqueue),
                    )
                )
        _set(cls, "read", wrapped)

    try:
        from v3.adapters.gpio_encoder import NativeGpioEncoderSource
        from v3.adapters.live_imu import NativeImuSource
        from v3.adapters.live_lidar import NativeLidarSource
        from v3.adapters.live_camera import NativeCameraSource
        from v3.adapters.live_person_detection import NativePersonDetectionSource
        from v3.adapters.live_inputs import NativeLiveInputReader
        from v3.adapters.resident_command import AtomicResidentCommandGateway
        from v3.observation import ObservationHub
        from v3.runtime_performance import RuntimeTimingAccumulator

        _wrap_source(NativeGpioEncoderSource, "ENCODER")
        _wrap_source(NativeImuSource, "IMU")
        _wrap_source(NativeLidarSource, "LIDAR")
        _wrap_source(NativeCameraSource, "CAMERA")
        _wrap_source(NativePersonDetectionSource, "PERSON")

        original_reader = NativeLiveInputReader.read
        def reader_read(self, context):
            global current_tick_id
            tick_id = _tick_id(context)
            current_tick_id = tick_id
            start = time.perf_counter_ns()
            try:
                return original_reader(self, context)
            finally:
                l0_rows.append((tick_id, max(0, time.perf_counter_ns() - start)))
        _set(NativeLiveInputReader, "read", reader_read)

        original_snapshot = AtomicResidentCommandGateway.snapshot
        def snapshot(self, context):
            tick_id = _tick_id(context)
            start = time.perf_counter_ns()
            result = None
            try:
                result = original_snapshot(self, context)
                return result
            finally:
                mode = getattr(getattr(result, "mode", None), "value", "ERROR")
                command_rows.append(
                    (tick_id, str(mode), max(0, time.perf_counter_ns() - start))
                )
        _set(AtomicResidentCommandGateway, "snapshot", snapshot)

        original_phase = RuntimeTimingAccumulator.observe_control_phase
        def observe_control_phase(self, name, duration_ns):
            phase_rows.append((current_tick_id, str(name), int(duration_ns)))
            return original_phase(self, name, duration_ns)
        _set(RuntimeTimingAccumulator, "observe_control_phase", observe_control_phase)

        for method_name, label in (
            ("observe_control", "CONTROL_TOTAL"),
            ("observe_observer", "OBSERVER_TOTAL"),
            ("observe_work", "WORK_TOTAL"),
            ("observe_period", "PERIOD"),
        ):
            if not hasattr(RuntimeTimingAccumulator, method_name):
                continue
            original = getattr(RuntimeTimingAccumulator, method_name)
            def factory(orig, fixed_label):
                def wrapped(self, duration_ns, *args, **kwargs):
                    runtime_rows.append(
                        (current_tick_id, fixed_label, int(duration_ns))
                    )
                    return orig(self, duration_ns, *args, **kwargs)
                return wrapped
            _set(RuntimeTimingAccumulator, method_name, factory(original, label))

        original_publish = ObservationHub.publish
        def publish(self, value, *, topic):
            start = time.perf_counter_ns()
            try:
                return original_publish(self, value, topic=topic)
            finally:
                observer_rows.append(
                    (
                        current_tick_id,
                        str(topic),
                        max(0, time.perf_counter_ns() - start),
                    )
                )
        _set(ObservationHub, "publish", publish)

    except Exception as exc:
        RAW_PATH.parent.mkdir(parents=True, exist_ok=True)
        RAW_PATH.write_text(
            json.dumps(
                {
                    "schema": SCHEMA,
                    "instrumentation_error": f"{type(exc).__name__}:{exc}",
                    "argv": sys.argv,
                },
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )

    def _dump():
        try:
            payload = {
                "schema": SCHEMA,
                "pid": os.getpid(),
                "argv": sys.argv,
                "started_monotonic_ns": started_ns,
                "ended_monotonic_ns": time.monotonic_ns(),
                "mechanism_every": MECHANISM_EVERY,
                "profiled_ticks": sorted(profiled_ticks),
                "source_rows": source_rows,
                "l0_rows": l0_rows,
                "command_rows": command_rows,
                "phase_rows": phase_rows,
                "runtime_rows": runtime_rows,
                "observer_rows": observer_rows,
                "mechanism_rows": mechanism_rows,
            }
            RAW_PATH.parent.mkdir(parents=True, exist_ok=True)
            tmp = RAW_PATH.with_name(
                f".{RAW_PATH.name}.tmp.{os.getpid()}"
            )
            tmp.write_text(
                json.dumps(payload, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            os.replace(tmp, RAW_PATH)
        except Exception:
            pass

    atexit.register(_dump)
"""


def percentile(values: list[int], q: float) -> int:
    if not values:
        return 0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * q) - 1))
    return int(ordered[index])


def stats_ns(values: list[int]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean_ns": 0,
            "p50_ns": 0,
            "p95_ns": 0,
            "p99_ns": 0,
            "max_ns": 0,
            "mean_ms": 0.0,
            "p50_ms": 0.0,
            "p95_ms": 0.0,
            "p99_ms": 0.0,
            "max_ms": 0.0,
        }
    mean = int(round(sum(values) / len(values)))
    p50 = percentile(values, .50)
    p95 = percentile(values, .95)
    p99 = percentile(values, .99)
    mx = max(values)
    return {
        "count": len(values),
        "mean_ns": mean,
        "p50_ns": p50,
        "p95_ns": p95,
        "p99_ns": p99,
        "max_ns": mx,
        "mean_ms": mean / 1e6,
        "p50_ms": p50 / 1e6,
        "p95_ms": p95 / 1e6,
        "p99_ms": p99 / 1e6,
        "max_ms": mx / 1e6,
    }


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else None
    except Exception:
        return None


def mode_by_tick(raw: Mapping[str, Any]) -> dict[int, str]:
    result = {}
    for row in raw.get("command_rows", []):
        if not isinstance(row, list) or len(row) < 2:
            continue
        tick_id, mode = row[0], row[1]
        if isinstance(tick_id, int) and isinstance(mode, str):
            result[tick_id] = mode
    return result


def active_interval(modes: Mapping[int, str]) -> tuple[int, int] | None:
    active = sorted(tick for tick, mode in modes.items() if mode in ACTIVE_MODES)
    if not active:
        return None
    return active[0], active[-1]


def select_ticks(
    raw: Mapping[str, Any],
    *,
    active: bool,
) -> set[int]:
    modes = mode_by_tick(raw)
    profiled = {int(x) for x in raw.get("profiled_ticks", []) if isinstance(x, int)}
    if active:
        ticks = {tick for tick, mode in modes.items() if mode in ACTIVE_MODES}
    else:
        ticks = {tick for tick, mode in modes.items() if mode == "STOP"}
        interval = active_interval(modes)
        if interval is not None:
            ticks = {tick for tick in ticks if tick < interval[0]}
            if len(ticks) > 150:
                ticks = set(sorted(ticks)[-150:])
    return ticks - profiled


def aggregate_rows(
    rows: list[Any],
    ticks: set[int],
    *,
    key_index: int,
    value_index: int,
) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[int]] = defaultdict(list)
    for row in rows:
        if not isinstance(row, list):
            continue
        if len(row) <= max(key_index, value_index):
            continue
        tick = row[0]
        if tick not in ticks:
            continue
        key = str(row[key_index])
        value = row[value_index]
        if isinstance(value, int):
            grouped[key].append(value)
    return {key: stats_ns(values) for key, values in sorted(grouped.items())}


def analyze(raw: Mapping[str, Any], runtime_status: Mapping[str, Any] | None) -> dict[str, Any]:
    modes = mode_by_tick(raw)
    interval = active_interval(modes)
    active_ticks = select_ticks(raw, active=True)
    baseline_ticks = select_ticks(raw, active=False)

    def section(ticks: set[int]) -> dict[str, Any]:
        source_group: dict[str, list[int]] = defaultdict(list)
        source_by_tick: dict[int, dict[str, int]] = defaultdict(dict)
        for row in raw.get("source_rows", []):
            if not isinstance(row, list) or len(row) != 3:
                continue
            tick, role, value = row
            if tick in ticks and isinstance(role, str) and isinstance(value, int):
                source_group[role].append(value)
                source_by_tick[tick][role] = source_by_tick[tick].get(role, 0) + value

        l0_map = {}
        for row in raw.get("l0_rows", []):
            if isinstance(row, list) and len(row) == 2 and row[0] in ticks and isinstance(row[1], int):
                l0_map[int(row[0])] = int(row[1])

        residuals = []
        for tick, l0 in l0_map.items():
            residuals.append(max(0, l0 - sum(source_by_tick.get(tick, {}).values())))

        phases = aggregate_rows(
            list(raw.get("phase_rows", [])),
            ticks,
            key_index=1,
            value_index=2,
        )
        runtime = aggregate_rows(
            list(raw.get("runtime_rows", [])),
            ticks,
            key_index=1,
            value_index=2,
        )
        observers = aggregate_rows(
            list(raw.get("observer_rows", [])),
            ticks,
            key_index=1,
            value_index=2,
        )
        command = stats_ns(
            [
                int(row[2])
                for row in raw.get("command_rows", [])
                if isinstance(row, list)
                and len(row) >= 3
                and row[0] in ticks
                and isinstance(row[2], int)
            ]
        )

        return {
            "tick_count": len(ticks),
            "sources": {role: stats_ns(values) for role, values in sorted(source_group.items())},
            "l0_total": stats_ns(list(l0_map.values())),
            "l0_reader_residual": stats_ns(residuals),
            "command_snapshot_direct": command,
            "control_phases": phases,
            "runtime": runtime,
            "observer_publish": observers,
        }

    baseline = section(baseline_ticks)
    active_sec = section(active_ticks)

    mechanism: dict[str, dict[str, list[int]]] = defaultdict(lambda: defaultdict(list))
    for row in raw.get("mechanism_rows", []):
        if not isinstance(row, list) or len(row) != 7:
            continue
        tick, role, wall, thread_cpu, process_cpu, runqueue, blocked = row
        if tick not in set(mode_by_tick(raw)):
            continue
        mode = modes.get(tick)
        if mode not in ACTIVE_MODES:
            continue
        mechanism[str(role)]["wall"].append(int(wall))
        mechanism[str(role)]["thread_cpu"].append(int(thread_cpu))
        mechanism[str(role)]["process_cpu"].append(int(process_cpu))
        mechanism[str(role)]["runqueue_wait"].append(int(runqueue))
        mechanism[str(role)]["blocked_or_gil"].append(int(blocked))
    mechanism_summary = {
        role: {name: stats_ns(values) for name, values in fields.items()}
        for role, fields in sorted(mechanism.items())
    }

    source_ranking = sorted(
        (
            (role, float(payload["mean_ms"]), float(payload["p99_ms"]))
            for role, payload in active_sec["sources"].items()
        ),
        key=lambda x: x[1],
        reverse=True,
    )

    layer_ranking = sorted(
        (
            (name, float(payload["mean_ms"]), float(payload["p99_ms"]))
            for name, payload in active_sec["control_phases"].items()
            if name.startswith("L") and name[1:].isdigit()
        ),
        key=lambda x: x[1],
        reverse=True,
    )

    phases = active_sec["control_phases"]
    top_components = {}
    for name in ("L0_READ", "COMMAND_SNAPSHOT", "PIPELINE_TOTAL", "POST_CONTROL"):
        if name in phases:
            top_components[name] = float(phases[name]["mean_ms"])
    observer_mean = float(active_sec["runtime"].get("OBSERVER_TOTAL", {}).get("mean_ms", 0.0))
    if observer_mean:
        top_components["OBSERVER_TOTAL"] = observer_mean

    period_mean = float(active_sec["runtime"].get("PERIOD", {}).get("mean_ms", 0.0))
    work_mean = float(active_sec["runtime"].get("WORK_TOTAL", {}).get("mean_ms", 0.0))
    sched_residual = max(0.0, period_mean - work_mean)
    if sched_residual:
        top_components["PERIOD_SCHEDULER_RESIDUAL"] = sched_residual

    top_ranking = sorted(top_components.items(), key=lambda x: x[1], reverse=True)
    top_cause = top_ranking[0][0] if top_ranking else None

    concrete = None
    fix_files: list[str] = []
    mechanism_note = None
    if top_cause == "L0_READ" and source_ranking:
        concrete = source_ranking[0][0]
        fix_files = SOURCE_TARGETS.get(concrete, [])
        mech = mechanism_summary.get(concrete)
        if mech:
            wall = float(mech.get("wall", {}).get("mean_ms", 0.0))
            cpu = float(mech.get("thread_cpu", {}).get("mean_ms", 0.0))
            rq = float(mech.get("runqueue_wait", {}).get("mean_ms", 0.0))
            blocked = float(mech.get("blocked_or_gil", {}).get("mean_ms", 0.0))
            mechanism_note = {
                "wall_mean_ms": wall,
                "thread_cpu_mean_ms": cpu,
                "runqueue_wait_mean_ms": rq,
                "blocked_or_gil_mean_ms": blocked,
                "interpretation": (
                    "blocked_or_gil is wall-threadCPU-runqueue; it may contain blocking I/O, "
                    "GIL/futex wait or sleeping and is not automatically pure I/O."
                ),
            }
    elif top_cause == "PIPELINE_TOTAL" and layer_ranking:
        concrete = layer_ranking[0][0]
        fix_files = LAYER_TARGETS.get(concrete, [])
    elif top_cause is not None:
        concrete = top_cause
        fix_files = TOP_TARGETS.get(top_cause, [])

    hz = 1000.0 / period_mean if period_mean > 0 else 0.0
    residual_mean = float(active_sec["l0_reader_residual"]["mean_ms"])
    l0_mean = float(active_sec["l0_total"]["mean_ms"])
    residual_pct = residual_mean / l0_mean * 100.0 if l0_mean > 0 else 0.0

    confidence = "INCOMPLETE"
    if len(active_ticks) >= 100:
        confidence = "HIGH_CONFIDENCE_LIVE"
    if len(active_ticks) >= 300 and residual_pct <= 3.0:
        confidence = "VERY_HIGH_CONFIDENCE_LIVE"

    baseline_sources = baseline["sources"]
    source_delta = {}
    for role in sorted(set(baseline_sources) | set(active_sec["sources"])):
        b = float(baseline_sources.get(role, {}).get("mean_ms", 0.0))
        a = float(active_sec["sources"].get(role, {}).get("mean_ms", 0.0))
        source_delta[role] = {
            "baseline_ms": b,
            "active_ms": a,
            "delta_ms": a - b,
        }

    return {
        "confidence": confidence,
        "active_interval": (
            None if interval is None else {"first_tick": interval[0], "last_tick": interval[1]}
        ),
        "active_tick_count": len(active_ticks),
        "baseline_tick_count": len(baseline_ticks),
        "active_effective_hz": hz,
        "active_period_mean_ms": period_mean,
        "active_work_mean_ms": work_mean,
        "top_level_ranking": [
            {"component": name, "mean_ms": value}
            for name, value in top_ranking
        ],
        "source_ranking": [
            {"component": role, "mean_ms": mean, "p99_ms": p99}
            for role, mean, p99 in source_ranking
        ],
        "layer_ranking": [
            {"layer": layer, "mean_ms": mean, "p99_ms": p99}
            for layer, mean, p99 in layer_ranking
        ],
        "root_cause": {
            "top_region": top_cause,
            "concrete_component": concrete,
            "target_files": fix_files,
            "mechanism": mechanism_note,
        },
        "l0_accounting": {
            "l0_mean_ms": l0_mean,
            "reader_residual_mean_ms": residual_mean,
            "reader_residual_percent": residual_pct,
        },
        "source_active_vs_stop_delta": source_delta,
        "baseline_stop": baseline,
        "active": active_sec,
        "mechanism_samples": mechanism_summary,
        "runtime_status_after_shutdown": dict(runtime_status or {}),
    }


def wait_for_ready(status_path: Path, pid_path: Path, timeout_s: float) -> int:
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        try:
            pid = int(pid_path.read_text(encoding="ascii").strip())
        except Exception:
            pid = 0
        status = read_json(status_path)
        if status:
            last = status
            if (
                status.get("state") == "RUNNING"
                and status.get("ready_for_active") is True
                and pid > 0
            ):
                return pid
        time.sleep(0.1)
    raise RuntimeError(f"runtime not ready within {timeout_s}s; last_status={last}")


def monitor_motion(
    status_path: Path,
    *,
    post_idle_s: float,
    poll_s: float,
    max_wait_s: float,
) -> str:
    started = time.monotonic()
    active_seen = False
    idle_since = None
    last_print = None

    while True:
        if max_wait_s > 0 and time.monotonic() - started > max_wait_s:
            return "MAX_WAIT"
        status = read_json(status_path)
        if status and status.get("state") == "RUNNING":
            mission = status.get("mission")
            mode = mission.get("mode") if isinstance(mission, dict) else None
            enabled = status.get("enabled") is True
            is_active = mode in ACTIVE_MODES or enabled
            if is_active:
                if not active_seen:
                    print(f"LIVE MOTION DETECTED: mode={mode or 'UNKNOWN'}", flush=True)
                active_seen = True
                idle_since = None
            elif active_seen:
                if idle_since is None:
                    idle_since = time.monotonic()
                    print("motion returned to IDLE/STOP; collecting tail...", flush=True)
                elif time.monotonic() - idle_since >= post_idle_s:
                    return "MOTION_COMPLETE"
        time.sleep(poll_s)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path("/home/alba/project_r2b4"))
    parser.add_argument("--capture-mode", choices=("alap", "full", "nincs"), default="full")
    parser.add_argument("--mechanism-every", type=int, default=50)
    parser.add_argument("--post-idle-s", type=float, default=3.0)
    parser.add_argument("--ready-timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--max-wait-s",
        type=float,
        default=180.0,
        help="maximum total wait for one live motion session; 0 = unlimited",
    )
    parser.add_argument(
        "--manual-stop",
        action="store_true",
        help="do not auto-shutdown after the first motion returns to idle",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if args.mechanism_every < 0:
        parser.error("--mechanism-every must be >= 0")
    if args.post_idle_s < 0:
        parser.error("--post-idle-s must be >= 0")

    repo = args.repo.expanduser().resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"ERROR: not an R2B4 repo: {repo}")

    sys.path.insert(0, str(repo))
    from v3.operator_controller import OperatorController, OperatorEvent

    runtime_dir = repo / "runtime"
    diag_dir = runtime_dir / "diag"
    diag_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = diag_dir / f"live_control_rootcause_raw_{stamp}.json"
    output = args.output
    if output is None:
        output = diag_dir / f"live_control_rootcause_diag_{stamp}.json"
    elif not output.is_absolute():
        output = (repo / output).resolve()

    inject_dir = Path(tempfile.mkdtemp(prefix="r2b4-live-diag-inject-", dir="/tmp"))
    (inject_dir / "sitecustomize.py").write_text(SITE_CUSTOMIZE, encoding="utf-8")

    def event_sink(event: OperatorEvent) -> None:
        print(event.message, flush=True)

    controller = OperatorController(repo, event_sink=event_sink)
    snap = controller.snapshot()
    if snap.runtime_running:
        shutil.rmtree(inject_dir, ignore_errors=True)
        raise SystemExit(
            f"ERROR: runtime already running (PID {snap.runtime_pid}). "
            "Shut it down first; this diagnostic must own the production runtime from startup."
        )

    old_pythonpath = os.environ.get("PYTHONPATH")
    old_raw = os.environ.get("R2B4_LIVE_DIAG_RAW")
    old_every = os.environ.get("R2B4_LIVE_DIAG_MECHANISM_EVERY")
    injected_path = str(inject_dir)
    if old_pythonpath:
        injected_path += os.pathsep + old_pythonpath
    os.environ["PYTHONPATH"] = injected_path
    os.environ["R2B4_LIVE_DIAG_RAW"] = str(raw_path)
    os.environ["R2B4_LIVE_DIAG_MECHANISM_EVERY"] = str(args.mechanism_every)

    pid = None
    stop_reason = None
    try:
        print("R2B4 LIVE CONTROL ROOT-CAUSE DIAG")
        print(f"repo:             {repo}")
        print(f"capture mode:     {args.capture_mode}")
        print(f"mechanism sample: every {args.mechanism_every} ticks")
        print("motors:           REAL PRODUCTION MOTOR PATH")
        print("runtime:          canonical v3_process_runtime.py")
        print("instrumentation:  low-perturbation in-memory timing")
        print()

        pid = controller.runtime_start(args.capture_mode)

        # Restore parent environment after child/supervisor inherited it.
        if old_pythonpath is None:
            os.environ.pop("PYTHONPATH", None)
        else:
            os.environ["PYTHONPATH"] = old_pythonpath
        if old_raw is None:
            os.environ.pop("R2B4_LIVE_DIAG_RAW", None)
        else:
            os.environ["R2B4_LIVE_DIAG_RAW"] = old_raw
        if old_every is None:
            os.environ.pop("R2B4_LIVE_DIAG_MECHANISM_EVERY", None)
        else:
            os.environ["R2B4_LIVE_DIAG_MECHANISM_EVERY"] = old_every

        wait_for_ready(
            runtime_dir / "v3_status.json",
            runtime_dir / ".r2b4_runtime_pid",
            args.ready_timeout_s,
        )

        print()
        print("READY FOR LIVE RUN")
        print("In another terminal run, for example:")
        print(f"  r rc 30 c {args.capture_mode}")
        print()
        print("The diagnostic is now measuring the REAL live runtime.", flush=True)

        if args.manual_stop:
            while True:
                time.sleep(1.0)
        else:
            stop_reason = monitor_motion(
                runtime_dir / "v3_status.json",
                post_idle_s=args.post_idle_s,
                poll_s=0.2,
                max_wait_s=args.max_wait_s,
            )
            print(f"diagnostic stop condition: {stop_reason}", flush=True)

    except KeyboardInterrupt:
        stop_reason = "KEYBOARD_INTERRUPT"
        print("\nCtrl+C: requesting normal STOP + runtime shutdown...", flush=True)
    finally:
        # Ensure parent env is restored even if runtime startup failed.
        if old_pythonpath is None:
            os.environ.pop("PYTHONPATH", None)
        else:
            os.environ["PYTHONPATH"] = old_pythonpath
        if old_raw is None:
            os.environ.pop("R2B4_LIVE_DIAG_RAW", None)
        else:
            os.environ["R2B4_LIVE_DIAG_RAW"] = old_raw
        if old_every is None:
            os.environ.pop("R2B4_LIVE_DIAG_MECHANISM_EVERY", None)
        else:
            os.environ["R2B4_LIVE_DIAG_MECHANISM_EVERY"] = old_every

        try:
            if controller.snapshot().runtime_running:
                controller.runtime_stop()
        except Exception as exc:
            print(f"WARNING: runtime shutdown: {type(exc).__name__}: {exc}", file=sys.stderr)

    # Child atexit writes the raw file after normal runtime termination.
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and not raw_path.is_file():
        time.sleep(0.1)

    runtime_status = read_json(runtime_dir / "v3_status.json")
    if not raw_path.is_file():
        shutil.rmtree(inject_dir, ignore_errors=True)
        raise SystemExit(
            "ERROR: instrumentation raw evidence was not produced. "
            "Check the runtime log for a sitecustomize/import failure."
        )

    raw = read_json(raw_path)
    if raw is None:
        shutil.rmtree(inject_dir, ignore_errors=True)
        raise SystemExit("ERROR: invalid raw diagnostic JSON")
    if raw.get("instrumentation_error"):
        shutil.rmtree(inject_dir, ignore_errors=True)
        raise SystemExit(f"ERROR: {raw['instrumentation_error']}")

    analysis = analyze(raw, runtime_status)
    capture_path = controller.current_capture_path()
    payload = {
        "schema": SCHEMA,
        "generated_local": datetime.now().astimezone().isoformat(),
        "repo": str(repo),
        "runtime_pid": pid,
        "stop_reason": stop_reason,
        "capture_mode": args.capture_mode,
        "capture_path": str(capture_path) if capture_path is not None else None,
        "raw_evidence": str(raw_path),
        "measurement_method": {
            "real_motor_path": True,
            "production_runtime": True,
            "repository_modified": False,
            "primary_clock": "time.perf_counter_ns",
            "detailed_mechanism_every_ticks": args.mechanism_every,
            "profiled_ticks_excluded_from_primary_stats": True,
            "runtime_existing_layer_timing_reused": True,
        },
        "analysis": analysis,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    root = analysis["root_cause"]
    print()
    print("LIVE ROOT-CAUSE RESULT")
    print(f"confidence:        {analysis['confidence']}")
    print(f"active ticks:      {analysis['active_tick_count']}")
    print(f"effective Hz:      {analysis['active_effective_hz']:.2f}")
    print(f"period mean:       {analysis['active_period_mean_ms']:.3f} ms")
    print(f"work mean:         {analysis['active_work_mean_ms']:.3f} ms")
    print(f"L0 mean:           {analysis['l0_accounting']['l0_mean_ms']:.3f} ms")
    print(f"L0 residual:       {analysis['l0_accounting']['reader_residual_mean_ms']:.3f} ms "
          f"({analysis['l0_accounting']['reader_residual_percent']:.2f}%)")
    print(f"top region:        {root['top_region']}")
    print(f"concrete cause:    {root['concrete_component']}")
    if root["target_files"]:
        print("target files:")
        for item in root["target_files"]:
            print(f"  - {item}")
    mech = root.get("mechanism")
    if isinstance(mech, dict):
        print(
            "mechanism:         "
            f"wall={mech['wall_mean_ms']:.3f} ms "
            f"threadCPU={mech['thread_cpu_mean_ms']:.3f} "
            f"runqueue={mech['runqueue_wait_mean_ms']:.3f} "
            f"blocked/GIL={mech['blocked_or_gil_mean_ms']:.3f}"
        )
    print()
    print("Top-level:")
    for item in analysis["top_level_ranking"][:6]:
        print(f"  {item['component']:<28} {item['mean_ms']:8.3f} ms")
    print("Sources:")
    for item in analysis["source_ranking"][:5]:
        print(
            f"  {item['component']:<10} mean={item['mean_ms']:8.3f} "
            f"p99={item['p99_ms']:8.3f} ms"
        )
    print("Layers:")
    for item in analysis["layer_ranking"][:5]:
        print(
            f"  {item['layer']:<4} mean={item['mean_ms']:8.3f} "
            f"p99={item['p99_ms']:8.3f} ms"
        )
    print()
    print(f"JSON: {output}")
    print(f"RAW:  {raw_path}")

    shutil.rmtree(inject_dir, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
