#!/usr/bin/env python3
"""
R2B4 V3 low-perturbation full-runtime control-loop root-cause diagnostic.

V2 fixes two problems found by the first full-runtime diagnostic:
1) EXPLORE + NULL motor intentionally produces no encoder motion, so production
   L11 eventually faults on missing control-grade wheel feedback. In this tool
   only, the global L11 feedback-uncertainty watchdog is put into SHADOW mode.
   L11 still executes its normal uncertain-feedback/feed-forward path, L12 still
   executes, and the physical motor GPIO capability is never opened.
2) /proc/schedstat around every source read perturbed L0. Primary accounting now
   uses only perf_counter_ns() around the real source reads. Detailed CPU/runqueue
   mechanism sampling is sparse and those profiled ticks are excluded from the
   primary latency statistics.

The tool performs, in one production resident-runtime ownership session:
  WARMUP STOP
  BASELINE_STOP measurement
  EXPLORE measurement

Thus hardware, camera, person detector, LiDAR matcher, planner process, capture,
affinity and Python process environment stay alive while command workload changes.

Primary question:
  Why is the real control loop slower than the 20 ms / 50 Hz target?

Evidence:
  * exact per-tick L0 source wall time: encoder / IMU / LiDAR / camera / person
  * actual NativeLiveInputReader total and residual
  * production runtime phases (L0_READ, COMMAND_SNAPSHOT, PIPELINE_TOTAL,
    POST_CONTROL) via the existing RuntimeTimingAccumulator
  * observer / work / period timing
  * sparse mechanism samples: thread CPU, Linux runqueue wait, residual blocked
    or GIL/sleep time
  * BASELINE_STOP vs EXPLORE deltas
  * source-first fix-target file mapping

No repository file is modified by the diagnostic.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import threading
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping

SCHEMA = "R2B4_V3_CONTROL_ROOTCAUSE_DIAG_V2"


def percentile(values: list[int], q: float) -> int:
    if not values:
        return 0
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * q) - 1))
    return int(ordered[idx])


def stats_ns(values: list[int]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean_ns": 0, "p50_ns": 0, "p95_ns": 0, "p99_ns": 0, "max_ns": 0,
            "mean_ms": 0.0, "p50_ms": 0.0, "p95_ms": 0.0, "p99_ms": 0.0, "max_ms": 0.0,
        }
    mean = int(round(sum(values) / len(values)))
    p50 = percentile(values, .50)
    p95 = percentile(values, .95)
    p99 = percentile(values, .99)
    mx = max(values)
    return {
        "count": len(values),
        "mean_ns": mean, "p50_ns": p50, "p95_ns": p95, "p99_ns": p99, "max_ns": mx,
        "mean_ms": mean / 1e6, "p50_ms": p50 / 1e6, "p95_ms": p95 / 1e6,
        "p99_ms": p99 / 1e6, "max_ms": mx / 1e6,
    }


def active_pid(path: Path) -> int | None:
    try:
        pid = int(path.read_text(encoding="ascii").strip())
    except Exception:
        return None
    if pid <= 0:
        return None
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return None
    except PermissionError:
        return pid
    return pid


def read_schedstat() -> tuple[int, int, int] | None:
    tid = threading.get_native_id()
    try:
        parts = Path(f"/proc/self/task/{tid}/schedstat").read_text(encoding="ascii").split()
        if len(parts) >= 3:
            return int(parts[0]), int(parts[1]), int(parts[2])
    except Exception:
        pass
    return None


def source_role(source: Any) -> str:
    token = (
        type(source).__name__.upper()
        + " "
        + str(getattr(source, "device_id", "")).upper()
    )
    if "ENCODER" in token:
        return "ENCODER"
    if "IMU" in token or "BNO055" in token:
        return "IMU"
    if "LIDAR" in token or "RPLIDAR" in token:
        return "LIDAR"
    if "PERSON" in token:
        return "PERSON"
    if "CAMERA" in token:
        return "CAMERA"
    return f"OTHER:{type(source).__name__}:{getattr(source, 'device_id', 'unknown')}"


@dataclass(slots=True)
class TickRow:
    phase: str
    tick_id: int
    reader_wall_ns: int
    source_wall_ns: dict[str, int]
    reader_non_source_ns: int
    detailed_profile_tick: bool


@dataclass(slots=True)
class MechanismRow:
    phase: str
    tick_id: int
    role: str
    wall_ns: int
    thread_cpu_ns: int
    process_cpu_ns: int
    runqueue_wait_ns: int
    blocked_or_gil_ns: int


class Collector:
    """Low-overhead main-thread accounting plus sparse mechanism profiling."""

    def __init__(self, mechanism_every: int) -> None:
        self.mechanism_every = int(mechanism_every)
        self.phase: str | None = None
        self.phase_lock = threading.Lock()
        self.runtime_tick_event = threading.Event()
        self.tick_rows: list[TickRow] = []
        self.mechanism_rows: list[MechanismRow] = []
        self.scalars: dict[str, dict[str, list[int]]] = defaultdict(
            lambda: defaultdict(list)
        )
        self.light_spans: dict[str, dict[str, list[int]]] = defaultdict(
            lambda: defaultdict(list)
        )
        self.phase_started_ns: dict[str, int] = {}
        self.phase_ended_ns: dict[str, int] = {}

    def set_phase(self, phase: str | None) -> None:
        with self.phase_lock:
            old = self.phase
            now = time.monotonic_ns()
            if old is not None:
                self.phase_ended_ns[old] = now
            self.phase = phase
            if phase is not None and phase not in self.phase_started_ns:
                self.phase_started_ns[phase] = now

    def current_phase(self) -> str | None:
        return self.phase

    def add_scalar(self, label: str, duration_ns: int) -> None:
        phase = self.phase
        if phase is None:
            return
        # list.append under the GIL is sufficient here; this diagnostic is
        # intentionally avoiding a lock in the control hot path.
        self.scalars[phase][label].append(max(0, int(duration_ns)))

    def light_measure(self, label: str, fn: Callable[[], Any]) -> Any:
        phase = self.phase
        if phase is None:
            return fn()
        start = time.perf_counter_ns()
        try:
            return fn()
        finally:
            self.light_spans[phase][label].append(
                max(0, time.perf_counter_ns() - start)
            )

    def detailed_source(
        self,
        *,
        phase: str,
        tick_id: int,
        role: str,
        fn: Callable[[], Any],
    ) -> tuple[Any, int]:
        sched0 = read_schedstat()
        p0 = time.process_time_ns()
        c0 = time.thread_time_ns()
        w0 = time.perf_counter_ns()
        try:
            result = fn()
        finally:
            w1 = time.perf_counter_ns()
            c1 = time.thread_time_ns()
            p1 = time.process_time_ns()
            sched1 = read_schedstat()
        wall = max(0, w1 - w0)
        thread_cpu = max(0, c1 - c0)
        process_cpu = max(0, p1 - p0)
        rq = 0
        if sched0 is not None and sched1 is not None:
            rq = max(0, sched1[1] - sched0[1])
        self.mechanism_rows.append(
            MechanismRow(
                phase=phase,
                tick_id=tick_id,
                role=role,
                wall_ns=wall,
                thread_cpu_ns=thread_cpu,
                process_cpu_ns=process_cpu,
                runqueue_wait_ns=rq,
                blocked_or_gil_ns=max(0, wall - thread_cpu - rq),
            )
        )
        return result, wall


class NullMotorSink:
    """Drop-in GpioMotorFrameSink replacement; never opens motor GPIO."""

    instances: list["NullMotorSink"] = []

    def __init__(self, _backend: Any, _config: Any) -> None:
        self._closed = False
        self._failed = False
        self.write_count = 0
        self.last_frame: Any = None
        type(self).instances.append(self)

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def failed(self) -> bool:
        return self._failed

    def write(self, frame: Any) -> None:
        if self._closed:
            raise OSError("null motor sink is closed")
        self.write_count += 1
        self.last_frame = frame

    def close(self) -> None:
        self._closed = True


class PatchSet:
    def __init__(self) -> None:
        self.items: list[tuple[Any, str, Any]] = []

    def set(self, obj: Any, name: str, value: Any) -> None:
        self.items.append((obj, name, getattr(obj, name)))
        setattr(obj, name, value)

    def restore(self) -> None:
        for obj, name, original in reversed(self.items):
            setattr(obj, name, original)
        self.items.clear()


def build_patches(collector: Collector) -> PatchSet:
    from v3.contracts import RawDeviceBatch, TickContext
    from v3.adapters.live_inputs import NativeLiveInputReader, LiveDeviceSnapshot
    from v3.adapters.resident_command import AtomicResidentCommandGateway
    from v3.observation import ObservationHub
    from v3.layers.l11_actuator_control import WheelActuatorController
    import v3.composition.motor_output as motor_output_module
    import v3.runtime_performance as runtime_performance
    import v3_process_runtime as process_runtime

    patches = PatchSet()

    # Safety: retain final actuation planning/writer path but remove physical GPIO.
    patches.set(motor_output_module, "GpioMotorFrameSink", NullMotorSink)

    # Diagnostic-only L11 shadow:
    # With a NULL motor, an EXPLORE command cannot create encoder motion. Production
    # L11 correctly faults after bounded feedback uncertainty. Suppress ONLY that
    # expected no-motion watchdog so the upstream EXPLORE workload can be profiled.
    original_bounded_uncertainty = WheelActuatorController._bounded_uncertainty_start

    def shadow_bounded_uncertainty(self, side, started_ns, now_ns):
        if side == "feedback":
            if started_ns is None:
                return now_ns
            if now_ns < started_ns:
                raise ValueError("L11 feedback uncertainty time must be monotonic")
            return started_ns
        return original_bounded_uncertainty(self, side, started_ns, now_ns)

    patches.set(
        WheelActuatorController,
        "_bounded_uncertainty_start",
        shadow_bounded_uncertainty,
    )

    original_reader_read = NativeLiveInputReader.read

    def diagnostic_reader_read(self, context):
        collector.runtime_tick_event.set()
        phase = collector.current_phase()
        if phase is None:
            return original_reader_read(self, context)
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")

        detailed = (
            collector.mechanism_every > 0
            and context.tick_id % collector.mechanism_every == 0
        )
        reader_start = time.perf_counter_ns()
        samples = []
        health = []
        source_times: dict[str, int] = {}

        for expected_device_id, source in zip(self._source_ids, self._sources):
            role = source_role(source)
            if detailed:
                snapshot, source_ns = collector.detailed_source(
                    phase=phase,
                    tick_id=context.tick_id,
                    role=role,
                    fn=lambda source=source: source.read(context),
                )
            else:
                source_start = time.perf_counter_ns()
                snapshot = source.read(context)
                source_ns = max(0, time.perf_counter_ns() - source_start)

            if not isinstance(snapshot, LiveDeviceSnapshot):
                raise TypeError("live device source must return LiveDeviceSnapshot")
            if snapshot.context != context:
                raise ValueError("live device snapshot context must match the tick")
            if snapshot.health.device_id != expected_device_id:
                raise ValueError("live device snapshot ID must match its configured source")
            source_times[role] = source_times.get(role, 0) + source_ns
            samples.extend(snapshot.samples)
            health.append(snapshot.health)

        batch = RawDeviceBatch(context, tuple(samples), tuple(health))
        reader_wall = max(0, time.perf_counter_ns() - reader_start)
        source_sum = sum(source_times.values())
        collector.tick_rows.append(
            TickRow(
                phase=phase,
                tick_id=context.tick_id,
                reader_wall_ns=reader_wall,
                source_wall_ns=source_times,
                reader_non_source_ns=max(0, reader_wall - source_sum),
                detailed_profile_tick=detailed,
            )
        )
        return batch

    patches.set(NativeLiveInputReader, "read", diagnostic_reader_read)

    original_snapshot = AtomicResidentCommandGateway.snapshot

    def timed_snapshot(self, context):
        return collector.light_measure(
            "COMMAND_SNAPSHOT_DIRECT",
            lambda: original_snapshot(self, context),
        )

    patches.set(AtomicResidentCommandGateway, "snapshot", timed_snapshot)

    original_hub_publish = ObservationHub.publish

    def timed_hub_publish(self, value, *, topic):
        return collector.light_measure(
            f"OBSERVER_HUB_PUBLISH:{topic}",
            lambda: original_hub_publish(self, value, topic=topic),
        )

    patches.set(ObservationHub, "publish", timed_hub_publish)

    original_status_publish = process_runtime.AsyncResidentStatusPublisher.publish_tick

    def timed_status_publish(self, result, ready_for_active=False):
        return collector.light_measure(
            "OBSERVER_STATUS_PUBLISH",
            lambda: original_status_publish(self, result, ready_for_active),
        )

    patches.set(
        process_runtime.AsyncResidentStatusPublisher,
        "publish_tick",
        timed_status_publish,
    )

    # Tee the timing accumulator with one list.append. Existing production
    # measurement remains authoritative; this adds negligible hot-path work.
    Acc = runtime_performance.RuntimeTimingAccumulator
    timing_methods = (
        ("observe_control", "RUNTIME_CONTROL_TOTAL"),
        ("observe_observer", "RUNTIME_OBSERVER_TOTAL"),
        ("observe_work", "RUNTIME_WORK_TOTAL"),
        ("observe_period", "RUNTIME_PERIOD"),
    )
    for method_name, label in timing_methods:
        if not hasattr(Acc, method_name):
            continue
        original = getattr(Acc, method_name)

        def make_wrapper(orig, fixed_label):
            def wrapper(self, duration_ns, *args, **kwargs):
                collector.add_scalar(fixed_label, duration_ns)
                return orig(self, duration_ns, *args, **kwargs)
            return wrapper

        patches.set(Acc, method_name, make_wrapper(original, label))

    if hasattr(Acc, "observe_control_phase"):
        original_phase = Acc.observe_control_phase

        def phase_wrapper(self, name, duration_ns):
            collector.add_scalar(f"RUNTIME_PHASE:{name}", duration_ns)
            return original_phase(self, name, duration_ns)

        patches.set(Acc, "observe_control_phase", phase_wrapper)

    return patches


def phase_rows(collector: Collector, phase: str, *, primary_only: bool) -> list[TickRow]:
    rows = [row for row in collector.tick_rows if row.phase == phase]
    if primary_only:
        rows = [row for row in rows if not row.detailed_profile_tick]
    return rows


def role_stats(rows: list[TickRow]) -> dict[str, dict[str, Any]]:
    roles = sorted({role for row in rows for role in row.source_wall_ns})
    return {
        role: stats_ns([row.source_wall_ns.get(role, 0) for row in rows])
        for role in roles
    }


def mechanism_summary(collector: Collector, phase: str) -> dict[str, Any]:
    rows = [row for row in collector.mechanism_rows if row.phase == phase]
    roles = sorted({row.role for row in rows})
    result = {}
    for role in roles:
        selected = [row for row in rows if row.role == role]
        result[role] = {
            "count": len(selected),
            "wall": stats_ns([x.wall_ns for x in selected]),
            "thread_cpu": stats_ns([x.thread_cpu_ns for x in selected]),
            "process_cpu": stats_ns([x.process_cpu_ns for x in selected]),
            "runqueue_wait": stats_ns([x.runqueue_wait_ns for x in selected]),
            "blocked_or_gil": stats_ns([x.blocked_or_gil_ns for x in selected]),
        }
    return result


FIX_TARGETS = {
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
    "COMMAND_SNAPSHOT": ["v3/adapters/resident_command.py"],
    "CAPTURE_OBSERVER": [
        "v3/observation.py",
        "v3_process_runtime.py",
        "v3/mcap_capture.py",
    ],
}


def analyze_phase(collector: Collector, phase: str, period_ns: int) -> dict[str, Any]:
    all_rows = phase_rows(collector, phase, primary_only=False)
    rows = phase_rows(collector, phase, primary_only=True)
    sources = role_stats(rows)
    reader = stats_ns([row.reader_wall_ns for row in rows])
    residual = stats_ns([row.reader_non_source_ns for row in rows])
    deadline_rows = [row for row in rows if row.reader_wall_ns > period_ns]

    slow = []
    for row in deadline_rows:
        largest = max(
            row.source_wall_ns.items(),
            key=lambda item: item[1],
            default=("UNKNOWN", 0),
        )
        slow.append({
            "tick_id": row.tick_id,
            "l0_ms": row.reader_wall_ns / 1e6,
            "largest_source": largest[0],
            "largest_source_ms": largest[1] / 1e6,
            "sources_ms": {
                role: ns / 1e6 for role, ns in row.source_wall_ns.items()
            },
            "reader_non_source_ms": row.reader_non_source_ns / 1e6,
        })

    scalar = {
        label: stats_ns(values)
        for label, values in sorted(collector.scalars.get(phase, {}).items())
    }
    light = {
        label: stats_ns(values)
        for label, values in sorted(collector.light_spans.get(phase, {}).items())
    }

    source_mean = {
        role: payload["mean_ns"]
        for role, payload in sources.items()
    }
    dominant = max(source_mean, key=source_mean.get) if source_mean else None

    return {
        "primary_tick_count": len(rows),
        "detailed_profile_tick_count": len(all_rows) - len(rows),
        "reader_total": reader,
        "reader_non_source": residual,
        "sources": sources,
        "dominant_mean_source": dominant,
        "l0_deadline_miss_count": len(deadline_rows),
        "l0_deadline_miss_ticks": slow,
        "runtime_timing": scalar,
        "direct_light_spans": light,
        "mechanism": mechanism_summary(collector, phase),
    }


def delta_summary(stop: Mapping[str, Any], explore: Mapping[str, Any]) -> dict[str, Any]:
    stop_sources = stop.get("sources", {})
    exp_sources = explore.get("sources", {})
    roles = sorted(set(stop_sources) | set(exp_sources))
    source_delta = {}
    for role in roles:
        b = int(stop_sources.get(role, {}).get("mean_ns", 0))
        e = int(exp_sources.get(role, {}).get("mean_ns", 0))
        source_delta[role] = {
            "baseline_mean_ms": b / 1e6,
            "explore_mean_ms": e / 1e6,
            "delta_ms": (e - b) / 1e6,
            "ratio": (e / b) if b > 0 else None,
        }

    def timing_mean(payload: Mapping[str, Any], label: str) -> int:
        return int(payload.get("runtime_timing", {}).get(label, {}).get("mean_ns", 0))

    timing_labels = sorted(
        set(stop.get("runtime_timing", {})) | set(explore.get("runtime_timing", {}))
    )
    timing_delta = {}
    for label in timing_labels:
        b = timing_mean(stop, label)
        e = timing_mean(explore, label)
        timing_delta[label] = {
            "baseline_mean_ms": b / 1e6,
            "explore_mean_ms": e / 1e6,
            "delta_ms": (e - b) / 1e6,
            "ratio": (e / b) if b > 0 else None,
        }

    return {
        "source_delta": source_delta,
        "runtime_timing_delta": timing_delta,
    }


def build_root_cause(
    baseline: Mapping[str, Any],
    explore: Mapping[str, Any],
    delta: Mapping[str, Any],
    runtime_report: Mapping[str, Any],
) -> dict[str, Any]:
    sources = explore.get("sources", {})
    ranked = sorted(
        (
            (
                role,
                float(stats.get("mean_ms", 0.0)),
                float(stats.get("p99_ms", 0.0)),
            )
            for role, stats in sources.items()
        ),
        key=lambda item: item[1],
        reverse=True,
    )

    source_delta = delta.get("source_delta", {})
    ranked_growth = sorted(
        (
            (
                role,
                float(values.get("delta_ms", 0.0)),
            )
            for role, values in source_delta.items()
        ),
        key=lambda item: item[1],
        reverse=True,
    )

    explore_count = int(explore.get("primary_tick_count", 0))
    status_ok = (
        runtime_report.get("status") == "PASS"
        or runtime_report.get("run_status") == 0
    )
    confidence = "INCOMPLETE"
    if status_ok and explore_count >= 100:
        confidence = "HIGH_CONFIDENCE_FULL_RUNTIME_SHADOW_MOTOR"
    if status_ok and explore_count >= 300:
        confidence = "VERY_HIGH_CONFIDENCE_FULL_RUNTIME_SHADOW_MOTOR"

    primary = ranked[0][0] if ranked else None
    growth = ranked_growth[0][0] if ranked_growth else None

    targets = []
    for role, mean_ms, p99_ms in ranked[:3]:
        targets.append({
            "component": role,
            "mean_ms": mean_ms,
            "p99_ms": p99_ms,
            "source_files": FIX_TARGETS.get(role, []),
        })

    return {
        "confidence": confidence,
        "runtime_completed_without_fault": status_ok,
        "explore_primary_tick_count": explore_count,
        "largest_direct_l0_component": primary,
        "largest_explore_vs_stop_growth": growth,
        "ranked_direct_l0_sources": [
            {"component": role, "mean_ms": mean_ms, "p99_ms": p99_ms}
            for role, mean_ms, p99_ms in ranked
        ],
        "ranked_explore_vs_stop_growth": [
            {"component": role, "delta_ms": delta_ms}
            for role, delta_ms in ranked_growth
        ],
        "target_files": targets,
        "important_limit": (
            "Motor GPIO is intentionally absent. L11 global feedback-uncertainty "
            "watchdog is shadowed only inside this diagnostic because stationary "
            "encoders cannot validate a non-zero EXPLORE wheel target. Movement-"
            "dependent encoder callback load and physical motor-side effects are "
            "not reproduced."
        ),
    }


def heartbeat_worker(
    stop_event: threading.Event,
    collector: Collector,
    client: Any,
    *,
    warmup_s: float,
    baseline_s: float,
    measure_s: float,
    heartbeat_s: float,
    mode: str,
    max_v_mps: float,
    max_omega_rad_s: float,
) -> None:
    command_id = f"diag-v2-{os.getpid()}-{time.monotonic_ns()}"
    ttl_ns = 200_000_000

    # Hardware initialization may take seconds. Keep a STOP heartbeat alive until
    # the first real L0 tick proves that resident control is running.
    while not stop_event.is_set() and not collector.runtime_tick_event.is_set():
        client.publish_stop(command_id, ttl_ns=ttl_ns)
        collector.runtime_tick_event.wait(timeout=heartbeat_s)

    warmup_end = time.monotonic() + warmup_s
    while not stop_event.is_set() and time.monotonic() < warmup_end:
        client.publish_stop(command_id, ttl_ns=ttl_ns)
        stop_event.wait(heartbeat_s)

    collector.set_phase("BASELINE_STOP")
    baseline_end = time.monotonic() + baseline_s
    while not stop_event.is_set() and time.monotonic() < baseline_end:
        client.publish_stop(command_id, ttl_ns=ttl_ns)
        stop_event.wait(heartbeat_s)

    collector.set_phase("EXPLORE" if mode == "explore" else "STOP_MEASURE")
    measure_end = time.monotonic() + measure_s
    try:
        while not stop_event.is_set() and time.monotonic() < measure_end:
            if mode == "explore":
                client.publish_explore(
                    command_id,
                    max_v_mps=max_v_mps,
                    max_omega_rad_s=max_omega_rad_s,
                    ttl_ns=ttl_ns,
                )
            else:
                client.publish_stop(command_id, ttl_ns=ttl_ns)
            stop_event.wait(heartbeat_s)
    finally:
        collector.set_phase(None)
        try:
            client.publish_stop(command_id, ttl_ns=ttl_ns)
        finally:
            stop_event.set()


def print_phase(name: str, payload: Mapping[str, Any]) -> None:
    print(name)
    print(f"  primary ticks: {payload.get('primary_tick_count', 0)}")
    for role in ("ENCODER", "IMU", "LIDAR", "CAMERA", "PERSON"):
        stats = payload.get("sources", {}).get(role)
        if not stats:
            continue
        print(
            f"  {role:<8} mean={stats['mean_ms']:7.3f} ms "
            f"p95={stats['p95_ms']:7.3f} p99={stats['p99_ms']:7.3f} "
            f"max={stats['max_ms']:7.3f}"
        )
    reader = payload.get("reader_total", {})
    residual = payload.get("reader_non_source", {})
    print(
        f"  L0 TOTAL mean={reader.get('mean_ms', 0):7.3f} ms "
        f"p99={reader.get('p99_ms', 0):7.3f} "
        f"residual={residual.get('mean_ms', 0):7.3f} ms"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Low-perturbation full production V3 runtime diagnostic with NULL "
            "motor output and diagnostic-only L11 feedback-watchdog shadow."
        )
    )
    parser.add_argument("--repo", type=Path, default=Path("/home/alba/project_r2b4"))
    parser.add_argument("--mode", choices=("explore", "stop"), default="explore")
    parser.add_argument("--warmup-s", type=float, default=3.0)
    parser.add_argument("--baseline-s", type=float, default=5.0)
    parser.add_argument("--measure-s", type=float, default=20.0)
    parser.add_argument("--heartbeat-s", type=float, default=0.08)
    parser.add_argument("--max-v-mps", type=float, default=0.25)
    parser.add_argument("--max-omega-rad-s", type=float, default=0.8)
    parser.add_argument(
        "--mechanism-every",
        type=int,
        default=25,
        help=(
            "Detailed /proc CPU/runqueue sample cadence in ticks. Profiled ticks "
            "are excluded from primary latency stats. 0 disables detailed samples."
        ),
    )
    parser.add_argument("--capture", choices=("on", "off"), default="on")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if not 1.0 <= args.warmup_s <= 20.0:
        parser.error("--warmup-s must be within [1,20]")
    if not 2.0 <= args.baseline_s <= 60.0:
        parser.error("--baseline-s must be within [2,60]")
    if not 5.0 <= args.measure_s <= 120.0:
        parser.error("--measure-s must be within [5,120]")
    if not 0.04 <= args.heartbeat_s <= 0.20:
        parser.error("--heartbeat-s must be within [0.04,0.20]")
    if args.mechanism_every < 0:
        parser.error("--mechanism-every must be >= 0")

    repo = args.repo.expanduser().resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"ERROR: not an R2B4 repo: {repo}")

    existing = active_pid(repo / "runtime" / ".r2b4_runtime_pid")
    if existing is not None:
        raise SystemExit(
            f"ERROR: resident runtime appears active (PID {existing}). Stop it first."
        )

    sys.path.insert(0, str(repo))

    try:
        import lgpio
        import serial
        import smbus2
        from v3.adapters.resident_command import (
            AtomicResidentCommandGateway,
            ResidentCommandClient,
            ResidentCommandMailboxConfig,
        )
        from v3.mcap_capture import McapCaptureConfig
        from v3.runtime_performance import load_runtime_affinity_config
        from v3_process_runtime import (
            AsyncResidentStatusPublisher,
            McapCaptureSession,
            ResidentStatusConfig,
            _capture_configuration,
            load_resident_runtime_config,
            native_lidar_factory,
            run_v3_resident_process,
        )
    except Exception as exc:
        raise SystemExit(
            f"ERROR importing production runtime: {type(exc).__name__}: {exc}"
        ) from exc

    destination = Path(
        tempfile.mkdtemp(prefix="r2b4-rootcause-v2-", dir="/tmp")
    )
    command_path = destination / "command.json"
    mailbox = ResidentCommandMailboxConfig(command_path)
    gateway = AtomicResidentCommandGateway(mailbox)
    client = ResidentCommandClient(mailbox)

    config = load_resident_runtime_config(repo)
    affinity = load_runtime_affinity_config(repo / "conf" / "vezerles.json")
    period_ns = int(config.tick_period_ns)

    collector = Collector(args.mechanism_every)
    patches = build_patches(collector)

    capture_session = None
    if args.capture == "on":
        capture_session = McapCaptureSession(
            destination.name,
            destination / "capture.mcap",
            configuration=_capture_configuration(repo, config),
            metadata={
                "purpose": "control root-cause diagnostic v2",
                "motor_output": "NULL_SINK_NO_GPIO",
                "l11_feedback_watchdog": "DIAGNOSTIC_SHADOW",
                "mode": args.mode,
            },
            config=McapCaptureConfig(mode="append_only"),
        )

    publisher = AsyncResidentStatusPublisher(
        ResidentStatusConfig(destination / "status.json")
    )
    stop_event = threading.Event()
    hb = threading.Thread(
        target=heartbeat_worker,
        name="r2b4-diag-v2-heartbeat",
        args=(stop_event, collector, client),
        kwargs={
            "warmup_s": args.warmup_s,
            "baseline_s": args.baseline_s,
            "measure_s": args.measure_s,
            "heartbeat_s": args.heartbeat_s,
            "mode": args.mode,
            "max_v_mps": args.max_v_mps,
            "max_omega_rad_s": args.max_omega_rad_s,
        },
        daemon=False,
    )

    print("R2B4 V3 CONTROL ROOT-CAUSE DIAG V2")
    print(f"target:          {1e9 / period_ns:.3f} Hz / {period_ns / 1e6:.3f} ms")
    print(f"mode:            {args.mode.upper()}")
    print(f"warmup:          {args.warmup_s:.1f} s")
    print(f"baseline STOP:   {args.baseline_s:.1f} s")
    print(f"measurement:     {args.measure_s:.1f} s")
    print(f"capture:         {args.capture.upper()}")
    print(f"mechanism every: {args.mechanism_every} ticks")
    print("motor output:    NULL -- motor GPIO NOT opened")
    print("L11 watchdog:    diagnostic SHADOW for missing stationary encoder feedback")
    print()

    report = None
    try:
        hb.start()
        report = run_v3_resident_process(
            lgpio,
            smbus2.SMBus,
            native_lidar_factory(
                config.sensor_inputs,
                serial.Serial,
                repo,
                affinity,
            ),
            lgpio,
            gateway,
            config,
            publisher,
            approval="native-resident-v3",
            stop_requested=stop_event.is_set,
            capture_session=capture_session,
        )
    finally:
        collector.set_phase(None)
        stop_event.set()
        if hb.is_alive():
            hb.join(timeout=3.0)
        patches.restore()

    if report is None:
        raise SystemExit("ERROR: resident runtime returned no report")

    report_dict = report.as_dict()
    baseline = analyze_phase(collector, "BASELINE_STOP", period_ns)
    measured_name = "EXPLORE" if args.mode == "explore" else "STOP_MEASURE"
    measured = analyze_phase(collector, measured_name, period_ns)
    delta = delta_summary(baseline, measured)
    root = build_root_cause(baseline, measured, delta, report_dict)

    null_writes = sum(item.write_count for item in NullMotorSink.instances)
    payload = {
        "schema": SCHEMA,
        "generated_local": datetime.now().astimezone().isoformat(),
        "repo": str(repo),
        "scope": {
            "canonical_resident_runtime": True,
            "production_sensors": True,
            "production_camera_person_when_configured": True,
            "production_l0_l12": True,
            "production_l6_planner": True,
            "production_capture_enabled": args.capture == "on",
            "physical_motor_gpio_opened": False,
            "physical_motion": False,
            "l11_feedback_watchdog": "DIAGNOSTIC_SHADOW_ONLY",
            "l11_shadow_reason": (
                "NULL motor cannot produce encoder feedback for non-zero EXPLORE "
                "targets; production L11 would correctly fail closed."
            ),
        },
        "configuration": {
            "target_period_ns": period_ns,
            "target_hz": 1e9 / period_ns,
            "warmup_s": args.warmup_s,
            "baseline_s": args.baseline_s,
            "measure_s": args.measure_s,
            "heartbeat_s": args.heartbeat_s,
            "mechanism_every": args.mechanism_every,
            "max_v_mps": args.max_v_mps,
            "max_omega_rad_s": args.max_omega_rad_s,
            "runtime_affinity": affinity.as_dict(),
        },
        "motor_safety_evidence": {
            "null_sink_instance_count": len(NullMotorSink.instances),
            "null_sink_write_count": null_writes,
            "all_null_sinks_closed": all(
                item.closed for item in NullMotorSink.instances
            ),
            "physical_gpio_write_capability_created": False,
        },
        "runtime_report": report_dict,
        "baseline_stop": baseline,
        measured_name.lower(): measured,
        "baseline_vs_measurement": delta,
        "root_cause": root,
        "raw_primary_ticks": [
            {
                "phase": row.phase,
                "tick_id": row.tick_id,
                "reader_wall_ns": row.reader_wall_ns,
                "source_wall_ns": row.source_wall_ns,
                "reader_non_source_ns": row.reader_non_source_ns,
                "detailed_profile_tick": row.detailed_profile_tick,
            }
            for row in collector.tick_rows
        ],
        "raw_mechanism_samples": [
            {
                "phase": row.phase,
                "tick_id": row.tick_id,
                "role": row.role,
                "wall_ns": row.wall_ns,
                "thread_cpu_ns": row.thread_cpu_ns,
                "process_cpu_ns": row.process_cpu_ns,
                "runqueue_wait_ns": row.runqueue_wait_ns,
                "blocked_or_gil_ns": row.blocked_or_gil_ns,
            }
            for row in collector.mechanism_rows
        ],
    }

    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = repo / "runtime" / "diag" / f"control_rootcause_diag_v2_{stamp}.json"
    elif not output.is_absolute():
        output = (repo / output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    print_phase("BASELINE_STOP", baseline)
    print()
    print_phase(measured_name, measured)
    print()

    print("EXPLORE vs BASELINE SOURCE DELTA")
    for role, item in delta["source_delta"].items():
        print(
            f"  {role:<8} baseline={item['baseline_mean_ms']:7.3f} ms "
            f"measure={item['explore_mean_ms']:7.3f} ms "
            f"delta={item['delta_ms']:+7.3f} ms"
        )

    print()
    print("ROOT CAUSE")
    print(f"  confidence:              {root['confidence']}")
    print(f"  runtime PASS:            {root['runtime_completed_without_fault']}")
    print(f"  measurement tick count:  {root['explore_primary_tick_count']}")
    print(f"  largest direct source:   {root['largest_direct_l0_component']}")
    print(f"  largest workload growth: {root['largest_explore_vs_stop_growth']}")
    print(f"  runtime termination:     {report_dict.get('exit_reason')}")
    print()
    print(f"JSON: {output}")
    print(f"temp: {destination}")

    return 0 if root["runtime_completed_without_fault"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
