#!/usr/bin/env python3
"""
R2B4 V3 full control-loop root-cause diagnostic.

This is a diagnostic launcher, not a synthetic micro-benchmark.

It starts the canonical production resident V3 process with:
  - production encoder owner
  - production BNO055 IMU
  - production RPLIDAR + matcher process
  - production camera and person detector if configured
  - production L0-L12 control path
  - production L6 process planner
  - production command mailbox parser
  - production MCAP capture / ObservationHub when enabled
  - production CPU-affinity policy

For safety, the physical motor frame sink is replaced in-memory by a NULL sink.
The complete L12 planning/writer conversion still runs, but no motor GPIO is
opened and no PWM can be emitted.

The tool instruments the REAL runtime in-process:
  * each L0 source read separately
  * total NativeLiveInputReader time
  * L0 reader aggregation overhead
  * command snapshot
  * runtime timing phases already emitted by V3
  * ObservationHub publication
  * async status publication callback
  * wall time vs thread CPU
  * Linux schedstat runqueue wait
  * inferred blocked/GIL/sleep elapsed
  * process CPU growth while each main-thread span is active

The resulting JSON includes per-tick raw measurements and an accounting check:
sum(source reads + aggregation overhead) versus the actual L0 reader duration.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import statistics
import sys
import tempfile
import threading
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping

SCHEMA = "R2B4_V3_CONTROL_ROOTCAUSE_DIAG_V1"


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


def active_pid(pid_path: Path) -> int | None:
    try:
        pid = int(pid_path.read_text(encoding="ascii").strip())
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
    """Return current Linux thread runtime_ns, runqueue_wait_ns, timeslices."""
    tid = threading.get_native_id()
    try:
        parts = Path(f"/proc/self/task/{tid}/schedstat").read_text(encoding="ascii").split()
        if len(parts) >= 3:
            return int(parts[0]), int(parts[1]), int(parts[2])
    except Exception:
        pass
    return None


@dataclass(slots=True)
class Span:
    tick_id: int | None
    wall_ns: int
    thread_cpu_ns: int
    process_cpu_ns: int
    runqueue_wait_ns: int
    blocked_or_gil_ns: int
    thread_name: str
    native_tid: int


class Collector:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.active = False
        self.spans: dict[str, list[Span]] = defaultdict(list)
        self.scalar_ns: dict[str, list[int]] = defaultdict(list)
        self._tls = threading.local()
        self.measurement_started_ns: int | None = None
        self.measurement_ended_ns: int | None = None
        self.runtime_tick_event = threading.Event()

    def set_active(self, value: bool) -> None:
        with self.lock:
            self.active = bool(value)
            now = time.monotonic_ns()
            if value and self.measurement_started_ns is None:
                self.measurement_started_ns = now
            if not value and self.measurement_started_ns is not None:
                self.measurement_ended_ns = now

    @property
    def current_tick_id(self) -> int | None:
        return getattr(self._tls, "tick_id", None)

    @contextlib.contextmanager
    def tick(self, tick_id: int):
        old = getattr(self._tls, "tick_id", None)
        self._tls.tick_id = int(tick_id)
        try:
            yield
        finally:
            self._tls.tick_id = old

    def add_scalar(self, label: str, duration_ns: int) -> None:
        if not self.active:
            return
        with self.lock:
            self.scalar_ns[label].append(max(0, int(duration_ns)))

    def measure(self, label: str, fn: Callable[[], Any], *, tick_id: int | None = None) -> Any:
        if not self.active:
            return fn()
        tick = self.current_tick_id if tick_id is None else tick_id
        sched0 = read_schedstat()
        p0 = time.process_time_ns()
        c0 = time.thread_time_ns()
        w0 = time.perf_counter_ns()
        try:
            return fn()
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
            # What wall time remains after this thread was executing or waiting
            # runnable on a CPU. It can contain blocking I/O, futex/GIL wait,
            # condition waits, kernel sleep, etc. It is deliberately not named
            # "I/O time".
            blocked_or_gil = max(0, wall - thread_cpu - rq)
            span = Span(
                tick_id=tick,
                wall_ns=wall,
                thread_cpu_ns=thread_cpu,
                process_cpu_ns=process_cpu,
                runqueue_wait_ns=rq,
                blocked_or_gil_ns=blocked_or_gil,
                thread_name=threading.current_thread().name,
                native_tid=threading.get_native_id(),
            )
            with self.lock:
                self.spans[label].append(span)

    def summary(self, label: str) -> dict[str, Any]:
        rows = self.spans.get(label, [])
        return {
            "wall": stats_ns([x.wall_ns for x in rows]),
            "thread_cpu": stats_ns([x.thread_cpu_ns for x in rows]),
            "process_cpu": stats_ns([x.process_cpu_ns for x in rows]),
            "runqueue_wait": stats_ns([x.runqueue_wait_ns for x in rows]),
            "blocked_or_gil": stats_ns([x.blocked_or_gil_ns for x in rows]),
            "thread_names": dict(Counter(x.thread_name for x in rows)),
            "native_tids": sorted(set(x.native_tid for x in rows)),
        }

    def raw(self, label: str) -> dict[str, list[Any]]:
        rows = self.spans.get(label, [])
        return {
            "tick_id": [x.tick_id for x in rows],
            "wall_ns": [x.wall_ns for x in rows],
            "thread_cpu_ns": [x.thread_cpu_ns for x in rows],
            "process_cpu_ns": [x.process_cpu_ns for x in rows],
            "runqueue_wait_ns": [x.runqueue_wait_ns for x in rows],
            "blocked_or_gil_ns": [x.blocked_or_gil_ns for x in rows],
        }


class NullMotorSink:
    """Drop-in GpioMotorFrameSink replacement; never opens GPIO."""

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


def source_role(source: Any) -> str:
    name = type(source).__name__.upper()
    device = str(getattr(source, "device_id", "")).upper()
    token = name + " " + device
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


class PatchSet:
    def __init__(self) -> None:
        self._items: list[tuple[Any, str, Any]] = []

    def set(self, obj: Any, name: str, value: Any) -> None:
        self._items.append((obj, name, getattr(obj, name)))
        setattr(obj, name, value)

    def restore(self) -> None:
        for obj, name, original in reversed(self._items):
            setattr(obj, name, original)
        self._items.clear()


def build_patches(collector: Collector):
    from v3.contracts import RawDeviceBatch, TickContext
    from v3.adapters.live_inputs import NativeLiveInputReader, LiveDeviceSnapshot
    from v3.adapters.resident_command import AtomicResidentCommandGateway
    from v3.observation import ObservationHub
    import v3.composition.motor_output as motor_output_module
    import v3.runtime_performance as runtime_performance
    import v3_process_runtime as process_runtime

    patches = PatchSet()

    # Motor GPIO removal: leave NativeMotorWriter + final-actuation planning intact.
    patches.set(motor_output_module, "GpioMotorFrameSink", NullMotorSink)

    original_reader_read = NativeLiveInputReader.read

    def diagnostic_reader_read(self, context):
        collector.runtime_tick_event.set()
        if not collector.active:
            return original_reader_read(self, context)
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")

        def whole_reader():
            samples = []
            health = []
            source_total_ns = 0
            with collector.tick(context.tick_id):
                for expected_device_id, source in zip(self._source_ids, self._sources):
                    role = source_role(source)
                    source_label = f"L0_SOURCE:{role}"
                    snapshot = collector.measure(
                        source_label,
                        lambda source=source: source.read(context),
                        tick_id=context.tick_id,
                    )
                    with collector.lock:
                        source_total_holder[0] += collector.spans[source_label][-1].wall_ns
                    if not isinstance(snapshot, LiveDeviceSnapshot):
                        raise TypeError("live device source must return LiveDeviceSnapshot")
                    if snapshot.context != context:
                        raise ValueError("live device snapshot context must match the tick")
                    if snapshot.health.device_id != expected_device_id:
                        raise ValueError("live device snapshot ID must match its configured source")
                    samples.extend(snapshot.samples)
                    health.append(snapshot.health)

                agg0 = time.perf_counter_ns()
                batch = RawDeviceBatch(context, tuple(samples), tuple(health))
                agg_ns = max(0, time.perf_counter_ns() - agg0)
                collector.add_scalar("L0_RAW_BATCH_CONSTRUCTION", agg_ns)
                return batch

        source_total_holder = [0]
        result = collector.measure("L0_READER_TOTAL", whole_reader, tick_id=context.tick_id)
        with collector.lock:
            reader_wall_ns = collector.spans["L0_READER_TOTAL"][-1].wall_ns
        # Exact tick accounting inside the instrumented production reader:
        # reader total = source spans + validation/list/RawDeviceBatch/instrumentation residue.
        collector.add_scalar(
            "L0_READER_NON_SOURCE",
            max(0, reader_wall_ns - source_total_holder[0]),
        )
        return result

    patches.set(NativeLiveInputReader, "read", diagnostic_reader_read)

    original_snapshot = AtomicResidentCommandGateway.snapshot
    def timed_snapshot(self, context):
        tick_id = getattr(context, "tick_id", None)
        return collector.measure(
            "COMMAND_SNAPSHOT_DIRECT",
            lambda: original_snapshot(self, context),
            tick_id=tick_id,
        )
    patches.set(AtomicResidentCommandGateway, "snapshot", timed_snapshot)

    original_hub_publish = ObservationHub.publish
    def timed_hub_publish(self, value, *, topic):
        return collector.measure(
            f"OBSERVER_HUB_PUBLISH:{topic}",
            lambda: original_hub_publish(self, value, topic=topic),
        )
    patches.set(ObservationHub, "publish", timed_hub_publish)

    original_publish_tick = process_runtime.AsyncResidentStatusPublisher.publish_tick
    def timed_publish_tick(self, result, ready_for_active=False):
        tick_id = getattr(getattr(getattr(result, "trace", None), "context", None), "tick_id", None)
        return collector.measure(
            "OBSERVER_STATUS_PUBLISH",
            lambda: original_publish_tick(self, result, ready_for_active),
            tick_id=tick_id,
        )
    patches.set(process_runtime.AsyncResidentStatusPublisher, "publish_tick", timed_publish_tick)

    # Tee the existing native timing recorder. This preserves current production
    # timing while providing its values directly in the diagnostic JSON.
    Acc = runtime_performance.RuntimeTimingAccumulator
    for method_name, label_prefix in (
        ("observe_control_phase", "RUNTIME_PHASE"),
        ("observe_control", "RUNTIME_CONTROL_TOTAL"),
        ("observe_observer", "RUNTIME_OBSERVER_TOTAL"),
        ("observe_work", "RUNTIME_WORK_TOTAL"),
        ("observe_period", "RUNTIME_PERIOD"),
    ):
        if not hasattr(Acc, method_name):
            continue
        original = getattr(Acc, method_name)
        if method_name == "observe_control_phase":
            def make_phase_wrapper(orig):
                def wrapper(self, name, duration_ns):
                    collector.add_scalar(f"RUNTIME_PHASE:{name}", duration_ns)
                    return orig(self, name, duration_ns)
                return wrapper
            patches.set(Acc, method_name, make_phase_wrapper(original))
        else:
            def make_wrapper(orig, label):
                def wrapper(self, duration_ns, *args, **kwargs):
                    collector.add_scalar(label, duration_ns)
                    return orig(self, duration_ns, *args, **kwargs)
                return wrapper
            patches.set(Acc, method_name, make_wrapper(original, label_prefix))

    return patches


def print_stat(label: str, payload: dict[str, Any]) -> None:
    s = payload["wall"]
    print(
        f"{label:<30} mean={s['mean_ms']:8.3f} ms  p50={s['p50_ms']:8.3f}  "
        f"p95={s['p95_ms']:8.3f}  p99={s['p99_ms']:8.3f}  max={s['max_ms']:8.3f}"
    )


def safe_mean(stats: Mapping[str, Any]) -> float:
    return float(stats.get("mean_ms", 0.0))


def build_analysis(
    collector: Collector,
    runtime_report: Mapping[str, Any],
    period_ns: int,
) -> dict[str, Any]:
    labels = sorted(collector.spans)
    summaries = {label: collector.summary(label) for label in labels}
    raw = {label: collector.raw(label) for label in labels}

    source_labels = [x for x in labels if x.startswith("L0_SOURCE:")]
    source_means_ns = {
        label: summaries[label]["wall"]["mean_ns"]
        for label in source_labels
    }
    source_sum_mean_ns = sum(source_means_ns.values())
    l0 = summaries.get("L0_READER_TOTAL", {})
    l0_mean_ns = int(l0.get("wall", {}).get("mean_ns", 0))
    non_source_values = collector.scalar_ns.get("L0_READER_NON_SOURCE", [])
    non_source = stats_ns(non_source_values)
    accounted_mean_ns = source_sum_mean_ns + int(non_source["mean_ns"])
    residual_ns = l0_mean_ns - accounted_mean_ns

    by_tick: dict[int, dict[str, int]] = defaultdict(dict)
    for label in source_labels + ["L0_READER_TOTAL"]:
        for row in collector.spans.get(label, []):
            if row.tick_id is not None:
                by_tick[row.tick_id][label] = row.wall_ns

    # Which source owns the large L0 ticks?
    slow_ticks = []
    for tick_id, row in by_tick.items():
        total = row.get("L0_READER_TOTAL", 0)
        if total <= period_ns:
            continue
        source_parts = {
            k.split(":", 1)[1]: v
            for k, v in row.items()
            if k.startswith("L0_SOURCE:")
        }
        owner = max(source_parts.items(), key=lambda item: item[1], default=("UNKNOWN", 0))
        slow_ticks.append({
            "tick_id": tick_id,
            "l0_ms": total / 1e6,
            "largest_source": owner[0],
            "largest_source_ms": owner[1] / 1e6,
            "sources_ms": {k: v / 1e6 for k, v in source_parts.items()},
        })

    dominant_mean = None
    if source_means_ns:
        dominant_mean = max(source_means_ns, key=source_means_ns.get)

    dominant_p99 = None
    if source_labels:
        dominant_p99 = max(
            source_labels,
            key=lambda label: summaries[label]["wall"]["p99_ns"],
        )

    dominant_blocked = None
    if source_labels:
        dominant_blocked = max(
            source_labels,
            key=lambda label: summaries[label]["blocked_or_gil"]["mean_ns"],
        )

    dominant_cpu = None
    if source_labels:
        dominant_cpu = max(
            source_labels,
            key=lambda label: summaries[label]["thread_cpu"]["mean_ns"],
        )

    accounting_error_pct = (
        abs(residual_ns) / l0_mean_ns * 100.0
        if l0_mean_ns > 0 else 0.0
    )

    fix_targets = {
        "L0_SOURCE:ENCODER": [
            "v3/adapters/gpio_encoder.py",
            "v3/adapters/counter_encoder.py",
            "v3/adapters/gpio_counter.py",
        ],
        "L0_SOURCE:IMU": [
            "v3/adapters/bno055_device.py",
            "v3/adapters/bno055_imu.py",
            "v3/adapters/live_imu.py",
        ],
        "L0_SOURCE:LIDAR": [
            "v3/adapters/latest_lidar.py",
            "v3/adapters/live_lidar.py",
            "v3/adapters/native_lidar_port.py",
        ],
        "L0_SOURCE:CAMERA": [
            "v3/adapters/live_camera.py",
            "v3/adapters/picamera2_camera.py",
        ],
        "L0_SOURCE:PERSON": [
            "v3/adapters/live_person_detection.py",
            "v3/adapters/person_detection.py",
            "v3/adapters/litert_person_detector.py",
        ],
        "L0_READER_NON_SOURCE": ["v3/adapters/live_inputs.py"],
        "COMMAND_SNAPSHOT_DIRECT": ["v3/adapters/resident_command.py"],
        "OBSERVER_HUB_PUBLISH:v3.capture_record": [
            "v3/observation.py",
            "v3_process_runtime.py",
            "v3/mcap_capture.py",
        ],
        "OBSERVER_STATUS_PUBLISH": ["v3_process_runtime.py"],
    }

    confidence = "INCOMPLETE"
    if l0_mean_ns > 0 and accounting_error_pct <= 1.0:
        confidence = "DIRECT_L0_ACCOUNTING"
    if (
        confidence == "DIRECT_L0_ACCOUNTING"
        and source_labels
        and len(collector.spans.get("L0_READER_TOTAL", [])) >= 100
    ):
        confidence = "HIGH_CONFIDENCE_DIRECT_RUNTIME_EVIDENCE"

    return {
        "confidence": confidence,
        "l0_accounting": {
            "reader_total": l0.get("wall", stats_ns([])),
            "source_mean_ms": {
                label.split(":", 1)[1]: ns / 1e6
                for label, ns in source_means_ns.items()
            },
            "source_sum_mean_ms": source_sum_mean_ns / 1e6,
            "non_source_mean_ms": non_source["mean_ms"],
            "accounted_mean_ms": accounted_mean_ns / 1e6,
            "residual_mean_ms": residual_ns / 1e6,
            "accounting_error_percent": accounting_error_pct,
            "deadline_miss_count": len(slow_ticks),
            "deadline_miss_ticks": slow_ticks,
        },
        "dominant": {
            "mean_wall_source": dominant_mean,
            "p99_wall_source": dominant_p99,
            "mean_blocked_or_gil_source": dominant_blocked,
            "mean_thread_cpu_source": dominant_cpu,
        },
        "summaries": summaries,
        "scalar_summaries": {
            label: stats_ns(values)
            for label, values in sorted(collector.scalar_ns.items())
        },
        "raw": raw,
        "fix_targets": {
            key: value
            for key, value in fix_targets.items()
            if key in labels or key in collector.scalar_ns
        },
        "runtime_report": dict(runtime_report),
        "interpretation": {
            "thread_cpu": "Actual CPU time consumed by the instrumented Python thread.",
            "runqueue_wait": "Linux schedstat time runnable but waiting for CPU.",
            "blocked_or_gil": (
                "wall - thread_cpu - runqueue_wait. Can include blocking I/O, "
                "GIL/futex wait, condition wait or other sleeping; it is not "
                "automatically equivalent to I/O."
            ),
            "process_cpu": (
                "CPU consumed by the whole process during the span. If much larger "
                "than thread_cpu, other runtime threads were consuming CPU concurrently."
            ),
        },
    }


def heartbeat_worker(
    stop_event: threading.Event,
    measurement_event: threading.Event,
    collector: Collector,
    client: Any,
    *,
    warmup_s: float,
    measure_s: float,
    heartbeat_s: float,
    mode: str,
    max_v_mps: float,
    max_omega_rad_s: float,
) -> None:
    command_id = f"diag-{mode}-{os.getpid()}-{time.monotonic_ns()}"
    ttl_ns = 200_000_000

    # Keep STOP heartbeats alive during hardware initialization, then begin the
    # warmup clock only after the first real L0 tick has occurred.
    while not stop_event.is_set() and not collector.runtime_tick_event.is_set():
        client.publish_stop(command_id, ttl_ns=ttl_ns)
        collector.runtime_tick_event.wait(timeout=heartbeat_s)

    stop_until = time.monotonic() + warmup_s
    while not stop_event.is_set() and time.monotonic() < stop_until:
        client.publish_stop(command_id, ttl_ns=ttl_ns)
        stop_event.wait(heartbeat_s)

    collector.set_active(True)
    measurement_event.set()
    measure_until = time.monotonic() + measure_s
    try:
        while not stop_event.is_set() and time.monotonic() < measure_until:
            if mode == "explore":
                client.publish_explore(
                    command_id,
                    max_v_mps=max_v_mps,
                    max_omega_rad_s=max_omega_rad_s,
                    ttl_ns=ttl_ns,
                )
            elif mode == "stop":
                client.publish_stop(command_id, ttl_ns=ttl_ns)
            else:
                raise ValueError(f"unsupported mode: {mode}")
            stop_event.wait(heartbeat_s)
    finally:
        collector.set_active(False)
        try:
            client.publish_stop(command_id, ttl_ns=ttl_ns)
        finally:
            stop_event.set()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run the production R2B4 V3 runtime with NULL motor output and "
            "attribute control-loop/L0 latency to exact source locations."
        )
    )
    parser.add_argument("--repo", type=Path, default=Path("/home/alba/project_r2b4"))
    parser.add_argument("--mode", choices=("explore", "stop"), default="explore")
    parser.add_argument("--warmup-s", type=float, default=3.0)
    parser.add_argument("--measure-s", type=float, default=20.0)
    parser.add_argument("--heartbeat-s", type=float, default=0.08)
    parser.add_argument("--max-v-mps", type=float, default=0.25)
    parser.add_argument("--max-omega-rad-s", type=float, default=0.8)
    parser.add_argument(
        "--capture",
        choices=("on", "off"),
        default="on",
        help="Keep production MCAP ObservationHub/capture workload active.",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if not 1.0 <= args.warmup_s <= 20.0:
        parser.error("--warmup-s must be within [1, 20]")
    if not 5.0 <= args.measure_s <= 120.0:
        parser.error("--measure-s must be within [5, 120]")
    if not 0.04 <= args.heartbeat_s <= 0.20:
        parser.error("--heartbeat-s must be within [0.04, 0.20]")
    if not 0.01 <= args.max_v_mps <= 0.50:
        parser.error("--max-v-mps must be within [0.01, 0.50]")
    if not 0.05 <= args.max_omega_rad_s <= 1.20:
        parser.error("--max-omega-rad-s must be within [0.05, 1.20]")

    repo = args.repo.expanduser().resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"ERROR: not an R2B4 repo: {repo}")

    existing = active_pid(repo / "runtime" / ".r2b4_runtime_pid")
    if existing is not None:
        raise SystemExit(
            f"ERROR: normal resident runtime appears active (PID {existing}). "
            "Stop it before starting the diagnostic owner."
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
        tempfile.mkdtemp(prefix="r2b4-rootcause-diag-", dir="/tmp")
    )
    command_path = destination / "command.json"
    mailbox = ResidentCommandMailboxConfig(command_path)
    gateway = AtomicResidentCommandGateway(mailbox)
    client = ResidentCommandClient(mailbox)
    config = load_resident_runtime_config(repo)
    affinity_config = load_runtime_affinity_config(repo / "conf" / "vezerles.json")
    period_ns = int(config.tick_period_ns)

    collector = Collector()
    patches = build_patches(collector)

    capture_session = None
    if args.capture == "on":
        capture_session = McapCaptureSession(
            destination.name,
            destination / "capture.mcap",
            configuration=_capture_configuration(repo, config),
            metadata={
                "purpose": "full runtime root-cause diagnostic",
                "motor_output": "NULL_SINK_NO_GPIO",
                "mode": args.mode,
            },
            config=McapCaptureConfig(mode="append_only"),
        )

    publisher = AsyncResidentStatusPublisher(
        ResidentStatusConfig(destination / "status.json")
    )

    stop_event = threading.Event()
    measurement_event = threading.Event()
    hb = threading.Thread(
        target=heartbeat_worker,
        name="r2b4-diag-heartbeat",
        args=(stop_event, measurement_event, collector, client),
        kwargs={
            "warmup_s": args.warmup_s,
            "measure_s": args.measure_s,
            "heartbeat_s": args.heartbeat_s,
            "mode": args.mode,
            "max_v_mps": args.max_v_mps,
            "max_omega_rad_s": args.max_omega_rad_s,
        },
        daemon=False,
    )

    print("R2B4 V3 FULL CONTROL ROOT-CAUSE DIAG")
    print(f"repo:          {repo}")
    print(f"target:        {1e9 / period_ns:.3f} Hz / {period_ns / 1e6:.3f} ms")
    print(f"mode:          {args.mode.upper()}")
    print(f"warmup:        {args.warmup_s:.1f} s")
    print(f"measurement:   {args.measure_s:.1f} s")
    print(f"capture:       {args.capture.upper()}")
    print("motor output:  NULL SINK -- motor GPIO is NOT opened")
    print("sources:       production encoder + IMU + LiDAR + camera/person when configured")
    print()

    report = None
    runtime_error: BaseException | None = None
    try:
        hb.start()
        try:
            report = run_v3_resident_process(
                lgpio,
                smbus2.SMBus,
                native_lidar_factory(
                    config.sensor_inputs,
                    serial.Serial,
                    repo,
                    affinity_config,
                ),
                lgpio,
                gateway,
                config,
                publisher,
                approval="native-resident-v3",
                stop_requested=stop_event.is_set,
                capture_session=capture_session,
            )
        except BaseException as exc:
            runtime_error = exc
            raise
    finally:
        collector.set_active(False)
        stop_event.set()
        if hb.is_alive():
            hb.join(timeout=3.0)
        patches.restore()

    if report is None:
        raise SystemExit("ERROR: runtime returned no report")

    report_dict = report.as_dict()
    analysis = build_analysis(collector, report_dict, period_ns)

    null_writes = sum(x.write_count for x in NullMotorSink.instances)
    null_all_closed = all(x.closed for x in NullMotorSink.instances)

    payload = {
        "schema": SCHEMA,
        "generated_local": datetime.now().astimezone().isoformat(),
        "repo": str(repo),
        "scope": {
            "canonical_resident_runtime": True,
            "production_sensors": True,
            "production_camera_person_when_configured": True,
            "production_l0_l12": True,
            "production_planner": True,
            "production_command_mailbox": True,
            "production_capture_enabled": args.capture == "on",
            "physical_motor_gpio_opened": False,
            "motor_sink": "NULL",
            "physical_motion": False,
            "mode": args.mode,
            "important_limit": (
                "The diagnostic executes the full control workload but the robot does "
                "not physically move. Movement-dependent encoder callback load is "
                "therefore not reproduced."
            ),
        },
        "configuration": {
            "target_period_ns": period_ns,
            "target_hz": 1e9 / period_ns,
            "warmup_s": args.warmup_s,
            "measure_s": args.measure_s,
            "heartbeat_s": args.heartbeat_s,
            "max_v_mps": args.max_v_mps,
            "max_omega_rad_s": args.max_omega_rad_s,
            "runtime_affinity": affinity_config.as_dict(),
        },
        "motor_safety_evidence": {
            "null_sink_instance_count": len(NullMotorSink.instances),
            "null_sink_write_count": null_writes,
            "all_null_sinks_closed": null_all_closed,
            "physical_gpio_write_capability_created": False,
        },
        "analysis": analysis,
    }

    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = repo / "runtime" / "diag" / f"control_rootcause_diag_{stamp}.json"
    elif not output.is_absolute():
        output = (repo / output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("L0 SOURCE BREAKDOWN")
    for role in ("ENCODER", "IMU", "LIDAR", "CAMERA", "PERSON"):
        label = f"L0_SOURCE:{role}"
        if label in analysis["summaries"]:
            print_stat(role, analysis["summaries"][label])
    if "L0_READER_TOTAL" in analysis["summaries"]:
        print_stat("L0 TOTAL", analysis["summaries"]["L0_READER_TOTAL"])
    print()

    acc = analysis["l0_accounting"]
    print("L0 ACCOUNTING")
    print(f"source sum mean:       {acc['source_sum_mean_ms']:.3f} ms")
    print(f"reader non-source:     {acc['non_source_mean_ms']:.3f} ms")
    print(f"accounted mean:        {acc['accounted_mean_ms']:.3f} ms")
    print(f"actual reader mean:    {acc['reader_total']['mean_ms']:.3f} ms")
    print(f"accounting residual:   {acc['residual_mean_ms']:.6f} ms")
    print(f"accounting error:      {acc['accounting_error_percent']:.3f}%")
    print(f"L0 > period:           {acc['deadline_miss_count']} ticks")
    print()

    print("DOMINANT ROOT-CAUSE SIGNALS")
    for key, value in analysis["dominant"].items():
        print(f"{key:<28} {value}")
    print()

    print("WAIT / CPU MECHANISM")
    for role in ("ENCODER", "IMU", "LIDAR", "CAMERA", "PERSON"):
        label = f"L0_SOURCE:{role}"
        if label not in analysis["summaries"]:
            continue
        s = analysis["summaries"][label]
        print(
            f"{role:<10} wall={s['wall']['mean_ms']:.3f} ms  "
            f"threadCPU={s['thread_cpu']['mean_ms']:.3f}  "
            f"runqueue={s['runqueue_wait']['mean_ms']:.3f}  "
            f"blocked/GIL={s['blocked_or_gil']['mean_ms']:.3f}"
        )
    print()
    print(f"EVIDENCE: {analysis['confidence']}")
    print(f"JSON:     {output}")
    print(f"temp:     {destination}")

    return 0 if report.status == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
