#!/usr/bin/env python3
"""
R2B4 V3 peripheral critical-path diagnostic (IMU + ENCODER + LIDAR).

This tool does NOT start the resident runtime and does NOT open motor outputs.
It intentionally reuses the production V3 peripheral implementations and the
resident runtime configuration so it can measure how much the three core L0
peripherals contribute to a 50 Hz control-loop budget.

Production paths exercised
--------------------------
ENCODER:
    lgpio
      -> NativeGpioSignedCounterPair
      -> NativeCounterEncoderBackend
      -> NativeGpioEncoderSource.read(TickContext)

IMU:
    smbus2.SMBus
      -> NativeBno055Device
      -> NativeBno055ImuBackend
      -> NativeImuSource.read(TickContext)

LIDAR:
    pyserial / NativeRplidarC1
      -> NativeLidarPort (driver + matcher process + pump thread)
      -> NativeLatestLidarBackend
      -> NativeLidarSource.read(TickContext)

COMBINED CORE L0:
    NativeLiveInputReader(encoder, imu, lidar)

The three peripheral owners remain open simultaneously during every measured
phase. This preserves the background contention model much better than three
independent micro-benchmarks.

Evidence boundaries
-------------------
The tool directly proves critical-path elapsed time for the production source
reads in this standalone hardware session. It can also prove whether those
reads alone cause 20 ms deadline misses.

It cannot prove delays that exist only when other runtime capabilities are
active (camera/person/capture/command/observer/control layers). If an observed
runtime L0_READ mean is supplied, the unexplained remainder is reported but is
not assigned to a specific subsystem without direct evidence.

Encoder note: no motor is driven. Encoder callback/load evidence therefore
reflects actual physical wheel activity during the test. If the wheels remain
stationary, timing is a stationary encoder-path measurement and the report says
so explicitly.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable


SCHEMA = "R2B4_V3_PERIFERIAL_CONTROL_DIAG_V1"
TOOL_VERSION = 1


def _percentile(values: list[int], q: float) -> int:
    if not values:
        return 0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * q) - 1))
    return int(ordered[index])


def _stats_ns(values: list[int]) -> dict[str, Any]:
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
    mean_ns = int(round(sum(values) / len(values)))
    p50 = _percentile(values, 0.50)
    p95 = _percentile(values, 0.95)
    p99 = _percentile(values, 0.99)
    maximum = max(values)
    return {
        "count": len(values),
        "mean_ns": mean_ns,
        "p50_ns": p50,
        "p95_ns": p95,
        "p99_ns": p99,
        "max_ns": maximum,
        "mean_ms": mean_ns / 1_000_000.0,
        "p50_ms": p50 / 1_000_000.0,
        "p95_ms": p95 / 1_000_000.0,
        "p99_ms": p99 / 1_000_000.0,
        "max_ms": maximum / 1_000_000.0,
    }


def _active_pid(pid_path: Path) -> int | None:
    try:
        pid = int(pid_path.read_text(encoding="ascii").strip())
    except (FileNotFoundError, ValueError, OSError, UnicodeError):
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


def _sleep_until(deadline_ns: int) -> int:
    now_ns = time.monotonic_ns()
    while now_ns < deadline_ns:
        time.sleep((deadline_ns - now_ns) / 1_000_000_000.0)
        now_ns = time.monotonic_ns()
    return now_ns


def _effective_hz(starts: list[int]) -> float:
    if len(starts) < 2:
        return 0.0
    elapsed = starts[-1] - starts[0]
    if elapsed <= 0:
        return 0.0
    return (len(starts) - 1) * 1_000_000_000.0 / elapsed


def _field_map(snapshot: Any) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for sample in getattr(snapshot, "samples", ()):
        kind = str(getattr(sample, "kind", ""))
        result[f"sample_sequence:{kind}"] = getattr(sample, "sequence", None)
        for field in getattr(sample, "values", ()):
            key = getattr(field, "key", None)
            if isinstance(key, str):
                result[f"{kind}:{key}"] = getattr(field, "value", None)
    return result


def _health_state(snapshot: Any) -> str:
    health = getattr(snapshot, "health", None)
    state = getattr(health, "state", None)
    return str(getattr(state, "value", state))


@dataclass(slots=True)
class OpEvent:
    phase: str
    tick_id: int | None
    operation: str
    wall_ns: int
    thread_cpu_ns: int


@dataclass(slots=True)
class SourceEvent:
    phase: str
    tick_id: int
    wall_ns: int
    thread_cpu_ns: int
    health_state: str
    activity: dict[str, Any]


class ProbeContext:
    __slots__ = ("phase", "tick_id")

    def __init__(self) -> None:
        self.phase = "INIT"
        self.tick_id: int | None = None

    def set(self, phase: str, tick_id: int | None) -> None:
        self.phase = phase
        self.tick_id = tick_id


class TimedBus:
    """Transparent timing proxy around the production smbus2.SMBus object."""

    def __init__(self, bus: Any, device_cls: type, probe: ProbeContext) -> None:
        self._bus = bus
        self._device_cls = device_cls
        self._probe = probe
        self.events: list[OpEvent] = []

    def clear_events(self) -> None:
        self.events.clear()

    def _measure(self, operation: str, call: Callable[[], Any]) -> Any:
        cpu0 = time.thread_time_ns()
        wall0 = time.perf_counter_ns()
        try:
            return call()
        finally:
            wall1 = time.perf_counter_ns()
            cpu1 = time.thread_time_ns()
            self.events.append(
                OpEvent(
                    self._probe.phase,
                    self._probe.tick_id,
                    operation,
                    max(0, wall1 - wall0),
                    max(0, cpu1 - cpu0),
                )
            )

    def read_i2c_block_data(self, address: int, register: int, length: int) -> Any:
        operation = (
            "IMU_BURST_32"
            if register == self._device_cls.REG_GYRO_DATA_X_LSB and length == 32
            else "I2C_BLOCK_OTHER"
        )
        return self._measure(
            operation,
            lambda: self._bus.read_i2c_block_data(address, register, length),
        )

    def read_byte_data(self, address: int, register: int) -> Any:
        labels = {
            self._device_cls.REG_CALIB_STAT: "IMU_CALIB",
            self._device_cls.REG_SYS_STATUS: "IMU_SYS_STATUS",
            self._device_cls.REG_SYS_ERR: "IMU_SYS_ERROR",
            self._device_cls.REG_CHIP_ID: "IMU_CHIP_ID",
        }
        return self._measure(
            labels.get(register, "I2C_BYTE_OTHER"),
            lambda: self._bus.read_byte_data(address, register),
        )

    def write_byte_data(self, address: int, register: int, value: int) -> Any:
        return self._bus.write_byte_data(address, register, value)

    def close(self) -> Any:
        return self._bus.close()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._bus, name)


class TimedLidarPort:
    """Transparent timing proxy around the production latest-only LiDAR port."""

    def __init__(self, port: Any, probe: ProbeContext) -> None:
        self._port = port
        self._probe = probe
        self.events: list[OpEvent] = []

    def clear_events(self) -> None:
        self.events.clear()

    def _measure(self, operation: str, call: Callable[[], Any]) -> Any:
        cpu0 = time.thread_time_ns()
        wall0 = time.perf_counter_ns()
        try:
            return call()
        finally:
            wall1 = time.perf_counter_ns()
            cpu1 = time.thread_time_ns()
            self.events.append(
                OpEvent(
                    self._probe.phase,
                    self._probe.tick_id,
                    operation,
                    max(0, wall1 - wall0),
                    max(0, cpu1 - cpu0),
                )
            )

    def get_matcher_result(self) -> Any:
        return self._measure("LIDAR_GET_MATCHER_RESULT", self._port.get_matcher_result)

    def get_runtime_status(self) -> Any:
        return self._measure("LIDAR_GET_RUNTIME_STATUS", self._port.get_runtime_status)

    def get_raw_scan_snapshot(self) -> Any:
        return self._measure("LIDAR_GET_RAW_SCAN", self._port.get_raw_scan_snapshot)

    def stop(self) -> Any:
        return self._port.stop()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._port, name)


class TimedSource:
    """Passive wrapper preserving the V3 source contract while timing read()."""

    def __init__(
        self,
        label: str,
        source: Any,
        probe: ProbeContext,
    ) -> None:
        self.label = label
        self._source = source
        self._probe = probe
        self.phase = "UNSET"
        self.events: list[SourceEvent] = []

    @property
    def device_id(self) -> str:
        return self._source.device_id

    def set_phase(self, phase: str) -> None:
        self.phase = phase

    def clear_events(self) -> None:
        self.events.clear()

    def read(self, context: Any) -> Any:
        self._probe.set(self.phase, int(context.tick_id))
        cpu0 = time.thread_time_ns()
        wall0 = time.perf_counter_ns()
        snapshot = self._source.read(context)
        wall1 = time.perf_counter_ns()
        cpu1 = time.thread_time_ns()
        self.events.append(
            SourceEvent(
                phase=self.phase,
                tick_id=int(context.tick_id),
                wall_ns=max(0, wall1 - wall0),
                thread_cpu_ns=max(0, cpu1 - cpu0),
                health_state=_health_state(snapshot),
                activity=_field_map(snapshot),
            )
        )
        return snapshot


def _phase_events(source: TimedSource, phase: str) -> list[SourceEvent]:
    return [event for event in source.events if event.phase == phase]


def _source_summary(source: TimedSource, phase: str, period_ns: int) -> dict[str, Any]:
    events = _phase_events(source, phase)
    wall = [item.wall_ns for item in events]
    cpu = [item.thread_cpu_ns for item in events]
    non_cpu = [max(0, w - c) for w, c in zip(wall, cpu)]
    return {
        "wall": _stats_ns(wall),
        "thread_cpu": _stats_ns(cpu),
        "non_cpu_elapsed": _stats_ns(non_cpu),
        "over_5ms_count": sum(value > 5_000_000 for value in wall),
        "over_10ms_count": sum(value > 10_000_000 for value in wall),
        "over_period_count": sum(value > period_ns for value in wall),
        "health_states": dict(Counter(item.health_state for item in events)),
        "raw": {
            "tick_ids": [item.tick_id for item in events],
            "wall_ns": wall,
            "thread_cpu_ns": cpu,
        },
    }


def _operation_summary(events: Iterable[OpEvent], phase: str) -> dict[str, Any]:
    by_op_wall: dict[str, list[int]] = defaultdict(list)
    by_op_cpu: dict[str, list[int]] = defaultdict(list)
    for event in events:
        if event.phase != phase or event.tick_id is None:
            continue
        by_op_wall[event.operation].append(event.wall_ns)
        by_op_cpu[event.operation].append(event.thread_cpu_ns)
    return {
        name: {
            "wall": _stats_ns(by_op_wall[name]),
            "thread_cpu": _stats_ns(by_op_cpu[name]),
        }
        for name in sorted(by_op_wall)
    }


def _operation_per_tick(events: Iterable[OpEvent], phase: str) -> dict[str, Any]:
    wall: dict[int, int] = defaultdict(int)
    cpu: dict[int, int] = defaultdict(int)
    for event in events:
        if event.phase != phase or event.tick_id is None:
            continue
        wall[event.tick_id] += event.wall_ns
        cpu[event.tick_id] += event.thread_cpu_ns
    ids = sorted(wall)
    return {
        "wall": _stats_ns([wall[item] for item in ids]),
        "thread_cpu": _stats_ns([cpu[item] for item in ids]),
        "raw": {
            "tick_ids": ids,
            "wall_ns": [wall[item] for item in ids],
            "thread_cpu_ns": [cpu[item] for item in ids],
        },
    }


def _baseline_phase(*, samples: int, period_ns: int, TickContext: type, start_id: int) -> dict[str, Any]:
    starts: list[int] = []
    lateness: list[int] = []
    work: list[int] = []
    next_deadline = time.monotonic_ns()
    for offset in range(samples):
        now_ns = _sleep_until(next_deadline)
        starts.append(now_ns)
        lateness.append(max(0, now_ns - next_deadline))
        p0 = time.perf_counter_ns()
        _ = TickContext(start_id + offset, now_ns)
        p1 = time.perf_counter_ns()
        work.append(max(0, p1 - p0))
        next_deadline = max(next_deadline + period_ns, now_ns + 1)
    periods = [b - a for a, b in zip(starts, starts[1:])]
    return {
        "samples": samples,
        "effective_hz": _effective_hz(starts),
        "period": _stats_ns(periods),
        "wake_lateness": _stats_ns(lateness),
        "context_work": _stats_ns(work),
        "raw": {"period_ns": periods, "wake_lateness_ns": lateness},
    }


def _single_source_phase(
    *,
    phase: str,
    source: TimedSource,
    samples: int,
    period_ns: int,
    TickContext: type,
    start_id: int,
) -> dict[str, Any]:
    source.set_phase(phase)
    starts: list[int] = []
    lateness: list[int] = []
    errors: list[dict[str, Any]] = []
    next_deadline = time.monotonic_ns()
    for offset in range(samples):
        tick_id = start_id + offset
        now_ns = _sleep_until(next_deadline)
        starts.append(now_ns)
        lateness.append(max(0, now_ns - next_deadline))
        try:
            source.read(TickContext(tick_id, now_ns))
        except BaseException as exc:
            errors.append({"tick_id": tick_id, "type": type(exc).__name__, "message": str(exc)})
            break
        next_deadline = max(next_deadline + period_ns, now_ns + 1)
    periods = [b - a for a, b in zip(starts, starts[1:])]
    summary = _source_summary(source, phase, period_ns)
    summary.update(
        {
            "samples_requested": samples,
            "samples_completed": summary["wall"]["count"],
            "effective_hz": _effective_hz(starts),
            "period": _stats_ns(periods),
            "wake_lateness": _stats_ns(lateness),
            "errors": errors,
        }
    )
    return summary


def _combined_phase(
    *,
    phase: str,
    reader: Any,
    sources: tuple[TimedSource, ...],
    samples: int,
    period_ns: int,
    TickContext: type,
    start_id: int,
) -> dict[str, Any]:
    for source in sources:
        source.set_phase(phase)

    starts: list[int] = []
    lateness: list[int] = []
    wall: list[int] = []
    cpu: list[int] = []
    errors: list[dict[str, Any]] = []
    next_deadline = time.monotonic_ns()

    for offset in range(samples):
        tick_id = start_id + offset
        now_ns = _sleep_until(next_deadline)
        starts.append(now_ns)
        lateness.append(max(0, now_ns - next_deadline))
        context = TickContext(tick_id, now_ns)
        cpu0 = time.thread_time_ns()
        wall0 = time.perf_counter_ns()
        try:
            reader.read(context)
        except BaseException as exc:
            wall1 = time.perf_counter_ns()
            cpu1 = time.thread_time_ns()
            wall.append(max(0, wall1 - wall0))
            cpu.append(max(0, cpu1 - cpu0))
            errors.append({"tick_id": tick_id, "type": type(exc).__name__, "message": str(exc)})
            break
        wall1 = time.perf_counter_ns()
        cpu1 = time.thread_time_ns()
        wall.append(max(0, wall1 - wall0))
        cpu.append(max(0, cpu1 - cpu0))
        next_deadline = max(next_deadline + period_ns, now_ns + 1)

    periods = [b - a for a, b in zip(starts, starts[1:])]
    per_source = {source.label: _source_summary(source, phase, period_ns) for source in sources}

    event_maps = {
        source.label: {event.tick_id: event.wall_ns for event in _phase_events(source, phase)}
        for source in sources
    }
    tick_ids = [start_id + index for index in range(len(wall))]
    source_sum: list[int] = []
    aggregation: list[int] = []
    for index, tick_id in enumerate(tick_ids):
        summed = sum(event_maps[label].get(tick_id, 0) for label in event_maps)
        source_sum.append(summed)
        aggregation.append(max(0, wall[index] - summed))

    return {
        "samples_requested": samples,
        "samples_completed": len(wall),
        "effective_hz": _effective_hz(starts),
        "period": _stats_ns(periods),
        "wake_lateness": _stats_ns(lateness),
        "wall": _stats_ns(wall),
        "thread_cpu": _stats_ns(cpu),
        "non_cpu_elapsed": _stats_ns([max(0, w - c) for w, c in zip(wall, cpu)]),
        "source_sum": _stats_ns(source_sum),
        "reader_aggregation_overhead": _stats_ns(aggregation),
        "over_5ms_count": sum(value > 5_000_000 for value in wall),
        "over_10ms_count": sum(value > 10_000_000 for value in wall),
        "over_period_count": sum(value > period_ns for value in wall),
        "sources": per_source,
        "errors": errors,
        "raw": {
            "tick_ids": tick_ids,
            "wall_ns": wall,
            "thread_cpu_ns": cpu,
            "source_sum_ns": source_sum,
            "reader_aggregation_overhead_ns": aggregation,
        },
    }


def _phase_activity(source: TimedSource, phase: str) -> dict[str, Any]:
    events = _phase_events(source, phase)
    if not events:
        return {}
    if source.label == "ENCODER":
        left = [e.activity.get("wheel_velocity:raw_left_pulse_count") for e in events]
        right = [e.activity.get("wheel_velocity:raw_right_pulse_count") for e in events]
        left = [int(v) for v in left if isinstance(v, int) and not isinstance(v, bool)]
        right = [int(v) for v in right if isinstance(v, int) and not isinstance(v, bool)]
        return {
            "left_first": left[0] if left else None,
            "left_last": left[-1] if left else None,
            "left_delta": (left[-1] - left[0]) if len(left) >= 2 else None,
            "right_first": right[0] if right else None,
            "right_last": right[-1] if right else None,
            "right_delta": (right[-1] - right[0]) if len(right) >= 2 else None,
            "physical_edge_activity_observed": bool(
                (len(left) >= 2 and left[-1] != left[0])
                or (len(right) >= 2 and right[-1] != right[0])
            ),
        }
    if source.label == "LIDAR":
        sequences = []
        points = []
        for event in events:
            seq = event.activity.get("sample_sequence:lidar_health")
            if isinstance(seq, int) and not isinstance(seq, bool):
                sequences.append(seq)
            point_count = event.activity.get("lidar_health:point_count")
            if isinstance(point_count, int) and not isinstance(point_count, bool):
                points.append(point_count)
        distinct = len(set(sequences))
        return {
            "first_revision": sequences[0] if sequences else None,
            "last_revision": sequences[-1] if sequences else None,
            "distinct_revisions": distinct,
            "revision_change_count": sum(a != b for a, b in zip(sequences, sequences[1:])),
            "point_count_mean": (sum(points) / len(points)) if points else None,
            "physical_scan_activity_observed": distinct > 1,
        }
    if source.label == "IMU":
        sequences = []
        for event in events:
            seq = event.activity.get("sample_sequence:ekf_heading")
            if isinstance(seq, int) and not isinstance(seq, bool):
                sequences.append(seq)
        return {
            "sample_count": len(sequences),
            "sequence_matches_tick_count": len(sequences),
        }
    return {}


def _source_verdict(name: str, summary: dict[str, Any], period_ns: int) -> dict[str, Any]:
    wall = summary["wall"]
    samples = int(summary.get("samples_completed", wall["count"]))
    over = int(summary["over_period_count"])
    target_hz = 1_000_000_000.0 / period_ns
    hz = float(summary.get("effective_hz", 0.0))
    if samples > 0 and over > 0:
        primary = "PROVEN_DEADLINE_MISS_CONTRIBUTOR"
    elif samples > 0 and wall["p99_ns"] < period_ns and hz >= target_hz * 0.98:
        primary = "PROVEN_DOES_NOT_BY_ITSELF_BREAK_50HZ"
    else:
        primary = "MEASURED_CONTRIBUTION_ROOT_CAUSE_NOT_ISOLATED"
    return {
        "primary": primary,
        "mean_budget_fraction": wall["mean_ns"] / period_ns if period_ns else 0.0,
        "p99_budget_fraction": wall["p99_ns"] / period_ns if period_ns else 0.0,
        "deadline_miss_count": over,
        "samples": samples,
        "statement": (
            f"{name} direct source-read cost: mean {wall['mean_ms']:.3f} ms, "
            f"p95 {wall['p95_ms']:.3f} ms, p99 {wall['p99_ms']:.3f} ms, "
            f"{over}/{samples} reads over {period_ns / 1e6:.3f} ms."
        ),
    }


def _build_overall_verdict(
    *,
    period_ns: int,
    phases: dict[str, Any],
    observed_l0_ms: float | None,
    observed_loop_ms: float | None,
) -> dict[str, Any]:
    source_verdicts = {
        "ENCODER": _source_verdict("ENCODER", phases["ENCODER_ONLY"], period_ns),
        "IMU": _source_verdict("IMU", phases["IMU_ONLY"], period_ns),
        "LIDAR": _source_verdict("LIDAR", phases["LIDAR_ONLY"], period_ns),
    }
    core = phases["CORE_L0"]
    core_wall = core["wall"]
    core_sources = {
        name: data["wall"]["mean_ns"] for name, data in core["sources"].items()
    }
    ranking = sorted(core_sources, key=core_sources.get, reverse=True)
    source_mean_sum = sum(core_sources.values())

    result: dict[str, Any] = {
        "source_verdicts": source_verdicts,
        "core_l0": {
            "mean_ms": core_wall["mean_ms"],
            "p95_ms": core_wall["p95_ms"],
            "p99_ms": core_wall["p99_ms"],
            "max_ms": core_wall["max_ms"],
            "deadline_miss_count": core["over_period_count"],
            "samples": core["samples_completed"],
            "mean_budget_fraction": core_wall["mean_ns"] / period_ns,
            "source_mean_contribution_ms": {
                name: value / 1_000_000.0 for name, value in core_sources.items()
            },
            "source_mean_fraction_of_core": {
                name: (value / core_wall["mean_ns"] if core_wall["mean_ns"] else 0.0)
                for name, value in core_sources.items()
            },
            "source_ranking_by_mean": ranking,
            "reader_aggregation_mean_ms": core["reader_aggregation_overhead"]["mean_ms"],
            "source_mean_sum_ms": source_mean_sum / 1_000_000.0,
        },
        "interpretation_limit": (
            "Measured costs are direct standalone hardware evidence with all three core peripheral owners active. "
            "They do not include camera/person/capture/command/observer/control-layer interaction from the resident runtime."
        ),
    }

    if observed_l0_ms is not None and observed_l0_ms > 0:
        unexplained = observed_l0_ms - core_wall["mean_ms"]
        result["runtime_comparison"] = {
            "observed_l0_mean_ms": observed_l0_ms,
            "measured_core_l0_mean_ms": core_wall["mean_ms"],
            "measured_core_fraction_of_observed_l0": core_wall["mean_ms"] / observed_l0_ms,
            "unexplained_observed_l0_mean_ms": unexplained,
            "unexplained_fraction": unexplained / observed_l0_ms,
            "unexplained_claim": (
                "INDICATED_OTHER_RUNTIME_OR_AUXILIARY_COST"
                if unexplained > 0
                else "NO_POSITIVE_UNEXPLAINED_MEAN"
            ),
        }
    if observed_loop_ms is not None and observed_loop_ms > 0:
        result.setdefault("runtime_comparison", {})["observed_loop_mean_ms"] = observed_loop_ms
        result["runtime_comparison"]["measured_core_fraction_of_observed_loop"] = (
            core_wall["mean_ms"] / observed_loop_ms
        )
    return result


def _print_stats(label: str, stats: dict[str, Any]) -> None:
    print(
        f"{label:<31} mean={stats['mean_ms']:8.3f} ms  "
        f"p50={stats['p50_ms']:8.3f}  p95={stats['p95_ms']:8.3f}  "
        f"p99={stats['p99_ms']:8.3f}  max={stats['max_ms']:8.3f}"
    )


def _phase_gap(period_ns: int) -> None:
    time.sleep(max(0.02, period_ns / 1_000_000_000.0))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Diagnose R2B4 V3 encoder + IMU + LiDAR critical-path cost without resident runtime."
    )
    parser.add_argument("--repo", type=Path, default=Path("/home/alba/project_r2b4"))
    parser.add_argument(
        "--samples",
        type=int,
        default=300,
        help="Measured ticks per peripheral and combined phase (default: 300).",
    )
    parser.add_argument(
        "--baseline-samples",
        type=int,
        default=100,
        help="Baseline ticks before and after the hardware phases (default: 100).",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=100,
        help="Unmeasured production-order core-L0 warmup ticks (default: 100).",
    )
    parser.add_argument("--observed-l0-ms", type=float, default=None)
    parser.add_argument("--observed-loop-ms", type=float, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if args.samples < 20:
        parser.error("--samples must be at least 20")
    if args.baseline_samples < 20:
        parser.error("--baseline-samples must be at least 20")
    if args.warmup < 1:
        parser.error("--warmup must be at least 1")

    repo = args.repo.expanduser().resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"ERROR: not an R2B4 repo: {repo}")

    runtime_pid = _active_pid(repo / "runtime" / ".r2b4_runtime_pid")
    if runtime_pid is not None:
        raise SystemExit(
            f"ERROR: resident runtime appears active (PID {runtime_pid}). Stop it before this diagnostic."
        )

    sys.path.insert(0, str(repo))

    try:
        import lgpio
        import serial
        import smbus2
        from v3.adapters.bno055_device import NativeBno055Device
        from v3.adapters.live_inputs import NativeLiveInputReader
        from v3.adapters.native_lidar_port import TimedPoseReference
        from v3.composition.native_sensor_inputs import NativeSensorInputOwner
        from v3.contracts import TickContext
        from v3.runtime_performance import apply_process_affinity_layout, load_runtime_affinity_config
        from v3_process_runtime import load_resident_runtime_config, native_lidar_factory
    except Exception as exc:
        raise SystemExit(
            f"ERROR importing production V3 peripheral path: {type(exc).__name__}: {exc}"
        ) from exc

    runtime_config = load_resident_runtime_config(repo)
    sensor_hw = runtime_config.sensor_inputs
    period_ns = int(runtime_config.tick_period_ns)
    target_hz = 1_000_000_000.0 / period_ns
    affinity = load_runtime_affinity_config(repo / "conf" / "vezerles.json")

    affinity_evidence = []
    if affinity.enabled:
        affinity_evidence = [item.as_dict() for item in apply_process_affinity_layout(affinity)]

    print("R2B4 V3 PERIFERIAL CONTROL DIAG")
    print(f"repo:          {repo}")
    print(f"target:        {target_hz:.3f} Hz / {period_ns / 1e6:.3f} ms")
    print(
        f"IMU:           bus={sensor_hw.imu_device.bus_number} "
        f"addr=0x{sensor_hw.imu_device.address:02x} mode={sensor_hw.imu_device.operation_mode}"
    )
    print("ENCODER:       production lgpio owner + counter backend")
    print("LIDAR:         production NativeLidarPort + RPLIDAR + matcher process")
    print(f"affinity:      {'production layout' if affinity.enabled else 'disabled'}")
    print(
        f"phases:        baseline {args.baseline_samples} + encoder {args.samples} + "
        f"imu {args.samples} + lidar {args.samples} + core-L0 {args.samples} + baseline {args.baseline_samples}"
    )
    print("motors:        NOT OPENED")
    print()

    probe = ProbeContext()
    timed_bus: TimedBus | None = None
    timed_lidar: TimedLidarPort | None = None
    owner: Any = None

    try:
        real_bus = smbus2.SMBus(sensor_hw.imu_device.bus_number)
        timed_bus = TimedBus(real_bus, NativeBno055Device, probe)
        imu_device = NativeBno055Device(timed_bus, sensor_hw.imu_device)
        imu_device.initialize()

        # Production LiDAR opener, including configured CPU2 affinity, serial driver,
        # latest-only matcher process and pump thread. The standalone diagnostic has
        # no L3 estimator, so it provides a stationary pose reference at the exact
        # requested scan timestamp. This keeps matcher work alive without inventing
        # robot motion.
        open_lidar = native_lidar_factory(sensor_hw, serial.Serial, repo, affinity)

        def stationary_pose(monotonic_ns: int) -> Any:
            return TimedPoseReference(monotonic_ns, 0.0, 0.0, 0.0)

        real_lidar = open_lidar(stationary_pose)
        timed_lidar = TimedLidarPort(real_lidar, probe)

        # Keep the exact production core configs, but intentionally omit optional
        # camera/person sources because this tool diagnoses only encoder+IMU+LiDAR.
        core_inputs = replace(
            sensor_hw.inputs,
            camera_source=None,
            person_detection_source=None,
        )
        owner = NativeSensorInputOwner(
            lgpio,
            imu_device,
            timed_lidar,
            core_inputs,
        )

        encoder = TimedSource("ENCODER", owner.encoder_source, probe)
        imu = TimedSource("IMU", owner.imu_source, probe)
        lidar = TimedSource("LIDAR", owner.lidar_source, probe)
        sources = (encoder, imu, lidar)
        reader = NativeLiveInputReader(sources)

        # Warm up the exact production source order and source state.
        for source in sources:
            source.set_phase("WARMUP")
        next_deadline = time.monotonic_ns()
        for tick_id in range(args.warmup):
            now_ns = _sleep_until(next_deadline)
            reader.read(TickContext(tick_id, now_ns))
            next_deadline = max(next_deadline + period_ns, now_ns + 1)

        for source in sources:
            source.clear_events()
        timed_bus.clear_events()
        timed_lidar.clear_events()

        phases: dict[str, Any] = {}
        phases["BASELINE_PRE"] = _baseline_phase(
            samples=args.baseline_samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=1_000_000,
        )
        _phase_gap(period_ns)

        phases["ENCODER_ONLY"] = _single_source_phase(
            phase="ENCODER_ONLY",
            source=encoder,
            samples=args.samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=2_000_000,
        )
        _phase_gap(period_ns)

        phases["IMU_ONLY"] = _single_source_phase(
            phase="IMU_ONLY",
            source=imu,
            samples=args.samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=3_000_000,
        )
        _phase_gap(period_ns)

        phases["LIDAR_ONLY"] = _single_source_phase(
            phase="LIDAR_ONLY",
            source=lidar,
            samples=args.samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=4_000_000,
        )
        _phase_gap(period_ns)

        phases["CORE_L0"] = _combined_phase(
            phase="CORE_L0",
            reader=reader,
            sources=sources,
            samples=args.samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=5_000_000,
        )
        _phase_gap(period_ns)

        phases["BASELINE_POST"] = _baseline_phase(
            samples=args.baseline_samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=6_000_000,
        )

        # Attach peripheral-specific lower-level evidence.
        phases["IMU_ONLY"]["i2c_operations"] = _operation_summary(timed_bus.events, "IMU_ONLY")
        phases["IMU_ONLY"]["i2c_sum_per_tick"] = _operation_per_tick(timed_bus.events, "IMU_ONLY")
        phases["CORE_L0"]["imu_i2c_operations"] = _operation_summary(timed_bus.events, "CORE_L0")
        phases["CORE_L0"]["imu_i2c_sum_per_tick"] = _operation_per_tick(timed_bus.events, "CORE_L0")

        phases["LIDAR_ONLY"]["port_operations"] = _operation_summary(timed_lidar.events, "LIDAR_ONLY")
        phases["LIDAR_ONLY"]["port_sum_per_tick"] = _operation_per_tick(timed_lidar.events, "LIDAR_ONLY")
        phases["CORE_L0"]["lidar_port_operations"] = _operation_summary(timed_lidar.events, "CORE_L0")
        phases["CORE_L0"]["lidar_port_sum_per_tick"] = _operation_per_tick(timed_lidar.events, "CORE_L0")

        activity = {
            "ENCODER_ONLY": _phase_activity(encoder, "ENCODER_ONLY"),
            "LIDAR_ONLY": _phase_activity(lidar, "LIDAR_ONLY"),
            "CORE_L0_ENCODER": _phase_activity(encoder, "CORE_L0"),
            "CORE_L0_LIDAR": _phase_activity(lidar, "CORE_L0"),
        }

        # Verify the IMU production transaction contract for each measured IMU tick.
        imu_ops = phases["IMU_ONLY"]["i2c_operations"]
        completed_imu = int(phases["IMU_ONLY"]["samples_completed"])
        expected_imu_ops = {
            "IMU_BURST_32": completed_imu,
            "IMU_CALIB": completed_imu,
            "IMU_SYS_STATUS": completed_imu,
            "IMU_SYS_ERROR": completed_imu,
        }
        actual_imu_ops = {
            name: int(imu_ops.get(name, {}).get("wall", {}).get("count", 0))
            for name in expected_imu_ops
        }
        imu_contract_ok = actual_imu_ops == expected_imu_ops

        verdict = _build_overall_verdict(
            period_ns=period_ns,
            phases=phases,
            observed_l0_ms=args.observed_l0_ms,
            observed_loop_ms=args.observed_loop_ms,
        )

        encoder_motion = activity["CORE_L0_ENCODER"].get("physical_edge_activity_observed", False)
        if not encoder_motion:
            verdict["encoder_activity_limit"] = (
                "No encoder pulse-count change was observed during CORE_L0. Encoder timing therefore proves the stationary read path; "
                "moving-wheel callback contention was not exercised."
            )

        report = {
            "schema": SCHEMA,
            "tool_version": TOOL_VERSION,
            "generated_local": datetime.now().astimezone().isoformat(),
            "repo": str(repo),
            "runtime_was_active": False,
            "motors_opened": False,
            "production_configuration": {
                "tick_period_ns": period_ns,
                "target_hz": target_hz,
                "imu_bus": sensor_hw.imu_device.bus_number,
                "imu_address": sensor_hw.imu_device.address,
                "imu_operation_mode": sensor_hw.imu_device.operation_mode,
                "encoder_device_id": core_inputs.encoder_source.device_id,
                "imu_device_id": core_inputs.imu_source.device_id,
                "lidar_device_id": core_inputs.lidar_source.device_id,
                "affinity": affinity.as_dict(),
                "affinity_evidence": affinity_evidence,
            },
            "method": {
                "core_source_order": ["ENCODER", "IMU", "LIDAR"],
                "combined_reader": "NativeLiveInputReader",
                "all_core_owners_active_during_all_phases": True,
                "scheduler_policy": "next_deadline=max(previous_deadline+period,tick_start+1)",
                "clock_wall": "time.perf_counter_ns",
                "clock_thread_cpu": "time.thread_time_ns",
                "non_cpu_elapsed_definition": "wall-thread_cpu; includes I/O wait and scheduler preemption",
                "lidar_pose_provider": "stationary exact-timestamp TimedPoseReference(0,0,0)",
                "camera_opened": False,
                "person_detector_opened": False,
                "capture_started": False,
                "resident_runtime_started": False,
            },
            "phases": phases,
            "activity": activity,
            "imu_transaction_contract": {
                "expected": expected_imu_ops,
                "actual": actual_imu_ops,
                "pass": imu_contract_ok,
            },
            "verdict": verdict,
        }

        output = args.output
        if output is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output = repo / "runtime" / "diag" / f"periferial_control_diag_{stamp}.json"
        elif not output.is_absolute():
            output = (repo / output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        print("RESULTS")
        print(f"baseline-pre effective:  {phases['BASELINE_PRE']['effective_hz']:.3f} Hz")
        print(f"encoder-only effective:  {phases['ENCODER_ONLY']['effective_hz']:.3f} Hz")
        print(f"imu-only effective:      {phases['IMU_ONLY']['effective_hz']:.3f} Hz")
        print(f"lidar-only effective:    {phases['LIDAR_ONLY']['effective_hz']:.3f} Hz")
        print(f"core-L0 effective:       {phases['CORE_L0']['effective_hz']:.3f} Hz")
        print(f"baseline-post effective: {phases['BASELINE_POST']['effective_hz']:.3f} Hz")
        print()

        _print_stats("ENCODER source.read", phases["ENCODER_ONLY"]["wall"])
        _print_stats("IMU source.read", phases["IMU_ONLY"]["wall"])
        _print_stats("LIDAR source.read", phases["LIDAR_ONLY"]["wall"])
        _print_stats("CORE L0 reader", phases["CORE_L0"]["wall"])
        _print_stats("CORE source-sum", phases["CORE_L0"]["source_sum"])
        _print_stats("CORE reader overhead", phases["CORE_L0"]["reader_aggregation_overhead"])
        print()

        print("CORE L0 SAME-TICK SOURCE BREAKDOWN")
        for label in ("ENCODER", "IMU", "LIDAR"):
            _print_stats(label, phases["CORE_L0"]["sources"][label]["wall"])
        print()

        print("IMU I2C BREAKDOWN")
        for name in ("IMU_BURST_32", "IMU_CALIB", "IMU_SYS_STATUS", "IMU_SYS_ERROR"):
            item = phases["IMU_ONLY"]["i2c_operations"].get(name)
            if item is None:
                print(f"{name:<31} MISSING")
            else:
                _print_stats(name, item["wall"])
        print(f"IMU transaction contract: {'PASS' if imu_contract_ok else 'FAIL'}")
        print()

        print("LIDAR PORT BREAKDOWN")
        for name in ("LIDAR_GET_MATCHER_RESULT", "LIDAR_GET_RUNTIME_STATUS", "LIDAR_GET_RAW_SCAN"):
            item = phases["LIDAR_ONLY"]["port_operations"].get(name)
            if item is None:
                print(f"{name:<31} MISSING")
            else:
                _print_stats(name, item["wall"])
        print()

        core_v = verdict["core_l0"]
        print("VERDICT")
        for label in core_v["source_ranking_by_mean"]:
            ms = core_v["source_mean_contribution_ms"][label]
            fraction = core_v["source_mean_fraction_of_core"][label] * 100.0
            print(f"{label:<8} {ms:8.3f} ms mean  ({fraction:5.1f}% of measured core L0)")
        print(
            f"CORE_L0 mean={core_v['mean_ms']:.3f} ms, p99={core_v['p99_ms']:.3f} ms, "
            f"deadline misses={core_v['deadline_miss_count']}/{core_v['samples']}"
        )
        runtime_cmp = verdict.get("runtime_comparison")
        if isinstance(runtime_cmp, dict) and "observed_l0_mean_ms" in runtime_cmp:
            print(
                f"Measured core peripherals explain {runtime_cmp['measured_core_fraction_of_observed_l0'] * 100.0:.1f}% "
                f"of supplied runtime L0 mean ({runtime_cmp['observed_l0_mean_ms']:.3f} ms)."
            )
            print(
                f"Unexplained runtime L0 mean: {runtime_cmp['unexplained_observed_l0_mean_ms']:.3f} ms "
                f"({runtime_cmp['unexplained_fraction'] * 100.0:.1f}%)."
            )
        if not encoder_motion:
            print("ENCODER NOTE: no pulse-count change observed; stationary encoder path measured.")
        print()
        print(f"Evidence JSON: {output}")

        failures = []
        for name in ("ENCODER_ONLY", "IMU_ONLY", "LIDAR_ONLY", "CORE_L0"):
            if phases[name]["errors"]:
                failures.append(name)
        if failures:
            print(f"ERROR: measurement errors in {', '.join(failures)}; see JSON.", file=sys.stderr)
            return 2
        if not imu_contract_ok:
            print("ERROR: IMU transaction contract mismatch; see JSON.", file=sys.stderr)
            return 3
        return 0

    finally:
        probe.set("CLOSE", None)
        if owner is not None:
            try:
                owner.close()
            except Exception as exc:
                print(f"WARNING: peripheral close error: {type(exc).__name__}: {exc}", file=sys.stderr)
        else:
            # If owner construction failed after one lower-level owner opened,
            # close what is still reachable.
            if timed_lidar is not None:
                try:
                    timed_lidar.stop()
                except Exception:
                    pass
            if timed_bus is not None:
                try:
                    timed_bus.close()
                except Exception:
                    pass


if __name__ == "__main__":
    raise SystemExit(main())
