#!/usr/bin/env python3
"""
R2B4 V3 IMU control-loop diagnostic.

Purpose
-------
Measure the *actual production V3 IMU path* without starting the robot runtime:

    smbus2.SMBus
      -> NativeBno055Device
      -> NativeBno055ImuBackend
      -> NativeImuSource.read(TickContext)

The tool also reproduces the resident runtime's nominal tick schedule and CPU
affinity policy, runs baseline/IMU/baseline A-B phases, and instruments every
BNO055 I2C read.

It does NOT open motors, LiDAR, camera, encoders, command interfaces, L0-L12,
or the resident runtime.

Evidence limits
---------------
A standalone test can prove how much time the production IMU path itself costs,
whether it directly exceeds a 20 ms / 50 Hz budget, and whether the delay is
mainly CPU time or non-CPU elapsed time (I/O wait + scheduler effects).

It cannot prove interaction-only effects that occur exclusively while the full
runtime is active. The JSON report states this limitation explicitly.
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
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable


SCHEMA = "R2B4_V3_IMU_CONTROL_DIAG_V1"


def _percentile(values: list[int], q: float) -> int:
    if not values:
        return 0
    if not 0.0 < q <= 1.0:
        raise ValueError("q must be in (0, 1]")
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


def _rate(count: int, total: int) -> float:
    return 0.0 if total <= 0 else count / total


def _active_pid(pid_path: Path) -> int | None:
    try:
        raw = pid_path.read_text(encoding="ascii").strip()
        pid = int(raw)
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


@dataclass(slots=True)
class BusEvent:
    tick_id: int | None
    operation: str
    register: int
    length: int
    wall_ns: int
    thread_cpu_ns: int


class TimedBus:
    """Transparent timing proxy around the real smbus2.SMBus object."""

    def __init__(self, bus: Any, device_cls: type) -> None:
        self._bus = bus
        self._device_cls = device_cls
        self._tick_id: int | None = None
        self.events: list[BusEvent] = []

    def set_tick(self, tick_id: int | None) -> None:
        self._tick_id = tick_id

    def clear_events(self) -> None:
        self.events.clear()

    def _measure(self, operation: str, register: int, length: int, call: Callable[[], Any]) -> Any:
        cpu0 = time.thread_time_ns()
        wall0 = time.perf_counter_ns()
        try:
            return call()
        finally:
            wall1 = time.perf_counter_ns()
            cpu1 = time.thread_time_ns()
            self.events.append(
                BusEvent(
                    tick_id=self._tick_id,
                    operation=operation,
                    register=int(register),
                    length=int(length),
                    wall_ns=max(0, wall1 - wall0),
                    thread_cpu_ns=max(0, cpu1 - cpu0),
                )
            )

    def read_i2c_block_data(self, address: int, register: int, length: int) -> Any:
        op = (
            "IMU_BURST_32"
            if register == self._device_cls.REG_GYRO_DATA_X_LSB and length == 32
            else "I2C_BLOCK_OTHER"
        )
        return self._measure(
            op,
            register,
            length,
            lambda: self._bus.read_i2c_block_data(address, register, length),
        )

    def read_byte_data(self, address: int, register: int) -> Any:
        labels = {
            self._device_cls.REG_CALIB_STAT: "IMU_CALIB",
            self._device_cls.REG_SYS_STATUS: "IMU_SYS_STATUS",
            self._device_cls.REG_SYS_ERR: "IMU_SYS_ERROR",
            self._device_cls.REG_CHIP_ID: "IMU_CHIP_ID",
        }
        op = labels.get(register, "I2C_BYTE_OTHER")
        return self._measure(
            op,
            register,
            1,
            lambda: self._bus.read_byte_data(address, register),
        )

    def write_byte_data(self, address: int, register: int, value: int) -> Any:
        return self._bus.write_byte_data(address, register, value)

    def close(self) -> Any:
        return self._bus.close()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._bus, name)


def _sleep_until(deadline_ns: int) -> int:
    """Use the same sleep-until-deadline shape as the resident loop."""
    now_ns = time.monotonic_ns()
    while now_ns < deadline_ns:
        time.sleep((deadline_ns - now_ns) / 1_000_000_000.0)
        now_ns = time.monotonic_ns()
    return now_ns


def _run_baseline_phase(*, samples: int, period_ns: int, TickContext: type, start_id: int) -> dict[str, Any]:
    tick_starts: list[int] = []
    wake_lateness: list[int] = []
    work: list[int] = []

    next_deadline_ns = time.monotonic_ns()
    for offset in range(samples):
        now_ns = _sleep_until(next_deadline_ns)
        wake_lateness.append(max(0, now_ns - next_deadline_ns))
        tick_starts.append(now_ns)

        p0 = time.perf_counter_ns()
        # Reproduce the immutable tick-context closure that the production loop
        # performs before it enters the live composition.
        _ = TickContext(start_id + offset, now_ns)
        p1 = time.perf_counter_ns()
        work.append(max(0, p1 - p0))

        # Exact resident scheduling policy:
        # next_deadline = max(previous_deadline + period, tick_start + 1)
        next_deadline_ns = max(next_deadline_ns + period_ns, now_ns + 1)

    periods = [b - a for a, b in zip(tick_starts, tick_starts[1:])]
    elapsed = tick_starts[-1] - tick_starts[0] if len(tick_starts) > 1 else 0
    effective_hz = ((len(tick_starts) - 1) * 1_000_000_000.0 / elapsed) if elapsed > 0 else 0.0
    return {
        "samples": samples,
        "effective_hz": effective_hz,
        "period": _stats_ns(periods),
        "wake_lateness": _stats_ns(wake_lateness),
        "context_work": _stats_ns(work),
    }


def _run_imu_phase(
    *,
    samples: int,
    period_ns: int,
    source: Any,
    bus: TimedBus,
    TickContext: type,
    start_id: int,
) -> dict[str, Any]:
    tick_starts: list[int] = []
    wake_lateness: list[int] = []
    source_wall: list[int] = []
    source_cpu: list[int] = []
    source_non_cpu: list[int] = []
    health = Counter()
    sample_kinds = Counter()
    errors: list[dict[str, Any]] = []

    next_deadline_ns = time.monotonic_ns()
    for offset in range(samples):
        tick_id = start_id + offset
        now_ns = _sleep_until(next_deadline_ns)
        tick_starts.append(now_ns)
        wake_lateness.append(max(0, now_ns - next_deadline_ns))
        context = TickContext(tick_id, now_ns)

        bus.set_tick(tick_id)
        cpu0 = time.thread_time_ns()
        wall0 = time.perf_counter_ns()
        try:
            snapshot = source.read(context)
        except BaseException as exc:
            wall1 = time.perf_counter_ns()
            cpu1 = time.thread_time_ns()
            source_wall.append(max(0, wall1 - wall0))
            source_cpu.append(max(0, cpu1 - cpu0))
            source_non_cpu.append(max(0, (wall1 - wall0) - (cpu1 - cpu0)))
            errors.append(
                {
                    "tick_id": tick_id,
                    "type": type(exc).__name__,
                    "message": str(exc),
                }
            )
            break
        wall1 = time.perf_counter_ns()
        cpu1 = time.thread_time_ns()

        wall_ns = max(0, wall1 - wall0)
        cpu_ns = max(0, cpu1 - cpu0)
        source_wall.append(wall_ns)
        source_cpu.append(cpu_ns)
        # This is deliberately called non-CPU elapsed, not pure I/O time:
        # scheduler preemption can also contribute.
        source_non_cpu.append(max(0, wall_ns - cpu_ns))

        state = getattr(getattr(snapshot, "health", None), "state", None)
        health[str(getattr(state, "value", state))] += 1
        for sample in getattr(snapshot, "samples", ()):
            sample_kinds[str(getattr(sample, "kind", "UNKNOWN"))] += 1

        next_deadline_ns = max(next_deadline_ns + period_ns, now_ns + 1)

    bus.set_tick(None)

    periods = [b - a for a, b in zip(tick_starts, tick_starts[1:])]
    elapsed = tick_starts[-1] - tick_starts[0] if len(tick_starts) > 1 else 0
    effective_hz = ((len(tick_starts) - 1) * 1_000_000_000.0 / elapsed) if elapsed > 0 else 0.0

    per_tick_bus_wall: dict[int, int] = defaultdict(int)
    per_tick_bus_cpu: dict[int, int] = defaultdict(int)
    by_operation_wall: dict[str, list[int]] = defaultdict(list)
    by_operation_cpu: dict[str, list[int]] = defaultdict(list)
    op_counts = Counter()
    for event in bus.events:
        if event.tick_id is None:
            continue
        per_tick_bus_wall[event.tick_id] += event.wall_ns
        per_tick_bus_cpu[event.tick_id] += event.thread_cpu_ns
        by_operation_wall[event.operation].append(event.wall_ns)
        by_operation_cpu[event.operation].append(event.thread_cpu_ns)
        op_counts[event.operation] += 1

    completed_ids = [start_id + i for i in range(len(source_wall))]
    bus_wall = [per_tick_bus_wall[tick_id] for tick_id in completed_ids]
    bus_cpu = [per_tick_bus_cpu[tick_id] for tick_id in completed_ids]
    adapter_non_bus = [
        max(0, source_wall[i] - bus_wall[i])
        for i in range(min(len(source_wall), len(bus_wall)))
    ]

    operation_stats = {}
    for name in sorted(by_operation_wall):
        operation_stats[name] = {
            "count": op_counts[name],
            "wall": _stats_ns(by_operation_wall[name]),
            "thread_cpu": _stats_ns(by_operation_cpu[name]),
        }

    return {
        "samples_requested": samples,
        "samples_completed": len(source_wall),
        "effective_hz": effective_hz,
        "period": _stats_ns(periods),
        "wake_lateness": _stats_ns(wake_lateness),
        "source_read_wall": _stats_ns(source_wall),
        "source_read_thread_cpu": _stats_ns(source_cpu),
        "source_read_non_cpu_elapsed": _stats_ns(source_non_cpu),
        "bus_wall_per_tick": _stats_ns(bus_wall),
        "bus_thread_cpu_per_tick": _stats_ns(bus_cpu),
        "adapter_non_bus_wall_per_tick": _stats_ns(adapter_non_bus),
        "over_5ms_count": sum(v > 5_000_000 for v in source_wall),
        "over_10ms_count": sum(v > 10_000_000 for v in source_wall),
        "over_20ms_count": sum(v > period_ns for v in source_wall),
        "health_states": dict(health),
        "sample_kinds": dict(sample_kinds),
        "i2c_operation_counts": dict(op_counts),
        "i2c_operations": operation_stats,
        "errors": errors,
    }


def _build_verdict(
    *,
    target_period_ns: int,
    baseline_pre: dict[str, Any],
    baseline_post: dict[str, Any],
    imu: dict[str, Any],
    observed_l0_ms: float | None,
    observed_loop_ms: float | None,
) -> dict[str, Any]:
    samples = int(imu["samples_completed"])
    read = imu["source_read_wall"]
    over_budget = int(imu["over_20ms_count"])
    target_hz = 1_000_000_000.0 / target_period_ns
    baseline_hz = statistics.mean(
        [float(baseline_pre["effective_hz"]), float(baseline_post["effective_hz"])]
    )
    imu_hz = float(imu["effective_hz"])
    mean_ns = int(read["mean_ns"])
    p95_ns = int(read["p95_ns"])
    p99_ns = int(read["p99_ns"])

    findings: list[dict[str, Any]] = []

    findings.append(
        {
            "status": "PROVEN",
            "claim": "V3_IMU_DIRECT_COST",
            "statement": (
                f"The exact production V3 IMU source path consumed "
                f"{read['mean_ms']:.3f} ms mean, {read['p95_ms']:.3f} ms p95, "
                f"{read['p99_ms']:.3f} ms p99 per read in this standalone test."
            ),
        }
    )

    if samples and over_budget:
        findings.append(
            {
                "status": "PROVEN",
                "claim": "IMU_CAUSES_50HZ_DEADLINE_MISSES",
                "statement": (
                    f"{over_budget}/{samples} IMU reads exceeded the full "
                    f"{target_period_ns / 1e6:.3f} ms control-period budget."
                ),
            }
        )
    else:
        findings.append(
            {
                "status": "PROVEN",
                "claim": "IMU_READS_STAY_BELOW_FULL_50HZ_BUDGET",
                "statement": (
                    f"No measured IMU read exceeded the full "
                    f"{target_period_ns / 1e6:.3f} ms control-period budget."
                ),
            }
        )

    if mean_ns >= target_period_ns:
        primary = "PROVEN_IMU_ALONE_MAKES_50HZ_UNSUSTAINABLE"
        reason = "Mean IMU read time alone is at or above the entire 50 Hz period."
    elif p95_ns >= target_period_ns:
        primary = "PROVEN_IMU_CAUSES_RECURRENT_50HZ_OVERRUNS"
        reason = "At least the p95 IMU read reaches/exceeds the entire 50 Hz period."
    elif (
        samples > 0
        and over_budget == 0
        and imu_hz >= target_hz * 0.98
        and baseline_hz >= target_hz * 0.98
    ):
        primary = "PROVEN_IMU_DOES_NOT_BY_ITSELF_BREAK_50HZ_IN_THIS_TEST"
        reason = (
            "The production IMU path stayed inside the 20 ms work budget and the "
            "V3-shaped IMU loop maintained at least 98% of nominal 50 Hz."
        )
    else:
        primary = "IMU_CONTRIBUTION_PROVEN_ROOT_CAUSE_NOT_ISOLATED"
        reason = (
            "The IMU cost is measured, but baseline scheduler jitter or intermediate "
            "timing prevents a stronger standalone root-cause conclusion."
        )

    comparison: dict[str, Any] = {
        "target_hz": target_hz,
        "target_period_ms": target_period_ns / 1_000_000.0,
        "baseline_effective_hz_mean": baseline_hz,
        "imu_effective_hz": imu_hz,
        "imu_mean_budget_fraction": mean_ns / target_period_ns,
        "imu_p95_budget_fraction": p95_ns / target_period_ns,
        "imu_p99_budget_fraction": p99_ns / target_period_ns,
    }
    if observed_l0_ms is not None and observed_l0_ms > 0:
        comparison["observed_runtime_l0_mean_ms"] = observed_l0_ms
        comparison["imu_mean_fraction_of_observed_l0"] = (
            (mean_ns / 1_000_000.0) / observed_l0_ms
        )
    if observed_loop_ms is not None and observed_loop_ms > 0:
        comparison["observed_runtime_loop_mean_ms"] = observed_loop_ms
        comparison["imu_mean_fraction_of_observed_loop"] = (
            (mean_ns / 1_000_000.0) / observed_loop_ms
        )

    return {
        "primary": primary,
        "reason": reason,
        "findings": findings,
        "comparison": comparison,
        "interpretation_limit": (
            "This is direct causal evidence for the standalone production IMU path. "
            "It does not measure interaction-only delays that require the full runtime "
            "(for example contention from other processes/threads)."
        ),
    }


def _print_stats(label: str, stats: dict[str, Any]) -> None:
    print(
        f"{label:<28} "
        f"mean={stats['mean_ms']:8.3f} ms  "
        f"p50={stats['p50_ms']:8.3f}  "
        f"p95={stats['p95_ms']:8.3f}  "
        f"p99={stats['p99_ms']:8.3f}  "
        f"max={stats['max_ms']:8.3f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure the production R2B4 V3 BNO055 path without starting runtime."
    )
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path("/home/alba/project_r2b4"),
        help="R2B4 repository root.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=500,
        help="Number of measured IMU ticks (default: 500 = about 10 s at 50 Hz).",
    )
    parser.add_argument(
        "--baseline-samples",
        type=int,
        default=100,
        help="Baseline ticks before and after IMU phase (default: 100 each).",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=25,
        help="Unmeasured V3 IMU warmup reads (default: 25).",
    )
    parser.add_argument(
        "--observed-l0-ms",
        type=float,
        default=None,
        help="Optional runtime L0_READ mean for direct fraction comparison.",
    )
    parser.add_argument(
        "--observed-loop-ms",
        type=float,
        default=None,
        help="Optional runtime loop mean for direct fraction comparison.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="JSON evidence path. Default: runtime/diag/imu_control_diag_<timestamp>.json",
    )
    args = parser.parse_args()

    if args.samples < 20:
        parser.error("--samples must be at least 20")
    if args.baseline_samples < 20:
        parser.error("--baseline-samples must be at least 20")
    if args.warmup < 0:
        parser.error("--warmup must be non-negative")

    repo = args.repo.expanduser().resolve()
    if not (repo / "v3").is_dir():
        raise SystemExit(f"ERROR: not an R2B4 repo: {repo}")

    active_pid = _active_pid(repo / "runtime" / ".r2b4_runtime_pid")
    if active_pid is not None:
        raise SystemExit(
            f"ERROR: resident runtime appears active (PID {active_pid}). "
            "Stop runtime before this diagnostic so the IMU has one owner."
        )

    sys.path.insert(0, str(repo))

    try:
        import smbus2
        from v3.adapters.bno055_device import NativeBno055Device
        from v3.adapters.bno055_imu import NativeBno055ImuBackend
        from v3.adapters.live_imu import NativeImuSource
        from v3.contracts import TickContext
        from v3.runtime_performance import (
            apply_process_affinity_layout,
            load_runtime_affinity_config,
        )
        from v3_process_runtime import load_resident_runtime_config
    except Exception as exc:
        raise SystemExit(
            f"ERROR importing production V3 IMU path: {type(exc).__name__}: {exc}"
        ) from exc

    # Canonical production closure: same hardver/fizika/speed-map/control config,
    # same native_sensor_policy(), same tick period used by resident runtime.
    runtime_config = load_resident_runtime_config(repo)
    sensor_config = runtime_config.sensor_inputs
    period_ns = int(runtime_config.tick_period_ns)
    affinity = load_runtime_affinity_config(repo / "conf" / "vezerles.json")

    affinity_evidence = []
    if affinity.enabled:
        affinity_evidence = [
            item.as_dict() for item in apply_process_affinity_layout(affinity)
        ]

    print("R2B4 V3 IMU CONTROL DIAG")
    print(f"repo:          {repo}")
    print(f"target:        {1_000_000_000.0 / period_ns:.3f} Hz / {period_ns / 1e6:.3f} ms")
    print(
        f"IMU:           bus={sensor_config.imu_device.bus_number} "
        f"addr=0x{sensor_config.imu_device.address:02x} "
        f"mode={sensor_config.imu_device.operation_mode}"
    )
    if affinity.enabled:
        print(f"affinity:      production layout, main task CPU{affinity.runtime_cpu}")
    else:
        print("affinity:      disabled by production config")
    print(f"samples:       baseline {args.baseline_samples} + IMU {args.samples} + baseline {args.baseline_samples}")
    print()

    real_bus = smbus2.SMBus(sensor_config.imu_device.bus_number)
    timed_bus = TimedBus(real_bus, NativeBno055Device)
    device = NativeBno055Device(timed_bus, sensor_config.imu_device)

    try:
        device.initialize()
        backend = NativeBno055ImuBackend(device, sensor_config.inputs.imu_backend)
        source = NativeImuSource(backend, sensor_config.inputs.imu_source)

        # Warmup uses the production source chain too, but is deliberately
        # excluded from measured evidence.
        next_deadline_ns = time.monotonic_ns()
        for tick_id in range(args.warmup):
            now_ns = _sleep_until(next_deadline_ns)
            timed_bus.set_tick(None)
            source.read(TickContext(tick_id, now_ns))
            next_deadline_ns = max(next_deadline_ns + period_ns, now_ns + 1)

        timed_bus.clear_events()

        baseline_pre = _run_baseline_phase(
            samples=args.baseline_samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=1_000_000,
        )

        # Give the scheduler one clean period between phases.
        time.sleep(period_ns / 1_000_000_000.0)

        timed_bus.clear_events()
        imu = _run_imu_phase(
            samples=args.samples,
            period_ns=period_ns,
            source=source,
            bus=timed_bus,
            TickContext=TickContext,
            start_id=2_000_000,
        )

        time.sleep(period_ns / 1_000_000_000.0)

        baseline_post = _run_baseline_phase(
            samples=args.baseline_samples,
            period_ns=period_ns,
            TickContext=TickContext,
            start_id=3_000_000,
        )

        verdict = _build_verdict(
            target_period_ns=period_ns,
            baseline_pre=baseline_pre,
            baseline_post=baseline_post,
            imu=imu,
            observed_l0_ms=args.observed_l0_ms,
            observed_loop_ms=args.observed_loop_ms,
        )

        expected_each = int(imu["samples_completed"])
        counts = imu["i2c_operation_counts"]
        expected_ops = {
            "IMU_BURST_32": expected_each,
            "IMU_CALIB": expected_each,
            "IMU_SYS_STATUS": expected_each,
            "IMU_SYS_ERROR": expected_each,
        }
        transaction_contract_ok = all(counts.get(k, 0) == v for k, v in expected_ops.items())

        report = {
            "schema": SCHEMA,
            "generated_local": datetime.now().astimezone().isoformat(),
            "repo": str(repo),
            "runtime_was_active": False,
            "production_configuration": {
                "tick_period_ns": period_ns,
                "target_hz": 1_000_000_000.0 / period_ns,
                "imu_bus": sensor_config.imu_device.bus_number,
                "imu_address": sensor_config.imu_device.address,
                "imu_operation_mode": sensor_config.imu_device.operation_mode,
                "imu_use_external_crystal": sensor_config.imu_device.use_external_crystal,
                "imu_maximum_sample_age_ns": sensor_config.inputs.imu_backend.maximum_sample_age_ns,
                "imu_minimum_confidence": sensor_config.inputs.imu_source.minimum_confidence,
                "imu_minimum_calibration": sensor_config.inputs.imu_source.minimum_calibration,
                "imu_allow_rate_only": sensor_config.inputs.imu_source.allow_rate_only,
                "affinity": affinity.as_dict(),
                "affinity_evidence": affinity_evidence,
            },
            "method": {
                "production_path": [
                    "smbus2.SMBus",
                    "NativeBno055Device",
                    "NativeBno055ImuBackend",
                    "NativeImuSource",
                    "TickContext",
                ],
                "scheduler_policy": "next_deadline=max(previous_deadline+period,tick_start+1)",
                "baseline_design": "baseline -> IMU -> baseline",
                "clock_wall": "time.perf_counter_ns",
                "clock_thread_cpu": "time.thread_time_ns",
                "non_cpu_elapsed_definition": "wall - thread_cpu; includes I/O wait and scheduler preemption",
                "motors_opened": False,
                "lidar_opened": False,
                "camera_opened": False,
                "resident_runtime_started": False,
            },
            "baseline_pre": baseline_pre,
            "imu": imu,
            "baseline_post": baseline_post,
            "i2c_transaction_contract": {
                "expected_per_completed_tick": [
                    "1x 32-byte fused burst",
                    "1x calibration byte",
                    "1x system-status byte",
                    "1x system-error byte",
                ],
                "expected_counts": expected_ops,
                "actual_counts": counts,
                "pass": transaction_contract_ok,
            },
            "verdict": verdict,
        }

        output = args.output
        if output is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output = repo / "runtime" / "diag" / f"imu_control_diag_{stamp}.json"
        elif not output.is_absolute():
            output = (repo / output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        print("RESULTS")
        print(f"baseline-pre effective:  {baseline_pre['effective_hz']:.3f} Hz")
        print(f"IMU effective:           {imu['effective_hz']:.3f} Hz")
        print(f"baseline-post effective: {baseline_post['effective_hz']:.3f} Hz")
        print()
        _print_stats("V3 IMU source.read", imu["source_read_wall"])
        _print_stats("thread CPU inside read", imu["source_read_thread_cpu"])
        _print_stats("non-CPU elapsed", imu["source_read_non_cpu_elapsed"])
        _print_stats("sum I2C calls / tick", imu["bus_wall_per_tick"])
        _print_stats("adapter non-bus / tick", imu["adapter_non_bus_wall_per_tick"])
        print()

        print("I2C BREAKDOWN")
        for op in ("IMU_BURST_32", "IMU_CALIB", "IMU_SYS_STATUS", "IMU_SYS_ERROR"):
            item = imu["i2c_operations"].get(op)
            if item is None:
                print(f"{op:<28} MISSING")
            else:
                _print_stats(op, item["wall"])
        print()

        completed = int(imu["samples_completed"])
        print(
            f">5 ms:  {imu['over_5ms_count']}/{completed}  "
            f">10 ms: {imu['over_10ms_count']}/{completed}  "
            f">20 ms: {imu['over_20ms_count']}/{completed}"
        )
        print(f"I2C transaction contract: {'PASS' if transaction_contract_ok else 'FAIL'}")
        print()
        print(f"VERDICT: {verdict['primary']}")
        print(verdict["reason"])
        comparison = verdict["comparison"]
        print(
            f"IMU mean consumes {comparison['imu_mean_budget_fraction'] * 100.0:.1f}% "
            f"of the {comparison['target_period_ms']:.1f} ms control budget."
        )
        if "imu_mean_fraction_of_observed_l0" in comparison:
            print(
                f"IMU mean is {comparison['imu_mean_fraction_of_observed_l0'] * 100.0:.1f}% "
                f"of supplied observed L0 mean ({comparison['observed_runtime_l0_mean_ms']:.3f} ms)."
            )
        if "imu_mean_fraction_of_observed_loop" in comparison:
            print(
                f"IMU mean is {comparison['imu_mean_fraction_of_observed_loop'] * 100.0:.1f}% "
                f"of supplied observed runtime-loop mean ({comparison['observed_runtime_loop_mean_ms']:.3f} ms)."
            )
        print()
        print(f"Evidence JSON: {output}")

        if imu["errors"]:
            print("ERROR: IMU measurement phase contained errors; see JSON.", file=sys.stderr)
            return 2
        if not transaction_contract_ok:
            print("ERROR: production I2C transaction contract did not match expectation.", file=sys.stderr)
            return 3
        return 0
    finally:
        try:
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
