#!/usr/bin/env python3
"""Offline stress harness for the R2B4 multi-rate async input boundary.

This does not drive hardware or motors. It exercises the same
MultiRateLiveInputReader used by production with independent critical streams,
periodic LiDAR blocking, 50 Hz tick closure and passive liveness evidence.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import threading
import time
from dataclasses import dataclass

from v3.adapters.live_inputs import LiveDeviceSnapshot
from v3.adapters.multirate_inputs import (
    MultiRateInputConfig,
    MultiRateLiveInputReader,
    SourcePeriod,
)
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)


@dataclass
class SyntheticSource:
    device_id: str
    nominal_delay_s: float
    blocking_every: int = 0
    blocking_delay_s: float = 0.0

    def __post_init__(self) -> None:
        self._calls = 0
        self._lock = threading.Lock()

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        with self._lock:
            self._calls += 1
            sequence = self._calls
        delay = self.nominal_delay_s
        if self.blocking_every > 0 and sequence % self.blocking_every == 0:
            delay = max(delay, self.blocking_delay_s)
        if delay > 0.0:
            time.sleep(delay)
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(self.device_id, DeviceHealthState.OK),
            (
                DeviceSample(
                    device_id=self.device_id,
                    kind="async_soak_sample",
                    sequence=sequence,
                    captured_monotonic_ns=context.monotonic_ns,
                    values=(DataField("sequence", sequence),),
                ),
            ),
        )


def _percentile_ms(values_ns: list[int], q: float) -> float:
    if not values_ns:
        return 0.0
    ordered = sorted(values_ns)
    index = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * q) - 1))
    return ordered[index] / 1_000_000.0


def run(seconds: float, lidar_block_ms: float, lidar_block_every: int) -> dict[str, object]:
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError("seconds must be finite and positive")
    if not math.isfinite(lidar_block_ms) or lidar_block_ms < 0.0:
        raise ValueError("lidar_block_ms must be finite and non-negative")
    if lidar_block_every <= 0:
        raise ValueError("lidar_block_every must be positive")

    encoder = SyntheticSource("WHEEL_ENCODERS", 0.0004)
    imu = SyntheticSource("BNO055_IMU", 0.0008)
    lidar = SyntheticSource(
        "RPLIDAR_C1",
        0.003,
        blocking_every=lidar_block_every,
        blocking_delay_s=lidar_block_ms / 1000.0,
    )
    reader = MultiRateLiveInputReader(
        (encoder, imu, lidar),
        critical_device_ids=frozenset(
            {"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}
        ),
        config=MultiRateInputConfig(
            critical_default_period_ns=20_000_000,
            auxiliary_default_period_ns=50_000_000,
            history_size=16,
            max_snapshot_age_ns=250_000_000,
            worker_join_timeout_s=2.0,
            source_periods=(
                SourcePeriod("WHEEL_ENCODERS", 20_000_000),
                SourcePeriod("BNO055_IMU", 20_000_000),
                SourcePeriod("RPLIDAR_C1", 20_000_000),
            ),
        ),
        monotonic_ns=time.monotonic_ns,
    )

    tick_durations_ns: list[int] = []
    health_counts: dict[str, int] = {}
    future_sample_faults = 0
    tick_id = 0
    started = time.monotonic()
    next_tick = started
    try:
        while True:
            now = time.monotonic()
            if now - started >= seconds:
                break
            if now < next_tick:
                time.sleep(next_tick - now)
            close_started = time.perf_counter_ns()
            context = reader.begin_tick(tick_id)
            batch = reader.read(context)
            tick_durations_ns.append(time.perf_counter_ns() - close_started)
            for sample in batch.samples:
                if sample.captured_monotonic_ns > context.monotonic_ns:
                    future_sample_faults += 1
            for item in batch.device_health:
                key = f"{item.device_id}:{item.state.value}:{item.reason or '-'}"
                health_counts[key] = health_counts.get(key, 0) + 1
            tick_id += 1
            next_tick += 0.020
            # Do not catch up in a burst if the host is badly delayed.
            late = time.monotonic() - next_tick
            if late > 0.020:
                skipped = int(late // 0.020) + 1
                next_tick += skipped * 0.020
    finally:
        final_now_ns = time.monotonic_ns()
        liveness = reader.liveness_snapshot(final_now_ns)
        reader.close()

    by_id = {item.device_id: item for item in liveness}
    expected_fast = max(1, int(seconds / 0.020 * 0.40))
    fast_streams_ok = all(
        by_id[device_id].publication_count >= expected_fast
        for device_id in ("WHEEL_ENCODERS", "BNO055_IMU")
    )
    lidar_overrun_observed = (
        by_id["RPLIDAR_C1"].deadline_overrun_count > 0
        if lidar_block_ms >= 20.0
        else True
    )
    passed = bool(
        tick_id > 0
        and future_sample_faults == 0
        and fast_streams_ok
        and lidar_overrun_observed
    )

    mean_ms = (
        statistics.fmean(tick_durations_ns) / 1_000_000.0
        if tick_durations_ns
        else 0.0
    )
    return {
        "schema": "R2B4_V3_ASYNC_INPUT_SOAK_V1",
        "status": "PASS" if passed else "FAIL",
        "duration_s": seconds,
        "control_ticks": tick_id,
        "future_sample_faults": future_sample_faults,
        "fast_stream_min_publications": expected_fast,
        "tick_close_timing_ms": {
            "mean": mean_ms,
            "p95": _percentile_ms(tick_durations_ns, 0.95),
            "p99": _percentile_ms(tick_durations_ns, 0.99),
            "max": (max(tick_durations_ns) / 1_000_000.0 if tick_durations_ns else 0.0),
        },
        "health_counts": health_counts,
        "sources": [item.as_dict() for item in liveness],
        "acceptance": {
            "fast_streams_continue_during_lidar_block": fast_streams_ok,
            "no_future_samples": future_sample_faults == 0,
            "lidar_overrun_visible": lidar_overrun_observed,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--lidar-block-ms", type=float, default=60.0)
    parser.add_argument("--lidar-block-every", type=int, default=10)
    args = parser.parse_args()
    report = run(args.seconds, args.lidar_block_ms, args.lidar_block_every)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
