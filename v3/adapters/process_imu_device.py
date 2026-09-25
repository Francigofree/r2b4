"""BNO055 acquisition with its own interpreter and bounded shared sample history.

Only the child owns the I2C bus. The parent reads completed physical samples;
it never retimestamps a cached sample or waits for I2C/IPC completion.
"""

from __future__ import annotations

import multiprocessing
import time
from collections.abc import Callable, Mapping

from v3.adapters.bno055_device import NativeBno055Device, NativeBno055DeviceConfig
from v3.runtime_performance import apply_current_affinity, temporary_current_affinity
from v3.config_types import ImuProcessConfig
from v3.async_capability import TransportSemantics


_VALUE_COUNT = 11


def _acquire_imu(config, open_bus, lock, sequence, times, values,
                 ready, stop, failed, worker_cpu, strict_affinity, process_config) -> None:
    device = None
    try:
        if worker_cpu is not None:
            apply_current_affinity(worker_cpu, role="imu-acquire", strict=strict_affinity)
        device = NativeBno055Device(open_bus(config.bus_number), config)
        device.initialize()
        deadline = time.monotonic_ns()
        while not stop.is_set():
            acquired_ns = time.monotonic_ns()
            sample = device.read_sample_at(acquired_ns)
            calibration = sample["calibration"]
            fields = (
                sample["heading_deg"], *sample["gyro_dps"],
                *(calibration[key] for key in ("sys", "gyro", "accel", "mag")),
                sample["sys_status"], sample["sys_error"], int(device.sensor_ok),
            )
            with lock:
                revision = int(sequence.value)
                slot = revision % process_config.history_size
                for offset, value in enumerate(fields):
                    values[slot * _VALUE_COUNT + offset] = value
                times[slot * 2] = acquired_ns
                times[slot * 2 + 1] = time.monotonic_ns()
                sequence.value = revision + 1
            ready.set()
            deadline += process_config.sample_period_ns
            now = time.monotonic_ns()
            if deadline <= now:
                deadline += ((now - deadline) // process_config.sample_period_ns + 1) * process_config.sample_period_ns
            stop.wait(max(0, deadline - now) / 1e9)
    except BaseException:
        failed.set()
        ready.set()
    finally:
        if device is not None:
            device.close()


class ProcessBno055Device:
    """Sample-port-compatible proxy; no motor or production layer authority."""

    transport_semantics = TransportSemantics.LATEST_STATE

    def __init__(self, config: NativeBno055DeviceConfig, *, open_bus: Callable,
                 worker_cpu: int | None = None, strict_affinity: bool = False,
                 process_config: ImuProcessConfig) -> None:
        if not isinstance(config, NativeBno055DeviceConfig):
            raise TypeError("config must be NativeBno055DeviceConfig")
        if not callable(open_bus):
            raise TypeError("open_bus must be callable")
        if not isinstance(process_config, ImuProcessConfig):
            raise TypeError("process_config must be ImuProcessConfig")
        self._process_config = process_config
        context = multiprocessing.get_context("spawn")
        self._lock = context.Lock()
        self._sequence = context.RawValue("Q", 0)
        self._times = context.RawArray("q", self._process_config.history_size * 2)
        self._values = context.RawArray("d", self._process_config.history_size * _VALUE_COUNT)
        self._ready = context.Event()
        self._stop = context.Event()
        self._failed = context.Event()
        self._cached: Mapping[str, object] | None = None
        self._closed = False
        self.initialized = False
        self.sensor_ok = False
        self._process = context.Process(
            target=_acquire_imu,
            args=(config, open_bus, self._lock, self._sequence, self._times,
                  self._values, self._ready, self._stop, self._failed,
                  worker_cpu, strict_affinity, process_config),
            name="v3-imu-owner", daemon=False,
        )
        with temporary_current_affinity(worker_cpu, role="imu-start", strict=strict_affinity):
            self._process.start()
        if not self._ready.wait(process_config.ready_timeout_s) or self._failed.is_set() or not self._process.is_alive():
            self.close()
            raise RuntimeError("BNO055 acquisition process failed during startup")
        self.initialized = True
        self.read_sample(force=True)

    def read_sample_at(self, captured_monotonic_ns: int) -> Mapping[str, object]:
        if self._closed or self._failed.is_set() or not self._process.is_alive():
            self.sensor_ok = False
            raise RuntimeError("BNO055 acquisition process unavailable")
        # Never wait for a preempted publisher. A cached physical sample ages
        # normally and the existing source/admission/estimator gates still apply.
        if self._lock.acquire(False):
            try:
                latest = int(self._sequence.value)
                for revision in range(latest - 1, max(-1, latest - self._process_config.history_size - 1), -1):
                    slot = revision % self._process_config.history_size
                    if self._times[slot * 2 + 1] > captured_monotonic_ns:
                        continue
                    data = tuple(self._values[slot * _VALUE_COUNT + i] for i in range(_VALUE_COUNT))
                    self._cached = {
                        "sequence": revision,
                        "timestamp": self._times[slot * 2] / 1e9,
                        "heading_deg": data[0], "gyro_dps": data[1:4],
                        "calibration": dict(zip(("sys", "gyro", "accel", "mag"), map(int, data[4:8]))),
                        "sys_status": int(data[8]), "sys_error": int(data[9]),
                    }
                    self.sensor_ok = bool(data[10])
                    break
            finally:
                self._lock.release()
        if self._cached is None or self._cached["timestamp"] * 1e9 > captured_monotonic_ns:
            raise RuntimeError("no BNO055 sample visible at acquisition time")
        return self._cached

    def read_sample(self, *, force: bool = False) -> Mapping[str, object]:
        return self.read_sample_at(time.monotonic_ns())

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.initialized = self.sensor_ok = False
        self._stop.set()
        self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            raise RuntimeError("BNO055 acquisition process did not stop")
