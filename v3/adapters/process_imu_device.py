"""BNO055 acquisition with its own interpreter and bounded shared sample history.

Only the child owns the I2C bus. The parent reads completed physical samples;
it never retimestamps a cached sample or waits for I2C/IPC completion.
"""

from __future__ import annotations

import errno
import json
import multiprocessing
import sys
import threading
from types import SimpleNamespace
import time
from collections.abc import Callable, Mapping

from v3.adapters.bno055_device import NativeBno055Device, NativeBno055DeviceConfig
from v3.runtime_performance import CpuSet, normalize_cpus, apply_process_cpuset, temporary_current_affinity
from v3.config_types import ImuProcessConfig
from v3.async_capability import SensorUnavailable, TransportSemantics


_VALUE_COUNT = 11
_ERROR_BYTES = 256
_MAX_RESTARTS = 3
_RESTART_BACKOFF_S = 0.25


def _acquire_imu(config, open_bus, lock, sequence, times, values,
                 ready, stop, failed, worker_cpus, strict_affinity, process_config,
                 error_bytes, recoverable=None) -> None:
    device = None
    retry_reported = False
    try:
        if worker_cpus is not None:
            apply_process_cpuset(worker_cpus, role="imu-acquire", strict=strict_affinity)
        device = NativeBno055Device(open_bus(config.bus_number), config)
        device.initialize()
        deadline = time.monotonic_ns()
        while not stop.value:
            acquired_ns = time.monotonic_ns()
            try:
                sample = device.read_sample_at(acquired_ns)
            except OSError as exc:
                if exc.errno not in (errno.EIO, errno.EREMOTEIO, errno.ETIMEDOUT) or stop.value:
                    raise
                # One fresh physical transaction can recover a transient bus
                # error. Never republish/retimestamp the failed or cached read.
                acquired_ns = time.monotonic_ns()
                sample = device.read_sample_at(acquired_ns)
                if not retry_reported:
                    # One bounded passive edge log per owner, outside control.
                    print(json.dumps({"event": "IMU_I2C_READ_RECOVERED", "errno": exc.errno,
                                      "error": str(exc)[:160], "measurement_ns": acquired_ns}),
                          file=sys.stderr, flush=True)
                    retry_reported = True
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
            ready.value = True
            deadline += process_config.sample_period_ns
            now = time.monotonic_ns()
            if deadline <= now:
                deadline += ((now - deadline) // process_config.sample_period_ns + 1) * process_config.sample_period_ns
            time.sleep(max(0, deadline - now) / 1e9)
    except BaseException as exc:
        detail = f"{type(exc).__name__}:{exc}".encode("utf-8", errors="replace")[:_ERROR_BYTES - 1]
        error_bytes[:len(detail)] = detail
        error_bytes[len(detail)] = 0
        if recoverable is not None:
            recoverable.value = isinstance(exc, OSError) and exc.errno in (errno.EIO, errno.EREMOTEIO, errno.ETIMEDOUT)
        failed.value = True
        ready.value = True
    finally:
        if device is not None:
            device.close()


class ProcessBno055Device:
    """Sample-port-compatible proxy; no motor or production layer authority."""

    transport_semantics = TransportSemantics.LATEST_STATE

    def __init__(self, config: NativeBno055DeviceConfig, *, open_bus: Callable,
                 worker_cpus: int | CpuSet | None = None, strict_affinity: bool = False,
                 process_config: ImuProcessConfig) -> None:
        if not isinstance(config, NativeBno055DeviceConfig):
            raise TypeError("config must be NativeBno055DeviceConfig")
        if not callable(open_bus):
            raise TypeError("open_bus must be callable")
        if not isinstance(process_config, ImuProcessConfig):
            raise TypeError("process_config must be ImuProcessConfig")
        if worker_cpus is not None:
            worker_cpus = normalize_cpus(worker_cpus)
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        self._process_config = process_config
        self._config, self._open_bus = config, open_bus
        self._worker_cpus, self._strict_affinity = worker_cpus, strict_affinity
        self._cached: Mapping[str, object] | None = None
        self._closed = False
        self._recovering = False
        self._terminal_error = ""
        self._restarts = 0
        self._sequence_base = 0
        self._access_lock = threading.Lock()
        self._supervisor_stop = threading.Event()
        self.initialized = self.sensor_ok = False
        self._start_owner()
        try:
            self._await_ready()
            self.initialized = True
            self.read_sample(force=True)
        except BaseException:
            self.close()
            raise
        self._supervisor = threading.Thread(target=self._supervise, name="imu-recovery", daemon=True)
        with temporary_current_affinity(worker_cpus, role="imu-recovery", strict=strict_affinity):
            self._supervisor.start()

    def _start_owner(self) -> None:
        context = multiprocessing.get_context("spawn")
        state = SimpleNamespace(
            lock=context.Lock(), sequence=context.RawValue("Q", 0),
            times=context.RawArray("q", self._process_config.history_size * 2),
            values=context.RawArray("d", self._process_config.history_size * _VALUE_COUNT),
            ready=context.RawValue("b", False), stop=context.RawValue("b", False),
            failed=context.RawValue("b", False), recoverable=context.RawValue("b", False), error_bytes=context.RawArray("B", _ERROR_BYTES),
            sequence_base=self._sequence_base,
        )
        process = context.Process(target=_acquire_imu,
            args=(self._config, self._open_bus, state.lock, state.sequence, state.times,
                  state.values, state.ready, state.stop, state.failed,
                  self._worker_cpus, self._strict_affinity, self._process_config, state.error_bytes, state.recoverable),
            name="v3-imu-owner", daemon=False)
        state.process = process
        with temporary_current_affinity(self._worker_cpus, role="imu-start", strict=self._strict_affinity):
            process.start()
        self._state = state
        self._process = process

    def _await_ready(self) -> None:
        state = self._state
        deadline = time.monotonic() + self._process_config.ready_timeout_s
        while not state.ready.value and state.process.is_alive() and time.monotonic() < deadline:
            if self._supervisor_stop.wait(.005):
                raise RuntimeError("BNO055 acquisition stopped during startup")
        if not state.ready.value or state.failed.value or not state.process.is_alive():
            raise RuntimeError("BNO055 acquisition process failed during startup:" + self._error_detail(state))

    def _stop_owner(self) -> None:
        state = self._state
        state.stop.value = True
        state.process.join(timeout=self._process_config.stop_timeout_s)
        if state.process.is_alive():
            state.process.terminate()
            state.process.join(timeout=self._process_config.stop_timeout_s)
        if state.process.is_alive():
            state.process.kill()
            state.process.join(timeout=self._process_config.stop_timeout_s)
        if state.process.is_alive():
            raise RuntimeError("BNO055 acquisition process did not stop")

    def _supervise(self) -> None:
        while not self._supervisor_stop.wait(.02):
            state = self._state
            latest = int(state.sequence.value)
            published_ns = state.times[((latest - 1) % self._process_config.history_size) * 2 + 1] if latest else 0
            expired = published_ns > 0 and time.monotonic_ns() - published_ns > int(self._process_config.ready_timeout_s * 1e9)
            if not (state.failed.value or not state.process.is_alive() or expired):
                continue
            with self._access_lock:
                self._recovering = True
                # Allocate a new transport too: a dead child may have held its lock.
                self._sequence_base = state.sequence_base + int(state.sequence.value)
                self._cached = None
            try:
                detail = self._error_detail(state)
                # Malformed samples and configuration errors are not I/O recovery.
                if state.failed.value and not state.recoverable.value:
                    raise RuntimeError(detail)
                while True:
                    self._stop_owner()
                    if self._restarts >= _MAX_RESTARTS:
                        raise RuntimeError("IMU_RECOVERY_EXHAUSTED:" + detail)
                    self._restarts += 1
                    if self._supervisor_stop.wait(_RESTART_BACKOFF_S * self._restarts):
                        return
                    self._start_owner()
                    try:
                        self._await_ready()
                        break
                    except RuntimeError as exc:
                        if self._state.failed.value and not self._state.recoverable.value:
                            raise
                        detail = str(exc)
                self.initialized = True
                self._recovering = False
            except Exception as exc:
                self.initialized = self.sensor_ok = False
                self._terminal_error = str(exc)[:256]
                return

    def read_sample_at(self, captured_monotonic_ns: int) -> Mapping[str, object]:
        if not self._access_lock.acquire(False):
            raise SensorUnavailable("IMU_RESTARTING")
        try:
            state = self._state
            if self._terminal_error or self._closed:
                self.sensor_ok = False
                raise RuntimeError("BNO055 acquisition process unavailable:" + (self._terminal_error or "closed"))
            if state.failed.value and not state.recoverable.value:
                self.sensor_ok = False
                raise RuntimeError("BNO055 acquisition process unavailable:" + self._error_detail(state))
            if self._recovering or state.failed.value or not state.process.is_alive():
                self.sensor_ok = False
                raise SensorUnavailable("IMU_RESTARTING")
            # Never wait for a preempted publisher. A cached physical sample ages
            # normally and the existing source/admission/estimator gates still apply.
            if state.lock.acquire(False):
                try:
                    latest = int(state.sequence.value)
                    for revision in range(latest - 1, max(-1, latest - self._process_config.history_size - 1), -1):
                        slot = revision % self._process_config.history_size
                        if state.times[slot * 2 + 1] > captured_monotonic_ns:
                            continue
                        data = tuple(state.values[slot * _VALUE_COUNT + i] for i in range(_VALUE_COUNT))
                        self._cached = {
                            "sequence": state.sequence_base + revision,
                            "timestamp": state.times[slot * 2] / 1e9,
                            "heading_deg": data[0], "gyro_dps": data[1:4],
                            "calibration": dict(zip(("sys", "gyro", "accel", "mag"), map(int, data[4:8]))),
                            "sys_status": int(data[8]), "sys_error": int(data[9]),
                        }
                        self.sensor_ok = bool(data[10])
                        break
                finally:
                    state.lock.release()
            if state is not self._state or self._recovering:
                raise SensorUnavailable("IMU_RESTARTING")
            if self._cached is None or self._cached["timestamp"] * 1e9 > captured_monotonic_ns:
                raise SensorUnavailable("no BNO055 sample visible at acquisition time")
            return self._cached
        finally:
            self._access_lock.release()

    def read_sample(self, *, force: bool = False) -> Mapping[str, object]:
        return self.read_sample_at(time.monotonic_ns())

    def _error_detail(self, state=None) -> str:
        state = self._state if state is None else state
        if not state.failed.value:
            return "owner closed, silent or exited"
        return bytes(state.error_bytes).split(b"\0", 1)[0].decode("utf-8", errors="replace")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.initialized = self.sensor_ok = False
        self._supervisor_stop.set()
        supervisor = getattr(self, "_supervisor", None)
        if supervisor is not None:
            supervisor.join(timeout=self._process_config.ready_timeout_s + 3 * self._process_config.stop_timeout_s + 1)
            if supervisor.is_alive():
                raise RuntimeError("BNO055 recovery supervisor did not stop")
        self._stop_owner()
