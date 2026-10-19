"""Fresh IMU retry and bounded failure evidence without physical I2C access."""

import errno
import multiprocessing
import re
import time
from types import SimpleNamespace

import pytest

import v3.adapters.process_imu_device as process_imu
from v3.adapters.bno055_device import NativeBno055DeviceConfig
from v3.adapters.live_imu import ImuHeadingReading, NativeImuConfig, NativeImuSource
from v3.async_capability import CapabilityState
from v3.config_types import ImuProcessConfig
from v3.contracts import DeviceHealthState, TickContext


def _process_config():
    return ImuProcessConfig(
        sample_period_ns=20_000_000,
        history_size=4,
        ready_timeout_s=3.0,
        stop_timeout_s=1.0,
    )


def _sample():
    return {
        "heading_deg": 12.5,
        "gyro_dps": (1.0, 2.0, 3.0),
        "calibration": {"sys": 3, "gyro": 3, "accel": 3, "mag": 3},
        "sys_status": 5,
        "sys_error": 0,
    }


def _run_acquisition(monkeypatch, outcomes):
    """Run the production pump with real shared memory and a finite device."""
    context = multiprocessing.get_context("spawn")
    policy = _process_config()
    state = SimpleNamespace(
        lock=context.Lock(),
        sequence=context.RawValue("Q", 0),
        times=context.RawArray("q", policy.history_size * 2),
        values=context.RawArray("d", policy.history_size * process_imu._VALUE_COUNT),
        ready=context.Event(),
        stop=context.Event(),
        failed=context.Event(),
        error_bytes=context.RawArray("B", 256),
        read_times=[],
        closed=False,
    )
    pending = iter(outcomes)

    class Device:
        sensor_ok = True

        def __init__(self, bus, config):
            pass

        def initialize(self):
            pass

        def read_sample_at(self, acquired_ns):
            state.read_times.append(acquired_ns)
            outcome = next(pending)
            if isinstance(outcome, BaseException):
                raise outcome
            state.stop.set()
            return outcome

        def close(self):
            state.closed = True

    ticks = iter(range(1_000_000, 2_000_000, 100))
    monkeypatch.setattr(process_imu, "NativeBno055Device", Device)
    monkeypatch.setattr(process_imu.time, "monotonic_ns", lambda: next(ticks))
    process_imu._acquire_imu(
        NativeBno055DeviceConfig(), lambda bus_number: object(),
        state.lock, state.sequence, state.times, state.values,
        state.ready, state.stop, state.failed, None, False, policy,
        state.error_bytes,
    )
    return state


@pytest.mark.parametrize("error_number", (errno.EIO, errno.EREMOTEIO, errno.ETIMEDOUT))
def test_transient_bus_error_publishes_only_one_fresh_retry_sample(monkeypatch, error_number):
    state = _run_acquisition(monkeypatch, (OSError(error_number, "transient I2C read"), _sample()))

    assert state.ready.is_set() and not state.failed.is_set()
    assert state.sequence.value == 1
    assert len(state.read_times) == 2
    failed_time, successful_time = state.read_times
    assert successful_time > failed_time
    assert state.times[0] == successful_time
    assert state.times[1] >= successful_time
    assert state.values[0] == 12.5
    assert state.values[10] == 1.0
    assert bytes(state.error_bytes).strip(b"\0") == b""
    assert state.closed


def test_second_bus_failure_is_fatal_and_preserves_bounded_error(monkeypatch):
    state = _run_acquisition(monkeypatch, (
        OSError(errno.EIO, "first transfer failed"),
        OSError(errno.EREMOTEIO, "persistent I2C transfer " + "x" * 1000),
    ))

    assert state.failed.is_set() and state.ready.is_set()
    assert len(state.read_times) == 2
    assert state.sequence.value == 0
    assert list(state.times) == [0] * len(state.times)
    detail = bytes(state.error_bytes).split(b"\0", 1)[0].decode("utf-8")
    assert detail.startswith("OSError:[Errno 121] persistent I2C transfer ")
    assert len(detail.encode("utf-8")) <= 255
    assert state.closed


@pytest.mark.parametrize("error", (
    ValueError("malformed sample"),
    OSError(errno.EINVAL, "invalid bus operation"),
    OSError("malformed fused burst"),
))
def test_non_transient_failures_are_not_retried(monkeypatch, error):
    state = _run_acquisition(monkeypatch, (error, _sample()))

    assert state.failed.is_set() and state.ready.is_set()
    assert len(state.read_times) == 1
    assert state.sequence.value == 0
    assert bytes(state.error_bytes).split(b"\0", 1)[0].decode().startswith(type(error).__name__ + ":")
    assert state.closed


class _FailingRegisterBus:
    """Pickleable register bus for an actual spawned production owner."""

    def __init__(self):
        self.bursts = 0

    def read_byte_data(self, address, register):
        return {0x00: 0xA0, 0x35: 0xFF, 0x39: 5, 0x3A: 0}.get(register, 0)

    def write_byte_data(self, address, register, value):
        pass

    def read_i2c_block_data(self, address, register, length):
        self.bursts += 1
        if self.bursts >= 10:
            raise OSError(errno.EREMOTEIO, "injected persistent register failure")
        return [0] * length

    def close(self):
        pass


def _open_failing_register_bus(bus_number):
    return _FailingRegisterBus()


def _open_unavailable_bus(bus_number):
    raise OSError(errno.EIO, "injected startup bus unavailable")


def test_spawned_owner_failure_rejects_cached_sample_and_retains_original_errno():
    device = process_imu.ProcessBno055Device(
        NativeBno055DeviceConfig(), open_bus=_open_failing_register_bus,
        process_config=_process_config(),
    )
    try:
        first = device.read_sample(force=True)
        assert first["timestamp"] > 0
        assert device._failed.wait(3.0), "injected worker failure did not arrive"
        with pytest.raises(RuntimeError, match=r"unavailable:OSError:\[Errno 121\].*injected persistent register failure"):
            device.read_sample_at(time.monotonic_ns())
        assert not device.sensor_ok
    finally:
        device.close()
    assert not device._process.is_alive()


def test_spawned_startup_failure_preserves_original_errno():
    with pytest.raises(RuntimeError, match=r"startup:OSError:\[Errno 5\].*injected startup bus unavailable"):
        process_imu.ProcessBno055Device(
            NativeBno055DeviceConfig(), open_bus=_open_unavailable_bus,
            process_config=_process_config(),
        )


def test_native_source_failure_keeps_specific_health_reason_and_no_positive_sample():
    reading = ImuHeadingReading(7, 1_000, 0.1, 0.0, 1.0, 3, False, True)

    class Backend:
        calls = 0

        def read(self, context):
            self.calls += 1
            if self.calls == 1:
                return reading
            raise RuntimeError("BNO055 acquisition process unavailable:OSError:[Errno 121] register transfer failed " + "x" * 500)

    source = NativeImuSource(Backend(), NativeImuConfig("BNO055_IMU", .5, 2))
    assert source.read(TickContext(0, 1_010)).health.state is DeviceHealthState.OK
    failed = source.read(TickContext(1, 1_020))

    assert failed.health.state is DeviceHealthState.FAILED
    assert failed.health.reason.startswith("IMU_READ_ERROR:RuntimeError:")
    assert all(detail in failed.health.reason for detail in ("BNO055", "OSError", "121"))
    assert re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:/-]*", failed.health.reason)
    assert len(failed.health.reason) <= 256
    assert failed.samples == ()
    capability = source.capability_snapshot(1_030)
    assert capability.state is CapabilityState.FAILED
    assert "Errno 121" in capability.error
    assert capability.source_sequence == 7
    assert capability.source_monotonic_ns == 1_000
    assert source.integrity_snapshot().read_errors == 1
