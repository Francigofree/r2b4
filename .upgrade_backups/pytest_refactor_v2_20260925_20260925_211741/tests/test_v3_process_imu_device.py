from functools import partial
import ctypes
import multiprocessing
import os
import time

import pytest

from v3.adapters.bno055_device import NativeBno055DeviceConfig
from v3.adapters.bno055_imu import Bno055ImuBackendConfig, NativeBno055ImuBackend
from v3.adapters.process_imu_device import ProcessBno055Device
from v3.contracts import TickContext


class FakeBus:
    def __init__(self, bus_number, *, opened_pid, fail, closed):
        opened_pid.value = os.getpid()
        self.fail, self.closed = fail, closed

    def read_byte_data(self, address, register):
        return {0: 0xA0, 0x35: 0xFF, 0x39: 5}.get(register, 0)

    def write_byte_data(self, address, register, value):
        pass

    def read_i2c_block_data(self, address, register, length):
        if self.fail.is_set():
            raise OSError("injected bus failure")
        return [0] * length

    def close(self):
        self.closed.set()


@pytest.fixture
def device():
    context = multiprocessing.get_context("spawn")
    opened_pid = context.RawValue("i", 0)
    fail, closed = context.Event(), context.Event()
    instance = ProcessBno055Device(
        NativeBno055DeviceConfig(),
        open_bus=partial(FakeBus, opened_pid=opened_pid, fail=fail, closed=closed),
    )
    assert opened_pid.value != os.getpid()
    try:
        yield instance, fail
    finally:
        instance.close()
        assert closed.is_set()


def test_imu_acquisition_continues_while_parent_holds_gil(device):
    instance, _ = device
    first = instance.read_sample()
    # PyDLL deliberately retains the calling interpreter's GIL during the wait.
    libc = ctypes.PyDLL(None)
    libc.usleep(120_000)
    second = instance.read_sample()
    assert second["sequence"] >= first["sequence"] + 3
    assert second["timestamp"] > first["timestamp"]
    assert time.monotonic() - second["timestamp"] < 0.1


def test_busy_publisher_preserves_sequence_timestamp_and_stale_gate(device):
    instance, _ = device
    backend = NativeBno055ImuBackend(instance, Bno055ImuBackendConfig(
        maximum_sample_age_ns=100_000_000, heading_clockwise_positive=True,
        yaw_rate_axis=2, yaw_rate_clockwise_positive=False,
    ))
    first = backend.read(TickContext(10, time.monotonic_ns()))
    with instance._lock:
        started = time.monotonic()
        duplicate = backend.read(TickContext(11, time.monotonic_ns()))
        expired = backend.read(TickContext(12, first.captured_monotonic_ns + 300_000_000))
        assert time.monotonic() - started < 0.05
    assert duplicate.sequence == first.sequence
    assert duplicate.captured_monotonic_ns == first.captured_monotonic_ns
    assert expired.stale


def test_worker_failure_is_fail_closed_and_cannot_reuse_healthy_sample(device):
    instance, fail = device
    fail.set()
    assert instance._failed.wait(1.0)
    with pytest.raises(RuntimeError, match="unavailable"):
        instance.read_sample()
    assert instance.sensor_ok is False
