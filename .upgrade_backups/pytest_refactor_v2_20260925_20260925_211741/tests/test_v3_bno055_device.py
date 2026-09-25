from dataclasses import FrozenInstanceError, replace

import pytest

from v3.adapters.bno055_device import (
    NativeBno055Device,
    NativeBno055DeviceConfig,
)
from v3.adapters.bno055_imu import Bno055ImuBackendConfig, NativeBno055ImuBackend
from v3.contracts import TickContext


class Bus:
    def __init__(self, *, chip_ids=(0xA0,), fail_burst=False) -> None:
        self.chip_ids = iter(chip_ids)
        self.last_chip_id = 0
        self.fail_burst = fail_burst
        self.calls = []
        self.close_calls = 0
        self.calibration = 0xFF
        self.system_status = 5
        self.system_error = 0
        self.burst = [0] * 32
        self.burst[4:6] = [0x10, 0x00]  # +1 dps around physical Z.
        self.burst[6:8] = [0x80, 0x05]  # 88 degrees heading.

    def read_byte_data(self, address, register):
        self.calls.append(("read_byte", address, register))
        if register == NativeBno055Device.REG_CHIP_ID:
            try:
                self.last_chip_id = next(self.chip_ids)
            except StopIteration:
                pass
            return self.last_chip_id
        if register == NativeBno055Device.REG_CALIB_STAT:
            return self.calibration
        if register == NativeBno055Device.REG_SYS_STATUS:
            return self.system_status
        if register == NativeBno055Device.REG_SYS_ERR:
            return self.system_error
        raise AssertionError(register)

    def write_byte_data(self, address, register, value):
        self.calls.append(("write_byte", address, register, value))
        return 0

    def read_i2c_block_data(self, address, register, length):
        self.calls.append(("burst", address, register, length))
        if self.fail_burst:
            raise OSError("injected burst failure")
        return list(self.burst)

    def close(self):
        self.close_calls += 1


class Clock:
    def __init__(self, values) -> None:
        self.values = iter(values)

    def __call__(self):
        return next(self.values)


def test_native_device_initializes_and_uses_tick_clock_domain_for_fresh_sample():
    bus = Bus(chip_ids=(0, 0xA0))
    sleeps = []
    clock = Clock((1_000_000_000, 1_010_000_000, 1_020_000_000))
    device = NativeBno055Device(bus, NativeBno055DeviceConfig(), monotonic_ns=clock, sleep=sleeps.append)

    device.initialize()
    backend = NativeBno055ImuBackend(
        device,
        Bno055ImuBackendConfig(
            maximum_sample_age_ns=20_000_000,
            heading_clockwise_positive=True,
            yaw_rate_axis=2,
            yaw_rate_clockwise_positive=True,
        ),
    )
    reading = backend.read(TickContext(7, 1_040_000_000))

    assert device.initialized is True
    assert device.sensor_ok is True
    assert reading.sequence == 7
    assert reading.captured_monotonic_ns == 1_040_000_000
    assert reading.stale is False
    assert reading.timing_valid is True
    assert reading.yaw_rad == pytest.approx(-1.53588974175501)
    assert reading.omega_rad_s == pytest.approx(-0.017453292519943295)
    assert [call for call in bus.calls if call[0] == "burst"] == [
        ("burst", 0x28, 0x14, 32),
        ("burst", 0x28, 0x14, 32),
    ]
    assert sleeps == [0.05, 0.03, 0.01, 0.08]


def test_axis_mapping_is_applied_before_backend_selects_yaw_rate_axis():
    bus = Bus()
    bus.burst[0:6] = [0x10, 0x00, 0x20, 0x00, 0x30, 0x00]
    config = NativeBno055DeviceConfig(
        axis_order=(2, 0, 1),
        axis_sign=(-1, 1, -1),
    )
    device = NativeBno055Device(
        bus,
        config,
        monotonic_ns=Clock((1_000, 2_000, 3_000)),
        sleep=lambda _: None,
    )
    device.initialize()

    sample = device.read_sample(force=True)

    assert sample["gyro_dps"] == (-3.0, 1.0, -2.0)


def test_initialization_failure_closes_bus_once_and_latches_device_closed():
    bus = Bus(chip_ids=(0,))
    device = NativeBno055Device(
        bus,
        NativeBno055DeviceConfig(
            startup_timeout_ns=10,
            startup_poll_interval_ns=10,
        ),
        monotonic_ns=Clock((100, 110)),
        sleep=lambda _: None,
    )

    with pytest.raises(OSError, match="chip ID"):
        device.initialize()

    assert bus.close_calls == 1
    assert device.initialized is False
    assert device.sensor_ok is False
    with pytest.raises(RuntimeError, match="closed"):
        device.initialize()
    device.close()
    assert bus.close_calls == 1


def test_read_failure_marks_sensor_failed_without_retry():
    bus = Bus()
    device = NativeBno055Device(
        bus,
        NativeBno055DeviceConfig(),
        monotonic_ns=Clock((100, 200, 300)),
        sleep=lambda _: None,
    )
    device.initialize()
    bus.fail_burst = True

    with pytest.raises(OSError, match="burst failure"):
        device.read_sample(force=True)

    assert device.sensor_ok is False
    assert len([call for call in bus.calls if call[0] == "burst"]) == 2


@pytest.mark.parametrize(
    ("change", "message"),
    (
        ({"address": 0x80}, "seven-bit"),
        ({"operation_mode": "CONFIG"}, "operation_mode"),
        ({"axis_order": (0, 0, 2)}, "axis_order"),
        ({"axis_sign": (1, 0, -1)}, "axis_sign"),
        ({"use_external_crystal": 1}, "must be bool"),
        (
            {"startup_timeout_ns": 10, "startup_poll_interval_ns": 11},
            "cannot exceed",
        ),
    ),
)
def test_device_config_rejects_invalid_physical_values(change, message):
    with pytest.raises(ValueError, match=message):
        replace(NativeBno055DeviceConfig(), **change)


def test_device_config_is_immutable():
    config = NativeBno055DeviceConfig()
    with pytest.raises(FrozenInstanceError):
        config.address = 0x29  # type: ignore[misc]
