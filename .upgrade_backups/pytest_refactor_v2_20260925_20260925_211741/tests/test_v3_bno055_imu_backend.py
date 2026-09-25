import math

import pytest

from v3.adapters.bno055_imu import (
    Bno055ImuBackendConfig,
    NativeBno055ImuBackend,
)
from v3.adapters.live_imu import NativeImuConfig, NativeImuSource
from v3.contracts import DeviceHealthState, TickContext


class Device:
    def __init__(self, sample: object) -> None:
        self.sample = sample
        self.initialized = True
        self.sensor_ok = True
        self.read_calls: list[bool] = []
        self.close_calls = 0

    def read_sample(self, *, force: bool = False):
        self.read_calls.append(force)
        return self.sample

    def close(self) -> None:
        self.close_calls += 1


def _sample(**changes):
    value = {
        "timestamp": 0.990,
        "heading_deg": 90.0,
        "gyro_dps": (0.0, 0.0, 30.0),
        "calibration": {"sys": 3, "gyro": 3, "accel": 2, "mag": 3},
        "sys_error": 0,
    }
    value.update(changes)
    return value


def _config(**changes) -> Bno055ImuBackendConfig:
    values = {
        "maximum_sample_age_ns": 20_000_000,
        "heading_clockwise_positive": True,
        "yaw_rate_axis": 2,
        "yaw_rate_clockwise_positive": False,
    }
    values.update(changes)
    return Bno055ImuBackendConfig(**values)


def test_one_forced_sample_maps_units_sign_calibration_and_tick_identity():
    device = Device(_sample())
    backend = NativeBno055ImuBackend(device, _config())

    reading = backend.read(TickContext(7, 1_000_000_000))

    assert device.read_calls == [True]
    assert reading.sequence == 7
    assert reading.captured_monotonic_ns == 990_000_000
    assert reading.yaw_rad == pytest.approx(-math.pi / 2.0)
    assert reading.omega_rad_s == pytest.approx(math.pi / 6.0)
    assert reading.calibration == 2
    assert reading.confidence == pytest.approx(2.0 / 3.0)
    assert reading.omega_calibration == 3
    assert reading.omega_confidence == 1.0
    assert reading.stale is False
    assert reading.timing_valid is True


def test_explicit_sign_and_offset_policy_produces_wrapped_ccw_yaw():
    backend = NativeBno055ImuBackend(
        Device(_sample(heading_deg=350.0, gyro_dps=(10.0, 0.0, 0.0))),
        _config(
            heading_clockwise_positive=False,
            yaw_rate_axis=0,
            yaw_rate_clockwise_positive=True,
            yaw_offset_rad=math.radians(20.0),
        ),
    )

    reading = backend.read(TickContext(1, 1_000_000_000))

    assert reading.yaw_rad == pytest.approx(math.radians(10.0))
    assert reading.omega_rad_s == pytest.approx(-math.radians(10.0))


def test_stale_and_device_failure_map_through_existing_native_source_health():
    stale_device = Device(_sample(timestamp=0.970))
    stale = NativeImuSource(
        NativeBno055ImuBackend(stale_device, _config()),
        NativeImuConfig("imu", 0.5, 2),
    ).read(TickContext(1, 1_000_000_000))
    failed_device = Device(_sample())
    failed_device.sensor_ok = False
    failed = NativeImuSource(
        NativeBno055ImuBackend(failed_device, _config()),
        NativeImuConfig("imu", 0.5, 2),
    ).read(TickContext(1, 1_000_000_000))

    assert stale_device.read_calls == [True]
    assert stale.health.state is DeviceHealthState.DEGRADED
    assert stale.health.reason == "IMU_STALE"
    assert failed_device.read_calls == [True]
    assert failed.health.state is DeviceHealthState.FAILED
    assert failed.health.reason == "IMU_TIMING_INVALID"


def test_lidar_first_rate_only_health_keeps_calibrated_gyro_without_trusting_heading():
    source = NativeImuSource(
        NativeBno055ImuBackend(
            Device(
                _sample(
                    calibration={"sys": 0, "gyro": 3, "accel": 0, "mag": 0},
                )
            ),
            _config(),
        ),
        NativeImuConfig("imu", 0.5, 2, allow_rate_only=True),
    )

    snapshot = source.read(TickContext(1, 1_000_000_000))
    fields = {field.key: field.value for field in snapshot.samples[0].values}

    assert snapshot.health.state is DeviceHealthState.OK
    assert fields["confidence"] == 0.0
    assert fields["calibration"] == 0
    assert fields["omega_confidence"] == 1.0
    assert fields["omega_calibration"] == 3


@pytest.mark.parametrize(
    "sample",
    (
        object(),
        _sample(timestamp=float("nan")),
        _sample(gyro_dps=(0.0, 1.0)),
        _sample(calibration={"sys": 4, "gyro": 3, "accel": 3, "mag": 3}),
    ),
)
def test_malformed_atomic_sample_fails_closed_without_retry(sample):
    device = Device(sample)
    backend = NativeBno055ImuBackend(device, _config())

    with pytest.raises((TypeError, ValueError)):
        backend.read(TickContext(1, 1_000_000_000))

    assert device.read_calls == [True]


def test_invalid_backend_config_is_rejected():
    with pytest.raises(ValueError, match="yaw_rate_axis"):
        _config(yaw_rate_axis=3)
    with pytest.raises(ValueError, match="must be bool"):
        _config(heading_clockwise_positive=1)
