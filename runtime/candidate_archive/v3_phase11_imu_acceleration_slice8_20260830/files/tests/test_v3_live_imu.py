from dataclasses import FrozenInstanceError
import math

import pytest

from v3.adapters.live_imu import (
    ImuAccelerationReading,
    ImuHeadingReading,
    NativeImuConfig,
    NativeImuSource,
)
from v3.contracts import DeviceHealthState, TickContext


class _Backend:
    def __init__(self, result: object) -> None:
        self.result = result
        self.calls: list[TickContext] = []

    def read(self, context: TickContext):
        self.calls.append(context)
        return self.result


def _reading(
    *,
    stale: bool = False,
    timing_valid: bool = True,
    acceleration: ImuAccelerationReading | None = ImuAccelerationReading(0.4),
) -> ImuHeadingReading:
    return ImuHeadingReading(
        sequence=17,
        captured_monotonic_ns=1_000,
        yaw_rad=math.pi / 4.0,
        omega_rad_s=0.1,
        confidence=0.9,
        calibration=3,
        stale=stale,
        timing_valid=timing_valid,
        acceleration=acceleration,
    )


def _config() -> NativeImuConfig:
    return NativeImuConfig(
        "BNO055_IMU",
        minimum_confidence=0.4,
        minimum_calibration=2,
        maximum_abs_longitudinal_acceleration_mps2=5.0,
    )


def test_native_imu_source_closes_heading_and_acceleration_from_one_read():
    context = TickContext(7, 1_000)
    backend = _Backend(_reading())
    source = NativeImuSource(backend, _config())

    snapshot = source.read(context)

    assert backend.calls == [context]
    assert snapshot.context == context
    assert snapshot.health.state is DeviceHealthState.OK
    assert tuple(sample.kind for sample in snapshot.samples) == (
        "ekf_heading",
        "imu_acceleration",
    )
    heading, acceleration = snapshot.samples
    assert heading.sequence == acceleration.sequence == 17
    assert heading.captured_monotonic_ns == acceleration.captured_monotonic_ns == 1_000
    assert {field.key: field.value for field in acceleration.values} == {
        "longitudinal_mps2": 0.4,
    }


@pytest.mark.parametrize(
    ("reading", "health_state"),
    (
        (_reading(stale=True), DeviceHealthState.DEGRADED),
        (_reading(timing_valid=False), DeviceHealthState.FAILED),
    ),
)
def test_native_imu_source_omits_untrusted_acceleration(reading, health_state):
    snapshot = NativeImuSource(_Backend(reading), _config()).read(
        TickContext(7, 1_000)
    )

    assert snapshot.health.state is health_state
    assert tuple(sample.kind for sample in snapshot.samples) == ("ekf_heading",)


def test_native_imu_heading_only_compatibility_path_is_preserved():
    snapshot = NativeImuSource(
        _Backend(_reading(acceleration=None)),
        _config(),
    ).read(TickContext(7, 1_000))

    assert tuple(sample.kind for sample in snapshot.samples) == ("ekf_heading",)


def test_native_imu_acceleration_contract_is_immutable_and_fail_closed():
    acceleration = ImuAccelerationReading(0.4)
    with pytest.raises(FrozenInstanceError):
        acceleration.longitudinal_mps2 = 0.5
    with pytest.raises(ValueError, match="longitudinal_mps2"):
        ImuAccelerationReading(float("nan"))
    with pytest.raises(ValueError, match="maximum_abs_longitudinal"):
        NativeImuConfig("IMU", 0.4, 2, 0.0)

    source = NativeImuSource(
        _Backend(_reading(acceleration=ImuAccelerationReading(5.1))),
        _config(),
    )
    with pytest.raises(ValueError, match="physical range"):
        source.read(TickContext(7, 1_000))
