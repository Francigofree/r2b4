from dataclasses import FrozenInstanceError
import math

import pytest

from v3.adapters.live_imu import ImuHeadingReading, NativeImuConfig, NativeImuSource
from v3.adapters.live_inputs import NativeLiveInputReader
from v3.contracts import (
    AcquisitionFrame,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    RejectionReason,
    TickContext,
)
from v3.layers.l2_admission import AdmissionConfig, InputAdmission


class _FakeImuBackend:
    def __init__(self, result: ImuHeadingReading | Exception) -> None:
        self._result = result
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> ImuHeadingReading:
        self.calls.append(context)
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


def _reading(
    *,
    confidence: float = 0.9,
    calibration: int = 3,
    stale: bool = False,
    timing_valid: bool = True,
) -> ImuHeadingReading:
    return ImuHeadingReading(
        sequence=21,
        captured_monotonic_ns=990,
        yaw_rad=math.pi / 2.0,
        omega_rad_s=-0.2,
        confidence=confidence,
        calibration=calibration,
        stale=stale,
        timing_valid=timing_valid,
    )


def _source(
    reading: ImuHeadingReading,
) -> tuple[NativeImuSource, _FakeImuBackend]:
    backend = _FakeImuBackend(reading)
    source = NativeImuSource(
        backend,
        NativeImuConfig(
            "BNO055_IMU",
            minimum_confidence=0.4,
            minimum_calibration=2,
        ),
    )
    return source, backend


def test_native_imu_source_closes_one_typed_heading_sample():
    context = TickContext(5, 1_000)
    source, backend = _source(_reading())

    snapshot = source.read(context)

    assert backend.calls == [context]
    assert snapshot.context == context
    assert snapshot.health == DeviceHealth("BNO055_IMU", DeviceHealthState.OK)
    assert snapshot.samples == (
        DeviceSample(
            "BNO055_IMU",
            "ekf_heading",
            21,
            990,
            (
                DataField("yaw_rad", math.pi / 2.0),
                DataField("omega_rad_s", -0.2),
                DataField("confidence", 0.9),
                DataField("calibration", 3),
            ),
        ),
    )


@pytest.mark.parametrize(
    ("reading", "expected_state", "expected_reason"),
    (
        (
            _reading(timing_valid=False),
            DeviceHealthState.FAILED,
            "IMU_TIMING_INVALID",
        ),
        (_reading(stale=True), DeviceHealthState.DEGRADED, "IMU_STALE"),
        (
            _reading(calibration=1),
            DeviceHealthState.DEGRADED,
            "IMU_CALIBRATION_LOW",
        ),
        (
            _reading(confidence=0.2),
            DeviceHealthState.DEGRADED,
            "IMU_LOW_CONFIDENCE",
        ),
    ),
)
def test_native_imu_source_maps_health_fail_closed(
    reading: ImuHeadingReading,
    expected_state: DeviceHealthState,
    expected_reason: str,
):
    snapshot = _source(reading)[0].read(TickContext(5, 1_000))

    assert snapshot.health.state is expected_state
    assert snapshot.health.reason == expected_reason


def test_invalid_imu_timing_is_rejected_by_existing_l2_admission():
    context = TickContext(5, 1_000)
    source, _ = _source(_reading(timing_valid=False))
    batch = NativeLiveInputReader((source,)).read(context)
    admitted = InputAdmission(AdmissionConfig(max_sample_age_ns=100))(
        AcquisitionFrame(batch.context, batch.samples, batch.device_health)
    )

    assert admitted.accepted == ()
    assert tuple(item.reason for item in admitted.rejected) == (
        RejectionReason.UNTRUSTED,
    )
    assert admitted.degraded_sources == ("BNO055_IMU",)


def test_imu_backend_failure_propagates_once_without_retry():
    context = TickContext(5, 1_000)
    backend = _FakeImuBackend(OSError("injected IMU failure"))
    source = NativeImuSource(
        backend,
        NativeImuConfig(
            "BNO055_IMU",
            minimum_confidence=0.4,
            minimum_calibration=2,
        ),
    )

    with pytest.raises(OSError, match="IMU failure"):
        source.read(context)

    assert backend.calls == [context]


def test_imu_reading_and_config_are_immutable_and_validated():
    reading = _reading()
    config = NativeImuConfig(
        "BNO055_IMU",
        minimum_confidence=0.4,
        minimum_calibration=2,
    )

    with pytest.raises(FrozenInstanceError):
        reading.sequence = 22
    with pytest.raises(FrozenInstanceError):
        config.minimum_calibration = 1
    with pytest.raises(ValueError, match="confidence must be within"):
        _reading(confidence=1.1)
    with pytest.raises(ValueError, match="calibration must be within"):
        _reading(calibration=4)
    with pytest.raises(ValueError, match="yaw_rad must be within"):
        ImuHeadingReading(21, 990, 4.0, -0.2, 0.9, 3, False, True)
    with pytest.raises(ValueError, match="timing_valid must be bool"):
        ImuHeadingReading(21, 990, 0.0, -0.2, 0.9, 3, False, 1)
