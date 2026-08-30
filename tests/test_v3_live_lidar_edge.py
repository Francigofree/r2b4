from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.live_inputs import NativeLiveInputReader
from v3.adapters.live_lidar import (
    LidarHealthReading,
    NativeLidarConfig,
    NativeLidarSource,
)
from v3.contracts import (
    AcquisitionFrame,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    RejectionReason,
    RobotEstimate,
    TickContext,
)
from v3.layers.l2_admission import AdmissionConfig, InputAdmission
from v3.layers.l4_world_model import ShadowWorldModel


class _FakeLidarBackend:
    def __init__(self, result: LidarHealthReading | Exception) -> None:
        self._result = result
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> LidarHealthReading:
        self.calls.append(context)
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


def _reading(
    *,
    measurement_age_ns: int = 40,
    confidence: float = 0.8,
    stale: bool = False,
    timing_valid: bool = True,
) -> LidarHealthReading:
    return LidarHealthReading(
        revision=31,
        captured_monotonic_ns=990,
        measurement_age_ns=measurement_age_ns,
        confidence=confidence,
        stale=stale,
        timing_valid=timing_valid,
    )


def _source(
    reading: LidarHealthReading,
) -> tuple[NativeLidarSource, _FakeLidarBackend]:
    backend = _FakeLidarBackend(reading)
    source = NativeLidarSource(
        backend,
        NativeLidarConfig(
            "LIDAR_LOCALIZATION",
            minimum_confidence=0.3,
            maximum_measurement_age_ns=100,
        ),
    )
    return source, backend


def test_native_lidar_source_closes_one_l4_compatible_health_sample():
    context = TickContext(5, 1_000)
    source, backend = _source(_reading())

    snapshot = source.read(context)

    assert backend.calls == [context]
    assert snapshot.context == context
    assert snapshot.health == DeviceHealth(
        "LIDAR_LOCALIZATION",
        DeviceHealthState.OK,
    )
    assert snapshot.samples == (
        DeviceSample(
            "LIDAR_LOCALIZATION",
            "lidar_health",
            31,
            990,
            (
                DataField("age_ns", 40),
                DataField("confidence", 0.8),
            ),
        ),
    )


@pytest.mark.parametrize(
    ("reading", "expected_state", "expected_reason"),
    (
        (
            _reading(timing_valid=False),
            DeviceHealthState.FAILED,
            "LIDAR_TIMING_INVALID",
        ),
        (_reading(stale=True), DeviceHealthState.DEGRADED, "LIDAR_STALE"),
        (
            _reading(measurement_age_ns=101),
            DeviceHealthState.DEGRADED,
            "LIDAR_STALE",
        ),
        (
            _reading(confidence=0.2),
            DeviceHealthState.DEGRADED,
            "LIDAR_LOW_CONFIDENCE",
        ),
    ),
)
def test_native_lidar_source_maps_health_fail_closed(
    reading: LidarHealthReading,
    expected_state: DeviceHealthState,
    expected_reason: str,
):
    snapshot = _source(reading)[0].read(TickContext(5, 1_000))

    assert snapshot.health.state is expected_state
    assert snapshot.health.reason == expected_reason


def test_invalid_lidar_timing_is_rejected_by_existing_l2_admission():
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
    assert admitted.degraded_sources == ("LIDAR_LOCALIZATION",)


def test_native_lidar_sample_drives_existing_l4_revision_and_freshness():
    context = TickContext(5, 1_000)
    source, _ = _source(_reading())
    batch = NativeLiveInputReader((source,)).read(context)
    admitted = InputAdmission(AdmissionConfig(max_sample_age_ns=100))(
        AcquisitionFrame(batch.context, batch.samples, batch.device_health)
    )
    estimate = RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        v_mps=0.0,
        omega_rad_s=0.0,
        covariance_5x5=(0.0,) * 25,
    )

    world = ShadowWorldModel()(admitted, estimate)

    assert world.map_revision == 1
    assert world.freshness_ns == 50
    assert world.obstacle_tracks == ()


def test_lidar_backend_failure_propagates_once_without_retry():
    context = TickContext(5, 1_000)
    backend = _FakeLidarBackend(OSError("injected lidar failure"))
    source = NativeLidarSource(
        backend,
        NativeLidarConfig(
            "LIDAR_LOCALIZATION",
            minimum_confidence=0.3,
            maximum_measurement_age_ns=100,
        ),
    )

    with pytest.raises(OSError, match="lidar failure"):
        source.read(context)

    assert backend.calls == [context]


def test_lidar_reading_and_config_are_immutable_and_validated():
    reading = _reading()
    config = NativeLidarConfig(
        "LIDAR_LOCALIZATION",
        minimum_confidence=0.3,
        maximum_measurement_age_ns=100,
    )

    with pytest.raises(FrozenInstanceError):
        reading.revision = 32
    with pytest.raises(FrozenInstanceError):
        config.minimum_confidence = 0.2
    with pytest.raises(ValueError, match="confidence must be within"):
        _reading(confidence=1.1)
    with pytest.raises(ValueError, match="measurement_age_ns"):
        _reading(measurement_age_ns=-1)
    with pytest.raises(ValueError, match="timing_valid must be bool"):
        LidarHealthReading(31, 990, 40, 0.8, False, 1)
