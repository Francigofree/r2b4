from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.live_encoder import (
    EncoderVelocityReading,
    NativeEncoderConfig,
    NativeEncoderSource,
)
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


class _FakeEncoderBackend:
    def __init__(self, result: EncoderVelocityReading | Exception) -> None:
        self._result = result
        self.calls: list[TickContext] = []

    def read(self, context: TickContext) -> EncoderVelocityReading:
        self.calls.append(context)
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


def _reading(
    *,
    trust: float = 0.8,
    stale: bool = False,
    timing_valid: bool = True,
) -> EncoderVelocityReading:
    return EncoderVelocityReading(
        sequence=12,
        captured_monotonic_ns=990,
        left_mps=0.12,
        right_mps=0.15,
        trust=trust,
        stale=stale,
        timing_valid=timing_valid,
    )


def _source(
    reading: EncoderVelocityReading,
) -> tuple[NativeEncoderSource, _FakeEncoderBackend]:
    backend = _FakeEncoderBackend(reading)
    source = NativeEncoderSource(
        backend,
        NativeEncoderConfig("KIT0085_ENCODER", minimum_trust=0.3),
    )
    return source, backend


def test_native_encoder_source_closes_one_typed_wheel_velocity_sample():
    context = TickContext(5, 1_000)
    source, backend = _source(_reading())

    snapshot = source.read(context)

    assert backend.calls == [context]
    assert snapshot.context == context
    assert snapshot.health == DeviceHealth(
        "KIT0085_ENCODER",
        DeviceHealthState.OK,
    )
    assert snapshot.samples == (
        DeviceSample(
            "KIT0085_ENCODER",
            "wheel_velocity",
            12,
            990,
            (
                DataField("left_mps", 0.12),
                DataField("right_mps", 0.15),
                DataField("trust", 0.8),
            ),
        ),
    )


@pytest.mark.parametrize(
    ("reading", "expected_state", "expected_reason"),
    (
        (
            _reading(timing_valid=False),
            DeviceHealthState.FAILED,
            "ENCODER_TIMING_INVALID",
        ),
        (_reading(stale=True), DeviceHealthState.DEGRADED, "ENCODER_STALE"),
        (_reading(trust=0.2), DeviceHealthState.DEGRADED, "ENCODER_LOW_TRUST"),
    ),
)
def test_native_encoder_source_maps_health_fail_closed(
    reading: EncoderVelocityReading,
    expected_state: DeviceHealthState,
    expected_reason: str,
):
    snapshot = _source(reading)[0].read(TickContext(5, 1_000))

    assert snapshot.health.state is expected_state
    assert snapshot.health.reason == expected_reason


def test_invalid_encoder_timing_is_rejected_by_existing_l2_admission():
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
    assert admitted.degraded_sources == ("KIT0085_ENCODER",)


def test_encoder_backend_failure_propagates_once_without_retry():
    context = TickContext(5, 1_000)
    backend = _FakeEncoderBackend(OSError("injected encoder failure"))
    source = NativeEncoderSource(
        backend,
        NativeEncoderConfig("KIT0085_ENCODER", minimum_trust=0.3),
    )

    with pytest.raises(OSError, match="encoder failure"):
        source.read(context)

    assert backend.calls == [context]


def test_encoder_reading_and_config_are_immutable_and_validated():
    reading = _reading()
    config = NativeEncoderConfig("KIT0085_ENCODER", minimum_trust=0.3)

    with pytest.raises(FrozenInstanceError):
        reading.sequence = 13
    with pytest.raises(FrozenInstanceError):
        config.minimum_trust = 0.2
    with pytest.raises(ValueError, match="trust must be within"):
        _reading(trust=1.1)
    with pytest.raises(ValueError, match="timing_valid must be bool"):
        EncoderVelocityReading(12, 990, 0.12, 0.15, 0.8, False, 1)
