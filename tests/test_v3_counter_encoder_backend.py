import pytest

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig,
    NativeCounterEncoderBackend,
    SignedPulseCounterSnapshot,
)
from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
from v3.adapters.live_encoder import EncoderRejectionCode
from v3.contracts import DeviceHealthState, TickContext


class Counter:
    def __init__(
        self,
        snapshots: tuple[SignedPulseCounterSnapshot, ...],
        *,
        running: bool = True,
    ) -> None:
        self._snapshots = iter(snapshots)
        self.running = running
        self.calls = 0

    def snapshot(self) -> SignedPulseCounterSnapshot:
        self.calls += 1
        return next(self._snapshots)


def _snapshot(
    pulses: int,
    *,
    read_errors: int = 0,
    invalid_alerts: int = 0,
) -> SignedPulseCounterSnapshot:
    return SignedPulseCounterSnapshot(pulses, read_errors, invalid_alerts)


def _config() -> CounterEncoderBackendConfig:
    return CounterEncoderBackendConfig(
        left_step_distance_m=0.001,
        right_step_distance_m=0.002,
        maximum_sample_interval_ns=200_000_000,
        maximum_abs_velocity_mps=1.5,
    )


def _backend(
    left_values: tuple[SignedPulseCounterSnapshot, ...],
    right_values: tuple[SignedPulseCounterSnapshot, ...],
):
    left = Counter(left_values)
    right = Counter(right_values)
    backend = NativeCounterEncoderBackend(left, right, _config())
    return backend, left, right


def _assert_rejected(reading) -> None:
    assert reading.trust == 0.0
    assert reading.left_mps == 0.0
    assert reading.right_mps == 0.0


def _sample_values(snapshot) -> dict[str, object]:
    return {
        field.key: field.value
        for field in snapshot.samples[0].values
    }


def test_constructor_does_not_create_an_untimed_counter_baseline():
    backend, left, right = _backend(
        (_snapshot(100),),
        (_snapshot(200),),
    )

    assert left.calls == 0
    assert right.calls == 0
    baseline_context = TickContext(7, 1_000_000_000)
    baseline = backend.read(baseline_context)

    _assert_rejected(baseline)
    assert baseline.sequence == baseline_context.tick_id
    assert baseline.captured_monotonic_ns == baseline_context.monotonic_ns
    assert baseline.timing_valid is True
    assert baseline.stale is False
    assert baseline.diagnostics is not None
    assert baseline.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert baseline.diagnostics.raw_left_pulse_count == 100
    assert baseline.diagnostics.raw_right_pulse_count == 200
    assert baseline.diagnostics.left_pulse_delta is None
    assert baseline.diagnostics.right_pulse_delta is None
    assert baseline.diagnostics.sample_interval_ns is None
    assert baseline.diagnostics.computed_left_mps is None
    assert baseline.diagnostics.computed_right_mps is None
    assert left.calls == 1
    assert right.calls == 1


def test_signed_delta_uses_the_same_read_api_baseline_and_tick_time():
    backend, left, right = _backend(
        (_snapshot(100), _snapshot(110)),
        (_snapshot(200), _snapshot(195)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_100_000_000))

    assert reading.left_mps == pytest.approx(0.1)
    assert reading.right_mps == pytest.approx(-0.1)
    assert reading.trust == 1.0
    assert reading.stale is False
    assert reading.timing_valid is True
    assert reading.diagnostics is not None
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.NONE
    assert reading.diagnostics.raw_left_pulse_count == 110
    assert reading.diagnostics.raw_right_pulse_count == 195
    assert reading.diagnostics.left_pulse_delta == 10
    assert reading.diagnostics.right_pulse_delta == -5
    assert reading.diagnostics.sample_interval_ns == 100_000_000
    assert reading.diagnostics.computed_left_mps == pytest.approx(0.1)
    assert reading.diagnostics.computed_right_mps == pytest.approx(-0.1)
    assert left.calls == 2
    assert right.calls == 2
    assert not hasattr(backend, "set_last_pwm")


def test_each_increasing_tick_reanchors_the_next_delta():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(10), _snapshot(13)),
        (_snapshot(0), _snapshot(5), _snapshot(7)),
    )

    baseline = backend.read(TickContext(0, 1_000_000_000))
    first = backend.read(TickContext(1, 1_100_000_000))
    second = backend.read(TickContext(2, 1_200_000_000))

    _assert_rejected(baseline)
    assert first.left_mps == pytest.approx(0.1)
    assert first.right_mps == pytest.approx(0.1)
    assert second.left_mps == pytest.approx(0.03)
    assert second.right_mps == pytest.approx(0.04)


def test_stale_interval_is_untrusted_zero_and_reanchors_for_recovery():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(20), _snapshot(25)),
        (_snapshot(0), _snapshot(10), _snapshot(12)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    stale = backend.read(TickContext(1, 1_300_000_000))
    recovered = backend.read(TickContext(2, 1_400_000_000))

    _assert_rejected(stale)
    assert stale.stale is True
    assert stale.timing_valid is True
    assert stale.diagnostics is not None
    assert (
        stale.diagnostics.rejection_code
        is EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
    )
    assert stale.diagnostics.left_pulse_delta == 20
    assert stale.diagnostics.right_pulse_delta == 10
    assert stale.diagnostics.sample_interval_ns == 300_000_000
    assert stale.diagnostics.computed_left_mps == pytest.approx(0.0666666667)
    assert stale.diagnostics.computed_right_mps == pytest.approx(0.0666666667)
    assert recovered.trust == 1.0
    assert recovered.left_mps == pytest.approx(0.05)
    assert recovered.right_mps == pytest.approx(0.04)


@pytest.mark.parametrize(
    ("side", "diagnostic"),
    (
        ("left", {"read_errors": 1}),
        ("left", {"invalid_alerts": 1}),
        ("right", {"read_errors": 1}),
        ("right", {"invalid_alerts": 1}),
    ),
)
def test_counter_diagnostic_error_is_untrusted_and_zero(side, diagnostic):
    left_current = _snapshot(1, **diagnostic) if side == "left" else _snapshot(1)
    right_current = _snapshot(1, **diagnostic) if side == "right" else _snapshot(1)
    backend, _, _ = _backend(
        (_snapshot(0), left_current),
        (_snapshot(0), right_current),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_100_000_000))

    _assert_rejected(reading)
    assert reading.timing_valid is True
    assert reading.stale is False
    assert reading.diagnostics is not None
    expected_code = (
        EncoderRejectionCode.COUNTER_READ_ERROR_CHANGED
        if "read_errors" in diagnostic
        else EncoderRejectionCode.COUNTER_INVALID_ALERT_CHANGED
    )
    assert reading.diagnostics.rejection_code is expected_code
    assert reading.diagnostics.left_read_error_delta == (
        1 if side == "left" and "read_errors" in diagnostic else 0
    )
    assert reading.diagnostics.right_read_error_delta == (
        1 if side == "right" and "read_errors" in diagnostic else 0
    )
    assert reading.diagnostics.left_invalid_alert_delta == (
        1 if side == "left" and "invalid_alerts" in diagnostic else 0
    )
    assert reading.diagnostics.right_invalid_alert_delta == (
        1 if side == "right" and "invalid_alerts" in diagnostic else 0
    )


@pytest.mark.parametrize(
    ("side", "diagnostic"),
    (
        ("left", {"read_errors": 1}),
        ("left", {"invalid_alerts": 1}),
        ("right", {"read_errors": 1}),
        ("right", {"invalid_alerts": 1}),
    ),
)
def test_counter_diagnostic_reanchors_then_recovers_when_total_stays_constant(
    side,
    diagnostic,
):
    left_error = _snapshot(10, **diagnostic) if side == "left" else _snapshot(10)
    right_error = _snapshot(5, **diagnostic) if side == "right" else _snapshot(5)
    left_clean = _snapshot(14, **diagnostic) if side == "left" else _snapshot(14)
    right_clean = _snapshot(7, **diagnostic) if side == "right" else _snapshot(7)
    backend, _, _ = _backend(
        (_snapshot(0), left_error, left_clean),
        (_snapshot(0), right_error, right_clean),
    )
    backend.read(TickContext(0, 1_000_000_000))

    rejected = backend.read(TickContext(1, 1_100_000_000))
    recovered = backend.read(TickContext(2, 1_200_000_000))

    _assert_rejected(rejected)
    assert recovered.trust == 1.0
    assert recovered.left_mps == pytest.approx(0.04)
    assert recovered.right_mps == pytest.approx(0.04)


def test_stopped_counter_is_timing_invalid_untrusted_and_zero():
    backend, left, _ = _backend(
        (_snapshot(0), _snapshot(10)),
        (_snapshot(0), _snapshot(10)),
    )
    backend.read(TickContext(0, 1_000_000_000))
    left.running = False

    reading = backend.read(TickContext(1, 1_100_000_000))

    _assert_rejected(reading)
    assert reading.timing_valid is False
    assert reading.stale is False
    assert reading.diagnostics is not None
    assert (
        reading.diagnostics.rejection_code
        is EncoderRejectionCode.COUNTER_NOT_RUNNING
    )
    assert reading.diagnostics.left_counter_running is False
    assert reading.diagnostics.right_counter_running is True
    assert reading.diagnostics.computed_left_mps == pytest.approx(0.1)
    assert reading.diagnostics.computed_right_mps == pytest.approx(0.2)


def test_impossible_velocity_is_untrusted_and_zero():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(1_000)),
        (_snapshot(0), _snapshot(0)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_100_000_000))

    _assert_rejected(reading)
    assert reading.timing_valid is True
    assert reading.stale is False
    assert reading.diagnostics is not None
    assert (
        reading.diagnostics.rejection_code
        is EncoderRejectionCode.LEFT_VELOCITY_LIMIT_EXCEEDED
    )
    assert reading.diagnostics.left_pulse_delta == 1_000
    assert reading.diagnostics.right_pulse_delta == 0
    assert reading.diagnostics.computed_left_mps == pytest.approx(10.0)
    assert reading.diagnostics.computed_right_mps == pytest.approx(0.0)
    assert reading.diagnostics.maximum_abs_velocity_mps == pytest.approx(1.5)


def test_nonincreasing_tick_time_is_invalid_zero_and_does_not_reanchor():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(1), _snapshot(2)),
        (_snapshot(0), _snapshot(1), _snapshot(2)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    invalid = backend.read(TickContext(1, 1_000_000_000))
    recovered = backend.read(TickContext(2, 1_100_000_000))

    _assert_rejected(invalid)
    assert invalid.timing_valid is False
    assert invalid.diagnostics is not None
    assert (
        invalid.diagnostics.rejection_code
        is EncoderRejectionCode.NONINCREASING_TICK_TIME
    )
    assert invalid.diagnostics.sample_interval_ns == 0
    assert invalid.diagnostics.left_pulse_delta == 1
    assert invalid.diagnostics.right_pulse_delta == 1
    assert invalid.diagnostics.computed_left_mps is None
    assert invalid.diagnostics.computed_right_mps is None
    assert recovered.trust == 1.0
    assert recovered.left_mps == pytest.approx(0.02)
    assert recovered.right_mps == pytest.approx(0.04)


def test_native_encoder_source_sees_low_trust_baseline_then_ok_delta():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(10)),
        (_snapshot(0), _snapshot(5)),
    )
    source = NativeEncoderSource(backend, NativeEncoderConfig("encoder", 0.5))

    baseline = source.read(TickContext(0, 1_000_000_000))
    current = source.read(TickContext(1, 1_100_000_000))

    assert baseline.health.state is DeviceHealthState.DEGRADED
    assert baseline.health.reason == "ENCODER_LOW_TRUST"
    baseline_values = _sample_values(baseline)
    assert (baseline_values["left_mps"], baseline_values["right_mps"]) == (0.0, 0.0)
    assert baseline_values["trust"] == 0.0
    assert baseline_values["rejection_code"] == "BASELINE"
    assert baseline_values["raw_left_pulse_count"] == 0
    assert baseline_values["raw_right_pulse_count"] == 0
    assert baseline_values["left_pulse_delta"] is None
    assert baseline_values["right_pulse_delta"] is None
    assert current.health.state is DeviceHealthState.OK
    assert current.samples[0].sequence == 1
    current_values = _sample_values(current)
    assert (
        current_values["left_mps"],
        current_values["right_mps"],
        current_values["trust"],
    ) == pytest.approx((0.1, 0.1, 1.0))
    assert current_values["rejection_code"] == "NONE"
    assert current_values["raw_left_pulse_count"] == 10
    assert current_values["raw_right_pulse_count"] == 5
    assert current_values["left_pulse_delta"] == 10
    assert current_values["right_pulse_delta"] == 5
    assert current_values["sample_interval_ns"] == 100_000_000
    assert current_values["computed_left_mps"] == pytest.approx(0.1)
    assert current_values["computed_right_mps"] == pytest.approx(0.1)


def test_first_read_from_stopped_counter_is_invalid_zero_baseline():
    left = Counter((_snapshot(5),), running=False)
    right = Counter((_snapshot(8),))
    backend = NativeCounterEncoderBackend(left, right, _config())

    reading = backend.read(TickContext(0, 1_000))

    _assert_rejected(reading)
    assert reading.timing_valid is False
    assert reading.diagnostics is not None
    assert (
        reading.diagnostics.rejection_code
        is EncoderRejectionCode.COUNTER_NOT_RUNNING
    )


def test_combined_counter_diagnostics_have_one_exact_code_and_per_side_deltas():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(4, read_errors=2)),
        (_snapshot(0), _snapshot(3, invalid_alerts=1)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_100_000_000))

    _assert_rejected(reading)
    assert reading.diagnostics is not None
    assert (
        reading.diagnostics.rejection_code
        is EncoderRejectionCode.COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED
    )
    assert reading.diagnostics.left_read_error_delta == 2
    assert reading.diagnostics.right_read_error_delta == 0
    assert reading.diagnostics.left_invalid_alert_delta == 0
    assert reading.diagnostics.right_invalid_alert_delta == 1
    assert reading.diagnostics.computed_left_mps == pytest.approx(0.04)
    assert reading.diagnostics.computed_right_mps == pytest.approx(0.06)


def test_both_velocity_limits_have_a_distinct_rejection_code():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(200)),
        (_snapshot(0), _snapshot(-100)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_100_000_000))

    _assert_rejected(reading)
    assert reading.diagnostics is not None
    assert (
        reading.diagnostics.rejection_code
        is EncoderRejectionCode.BOTH_VELOCITY_LIMIT_EXCEEDED
    )
    assert reading.diagnostics.computed_left_mps == pytest.approx(2.0)
    assert reading.diagnostics.computed_right_mps == pytest.approx(-2.0)


def test_malformed_snapshot_is_rejected_without_a_second_left_read():
    class BadCounter:
        running = True

        def __init__(self) -> None:
            self.calls = 0

        def snapshot(self):
            self.calls += 1
            return object()

    left = BadCounter()
    right = Counter((_snapshot(0),))
    backend = NativeCounterEncoderBackend(
        left,  # type: ignore[arg-type]
        right,
        _config(),
    )

    with pytest.raises(TypeError, match="SignedPulseCounterSnapshot"):
        backend.read(TickContext(0, 1))

    assert left.calls == 1
    assert right.calls == 0


@pytest.mark.parametrize(
    "kwargs",
    (
        {"left_step_distance_m": 0.0},
        {"right_step_distance_m": float("nan")},
        {"maximum_sample_interval_ns": 0},
        {"maximum_abs_velocity_mps": -1.0},
    ),
)
def test_backend_config_rejects_invalid_geometry_or_bounds(kwargs):
    values = {
        "left_step_distance_m": 0.001,
        "right_step_distance_m": 0.001,
        "maximum_sample_interval_ns": 1,
        "maximum_abs_velocity_mps": 1.0,
    }
    values.update(kwargs)

    with pytest.raises(ValueError):
        CounterEncoderBackendConfig(**values)
