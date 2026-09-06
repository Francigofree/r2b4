import pytest

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig,
    NativeCounterEncoderBackend,
    SignedPulseCounterSnapshot,
)
from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
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
    assert tuple(field.value for field in baseline.samples[0].values) == (
        0.0,
        0.0,
        0.0,
    )
    assert current.health.state is DeviceHealthState.OK
    assert current.samples[0].sequence == 1
    assert tuple(field.value for field in current.samples[0].values) == pytest.approx(
        (0.1, 0.1, 1.0)
    )


def test_first_read_from_stopped_counter_is_invalid_zero_baseline():
    left = Counter((_snapshot(5),), running=False)
    right = Counter((_snapshot(8),))
    backend = NativeCounterEncoderBackend(left, right, _config())

    reading = backend.read(TickContext(0, 1_000))

    _assert_rejected(reading)
    assert reading.timing_valid is False


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
