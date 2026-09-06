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


def _config(**kwargs) -> CounterEncoderBackendConfig:
    return CounterEncoderBackendConfig(
        left_step_distance_m=0.001,
        right_step_distance_m=0.002,
        maximum_sample_interval_ns=200_000_000,
        maximum_abs_velocity_mps=1.5,
        **kwargs,
    )


def _backend(
    left_values: tuple[SignedPulseCounterSnapshot, ...],
    right_values: tuple[SignedPulseCounterSnapshot, ...],
    *,
    config: CounterEncoderBackendConfig | None = None,
):
    left = Counter(left_values)
    right = Counter(right_values)
    backend = NativeCounterEncoderBackend(
        left,
        right,
        _config() if config is None else config,
        baseline_monotonic_ns=1_000_000_000,
    )
    return backend, left, right


def test_signed_count_delta_and_tick_time_are_the_only_velocity_inputs():
    backend, left, right = _backend(
        (_snapshot(100), _snapshot(110)),
        (_snapshot(200), _snapshot(195)),
    )

    reading = backend.read(TickContext(0, 1_100_000_000))

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

    first = backend.read(TickContext(0, 1_100_000_000))
    second = backend.read(TickContext(1, 1_200_000_000))

    assert first.left_mps == pytest.approx(0.1)
    assert first.right_mps == pytest.approx(0.1)
    assert second.left_mps == pytest.approx(0.03)
    assert second.right_mps == pytest.approx(0.04)


def test_stale_interval_is_zeroed_and_reanchors_for_recovery():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(20), _snapshot(25)),
        (_snapshot(0), _snapshot(10), _snapshot(12)),
    )

    stale = backend.read(TickContext(0, 1_300_000_000))
    recovered = backend.read(TickContext(1, 1_400_000_000))

    assert stale.stale is True
    assert stale.timing_valid is True
    assert stale.left_mps == 0.0
    assert stale.right_mps == 0.0
    assert recovered.stale is False
    assert recovered.left_mps == pytest.approx(0.05)
    assert recovered.right_mps == pytest.approx(0.04)


@pytest.mark.parametrize(
    "diagnostic",
    (
        {"read_errors": 1},
        {"invalid_alerts": 1},
    ),
)
def test_counter_diagnostic_error_degrades_trust(diagnostic):
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(1, **diagnostic)),
        (_snapshot(0), _snapshot(1)),
    )

    reading = backend.read(TickContext(0, 1_100_000_000))

    assert reading.trust == 0.0
    assert reading.timing_valid is True


def test_stopped_counter_makes_timing_invalid_and_zero():
    backend, left, _ = _backend(
        (_snapshot(0), _snapshot(10)),
        (_snapshot(0), _snapshot(10)),
    )
    left.running = False

    reading = backend.read(TickContext(0, 1_100_000_000))

    assert reading.timing_valid is False
    assert reading.trust == 0.0
    assert reading.left_mps == 0.0
    assert reading.right_mps == 0.0


def test_impossible_velocity_is_zeroed_and_untrusted():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(1_000)),
        (_snapshot(0), _snapshot(0)),
    )

    reading = backend.read(TickContext(0, 1_100_000_000))

    assert reading.timing_valid is True
    assert reading.stale is False
    assert reading.trust == 0.0
    assert reading.left_mps == 0.0
    assert reading.right_mps == 0.0


def test_nonincreasing_tick_time_is_invalid_and_does_not_move_baseline_time():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(1), _snapshot(2)),
        (_snapshot(0), _snapshot(1), _snapshot(2)),
    )

    invalid = backend.read(TickContext(0, 1_000_000_000))
    recovered = backend.read(TickContext(1, 1_100_000_000))

    assert invalid.timing_valid is False
    assert recovered.timing_valid is True
    assert recovered.left_mps == pytest.approx(0.02)
    assert recovered.right_mps == pytest.approx(0.04)


def test_native_encoder_source_receives_ok_typed_velocity_snapshot():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(10)),
        (_snapshot(0), _snapshot(5)),
    )
    source = NativeEncoderSource(backend, NativeEncoderConfig("encoder", 0.5))
    context = TickContext(0, 1_100_000_000)

    snapshot = source.read(context)

    assert snapshot.context == context
    assert snapshot.health.state is DeviceHealthState.OK
    assert snapshot.samples[0].sequence == 0
    assert snapshot.samples[0].captured_monotonic_ns == context.monotonic_ns
    assert tuple(field.value for field in snapshot.samples[0].values) == pytest.approx(
        (0.1, 0.1, 1.0)
    )


def test_baseline_requires_running_counters():
    left = Counter((_snapshot(0),), running=False)
    right = Counter((_snapshot(0),))

    with pytest.raises(OSError, match="must be running"):
        NativeCounterEncoderBackend(
            left,
            right,
            _config(),
            baseline_monotonic_ns=1,
        )

    assert left.calls == 1
    assert right.calls == 1


def test_malformed_snapshot_is_rejected_without_a_second_read():
    class BadCounter:
        running = True

        def __init__(self) -> None:
            self.calls = 0

        def snapshot(self):
            self.calls += 1
            return object()

    left = BadCounter()
    right = Counter((_snapshot(0),))

    with pytest.raises(TypeError, match="SignedPulseCounterSnapshot"):
        NativeCounterEncoderBackend(
            left,  # type: ignore[arg-type]
            right,
            _config(),
            baseline_monotonic_ns=1,
        )

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
