import json
from pathlib import Path

import pytest

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig,
    NativeCounterEncoderBackend,
    SignedPulseCounterSnapshot,
    SignedPulseEdge,
)
from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
from v3.adapters.live_encoder import EncoderRejectionCode
from v3.contracts import DeviceHealthState, TickContext


PROJECT_ROOT = Path(__file__).resolve().parents[1]


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
    edge_history: tuple[SignedPulseEdge, ...] = (),
) -> SignedPulseCounterSnapshot:
    return SignedPulseCounterSnapshot(
        pulses,
        read_errors,
        invalid_alerts,
        edge_history,
    )


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


def test_adaptive_window_keeps_recent_pulses_instead_of_one_tick_noise():
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
    assert second.left_mps == pytest.approx(0.065)
    assert second.right_mps == pytest.approx(0.07)
    assert second.diagnostics is not None
    assert second.diagnostics.left_pulse_delta == 3
    assert second.diagnostics.left_estimation_pulse_delta == 13
    assert second.diagnostics.left_estimation_window_ns == 200_000_000


def test_delayed_multi_pulse_batch_uses_physical_edge_window_not_tick_window():
    edges = tuple(
        SignedPulseEdge(timestamp_ns, pulse_count)
        for pulse_count, timestamp_ns in enumerate(
            (10_000_000, 20_000_000, 30_000_000, 40_000_000, 50_000_000),
            start=1,
        )
    )
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(5, edge_history=edges)),
        (_snapshot(0), _snapshot(5, edge_history=edges)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_020_000_000))

    assert reading.left_mps == pytest.approx(4 * 0.001 / 0.040)
    assert reading.right_mps == pytest.approx(4 * 0.002 / 0.040)
    assert reading.trust == 1.0
    diagnostics = reading.diagnostics
    assert diagnostics is not None
    assert diagnostics.sample_interval_ns == 20_000_000
    assert diagnostics.left_estimation_timebase == "GPIO_EDGE_HISTORY"
    assert diagnostics.right_estimation_timebase == "GPIO_EDGE_HISTORY"
    assert diagnostics.left_estimation_window_ns == 40_000_000
    assert diagnostics.right_estimation_window_ns == 40_000_000
    assert diagnostics.left_estimation_pulse_delta == 4
    assert diagnostics.right_estimation_pulse_delta == 4
    assert diagnostics.left_estimation_start_edge_timestamp_ns == 10_000_000
    assert diagnostics.left_estimation_end_edge_timestamp_ns == 50_000_000
    assert diagnostics.left_edge_history_count == 5
    assert diagnostics.right_edge_history_count == 5


def test_edge_window_prevents_delayed_batch_from_false_velocity_rejection():
    edges = tuple(
        SignedPulseEdge(pulse_count * 2_000_000, pulse_count)
        for pulse_count in range(1, 51)
    )
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(50, edge_history=edges)),
        (_snapshot(0), _snapshot(50, edge_history=edges)),
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_020_000_000))

    assert reading.diagnostics is not None
    assert reading.diagnostics.instantaneous_left_mps == pytest.approx(2.5)
    assert reading.diagnostics.instantaneous_right_mps == pytest.approx(5.0)
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.NONE
    assert reading.left_mps == pytest.approx(0.5)
    assert reading.right_mps == pytest.approx(1.0)


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
    assert recovered.trust == pytest.approx(0.5)
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
    assert recovered.trust == pytest.approx(0.5)
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
    assert recovered.trust == pytest.approx(0.5)
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

    assert baseline.health.state is DeviceHealthState.OK
    assert baseline.health.reason is None
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
    assert current_values["raw_left_distance_m"] == pytest.approx(0.01)
    assert current_values["raw_right_distance_m"] == pytest.approx(0.01)
    assert current_values["left_distance_delta_m"] == pytest.approx(0.01)
    assert current_values["right_distance_delta_m"] == pytest.approx(0.01)
    assert current_values["computed_left_mps"] == pytest.approx(0.1)
    assert current_values["computed_right_mps"] == pytest.approx(0.1)


def test_concrete_counter_diagnostic_degrades_device_health_separately():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(1, read_errors=1)),
        (_snapshot(0), _snapshot(1)),
    )
    source = NativeEncoderSource(backend, NativeEncoderConfig("encoder", 0.5))
    source.read(TickContext(0, 1_000_000_000))

    snapshot = source.read(TickContext(1, 1_100_000_000))

    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == "ENCODER_COUNTER_DIAGNOSTIC"
    assert _sample_values(snapshot)["trust"] == 0.0


def test_low_speed_window_grows_until_four_pulses_then_stays_stable():
    counts = tuple(_snapshot(value) for value in (0, 1, 1, 2, 2, 3, 3, 4, 4, 5))
    backend, _, _ = _backend(counts, counts)

    readings = [
        backend.read(TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000))
        for tick_id in range(len(counts))
    ]

    assert readings[7].left_mps == pytest.approx(4 * 0.001 / 0.14)
    assert readings[7].trust == 1.0
    assert readings[7].diagnostics is not None
    assert readings[7].diagnostics.left_estimation_window_ns == 140_000_000
    assert readings[8].left_mps == pytest.approx(4 * 0.001 / 0.16)
    assert readings[9].left_mps == pytest.approx(4 * 0.001 / 0.14)
    assert readings[9].diagnostics is not None
    assert readings[9].diagnostics.instantaneous_left_mps == pytest.approx(0.05)
    assert readings[9].diagnostics.left_velocity_uncertainty_mps == pytest.approx(
        0.001 / 0.14
    )


def test_high_speed_window_is_minimal_and_raw_distance_is_never_smoothed():
    backend, _, _ = _backend(
        (_snapshot(0), _snapshot(6), _snapshot(12)),
        (_snapshot(0), _snapshot(5), _snapshot(10)),
    )
    backend.read(TickContext(0, 1_000_000_000))
    backend.read(TickContext(1, 1_020_000_000))

    reading = backend.read(TickContext(2, 1_040_000_000))

    assert reading.left_mps == pytest.approx(0.3)
    assert reading.right_mps == pytest.approx(0.5)
    assert reading.trust == 1.0
    assert reading.diagnostics is not None
    assert reading.diagnostics.left_estimation_window_ns == 40_000_000
    assert reading.diagnostics.right_estimation_window_ns == 40_000_000
    assert reading.diagnostics.left_pulse_delta == 6
    assert reading.diagnostics.right_pulse_delta == 5
    assert reading.diagnostics.raw_left_distance_m == pytest.approx(0.012)
    assert reading.diagnostics.raw_right_distance_m == pytest.approx(0.020)
    assert reading.diagnostics.left_distance_delta_m == pytest.approx(0.006)
    assert reading.diagnostics.right_distance_delta_m == pytest.approx(0.010)


def test_existing_capture_raw_deltas_are_quieter_without_distance_drift():
    physics = json.loads(
        (PROJECT_ROOT / "conf/fizika.json").read_text(encoding="utf-8")
    )
    step_distance_m = float(physics["lepes_hossz_m"])
    rows = tuple(
        json.loads(line)
        for line in (
            PROJECT_ROOT / "tests/fixtures/v3_l0_l4_capture_excerpt.jsonl"
        ).read_text(encoding="utf-8").splitlines()
    )
    left_count = 0
    right_count = 0
    left_snapshots = []
    right_snapshots = []
    for row in rows:
        feedback = row["executor_call"]["kwargs"]["sensor_feedback"]
        left_count += round(
            float(feedback["encoder_left_distance_delta_m"]) / step_distance_m
        )
        right_count += round(
            float(feedback["encoder_right_distance_delta_m"]) / step_distance_m
        )
        left_snapshots.append(_snapshot(left_count))
        right_snapshots.append(_snapshot(right_count))
    backend = NativeCounterEncoderBackend(
        Counter(tuple(left_snapshots)),
        Counter(tuple(right_snapshots)),
        CounterEncoderBackendConfig(
            step_distance_m,
            step_distance_m,
            maximum_sample_interval_ns=100_000_000,
            maximum_abs_velocity_mps=1.5,
            minimum_estimation_pulses=4,
            minimum_estimation_window_ns=40_000_000,
            maximum_estimation_window_ns=160_000_000,
        ),
    )

    readings = tuple(
        backend.read(TickContext(index, int(row["monotonic_ns"])))
        for index, row in enumerate(rows)
    )
    old_left = tuple(
        float(row["executor_call"]["kwargs"]["sensor_feedback"]["v_l_encoder_raw"])
        for row in rows[1:]
    )
    old_right = tuple(
        float(row["executor_call"]["kwargs"]["sensor_feedback"]["v_r_encoder_raw"])
        for row in rows[1:]
    )
    new_left = tuple(reading.left_mps for reading in readings[1:])
    new_right = tuple(reading.right_mps for reading in readings[1:])

    assert max(new_left) - min(new_left) < max(old_left) - min(old_left)
    assert max(new_right) - min(new_right) < max(old_right) - min(old_right)
    diagnostics = readings[-1].diagnostics
    assert diagnostics is not None
    assert diagnostics.raw_left_distance_m == pytest.approx(
        left_count * step_distance_m
    )
    assert diagnostics.raw_right_distance_m == pytest.approx(
        right_count * step_distance_m
    )


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
        {"minimum_estimation_pulses": 0},
        {"minimum_estimation_window_ns": 0},
        {"maximum_estimation_window_ns": 0},
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
