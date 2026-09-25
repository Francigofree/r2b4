"""Regression tests for the 2026-09-15 encoder robustness incident."""

from __future__ import annotations
from v3_config_fixtures import configured

import pytest

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig,
    NativeCounterEncoderBackend,
    SignedPulseCounterSnapshot,
    SignedPulseEdge,
)
from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
    NativeGpioSignedCounterPair,
)
from v3.adapters.live_encoder import EncoderRejectionCode
from v3.contracts import (
    AdmittedFrame,
    DataField,
    Observation,
    TickContext,
    WheelVelocitySetpoint,
)
from v3.layers.l11_actuator_control import (
    WheelActuatorController,
    WheelPiConfig,
    WheelSpeedMap,
)

STEP = 0.000644429262323014
CONFIG = CounterEncoderBackendConfig(
    STEP,
    STEP,
    maximum_sample_interval_ns=100_000_000,
    maximum_abs_velocity_mps=1.5,
    minimum_estimation_pulses=4,
    minimum_estimation_window_ns=40_000_000,
    maximum_estimation_window_ns=160_000_000,
)


class Counter:
    running = True

    def __init__(self, snapshots):
        self._snapshots = iter(snapshots)

    def snapshot(self):
        return next(self._snapshots)


def _snapshot(count, edges=(), *, invalid_alerts=0):
    return SignedPulseCounterSnapshot(
        count,
        invalid_alerts=invalid_alerts,
        edge_history=tuple(edges),
    )


def _edge(timestamp_ns, pulse_count):
    return SignedPulseEdge(timestamp_ns, pulse_count)


def test_short_reversal_candidate_is_untrusted_and_visible_in_diagnostics():
    forward = (
        _edge(970_000_000, 7),
        _edge(980_000_000, 8),
        _edge(990_000_000, 9),
        _edge(1_000_000_000, 10),
    )
    # Only 0.15 ms of opposite-sign evidence: far too short to qualify.
    reversal = forward + (
        _edge(1_000_100_000, 9),
        _edge(1_000_250_000, 8),
    )
    backend = NativeCounterEncoderBackend(
        Counter((_snapshot(10, forward), _snapshot(8, reversal))),
        Counter((_snapshot(10, forward), _snapshot(8, reversal))),
        CONFIG,
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_001_000_000))

    assert reading.diagnostics is not None
    assert reading.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
    assert reading.left_mps == reading.right_mps == 0.0
    assert 0.0 < reading.diagnostics.left_measurement_trust < 1.0
    assert abs(reading.diagnostics.computed_left_mps) > CONFIG.maximum_abs_velocity_mps
    # The untrusted candidate must not be promoted into a velocity-limit fault.
    assert reading.diagnostics.rejection_code is not EncoderRejectionCode.LEFT_VELOCITY_LIMIT_EXCEEDED


def test_stationary_wheel_does_not_stale_the_moving_wheel():
    left_edges = tuple(
        _edge(1_150_000_000 + i * 10_000_000, i + 1)
        for i in range(5)
    )
    backend = NativeCounterEncoderBackend(
        Counter((_snapshot(0), _snapshot(5, left_edges))),
        Counter((_snapshot(0), _snapshot(0))),
        CONFIG,
    )
    backend.read(TickContext(0, 1_000_000_000))

    reading = backend.read(TickContext(1, 1_200_000_000))

    assert reading.stale is False
    assert reading.timing_valid is True
    assert reading.left_mps > 0.0
    assert reading.right_mps == 0.0
    assert reading.diagnostics is not None
    assert reading.diagnostics.right_estimation_timebase is None


class FakeCallback:
    def __init__(self, function):
        self.function = function

    def cancel(self):
        return 0


class FakeGpio:
    RISING_EDGE = 1
    BOTH_EDGES = 3
    SET_PULL_UP = 32

    def __init__(self):
        self.levels = {18: 1, 23: 0}
        self.callbacks = {}

    def gpiochip_open(self, chip):
        return 7

    def gpio_claim_alert(self, *args):
        return 0

    def gpio_set_debounce_micros(self, *args):
        return 0

    def gpio_read(self, handle, pin):
        return self.levels.get(pin, 0)

    def callback(self, handle, pin, edge, function):
        registration = FakeCallback(function)
        self.callbacks[pin] = registration
        return registration

    def gpio_free(self, *args):
        return 0

    def gpiochip_close(self, *args):
        return 0

    def emit(self, pin, level, tick):
        self.callbacks[pin].function(2, pin, level, tick)


def _gpio_config():
    return GpioCounterPairConfig(
        GpioCounterChannelConfig(
            17,
            18,
            forward_b_level=1,
            a_debounce_micros=150,
        ),
        GpioCounterChannelConfig(22, 23, forward_b_level=0),
        gpio_chip=2,
    )


def test_a_direction_uses_b_state_at_physical_pre_debounce_time():
    gpio = FakeGpio()
    owner = NativeGpioSignedCounterPair(gpio, _gpio_config())

    # B changes at 0.900 ms and is delivered first.  A callback arrives at
    # 1.000 ms, but 150 us debounce means physical A happened at 0.850 ms.
    gpio.emit(18, 0, 900_000)
    gpio.emit(17, 1, 1_000_000)

    snapshot = owner.left_counter.snapshot()
    assert snapshot.pulse_count == 1  # initial B=1 was correct at 0.850 ms
    assert snapshot.edge_history[-1].timestamp_ns == 850_000
    owner.close()


def test_late_b_event_is_diagnostic_not_silent_direction_change():
    gpio = FakeGpio()
    owner = NativeGpioSignedCounterPair(gpio, _gpio_config())
    gpio.emit(17, 1, 1_000_000)  # physical A at 0.850 ms

    gpio.emit(18, 0, 800_000)  # delivered late, belongs before committed A

    snapshot = owner.left_counter.snapshot()
    assert snapshot.pulse_count == 1
    assert snapshot.invalid_alerts == 1
    owner.close()


def _speed_map():
    raw = {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "ACTIVE",
        "curves": {
            name: {
                "maintenance_pwm": 0.10,
                "startup_pwm": 0.15,
                "points": [
                    {"speed_mps": 0.10, "pwm": 0.16},
                    {"speed_mps": 0.30, "pwm": 0.30},
                ],
            }
            for name in (
                "left_forward",
                "left_reverse",
                "right_forward",
                "right_reverse",
            )
        },
    }
    return WheelSpeedMap.from_mapping(raw)


def _feedback_frame(
    context,
    *,
    left_trust,
    right_trust,
    left_mps=0.0,
    right_mps=0.0,
    left_timebase="GPIO_EDGE_HISTORY",
    right_timebase="GPIO_EDGE_HISTORY",
    rejection_code="NONE",
):
    observation = Observation(
        kind="wheel_velocity",
        source_device_id="encoder",
        source_sequence=context.tick_id,
        captured_monotonic_ns=context.monotonic_ns,
        values=(
            DataField("left_mps", left_mps),
            DataField("right_mps", right_mps),
            DataField("trust", min(left_trust, right_trust)),
            DataField("left_measurement_trust", left_trust),
            DataField("right_measurement_trust", right_trust),
            DataField("left_estimation_timebase", left_timebase),
            DataField("right_estimation_timebase", right_timebase),
            DataField("measurement_stale", False),
            DataField("measurement_timing_valid", True),
            DataField("rejection_code", rejection_code),
            DataField("left_counter_running", True),
            DataField("right_counter_running", True),
            DataField("left_read_error_delta", 0),
            DataField("right_read_error_delta", 0),
            DataField("left_invalid_alert_delta", 0),
            DataField("right_invalid_alert_delta", 0),
        ),
    )
    return AdmittedFrame(context, (observation,), (), ())


def _pi_config():
    return configured(WheelPiConfig, 
        kp=0.25,
        ki=0.08,
        integrator_limit=0.18,
        max_normalized_output=0.95,
        max_feedback_uncertainty_ns=100_000_000,
    )


def test_l11_baseline_zero_is_feedforward_only_then_time_bounded_fault():
    speed_map = _speed_map()
    controller = WheelActuatorController(speed_map, _pi_config())
    expected_ff = speed_map.lookup("left", 0.15)[0]

    for tick_id in range(5):
        context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
        result = controller(
            WheelVelocitySetpoint(context, 0.15, 0.15),
            _feedback_frame(
                context,
                left_trust=0.0,
                right_trust=0.0,
                rejection_code="BASELINE",
            ),
        )
        assert result.left_normalized == pytest.approx(expected_ff)
        assert result.right_normalized == pytest.approx(expected_ff)
        state = controller.checkpoint()
        assert state.left_integral == state.right_integral == 0.0

    context = TickContext(5, 1_100_000_000)
    with pytest.raises(ValueError, match="uncertain too long"):
        controller(
            WheelVelocitySetpoint(context, 0.15, 0.15),
            _feedback_frame(
                context,
                left_trust=0.0,
                right_trust=0.0,
                rejection_code="BASELINE",
            ),
        )


def test_l11_zero_target_wheel_does_not_consume_uncertainty_budget():
    controller = WheelActuatorController(_speed_map(), _pi_config())

    for tick_id in range(10):
        context = TickContext(tick_id, 2_000_000_000 + tick_id * 20_000_000)
        result = controller(
            WheelVelocitySetpoint(context, 0.15, 0.0),
            _feedback_frame(
                context,
                left_trust=1.0,
                right_trust=0.0,
                left_mps=0.14,
                right_mps=0.0,
                right_timebase="TICK_SNAPSHOT",
                rejection_code="BASELINE",
            ),
        )
        assert result.left_normalized > 0.0
        assert result.right_normalized == 0.0
        assert controller.checkpoint().right_uncertain_since_ns is None
