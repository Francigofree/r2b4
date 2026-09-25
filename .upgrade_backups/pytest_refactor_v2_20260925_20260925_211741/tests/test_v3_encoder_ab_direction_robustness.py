"""Regression coverage for the live false-direction encoder incident."""
from __future__ import annotations

from dataclasses import asdict

import pytest

from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
    NativeGpioSignedCounterPair,
)


class FakeCallback:
    def __init__(self, function):
        self.function = function
    def cancel(self):
        return 0


class FakeGpio:
    RISING_EDGE = 1
    BOTH_EDGES = 3
    SET_PULL_UP = 32

    def __init__(self, levels=None):
        self.levels = dict(levels or {})
        self.callbacks = {}
    def gpiochip_open(self, chip): return 7
    def gpio_claim_alert(self, handle, pin, edge, flags): return 0
    def gpio_set_debounce_micros(self, handle, pin, value): return 0
    def gpio_read(self, handle, pin): return self.levels.get(pin, 0)
    def callback(self, handle, pin, edge, function):
        cb = FakeCallback(function)
        self.callbacks[pin] = cb
        return cb
    def gpio_free(self, handle, pin): return 0
    def gpiochip_close(self, handle): return 0
    def emit(self, pin, level, tick, *, chip=2):
        self.levels[pin] = level
        self.callbacks[pin].function(chip, pin, level, tick)


def robust_config(*, guard_us=50, confirm_edges=3):
    return GpioCounterPairConfig(
        left=GpioCounterChannelConfig(
            17, 18,
            forward_b_level=1,
            a_debounce_micros=150,
            direction_guard_micros=guard_us,
            direction_change_confirm_edges=confirm_edges,
            direction_change_confirm_window_micros=250_000,
        ),
        right=GpioCounterChannelConfig(22, 23, forward_b_level=1),
        gpio_chip=2,
    )


def a(gpio, physical_ns, *, pin=17):
    # lgpio callback time includes the configured 150 us debounce; production
    # code reconstructs the physical A edge by subtracting it.
    gpio.emit(pin, 1, physical_ns + 150_000)


def test_one_false_opposite_a_candidate_never_enters_signed_edge_history():
    gpio = FakeGpio({18: 1, 23: 1})
    owner = NativeGpioSignedCounterPair(gpio, robust_config())
    a(gpio, 1_000_000)
    a(gpio, 2_000_000)
    assert owner.left_counter.snapshot().pulse_count == 2

    gpio.emit(18, 0, 2_600_000)
    a(gpio, 3_000_000)  # one reverse candidate: pending only
    snap = owner.left_counter.snapshot()
    assert snap.pulse_count == 2
    assert snap.pending_direction == -1
    assert snap.pending_direction_edges == 1

    gpio.emit(18, 1, 3_600_000)
    a(gpio, 4_000_000)  # forward returns; pending reverse is discarded
    snap = owner.left_counter.snapshot()
    assert snap.pulse_count == 3
    assert snap.quadrature_rejections == 1
    assert snap.confirmed_direction == 1
    assert snap.pending_direction_edges == 0
    assert tuple(edge.pulse_count for edge in snap.edge_history) == (1, 2, 3)
    owner.close()


def test_two_false_opposite_candidates_do_not_create_direction_boundary():
    gpio = FakeGpio({18: 1, 23: 1})
    owner = NativeGpioSignedCounterPair(gpio, robust_config())
    a(gpio, 1_000_000)
    a(gpio, 2_000_000)
    gpio.emit(18, 0, 2_600_000)
    a(gpio, 3_000_000)
    a(gpio, 4_000_000)
    assert owner.left_counter.snapshot().pulse_count == 2
    gpio.emit(18, 1, 4_600_000)
    a(gpio, 5_000_000)
    snap = owner.left_counter.snapshot()
    assert snap.pulse_count == 3
    assert snap.quadrature_rejections == 2
    assert tuple(edge.pulse_count for edge in snap.edge_history) == (1, 2, 3)
    owner.close()


def test_three_consistent_opposite_candidates_confirm_real_reversal_retroactively():
    gpio = FakeGpio({18: 1, 23: 1})
    owner = NativeGpioSignedCounterPair(gpio, robust_config())
    a(gpio, 1_000_000)
    a(gpio, 2_000_000)
    gpio.emit(18, 0, 2_600_000)
    a(gpio, 3_000_000)
    a(gpio, 4_000_000)
    assert owner.left_counter.snapshot().pulse_count == 2
    a(gpio, 5_000_000)
    snap = owner.left_counter.snapshot()
    assert snap.pulse_count == -1
    assert snap.confirmed_direction == -1
    assert snap.direction_change_candidates == 3
    assert snap.direction_changes_confirmed == 1
    assert snap.quadrature_rejections == 0
    assert tuple(edge.timestamp_ns for edge in snap.edge_history[-3:]) == (
        3_000_000, 4_000_000, 5_000_000,
    )
    assert tuple(edge.pulse_count for edge in snap.edge_history[-3:]) == (1, 0, -1)
    owner.close()


def test_b_transition_inside_guard_rejects_a_without_hard_gpio_error():
    gpio = FakeGpio({18: 1, 23: 1})
    owner = NativeGpioSignedCounterPair(gpio, robust_config(guard_us=50))
    a(gpio, 1_000_000)
    gpio.emit(18, 0, 1_975_000)  # 25 us before physical A
    a(gpio, 2_000_000)
    snap = owner.left_counter.snapshot()
    assert snap.pulse_count == 1
    assert snap.invalid_alerts == 0
    assert snap.quadrature_rejections == 1
    events = owner.diagnostic_events("left")
    assert events[-1].channel == "A"
    assert events[-1].accepted is False
    assert events[-1].reason == "B_UNSTABLE_BEFORE_A"
    owner.close()


def test_raw_ab_diagnostic_trace_preserves_callback_and_physical_a_time():
    gpio = FakeGpio({18: 1, 23: 1})
    owner = NativeGpioSignedCounterPair(gpio, robust_config())
    gpio.emit(18, 0, 900_000)
    a(gpio, 1_000_000)
    events = owner.diagnostic_events("left")
    assert events[0].channel == "B"
    assert events[-1].channel == "A"
    assert events[-1].callback_timestamp_ns == 1_150_000
    assert events[-1].physical_timestamp_ns == 1_000_000
    assert events[-1].candidate_direction == -1
    # dataclass stays serializable for the motor-output-free probe.
    assert asdict(events[-1])["reason"]
    owner.close()


def test_generic_defaults_preserve_existing_one_edge_reversal_semantics():
    cfg = GpioCounterChannelConfig(1, 2)
    assert cfg.direction_guard_micros == 0
    assert cfg.direction_change_confirm_edges == 1
    assert cfg.direction_change_confirm_window_micros == 250_000


@pytest.mark.parametrize("bad", (0, 9))
def test_direction_confirmation_count_is_bounded(bad):
    with pytest.raises(ValueError, match="direction_change_confirm_edges"):
        GpioCounterChannelConfig(1, 2, direction_change_confirm_edges=bad)
