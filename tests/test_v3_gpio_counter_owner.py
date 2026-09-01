from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig,
    NativeCounterEncoderBackend,
)
from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
    NativeGpioSignedCounterPair,
)
from v3.contracts import TickContext


class FakeCallback:
    def __init__(self, function) -> None:
        self.function = function
        self.cancel_calls = 0

    def cancel(self):
        self.cancel_calls += 1
        return 0


class FakeGpio:
    RISING_EDGE = 1
    BOTH_EDGES = 3
    SET_PULL_UP = 32

    def __init__(self, levels=None) -> None:
        self.levels = dict(levels or {})
        self.calls = []
        self.callbacks = {}
        self.open_calls = 0
        self.fail_callback_pin = None

    def gpiochip_open(self, chip):
        self.open_calls += 1
        self.calls.append(("open", chip))
        return 7

    def gpio_claim_alert(self, handle, pin, edge, flags):
        self.calls.append(("claim", handle, pin, edge, flags))
        return 0

    def gpio_set_debounce_micros(self, handle, pin, debounce_micros):
        self.calls.append(("debounce", handle, pin, debounce_micros))
        return 0

    def gpio_read(self, handle, pin):
        self.calls.append(("read", handle, pin))
        return self.levels.get(pin, 0)

    def callback(self, handle, pin, edge, function):
        self.calls.append(("callback", handle, pin, edge))
        if pin == self.fail_callback_pin:
            return object()
        registration = FakeCallback(function)
        self.callbacks[pin] = registration
        return registration

    def gpio_free(self, handle, pin):
        self.calls.append(("free", handle, pin))
        return 0

    def gpiochip_close(self, handle):
        self.calls.append(("close", handle))
        return 0

    def emit(self, pin, level, *, chip=7, delivered_pin=None, tick=1):
        self.callbacks[pin].function(
            chip,
            pin if delivered_pin is None else delivered_pin,
            level,
            tick,
        )


def _config() -> GpioCounterPairConfig:
    return GpioCounterPairConfig(
        left=GpioCounterChannelConfig(
            pin_a=17,
            pin_b=18,
            forward_b_level=1,
            pull_up=True,
            a_debounce_micros=75,
        ),
        right=GpioCounterChannelConfig(
            pin_a=22,
            pin_b=23,
            forward_b_level=0,
            invert=True,
        ),
        gpio_chip=2,
    )


def test_pair_owns_one_handle_and_claims_four_configured_alerts():
    gpio = FakeGpio({18: 1, 23: 0})
    owner = NativeGpioSignedCounterPair(gpio, _config())

    assert gpio.open_calls == 1
    assert [call for call in gpio.calls if call[0] == "claim"] == [
        ("claim", 7, 17, gpio.RISING_EDGE, gpio.SET_PULL_UP),
        ("claim", 7, 18, gpio.BOTH_EDGES, gpio.SET_PULL_UP),
        ("claim", 7, 22, gpio.RISING_EDGE, 0),
        ("claim", 7, 23, gpio.BOTH_EDGES, 0),
    ]
    assert [call for call in gpio.calls if call[0] == "debounce"] == [
        ("debounce", 7, 17, 75)
    ]
    assert [call[2] for call in gpio.calls if call[0] == "callback"] == [
        18,
        17,
        23,
        22,
    ]
    assert [call for call in gpio.calls if call[0] == "read"] == [
        ("read", 7, 18),
        ("read", 7, 23),
    ]
    assert owner.running is True
    assert owner.left_counter.running is True
    assert owner.right_counter.running is True
    assert owner.left_counter.snapshot().pulse_count == 0
    assert owner.right_counter.snapshot().pulse_count == 0

    owner.close()


def test_b_latch_controls_a_rising_direction_and_invert():
    gpio = FakeGpio({18: 1, 23: 0})
    owner = NativeGpioSignedCounterPair(gpio, _config())

    gpio.emit(17, 1)
    gpio.emit(18, 0)
    gpio.emit(17, 1)
    gpio.emit(22, 1)
    gpio.emit(23, 1)
    gpio.emit(22, 1)

    assert owner.left_counter.snapshot().pulse_count == 0
    assert owner.right_counter.snapshot().pulse_count == 0
    assert owner.left_counter.snapshot().invalid_alerts == 0
    assert owner.right_counter.snapshot().invalid_alerts == 0

    owner.close()


def test_malformed_or_misrouted_callbacks_are_diagnostic_not_motion():
    gpio = FakeGpio({18: 1, 23: 0})
    owner = NativeGpioSignedCounterPair(gpio, _config())

    gpio.emit(17, 0)
    gpio.emit(18, 2)
    gpio.emit(22, 1, chip=8)
    gpio.emit(23, 1, delivered_pin=99)

    left = owner.left_counter.snapshot()
    right = owner.right_counter.snapshot()
    assert left.pulse_count == 0
    assert left.invalid_alerts == 2
    assert right.pulse_count == 0
    assert right.invalid_alerts == 2

    owner.close()


def test_counter_snapshot_is_an_immutable_value():
    gpio = FakeGpio({18: 1, 23: 0})
    owner = NativeGpioSignedCounterPair(gpio, _config())
    snapshot = owner.left_counter.snapshot()

    with pytest.raises(FrozenInstanceError):
        snapshot.pulse_count = 9

    owner.close()


def test_close_is_idempotent_and_stops_callbacks_before_releasing_resources():
    gpio = FakeGpio({18: 1, 23: 0})
    owner = NativeGpioSignedCounterPair(gpio, _config())
    registrations = tuple(gpio.callbacks.values())
    late_a_callback = gpio.callbacks[17].function

    owner.close()
    late_a_callback(7, 17, 1, 99)
    owner.close()

    assert owner.running is False
    assert owner.closed is True
    assert owner.failed is False
    assert owner.left_counter.running is False
    assert owner.left_counter.snapshot().pulse_count == 0
    assert all(callback.cancel_calls == 1 for callback in registrations)
    assert [call for call in gpio.calls if call[0] == "free"] == [
        ("free", 7, 23),
        ("free", 7, 22),
        ("free", 7, 18),
        ("free", 7, 17),
    ]
    assert [call for call in gpio.calls if call[0] == "close"] == [("close", 7)]


def test_partial_callback_failure_cancels_and_frees_every_acquired_resource():
    gpio = FakeGpio({18: 1, 23: 0})
    gpio.fail_callback_pin = 17

    with pytest.raises(TypeError, match="provide cancel"):
        NativeGpioSignedCounterPair(gpio, _config())

    assert gpio.callbacks[18].cancel_calls == 1
    assert [call for call in gpio.calls if call[0] == "free"] == [
        ("free", 7, 23),
        ("free", 7, 22),
        ("free", 7, 18),
        ("free", 7, 17),
    ]
    assert [call for call in gpio.calls if call[0] == "close"] == [("close", 7)]


@pytest.mark.parametrize(
    "config",
    (
        GpioCounterPairConfig(
            GpioCounterChannelConfig(1, 2),
            GpioCounterChannelConfig(3, 4),
        ),
    ),
)
def test_invalid_backend_is_rejected_before_open(config):
    class MissingCallbackBackend(FakeGpio):
        callback = None

    gpio = MissingCallbackBackend()

    with pytest.raises(TypeError, match="callable callback"):
        NativeGpioSignedCounterPair(gpio, config)

    assert gpio.open_calls == 0


@pytest.mark.parametrize(
    "left,right",
    (
        (GpioCounterChannelConfig(1, 2), GpioCounterChannelConfig(2, 3)),
        (GpioCounterChannelConfig(1, 2), GpioCounterChannelConfig(3, 1)),
    ),
)
def test_pair_config_rejects_cross_channel_pin_aliases(left, right):
    with pytest.raises(ValueError, match="must be unique"):
        GpioCounterPairConfig(left, right)


def test_native_counter_backend_consumes_the_two_owned_snapshot_views():
    gpio = FakeGpio({18: 1, 23: 1})
    owner = NativeGpioSignedCounterPair(gpio, _config())
    backend = NativeCounterEncoderBackend(
        owner.left_counter,
        owner.right_counter,
        CounterEncoderBackendConfig(
            left_step_distance_m=0.001,
            right_step_distance_m=0.001,
            maximum_sample_interval_ns=200_000_000,
            maximum_abs_velocity_mps=1.0,
        ),
    )

    baseline = backend.read(TickContext(0, 1_000_000_000))
    for _ in range(10):
        gpio.emit(17, 1)
    for _ in range(5):
        gpio.emit(22, 1)
    reading = backend.read(TickContext(1, 1_100_000_000))

    assert baseline.trust == 0.0
    assert baseline.left_mps == 0.0
    assert baseline.right_mps == 0.0
    assert reading.trust == 1.0
    assert reading.left_mps == pytest.approx(0.1)
    assert reading.right_mps == pytest.approx(0.05)

    owner.close()
