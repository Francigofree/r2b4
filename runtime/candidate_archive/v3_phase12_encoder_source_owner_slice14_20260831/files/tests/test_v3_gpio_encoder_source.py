import pytest

from v3.adapters.counter_encoder import CounterEncoderBackendConfig
from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
)
from v3.adapters.gpio_encoder import NativeGpioEncoderSource
from v3.adapters.live_encoder import NativeEncoderConfig, NativeEncoderSource
from v3.contracts import DeviceHealthState, TickContext


class Callback:
    def __init__(self, function) -> None:
        self.function = function
        self.cancel_calls = 0

    def cancel(self) -> int:
        self.cancel_calls += 1
        return 0


class GpioBackend:
    RISING_EDGE = 1
    BOTH_EDGES = 2
    SET_PULL_UP = 4

    def __init__(self, *, fail_callback_call: int | None = None) -> None:
        self.fail_callback_call = fail_callback_call
        self.open_calls = 0
        self.close_calls = 0
        self.claimed: list[int] = []
        self.freed: list[int] = []
        self.callbacks: dict[int, Callback] = {}
        self.callback_calls = 0
        self.levels = {11: 1, 13: 1}
        self.handle = 17

    def gpiochip_open(self, chip: int) -> int:
        assert chip == 0
        self.open_calls += 1
        return self.handle

    def gpio_claim_alert(self, handle: int, pin: int, edge: int, flags: int) -> int:
        assert handle == self.handle
        assert edge in (self.RISING_EDGE, self.BOTH_EDGES)
        assert flags in (0, self.SET_PULL_UP)
        self.claimed.append(pin)
        return 0

    def gpio_set_debounce_micros(
        self,
        handle: int,
        pin: int,
        debounce_micros: int,
    ) -> int:
        assert handle == self.handle
        assert pin in (10, 12)
        assert debounce_micros == 50
        return 0

    def gpio_read(self, handle: int, pin: int) -> int:
        assert handle == self.handle
        return self.levels[pin]

    def callback(self, handle: int, pin: int, edge: int, function) -> Callback:
        assert handle == self.handle
        assert edge in (self.RISING_EDGE, self.BOTH_EDGES)
        self.callback_calls += 1
        if self.callback_calls == self.fail_callback_call:
            raise OSError("callback registration failed")
        callback = Callback(function)
        self.callbacks[pin] = callback
        return callback

    def gpio_free(self, handle: int, pin: int) -> int:
        assert handle == self.handle
        self.freed.append(pin)
        return 0

    def gpiochip_close(self, handle: int) -> int:
        assert handle == self.handle
        self.close_calls += 1
        return 0

    def fire(self, pin: int, level: int) -> None:
        self.callbacks[pin].function(self.handle, pin, level, 123)


def _counter_config() -> GpioCounterPairConfig:
    return GpioCounterPairConfig(
        left=GpioCounterChannelConfig(
            pin_a=10,
            pin_b=11,
            pull_up=True,
            a_debounce_micros=50,
        ),
        right=GpioCounterChannelConfig(
            pin_a=12,
            pin_b=13,
            pull_up=True,
            a_debounce_micros=50,
        ),
    )


def _backend_config() -> CounterEncoderBackendConfig:
    return CounterEncoderBackendConfig(
        left_step_distance_m=0.01,
        right_step_distance_m=0.02,
        maximum_sample_interval_ns=200_000_000,
        maximum_abs_velocity_mps=1.0,
    )


def _source(backend: GpioBackend) -> NativeGpioEncoderSource:
    return NativeGpioEncoderSource(
        backend,
        _counter_config(),
        _backend_config(),
        NativeEncoderConfig("encoder", 0.5),
    )


def test_owned_source_closes_gpio_counts_and_emits_one_typed_snapshot_per_tick():
    gpio = GpioBackend()
    source = _source(gpio)

    assert isinstance(source, NativeEncoderSource)
    baseline = source.read(TickContext(0, 1_000_000_000))
    gpio.fire(10, 1)
    gpio.fire(12, 1)
    current = source.read(TickContext(1, 1_100_000_000))

    assert gpio.open_calls == 1
    assert baseline.health.state is DeviceHealthState.DEGRADED
    assert baseline.health.reason == "ENCODER_LOW_TRUST"
    assert current.health.state is DeviceHealthState.OK
    assert current.context == TickContext(1, 1_100_000_000)
    assert tuple(field.value for field in current.samples[0].values) == pytest.approx(
        (0.1, 0.2, 1.0)
    )

    callbacks = tuple(gpio.callbacks.values())
    source.close()
    source.close()

    assert source.closed is True
    assert gpio.close_calls == 1
    assert gpio.freed == [13, 12, 11, 10]
    assert all(callback.cancel_calls == 1 for callback in callbacks)
    with pytest.raises(RuntimeError, match="source is closed"):
        source.read(TickContext(2, 1_200_000_000))


@pytest.mark.parametrize(
    ("argument", "message"),
    (
        ("counter", "counter_config"),
        ("backend", "backend_config"),
        ("source", "source_config"),
    ),
)
def test_all_typed_configs_are_rejected_before_gpio_open(argument, message):
    gpio = GpioBackend()
    values = {
        "counter_config": _counter_config(),
        "backend_config": _backend_config(),
        "source_config": NativeEncoderConfig("encoder", 0.5),
    }
    values[f"{argument}_config"] = object()

    with pytest.raises(TypeError, match=message):
        NativeGpioEncoderSource(gpio, **values)  # type: ignore[arg-type]

    assert gpio.open_calls == 0


def test_partial_gpio_construction_failure_releases_every_owned_resource():
    gpio = GpioBackend(fail_callback_call=2)

    with pytest.raises(OSError, match="callback registration failed"):
        _source(gpio)

    assert gpio.open_calls == 1
    assert gpio.close_calls == 1
    assert gpio.freed == [13, 12, 11, 10]
    assert gpio.callbacks[11].cancel_calls == 1
