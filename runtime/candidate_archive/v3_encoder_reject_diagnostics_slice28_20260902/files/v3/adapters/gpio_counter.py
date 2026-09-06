"""Native paired GPIO owner for two signed V3 pulse counters."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Protocol

from .counter_encoder import SignedPulseCounterSnapshot


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _bool(value: object, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name} must be bool")
    return value


@dataclass(frozen=True, slots=True)
class GpioCounterChannelConfig:
    """Immutable pins and quadrature interpretation for one wheel counter."""

    pin_a: int
    pin_b: int
    forward_b_level: int = 1
    invert: bool = False
    pull_up: bool = False
    a_debounce_micros: int = 0

    def __post_init__(self) -> None:
        _nonnegative_int(self.pin_a, "pin_a")
        _nonnegative_int(self.pin_b, "pin_b")
        if self.pin_a == self.pin_b:
            raise ValueError("pin_a and pin_b must be distinct")
        if (
            not isinstance(self.forward_b_level, int)
            or isinstance(self.forward_b_level, bool)
            or self.forward_b_level not in (0, 1)
        ):
            raise ValueError("forward_b_level must be 0 or 1")
        _bool(self.invert, "invert")
        _bool(self.pull_up, "pull_up")
        _nonnegative_int(self.a_debounce_micros, "a_debounce_micros")


@dataclass(frozen=True, slots=True)
class GpioCounterPairConfig:
    """Immutable ownership for two channels on exactly one GPIO chip."""

    left: GpioCounterChannelConfig
    right: GpioCounterChannelConfig
    gpio_chip: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.left, GpioCounterChannelConfig):
            raise TypeError("left must be GpioCounterChannelConfig")
        if not isinstance(self.right, GpioCounterChannelConfig):
            raise TypeError("right must be GpioCounterChannelConfig")
        _nonnegative_int(self.gpio_chip, "gpio_chip")
        if len(set(self.pins)) != 4:
            raise ValueError("left and right counter GPIO pins must be unique")

    @property
    def pins(self) -> tuple[int, int, int, int]:
        return (
            self.left.pin_a,
            self.left.pin_b,
            self.right.pin_a,
            self.right.pin_b,
        )


class GpioAlertCallback(Protocol):
    """One cancellable callback registration returned by the GPIO backend."""

    def cancel(self) -> object: ...


class GpioCounterBackend(Protocol):
    """Injected lgpio-style operations; the concrete module stays outside V3."""

    RISING_EDGE: int
    BOTH_EDGES: int
    SET_PULL_UP: int

    def gpiochip_open(self, chip: int) -> int: ...

    def gpio_claim_alert(
        self,
        handle: int,
        pin: int,
        edge: int,
        flags: int,
    ) -> object: ...

    def gpio_set_debounce_micros(
        self,
        handle: int,
        pin: int,
        debounce_micros: int,
    ) -> object: ...

    def gpio_read(self, handle: int, pin: int) -> int: ...

    def callback(
        self,
        handle: int,
        pin: int,
        edge: int,
        function: Callable[[int, int, int, int], None],
    ) -> GpioAlertCallback: ...

    def gpio_free(self, handle: int, pin: int) -> object: ...

    def gpiochip_close(self, handle: int) -> object: ...


@dataclass(slots=True)
class _CounterState:
    pulse_count: int = 0
    read_errors: int = 0
    invalid_alerts: int = 0
    level_b: int = 0


class _CounterView:
    """Capability-limited immutable snapshot view for one owned channel."""

    __slots__ = ("_owner", "_side")

    def __init__(self, owner: NativeGpioSignedCounterPair, side: str) -> None:
        self._owner = owner
        self._side = side

    @property
    def running(self) -> bool:
        return self._owner.running

    def snapshot(self) -> SignedPulseCounterSnapshot:
        return self._owner._snapshot(self._side)


class NativeGpioSignedCounterPair:
    """Own one GPIO handle and expose two signed pulse-counter views.

    Each B alert latches direction state. Each A rising alert increments or
    decrements its signed counter under the same lock. No worker, clock,
    velocity policy or concrete GPIO module is owned here.
    """

    __slots__ = (
        "_backend",
        "_callbacks",
        "_claimed_pins",
        "_closed",
        "_config",
        "_failed",
        "_handle",
        "_left_counter",
        "_lock",
        "_right_counter",
        "_running",
        "_states",
    )

    def __init__(
        self,
        backend: GpioCounterBackend,
        config: GpioCounterPairConfig,
    ) -> None:
        if not isinstance(config, GpioCounterPairConfig):
            raise TypeError("config must be GpioCounterPairConfig")
        self._validate_backend(backend)

        self._backend = backend
        self._config = config
        self._lock = threading.Lock()
        self._states = {"left": _CounterState(), "right": _CounterState()}
        self._callbacks: list[GpioAlertCallback] = []
        self._claimed_pins: list[int] = []
        self._running = False
        self._closed = False
        self._failed = False
        self._left_counter = _CounterView(self, "left")
        self._right_counter = _CounterView(self, "right")
        self._handle = self._open_handle()
        try:
            self._initialize()
        except Exception:
            self._failed = True
            self._closed = True
            self._release_resources(suppress_errors=True)
            raise
        self._running = True

    @staticmethod
    def _validate_backend(backend: GpioCounterBackend) -> None:
        for method_name in (
            "gpiochip_open",
            "gpio_claim_alert",
            "gpio_set_debounce_micros",
            "gpio_read",
            "callback",
            "gpio_free",
            "gpiochip_close",
        ):
            if not callable(getattr(backend, method_name, None)):
                raise TypeError(
                    f"backend must provide a callable {method_name} method"
                )
        for constant_name in ("RISING_EDGE", "BOTH_EDGES", "SET_PULL_UP"):
            value = getattr(backend, constant_name, None)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise TypeError(
                    f"backend {constant_name} must be a non-negative integer"
                )

    @staticmethod
    def _checked_call(result: object, operation: str) -> None:
        if isinstance(result, int) and not isinstance(result, bool) and result < 0:
            raise OSError(f"{operation} failed with status {result}")

    def _open_handle(self) -> int:
        handle = self._backend.gpiochip_open(self._config.gpio_chip)
        if not isinstance(handle, int) or isinstance(handle, bool) or handle < 0:
            raise OSError("opening the configured GPIO chip failed")
        return handle

    def _initialize(self) -> None:
        channels = (("left", self._config.left), ("right", self._config.right))
        for _, channel in channels:
            flags = self._backend.SET_PULL_UP if channel.pull_up else 0
            for pin, edge in (
                (channel.pin_a, self._backend.RISING_EDGE),
                (channel.pin_b, self._backend.BOTH_EDGES),
            ):
                self._checked_call(
                    self._backend.gpio_claim_alert(
                        self._handle,
                        pin,
                        edge,
                        flags,
                    ),
                    f"claim GPIO alert {pin}",
                )
                self._claimed_pins.append(pin)

        for side, channel in channels:
            if channel.a_debounce_micros:
                self._checked_call(
                    self._backend.gpio_set_debounce_micros(
                        self._handle,
                        channel.pin_a,
                        channel.a_debounce_micros,
                    ),
                    f"set GPIO debounce {channel.pin_a}",
                )
            self._callbacks.append(
                self._register_callback(
                    channel.pin_b,
                    self._backend.BOTH_EDGES,
                    self._b_handler(side, channel.pin_b),
                )
            )
            with self._lock:
                self._states[side].level_b = self._read_level(channel.pin_b)
            self._callbacks.append(
                self._register_callback(
                    channel.pin_a,
                    self._backend.RISING_EDGE,
                    self._a_handler(side, channel),
                )
            )

    def _read_level(self, pin: int) -> int:
        level = self._backend.gpio_read(self._handle, pin)
        if not isinstance(level, int) or isinstance(level, bool) or level not in (0, 1):
            raise OSError(f"reading GPIO {pin} did not return a digital level")
        return level

    def _register_callback(
        self,
        pin: int,
        edge: int,
        function: Callable[[int, int, int, int], None],
    ) -> GpioAlertCallback:
        callback = self._backend.callback(self._handle, pin, edge, function)
        if not callable(getattr(callback, "cancel", None)):
            raise TypeError("GPIO callback registration must provide cancel")
        return callback

    def _valid_alert(self, chip: object, gpio: object, expected_pin: int) -> bool:
        # lgpio callback payloads identify the gpiochip device number, not the
        # opaque handle returned by gpiochip_open().  Backend operations still
        # use the handle; only asynchronous alert lineage uses gpio_chip.
        return (
            isinstance(chip, int)
            and not isinstance(chip, bool)
            and chip == self._config.gpio_chip
            and isinstance(gpio, int)
            and not isinstance(gpio, bool)
            and gpio == expected_pin
        )

    def _b_handler(
        self,
        side: str,
        expected_pin: int,
    ) -> Callable[[int, int, int, int], None]:
        def handle(chip: int, gpio: int, level: int, tick: int) -> None:
            del tick
            with self._lock:
                if self._closed:
                    return
                state = self._states[side]
                if (
                    not self._valid_alert(chip, gpio, expected_pin)
                    or not isinstance(level, int)
                    or isinstance(level, bool)
                    or level not in (0, 1)
                ):
                    state.invalid_alerts += 1
                    return
                state.level_b = level

        return handle

    def _a_handler(
        self,
        side: str,
        channel: GpioCounterChannelConfig,
    ) -> Callable[[int, int, int, int], None]:
        def handle(chip: int, gpio: int, level: int, tick: int) -> None:
            del tick
            with self._lock:
                if not self._running:
                    return
                state = self._states[side]
                if (
                    not self._valid_alert(chip, gpio, channel.pin_a)
                    or not isinstance(level, int)
                    or isinstance(level, bool)
                    or level != 1
                ):
                    state.invalid_alerts += 1
                    return
                direction = 1 if state.level_b == channel.forward_b_level else -1
                if channel.invert:
                    direction = -direction
                state.pulse_count += direction

        return handle

    @property
    def left_counter(self) -> _CounterView:
        return self._left_counter

    @property
    def right_counter(self) -> _CounterView:
        return self._right_counter

    @property
    def running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed

    @property
    def failed(self) -> bool:
        with self._lock:
            return self._failed

    def _snapshot(self, side: str) -> SignedPulseCounterSnapshot:
        with self._lock:
            state = self._states[side]
            return SignedPulseCounterSnapshot(
                pulse_count=state.pulse_count,
                read_errors=state.read_errors,
                invalid_alerts=state.invalid_alerts,
            )

    def _release_resources(self, *, suppress_errors: bool) -> None:
        first_error: Exception | None = None
        for callback in reversed(self._callbacks):
            try:
                self._checked_call(callback.cancel(), "cancel GPIO callback")
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self._callbacks.clear()
        for pin in reversed(self._claimed_pins):
            try:
                self._checked_call(
                    self._backend.gpio_free(self._handle, pin),
                    f"free GPIO {pin}",
                )
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self._claimed_pins.clear()
        try:
            self._checked_call(
                self._backend.gpiochip_close(self._handle),
                "close GPIO chip",
            )
        except Exception as exc:
            if first_error is None:
                first_error = exc
        if first_error is not None and not suppress_errors:
            raise first_error

    def close(self) -> None:
        """Stop alert mutation and release every GPIO resource exactly once."""

        with self._lock:
            if self._closed:
                return
            self._running = False
            self._closed = True
        try:
            self._release_resources(suppress_errors=False)
        except Exception:
            with self._lock:
                self._failed = True
            raise


__all__ = [
    "GpioAlertCallback",
    "GpioCounterBackend",
    "GpioCounterChannelConfig",
    "GpioCounterPairConfig",
    "NativeGpioSignedCounterPair",
]
