"""Native paired GPIO owner for two signed V3 pulse counters."""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass
from typing import Callable, Protocol

from .counter_encoder import SignedPulseCounterSnapshot, SignedPulseEdge


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
    # Around an A physical edge, B must be stable for this interval.  The
    # production config uses 50 us; zero preserves the generic X1 primitive.
    direction_guard_micros: int = 0
    # A genuine sign reversal must persist for this many A-rising candidates
    # before it is committed to signed edge history.
    direction_change_confirm_edges: int = 1
    direction_change_confirm_window_micros: int = 250_000

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
        _nonnegative_int(self.direction_guard_micros, "direction_guard_micros")
        confirm_edges = _nonnegative_int(
            self.direction_change_confirm_edges,
            "direction_change_confirm_edges",
        )
        if not 1 <= confirm_edges <= 8:
            raise ValueError("direction_change_confirm_edges must be within [1, 8]")
        confirm_window = _nonnegative_int(
            self.direction_change_confirm_window_micros,
            "direction_change_confirm_window_micros",
        )
        if confirm_window == 0:
            raise ValueError("direction_change_confirm_window_micros must be positive")


@dataclass(frozen=True, slots=True)
class GpioCounterPairConfig:
    """Immutable ownership for two channels on exactly one GPIO chip."""

    left: GpioCounterChannelConfig
    right: GpioCounterChannelConfig
    gpio_chip: int = 0
    edge_history_capacity: int = 512
    diagnostic_event_capacity: int = 2048

    def __post_init__(self) -> None:
        if not isinstance(self.left, GpioCounterChannelConfig):
            raise TypeError("left must be GpioCounterChannelConfig")
        if not isinstance(self.right, GpioCounterChannelConfig):
            raise TypeError("right must be GpioCounterChannelConfig")
        _nonnegative_int(self.gpio_chip, "gpio_chip")
        capacity = _nonnegative_int(
            self.edge_history_capacity,
            "edge_history_capacity",
        )
        if not 2 <= capacity <= 4096:
            raise ValueError("edge_history_capacity must be within [2, 4096]")
        diagnostic_capacity = _nonnegative_int(
            self.diagnostic_event_capacity,
            "diagnostic_event_capacity",
        )
        if not 16 <= diagnostic_capacity <= 16384:
            raise ValueError("diagnostic_event_capacity must be within [16, 16384]")
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


@dataclass(frozen=True, slots=True)
class QuadratureAlertEvent:
    """One bounded raw A/B callback decision for offline diagnosis."""

    channel: str
    callback_timestamp_ns: int
    physical_timestamp_ns: int | None
    level: int
    accepted: bool
    reason: str
    candidate_direction: int | None
    pulse_count_after: int

    def __post_init__(self) -> None:
        if self.channel not in ("A", "B"):
            raise ValueError("channel must be A or B")
        _nonnegative_int(self.callback_timestamp_ns, "callback_timestamp_ns")
        if self.physical_timestamp_ns is not None:
            _nonnegative_int(self.physical_timestamp_ns, "physical_timestamp_ns")
        if self.level not in (0, 1):
            raise ValueError("level must be 0 or 1")
        _bool(self.accepted, "accepted")
        if not isinstance(self.reason, str) or not self.reason:
            raise ValueError("reason must be a non-empty string")
        if self.candidate_direction not in (None, -1, 1):
            raise ValueError("candidate_direction must be -1, 1 or None")
        if not isinstance(self.pulse_count_after, int) or isinstance(self.pulse_count_after, bool):
            raise ValueError("pulse_count_after must be an integer")


@dataclass(slots=True)
class _CounterState:
    pulse_count: int = 0
    read_errors: int = 0
    invalid_alerts: int = 0
    initial_level_b: int = 0
    level_b: int = 0
    edge_history: deque[SignedPulseEdge] | None = None
    b_history: deque[tuple[int, int]] | None = None
    last_a_timestamp_ns: int | None = None
    last_b_timestamp_ns: int | None = None
    last_b_level: int | None = None
    confirmed_direction: int = 0
    pending_direction: int = 0
    pending_a_timestamps: list[int] | None = None
    quadrature_rejections: int = 0
    direction_change_candidates: int = 0
    direction_changes_confirmed: int = 0
    diagnostic_events: deque[QuadratureAlertEvent] | None = None


class _CounterView:
    """Capability-limited immutable snapshot view for one owned channel."""

    __slots__ = ("_owner", "_side")

    def __init__(self, owner: "NativeGpioSignedCounterPair", side: str) -> None:
        self._owner = owner
        self._side = side

    @property
    def running(self) -> bool:
        return self._owner.running

    def snapshot(self) -> SignedPulseCounterSnapshot:
        return self._owner._snapshot(self._side)


class NativeGpioSignedCounterPair:
    """Own one GPIO handle and expose two signed pulse-counter views.

    Direction is resolved against the *physical* A-edge time, not simply the
    last B callback delivered by Python.  B transitions are kept in a bounded
    timestamped history.  The A callback timestamp is moved back by the
    configured lgpio debounce delay before direction lookup and before it is
    written to encoder edge history.

    A late B callback whose timestamp belongs at or before an already committed
    A edge is diagnostic evidence of callback-order ambiguity.  It is rejected
    and increments ``invalid_alerts`` so the next encoder read fails closed.
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
        self._states = {
            side: _CounterState(
                edge_history=deque(maxlen=config.edge_history_capacity),
                b_history=deque(maxlen=config.edge_history_capacity),
                pending_a_timestamps=[],
                diagnostic_events=deque(maxlen=config.diagnostic_event_capacity),
            )
            for side in ("left", "right")
        }
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
                initial = self._read_level(channel.pin_b)
                state = self._states[side]
                state.initial_level_b = initial
                state.level_b = initial
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
        # opaque handle returned by gpiochip_open().
        return (
            isinstance(chip, int)
            and not isinstance(chip, bool)
            and chip == self._config.gpio_chip
            and isinstance(gpio, int)
            and not isinstance(gpio, bool)
            and gpio == expected_pin
        )

    @staticmethod
    def _valid_tick(tick: object) -> bool:
        return (
            isinstance(tick, int)
            and not isinstance(tick, bool)
            and tick >= 0
        )

    @staticmethod
    def _record_event(
        state: _CounterState,
        *,
        channel: str,
        callback_timestamp_ns: int,
        physical_timestamp_ns: int | None,
        level: int,
        accepted: bool,
        reason: str,
        candidate_direction: int | None = None,
    ) -> None:
        assert state.diagnostic_events is not None
        state.diagnostic_events.append(
            QuadratureAlertEvent(
                channel=channel,
                callback_timestamp_ns=callback_timestamp_ns,
                physical_timestamp_ns=physical_timestamp_ns,
                level=level,
                accepted=accepted,
                reason=reason,
                candidate_direction=candidate_direction,
                pulse_count_after=state.pulse_count,
            )
        )

    def _b_handler(
        self,
        side: str,
        expected_pin: int,
    ) -> Callable[[int, int, int, int], None]:
        def handle(chip: int, gpio: int, level: int, tick: int) -> None:
            with self._lock:
                if self._closed:
                    return
                state = self._states[side]
                if (
                    not self._valid_alert(chip, gpio, expected_pin)
                    or not isinstance(level, int)
                    or isinstance(level, bool)
                    or level not in (0, 1)
                    or not self._valid_tick(tick)
                ):
                    state.invalid_alerts += 1
                    return

                assert state.b_history is not None
                # A B transition which belongs at/before an already observed
                # physical A edge arrived too late to establish that A edge's
                # direction. This remains a hard ordering diagnostic.
                if (
                    state.last_a_timestamp_ns is not None
                    and tick <= state.last_a_timestamp_ns
                ):
                    state.invalid_alerts += 1
                    self._record_event(
                        state, channel="B", callback_timestamp_ns=tick,
                        physical_timestamp_ns=tick, level=level, accepted=False,
                        reason="B_LATE_AFTER_A",
                    )
                    return
                if state.b_history and tick <= state.b_history[-1][0]:
                    state.invalid_alerts += 1
                    self._record_event(
                        state, channel="B", callback_timestamp_ns=tick,
                        physical_timestamp_ns=tick, level=level, accepted=False,
                        reason="B_NONMONOTONIC",
                    )
                    return
                if level == state.level_b:
                    state.quadrature_rejections += 1
                    self._record_event(
                        state, channel="B", callback_timestamp_ns=tick,
                        physical_timestamp_ns=tick, level=level, accepted=False,
                        reason="B_DUPLICATE_LEVEL",
                    )
                    return

                state.b_history.append((tick, level))
                state.level_b = level
                state.last_b_timestamp_ns = tick
                state.last_b_level = level
                self._record_event(
                    state, channel="B", callback_timestamp_ns=tick,
                    physical_timestamp_ns=tick, level=level, accepted=True,
                    reason="B_TRANSITION",
                )

        return handle

    @staticmethod
    def _b_evidence_at_physical_a(
        state: _CounterState,
        physical_tick: int,
        guard_ns: int,
    ) -> tuple[int | None, str]:
        assert state.b_history is not None
        previous: tuple[int, int] | None = None
        following: tuple[int, int] | None = None
        for event in state.b_history:
            if event[0] < physical_tick:
                previous = event
                continue
            if event[0] == physical_tick:
                return None, "B_AT_A_TIMESTAMP"
            following = event
            break
        if guard_ns > 0:
            if previous is not None and physical_tick - previous[0] <= guard_ns:
                return None, "B_UNSTABLE_BEFORE_A"
            if following is not None and following[0] - physical_tick <= guard_ns:
                return None, "B_UNSTABLE_AFTER_A"
        return (
            previous[1] if previous is not None else state.initial_level_b,
            "B_STABLE_AT_A",
        )

    @staticmethod
    def _append_signed_edge(
        state: _CounterState,
        timestamp_ns: int,
        direction: int,
    ) -> None:
        assert state.edge_history is not None
        state.pulse_count += direction
        state.edge_history.append(SignedPulseEdge(timestamp_ns, state.pulse_count))

    @staticmethod
    def _reject_pending_direction(state: _CounterState) -> None:
        assert state.pending_a_timestamps is not None
        if state.pending_a_timestamps:
            state.quadrature_rejections += len(state.pending_a_timestamps)
        state.pending_a_timestamps.clear()
        state.pending_direction = 0

    def _commit_a_direction(
        self,
        state: _CounterState,
        channel: GpioCounterChannelConfig,
        physical_tick: int,
        direction: int,
    ) -> tuple[bool, str]:
        assert state.pending_a_timestamps is not None
        confirm_edges = channel.direction_change_confirm_edges
        if state.confirmed_direction == 0:
            state.confirmed_direction = direction
            self._append_signed_edge(state, physical_tick, direction)
            return True, "A_INITIAL_DIRECTION"

        if direction == state.confirmed_direction:
            if state.pending_a_timestamps:
                self._reject_pending_direction(state)
            self._append_signed_edge(state, physical_tick, direction)
            return True, "A_CONFIRMED_DIRECTION"

        if confirm_edges == 1:
            state.direction_change_candidates += 1
            state.direction_changes_confirmed += 1
            state.confirmed_direction = direction
            self._append_signed_edge(state, physical_tick, direction)
            return True, "A_REVERSAL_CONFIRMED"

        window_ns = channel.direction_change_confirm_window_micros * 1_000
        if state.pending_direction != direction:
            self._reject_pending_direction(state)
        elif (
            state.pending_a_timestamps
            and physical_tick - state.pending_a_timestamps[0] > window_ns
        ):
            self._reject_pending_direction(state)

        if not state.pending_a_timestamps:
            state.pending_direction = direction
        state.pending_a_timestamps.append(physical_tick)
        state.direction_change_candidates += 1

        if len(state.pending_a_timestamps) < confirm_edges:
            return False, "A_REVERSAL_PENDING"

        pending = tuple(state.pending_a_timestamps)
        state.pending_a_timestamps.clear()
        state.pending_direction = 0
        state.confirmed_direction = direction
        state.direction_changes_confirmed += 1
        for timestamp_ns in pending:
            self._append_signed_edge(state, timestamp_ns, direction)
        return True, "A_REVERSAL_CONFIRMED"

    def _a_handler(
        self,
        side: str,
        channel: GpioCounterChannelConfig,
    ) -> Callable[[int, int, int, int], None]:
        def handle(chip: int, gpio: int, level: int, tick: int) -> None:
            with self._lock:
                if not self._running:
                    return
                state = self._states[side]
                if (
                    not self._valid_alert(chip, gpio, channel.pin_a)
                    or not isinstance(level, int)
                    or isinstance(level, bool)
                    or level != 1
                    or not self._valid_tick(tick)
                ):
                    state.invalid_alerts += 1
                    return

                # lgpio timestamps a debounced level change after the configured
                # debounce interval.  Recover the physical A transition time so
                # B is sampled from the matching quadrature instant.
                debounce_ns = channel.a_debounce_micros * 1_000
                # Real lgpio epochs are vastly larger than the debounce delay.
                # Preserve tiny synthetic test clocks (< debounce_ns) as-is;
                # live callbacks use the documented debounce correction.
                physical_tick = tick - debounce_ns if tick >= debounce_ns else tick

                if (
                    state.last_a_timestamp_ns is not None
                    and physical_tick <= state.last_a_timestamp_ns
                ):
                    state.invalid_alerts += 1
                    return

                guard_ns = channel.direction_guard_micros * 1_000
                level_b, evidence_reason = self._b_evidence_at_physical_a(
                    state, physical_tick, guard_ns
                )
                if level_b is None:
                    state.quadrature_rejections += 1
                    state.last_a_timestamp_ns = physical_tick
                    self._record_event(
                        state, channel="A", callback_timestamp_ns=tick,
                        physical_timestamp_ns=physical_tick, level=1, accepted=False,
                        reason=evidence_reason,
                    )
                    return

                direction = 1 if level_b == channel.forward_b_level else -1
                if channel.invert:
                    direction = -direction
                accepted, reason = self._commit_a_direction(
                    state, channel, physical_tick, direction
                )
                state.last_a_timestamp_ns = physical_tick
                self._record_event(
                    state, channel="A", callback_timestamp_ns=tick,
                    physical_timestamp_ns=physical_tick, level=1, accepted=accepted,
                    reason=reason, candidate_direction=direction,
                )

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
            assert state.edge_history is not None
            return SignedPulseCounterSnapshot(
                pulse_count=state.pulse_count,
                read_errors=state.read_errors,
                invalid_alerts=state.invalid_alerts,
                edge_history=tuple(state.edge_history),
                quadrature_rejections=state.quadrature_rejections,
                direction_change_candidates=state.direction_change_candidates,
                direction_changes_confirmed=state.direction_changes_confirmed,
                confirmed_direction=state.confirmed_direction,
                pending_direction=state.pending_direction,
                pending_direction_edges=len(state.pending_a_timestamps or ()),
                last_a_timestamp_ns=state.last_a_timestamp_ns,
                last_b_timestamp_ns=state.last_b_timestamp_ns,
                last_b_level=state.last_b_level,
            )

    def diagnostic_events(self, side: str) -> tuple[QuadratureAlertEvent, ...]:
        """Return the bounded raw A/B decision trace without motor authority."""

        if side not in self._states:
            raise ValueError("side must be left or right")
        with self._lock:
            events = self._states[side].diagnostic_events
            assert events is not None
            return tuple(events)

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
    "QuadratureAlertEvent",
]
