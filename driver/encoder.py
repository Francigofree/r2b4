#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""DFRobot KIT0085 / 28PA51G two-phase Hall encoder driver."""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import lgpio


@dataclass(frozen=True)
class EncoderDriverSnapshot:
    pulse_count: int
    edge_count: int
    a_edge_count: int
    a_rising_count: int
    a_falling_count: int
    b_edge_count: int
    b_rising_count: int
    b_falling_count: int
    forward_count: int
    reverse_count: int
    last_direction: int
    last_edge_time: float
    last_edge_tick: int
    level_a: int
    level_b: int
    read_errors: int
    invalid_alerts: int


class DFRobotQuadratureEncoder:
    """
    Interrupt-driven x1 quadrature counter.

    The DFRobot reference implementation counts the rising edge of channel A
    and uses channel B for direction. Channel B is latched from its own edge
    callback, so Python callback latency cannot turn a later B level into the
    direction of an earlier A edge. Only A rising edges increment the signed
    counter; 663 counts/output-shaft revolution therefore remains unchanged.
    """

    model = "DFROBOT_KIT0085_28PA51G"
    count_mode = "X1_A_RISING"
    direction_source = "QUADRATURE_AB"
    signed_counts = True

    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        *,
        name: str = "Encoder",
        counts_per_revolution: int = 663,
        forward_b_level: int = 1,
        a_debounce_micros: int = 0,
        invert: bool = False,
        pull_up: bool = False,
        chip: int = 0,
    ):
        self.pin_a = int(pin_a)
        self.pin_b = int(pin_b)
        if self.pin_a == self.pin_b:
            raise ValueError(f"{name}: encoder A/B GPIO pins must be different")

        self.name = str(name)
        self.counts_per_revolution = max(1, int(counts_per_revolution))
        self.forward_b_level = 1 if int(forward_b_level) else 0
        self.a_debounce_micros = max(0, min(10_000, int(a_debounce_micros)))
        self.invert = bool(invert)
        self.pull_up = bool(pull_up)
        self.chip = int(chip)

        self._lock = threading.Lock()
        self._handle: int | None = None
        self._callback_a: Any = None
        self._callback_b: Any = None
        self._running = False

        self._pulse_count = 0
        self.edge_count = 0
        self.a_edge_count = 0
        self.a_rising_count = 0
        self.a_falling_count = 0
        self.b_edge_count = 0
        self.b_rising_count = 0
        self.b_falling_count = 0
        self.forward_count = 0
        self.reverse_count = 0
        self.last_direction = 0
        self.last_edge_time = 0.0
        self.last_edge_tick = 0
        self.level_a = 0
        self.level_b = 0
        self.read_errors = 0
        self.invalid_alerts = 0
        self.edge_trace_enabled = str(
            os.environ.get("R2B4_ENCODER_EDGE_TRACE", "")
        ).strip().lower() in ("1", "true", "yes", "on")
        self._a_rising_trace = deque(maxlen=192)
        self._a_rising_trace_sequence = 0

    @staticmethod
    def direction_from_b(
        level_b: int,
        *,
        forward_b_level: int = 1,
        invert: bool = False,
    ) -> int:
        direction = 1 if int(level_b) == (1 if int(forward_b_level) else 0) else -1
        return -direction if bool(invert) else direction

    @property
    def pulse_count(self) -> int:
        return int(self.snapshot().pulse_count)

    def snapshot(self) -> EncoderDriverSnapshot:
        """Return one lock-consistent driver state sample."""
        with self._lock:
            return EncoderDriverSnapshot(
                pulse_count=int(self._pulse_count),
                edge_count=int(self.edge_count),
                a_edge_count=int(self.a_edge_count),
                a_rising_count=int(self.a_rising_count),
                a_falling_count=int(self.a_falling_count),
                b_edge_count=int(self.b_edge_count),
                b_rising_count=int(self.b_rising_count),
                b_falling_count=int(self.b_falling_count),
                forward_count=int(self.forward_count),
                reverse_count=int(self.reverse_count),
                last_direction=int(self.last_direction),
                last_edge_time=float(self.last_edge_time),
                last_edge_tick=int(self.last_edge_tick),
                level_a=int(self.level_a),
                level_b=int(self.level_b),
                read_errors=int(self.read_errors),
                invalid_alerts=int(self.invalid_alerts),
            )

    def recent_a_rising_events(self, limit: int = 20) -> list[dict[str, Any]]:
        """Return a bounded edge-level trace when explicitly enabled."""
        if not self.edge_trace_enabled:
            return []
        safe_limit = max(1, min(64, int(limit)))
        with self._lock:
            rows = list(self._a_rising_trace)[-safe_limit:]
        return [
            {
                "sequence": int(row[0]),
                "perf_time_s": float(row[1]),
                "gpio_tick": int(row[2]),
                "b_level_at_a_rising": int(row[3]),
                "direction": int(row[4]),
                "signed_pulse_count": int(row[5]),
            }
            for row in rows
        ]

    @property
    def running(self) -> bool:
        return bool(self._running)

    @property
    def health(self) -> str:
        if not self._running:
            return "STOPPED"
        snap = self.snapshot()
        return "OK" if snap.read_errors == 0 and snap.invalid_alerts == 0 else "DEGRADED"

    def reset_count(self, value: int = 0) -> None:
        with self._lock:
            self._pulse_count = int(value)
            self.edge_count = 0
            self.a_edge_count = 0
            self.a_rising_count = 0
            self.a_falling_count = 0
            self.b_edge_count = 0
            self.b_rising_count = 0
            self.b_falling_count = 0
            self.forward_count = 0
            self.reverse_count = 0
            self.last_direction = 0
            self._a_rising_trace.clear()
            self._a_rising_trace_sequence = 0

    def _on_a_edge(self, _chip: int, gpio: int, level: int, tick: int) -> None:
        if not self._running or int(gpio) != self.pin_a:
            return
        level_i = int(level)
        if level_i not in (0, 1):
            with self._lock:
                self.invalid_alerts += 1
            return
        if level_i != 1:
            return

        now = time.perf_counter()
        with self._lock:
            level_b = int(self.level_b)
            direction = self.direction_from_b(
                level_b,
                forward_b_level=self.forward_b_level,
                invert=self.invert,
            )
            self.level_a = 1
            self.a_edge_count += 1
            self.a_rising_count += 1
            self._pulse_count += int(direction)
            self.edge_count += 1
            self._a_rising_trace_sequence += 1
            if self.edge_trace_enabled:
                self._a_rising_trace.append(
                    (
                        int(self._a_rising_trace_sequence),
                        float(now),
                        int(tick),
                        int(level_b),
                        int(direction),
                        int(self._pulse_count),
                    )
                )
            if direction > 0:
                self.forward_count += 1
            else:
                self.reverse_count += 1
            self.last_direction = int(direction)
            self.last_edge_time = float(now)
            self.last_edge_tick = int(tick)

    def _on_b_edge(self, _chip: int, gpio: int, level: int, tick: int) -> None:
        if not self._running or int(gpio) != self.pin_b:
            return
        level_i = int(level)
        if level_i not in (0, 1):
            with self._lock:
                self.invalid_alerts += 1
            return
        with self._lock:
            self.level_b = level_i
            self.b_edge_count += 1
            if level_i == 1:
                self.b_rising_count += 1
            else:
                self.b_falling_count += 1

    def start(self) -> bool:
        if self._running:
            return True

        handle = None
        callback_a = None
        callback_b = None
        flags = int(lgpio.SET_PULL_UP) if self.pull_up else 0
        try:
            handle = lgpio.gpiochip_open(self.chip)
            lgpio.gpio_claim_alert(handle, self.pin_a, lgpio.RISING_EDGE, flags)
            lgpio.gpio_claim_alert(handle, self.pin_b, lgpio.BOTH_EDGES, flags)
            if self.a_debounce_micros > 0:
                # The live pivot trace exposed 125 us A-edge bursts coupled to
                # the motor PWM.  Requiring 150 us of stable A level rejects
                # those pulses while retaining margin at the proven 0.582 m/s
                # wheel-map ceiling (about 276 us A-to-next-quadrature edge).
                lgpio.gpio_set_debounce_micros(
                    handle,
                    self.pin_a,
                    self.a_debounce_micros,
                )
            self._handle = handle
            self.level_a = int(lgpio.gpio_read(handle, self.pin_a))
            self.level_b = int(lgpio.gpio_read(handle, self.pin_b))
            callback_b = lgpio.callback(
                handle,
                self.pin_b,
                lgpio.BOTH_EDGES,
                self._on_b_edge,
            )
            self._callback_b = callback_b
            callback_a = lgpio.callback(
                handle,
                self.pin_a,
                lgpio.RISING_EDGE,
                self._on_a_edge,
            )
            self._callback_a = callback_a
            self._running = True
            return True
        except Exception:
            self._running = False
            for callback in (callback_a, callback_b):
                if callback is None:
                    continue
                try:
                    callback.cancel()
                except Exception:
                    pass
            if handle is not None:
                for pin in (self.pin_a, self.pin_b):
                    try:
                        lgpio.gpio_free(handle, pin)
                    except Exception:
                        pass
                try:
                    lgpio.gpiochip_close(handle)
                except Exception:
                    pass
            self._handle = None
            self._callback_a = None
            self._callback_b = None
            raise

    def stop(self) -> None:
        self._running = False
        callbacks = (self._callback_a, self._callback_b)
        self._callback_a = None
        self._callback_b = None
        handle, self._handle = self._handle, None

        for callback in callbacks:
            if callback is None:
                continue
            try:
                callback.cancel()
            except Exception:
                pass
        if handle is not None:
            for pin in (self.pin_a, self.pin_b):
                try:
                    lgpio.gpio_free(handle, pin)
                except Exception:
                    pass
            try:
                lgpio.gpiochip_close(handle)
            except Exception:
                pass
