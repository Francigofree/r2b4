"""Hardware-free native V3 final motor-writer boundary."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Protocol

from v3.contracts import FinalActuation

from .motor_pwm import (
    Drv8871MotorFrame,
    MotorChannelPhysicalConfig,
    plan_final_actuation,
)


class MotorFrameSink(Protocol):
    """Injected owner of the eventual paired physical frame write."""

    def write(self, frame: Drv8871MotorFrame) -> None: ...


class NativeMotorWriter:
    """Close one L12 decision into exactly one validated frame-sink call.

    The injected sink is the atomic edge boundary.  This adapter deliberately
    owns no GPIO backend, lifecycle, retry, loop or fallback write path.
    """

    __slots__ = ("_left_config", "_right_config", "_sink", "_timing_observer")

    def __init__(
        self,
        left_config: MotorChannelPhysicalConfig,
        right_config: MotorChannelPhysicalConfig,
        sink: MotorFrameSink,
    ) -> None:
        if not isinstance(left_config, MotorChannelPhysicalConfig):
            raise TypeError("left_config must be MotorChannelPhysicalConfig")
        if not isinstance(right_config, MotorChannelPhysicalConfig):
            raise TypeError("right_config must be MotorChannelPhysicalConfig")
        pins = (
            left_config.in1,
            left_config.in2,
            right_config.in1,
            right_config.in2,
        )
        if len(set(pins)) != len(pins):
            raise ValueError("left and right motor GPIO pins must be unique")
        if not callable(getattr(sink, "write", None)):
            raise TypeError("sink must provide a callable write method")

        self._left_config = left_config
        self._right_config = right_config
        self._sink = sink
        self._timing_observer: Callable[[str, int], None] | None = None

    def set_timing_observer(self, observer: Callable[[str, int], None] | None) -> None:
        """Observe the existing final write region; never alter its result."""

        if observer is not None and not callable(observer):
            raise TypeError("timing observer must be callable or None")
        self._timing_observer = observer

    def write(self, command: FinalActuation) -> None:
        """Validate and forward one decision without catch, retry or fallback."""

        if not isinstance(command, FinalActuation):
            raise TypeError("command must be FinalActuation")
        observer = self._timing_observer
        started = (time.perf_counter_ns(), time.thread_time_ns()) if observer is not None else None
        try:
            frame = plan_final_actuation(
                self._left_config,
                self._right_config,
                command,
            )
            self._sink.write(frame)
        finally:
            if started is not None:
                wall_ns = time.perf_counter_ns() - started[0]
                cpu_ns = time.thread_time_ns() - started[1]
                try:
                    observer("MOTOR_WRITE", max(0, wall_ns))
                    observer("MOTOR_WRITE_CPU", max(0, cpu_ns))
                except Exception:
                    # A diagnostic failure cannot fail a successful motor write
                    # or replace a physical writer exception with its own error.
                    self._timing_observer = None


__all__ = ["MotorFrameSink", "NativeMotorWriter"]
