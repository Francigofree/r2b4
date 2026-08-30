"""Hardware-free native V3 final motor-writer boundary."""

from __future__ import annotations

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

    __slots__ = ("_left_config", "_right_config", "_sink")

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

    def write(self, command: FinalActuation) -> None:
        """Validate and forward one decision without catch, retry or fallback."""

        if not isinstance(command, FinalActuation):
            raise TypeError("command must be FinalActuation")
        frame = plan_final_actuation(
            self._left_config,
            self._right_config,
            command,
        )
        self._sink.write(frame)


__all__ = ["MotorFrameSink", "NativeMotorWriter"]
