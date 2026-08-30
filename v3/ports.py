"""Small edge-port definitions used by the V3 composition root."""

from __future__ import annotations

from typing import Protocol

from .contracts import CommandRequest, FinalActuation, RawDeviceBatch, TickContext


class DeviceReader(Protocol):
    """Close the L0 device input for one supplied tick context."""

    def read(self, context: TickContext) -> RawDeviceBatch:
        """Return one immutable raw device snapshot or raise on I/O failure."""


class CommandGateway(Protocol):
    """Close the authenticated external command input for one tick."""

    def snapshot(self, context: TickContext) -> CommandRequest:
        """Return one immutable command request or raise on gateway failure."""


class MotorWriter(Protocol):
    """Atomic physical motor write owned exclusively by the L12 final stage."""

    def write(self, command: FinalActuation) -> None:
        """Apply exactly one final command or raise on write failure."""


__all__ = ["CommandGateway", "DeviceReader", "MotorWriter"]
