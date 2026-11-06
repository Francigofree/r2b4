"""Deterministic fake HAL and command edge for offline V3 composition tests."""

from __future__ import annotations

from v3.contracts import (
    CommandMode,
    CommandRequest,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    FinalActuation,
    RawDeviceBatch,
    TickContext,
)


class FakeHal:
    """Close fake L0 reads and record the final L12 writes by tick id."""

    __slots__ = ("_device_health", "_fail_read_ticks", "_fail_write_ticks", "_samples", "_writes")

    def __init__(
        self,
        *,
        samples: tuple[DeviceSample, ...] = (),
        device_health: tuple[DeviceHealth, ...] | None = None,
        fail_read_ticks: frozenset[int] = frozenset(),
        fail_write_ticks: frozenset[int] = frozenset(),
    ) -> None:
        self._samples = samples
        self._device_health = (
            device_health
            if device_health is not None
            else (DeviceHealth("fake-motor-driver", DeviceHealthState.OK),)
        )
        self._fail_read_ticks = fail_read_ticks
        self._fail_write_ticks = fail_write_ticks
        self._writes: list[FinalActuation] = []

    @property
    def writes(self) -> tuple[FinalActuation, ...]:
        return tuple(self._writes)

    def read(self, context: TickContext) -> RawDeviceBatch:
        if context.tick_id in self._fail_read_ticks:
            raise OSError("injected fake HAL read failure")
        return RawDeviceBatch(context, self._samples, self._device_health)

    def write(self, command: FinalActuation) -> None:
        self._writes.append(command)
        if command.context.tick_id in self._fail_write_ticks:
            raise OSError("injected fake HAL write failure")


class FakeCommandGateway:
    """Produce one deterministic STOP command for each supplied context."""

    __slots__ = ("_fail_ticks",)

    def __init__(self, *, fail_ticks: frozenset[int] = frozenset()) -> None:
        self._fail_ticks = fail_ticks

    def snapshot(self, context: TickContext) -> CommandRequest:
        if context.tick_id in self._fail_ticks:
            raise OSError("injected fake command gateway failure")
        return CommandRequest(
            context=context,
            command_id=f"stop-{context.tick_id}",
            mode=CommandMode.STOP,
            goal=(),
            expiry_tick=context.tick_id,
        )


__all__ = ["FakeCommandGateway", "FakeHal"]
