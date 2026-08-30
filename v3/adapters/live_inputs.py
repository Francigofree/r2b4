"""Native V3 L0 aggregation boundary for injected live device sources."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import DeviceHealth, DeviceSample, RawDeviceBatch, TickContext


def _require_device_id(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


@dataclass(frozen=True, slots=True)
class LiveDeviceSnapshot:
    """One device's immutable result for one supplied tick context."""

    context: TickContext
    health: DeviceHealth
    samples: tuple[DeviceSample, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("context must be TickContext")
        if not isinstance(self.health, DeviceHealth):
            raise TypeError("health must be DeviceHealth")
        if not isinstance(self.samples, tuple) or not all(
            isinstance(sample, DeviceSample) for sample in self.samples
        ):
            raise TypeError("samples must be a tuple of DeviceSample values")
        identities: set[tuple[str, int]] = set()
        for sample in self.samples:
            if sample.device_id != self.health.device_id:
                raise ValueError("sample and health device IDs must match")
            identity = (sample.kind, sample.sequence)
            if identity in identities:
                raise ValueError(
                    "device snapshot contains a duplicate sample identity"
                )
            identities.add(identity)


class LiveDeviceSource(Protocol):
    """L0-owned source polled once with the composition's tick context."""

    @property
    def device_id(self) -> str: ...

    def read(self, context: TickContext) -> LiveDeviceSnapshot: ...


class NativeLiveInputReader:
    """Close one ordered multi-device poll into one immutable L0 batch."""

    __slots__ = ("_source_ids", "_sources")

    def __init__(self, sources: Iterable[LiveDeviceSource]) -> None:
        closed_sources = tuple(sources)
        if not closed_sources:
            raise ValueError("at least one live device source is required")
        source_ids = tuple(
            _require_device_id(getattr(source, "device_id", None), "source.device_id")
            for source in closed_sources
        )
        if len(set(source_ids)) != len(source_ids):
            raise ValueError("live device source IDs must be unique")
        self._sources = closed_sources
        self._source_ids = source_ids

    def read(self, context: TickContext) -> RawDeviceBatch:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")

        samples: list[DeviceSample] = []
        health: list[DeviceHealth] = []
        for expected_device_id, source in zip(self._source_ids, self._sources):
            snapshot = source.read(context)
            if not isinstance(snapshot, LiveDeviceSnapshot):
                raise TypeError("live device source must return LiveDeviceSnapshot")
            if snapshot.context != context:
                raise ValueError("live device snapshot context must match the tick")
            if snapshot.health.device_id != expected_device_id:
                raise ValueError("live device snapshot ID must match its configured source")
            samples.extend(snapshot.samples)
            health.append(snapshot.health)

        return RawDeviceBatch(context, tuple(samples), tuple(health))


__all__ = [
    "LiveDeviceSnapshot",
    "LiveDeviceSource",
    "NativeLiveInputReader",
]
