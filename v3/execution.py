"""Reusable V3 execution boundary with no device or runtime authority."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from typing import Protocol

from .contracts import DeviceHealth, LifecycleState, RawDeviceBatch, TickContext
from .engine import TickInputs, TickResult


class InputSource(Protocol):
    """Provide already closed V3 tick inputs."""

    def __iter__(self) -> Iterator[TickInputs]: ...


class ProductionV3(Protocol):
    """The canonical production computation exposed at one tick boundary."""

    def run_tick(self, inputs: TickInputs) -> TickResult: ...


@dataclass(frozen=True, slots=True)
class ExecutionRecord:
    """A normal tick whose complete inputs were closed before L1."""

    inputs: TickInputs
    result: TickResult
    evidence: tuple[object, ...] = ()


@dataclass(frozen=True, slots=True)
class EdgeFaultRecord:
    """A completed L12 fail-closed tick for a fault before input closure."""

    context: TickContext
    lifecycle: LifecycleState
    reason: str
    fault_layer: str
    critical_health: tuple[DeviceHealth, ...]
    result: TickResult
    raw_devices: RawDeviceBatch | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("context must be TickContext")
        if not isinstance(self.lifecycle, LifecycleState):
            raise TypeError("lifecycle must be LifecycleState")
        if not isinstance(self.reason, str) or not self.reason.strip():
            raise ValueError("reason must be non-empty")
        if not isinstance(self.fault_layer, str) or not self.fault_layer.strip():
            raise ValueError("fault_layer must be non-empty")
        if any(not isinstance(item, DeviceHealth) for item in self.critical_health):
            raise TypeError("critical_health must contain DeviceHealth values")
        if self.raw_devices is not None and self.raw_devices.context != self.context:
            raise ValueError("raw_devices must use the fault context")
        if self.result.trace.context != self.context:
            raise ValueError("fault result must use the fault context")


@dataclass(frozen=True, slots=True)
class WriterFailureRecord:
    """A terminal edge record when L12 could not complete its sole motor write."""

    context: TickContext
    lifecycle: LifecycleState
    reason: str
    attempted_actuation: object | None
    inputs: TickInputs | None = None
    raw_devices: RawDeviceBatch | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("context must be TickContext")
        if not isinstance(self.lifecycle, LifecycleState):
            raise TypeError("lifecycle must be LifecycleState")
        if not isinstance(self.reason, str) or not self.reason.strip():
            raise ValueError("reason must be non-empty")
        if self.inputs is not None and self.inputs.context != self.context:
            raise ValueError("inputs must use the failure context")
        if self.raw_devices is not None and self.raw_devices.context != self.context:
            raise ValueError("raw_devices must use the failure context")


CaptureRecord = ExecutionRecord | EdgeFaultRecord | WriterFailureRecord


class OutputSink(Protocol):
    """Observe a completed production tick without feeding back into control."""

    def write(self, record: ExecutionRecord) -> None: ...


@dataclass(frozen=True, slots=True)
class ExecutionSummary:
    tick_count: int
    first_tick_id: int | None
    last_tick_id: int | None


class IterableInputSource:
    """Small deterministic input source useful for replay and future simulation."""

    __slots__ = ("_inputs",)

    def __init__(self, inputs: Iterable[TickInputs]) -> None:
        values = tuple(inputs)
        if any(not isinstance(value, TickInputs) for value in values):
            raise TypeError("input source must contain TickInputs")
        self._inputs = values

    def __iter__(self) -> Iterator[TickInputs]:
        return iter(self._inputs)


class MemoryOutputSink:
    """Passive in-memory sink; it owns no production state or hardware capability."""

    __slots__ = ("_records",)

    def __init__(self) -> None:
        self._records: list[ExecutionRecord] = []

    @property
    def records(self) -> tuple[ExecutionRecord, ...]:
        return tuple(self._records)

    def write(self, record: ExecutionRecord) -> None:
        if not isinstance(record, ExecutionRecord):
            raise TypeError("output sink requires ExecutionRecord")
        self._records.append(record)


class ExecutionBoundary:
    """Connect exactly one input source, production V3 executor, and output sink."""

    __slots__ = ("_production",)

    def __init__(self, production: ProductionV3) -> None:
        if not callable(getattr(production, "run_tick", None)):
            raise TypeError("production must provide run_tick")
        self._production = production

    def run(self, source: InputSource, sink: OutputSink) -> ExecutionSummary:
        if not callable(getattr(source, "__iter__", None)):
            raise TypeError("source must be iterable")
        if not callable(getattr(sink, "write", None)):
            raise TypeError("sink must provide write")
        first_tick_id: int | None = None
        last_tick_id: int | None = None
        count = 0
        for inputs in source:
            if not isinstance(inputs, TickInputs):
                raise TypeError("input source yielded a non-TickInputs value")
            result = self._production.run_tick(inputs)
            if not isinstance(result, TickResult):
                raise TypeError("production returned a non-TickResult value")
            if result.trace.context != inputs.context or result.final_actuation.context != inputs.context:
                raise ValueError("production result context differs from closed input")
            evidence = getattr(self._production, "tick_evidence", ())
            if not isinstance(evidence, tuple):
                raise TypeError("production tick_evidence must be a tuple")
            sink.write(ExecutionRecord(inputs, result, evidence))
            count += 1
            if first_tick_id is None:
                first_tick_id = inputs.context.tick_id
            last_tick_id = inputs.context.tick_id
        return ExecutionSummary(count, first_tick_id, last_tick_id)


__all__ = [
    "ExecutionBoundary",
    "CaptureRecord",
    "EdgeFaultRecord",
    "ExecutionRecord",
    "ExecutionSummary",
    "InputSource",
    "IterableInputSource",
    "MemoryOutputSink",
    "OutputSink",
    "ProductionV3",
    "WriterFailureRecord",
]
