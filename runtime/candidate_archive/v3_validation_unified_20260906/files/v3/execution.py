"""Reusable V3 execution boundary with no device or runtime authority."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from typing import Protocol

from .engine import TickInputs, TickResult


class InputSource(Protocol):
    """Provide already closed V3 tick inputs."""

    def __iter__(self) -> Iterator[TickInputs]: ...


class ProductionV3(Protocol):
    """The canonical production computation exposed at one tick boundary."""

    def run_tick(self, inputs: TickInputs) -> TickResult: ...


@dataclass(frozen=True, slots=True)
class ExecutionRecord:
    inputs: TickInputs
    result: TickResult


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
            sink.write(ExecutionRecord(inputs, result))
            count += 1
            if first_tick_id is None:
                first_tick_id = inputs.context.tick_id
            last_tick_id = inputs.context.tick_id
        return ExecutionSummary(count, first_tick_id, last_tick_id)


__all__ = [
    "ExecutionBoundary",
    "ExecutionRecord",
    "ExecutionSummary",
    "InputSource",
    "IterableInputSource",
    "MemoryOutputSink",
    "OutputSink",
    "ProductionV3",
]
