"""Direct-value deterministic replay diagnostics without hash/provenance machinery."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .engine import LayerValue, TickEngine, TickInputs, TickTrace


@dataclass(frozen=True, slots=True)
class ReplayDivergence:
    tick_id: int
    layer: str
    expected: LayerValue | TickTrace | None
    actual: LayerValue | TickTrace | None


def run_replay(engine: TickEngine, inputs: Iterable[TickInputs]) -> tuple[TickTrace, ...]:
    """Run a closed input sequence with an offline writer and return typed traces."""

    return tuple(engine.run_tick(item).trace for item in inputs)


def first_divergence(
    expected: tuple[TickTrace, ...],
    actual: tuple[TickTrace, ...],
) -> ReplayDivergence | None:
    """Return the first tick and layer whose typed output differs."""

    count = max(len(expected), len(actual))
    for tick_index in range(count):
        expected_trace = expected[tick_index] if tick_index < len(expected) else None
        actual_trace = actual[tick_index] if tick_index < len(actual) else None
        if expected_trace is None or actual_trace is None:
            present = expected_trace or actual_trace
            tick_id = present.context.tick_id if present is not None else tick_index
            return ReplayDivergence(tick_id, "TickEngine", expected_trace, actual_trace)
        if expected_trace.context != actual_trace.context:
            return ReplayDivergence(
                expected_trace.context.tick_id,
                "TickEngine",
                expected_trace,
                actual_trace,
            )

        layer_count = max(len(expected_trace.layers), len(actual_trace.layers))
        for layer_index in range(layer_count):
            expected_layer = (
                expected_trace.layers[layer_index]
                if layer_index < len(expected_trace.layers)
                else None
            )
            actual_layer = (
                actual_trace.layers[layer_index]
                if layer_index < len(actual_trace.layers)
                else None
            )
            if expected_layer == actual_layer:
                continue
            layer_name = (
                expected_layer.layer
                if expected_layer is not None
                else actual_layer.layer if actual_layer is not None else "TickEngine"
            )
            return ReplayDivergence(
                expected_trace.context.tick_id,
                layer_name,
                expected_layer.output if expected_layer is not None else None,
                actual_layer.output if actual_layer is not None else None,
            )
        if expected_trace.fault_layer != actual_trace.fault_layer:
            return ReplayDivergence(
                expected_trace.context.tick_id,
                expected_trace.fault_layer or actual_trace.fault_layer or "TickEngine",
                expected_trace,
                actual_trace,
            )
    return None


__all__ = ["ReplayDivergence", "first_divergence", "run_replay"]
