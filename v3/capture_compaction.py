"""Lossless on-disk compaction for R2B4 MCAP JSON payloads.

This module has no runtime/control authority. It only reduces redundant JSON
representation inside the capture sidecar and restores the canonical expanded
shape at the MCAP reader boundary.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

_DATA_FIELD_KEY = "$df"
_REFERENCE_KEY = "$ref"


class CaptureCompactionError(RuntimeError):
    """Stored compact capture data is malformed or cannot be expanded."""


def _compact_data_fields(value: object) -> object:
    if isinstance(value, Mapping):
        if (
            set(value) == {"__type__", "key", "value"}
            and value.get("__type__") == "DataField"
            and isinstance(value.get("key"), str)
        ):
            return {
                _DATA_FIELD_KEY: [
                    value["key"],
                    _compact_data_fields(value["value"]),
                ]
            }
        return {str(key): _compact_data_fields(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_compact_data_fields(item) for item in value]
    if isinstance(value, tuple):
        return [_compact_data_fields(item) for item in value]
    return value


def _expand_data_fields(value: object) -> object:
    if isinstance(value, Mapping):
        if set(value) == {_DATA_FIELD_KEY}:
            payload = value[_DATA_FIELD_KEY]
            if (
                not isinstance(payload, Sequence)
                or isinstance(payload, (str, bytes))
                or len(payload) != 2
                or not isinstance(payload[0], str)
            ):
                raise CaptureCompactionError("invalid compact DataField")
            return {
                "__type__": "DataField",
                "key": payload[0],
                "value": _expand_data_fields(payload[1]),
            }
        return {str(key): _expand_data_fields(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_expand_data_fields(item) for item in value]
    return value


def compact_tick_row(row: Mapping[str, object]) -> dict[str, object]:
    """Return a losslessly compacted tick row for on-disk JSON storage."""

    compacted = _compact_data_fields(row)
    if not isinstance(compacted, dict):
        raise CaptureCompactionError("tick row must compact to an object")

    inputs = compacted.get("inputs")
    expected = compacted.get("expected")
    if not isinstance(inputs, dict) or not isinstance(expected, dict):
        return compacted
    layers = expected.get("layers")
    if not isinstance(layers, dict):
        return compacted

    input_context = inputs.get("context")
    for layer_value in layers.values():
        if isinstance(layer_value, dict) and layer_value.get("context") == input_context:
            layer_value["context"] = {_REFERENCE_KEY: "input_context"}

    l5 = layers.get("L5")
    l5_constraints = l5.get("constraints") if isinstance(l5, dict) else None
    if l5_constraints is not None:
        for layer_name in ("L6", "L7", "L8"):
            layer_value = layers.get(layer_name)
            if (
                isinstance(layer_value, dict)
                and layer_value.get("constraints") == l5_constraints
            ):
                layer_value["constraints"] = {_REFERENCE_KEY: "l5_constraints"}

    planner_candidates = None
    planner_input = inputs.get("planner_input")
    if isinstance(planner_input, dict):
        planner_result = planner_input.get("result")
        if isinstance(planner_result, dict):
            planner_candidates = planner_result.get("trajectory_candidates")

    l6 = layers.get("L6")
    if isinstance(l6, dict):
        l6_candidates = l6.get("trajectory_candidates")

        l7 = layers.get("L7")
        if isinstance(l7, dict) and isinstance(l6_candidates, list):
            trajectory = l7.get("trajectory")
            for index, candidate in enumerate(l6_candidates):
                if trajectory == candidate:
                    l7["trajectory"] = {
                        _REFERENCE_KEY: "l6_candidate",
                        "index": index,
                    }
                    break

        if planner_candidates is not None and l6_candidates == planner_candidates:
            l6["trajectory_candidates"] = {_REFERENCE_KEY: "planner_candidates"}

    return compacted


def expand_tick_row(row: Mapping[str, object]) -> dict[str, object]:
    """Restore the canonical pre-compaction tick JSON shape."""

    expanded = _expand_data_fields(row)
    if not isinstance(expanded, dict):
        raise CaptureCompactionError("tick row must expand to an object")

    inputs = expanded.get("inputs")
    expected = expanded.get("expected")
    if not isinstance(inputs, dict) or not isinstance(expected, dict):
        return expanded
    layers = expected.get("layers")
    if not isinstance(layers, dict):
        return expanded

    input_context = inputs.get("context")
    for layer_name, layer_value in layers.items():
        if not isinstance(layer_value, dict):
            continue
        context = layer_value.get("context")
        if context == {_REFERENCE_KEY: "input_context"}:
            if input_context is None:
                raise CaptureCompactionError(
                    f"{layer_name} references missing input context"
                )
            layer_value["context"] = input_context

    l5 = layers.get("L5")
    l5_constraints = l5.get("constraints") if isinstance(l5, dict) else None
    for layer_name in ("L6", "L7", "L8"):
        layer_value = layers.get(layer_name)
        if not isinstance(layer_value, dict):
            continue
        if layer_value.get("constraints") == {_REFERENCE_KEY: "l5_constraints"}:
            if l5_constraints is None:
                raise CaptureCompactionError(
                    f"{layer_name} references missing L5 constraints"
                )
            layer_value["constraints"] = l5_constraints

    l6 = layers.get("L6")
    if isinstance(l6, dict):
        candidates = l6.get("trajectory_candidates")
        if candidates == {_REFERENCE_KEY: "planner_candidates"}:
            planner_input = inputs.get("planner_input")
            planner_result = (
                planner_input.get("result")
                if isinstance(planner_input, dict)
                else None
            )
            planner_candidates = (
                planner_result.get("trajectory_candidates")
                if isinstance(planner_result, dict)
                else None
            )
            if not isinstance(planner_candidates, list):
                raise CaptureCompactionError(
                    "L6 references missing planner trajectory candidates"
                )
            l6["trajectory_candidates"] = planner_candidates

    l7 = layers.get("L7")
    if isinstance(l7, dict):
        trajectory = l7.get("trajectory")
        if (
            isinstance(trajectory, dict)
            and trajectory.get(_REFERENCE_KEY) == "l6_candidate"
        ):
            if set(trajectory) != {_REFERENCE_KEY, "index"}:
                raise CaptureCompactionError("invalid L7 candidate reference")
            index = trajectory.get("index")
            candidates = (
                l6.get("trajectory_candidates")
                if isinstance(l6, dict)
                else None
            )
            if (
                not isinstance(index, int)
                or isinstance(index, bool)
                or not isinstance(candidates, list)
                or index < 0
                or index >= len(candidates)
            ):
                raise CaptureCompactionError("L7 candidate reference is out of range")
            l7["trajectory"] = candidates[index]

    return expanded


def compact_checkpoint_row(row: Mapping[str, object]) -> dict[str, object]:
    """Compact verbose scalar DataField wrappers in a checkpoint row."""

    compacted = _compact_data_fields(row)
    if not isinstance(compacted, dict):
        raise CaptureCompactionError("checkpoint row must compact to an object")
    return compacted


def expand_checkpoint_row(row: Mapping[str, object]) -> dict[str, object]:
    """Restore canonical DataField wrappers in a checkpoint row."""

    expanded = _expand_data_fields(row)
    if not isinstance(expanded, dict):
        raise CaptureCompactionError("checkpoint row must expand to an object")
    return expanded


__all__ = [
    "CaptureCompactionError",
    "compact_checkpoint_row",
    "compact_tick_row",
    "expand_checkpoint_row",
    "expand_tick_row",
]
