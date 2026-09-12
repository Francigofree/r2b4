"""Source-following deterministic replay for native V3 captures.

The replayer intentionally does not own copies of production checkpoint,
resolved-config, or TickInputs construction rules. Typed capture values are
reconstructed from the current production dataclass/enum type annotations, so
production contract evolution is followed by source rather than by parallel
hand-written replay decoders.
"""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import math
import os
import sys
import time
import types
import typing
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import MISSING, dataclass, fields, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, get_args, get_origin, get_type_hints

from .capture import (
    LAYER_ORDER,
    V3_CAPTURE_SCHEMA,
    V3CaptureError,
    encode_value,
    inspect_capture as inspect_general_capture,
    load_capture,
    validate_capture as validate_general_capture,
)
from .composition.native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
    NativeControlStateCheckpoint,
    v3_navigation_config_from_mapping,
)
from .contracts import DeviceHealth, LifecycleState, SafetyDecision, TickContext
from .engine import LayerValue, TickEngine, TickInputs, TickResult, TickTrace
from .layers.l10_chassis_control import ChassisControlConfig
from .layers.l11_actuator_control import WheelSpeedMap
from .layers.l12_safety_final import LidarSafetyConfig
from .layers.l3_state_estimation import NativeStateEstimatorConfig


V3_REPLAY_RESULT_SCHEMA = "R2B4_REPLAYER_V3_RESULT_V3"
V3_REPLAY_STATUS_MATCH = "MATCH"
V3_REPLAY_STATUS_MISMATCH = "MISMATCH"
_LAYER_ORDER = LAYER_ORDER
_INPUT_REFERENCE_KEY = "__capture_input_reference__"
_SOURCE_FIRST_ROOTS = (
    "v3_process_runtime.py",
    "v3_runtime.py",
    "v3_hardware_runtime.py",
    "v3/replay.py",
    "v3/test_hub.py",
)
_SOURCE_FIRST_STATIC_PATHS = (
    "STRUKTURALIS_RETEGEK_V3.md",
    "conf/hardver.json",
    "conf/fizika.json",
    "conf/speed_map.json",
    "conf/vezerles.json",
)


class V3ReplayError(RuntimeError):
    """The immutable V3 capture or an explicit replay input is invalid."""


@dataclass(frozen=True, slots=True)
class ReplayDivergence:
    tick_id: int
    layer: str
    expected: LayerValue | TickTrace | None
    actual: LayerValue | TickTrace | None


@dataclass(frozen=True, slots=True)
class _ReplayEntry:
    context: TickContext
    inputs: TickInputs | None
    lifecycle: LifecycleState
    reason: str | None = None
    fault_layer: str | None = None
    critical_health: tuple[DeviceHealth, ...] = ()
    writer_failure: bool = False


@dataclass(frozen=True, slots=True)
class _ReplayOutcome:
    result: TickResult | None
    writer_failure: bool
    attempted_actuation: object | None


@dataclass(frozen=True, slots=True)
class ReplaySelection:
    """Inclusive tick/time/layer selection with automatic prefix state warmup."""

    start_tick_id: int | None = None
    end_tick_id: int | None = None
    start_monotonic_ns: int | None = None
    end_monotonic_ns: int | None = None
    start_layer: str = "L1"
    end_layer: str = "L12"

    def __post_init__(self) -> None:
        for name in (
            "start_tick_id",
            "end_tick_id",
            "start_monotonic_ns",
            "end_monotonic_ns",
        ):
            value = getattr(self, name)
            if value is not None and (
                not isinstance(value, int) or isinstance(value, bool) or value < 0
            ):
                raise ValueError(f"{name} must be a non-negative integer or None")
        if (
            self.start_tick_id is not None
            and self.end_tick_id is not None
            and self.start_tick_id > self.end_tick_id
        ):
            raise ValueError("start_tick_id must not exceed end_tick_id")
        if (
            self.start_monotonic_ns is not None
            and self.end_monotonic_ns is not None
            and self.start_monotonic_ns > self.end_monotonic_ns
        ):
            raise ValueError("start_monotonic_ns must not exceed end_monotonic_ns")
        _layer_index(self.start_layer)
        _layer_index(self.end_layer)
        if _layer_index(self.start_layer) > _layer_index(self.end_layer):
            raise ValueError("start_layer must not follow end_layer")

    @property
    def layers(self) -> tuple[str, ...]:
        return _LAYER_ORDER[
            _layer_index(self.start_layer) : _layer_index(self.end_layer) + 1
        ]

    def includes(self, tick_id: int, monotonic_ns: int) -> bool:
        return bool(
            (self.start_tick_id is None or tick_id >= self.start_tick_id)
            and (self.end_tick_id is None or tick_id <= self.end_tick_id)
            and (
                self.start_monotonic_ns is None
                or monotonic_ns >= self.start_monotonic_ns
            )
            and (
                self.end_monotonic_ns is None
                or monotonic_ns <= self.end_monotonic_ns
            )
        )


class _RecordingWriter:
    __slots__ = ("attempts", "commands", "failure_write_indices")

    def __init__(self, failure_write_indices: frozenset[int] = frozenset()) -> None:
        self.commands: list[object] = []
        self.attempts: list[object] = []
        self.failure_write_indices = failure_write_indices

    def write(self, command: object) -> None:
        write_index = len(self.attempts)
        self.attempts.append(command)
        if write_index in self.failure_write_indices:
            raise OSError("captured motor writer failure")
        self.commands.append(command)


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


def inspect_capture(capture_path: str | Path) -> dict[str, object]:
    """Inspect a native V3 capture."""

    if Path(capture_path).suffix.lower() == ".mcap":
        from .test_hub_v2 import inspect_mcap
        return inspect_mcap(capture_path, deep=True)
    path = _regular_file(capture_path, "capture")
    try:
        return inspect_general_capture(path)
    except V3CaptureError as exc:
        raise V3ReplayError(str(exc)) from exc


def replay_capture(
    capture_path: str | Path,
    *,
    selection: ReplaySelection | None = None,
    project_root: str | Path | None = None,
    capture_source_manifest_path: str | Path | None = None,
) -> dict[str, object]:
    """Replay a V3 capture through current production source without live I/O."""

    if Path(capture_path).suffix.lower() == ".mcap":
        from .mcap_replay_bridge import ReplayWindow, replay_mcap
        selected = selection or ReplaySelection()
        return replay_mcap(capture_path, window=ReplayWindow(
            requested_start_tick_id=selected.start_tick_id,
            requested_end_tick_id=selected.end_tick_id,
            requested_start_ns=selected.start_monotonic_ns,
            requested_end_ns=selected.end_monotonic_ns,
            start_layer=selected.start_layer, end_layer=selected.end_layer,
        ), project_root=project_root, capture_source_manifest_path=capture_source_manifest_path)
    path = _regular_file(capture_path, "capture")
    selected = selection or ReplaySelection()
    try:
        general_payload = load_capture(path)
        ticks = validate_general_capture(general_payload)
    except V3CaptureError as exc:
        raise V3ReplayError(str(exc)) from exc

    entries = tuple(_production_entry(tick) for tick in ticks)
    indices = _selection_indices(ticks, selected)
    execution_entries = entries[: indices[-1] + 1]
    closed_inputs = tuple(
        entry.inputs for entry in execution_entries if entry.inputs is not None
    )
    config, config_authority = _production_control_config(
        general_payload,
        closed_inputs,
    )
    checkpoint = _production_checkpoint(general_payload)

    first_results, first_writes = _run_native_replay(
        execution_entries,
        config,
        checkpoint,
    )
    second_results, second_writes = _run_native_replay(
        execution_entries,
        config,
        checkpoint,
    )
    repeated = first_results == second_results and first_writes == second_writes
    selected_ticks = tuple(ticks[index] for index in indices)
    selected_results = tuple(first_results[index] for index in indices)
    divergence, layer_rows = _general_diagnostics(
        selected_ticks,
        selected_results,
        selected.layers,
    )

    integrity = general_payload.get("capture_integrity")
    replay_eligible = not isinstance(integrity, Mapping) or (
        integrity.get("complete") is True
        and integrity.get("replay_match_eligible") is True
    )
    status = (
        V3_REPLAY_STATUS_MATCH
        if divergence is None and repeated and replay_eligible
        else V3_REPLAY_STATUS_MISMATCH
    )
    if divergence is None and not replay_eligible:
        divergence = {
            "tick_id": None,
            "layer": "Capture",
            "field_path": "capture_integrity",
            "reason": "CAPTURE_INCOMPLETE",
            "expected": {"complete": True, "replay_match_eligible": True},
            "actual": integrity,
            "evidence": {"capture_integrity": integrity},
        }
    if divergence is None and not repeated:
        divergence = {
            "tick_id": None,
            "layer": "TickEngine",
            "reason": "REPEATED_REPLAY_MISMATCH",
            "expected": "identical production traces and sink values",
            "actual": "different repeated replay result",
        }

    root = Path(project_root).resolve() if project_root is not None else Path.cwd()
    source_first = _source_first_evidence(root, capture_source_manifest_path)
    first_live_incident = _first_live_incident(selected_ticks)
    physical_root_cause = _physical_root_cause(
        general_payload,
        first_live_incident,
        divergence,
    )
    result: dict[str, object] = {
        "schema": V3_REPLAY_RESULT_SCHEMA,
        "status": status,
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "capture": {
            "path": str(path.resolve()),
            "capture_id": general_payload.get("capture_id"),
            "sha256": _sha256_file(path),
            "payload_sha256": general_payload.get("capture_sha256"),
            "schema": V3_CAPTURE_SCHEMA,
            "tick_count": len(ticks),
            "status": general_payload.get("status"),
        },
        "scope": _selection_summary(ticks, indices, selected),
        "execution": _general_execution_summary(
            selected_results,
            str(general_payload["status"]),
        ),
        "reconstruction_contract": {
            "inputs": "CURRENT_SOURCE_TYPED_TICK_INPUTS",
            "configuration": config_authority,
            "checkpoint": "CURRENT_SOURCE_TYPED_CHECKPOINT",
            "production": "NATIVE_V3_L1_L12",
            "external_io": "NONE",
        },
        "determinism": {
            "production_layers": "L1-L12",
            "executed_tick_count": len(first_results),
            "selected_tick_count": len(indices),
            "repeated_trace_match": repeated,
            "offline_write_count_first": len(first_writes),
            "offline_write_count_second": len(second_writes),
        },
        "diagnostics": {
            "layers": layer_rows,
            "first_divergence": divergence,
            "first_live_incident": first_live_incident,
            "physical_root_cause": physical_root_cause,
        },
        "source_first": source_first,
        "first_divergence": divergence,
        "first_live_incident": first_live_incident,
        "physical_root_cause": physical_root_cause,
    }
    result["result_sha256"] = _payload_sha256(result)
    return result


def verify_replay_result(result_path: str | Path) -> dict[str, object]:
    """Verify the result checksum and its direct-value MATCH gates."""

    path = _regular_file(result_path, "replay result")
    result = _json_object(path, "replay result")
    expected = result.get("result_sha256")
    unsigned = dict(result)
    unsigned.pop("result_sha256", None)
    checksum_ok = isinstance(expected, str) and expected == _payload_sha256(unsigned)
    determinism = result.get("determinism")
    repeat_match = bool(
        isinstance(determinism, Mapping)
        and determinism.get("repeated_trace_match") is True
    )
    valid = bool(
        result.get("schema") == V3_REPLAY_RESULT_SCHEMA
        and result.get("status") == V3_REPLAY_STATUS_MATCH
        and result.get("first_divergence") is None
        and repeat_match
        and checksum_ok
    )
    return {
        "status": "PASS" if valid else "FAIL",
        "result_path": str(path.resolve()),
        "result_schema": result.get("schema"),
        "replay_status": result.get("status"),
        "checksum_ok": checksum_ok,
        "repeated_trace_match": repeat_match,
        "first_divergence": result.get("first_divergence"),
    }


def write_replay_result(result: Mapping[str, object], output_path: str | Path) -> Path:
    """Atomically write one run-scoped replay result outside the capture."""

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


# ---------------------------------------------------------------------------
# Source-following reconstruction
# ---------------------------------------------------------------------------


def _decode_production_value(value: object, expected_type: object, name: str) -> object:
    """Decode capture data from the current production type definition.

    This is deliberately generic. It contains no L2/L3/L4/L11 checkpoint field
    list, no NativeControlCompositionConfig field list, and no TickInputs field
    list. Those shapes are owned by the current source dataclasses/type hints.
    """

    if expected_type in (Any, object):
        return value

    origin = get_origin(expected_type)
    args = get_args(expected_type)

    if origin in (typing.Union, types.UnionType):
        if value is None and type(None) in args:
            return None
        failures: list[str] = []
        for member in args:
            if member is type(None):
                continue
            try:
                return _decode_production_value(value, member, name)
            except V3ReplayError as exc:
                failures.append(str(exc))
        raise V3ReplayError(
            f"{name} does not match the current production union contract"
            + (f": {'; '.join(failures)}" if failures else "")
        )

    if origin is typing.Annotated:
        if not args:
            return value
        return _decode_production_value(value, args[0], name)

    if origin is typing.Literal:
        if value not in args:
            raise V3ReplayError(f"{name} is not one of the current literal values")
        return value

    if origin is tuple:
        values = _sequence(value, name)
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(
                _decode_production_value(item, args[0], f"{name}[]")
                for item in values
            )
        if args and len(values) != len(args):
            raise V3ReplayError(
                f"{name} must contain {len(args)} values for the current source contract"
            )
        if not args:
            return tuple(values)
        return tuple(
            _decode_production_value(item, item_type, f"{name}[{index}]")
            for index, (item, item_type) in enumerate(zip(values, args))
        )

    if origin is Sequence:
        values = _sequence(value, name)
        item_type = args[0] if args else Any
        return tuple(
            _decode_production_value(item, item_type, f"{name}[]")
            for item in values
        )

    if origin is list:
        values = _sequence(value, name)
        item_type = args[0] if args else Any
        return [
            _decode_production_value(item, item_type, f"{name}[]")
            for item in values
        ]

    if origin in (set, frozenset):
        values = _sequence(value, name)
        item_type = args[0] if args else Any
        converted = (
            _decode_production_value(item, item_type, f"{name}[]")
            for item in values
        )
        return set(converted) if origin is set else frozenset(converted)

    if origin in (dict, Mapping, typing.Mapping):
        row = _mapping(value, name)
        key_type = args[0] if len(args) > 0 else Any
        value_type = args[1] if len(args) > 1 else Any
        return {
            _decode_production_value(key, key_type, f"{name}.<key>"): _decode_production_value(
                item,
                value_type,
                f"{name}[{key!r}]",
            )
            for key, item in row.items()
        }

    if expected_type is type(None):
        if value is not None:
            raise V3ReplayError(f"{name} must be null")
        return None

    if isinstance(expected_type, type) and issubclass(expected_type, Enum):
        try:
            return expected_type(value)
        except (TypeError, ValueError) as exc:
            raise V3ReplayError(
                f"{name} is not valid for current {expected_type.__name__}"
            ) from exc

    if isinstance(expected_type, type) and is_dataclass(expected_type):
        row = _mapping(value, name)
        _require_type(row, expected_type.__name__, name)
        try:
            hints = get_type_hints(expected_type)
        except (NameError, TypeError) as exc:
            raise V3ReplayError(
                f"cannot resolve current source type hints for {expected_type.__name__}: {exc}"
            ) from exc

        source_fields = {field.name: field for field in fields(expected_type)}
        unknown = sorted(set(row) - {"__type__"} - set(source_fields))
        if unknown:
            raise V3ReplayError(
                f"{name} contains fields absent from current source {expected_type.__name__}: "
                + ", ".join(unknown)
            )

        kwargs: dict[str, object] = {}
        deferred: dict[str, object] = {}
        for field_name, field in source_fields.items():
            if field_name not in row:
                if (
                    not field.init
                    or field.default is not MISSING
                    or field.default_factory is not MISSING
                ):
                    continue
                raise V3ReplayError(
                    f"{name} lacks current source field {expected_type.__name__}.{field_name}"
                )
            field_type = hints.get(field_name, Any)
            decoded_field = _decode_production_value(
                row[field_name],
                field_type,
                f"{name}.{field_name}",
            )
            if field.init:
                kwargs[field_name] = decoded_field
            else:
                deferred[field_name] = decoded_field
        try:
            instance = expected_type(**kwargs)
        except (TypeError, ValueError) as exc:
            raise V3ReplayError(
                f"{name} violates current production {expected_type.__name__}: {exc}"
            ) from exc
        for field_name, captured_value in deferred.items():
            if getattr(instance, field_name) != captured_value:
                raise V3ReplayError(
                    f"{name}.{field_name} differs from the current production computed field"
                )
        return instance

    if expected_type is bool:
        if type(value) is not bool:
            raise V3ReplayError(f"{name} must be bool")
        return value
    if expected_type is int:
        if not isinstance(value, int) or isinstance(value, bool):
            raise V3ReplayError(f"{name} must be integer")
        return value
    if expected_type is float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise V3ReplayError(f"{name} must be finite numeric")
        return float(value)
    if expected_type is str:
        if not isinstance(value, str):
            raise V3ReplayError(f"{name} must be string")
        return value

    if isinstance(expected_type, type):
        if not isinstance(value, expected_type):
            raise V3ReplayError(
                f"{name} must satisfy current source type {expected_type.__name__}"
            )
        return value

    raise V3ReplayError(
        f"{name} uses unsupported current source annotation {expected_type!r}"
    )


def _find_unique_typed_value(value: object, type_name: str) -> Mapping[str, object] | None:
    matches: list[Mapping[str, object]] = []

    def visit(node: object) -> None:
        if isinstance(node, Mapping):
            if node.get("__type__") == type_name:
                matches.append(node)
            for child in node.values():
                visit(child)
        elif isinstance(node, Sequence) and not isinstance(node, (str, bytes)):
            for child in node:
                visit(child)

    visit(value)
    if len(matches) > 1:
        raise V3ReplayError(
            f"capture configuration contains multiple {type_name} values"
        )
    return matches[0] if matches else None


def _production_inputs(tick: Mapping[str, object]) -> TickInputs:
    encoded = tick.get("inputs")
    decoded = _decode_production_value(encoded, TickInputs, "tick.inputs")
    if not isinstance(decoded, TickInputs):
        raise V3ReplayError("current source did not decode TickInputs")
    if decoded.context.tick_id != _integer(tick.get("tick_id"), "tick.tick_id"):
        raise V3ReplayError("captured TickInputs tick_id mismatch")
    if decoded.context.monotonic_ns != _integer(
        tick.get("monotonic_ns"),
        "tick.monotonic_ns",
    ):
        raise V3ReplayError("captured TickInputs monotonic_ns mismatch")
    return decoded


def _production_checkpoint(
    payload: Mapping[str, object],
) -> NativeControlStateCheckpoint | None:
    value = payload.get("initial_state_checkpoint")
    if value is None:
        return None
    decoded = _decode_production_value(
        value,
        NativeControlStateCheckpoint,
        "initial_state_checkpoint",
    )
    if not isinstance(decoded, NativeControlStateCheckpoint):
        raise V3ReplayError("current source did not decode NativeControlStateCheckpoint")
    return decoded


def _production_entry(tick: Mapping[str, object]) -> _ReplayEntry:
    record_type = tick.get("record_type", "closed_input_tick")
    expected = _mapping(tick.get("expected"), "tick.expected")
    writer_failure = isinstance(expected.get("writer_failure"), Mapping)
    if record_type == "closed_input_tick":
        inputs = _production_inputs(tick)
        return _ReplayEntry(
            context=inputs.context,
            inputs=inputs,
            lifecycle=inputs.lifecycle,
            writer_failure=writer_failure,
        )
    if record_type != "edge_fault_tick":
        raise V3ReplayError("capture tick has an unsupported record_type")
    edge = _mapping(tick.get("edge_fault"), "tick.edge_fault")
    context = _decode_production_value(
        edge.get("context"),
        TickContext,
        "tick.edge_fault.context",
    )
    lifecycle = _decode_production_value(
        edge.get("lifecycle"),
        LifecycleState,
        "tick.edge_fault.lifecycle",
    )
    health = _decode_production_value(
        edge.get("critical_health", ()),
        tuple[DeviceHealth, ...],
        "tick.edge_fault.critical_health",
    )
    if not isinstance(context, TickContext):
        raise V3ReplayError("edge fault context did not decode to TickContext")
    if not isinstance(lifecycle, LifecycleState):
        raise V3ReplayError("edge fault lifecycle did not decode to LifecycleState")
    if not isinstance(health, tuple) or any(
        not isinstance(item, DeviceHealth) for item in health
    ):
        raise V3ReplayError("edge fault health did not decode to DeviceHealth values")
    reason = edge.get("reason")
    fault_layer = edge.get("fault_layer")
    if not isinstance(reason, str) or not reason:
        raise V3ReplayError("edge fault reason must be non-empty")
    if not isinstance(fault_layer, str) or not fault_layer:
        raise V3ReplayError("edge fault layer must be non-empty")
    return _ReplayEntry(
        context=context,
        inputs=None,
        lifecycle=lifecycle,
        reason=reason,
        fault_layer=fault_layer,
        critical_health=health,
        writer_failure=writer_failure,
    )


def _production_control_config(
    payload: Mapping[str, object],
    inputs: Sequence[TickInputs],
) -> tuple[NativeControlCompositionConfig, str]:
    """Use the current production config type whenever capture stored a typed config.

    Modern production captures contain resolved runtime/config dataclasses. We do
    not reconstruct their nested fields here: the current
    NativeControlCompositionConfig annotations are the authority. The document
    branch only preserves compatibility with older/test captures that predate a
    typed resolved config.
    """

    configuration = _mapping(payload.get("configuration"), "capture.configuration")
    encoded = _find_unique_typed_value(
        configuration,
        NativeControlCompositionConfig.__name__,
    )
    if encoded is not None:
        decoded = _decode_production_value(
            encoded,
            NativeControlCompositionConfig,
            "capture.configuration.production_control",
        )
        if not isinstance(decoded, NativeControlCompositionConfig):
            raise V3ReplayError("current source did not decode production control config")
        return decoded, "CURRENT_SOURCE_TYPED_CONFIG"

    # Compatibility only: older validation captures store the four source JSON
    # documents rather than a typed resolved config. This path intentionally
    # stays isolated from the modern runtime-following path.
    documents_value = configuration.get("legacy_documents", configuration)
    documents = _mapping(documents_value, "capture.configuration.documents")
    physics = _mapping(documents.get("physics"), "capture.configuration.physics")
    speed_map = _mapping(
        documents.get("speed_map"),
        "capture.configuration.speed_map",
    )
    hardware = _mapping(
        documents.get("hardware"),
        "capture.configuration.hardware",
    )
    control_value = documents.get("control")
    control = (
        _mapping(control_value, "capture.configuration.control")
        if control_value is not None
        else None
    )
    return (
        _compat_control_config_from_documents(
            physics,
            speed_map,
            hardware,
            inputs,
            control,
        ),
        "DOCUMENT_COMPATIBILITY",
    )


def _compat_control_config_from_documents(
    physics: Mapping[str, object],
    speed_map_raw: Mapping[str, object],
    hardware: Mapping[str, object],
    inputs: Sequence[TickInputs],
    control: Mapping[str, object] | None = None,
) -> NativeControlCompositionConfig:
    track_width = _number(
        physics.get("nyomtav_szelesseg_m"),
        "physics.nyomtav_szelesseg_m",
    )
    if track_width <= 0.0:
        raise V3ReplayError("physics.nyomtav_szelesseg_m must be positive")
    lidar_safety = _compat_lidar_safety_config(hardware, inputs)
    navigation = (
        v3_navigation_config_from_mapping(dict(control))
        if control is not None and "v3_navigation" in control
        else None
    )
    navigation_kwargs = (
        {
            "world_model": navigation.world_model,
            "navigation": navigation.navigation,
        }
        if navigation is not None
        else {}
    )
    return NativeControlCompositionConfig(
        speed_map=WheelSpeedMap.from_mapping(speed_map_raw),
        estimation=NativeStateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=track_width,
        ),
        chassis_control=ChassisControlConfig(track_width),
        lidar_safety=lidar_safety,
        **navigation_kwargs,
    )


def _compat_lidar_safety_config(
    hardware: Mapping[str, object],
    inputs: Sequence[TickInputs],
) -> LidarSafetyConfig:
    device_ids = {
        sample.device_id
        for tick_input in inputs
        for sample in tick_input.raw_devices.samples
        if sample.kind == "lidar_safety_clearance"
    }
    if not device_ids:
        raise V3ReplayError("capture lacks native lidar safety samples")
    if len(device_ids) != 1:
        raise V3ReplayError(
            "capture contains lidar safety samples from multiple device IDs"
        )
    lidar = _mapping(hardware.get("lidar"), "hardware config lidar")
    minimum_clearance_m = _number(
        lidar.get("biztonsagi_zona_m"),
        "hardware.lidar.biztonsagi_zona_m",
    )
    if minimum_clearance_m <= 0.0:
        raise V3ReplayError("hardware.lidar.biztonsagi_zona_m must be positive")
    return LidarSafetyConfig(
        device_id=next(iter(device_ids)),
        minimum_clearance_m=minimum_clearance_m,
        maximum_sample_age_ns=250_000_000,
    )


# ---------------------------------------------------------------------------
# Execution and diagnostics
# ---------------------------------------------------------------------------


def _run_native_replay(
    entries: tuple[_ReplayEntry, ...],
    config: NativeControlCompositionConfig,
    checkpoint: NativeControlStateCheckpoint | None = None,
) -> tuple[tuple[_ReplayOutcome, ...], tuple[object, ...]]:
    failure_indices = frozenset(
        index for index, entry in enumerate(entries) if entry.writer_failure
    )
    writer = _RecordingWriter(failure_indices)
    composition = NativeControlComposition(writer, config)
    if checkpoint is not None:
        composition.restore(checkpoint)
    outcomes: list[_ReplayOutcome] = []
    for entry in entries:
        try:
            if entry.inputs is not None:
                result = composition.run_tick(entry.inputs)
            else:
                if entry.reason is None or entry.fault_layer is None:
                    raise V3ReplayError("edge fault replay entry is incomplete")
                result = composition.run_fault_tick(
                    entry.context,
                    entry.lifecycle,
                    entry.reason,
                    entry.fault_layer,
                    entry.critical_health,
                )
        except Exception as exc:
            from .engine import TickExecutionError

            if not isinstance(exc, TickExecutionError):
                raise
            outcomes.append(_ReplayOutcome(None, True, exc.attempted_actuation))
            if not entry.writer_failure:
                break
        else:
            outcomes.append(_ReplayOutcome(result, False, None))
    return tuple(outcomes), tuple(writer.attempts)


def _layer_index(layer: str) -> int:
    try:
        return _LAYER_ORDER.index(str(layer).upper())
    except ValueError as exc:
        raise ValueError(f"layer must be one of {', '.join(_LAYER_ORDER)}") from exc


def _selection_indices(
    ticks: Sequence[Mapping[str, object]],
    selection: ReplaySelection,
) -> tuple[int, ...]:
    indices = tuple(
        index
        for index, tick in enumerate(ticks)
        if selection.includes(
            _integer(tick.get("tick_id"), "tick.tick_id"),
            _integer(tick.get("monotonic_ns"), "tick.monotonic_ns"),
        )
    )
    if not indices:
        raise V3ReplayError("replay selection contains no capture ticks")
    return indices


def _selection_summary(
    ticks: Sequence[Mapping[str, object]],
    indices: Sequence[int],
    selection: ReplaySelection,
) -> dict[str, object]:
    first = ticks[indices[0]]
    last = ticks[indices[-1]]
    return {
        "requested": {
            "start_tick_id": selection.start_tick_id,
            "end_tick_id": selection.end_tick_id,
            "start_monotonic_ns": selection.start_monotonic_ns,
            "end_monotonic_ns": selection.end_monotonic_ns,
            "start_layer": selection.start_layer,
            "end_layer": selection.end_layer,
        },
        "resolved": {
            "first_tick_id": first["tick_id"],
            "last_tick_id": last["tick_id"],
            "first_monotonic_ns": first["monotonic_ns"],
            "last_monotonic_ns": last["monotonic_ns"],
            "tick_count": len(indices),
            "layers": list(selection.layers),
        },
        "state_warmup": {
            "strategy": "CAPTURE_PREFIX",
            "tick_count": indices[0],
            "first_tick_id": ticks[0]["tick_id"] if indices[0] else None,
            "last_tick_id": ticks[indices[0] - 1]["tick_id"] if indices[0] else None,
        },
    }


def _general_diagnostics(
    ticks: Sequence[Mapping[str, object]],
    outcomes: Sequence[_ReplayOutcome],
    layers: Sequence[str],
) -> tuple[dict[str, object] | None, dict[str, object]]:
    layer_rows: dict[str, object] = {
        layer: {
            "compared_tick_count": 0,
            "not_executed_tick_count": 0,
            "expected_present_count": 0,
            "actual_present_count": 0,
            "mismatch_count": 0,
        }
        for layer in layers
    }
    first: dict[str, object] | None = None
    if len(ticks) != len(outcomes):
        first = {
            "tick_id": None,
            "layer": "TickEngine",
            "field_path": "ticks",
            "reason": "TICK_COUNT_MISMATCH",
            "expected": len(ticks),
            "actual": len(outcomes),
            "evidence": {"capture_tick_count": len(ticks)},
        }
        return first, layer_rows

    for tick, outcome in zip(ticks, outcomes):
        expected = _mapping(tick.get("expected"), "tick.expected")
        expected_writer_failure = expected.get("writer_failure")
        if isinstance(expected_writer_failure, Mapping):
            expected_attempt = expected_writer_failure.get("attempted_actuation")
            actual_attempt = _capture_value(outcome.attempted_actuation)
            if not outcome.writer_failure or expected_attempt != actual_attempt:
                difference = _first_value_difference(expected_attempt, actual_attempt)
                path, expected_leaf, actual_leaf = difference or (
                    "writer_failure",
                    True,
                    outcome.writer_failure,
                )
                if first is None:
                    first = _divergence_row(
                        tick,
                        "L12",
                        f"L12.writer_failure.{path}",
                        "WRITER_FAILURE_MISMATCH",
                        expected_leaf,
                        actual_leaf,
                    )
            row = dict(_mapping(layer_rows.get("L12", {}), "diagnostics.L12"))
            if row:
                row["compared_tick_count"] = int(row["compared_tick_count"]) + 1
                if first is not None and first.get("tick_id") == tick.get("tick_id"):
                    row["mismatch_count"] = int(row["mismatch_count"]) + 1
                layer_rows["L12"] = row
            continue

        result = outcome.result
        if result is None:
            if first is None:
                first = _divergence_row(
                    tick,
                    "L12",
                    "L12.writer",
                    "UNEXPECTED_WRITER_FAILURE",
                    "completed L12",
                    "writer failure",
                )
            continue

        expected_layers = _expanded_expected_layers(
            tick,
            _mapping(expected.get("layers"), "tick.expected.layers"),
        )
        actual_layers = {
            record.layer: _capture_value(record.output)
            for record in result.trace.layers
        }
        for layer in layers:
            expected_present = layer in expected_layers
            actual_present = layer in actual_layers
            row = dict(_mapping(layer_rows[layer], f"diagnostics.{layer}"))
            row["expected_present_count"] = int(row["expected_present_count"]) + int(
                expected_present
            )
            row["actual_present_count"] = int(row["actual_present_count"]) + int(
                actual_present
            )
            if not expected_present and not actual_present:
                row["not_executed_tick_count"] = int(
                    row["not_executed_tick_count"]
                ) + 1
                layer_rows[layer] = row
                continue
            if expected_present and actual_present:
                row["compared_tick_count"] = int(row["compared_tick_count"]) + 1
                if expected_layers[layer] == actual_layers[layer]:
                    layer_rows[layer] = row
                    continue
            row["mismatch_count"] = int(row["mismatch_count"]) + 1
            layer_rows[layer] = row
            if first is None:
                expected_value = (
                    expected_layers[layer] if expected_present else "NOT_EXECUTED"
                )
                actual_value = (
                    actual_layers[layer] if actual_present else "NOT_EXECUTED"
                )
                difference = _first_value_difference(expected_value, actual_value)
                path, expected_leaf, actual_leaf = difference or (
                    "",
                    expected_value,
                    actual_value,
                )
                first = _divergence_row(
                    tick,
                    layer,
                    layer if not path else f"{layer}.{path}",
                    (
                        "DIRECT_VALUE_MISMATCH"
                        if expected_present and actual_present
                        else "LAYER_PRESENCE_MISMATCH"
                    ),
                    expected_leaf,
                    actual_leaf,
                )

        expected_fault = expected.get("fault_layer")
        actual_fault = result.trace.fault_layer
        if (
            expected_fault != actual_fault
            and (
                expected_fault in layers
                or actual_fault in layers
                or tuple(layers) == _LAYER_ORDER
            )
            and first is None
        ):
            first = _divergence_row(
                tick,
                "TickEngine",
                "TickEngine.fault_layer",
                "FAULT_LAYER_MISMATCH",
                expected_fault,
                actual_fault,
            )
    return first, layer_rows


def _expanded_expected_layers(
    tick: Mapping[str, object],
    layers: Mapping[str, object],
) -> dict[str, object]:
    """Expand the two measured input duplications before field diagnostics."""

    expanded = dict(layers)
    inputs = tick.get("inputs")
    if not isinstance(inputs, Mapping):
        return expanded
    raw = inputs.get("raw_devices")
    if not isinstance(raw, Mapping):
        return expanded
    for layer, value in tuple(expanded.items()):
        if not isinstance(value, Mapping):
            continue
        reference = value.get(_INPUT_REFERENCE_KEY)
        if reference == "RAW_DEVICE_BATCH":
            expanded[layer] = {
                "__type__": "AcquisitionFrame",
                "context": copy.deepcopy(inputs.get("context")),
                "samples": copy.deepcopy(raw.get("samples")),
                "io_health": copy.deepcopy(raw.get("device_health")),
            }
            continue
        if reference != "ADMITTED_FRAME":
            continue
        samples = _sequence(raw.get("samples"), "tick.inputs.raw_devices.samples")
        indices = _sequence(
            value.get("accepted_sample_indices"),
            "tick.expected.layers.L2.accepted_sample_indices",
        )
        accepted: list[dict[str, object]] = []
        for encoded_index in indices:
            index = _integer(
                encoded_index,
                "tick.expected.layers.L2.accepted_sample_indices[]",
            )
            if index < 0 or index >= len(samples):
                raise V3ReplayError("captured L2 input reference is out of range")
            sample = _mapping(samples[index], "tick.inputs.raw_devices.samples[]")
            accepted.append(
                {
                    "__type__": "Observation",
                    "kind": sample.get("kind"),
                    "source_device_id": sample.get("device_id"),
                    "source_sequence": sample.get("sequence"),
                    "captured_monotonic_ns": sample.get("captured_monotonic_ns"),
                    "values": copy.deepcopy(sample.get("values")),
                }
            )
        expanded[layer] = {
            "__type__": "AdmittedFrame",
            "context": copy.deepcopy(inputs.get("context")),
            "accepted": accepted,
            "rejected": copy.deepcopy(value.get("rejected")),
            "degraded_sources": copy.deepcopy(value.get("degraded_sources")),
        }
    return expanded


def _divergence_row(
    tick: Mapping[str, object],
    layer: str,
    field_path: str,
    reason: str,
    expected: object,
    actual: object,
) -> dict[str, object]:
    return {
        "tick_id": tick.get("tick_id"),
        "layer": layer,
        "field_path": field_path,
        "reason": reason,
        "expected": expected,
        "actual": actual,
        "evidence": _tick_evidence(tick),
    }


def _first_value_difference(
    expected: object,
    actual: object,
    prefix: str = "",
) -> tuple[str, object, object] | None:
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        keys = list(expected)
        keys.extend(sorted(str(key) for key in actual if key not in expected))
        for key in keys:
            path = str(key) if not prefix else f"{prefix}.{key}"
            if key not in expected:
                return path, "MISSING", actual[key]
            if key not in actual:
                return path, expected[key], "MISSING"
            difference = _first_value_difference(expected[key], actual[key], path)
            if difference is not None:
                return difference
        return None
    if (
        isinstance(expected, Sequence)
        and not isinstance(expected, (str, bytes))
        and isinstance(actual, Sequence)
        and not isinstance(actual, (str, bytes))
    ):
        for index in range(max(len(expected), len(actual))):
            path = f"[{index}]" if not prefix else f"{prefix}[{index}]"
            if index >= len(expected):
                return path, "MISSING", actual[index]
            if index >= len(actual):
                return path, expected[index], "MISSING"
            difference = _first_value_difference(expected[index], actual[index], path)
            if difference is not None:
                return difference
        return None
    if expected != actual:
        return prefix, expected, actual
    return None


def _tick_evidence(tick: Mapping[str, object]) -> dict[str, object]:
    evidence: dict[str, object] = {
        "record_type": tick.get("record_type", "closed_input_tick"),
        "input_reference": f"ticks[{tick.get('tick_id')}].inputs",
    }
    if "tick_evidence" in tick:
        evidence["ekf_updates"] = tick.get("tick_evidence")
    raw: object | None = None
    inputs = tick.get("inputs")
    if isinstance(inputs, Mapping):
        raw = inputs.get("raw_devices")
    else:
        edge = tick.get("edge_fault")
        if isinstance(edge, Mapping):
            raw = edge.get("raw_devices")
            evidence["edge_fault"] = {
                "reason": edge.get("reason"),
                "fault_layer": edge.get("fault_layer"),
            }
    if isinstance(raw, Mapping):
        evidence["device_health"] = raw.get("device_health", ())
        samples = raw.get("samples", ())
        if isinstance(samples, Sequence) and not isinstance(samples, (str, bytes)):
            evidence["sensor_samples"] = [
                sample
                for sample in samples
                if isinstance(sample, Mapping)
                and str(sample.get("kind", "")).startswith(
                    ("wheel_velocity", "imu_", "lidar_")
                )
            ]
    return evidence


def _first_live_incident(
    ticks: Sequence[Mapping[str, object]],
) -> dict[str, object] | None:
    for tick in ticks:
        expected = _mapping(tick.get("expected"), "tick.expected")
        edge = tick.get("edge_fault")
        if isinstance(edge, Mapping):
            return {
                "tick_id": tick.get("tick_id"),
                "layer": edge.get("fault_layer"),
                "reason": edge.get("reason"),
                "evidence": _tick_evidence(tick),
            }
        writer_failure = expected.get("writer_failure")
        if isinstance(writer_failure, Mapping):
            return {
                "tick_id": tick.get("tick_id"),
                "layer": "L12",
                "reason": writer_failure.get("reason"),
                "evidence": _tick_evidence(tick),
            }
        raw = tick.get("inputs")
        if isinstance(raw, Mapping):
            devices = raw.get("raw_devices")
            health = devices.get("device_health") if isinstance(devices, Mapping) else None
            if isinstance(health, Sequence) and not isinstance(health, (str, bytes)):
                degraded = next(
                    (
                        row
                        for row in health
                        if isinstance(row, Mapping) and row.get("state") != "OK"
                    ),
                    None,
                )
                if degraded is not None:
                    return {
                        "tick_id": tick.get("tick_id"),
                        "layer": "L0",
                        "reason": degraded.get("reason") or degraded.get("state"),
                        "device_id": degraded.get("device_id"),
                        "evidence": _tick_evidence(tick),
                    }
        fault_layer = expected.get("fault_layer")
        layers = expected.get("layers")
        l12 = layers.get("L12") if isinstance(layers, Mapping) else None
        decision = l12.get("safety_decision") if isinstance(l12, Mapping) else None
        if fault_layer is not None or decision == "FAULT":
            return {
                "tick_id": tick.get("tick_id"),
                "layer": fault_layer or "L12",
                "reason": l12.get("reason") if isinstance(l12, Mapping) else None,
                "evidence": _tick_evidence(tick),
            }
    return None


def _physical_root_cause(
    payload: Mapping[str, object],
    incident: Mapping[str, object] | None,
    divergence: Mapping[str, object] | None,
) -> dict[str, object]:
    reason = str(incident.get("reason") or "") if incident is not None else ""
    if reason == "MOTOR_WRITER_FAILURE":
        return {
            "status": "PROVEN",
            "cause": "MOTOR_WRITER_FAILURE",
            "reason": "THE_CANONICAL_L12_WRITE_RAISED",
            "evidence": incident.get("evidence"),
        }
    raw = payload.get("raw_lidar_evidence")
    missing = raw.get("missing_revisions") if isinstance(raw, Mapping) else ()
    if (
        isinstance(missing, Sequence)
        and not isinstance(missing, (str, bytes))
        and missing
    ):
        return {
            "status": "NOT_PROVEN",
            "cause": None,
            "reason": "REFERENCED_RAW_LIDAR_MISSING",
            "evidence": {"missing_raw_lidar_revisions": list(missing)},
        }
    if incident is None:
        return {
            "status": "NOT_PROVEN",
            "cause": None,
            "reason": "NO_LIVE_INCIDENT_IN_SCOPE",
            "evidence": None,
        }
    evidence = incident.get("evidence")
    device_health = evidence.get("device_health") if isinstance(evidence, Mapping) else None
    has_degraded_health = bool(
        isinstance(device_health, Sequence)
        and not isinstance(device_health, (str, bytes))
        and any(
            isinstance(row, Mapping) and row.get("state") != "OK"
            for row in device_health
        )
    )
    if has_degraded_health:
        return {
            "status": "INDICATED",
            "cause": reason or "DEVICE_HEALTH_DEGRADATION",
            "reason": "EDGE_HEALTH_AND_TICK_EVIDENCE_AGREE",
            "evidence": evidence,
        }
    return {
        "status": "NOT_PROVEN",
        "cause": None,
        "reason": (
            "REPLAY_DIVERGENCE_IS_NOT_PHYSICAL_PROOF"
            if divergence is not None
            else "INSUFFICIENT_PHYSICAL_EDGE_EVIDENCE"
        ),
        "evidence": evidence,
    }


def _general_execution_summary(
    outcomes: Sequence[_ReplayOutcome],
    capture_status: str,
) -> dict[str, object]:
    decision_counts = {item.value: 0 for item in SafetyDecision}
    results = tuple(
        outcome.result for outcome in outcomes if outcome.result is not None
    )
    for result in results:
        decision_counts[result.final_actuation.safety_decision.value] += 1
    terminal = results[-1] if results else None
    return {
        "capture_status": capture_status,
        "capture_passed": capture_status == "PASS",
        "decision_counts": decision_counts,
        "writer_failure_count": sum(
            int(outcome.writer_failure) for outcome in outcomes
        ),
        "terminal_tick_id": terminal.trace.context.tick_id if terminal is not None else None,
        "terminal_safety_decision": (
            terminal.final_actuation.safety_decision.value
            if terminal is not None
            else None
        ),
        "terminal_reason": terminal.final_actuation.reason if terminal is not None else None,
        "terminal_fault_layer": terminal.trace.fault_layer if terminal is not None else "L12",
    }


def _module_name_for_source(relative: Path) -> tuple[str, bool]:
    parts = list(relative.with_suffix("").parts)
    is_package = bool(parts and parts[-1] == "__init__")
    if is_package:
        parts.pop()
    return ".".join(parts), is_package


def _local_module_path(root: Path, module_name: str) -> Path | None:
    if not module_name:
        return None
    parts = module_name.split(".")
    file_path = root.joinpath(*parts).with_suffix(".py")
    package_path = root.joinpath(*parts, "__init__.py")
    if file_path.is_file() and not file_path.is_symlink():
        return file_path
    if package_path.is_file() and not package_path.is_symlink():
        return package_path
    return None


def _resolved_import_modules(
    node: ast.Import | ast.ImportFrom,
    *,
    importer: str,
    is_package: bool,
) -> tuple[str, ...]:
    if isinstance(node, ast.Import):
        return tuple(alias.name for alias in node.names)

    if node.level == 0:
        base = node.module or ""
    else:
        package = importer.split(".") if is_package else importer.split(".")[:-1]
        keep = len(package) - (node.level - 1)
        if keep < 0:
            return ()
        prefix = package[:keep]
        if node.module:
            prefix.extend(node.module.split("."))
        base = ".".join(prefix)

    candidates: list[str] = []
    if base:
        candidates.append(base)
    for alias in node.names:
        if alias.name == "*":
            continue
        candidates.append(f"{base}.{alias.name}" if base else alias.name)
    return tuple(dict.fromkeys(candidates))


def _source_first_paths(root: Path) -> tuple[str, ...]:
    """Derive the current local runtime/replay dependency closure from source imports."""

    root = root.resolve()
    pending: list[Path] = [
        _regular_file(root / relative, relative)
        for relative in _SOURCE_FIRST_ROOTS
    ]
    discovered: set[Path] = set()
    while pending:
        path = pending.pop().resolve()
        if path in discovered:
            continue
        try:
            relative = path.relative_to(root)
        except ValueError as exc:
            raise V3ReplayError("source-first dependency escaped project root") from exc
        discovered.add(path)
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=relative.as_posix())
        except (OSError, UnicodeError, SyntaxError) as exc:
            raise V3ReplayError(
                f"cannot inspect source dependency {relative.as_posix()}: {exc}"
            ) from exc
        importer, is_package = _module_name_for_source(relative)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            for module_name in _resolved_import_modules(
                node, importer=importer, is_package=is_package
            ):
                dependency = _local_module_path(root, module_name)
                if dependency is not None and dependency.resolve() not in discovered:
                    pending.append(dependency)

    paths = {path.relative_to(root).as_posix() for path in discovered}
    for relative in _SOURCE_FIRST_STATIC_PATHS:
        _regular_file(root / relative, relative)
        paths.add(relative)
    return tuple(sorted(paths))


def _source_first_evidence(
    root: Path,
    manifest_path: str | Path | None,
) -> dict[str, object]:
    current: dict[str, object] = {}
    expected_files: Mapping[str, object] = {}
    manifest: Path | None = None
    if manifest_path is not None:
        manifest = _regular_file(manifest_path, "capture source manifest")
        expected_files = _mapping(
            _json_object(manifest, "capture source manifest").get("files"),
            "capture source manifest.files",
        )
    all_match = manifest is not None
    for relative in _source_first_paths(root):
        path = _regular_file(root / relative, relative)
        actual_hash = _sha256_file(path)
        expected_row = expected_files.get(relative)
        expected_hash = (
            str(expected_row.get("sha256"))
            if isinstance(expected_row, Mapping)
            and expected_row.get("sha256") is not None
            else None
        )
        matches = expected_hash == actual_hash if expected_hash is not None else None
        if matches is not True:
            all_match = False
        current[relative] = {
            "sha256": actual_hash,
            "capture_baseline_sha256": expected_hash,
            "capture_baseline_match": matches,
        }
    return {
        "source_order": ["SOURCE", "ACTIVE_CONFIG", "CAPTURE"],
        "source_discovery": "CURRENT_LOCAL_IMPORT_CLOSURE",
        "capture_source_manifest_path": (
            None if manifest is None else str(manifest.resolve())
        ),
        "all_capture_baseline_hashes_match": all_match,
        "files": current,
    }


def _capture_value(value: object) -> object:
    try:
        return encode_value(value)
    except V3CaptureError as exc:
        raise V3ReplayError(str(exc)) from exc


def _require_type(value: Mapping[str, object], expected: str, name: str) -> None:
    if value.get("__type__") != expected:
        raise V3ReplayError(f"{name} must contain {expected}")


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise V3ReplayError(f"{name} must be an object")
    return value


def _sequence(value: object, name: str) -> Sequence[object]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V3ReplayError(f"{name} must be an array")
    return value


def _integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise V3ReplayError(f"{name} must be a non-negative integer")
    return value


def _number(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise V3ReplayError(f"{name} must be finite numeric")
    return float(value)


def _regular_file(path_value: str | Path, name: str) -> Path:
    path = Path(path_value)
    if path.is_symlink() or not path.is_file():
        raise V3ReplayError(f"{name} must be a regular non-symlink file")
    return path


def _json_object(path: Path, name: str) -> Mapping[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise V3ReplayError(f"{name} must contain valid UTF-8 JSON") from exc
    return _mapping(value, name)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(value: Mapping[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect_parser = subparsers.add_parser("inspect", help="inspect a V3 capture")
    inspect_parser.add_argument("capture_path")
    replay_parser = subparsers.add_parser("replay", help="replay production L1-L12")
    replay_parser.add_argument("capture_path")
    replay_parser.add_argument("--project-root", default=".")
    replay_parser.add_argument("--capture-source-manifest")
    replay_parser.add_argument("--start-tick-id", type=int)
    replay_parser.add_argument("--end-tick-id", type=int)
    replay_parser.add_argument("--start-monotonic-ns", type=int)
    replay_parser.add_argument("--end-monotonic-ns", type=int)
    replay_parser.add_argument("--start-layer", default="L1")
    replay_parser.add_argument("--end-layer", default="L12")
    replay_parser.add_argument("--output", required=True)
    verify_parser = subparsers.add_parser("verify-result", help="verify a replay result")
    verify_parser.add_argument("result_path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "inspect":
            output = inspect_capture(args.capture_path)
        elif args.command == "replay":
            selection = ReplaySelection(
                start_tick_id=args.start_tick_id,
                end_tick_id=args.end_tick_id,
                start_monotonic_ns=args.start_monotonic_ns,
                end_monotonic_ns=args.end_monotonic_ns,
                start_layer=args.start_layer,
                end_layer=args.end_layer,
            )
            replay_result = replay_capture(
                args.capture_path,
                selection=selection,
                project_root=args.project_root,
                capture_source_manifest_path=args.capture_source_manifest,
            )
            path = write_replay_result(replay_result, args.output)
            output = {
                "status": replay_result["status"],
                "result_path": str(path.resolve()),
                "result_sha256": replay_result["result_sha256"],
                "first_divergence": replay_result["first_divergence"],
                "scope": replay_result.get("scope"),
            }
        else:
            output = verify_replay_result(args.result_path)
        print(json.dumps(output, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if output.get("status") in {"PASS", V3_REPLAY_STATUS_MATCH} else 2
    except (V3ReplayError, ValueError, TypeError) as exc:
        print(
            json.dumps(
                {"status": "ERROR", "error": str(exc)},
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ReplayDivergence",
    "ReplaySelection",
    "V3ReplayError",
    "first_divergence",
    "inspect_capture",
    "replay_capture",
    "run_replay",
    "verify_replay_result",
    "write_replay_result",
]
