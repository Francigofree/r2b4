"""Direct-value deterministic replay for native V3 captures."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

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
    v3_navigation_config_from_mapping,
)
from .contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
)
from .engine import LayerValue, TickEngine, TickInputs, TickResult, TickTrace
from .execution import ExecutionBoundary, IterableInputSource, MemoryOutputSink
from .layers.l10_chassis_control import ChassisControlConfig
from .layers.l11_actuator_control import WheelSpeedMap
from .layers.l3_state_estimation import NativeStateEstimatorConfig
from .layers.l12_safety_final import LidarSafetyConfig


V3_REPLAY_RESULT_SCHEMA = "R2B4_REPLAYER_V3_RESULT_V3"
V3_REPLAY_STATUS_MATCH = "MATCH"
V3_REPLAY_STATUS_MISMATCH = "MISMATCH"
_LAYER_ORDER = LAYER_ORDER
_SOURCE_FIRST_PATHS = (
    "STRUKTURALIS_RETEGEK_V3.md",
    "conf/hardver.json",
    "conf/fizika.json",
    "conf/speed_map.json",
    "conf/vezerles.json",
    "v3/adapters/bounded_command.py",
    "v3/adapters/latest_lidar.py",
    "v3/adapters/live_lidar.py",
    "v3/adapters/native_lidar_port.py",
    "v3/adapters/resident_command.py",
    "v3/composition/bounded_live_control.py",
    "v3/composition/native_control.py",
    "v3/contracts/base.py",
    "v3/contracts/__init__.py",
    "v3/contracts/messages.py",
    "v3/engine.py",
    "v3/layers/l4_world_model.py",
    "v3/layers/l5_command_mission.py",
    "v3/layers/l6_navigation.py",
    "v3/layers/l7_motion_selection.py",
    "v3/layers/l8_motion_realization.py",
    "v3/layers/l12_safety_final.py",
    "v3/layers/l9_operational_constraints.py",
    "v3/layers/l10_chassis_control.py",
    "v3/layers/l11_actuator_control.py",
    "v3/replay.py",
    "v3/test_hub.py",
    "v3_bounded_config.py",
    "v3_process_runtime.py",
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
    __slots__ = ("commands",)

    def __init__(self) -> None:
        self.commands: list[object] = []

    def write(self, command: object) -> None:
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
    """Replay a V3 capture through production code without live hardware imports."""

    path = _regular_file(capture_path, "capture")
    selected = selection or ReplaySelection()
    try:
        general_payload = load_capture(path)
        ticks = validate_general_capture(general_payload)
    except V3CaptureError as exc:
        raise V3ReplayError(str(exc)) from exc
    inputs = tuple(_reconstruct_general_inputs(tick) for tick in ticks)
    indices = _selection_indices(ticks, selected)
    execution_inputs = inputs[: indices[-1] + 1]
    config = _embedded_control_config(general_payload, execution_inputs)
    first_results, first_writes = _run_native_replay(execution_inputs, config)
    second_results, second_writes = _run_native_replay(execution_inputs, config)
    repeated = first_results == second_results and first_writes == second_writes
    selected_ticks = tuple(ticks[index] for index in indices)
    selected_results = tuple(first_results[index] for index in indices)
    selected_writes = tuple(first_writes[index] for index in indices)
    divergence, layer_rows = _general_diagnostics(
        selected_ticks,
        selected_results,
        selected_writes,
        selected.layers,
    )
    status = (
        V3_REPLAY_STATUS_MATCH
        if divergence is None and repeated
        else V3_REPLAY_STATUS_MISMATCH
    )
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
        "execution": _general_execution_summary(selected_results, str(general_payload["status"])),
        "reconstruction_contract": {
            "inputs": "CAPTURED_TYPED_TICK_INPUTS",
            "configuration": "CAPTURED_ONCE_BY_CONTENT",
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
        },
        "source_first": source_first,
        "first_divergence": divergence,
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


def _run_native_replay(
    inputs: tuple[TickInputs, ...],
    config: NativeControlCompositionConfig,
) -> tuple[tuple[TickResult, ...], tuple[object, ...]]:
    writer = _RecordingWriter()
    composition = NativeControlComposition(writer, config)
    sink = MemoryOutputSink()
    ExecutionBoundary(composition).run(IterableInputSource(inputs), sink)
    results = tuple(record.result for record in sink.records)
    return results, tuple(writer.commands)


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


def _reconstruct_general_inputs(tick: Mapping[str, object]) -> TickInputs:
    encoded = _mapping(tick.get("inputs"), "tick.inputs")
    _require_type(encoded, "TickInputs", "tick.inputs")
    context = _context(encoded.get("context"), "tick.inputs.context")
    if context.tick_id != _integer(tick.get("tick_id"), "tick.tick_id"):
        raise V3ReplayError("captured TickInputs tick_id mismatch")
    if context.monotonic_ns != _integer(tick.get("monotonic_ns"), "tick.monotonic_ns"):
        raise V3ReplayError("captured TickInputs monotonic_ns mismatch")
    raw_row = _mapping(encoded.get("raw_devices"), "tick.inputs.raw_devices")
    _require_type(raw_row, "RawDeviceBatch", "tick.inputs.raw_devices")
    raw_context = _context(raw_row.get("context"), "RawDeviceBatch.context")
    samples = tuple(
        DeviceSample(
            device_id=str(row.get("device_id", "")),
            kind=str(row.get("kind", "")),
            sequence=_integer(row.get("sequence"), "DeviceSample.sequence"),
            captured_monotonic_ns=_integer(
                row.get("captured_monotonic_ns"),
                "DeviceSample.captured_monotonic_ns",
            ),
            values=_data_fields(row.get("values"), "DeviceSample.values"),
        )
        for row in _mapping_sequence(raw_row.get("samples"), "RawDeviceBatch.samples")
    )
    health = tuple(
        DeviceHealth(
            device_id=str(row.get("device_id", "")),
            state=DeviceHealthState(str(row.get("state", ""))),
            reason=None if row.get("reason") is None else str(row.get("reason")),
        )
        for row in _mapping_sequence(
            raw_row.get("device_health"),
            "RawDeviceBatch.device_health",
        )
    )
    raw = RawDeviceBatch(raw_context, samples, health)
    command_row = _mapping(encoded.get("command"), "tick.inputs.command")
    _require_type(command_row, "CommandRequest", "tick.inputs.command")
    command = CommandRequest(
        context=_context(command_row.get("context"), "CommandRequest.context"),
        command_id=str(command_row.get("command_id", "")),
        mode=CommandMode(str(command_row.get("mode", ""))),
        goal=_data_fields(command_row.get("goal"), "CommandRequest.goal"),
        expiry_tick=_integer(command_row.get("expiry_tick"), "CommandRequest.expiry_tick"),
    )
    return TickInputs(
        context,
        raw,
        command,
        LifecycleState(str(encoded.get("lifecycle", ""))),
    )


def _embedded_control_config(
    payload: Mapping[str, object],
    inputs: Sequence[TickInputs],
) -> NativeControlCompositionConfig:
    configuration = _mapping(payload.get("configuration"), "capture.configuration")
    physics = _mapping(configuration.get("physics"), "capture.configuration.physics")
    speed_map = _mapping(
        configuration.get("speed_map"),
        "capture.configuration.speed_map",
    )
    hardware = _mapping(
        configuration.get("hardware"),
        "capture.configuration.hardware",
    )
    control_value = configuration.get("control")
    control = (
        _mapping(control_value, "capture.configuration.control")
        if control_value is not None
        else None
    )
    return _control_config_from_mappings(
        physics,
        speed_map,
        hardware,
        inputs,
        control,
    )


def _general_diagnostics(
    ticks: Sequence[Mapping[str, object]],
    results: Sequence[TickResult],
    writes: Sequence[object],
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
    if len(ticks) != len(results):
        first = {
            "tick_id": None,
            "layer": "TickEngine",
            "reason": "TICK_COUNT_MISMATCH",
            "expected": len(ticks),
            "actual": len(results),
        }
        return first, layer_rows
    for index, (tick, result) in enumerate(zip(ticks, results)):
        expected = _mapping(tick.get("expected"), "tick.expected")
        expected_layers = _mapping(expected.get("layers"), "tick.expected.layers")
        actual_layers = {
            record.layer: _capture_value(record.output)
            for record in result.trace.layers
        }
        mismatched_layers: set[str] = set()
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
            mismatched_layers.add(layer)
            row["mismatch_count"] = int(row["mismatch_count"]) + 1
            layer_rows[layer] = row
            if first is None:
                first = {
                    "tick_id": tick["tick_id"],
                    "layer": layer,
                    "reason": (
                        "DIRECT_VALUE_MISMATCH"
                        if expected_present and actual_present
                        else "LAYER_PRESENCE_MISMATCH"
                    ),
                    "expected": (
                        expected_layers[layer] if expected_present else "NOT_EXECUTED"
                    ),
                    "actual": (
                        actual_layers[layer] if actual_present else "NOT_EXECUTED"
                    ),
                }
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
            first = {
                "tick_id": tick["tick_id"],
                "layer": "TickEngine",
                "reason": "FAULT_LAYER_MISMATCH",
                "expected": expected_fault,
                "actual": actual_fault,
            }
        if "L12" in layers:
            expected_final = expected.get("final_actuation")
            actual_final = _capture_value(result.final_actuation)
            final_mismatch = expected_final != actual_final
            write_mismatch = index >= len(writes) or writes[index] != result.final_actuation
            if final_mismatch or write_mismatch:
                if "L12" not in mismatched_layers:
                    row = dict(_mapping(layer_rows["L12"], "diagnostics.L12"))
                    row["mismatch_count"] = int(row["mismatch_count"]) + 1
                    layer_rows["L12"] = row
                if first is None and final_mismatch:
                    first = {
                        "tick_id": tick["tick_id"],
                        "layer": "L12",
                        "reason": "FINAL_ACTUATION_MISMATCH",
                        "expected": expected_final,
                        "actual": actual_final,
                    }
                if first is None and write_mismatch:
                    first = {
                        "tick_id": tick["tick_id"],
                        "layer": "L12",
                        "reason": "OFFLINE_WRITE_VALUE_MISMATCH",
                        "expected": actual_final,
                        "actual": None if index >= len(writes) else _capture_value(writes[index]),
                    }
    return first, layer_rows


def _general_execution_summary(
    results: Sequence[TickResult],
    capture_status: str,
) -> dict[str, object]:
    decision_counts = {item.value: 0 for item in SafetyDecision}
    for result in results:
        decision_counts[result.final_actuation.safety_decision.value] += 1
    terminal = results[-1]
    return {
        "capture_status": capture_status,
        "capture_passed": capture_status == "PASS",
        "decision_counts": decision_counts,
        "terminal_tick_id": terminal.trace.context.tick_id,
        "terminal_safety_decision": terminal.final_actuation.safety_decision.value,
        "terminal_reason": terminal.final_actuation.reason,
        "terminal_fault_layer": terminal.trace.fault_layer,
    }


def _control_config_from_mappings(
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
    lidar_safety = _load_lidar_safety_config(hardware, inputs)
    navigation = (
        v3_navigation_config_from_mapping(control)
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


def _load_lidar_safety_config(
    hardware: Mapping[str, object],
    inputs: Sequence[TickInputs],
) -> LidarSafetyConfig:
    """Reconstruct the native gate from the captured native LiDAR stream."""

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
        raise V3ReplayError(
            "hardware.lidar.biztonsagi_zona_m must be positive"
        )
    return LidarSafetyConfig(
        device_id=next(iter(device_ids)),
        minimum_clearance_m=minimum_clearance_m,
        maximum_sample_age_ns=250_000_000,
    )


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
    for relative in _SOURCE_FIRST_PATHS:
        path = _regular_file(root / relative, relative)
        actual_hash = _sha256_file(path)
        expected_row = expected_files.get(relative)
        expected_hash = (
            str(expected_row.get("sha256"))
            if isinstance(expected_row, Mapping) and expected_row.get("sha256") is not None
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


def _context(value: object, name: str) -> TickContext:
    row = _mapping(value, name)
    _require_type(row, "TickContext", name)
    return TickContext(
        _integer(row.get("tick_id"), f"{name}.tick_id"),
        _integer(row.get("monotonic_ns"), f"{name}.monotonic_ns"),
    )


def _data_fields(value: object, name: str) -> tuple[DataField, ...]:
    return tuple(
        DataField(str(row.get("key", "")), row.get("value"))
        for row in _mapping_sequence(value, name)
    )


def _require_type(value: Mapping[str, object], expected: str, name: str) -> None:
    if value.get("__type__") != expected:
        raise V3ReplayError(f"{name} must contain {expected}")


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise V3ReplayError(f"{name} must be an object")
    return value


def _mapping_sequence(
    value: object,
    name: str,
) -> tuple[Mapping[str, object], ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V3ReplayError(f"{name} must be an array")
    return tuple(_mapping(item, name) for item in value)


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
