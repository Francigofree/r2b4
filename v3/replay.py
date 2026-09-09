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
from dataclasses import dataclass, fields
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
from .adapters.bno055_device import NativeBno055DeviceConfig
from .adapters.bno055_imu import Bno055ImuBackendConfig
from .adapters.counter_encoder import CounterEncoderBackendConfig
from .adapters.gpio_counter import GpioCounterChannelConfig, GpioCounterPairConfig
from .adapters.gpio_motor import GpioMotorFrameSinkConfig
from .adapters.latest_lidar import LatestLidarBackendConfig
from .adapters.live_encoder import NativeEncoderConfig
from .adapters.live_imu import NativeImuConfig
from .adapters.live_lidar import NativeLidarConfig
from .adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode
from .composition.native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
    NativeControlStateCheckpoint,
    v3_navigation_config_from_mapping,
)
from .composition.native_sensor_inputs import (
    NativeSensorHardwareConfig,
    NativeSensorInputConfig,
)
from .composition.resident_live_control import ResidentLiveControlConfig
from .composition.resident_physical_control import ResidentPhysicalControlConfig
from .contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    MissionConstraints,
    ObstacleTrack,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
)
from .engine import LayerValue, TickEngine, TickInputs, TickResult, TickTrace
from .execution import ExecutionBoundary, IterableInputSource, MemoryOutputSink
from .layers.l2_admission import AdmissionConfig, AdmissionStateCheckpoint
from .layers.l4_world_model import WorldModelConfig, WorldModelStateCheckpoint
from .layers.l5_command_mission import MissionConfig, MissionStateCheckpoint
from .layers.l6_navigation import NavigationConfig, NavigationStateCheckpoint
from .layers.l8_motion_realization import MotionRealizationConfig
from .layers.l9_operational_constraints import (
    OperationalConstraintsConfig,
    OperationalConstraintsStateCheckpoint,
)
from .layers.l10_chassis_control import ChassisControlConfig
from .layers.l11_actuator_control import (
    SpeedMapPoint,
    WheelPiConfig,
    WheelActuatorStateCheckpoint,
    WheelSpeedCurve,
    WheelSpeedMap,
)
from .layers.l3_state_estimation import (
    NativeEstimatorStateCheckpoint,
    NativeStateEstimatorConfig,
)
from .layers.l12_safety_final import FinalSafetyStateCheckpoint, LidarSafetyConfig


V3_REPLAY_RESULT_SCHEMA = "R2B4_REPLAYER_V3_RESULT_V3"
V3_REPLAY_STATUS_MATCH = "MATCH"
V3_REPLAY_STATUS_MISMATCH = "MISMATCH"
_LAYER_ORDER = LAYER_ORDER
_INPUT_REFERENCE_KEY = "__capture_input_reference__"
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
class _ResolvedReplayRuntime:
    composition: ResidentPhysicalControlConfig
    sensor_inputs: NativeSensorHardwareConfig
    tick_period_ns: int

    def __post_init__(self) -> None:
        if self.tick_period_ns <= 0:
            raise ValueError("tick_period_ns must be positive")
        if self.tick_period_ns > self.composition.live_control.max_preflight_age_ns:
            raise ValueError("tick_period_ns exceeds resident preflight policy")


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
    entries = tuple(_reconstruct_general_entry(tick) for tick in ticks)
    indices = _selection_indices(ticks, selected)
    execution_entries = entries[: indices[-1] + 1]
    closed_inputs = tuple(
        entry.inputs for entry in execution_entries if entry.inputs is not None
    )
    config = _embedded_control_config(general_payload, closed_inputs)
    checkpoint = _decode_state_checkpoint(general_payload)
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
            outcomes.append(
                _ReplayOutcome(None, True, exc.attempted_actuation)
            )
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


def _reconstruct_general_entry(tick: Mapping[str, object]) -> _ReplayEntry:
    record_type = tick.get("record_type", "closed_input_tick")
    expected = _mapping(tick.get("expected"), "tick.expected")
    writer_failure = isinstance(expected.get("writer_failure"), Mapping)
    if record_type == "closed_input_tick":
        inputs = _reconstruct_general_inputs(tick)
        return _ReplayEntry(
            context=inputs.context,
            inputs=inputs,
            lifecycle=inputs.lifecycle,
            writer_failure=writer_failure,
        )
    if record_type != "edge_fault_tick":
        raise V3ReplayError("capture tick has an unsupported record_type")
    edge = _mapping(tick.get("edge_fault"), "tick.edge_fault")
    context = _context(edge.get("context"), "tick.edge_fault.context")
    raw_health = edge.get("critical_health", ())
    health = tuple(
        DeviceHealth(
            device_id=str(row.get("device_id", "")),
            state=DeviceHealthState(str(row.get("state", ""))),
            reason=None if row.get("reason") is None else str(row.get("reason")),
        )
        for row in _mapping_sequence(raw_health, "tick.edge_fault.critical_health")
    )
    return _ReplayEntry(
        context=context,
        inputs=None,
        lifecycle=LifecycleState(str(edge.get("lifecycle", ""))),
        reason=str(edge.get("reason", "")),
        fault_layer=str(edge.get("fault_layer", "")),
        critical_health=health,
        writer_failure=writer_failure,
    )


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
    resolved_runtime = configuration.get("resolved_runtime")
    if resolved_runtime is not None:
        return _decode_resolved_runtime(
            _mapping(resolved_runtime, "capture.configuration.resolved_runtime")
        ).composition.live_control.control
    resolved = configuration.get("resolved_control")
    if resolved is not None:
        return _decode_native_control_config(
            _mapping(resolved, "capture.configuration.resolved_control")
        )
    legacy_value = configuration.get("legacy_documents", configuration)
    legacy = _mapping(legacy_value, "capture.configuration.legacy_documents")
    physics = _mapping(legacy.get("physics"), "capture.configuration.physics")
    speed_map = _mapping(
        legacy.get("speed_map"),
        "capture.configuration.speed_map",
    )
    hardware = _mapping(
        legacy.get("hardware"),
        "capture.configuration.hardware",
    )
    control_value = legacy.get("control")
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


def _decode_resolved_runtime(
    row: Mapping[str, object],
) -> _ResolvedReplayRuntime:
    _require_type(row, "ResidentPhysicalRuntimeConfig", "resolved_runtime")
    composition_row = _mapping(
        row.get("composition"),
        "resolved_runtime.composition",
    )
    _require_type(
        composition_row,
        "ResidentPhysicalControlConfig",
        "resolved_runtime.composition",
    )
    live_row = _mapping(
        composition_row.get("live_control"),
        "resolved_runtime.composition.live_control",
    )
    _require_type(
        live_row,
        "ResidentLiveControlConfig",
        "resolved_runtime.composition.live_control",
    )
    control = _decode_native_control_config(
        _mapping(
            live_row.get("control"),
            "resolved_runtime.composition.live_control.control",
        )
    )
    try:
        live = ResidentLiveControlConfig(
            control=control,
            max_preflight_age_ns=_integer(
                live_row.get("max_preflight_age_ns"),
                "resolved_runtime.max_preflight_age_ns",
            ),
            required_lidar_preflight_revisions=_integer(
                live_row.get("required_lidar_preflight_revisions"),
                "resolved_runtime.required_lidar_preflight_revisions",
            ),
        )
        motor = _decode_motor_edge(
            _mapping(
                composition_row.get("motor_output"),
                "resolved_runtime.composition.motor_output",
            )
        )
        composition = ResidentPhysicalControlConfig(live, motor)
        sensors = _decode_sensor_hardware(
            _mapping(row.get("sensor_inputs"), "resolved_runtime.sensor_inputs")
        )
        tick_period_ns = _integer(
            row.get("tick_period_ns"),
            "resolved_runtime.tick_period_ns",
        )
        resolved = _ResolvedReplayRuntime(composition, sensors, tick_period_ns)
    except (TypeError, ValueError) as exc:
        raise V3ReplayError(f"resolved_runtime is invalid: {exc}") from exc
    return resolved


def _decode_motor_edge(row: Mapping[str, object]) -> GpioMotorFrameSinkConfig:
    _require_type(row, "GpioMotorFrameSinkConfig", "resolved_runtime.motor_output")

    def channel(value: object, name: str) -> MotorChannelPhysicalConfig:
        item = _mapping(value, name)
        _require_type(item, "MotorChannelPhysicalConfig", name)
        try:
            return MotorChannelPhysicalConfig(
                in1=_integer(item.get("in1"), f"{name}.in1"),
                in2=_integer(item.get("in2"), f"{name}.in2"),
                invert=_required_bool(item.get("invert"), f"{name}.invert"),
                pwm_decay_mode=PwmDecayMode(str(item.get("pwm_decay_mode", ""))),
            )
        except (TypeError, ValueError) as exc:
            raise V3ReplayError(f"{name} is invalid: {exc}") from exc

    try:
        return GpioMotorFrameSinkConfig(
            left=channel(row.get("left"), "resolved_runtime.motor_output.left"),
            right=channel(row.get("right"), "resolved_runtime.motor_output.right"),
            gpio_chip=_integer(row.get("gpio_chip"), "resolved_runtime.motor_output.gpio_chip"),
            pwm_frequency_hz=_integer(
                row.get("pwm_frequency_hz"),
                "resolved_runtime.motor_output.pwm_frequency_hz",
            ),
        )
    except (TypeError, ValueError) as exc:
        raise V3ReplayError(f"resolved_runtime.motor_output is invalid: {exc}") from exc


def _decode_sensor_hardware(row: Mapping[str, object]) -> NativeSensorHardwareConfig:
    _require_type(row, "NativeSensorHardwareConfig", "resolved_runtime.sensor_inputs")
    imu_device = _decode_flat_config(
        NativeBno055DeviceConfig,
        row.get("imu_device"),
        "resolved_runtime.sensor_inputs.imu_device",
        tuple_fields={"axis_order", "axis_sign"},
    )
    inputs_row = _mapping(
        row.get("inputs"),
        "resolved_runtime.sensor_inputs.inputs",
    )
    _require_type(inputs_row, "NativeSensorInputConfig", "resolved_runtime.sensor_inputs.inputs")
    counter_row = _mapping(
        inputs_row.get("encoder_counter"),
        "resolved_runtime.sensor_inputs.inputs.encoder_counter",
    )
    _require_type(
        counter_row,
        "GpioCounterPairConfig",
        "resolved_runtime.sensor_inputs.inputs.encoder_counter",
    )
    try:
        counter = GpioCounterPairConfig(
            left=_decode_flat_config(
                GpioCounterChannelConfig,
                counter_row.get("left"),
                "resolved_runtime.sensor_inputs.inputs.encoder_counter.left",
            ),
            right=_decode_flat_config(
                GpioCounterChannelConfig,
                counter_row.get("right"),
                "resolved_runtime.sensor_inputs.inputs.encoder_counter.right",
            ),
            gpio_chip=_integer(
                counter_row.get("gpio_chip"),
                "resolved_runtime.sensor_inputs.inputs.encoder_counter.gpio_chip",
            ),
            edge_history_capacity=_integer(
                counter_row.get("edge_history_capacity"),
                "resolved_runtime.sensor_inputs.inputs.encoder_counter.edge_history_capacity",
            ),
        )
        inputs = NativeSensorInputConfig(
            encoder_counter=counter,
            encoder_backend=_decode_flat_config(
                CounterEncoderBackendConfig,
                inputs_row.get("encoder_backend"),
                "resolved_runtime.sensor_inputs.inputs.encoder_backend",
            ),
            encoder_source=_decode_flat_config(
                NativeEncoderConfig,
                inputs_row.get("encoder_source"),
                "resolved_runtime.sensor_inputs.inputs.encoder_source",
            ),
            imu_backend=_decode_flat_config(
                Bno055ImuBackendConfig,
                inputs_row.get("imu_backend"),
                "resolved_runtime.sensor_inputs.inputs.imu_backend",
            ),
            imu_source=_decode_flat_config(
                NativeImuConfig,
                inputs_row.get("imu_source"),
                "resolved_runtime.sensor_inputs.inputs.imu_source",
            ),
            lidar_backend=_decode_flat_config(
                LatestLidarBackendConfig,
                inputs_row.get("lidar_backend"),
                "resolved_runtime.sensor_inputs.inputs.lidar_backend",
            ),
            lidar_source=_decode_flat_config(
                NativeLidarConfig,
                inputs_row.get("lidar_source"),
                "resolved_runtime.sensor_inputs.inputs.lidar_source",
            ),
        )
        return NativeSensorHardwareConfig(
            imu_device=imu_device,
            inputs=inputs,
            lidar_danger_zone_m=_number(
                row.get("lidar_danger_zone_m"),
                "resolved_runtime.sensor_inputs.lidar_danger_zone_m",
            ),
        )
    except (TypeError, ValueError) as exc:
        raise V3ReplayError(f"resolved_runtime.sensor_inputs is invalid: {exc}") from exc


def _decode_native_control_config(
    row: Mapping[str, object],
) -> NativeControlCompositionConfig:
    _require_type(row, "NativeControlCompositionConfig", "resolved_control")
    expected = {field.name for field in fields(NativeControlCompositionConfig)}
    missing = sorted(expected - set(row))
    if missing:
        raise V3ReplayError(
            "resolved_control lacks fields: " + ", ".join(missing)
        )
    return NativeControlCompositionConfig(
        speed_map=_decode_speed_map(
            _mapping(row.get("speed_map"), "resolved_control.speed_map")
        ),
        admission=_decode_flat_config(
            AdmissionConfig,
            row.get("admission"),
            "resolved_control.admission",
        ),
        estimation=_decode_flat_config(
            NativeStateEstimatorConfig,
            row.get("estimation"),
            "resolved_control.estimation",
            tuple_fields={
                "process_noise",
                "initial_covariance",
                "lidar_measurement_variance",
            },
        ),
        world_model=_decode_flat_config(
            WorldModelConfig,
            row.get("world_model"),
            "resolved_control.world_model",
        ),
        mission=_decode_mission_config(
            _mapping(row.get("mission"), "resolved_control.mission")
        ),
        navigation=_decode_flat_config(
            NavigationConfig,
            row.get("navigation"),
            "resolved_control.navigation",
        ),
        motion_realization=_decode_flat_config(
            MotionRealizationConfig,
            row.get("motion_realization"),
            "resolved_control.motion_realization",
        ),
        operational_constraints=_decode_flat_config(
            OperationalConstraintsConfig,
            row.get("operational_constraints"),
            "resolved_control.operational_constraints",
        ),
        chassis_control=_decode_flat_config(
            ChassisControlConfig,
            row.get("chassis_control"),
            "resolved_control.chassis_control",
        ),
        wheel_pi=_decode_flat_config(
            WheelPiConfig,
            row.get("wheel_pi"),
            "resolved_control.wheel_pi",
        ),
        lidar_safety=(
            None
            if row.get("lidar_safety") is None
            else _decode_flat_config(
                LidarSafetyConfig,
                row.get("lidar_safety"),
                "resolved_control.lidar_safety",
            )
        ),
    )


def _decode_state_checkpoint(
    payload: Mapping[str, object],
) -> NativeControlStateCheckpoint | None:
    value = payload.get("initial_state_checkpoint")
    if value is None:
        return None
    root = _mapping(value, "initial_state_checkpoint")
    _require_type(root, "NativeControlStateCheckpoint", "initial_state_checkpoint")
    admission = _typed_mapping(
        root.get("admission"),
        "AdmissionStateCheckpoint",
        "initial_state_checkpoint.admission",
    )
    estimator = _typed_mapping(
        root.get("estimation"),
        "NativeEstimatorStateCheckpoint",
        "initial_state_checkpoint.estimation",
    )
    world = _typed_mapping(
        root.get("world_model"),
        "WorldModelStateCheckpoint",
        "initial_state_checkpoint.world_model",
    )
    mission = _typed_mapping(
        root.get("mission"),
        "MissionStateCheckpoint",
        "initial_state_checkpoint.mission",
    )
    navigation = _typed_mapping(
        root.get("navigation"),
        "NavigationStateCheckpoint",
        "initial_state_checkpoint.navigation",
    )
    constraints = _typed_mapping(
        root.get("operational_constraints"),
        "OperationalConstraintsStateCheckpoint",
        "initial_state_checkpoint.operational_constraints",
    )
    actuator = _typed_mapping(
        root.get("actuator_control"),
        "WheelActuatorStateCheckpoint",
        "initial_state_checkpoint.actuator_control",
    )
    final_safety = _typed_mapping(
        root.get("final_safety"),
        "FinalSafetyStateCheckpoint",
        "initial_state_checkpoint.final_safety",
    )
    try:
        return NativeControlStateCheckpoint(
            engine_last_context=_context(
                root.get("engine_last_context"),
                "initial_state_checkpoint.engine_last_context",
            ),
            admission=AdmissionStateCheckpoint(
                tuple(
                    _decode_admission_sequence(row)
                    for row in _sequence(
                        admission.get("last_sequences"),
                        "admission.last_sequences",
                    )
                )
            ),
            estimation=NativeEstimatorStateCheckpoint(
                tuple(
                    _number(item, "estimation.state[]")
                    for item in _sequence(estimator.get("state"), "estimation.state")
                ),
                tuple(
                    tuple(
                        _number(item, "estimation.covariance[][]")
                        for item in _sequence(row, "estimation.covariance[]")
                    )
                    for row in _sequence(
                        estimator.get("covariance"),
                        "estimation.covariance",
                    )
                ),
                _optional_context(estimator.get("last_context"), "estimation.last_context"),
                _number(estimator.get("last_omega"), "estimation.last_omega"),
            ),
            world_model=WorldModelStateCheckpoint(
                _optional_integer(
                    world.get("last_lidar_measurement_ns"),
                    "world_model.last_lidar_measurement_ns",
                ),
                _optional_integer(
                    world.get("last_lidar_sequence"),
                    "world_model.last_lidar_sequence",
                ),
                _integer(world.get("map_revision"), "world_model.map_revision"),
                tuple(
                    _decode_world_track_state(row)
                    for row in _sequence(world.get("tracks"), "world_model.tracks")
                ),
                _optional_integer(
                    world.get("last_local_measurement_ns"),
                    "world_model.last_local_measurement_ns",
                ),
                _optional_integer(
                    world.get("last_local_sequence"),
                    "world_model.last_local_sequence",
                ),
                (
                    None
                    if world.get("last_local_values") is None
                    else _data_fields(
                        world.get("last_local_values"),
                        "world_model.last_local_values",
                    )
                ),
                _integer(
                    world.get("costmap_revision"),
                    "world_model.costmap_revision",
                ),
                tuple(
                    _decode_grid_state(row, "world_model.cells[]")
                    for row in _sequence(world.get("cells"), "world_model.cells")
                ),
            ),
            mission=MissionStateCheckpoint(
                (
                    None
                    if mission.get("command_id") is None
                    else str(mission.get("command_id"))
                ),
                (
                    None
                    if mission.get("mode") is None
                    else CommandMode(str(mission.get("mode")))
                ),
                _data_fields(mission.get("goal"), "mission.goal"),
            ),
            navigation=NavigationStateCheckpoint(
                (
                    None
                    if navigation.get("mission_id") is None
                    else str(navigation.get("mission_id"))
                ),
                _number(
                    navigation.get("initial_distance_m"),
                    "navigation.initial_distance_m",
                ),
                _number(navigation.get("progress"), "navigation.progress"),
                _required_bool(navigation.get("completed"), "navigation.completed"),
                tuple(
                    _decode_grid_state(row, "navigation.coverage[]")
                    for row in _sequence(
                        navigation.get("coverage"),
                        "navigation.coverage",
                    )
                ),
                _optional_waypoint(
                    navigation.get("local_goal"),
                    "navigation.local_goal",
                ),
                _integer(
                    navigation.get("goal_selected_ns"),
                    "navigation.goal_selected_ns",
                ),
                _optional_integer(
                    navigation.get("last_replan_ns"),
                    "navigation.last_replan_ns",
                ),
                tuple(
                    _trajectory_evaluation(row, "navigation.trajectory_candidates[]")
                    for row in _mapping_sequence(
                        navigation.get("trajectory_candidates"),
                        "navigation.trajectory_candidates",
                    )
                ),
            ),
            operational_constraints=OperationalConstraintsStateCheckpoint(
                _optional_context(
                    constraints.get("last_context"),
                    "operational_constraints.last_context",
                ),
                _number(
                    constraints.get("last_v_mps"),
                    "operational_constraints.last_v_mps",
                ),
                _number(
                    constraints.get("last_omega_rad_s"),
                    "operational_constraints.last_omega_rad_s",
                ),
            ),
            actuator_control=WheelActuatorStateCheckpoint(
                _optional_context(
                    actuator.get("last_context"),
                    "actuator_control.last_context",
                ),
                _number(
                    actuator.get("left_integral"),
                    "actuator_control.left_integral",
                ),
                _number(
                    actuator.get("right_integral"),
                    "actuator_control.right_integral",
                ),
            ),
            final_safety=FinalSafetyStateCheckpoint(
                _required_bool(
                    final_safety.get("fault_latched"),
                    "final_safety.fault_latched",
                )
            ),
        )
    except (TypeError, ValueError) as exc:
        if isinstance(exc, V3ReplayError):
            raise
        raise V3ReplayError(f"initial_state_checkpoint is invalid: {exc}") from exc


def _decode_flat_config(
    cls: type,
    value: object,
    name: str,
    *,
    tuple_fields: set[str] | None = None,
):
    row = _mapping(value, name)
    _require_type(row, cls.__name__, name)
    names = tuple(field.name for field in fields(cls))
    missing = [field_name for field_name in names if field_name not in row]
    if missing:
        raise V3ReplayError(f"{name} lacks fields: {', '.join(missing)}")
    converted = {
        field_name: (
            tuple(_sequence(row[field_name], f"{name}.{field_name}"))
            if tuple_fields and field_name in tuple_fields
            else row[field_name]
        )
        for field_name in names
    }
    try:
        return cls(**converted)
    except (TypeError, ValueError) as exc:
        raise V3ReplayError(f"{name} is invalid: {exc}") from exc


def _decode_speed_map(row: Mapping[str, object]) -> WheelSpeedMap:
    _require_type(row, "WheelSpeedMap", "resolved_control.speed_map")
    curves = tuple(
        WheelSpeedCurve(
            name=str(curve.get("name", "")),
            points=tuple(
                SpeedMapPoint(
                    speed_mps=_number(point.get("speed_mps"), "speed_map.speed_mps"),
                    normalized_output=_number(
                        point.get("normalized_output"),
                        "speed_map.normalized_output",
                    ),
                )
                for point in _mapping_sequence(curve.get("points"), "speed_map.points")
                if _required_encoded_type(point, "SpeedMapPoint", "speed_map.point")
            ),
            maintenance_output=_number(
                curve.get("maintenance_output"),
                "speed_map.maintenance_output",
            ),
            startup_output=_number(
                curve.get("startup_output"),
                "speed_map.startup_output",
            ),
        )
        for curve in _mapping_sequence(row.get("curves"), "resolved_control.speed_map.curves")
        if _required_encoded_type(curve, "WheelSpeedCurve", "speed_map.curve")
    )
    try:
        return WheelSpeedMap(
            schema=str(row.get("schema", "")),
            map_state=str(row.get("map_state", "")),
            curves=curves,
        )
    except (TypeError, ValueError) as exc:
        raise V3ReplayError(f"resolved_control.speed_map is invalid: {exc}") from exc


def _decode_mission_config(row: Mapping[str, object]) -> MissionConfig:
    _require_type(row, "MissionConfig", "resolved_control.mission")
    constraints = _decode_flat_config(
        MissionConstraints,
        row.get("default_constraints"),
        "resolved_control.mission.default_constraints",
    )
    return MissionConfig(default_constraints=constraints)


def _required_encoded_type(
    row: Mapping[str, object],
    expected: str,
    name: str,
) -> bool:
    _require_type(row, expected, name)
    return True


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
                expected_value = (
                    expected_layers[layer] if expected_present else "NOT_EXECUTED"
                )
                actual_value = actual_layers[layer] if actual_present else "NOT_EXECUTED"
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
                "context": inputs.get("context"),
                "samples": raw.get("samples"),
                "io_health": raw.get("device_health"),
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
                    "values": sample.get("values"),
                }
            )
        expanded[layer] = {
            "__type__": "AdmittedFrame",
            "context": inputs.get("context"),
            "accepted": accepted,
            "rejected": value.get("rejected"),
            "degraded_sources": value.get("degraded_sources"),
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
    if isinstance(missing, Sequence) and not isinstance(missing, (str, bytes)) and missing:
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


def _typed_mapping(value: object, expected: str, name: str) -> Mapping[str, object]:
    row = _mapping(value, name)
    _require_type(row, expected, name)
    return row


def _optional_context(value: object, name: str) -> TickContext | None:
    return None if value is None else _context(value, name)


def _optional_integer(value: object, name: str) -> int | None:
    return None if value is None else _integer(value, name)


def _signed_integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise V3ReplayError(f"{name} must be an integer")
    return value


def _decode_grid_state(value: object, name: str) -> tuple[int, int, int, int]:
    row = _sequence(value, name)
    if len(row) != 4:
        raise V3ReplayError(f"{name} must contain four integers")
    return (
        _signed_integer(row[0], f"{name}[0]"),
        _signed_integer(row[1], f"{name}[1]"),
        _integer(row[2], f"{name}[2]"),
        _integer(row[3], f"{name}[3]"),
    )


def _decode_admission_sequence(value: object) -> tuple[str, str, int]:
    row = _sequence(value, "admission.last_sequences[]")
    if len(row) != 3:
        raise V3ReplayError("admission.last_sequences[] must contain three values")
    return (
        str(row[0]),
        str(row[1]),
        _integer(row[2], "admission.last_sequences[].sequence"),
    )


def _decode_world_track_state(value: object) -> tuple[ObstacleTrack, int]:
    row = _sequence(value, "world_model.tracks[]")
    if len(row) != 2:
        raise V3ReplayError("world_model.tracks[] must contain track and timestamp")
    track = _typed_mapping(row[0], "ObstacleTrack", "world_model.tracks[].track")
    return (
        ObstacleTrack(
            track_id=str(track.get("track_id", "")),
            x_m=_number(track.get("x_m"), "world_model.track.x_m"),
            y_m=_number(track.get("y_m"), "world_model.track.y_m"),
            radius_m=_number(track.get("radius_m"), "world_model.track.radius_m"),
            vx_mps=_number(track.get("vx_mps"), "world_model.track.vx_mps"),
            vy_mps=_number(track.get("vy_mps"), "world_model.track.vy_mps"),
            confidence=_number(
                track.get("confidence"),
                "world_model.track.confidence",
            ),
        ),
        _integer(row[1], "world_model.tracks[].captured_monotonic_ns"),
    )


def _optional_waypoint(value: object, name: str) -> Waypoint | None:
    if value is None:
        return None
    row = _typed_mapping(value, "Waypoint", name)
    return Waypoint(
        _number(row.get("x_m"), f"{name}.x_m"),
        _number(row.get("y_m"), f"{name}.y_m"),
        (
            None
            if row.get("yaw_rad") is None
            else _number(row.get("yaw_rad"), f"{name}.yaw_rad")
        ),
    )


def _trajectory_evaluation(
    value: Mapping[str, object],
    name: str,
) -> TrajectoryEvaluation:
    _require_type(value, "TrajectoryEvaluation", name)
    samples = tuple(
        TrajectoryPose(
            _number(row.get("x_m"), f"{name}.samples[].x_m"),
            _number(row.get("y_m"), f"{name}.samples[].y_m"),
            _number(row.get("yaw_rad"), f"{name}.samples[].yaw_rad"),
            _integer(
                row.get("time_offset_ns"),
                f"{name}.samples[].time_offset_ns",
            ),
        )
        for row in _mapping_sequence(value.get("samples"), f"{name}.samples")
        if _required_encoded_type(row, "TrajectoryPose", f"{name}.samples[]")
    )
    return TrajectoryEvaluation(
        candidate_id=str(value.get("candidate_id", "")),
        v_mps=_number(value.get("v_mps"), f"{name}.v_mps"),
        omega_rad_s=_number(value.get("omega_rad_s"), f"{name}.omega_rad_s"),
        horizon_ns=_integer(value.get("horizon_ns"), f"{name}.horizon_ns"),
        samples=samples,
        collision=_required_bool(value.get("collision"), f"{name}.collision"),
        min_clearance_m=_number(
            value.get("min_clearance_m"),
            f"{name}.min_clearance_m",
        ),
        progress_score=_number(
            value.get("progress_score"),
            f"{name}.progress_score",
        ),
        smoothness_score=_number(
            value.get("smoothness_score"),
            f"{name}.smoothness_score",
        ),
        novelty_score=_number(
            value.get("novelty_score"),
            f"{name}.novelty_score",
        ),
        total_score=_number(value.get("total_score"), f"{name}.total_score"),
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


def _sequence(value: object, name: str) -> Sequence[object]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise V3ReplayError(f"{name} must be an array")
    return value


def _integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise V3ReplayError(f"{name} must be a non-negative integer")
    return value


def _required_bool(value: object, name: str) -> bool:
    if type(value) is not bool:
        raise V3ReplayError(f"{name} must be bool")
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
