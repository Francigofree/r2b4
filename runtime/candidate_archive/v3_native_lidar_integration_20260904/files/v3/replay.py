"""Direct-value deterministic V3 replay and floor-load control diagnostics."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from .composition.native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
)
from .contracts import (
    AdmittedFrame,
    ActuatorRequest,
    CommandMode,
    CommandRequest,
    ConstraintCode,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
    WheelVelocitySetpoint,
)
from .engine import LayerValue, TickEngine, TickInputs, TickResult, TickTrace
from .layers.l10_chassis_control import ChassisControlConfig
from .layers.l11_actuator_control import (
    WheelActuatorController,
    WheelSpeedMap,
)
from .layers.l3_state_estimation import NativeStateEstimatorConfig
from .layers.l12_safety_final import LidarSafetyConfig


V3_FLOOR_CAPTURE_SCHEMA = "R2B4_V3_NATIVE_FLOOR_TICK_CAPTURE_V1"
V3_REPLAY_RESULT_SCHEMA = "R2B4_REPLAYER_V3_RESULT_V2"
V3_REPLAY_STATUS_MATCH = "MATCH"
V3_REPLAY_STATUS_MISMATCH = "MISMATCH"
_REPLAYABLE_CAPTURE_STATUSES = frozenset(("PASS", "FAIL", "FAULT"))
_LAYER_ORDER = tuple(f"L{index}" for index in range(1, 13))
_SOURCE_FIRST_PATHS = (
    "conf/hardver.json",
    "conf/fizika.json",
    "conf/speed_map.json",
    "v3/composition/native_control.py",
    "v3/engine.py",
    "v3/layers/l12_safety_final.py",
    "v3/layers/l9_operational_constraints.py",
    "v3/layers/l10_chassis_control.py",
    "v3/layers/l11_actuator_control.py",
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
class WheelReplayTerms:
    """Offline observations around one unchanged production wheel-loop call."""

    reference_mps: float
    measured_mps: float | None
    dt_s: float
    feedforward: float
    maintenance_floor: float
    error_mps: float
    proportional: float
    integrator_state: float
    integral: float
    raw_unclamped: float
    output_normalized: float
    saturated: bool
    maintenance_floor_applied: bool
    integrator_reset: bool
    integrator_clamped: bool


@dataclass(frozen=True, slots=True)
class WheelReplayDiagnostics:
    context: TickContext
    left: WheelReplayTerms
    right: WheelReplayTerms


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


def inspect_floor_capture(capture_path: str | Path) -> dict[str, object]:
    """Return a bounded manifest-only view of a V3 full-tick capture."""

    path = _regular_file(capture_path, "capture")
    payload = _json_object(path, "capture")
    ticks = _validate_capture(payload)
    first = ticks[0]
    last = ticks[-1]
    return {
        "schema": payload["schema"],
        "status": payload["status"],
        "execution_status": payload["status"],
        "execution_passed": payload["status"] == "PASS",
        "profile": payload.get("profile"),
        "tick_count": len(ticks),
        "first_tick_id": first["tick_id"],
        "last_tick_id": last["tick_id"],
        "first_monotonic_ns": first["monotonic_ns"],
        "last_monotonic_ns": last["monotonic_ns"],
        "capture_sha256": _sha256_file(path),
    }


def replay_floor_capture(
    capture_path: str | Path,
    *,
    physics_config_path: str | Path,
    speed_map_config_path: str | Path,
    hardware_config_path: str | Path | None = None,
    project_root: str | Path | None = None,
    capture_source_manifest_path: str | Path | None = None,
) -> dict[str, object]:
    """Replay a captured V3 floor profile through the production L1-L12 chain.

    The capture's L1 value is a lossless copy of ``RawDeviceBatch``. The floor
    profile's per-tick STOP/TELEOP request is reconstructed from the captured L5
    value and its capture-time gateway contract. No sensor, clock, GPIO or motor
    I/O is opened.
    """

    capture = _regular_file(capture_path, "capture")
    root = Path(project_root).resolve() if project_root is not None else Path.cwd()
    physics_path = _regular_file(physics_config_path, "physics config")
    speed_map_path = _regular_file(speed_map_config_path, "speed-map config")
    hardware_path = _regular_file(
        hardware_config_path or root / "conf/hardver.json",
        "hardware config",
    )
    payload = _json_object(capture, "capture")
    ticks = _validate_capture(payload)
    inputs = tuple(_reconstruct_inputs(tick) for tick in ticks)
    config = _load_control_config(
        physics_path,
        speed_map_path,
        hardware_path,
        inputs,
    )

    first_results, first_writes = _run_native_replay(inputs, config)
    second_results, second_writes = _run_native_replay(inputs, config)
    repeat_match = first_results == second_results and first_writes == second_writes
    divergence = _first_capture_divergence(ticks, first_results, first_writes)
    rows, diagnostic_divergence = _control_rows(first_results, config)
    if divergence is None:
        divergence = diagnostic_divergence

    analysis = _analyze_control_rows(rows, config)
    source_first = _source_first_evidence(root, capture_source_manifest_path)
    status = (
        V3_REPLAY_STATUS_MATCH
        if divergence is None and repeat_match
        else V3_REPLAY_STATUS_MISMATCH
    )
    result: dict[str, object] = {
        "schema": V3_REPLAY_RESULT_SCHEMA,
        "status": status,
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "capture": {
            "path": str(capture.resolve()),
            "sha256": _sha256_file(capture),
            "schema": payload["schema"],
            "profile": payload.get("profile"),
            "tick_count": len(ticks),
            "status": payload["status"],
        },
        "execution": _execution_summary(ticks, str(payload["status"])),
        "reconstruction_contract": {
            "raw_devices": "L1_REVERSIBLE_ACQUISITION_COPY",
            "command": "CAPTURE_TIME_FLOOR_GATEWAY_FROM_L5",
            "lifecycle": "ACTIVE_IFF_CAPTURED_L5_MISSION_ACTIVE_OTHERWISE_IDLE",
            "lidar_safety": _lidar_safety_reconstruction(config.lidar_safety),
            "external_io": "NONE",
        },
        "source_first": source_first,
        "determinism": {
            "production_layers": "L1-L12",
            "first_run_tick_count": len(first_results),
            "second_run_tick_count": len(second_results),
            "repeated_trace_match": repeat_match,
            "offline_write_count_first": len(first_writes),
            "offline_write_count_second": len(second_writes),
        },
        "first_divergence": divergence,
        "analysis": analysis,
        "control_rows": rows,
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
    results = tuple(composition.run_tick(item) for item in inputs)
    return results, tuple(writer.commands)


def _first_capture_divergence(
    ticks: tuple[Mapping[str, object], ...],
    results: tuple[TickResult, ...],
    writes: tuple[object, ...],
) -> dict[str, object] | None:
    if len(ticks) != len(results):
        return {
            "tick_id": None,
            "layer": "TickEngine",
            "reason": "TICK_COUNT_MISMATCH",
            "expected": len(ticks),
            "actual": len(results),
        }
    if len(writes) != len(results):
        return {
            "tick_id": None,
            "layer": "L12",
            "reason": "OFFLINE_WRITE_COUNT_MISMATCH",
            "expected": len(results),
            "actual": len(writes),
        }
    for tick, result, write in zip(ticks, results, writes):
        tick_id = int(tick["tick_id"])
        expected_layers = _mapping(tick.get("layers"), "tick.layers")
        actual_layers = {
            record.layer: _capture_value(record.output)
            for record in result.trace.layers
        }
        for layer in _LAYER_ORDER:
            if expected_layers.get(layer) != actual_layers.get(layer):
                return {
                    "tick_id": tick_id,
                    "layer": layer,
                    "reason": "DIRECT_VALUE_MISMATCH",
                    "expected": expected_layers.get(layer),
                    "actual": actual_layers.get(layer),
                }
        if tick.get("fault_layer") != result.trace.fault_layer:
            return {
                "tick_id": tick_id,
                "layer": "TickEngine",
                "reason": "FAULT_LAYER_MISMATCH",
                "expected": tick.get("fault_layer"),
                "actual": result.trace.fault_layer,
            }
        expected_final = tick.get("final_actuation")
        actual_final = _capture_value(result.final_actuation)
        if expected_final != actual_final:
            return {
                "tick_id": tick_id,
                "layer": "L12",
                "reason": "FINAL_ACTUATION_MISMATCH",
                "expected": expected_final,
                "actual": actual_final,
            }
        if write != result.final_actuation:
            return {
                "tick_id": tick_id,
                "layer": "L12",
                "reason": "OFFLINE_WRITE_VALUE_MISMATCH",
                "expected": actual_final,
                "actual": _capture_value(write),
            }
    return None


def _control_rows(
    results: tuple[TickResult, ...],
    config: NativeControlCompositionConfig,
) -> tuple[list[dict[str, object]], dict[str, object] | None]:
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    rows: list[dict[str, object]] = []
    first_divergence_value: dict[str, object] | None = None
    for result in results:
        layers = {record.layer: record.output for record in result.trace.layers}
        wheels = layers.get("L10")
        frame = layers.get("L2")
        expected_request = layers.get("L11")
        if wheels is None or frame is None or expected_request is None:
            continue
        request, diagnostics, state_match = _evaluate_l11_with_diagnostics(
            controller,
            wheels,  # type: ignore[arg-type]
            frame,  # type: ignore[arg-type]
            config,
        )
        if request != expected_request and first_divergence_value is None:
            first_divergence_value = {
                "tick_id": result.trace.context.tick_id,
                "layer": "L11_DIAGNOSTICS",
                "reason": "DIAGNOSTIC_PATH_OUTPUT_MISMATCH",
                "expected": _capture_value(expected_request),
                "actual": _capture_value(request),
            }
        if not state_match and first_divergence_value is None:
            first_divergence_value = {
                "tick_id": result.trace.context.tick_id,
                "layer": "L11_DIAGNOSTICS",
                "reason": "OBSERVED_INTEGRATOR_STATE_MISMATCH",
                "expected": "source_formula_state",
                "actual": "production_private_state",
            }
        if result.final_actuation.safety_decision is not SafetyDecision.ALLOW:
            continue
        motion = layers["L8"]
        constrained = layers["L9"]
        rows.append(
            _control_row(
                result,
                motion,
                constrained,
                wheels,
                diagnostics,
            )
        )
    return rows, first_divergence_value


def _evaluate_l11_with_diagnostics(
    controller: WheelActuatorController,
    wheels: WheelVelocitySetpoint,
    frame: AdmittedFrame,
    config: NativeControlCompositionConfig,
) -> tuple[ActuatorRequest, WheelReplayDiagnostics, bool]:
    """Observe source-owned state around exactly one production L11 call."""

    previous_context = controller._last_context
    left_before = float(controller._left_pi._integral)
    right_before = float(controller._right_pi._integral)
    if previous_context is None:
        dt_s = 0.0
    elif (
        wheels.context.tick_id != previous_context.tick_id + 1
        or wheels.context.monotonic_ns <= previous_context.monotonic_ns
    ):
        dt_s = 0.0
    else:
        dt_s = (
            wheels.context.monotonic_ns - previous_context.monotonic_ns
        ) / 1_000_000_000.0

    if abs(wheels.left_mps) <= 1e-12 and abs(wheels.right_mps) <= 1e-12:
        request = controller(wheels, frame)
        left = _zero_replay_terms(wheels.left_mps, dt_s)
        right = _zero_replay_terms(wheels.right_mps, dt_s)
        state_match = bool(
            controller._left_pi._integral == 0.0
            and controller._right_pi._integral == 0.0
        )
        return request, WheelReplayDiagnostics(wheels.context, left, right), state_match

    measured_left, measured_right = controller._wheel_feedback(frame)
    left_expected = _expected_wheel_terms(
        side="left",
        reference_mps=wheels.left_mps,
        measured_mps=measured_left,
        dt_s=dt_s,
        integrator_before=left_before,
        config=config,
    )
    right_expected = _expected_wheel_terms(
        side="right",
        reference_mps=wheels.right_mps,
        measured_mps=measured_right,
        dt_s=dt_s,
        integrator_before=right_before,
        config=config,
    )
    request = controller(wheels, frame)
    left = _terms_with_production_output(
        left_expected,
        float(controller._left_pi._integral),
        request.left_normalized,
        config,
    )
    right = _terms_with_production_output(
        right_expected,
        float(controller._right_pi._integral),
        request.right_normalized,
        config,
    )
    state_match = bool(
        math.isclose(
            left.integrator_state,
            left_expected.integrator_state,
            rel_tol=0.0,
            abs_tol=1e-15,
        )
        and math.isclose(
            right.integrator_state,
            right_expected.integrator_state,
            rel_tol=0.0,
            abs_tol=1e-15,
        )
    )
    return request, WheelReplayDiagnostics(wheels.context, left, right), state_match


def _expected_wheel_terms(
    *,
    side: str,
    reference_mps: float,
    measured_mps: float,
    dt_s: float,
    integrator_before: float,
    config: NativeControlCompositionConfig,
) -> WheelReplayTerms:
    if abs(reference_mps) <= 1e-9:
        return _zero_replay_terms(reference_mps, dt_s, measured_mps)
    feedforward, maintenance_floor = config.speed_map.lookup(side, reference_mps)
    error = float(reference_mps - measured_mps)
    reset = bool(abs(error) > 0.006 and integrator_before * error < 0.0)
    integral_state = 0.0 if reset else integrator_before
    integral_state += error * dt_s
    limit = float(config.wheel_pi.integrator_limit)
    unclamped_integral = integral_state
    if limit > 0.0:
        integral_state = max(-limit, min(limit, integral_state))
    proportional = float(config.wheel_pi.kp * error)
    integral = float(config.wheel_pi.ki * integral_state)
    raw_unclamped = float(feedforward + proportional + integral)
    maximum = float(config.wheel_pi.max_normalized_output)
    clamped_output = max(-maximum, min(maximum, raw_unclamped))
    sign_reset = reference_mps * clamped_output < 0.0
    return WheelReplayTerms(
        reference_mps=float(reference_mps),
        measured_mps=float(measured_mps),
        dt_s=float(dt_s),
        feedforward=float(feedforward),
        maintenance_floor=float(maintenance_floor),
        error_mps=error,
        proportional=proportional,
        integrator_state=0.0 if sign_reset else float(integral_state),
        integral=0.0 if sign_reset else integral,
        raw_unclamped=raw_unclamped,
        output_normalized=0.0,
        saturated=abs(raw_unclamped) > maximum + 1e-12,
        maintenance_floor_applied=False,
        integrator_reset=reset or sign_reset,
        integrator_clamped=integral_state != unclamped_integral,
    )


def _terms_with_production_output(
    expected: WheelReplayTerms,
    observed_integrator: float,
    output: float,
    config: NativeControlCompositionConfig,
) -> WheelReplayTerms:
    maximum = float(config.wheel_pi.max_normalized_output)
    clamped = max(-maximum, min(maximum, expected.raw_unclamped))
    floor = min(maximum, abs(expected.maintenance_floor))
    floor_applied = bool(
        abs(clamped) < floor
        and math.isclose(abs(output), floor, rel_tol=0.0, abs_tol=1e-15)
    )
    return WheelReplayTerms(
        reference_mps=expected.reference_mps,
        measured_mps=expected.measured_mps,
        dt_s=expected.dt_s,
        feedforward=expected.feedforward,
        maintenance_floor=expected.maintenance_floor,
        error_mps=expected.error_mps,
        proportional=expected.proportional,
        integrator_state=float(observed_integrator),
        integral=float(config.wheel_pi.ki * observed_integrator),
        raw_unclamped=expected.raw_unclamped,
        output_normalized=float(output),
        saturated=expected.saturated,
        maintenance_floor_applied=floor_applied,
        integrator_reset=expected.integrator_reset,
        integrator_clamped=expected.integrator_clamped,
    )


def _zero_replay_terms(
    reference_mps: float,
    dt_s: float,
    measured_mps: float | None = None,
) -> WheelReplayTerms:
    return WheelReplayTerms(
        reference_mps=float(reference_mps),
        measured_mps=None if measured_mps is None else float(measured_mps),
        dt_s=float(dt_s),
        feedforward=0.0,
        maintenance_floor=0.0,
        error_mps=0.0,
        proportional=0.0,
        integrator_state=0.0,
        integral=0.0,
        raw_unclamped=0.0,
        output_normalized=0.0,
        saturated=False,
        maintenance_floor_applied=False,
        integrator_reset=False,
        integrator_clamped=False,
    )


def _control_row(
    result: TickResult,
    motion: Any,
    constrained: Any,
    wheels: Any,
    diagnostics: WheelReplayDiagnostics,
) -> dict[str, object]:
    left = diagnostics.left
    right = diagnostics.right
    return {
        "tick_id": result.trace.context.tick_id,
        "monotonic_ns": result.trace.context.monotonic_ns,
        "l8_requested_v_mps": motion.requested_v_mps,
        "l8_requested_omega_rad_s": motion.requested_omega_rad_s,
        "l9_allowed_v_mps": constrained.allowed_v_mps,
        "l9_allowed_omega_rad_s": constrained.allowed_omega_rad_s,
        "l9_active_constraints": tuple(item.value for item in constrained.active_constraints),
        "l10_left_setpoint_mps": wheels.left_mps,
        "l10_right_setpoint_mps": wheels.right_mps,
        "measured_left_mps": left.measured_mps,
        "measured_right_mps": right.measured_mps,
        "measured_bias_right_minus_left_mps": (
            None
            if left.measured_mps is None or right.measured_mps is None
            else right.measured_mps - left.measured_mps
        ),
        "control_dt_s": left.dt_s,
        "left": _capture_value(left),
        "right": _capture_value(right),
        "l12_left_output": result.final_actuation.left_output,
        "l12_right_output": result.final_actuation.right_output,
        "l12_safety_decision": result.final_actuation.safety_decision.value,
    }


def _analyze_control_rows(
    rows: list[dict[str, object]],
    config: NativeControlCompositionConfig,
) -> dict[str, object]:
    if not rows:
        return {
            "primary_cause": "0_NO_L12_ALLOW_CONTROL_WINDOW",
            "primary_cause_explanation": (
                "The terminal capture is replayable, but it contains no L12 ALLOW "
                "tick from which wheel-control performance could be classified."
            ),
            "secondary_findings": ["CONTROL_TUNING_REQUIRES_ALLOW_TICKS"],
            "classification_gates": {"available": False},
            "setpoint_timeline": {
                "active_tick_count": 0,
                "first_active_tick_id": None,
                "first_full_setpoint_tick_id": None,
                "ramp_tick_count": 0,
                "ramp_duration_s": None,
                "requested_v_mps": None,
                "first_l10_left_mps": None,
                "first_l10_right_mps": None,
                "steady_tick_count": 0,
            },
            "speed_map_feed_forward": {"available": False},
            "pi_dynamics": {
                "available": False,
                "kp": config.wheel_pi.kp,
                "ki": config.wheel_pi.ki,
                "integrator_limit": config.wheel_pi.integrator_limit,
            },
            "limits": {
                "available": False,
                "l11_max_normalized_output": config.wheel_pi.max_normalized_output,
            },
            "measured_wheel_speed": {"available": False},
        }
    requested_max = max(abs(float(row["l8_requested_v_mps"])) for row in rows)
    steady = [
        row
        for row in rows
        if math.isclose(
            abs(float(row["l10_left_setpoint_mps"])),
            requested_max,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        and math.isclose(
            abs(float(row["l10_right_setpoint_mps"])),
            requested_max,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ]
    if not steady:
        steady = list(rows)
    first = rows[0]
    first_steady = steady[0]

    def side_values(source: list[dict[str, object]], side: str, field: str) -> list[float]:
        values: list[float] = []
        for row in source:
            side_value = _mapping(row[side], f"row.{side}").get(field)
            if side_value is not None:
                values.append(float(side_value))
        return values

    left_measured = side_values(steady, "left", "measured_mps")
    right_measured = side_values(steady, "right", "measured_mps")
    left_error = side_values(steady, "left", "error_mps")
    right_error = side_values(steady, "right", "error_mps")
    left_ff = side_values(steady, "left", "feedforward")
    right_ff = side_values(steady, "right", "feedforward")
    left_p = side_values(steady, "left", "proportional")
    right_p = side_values(steady, "right", "proportional")
    left_i = side_values(steady, "left", "integral")
    right_i = side_values(steady, "right", "integral")
    left_output = side_values(steady, "left", "output_normalized")
    right_output = side_values(steady, "right", "output_normalized")
    left_integrator = side_values(rows, "left", "integrator_state")
    right_integrator = side_values(rows, "right", "integrator_state")

    ramp_rows = [
        row
        for row in rows
        if "ACCELERATION_LIMIT" in row["l9_active_constraints"]  # type: ignore[operator]
    ]
    saturation_count = sum(
        bool(_mapping(row[side], side).get("saturated"))
        for row in rows
        for side in ("left", "right")
    )
    floor_count = sum(
        bool(_mapping(row[side], side).get("maintenance_floor_applied"))
        for row in rows
        for side in ("left", "right")
    )
    reset_counts = {
        side: sum(
            bool(_mapping(row[side], side).get("integrator_reset")) for row in rows
        )
        for side in ("left", "right")
    }
    clamp_counts = {
        side: sum(
            bool(_mapping(row[side], side).get("integrator_clamped")) for row in rows
        )
        for side in ("left", "right")
    }
    l12_modified_count = sum(
        not math.isclose(
            float(row[f"l12_{side}_output"]),
            float(_mapping(row[side], side)["output_normalized"]),
            rel_tol=0.0,
            abs_tol=1e-15,
        )
        for row in rows
        for side in ("left", "right")
    )
    constraint_counts = {
        code.value: sum(
            code.value in row["l9_active_constraints"]  # type: ignore[operator]
            for row in rows
        )
        for code in ConstraintCode
    }

    left_ratio = _mean(left_measured) / requested_max
    right_ratio = _mean(right_measured) / requested_max
    left_correction_fraction = abs(_mean(left_output) - _mean(left_ff)) / abs(_mean(left_ff))
    right_correction_fraction = abs(_mean(right_output) - _mean(right_ff)) / abs(_mean(right_ff))
    steady_limit_count = sum(bool(row["l9_active_constraints"]) for row in steady)
    ff_mismatch_primary = bool(
        min(left_ratio, right_ratio) < 0.90
        and max(left_correction_fraction, right_correction_fraction) < 0.15
        and steady_limit_count == 0
        and saturation_count == 0
        and l12_modified_count == 0
    )
    primary = (
        "1_SPEED_MAP_FEED_FORWARD_MISMATCH"
        if ff_mismatch_primary
        else "5_OTHER_REQUIRES_FURTHER_EVIDENCE"
    )

    return {
        "primary_cause": primary,
        "primary_cause_explanation": (
            "At the full 0.15 m/s setpoint the capture-time ACTIVE map and source "
            "replay command only small PI additions, yet both measured wheel means "
            "remain below 90% of target with no steady L9, L11 saturation, or L12 "
            "output limit. This makes the floor-load feed-forward/plant mismatch "
            "primary; PI dynamics and encoder quantization are secondary."
            if ff_mismatch_primary
            else "The bounded automatic classifier could not isolate one primary cause."
        ),
        "secondary_findings": [
            "PI_INTEGRATOR_RESETS_ON_OPPOSITE_SIGN_QUANTIZED_ERROR"
            if reset_counts["left"] or reset_counts["right"]
            else "PI_INTEGRATOR_NO_SIGN_RESET",
            "L9_ACCELERATION_LIMIT_TRANSIENT_ONLY"
            if ramp_rows and steady_limit_count == 0
            else "L9_LIMIT_REQUIRES_REVIEW",
        ],
        "classification_gates": {
            "minimum_steady_measured_to_target_ratio": min(left_ratio, right_ratio),
            "maximum_mean_pi_addition_to_ff_fraction": max(
                left_correction_fraction,
                right_correction_fraction,
            ),
            "steady_l9_limited_tick_count": steady_limit_count,
            "l11_saturated_wheel_tick_count": saturation_count,
            "l12_modified_wheel_output_count": l12_modified_count,
        },
        "setpoint_timeline": {
            "active_tick_count": len(rows),
            "first_active_tick_id": first["tick_id"],
            "first_full_setpoint_tick_id": first_steady["tick_id"],
            "ramp_tick_count": len(ramp_rows),
            "ramp_duration_s": (
                int(first_steady["monotonic_ns"]) - int(first["monotonic_ns"])
            )
            / 1_000_000_000.0,
            "requested_v_mps": requested_max,
            "first_l10_left_mps": first["l10_left_setpoint_mps"],
            "first_l10_right_mps": first["l10_right_setpoint_mps"],
            "steady_tick_count": len(steady),
        },
        "speed_map_feed_forward": {
            "steady_left_mean": _mean(left_ff),
            "steady_right_mean": _mean(right_ff),
            "steady_left_output_mean": _mean(left_output),
            "steady_right_output_mean": _mean(right_output),
            "left_pi_addition_fraction_of_ff": left_correction_fraction,
            "right_pi_addition_fraction_of_ff": right_correction_fraction,
        },
        "pi_dynamics": {
            "kp": config.wheel_pi.kp,
            "ki": config.wheel_pi.ki,
            "integrator_limit": config.wheel_pi.integrator_limit,
            "steady_left_p_mean": _mean(left_p),
            "steady_right_p_mean": _mean(right_p),
            "steady_left_i_mean": _mean(left_i),
            "steady_right_i_mean": _mean(right_i),
            "left_integrator_end": left_integrator[-1],
            "right_integrator_end": right_integrator[-1],
            "left_integrator_max_abs": max(map(abs, left_integrator)),
            "right_integrator_max_abs": max(map(abs, right_integrator)),
            "integrator_reset_count": reset_counts,
            "integrator_clamp_count": clamp_counts,
        },
        "limits": {
            "l9_constraint_tick_count": constraint_counts,
            "l11_max_normalized_output": config.wheel_pi.max_normalized_output,
            "l11_saturated_wheel_tick_count": saturation_count,
            "l11_maintenance_floor_applied_wheel_tick_count": floor_count,
            "maximum_abs_l11_raw_unclamped": max(
                abs(value)
                for side in ("left", "right")
                for value in side_values(rows, side, "raw_unclamped")
            ),
            "maximum_abs_l11_output": max(
                abs(value)
                for side in ("left", "right")
                for value in side_values(rows, side, "output_normalized")
            ),
            "l12_modified_wheel_output_count": l12_modified_count,
        },
        "measured_wheel_speed": {
            "steady_left_mean_mps": _mean(left_measured),
            "steady_right_mean_mps": _mean(right_measured),
            "steady_left_tracking_error_mean_mps": _mean(left_error),
            "steady_right_tracking_error_mean_mps": _mean(right_error),
            "steady_left_tracking_error_rms_mps": _rms(left_error),
            "steady_right_tracking_error_rms_mps": _rms(right_error),
            "steady_left_measured_to_target_ratio": left_ratio,
            "steady_right_measured_to_target_ratio": right_ratio,
            "steady_bias_right_minus_left_mean_mps": _mean(
                [right - left for left, right in zip(left_measured, right_measured)]
            ),
            "steady_left_min_mps": min(left_measured),
            "steady_left_max_mps": max(left_measured),
            "steady_right_min_mps": min(right_measured),
            "steady_right_max_mps": max(right_measured),
        },
    }


def _reconstruct_inputs(tick: Mapping[str, object]) -> TickInputs:
    tick_id = _integer(tick.get("tick_id"), "tick.tick_id")
    monotonic_ns = _integer(tick.get("monotonic_ns"), "tick.monotonic_ns")
    context = TickContext(tick_id, monotonic_ns)
    layers = _mapping(tick.get("layers"), "tick.layers")
    l1 = _mapping(layers.get("L1"), "tick.layers.L1")
    _require_type(l1, "AcquisitionFrame", "L1")
    if _context(l1.get("context"), "L1.context") != context:
        raise V3ReplayError(f"tick {tick_id} L1 context mismatch")
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
        for row in _mapping_sequence(l1.get("samples"), "L1.samples")
    )
    health = tuple(
        DeviceHealth(
            device_id=str(row.get("device_id", "")),
            state=DeviceHealthState(str(row.get("state", ""))),
            reason=None if row.get("reason") is None else str(row.get("reason")),
        )
        for row in _mapping_sequence(l1.get("io_health"), "L1.io_health")
    )
    raw = RawDeviceBatch(context, samples, health)

    l5 = _mapping(layers.get("L5"), "tick.layers.L5")
    _require_type(l5, "MissionIntent", "L5")
    if _context(l5.get("context"), "L5.context") != context:
        raise V3ReplayError(f"tick {tick_id} L5 context mismatch")
    mission_id = str(l5.get("mission_id", ""))
    if not mission_id.startswith("mission-"):
        raise V3ReplayError(f"tick {tick_id} mission id cannot reconstruct command")
    command_id = mission_id.removeprefix("mission-")
    mode = CommandMode(str(l5.get("mode", "")))
    if mode is CommandMode.TELEOP:
        velocity = _mapping(l5.get("velocity_target"), "L5.velocity_target")
        constraints = _mapping(l5.get("constraints"), "L5.constraints")
        goal = (
            DataField("v_mps", _number(velocity.get("v_mps"), "v_mps")),
            DataField(
                "omega_rad_s",
                _number(velocity.get("omega_rad_s"), "omega_rad_s"),
            ),
            DataField("max_v_mps", _number(constraints.get("max_v_mps"), "max_v_mps")),
            DataField(
                "max_omega_rad_s",
                _number(constraints.get("max_omega_rad_s"), "max_omega_rad_s"),
            ),
        )
    elif mode is CommandMode.STOP:
        goal = ()
    else:
        raise V3ReplayError(
            f"tick {tick_id} floor capture command mode is not STOP/TELEOP"
        )
    command = CommandRequest(context, command_id, mode, goal, tick_id)
    mission_lifecycle = str(l5.get("lifecycle", ""))
    lifecycle = (
        LifecycleState.ACTIVE
        if mission_lifecycle == LifecycleState.ACTIVE.value
        else LifecycleState.IDLE
    )
    return TickInputs(context, raw, command, lifecycle)


def _load_control_config(
    physics_path: Path,
    speed_map_path: Path,
    hardware_path: Path,
    inputs: Sequence[TickInputs],
) -> NativeControlCompositionConfig:
    physics = _json_object(physics_path, "physics config")
    speed_map_raw = _json_object(speed_map_path, "speed-map config")
    hardware = _json_object(hardware_path, "hardware config")
    track_width = _number(
        physics.get("nyomtav_szelesseg_m"),
        "physics.nyomtav_szelesseg_m",
    )
    if track_width <= 0.0:
        raise V3ReplayError("physics.nyomtav_szelesseg_m must be positive")
    lidar_safety = _load_lidar_safety_config(hardware, inputs)
    return NativeControlCompositionConfig(
        speed_map=WheelSpeedMap.from_mapping(speed_map_raw),
        estimation=NativeStateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=track_width,
        ),
        chassis_control=ChassisControlConfig(track_width),
        lidar_safety=lidar_safety,
    )


def _load_lidar_safety_config(
    hardware: Mapping[str, object],
    inputs: Sequence[TickInputs],
) -> LidarSafetyConfig | None:
    """Reconstruct the native gate while retaining pre-native capture support."""

    device_ids = {
        sample.device_id
        for tick_input in inputs
        for sample in tick_input.raw_devices.samples
        if sample.kind == "lidar_safety_clearance"
    }
    if not device_ids:
        return None
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


def _lidar_safety_reconstruction(
    config: LidarSafetyConfig | None,
) -> dict[str, object]:
    if config is None:
        return {
            "active": False,
            "compatibility": "PRE_NATIVE_CAPTURE_WITHOUT_LIDAR_SAFETY_SAMPLE",
        }
    return {
        "active": True,
        "device_id": config.device_id,
        "minimum_clearance_m": config.minimum_clearance_m,
        "maximum_sample_age_ns": config.maximum_sample_age_ns,
        "source": "CAPTURE_DEVICE_ID_PLUS_ACTIVE_HARDWARE_CONFIG",
    }


def _validate_capture(
    payload: Mapping[str, object],
) -> tuple[Mapping[str, object], ...]:
    if payload.get("schema") != V3_FLOOR_CAPTURE_SCHEMA:
        raise V3ReplayError("unsupported V3 floor capture schema")
    if payload.get("status") not in _REPLAYABLE_CAPTURE_STATUSES:
        raise V3ReplayError(
            "V3 floor capture status must be terminal PASS/FAIL/FAULT"
        )
    ticks = _mapping_sequence(payload.get("ticks"), "capture.ticks")
    if not ticks:
        raise V3ReplayError("V3 floor capture contains no ticks")
    declared = _integer(payload.get("tick_count"), "capture.tick_count")
    if declared != len(ticks):
        raise V3ReplayError("V3 floor capture tick count mismatch")
    previous_tick_id: int | None = None
    previous_ns: int | None = None
    for tick in ticks:
        tick_id = _integer(tick.get("tick_id"), "tick.tick_id")
        monotonic_ns = _integer(tick.get("monotonic_ns"), "tick.monotonic_ns")
        if previous_tick_id is not None and tick_id != previous_tick_id + 1:
            raise V3ReplayError("V3 floor capture tick ids must be contiguous")
        if previous_ns is not None and monotonic_ns <= previous_ns:
            raise V3ReplayError("V3 floor capture time must increase")
        layers = _mapping(tick.get("layers"), "tick.layers")
        if tuple(sorted(layers, key=lambda item: int(item[1:]))) != _LAYER_ORDER:
            raise V3ReplayError(f"tick {tick_id} must contain exactly L1-L12")
        if _integer(tick.get("layer_count"), "tick.layer_count") != 12:
            raise V3ReplayError(f"tick {tick_id} layer_count must be 12")
        previous_tick_id = tick_id
        previous_ns = monotonic_ns
    return ticks


def _execution_summary(
    ticks: tuple[Mapping[str, object], ...],
    capture_status: str,
) -> dict[str, object]:
    """Describe the recorded execution independently of replay equivalence."""

    decision_counts = {item.value: 0 for item in SafetyDecision}
    first_allow_tick_id: int | None = None
    first_non_allow_active_tick_id: int | None = None
    for tick in ticks:
        tick_id = _integer(tick.get("tick_id"), "tick.tick_id")
        layers = _mapping(tick.get("layers"), "tick.layers")
        l5 = _mapping(layers.get("L5"), "tick.layers.L5")
        l12 = _mapping(layers.get("L12"), "tick.layers.L12")
        decision = SafetyDecision(str(l12.get("safety_decision", "")))
        decision_counts[decision.value] += 1
        if decision is SafetyDecision.ALLOW and first_allow_tick_id is None:
            first_allow_tick_id = tick_id
        if (
            str(l5.get("lifecycle", "")) == LifecycleState.ACTIVE.value
            and decision is not SafetyDecision.ALLOW
            and first_non_allow_active_tick_id is None
        ):
            first_non_allow_active_tick_id = tick_id

    terminal = _mapping(ticks[-1].get("final_actuation"), "tick.final_actuation")
    terminal_layers = _mapping(ticks[-1].get("layers"), "tick.layers")
    terminal_l5 = _mapping(terminal_layers.get("L5"), "tick.layers.L5")
    return {
        "capture_status": capture_status,
        "capture_passed": capture_status == "PASS",
        "decision_counts": decision_counts,
        "first_allow_tick_id": first_allow_tick_id,
        "first_non_allow_active_tick_id": first_non_allow_active_tick_id,
        "terminal_tick_id": _integer(ticks[-1].get("tick_id"), "tick.tick_id"),
        "terminal_lifecycle": str(terminal_l5.get("lifecycle", "")),
        "terminal_safety_decision": str(terminal.get("safety_decision", "")),
        "terminal_reason": terminal.get("reason"),
        "terminal_fault_layer": ticks[-1].get("fault_layer"),
    }


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
    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "__type__": type(value).__name__,
            **{
                field.name: _capture_value(getattr(value, field.name))
                for field in fields(value)
            },
        }
    if isinstance(value, Mapping):
        return {str(key): _capture_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_capture_value(item) for item in value]
    raise V3ReplayError(f"cannot serialize replay value {type(value).__name__}")


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


def _mean(values: Sequence[float]) -> float:
    if not values:
        raise V3ReplayError("cannot calculate a mean from no values")
    return math.fsum(values) / len(values)


def _rms(values: Sequence[float]) -> float:
    return math.sqrt(_mean([value * value for value in values]))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect_parser = subparsers.add_parser("inspect", help="inspect a V3 floor capture")
    inspect_parser.add_argument("capture_path")
    replay_parser = subparsers.add_parser("replay", help="replay production L1-L12")
    replay_parser.add_argument("capture_path")
    replay_parser.add_argument("--physics-config", default="conf/fizika.json")
    replay_parser.add_argument("--speed-map-config", default="conf/speed_map.json")
    replay_parser.add_argument("--hardware-config", default="conf/hardver.json")
    replay_parser.add_argument("--project-root", default=".")
    replay_parser.add_argument("--capture-source-manifest")
    replay_parser.add_argument("--output", required=True)
    verify_parser = subparsers.add_parser("verify-result", help="verify a replay result")
    verify_parser.add_argument("result_path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "inspect":
            output = inspect_floor_capture(args.capture_path)
        elif args.command == "replay":
            output = replay_floor_capture(
                args.capture_path,
                physics_config_path=args.physics_config,
                speed_map_config_path=args.speed_map_config,
                hardware_config_path=args.hardware_config,
                project_root=args.project_root,
                capture_source_manifest_path=args.capture_source_manifest,
            )
            path = write_replay_result(output, args.output)
            output = {
                "status": output["status"],
                "result_path": str(path.resolve()),
                "result_sha256": output["result_sha256"],
                "first_divergence": output["first_divergence"],
                "primary_cause": _mapping(output["analysis"], "analysis")[
                    "primary_cause"
                ],
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
    "V3ReplayError",
    "first_divergence",
    "inspect_floor_capture",
    "replay_floor_capture",
    "run_replay",
    "verify_replay_result",
    "write_replay_result",
]
