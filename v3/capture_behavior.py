"""Small immutable system-behavior observations for sampled capture.

Projection is shallow and performed only for requested capture ticks, before
IPC. Sensor values, costmap cells, rollout geometry and replay checkpoints are
never walked or retained. Encoding remains a consumer responsibility.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

from .capture_encoding import encode_value
from .contracts import TickContext
from .execution import CaptureRecord, EdgeFaultRecord, ExecutionRecord, WriterFailureRecord


@dataclass(frozen=True, slots=True)
class BehaviorFields:
    values: tuple[tuple[str, object], ...]


@dataclass(frozen=True, slots=True)
class BehaviorCaptureRecord:
    context: TickContext
    record_type: str
    inputs: BehaviorFields
    layers: tuple[tuple[str, BehaviorFields], ...]
    fault_layer: str | None
    trigger_reason: str | None
    edge_fault: BehaviorFields | None = None
    writer_failure: BehaviorFields | None = None
    evidence: tuple[object, ...] = ()


def _fields(value: object, *, omit: tuple[str, ...] = (),
            extra: tuple[tuple[str, object], ...] = ()) -> BehaviorFields:
    return BehaviorFields(tuple(
        (field.name, getattr(value, field.name))
        for field in fields(value) if field.name not in omit
    ) + extra)


def _layer_fields(layer: str, value: object) -> BehaviorFields:
    if layer == "L1":
        return _fields(value, omit=("samples",), extra=(("sample_count", len(value.samples)),))
    if layer == "L2":
        return _fields(value, omit=("accepted",), extra=(("accepted_count", len(value.accepted)),))
    if layer == "L4":
        costmap = value.local_costmap
        return _fields(value, omit=("obstacle_tracks", "local_costmap"), extra=(
            ("obstacle_track_count", len(value.obstacle_tracks)),
            ("local_costmap", None if costmap is None else _fields(
                costmap, omit=("occupied_cells",),
                extra=(("occupied_cell_count", len(costmap.occupied_cells)),))),
        ))
    if layer == "L6":
        return _fields(value, omit=("route", "trajectory_candidates"), extra=(
            ("route_waypoint_count", len(value.route)),
            ("trajectory_candidate_count", len(value.trajectory_candidates)),
        ))
    if layer == "L7":
        trajectory = value.trajectory
        return _fields(value, omit=("trajectory",), extra=(
            ("trajectory", None if trajectory is None else _fields(
                trajectory, omit=("samples",),
                extra=(("sample_count", len(trajectory.samples)),))),
        ))
    # Pose/covariance, mission, requested/constrained motion, wheels and final
    # actuation are already small immutable system-state values.
    return _fields(value)


def project_behavior_record(record: CaptureRecord) -> BehaviorCaptureRecord:
    """Retain behavior and lineage without copying any sensor/geometry payload."""
    closed = getattr(record, "inputs", None)
    context = closed.context if closed is not None else record.context
    raw = closed.raw_devices if closed is not None else record.raw_devices
    inputs = BehaviorFields((
        ("context", context),
        ("lifecycle", closed.lifecycle if closed is not None else record.lifecycle),
        ("command", closed.command if closed is not None else None),
        ("raw_devices", BehaviorFields((
            ("device_health", raw.device_health if raw is not None else ()),
        ))),
    ))
    if isinstance(record, WriterFailureRecord):
        return BehaviorCaptureRecord(
            context, "system_behavior_tick", inputs, (), "L12", record.reason,
            writer_failure=BehaviorFields((
                ("reason", record.reason), ("attempted_actuation", record.attempted_actuation),
            )),
        )
    result = record.result
    fault = result.trace.fault_layer
    trigger = None
    edge = None
    if isinstance(record, EdgeFaultRecord):
        trigger = record.reason
        edge = BehaviorFields((("reason", record.reason), ("fault_layer", record.fault_layer)))
    elif fault is not None or result.final_actuation.safety_decision.value == "FAULT":
        trigger = result.final_actuation.reason or fault or "FAULT"
    return BehaviorCaptureRecord(
        context, "system_behavior_tick", inputs,
        tuple((layer.layer, _layer_fields(layer.layer, layer.output)) for layer in result.trace.layers),
        fault, trigger, edge_fault=edge,
        evidence=record.evidence if isinstance(record, ExecutionRecord) else (),
    )


def _encode_fields(value: BehaviorFields) -> dict[str, object]:
    return {
        name: _encode_fields(item) if isinstance(item, BehaviorFields) else encode_value(item)
        for name, item in value.values
    }


def encode_behavior_record(record: BehaviorCaptureRecord) -> dict[str, object]:
    expected = {
        "fault_layer": record.fault_layer,
        "layers": {name: _encode_fields(value) for name, value in record.layers},
    }
    if record.writer_failure is not None:
        expected["writer_failure"] = _encode_fields(record.writer_failure)
    row = {
        "record_type": record.record_type,
        "tick_id": record.context.tick_id,
        "monotonic_ns": record.context.monotonic_ns,
        "inputs": _encode_fields(record.inputs),
        "expected": expected,
    }
    if record.edge_fault is not None:
        row["edge_fault"] = _encode_fields(record.edge_fault)
    if record.evidence:
        row["tick_evidence"] = encode_value(record.evidence)
    return row
