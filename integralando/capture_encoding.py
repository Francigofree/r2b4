"""Container-neutral R2B4 V3 capture encoding.

This module converts immutable V3 typed values into JSON-compatible values. It
contains no file format, MCAP, JSON-document, replay, motor, lifecycle or device
I/O authority. A capture container may encode the returned mapping once to bytes.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import fields, is_dataclass
from enum import Enum

from .engine import LAYER_ORDER
from .execution import CaptureRecord, EdgeFaultRecord, ExecutionRecord, WriterFailureRecord

_INPUT_REFERENCE_KEY = "__capture_input_reference__"


class CaptureEncodingError(RuntimeError):
    """A capture value violates the serialization/integrity contract."""


def _expected_trace_layers(fault_layer: object) -> tuple[str, ...]:
    if fault_layer is None:
        return LAYER_ORDER
    if not isinstance(fault_layer, str) or not fault_layer.strip():
        raise CaptureEncodingError("fault_layer must be a non-empty string or null")
    normalized = fault_layer.strip()
    if normalized == "L12":
        raise CaptureEncodingError("a completed trace cannot report L12 as its fault layer")
    if normalized in LAYER_ORDER[:-1]:
        return LAYER_ORDER[: LAYER_ORDER.index(normalized)] + ("L12",)
    return ("L12",)


def _validate_trace_layers(
    layer_names: Sequence[str],
    fault_layer: object,
    *,
    preserve_order: bool,
) -> tuple[str, ...]:
    expected = _expected_trace_layers(fault_layer)
    observed = tuple(layer_names)
    valid = observed == expected if preserve_order else (
        len(observed) == len(expected) and set(observed) == set(expected)
    )
    if not valid:
        fault = "none" if fault_layer is None else str(fault_layer)
        raise CaptureEncodingError(
            "trace layers must be the completed L1 prefix followed by L12 "
            f"for fault_layer={fault}; expected {','.join(expected)}"
        )
    return expected


def encode_value(value: object) -> object:
    """Encode typed V3 values without making serialization runtime authority."""

    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise CaptureEncodingError("capture values must be finite")
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "__type__": type(value).__name__,
            **{
                field.name: encode_value(getattr(value, field.name))
                for field in fields(value)
            },
        }
    if isinstance(value, Mapping):
        return {str(key): encode_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [encode_value(item) for item in value]
    raise CaptureEncodingError(f"cannot serialize capture value {type(value).__name__}")


def encode_capture_record(record: CaptureRecord) -> dict[str, object]:
    """Encode one immutable completed production record."""

    if isinstance(record, ExecutionRecord):
        context = record.inputs.context
        row: dict[str, object] = {
            "record_type": "closed_input_tick",
            "tick_id": context.tick_id,
            "monotonic_ns": context.monotonic_ns,
            "inputs": encode_value(record.inputs),
            "expected": _encode_completed_result(record.result, record.inputs),
        }
        if record.evidence:
            row["tick_evidence"] = encode_value(record.evidence)
        return row
    if isinstance(record, EdgeFaultRecord):
        return {
            "record_type": "edge_fault_tick",
            "tick_id": record.context.tick_id,
            "monotonic_ns": record.context.monotonic_ns,
            "edge_fault": {
                "context": encode_value(record.context),
                "lifecycle": record.lifecycle.value,
                "reason": record.reason,
                "fault_layer": record.fault_layer,
                "critical_health": encode_value(record.critical_health),
                "raw_devices": encode_value(record.raw_devices),
            },
            "expected": _encode_completed_result(record.result),
        }
    if isinstance(record, WriterFailureRecord):
        row = {
            "record_type": (
                "closed_input_tick" if record.inputs is not None else "edge_fault_tick"
            ),
            "tick_id": record.context.tick_id,
            "monotonic_ns": record.context.monotonic_ns,
            "expected": {
                "fault_layer": "L12",
                "layers": {},
                "writer_failure": {
                    "reason": record.reason,
                    "attempted_actuation": encode_value(record.attempted_actuation),
                },
            },
        }
        if record.inputs is not None:
            row["inputs"] = encode_value(record.inputs)
        else:
            row["edge_fault"] = {
                "context": encode_value(record.context),
                "lifecycle": record.lifecycle.value,
                "reason": record.reason,
                "fault_layer": "L12",
                "critical_health": [],
                "raw_devices": encode_value(record.raw_devices),
            }
        return row
    raise TypeError("capture record must be a supported immutable capture record")


def _encode_completed_result(
    result: object,
    inputs: object | None = None,
) -> dict[str, object]:
    from .contracts import AcquisitionFrame, AdmittedFrame
    from .engine import TickInputs, TickResult

    if not isinstance(result, TickResult):
        raise TypeError("capture result must be TickResult")
    layer_names = tuple(layer.layer for layer in result.trace.layers)
    _validate_trace_layers(layer_names, result.trace.fault_layer, preserve_order=True)
    encoded_layers: dict[str, object] = {}
    for layer in result.trace.layers:
        output = layer.output
        if (
            layer.layer == "L1"
            and isinstance(inputs, TickInputs)
            and isinstance(output, AcquisitionFrame)
            and output.context == inputs.context
            and output.samples == inputs.raw_devices.samples
            and output.io_health == inputs.raw_devices.device_health
        ):
            encoded_layers[layer.layer] = {_INPUT_REFERENCE_KEY: "RAW_DEVICE_BATCH"}
            continue
        if (
            layer.layer == "L2"
            and isinstance(inputs, TickInputs)
            and isinstance(output, AdmittedFrame)
        ):
            compact = _compact_admitted_frame(output, inputs)
            if compact is not None:
                encoded_layers[layer.layer] = compact
                continue
        encoded_layers[layer.layer] = encode_value(output)
    return {
        "fault_layer": result.trace.fault_layer,
        "layers": encoded_layers,
    }


def _compact_admitted_frame(output: object, inputs: object) -> dict[str, object] | None:
    from .contracts import AdmittedFrame
    from .engine import TickInputs

    if not isinstance(output, AdmittedFrame) or not isinstance(inputs, TickInputs):
        return None
    if output.context != inputs.context:
        return None
    sample_indices: dict[tuple[str, str, int], int] = {
        (sample.device_id, sample.kind, sample.sequence): index
        for index, sample in enumerate(inputs.raw_devices.samples)
    }
    accepted_indices: list[int] = []
    for observation in output.accepted:
        index = sample_indices.get(
            (
                observation.source_device_id,
                observation.kind,
                observation.source_sequence,
            )
        )
        if index is None:
            return None
        sample = inputs.raw_devices.samples[index]
        if (
            observation.captured_monotonic_ns != sample.captured_monotonic_ns
            or observation.values != sample.values
        ):
            return None
        accepted_indices.append(index)
    return {
        _INPUT_REFERENCE_KEY: "ADMITTED_FRAME",
        "accepted_sample_indices": accepted_indices,
        "rejected": encode_value(output.rejected),
        "degraded_sources": encode_value(output.degraded_sources),
    }


def encode_raw_lidar_snapshot(snapshot: object, point_limit: int) -> dict[str, object]:
    """Encode one native LiDAR scan, rejecting silent contract drift."""

    revision = getattr(snapshot, "raw_scan_id", None)
    timestamp = getattr(snapshot, "raw_scan_timestamp", None)
    scan_start_ns = getattr(snapshot, "scan_start_monotonic_ns", None)
    scan_end_ns = getattr(snapshot, "scan_end_monotonic_ns", None)
    measurement_ns = getattr(snapshot, "measurement_monotonic_ns", None)
    health = getattr(snapshot, "health", None)
    points = getattr(snapshot, "raw_scan", None)
    summary = getattr(snapshot, "summary", None)
    if (
        not isinstance(revision, int)
        or isinstance(revision, bool)
        or revision <= 0
        or isinstance(timestamp, bool)
        or not isinstance(timestamp, (int, float))
        or not math.isfinite(timestamp)
        or timestamp < 0.0
        or not isinstance(scan_start_ns, int)
        or isinstance(scan_start_ns, bool)
        or scan_start_ns < 0
        or not isinstance(scan_end_ns, int)
        or isinstance(scan_end_ns, bool)
        or scan_end_ns < scan_start_ns
        or not isinstance(measurement_ns, int)
        or isinstance(measurement_ns, bool)
        or measurement_ns != scan_start_ns + (scan_end_ns - scan_start_ns) // 2
        or int(round(float(timestamp) * 1_000_000_000)) != scan_end_ns
        or not isinstance(health, str)
        or not health
        or not isinstance(points, tuple)
        or not isinstance(summary, Mapping)
    ):
        raise CaptureEncodingError("raw lidar snapshot has an invalid native contract")

    selected_points = points[:point_limit]
    compact_points: list[list[float | int]] = []
    for point in selected_points:
        angle_deg = getattr(point, "angle_deg", None)
        distance_m = getattr(point, "distance_m", None)
        quality = getattr(point, "quality", None)
        if (
            isinstance(angle_deg, bool)
            or not isinstance(angle_deg, (int, float))
            or not math.isfinite(angle_deg)
            or isinstance(distance_m, bool)
            or not isinstance(distance_m, (int, float))
            or not math.isfinite(distance_m)
            or not isinstance(quality, int)
            or isinstance(quality, bool)
        ):
            raise CaptureEncodingError("raw lidar point has an invalid native contract")
        compact_points.append([float(angle_deg), float(distance_m), quality])

    return {
        "revision": revision,
        "captured_monotonic_ns": int(round(float(timestamp) * 1_000_000_000)),
        "scan_start_monotonic_ns": scan_start_ns,
        "scan_end_monotonic_ns": scan_end_ns,
        "measurement_monotonic_ns": measurement_ns,
        "health": health,
        "source_point_count": len(points),
        "points_truncated": len(selected_points) != len(points),
        "point_encoding": "ANGLE_DEG_DISTANCE_M_QUALITY",
        "points": compact_points,
        "summary": encode_value(summary),
    }


__all__ = [
    "CaptureEncodingError",
    "encode_capture_record",
    "encode_raw_lidar_snapshot",
    "encode_value",
]
