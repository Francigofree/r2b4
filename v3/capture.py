"""General run-bound V3 capture sink, separate from live and replay authority."""

from __future__ import annotations

import hashlib
import json
import math
import os
import queue
import threading
from collections import OrderedDict, deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any

from .engine import LAYER_ORDER
from .execution import (
    CaptureRecord,
    EdgeFaultRecord,
    ExecutionRecord,
    OutputSink,
    WriterFailureRecord,
)


V3_CAPTURE_SCHEMA = "R2B4_V3_CAPTURE_V1"
V3_CAPTURE_STATUSES = frozenset(("PASS", "FAIL", "FAULT"))


@dataclass(frozen=True, slots=True)
class CaptureWindowConfig:
    """Hard RAM bounds for the passive triggered capture path."""

    pre_event_ns: int = 5_000_000_000
    post_event_ns: int = 2_000_000_000
    ingress_queue_capacity: int = 256
    max_tick_count: int = 512
    max_byte_capacity: int = 16 * 1024 * 1024
    max_raw_lidar_scans: int = 64
    max_raw_lidar_points_per_scan: int = 4_096

    def __post_init__(self) -> None:
        for name in ("pre_event_ns", "post_event_ns"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        for name in (
            "ingress_queue_capacity",
            "max_tick_count",
            "max_byte_capacity",
            "max_raw_lidar_scans",
            "max_raw_lidar_points_per_scan",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


class V3CaptureError(RuntimeError):
    """A V3 capture is incomplete, corrupt, or not serializable."""


def _expected_trace_layers(fault_layer: object) -> tuple[str, ...]:
    """Return the only valid completed trace shape for one production tick."""

    if fault_layer is None:
        return LAYER_ORDER
    if not isinstance(fault_layer, str) or not fault_layer.strip():
        raise V3CaptureError("fault_layer must be a non-empty string or null")
    normalized = fault_layer.strip()
    if normalized == "L12":
        raise V3CaptureError("a completed trace cannot report L12 as its fault layer")
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
        raise V3CaptureError(
            "trace layers must be the completed L1 prefix followed by L12 "
            f"for fault_layer={fault}; expected {','.join(expected)}"
        )
    return expected


def encode_value(value: object) -> object:
    """Encode typed V3 values without turning serialization into runtime authority."""

    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise V3CaptureError("capture values must be finite")
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
    raise V3CaptureError(f"cannot serialize capture value {type(value).__name__}")


def encode_capture_record(record: CaptureRecord) -> dict[str, object]:
    """Encode one immutable observer record outside the production tick loop."""

    if isinstance(record, ExecutionRecord):
        context = record.inputs.context
        return {
            "record_type": "closed_input_tick",
            "tick_id": context.tick_id,
            "monotonic_ns": context.monotonic_ns,
            "inputs": encode_value(record.inputs),
            "expected": _encode_completed_result(record.result),
        }
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
        row: dict[str, object] = {
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


def _encode_completed_result(result: object) -> dict[str, object]:
    from .engine import TickResult

    if not isinstance(result, TickResult):
        raise TypeError("capture result must be TickResult")
    layer_names = tuple(layer.layer for layer in result.trace.layers)
    _validate_trace_layers(
        layer_names,
        result.trace.fault_layer,
        preserve_order=True,
    )
    return {
        "fault_layer": result.trace.fault_layer,
        "layers": {
            layer.layer: encode_value(layer.output)
            for layer in result.trace.layers
        },
    }


class CaptureSink(OutputSink):
    """Passively capture closed inputs and their production L1-L12 outputs."""

    __slots__ = ("_capture_id", "_configuration", "_metadata", "_ticks")

    def __init__(
        self,
        capture_id: str,
        *,
        configuration: Mapping[str, object],
        metadata: Mapping[str, object] | None = None,
    ) -> None:
        identifier = str(capture_id or "").strip()
        if not identifier:
            raise ValueError("capture_id must be non-empty")
        self._capture_id = identifier
        encoded_configuration = encode_value(configuration)
        encoded_metadata = encode_value(metadata or {})
        if not isinstance(encoded_configuration, dict) or not isinstance(encoded_metadata, dict):
            raise TypeError("capture configuration and metadata must be mappings")
        self._configuration = encoded_configuration
        self._metadata = encoded_metadata
        self._ticks: list[dict[str, object]] = []

    def write(self, record: CaptureRecord) -> None:
        self.write_encoded(encode_capture_record(record))

    def write_encoded(self, tick: Mapping[str, object]) -> None:
        row = dict(tick)
        tick_id = _non_negative_integer(row.get("tick_id"), "tick_id")
        monotonic_ns = _non_negative_integer(row.get("monotonic_ns"), "monotonic_ns")
        if self._ticks:
            previous = self._ticks[-1]
            if tick_id != int(previous["tick_id"]) + 1:
                raise V3CaptureError("capture tick ids must be contiguous")
            if monotonic_ns <= int(previous["monotonic_ns"]):
                raise V3CaptureError("capture monotonic time must increase")
        self._ticks.append(row)

    def document(
        self,
        status: str,
        *,
        capture_window: Mapping[str, object] | None = None,
        capture_integrity: Mapping[str, object] | None = None,
        raw_lidar_scans: Sequence[Mapping[str, object]] = (),
        raw_lidar_evidence: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        terminal_status = str(status or "").upper()
        if terminal_status not in V3_CAPTURE_STATUSES:
            raise V3CaptureError("capture status must be terminal PASS/FAIL/FAULT")
        if not self._ticks:
            raise V3CaptureError("capture contains no ticks")
        payload: dict[str, object] = {
            "schema": V3_CAPTURE_SCHEMA,
            "capture_id": self._capture_id,
            "status": terminal_status,
            "created_at_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
                "+00:00", "Z"
            ),
            "configuration": self._configuration,
            "metadata": self._metadata,
            "tick_count": len(self._ticks),
            "ticks": list(self._ticks),
        }
        if capture_window is not None:
            payload["capture_window"] = encode_value(capture_window)
        if capture_integrity is not None:
            payload["capture_integrity"] = encode_value(capture_integrity)
        if raw_lidar_scans:
            payload["raw_lidar_scans"] = encode_value(raw_lidar_scans)
        if raw_lidar_evidence is not None:
            payload["raw_lidar_evidence"] = encode_value(raw_lidar_evidence)
        payload["capture_sha256"] = payload_sha256(payload)
        return payload

    def finalize(
        self,
        status: str,
        output_path: str | Path,
        **document_options: object,
    ) -> Path:
        return write_capture(self.document(status, **document_options), output_path)


def payload_sha256(payload: Mapping[str, object]) -> str:
    unsigned = dict(payload)
    unsigned.pop("capture_sha256", None)
    try:
        encoded = json.dumps(
            unsigned,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise V3CaptureError("capture is not canonical JSON") from exc
    return hashlib.sha256(encoded).hexdigest()


def write_capture(payload: Mapping[str, object], output_path: str | Path) -> Path:
    validate_capture(payload)
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def load_capture(path_value: str | Path) -> dict[str, object]:
    path = Path(path_value)
    if path.is_symlink() or not path.is_file():
        raise V3CaptureError(f"capture is not a regular file: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V3CaptureError(f"cannot read V3 capture: {path}") from exc
    if not isinstance(payload, dict):
        raise V3CaptureError("capture root must be an object")
    validate_capture(payload)
    return payload


def validate_capture(payload: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    if payload.get("schema") != V3_CAPTURE_SCHEMA:
        raise V3CaptureError("unsupported V3 capture schema")
    if not isinstance(payload.get("capture_id"), str) or not str(
        payload.get("capture_id")
    ).strip():
        raise V3CaptureError("capture_id must be non-empty")
    if payload.get("status") not in V3_CAPTURE_STATUSES:
        raise V3CaptureError("capture status must be terminal PASS/FAIL/FAULT")
    expected_hash = payload.get("capture_sha256")
    if not isinstance(expected_hash, str) or expected_hash != payload_sha256(payload):
        raise V3CaptureError("capture checksum mismatch")
    configuration = payload.get("configuration")
    if not isinstance(configuration, Mapping):
        raise V3CaptureError("capture configuration must be an object")
    if not isinstance(payload.get("metadata"), Mapping):
        raise V3CaptureError("capture metadata must be an object")
    raw_ticks = payload.get("ticks")
    if isinstance(raw_ticks, (str, bytes)) or not isinstance(raw_ticks, Sequence):
        raise V3CaptureError("capture ticks must be an array")
    ticks: list[Mapping[str, object]] = []
    previous_tick_id: int | None = None
    previous_ns: int | None = None
    for raw_tick in raw_ticks:
        if not isinstance(raw_tick, Mapping):
            raise V3CaptureError("capture tick must be an object")
        tick_id = _non_negative_integer(raw_tick.get("tick_id"), "tick_id")
        monotonic_ns = _non_negative_integer(raw_tick.get("monotonic_ns"), "monotonic_ns")
        if previous_tick_id is not None and tick_id != previous_tick_id + 1:
            raise V3CaptureError("capture tick ids must be contiguous")
        if previous_ns is not None and monotonic_ns <= previous_ns:
            raise V3CaptureError("capture monotonic time must increase")
        inputs = raw_tick.get("inputs")
        expected = raw_tick.get("expected")
        if not isinstance(inputs, Mapping) or inputs.get("__type__") != "TickInputs":
            raise V3CaptureError(f"tick {tick_id} lacks typed TickInputs")
        if not isinstance(expected, Mapping):
            raise V3CaptureError(f"tick {tick_id} expected output must be an object")
        layers = expected.get("layers")
        if not isinstance(layers, Mapping):
            raise V3CaptureError(f"tick {tick_id} layers must be an object")
        _validate_trace_layers(
            tuple(str(layer) for layer in layers),
            expected.get("fault_layer"),
            preserve_order=False,
        )
        if expected.get("final_actuation") != layers.get("L12"):
            raise V3CaptureError(
                f"tick {tick_id} final_actuation must equal the terminal L12 output"
            )
        ticks.append(raw_tick)
        previous_tick_id = tick_id
        previous_ns = monotonic_ns
    if not ticks:
        raise V3CaptureError("capture contains no ticks")
    if payload.get("tick_count") != len(ticks):
        raise V3CaptureError("capture tick count mismatch")
    return tuple(ticks)


def inspect_capture(path_value: str | Path) -> dict[str, object]:
    path = Path(path_value)
    payload = load_capture(path)
    ticks = validate_capture(payload)
    return {
        "schema": payload["schema"],
        "capture_id": payload["capture_id"],
        "status": payload["status"],
        "execution_status": payload["status"],
        "execution_passed": payload["status"] == "PASS",
        "tick_count": len(ticks),
        "first_tick_id": ticks[0]["tick_id"],
        "last_tick_id": ticks[-1]["tick_id"],
        "first_monotonic_ns": ticks[0]["monotonic_ns"],
        "last_monotonic_ns": ticks[-1]["monotonic_ns"],
        "capture_sha256": payload["capture_sha256"],
        "path": str(path.resolve()),
    }


def _non_negative_integer(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise V3CaptureError(f"{name} must be a non-negative integer")
    return value


__all__ = [
    "CaptureSink",
    "LAYER_ORDER",
    "V3_CAPTURE_SCHEMA",
    "V3CaptureError",
    "encode_value",
    "inspect_capture",
    "load_capture",
    "payload_sha256",
    "validate_capture",
    "write_capture",
]
