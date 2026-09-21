"""Capture-specific compact IPC projection for the passive V3 observation edge.

This is not a production layer or runtime authority. It reduces the Python
object graph crossing control -> capture-process IPC. The canonical encoded
capture row is reconstructed in the capture sidecar before MCAP consumption.
"""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass

from .capture_encoding import encode_capture_record, encode_value
from .execution import CaptureRecord, EdgeFaultRecord, ExecutionRecord, WriterFailureRecord

IPC_REFERENCE_KEY = "__capture_ipc_reference__"
IPC_CHECKPOINT_KEY = "__capture_state_checkpoint_after__"
IPC_TRIGGER_REASON_KEY = "__capture_trigger_reason__"
IPC_SCHEMA = "R2B4_CAPTURE_CORE_IPC_V1"
DEFAULT_MAX_NODES = 250_000
DEFAULT_MAX_ESTIMATED_BYTES = 4 * 1024 * 1024


class CaptureIpcProjectionError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class CaptureCoreFrame:
    schema: str
    tick_id: int
    monotonic_ns: int
    row: Mapping[str, object]
    checkpoint_after: object | None
    trigger_reason: str | None
    node_count: int
    estimated_bytes: int


class CaptureCoreProjector:
    """Observation-only compactor used before multiprocessing.Queue."""

    __slots__ = ("_committed_l4_cells_key", "_max_nodes", "_max_estimated_bytes")

    def __init__(self, *, max_nodes: int = DEFAULT_MAX_NODES,
                 max_estimated_bytes: int = DEFAULT_MAX_ESTIMATED_BYTES) -> None:
        self._max_nodes = max_nodes
        self._max_estimated_bytes = max_estimated_bytes
        self._committed_l4_cells_key: tuple[object, ...] | None = None

    def project(self, record: CaptureRecord) -> CaptureCoreFrame:
        row = encode_capture_record(record)
        self._compact_l4_cells(row)
        self._compact_l7_trajectory(row)
        checkpoint_after = None
        if isinstance(record, ExecutionRecord) and record.state_checkpoint_after is not None:
            checkpoint_after = encode_value(record.state_checkpoint_after)
        trigger_reason = _trigger_reason(record)
        nodes, estimated = _measure_projection(
            (row, checkpoint_after, trigger_reason),
            max_nodes=self._max_nodes,
            max_estimated_bytes=self._max_estimated_bytes,
        )
        tick_id = int(row["tick_id"])
        monotonic_ns = int(row["monotonic_ns"])
        return CaptureCoreFrame(
            IPC_SCHEMA, tick_id, monotonic_ns, row, checkpoint_after,
            trigger_reason, nodes, estimated,
        )

    def commit(self, frame: CaptureCoreFrame) -> None:
        layers = _layers(frame.row)
        l4 = layers.get("L4")
        if not isinstance(l4, Mapping):
            return
        costmap = l4.get("local_costmap")
        if not isinstance(costmap, Mapping):
            return
        cells = costmap.get("occupied_cells")
        if _is_reference(cells, "L4_OCCUPIED_CELLS"):
            return
        self._committed_l4_cells_key = _costmap_key(costmap)

    def _compact_l4_cells(self, row: dict[str, object]) -> None:
        layers = _layers(row)
        l4 = layers.get("L4")
        if not isinstance(l4, dict):
            return
        costmap = l4.get("local_costmap")
        if not isinstance(costmap, dict):
            return
        key = _costmap_key(costmap)
        if self._committed_l4_cells_key == key:
            costmap["occupied_cells"] = {
                IPC_REFERENCE_KEY: "L4_OCCUPIED_CELLS",
                "key": list(key),
            }

    def _compact_l7_trajectory(self, row: dict[str, object]) -> None:
        layers = _layers(row)
        l6 = layers.get("L6")
        l7 = layers.get("L7")
        if not isinstance(l6, dict) or not isinstance(l7, dict):
            return
        trajectory = l7.get("trajectory")
        candidates = l6.get("trajectory_candidates")
        if not isinstance(trajectory, dict) or not isinstance(candidates, list):
            return
        candidate_id = trajectory.get("candidate_id")
        for candidate in candidates:
            if isinstance(candidate, dict) and candidate.get("candidate_id") == candidate_id and candidate == trajectory:
                l7["trajectory"] = {
                    IPC_REFERENCE_KEY: "L6_TRAJECTORY_CANDIDATE",
                    "candidate_id": candidate_id,
                }
                return


class CaptureCoreExpander:
    """Reconstruct canonical encoded rows in the capture sidecar."""

    __slots__ = ("_l4_cells",)

    def __init__(self) -> None:
        self._l4_cells: dict[tuple[object, ...], object] = {}

    def expand(self, frame: CaptureCoreFrame) -> dict[str, object]:
        row = deepcopy(dict(frame.row))
        self._expand_l4_cells(row)
        self._expand_l7_trajectory(row)
        return row

    def expand_transport(self, frame: CaptureCoreFrame) -> dict[str, object]:
        row = self.expand(frame)
        if frame.checkpoint_after is not None:
            row[IPC_CHECKPOINT_KEY] = deepcopy(frame.checkpoint_after)
        if frame.trigger_reason is not None:
            row[IPC_TRIGGER_REASON_KEY] = frame.trigger_reason
        return row

    def _expand_l4_cells(self, row: dict[str, object]) -> None:
        layers = _layers(row)
        l4 = layers.get("L4")
        if not isinstance(l4, dict):
            return
        costmap = l4.get("local_costmap")
        if not isinstance(costmap, dict):
            return
        cells = costmap.get("occupied_cells")
        key = _costmap_key(costmap)
        if _is_reference(cells, "L4_OCCUPIED_CELLS"):
            ref_value = cells.get("key") if isinstance(cells, dict) else None
            ref_key = tuple(ref_value) if isinstance(ref_value, list) else key
            costmap["occupied_cells"] = deepcopy(self._l4_cells[ref_key])
        else:
            self._l4_cells[key] = deepcopy(cells)

    def _expand_l7_trajectory(self, row: dict[str, object]) -> None:
        layers = _layers(row)
        l6 = layers.get("L6")
        l7 = layers.get("L7")
        if not isinstance(l6, dict) or not isinstance(l7, dict):
            return
        trajectory = l7.get("trajectory")
        if not _is_reference(trajectory, "L6_TRAJECTORY_CANDIDATE"):
            return
        candidate_id = trajectory.get("candidate_id") if isinstance(trajectory, dict) else None
        candidates = l6.get("trajectory_candidates")
        if not isinstance(candidates, list):
            return
        for candidate in candidates:
            if isinstance(candidate, dict) and candidate.get("candidate_id") == candidate_id:
                l7["trajectory"] = deepcopy(candidate)
                return


def _trigger_reason(record: CaptureRecord) -> str | None:
    if isinstance(record, (EdgeFaultRecord, WriterFailureRecord)):
        return record.reason
    if not isinstance(record, ExecutionRecord):
        return None
    result = record.result
    if result.trace.fault_layer is None and result.final_actuation.safety_decision.value != "FAULT":
        return None
    return result.final_actuation.reason or result.trace.fault_layer or "FAULT"


def _costmap_key(costmap: Mapping[str, object]) -> tuple[object, ...]:
    return (costmap.get("frame_id"), costmap.get("revision"), costmap.get("source_sequence"))


def _layers(row: Mapping[str, object]) -> dict[str, object]:
    expected = row.get("expected")
    if not isinstance(expected, dict):
        return {}
    layers = expected.get("layers")
    return layers if isinstance(layers, dict) else {}


def _is_reference(value: object, kind: str) -> bool:
    return isinstance(value, dict) and value.get(IPC_REFERENCE_KEY) == kind


def _measure_projection(value: object, *, max_nodes: int, max_estimated_bytes: int) -> tuple[int, int]:
    nodes = 0
    estimated = 0
    stack = [value]
    while stack:
        current = stack.pop()
        nodes += 1
        if nodes > max_nodes:
            raise CaptureIpcProjectionError("capture core IPC node bound exceeded")
        if current is None or isinstance(current, bool):
            estimated += 1
        elif isinstance(current, int):
            estimated += 8
        elif isinstance(current, float):
            estimated += 8
        elif isinstance(current, str):
            estimated += len(current.encode("utf-8")) + 8
        elif isinstance(current, Mapping):
            estimated += 32 + 16 * len(current)
            for key, item in current.items():
                stack.append(str(key))
                stack.append(item)
        elif isinstance(current, (tuple, list)):
            estimated += 24 + 8 * len(current)
            stack.extend(current)
        else:
            raise CaptureIpcProjectionError(
                f"capture IPC projection contains unsupported {type(current).__name__}"
            )
        if estimated > max_estimated_bytes:
            raise CaptureIpcProjectionError("capture core IPC estimated byte bound exceeded")
    return nodes, estimated


__all__ = [
    "CaptureCoreExpander", "CaptureCoreFrame", "CaptureCoreProjector",
    "CaptureIpcProjectionError", "DEFAULT_MAX_ESTIMATED_BYTES", "DEFAULT_MAX_NODES",
    "IPC_CHECKPOINT_KEY", "IPC_REFERENCE_KEY", "IPC_SCHEMA", "IPC_TRIGGER_REASON_KEY",
]
