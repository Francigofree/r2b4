"""Capture-specific compact IPC projection for the passive V3 observation edge.

This is not a production layer or runtime authority. It reduces the Python
object graph crossing control -> capture-process IPC. The canonical encoded
capture row is reconstructed in the capture sidecar before MCAP consumption.
"""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass

from .capture_encoding import encode_capture_record
from .execution import CaptureRecord

IPC_REFERENCE_KEY = "__capture_ipc_reference__"
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
    node_count: int
    estimated_bytes: int


class CaptureCoreProjector:
    """Observation-only compactor used before multiprocessing.Queue."""

    __slots__ = ("_last_l4_cells_key", "_max_nodes", "_max_estimated_bytes")

    def __init__(
        self,
        *,
        max_nodes: int = DEFAULT_MAX_NODES,
        max_estimated_bytes: int = DEFAULT_MAX_ESTIMATED_BYTES,
    ) -> None:
        if not isinstance(max_nodes, int) or isinstance(max_nodes, bool) or max_nodes <= 0:
            raise ValueError("max_nodes must be a positive integer")
        if (
            not isinstance(max_estimated_bytes, int)
            or isinstance(max_estimated_bytes, bool)
            or max_estimated_bytes <= 0
        ):
            raise ValueError("max_estimated_bytes must be a positive integer")
        self._max_nodes = max_nodes
        self._max_estimated_bytes = max_estimated_bytes
        self._last_l4_cells_key: tuple[object, ...] | None = None

    def project(self, record: CaptureRecord) -> CaptureCoreFrame:
        row = encode_capture_record(record)
        self._compact_l4_cells(row)
        self._compact_l7_trajectory(row)
        nodes, estimated = _measure_projection(
            row,
            max_nodes=self._max_nodes,
            max_estimated_bytes=self._max_estimated_bytes,
        )
        tick_id = row.get("tick_id")
        monotonic_ns = row.get("monotonic_ns")
        if (
            not isinstance(tick_id, int)
            or isinstance(tick_id, bool)
            or tick_id < 0
            or not isinstance(monotonic_ns, int)
            or isinstance(monotonic_ns, bool)
            or monotonic_ns < 0
        ):
            raise CaptureIpcProjectionError("projected capture row lacks valid tick identity")
        return CaptureCoreFrame(IPC_SCHEMA, tick_id, monotonic_ns, row, nodes, estimated)

    def _compact_l4_cells(self, row: dict[str, object]) -> None:
        layers = _layers(row)
        l4 = layers.get("L4")
        if not isinstance(l4, dict):
            return
        costmap = l4.get("local_costmap")
        if not isinstance(costmap, dict):
            return
        key = (costmap.get("frame_id"), costmap.get("revision"), costmap.get("source_sequence"))
        if self._last_l4_cells_key == key:
            costmap["occupied_cells"] = {
                IPC_REFERENCE_KEY: "L4_OCCUPIED_CELLS",
                "key": list(key),
            }
        else:
            self._last_l4_cells_key = key

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
            if (
                isinstance(candidate, dict)
                and candidate.get("candidate_id") == candidate_id
                and candidate == trajectory
            ):
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
        if not isinstance(frame, CaptureCoreFrame) or frame.schema != IPC_SCHEMA:
            raise CaptureIpcProjectionError("invalid capture core IPC frame")
        row = deepcopy(dict(frame.row))
        self._expand_l4_cells(row)
        self._expand_l7_trajectory(row)
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
        key = (costmap.get("frame_id"), costmap.get("revision"), costmap.get("source_sequence"))
        if _is_reference(cells, "L4_OCCUPIED_CELLS"):
            ref_value = cells.get("key") if isinstance(cells, dict) else None
            ref_key = tuple(ref_value) if isinstance(ref_value, list) else key
            if ref_key not in self._l4_cells:
                raise CaptureIpcProjectionError("L4 occupied-cell reference has no sidecar cache entry")
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
            raise CaptureIpcProjectionError("L7 trajectory reference lacks L6 candidates")
        for candidate in candidates:
            if isinstance(candidate, dict) and candidate.get("candidate_id") == candidate_id:
                l7["trajectory"] = deepcopy(candidate)
                return
        raise CaptureIpcProjectionError("L7 trajectory reference target is absent")


def _layers(row: Mapping[str, object]) -> dict[str, object]:
    expected = row.get("expected")
    if not isinstance(expected, dict):
        return {}
    layers = expected.get("layers")
    return layers if isinstance(layers, dict) else {}


def _is_reference(value: object, kind: str) -> bool:
    return isinstance(value, dict) and value.get(IPC_REFERENCE_KEY) == kind


def _measure_projection(
    value: object,
    *,
    max_nodes: int,
    max_estimated_bytes: int,
) -> tuple[int, int]:
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
    "CaptureCoreExpander",
    "CaptureCoreFrame",
    "CaptureCoreProjector",
    "CaptureIpcProjectionError",
    "DEFAULT_MAX_ESTIMATED_BYTES",
    "DEFAULT_MAX_NODES",
    "IPC_REFERENCE_KEY",
    "IPC_SCHEMA",
]
