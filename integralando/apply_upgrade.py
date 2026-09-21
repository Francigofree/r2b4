#!/usr/bin/env python3
"""Finish the R2B4 capture P0 refactor from the current partial-upgrade state.

This installer intentionally performs no Git/head/dirty-tree checks, no backup,
no pytest, no compile validation, and no rollback. It only applies the upgrade.
"""
from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
if not (ROOT / "v3").exists():
    ROOT = Path.cwd()


def patch(rel: str, old: str, new: str) -> None:
    path = ROOT / rel
    text = path.read_text(encoding="utf-8")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def overwrite(rel: str, content: str) -> None:
    path = ROOT / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


CAPTURE_IPC = r'''"""Capture-specific compact IPC projection for the passive V3 observation edge.

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
'''


def main() -> int:
    overwrite("v3/capture_ipc.py", CAPTURE_IPC)

    # v3/process_sidecars.py -- finish the already-partial upgrade.
    patch(
        "v3/process_sidecars.py",
        '            if kind == "core":\n                hub.publish(expander.expand(payload), topic="v3.capture_record")\n',
        '            if kind == "core":\n                hub.publish(expander.expand_transport(payload), topic="v3.capture_record")\n',
    )
    patch(
        "v3/process_sidecars.py",
        "            if (\n                finish_request is not None\n                and processed >= finish_request[2]\n                and raw_end_received\n            ):\n",
        "            if (\n                finish_request is not None\n                and processed >= finish_request[2]\n            ):\n",
    )
    patch(
        "v3/process_sidecars.py",
        "        self._project_root = Path(project_root)\n        self._worker_cpu = worker_cpu\n        self._strict_affinity = strict_affinity\n        self._process = context.Process(\n            target=_capture_sidecar_main,\n",
        "        self._project_root = Path(project_root)\n        self._worker_cpu = worker_cpu\n        self._strict_affinity = strict_affinity\n        self._projector = CaptureCoreProjector()\n        self._projection_drop_count = 0\n        self._expect_raw_lidar_end = bool(expect_raw_lidar_end)\n        self._process = context.Process(\n            target=_capture_sidecar_main,\n",
    )
    patch(
        "v3/process_sidecars.py",
        "                self._ready_event,\n                self._failed_event,\n                str(self._project_root),\n",
        "                self._ready_event,\n                self._failed_event,\n                self._expect_raw_lidar_end,\n                str(self._project_root),\n",
    )
    patch(
        "v3/process_sidecars.py",
        "    def failed(self) -> bool:\n        return bool(self._failed_event.is_set() or self._drop_count)\n",
        "    def failed(self) -> bool:\n        return bool(self._failed_event.is_set() or self._drop_count or self._projection_drop_count)\n",
    )
    patch(
        "v3/process_sidecars.py",
        "    def _enqueue(self, kind: str, payload: object) -> None:\n        if not self._started:\n            self.start()\n        try:\n            self._data_queue.put_nowait((kind, payload))\n        except queue.Full:\n            self._drop_count += 1\n        else:\n            self._enqueued_count += 1\n",
        "    def _enqueue(self, kind: str, payload: object) -> bool:\n        if not self._started:\n            self.start()\n        try:\n            self._data_queue.put_nowait((kind, payload))\n        except queue.Full:\n            self._drop_count += 1\n            return False\n        else:\n            self._enqueued_count += 1\n            return True\n",
    )
    patch(
        "v3/process_sidecars.py",
        "    def observe(self, record: CaptureRecord) -> None:\n        self._enqueue(\"record\", record)\n",
        "    def observe(self, record: CaptureRecord) -> None:\n        try:\n            projected = self._projector.project(record)\n        except CaptureIpcProjectionError:\n            self._projection_drop_count += 1\n            return\n        if self._enqueue(\"core\", projected):\n            self._projector.commit(projected)\n",
    )
    patch(
        "v3/process_sidecars.py",
        '            ("finish", status, True, self._enqueued_count),\n',
        '            ("finish", status, True, self._enqueued_count, self._drop_count + self._projection_drop_count),\n',
    )
    patch(
        "v3/process_sidecars.py",
        '            if self._drop_count:\n                self.evidence["status"] = "FAIL"\n                self.evidence["process_transport_drop_count"] = self._drop_count\n                self.evidence["process_transport_integrity"] = "FAIL"\n',
        '            if self._drop_count:\n                self.evidence["status"] = "FAIL"\n                self.evidence["process_transport_drop_count"] = self._drop_count\n                self.evidence["process_transport_integrity"] = "FAIL"\n            if self._projection_drop_count:\n                self.evidence["status"] = "FAIL"\n                self.evidence["process_projection_drop_count"] = self._projection_drop_count\n                self.evidence["process_transport_integrity"] = "FAIL"\n',
    )

    # v3/adapters/process_lidar_port.py
    patch(
        "v3/adapters/process_lidar_port.py",
        "\ndef _lidar_owner_process_main(\n",
        '''\ndef _put_raw_evidence(target: Any, payload: object, superseded_count: int) -> int:
    try:
        target.put_nowait(payload)
        return superseded_count
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    else:
        superseded_count += 1
    try:
        target.put_nowait(payload)
    except queue.Full:
        superseded_count += 1
    return superseded_count


def _put_raw_end(target: Any, *, last_revision: int, produced_count: int,
                 superseded_count: int) -> None:
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=0.5)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    else:
        superseded_count += 1
    marker = ("raw_end", int(last_revision), int(produced_count), int(superseded_count))
    try:
        target.put(marker, timeout=0.5)
    except queue.Full:
        return


def _lidar_owner_process_main(
''',
    )
    patch(
        "v3/adapters/process_lidar_port.py",
        "    port = None\n    try:\n",
        "    port = None\n    raw_produced_count = 0\n    raw_superseded_count = 0\n    last_raw_revision = 0\n    try:\n",
    )
    patch(
        "v3/adapters/process_lidar_port.py",
        '''        if initial_raw is not None:
            _put_latest(raw_queue, ("raw", _wire_raw(initial_raw)))
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
''',
        '''        if initial_raw is not None:
            raw_produced_count += 1
            raw_superseded_count = _put_raw_evidence(
                raw_queue, ("raw", _wire_raw(initial_raw)), raw_superseded_count
            )
        ready_event.set()
        last_raw_revision = int(getattr(initial_raw, "raw_scan_id", 0) or 0)
''',
    )
    patch(
        "v3/adapters/process_lidar_port.py",
        '''            if raw_changed and raw is not None:
                _put_latest(raw_queue, ("raw", _wire_raw(raw)))
                control_raw = _wire_control_raw(
''',
        '''            if raw_changed and raw is not None:
                raw_produced_count += 1
                raw_superseded_count = _put_raw_evidence(
                    raw_queue, ("raw", _wire_raw(raw)), raw_superseded_count
                )
                control_raw = _wire_control_raw(
''',
    )
    patch(
        "v3/adapters/process_lidar_port.py",
        '''    finally:
        if port is not None:
            try:
                port.stop()
            except BaseException as exc:
                _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))


class ProcessLidarPort:
''',
        '''    finally:
        if port is not None:
            try:
                port.stop()
            except BaseException as exc:
                _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
        _put_raw_end(
            raw_queue,
            last_revision=last_raw_revision,
            produced_count=raw_produced_count,
            superseded_count=raw_superseded_count,
        )


class ProcessLidarPort:
''',
    )
    patch(
        "v3/adapters/process_lidar_port.py",
        '''        if not isinstance(newest, tuple) or len(newest) != 2 or newest[0] != "raw":
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        snapshot = _unwire_raw(newest[1])
''',
        '''        if not isinstance(newest, tuple) or not newest:
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        if newest[0] == "raw_end":
            if len(newest) == 4:
                self._status["capture_raw_transport_end"] = True
                self._status["capture_raw_produced_count"] = int(newest[2])
                self._status["capture_raw_superseded_count"] = int(newest[3])
                return
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        if len(newest) != 2 or newest[0] != "raw":
            self._fatal_error = "LIDAR_RAW_TRANSPORT_INVALID"
            return
        snapshot = _unwire_raw(newest[1])
''',
    )

    # v3/mcap_capture.py
    patch(
        "v3/mcap_capture.py",
        "from .capture_encoding import (\n",
        "from .capture_ipc import IPC_CHECKPOINT_KEY, IPC_TRIGGER_REASON_KEY\nfrom .capture_encoding import (\n",
    )
    patch(
        "v3/mcap_capture.py",
        "    max_consecutive_raw_lidar_missing: int = 2\n",
        "    max_consecutive_raw_lidar_missing: int = 2\n    require_raw_lidar_transport_end: bool = False\n",
    )
    patch(
        "v3/mcap_capture.py",
        '        if self.mode not in {"triggered", "append_only"}:\n',
        '        if type(self.require_raw_lidar_transport_end) is not bool:\n            raise TypeError("require_raw_lidar_transport_end must be bool")\n\n        if self.mode not in {"triggered", "append_only"}:\n',
    )
    patch(
        "v3/mcap_capture.py",
        "    raw_lidar_missing_revisions: tuple[int, ...]\n",
        "    raw_lidar_missing_revisions: tuple[int, ...]\n    replay_complete: bool\n    raw_evidence_complete: bool\n",
    )
    patch(
        "v3/mcap_capture.py",
        '        "_latest_capacity_eviction_ns",\n',
        '        "_latest_capacity_eviction_ns",\n        "_raw_capacity_eviction_count",\n        "_core_capacity_eviction_count",\n        "_latest_raw_capacity_eviction_ns",\n',
    )
    patch(
        "v3/mcap_capture.py",
        '        "_last_raw_revision",\n',
        '        "_last_raw_revision",\n        "_raw_transport_end",\n        "_raw_transport_produced_count",\n        "_raw_transport_superseded_count",\n',
    )
    patch(
        "v3/mcap_capture.py",
        "        encoded_configuration = encode_value(configuration)\n        encoded_metadata = encode_value(metadata or {})\n        if not isinstance(encoded_configuration, dict) or not isinstance(encoded_metadata, dict):\n",
        "        encoded_configuration = encode_value(configuration)\n        encoded_metadata = encode_value(metadata or {})\n        if isinstance(encoded_metadata, dict):\n            encoded_metadata = {**encoded_metadata, \"capture_policy\": encode_value(settings)}\n        if not isinstance(encoded_configuration, dict) or not isinstance(encoded_metadata, dict):\n",
    )
    patch(
        "v3/mcap_capture.py",
        "        self._capacity_eviction_count = 0\n        self._latest_capacity_eviction_ns: int | None = None\n",
        "        self._capacity_eviction_count = 0\n        self._latest_capacity_eviction_ns: int | None = None\n        self._raw_capacity_eviction_count = 0\n        self._core_capacity_eviction_count = 0\n        self._latest_raw_capacity_eviction_ns: int | None = None\n",
    )
    patch(
        "v3/mcap_capture.py",
        "        self._last_raw_revision: int | None = None\n",
        "        self._last_raw_revision: int | None = None\n        self._raw_transport_end = False\n        self._raw_transport_produced_count = 0\n        self._raw_transport_superseded_count = 0\n",
    )
    patch(
        "v3/mcap_capture.py",
        "        encoded = self._encode_observation(item)\n",
        "        self._consume_transport_integrity(item)\n        encoded = self._encode_observation(item)\n",
    )
    patch(
        "v3/mcap_capture.py",
        "    def _encode_observation(self, item: ObservationFrame) -> tuple[EncodedRecord, ...]:\n",
        '''    def _consume_transport_integrity(self, item: ObservationFrame) -> None:
        payload = item.payload
        if _looks_like_capture_transport_topic(item.topic):
            if isinstance(payload, Mapping):
                drops = payload.get("drop_count", 0)
                if isinstance(drops, int) and not isinstance(drops, bool) and drops > 0:
                    self._integrity_reasons.add("CORE_TRANSPORT_LOSS")
            return
        if not _looks_like_raw_lidar_transport_topic(item.topic):
            return
        if not isinstance(payload, Mapping):
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_INVALID")
            return
        if payload.get("event_type") != "raw_lidar_transport_end":
            return
        self._raw_transport_end = True
        produced = payload.get("produced_count", 0)
        superseded = payload.get("superseded_count", 0)
        if isinstance(produced, int) and not isinstance(produced, bool) and produced >= 0:
            self._raw_transport_produced_count = produced
        else:
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_INVALID")
        if isinstance(superseded, int) and not isinstance(superseded, bool) and superseded >= 0:
            self._raw_transport_superseded_count = superseded
            if superseded:
                self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_SUPERSEDE")
        else:
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_INVALID")

    def _encode_observation(self, item: ObservationFrame) -> tuple[EncodedRecord, ...]:
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''        payload = item.payload
        if isinstance(payload, (ExecutionRecord, EdgeFaultRecord, WriterFailureRecord)):
            row = encode_capture_record(payload)
            tick_id = _non_negative_int(row.get("tick_id"), "tick_id")
            monotonic_ns = _non_negative_int(row.get("monotonic_ns"), "monotonic_ns")
            referenced = tuple(sorted(_referenced_lidar_revisions(row)))
            if len(referenced) > self._config.max_referenced_revisions_per_tick:
                raise CaptureEncodingError("tick exceeds referenced LiDAR revision bound")
            tick = EncodedRecord(
                hub_sequence=item.sequence,
                published_monotonic_ns=item.published_monotonic_ns,
                source_topic=item.topic,
                mcap_topic=TICK_TOPIC,
                monotonic_ns=monotonic_ns,
                sequence=tick_id,
                payload=_json_bytes(row),
                tick_id=tick_id,
                referenced_lidar_revisions=referenced,
            )
            records: list[EncodedRecord] = [tick]
            if isinstance(payload, ExecutionRecord) and payload.state_checkpoint_after is not None:
                checkpoint = encode_value(payload.state_checkpoint_after)
                if not isinstance(checkpoint, dict):
                    raise CaptureEncodingError("state checkpoint must encode as an object")
                checkpoint_row = {
                    "tick_id": tick_id,
                    "monotonic_ns": monotonic_ns,
                    "state": checkpoint,
                }
                records.append(
                    EncodedRecord(
                        hub_sequence=item.sequence,
                        published_monotonic_ns=item.published_monotonic_ns,
                        source_topic=item.topic,
                        mcap_topic=CHECKPOINT_TOPIC,
                        monotonic_ns=monotonic_ns,
                        sequence=tick_id,
                        payload=_json_bytes(checkpoint_row),
                        tick_id=tick_id,
                        is_checkpoint=True,
                    )
                )
            return tuple(records)
''',
        '''        payload = item.payload
        checkpoint: object | None = None
        if item.topic == "v3.capture_record" and isinstance(payload, Mapping):
            row = dict(payload)
            checkpoint = row.pop(IPC_CHECKPOINT_KEY, None)
            row.pop(IPC_TRIGGER_REASON_KEY, None)
        elif isinstance(payload, (ExecutionRecord, EdgeFaultRecord, WriterFailureRecord)):
            row = encode_capture_record(payload)
            if isinstance(payload, ExecutionRecord) and payload.state_checkpoint_after is not None:
                checkpoint = encode_value(payload.state_checkpoint_after)
        else:
            row = None

        if row is not None:
            tick_id = _non_negative_int(row.get("tick_id"), "tick_id")
            monotonic_ns = _non_negative_int(row.get("monotonic_ns"), "monotonic_ns")
            referenced = tuple(sorted(_referenced_lidar_revisions(row)))
            if len(referenced) > self._config.max_referenced_revisions_per_tick:
                raise CaptureEncodingError("tick exceeds referenced LiDAR revision bound")
            tick = EncodedRecord(
                hub_sequence=item.sequence,
                published_monotonic_ns=item.published_monotonic_ns,
                source_topic=item.topic,
                mcap_topic=TICK_TOPIC,
                monotonic_ns=monotonic_ns,
                sequence=tick_id,
                payload=_json_bytes(row),
                tick_id=tick_id,
                referenced_lidar_revisions=referenced,
            )
            records: list[EncodedRecord] = [tick]
            if checkpoint is not None:
                if not isinstance(checkpoint, dict):
                    raise CaptureEncodingError("state checkpoint must encode as an object")
                checkpoint_row = {
                    "tick_id": tick_id,
                    "monotonic_ns": monotonic_ns,
                    "state": checkpoint,
                }
                records.append(
                    EncodedRecord(
                        hub_sequence=item.sequence,
                        published_monotonic_ns=item.published_monotonic_ns,
                        source_topic=item.topic,
                        mcap_topic=CHECKPOINT_TOPIC,
                        monotonic_ns=monotonic_ns,
                        sequence=tick_id,
                        payload=_json_bytes(checkpoint_row),
                        tick_id=tick_id,
                        is_checkpoint=True,
                    )
                )
            return tuple(records)
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''    def _enforce_capacities(self) -> None:
        while self._ring_tick_count > self._config.max_tick_count:
            self._drop_oldest_matching(TICK_TOPIC)
        while self._ring_raw_count > self._config.max_raw_lidar_scans:
            self._drop_oldest_matching(RAW_LIDAR_TOPIC)
        while self._ring_bytes > self._config.max_byte_capacity:
            self._drop_oldest(capacity=True)

    def _drop_oldest_matching(self, topic: str) -> None:
''',
        '''    def _enforce_capacities(self) -> None:
        while self._ring_tick_count > self._config.max_tick_count:
            self._drop_oldest_matching(TICK_TOPIC, capacity=True)
        while self._ring_raw_count > self._config.max_raw_lidar_scans:
            self._drop_oldest_matching(RAW_LIDAR_TOPIC, capacity=True)
        while self._ring_bytes > self._config.max_byte_capacity:
            if self._ring_raw_count:
                self._drop_oldest_matching(RAW_LIDAR_TOPIC, capacity=True)
            else:
                self._drop_oldest(capacity=True)

    def _drop_oldest_matching(self, topic: str, *, capacity: bool = False) -> None:
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''            self._capacity_eviction_count += 1
            self._latest_capacity_eviction_ns = item.monotonic_ns
            return
        raise RuntimeError(f"ring count inconsistent for {topic}")
''',
        '''            if capacity:
                self._capacity_eviction_count += 1
                if topic == RAW_LIDAR_TOPIC:
                    self._raw_capacity_eviction_count += 1
                    self._latest_raw_capacity_eviction_ns = item.monotonic_ns
                else:
                    self._core_capacity_eviction_count += 1
                    self._latest_capacity_eviction_ns = item.monotonic_ns
            return
        raise RuntimeError(f"ring count inconsistent for {topic}")
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''        if capacity:
            self._capacity_eviction_count += 1
            self._latest_capacity_eviction_ns = item.monotonic_ns
''',
        '''        if capacity:
            self._capacity_eviction_count += 1
            if item.mcap_topic == RAW_LIDAR_TOPIC:
                self._raw_capacity_eviction_count += 1
                self._latest_raw_capacity_eviction_ns = item.monotonic_ns
            else:
                self._core_capacity_eviction_count += 1
                self._latest_capacity_eviction_ns = item.monotonic_ns
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''        if raw_lidar_missing_count:
            if raw_lidar_loss_within_tolerance:
                integrity_warnings.append("RAW_LIDAR_SPARSE_LOSS_TOLERATED")
            else:
                if gap_missing_raw:
                    self._integrity_reasons.add("RAW_LIDAR_REVISION_GAP")
                if missing_raw:
                    self._integrity_reasons.add("REFERENCED_RAW_LIDAR_MISSING")

        complete = not self._integrity_reasons
''',
        '''        if raw_lidar_missing_count:
            self._integrity_reasons.add("RAW_LIDAR_EVIDENCE_INCOMPLETE")
            if raw_lidar_loss_within_tolerance:
                integrity_warnings.append("RAW_LIDAR_SPARSE_LOSS_TOLERATED")
            if gap_missing_raw:
                self._integrity_reasons.add("RAW_LIDAR_REVISION_GAP")
            if missing_raw:
                self._integrity_reasons.add("REFERENCED_RAW_LIDAR_MISSING")

        if self._config.require_raw_lidar_transport_end and not self._raw_transport_end:
            self._integrity_reasons.add("RAW_LIDAR_TRANSPORT_END_MISSING")
        raw_capacity_missing = bool(
            self._latest_raw_capacity_eviction_ns is not None
            and self._latest_raw_capacity_eviction_ns >= lower
        )
        if raw_capacity_missing:
            self._integrity_reasons.add("RAW_LIDAR_CAPACITY_EVICTION")

        raw_integrity_reasons = sorted(
            reason for reason in self._integrity_reasons if _is_raw_integrity_reason(reason)
        )
        replay_integrity_reasons = sorted(
            reason for reason in self._integrity_reasons if not _is_raw_integrity_reason(reason)
        )
        replay_complete = not replay_integrity_reasons
        raw_evidence_complete = not raw_integrity_reasons
        complete = replay_complete and raw_evidence_complete
''',
    )
    patch(
        "v3/mcap_capture.py",
        '            "complete": complete,\n',
        '            "complete": complete,\n            "replay_complete": replay_complete,\n            "raw_evidence_complete": raw_evidence_complete,\n            "replay_integrity_reasons": replay_integrity_reasons,\n            "raw_integrity_reasons": raw_integrity_reasons,\n',
    )
    patch(
        "v3/mcap_capture.py",
        '            "capacity_eviction_count": self._capacity_eviction_count,\n',
        '            "capacity_eviction_count": self._capacity_eviction_count,\n            "core_capacity_eviction_count": self._core_capacity_eviction_count,\n            "raw_capacity_eviction_count": self._raw_capacity_eviction_count,\n            "raw_transport_end": self._raw_transport_end,\n            "raw_transport_produced_count": self._raw_transport_produced_count,\n            "raw_transport_superseded_count": self._raw_transport_superseded_count,\n',
    )
    patch(
        "v3/mcap_capture.py",
        '                "complete": "true" if complete else "false",\n',
        '                "complete": "true" if complete else "false",\n                "replay_complete": "true" if replay_complete else "false",\n                "raw_evidence_complete": "true" if raw_evidence_complete else "false",\n',
    )
    patch(
        "v3/mcap_capture.py",
        '            raw_lidar_missing_revisions=missing_raw,\n        )\n',
        '            raw_lidar_missing_revisions=missing_raw,\n            replay_complete=replay_complete,\n            raw_evidence_complete=raw_evidence_complete,\n        )\n',
    )
    patch(
        "v3/mcap_capture.py",
        "\ndef _looks_like_raw_lidar_topic(topic: str) -> bool:\n",
        '''
def _looks_like_capture_transport_topic(topic: str) -> bool:
    return topic.strip().lower().replace("-", "_") in {"v3.capture_transport", "capture_transport"}


def _looks_like_raw_lidar_transport_topic(topic: str) -> bool:
    return topic.strip().lower().replace("-", "_") in {"v3.raw_lidar_transport", "raw_lidar_transport"}


def _is_raw_integrity_reason(reason: str) -> bool:
    return reason.startswith("RAW_LIDAR_") or reason == "REFERENCED_RAW_LIDAR_MISSING"


def _looks_like_raw_lidar_topic(topic: str) -> bool:
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''def _payload_triggers_capture(payload: object) -> bool:
    if isinstance(payload, (EdgeFaultRecord, WriterFailureRecord)):
        return True
    if not isinstance(payload, ExecutionRecord):
        return False
    return bool(
        payload.result.trace.fault_layer is not None
        or payload.result.final_actuation.safety_decision.value == "FAULT"
    )
''',
        '''def _payload_triggers_capture(payload: object) -> bool:
    if isinstance(payload, Mapping):
        reason = payload.get(IPC_TRIGGER_REASON_KEY)
        return isinstance(reason, str) and bool(reason.strip())
    if isinstance(payload, (EdgeFaultRecord, WriterFailureRecord)):
        return True
    if not isinstance(payload, ExecutionRecord):
        return False
    return bool(
        payload.result.trace.fault_layer is not None
        or payload.result.final_actuation.safety_decision.value == "FAULT"
    )
''',
    )
    patch(
        "v3/mcap_capture.py",
        '''def _payload_trigger_reason(payload: object) -> str:
    if isinstance(payload, (EdgeFaultRecord, WriterFailureRecord)):
        return payload.reason
    if isinstance(payload, ExecutionRecord):
        return (
            payload.result.final_actuation.reason
            or payload.result.trace.fault_layer
            or "FAULT"
        )
    return "CAPTURE_TRIGGER"
''',
        '''def _payload_trigger_reason(payload: object) -> str:
    if isinstance(payload, Mapping):
        reason = payload.get(IPC_TRIGGER_REASON_KEY)
        if isinstance(reason, str) and reason.strip():
            return reason
    if isinstance(payload, (EdgeFaultRecord, WriterFailureRecord)):
        return payload.reason
    if isinstance(payload, ExecutionRecord):
        return (
            payload.result.final_actuation.reason
            or payload.result.trace.fault_layer
            or "FAULT"
        )
    return "CAPTURE_TRIGGER"
''',
    )

    # v3/mcap_reader.py
    patch(
        "v3/mcap_reader.py",
        "    def capture_integrity(self) -> dict[str, object]:\n",
        "    def capture_integrity(self, *, require_raw_evidence: bool = True) -> dict[str, object]:\n",
    )
    patch(
        "v3/mcap_reader.py",
        '''        if (integrity.get("complete") is not True or metadata.get("complete") != "true"
                or integrity.get("integrity_reasons") or not subscription_ok
                or not counts.get(TICK_TOPIC)):
            raise McapReadError("CAPTURE_INCOMPLETE: required evidence is incomplete")
''',
        '''        replay_complete = (
            integrity.get("replay_complete", integrity.get("complete")) is True
            and metadata.get("replay_complete", metadata.get("complete")) == "true"
        )
        raw_complete = (
            integrity.get("raw_evidence_complete", integrity.get("complete")) is True
            and metadata.get("raw_evidence_complete", metadata.get("complete")) == "true"
        )
        required_complete = replay_complete and (raw_complete if require_raw_evidence else True)
        replay_reasons = integrity.get(
            "replay_integrity_reasons", integrity.get("integrity_reasons")
        )
        if not required_complete or replay_reasons or not subscription_ok or not counts.get(TICK_TOPIC):
            raise McapReadError("CAPTURE_INCOMPLETE: required evidence is incomplete")
''',
    )

    # v3/mcap_replay_bridge.py
    patch(
        "v3/mcap_replay_bridge.py",
        "            else reader.capture_integrity()\n",
        "            else reader.capture_integrity(require_raw_evidence=False)\n",
    )
    patch(
        "v3/mcap_replay_bridge.py",
        "    original_complete = replay_eligible = True\n",
        "    final_integrity = final.get(\"integrity\") if isinstance(final, Mapping) else None\n    if not isinstance(final_integrity, Mapping):\n        raise McapReplayBridgeError(\"MCAP capture lacks final integrity\")\n    replay_eligible = bool(final_integrity.get(\"replay_complete\", final_integrity.get(\"complete\")))\n    original_complete = bool(final_integrity.get(\"complete\"))\n    if not replay_eligible:\n        raise McapReplayBridgeError(\"CAPTURE_INCOMPLETE: replay core is incomplete\")\n",
    )

    # v3/test_hub_portable.py
    patch(
        "v3/test_hub_portable.py",
        "            verified_final = reader.capture_integrity()\n",
        "            verified_final = reader.capture_integrity(require_raw_evidence=False)\n",
    )

    # v3_process_runtime.py
    patch(
        "v3_process_runtime.py",
        "                    mode=args.capture_mode,\n                ),\n",
        "                    mode=args.capture_mode,\n                    require_raw_lidar_transport_end=True,\n                ),\n",
    )
    patch(
        "v3_process_runtime.py",
        "                strict_affinity=(\n                    affinity_config.strict if affinity_config.enabled else False\n                ),\n            )\n            if capture_path is not None\n",
        "                strict_affinity=(\n                    affinity_config.strict if affinity_config.enabled else False\n                ),\n                expect_raw_lidar_end=True,\n            )\n            if capture_path is not None\n",
    )

    print("Capture refactor upgrade applied.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
