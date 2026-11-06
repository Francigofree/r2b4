"""General run-bound V3 capture sink, separate from live and replay authority."""

from __future__ import annotations

import hashlib
import json
import math
import os
import queue
import threading
import time
from collections import OrderedDict, deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .engine import LAYER_ORDER
from .capture_encoding import (
    CaptureEncodingError as V3CaptureError,
    _INPUT_REFERENCE_KEY, _expected_trace_layers, _validate_trace_layers,
    _encode_completed_result, _compact_admitted_frame,
    encode_value, encode_capture_record,
    encode_raw_lidar_snapshot as _encode_raw_lidar_snapshot,
)
from .execution import (
    CaptureRecord,
    EdgeFaultRecord,
    ExecutionRecord,
    OutputSink,
    REPLAY_STATE_CHECKPOINT_INTERVAL_NS,
    WriterFailureRecord,
)


V3_CAPTURE_SCHEMA = "R2B4_V3_CAPTURE_V1"
V3_CAPTURE_STATUSES = frozenset(("PASS", "FAIL", "FAULT"))


@dataclass(frozen=True, slots=True)
class CaptureWindowConfig:
    """Hard RAM bounds for the passive triggered capture path."""

    pre_event_ns: int = 8_000_000_000
    post_event_ns: int = 2_000_000_000
    ingress_queue_capacity: int = 256
    max_tick_count: int = 768
    max_byte_capacity: int = 64 * 1024 * 1024
    max_raw_lidar_scans: int = 128
    max_raw_lidar_points_per_scan: int = 4_096
    mode: str = "triggered"

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
        if self.mode not in {"triggered", "append_only"}:
            raise ValueError("mode must be triggered or append_only")


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

    def write_encoded(
        self,
        tick: Mapping[str, object],
        *,
        allow_gap: bool = False,
    ) -> None:
        row = dict(tick)
        tick_id = _non_negative_integer(row.get("tick_id"), "tick_id")
        monotonic_ns = _non_negative_integer(row.get("monotonic_ns"), "monotonic_ns")
        if self._ticks:
            previous = self._ticks[-1]
            if not allow_gap and tick_id != int(previous["tick_id"]) + 1:
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
        initial_state_checkpoint: Mapping[str, object] | None = None,
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
        if initial_state_checkpoint is not None:
            payload["initial_state_checkpoint"] = encode_value(
                initial_state_checkpoint
            )
        payload["capture_sha256"] = payload_sha256(payload)
        return payload

    def finalize(
        self,
        status: str,
        output_path: str | Path,
        **document_options: object,
    ) -> Path:
        return write_capture(self.document(status, **document_options), output_path)


@dataclass(frozen=True, slots=True)
class _BufferedTick:
    row: dict[str, object]
    size_bytes: int
    checkpoint_after: dict[str, object] | None = None


class TriggeredCaptureWorker:
    """Bounded passive ingress with all encoding and persistence on one worker."""

    __slots__ = (
        "_capacity_evictions",
        "_config",
        "_dropped_ingress",
        "_emergency_trigger",
        "_error",
        "_fault_observed",
        "_finished",
        "_first_seen_ns",
        "_last_seen_ns",
        "_latest_capacity_eviction_ns",
        "_output_path",
        "_post_window_complete",
        "_queue",
        "_raw_bytes",
        "_raw_evictions",
        "_raw_scans",
        "_result_path",
        "_ring",
        "_ring_bytes",
        "_sink",
        "_started",
        "_stream_handle",
        "_stream_path",
        "_stream_tick_count",
        "_thread",
        "_trigger_ns",
        "_trigger_reason",
    )

    def __init__(
        self,
        sink: CaptureSink,
        output_path: str | Path,
        config: CaptureWindowConfig | None = None,
    ) -> None:
        if not isinstance(sink, CaptureSink):
            raise TypeError("sink must be CaptureSink")
        settings = config or CaptureWindowConfig()
        if not isinstance(settings, CaptureWindowConfig):
            raise TypeError("config must be CaptureWindowConfig")
        path = Path(output_path)
        if not path.is_absolute():
            raise ValueError("capture output path must be absolute")
        self._sink = sink
        self._output_path = path
        self._config = settings
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue(
            maxsize=settings.ingress_queue_capacity
        )
        self._ring: deque[_BufferedTick] = deque()
        self._ring_bytes = 0
        self._raw_scans: OrderedDict[int, tuple[dict[str, object], int]] = OrderedDict()
        self._raw_bytes = 0
        self._dropped_ingress = 0
        self._capacity_evictions = 0
        self._raw_evictions = 0
        self._trigger_ns: int | None = None
        self._trigger_reason: str | None = None
        self._emergency_trigger: tuple[int | None, str] | None = None
        self._first_seen_ns: int | None = None
        self._last_seen_ns: int | None = None
        self._latest_capacity_eviction_ns: int | None = None
        self._post_window_complete = False
        self._error: BaseException | None = None
        self._fault_observed = False
        self._result_path: Path | None = None
        self._started = False
        self._finished = False
        self._stream_path = path.with_name(
            f".{path.name}.{os.getpid()}.{time.monotonic_ns()}.stream"
        )
        self._stream_handle: Any | None = None
        self._stream_tick_count = 0
        self._thread = threading.Thread(
            target=self._run,
            name="v3-capture-worker",
            daemon=True,
        )

    @property
    def failed(self) -> bool:
        return self._error is not None

    @property
    def dropped_ingress_count(self) -> int:
        return self._dropped_ingress

    @property
    def triggered(self) -> bool:
        return self._trigger_ns is not None or self._emergency_trigger is not None

    def start(self) -> None:
        if self._started:
            return
        if self._finished:
            raise RuntimeError("capture worker is already finished")
        self._started = True
        self._thread.start()

    def observe(self, record: CaptureRecord) -> None:
        """Return immediately; queue pressure only invalidates capture evidence."""

        if not isinstance(record, (ExecutionRecord, EdgeFaultRecord, WriterFailureRecord)):
            raise TypeError("capture observer requires a CaptureRecord")
        if not self._started or self._finished or self._error is not None:
            return
        if _record_triggers_capture(record):
            self._fault_observed = True
        try:
            self._queue.put_nowait(("record", record))
        except queue.Full:
            self._dropped_ingress += 1
            if _record_triggers_capture(record):
                context = _record_context(record)
                self._emergency_trigger = (
                    context.monotonic_ns,
                    _record_trigger_reason(record),
                )

    def observe_raw_lidar(self, snapshot: object | None) -> None:
        """Keep only a reference on ingress; truncation and encoding run in the worker."""

        if snapshot is None or not self._started or self._finished or self._error is not None:
            return
        try:
            self._queue.put_nowait(("raw_lidar", snapshot))
        except queue.Full:
            self._dropped_ingress += 1

    def trigger(self, reason: str = "MANUAL", monotonic_ns: int | None = None) -> None:
        normalized = str(reason or "").strip()
        if not normalized:
            raise ValueError("trigger reason must be non-empty")
        if monotonic_ns is not None and (
            not isinstance(monotonic_ns, int)
            or isinstance(monotonic_ns, bool)
            or monotonic_ns < 0
        ):
            raise ValueError("trigger monotonic_ns must be non-negative or None")
        if not self._started or self._finished:
            return
        try:
            self._queue.put_nowait(("trigger", (monotonic_ns, normalized)))
        except queue.Full:
            self._emergency_trigger = (monotonic_ns, normalized)

    def finish(self, status: str, *, terminal: bool = True) -> Path | None:
        if not self._started:
            self.start()
        if self._finished:
            if self._error is not None:
                raise RuntimeError("production V3 capture failed") from self._error
            return self._result_path
        while self._thread.is_alive():
            try:
                self._queue.put(
                    ("finish", (status, bool(terminal))),
                    timeout=0.05,
                )
                break
            except queue.Full:
                continue
        self._thread.join()
        self._finished = True
        if self._error is not None:
            raise RuntimeError("production V3 capture failed") from self._error
        return self._result_path

    def _run(self) -> None:
        try:
            if self._config.mode == "append_only":
                self._stream_path.parent.mkdir(parents=True, exist_ok=True)
                self._stream_handle = self._stream_path.open("x", encoding="utf-8")
                self._stream_path.chmod(0o600)
            while True:
                try:
                    kind, value = self._queue.get(timeout=0.01)
                except queue.Empty:
                    if (
                        self._config.mode == "triggered"
                        and self._post_window_complete
                        and self._result_path is None
                    ):
                        self._finalize(self._trigger_capture_status(), False)
                    continue
                self._consume_emergency_trigger()
                if kind == "record":
                    self._consume_record(value)
                elif kind == "raw_lidar":
                    self._consume_raw_lidar(value)
                elif kind == "trigger":
                    trigger_ns, reason = value
                    self._activate_trigger(trigger_ns, str(reason))
                elif kind == "finish":
                    status, terminal = value
                    self._consume_emergency_trigger()
                    self._finalize(str(status), bool(terminal))
                    return
                else:
                    raise RuntimeError(f"unknown capture work item {kind}")
        except BaseException as exc:
            handle = self._stream_handle
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
                self._stream_handle = None
            self._error = exc

    def _consume_emergency_trigger(self) -> None:
        pending = self._emergency_trigger
        if pending is None:
            return
        self._emergency_trigger = None
        self._activate_trigger(pending[0], pending[1])

    def _consume_record(self, value: object) -> None:
        if not isinstance(value, (ExecutionRecord, EdgeFaultRecord, WriterFailureRecord)):
            raise TypeError("capture worker received an invalid record")
        context = _record_context(value)
        if self._first_seen_ns is None:
            self._first_seen_ns = context.monotonic_ns
        self._last_seen_ns = context.monotonic_ns
        if self._emergency_trigger is not None:
            self._consume_emergency_trigger()
        row = encode_capture_record(value)
        checkpoint_after: dict[str, object] | None = None
        if isinstance(value, ExecutionRecord) and value.state_checkpoint_after is not None:
            encoded_checkpoint = encode_value(value.state_checkpoint_after)
            if not isinstance(encoded_checkpoint, dict):
                raise V3CaptureError("state checkpoint must encode as an object")
            checkpoint_after = encoded_checkpoint
        encoded_size = _canonical_json_size(row) + (
            _canonical_json_size(checkpoint_after)
            if checkpoint_after is not None
            else 0
        )
        if self._config.mode == "append_only":
            assert self._stream_handle is not None
            self._stream_handle.write(
                json.dumps(
                    row,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                + "\n"
            )
            self._stream_handle.flush()
            self._stream_tick_count += 1
            return
        trigger_now = _record_triggers_capture(value)
        if self._trigger_ns is None:
            self._append_tick(_BufferedTick(row, encoded_size, checkpoint_after))
            self._trim_untriggered_ring(context.monotonic_ns)
            if trigger_now:
                self._activate_trigger(
                    context.monotonic_ns,
                    _record_trigger_reason(value),
                )
            return
        upper = self._trigger_ns + self._config.post_event_ns
        if context.monotonic_ns <= upper:
            self._append_tick(_BufferedTick(row, encoded_size, checkpoint_after))
            self._enforce_capacities()
        if context.monotonic_ns >= upper:
            self._post_window_complete = True

    def _append_tick(self, item: _BufferedTick) -> None:
        self._ring.append(item)
        self._ring_bytes += item.size_bytes

    def _trim_untriggered_ring(self, newest_ns: int) -> None:
        lower = (
            newest_ns
            - self._config.pre_event_ns
            - REPLAY_STATE_CHECKPOINT_INTERVAL_NS
        )
        while self._ring and int(self._ring[0].row["monotonic_ns"]) < lower:
            self._drop_oldest_tick(capacity=False)
        self._enforce_capacities()

    def _enforce_capacities(self) -> None:
        while len(self._ring) > self._config.max_tick_count:
            self._drop_oldest_tick(capacity=True)
        while self._ring_bytes + self._raw_bytes > self._config.max_byte_capacity:
            if self._raw_scans:
                self._drop_oldest_raw()
            elif self._ring:
                self._drop_oldest_tick(capacity=True)
            else:
                break

    def _drop_oldest_tick(self, *, capacity: bool) -> None:
        item = self._ring.popleft()
        self._ring_bytes -= item.size_bytes
        if capacity:
            self._capacity_evictions += 1
            self._latest_capacity_eviction_ns = int(item.row["monotonic_ns"])

    def _activate_trigger(self, requested_ns: object, reason: str) -> None:
        if self._trigger_ns is not None:
            return
        if requested_ns is None:
            requested_ns = self._last_seen_ns
        if requested_ns is None:
            self._emergency_trigger = (None, reason)
            return
        trigger_ns = _non_negative_integer(requested_ns, "trigger monotonic_ns")
        self._trigger_ns = trigger_ns
        self._trigger_reason = reason
        lower = (
            trigger_ns
            - self._config.pre_event_ns
            - REPLAY_STATE_CHECKPOINT_INTERVAL_NS
        )
        while self._ring and int(self._ring[0].row["monotonic_ns"]) < lower:
            self._drop_oldest_tick(capacity=False)
        if self._config.post_event_ns == 0 or (
            self._last_seen_ns is not None
            and self._last_seen_ns >= trigger_ns + self._config.post_event_ns
        ):
            self._post_window_complete = True

    def _consume_raw_lidar(self, snapshot: object) -> None:
        row = _encode_raw_lidar_snapshot(
            snapshot,
            self._config.max_raw_lidar_points_per_scan,
        )
        revision = int(row["revision"])
        if revision in self._raw_scans:
            return
        size = _canonical_json_size(row)
        self._raw_scans[revision] = (row, size)
        self._raw_bytes += size
        while len(self._raw_scans) > self._config.max_raw_lidar_scans:
            self._drop_oldest_raw()
        self._enforce_capacities()

    def _drop_oldest_raw(self) -> None:
        _revision, (_row, size) = self._raw_scans.popitem(last=False)
        self._raw_bytes -= size
        self._raw_evictions += 1

    def _finalize(self, status: str, terminal: bool) -> None:
        if self._result_path is not None:
            return
        if self._config.mode == "append_only":
            self._finalize_append_only(status)
            return
        if self._trigger_ns is None:
            return
        lower = self._trigger_ns - self._config.pre_event_ns
        upper = self._trigger_ns + self._config.post_event_ns
        retained = tuple(self._ring)
        checkpoint_index: int | None = None
        for index, item in enumerate(retained):
            if (
                int(item.row["monotonic_ns"]) < lower
                and item.checkpoint_after is not None
            ):
                checkpoint_index = index
        initial_checkpoint = (
            retained[checkpoint_index].checkpoint_after
            if checkpoint_index is not None
            else None
        )
        selected = tuple(
            item
            for index, item in enumerate(retained)
            if (checkpoint_index is None or index > checkpoint_index)
            and (
                (lower <= int(item.row["monotonic_ns"]))
                if checkpoint_index is None
                else True
            )
            and int(item.row["monotonic_ns"]) <= upper
        )
        if not selected:
            raise V3CaptureError("triggered capture contains no ticks")
        sequence_gaps = _sequence_gaps(tuple(item.row for item in selected))
        for item in selected:
            self._sink.write_encoded(item.row, allow_gap=True)
        referenced = _referenced_lidar_revisions(tuple(item.row for item in selected))
        retained_scans = tuple(
            self._raw_scans[revision][0]
            for revision in sorted(referenced)
            if revision in self._raw_scans
        )
        available = {int(row["revision"]) for row in retained_scans}
        missing = sorted(referenced - available)
        capacity_missing = bool(
            self._latest_capacity_eviction_ns is not None
            and self._latest_capacity_eviction_ns >= lower
        )
        ingress_complete = (
            self._dropped_ingress == 0
            and not sequence_gaps
            and not capacity_missing
        )
        state_available = bool(
            initial_checkpoint is not None
            or int(selected[0].row["tick_id"]) == 0
        )
        post_complete = self._post_window_complete
        if terminal and self._last_seen_ns is not None and self._last_seen_ns < upper:
            post_complete = False
        self._result_path = self._sink.finalize(
            status,
            self._output_path,
            capture_window={
                "strategy": "TRIGGERED_RING",
                "trigger_reason": self._trigger_reason,
                "trigger_monotonic_ns": self._trigger_ns,
                "requested_pre_event_ns": self._config.pre_event_ns,
                "requested_post_event_ns": self._config.post_event_ns,
                "captured_first_monotonic_ns": selected[0].row["monotonic_ns"],
                "captured_last_monotonic_ns": selected[-1].row["monotonic_ns"],
                "state_checkpoint_tick_id": (
                    retained[checkpoint_index].row["tick_id"]
                    if checkpoint_index is not None
                    else None
                ),
                "state_warmup_tick_count": sum(
                    int(item.row["monotonic_ns"]) < lower for item in selected
                ),
                "pre_window_complete": bool(
                    self._config.pre_event_ns == 0
                    or (
                        self._first_seen_ns is not None
                        and self._first_seen_ns <= lower
                        and not capacity_missing
                    )
                ),
                "post_window_complete": post_complete,
                "terminal_short_post_window": bool(terminal and not post_complete),
            },
            capture_integrity={
                "complete": ingress_complete,
                "replay_match_eligible": ingress_complete and state_available,
                "dropped_ingress_count": self._dropped_ingress,
                "sequence_gaps": sequence_gaps,
                "tick_capacity_evictions": self._capacity_evictions,
                "state_checkpoint_complete": state_available,
            },
            raw_lidar_scans=retained_scans,
            raw_lidar_evidence={
                "referenced_revisions": sorted(referenced),
                "retained_revisions": sorted(available),
                "missing_revisions": missing,
                "raw_ring_evictions": self._raw_evictions,
                "physical_diagnosis": (
                    "NOT_PROVEN" if missing else "INDICATED" if referenced else "NOT_PROVEN"
                ),
            },
            initial_state_checkpoint=initial_checkpoint,
        )
        self._result_path.chmod(0o600)

    def _finalize_append_only(self, status: str) -> None:
        handle = self._stream_handle
        if handle is not None:
            handle.close()
            self._stream_handle = None
        rows: list[dict[str, object]] = []
        with self._stream_path.open("r", encoding="utf-8") as stream:
            for line in stream:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise V3CaptureError("append-only stream row must be an object")
                rows.append(value)
        if not rows:
            raise V3CaptureError("append-only capture contains no ticks")
        gaps = _sequence_gaps(rows)
        for row in rows:
            self._sink.write_encoded(row, allow_gap=True)
        referenced = _referenced_lidar_revisions(rows)
        retained = tuple(
            self._raw_scans[revision][0]
            for revision in sorted(referenced)
            if revision in self._raw_scans
        )
        available = {int(row["revision"]) for row in retained}
        missing = sorted(referenced - available)
        complete = self._dropped_ingress == 0 and not gaps
        self._result_path = self._sink.finalize(
            status,
            self._output_path,
            capture_window={
                "strategy": "APPEND_ONLY",
                "trigger_reason": None,
                "trigger_monotonic_ns": None,
                "requested_pre_event_ns": None,
                "requested_post_event_ns": None,
                "captured_first_monotonic_ns": rows[0]["monotonic_ns"],
                "captured_last_monotonic_ns": rows[-1]["monotonic_ns"],
                "pre_window_complete": True,
                "post_window_complete": True,
                "terminal_short_post_window": False,
            },
            capture_integrity={
                "complete": complete,
                "replay_match_eligible": complete,
                "dropped_ingress_count": self._dropped_ingress,
                "sequence_gaps": gaps,
                "tick_capacity_evictions": 0,
            },
            raw_lidar_scans=retained,
            raw_lidar_evidence={
                "referenced_revisions": sorted(referenced),
                "retained_revisions": sorted(available),
                "missing_revisions": missing,
                "raw_ring_evictions": self._raw_evictions,
                "physical_diagnosis": (
                    "NOT_PROVEN" if missing else "INDICATED" if referenced else "NOT_PROVEN"
                ),
            },
        )
        self._result_path.chmod(0o600)
        self._stream_path.unlink()

    def _trigger_capture_status(self) -> str:
        return (
            "PASS"
            if not self._fault_observed
            and self._trigger_reason in {"MANUAL", "SIGUSR1", "EXPLICIT_WHOLE_SESSION"}
            else "FAULT"
        )


def _record_context(record: CaptureRecord):
    return record.inputs.context if isinstance(record, ExecutionRecord) else record.context


def _record_triggers_capture(record: CaptureRecord) -> bool:
    if isinstance(record, (EdgeFaultRecord, WriterFailureRecord)):
        return True
    return bool(
        record.result.trace.fault_layer is not None
        or record.result.final_actuation.safety_decision.value == "FAULT"
    )


def _record_trigger_reason(record: CaptureRecord) -> str:
    if isinstance(record, (EdgeFaultRecord, WriterFailureRecord)):
        return record.reason
    return record.result.final_actuation.reason or record.result.trace.fault_layer or "FAULT"


def _canonical_json_size(value: Mapping[str, object]) -> int:
    return len(
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    )


def _sequence_gaps(ticks: Sequence[Mapping[str, object]]) -> list[dict[str, int]]:
    gaps: list[dict[str, int]] = []
    for previous, current in zip(ticks, ticks[1:]):
        previous_id = int(previous["tick_id"])
        current_id = int(current["tick_id"])
        if current_id != previous_id + 1:
            gaps.append({"after_tick_id": previous_id, "before_tick_id": current_id})
    return gaps


def _referenced_lidar_revisions(
    ticks: Sequence[Mapping[str, object]],
) -> set[int]:
    revisions: set[int] = set()
    physical_kinds = {
        "lidar_health",
        "lidar_safety_clearance",
        "lidar_local_points",
    }
    for tick in ticks:
        raw: object | None = None
        inputs = tick.get("inputs")
        if isinstance(inputs, Mapping):
            raw = inputs.get("raw_devices")
        else:
            edge = tick.get("edge_fault")
            if isinstance(edge, Mapping):
                raw = edge.get("raw_devices")
        if not isinstance(raw, Mapping):
            continue
        samples = raw.get("samples")
        if isinstance(samples, (str, bytes)) or not isinstance(samples, Sequence):
            continue
        for sample in samples:
            if not isinstance(sample, Mapping):
                continue
            kind = sample.get("kind")
            sequence = sample.get("sequence")
            if kind in physical_kinds and isinstance(sequence, int) and sequence > 0:
                revisions.add(sequence)
            if kind != "lidar_matcher_diagnostics":
                continue
            values = sample.get("values")
            if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
                continue
            for field in values:
                if (
                    isinstance(field, Mapping)
                    and field.get("key") == "source_raw_scan_id"
                    and isinstance(field.get("value"), int)
                    and int(field["value"]) > 0
                ):
                    revisions.add(int(field["value"]))
    return revisions


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
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n",
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
    checkpoint = payload.get("initial_state_checkpoint")
    if checkpoint is not None and (
        not isinstance(checkpoint, Mapping)
        or checkpoint.get("__type__") != "NativeControlStateCheckpoint"
    ):
        raise V3CaptureError("initial_state_checkpoint must be typed native state")
    raw_ticks = payload.get("ticks")
    if isinstance(raw_ticks, (str, bytes)) or not isinstance(raw_ticks, Sequence):
        raise V3CaptureError("capture ticks must be an array")
    ticks: list[Mapping[str, object]] = []
    integrity = payload.get("capture_integrity")
    integrity_complete = True
    if integrity is not None:
        if not isinstance(integrity, Mapping):
            raise V3CaptureError("capture_integrity must be an object")
        if type(integrity.get("complete")) is not bool:
            raise V3CaptureError("capture_integrity.complete must be bool")
        if type(integrity.get("replay_match_eligible")) is not bool:
            raise V3CaptureError("capture_integrity.replay_match_eligible must be bool")
        integrity_complete = bool(integrity["complete"])
    previous_tick_id: int | None = None
    previous_ns: int | None = None
    for raw_tick in raw_ticks:
        if not isinstance(raw_tick, Mapping):
            raise V3CaptureError("capture tick must be an object")
        tick_id = _non_negative_integer(raw_tick.get("tick_id"), "tick_id")
        monotonic_ns = _non_negative_integer(raw_tick.get("monotonic_ns"), "monotonic_ns")
        if (
            previous_tick_id is not None
            and tick_id != previous_tick_id + 1
            and integrity_complete
        ):
            raise V3CaptureError("complete capture tick ids must be contiguous")
        if previous_ns is not None and monotonic_ns <= previous_ns:
            raise V3CaptureError("capture monotonic time must increase")
        record_type = raw_tick.get("record_type", "closed_input_tick")
        if record_type not in {"closed_input_tick", "edge_fault_tick"}:
            raise V3CaptureError(f"tick {tick_id} record_type is invalid")
        inputs = raw_tick.get("inputs")
        expected = raw_tick.get("expected")
        if record_type == "closed_input_tick":
            if not isinstance(inputs, Mapping) or inputs.get("__type__") != "TickInputs":
                raise V3CaptureError(f"tick {tick_id} lacks typed TickInputs")
        else:
            edge_fault = raw_tick.get("edge_fault")
            if not isinstance(edge_fault, Mapping):
                raise V3CaptureError(f"tick {tick_id} lacks edge_fault evidence")
            context = edge_fault.get("context")
            if not isinstance(context, Mapping) or context.get("__type__") != "TickContext":
                raise V3CaptureError(f"tick {tick_id} edge fault lacks TickContext")
            if context.get("tick_id") != tick_id or context.get("monotonic_ns") != monotonic_ns:
                raise V3CaptureError(f"tick {tick_id} edge fault context mismatch")
        if not isinstance(expected, Mapping):
            raise V3CaptureError(f"tick {tick_id} expected output must be an object")
        layers = expected.get("layers")
        if not isinstance(layers, Mapping):
            raise V3CaptureError(f"tick {tick_id} layers must be an object")
        writer_failure = expected.get("writer_failure")
        if writer_failure is None:
            _validate_trace_layers(
                tuple(str(layer) for layer in layers),
                expected.get("fault_layer"),
                preserve_order=False,
            )
        elif not isinstance(writer_failure, Mapping) or expected.get("fault_layer") != "L12":
            raise V3CaptureError(f"tick {tick_id} writer failure is invalid")
        if (
            "final_actuation" in expected
            and expected.get("final_actuation") != layers.get("L12")
        ):
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
    raw_scans = payload.get("raw_lidar_scans", ())
    if isinstance(raw_scans, (str, bytes)) or not isinstance(raw_scans, Sequence):
        raise V3CaptureError("raw_lidar_scans must be an array")
    revisions: set[int] = set()
    for raw_scan in raw_scans:
        if not isinstance(raw_scan, Mapping):
            raise V3CaptureError("raw lidar scan must be an object")
        revision = _non_negative_integer(raw_scan.get("revision"), "raw lidar revision")
        if revision == 0 or revision in revisions:
            raise V3CaptureError("raw lidar revisions must be positive and unique")
        revisions.add(revision)
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
        "capture_window": payload.get("capture_window"),
        "capture_integrity": payload.get("capture_integrity"),
        "raw_lidar_scan_count": len(payload.get("raw_lidar_scans", ())),
        "raw_lidar_evidence": payload.get("raw_lidar_evidence"),
        "path": str(path.resolve()),
    }


def _non_negative_integer(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise V3CaptureError(f"{name} must be a non-negative integer")
    return value


__all__ = [
    "CaptureSink",
    "CaptureWindowConfig",
    "LAYER_ORDER",
    "V3_CAPTURE_SCHEMA",
    "V3CaptureError",
    "TriggeredCaptureWorker",
    "encode_capture_record",
    "encode_value",
    "inspect_capture",
    "load_capture",
    "payload_sha256",
    "validate_capture",
    "write_capture",
]
