"""ObservationHub -> MCAP capture consumer for R2B4 V3.

Design goals:
- passive consumer: never owns control, motor, lifecycle or safety authority;
- RELIABLE ObservationSubscription integrity authority;
- one-time canonical JSON encoding into bytes, then bytes-only buffering;
- bounded pre-trigger RAM and streaming post-trigger writes;
- MCAP-only output, indexed and CRC-protected, with atomic final rename;
- Python stdlib only for MCAP writing (no pip, venv, ROS or Foxglove runtime);
- source-first R2B4 payloads: current V3 encode_capture_record()/encode_value()
  remain the data authority; MCAP is only the container.
"""

from __future__ import annotations

import hashlib
import json
import os
import queue
import resource
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Mapping

from .capture_encoding import (
    CaptureEncodingError,
    encode_capture_record,
    encode_raw_lidar_snapshot,
    encode_value,
)
from .execution import (
    CaptureRecord,
    EdgeFaultRecord,
    ExecutionRecord,
    REPLAY_STATE_CHECKPOINT_INTERVAL_NS,
    WriterFailureRecord,
)
from .mcap_writer import StdlibMcapWriter
from .observation import (
    DeliveryMode, ObservationClosed, ObservationFrame,
    ObservationIntegrityError, ObservationSubscription,
)

CAPTURE_FORMAT_VERSION = "R2B4_MCAP_CAPTURE_V1"
CLOCK_EPOCH = "linux_monotonic_boot"

TICK_TOPIC = "/r2b4/tick"
RAW_LIDAR_TOPIC = "/r2b4/raw_lidar"
CHECKPOINT_TOPIC = "/r2b4/checkpoint"
EVENT_TOPIC = "/r2b4/event"
RUNTIME_TOPIC = "/r2b4/runtime"

TERMINAL_STATUSES = frozenset(("PASS", "FAIL", "FAULT"))


class CaptureState(str, Enum):
    ARMED = "ARMED"
    TRIGGERED = "TRIGGERED"
    FINALIZING = "FINALIZING"
    FINISHED = "FINISHED"
    INTEGRITY_FAILED = "INTEGRITY_FAILED"


@dataclass(frozen=True, slots=True)
class McapCaptureConfig:
    """Hard bounds and writer settings for the passive capture consumer."""

    pre_event_ns: int = 8_000_000_000
    post_event_ns: int = 2_000_000_000
    max_tick_count: int = 768
    max_raw_lidar_scans: int = 128
    max_byte_capacity: int = 64 * 1024 * 1024
    max_raw_lidar_points_per_scan: int = 4_096
    chunk_target_bytes: int = 1 * 1024 * 1024
    checkpoint_context_ns: int = REPLAY_STATE_CHECKPOINT_INTERVAL_NS
    mode: str = "triggered"
    max_session_records: int = 100_000
    max_record_bytes: int = 16 * 1024 * 1024
    max_referenced_revisions_per_tick: int = 16

    def __post_init__(self) -> None:
        for name in ("pre_event_ns", "post_event_ns", "checkpoint_context_ns"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        for name in (
            "max_tick_count",
            "max_raw_lidar_scans",
            "max_byte_capacity",
            "max_raw_lidar_points_per_scan",
            "chunk_target_bytes",
            "max_session_records",
            "max_record_bytes",
            "max_referenced_revisions_per_tick",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

        if self.mode not in {"triggered", "append_only"}:
            raise ValueError("mode must be triggered or append_only")


@dataclass(frozen=True, slots=True)
class EncodedRecord:
    """Small ring entry: payload data is already immutable bytes."""

    hub_sequence: int
    source_topic: str
    mcap_topic: str
    monotonic_ns: int
    sequence: int
    payload: bytes
    tick_id: int | None = None
    raw_lidar_revision: int | None = None
    referenced_lidar_revisions: tuple[int, ...] = ()
    is_checkpoint: bool = False
    source_point_count: int | None = None
    points_truncated: bool = False
    published_monotonic_ns: int | None = None

    @property
    def size_bytes(self) -> int:
        # Charge object/tuple/reference overhead as well as encoded bytes.
        return (sys.getsizeof(self) + sys.getsizeof(self.payload)
                + sys.getsizeof(self.referenced_lidar_revisions)
                + sum(sys.getsizeof(v) for v in self.referenced_lidar_revisions) + 256)


@dataclass(frozen=True, slots=True)
class CaptureResult:
    path: Path
    state: CaptureState
    status: str
    complete: bool
    message_stream_sha256: str
    ingress_drop_count: int
    hub_sequence_gap_count: int
    tick_sequence_gap_count: int
    raw_lidar_missing_revisions: tuple[int, ...]


class McapCaptureConsumer:
    """Bounded, threaded MCAP capture consumer for ObservationHub.

    The worker consumes the sole RELIABLE mailbox directly. Call finish only
    after all publishers have stopped and the hub has closed; accepted frames
    are drained before subscription integrity is checked and the file finalized.
    """

    __slots__ = (
        "_capture_id",
        "_configuration",
        "_metadata",
        "_output_path",
        "_config",
        "_subscription",
        "_control",
        "_seen_records",
        "_started_ns",
        "_write_duration_ns",
        "_cpu_started_ns",
        "_timing",
        "_thread",
        "_started",
        "_finished",
        "_finish_requested",
        "_state",
        "_lock",
        "_error",
        "_partial_path",
        "_result",
        "_ring",
        "_ring_bytes",
        "_ring_tick_count",
        "_ring_raw_count",
        "_capacity_eviction_count",
        "_latest_capacity_eviction_ns",
        "_first_seen_ns",
        "_last_seen_ns",
        "_captured_tick_gaps",
        "_captured_last_tick_id",
        "_trigger_ns",
        "_trigger_reason",
        "_post_window_complete",
        "_fault_observed",
        "_integrity_reasons",
        "_writer",
        "_handle",
        "_channels",
        "_message_digest",
        "_written_raw_revisions",
        "_referenced_raw_revisions",
        "_raw_duplicate_count",
        "_raw_out_of_order_count",
        "_raw_revision_gaps",
        "_raw_truncated_count",
        "_last_raw_revision",
        "_captured_tick_count",
        "_captured_raw_count",
        "_captured_checkpoint_count",
        "_captured_event_count",
        "_emergency_trigger",
    )

    def __init__(
        self,
        capture_id: str,
        output_path: str | Path,
        *,
        subscription: ObservationSubscription,
        configuration: Mapping[str, object],
        metadata: Mapping[str, object] | None = None,
        config: McapCaptureConfig | None = None,
    ) -> None:
        identifier = str(capture_id or "").strip()
        if not identifier:
            raise ValueError("capture_id must be non-empty")
        path = Path(output_path)
        if not path.is_absolute():
            raise ValueError("MCAP output path must be absolute")
        if path.suffix.lower() != ".mcap":
            raise ValueError("MCAP output path must end in .mcap")
        settings = config or McapCaptureConfig()
        if not isinstance(settings, McapCaptureConfig):
            raise TypeError("config must be McapCaptureConfig")

        encoded_configuration = encode_value(configuration)
        encoded_metadata = encode_value(metadata or {})
        if not isinstance(encoded_configuration, dict) or not isinstance(encoded_metadata, dict):
            raise TypeError("configuration and metadata must encode as mappings")

        self._capture_id = identifier
        self._configuration = encoded_configuration
        self._metadata = encoded_metadata
        self._output_path = path
        self._config = settings
        if not isinstance(subscription, ObservationSubscription):
            raise TypeError("subscription must be ObservationSubscription")
        if subscription.config.mode is not DeliveryMode.RELIABLE or not subscription.config.required:
            raise ValueError("capture requires a required RELIABLE subscription")
        self._subscription = subscription
        self._control = threading.Event()
        self._seen_records = 0
        self._started_ns = 0
        self._write_duration_ns = 0
        self._cpu_started_ns = 0
        self._timing = {"tick_count": 0, "period_count": 0, "period_sum_ns": 0,
                        "period_max_ns": 0, "completion_latency_sum_ns": 0,
                        "completion_latency_max_ns": 0}
        self._thread = threading.Thread(
            target=self._run,
            name="v3-mcap-capture-consumer",
            daemon=True,
        )
        self._started = False
        self._finished = False
        self._finish_requested: tuple[str, bool] | None = None
        self._state = CaptureState.ARMED
        self._lock = threading.Lock()
        self._error: BaseException | None = None
        self._partial_path: Path | None = None
        self._result: CaptureResult | None = None

        self._ring: deque[EncodedRecord] = deque()
        self._ring_bytes = 0
        self._ring_tick_count = 0
        self._ring_raw_count = 0
        self._capacity_eviction_count = 0
        self._latest_capacity_eviction_ns: int | None = None

        self._first_seen_ns: int | None = None
        self._last_seen_ns: int | None = None
        self._captured_tick_gaps: list[tuple[int, int]] = []
        self._captured_last_tick_id: int | None = None

        self._trigger_ns: int | None = None
        self._trigger_reason: str | None = None
        self._post_window_complete = False
        self._fault_observed = False
        self._integrity_reasons: set[str] = set()

        self._writer: StdlibMcapWriter | None = None
        self._handle = None
        self._channels: dict[str, int] = {}
        self._message_digest = hashlib.sha256()

        self._written_raw_revisions: set[int] = set()
        self._referenced_raw_revisions: set[int] = set()
        self._raw_duplicate_count = 0
        self._raw_out_of_order_count = 0
        self._raw_revision_gaps: list[tuple[int, int]] = []
        self._raw_truncated_count = 0
        self._last_raw_revision: int | None = None

        self._captured_tick_count = 0
        self._captured_raw_count = 0
        self._captured_checkpoint_count = 0
        self._captured_event_count = 0
        self._emergency_trigger: tuple[int | None, str] | None = None

    @property
    def state(self) -> CaptureState:
        return self._state

    @property
    def failed(self) -> bool:
        return self._error is not None

    @property
    def partial_path(self) -> Path | None:
        return self._partial_path

    @property
    def result(self) -> CaptureResult | None:
        return self._result

    def start(self) -> None:
        if self._started:
            return
        if self._finished:
            raise RuntimeError("capture consumer is already finished")
        self._started = True
        self._started_ns = time.monotonic_ns()
        self._cpu_started_ns = time.process_time_ns()
        self._thread.start()

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
        if not self._started:
            self.start()
        if self._finished:
            return
        with self._lock:
            if self._emergency_trigger is None:
                self._emergency_trigger = (monotonic_ns, normalized)
        self._control.set()

    def finish(self, status: str = "PASS", *, terminal: bool = True) -> CaptureResult | None:
        terminal_status = str(status or "").upper()
        if terminal_status not in TERMINAL_STATUSES:
            raise ValueError("status must be PASS, FAIL or FAULT")
        if not self._started:
            self.start()
        if self._finished:
            if self._error is not None:
                raise RuntimeError(
                    f"MCAP capture failed; partial file kept at {self._partial_path}"
                ) from self._error
            return self._result

        if not self._subscription.snapshot().closed:
            raise RuntimeError("stop publishers and close ObservationHub before capture finish")
        with self._lock:
            self._finish_requested = (terminal_status, bool(terminal))
        self._control.set()
        self._thread.join()
        self._finished = True
        if self._error is not None:
            raise RuntimeError(
                f"MCAP capture failed; partial file kept at {self._partial_path}"
            ) from self._error
        return self._result

    def _run(self) -> None:
        try:
            while True:
                self._consume_emergency_trigger()
                try:
                    frame = self._subscription.get(timeout=0.01)
                except queue.Empty:
                    continue
                except ObservationClosed:
                    # A closed mailbox raises only after its complete backlog drains.
                    with self._lock:
                        finish = self._finish_requested
                    if finish is None:
                        self._control.wait(0.01)
                        self._control.clear()
                        continue
                    try:
                        self._subscription.assert_integrity()
                    except ObservationIntegrityError:
                        self._integrity_reasons.add("OBSERVATION_RELIABLE_LOSS")
                    self._consume_emergency_trigger()
                    if self._integrity_reasons and self._trigger_ns is None:
                        self._activate_trigger(self._last_seen_ns, "CAPTURE_INTEGRITY_FAILURE")
                    if self._writer is not None:
                        self._finalize(finish[0], terminal=finish[1])
                    elif self._integrity_reasons:
                        raise CaptureEncodingError("capture lost all tick evidence")
                    return
                self._consume_observation(frame)
                self._consume_emergency_trigger()
        except BaseException as exc:
            self._integrity_reasons.add("CAPTURE_WORKER_EXCEPTION")
            self._state = CaptureState.INTEGRITY_FAILED
            self._error = exc
            handle = self._handle
            if handle is not None:
                try:
                    handle.flush()
                    os.fsync(handle.fileno())
                except Exception:
                    pass
                try:
                    handle.close()
                except Exception:
                    pass
                self._handle = None

    def _consume_emergency_trigger(self) -> None:
        with self._lock:
            pending = self._emergency_trigger
            self._emergency_trigger = None
        if pending is not None:
            self._activate_trigger(pending[0], pending[1])

    def _consume_observation(self, item: ObservationFrame) -> None:
        self._seen_records += 1
        if self._seen_records > self._config.max_session_records:
            self._integrity_reasons.add("SESSION_RECORD_LIMIT")
            return
        encoded = self._encode_observation(item)
        for record in encoded:
            if len(record.payload) > self._config.max_record_bytes:
                raise CaptureEncodingError("encoded record exceeds max_record_bytes")
            if record.mcap_topic == TICK_TOPIC:
                timing = self._timing
                timing["tick_count"] += 1
                latency = max(0, item.published_monotonic_ns - record.monotonic_ns)
                timing["completion_latency_sum_ns"] += latency
                timing["completion_latency_max_ns"] = max(timing["completion_latency_max_ns"], latency)
                if self._last_seen_ns is not None:
                    period = record.monotonic_ns - self._last_seen_ns
                    timing["period_count"] += 1
                    timing["period_sum_ns"] += period
                    timing["period_max_ns"] = max(timing["period_max_ns"], period)
                if self._first_seen_ns is None:
                    self._first_seen_ns = record.monotonic_ns
                self._last_seen_ns = record.monotonic_ns
            if self._config.mode == "append_only":
                if self._writer is None:
                    self._open_writer(first_time_ns=record.monotonic_ns)
                    self._state = CaptureState.TRIGGERED
                    self._trigger_reason = "APPEND_ONLY"
                self._write_encoded_record(record)
            elif self._trigger_ns is None:
                self._append_ring(record)
                if self._last_seen_ns is not None:
                    self._trim_untriggered_ring(self._last_seen_ns)
            else:
                upper = self._trigger_ns + self._config.post_event_ns
                if record.monotonic_ns <= upper:
                    self._write_encoded_record(record)
                if record.mcap_topic == TICK_TOPIC and record.monotonic_ns >= upper:
                    self._post_window_complete = True

        if _payload_triggers_capture(item.payload):
            self._fault_observed = True
            self._activate_trigger(self._last_seen_ns, _payload_trigger_reason(item.payload))

    def _encode_observation(self, item: ObservationFrame) -> tuple[EncodedRecord, ...]:
        payload = item.payload
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

        if _looks_like_raw_lidar_topic(item.topic):
            row = encode_raw_lidar_snapshot(
                payload,
                self._config.max_raw_lidar_points_per_scan,
            )
            revision = _non_negative_int(row.get("revision"), "raw lidar revision")
            if revision <= 0:
                raise CaptureEncodingError("raw lidar revision must be positive")
            previous = self._last_raw_revision
            if previous is not None:
                if revision == previous:
                    self._raw_duplicate_count += 1
                    return ()
                if revision < previous:
                    self._raw_out_of_order_count += 1
                    self._integrity_reasons.add("RAW_LIDAR_REVISION_OUT_OF_ORDER")
                elif revision > previous + 1:
                    self._raw_revision_gaps.append((previous, revision))
                    self._integrity_reasons.add("RAW_LIDAR_REVISION_GAP")
            self._last_raw_revision = max(previous or revision, revision)
            truncated = bool(row.get("points_truncated"))
            if truncated:
                self._raw_truncated_count += 1
                self._integrity_reasons.add("RAW_LIDAR_POINTS_TRUNCATED")
            return (
                EncodedRecord(
                    hub_sequence=item.sequence,
                    published_monotonic_ns=item.published_monotonic_ns,
                    source_topic=item.topic,
                    mcap_topic=RAW_LIDAR_TOPIC,
                    monotonic_ns=_non_negative_int(
                        row.get("measurement_monotonic_ns"),
                        "raw lidar measurement_monotonic_ns",
                    ),
                    sequence=revision,
                    payload=_json_bytes(row),
                    raw_lidar_revision=revision,
                    source_point_count=int(row.get("source_point_count", 0)),
                    points_truncated=truncated,
                ),
            )

        if _looks_like_checkpoint_topic(item.topic):
            checkpoint = encode_value(payload)
            return (
                EncodedRecord(
                    hub_sequence=item.sequence,
                    published_monotonic_ns=item.published_monotonic_ns,
                    source_topic=item.topic,
                    mcap_topic=CHECKPOINT_TOPIC,
                    monotonic_ns=item.published_monotonic_ns,
                    sequence=item.sequence,
                    payload=_json_bytes(
                        {
                            "monotonic_ns": item.published_monotonic_ns,
                            "state": checkpoint,
                        }
                    ),
                    is_checkpoint=True,
                ),
            )

        event = encode_value(payload)
        return (
            EncodedRecord(
                hub_sequence=item.sequence,
                published_monotonic_ns=item.published_monotonic_ns,
                source_topic=item.topic,
                mcap_topic=EVENT_TOPIC,
                monotonic_ns=item.published_monotonic_ns,
                sequence=item.sequence,
                payload=_json_bytes(
                    {
                        "source_topic": item.topic,
                        "monotonic_ns": item.published_monotonic_ns,
                        "payload": event,
                    }
                ),
            ),
        )

    def _append_ring(self, item: EncodedRecord) -> None:
        self._ring.append(item)
        self._ring_bytes += item.size_bytes
        if item.mcap_topic == TICK_TOPIC:
            self._ring_tick_count += 1
        elif item.mcap_topic == RAW_LIDAR_TOPIC:
            self._ring_raw_count += 1
        self._enforce_capacities()

    def _trim_untriggered_ring(self, newest_ns: int) -> None:
        lower = newest_ns - self._config.pre_event_ns - self._config.checkpoint_context_ns
        while self._ring and self._ring[0].monotonic_ns < lower:
            self._drop_oldest(capacity=False)
        self._enforce_capacities()

    def _enforce_capacities(self) -> None:
        while self._ring_tick_count > self._config.max_tick_count:
            self._drop_oldest_matching(TICK_TOPIC)
        while self._ring_raw_count > self._config.max_raw_lidar_scans:
            self._drop_oldest_matching(RAW_LIDAR_TOPIC)
        while self._ring_bytes > self._config.max_byte_capacity:
            self._drop_oldest(capacity=True)

    def _drop_oldest_matching(self, topic: str) -> None:
        # Count limits are per stream. Do not evict unrelated tick evidence just
        # because the raw-LiDAR stream reached its own count bound (or vice versa).
        for index, item in enumerate(self._ring):
            if item.mcap_topic != topic:
                continue
            del self._ring[index]
            self._ring_bytes -= item.size_bytes
            if topic == TICK_TOPIC:
                self._ring_tick_count -= 1
            elif topic == RAW_LIDAR_TOPIC:
                self._ring_raw_count -= 1
            self._capacity_eviction_count += 1
            self._latest_capacity_eviction_ns = item.monotonic_ns
            return
        raise RuntimeError(f"ring count inconsistent for {topic}")

    def _drop_oldest(self, *, capacity: bool) -> None:
        if not self._ring:
            return
        item = self._ring.popleft()
        self._ring_bytes -= item.size_bytes
        if item.mcap_topic == TICK_TOPIC:
            self._ring_tick_count -= 1
        elif item.mcap_topic == RAW_LIDAR_TOPIC:
            self._ring_raw_count -= 1
        if capacity:
            self._capacity_eviction_count += 1
            self._latest_capacity_eviction_ns = item.monotonic_ns

    def _activate_trigger(self, requested_ns: object, reason: str) -> None:
        if self._trigger_ns is not None or self._config.mode == "append_only":
            return
        if self._ring_tick_count == 0:
            with self._lock:
                self._emergency_trigger = (requested_ns, reason)
            return
        if requested_ns is None:
            requested_ns = self._last_seen_ns
        if requested_ns is not None and self._last_seen_ns is not None and requested_ns > self._last_seen_ns:
            with self._lock:
                self._emergency_trigger = (requested_ns, reason)
            return
        if requested_ns is None:
            with self._lock:
                self._emergency_trigger = (None, reason)
            return
        trigger_ns = _non_negative_int(requested_ns, "trigger monotonic_ns")
        self._trigger_ns = trigger_ns
        self._trigger_reason = reason
        self._state = CaptureState.TRIGGERED

        lower = trigger_ns - self._config.pre_event_ns
        retained = tuple(self._ring)
        checkpoint_index: int | None = None
        for index, item in enumerate(retained):
            if item.monotonic_ns < lower and item.is_checkpoint:
                checkpoint_index = index

        if checkpoint_index is None:
            selected = tuple(item for item in retained if item.monotonic_ns >= lower)
        else:
            # The checkpoint represents state after its associated tick. Keep all
            # following observations so replay/debug tooling can warm up exactly.
            selected = retained[checkpoint_index:]

        if not any(item.mcap_topic == TICK_TOPIC for item in selected):
            # Explicit early timestamps anchor to the first available completed tick.
            self._trigger_ns = None
            self._activate_trigger(self._last_seen_ns, reason)
            return

        needed_raw = {revision for item in selected for revision in item.referenced_lidar_revisions}
        selected_ids = {id(item) for item in selected}
        selected = tuple(item for item in retained if id(item) in selected_ids or item.raw_lidar_revision in needed_raw)
        self._open_writer(first_time_ns=min(item.monotonic_ns for item in selected))
        for item in selected:
            self._write_encoded_record(item)
        self._clear_ring()

        self._write_event(
            trigger_ns,
            {
                "event_type": "capture_trigger",
                "capture_id": self._capture_id,
                "reason": reason,
                "trigger_monotonic_ns": trigger_ns,
            },
        )

        if self._config.post_event_ns == 0 or (
            self._last_seen_ns is not None
            and self._last_seen_ns >= trigger_ns + self._config.post_event_ns
        ):
            self._post_window_complete = True

    def _clear_ring(self) -> None:
        self._ring.clear()
        self._ring_bytes = 0
        self._ring_tick_count = 0
        self._ring_raw_count = 0

    def _open_writer(self, *, first_time_ns: int) -> None:
        if self._writer is not None:
            return
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._partial_path = self._output_path.with_name(
            f".{self._output_path.name}.{os.getpid()}.{time.monotonic_ns()}.partial"
        )
        handle = self._partial_path.open("xb")
        os.chmod(self._partial_path, 0o600)
        writer = StdlibMcapWriter(
            handle,
            chunk_target_bytes=self._config.chunk_target_bytes,
        )
        writer.start(profile="r2b4-v3", library="R2B4 McapCaptureConsumer stdlib/1")
        channel_metadata = {
            "clock_epoch": CLOCK_EPOCH,
            "capture_format_version": CAPTURE_FORMAT_VERSION,
        }
        self._channels = {
            TICK_TOPIC: writer.register_channel(
                TICK_TOPIC, message_encoding="json", metadata=channel_metadata
            ),
            RAW_LIDAR_TOPIC: writer.register_channel(
                RAW_LIDAR_TOPIC, message_encoding="json", metadata=channel_metadata
            ),
            CHECKPOINT_TOPIC: writer.register_channel(
                CHECKPOINT_TOPIC, message_encoding="json", metadata=channel_metadata
            ),
            EVENT_TOPIC: writer.register_channel(
                EVENT_TOPIC, message_encoding="json", metadata=channel_metadata
            ),
            RUNTIME_TOPIC: writer.register_channel(
                RUNTIME_TOPIC, message_encoding="json", metadata=channel_metadata
            ),
        }
        writer.add_metadata(
            "r2b4.capture",
            {
                "capture_id": self._capture_id,
                "capture_format_version": CAPTURE_FORMAT_VERSION,
                "clock_epoch": CLOCK_EPOCH,
                "container": "MCAP v0",
                "message_encoding": "json",
                "compression": "none",
            },
        )
        self._writer = writer
        self._handle = handle
        self._write_message(
            RUNTIME_TOPIC,
            first_time_ns,
            0,
            _json_bytes(
                {
                    "capture_id": self._capture_id,
                    "capture_format_version": CAPTURE_FORMAT_VERSION,
                    "clock_epoch": CLOCK_EPOCH,
                    "configuration": self._configuration,
                    "metadata": self._metadata,
                }
            ),
        )

    def _write_encoded_record(self, item: EncodedRecord) -> None:
        if self._writer is None:
            raise RuntimeError("MCAP writer is not open")

        if item.mcap_topic == TICK_TOPIC:
            tick_id = int(item.tick_id or 0)
            previous = self._captured_last_tick_id
            if previous is not None and tick_id != previous + 1:
                self._captured_tick_gaps.append((previous, tick_id))
                self._integrity_reasons.add("CAPTURED_TICK_SEQUENCE_GAP")
            self._captured_last_tick_id = tick_id
            self._captured_tick_count += 1
            self._referenced_raw_revisions.update(item.referenced_lidar_revisions)
        elif item.mcap_topic == RAW_LIDAR_TOPIC:
            if item.raw_lidar_revision is not None:
                if item.raw_lidar_revision in self._written_raw_revisions:
                    self._raw_duplicate_count += 1
                    return
                self._written_raw_revisions.add(item.raw_lidar_revision)
            self._captured_raw_count += 1
        elif item.mcap_topic == CHECKPOINT_TOPIC:
            self._captured_checkpoint_count += 1
        message_sequence = item.sequence
        if item.mcap_topic == EVENT_TOPIC:
            self._captured_event_count += 1
            message_sequence = self._captured_event_count

        self._write_message(
            item.mcap_topic,
            item.monotonic_ns,
            message_sequence,
            item.payload,
            published_monotonic_ns=item.published_monotonic_ns,
        )

    def _write_message(
        self,
        topic: str,
        monotonic_ns: int,
        sequence: int,
        payload: bytes,
        *,
        include_in_digest: bool = True,
        published_monotonic_ns: int | None = None,
    ) -> None:
        writer = self._writer
        if writer is None:
            raise RuntimeError("MCAP writer is not open")
        channel_id = self._channels[topic]
        write_started = time.monotonic_ns()
        writer.add_message(
            channel_id,
            log_time_ns=monotonic_ns,
            publish_time_ns=monotonic_ns if published_monotonic_ns is None else published_monotonic_ns,
            sequence=sequence,
            data=payload,
        )
        self._write_duration_ns += time.monotonic_ns() - write_started
        if include_in_digest:
            topic_bytes = topic.encode("utf-8")
            self._message_digest.update(len(topic_bytes).to_bytes(4, "little"))
            self._message_digest.update(topic_bytes)
            self._message_digest.update((sequence & 0xFFFFFFFF).to_bytes(4, "little"))
            self._message_digest.update(monotonic_ns.to_bytes(8, "little"))
            self._message_digest.update(len(payload).to_bytes(8, "little"))
            self._message_digest.update(payload)

    def _write_event(self, monotonic_ns: int, payload: Mapping[str, object]) -> None:
        self._captured_event_count += 1
        self._write_message(
            EVENT_TOPIC,
            monotonic_ns,
            self._captured_event_count,
            _json_bytes(payload),
        )

    def _automatic_status(self) -> str:
        if self._fault_observed:
            return "FAULT"
        if self._integrity_reasons:
            return "FAIL"
        return "PASS"

    def _finalize(self, status: str, *, terminal: bool) -> None:
        if self._result is not None:
            return
        writer = self._writer
        handle = self._handle
        if writer is None or handle is None or self._partial_path is None:
            raise RuntimeError("cannot finalize unopened MCAP capture")

        self._state = CaptureState.FINALIZING
        status = str(status).upper()
        if status not in TERMINAL_STATUSES:
            raise ValueError("status must be PASS, FAIL or FAULT")

        upper = int(self._trigger_ns or 0) + self._config.post_event_ns
        post_complete = self._post_window_complete
        if terminal and self._last_seen_ns is not None and self._last_seen_ns < upper:
            post_complete = False

        lower = int(self._trigger_ns or 0) - self._config.pre_event_ns
        capacity_missing = bool(
            self._latest_capacity_eviction_ns is not None
            and self._latest_capacity_eviction_ns >= lower
        )
        if capacity_missing:
            self._integrity_reasons.add("PRE_TRIGGER_CAPACITY_EVICTION")
        pre_complete = bool(
            self._config.pre_event_ns == 0
            or (
                self._first_seen_ns is not None
                and self._first_seen_ns <= lower
                and not capacity_missing
            )
        )
        if self._config.mode == "append_only":
            pre_complete = post_complete = self._captured_tick_count > 0
        if not pre_complete:
            self._integrity_reasons.add("PRE_WINDOW_INCOMPLETE")
        subscription = self._subscription.snapshot()
        ingress_drops = subscription.lost_count
        if ingress_drops:
            self._integrity_reasons.add("OBSERVATION_RELIABLE_LOSS")
        if not post_complete:
            self._integrity_reasons.add("POST_WINDOW_INCOMPLETE")

        missing_raw = tuple(sorted(self._referenced_raw_revisions - self._written_raw_revisions))
        if missing_raw:
            self._integrity_reasons.add("REFERENCED_RAW_LIDAR_MISSING")

        complete = not self._integrity_reasons
        if self._fault_observed and status == "PASS":
            status = "FAULT"
        if not complete and status == "PASS":
            status = "FAIL"
        digest = self._message_digest.hexdigest()
        final_time = max(
            int(self._last_seen_ns or self._trigger_ns or 0),
            int(self._trigger_ns or 0),
        )
        integrity = {
            "complete": complete,
            "integrity_reasons": sorted(self._integrity_reasons),
            "ingress_drop_count": ingress_drops,
            "observation_subscription": encode_value(subscription),
            "captured_tick_sequence_gaps": [
                {"after": before, "before": after}
                for before, after in self._captured_tick_gaps
            ],
            "capacity_eviction_count": self._capacity_eviction_count,
            "raw_lidar_missing_revisions": list(missing_raw),
            "raw_lidar_duplicate_count": self._raw_duplicate_count,
            "raw_lidar_out_of_order_count": self._raw_out_of_order_count,
            "raw_lidar_revision_gaps": [
                {"after": before, "before": after}
                for before, after in self._raw_revision_gaps
            ],
            "raw_lidar_truncated_count": self._raw_truncated_count,
            "pre_window_complete": pre_complete,
            "post_window_complete": post_complete,
            "message_stream_sha256": digest,
            "digest_scope": "all MCAP messages before capture_finalized event",
        }
        final_event = {
            "event_type": "capture_finalized",
            "capture_id": self._capture_id,
            "status": status,
            "trigger_reason": self._trigger_reason,
            "trigger_monotonic_ns": self._trigger_ns,
            "captured_tick_count": self._captured_tick_count,
            "captured_raw_lidar_count": self._captured_raw_count,
            "captured_checkpoint_count": self._captured_checkpoint_count,
            "captured_event_count_before_final": self._captured_event_count,
            "integrity": integrity,
            "capture_mode": self._config.mode,
            "metrics": {
                "subscription_high_water_mark": subscription.high_water_mark,
                "write_duration_ns": self._write_duration_ns,
                "bytes_before_finalization": writer.tell(),
                "elapsed_ns": time.monotonic_ns() - self._started_ns,
                "process_cpu_ns": time.process_time_ns() - self._cpu_started_ns,
                "process_max_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
                "runtime_tick_timing": dict(self._timing),
            },
        }
        self._captured_event_count += 1
        self._write_message(
            EVENT_TOPIC,
            final_time,
            self._captured_event_count,
            _json_bytes(final_event),
            include_in_digest=False,
        )
        writer.add_metadata(
            "r2b4.capture.final",
            {
                "capture_id": self._capture_id,
                "status": status,
                "trigger_reason": str(self._trigger_reason or ""),
                "trigger_monotonic_ns": str(self._trigger_ns or 0),
                "complete": "true" if complete else "false",
                "message_stream_sha256": digest,
                "integrity_reasons": json.dumps(sorted(self._integrity_reasons), separators=(",", ":")),
                "tick_count": str(self._captured_tick_count),
                "raw_lidar_count": str(self._captured_raw_count),
                "checkpoint_count": str(self._captured_checkpoint_count),
            },
        )
        writer.finish()
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        self._handle = None

        # link is an atomic no-clobber final publication on the same filesystem.
        os.link(self._partial_path, self._output_path)
        self._partial_path.unlink()
        self._partial_path = None
        os.chmod(self._output_path, 0o600)
        _fsync_directory(self._output_path.parent)

        final_state = CaptureState.FINISHED if complete else CaptureState.INTEGRITY_FAILED
        self._state = final_state
        self._result = CaptureResult(
            path=self._output_path,
            state=final_state,
            status=status,
            complete=complete,
            message_stream_sha256=digest,
            ingress_drop_count=ingress_drops,
            hub_sequence_gap_count=0,
            tick_sequence_gap_count=len(self._captured_tick_gaps),
            raw_lidar_missing_revisions=missing_raw,
        )


def conservative_ram_budget(config: McapCaptureConfig, *, mailbox_capacity: int,
                            max_retained_observation_bytes: int,
                            max_encoding_working_bytes: int) -> dict[str, int]:
    """Conditional capacity budget, including CPython bookkeeping and peak copies.

    The caller must bound the *whole retained typed graph* and encoder working
    graph, including checkpoints. Encoded JSON length is not such a bound.
    These assumptions cannot be enforced by the data-blind producer. This is
    not an RSS guarantee: interpreter, layers, drivers and allocator are extra.
    Session records bound indexes and diagnostic revision/gap sets in both modes.
    """
    if min(mailbox_capacity, max_retained_observation_bytes, max_encoding_working_bytes) <= 0:
        raise ValueError("RAM budget assumptions must be positive")
    chunk = max(config.chunk_target_bytes, config.max_record_bytes + 64)
    parts = {
        "mailbox_and_active_payloads": (mailbox_capacity + 1) * (max_retained_observation_bytes + 512),
        "ring_with_entry_overhead": config.max_byte_capacity + 2 * (config.max_record_bytes + 1024),
        "encoder_working_graph_and_json": max_encoding_working_bytes + 8 * config.max_record_bytes,
        "writer_chunk_copies_and_message_indexes": 8 * chunk + (chunk // 31 + 1) * 256,
        # Worst case every observation yields tick + checkpoint and a separate chunk.
        # Also charge all bounded referenced revision sets and failure details.
        "session_indexes_and_diagnostics": config.max_session_records * (
            8192 + 128 * config.max_referenced_revisions_per_tick),
    }
    parts["total_bytes"] = sum(parts.values())
    return parts


def _json_bytes(value: object) -> bytes:
    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise CaptureEncodingError("capture value is not canonical JSON") from exc


def _non_negative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise CaptureEncodingError(f"{name} must be a non-negative integer")
    return value


def _looks_like_raw_lidar_topic(topic: str) -> bool:
    normalized = topic.strip().lower().replace("-", "_")
    return normalized in {
        RAW_LIDAR_TOPIC,
        "raw_lidar",
        "v3.raw_lidar",
        "lidar_raw",
        "observation.raw_lidar",
        "observation/raw_lidar",
    }


def _looks_like_checkpoint_topic(topic: str) -> bool:
    normalized = topic.strip().lower().replace("-", "_")
    return normalized in {
        CHECKPOINT_TOPIC,
        "checkpoint",
        "state_checkpoint",
        "observation.checkpoint",
        "observation/checkpoint",
    }


def _payload_triggers_capture(payload: object) -> bool:
    if isinstance(payload, (EdgeFaultRecord, WriterFailureRecord)):
        return True
    if not isinstance(payload, ExecutionRecord):
        return False
    return bool(
        payload.result.trace.fault_layer is not None
        or payload.result.final_actuation.safety_decision.value == "FAULT"
    )


def _payload_trigger_reason(payload: object) -> str:
    if isinstance(payload, (EdgeFaultRecord, WriterFailureRecord)):
        return payload.reason
    if isinstance(payload, ExecutionRecord):
        return (
            payload.result.final_actuation.reason
            or payload.result.trace.fault_layer
            or "FAULT"
        )
    return "CAPTURE_TRIGGER"


def _referenced_lidar_revisions(tick: Mapping[str, object]) -> set[int]:
    revisions: set[int] = set()
    physical_kinds = {
        "lidar_health",
        "lidar_safety_clearance",
        "lidar_local_points",
    }
    raw: object | None = None
    inputs = tick.get("inputs")
    if isinstance(inputs, Mapping):
        raw = inputs.get("raw_devices")
    else:
        edge = tick.get("edge_fault")
        if isinstance(edge, Mapping):
            raw = edge.get("raw_devices")
    if not isinstance(raw, Mapping):
        return revisions
    samples = raw.get("samples")
    if isinstance(samples, (str, bytes)) or not isinstance(samples, (list, tuple)):
        return revisions
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
        if isinstance(values, (str, bytes)) or not isinstance(values, (list, tuple)):
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


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    fd = os.open(path, flags)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


__all__ = [
    "CAPTURE_FORMAT_VERSION",
    "CHECKPOINT_TOPIC",
    "CLOCK_EPOCH",
    "CaptureResult",
    "CaptureState",
    "EVENT_TOPIC",
    "McapCaptureConfig",
    "McapCaptureConsumer",
    "conservative_ram_budget",
    "RAW_LIDAR_TOPIC",
    "RUNTIME_TOPIC",
    "TICK_TOPIC",
]
