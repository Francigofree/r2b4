"""Process-isolated passive runtime observers for the production V3 control path.

The control process may enqueue immutable TickResult/CaptureRecord values, but it
never performs status file I/O, MCAP encoding, hashing, compression, fsync or Test
Hub post-processing.  Sidecars own no command, mission, safety or motor authority.
"""

from __future__ import annotations

import multiprocessing
import queue
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from v3.async_capability import TransportSemantics
from v3.engine import TickResult
from v3.adapters.process_lidar_port import _unwire_raw
from v3.execution import CaptureRecord
from v3.capture_ipc import CaptureCoreExpander
from v3.hri_evidence import HRI_EVENT_TOPIC, load_hri_events_for_capture
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.observation import ObservationHub
from v3.runtime_performance import apply_current_affinity, temporary_current_affinity
from v3.resident_status import (
    RESIDENT_PROCESS_STATUS_SCHEMA,
    _atomic_private_json,
    _tick_status,
)


_SPAWN_METHOD = "spawn"
_CAPTURE_LOCAL_CAPACITY = 4096
_CAPTURE_TRANSPORT_MIN_CAPACITY = 512
_RAW_LIDAR_CAPACITY = 512
_RAW_LIDAR_DRAIN_BATCH = 256
_SIDECAR_READY_TIMEOUT_S = 10.0
_SIDECAR_FINISH_TIMEOUT_S = 120.0

# R2B4_HRI_P0_V1


def _capture_sidecar_main(
    capture_id: str,
    output_path: str,
    configuration: Mapping[str, object],
    metadata: Mapping[str, object],
    config: McapCaptureConfig,
    data_queue: Any,
    raw_lidar_queue: Any,
    control_queue: Any,
    result_queue: Any,
    ready_event: Any,
    failed_event: Any,
    expect_raw_lidar_end: bool,
    project_root: str,
    worker_cpu: int | None,
    strict_affinity: bool,
) -> None:
    try:
        if worker_cpu is not None:
            apply_current_affinity(
                worker_cpu,
                role="observer-sidecar",
                strict=strict_affinity,
            )
        hub = ObservationHub()
        subscription = hub.subscribe_reliable(
            "capture-process",
            capacity=_CAPTURE_LOCAL_CAPACITY,
            required=True,
            topics=(
                "v3.capture_record", "v3.raw_lidar",
                "v3.raw_lidar_transport", "v3.capture_transport", HRI_EVENT_TOPIC,
            ),
        )
        consumer = McapCaptureConsumer(
            capture_id,
            Path(output_path),
            subscription=subscription,
            configuration=configuration,
            metadata=metadata,
            config=config,
        )
        consumer.start()
        capture_started_ns = time.monotonic_ns()
        expander = CaptureCoreExpander()
        ready_event.set()

        processed = 0
        raw_end_received = not expect_raw_lidar_end
        finish_request: tuple[str, bool, int, int] | None = None
        running = True
        while running:
            # The LiDAR producer sends full scans directly here. The control
            # interpreter never unpickles/rebuilds/re-pickles raw geometry.
            for _ in range(_RAW_LIDAR_DRAIN_BATCH):
                try:
                    raw_message = raw_lidar_queue.get_nowait()
                except queue.Empty:
                    break
                if not isinstance(raw_message, tuple) or not raw_message:
                    raise RuntimeError("invalid LiDAR evidence message")
                raw_kind = raw_message[0]
                if raw_kind == "raw":
                    if len(raw_message) < 2:
                        raise RuntimeError("raw LiDAR evidence lacks payload")
                    snapshot = _unwire_raw(raw_message[1])
                    if snapshot is not None:
                        hub.publish(snapshot, topic="v3.raw_lidar")
                elif raw_kind == "raw_end":
                    if len(raw_message) != 4:
                        raise RuntimeError("invalid raw LiDAR end marker")
                    raw_end_received = True
                    hub.publish(
                        {
                            "event_type": "raw_lidar_transport_end",
                            "last_revision": int(raw_message[1]),
                            "produced_count": int(raw_message[2]),
                            "superseded_count": int(raw_message[3]),
                        },
                        topic="v3.raw_lidar_transport",
                    )
                else:
                    raise RuntimeError(f"unknown LiDAR evidence kind: {raw_kind!r}")
            while True:
                try:
                    command = control_queue.get_nowait()
                except queue.Empty:
                    break
                kind = command[0]
                if kind == "trigger":
                    consumer.trigger(command[1], command[2])
                elif kind == "warmup":
                    continue
                elif kind == "finish":
                    finish_request = (
                        str(command[1]), bool(command[2]), int(command[3]), int(command[4])
                    )
                elif kind == "abort":
                    raise RuntimeError("observer sidecar aborted by parent")
                else:
                    raise RuntimeError(f"unknown observer sidecar command: {kind!r}")

            if (
                finish_request is not None
                and processed >= finish_request[2]
            ):
                if finish_request[3]:
                    hub.publish(
                        {"event_type": "capture_core_transport_loss", "drop_count": finish_request[3]},
                        topic="v3.capture_transport",
                    )
                for hri_event in load_hri_events_for_capture(
                    Path(project_root), capture_started_ns, time.monotonic_ns()
                ):
                    hub.publish(hri_event, topic=HRI_EVENT_TOPIC)
                hub.close()
                result = consumer.finish(finish_request[0], terminal=finish_request[1])
                evidence: dict[str, object] | None = None
                result_path: str | None = None
                if result is not None:
                    result_path = str(result.path)
                    from v3.test_hub_runtime import postprocess_capture

                    evidence = postprocess_capture(
                        result.path,
                        project_root=Path(project_root),
                        replay_mode=(
                            "incident" if config.tick_sample_hz == 50 else "off"
                        ),
                    )
                result_queue.put(("done", result_path, evidence))
                running = False
                continue

            try:
                kind, payload = data_queue.get(timeout=0.01)
            except queue.Empty:
                continue
            if kind == "warmup":
                continue
            if kind == "core":
                hub.publish(expander.expand_transport(payload), topic="v3.capture_record")
            elif kind == "record":
                hub.publish(payload, topic="v3.capture_record")
            elif kind == "raw_lidar":
                if payload is not None:
                    hub.publish(payload, topic="v3.raw_lidar")
            else:
                raise RuntimeError(f"unknown observer data kind: {kind!r}")
            processed += 1
    except BaseException as exc:
        failed_event.set()
        try:
            result_queue.put(("error", type(exc).__name__, str(exc)))
        except BaseException:
            pass
        ready_event.set()


def _status_sidecar_main(
    path: str,
    file_mode: int,
    tick_queue: Any,
    control_queue: Any,
    result_queue: Any,
    ready_event: Any,
    failed_event: Any,
    worker_cpu: int | None,
    strict_affinity: bool,
    effective_config=None,
) -> None:
    try:
        if worker_cpu is not None:
            apply_current_affinity(
                worker_cpu,
                role="status-sidecar",
                strict=strict_affinity,
            )
        target = Path(path)
        configuration = ({"effective_config": effective_config.as_dict(), "config_snapshot_id": effective_config.snapshot_id}
                         if effective_config is not None else {})
        _atomic_private_json(
            target,
            {
                "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
                "state": "BOOTING",
                **configuration,
                "monotonic_ns": time.monotonic_ns(),
            },
            file_mode,
        )
        ready_event.set()

        finishing = False
        while not finishing:
            try:
                command = control_queue.get_nowait()
            except queue.Empty:
                command = None
            if command is not None:
                kind = command[0]
                if kind != "finish":
                    raise RuntimeError(f"unknown status sidecar command: {kind!r}")
                report_payload = command[1]
                error_type = command[2]
                error_text = command[3]
                payload: dict[str, object] = {
                    "schema": RESIDENT_PROCESS_STATUS_SCHEMA,
                    "state": "STOPPED" if error_type is None else "ERROR",
                    "monotonic_ns": time.monotonic_ns(),
                    "report": report_payload,
                    "error_type": error_type,
                    "error": error_text,
                }
                _atomic_private_json(target, {**payload, **configuration}, file_mode)
                finishing = True
                continue

            try:
                snapshot = tick_queue.get(timeout=0.02)
            except queue.Empty:
                continue
            if snapshot is None:  # startup-only feeder warmup
                continue
            _atomic_private_json(
                target,
                {**snapshot, **configuration},
                file_mode,
            )
        result_queue.put(("done",))
    except BaseException as exc:
        failed_event.set()
        try:
            result_queue.put(("error", type(exc).__name__, str(exc)))
        except BaseException:
            pass
        ready_event.set()


class ProcessMcapCaptureSession:
    """Production MCAP capture whose expensive work lives outside control process."""

    transport_semantics = TransportSemantics.EVIDENCE_STREAM

    __slots__ = (
        "_capture_id",
        "_config",
        "_configuration",
        "_control_queue",
        "_data_queue",
        "_raw_lidar_queue",
        "_drop_count",
        "_enqueued_count",
        "_failed_event",
        "_metadata",
        "_output_path",
        "_process",
        "_project_root",
        "_ready_event",
        "_result_queue",
        "_started",
        "_strict_affinity",
        "_transport_capacity",
        "_worker_cpu",
        "_expect_raw_lidar_end",
        "evidence",
    )

    def __init__(
        self,
        capture_id: str,
        output_path: str | Path,
        *,
        configuration: Mapping[str, object],
        metadata: Mapping[str, object] | None = None,
        config: McapCaptureConfig | None = None,
        capacity: int = 256,
        project_root: str | Path,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
        expect_raw_lidar_end: bool = False,
    ) -> None:
        identifier = str(capture_id or "").strip()
        if not identifier:
            raise ValueError("capture_id must be non-empty")
        target = Path(output_path)
        if not target.is_absolute() or target.suffix.lower() != ".mcap":
            raise ValueError("output_path must be an absolute .mcap path")
        if not isinstance(configuration, Mapping):
            raise TypeError("configuration must be a mapping")
        if metadata is not None and not isinstance(metadata, Mapping):
            raise TypeError("metadata must be a mapping or None")
        if not isinstance(capacity, int) or isinstance(capacity, bool) or capacity <= 0:
            raise ValueError("capacity must be a positive integer")
        if worker_cpu is not None and (
            not isinstance(worker_cpu, int) or isinstance(worker_cpu, bool) or worker_cpu < 0
        ):
            raise ValueError("worker_cpu must be non-negative or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")

        context = multiprocessing.get_context(_SPAWN_METHOD)
        self._capture_id = identifier
        self._output_path = target
        self._configuration = dict(configuration)
        self._metadata = dict(metadata or {})
        self._config = config or McapCaptureConfig()
        self._transport_capacity = max(_CAPTURE_TRANSPORT_MIN_CAPACITY, capacity * 2)
        self._data_queue = context.Queue(maxsize=self._transport_capacity)
        self._raw_lidar_queue = context.Queue(maxsize=_RAW_LIDAR_CAPACITY)
        self._control_queue = context.Queue(maxsize=16)
        self._result_queue = context.Queue(maxsize=4)
        self._ready_event = context.Event()
        self._failed_event = context.Event()
        self._project_root = Path(project_root)
        self._worker_cpu = worker_cpu
        self._strict_affinity = strict_affinity
        self._expect_raw_lidar_end = bool(expect_raw_lidar_end)
        self._process = context.Process(
            target=_capture_sidecar_main,
            args=(
                self._capture_id,
                str(self._output_path),
                self._configuration,
                self._metadata,
                self._config,
                self._data_queue,
                self._raw_lidar_queue,
                self._control_queue,
                self._result_queue,
                self._ready_event,
                self._failed_event,
                self._expect_raw_lidar_end,
                str(self._project_root),
                self._worker_cpu,
                self._strict_affinity,
            ),
            name="v3-observer-capture",
            daemon=False,
        )
        self._started = False
        self._enqueued_count = 0
        self._drop_count = 0
        self.evidence: dict[str, object] | None = None

    @property
    def raw_lidar_queue(self) -> Any:
        """Spawn-time producer → sidecar evidence lane; never a control input."""
        return self._raw_lidar_queue

    @property
    def raw_lidar_transport_capacity(self) -> int:
        """Bounded burst reserve; overflow remains explicit integrity failure."""
        return _RAW_LIDAR_CAPACITY

    @property
    def failed(self) -> bool:
        return bool(self._failed_event.is_set() or self._drop_count)

    @property
    def transport_drop_count(self) -> int:
        return self._drop_count

    def start(self) -> None:
        if self._started:
            return
        self._process.start()
        self._started = True
        if not self._ready_event.wait(_SIDECAR_READY_TIMEOUT_S):
            self._terminate()
            raise RuntimeError("capture sidecar did not become ready")
        self._raise_early_error()
        if not self._process.is_alive():
            raise RuntimeError("capture sidecar exited during startup")
        # Queue feeders are created by the first put, not by Process.start().
        # Start serialization off the control CPU before accepting observations.
        with temporary_current_affinity(
            self._worker_cpu, role="capture-feeder", strict=self._strict_affinity
        ):
            self._data_queue.put(("warmup", None), timeout=_SIDECAR_READY_TIMEOUT_S)
            self._control_queue.put(("warmup",), timeout=_SIDECAR_READY_TIMEOUT_S)

    def _raise_early_error(self) -> None:
        try:
            message = self._result_queue.get_nowait()
        except queue.Empty:
            return
        if message and message[0] == "error":
            raise RuntimeError(f"capture sidecar failed: {message[1]}:{message[2]}")
        self._result_queue.put(message)

    def _enqueue(self, kind: str, payload: object) -> bool:
        if not self._started:
            self.start()
        try:
            self._data_queue.put_nowait((kind, payload))
        except queue.Full:
            self._drop_count += 1
            return False
        else:
            self._enqueued_count += 1
            return True

    def observe(self, record: CaptureRecord) -> None:
        # Typed immutable handoff only. Recursive encoding/projection belongs
        # to the already isolated capture sidecar.
        self._enqueue("record", record)

    def observe_raw_lidar(self, snapshot: object | None) -> None:
        if snapshot is not None:
            self._enqueue("raw_lidar", snapshot)

    def trigger(self, reason: str = "MANUAL", monotonic_ns: int | None = None) -> None:
        if not self._started:
            self.start()
        try:
            self._control_queue.put_nowait(("trigger", str(reason), monotonic_ns))
        except queue.Full:
            self._drop_count += 1

    def finalize(self, report: object | None, *, error: BaseException | None = None) -> Path | None:
        if not self._started:
            self.start()
        status = "PASS" if error is None and getattr(report, "status", 1) == 0 else "FAULT"
        self._control_queue.put(
            ("finish", status, True, self._enqueued_count, self._drop_count),
            timeout=2.0,
        )
        self._process.join(_SIDECAR_FINISH_TIMEOUT_S)
        if self._process.is_alive():
            self._terminate()
            raise RuntimeError("capture sidecar did not stop")
        try:
            message = self._result_queue.get(timeout=2.0)
        except queue.Empty as exc:
            raise RuntimeError("capture sidecar returned no result") from exc
        if message[0] == "error":
            raise RuntimeError(f"capture sidecar failed: {message[1]}:{message[2]}")
        result_path = Path(message[1]) if message[1] is not None else None
        evidence = message[2]
        if isinstance(evidence, dict):
            self.evidence = dict(evidence)
            if self._drop_count:
                self.evidence["status"] = "FAIL"
                self.evidence["process_transport_drop_count"] = self._drop_count
                self.evidence["process_transport_integrity"] = "FAIL"
        return result_path

    def _terminate(self) -> None:
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)


class ProcessResidentStatusPublisher:
    """Compact status projection followed by sidecar-owned encoding/file I/O."""

    __slots__ = (
        "_config",
        "_control_queue",
        "_drop_count",
        "_failed_event",
        "_process",
        "_ready_event",
        "_result_queue",
        "_started",
        "_strict_affinity",
        "_tick_queue",
        "_worker_cpu",
        "_error_text",
    )

    def __init__(
        self,
        config: object,
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
    ) -> None:
        path = getattr(config, "path", None)
        file_mode = getattr(config, "file_mode", None)
        if not isinstance(path, Path) or not path.is_absolute():
            raise TypeError("config must provide absolute Path path")
        if file_mode != 0o600:
            raise ValueError("status file_mode must be 0o600")
        context = multiprocessing.get_context(_SPAWN_METHOD)
        self._config = config
        self._tick_queue = context.Queue(maxsize=2)
        self._control_queue = context.Queue(maxsize=4)
        self._result_queue = context.Queue(maxsize=4)
        self._ready_event = context.Event()
        self._failed_event = context.Event()
        self._worker_cpu = worker_cpu
        self._strict_affinity = strict_affinity
        self._process = context.Process(
            target=_status_sidecar_main,
            args=(
                str(path),
                int(file_mode),
                self._tick_queue,
                self._control_queue,
                self._result_queue,
                self._ready_event,
                self._failed_event,
                worker_cpu,
                strict_affinity,
                getattr(config, "effective_config", None),
            ),
            name="v3-observer-status",
            daemon=False,
        )
        self._started = False
        self._drop_count = 0
        self._error_text: str | None = None

    @property
    def failed(self) -> bool:
        self._poll_error()
        return bool(self._failed_event.is_set())

    @property
    def error(self) -> BaseException | None:
        self._poll_error()
        return RuntimeError(self._error_text) if self._error_text is not None else None

    @property
    def drop_count(self) -> int:
        return self._drop_count

    def _poll_error(self) -> None:
        if self._error_text is not None:
            return
        while True:
            try:
                message = self._result_queue.get_nowait()
            except queue.Empty:
                return
            if message and message[0] == "error":
                self._error_text = f"{message[1]}:{message[2]}"
                return

    def start(self) -> None:
        if self._started:
            raise RuntimeError("status publisher is already started")
        self._process.start()
        self._started = True
        if not self._ready_event.wait(_SIDECAR_READY_TIMEOUT_S):
            self._process.terminate()
            self._process.join(timeout=2.0)
            raise RuntimeError("status sidecar did not become ready")
        self._poll_error()
        if self._error_text is not None:
            raise RuntimeError(f"status sidecar failed: {self._error_text}")
        with temporary_current_affinity(
            self._worker_cpu, role="status-feeder", strict=self._strict_affinity
        ):
            self._tick_queue.put(None, timeout=_SIDECAR_READY_TIMEOUT_S)

    def publish_tick(self, result: TickResult, ready_for_active: bool = False) -> None:
        if not isinstance(result, TickResult):
            raise TypeError("result must be TickResult")
        if type(ready_for_active) is not bool:
            raise TypeError("ready_for_active must be bool")
        if not self._started:
            raise RuntimeError("status publisher is not started")
        try:
            # Project before IPC: the status consumer needs counts and scalars,
            # not the full L1-L12 graph/costmap serialized by a control-GIL feeder.
            self._tick_queue.put_nowait(_tick_status(result, ready_for_active))
        except queue.Full:
            # Status is explicitly latest/best-effort and has no safety authority.
            self._drop_count += 1

    def finish(self, *, report: object | None = None, error: BaseException | None = None) -> None:
        if not self._started:
            return
        if report is not None and error is not None:
            raise ValueError("finish accepts report or error, not both")
        report_payload = report.as_dict() if report is not None else None
        self._control_queue.put(
            (
                "finish",
                report_payload,
                type(error).__name__ if error is not None else None,
                str(error) if error is not None else None,
            ),
            timeout=2.0,
        )
        self._process.join(timeout=10.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
            raise RuntimeError("status sidecar did not stop")
        self._poll_error()


__all__ = [
    "ProcessMcapCaptureSession",
    "ProcessResidentStatusPublisher",
]
