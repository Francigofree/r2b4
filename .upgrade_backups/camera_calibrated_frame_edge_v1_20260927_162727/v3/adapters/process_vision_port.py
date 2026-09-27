"""Process-isolated Picamera2 + person-detection owner.

Raw frame bytes never cross the process boundary.  The parent receives bounded
camera metadata and semantic PersonDetectionSnapshot values only.
"""
from __future__ import annotations

import math
import multiprocessing as mp
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from v3.async_capability import TransportSemantics, latest_state_snapshot
from v3.runtime_performance import apply_current_affinity, temporary_current_affinity

from .camera_geometry import CameraGeometryConfig
from .litert_person_detector import LiteRtPersonDetectorConfig, LiteRtSsdPersonDetector
from .vision_media_socket import VisionMediaServer
from .person_detection import (
    NativePersonDetector,
    PersonDetectionRuntimeStatus,
    PersonDetectionSnapshot,
)
from .picamera2_camera import (
    CameraEdgeSnapshot,
    CameraRuntimeStatus,
    NativePicamera2Camera,
    Picamera2CameraConfig,
    default_picamera2_factory,
    raspberry_pi_sensor_timestamp_to_monotonic_ns,
)

_START_METHOD = "spawn"
_STATE_QUEUE_CAPACITY = 2
_COMMAND_QUEUE_CAPACITY = 1
_STATE_HEARTBEAT_NS = 100_000_000
_READY_TIMEOUT_S = 10.0
_STOP_TIMEOUT_S = 3.0


@dataclass(frozen=True, slots=True)
class CameraFrameControlSnapshot:
    sequence: int
    sensor_timestamp_ns: int
    measurement_monotonic_ns: int
    completed_monotonic_ns: int
    exposure_time_ns: int
    frame_duration_ns: int
    width: int
    height: int
    pixel_format: str
    stride_bytes: int
    frame_size_bytes: int
    focus_state: str
    lens_position: float | None

    @property
    def completion_lag_ns(self) -> int:
        return self.completed_monotonic_ns - self.measurement_monotonic_ns


def _put_latest(target: Any, payload: object) -> None:
    try:
        target.put_nowait(payload)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(payload)
    except queue.Full:
        pass


def _wire_camera(edge: CameraEdgeSnapshot) -> tuple[object, ...]:
    status = edge.status
    frame = edge.frame
    status_wire = (
        status.running,
        status.frame_sequence,
        status.frame_age_ns,
        status.last_error,
        status.camera_model,
        status.completion_lag_ns,
    )
    frame_wire = None
    if frame is not None:
        frame_wire = (
            frame.sequence,
            frame.sensor_timestamp_ns,
            frame.measurement_monotonic_ns,
            frame.completed_monotonic_ns,
            frame.exposure_time_ns,
            frame.frame_duration_ns,
            frame.width,
            frame.height,
            frame.pixel_format,
            frame.stride_bytes,
            frame.frame_size_bytes,
            frame.focus_state,
            frame.lens_position,
        )
    return status_wire, frame_wire


def _unwire_camera(value: tuple[object, ...]) -> CameraEdgeSnapshot:
    status_wire, frame_wire = value
    status = CameraRuntimeStatus(
        running=bool(status_wire[0]),
        frame_sequence=int(status_wire[1]),
        frame_age_ns=(None if status_wire[2] is None else int(status_wire[2])),
        last_error=(None if status_wire[3] is None else str(status_wire[3])),
        camera_model=(None if status_wire[4] is None else str(status_wire[4])),
        completion_lag_ns=(None if status_wire[5] is None else int(status_wire[5])),
    )
    frame = None
    if frame_wire is not None:
        frame = CameraFrameControlSnapshot(
            sequence=int(frame_wire[0]),
            sensor_timestamp_ns=int(frame_wire[1]),
            measurement_monotonic_ns=int(frame_wire[2]),
            completed_monotonic_ns=int(frame_wire[3]),
            exposure_time_ns=int(frame_wire[4]),
            frame_duration_ns=int(frame_wire[5]),
            width=int(frame_wire[6]),
            height=int(frame_wire[7]),
            pixel_format=str(frame_wire[8]),
            stride_bytes=int(frame_wire[9]),
            frame_size_bytes=int(frame_wire[10]),
            focus_state=str(frame_wire[11]),
            lens_position=(None if frame_wire[12] is None else float(frame_wire[12])),
        )
    # CameraEdgeSnapshot performs no runtime frame-type coercion.  The parent
    # intentionally exposes the metadata-only compatible view above.
    return CameraEdgeSnapshot(status=status, frame=frame)  # type: ignore[arg-type]


def _vision_process_main(
    camera_config: Picamera2CameraConfig,
    camera_geometry: CameraGeometryConfig | None,
    detector_config: LiteRtPersonDetectorConfig | None,
    state_queue: Any,
    command_queue: Any,
    ready_event: Any,
    stop_event: Any,
    worker_cpu: int | None,
    strict_affinity: bool,
) -> None:
    camera: NativePicamera2Camera | None = None
    detector: NativePersonDetector | None = None
    media_server: VisionMediaServer | None = None
    try:
        if worker_cpu is not None:
            apply_current_affinity(worker_cpu, role="vision-owner-process", strict=strict_affinity)
        camera = NativePicamera2Camera(
            camera_config,
            camera_geometry_config=camera_geometry,
            picamera_factory=default_picamera2_factory,
            sensor_timestamp_mapper=raspberry_pi_sensor_timestamp_to_monotonic_ns,
            monotonic_ns=time.monotonic_ns,
        )
        if not camera.start():
            raise RuntimeError("camera worker did not start")
        # R2B4_ER2_P0_20260925: large JPEG bytes leave directly from the
        # vision owner process. They never re-enter the 50 Hz control interpreter.
        media_server = VisionMediaServer(camera)
        media_server.start()
        if detector_config is not None:
            backend = LiteRtSsdPersonDetector(detector_config)
            detector = NativePersonDetector(
                camera, backend, camera_geometry_config=camera_geometry
            )
            if not detector.start():
                raise RuntimeError("person detector worker did not start")
        edge = camera.get_edge_snapshot()
        detection = detector.get_detection_snapshot() if detector is not None else None
        detection_status = (
            detector.get_detection_status()
            if detector is not None
            else PersonDetectionRuntimeStatus(False, 0, 0, None, None)
        )
        _put_latest(state_queue, ("state", _wire_camera(edge), detection, detection_status))
        ready_event.set()
        last_frame_sequence = edge.status.frame_sequence
        last_detection_sequence = detection_status.result_sequence
        last_status_ns = time.monotonic_ns()
        while not stop_event.is_set():
            while True:
                try:
                    command = command_queue.get_nowait()
                except queue.Empty:
                    break
                if not isinstance(command, tuple) or not command:
                    continue
                if command[0] == "jpeg" and len(command) == 3:
                    try:
                        camera.request_jpeg(Path(str(command[1])), stream_name=str(command[2]))
                    except Exception:
                        pass
            edge = camera.get_edge_snapshot()
            detection = detector.get_detection_snapshot() if detector is not None else None
            detection_status = (
                detector.get_detection_status()
                if detector is not None
                else PersonDetectionRuntimeStatus(False, 0, 0, None, None)
            )
            now_ns = time.monotonic_ns()
            if (
                edge.status.frame_sequence != last_frame_sequence
                or detection_status.result_sequence != last_detection_sequence
                or now_ns - last_status_ns >= _STATE_HEARTBEAT_NS
            ):
                _put_latest(
                    state_queue,
                    ("state", _wire_camera(edge), detection, detection_status),
                )
                last_frame_sequence = edge.status.frame_sequence
                last_detection_sequence = detection_status.result_sequence
                last_status_ns = now_ns
            stop_event.wait(0.010)
    except BaseException as exc:
        _put_latest(state_queue, ("error", type(exc).__name__, str(exc)))
        ready_event.set()
    finally:
        if media_server is not None:
            try:
                media_server.stop()
            except BaseException:
                pass
        if detector is not None:
            try:
                detector.stop()
            except BaseException:
                pass
        if camera is not None:
            try:
                camera.stop()
            except BaseException:
                pass


class ProcessVisionPort:
    """Small parent proxy implementing camera, detection and photo request ports."""

    transport_semantics = TransportSemantics.LATEST_STATE

    __slots__ = (
        "_camera_edge",
        "_closed",
        "_command_queue",
        "_condition",
        "_detection",
        "_detection_status",
        "_fatal_error",
        "_process",
        "_ready_event",
        "_state_queue",
        "_stop_event",
        "_collector",
        "_collector_stop",
    )

    def __init__(
        self,
        camera_config: Picamera2CameraConfig,
        detector_config: LiteRtPersonDetectorConfig | None,
        *,
        camera_geometry: CameraGeometryConfig | None = None,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
        ready_timeout_s: float = _READY_TIMEOUT_S,
    ) -> None:
        if not isinstance(camera_config, Picamera2CameraConfig):
            raise TypeError("camera_config must be Picamera2CameraConfig")
        if detector_config is not None and not isinstance(detector_config, LiteRtPersonDetectorConfig):
            raise TypeError("detector_config must be LiteRtPersonDetectorConfig or None")
        if camera_geometry is not None and not isinstance(camera_geometry, CameraGeometryConfig):
            raise TypeError("camera_geometry must be CameraGeometryConfig or None")
        if worker_cpu is not None and (
            not isinstance(worker_cpu, int) or isinstance(worker_cpu, bool) or worker_cpu < 0
        ):
            raise ValueError("worker_cpu must be non-negative or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        context = mp.get_context(_START_METHOD)
        self._state_queue = context.Queue(maxsize=_STATE_QUEUE_CAPACITY)
        self._command_queue = context.Queue(maxsize=_COMMAND_QUEUE_CAPACITY)
        self._ready_event = context.Event()
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_vision_process_main,
            args=(
                camera_config,
                camera_geometry,
                detector_config,
                self._state_queue,
                self._command_queue,
                self._ready_event,
                self._stop_event,
                worker_cpu,
                strict_affinity,
            ),
            name="v3-vision-owner-process",
            daemon=False,
        )
        self._condition = threading.Condition()
        self._camera_edge = CameraEdgeSnapshot(
            CameraRuntimeStatus(False, 0, None, None, None, None), None
        )
        self._detection: PersonDetectionSnapshot | None = None
        self._detection_status = PersonDetectionRuntimeStatus(False, 0, 0, None, None)
        self._fatal_error = ""
        self._collector_stop = threading.Event()
        self._collector: threading.Thread | None = None
        self._closed = False
        self._process.start()
        if not self._ready_event.wait(float(ready_timeout_s)):
            self.stop()
            raise RuntimeError("process-isolated vision did not become ready")
        try:
            first = self._state_queue.get(timeout=float(ready_timeout_s))
        except queue.Empty:
            first = None
        if first is not None:
            self._apply_message(first)
        self._drain_state()
        if self._fatal_error:
            error = self._fatal_error
            self.stop()
            raise RuntimeError(f"process-isolated vision startup failed: {error}")
        collector = threading.Thread(target=self._collect_state, name="r2b4-vision-ipc", daemon=True)
        self._collector = collector
        with temporary_current_affinity(worker_cpu, role="vision-ipc", strict=strict_affinity):
            collector.start()

    @property
    def pid(self) -> int | None:
        return self._process.pid

    def _set_failed(self, error: str) -> None:
        with self._condition:
            self._fatal_error = error[:256]
            old = self._camera_edge.status
            self._camera_edge = CameraEdgeSnapshot(
                CameraRuntimeStatus(
                    False,
                    old.frame_sequence,
                    old.frame_age_ns,
                    self._fatal_error,
                    old.camera_model,
                    old.completion_lag_ns,
                ),
                self._camera_edge.frame,
            )
            self._detection_status = PersonDetectionRuntimeStatus(
                False,
                self._detection_status.result_sequence,
                self._detection_status.source_frame_sequence,
                self._detection_status.result_age_ns,
                self._fatal_error,
            )
            self._condition.notify_all()

    def _apply_message(self, newest: object) -> None:
        if not isinstance(newest, tuple) or not newest:
            self._set_failed("VISION_TRANSPORT_INVALID")
            return
        if newest[0] == "error":
            self._set_failed(f"{newest[1]}:{newest[2]}")
            return
        if newest[0] != "state" or len(newest) != 4:
            self._set_failed("VISION_TRANSPORT_INVALID")
            return
        edge = _unwire_camera(newest[1])
        detection = newest[2]
        status = newest[3]
        if detection is not None and not isinstance(detection, PersonDetectionSnapshot):
            self._set_failed("VISION_DETECTION_INVALID")
            return
        if not isinstance(status, PersonDetectionRuntimeStatus):
            self._set_failed("VISION_DETECTION_STATUS_INVALID")
            return
        with self._condition:
            if self._fatal_error:
                return
            self._camera_edge = edge
            self._detection = detection
            self._detection_status = status
            self._condition.notify_all()

    def _drain_state(self) -> None:
        for _ in range(_STATE_QUEUE_CAPACITY):
            try:
                message = self._state_queue.get_nowait()
            except queue.Empty:
                break
            # Failure cannot be coalesced away by a later healthy snapshot.
            self._apply_message(message)

    def _collect_state(self) -> None:
        try:
            while not self._collector_stop.is_set():
                try:
                    message = self._state_queue.get(timeout=0.05)
                except queue.Empty:
                    if not self._process.is_alive() and not self._closed:
                        self._set_failed("VISION_OWNER_PROCESS_EXITED")
                        return
                    continue
                self._apply_message(message)
        except BaseException as exc:
            if not self._closed:
                self._set_failed(f"VISION_COLLECTOR_FAILED:{type(exc).__name__}:{exc}")

    def capability_snapshot(
        self,
        observed_monotonic_ns: int,
        *,
        stale_after_ns: int = 250_000_000,
    ):
        with self._condition:
            edge = self._camera_edge
            error = self._fatal_error or edge.status.last_error
            if not self._closed and not self._process.is_alive():
                error = error or "VISION_OWNER_PROCESS_EXITED"
        frame = edge.frame
        return latest_state_snapshot(
            name="vision.camera",
            observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=None if frame is None else int(frame.sequence),
            source_monotonic_ns=(
                None if frame is None else int(frame.measurement_monotonic_ns)
            ),
            stale_after_ns=stale_after_ns,
            running=bool(edge.status.running),
            error=error or None,
        )

    def detection_capability_snapshot(
        self,
        observed_monotonic_ns: int,
        *,
        stale_after_ns: int = 400_000_000,
    ):
        with self._condition:
            detection = self._detection
            status = self._detection_status
            error = self._fatal_error or status.last_error
            if not self._closed and not self._process.is_alive():
                error = error or "VISION_OWNER_PROCESS_EXITED"
        return latest_state_snapshot(
            name="vision.person_detection",
            observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=None if detection is None else int(detection.sequence),
            source_monotonic_ns=(
                None
                if detection is None
                else int(detection.measurement_monotonic_ns)
            ),
            stale_after_ns=stale_after_ns,
            running=bool(status.running),
            error=error or None,
        )

    def get_edge_snapshot(self) -> CameraEdgeSnapshot:
        with self._condition:
            return self._camera_edge

    def get_runtime_status(self) -> CameraRuntimeStatus:
        return self.get_edge_snapshot().status

    def wait_for_new_frame(self, after_sequence: int = 0, timeout_s: float = 1.0) -> CameraEdgeSnapshot:
        if not isinstance(after_sequence, int) or isinstance(after_sequence, bool) or after_sequence < 0:
            raise ValueError("after_sequence must be non-negative")
        if not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("timeout_s must be finite and non-negative")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while self._camera_edge.status.frame_sequence <= after_sequence and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            return self._camera_edge

    def get_detection_snapshot(self) -> PersonDetectionSnapshot | None:
        with self._condition:
            return self._detection

    def get_detection_status(self) -> PersonDetectionRuntimeStatus:
        with self._condition:
            return self._detection_status

    def wait_for_new_detection(self, after_sequence: int = 0, timeout_s: float = 1.0) -> PersonDetectionSnapshot | None:
        if not isinstance(after_sequence, int) or isinstance(after_sequence, bool) or after_sequence < 0:
            raise ValueError("after_sequence must be non-negative")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while (
                (self._detection is None or self._detection.sequence <= after_sequence)
                and not self._closed
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            if self._detection is not None and self._detection.sequence > after_sequence:
                return self._detection
            return None

    def request_jpeg(self, output: str | Path, *, stream_name: str = "lores") -> bool:
        if stream_name not in {"main", "lores"}:
            raise ValueError("stream_name must be main or lores")
        if self._closed or self._fatal_error:
            return False
        try:
            self._command_queue.put_nowait(("jpeg", str(Path(output)), stream_name))
            return True
        except queue.Full:
            return False

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._collector_stop.set()
        self._stop_event.set()
        self._process.join(timeout=_STOP_TIMEOUT_S)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=_STOP_TIMEOUT_S)
        collector = self._collector
        if collector is not None and collector is not threading.current_thread():
            collector.join(timeout=_STOP_TIMEOUT_S)
        for item in (self._state_queue, self._command_queue):
            try:
                item.close()
            except (OSError, ValueError):
                pass
        with self._condition:
            self._condition.notify_all()


class UnavailableVisionPort:
    """Non-critical failed vision capability without raw-frame ownership."""

    __slots__ = ("_edge", "_status")

    def __init__(self, error: str) -> None:
        error = str(error).strip() or "VISION_UNAVAILABLE"
        self._edge = CameraEdgeSnapshot(
            CameraRuntimeStatus(False, 0, None, error, None, None), None
        )
        self._status = PersonDetectionRuntimeStatus(False, 0, 0, None, error)

    def get_edge_snapshot(self) -> CameraEdgeSnapshot:
        return self._edge

    def get_runtime_status(self) -> CameraRuntimeStatus:
        return self._edge.status

    def wait_for_new_frame(self, after_sequence: int = 0, timeout_s: float = 1.0) -> CameraEdgeSnapshot:
        return self._edge

    def get_detection_snapshot(self) -> None:
        return None

    def get_detection_status(self) -> PersonDetectionRuntimeStatus:
        return self._status

    def wait_for_new_detection(self, after_sequence: int = 0, timeout_s: float = 1.0) -> None:
        return None

    def request_jpeg(self, output: str | Path, *, stream_name: str = "lores") -> bool:
        return False

    def stop(self) -> None:
        return None


__all__ = ["CameraFrameControlSnapshot", "ProcessVisionPort", "UnavailableVisionPort"]
