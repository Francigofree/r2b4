"""Non-owning client of the independent camera/person-detection process.

Raw frame bytes never cross the process boundary.  The parent receives bounded
camera metadata and semantic PersonDetectionSnapshot values only.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from v3.async_capability import TransportSemantics, latest_state_snapshot
from v3.runtime_performance import CpuSet, temporary_current_affinity

from .camera_geometry import CameraGeometryConfig
from .litert_person_detector import LiteRtPersonDetectorConfig
from .vision_media_socket import VisionClient, recv_json_line
from .person_detection import (
    PersonDetectionRuntimeStatus,
    PersonDetectionSnapshot,
)
from .picamera2_camera import (
    CameraEdgeSnapshot,
    CameraRuntimeStatus,
    Picamera2CameraConfig,
)

_READY_TIMEOUT_S = 15.0


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
    calibration_state: str
    calibration_id: str
    rectified_K: tuple[tuple[float, float, float], ...]
    rectification_duration_ns: int
    owner_generation: str = ""

    @property
    def completion_lag_ns(self) -> int:
        return self.completed_monotonic_ns - self.measurement_monotonic_ns


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
            frame.calibration_state,
            frame.calibration_id,
            frame.rectified_K,
            frame.rectification_duration_ns,
            frame.owner_generation,
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
            calibration_state=str(frame_wire[13]),
            calibration_id=str(frame_wire[14]),
            rectified_K=tuple(tuple(float(value) for value in row) for row in frame_wire[15]),
            rectification_duration_ns=int(frame_wire[16]),
            owner_generation=str(frame_wire[17]),
        )
    # CameraEdgeSnapshot performs no runtime frame-type coercion.  The parent
    # intentionally exposes the metadata-only compatible view above.
    return CameraEdgeSnapshot(status=status, frame=frame)  # type: ignore[arg-type]


class ProcessVisionPort:
    """Idle, metadata-only V3 consumer. It never owns a process or camera."""

    transport_semantics = TransportSemantics.LATEST_STATE

    def __init__(self, camera_config: Picamera2CameraConfig, detector_config: LiteRtPersonDetectorConfig | None,
                 *, camera_geometry: CameraGeometryConfig | None = None,
                 worker_cpus: int | CpuSet | None = None, strict_affinity: bool = False,
                 ready_timeout_s: float = _READY_TIMEOUT_S, root: str | Path | None = None,
                 socket_path: str | Path | None = None) -> None:
        self._client = VisionClient(socket_path, root=root, timeout_s=ready_timeout_s)
        self._condition = threading.Condition()
        self._wake = threading.Event()
        self._closed = False
        self._demand = False
        self._revision = 0
        self._conn = None
        self._pid = None
        self._generation = ""
        self._clear()
        # Idle until a mission requests vision. Launch/connect/cold camera work
        # happens here, never synchronously on the 50 Hz control interpreter.
        self._collector = threading.Thread(target=self._collect, name="vision-consumer", daemon=True)
        with temporary_current_affinity(worker_cpus, role="vision-consumer", strict=strict_affinity):
            self._collector.start()

    @property
    def pid(self) -> int | None:
        return self._pid

    def _clear(self, error: str | None = None) -> None:
        self._camera_edge = CameraEdgeSnapshot(CameraRuntimeStatus(False, 0, None, error, None, None), None)
        self._detection = None
        self._detection_status = PersonDetectionRuntimeStatus(False, 0, 0, None, error)
        self._error = error

    def set_person_detection_demand(self, active: bool) -> None:
        if type(active) is not bool:
            raise TypeError("active must be bool")
        with self._condition:
            if self._closed or self._demand == active:
                return
            self._demand = active
            self._revision += 1
            self._generation = ""
            self._clear()
            self._condition.notify_all()
        self._wake.set()

    def _accept(self, value: dict, revision: int) -> None:
        from dataclasses import replace
        from .person_detection import PersonBox, PersonDetection, PersonDetectionProjection
        generation = value["owner_generation"]
        if not isinstance(generation, str) or not generation:
            raise ValueError("VISION_GENERATION_INVALID")
        edge = _unwire_camera(value["camera"])
        frame = edge.frame
        if frame is not None:
            if frame.calibration_state != "CALIBRATED" or not frame.calibration_id:
                raise ValueError("VISION_FRAME_UNCALIBRATED")
            if frame.owner_generation != generation:
                raise ValueError("VISION_GENERATION_MISMATCH")
        raw = value["detection"]
        detection = None
        if raw is not None:
            raw = dict(raw)
            if len(raw["detections"]) > 64:
                raise ValueError("VISION_DETECTIONS_EXCEEDED_BOUND")
            raw["detections"] = tuple(PersonDetection(x["confidence"], PersonBox(**x["box"])) for x in raw["detections"])
            raw["projections"] = tuple(PersonDetectionProjection(**x) for x in raw["projections"])
            detection = PersonDetectionSnapshot(**raw)
            if detection.owner_generation != generation:
                raise ValueError("VISION_GENERATION_MISMATCH")
        status = PersonDetectionRuntimeStatus(**value["detection_status"])
        with self._condition:
            if self._closed or not self._demand or revision != self._revision:
                return
            if self._generation and self._generation != generation:
                raise ValueError("VISION_GENERATION_MISMATCH")
            self._generation = generation
            self._pid = value.get("owner_pid")
            self._camera_edge = edge
            self._detection = detection
            self._detection_status = status
            self._error = None
            self._condition.notify_all()

    def _collect(self) -> None:
        import select
        while not self._closed:
            self._wake.wait(0.1)
            self._wake.clear()
            with self._condition:
                if not self._demand:
                    continue
                revision = self._revision
            conn = None
            try:
                conn = self._client.connect()
                self._conn = conn
                conn.sendall(b"R2B4VISION1 PERSON\n")
                while not self._closed:
                    with self._condition:
                        if not self._demand or revision != self._revision:
                            break
                    readable, _, _ = select.select([conn], [], [], 0.05)
                    if readable:
                        self._accept(recv_json_line(conn), revision)
            except Exception as exc:
                with self._condition:
                    if self._demand and revision == self._revision and not self._closed:
                        self._clear(f"VISION_UNAVAILABLE:{type(exc).__name__}:{exc}"[:256])
                        self._condition.notify_all()
                # A failed active demand is explicit. Do not silently reconnect
                # and replace its generation; the mission must release it.
                while not self._closed:
                    with self._condition:
                        if revision != self._revision or not self._demand:
                            break
                    self._wake.wait(0.1)
                    self._wake.clear()
            finally:
                if conn is not None:
                    conn.close()
                self._conn = None

    def capability_snapshot(self, observed_monotonic_ns: int, *, stale_after_ns: int = 250_000_000):
        edge = self.get_edge_snapshot()
        frame = edge.frame
        return latest_state_snapshot(name="vision.camera", observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=None if frame is None else frame.sequence,
            source_monotonic_ns=None if frame is None else frame.measurement_monotonic_ns,
            stale_after_ns=stale_after_ns, running=edge.status.running, error=edge.status.last_error)

    def detection_capability_snapshot(self, observed_monotonic_ns: int, *, stale_after_ns: int = 400_000_000):
        with self._condition:
            result, status = self._detection, self._detection_status
        return latest_state_snapshot(name="vision.person_detection", observed_monotonic_ns=observed_monotonic_ns,
            source_sequence=None if result is None else result.sequence,
            source_monotonic_ns=None if result is None else result.measurement_monotonic_ns,
            stale_after_ns=stale_after_ns, running=status.running, error=status.last_error)

    def get_edge_snapshot(self) -> CameraEdgeSnapshot:
        with self._condition:
            return self._camera_edge

    def get_runtime_status(self) -> CameraRuntimeStatus:
        return self.get_edge_snapshot().status

    def get_detection_snapshot(self) -> PersonDetectionSnapshot | None:
        with self._condition:
            return self._detection

    def get_detection_status(self) -> PersonDetectionRuntimeStatus:
        with self._condition:
            return self._detection_status

    def wait_for_new_frame(self, after_sequence: int = 0, timeout_s: float = 1.0) -> CameraEdgeSnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._camera_edge.status.frame_sequence <= after_sequence and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            return self._camera_edge

    def wait_for_new_detection(self, after_sequence: int = 0, timeout_s: float = 1.0) -> PersonDetectionSnapshot | None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while (self._detection is None or self._detection.sequence <= after_sequence) and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            return self._detection

    def request_jpeg(self, output: str | Path, *, stream_name: str = "lores") -> bool:
        # Optional photo evidence has its own observation client outside control.
        return False

    def stop(self) -> None:
        import socket
        with self._condition:
            self._demand = False
            self._closed = True
            self._revision += 1
            self._clear()
            self._condition.notify_all()
        self._wake.set()
        conn = self._conn
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._collector.join(timeout=0.2)


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
