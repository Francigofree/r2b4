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
from v3.runtime_performance import CpuSet, normalize_cpus, apply_process_cpuset, temporary_current_affinity

from .camera_geometry import CameraGeometryConfig
from .litert_person_detector import LiteRtPersonDetectorConfig, LiteRtSsdPersonDetector
from .vision_media_socket import VisionClient, recv_json_line
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
        )
    # CameraEdgeSnapshot performs no runtime frame-type coercion.  The parent
    # intentionally exposes the metadata-only compatible view above.
    return CameraEdgeSnapshot(status=status, frame=frame)  # type: ignore[arg-type]


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
