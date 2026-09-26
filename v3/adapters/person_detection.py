"""Latest-only asynchronous person detection over the native camera frame port.

The detector is deliberately outside the L0-L12 control loop.  It consumes the
camera owner's immutable latest frame, performs bounded inference in one worker,
and publishes one immutable latest semantic snapshot.  Slow inference therefore
drops intermediate camera frames instead of building an unbounded queue.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Protocol

from .camera_geometry import (
    CameraGeometryConfig,
    CameraGeometryStatus,
    effective_geometry,
)


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _unit(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or not 0.0 <= float(value) <= 1.0
    ):
        raise ValueError(f"{name} must be finite and in [0, 1]")
    return float(value)


@dataclass(frozen=True, slots=True)
class PersonBox:
    """Normalized image-space bounding box."""

    ymin: float
    xmin: float
    ymax: float
    xmax: float

    def __post_init__(self) -> None:
        ymin = _unit(self.ymin, "ymin")
        xmin = _unit(self.xmin, "xmin")
        ymax = _unit(self.ymax, "ymax")
        xmax = _unit(self.xmax, "xmax")
        if ymax <= ymin or xmax <= xmin:
            raise ValueError("person bounding box must have positive area")

    @property
    def center_x(self) -> float:
        return (self.xmin + self.xmax) * 0.5

    @property
    def center_y(self) -> float:
        return (self.ymin + self.ymax) * 0.5

    @property
    def area(self) -> float:
        return (self.xmax - self.xmin) * (self.ymax - self.ymin)


@dataclass(frozen=True, slots=True)
class PersonDetection:
    confidence: float
    box: PersonBox

    def __post_init__(self) -> None:
        _unit(self.confidence, "confidence")
        if not isinstance(self.box, PersonBox):
            raise TypeError("box must be PersonBox")


@dataclass(frozen=True, slots=True)
class PersonDetectionProjection:
    """Compact camera-geometry result in the robot base-frame convention."""

    left_bearing_rad: float
    right_bearing_rad: float
    geometry_quality: str

    def __post_init__(self) -> None:
        for value, name in (
            (self.left_bearing_rad, "left_bearing_rad"),
            (self.right_bearing_rad, "right_bearing_rad"),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.geometry_quality not in {"factory_nominal", "empirical", "degraded"}:
            raise ValueError("geometry_quality is invalid")


@dataclass(frozen=True, slots=True)
class PersonDetectionSnapshot:
    """One semantic result tied to exactly one camera frame."""

    sequence: int
    source_frame_sequence: int
    measurement_monotonic_ns: int
    completed_monotonic_ns: int
    inference_duration_ns: int
    detections: tuple[PersonDetection, ...]
    geometry_state: str | None = None
    geometry_reason: str | None = None
    projections: tuple[PersonDetectionProjection, ...] = ()

    def __post_init__(self) -> None:
        _positive_int(self.sequence, "sequence")
        _positive_int(self.source_frame_sequence, "source_frame_sequence")
        _nonnegative_int(self.measurement_monotonic_ns, "measurement_monotonic_ns")
        _nonnegative_int(self.completed_monotonic_ns, "completed_monotonic_ns")
        _nonnegative_int(self.inference_duration_ns, "inference_duration_ns")
        if self.completed_monotonic_ns < self.measurement_monotonic_ns:
            raise ValueError("detection completion cannot precede frame measurement")
        if not isinstance(self.detections, tuple) or not all(
            isinstance(item, PersonDetection) for item in self.detections
        ):
            raise TypeError("detections must be tuple[PersonDetection, ...]")
        if tuple(sorted(self.detections, key=lambda item: item.confidence, reverse=True)) != self.detections:
            raise ValueError("detections must be sorted by descending confidence")
        if self.geometry_state is None:
            if self.geometry_reason is not None or self.projections:
                raise ValueError("legacy detection snapshots cannot carry geometry projection data")
        else:
            if self.geometry_state not in {"VALID", "DEGRADED", "INVALID"}:
                raise ValueError("geometry_state is invalid")
            if self.geometry_reason is not None and not isinstance(self.geometry_reason, str):
                raise TypeError("geometry_reason must be str or None")
            if not isinstance(self.projections, tuple) or not all(
                isinstance(item, PersonDetectionProjection) for item in self.projections
            ):
                raise TypeError("projections must be tuple[PersonDetectionProjection, ...]")
            if self.geometry_state == "INVALID":
                if self.projections:
                    raise ValueError("invalid geometry must not publish projected bearings")
            elif len(self.projections) != len(self.detections):
                raise ValueError("geometry projections must preserve detection ordering")

    @property
    def primary(self) -> PersonDetection | None:
        return self.detections[0] if self.detections else None


@dataclass(frozen=True, slots=True)
class PersonDetectionRuntimeStatus:
    running: bool
    result_sequence: int
    source_frame_sequence: int
    result_age_ns: int | None
    last_error: str | None


class CameraFrameLike(Protocol):
    sequence: int
    measurement_monotonic_ns: int
    width: int
    height: int
    pixel_format: str
    stride_bytes: int
    image_bytes: bytes


class CameraStatusLike(Protocol):
    running: bool
    last_error: str | None


class CameraEdgeLike(Protocol):
    status: CameraStatusLike
    frame: CameraFrameLike | None


class CameraFramePortLike(Protocol):
    def get_edge_snapshot(self) -> CameraEdgeLike: ...

    def wait_for_new_frame(self, after_sequence: int = 0, timeout_s: float = 1.0) -> CameraEdgeLike: ...


class PersonDetectorBackend(Protocol):
    def detect(self, frame: CameraFrameLike) -> tuple[PersonDetection, ...]: ...


class PersonDetectionPort(Protocol):
    def get_detection_snapshot(self) -> PersonDetectionSnapshot | None: ...

    def get_detection_status(self) -> PersonDetectionRuntimeStatus: ...

    def wait_for_new_detection(
        self, after_sequence: int = 0, timeout_s: float = 1.0
    ) -> PersonDetectionSnapshot | None: ...

    def stop(self) -> None: ...


class NativePersonDetector:
    """Own one latest-only person-detection worker over an existing camera owner."""

    __slots__ = (
        "_backend",
        "_camera",
        "_camera_geometry_config",
        "_condition",
        "_last_error",
        "_latest",
        "_lock",
        "_monotonic_ns",
        "_result_sequence",
        "_running",
        "_stop_event",
        "_thread",
        "_worker_poll_s",
        "_stop_join_timeout_s",
    )

    def __init__(
        self,
        camera: CameraFramePortLike,
        backend: PersonDetectorBackend,
        *,
        camera_geometry_config: CameraGeometryConfig | None = None,
        worker_poll_s: float = 0.20,
        stop_join_timeout_s: float = 2.0,
        monotonic_ns=time.monotonic_ns,
    ) -> None:
        if not callable(getattr(camera, "wait_for_new_frame", None)):
            raise TypeError("camera must provide wait_for_new_frame")
        if not callable(getattr(camera, "get_edge_snapshot", None)):
            raise TypeError("camera must provide get_edge_snapshot")
        if not callable(getattr(backend, "detect", None)):
            raise TypeError("backend must provide detect")
        if camera_geometry_config is not None and not isinstance(
            camera_geometry_config, CameraGeometryConfig
        ):
            raise TypeError("camera_geometry_config must be CameraGeometryConfig or None")
        for value, name in (
            (worker_poll_s, "worker_poll_s"),
            (stop_join_timeout_s, "stop_join_timeout_s"),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if not callable(monotonic_ns):
            raise TypeError("monotonic_ns must be callable")
        self._camera = camera
        self._backend = backend
        self._camera_geometry_config = camera_geometry_config
        self._worker_poll_s = float(worker_poll_s)
        self._stop_join_timeout_s = float(stop_join_timeout_s)
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False
        self._latest: PersonDetectionSnapshot | None = None
        self._result_sequence = 0
        self._last_error: str | None = None

    def start(self) -> bool:
        with self._condition:
            if self._running:
                return True
            thread = self._thread
            if thread is not None and thread.is_alive():
                return False
            self._stop_event.clear()
            self._last_error = None
            self._latest = None
            self._result_sequence = 0
            self._running = True
            self._thread = threading.Thread(
                target=self._run,
                name="r2b4-person-detector",
                daemon=True,
            )
            self._thread.start()
            self._condition.notify_all()
            return True

    def stop(self) -> None:
        with self._condition:
            self._stop_event.set()
            thread = self._thread
            self._condition.notify_all()
        if thread is not None and thread.is_alive():
            thread.join(self._stop_join_timeout_s)
        with self._condition:
            thread_alive = thread is not None and thread.is_alive()
            if thread_alive:
                self._last_error = "PERSON_DETECTOR_STOP_TIMEOUT"
            self._running = False
            # Preserve a still-running worker reference.  start() must not be
            # able to create a second detector worker after a bounded stop
            # timeout.  A dead worker may be replaced on the next start.
            if not thread_alive:
                self._thread = None
            self._condition.notify_all()

    def get_detection_snapshot(self) -> PersonDetectionSnapshot | None:
        with self._lock:
            return self._latest

    def get_detection_status(self) -> PersonDetectionRuntimeStatus:
        now = self._monotonic_ns()
        with self._lock:
            latest = self._latest
            return PersonDetectionRuntimeStatus(
                running=self._running,
                result_sequence=self._result_sequence,
                source_frame_sequence=(latest.source_frame_sequence if latest is not None else 0),
                result_age_ns=(
                    max(0, now - latest.measurement_monotonic_ns)
                    if latest is not None
                    else None
                ),
                last_error=self._last_error,
            )

    def wait_for_new_detection(
        self, after_sequence: int = 0, timeout_s: float = 1.0
    ) -> PersonDetectionSnapshot | None:
        _nonnegative_int(after_sequence, "after_sequence")
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s)
            or timeout_s < 0.0
        ):
            raise ValueError("timeout_s must be finite and non-negative")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while True:
                latest = self._latest
                if latest is not None and latest.sequence > after_sequence:
                    return latest
                if not self._running:
                    return latest if latest is not None and latest.sequence > after_sequence else None
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

    def _project_detections(
        self,
        frame: CameraFrameLike,
        detections: tuple[PersonDetection, ...],
    ) -> tuple[str | None, str | None, tuple[PersonDetectionProjection, ...]]:
        config = self._camera_geometry_config
        if config is None:
            return None, None, ()
        try:
            status_getter = getattr(self._camera, "get_camera_geometry_status", None)
            status = status_getter() if callable(status_getter) else None
            if status is not None and not isinstance(status, CameraGeometryStatus):
                return "INVALID", "CAMERA_GEOMETRY_STATUS_INVALID", ()
            if status is not None and status.state == "INVALID":
                return "INVALID", status.reason or "CAMERA_GEOMETRY_INVALID", ()
            runtime_status = status
            state = "DEGRADED" if status is None else status.state
            reason = (
                "CAMERA_GEOMETRY_STATUS_UNAVAILABLE"
                if status is None
                else (status.reason or None)
            )
            geometry = effective_geometry(
                config,
                output_width_px=frame.width,
                output_height_px=frame.height,
                sensor_crop=getattr(frame, "sensor_crop", None),
                runtime_status=runtime_status,
            )
            projection_quality = (
                "degraded" if state == "DEGRADED" else geometry.geometry_quality
            )
            projections: list[PersonDetectionProjection] = []
            for detection in detections:
                v_px = detection.box.center_y * frame.height
                left_ray = geometry.pixel_to_base_ray(detection.box.xmin * frame.width, v_px)
                right_ray = geometry.pixel_to_base_ray(detection.box.xmax * frame.width, v_px)
                projections.append(
                    PersonDetectionProjection(
                        left_bearing_rad=math.atan2(left_ray[1], left_ray[0]),
                        right_bearing_rad=math.atan2(right_ray[1], right_ray[0]),
                        geometry_quality=projection_quality,
                    )
                )
            return state, reason, tuple(projections)
        except Exception as exc:
            # 2D detections stay usable, but geometry-dependent spatial fusion must not.
            return "INVALID", f"{type(exc).__name__}:{exc}", ()

    def _run(self) -> None:
        last_frame_sequence = 0
        try:
            while not self._stop_event.is_set():
                edge = self._camera.wait_for_new_frame(
                    last_frame_sequence,
                    timeout_s=self._worker_poll_s,
                )
                if self._stop_event.is_set():
                    break
                frame = getattr(edge, "frame", None)
                status = getattr(edge, "status", None)
                if frame is None:
                    if status is not None and not getattr(status, "running", True):
                        error = getattr(status, "last_error", None)
                        if error:
                            raise RuntimeError(f"camera unavailable: {error}")
                    continue
                if frame.sequence <= last_frame_sequence:
                    continue
                last_frame_sequence = frame.sequence
                started_ns = self._monotonic_ns()
                detections = self._backend.detect(frame)
                completed_ns = self._monotonic_ns()
                if not isinstance(detections, tuple) or not all(
                    isinstance(item, PersonDetection) for item in detections
                ):
                    raise TypeError("person detector backend returned an invalid result")
                ordered = tuple(sorted(detections, key=lambda item: item.confidence, reverse=True))
                geometry_state, geometry_reason, projections = self._project_detections(
                    frame, ordered
                )
                with self._condition:
                    self._result_sequence += 1
                    self._latest = PersonDetectionSnapshot(
                        sequence=self._result_sequence,
                        source_frame_sequence=frame.sequence,
                        measurement_monotonic_ns=frame.measurement_monotonic_ns,
                        completed_monotonic_ns=completed_ns,
                        inference_duration_ns=max(0, completed_ns - started_ns),
                        detections=ordered,
                        geometry_state=geometry_state,
                        geometry_reason=geometry_reason,
                        projections=projections,
                    )
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._condition.notify_all()
        finally:
            with self._condition:
                self._running = False
                if self._thread is threading.current_thread():
                    self._thread = None
                self._condition.notify_all()


class UnavailablePersonDetectionPort:
    """Explicit failed capability used when an optional backend cannot be built."""

    __slots__ = ("_error",)

    def __init__(self, error: str) -> None:
        if not isinstance(error, str) or not error.strip():
            raise ValueError("error must be a non-empty string")
        self._error = error.strip()

    def get_detection_snapshot(self) -> None:
        return None

    def get_detection_status(self) -> PersonDetectionRuntimeStatus:
        return PersonDetectionRuntimeStatus(False, 0, 0, None, self._error)

    def wait_for_new_detection(self, after_sequence: int = 0, timeout_s: float = 1.0) -> None:
        _nonnegative_int(after_sequence, "after_sequence")
        return None

    def stop(self) -> None:
        return None


__all__ = [
    "NativePersonDetector",
    "PersonBox",
    "PersonDetection",
    "PersonDetectionProjection",
    "PersonDetectionPort",
    "PersonDetectionRuntimeStatus",
    "PersonDetectionSnapshot",
    "PersonDetectorBackend",
    "UnavailablePersonDetectionPort",
]
