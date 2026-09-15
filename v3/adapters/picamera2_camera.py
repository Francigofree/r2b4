"""Bounded Picamera2 owner for the R2B4 V3 camera foundation.

This adapter owns only the physical camera session and immutable latest-frame
snapshot. It has no navigation, world-model, safety, motor, capture-format or
vision-model authority.

Picamera2 is imported lazily by ``default_picamera2_factory`` so the production
V3 package remains importable on machines without camera packages installed.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Protocol


class Picamera2Request(Protocol):
    def get_metadata(self) -> Mapping[str, object]: ...

    def make_buffer(self, stream_name: str) -> object: ...

    def release(self) -> None: ...


class Picamera2Device(Protocol):
    def create_video_configuration(self, **kwargs: object) -> object: ...

    def configure(self, configuration: object) -> None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def capture_request(self) -> Picamera2Request: ...

    def close(self) -> None: ...


Picamera2Factory = Callable[[int], Picamera2Device]
SensorTimestampMapper = Callable[[int], int]


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _positive_float(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


@dataclass(frozen=True, slots=True)
class Picamera2CameraConfig:
    """Immutable physical camera acquisition configuration."""

    camera_index: int = 0
    stream_name: str = "main"
    width: int = 640
    height: int = 360
    pixel_format: str = "RGB888"
    fps: float = 15.0
    buffer_count: int = 4
    stop_join_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        _nonnegative_int(self.camera_index, "camera_index")
        _positive_int(self.width, "width")
        _positive_int(self.height, "height")
        _positive_int(self.buffer_count, "buffer_count")
        _positive_float(self.fps, "fps")
        _positive_float(self.stop_join_timeout_s, "stop_join_timeout_s")
        for value, name in (
            (self.stream_name, "stream_name"),
            (self.pixel_format, "pixel_format"),
        ):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")


@dataclass(frozen=True, slots=True)
class CameraFrameSnapshot:
    """One immutable camera frame and its physical timing lineage.

    ``sensor_timestamp_ns`` is retained exactly from Picamera2 metadata.
    ``measurement_monotonic_ns`` is produced by the injected clock mapper and is
    the canonical V3 measurement time. ``completed_monotonic_ns`` is only the
    host completion/copy time.

    The image payload intentionally stays outside DeviceSample/DataField.
    """

    sequence: int
    sensor_timestamp_ns: int
    measurement_monotonic_ns: int
    completed_monotonic_ns: int
    exposure_time_ns: int
    frame_duration_ns: int
    width: int
    height: int
    pixel_format: str
    focus_state: str
    lens_position: float | None
    image_bytes: bytes

    def __post_init__(self) -> None:
        _positive_int(self.sequence, "sequence")
        for value, name in (
            (self.sensor_timestamp_ns, "sensor_timestamp_ns"),
            (self.measurement_monotonic_ns, "measurement_monotonic_ns"),
            (self.completed_monotonic_ns, "completed_monotonic_ns"),
            (self.exposure_time_ns, "exposure_time_ns"),
            (self.frame_duration_ns, "frame_duration_ns"),
        ):
            _nonnegative_int(value, name)
        if self.measurement_monotonic_ns > self.completed_monotonic_ns:
            raise ValueError("camera measurement time cannot follow completion time")
        _positive_int(self.width, "width")
        _positive_int(self.height, "height")
        if not isinstance(self.pixel_format, str) or not self.pixel_format.strip():
            raise ValueError("pixel_format must be a non-empty string")
        if not isinstance(self.focus_state, str) or not self.focus_state.strip():
            raise ValueError("focus_state must be a non-empty string")
        if self.lens_position is not None:
            if (
                isinstance(self.lens_position, bool)
                or not isinstance(self.lens_position, (int, float))
                or not math.isfinite(self.lens_position)
            ):
                raise ValueError("lens_position must be finite or None")
        if not isinstance(self.image_bytes, bytes):
            raise TypeError("image_bytes must be immutable bytes")
        if not self.image_bytes:
            raise ValueError("image_bytes must not be empty")


@dataclass(frozen=True, slots=True)
class CameraRuntimeStatus:
    running: bool
    frame_sequence: int
    frame_age_ns: int | None
    last_error: str | None


class NativePicamera2Camera:
    """Own exactly one Picamera2 session and publish the latest copied frame."""

    __slots__ = (
        "_config",
        "_factory",
        "_last_error",
        "_latest",
        "_lock",
        "_monotonic_ns",
        "_picamera",
        "_running",
        "_sequence",
        "_stop_event",
        "_thread",
        "_timestamp_mapper",
    )

    def __init__(
        self,
        config: Picamera2CameraConfig,
        *,
        picamera_factory: Picamera2Factory,
        sensor_timestamp_mapper: SensorTimestampMapper,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(config, Picamera2CameraConfig):
            raise TypeError("config must be Picamera2CameraConfig")
        for callback, name in (
            (picamera_factory, "picamera_factory"),
            (sensor_timestamp_mapper, "sensor_timestamp_mapper"),
            (monotonic_ns, "monotonic_ns"),
        ):
            if not callable(callback):
                raise TypeError(f"{name} must be callable")
        self._config = config
        self._factory = picamera_factory
        self._timestamp_mapper = sensor_timestamp_mapper
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._picamera: Picamera2Device | None = None
        self._latest: CameraFrameSnapshot | None = None
        self._sequence = 0
        self._running = False
        self._last_error: str | None = None

    @property
    def config(self) -> Picamera2CameraConfig:
        return self._config

    def start(self) -> bool:
        with self._lock:
            if self._running:
                return True
            stale_owner = self._picamera is not None or self._thread is not None
        if stale_owner:
            # A previous acquisition thread may have ended after an exception.
            # Retire its physical handle before a new Picamera2 instance can
            # replace the reference.
            self.stop()
        with self._lock:
            self._last_error = None
            self._stop_event.clear()
        camera: Picamera2Device | None = None
        try:
            camera = self._factory(self._config.camera_index)
            configuration = camera.create_video_configuration(
                main={
                    "size": (self._config.width, self._config.height),
                    "format": self._config.pixel_format,
                },
                controls={"FrameRate": self._config.fps},
                buffer_count=self._config.buffer_count,
            )
            camera.configure(configuration)
            camera.start()
        except Exception as exc:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass
            with self._lock:
                self._last_error = f"{type(exc).__name__}:{exc}"
            return False
        with self._lock:
            self._picamera = camera
            self._running = True
        self._thread = threading.Thread(
            target=self._run,
            name="v3-picamera2-camera",
            daemon=False,
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        with self._lock:
            camera = self._picamera
            was_running = self._running
            self._running = False
            self._stop_event.set()
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
        thread = self._thread
        if thread is not None:
            thread.join(timeout=self._config.stop_join_timeout_s)
            if thread.is_alive():
                raise RuntimeError("Picamera2 acquisition thread did not stop")
        self._thread = None
        with self._lock:
            self._picamera = None
        if camera is not None:
            try:
                camera.close()
            except Exception:
                if was_running:
                    raise

    def get_latest_frame(self) -> CameraFrameSnapshot | None:
        with self._lock:
            return self._latest

    def get_runtime_status(self) -> CameraRuntimeStatus:
        now_ns = self._checked_clock()
        with self._lock:
            latest = self._latest
            return CameraRuntimeStatus(
                running=self._running,
                frame_sequence=latest.sequence if latest is not None else 0,
                frame_age_ns=(
                    max(0, now_ns - latest.measurement_monotonic_ns)
                    if latest is not None
                    else None
                ),
                last_error=self._last_error,
            )

    def capture_once_for_test(self, request: Picamera2Request) -> CameraFrameSnapshot:
        """Exercise the production metadata/buffer closure without a camera thread."""

        return self._snapshot_from_request(request)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                camera = self._picamera
                running = self._running
            if not running or camera is None:
                return
            request: Picamera2Request | None = None
            try:
                request = camera.capture_request()
                snapshot = self._snapshot_from_request(request)
                with self._lock:
                    self._latest = snapshot
                    self._last_error = None
            except Exception as exc:
                if self._stop_event.is_set():
                    return
                with self._lock:
                    self._last_error = f"{type(exc).__name__}:{exc}"
                    self._running = False
                return
            finally:
                if request is not None:
                    try:
                        request.release()
                    except Exception:
                        pass

    def _snapshot_from_request(self, request: Picamera2Request) -> CameraFrameSnapshot:
        metadata = request.get_metadata()
        sensor_timestamp_ns = _nonnegative_int(
            metadata.get("SensorTimestamp"),
            "SensorTimestamp",
        )
        exposure_us = _nonnegative_int(metadata.get("ExposureTime", 0), "ExposureTime")
        frame_duration_us = _nonnegative_int(
            metadata.get("FrameDuration", 0),
            "FrameDuration",
        )
        exposure_time_ns = exposure_us * 1_000
        frame_duration_ns = frame_duration_us * 1_000
        # Keep the documented sensor start-of-frame timestamp as the current
        # physical reference.  Do not invent an exposure/rolling-shutter shift
        # before an R2B4 camera-vs-LiDAR timing calibration has validated one.
        measurement_monotonic_ns = self._timestamp_mapper(sensor_timestamp_ns)
        _nonnegative_int(measurement_monotonic_ns, "mapped camera measurement timestamp")
        completed_monotonic_ns = self._checked_clock()
        raw_buffer = request.make_buffer(self._config.stream_name)
        image_bytes = bytes(raw_buffer)
        lens_raw = metadata.get("LensPosition")
        lens_position = (
            float(lens_raw)
            if isinstance(lens_raw, (int, float)) and not isinstance(lens_raw, bool)
            else None
        )
        focus_state = str(metadata.get("AfState", "UNKNOWN"))
        if not focus_state:
            focus_state = "UNKNOWN"
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
        return CameraFrameSnapshot(
            sequence=sequence,
            sensor_timestamp_ns=sensor_timestamp_ns,
            measurement_monotonic_ns=measurement_monotonic_ns,
            completed_monotonic_ns=completed_monotonic_ns,
            exposure_time_ns=exposure_time_ns,
            frame_duration_ns=frame_duration_ns,
            width=self._config.width,
            height=self._config.height,
            pixel_format=self._config.pixel_format,
            focus_state=focus_state,
            lens_position=lens_position,
            image_bytes=image_bytes,
        )

    def _checked_clock(self) -> int:
        return _nonnegative_int(self._monotonic_ns(), "monotonic_ns")


def default_picamera2_factory(camera_index: int) -> Picamera2Device:
    """Open Picamera2 lazily; deployment owns installation of python3-picamera2."""

    try:
        from picamera2 import Picamera2  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - hardware/deployment path
        raise RuntimeError(
            "Picamera2 is unavailable; install the Raspberry Pi system package "
            "python3-picamera2"
        ) from exc
    return Picamera2(camera_num=camera_index)


__all__ = [
    "CameraFrameSnapshot",
    "CameraRuntimeStatus",
    "NativePicamera2Camera",
    "Picamera2CameraConfig",
    "Picamera2Device",
    "Picamera2Factory",
    "Picamera2Request",
    "SensorTimestampMapper",
    "default_picamera2_factory",
]
