"""Robust Picamera2/libcamera owner for the native R2B4 camera edge.

The Raspberry Pi Camera Module 3 is an IMX708 sensor.  Raspberry Pi's supported
software path is the kernel/media driver -> libcamera Raspberry Pi pipeline ->
Picamera2.  This adapter owns exactly one Picamera2 session and publishes one
latest immutable analysis frame.  Large image bytes stay outside DeviceSample.

The default production configuration deliberately uses two streams:
- main: 1280x720 YUV420, suitable for recording/GUI evolution;
- lores: 640x360 RGB888, copied into the R2B4 latest-frame port for perception.

Picamera2's ``queue=False`` is used so a consumer does not receive a hidden
cached request that can be up to one frame period older.  Six camera buffers are
used to absorb normal processing/encoding jitter.  Camera Module 3 continuous
PDAF autofocus is enabled through libcamera controls.
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
    @property
    def camera_properties(self) -> Mapping[str, object]: ...

    def create_video_configuration(self, **kwargs: object) -> object: ...

    def configure(self, configuration: object) -> None: ...

    def set_controls(self, controls: Mapping[str, object]) -> None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def capture_request(self) -> Picamera2Request: ...

    def close(self) -> None: ...


Picamera2Factory = Callable[[int], Picamera2Device]
SensorTimestampMapper = Callable[[int], int]
CameraControlsFactory = Callable[[], Mapping[str, object]]


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


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


@dataclass(frozen=True, slots=True)
class Picamera2CameraConfig:
    """Immutable Camera Module 3 acquisition policy.

    ``width``/``height``/``pixel_format`` describe the frame published to R2B4.
    ``main_*`` describes the parallel high-value stream kept available for
    recording and future GUI use without changing the perception contract.
    """

    camera_index: int = 0
    expected_model: str = "imx708"
    stream_name: str = "lores"
    width: int = 640
    height: int = 360
    pixel_format: str = "RGB888"
    main_width: int = 1280
    main_height: int = 720
    main_pixel_format: str = "YUV420"
    fps: float = 20.0
    buffer_count: int = 6
    queue: bool = False
    continuous_autofocus: bool = True
    max_frame_completion_lag_ns: int = 500_000_000
    stop_join_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        _nonnegative_int(self.camera_index, "camera_index")
        _positive_int(self.width, "width")
        _positive_int(self.height, "height")
        _positive_int(self.main_width, "main_width")
        _positive_int(self.main_height, "main_height")
        _positive_int(self.buffer_count, "buffer_count")
        _positive_float(self.fps, "fps")
        _positive_float(self.stop_join_timeout_s, "stop_join_timeout_s")
        _positive_int(self.max_frame_completion_lag_ns, "max_frame_completion_lag_ns")
        _nonempty_string(self.expected_model, "expected_model")
        _nonempty_string(self.pixel_format, "pixel_format")
        _nonempty_string(self.main_pixel_format, "main_pixel_format")
        if self.stream_name not in {"main", "lores"}:
            raise ValueError("stream_name must be 'main' or 'lores'")
        if type(self.queue) is not bool:
            raise TypeError("queue must be bool")
        if type(self.continuous_autofocus) is not bool:
            raise TypeError("continuous_autofocus must be bool")

    @property
    def published_width(self) -> int:
        return self.main_width if self.stream_name == "main" else self.width

    @property
    def published_height(self) -> int:
        return self.main_height if self.stream_name == "main" else self.height

    @property
    def published_pixel_format(self) -> str:
        return self.main_pixel_format if self.stream_name == "main" else self.pixel_format


def picamera2_camera_config_from_mapping(
    value: Mapping[str, object],
) -> Picamera2CameraConfig:
    """Close one JSON-like camera mapping into the immutable driver config."""

    if not isinstance(value, Mapping):
        raise TypeError("camera config must be a mapping")
    allowed = {
        "enabled",
        "provider",
        "camera_index",
        "expected_model",
        "stream_name",
        "width",
        "height",
        "pixel_format",
        "main_width",
        "main_height",
        "main_pixel_format",
        "fps",
        "buffer_count",
        "queue",
        "continuous_autofocus",
        "max_frame_completion_lag_ns",
        "stop_join_timeout_s",
    }
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError("unknown camera config keys: " + ", ".join(unknown))
    provider = value.get("provider", "picamera2")
    if provider != "picamera2":
        raise ValueError("camera.provider must be picamera2")

    def bool_value(key: str, default: bool) -> bool:
        item = value.get(key, default)
        if type(item) is not bool:
            raise ValueError(f"camera.{key} must be bool")
        return item

    return Picamera2CameraConfig(
        camera_index=_nonnegative_int(value.get("camera_index", 0), "camera.camera_index"),
        expected_model=_nonempty_string(
            value.get("expected_model", "imx708"), "camera.expected_model"
        ),
        stream_name=_nonempty_string(
            value.get("stream_name", "lores"), "camera.stream_name"
        ),
        width=_positive_int(value.get("width", 640), "camera.width"),
        height=_positive_int(value.get("height", 360), "camera.height"),
        pixel_format=_nonempty_string(
            value.get("pixel_format", "RGB888"), "camera.pixel_format"
        ),
        main_width=_positive_int(value.get("main_width", 1280), "camera.main_width"),
        main_height=_positive_int(value.get("main_height", 720), "camera.main_height"),
        main_pixel_format=_nonempty_string(
            value.get("main_pixel_format", "YUV420"), "camera.main_pixel_format"
        ),
        fps=_positive_float(value.get("fps", 20.0), "camera.fps"),
        buffer_count=_positive_int(value.get("buffer_count", 6), "camera.buffer_count"),
        queue=bool_value("queue", False),
        continuous_autofocus=bool_value("continuous_autofocus", True),
        max_frame_completion_lag_ns=_positive_int(
            value.get("max_frame_completion_lag_ns", 500_000_000),
            "camera.max_frame_completion_lag_ns",
        ),
        stop_join_timeout_s=_positive_float(
            value.get("stop_join_timeout_s", 2.0), "camera.stop_join_timeout_s"
        ),
    )


@dataclass(frozen=True, slots=True)
class CameraFrameSnapshot:
    """One immutable latest frame with physical timing lineage."""

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
        _nonempty_string(self.pixel_format, "pixel_format")
        _nonempty_string(self.focus_state, "focus_state")
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

    @property
    def completion_lag_ns(self) -> int:
        return self.completed_monotonic_ns - self.measurement_monotonic_ns


@dataclass(frozen=True, slots=True)
class CameraRuntimeStatus:
    running: bool
    frame_sequence: int
    frame_age_ns: int | None
    last_error: str | None
    camera_model: str | None = None
    completion_lag_ns: int | None = None


@dataclass(frozen=True, slots=True)
class CameraEdgeSnapshot:
    """One coherent read of camera status and its corresponding latest frame."""

    status: CameraRuntimeStatus
    frame: CameraFrameSnapshot | None


class CameraFramePort(Protocol):
    """Stable latest-frame surface for GUI, diagnostics and future vision."""

    def get_edge_snapshot(self) -> CameraEdgeSnapshot: ...

    def wait_for_new_frame(
        self, after_sequence: int = 0, timeout_s: float = 1.0
    ) -> CameraEdgeSnapshot: ...


class NativePicamera2Camera:
    """Own one Picamera2 session and one latest-frame publication point."""

    __slots__ = (
        "_config",
        "_controls_factory",
        "_factory",
        "_frame_condition",
        "_last_error",
        "_last_sensor_timestamp_ns",
        "_latest",
        "_lock",
        "_model",
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
        camera_controls_factory: CameraControlsFactory | None = None,
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
        if camera_controls_factory is not None and not callable(camera_controls_factory):
            raise TypeError("camera_controls_factory must be callable or None")
        self._config = config
        self._factory = picamera_factory
        self._timestamp_mapper = sensor_timestamp_mapper
        self._controls_factory = camera_controls_factory
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._frame_condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._picamera: Picamera2Device | None = None
        self._latest: CameraFrameSnapshot | None = None
        self._sequence = 0
        self._last_sensor_timestamp_ns: int | None = None
        self._running = False
        self._last_error: str | None = None
        self._model: str | None = None

    @property
    def config(self) -> Picamera2CameraConfig:
        return self._config

    def start(self) -> bool:
        with self._lock:
            if self._running:
                return True
            stale_owner = self._picamera is not None or self._thread is not None
        if stale_owner:
            self.stop()

        with self._frame_condition:
            # Never allow a frame from an earlier camera session to masquerade
            # as the first frame of a restarted session.
            self._latest = None
            self._last_sensor_timestamp_ns = None
            self._last_error = None
            self._model = None
            self._stop_event.clear()
            self._frame_condition.notify_all()

        camera: Picamera2Device | None = None
        try:
            camera = self._factory(self._config.camera_index)
            model = _camera_model(camera)
            if self._config.expected_model.casefold() not in model.casefold():
                raise RuntimeError(
                    f"unexpected camera model {model!r}; expected "
                    f"{self._config.expected_model!r}"
                )
            configuration = build_video_configuration(camera, self._config)
            camera.configure(configuration)
            controls = (
                self._controls_factory()
                if self._controls_factory is not None
                else default_camera_controls(self._config)
            )
            if controls:
                camera.set_controls(controls)
            camera.start()
        except Exception as exc:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass
            with self._frame_condition:
                self._last_error = f"{type(exc).__name__}:{exc}"
                self._running = False
                self._frame_condition.notify_all()
            return False

        with self._frame_condition:
            self._picamera = camera
            self._model = model
            self._running = True
            self._frame_condition.notify_all()
        thread = threading.Thread(
            target=self._run,
            name="v3-picamera2-camera",
            daemon=False,
        )
        self._thread = thread
        thread.start()
        return True

    def stop(self) -> None:
        with self._frame_condition:
            camera = self._picamera
            was_running = self._running
            self._running = False
            self._stop_event.set()
            self._frame_condition.notify_all()
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
        with self._frame_condition:
            self._picamera = None
            self._frame_condition.notify_all()
        if camera is not None:
            try:
                camera.close()
            except Exception:
                if was_running:
                    raise

    def get_edge_snapshot(self) -> CameraEdgeSnapshot:
        now_ns = self._checked_clock()
        with self._lock:
            return self._edge_snapshot_locked(now_ns)

    def get_latest_frame(self) -> CameraFrameSnapshot | None:
        return self.get_edge_snapshot().frame

    def get_runtime_status(self) -> CameraRuntimeStatus:
        return self.get_edge_snapshot().status

    def wait_for_new_frame(
        self,
        after_sequence: int = 0,
        timeout_s: float = 1.0,
    ) -> CameraEdgeSnapshot:
        _nonnegative_int(after_sequence, "after_sequence")
        _positive_float(timeout_s, "timeout_s")
        deadline = time.monotonic() + timeout_s
        with self._frame_condition:
            while True:
                latest = self._latest
                if latest is not None and latest.sequence > after_sequence:
                    return self._edge_snapshot_locked(self._checked_clock())
                if not self._running and self._last_error is not None:
                    return self._edge_snapshot_locked(self._checked_clock())
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._edge_snapshot_locked(self._checked_clock())
                self._frame_condition.wait(remaining)

    def capture_once_for_test(self, request: Picamera2Request) -> CameraFrameSnapshot:
        return self._snapshot_from_request(request)

    def _edge_snapshot_locked(self, now_ns: int) -> CameraEdgeSnapshot:
        latest = self._latest
        status = CameraRuntimeStatus(
            running=self._running,
            frame_sequence=latest.sequence if latest is not None else 0,
            frame_age_ns=(
                max(0, now_ns - latest.measurement_monotonic_ns)
                if latest is not None
                else None
            ),
            last_error=self._last_error,
            camera_model=self._model,
            completion_lag_ns=(latest.completion_lag_ns if latest is not None else None),
        )
        return CameraEdgeSnapshot(status, latest)

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
                with self._frame_condition:
                    self._latest = snapshot
                    self._last_error = None
                    self._frame_condition.notify_all()
            except Exception as exc:
                if self._stop_event.is_set():
                    return
                with self._frame_condition:
                    self._last_error = f"{type(exc).__name__}:{exc}"
                    self._running = False
                    self._frame_condition.notify_all()
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
        measurement_monotonic_ns = self._timestamp_mapper(sensor_timestamp_ns)
        _nonnegative_int(measurement_monotonic_ns, "mapped camera measurement timestamp")

        raw_buffer = request.make_buffer(self._config.stream_name)
        image_bytes = bytes(raw_buffer)
        completed_monotonic_ns = self._checked_clock()
        if measurement_monotonic_ns > completed_monotonic_ns:
            raise ValueError("mapped camera timestamp is in the future")
        lag_ns = completed_monotonic_ns - measurement_monotonic_ns
        if lag_ns > self._config.max_frame_completion_lag_ns:
            raise RuntimeError("camera frame completion lag exceeded configured bound")

        lens_raw = metadata.get("LensPosition")
        lens_position = (
            float(lens_raw)
            if isinstance(lens_raw, (int, float)) and not isinstance(lens_raw, bool)
            else None
        )
        focus_state = str(metadata.get("AfState", "UNKNOWN")) or "UNKNOWN"
        with self._lock:
            previous_sensor_ns = self._last_sensor_timestamp_ns
            if previous_sensor_ns is not None and sensor_timestamp_ns <= previous_sensor_ns:
                raise RuntimeError("camera SensorTimestamp did not increase")
            self._last_sensor_timestamp_ns = sensor_timestamp_ns
            self._sequence += 1
            sequence = self._sequence

        return CameraFrameSnapshot(
            sequence=sequence,
            sensor_timestamp_ns=sensor_timestamp_ns,
            measurement_monotonic_ns=measurement_monotonic_ns,
            completed_monotonic_ns=completed_monotonic_ns,
            exposure_time_ns=exposure_us * 1_000,
            frame_duration_ns=frame_duration_us * 1_000,
            width=self._config.published_width,
            height=self._config.published_height,
            pixel_format=self._config.published_pixel_format,
            focus_state=focus_state,
            lens_position=lens_position,
            image_bytes=image_bytes,
        )

    def _checked_clock(self) -> int:
        return _nonnegative_int(self._monotonic_ns(), "monotonic_ns")


def _camera_model(camera: Picamera2Device) -> str:
    properties = getattr(camera, "camera_properties", None)
    if not isinstance(properties, Mapping):
        raise RuntimeError("Picamera2 camera_properties are unavailable")
    return _nonempty_string(properties.get("Model"), "camera_properties.Model")


def build_video_configuration(
    camera: Picamera2Device,
    config: Picamera2CameraConfig,
) -> object:
    """Build the single R2B4 two-stream configuration authority."""

    return camera.create_video_configuration(
        main={
            "size": (config.main_width, config.main_height),
            "format": config.main_pixel_format,
        },
        lores={
            "size": (config.width, config.height),
            "format": config.pixel_format,
        },
        controls={"FrameRate": config.fps},
        buffer_count=config.buffer_count,
        queue=config.queue,
    )


def default_camera_controls(config: Picamera2CameraConfig) -> Mapping[str, object]:
    """Return Camera Module 3 controls using libcamera's typed enums."""

    if not config.continuous_autofocus:
        return {}
    try:
        from libcamera import controls  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - hardware/deployment path
        raise RuntimeError("libcamera Python controls are unavailable") from exc
    return {
        "AfMode": controls.AfModeEnum.Continuous,
        "AfSpeed": controls.AfSpeedEnum.Normal,
    }


def raspberry_pi_sensor_timestamp_to_monotonic_ns(sensor_timestamp_ns: int) -> int:
    """Map Picamera2 SensorTimestamp to the R2B4 monotonic domain.

    Raspberry Pi documents SensorTimestamp as nanoseconds since system boot,
    sampled when the first pixel is read out.  Picamera2 itself uses
    ``time.monotonic_ns()`` for flush-time comparisons, so the supported Pi
    camera stack exposes this timestamp in the same monotonic boot-time domain.
    The mapping is therefore intentionally identity, with runtime future/lag
    validation performed for every frame.
    """

    return _nonnegative_int(sensor_timestamp_ns, "sensor_timestamp_ns")


def default_picamera2_factory(camera_index: int) -> Picamera2Device:
    """Open Picamera2 lazily; deployment owns python3-picamera2 installation."""

    try:
        from picamera2 import Picamera2  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - hardware/deployment path
        raise RuntimeError(
            "Picamera2 is unavailable; install the Raspberry Pi system package "
            "python3-picamera2"
        ) from exc
    return Picamera2(camera_num=camera_index)


__all__ = [
    "CameraControlsFactory",
    "CameraEdgeSnapshot",
    "CameraFramePort",
    "CameraFrameSnapshot",
    "CameraRuntimeStatus",
    "NativePicamera2Camera",
    "Picamera2CameraConfig",
    "Picamera2Device",
    "Picamera2Factory",
    "Picamera2Request",
    "SensorTimestampMapper",
    "build_video_configuration",
    "default_camera_controls",
    "default_picamera2_factory",
    "picamera2_camera_config_from_mapping",
    "raspberry_pi_sensor_timestamp_to_monotonic_ns",
]
