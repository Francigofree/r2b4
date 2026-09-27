"""Exclusive physical camera diagnostics using the same native Picamera2 policy.

These helpers are for deliberate photo/video hardware tests while the resident
robot runtime is not owning the camera.  They use Raspberry Pi's supported
Picamera2/libcamera APIs; no legacy PiCamera or OpenCV camera ownership exists.
"""

from __future__ import annotations

import time
from pathlib import Path

from v3.adapters.picamera2_camera import (
    CameraControlsFactory,
    Picamera2CameraConfig,
    Picamera2Factory,
    build_video_configuration,
    default_camera_controls,
    default_picamera2_factory,
)


def _output_path(value: str | Path, suffix: str) -> Path:
    path = Path(value).expanduser()
    if path.is_symlink():
        raise ValueError("camera output path must not be a symlink")
    if path.suffix.lower() != suffix:
        raise ValueError(f"camera output must use {suffix} suffix")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _configure(camera: object, config: Picamera2CameraConfig, controls_factory: CameraControlsFactory) -> str:
    properties = getattr(camera, "camera_properties", None)
    if not isinstance(properties, dict):
        try:
            properties = dict(properties)
        except Exception as exc:
            raise RuntimeError("Picamera2 camera_properties are unavailable") from exc
    model = properties.get("Model")
    if not isinstance(model, str) or not model.strip():
        raise RuntimeError("camera model identity is unavailable")
    if config.expected_model.casefold() not in model.casefold():
        raise RuntimeError(
            f"unexpected camera model {model!r}; expected {config.expected_model!r}"
        )
    camera.configure(build_video_configuration(camera, config))
    controls = controls_factory()
    if controls:
        camera.set_controls(controls)
    return model


def capture_photo(
    output: str | Path,
    config: Picamera2CameraConfig = Picamera2CameraConfig(),
    *,
    warmup_s: float = 2.0,
    picamera_factory: Picamera2Factory = default_picamera2_factory,
    controls_factory: CameraControlsFactory | None = None,
) -> dict[str, object]:
    """Capture one JPEG after a bounded warm-up using the R2B4 camera policy."""

    if warmup_s < 0.0:
        raise ValueError("warmup_s must be non-negative")
    path = _output_path(output, ".jpg")
    camera = picamera_factory(config.camera_index)
    controls = controls_factory or (lambda: default_camera_controls(config))
    started = False
    try:
        model = _configure(camera, config, controls)
        camera.start()
        started = True
        if warmup_s:
            time.sleep(warmup_s)
        capture = getattr(camera, "capture_file", None)
        if not callable(capture):
            raise RuntimeError("Picamera2 capture_file is unavailable")
        metadata = capture(str(path), name=config.stream_name, format="jpeg")
        return {
            "model": model,
            "output": str(path),
            "bytes": path.stat().st_size,
            "metadata": dict(metadata) if isinstance(metadata, dict) else {},
        }
    finally:
        if started:
            try:
                camera.stop()
            except Exception:
                pass
        camera.close()


def capture_h264_video(
    output: str | Path,
    duration_s: float,
    config: Picamera2CameraConfig = Picamera2CameraConfig(),
    *,
    bitrate: int = 4_000_000,
    warmup_s: float = 1.0,
    picamera_factory: Picamera2Factory = default_picamera2_factory,
    controls_factory: CameraControlsFactory | None = None,
) -> dict[str, object]:
    """Record an elementary H.264 stream from the main YUV420 stream."""

    if not isinstance(duration_s, (int, float)) or isinstance(duration_s, bool) or duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if not isinstance(bitrate, int) or isinstance(bitrate, bool) or bitrate <= 0:
        raise ValueError("bitrate must be a positive integer")
    if warmup_s < 0.0:
        raise ValueError("warmup_s must be non-negative")
    path = _output_path(output, ".h264")
    camera = picamera_factory(config.camera_index)
    controls = controls_factory or (lambda: default_camera_controls(config))
    encoder_started = False
    camera_started = False
    try:
        model = _configure(camera, config, controls)
        try:
            from picamera2.encoders import H264Encoder  # type: ignore[import-not-found]
            from picamera2.outputs import FileOutput  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - hardware path
            raise RuntimeError("Picamera2 H.264 encoder support is unavailable") from exc
        encoder = H264Encoder(bitrate=bitrate)
        encoder.output = FileOutput(str(path))
        camera.start()
        camera_started = True
        if warmup_s:
            time.sleep(warmup_s)
        camera.start_encoder(encoder)
        encoder_started = True
        time.sleep(float(duration_s))
        camera.stop_encoder()
        encoder_started = False
        return {
            "model": model,
            "output": str(path),
            "bytes": path.stat().st_size,
            "duration_s": float(duration_s),
            "bitrate": bitrate,
        }
    finally:
        if encoder_started:
            try:
                camera.stop_encoder()
            except Exception:
                pass
        if camera_started:
            try:
                camera.stop()
            except Exception:
                pass
        camera.close()


__all__ = ["capture_h264_video", "capture_photo"]
