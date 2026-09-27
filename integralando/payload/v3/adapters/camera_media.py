"""Fail-closed compatibility surface for exclusive camera diagnostics.

Direct Picamera2 photo/video capture used to bypass the resident calibrated
camera edge.  After the calibrated-frame cutover this adapter intentionally
publishes no image.  Calibration itself has a separate maintenance-only tool
(`tools/v3_camera_calibrate.py`) and normal image consumers must use the
resident VisionMediaServer canonical calibrated stream.
"""
from __future__ import annotations

from pathlib import Path

from v3.adapters.picamera2_camera import Picamera2CameraConfig


_DISABLED = (
    "direct camera media is disabled: raw Picamera2 output must not bypass the "
    "canonical calibrated vision edge"
)


def capture_photo(
    output: str | Path,
    config: Picamera2CameraConfig = Picamera2CameraConfig(),
    *,
    warmup_s: float = 2.0,
    **_: object,
) -> dict[str, object]:
    del output, config, warmup_s
    raise RuntimeError(_DISABLED)


def capture_h264_video(
    output: str | Path,
    duration_s: float,
    config: Picamera2CameraConfig = Picamera2CameraConfig(),
    *,
    bitrate: int = 4_000_000,
    warmup_s: float = 1.0,
    **_: object,
) -> dict[str, object]:
    del output, duration_s, config, bitrate, warmup_s
    raise RuntimeError(
        _DISABLED
        + "; calibrated high-resolution/H264 video needs an accelerated P1 pipeline"
    )


__all__ = ["capture_h264_video", "capture_photo"]
