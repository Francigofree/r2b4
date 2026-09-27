"""Mandatory calibrated-image edge for R2B4 camera consumers.

Raw Picamera2 pixels are an implementation detail of the camera owner. This
module converts one packed RGB/BGR frame into the canonical calibrated pixel
plane using an empirical K/D model. OpenCV rectification maps are computed once
per (crop, size, stride) geometry and reused; per-frame work is one native
``cv2.remap`` call on the low-resolution canonical stream.
"""
from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass

import numpy as np

from .camera_geometry import (
    CameraEffectiveGeometry,
    CameraGeometryConfig,
    CameraGeometryStatus,
    SensorCrop,
    effective_geometry,
)


class CameraCalibrationError(RuntimeError):
    """The camera frame cannot satisfy the calibrated-image contract."""


@dataclass(frozen=True, slots=True)
class CameraRectificationResult:
    image_bytes: bytes
    calibration_id: str
    rectified_K: tuple[tuple[float, float, float], ...]
    rectification_duration_ns: int


@dataclass(frozen=True, slots=True)
class CameraCalibratedJpegResult:
    jpeg_bytes: bytes
    calibration_id: str
    rectified_K: tuple[tuple[float, float, float], ...]
    rectification_duration_ns: int
    jpeg_encode_duration_ns: int


@dataclass(frozen=True, slots=True)
class _RectificationMap:
    map_x: np.ndarray
    map_y: np.ndarray
    calibration_id: str
    rectified_K: tuple[tuple[float, float, float], ...]


class CameraRectifier:
    """Cached empirical lens rectifier for packed RGB888/BGR888 frames."""

    __slots__ = ("_config", "_maps", "_cv2")

    def __init__(self, config: CameraGeometryConfig) -> None:
        if not isinstance(config, CameraGeometryConfig):
            raise TypeError("config must be CameraGeometryConfig")
        intrinsic = config.intrinsic
        if intrinsic.source != "empirical" or not intrinsic.can_rectify:
            raise CameraCalibrationError(
                "camera requires empirical K/D calibration before image publication"
            )
        if intrinsic.distortion_model not in {"opencv_pinhole", "opencv_fisheye"}:
            raise CameraCalibrationError(
                f"unsupported distortion model: {intrinsic.distortion_model}"
            )
        coeffs = intrinsic.distortion_coefficients
        if coeffs is None:
            raise CameraCalibrationError("distortion coefficients are unavailable")
        if intrinsic.distortion_model == "opencv_pinhole" and len(coeffs) not in {4, 5, 8}:
            raise CameraCalibrationError(
                "opencv_pinhole requires 4, 5 or 8 distortion coefficients"
            )
        if intrinsic.distortion_model == "opencv_fisheye" and len(coeffs) != 4:
            raise CameraCalibrationError("opencv_fisheye requires 4 coefficients")
        try:
            import cv2  # type: ignore[import-not-found]
        except ImportError as exc:
            raise CameraCalibrationError(
                "calibrated camera runtime requires OpenCV; install python3-opencv"
            ) from exc
        self._config = config
        self._maps: dict[tuple[int, int, int, SensorCrop], _RectificationMap] = {}
        self._cv2 = cv2

    @property
    def cached_map_count(self) -> int:
        return len(self._maps)

    def rectify_packed(
        self,
        image_bytes: bytes,
        *,
        width: int,
        height: int,
        pixel_format: str,
        stride_bytes: int,
        frame_size_bytes: int,
        sensor_crop: SensorCrop | None,
        runtime_status: CameraGeometryStatus | None,
        lens_position: float | None,
    ) -> CameraRectificationResult:
        if pixel_format not in {"RGB888", "BGR888"}:
            raise CameraCalibrationError(
                f"canonical rectifier only supports packed RGB888/BGR888, got {pixel_format}"
            )
        if width <= 0 or height <= 0 or stride_bytes < width * 3:
            raise CameraCalibrationError("invalid packed camera geometry")
        if frame_size_bytes != len(image_bytes):
            raise CameraCalibrationError("camera payload size changed before rectification")
        if frame_size_bytes < stride_bytes * height:
            raise CameraCalibrationError("camera frame does not contain all packed rows")
        if runtime_status is not None and runtime_status.state == "INVALID":
            raise CameraCalibrationError(
                f"camera geometry is invalid: {runtime_status.reason or 'unknown'}"
            )
        self._validate_lens_position(lens_position)

        geometry = effective_geometry(
            self._config,
            output_width_px=width,
            output_height_px=height,
            sensor_crop=sensor_crop,
            runtime_status=runtime_status,
        )
        crop = geometry.sensor_crop
        key = (width, height, stride_bytes, crop)
        rectification_map = self._maps.get(key)
        if rectification_map is None:
            rectification_map = self._build_map(geometry)
            self._maps[key] = rectification_map

        raw = np.frombuffer(image_bytes, dtype=np.uint8)
        rows = raw[: stride_bytes * height].reshape(height, stride_bytes)
        source = rows[:, : width * 3].reshape(height, width, 3)
        started_ns = time.monotonic_ns()
        output = self._cv2.remap(
            source,
            rectification_map.map_x,
            rectification_map.map_y,
            interpolation=self._cv2.INTER_LINEAR,
            borderMode=self._cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        duration_ns = max(0, time.monotonic_ns() - started_ns)
        if output.shape != (height, width, 3) or output.dtype != np.uint8:
            raise CameraCalibrationError("OpenCV rectifier returned an invalid frame")

        # Preserve libcamera row padding. The caller copies this calibrated
        # payload back into the still-live request, so request.save() also sees
        # calibrated pixels rather than the private raw frame.
        payload = bytearray(image_bytes)
        target_rows = np.frombuffer(payload, dtype=np.uint8)[: stride_bytes * height].reshape(
            height, stride_bytes
        )
        target_rows[:, : width * 3] = output.reshape(height, width * 3)
        return CameraRectificationResult(
            image_bytes=bytes(payload),
            calibration_id=rectification_map.calibration_id,
            rectified_K=rectification_map.rectified_K,
            rectification_duration_ns=duration_ns,
        )


    def rectify_yuv420_to_jpeg(
        self,
        image: np.ndarray,
        *,
        width: int,
        height: int,
        stride_bytes: int,
        sensor_crop: SensorCrop | None,
        runtime_status: CameraGeometryStatus | None,
        lens_position: float | None,
        jpeg_quality: int = 90,
    ) -> CameraCalibratedJpegResult:
        """Rectify one private Picamera2 YUV420 main frame only on demand.

        Picamera2 exposes YUV420 as a ``height * 3 / 2`` by ``stride`` array.
        We repack its Y/U/V planes into contiguous I420, convert to BGR, remap
        with the same cached empirical geometry used by the canonical lores
        stream, and encode the calibrated image. Raw main pixels never leave
        this method.
        """
        if width <= 0 or height <= 0 or width % 2 or height % 2:
            raise CameraCalibrationError("YUV420 dimensions must be positive and even")
        if stride_bytes < width:
            raise CameraCalibrationError("YUV420 stride is smaller than image width")
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise CameraCalibrationError("YUV420 camera image must be a uint8 numpy array")
        if image.shape != (height * 3 // 2, stride_bytes):
            raise CameraCalibrationError(
                f"unexpected YUV420 mapped shape {image.shape!r}; "
                f"expected {(height * 3 // 2, stride_bytes)!r}"
            )
        if runtime_status is not None and runtime_status.state == "INVALID":
            raise CameraCalibrationError(
                f"camera geometry is invalid: {runtime_status.reason or 'unknown'}"
            )
        if not isinstance(jpeg_quality, int) or isinstance(jpeg_quality, bool) or not 1 <= jpeg_quality <= 100:
            raise CameraCalibrationError("jpeg_quality must be an integer in [1, 100]")
        self._validate_lens_position(lens_position)

        geometry = effective_geometry(
            self._config,
            output_width_px=width,
            output_height_px=height,
            sensor_crop=sensor_crop,
            runtime_status=runtime_status,
        )
        crop = geometry.sensor_crop
        key = (width, height, stride_bytes, crop)
        rectification_map = self._maps.get(key)
        if rectification_map is None:
            rectification_map = self._build_map(geometry)
            self._maps[key] = rectification_map

        # Match Picamera2's documented YUV420 plane layout even when stride
        # exceeds image width. This avoids treating right-edge padding as pixels.
        y_plane = image[:height, :width]
        reshaped = image.reshape((image.shape[0] * 2, image.strides[0] // 2))
        u_plane = reshaped[2 * height : 2 * height + height // 2, : width // 2]
        v_plane = reshaped[2 * height + height // 2 :, : width // 2]
        i420 = np.empty(width * height * 3 // 2, dtype=np.uint8)
        y_size = width * height
        uv_size = y_size // 4
        i420[:y_size] = y_plane.reshape(-1)
        i420[y_size : y_size + uv_size] = u_plane.reshape(-1)
        i420[y_size + uv_size :] = v_plane.reshape(-1)
        i420 = i420.reshape(height * 3 // 2, width)
        bgr = self._cv2.cvtColor(i420, self._cv2.COLOR_YUV2BGR_I420)

        started_ns = time.monotonic_ns()
        calibrated = self._cv2.remap(
            bgr,
            rectification_map.map_x,
            rectification_map.map_y,
            interpolation=self._cv2.INTER_LINEAR,
            borderMode=self._cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        rectification_duration_ns = max(0, time.monotonic_ns() - started_ns)
        encode_started_ns = time.monotonic_ns()
        ok, encoded = self._cv2.imencode(
            ".jpg",
            calibrated,
            [int(self._cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
        )
        jpeg_encode_duration_ns = max(0, time.monotonic_ns() - encode_started_ns)
        if not ok:
            raise CameraCalibrationError("OpenCV failed to encode calibrated JPEG")
        payload = bytes(encoded)
        if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
            raise CameraCalibrationError("calibrated JPEG encoder returned an invalid payload")
        return CameraCalibratedJpegResult(
            jpeg_bytes=payload,
            calibration_id=rectification_map.calibration_id,
            rectified_K=rectification_map.rectified_K,
            rectification_duration_ns=rectification_duration_ns,
            jpeg_encode_duration_ns=jpeg_encode_duration_ns,
        )

    def _validate_lens_position(self, lens_position: float | None) -> None:
        intrinsic = self._config.intrinsic
        reference = intrinsic.reference_lens_position
        tolerance = intrinsic.lens_position_tolerance
        if reference is None:
            return
        if lens_position is None or not math.isfinite(lens_position):
            raise CameraCalibrationError("camera LensPosition is unavailable")
        if tolerance is None:
            if abs(float(lens_position) - reference) > 1e-6:
                raise CameraCalibrationError("camera LensPosition differs from calibration")
            return
        if abs(float(lens_position) - reference) > tolerance:
            raise CameraCalibrationError(
                "camera LensPosition is outside empirical calibration tolerance"
            )

    def _build_map(self, geometry: CameraEffectiveGeometry) -> _RectificationMap:
        cv2 = self._cv2
        intrinsic = self._config.intrinsic
        K = np.asarray(geometry.K, dtype=np.float64)
        D = np.asarray(intrinsic.distortion_coefficients, dtype=np.float64)
        size = (geometry.output_width_px, geometry.output_height_px)
        identity = np.eye(3, dtype=np.float64)
        if intrinsic.distortion_model == "opencv_fisheye":
            map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
                K,
                D.reshape(-1, 1),
                identity,
                K,
                size,
                cv2.CV_32FC1,
            )
        else:
            map_x, map_y = cv2.initUndistortRectifyMap(
                K,
                D,
                identity,
                K,
                size,
                cv2.CV_32FC1,
            )
        if map_x.shape != (size[1], size[0]) or map_y.shape != map_x.shape:
            raise CameraCalibrationError("OpenCV rectification map has invalid geometry")
        return _RectificationMap(
            map_x=map_x,
            map_y=map_y,
            calibration_id=self._calibration_id(geometry),
            rectified_K=geometry.K,
        )

    def _calibration_id(self, geometry: CameraEffectiveGeometry) -> str:
        intrinsic = self._config.intrinsic
        payload = {
            "source": intrinsic.source,
            "reference_size": [
                intrinsic.reference_width_px,
                intrinsic.reference_height_px,
            ],
            "K": [
                intrinsic.fx_px,
                intrinsic.fy_px,
                intrinsic.cx_px,
                intrinsic.cy_px,
            ],
            "model": intrinsic.distortion_model,
            "D": list(intrinsic.distortion_coefficients or ()),
            "lens": intrinsic.reference_lens_position,
            "crop": [
                geometry.sensor_crop.x,
                geometry.sensor_crop.y,
                geometry.sensor_crop.width,
                geometry.sensor_crop.height,
            ],
            "output": [geometry.output_width_px, geometry.output_height_px],
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return "cal-" + hashlib.sha256(encoded).hexdigest()[:16]


__all__ = [
    "CameraCalibratedJpegResult",
    "CameraCalibrationError",
    "CameraRectificationResult",
    "CameraRectifier",
]
