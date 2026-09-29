from __future__ import annotations

import numpy as np
import pytest

from v3.adapters.camera_geometry import (
    CameraGeometryStatus,
    SensorCrop,
    camera_geometry_config_from_mapping,
)
from v3.adapters.camera_rectification import CameraCalibrationError, CameraRectifier


def _empirical(*, coefficients=(0.0, 0.0, 0.0, 0.0, 0.0), lens=1.0):
    return camera_geometry_config_from_mapping(
        {
            "factory_profile": "imx708_wide_noir",
            "mount": {
                "x_m": 0.05,
                "y_m": 0.0,
                "z_m": 0.19,
                "roll_deg": 0.0,
                "pitch_deg": 15.0,
                "yaw_deg": 0.0,
            },
            "intrinsic": {
                "source": "empirical",
                "reference_size": [4608, 2592],
                "K": [
                    [2400.0, 0.0, 2304.0],
                    [0.0, 2400.0, 1296.0],
                    [0.0, 0.0, 1.0],
                ],
                "distortion_model": "opencv_pinhole",
                "distortion_coefficients": list(coefficients),
                "reference_lens_position": lens,
                "lens_position_tolerance": 0.05,
            },
        }
    )


def _nominal():
    return camera_geometry_config_from_mapping(
        {
            "factory_profile": "imx708_wide_noir",
            "mount": {
                "x_m": 0.05,
                "y_m": 0.0,
                "z_m": 0.19,
                "roll_deg": 0.0,
                "pitch_deg": 15.0,
                "yaw_deg": 0.0,
            },
            "intrinsic": {
                "source": "factory_nominal",
                "distortion_model": "unknown",
                "distortion_coefficients": None,
            },
        }
    )


def _status():
    return CameraGeometryStatus(
        state="VALID",
        reason="",
        camera_model="imx708",
        pixel_array_size=(4608, 2592),
        unit_cell_size_nm=(1400, 1400),
        rotation="0",
        sensor_crop=SensorCrop(0, 0, 4608, 2592),
    )


def _payload(width: int, height: int, stride: int) -> bytes:
    rows = np.zeros((height, stride), dtype=np.uint8)
    pixels = rows[:, : width * 3].reshape(height, width, 3)
    yy, xx = np.indices((height, width))
    pixels[..., 0] = xx % 251
    pixels[..., 1] = yy % 251
    pixels[..., 2] = (xx + yy) % 251
    return rows.tobytes()


def test_nominal_geometry_cannot_publish_a_calibrated_frame():
    with pytest.raises(CameraCalibrationError):
        CameraRectifier(_nominal())


def test_zero_distortion_is_identity_and_map_is_cached():
    width, height, stride = 32, 18, 32 * 3 + 8
    payload = _payload(width, height, stride)
    rectifier = CameraRectifier(_empirical())
    first = rectifier.rectify_packed(
        payload,
        width=width,
        height=height,
        pixel_format="BGR888",
        stride_bytes=stride,
        frame_size_bytes=len(payload),
        sensor_crop=SensorCrop(0, 0, 4608, 2592),
        runtime_status=_status(),
        lens_position=1.0,
    )
    second = rectifier.rectify_packed(
        payload,
        width=width,
        height=height,
        pixel_format="BGR888",
        stride_bytes=stride,
        frame_size_bytes=len(payload),
        sensor_crop=SensorCrop(0, 0, 4608, 2592),
        runtime_status=_status(),
        lens_position=1.0,
    )
    assert first.image_bytes == payload
    assert second.image_bytes == payload
    assert first.calibration_id == second.calibration_id
    assert first.calibration_id.startswith("cal-")
    assert rectifier.cached_map_count == 1


def test_nonzero_distortion_changes_pixels_but_preserves_frame_contract():
    width, height, stride = 64, 36, 64 * 3
    payload = _payload(width, height, stride)
    rectifier = CameraRectifier(_empirical(coefficients=(-0.18, 0.03, 0.0, 0.0, 0.0)))
    result = rectifier.rectify_packed(
        payload,
        width=width,
        height=height,
        pixel_format="RGB888",
        stride_bytes=stride,
        frame_size_bytes=len(payload),
        sensor_crop=SensorCrop(0, 0, 4608, 2592),
        runtime_status=_status(),
        lens_position=1.0,
    )
    assert len(result.image_bytes) == len(payload)
    assert result.image_bytes != payload
    assert result.rectification_duration_ns >= 0


def test_lens_position_outside_calibration_tolerance_fails_closed():
    width, height, stride = 16, 9, 16 * 3
    payload = _payload(width, height, stride)
    rectifier = CameraRectifier(_empirical(lens=1.0))
    with pytest.raises(CameraCalibrationError):
        rectifier.rectify_packed(
            payload,
            width=width,
            height=height,
            pixel_format="BGR888",
            stride_bytes=stride,
            frame_size_bytes=len(payload),
            sensor_crop=SensorCrop(0, 0, 4608, 2592),
            runtime_status=_status(),
            lens_position=1.2,
        )


def test_main_yuv420_jpeg_is_rectified_only_on_request_and_uses_cached_map():
    import cv2

    width, height, stride = 64, 36, 64
    bgr = np.zeros((height, width, 3), dtype=np.uint8)
    yy, xx = np.indices((height, width))
    bgr[..., 0] = xx * 3
    bgr[..., 1] = yy * 5
    bgr[..., 2] = (xx + yy) * 2
    i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    mapped = i420.reshape(height * 3 // 2, stride)
    rectifier = CameraRectifier(_empirical(coefficients=(-0.18, 0.03, 0.0, 0.0, 0.0)))
    first = rectifier.rectify_yuv420_to_jpeg(
        mapped,
        width=width,
        height=height,
        stride_bytes=stride,
        sensor_crop=SensorCrop(0, 0, 4608, 2592),
        runtime_status=_status(),
        lens_position=1.0,
    )
    second = rectifier.rectify_yuv420_to_jpeg(
        mapped,
        width=width,
        height=height,
        stride_bytes=stride,
        sensor_crop=SensorCrop(0, 0, 4608, 2592),
        runtime_status=_status(),
        lens_position=1.0,
    )
    assert first.jpeg_bytes.startswith(b"\xff\xd8")
    assert first.jpeg_bytes.endswith(b"\xff\xd9")
    assert first.calibration_id == second.calibration_id
    assert rectifier.cached_map_count == 1
    decoded = cv2.imdecode(np.frombuffer(first.jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (height, width, 3)
