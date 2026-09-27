#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

BASE_COMMIT = "3dbef91bd09f25fbe0e1aaff419846387fbf0e0f"
UPGRADE_NAME = "camera_calibrated_frame_edge_v1"


def _run(repo: Path, *args: str) -> str:
    return subprocess.check_output(args, cwd=repo, text=True).strip()


def _replace_once(text: str, old: str, new: str, *, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source match, found {count}")
    return text.replace(old, new, 1)


def _patch(path: Path, operations: list[tuple[str, str, str]]) -> None:
    text = path.read_text(encoding="utf-8")
    for label, old, new in operations:
        text = _replace_once(text, old, new, label=f"{path}:{label}")
    path.write_text(text, encoding="utf-8")


def _calibration(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema") != "R2B4_CAMERA_CALIBRATION_V1" or value.get("accepted") is not True:
        raise ValueError("calibration file is not an accepted R2B4_CAMERA_CALIBRATION_V1 result")
    if "imx708" not in str(value.get("camera_model", "")).casefold():
        raise ValueError("calibration camera_model must be IMX708")
    size = value.get("reference_size")
    if size != [4608, 2592]:
        raise ValueError(f"calibration reference_size must be [4608, 2592], got {size!r}")
    K = value.get("K")
    if not isinstance(K, list) or len(K) != 3 or any(not isinstance(row, list) or len(row) != 3 for row in K):
        raise ValueError("calibration K must be a 3x3 array")
    for row in K:
        for item in row:
            if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
                raise ValueError("calibration K must contain finite numbers")
    model = value.get("distortion_model")
    coeffs = value.get("distortion_coefficients")
    if model == "opencv_pinhole":
        if not isinstance(coeffs, list) or len(coeffs) not in {4, 5, 8}:
            raise ValueError("opencv_pinhole calibration requires 4, 5 or 8 coefficients")
    elif model == "opencv_fisheye":
        if not isinstance(coeffs, list) or len(coeffs) != 4:
            raise ValueError("opencv_fisheye calibration requires 4 coefficients")
    else:
        raise ValueError("unsupported calibration distortion_model")
    for item in coeffs:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
            raise ValueError("distortion coefficients must be finite")
    lens = value.get("reference_lens_position")
    tolerance = value.get("lens_position_tolerance")
    if isinstance(lens, bool) or not isinstance(lens, (int, float)) or not math.isfinite(float(lens)) or float(lens) < 0.0:
        raise ValueError("reference_lens_position must be finite and non-negative")
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or not math.isfinite(float(tolerance)) or float(tolerance) <= 0.0:
        raise ValueError("lens_position_tolerance must be finite and positive")
    return value


def _backup(repo: Path, paths: list[Path]) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    root = repo / ".upgrade_backups" / f"{UPGRADE_NAME}_{stamp}"
    for relative in paths:
        source = repo / relative
        if source.exists():
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    return root


def _patch_picamera(path: Path) -> None:
    ops: list[tuple[str, str, str]] = []
    ops.append((
        "numpy-import",
        "from typing import Protocol\n\nfrom .camera_geometry import (",
        "from typing import Protocol\n\nimport numpy as np\n\nfrom .camera_geometry import (",
    ))
    ops.append((
        "rectifier-import",
        "    sensor_crop_from_value,\n)\n\n\nclass Picamera2Request",
        "    sensor_crop_from_value,\n)\nfrom .camera_rectification import CameraRectifier\n\n\nclass Picamera2Request",
    ))
    ops.append((
        "nonnegative-float-helper",
        "def _nonempty_string(value: object, name: str) -> str:\n",
        "def _nonnegative_float(value: object, name: str) -> float:\n"
        "    if (\n"
        "        isinstance(value, bool)\n"
        "        or not isinstance(value, (int, float))\n"
        "        or not math.isfinite(value)\n"
        "        or value < 0.0\n"
        "    ):\n"
        "        raise ValueError(f\"{name} must be finite and non-negative\")\n"
        "    return float(value)\n\n\n"
        "def _nonempty_string(value: object, name: str) -> str:\n",
    ))
    ops.append((
        "config-fixed-lens-field",
        "    continuous_autofocus: bool = True\n    max_frame_completion_lag_ns: int = 500_000_000\n",
        "    continuous_autofocus: bool = True\n"
        "    fixed_lens_position: float | None = None\n"
        "    max_frame_completion_lag_ns: int = 500_000_000\n",
    ))
    ops.append((
        "config-fixed-lens-validation",
        "        if type(self.continuous_autofocus) is not bool:\n"
        "            raise TypeError(\"continuous_autofocus must be bool\")\n",
        "        if type(self.continuous_autofocus) is not bool:\n"
        "            raise TypeError(\"continuous_autofocus must be bool\")\n"
        "        if self.fixed_lens_position is not None:\n"
        "            _nonnegative_float(self.fixed_lens_position, \"fixed_lens_position\")\n"
        "        if self.continuous_autofocus and self.fixed_lens_position is not None:\n"
        "            raise ValueError(\"continuous autofocus and fixed lens position are mutually exclusive\")\n",
    ))
    ops.append((
        "config-allowed-key",
        "        \"continuous_autofocus\",\n        \"max_frame_completion_lag_ns\",\n",
        "        \"continuous_autofocus\",\n        \"fixed_lens_position\",\n        \"max_frame_completion_lag_ns\",\n",
    ))
    ops.append((
        "config-parse-fixed-lens",
        "        continuous_autofocus=bool_value(\"continuous_autofocus\", True),\n"
        "        max_frame_completion_lag_ns=_positive_int(\n",
        "        continuous_autofocus=bool_value(\"continuous_autofocus\", True),\n"
        "        fixed_lens_position=(\n"
        "            None\n"
        "            if value.get(\"fixed_lens_position\") is None\n"
        "            else _nonnegative_float(value.get(\"fixed_lens_position\"), \"camera.fixed_lens_position\")\n"
        "        ),\n"
        "        max_frame_completion_lag_ns=_positive_int(\n",
    ))
    ops.append((
        "frame-calibration-fields",
        "    lens_position: float | None\n    image_bytes: bytes\n    sensor_crop: SensorCrop | None = None\n",
        "    lens_position: float | None\n"
        "    image_bytes: bytes\n"
        "    calibration_state: str\n"
        "    calibration_id: str\n"
        "    rectified_K: tuple[tuple[float, float, float], ...]\n"
        "    rectification_duration_ns: int\n"
        "    sensor_crop: SensorCrop | None = None\n",
    ))
    ops.append((
        "frame-calibration-validation",
        "        if not self.image_bytes:\n"
        "            raise ValueError(\"image_bytes must not be empty\")\n"
        "        if len(self.image_bytes) != self.frame_size_bytes:\n"
        "            raise ValueError(\"camera payload size does not match configured frame size\")\n"
        "        if self.sensor_crop is not None and not isinstance(self.sensor_crop, SensorCrop):\n",
        "        if not self.image_bytes:\n"
        "            raise ValueError(\"image_bytes must not be empty\")\n"
        "        if len(self.image_bytes) != self.frame_size_bytes:\n"
        "            raise ValueError(\"camera payload size does not match configured frame size\")\n"
        "        if self.calibration_state != \"CALIBRATED\":\n"
        "            raise ValueError(\"public camera frames must be CALIBRATED\")\n"
        "        _nonempty_string(self.calibration_id, \"calibration_id\")\n"
        "        _nonnegative_int(self.rectification_duration_ns, \"rectification_duration_ns\")\n"
        "        if (\n"
        "            not isinstance(self.rectified_K, tuple)\n"
        "            or len(self.rectified_K) != 3\n"
        "            or any(not isinstance(row, tuple) or len(row) != 3 for row in self.rectified_K)\n"
        "        ):\n"
        "            raise TypeError(\"rectified_K must be an immutable 3x3 matrix\")\n"
        "        for row in self.rectified_K:\n"
        "            for item in row:\n"
        "                if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):\n"
        "                    raise ValueError(\"rectified_K must contain finite values\")\n"
        "        if self.sensor_crop is not None and not isinstance(self.sensor_crop, SensorCrop):\n",
    ))
    ops.append((
        "slot-rectifier",
        "        \"_camera_geometry_status\",\n        \"_default_sensor_crop\",\n",
        "        \"_camera_geometry_status\",\n        \"_default_sensor_crop\",\n        \"_rectifier\",\n",
    ))
    ops.append((
        "slot-main-geometry",
        "        \"_geometry\",\n        \"_last_error\",\n",
        "        \"_geometry\",\n        \"_main_geometry\",\n        \"_last_error\",\n",
    ))
    ops.append((
        "init-rectifier",
        "        self._camera_geometry_status: CameraGeometryStatus | None = None\n"
        "        self._default_sensor_crop: SensorCrop | None = None\n",
        "        self._camera_geometry_status: CameraGeometryStatus | None = None\n"
        "        self._default_sensor_crop: SensorCrop | None = None\n"
        "        self._rectifier: CameraRectifier | None = None\n",
    ))
    ops.append((
        "init-main-geometry",
        "        self._picamera: Picamera2Device | None = None\n        self._geometry: CameraStreamGeometry | None = None\n",
        "        self._picamera: Picamera2Device | None = None\n        self._geometry: CameraStreamGeometry | None = None\n        self._main_geometry: CameraStreamGeometry | None = None\n",
    ))
    ops.append((
        "start-reset-main-geometry",
        "            self._latest = None\n            self._geometry = None\n            self._last_sensor_timestamp_ns = None\n",
        "            self._latest = None\n            self._geometry = None\n            self._main_geometry = None\n            self._last_sensor_timestamp_ns = None\n",
    ))
    ops.append((
        "start-require-calibration",
        "            self._stop_event.clear()\n"
        "            self._frame_condition.notify_all()\n\n"
        "        camera: Picamera2Device | None = None\n",
        "            self._stop_event.clear()\n"
        "            self._rectifier = None\n"
        "            self._frame_condition.notify_all()\n\n"
        "        try:\n"
        "            geometry_config = self._camera_geometry_config\n"
        "            if (\n"
        "                geometry_config is None\n"
        "                or geometry_config.intrinsic.source != \"empirical\"\n"
        "                or not geometry_config.intrinsic.can_rectify\n"
        "            ):\n"
        "                raise RuntimeError(\"empirical camera K/D calibration is required\")\n"
        "            rectifier = CameraRectifier(geometry_config)\n"
        "        except Exception as exc:\n"
        "            with self._frame_condition:\n"
        "                self._last_error = f\"{type(exc).__name__}:{exc}\"\n"
        "                self._running = False\n"
        "                self._frame_condition.notify_all()\n"
        "            return False\n\n"
        "        camera: Picamera2Device | None = None\n",
    ))
    ops.append((
        "start-read-main-geometry",
        "            geometry = camera_stream_geometry(camera, self._config.stream_name)\n            camera_geometry_status = None\n",
        "            geometry = camera_stream_geometry(camera, self._config.stream_name)\n            main_geometry = camera_stream_geometry(camera, \"main\")\n            camera_geometry_status = None\n",
    ))
    ops.append((
        "start-store-rectifier-and-reject-invalid",
        "                default_sensor_crop = camera_geometry_status.sensor_crop\n"
        "            controls = (\n",
        "                default_sensor_crop = camera_geometry_status.sensor_crop\n"
        "                if camera_geometry_status.state == \"INVALID\":\n"
        "                    raise RuntimeError(\n"
        "                        \"camera geometry invalid: \" + (camera_geometry_status.reason or \"unknown\")\n"
        "                    )\n"
        "            controls = (\n",
    ))
    ops.append((
        "store-main-geometry",
        "            self._picamera = camera\n            self._geometry = geometry\n            self._model = model\n",
        "            self._picamera = camera\n            self._geometry = geometry\n            self._main_geometry = main_geometry\n            self._model = model\n",
    ))
    ops.append((
        "store-rectifier",
        "            self._camera_geometry_status = camera_geometry_status\n"
        "            self._default_sensor_crop = default_sensor_crop\n"
        "            self._running = True\n",
        "            self._camera_geometry_status = camera_geometry_status\n"
        "            self._default_sensor_crop = default_sensor_crop\n"
        "            self._rectifier = rectifier\n"
        "            self._running = True\n",
    ))
    ops.append((
        "stop-clear-rectifier",
        "            self._picamera = None\n"
        "            self._geometry = None\n"
        "            self._frame_condition.notify_all()\n",
        "            self._picamera = None\n"
        "            self._geometry = None\n"
        "            self._main_geometry = None\n"
        "            self._rectifier = None\n"
        "            self._frame_condition.notify_all()\n",
    ))
    ops.append((
        "jpeg-only-canonical-stream",
        "        if stream_name not in {\"main\", \"lores\"}:\n"
        "            raise ValueError(\"camera photo stream_name must be 'main' or 'lores'\")\n",
        "        if stream_name not in {self._config.stream_name, \"main\"}:\n"
        "            raise ValueError(\n"
        "                \"only calibrated canonical lores/main JPEG streams may be published\"\n"
        "            )\n",
    ))

    ops.append((
        "demand-driven-main-jpeg",
        "                        saver = getattr(request, \"save\", None)\n"
        "                        if not callable(saver):\n"
        "                            raise RuntimeError(\"Picamera2 request.save is unavailable\")\n"
        "                        saver(stream_name, output, format=\"jpeg\")\n",
        "                        if stream_name == geometry.stream_name:\n"
        "                            saver = getattr(request, \"save\", None)\n"
        "                            if not callable(saver):\n"
        "                                raise RuntimeError(\"Picamera2 request.save is unavailable\")\n"
        "                            saver(stream_name, output, format=\"jpeg\")\n"
        "                        elif stream_name == \"main\":\n"
        "                            self._save_calibrated_main_jpeg(request, output, snapshot)\n"
        "                        else:\n"
        "                            raise RuntimeError(\"requested camera JPEG stream is not calibrated\")\n",
    ))

    old_snapshot = '''    def _snapshot_from_request(
        self,
        request: Picamera2Request,
        geometry: CameraStreamGeometry,
    ) -> CameraFrameSnapshot:
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

        if hasattr(request, "stream_map") and hasattr(request, "picam2"):
            from picamera2 import MappedArray

            with MappedArray(
                request,
                geometry.stream_name,
                reshape=False,
                write=False,
            ) as mapped:
                if mapped.array is None:
                    raise RuntimeError("camera mapped buffer is unavailable")
                image_bytes = bytes(mapped.array)
        else:
            # Off-target/unit-test fallback for synthetic Picamera2Request objects.
            raw_buffer = request.make_buffer(geometry.stream_name)
            image_bytes = bytes(raw_buffer)
        if len(image_bytes) != geometry.frame_size_bytes:
            raise RuntimeError(
                "camera buffer size does not match Picamera2 stream configuration"
            )
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
        sensor_crop = sensor_crop_from_value(metadata.get("ScalerCrop"))
        with self._lock:
            if sensor_crop is None:
                sensor_crop = self._default_sensor_crop
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
            width=geometry.width,
            height=geometry.height,
            pixel_format=geometry.pixel_format,
            stride_bytes=geometry.stride_bytes,
            frame_size_bytes=geometry.frame_size_bytes,
            focus_state=focus_state,
            lens_position=lens_position,
            image_bytes=image_bytes,
            sensor_crop=sensor_crop,
        )
'''
    new_snapshot = '''    def _snapshot_from_request(
        self,
        request: Picamera2Request,
        geometry: CameraStreamGeometry,
    ) -> CameraFrameSnapshot:
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
        lens_raw = metadata.get("LensPosition")
        lens_position = (
            float(lens_raw)
            if isinstance(lens_raw, (int, float)) and not isinstance(lens_raw, bool)
            else None
        )
        focus_state = str(metadata.get("AfState", "UNKNOWN")) or "UNKNOWN"
        sensor_crop = sensor_crop_from_value(metadata.get("ScalerCrop"))
        with self._lock:
            if sensor_crop is None:
                sensor_crop = self._default_sensor_crop
            rectifier = self._rectifier
            geometry_status = self._camera_geometry_status
        if rectifier is None:
            raise RuntimeError("camera calibrated-frame rectifier is unavailable")

        if hasattr(request, "stream_map") and hasattr(request, "picam2"):
            from picamera2 import MappedArray

            with MappedArray(
                request,
                geometry.stream_name,
                reshape=False,
                write=True,
            ) as mapped:
                if mapped.array is None:
                    raise RuntimeError("camera mapped buffer is unavailable")
                raw_bytes = bytes(mapped.array)
                rectified = rectifier.rectify_packed(
                    raw_bytes,
                    width=geometry.width,
                    height=geometry.height,
                    pixel_format=geometry.pixel_format,
                    stride_bytes=geometry.stride_bytes,
                    frame_size_bytes=geometry.frame_size_bytes,
                    sensor_crop=sensor_crop,
                    runtime_status=geometry_status,
                    lens_position=lens_position,
                )
                flat = mapped.array.reshape(-1)
                calibrated = np.frombuffer(rectified.image_bytes, dtype=np.uint8)
                if flat.size != calibrated.size:
                    raise RuntimeError("calibrated frame no longer matches Picamera2 buffer size")
                flat[:] = calibrated
                image_bytes = rectified.image_bytes
        else:
            # Off-target/unit-test fallback. Raw bytes remain local to this call;
            # only the rectified result is allowed into CameraFrameSnapshot.
            raw_buffer = request.make_buffer(geometry.stream_name)
            rectified = rectifier.rectify_packed(
                bytes(raw_buffer),
                width=geometry.width,
                height=geometry.height,
                pixel_format=geometry.pixel_format,
                stride_bytes=geometry.stride_bytes,
                frame_size_bytes=geometry.frame_size_bytes,
                sensor_crop=sensor_crop,
                runtime_status=geometry_status,
                lens_position=lens_position,
            )
            image_bytes = rectified.image_bytes

        completed_monotonic_ns = self._checked_clock()
        if measurement_monotonic_ns > completed_monotonic_ns:
            raise ValueError("mapped camera timestamp is in the future")
        lag_ns = completed_monotonic_ns - measurement_monotonic_ns
        if lag_ns > self._config.max_frame_completion_lag_ns:
            raise RuntimeError("camera frame completion lag exceeded configured bound")
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
            width=geometry.width,
            height=geometry.height,
            pixel_format=geometry.pixel_format,
            stride_bytes=geometry.stride_bytes,
            frame_size_bytes=geometry.frame_size_bytes,
            focus_state=focus_state,
            lens_position=lens_position,
            image_bytes=image_bytes,
            calibration_state="CALIBRATED",
            calibration_id=rectified.calibration_id,
            rectified_K=rectified.rectified_K,
            rectification_duration_ns=rectified.rectification_duration_ns,
            sensor_crop=sensor_crop,
        )
'''
    ops.append(("snapshot-rectification", old_snapshot, new_snapshot))
    ops.append((
        "main-jpeg-helper",
        "    def _snapshot_from_request(\n",
        "    def _save_calibrated_main_jpeg(\n"
        "        self,\n"
        "        request: Picamera2Request,\n"
        "        output: str,\n"
        "        snapshot: CameraFrameSnapshot,\n"
        "    ) -> None:\n"
        "        with self._lock:\n"
        "            geometry = self._main_geometry\n"
        "            rectifier = self._rectifier\n"
        "            geometry_status = self._camera_geometry_status\n"
        "        if geometry is None or rectifier is None:\n"
        "            raise RuntimeError(\"calibrated main JPEG pipeline is unavailable\")\n"
        "        if geometry.pixel_format != \"YUV420\":\n"
        "            raise RuntimeError(\"calibrated main JPEG currently requires YUV420\")\n"
        "        if not hasattr(request, \"stream_map\") or not hasattr(request, \"picam2\"):\n"
        "            raise RuntimeError(\"calibrated main JPEG requires a native Picamera2 request\")\n"
        "        from picamera2 import MappedArray\n"
        "        with MappedArray(request, \"main\", reshape=True, write=False) as mapped:\n"
        "            if mapped.array is None:\n"
        "                raise RuntimeError(\"camera main YUV420 buffer is unavailable\")\n"
        "            result = rectifier.rectify_yuv420_to_jpeg(\n"
        "                mapped.array,\n"
        "                width=geometry.width,\n"
        "                height=geometry.height,\n"
        "                stride_bytes=geometry.stride_bytes,\n"
        "                sensor_crop=snapshot.sensor_crop,\n"
        "                runtime_status=geometry_status,\n"
        "                lens_position=snapshot.lens_position,\n"
        "            )\n"
        "        Path(output).write_bytes(result.jpeg_bytes)\n"
        "\n"
        "    def _snapshot_from_request(\n",
    ))
    ops.append((
        "manual-focus-controls",
        '''def default_camera_controls(config: Picamera2CameraConfig) -> Mapping[str, object]:
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
''',
        '''def default_camera_controls(config: Picamera2CameraConfig) -> Mapping[str, object]:
    """Return Camera Module 3 focus controls closed to the calibration contract."""

    if not config.continuous_autofocus and config.fixed_lens_position is None:
        return {}
    try:
        from libcamera import controls  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - hardware/deployment path
        raise RuntimeError("libcamera Python controls are unavailable") from exc
    if config.fixed_lens_position is not None:
        return {
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": config.fixed_lens_position,
        }
    return {
        "AfMode": controls.AfModeEnum.Continuous,
        "AfSpeed": controls.AfSpeedEnum.Normal,
    }
''',
    ))
    _patch(path, ops)


def _patch_process_vision(path: Path) -> None:
    _patch(path, [
        (
            "control-calibration-fields",
            "    focus_state: str\n    lens_position: float | None\n\n    @property\n",
            "    focus_state: str\n"
            "    lens_position: float | None\n"
            "    calibration_state: str\n"
            "    calibration_id: str\n"
            "    rectified_K: tuple[tuple[float, float, float], ...]\n"
            "    rectification_duration_ns: int\n\n"
            "    @property\n",
        ),
        (
            "wire-calibration",
            "            frame.focus_state,\n            frame.lens_position,\n        )\n",
            "            frame.focus_state,\n"
            "            frame.lens_position,\n"
            "            frame.calibration_state,\n"
            "            frame.calibration_id,\n"
            "            frame.rectified_K,\n"
            "            frame.rectification_duration_ns,\n"
            "        )\n",
        ),
        (
            "unwire-calibration",
            "            focus_state=str(frame_wire[11]),\n"
            "            lens_position=(None if frame_wire[12] is None else float(frame_wire[12])),\n"
            "        )\n",
            "            focus_state=str(frame_wire[11]),\n"
            "            lens_position=(None if frame_wire[12] is None else float(frame_wire[12])),\n"
            "            calibration_state=str(frame_wire[13]),\n"
            "            calibration_id=str(frame_wire[14]),\n"
            "            rectified_K=tuple(tuple(float(value) for value in row) for row in frame_wire[15]),\n"
            "            rectification_duration_ns=int(frame_wire[16]),\n"
            "        )\n",
        ),
    ])


def _patch_live_camera(path: Path) -> None:
    _patch(path, [
        (
            "reject-uncalibrated-parent-frame",
            "        if status.frame_sequence != frame.sequence:\n"
            "            return self._failed(context, \"CAMERA_LINEAGE_INVALID\")\n"
            "        if frame.measurement_monotonic_ns > context.monotonic_ns:\n",
            "        if status.frame_sequence != frame.sequence:\n"
            "            return self._failed(context, \"CAMERA_LINEAGE_INVALID\")\n"
            "        if getattr(frame, \"calibration_state\", None) != \"CALIBRATED\":\n"
            "            return self._failed(context, \"CAMERA_FRAME_UNCALIBRATED\")\n"
            "        if frame.measurement_monotonic_ns > context.monotonic_ns:\n",
        ),
        (
            "calibration-telemetry",
            "                DataField(\"lens_position\", frame.lens_position),\n"
            "                # Process vision exposes metadata only; raw image bytes stay child-owned.\n",
            "                DataField(\"lens_position\", frame.lens_position),\n"
            "                DataField(\"calibration_state\", frame.calibration_state),\n"
            "                DataField(\"calibration_id\", frame.calibration_id),\n"
            "                DataField(\"rectification_duration_ns\", frame.rectification_duration_ns),\n"
            "                DataField(\"rectified_fx_px\", frame.rectified_K[0][0]),\n"
            "                DataField(\"rectified_fy_px\", frame.rectified_K[1][1]),\n"
            "                DataField(\"rectified_cx_px\", frame.rectified_K[0][2]),\n"
            "                DataField(\"rectified_cy_px\", frame.rectified_K[1][2]),\n"
            "                # Process vision exposes metadata only; raw image bytes stay child-owned.\n",
        ),
    ])


def _patch_vision_media(path: Path) -> None:
    _patch(path, [
        (
            "protocol-server",
            "                    if len(parts) != 2 or parts[0] != \"R2B4JPEG1\" or parts[1] not in {\"lores\", \"main\"}:\n"
            "                        raise ValueError(\"expected: R2B4JPEG1 lores|main\")\n",
            "                    if len(parts) != 2 or parts[0] != \"R2B4CJPEG1\" or parts[1] not in {\"lores\", \"main\"}:\n"
            "                        raise ValueError(\"expected: R2B4CJPEG1 lores|main\")\n",
        ),
    ])


def _patch_er2_media(path: Path) -> None:
    _patch(path, [
        (
            "protocol-client",
            "                sock.sendall(f\"R2B4JPEG1 {stream_name}\\n\".encode(\"ascii\"))\n",
            "                sock.sendall(f\"R2B4CJPEG1 {stream_name}\\n\".encode(\"ascii\"))\n",
        ),
    ])


def _patch_config_hardware(path: Path) -> None:
    _patch(path, [
        (
            "empirical-production-gate",
            "            camera_device = picamera2_camera_config_from_mapping(\n"
            "                {key: item for key, item in camera.items() if key != \"geometry\"}\n"
            "            )\n"
            "            camera_source = NativeCameraConfig(\n",
            "            camera_device = picamera2_camera_config_from_mapping(\n"
            "                {key: item for key, item in camera.items() if key != \"geometry\"}\n"
            "            )\n"
            "            intrinsic = camera_geometry.intrinsic\n"
            "            if intrinsic.source != \"empirical\" or not intrinsic.can_rectify:\n"
            "                raise ValueError(\n"
            "                    \"enabled production camera requires empirical rectifiable K/D calibration\"\n"
            "                )\n"
            "            if camera_device.stream_name != \"lores\" or camera_device.pixel_format not in {\"RGB888\", \"BGR888\"}:\n"
            "                raise ValueError(\n"
            "                    \"canonical calibrated production stream must be packed RGB/BGR lores\"\n"
            "                )\n"
            "            if camera_device.continuous_autofocus:\n"
            "                raise ValueError(\n"
            "                    \"empirically calibrated production camera requires fixed focus\"\n"
            "                )\n"
            "            if intrinsic.reference_lens_position is not None:\n"
            "                if camera_device.fixed_lens_position is None:\n"
            "                    raise ValueError(\"camera.fixed_lens_position is required by empirical calibration\")\n"
            "                tolerance = intrinsic.lens_position_tolerance or 1e-6\n"
            "                if abs(camera_device.fixed_lens_position - intrinsic.reference_lens_position) > tolerance:\n"
            "                    raise ValueError(\"camera fixed lens position differs from empirical calibration\")\n"
            "            camera_source = NativeCameraConfig(\n",
        ),
    ])


def _patch_camera_adapter(path: Path) -> None:
    _patch(path, [
        (
            "direct-media-unavailable",
            "        reason = \"OWNED_BY_V3_RUNTIME\" if running else None\n",
            "        reason = (\n"
            "            \"OWNED_BY_V3_RUNTIME\"\n"
            "            if running\n"
            "            else \"CALIBRATED_MEDIA_ONLY\"\n"
            "        )\n",
        ),
        (
            "direct-media-capability-fail-closed",
            "        return {\n"
            "            \"camera.photo\": {\n"
            "                \"kind\": \"action\",\n"
            "                \"supported\": True,\n"
            "                \"available\": True,\n"
            "                \"ready\": not running,\n"
            "                \"reason\": reason,\n"
            "            },\n"
            "            \"camera.video\": {\n"
            "                \"kind\": \"action\",\n"
            "                \"supported\": True,\n"
            "                \"available\": True,\n"
            "                \"ready\": not running,\n"
            "                \"reason\": reason,\n"
            "            },\n"
            "        }\n",
            "        return {\n"
            "            \"camera.photo\": {\n"
            "                \"kind\": \"action\",\n"
            "                \"supported\": True,\n"
            "                \"available\": False,\n"
            "                \"ready\": False,\n"
            "                \"reason\": reason,\n"
            "            },\n"
            "            \"camera.video\": {\n"
            "                \"kind\": \"action\",\n"
            "                \"supported\": True,\n"
            "                \"available\": False,\n"
            "                \"ready\": False,\n"
            "                \"reason\": reason,\n"
            "            },\n"
            "        }\n",
        ),
    ])


def _patch_camera_tool(path: Path) -> None:
    _patch(path, [
        (
            "geometry-import",
            "from v3.adapters.camera_media import capture_h264_video, capture_photo\n",
            "from v3.adapters.camera_media import capture_h264_video, capture_photo\n"
            "from v3.adapters.camera_geometry import camera_geometry_config_from_mapping\n",
        ),
        (
            "geometry-helper",
            "\ndef _status(args: argparse.Namespace) -> int:\n",
            "\ndef _geometry_config():\n"
            "    repo = Path(__file__).resolve().parents[1]\n"
            "    path = repo / \"conf\" / \"hardver.json\"\n"
            "    document = json.loads(path.read_text(encoding=\"utf-8\"))\n"
            "    camera = document.get(\"camera\")\n"
            "    if not isinstance(camera, dict) or not isinstance(camera.get(\"geometry\"), dict):\n"
            "        raise SystemExit(\"conf/hardver.json camera.geometry is required\")\n"
            "    return camera_geometry_config_from_mapping(camera[\"geometry\"])\n\n\n"
            "def _status(args: argparse.Namespace) -> int:\n",
        ),
        (
            "owner-uses-geometry",
            "    owner = NativePicamera2Camera(\n        config,\n        picamera_factory=default_picamera2_factory,\n",
            "    owner = NativePicamera2Camera(\n        config,\n        camera_geometry_config=_geometry_config(),\n        picamera_factory=default_picamera2_factory,\n",
        ),
        (
            "status-calibration-fields",
            '            "continuous_autofocus": config.continuous_autofocus,\n',
            '            "continuous_autofocus": config.continuous_autofocus,\n'
            '            "fixed_lens_position": config.fixed_lens_position,\n'
            '            "calibration_state": (final.frame.calibration_state if final.frame is not None else None),\n'
            '            "calibration_id": (final.frame.calibration_id if final.frame is not None else None),\n'
            '            "rectification_duration_ms": (final.frame.rectification_duration_ns / 1_000_000.0 if final.frame is not None else None),\n',
        ),
    ])

def _patch_person_runtime_test(path: Path) -> None:
    _patch(path, [
        (
            "geometry-import",
            "from v3.adapters.person_photo_evidence import PersonPhotoEvidenceConfig, PersonPhotoEvidenceRecorder\n",
            "from v3.adapters.person_photo_evidence import PersonPhotoEvidenceConfig, PersonPhotoEvidenceRecorder\n"
            "from v3.adapters.camera_geometry import camera_geometry_config_from_mapping\n",
        ),
        (
            "empirical-helper",
            "\nclass _SavingRequest:\n",
            "\ndef _empirical_geometry():\n"
            "    return camera_geometry_config_from_mapping(\n"
            "        {\n"
            "            \"factory_profile\": \"imx708_wide_noir\",\n"
            "            \"mount\": {\n"
            "                \"x_m\": 0.05, \"y_m\": 0.0, \"z_m\": 0.19,\n"
            "                \"roll_deg\": 0.0, \"pitch_deg\": 15.0, \"yaw_deg\": 0.0,\n"
            "            },\n"
            "            \"intrinsic\": {\n"
            "                \"source\": \"empirical\",\n"
            "                \"reference_size\": [4608, 2592],\n"
            "                \"K\": [[2400.0, 0.0, 2304.0], [0.0, 2400.0, 1296.0], [0.0, 0.0, 1.0]],\n"
            "                \"distortion_model\": \"opencv_pinhole\",\n"
            "                \"distortion_coefficients\": [0.0, 0.0, 0.0, 0.0, 0.0],\n"
            "                \"reference_lens_position\": 1.0,\n"
            "                \"lens_position_tolerance\": 0.05,\n"
            "            },\n"
            "        }\n"
            "    )\n\n\n"
            "class _SavingRequest:\n",
        ),
        (
            "owner-empirical-geometry",
            "    owner = NativePicamera2Camera(Picamera2CameraConfig(max_frame_completion_lag_ns=2000000000), picamera_factory=lambda index: camera, sensor_timestamp_mapper=lambda value: value, camera_controls_factory=lambda: {}, monotonic_ns=monotonic_ns)\n",
            "    owner = NativePicamera2Camera(Picamera2CameraConfig(max_frame_completion_lag_ns=2000000000), camera_geometry_config=_empirical_geometry(), picamera_factory=lambda index: camera, sensor_timestamp_mapper=lambda value: value, camera_controls_factory=lambda: {}, monotonic_ns=monotonic_ns)\n",
        ),
    ])

def _patch_camera_test(path: Path) -> None:
    old = "return CameraFrameSnapshot(sequence=7, sensor_timestamp_ns=900, measurement_monotonic_ns=measurement_ns, completed_monotonic_ns=950, exposure_time_ns=10, frame_duration_ns=50, width=2, height=2, pixel_format='RGB888', stride_bytes=6, frame_size_bytes=12, focus_state='2', lens_position=1.0, image_bytes=b'abcdefghijkl')"
    new = "return CameraFrameSnapshot(sequence=7, sensor_timestamp_ns=900, measurement_monotonic_ns=measurement_ns, completed_monotonic_ns=950, exposure_time_ns=10, frame_duration_ns=50, width=2, height=2, pixel_format='RGB888', stride_bytes=6, frame_size_bytes=12, focus_state='2', lens_position=1.0, image_bytes=b'abcdefghijkl', calibration_state='CALIBRATED', calibration_id='test-calibration', rectified_K=((1.0, 0.0, 1.0), (0.0, 1.0, 1.0), (0.0, 0.0, 1.0)), rectification_duration_ns=0)"
    _patch(path, [("test-calibrated-frame", old, new)])


def _update_hardware_config(path: Path, calibration: dict[str, object]) -> None:
    document = json.loads(path.read_text(encoding="utf-8"))
    camera = document.get("camera")
    if not isinstance(camera, dict):
        raise ValueError("conf/hardver.json camera object is missing")
    geometry = camera.get("geometry")
    if not isinstance(geometry, dict):
        raise ValueError("conf/hardver.json camera.geometry object is missing")
    lens = float(calibration["reference_lens_position"])
    camera["continuous_autofocus"] = False
    camera["fixed_lens_position"] = lens
    geometry["intrinsic"] = {
        "source": "empirical",
        "reference_size": calibration["reference_size"],
        "K": calibration["K"],
        "distortion_model": calibration["distortion_model"],
        "distortion_coefficients": calibration["distortion_coefficients"],
        "reference_lens_position": lens,
        "lens_position_tolerance": calibration["lens_position_tolerance"],
    }
    path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Install R2B4 canonical calibrated-frame camera upgrade")
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--allow-head-mismatch", action="store_true")
    args = parser.parse_args()
    repo = args.repo.resolve()
    if not (repo / ".git").exists():
        raise RuntimeError(f"not a Git repository: {repo}")
    head = _run(repo, "git", "rev-parse", "HEAD")
    if head != BASE_COMMIT and not args.allow_head_mismatch:
        raise RuntimeError(
            f"upgrade was source-first built for {BASE_COMMIT}, current HEAD is {head}; "
            "refresh the package or explicitly use --allow-head-mismatch after reviewing the diff"
        )
    calibration = _calibration(args.calibration.resolve())
    subprocess.check_call([sys.executable, "-c", "import cv2, numpy; print('OpenCV', cv2.__version__)"])
    package = Path(__file__).resolve().parent

    touched = [
        Path("v3/adapters/picamera2_camera.py"),
        Path("v3/adapters/process_vision_port.py"),
        Path("v3/adapters/live_camera.py"),
        Path("v3/adapters/vision_media_socket.py"),
        Path("r2b4_er2/media.py"),
        Path("v3/config_hardware.py"),
        Path("v3/adapters/camera.py"),
        Path("v3/adapters/camera_media.py"),
        Path("tools/v3_camera_test.py"),
        Path("tests/feature/test_v3_person_detection_runtime_integration.py"),
        Path("conf/hardver.json"),
        Path("tests/feature/test_v3_camera_foundation.py"),
        Path("v3/adapters/camera_rectification.py"),
        Path("tools/v3_camera_calibrate.py"),
        Path("tests/feature/test_v3_camera_rectification.py"),
    ]
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain", "--", *[str(item) for item in touched]],
        cwd=repo,
        text=True,
    ).strip()
    if dirty:
        raise RuntimeError(
            "refusing to overwrite existing user changes in upgrade-owned paths:\n" + dirty
        )
    existed = {relative: (repo / relative).exists() for relative in touched}
    backup = _backup(repo, touched)

    # Copy complete new/replaced files first; existing source mutations below are
    # exact, count-checked replacements and abort instead of guessing on drift.
    copies = {
        Path("v3/adapters/camera_rectification.py"): package / "payload/v3/adapters/camera_rectification.py",
        Path("v3/adapters/camera_media.py"): package / "payload/v3/adapters/camera_media.py",
        Path("tools/v3_camera_calibrate.py"): package / "payload/tools/v3_camera_calibrate.py",
        Path("tests/feature/test_v3_camera_rectification.py"): package / "payload/tests/feature/test_v3_camera_rectification.py",
    }
    try:
        for relative, source in copies.items():
            target = repo / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)

        _patch_picamera(repo / "v3/adapters/picamera2_camera.py")
        _patch_process_vision(repo / "v3/adapters/process_vision_port.py")
        _patch_live_camera(repo / "v3/adapters/live_camera.py")
        _patch_vision_media(repo / "v3/adapters/vision_media_socket.py")
        _patch_er2_media(repo / "r2b4_er2/media.py")
        _patch_config_hardware(repo / "v3/config_hardware.py")
        _patch_camera_adapter(repo / "v3/adapters/camera.py")
        _patch_camera_tool(repo / "tools/v3_camera_test.py")
        _patch_person_runtime_test(repo / "tests/feature/test_v3_person_detection_runtime_integration.py")
        _patch_camera_test(repo / "tests/feature/test_v3_camera_foundation.py")
        _update_hardware_config(repo / "conf/hardver.json", calibration)

        # Syntax validation does not touch hardware.
        compile_targets = [
            "v3/adapters/camera_rectification.py",
            "v3/adapters/picamera2_camera.py",
            "v3/adapters/process_vision_port.py",
            "v3/adapters/live_camera.py",
            "v3/adapters/vision_media_socket.py",
            "r2b4_er2/media.py",
            "v3/config_hardware.py",
            "v3/adapters/camera.py",
            "v3/adapters/camera_media.py",
            "tools/v3_camera_test.py",
            "tests/feature/test_v3_person_detection_runtime_integration.py",
            "tests/feature/test_v3_camera_foundation.py",
            "tools/v3_camera_calibrate.py",
            "tests/feature/test_v3_camera_rectification.py",
        ]
        subprocess.check_call([sys.executable, "-m", "py_compile", *compile_targets], cwd=repo)
    except Exception:
        for relative in touched:
            target = repo / relative
            if existed[relative]:
                source = backup / relative
                if source.exists():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, target)
            elif target.is_file() or target.is_symlink():
                target.unlink()
        raise
    print(f"UPGRADE_APPLIED={UPGRADE_NAME}")
    print(f"BASE_COMMIT={BASE_COMMIT}")
    print(f"BACKUP={backup}")
    print("NEXT=./r test perception")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"UPGRADE_FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
