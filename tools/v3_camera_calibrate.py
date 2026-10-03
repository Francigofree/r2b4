#!/usr/bin/env python3
"""Maintenance-only empirical calibration for the R2B4 front camera.

This is the sole intentional raw-image path introduced by the calibrated-frame
upgrade.  It requires exclusive physical camera access.  It captures a
fixed-focus chessboard data set, solves an OpenCV pinhole calibration, scales K
back to the native sensor coordinate system, and writes a JSON file consumed by
the upgrade installer.

Example:
  python3 tools/v3_camera_calibrate.py \
    --lens-position 1.0 --cols 9 --rows 6 --square-mm 25 \
    --samples 24 --output /tmp/r2b4_camera_calibration.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from v3.adapters.camera_ownership import camera_device_lock  # noqa: E402


def _imports():
    try:
        import cv2  # type: ignore
        import numpy as np
        from libcamera import controls  # type: ignore
        from picamera2 import Picamera2  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "calibration requires Picamera2/libcamera and OpenCV (python3-opencv)"
        ) from exc
    return cv2, np, controls, Picamera2



def _pair(value):
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return int(value[0]), int(value[1])
    if hasattr(value, "width") and hasattr(value, "height"):
        return int(value.width), int(value.height)
    return None


def _crop_tuple(value):
    if isinstance(value, (tuple, list)) and len(value) == 4:
        return tuple(int(item) for item in value)
    if all(hasattr(value, name) for name in ("x", "y", "width", "height")):
        return (int(value.x), int(value.y), int(value.width), int(value.height))
    return None

def _descriptor(corners, width: int, height: int, np):
    points = corners.reshape(-1, 2)
    center = points.mean(axis=0)
    span = points.max(axis=0) - points.min(axis=0)
    area = float(span[0] * span[1]) / float(width * height)
    diagonal = math.hypot(width, height)
    first = points[0]
    last = points[-1]
    angle = math.atan2(float(last[1] - first[1]), float(last[0] - first[0]))
    return (
        float(center[0]) / width,
        float(center[1]) / height,
        area,
        angle / math.pi,
        diagonal,
    )


def _different(candidate, previous: list[tuple[float, ...]]) -> bool:
    if not previous:
        return True
    cx, cy, area, angle, _ = candidate
    for old in previous:
        distance = math.hypot(cx - old[0], cy - old[1])
        if distance < 0.055 and abs(area - old[2]) < 0.035 and abs(angle - old[3]) < 0.08:
            return False
    return True


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Empirically calibrate the R2B4 IMX708 camera")
    parser.add_argument("--lens-position", type=float, required=True,
                        help="fixed libcamera LensPosition (diopters) used by calibration and runtime")
    parser.add_argument("--lens-tolerance", type=float, default=0.05)
    parser.add_argument("--cols", type=int, default=9, help="inner chessboard corners horizontally")
    parser.add_argument("--rows", type=int, default=6, help="inner chessboard corners vertically")
    parser.add_argument("--square-mm", type=float, required=True, help="physical chessboard square size")
    parser.add_argument("--samples", type=int, default=24)
    parser.add_argument("--capture-width", type=int, default=1280)
    parser.add_argument("--capture-height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--minimum-interval", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--max-rms", type=float, default=1.5)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    for value, name in (
        (args.lens_tolerance, "lens-tolerance"),
        (args.square_mm, "square-mm"),
        (args.fps, "fps"),
        (args.minimum_interval, "minimum-interval"),
        (args.timeout, "timeout"),
        (args.max_rms, "max-rms"),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name} must be finite and positive")
    if not math.isfinite(args.lens_position) or args.lens_position < 0.0:
        raise ValueError("--lens-position must be finite and non-negative")
    if args.cols < 3 or args.rows < 3 or args.samples < 8:
        raise ValueError("use at least 3x3 inner corners and at least 8 samples")

    with camera_device_lock():
        return _calibrate(args)


def _calibrate(args: argparse.Namespace) -> int:
    cv2, np, controls, Picamera2 = _imports()
    camera = Picamera2(camera_num=args.camera_index)
    configured = False
    started = False
    try:
        properties = dict(camera.camera_properties)
        model = str(properties.get("Model", ""))
        if "imx708" not in model.casefold():
            raise RuntimeError(f"unexpected camera model {model!r}; expected IMX708")
        native = _pair(properties.get("PixelArraySize"))
        if native is None:
            raise RuntimeError("camera PixelArraySize is unavailable")
        native_width, native_height = native
        if native_width <= 0 or native_height <= 0:
            raise RuntimeError("camera native sensor size is invalid")

        configuration = camera.create_video_configuration(
            main={"size": (args.capture_width, args.capture_height), "format": "BGR888"},
            controls={"FrameRate": args.fps},
            buffer_count=4,
            queue=False,
        )
        camera.configure(configuration)
        configured = True
        camera.set_controls(
            {
                "AfMode": controls.AfModeEnum.Manual,
                "LensPosition": float(args.lens_position),
            }
        )
        camera.start()
        started = True
        time.sleep(1.0)

        object_template = np.zeros((args.rows * args.cols, 3), np.float32)
        object_template[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
        object_template *= float(args.square_mm) / 1000.0
        object_points = []
        image_points = []
        descriptors: list[tuple[float, ...]] = []
        calibration_crop: tuple[int, int, int, int] | None = None
        deadline = time.monotonic() + args.timeout
        next_allowed = 0.0
        pattern = (args.cols, args.rows)
        print(
            f"R2B4 calibration: move a {args.cols}x{args.rows} inner-corner chessboard through the field of view; "
            f"collecting {args.samples} diverse views at LensPosition={args.lens_position:.4f}",
            flush=True,
        )
        while len(image_points) < args.samples and time.monotonic() < deadline:
            request = camera.capture_request()
            try:
                metadata = request.get_metadata()
                crop = _crop_tuple(metadata.get("ScalerCrop"))
                if crop is None:
                    raise RuntimeError("camera ScalerCrop metadata is unavailable during calibration")
                if crop[0] < 0 or crop[1] < 0 or crop[2] <= 0 or crop[3] <= 0:
                    raise RuntimeError(f"invalid calibration ScalerCrop: {crop!r}")
                if crop[0] + crop[2] > native_width or crop[1] + crop[3] > native_height:
                    raise RuntimeError(f"calibration ScalerCrop is outside sensor: {crop!r}")
                if calibration_crop is None:
                    calibration_crop = crop
                elif crop != calibration_crop:
                    raise RuntimeError(
                        f"ScalerCrop changed during calibration: {calibration_crop!r} -> {crop!r}"
                    )
                maker = getattr(request, "make_array", None)
                if not callable(maker):
                    raise RuntimeError("Picamera2 request.make_array is unavailable")
                frame = maker("main")
            finally:
                request.release()
            if frame is None:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCornersSB(
                gray,
                pattern,
                flags=(
                    cv2.CALIB_CB_NORMALIZE_IMAGE
                    | cv2.CALIB_CB_EXHAUSTIVE
                    | cv2.CALIB_CB_ACCURACY
                ),
            )

            if not found:
                clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
                enhanced = clahe.apply(gray)

                found, corners = cv2.findChessboardCornersSB(
                    enhanced,
                    pattern,
                    flags=(
                        cv2.CALIB_CB_NORMALIZE_IMAGE
                        | cv2.CALIB_CB_EXHAUSTIVE
                        | cv2.CALIB_CB_ACCURACY
                    ),
                )

            if not found:
                found, classic = cv2.findChessboardCorners(
                    gray,
                    pattern,
                    flags=(
                        cv2.CALIB_CB_ADAPTIVE_THRESH
                        | cv2.CALIB_CB_NORMALIZE_IMAGE
                    ),
                )

                if found:
                    corners = cv2.cornerSubPix(
                        gray,
                        classic,
                        (11, 11),
                        (-1, -1),
                        (
                            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                            30,
                            0.001,
                        ),
                    )
            if not found or corners is None:
                continue
            now = time.monotonic()
            if now < next_allowed:
                continue
            descriptor = _descriptor(corners, args.capture_width, args.capture_height, np)
            if not _different(descriptor, descriptors):
                continue
            object_points.append(object_template.copy())
            image_points.append(corners.astype(np.float32))
            descriptors.append(descriptor)
            next_allowed = now + args.minimum_interval
            print(f"accepted {len(image_points):02d}/{args.samples}", flush=True)

        if len(image_points) < args.samples:
            raise RuntimeError(
                f"calibration timed out with only {len(image_points)}/{args.samples} diverse samples"
            )

        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            object_points,
            image_points,
            (args.capture_width, args.capture_height),
            None,
            None,
        )
        if not math.isfinite(float(rms)):
            raise RuntimeError("calibration returned a non-finite RMS error")

        reprojection_total = 0.0
        point_total = 0
        for obj, observed, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
            projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            error = cv2.norm(observed, projected, cv2.NORM_L2)
            reprojection_total += float(error * error)
            point_total += int(len(obj))
        mean_reprojection_px = math.sqrt(reprojection_total / max(1, point_total))

        if float(rms) > args.max_rms:
            raise RuntimeError(
                f"calibration RMS {float(rms):.4f}px exceeds --max-rms {args.max_rms:.4f}px"
            )

        if calibration_crop is None:
            raise RuntimeError("calibration did not observe a sensor crop")
        crop_x, crop_y, crop_width, crop_height = calibration_crop
        sx = crop_width / float(args.capture_width)
        sy = crop_height / float(args.capture_height)
        K_native = K.copy()
        K_native[0, 0] *= sx
        K_native[0, 2] = K_native[0, 2] * sx + crop_x
        K_native[1, 1] *= sy
        K_native[1, 2] = K_native[1, 2] * sy + crop_y
        K_native[2, :] = (0.0, 0.0, 1.0)
        coeffs = [float(value) for value in D.reshape(-1)[:5]]

        result = {
            "schema": "R2B4_CAMERA_CALIBRATION_V1",
            "camera_model": model,
            "capture_size": [args.capture_width, args.capture_height],
            "reference_size": [native_width, native_height],
            "calibration_sensor_crop": list(calibration_crop),
            "K": [[float(value) for value in row] for row in K_native],
            "distortion_model": "opencv_pinhole",
            "distortion_coefficients": coeffs,
            "reference_lens_position": float(args.lens_position),
            "lens_position_tolerance": float(args.lens_tolerance),
            "sample_count": len(image_points),
            "rms_reprojection_error_px": float(rms),
            "mean_reprojection_error_px": float(mean_reprojection_px),
            "accepted": True,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        print(f"wrote {args.output}", flush=True)
        return 0
    finally:
        if started:
            try:
                camera.stop()
            except Exception:
                pass
        try:
            camera.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"CALIBRATION_FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
