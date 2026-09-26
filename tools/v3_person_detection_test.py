#!/usr/bin/env python3
"""Bounded live person-detection test using the native R2B4 camera owner."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from v3.adapters.camera_geometry import camera_geometry_config_from_mapping  # noqa: E402
from v3.adapters.litert_person_detector import (  # noqa: E402
    LiteRtPersonDetectorConfig,
    LiteRtSsdPersonDetector,
)
from v3.adapters.person_detection import NativePersonDetector  # noqa: E402
from v3.adapters.picamera2_camera import (  # noqa: E402
    NativePicamera2Camera,
    default_picamera2_factory,
    picamera2_camera_config_from_mapping,
    raspberry_pi_sensor_timestamp_to_monotonic_ns,
)


def _refuse_parallel_runtime_owner() -> None:
    pid_path = REPO_ROOT / "runtime" / ".r2b4_runtime_pid"
    if not pid_path.is_file():
        return
    try:
        pid = int(pid_path.read_text(encoding="utf-8").strip())
        os.kill(pid, 0)
    except (OSError, ValueError):
        return
    raise SystemExit(
        f"refusing parallel camera ownership while R2B4 runtime PID {pid} is alive; stop the runtime first"
    )


def _camera_config():
    path = REPO_ROOT / "conf" / "hardver.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        camera = document.get("camera")
    except (OSError, UnicodeError, json.JSONDecodeError, AttributeError) as exc:
        raise SystemExit(f"cannot load camera config from {path}: {exc}") from exc
    if not isinstance(camera, dict) or camera.get("enabled") is not True:
        raise SystemExit("conf/hardver.json camera must be enabled")
    # Device parser is intentionally strict, so keep person_detection outside
    # the physical camera mapping when that optional section is added later.
    geometry = camera.get("geometry")
    if not isinstance(geometry, dict):
        raise SystemExit("conf/hardver.json enabled camera requires geometry")
    camera_geometry = camera_geometry_config_from_mapping(geometry)
    camera_device = {
        key: value
        for key, value in camera.items()
        if key not in {"person_detection", "geometry"}
    }
    return picamera2_camera_config_from_mapping(camera_device), camera_geometry


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/efficientdet_lite0.tflite")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--threshold", type=float, default=0.45)
    parser.add_argument("--person-class-id", type=int, default=0)
    parser.add_argument("--max-detections", type=int, default=5)
    parser.add_argument("--threads", type=int, default=2)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    _refuse_parallel_runtime_owner()
    camera_config, camera_geometry = _camera_config()
    camera = NativePicamera2Camera(
        camera_config,
        camera_geometry_config=camera_geometry,
        picamera_factory=default_picamera2_factory,
        sensor_timestamp_mapper=raspberry_pi_sensor_timestamp_to_monotonic_ns,
    )
    if not camera.start():
        print(json.dumps({"status": "FAIL", "stage": "camera", "error": camera.get_runtime_status().last_error}, indent=2))
        return 1

    try:
        backend = LiteRtSsdPersonDetector(
            LiteRtPersonDetectorConfig(
                model_path=args.model,
                person_class_id=args.person_class_id,
                score_threshold=args.threshold,
                max_detections=args.max_detections,
                num_threads=args.threads,
            )
        )
        detector = NativePersonDetector(
            camera, backend, camera_geometry_config=camera_geometry
        )
        detector.start()
        first_result = 0
        last_result = 0
        result_count = 0
        person_frames = 0
        best_confidence = 0.0
        last_payload = None
        started = time.monotonic()
        try:
            deadline = started + args.seconds
            while time.monotonic() < deadline:
                result = detector.wait_for_new_detection(last_result, timeout_s=1.0)
                status = detector.get_detection_status()
                if result is None:
                    if not status.running and status.last_error:
                        break
                    continue
                if first_result == 0:
                    first_result = result.sequence
                last_result = result.sequence
                result_count += 1
                if result.detections:
                    person_frames += 1
                    best_confidence = max(best_confidence, result.detections[0].confidence)
                primary = result.primary
                last_payload = {
                    "result_sequence": result.sequence,
                    "source_frame_sequence": result.source_frame_sequence,
                    "person_count": len(result.detections),
                    "inference_ms": result.inference_duration_ns / 1_000_000.0,
                    "geometry_state": result.geometry_state,
                    "geometry_reason": result.geometry_reason,
                    "primary_bearing": (
                        {
                            "left_rad": result.projections[0].left_bearing_rad,
                            "right_rad": result.projections[0].right_bearing_rad,
                            "quality": result.projections[0].geometry_quality,
                        }
                        if result.projections
                        else None
                    ),
                    "primary": (
                        {
                            "confidence": primary.confidence,
                            "center_x": primary.box.center_x,
                            "center_y": primary.box.center_y,
                            "area": primary.box.area,
                            "box": {
                                "xmin": primary.box.xmin,
                                "ymin": primary.box.ymin,
                                "xmax": primary.box.xmax,
                                "ymax": primary.box.ymax,
                            },
                        }
                        if primary is not None
                        else None
                    ),
                }
                print(json.dumps(last_payload, sort_keys=True))
        finally:
            detector.stop()
        elapsed = max(1e-9, time.monotonic() - started)
        final = detector.get_detection_status()
        summary = {
            "status": "PASS" if final.last_error is None and result_count > 0 else "FAIL",
            "results": result_count,
            "result_rate_hz": result_count / elapsed,
            "frames_with_person": person_frames,
            "best_confidence": best_confidence,
            "last_result": last_payload,
            "last_error": final.last_error,
        }
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary["status"] == "PASS" else 1
    finally:
        camera.stop()


if __name__ == "__main__":
    raise SystemExit(main())
