#!/usr/bin/env python3
"""Physical Camera Module 3 validation through the native R2B4 camera stack."""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict, replace
from pathlib import Path

from v3.adapters.camera_media import capture_h264_video, capture_photo
from v3.adapters.picamera2_camera import (
    NativePicamera2Camera,
    Picamera2CameraConfig,
    default_picamera2_factory,
    picamera2_camera_config_from_mapping,
    raspberry_pi_sensor_timestamp_to_monotonic_ns,
)



def _refuse_parallel_runtime_owner() -> None:
    repo = Path(__file__).resolve().parents[1]
    pid_path = repo / "runtime" / ".r2b4_runtime_pid"
    if not pid_path.is_file():
        return
    try:
        pid = int(pid_path.read_text(encoding="utf-8").strip())
        os.kill(pid, 0)
    except (OSError, ValueError):
        return
    raise SystemExit(
        f"refusing parallel camera ownership while R2B4 runtime PID {pid} is alive; "
        "stop the runtime first"
    )


def _config(args: argparse.Namespace) -> Picamera2CameraConfig:
    repo = Path(__file__).resolve().parents[1]
    path = repo / "conf" / "hardver.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        camera = document.get("camera")
    except (OSError, UnicodeError, json.JSONDecodeError, AttributeError) as exc:
        raise SystemExit(f"cannot load native camera config from {path}: {exc}") from exc
    if not isinstance(camera, dict) or camera.get("enabled") is not True:
        raise SystemExit("conf/hardver.json camera must be enabled for physical camera tests")
    config = picamera2_camera_config_from_mapping(camera)
    changes = {}
    if args.camera_index is not None:
        changes["camera_index"] = args.camera_index
    if args.expected_model is not None:
        changes["expected_model"] = args.expected_model
    if args.fps is not None:
        changes["fps"] = args.fps
    return replace(config, **changes) if changes else config


def _status(args: argparse.Namespace) -> int:
    config = _config(args)
    owner = NativePicamera2Camera(
        config,
        picamera_factory=default_picamera2_factory,
        sensor_timestamp_mapper=raspberry_pi_sensor_timestamp_to_monotonic_ns,
    )
    if not owner.start():
        print(json.dumps({"status": "FAIL", "camera": asdict(owner.get_runtime_status())}, default=str, indent=2))
        return 1
    started_ns = time.monotonic_ns()
    first_sequence = 0
    last_sequence = 0
    max_age_ns = 0
    max_completion_lag_ns = 0
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            edge = owner.wait_for_new_frame(last_sequence, timeout_s=1.0)
            frame = edge.frame
            if frame is None:
                if edge.status.last_error:
                    break
                continue
            if first_sequence == 0:
                first_sequence = frame.sequence
            last_sequence = frame.sequence
            if edge.status.frame_age_ns is not None:
                max_age_ns = max(max_age_ns, edge.status.frame_age_ns)
            max_completion_lag_ns = max(max_completion_lag_ns, frame.completion_lag_ns)
        final = owner.get_edge_snapshot()
        elapsed_s = max(1e-9, (time.monotonic_ns() - started_ns) / 1_000_000_000.0)
        observed_frames = max(0, last_sequence - first_sequence + 1) if first_sequence else 0
        result = {
            "status": "PASS" if final.status.running and final.frame is not None else "FAIL",
            "camera_model": final.status.camera_model,
            "configured_fps": config.fps,
            "observed_frames": observed_frames,
            "observed_fps": observed_frames / elapsed_s,
            "last_sequence": last_sequence,
            "frame_age_ms": (
                final.status.frame_age_ns / 1_000_000.0
                if final.status.frame_age_ns is not None
                else None
            ),
            "max_frame_age_ms": max_age_ns / 1_000_000.0,
            "completion_lag_ms": (
                final.status.completion_lag_ns / 1_000_000.0
                if final.status.completion_lag_ns is not None
                else None
            ),
            "max_completion_lag_ms": max_completion_lag_ns / 1_000_000.0,
            "last_error": final.status.last_error,
            "published_stream": config.stream_name,
            "published_size": (
                [final.frame.width, final.frame.height]
                if final.frame is not None
                else [config.published_width, config.published_height]
            ),
            "published_format": (
                final.frame.pixel_format
                if final.frame is not None
                else config.published_pixel_format
            ),
            "stride_bytes": final.frame.stride_bytes if final.frame is not None else None,
            "frame_size_bytes": (
                final.frame.frame_size_bytes if final.frame is not None else None
            ),
            "main_size": [config.main_width, config.main_height],
            "main_format": config.main_pixel_format,
            "queue": config.queue,
            "buffer_count": config.buffer_count,
            "continuous_autofocus": config.continuous_autofocus,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1
    finally:
        owner.stop()


def _photo(args: argparse.Namespace) -> int:
    result = capture_photo(Path(args.output), _config(args), warmup_s=args.warmup)
    print(json.dumps({"status": "PASS", **result}, indent=2, default=str))
    return 0


def _video(args: argparse.Namespace) -> int:
    result = capture_h264_video(
        Path(args.output),
        args.seconds,
        _config(args),
        bitrate=args.bitrate,
        warmup_s=args.warmup,
    )
    print(json.dumps({"status": "PASS", **result}, indent=2, default=str))
    return 0


def _common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--camera-index", type=int)
    parser.add_argument("--expected-model")
    parser.add_argument("--fps", type=float)


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="run a bounded live frame-stream probe")
    _common(status)
    status.add_argument("--seconds", type=float, default=5.0)
    status.set_defaults(func=_status)

    photo = sub.add_parser("photo", help="capture one JPEG with the native policy")
    _common(photo)
    photo.add_argument("output")
    photo.add_argument("--warmup", type=float, default=2.0)
    photo.set_defaults(func=_photo)

    video = sub.add_parser("video", help="record one bounded H.264 hardware test")
    _common(video)
    video.add_argument("output")
    video.add_argument("--seconds", type=float, default=10.0)
    video.add_argument("--warmup", type=float, default=1.0)
    video.add_argument("--bitrate", type=int, default=4_000_000)
    video.set_defaults(func=_video)

    args = parser.parse_args()
    _refuse_parallel_runtime_owner()
    if hasattr(args, "seconds") and args.seconds <= 0:
        parser.error("--seconds must be positive")
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
