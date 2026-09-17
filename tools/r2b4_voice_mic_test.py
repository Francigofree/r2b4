#!/usr/bin/env python3
"""Motor-free live validation for the R2B4 voice microphone HAL."""

from __future__ import annotations

import argparse
import json
import struct
import time
from dataclasses import asdict

from v3.adapters.microphone import (
    MicrophoneStreamConfig,
    NativeUsbMicrophone,
    resolve_usb_microphone,
)


def _json(value: object) -> str:
    return json.dumps(value, indent=2, default=str)


def _probe() -> int:
    try:
        identity = resolve_usb_microphone()
    except Exception as exc:
        print(_json({"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}))
        return 1
    print(_json({"status": "PASS", "identity": asdict(identity)}))
    return 0


def _capture(seconds: float) -> int:
    owner = NativeUsbMicrophone(stream=MicrophoneStreamConfig())
    if not owner.start():
        print(_json({"status": "FAIL", "health": asdict(owner.health())}))
        return 1

    first_sequence = 0
    last_sequence = 0
    observed_frames = 0
    sequence_gaps = 0
    peak_abs_s16 = 0
    started = time.monotonic()
    deadline = started + seconds

    try:
        while time.monotonic() < deadline:
            frame = owner.port.read_after(last_sequence, timeout_s=1.0)
            if frame is None:
                continue
            if first_sequence == 0:
                first_sequence = frame.sequence
            elif frame.sequence != last_sequence + 1:
                sequence_gaps += frame.sequence - last_sequence - 1
            last_sequence = frame.sequence
            observed_frames += 1

            # Native contract is S16_LE mono.  This is diagnostic-only signal
            # evidence; it does not modify the HAL or perform preprocessing.
            for (sample,) in struct.iter_unpack("<h", frame.pcm):
                peak_abs_s16 = max(peak_abs_s16, abs(sample))
    finally:
        owner.stop()

    elapsed_s = max(1e-9, time.monotonic() - started)
    health = owner.health()
    expected_fps = 1000.0 / owner.stream.frame_duration_ms
    result = {
        "status": (
            "PASS"
            if observed_frames > 0
            and sequence_gaps == 0
            else "FAIL"
        ),
        "identity": asdict(owner.identity) if owner.identity is not None else None,
        "stream": {
            "sample_rate_hz": owner.stream.sample_rate_hz,
            "channels": owner.stream.channels,
            "sample_format": owner.stream.sample_format,
            "frame_duration_ms": owner.stream.frame_duration_ms,
            "frame_samples": owner.stream.frame_samples,
            "frame_bytes": owner.stream.frame_bytes,
        },
        "observed_frames": observed_frames,
        "observed_frame_rate_hz": observed_frames / elapsed_s,
        "expected_frame_rate_hz": expected_fps,
        "first_sequence": first_sequence,
        "last_sequence": last_sequence,
        "sequence_gaps": sequence_gaps,
        "ring_overwrites": health.ring_overwrite_count,
        "peak_abs_s16": peak_abs_s16,
        "signal_seen": peak_abs_s16 > 0,
        "last_error": health.last_error,
    }
    print(_json(result))
    return 0 if result["status"] == "PASS" else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("probe", help="resolve the configured physical USB microphone")

    capture = sub.add_parser("capture", help="capture native PCM without VAD/STT")
    capture.add_argument("--seconds", type=float, default=10.0)

    args = parser.parse_args()
    if args.command == "probe":
        return _probe()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")
    return _capture(args.seconds)


if __name__ == "__main__":
    raise SystemExit(main())
