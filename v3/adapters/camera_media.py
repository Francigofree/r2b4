"""Host-side calibrated media capture through the single vision owner."""
from __future__ import annotations

import math
import os
from contextlib import suppress
from pathlib import Path
import select
import shutil
import subprocess
import tempfile
import time

from v3.adapters.vision_media_socket import VisionClient


def _output_path(output: str | Path, suffixes: set[str]) -> Path:
    target = Path(output).expanduser().resolve()
    if target.suffix.lower() not in suffixes:
        raise ValueError("camera output must use " + " / ".join(sorted(suffixes)))
    if target.exists():
        raise FileExistsError(f"camera output already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    return target


def capture_photo(
    output: str | Path,
    *,
    root: str | Path | None = None,
) -> dict[str, object]:
    target = _output_path(output, {".jpg", ".jpeg"})
    image = VisionClient(root=root).observe()
    with target.open("xb") as handle:
        handle.write(image.image_bytes)
    return {"output": str(target), "bytes": len(image.image_bytes), **image.metadata.to_jsonable()}


def _write_encoder_frame(fd: int, payload: bytes, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    remaining_payload = memoryview(payload)
    while remaining_payload:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0 or not select.select([], [fd], [], remaining_s)[1]:
            raise TimeoutError("calibrated video encoder stopped accepting frames")
        try:
            written = os.write(fd, remaining_payload)
        except BlockingIOError:
            continue
        if written <= 0:
            raise BrokenPipeError("calibrated video encoder closed its input")
        remaining_payload = remaining_payload[written:]


def capture_h264_video(
    output: str | Path,
    duration_s: float,
    *,
    bitrate: int = 4_000_000,
    fps: float = 5.0,
    write_timeout_s: float = 5.0,
    root: str | Path | None = None,
) -> dict[str, object]:
    duration = float(duration_s)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("camera video duration must be finite and positive")
    rate = float(fps)
    if not math.isfinite(rate) or not 0.1 <= rate <= 30.0:
        raise ValueError("camera video fps must be within [0.1, 30]")
    if not isinstance(bitrate, int) or isinstance(bitrate, bool) or bitrate <= 0:
        raise ValueError("camera video bitrate must be a positive integer")
    write_timeout = float(write_timeout_s)
    if not math.isfinite(write_timeout) or not 0.1 <= write_timeout <= 15.0:
        raise ValueError("camera video write timeout must be within [0.1, 15]")
    encoder = shutil.which("ffmpeg")
    if encoder is None:
        raise RuntimeError("calibrated video requires ffmpeg with the libx264 encoder")
    target = _output_path(output, {".mp4", ".h264"})
    fd, name = tempfile.mkstemp(prefix=".r2b4-camera-", suffix=target.suffix, dir=target.parent)
    os.close(fd)
    staging = Path(name)
    process: subprocess.Popen[bytes] | None = None
    frames = 0
    first: dict[str, object] | None = None
    last: dict[str, object] | None = None
    try:
        with tempfile.TemporaryFile() as errors, VisionClient(root=root).session() as session:
            process = subprocess.Popen(
                [encoder, "-hide_banner", "-loglevel", "error", "-y", "-f", "image2pipe",
                 "-vcodec", "mjpeg", "-framerate", str(rate), "-i", "pipe:0", "-an",
                 "-c:v", "libx264", "-threads", "1", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
                 "-b:v", str(bitrate), str(staging)],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors,
            )
            assert process.stdin is not None
            encoder_fd = process.stdin.fileno()
            os.set_blocking(encoder_fd, False)
            started: float | None = None
            next_frame = time.monotonic()
            while started is None or time.monotonic() < started + duration:
                if started is not None:
                    delay = min(next_frame, started + duration) - time.monotonic()
                    if delay > 0.0:
                        time.sleep(delay)
                    if time.monotonic() >= started + duration:
                        break
                image = session.observe()
                metadata = image.metadata.to_jsonable()
                if first is not None:
                    if metadata["owner_generation"] != first["owner_generation"]:
                        raise RuntimeError("vision owner generation changed during video capture")
                    if (metadata["width"], metadata["height"], metadata["calibration_id"]) != (
                        first["width"], first["height"], first["calibration_id"],
                    ):
                        raise RuntimeError("calibrated camera geometry changed during video capture")
                    if last is not None and metadata["source_sequence"] <= last["source_sequence"]:
                        raise RuntimeError("vision owner returned a repeated video frame")
                if started is None:
                    started = time.monotonic()
                    next_frame = started
                    first = metadata
                try:
                    _write_encoder_frame(encoder_fd, image.image_bytes, write_timeout)
                except BrokenPipeError as exc:
                    errors.seek(0)
                    detail = errors.read(1500).decode("utf-8", "replace").strip()
                    raise RuntimeError(f"calibrated video encoder failed: {detail or 'ffmpeg closed its input'}") from exc
                frames += 1
                last = metadata
                next_frame += 1.0 / rate
            process.stdin.close()
            return_code = process.wait(timeout=15.0)
            if return_code != 0:
                errors.seek(0)
                detail = errors.read(1500).decode("utf-8", "replace").strip()
                raise RuntimeError(f"calibrated video encoder failed ({return_code}): {detail}")
            if frames == 0 or staging.stat().st_size == 0:
                raise RuntimeError("calibrated video produced no frames")
        os.link(staging, target)
        return {
            "output": str(target), "bytes": target.stat().st_size, "frames": frames,
            "duration_s": duration, "fps": rate, "first_frame": first, "last_frame": last,
        }
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)
        if process is not None and process.stdin is not None and not process.stdin.closed:
            with suppress(OSError):
                process.stdin.close()
        staging.unlink(missing_ok=True)


__all__ = ["capture_h264_video", "capture_photo"]
