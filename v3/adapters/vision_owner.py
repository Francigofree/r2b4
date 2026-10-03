"""Demand-driven camera/vision owner, independent of the V3 control runtime."""
from __future__ import annotations

import argparse
import fcntl
import math
import os
import signal
import threading
import time
import uuid
from dataclasses import asdict
from pathlib import Path

from .person_detection import NativePersonDetector, PersonDetectionRuntimeStatus
from .picamera2_camera import NativePicamera2Camera, default_picamera2_factory, raspberry_pi_sensor_timestamp_to_monotonic_ns
from .litert_person_detector import LiteRtSsdPersonDetector
from .vision_media_socket import VisionMediaServer, default_vision_media_socket_path


class CameraVisionOwner:
    """A small demand set around the existing calibrated camera and detector."""

    def __init__(self, camera_factory, detector_factory=None, *, idle_grace_s: float = 1.0,
                 maximum_age_ns: int = 250_000_000) -> None:
        if not math.isfinite(idle_grace_s) or not 0 <= idle_grace_s <= 30:
            raise ValueError("camera idle grace must be within [0, 30]")
        if type(maximum_age_ns) is not int or maximum_age_ns <= 0:
            raise ValueError("camera maximum age must be positive")
        self._camera_factory = camera_factory
        self._detector_factory = detector_factory
        self.idle_grace_s = idle_grace_s
        self.maximum_age_ns = maximum_age_ns
        self._lock = threading.RLock()
        self._demands: dict[str, bool] = {}
        self._camera = None
        self._detector = None
        self._idle_deadline: float | None = None
        self._last_error: str | None = None
        self._detector_error: str | None = None
        self._closed = False
        self._stop = threading.Event()
        self._idle_thread = threading.Thread(target=self._idle_loop, name="vision-idle", daemon=True)
        self._idle_thread.start()

    @property
    def generation(self) -> str:
        with self._lock:
            return "" if self._camera is None else self._camera.owner_generation

    def _activate(self, person: bool) -> None:
        if self._closed:
            raise RuntimeError("VISION_OWNER_CLOSED")
        if self._camera is None:
            camera = None
            try:
                camera = self._camera_factory()
                self._camera = camera
                if not camera.start():
                    raise RuntimeError(camera.get_runtime_status().last_error or "VISION_CAMERA_FAILED")
            except BaseException as exc:
                self._last_error = f"{type(exc).__name__}:{exc}"
                try:
                    if camera is not None:
                        camera.stop()
                finally:
                    self._camera = None
                raise RuntimeError(self._last_error)
            self._last_error = None
        elif not self._camera.get_runtime_status().running:
            self._last_error = self._camera.get_runtime_status().last_error or "VISION_CAMERA_FAILED"
            raise RuntimeError(self._last_error)
        if person and (self._detector is None or not self._detector.get_detection_status().running):
            if self._detector_factory is None:
                self._detector_error = "PERSON_DETECTOR_UNAVAILABLE"
                raise RuntimeError(self._detector_error)
            try:
                detector = self._detector or self._detector_factory(self._camera)
                self._detector = detector
                if not detector.start():
                    raise RuntimeError(detector.get_detection_status().last_error or "PERSON_DETECTOR_FAILED")
                self._detector_error = None
            except Exception as exc:
                self._detector_error = f"{type(exc).__name__}:{exc}"
                if self._detector is not None:
                    self._detector.stop()
                    self._detector = None
                raise

    def acquire(self, *, person: bool = False) -> str:
        with self._lock:
            token = uuid.uuid4().hex
            try:
                self._activate(person)
            except BaseException:
                if not self._demands:
                    self._idle_deadline = time.monotonic() + self.idle_grace_s
                raise
            self._demands[token] = person
            self._idle_deadline = None
            return token

    def release(self, token: str) -> None:
        with self._lock:
            self._demands.pop(token, None)
            if not any(self._demands.values()) and self._detector is not None:
                try:
                    self._detector.stop()
                except Exception as exc:
                    self._detector_error = f"PERSON_DETECTOR_CLOSE_FAILED:{type(exc).__name__}:{exc}"
            if not self._demands:
                self._idle_deadline = time.monotonic() + self.idle_grace_s

    def set_manual_demand(self, active: bool) -> None:
        with self._lock:
            if active:
                if "manual" not in self._demands:
                    try:
                        self._activate(False)
                    except BaseException:
                        if not self._demands:
                            self._idle_deadline = time.monotonic() + self.idle_grace_s
                        raise
                    self._demands["manual"] = False
                    self._idle_deadline = None
            else:
                self.release("manual")

    def camera_for_generation(self, generation: str):
        with self._lock:
            if self._camera is None or self._camera.owner_generation != generation:
                raise RuntimeError("VISION_GENERATION_MISMATCH")
            status = self._camera.get_runtime_status()
            if not status.running or status.last_error:
                raise RuntimeError(status.last_error or "VISION_CAMERA_UNAVAILABLE")
            return self._camera

    def status(self) -> dict:
        with self._lock:
            camera_status = self._camera.get_runtime_status() if self._camera is not None else None
            detector_status = self._detector.get_detection_status() if self._detector is not None else None
            running = bool(camera_status and camera_status.running)
            error = self._last_error or (camera_status.last_error if camera_status else None)
            return {"running": running, "camera_state": "FAILED" if error else "ON" if running else "OFF",
                    "detector_running": bool(detector_status and detector_status.running),
                    "consumers": len(self._demands), "manual_demand": "manual" in self._demands,
                    "owner_generation": self.generation, "last_error": error,
                    "detector_last_error": self._detector_error or (detector_status.last_error if detector_status else None),
                    "owner_pid": os.getpid()}

    def control_state(self, generation: str) -> dict:
        from .process_vision_port import _wire_camera
        with self._lock:
            camera = self.camera_for_generation(generation)
            edge = camera.get_edge_snapshot()
            # Only compact semantic snapshots leave this process. Never asdict
            # the CameraFrameSnapshot: that would carry its image buffer.
            detector = self._detector
            detection = detector.get_detection_snapshot() if detector else None
            status = detector.get_detection_status() if detector else PersonDetectionRuntimeStatus(False, 0, 0, None, self._detector_error)
            return {"owner_generation": generation, "camera": _wire_camera(edge),
                    "detection": asdict(detection) if detection is not None else None,
                    "detection_status": asdict(status), "owner_pid": os.getpid()}

    def _deactivate(self) -> None:
        error = None
        if self._detector is not None:
            try:
                self._detector.stop()
                self._detector = None
            except Exception as exc:
                error = exc
        if self._camera is not None:
            self._camera.stop()
            self._camera = None
        self._idle_deadline = None
        if error is not None:
            raise error

    def _idle_loop(self) -> None:
        while not self._stop.wait(0.05):
            with self._lock:
                if not self._demands and self._idle_deadline is not None and time.monotonic() >= self._idle_deadline:
                    try:
                        self._deactivate()
                    except Exception as exc:
                        self._last_error = f"VISION_CLOSE_FAILED:{type(exc).__name__}:{exc}"
                        self._idle_deadline = None

    def close(self) -> None:
        self._stop.set()
        self._idle_thread.join(timeout=1)
        with self._lock:
            self._closed = True
            self._demands.clear()
            self._deactivate()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--socket", type=Path, default=default_vision_media_socket_path())
    args = parser.parse_args(argv)
    args.socket.parent.mkdir(parents=True, exist_ok=True)
    # Held for the service lifetime. A second launch exits before touching the
    # socket, configuration or camera; V3 never owns this process handle.
    with Path(str(args.socket) + ".owner.lock").open("a+b") as election:
        try:
            fcntl.flock(election, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        from v3.config import ConfigResolver
        from v3.runtime_performance import apply_process_cpuset
        resolved = ConfigResolver.for_project(args.root).resolve()
        config = resolved.runtime.sensor_inputs
        if resolved.affinity.enabled:
            apply_process_cpuset(resolved.affinity.vision_cpus, role="vision-owner-process", strict=resolved.affinity.strict)
        def camera_factory():
            if config.camera_device is None:
                raise RuntimeError("CAMERA_NOT_CONFIGURED")
            return NativePicamera2Camera(config.camera_device, camera_geometry_config=config.camera_geometry,
                picamera_factory=default_picamera2_factory,
                sensor_timestamp_mapper=raspberry_pi_sensor_timestamp_to_monotonic_ns)
        def detector_factory(camera):
            if config.person_detection_backend is None:
                raise RuntimeError("PERSON_DETECTOR_NOT_CONFIGURED")
            return NativePersonDetector(camera, LiteRtSsdPersonDetector(config.person_detection_backend),
                                        camera_geometry_config=config.camera_geometry)
        owner = CameraVisionOwner(camera_factory, detector_factory,
            idle_grace_s=float(os.environ.get("R2B4_VISION_IDLE_GRACE_S", "1.0")),
            maximum_age_ns=config.inputs.camera_source.maximum_frame_age_ns if config.inputs.camera_source else 250_000_000)
        server = VisionMediaServer(owner, args.socket)
        stopped = threading.Event()
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: stopped.set())
        try:
            server.start()
            while not stopped.wait(0.5):
                pass
        finally:
            server.stop()
            owner.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
