"""Independent owner lifecycle and failure acceptance, with no hardware I/O."""
from __future__ import annotations

import multiprocessing as mp
import time
import uuid
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from v3.adapters.camera_ownership import camera_device_lock
from v3.adapters.person_detection import PersonDetectionRuntimeStatus, PersonDetectionSnapshot
from v3.adapters.picamera2_camera import CameraEdgeSnapshot, CameraRuntimeStatus, Picamera2CameraConfig
from v3.adapters.process_vision_port import CameraFrameControlSnapshot, ProcessVisionPort
from v3.adapters.vision_media_contracts import CameraJpegMetadata
from v3.adapters.vision_media_socket import VisionClient, VisionMediaServer
from v3.adapters.vision_owner import CameraVisionOwner

K = ((200.0, 0.0, 160.0), (0.0, 200.0, 120.0), (0.0, 0.0, 1.0))


class Camera:
    def __init__(self, mode="normal"):
        self.mode = mode
        self.running = False
        self.owner_generation = ""
        self.sequence = 0
        self.photo = SimpleNamespace(last_output=None, last_metadata=None, last_error=None)

    def start(self):
        self.guard = camera_device_lock()
        self.guard.__enter__()
        self.owner_generation = uuid.uuid4().hex
        self.running = True
        return True

    def stop(self):
        if self.running:
            self.running = False
            self.guard.__exit__(None, None, None)

    def get_runtime_status(self):
        return CameraRuntimeStatus(self.running, self.sequence, 0, None, "fake-imx708", 0)

    def get_edge_snapshot(self):
        if not self.running:
            return CameraEdgeSnapshot(self.get_runtime_status(), None)
        self.sequence += 1
        now = time.monotonic_ns()
        frame = CameraFrameControlSnapshot(self.sequence, now, now, now, 0, 0,
            320, 240, "RGB888", 960, 230400, "FIXED", 1.0,
            "CALIBRATED", "cal-test", K, 0, self.owner_generation)
        return CameraEdgeSnapshot(self.get_runtime_status(), frame)

    def request_jpeg(self, output, *, stream_name="lores"):
        frame = self.get_edge_snapshot().frame
        now = time.monotonic_ns()
        metadata = CameraJpegMetadata(frame.sequence, frame.sensor_timestamp_ns,
            frame.measurement_monotonic_ns, now, "cal-test", stream_name, 320, 240,
            K, self.owner_generation)
        if self.mode == "stale":
            metadata = replace(metadata, measurement_monotonic_ns=now-1_000_000_000)
        elif self.mode == "generation":
            metadata = replace(metadata, owner_generation="retired-owner")
        elif self.mode == "raw":
            metadata = replace(metadata, calibration_state="RAW")
        Path(output).write_bytes(b"\xff\xd8frame:" + str(frame.sequence).encode() + b"\xff\xd9")
        self.photo = SimpleNamespace(last_output=str(output), last_metadata=metadata, last_error=None)
        # A subsequent latest frame has a different identity. The JPEG reply
        # must still carry the saved photo's lineage, not this later snapshot.
        self.get_edge_snapshot()
        return True

    def get_photo_status(self):
        return self.photo


class Detector:
    def __init__(self, camera):
        self.camera = camera
        self.running = False
        self.sequence = 0
        self.latest = None

    def start(self):
        self.running = True
        return True

    def stop(self):
        self.running = False

    def get_detection_snapshot(self):
        if not self.running:
            return None
        frame = self.camera.get_edge_snapshot().frame
        self.sequence += 1
        self.latest = PersonDetectionSnapshot(self.sequence, frame.sequence,
            frame.measurement_monotonic_ns, time.monotonic_ns(), 0, (),
            owner_generation=frame.owner_generation)
        return self.latest

    def get_detection_status(self):
        return PersonDetectionRuntimeStatus(self.running, self.sequence,
            0 if self.latest is None else self.latest.source_frame_sequence, 0, None)


def run_owner(socket_path, ready, stopped, mode):
    owner = CameraVisionOwner(lambda: Camera(mode), Detector, idle_grace_s=0.1)
    server = VisionMediaServer(owner, socket_path, photo_timeout_s=0.5)
    try:
        server.start()
        ready.send(True)
        stopped.wait()
    finally:
        server.stop()
        owner.close()


def start_owner(tmp_path, mode="normal"):
    context = mp.get_context("spawn")
    receiving, sending = context.Pipe(duplex=False)
    stopped = context.Event()
    path = tmp_path / (mode + ".sock")
    process = context.Process(target=run_owner, args=(str(path), sending, stopped, mode))
    process.start()
    sending.close()
    assert receiving.poll(5), "fake owner did not start"
    assert receiving.recv()
    receiving.close()
    return process, stopped, VisionClient(path, timeout_s=1.5)


def eventually(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


def finish(process, stopped):
    stopped.set()
    process.join(3)
    if process.is_alive():
        process.kill()
        process.join(3)


def test_camera_process_demand_is_independent_shared_and_closes_on_last_release(tmp_path):
    process, stopped, client = start_owner(tmp_path)
    port = ProcessVisionPort(Picamera2CameraConfig(), None, socket_path=client.socket_path)
    try:
        assert client.status()["camera_state"] == "OFF"
        assert port.get_detection_snapshot() is None
        port.set_person_detection_demand(True)
        eventually(lambda: port.get_detection_snapshot() is not None)
        assert client.status()["detector_running"]
        result = port.get_detection_snapshot()
        assert result.owner_generation == port.get_edge_snapshot().frame.owner_generation
        assert not hasattr(port.get_edge_snapshot().frame, "image_bytes")
        with client.session() as image:
            result = image.observe()
            assert result.image_bytes == b"\xff\xd8frame:" + str(result.metadata.source_sequence).encode() + b"\xff\xd9"
            assert result.metadata.rectified_K == K
            assert client.status()["owner_pid"] == port.pid == process.pid
            assert client.status()["consumers"] == 2
            with pytest.raises(RuntimeError, match="already owned"):
                with camera_device_lock():
                    pass
            # V3 stop releases only its own socket; observation remains usable.
            port.stop()
            eventually(lambda: client.status()["consumers"] == 1)
            assert client.status()["running"]
            assert not client.status()["detector_running"]
            assert image.observe().metadata.owner_generation == result.metadata.owner_generation
            client.set_manual_demand(True)
            with client.session() as second:
                client.set_manual_demand(False)
                assert client.status()["consumers"] == 2
                assert second.observe().metadata.owner_generation == result.metadata.owner_generation
        eventually(lambda: client.status()["camera_state"] == "OFF")
        assert not client.status()["detector_running"]
        with camera_device_lock():
            pass
        # Observation alone reactivates an independent owner generation.
        later = client.observe()
        assert later.metadata.owner_generation != result.metadata.owner_generation
        eventually(lambda: client.status()["camera_state"] == "OFF")
    finally:
        port.stop()
        finish(process, stopped)


def test_camera_process_rejects_stale_raw_generation_and_crash_without_old_results(tmp_path):
    for mode, reason in (("stale", "STALE"), ("raw", "calibration"), ("generation", "GENERATION")):
        process, stopped, client = start_owner(tmp_path, mode)
        try:
            with pytest.raises(RuntimeError, match=reason):
                client.observe()
            eventually(lambda: client.status()["consumers"] == 0)
            eventually(lambda: client.status()["camera_state"] == "OFF")
        finally:
            finish(process, stopped)
    process, stopped, client = start_owner(tmp_path)
    port = ProcessVisionPort(Picamera2CameraConfig(), None, socket_path=client.socket_path)
    try:
        port.set_person_detection_demand(True)
        eventually(lambda: port.get_detection_snapshot() is not None)
        with client.session() as image:
            image.observe()
            process.kill()
            process.join(3)
            eventually(lambda: bool(port.get_detection_status().last_error))
            assert port.get_detection_snapshot() is None
            assert port.get_edge_snapshot().frame is None
            assert not port.get_runtime_status().running
            with pytest.raises((EOFError, OSError, RuntimeError)):
                image.observe()
        with camera_device_lock():
            pass
    finally:
        port.stop()
        finish(process, stopped)
