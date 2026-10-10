"""Owning vision-edge observation evidence; no camera or motor is opened."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import threading
import time

import pytest

from r2b4_orchestration.skill_observation import SkillObservationAdapter
from v3.adapters.skill_observation import EventRecorder, PersonObserver
from v3.adapters.vision_media_contracts import CameraJpegMetadata, VisionJpeg
from v3.adapters.vision_media_socket import VisionClient, VisionMediaServer


class DetectorOwner:
    generation = "test-generation"
    maximum_age_ns = 500_000_000

    def __init__(self):
        self.lock = threading.Lock()
        self.demands = set()
        self.latest = None
        self.error = None

    def publish(self, *, present=False, sequence=None, measured=None):
        with self.lock:
            sequence = sequence or (self.latest["sequence"] + 1 if self.latest else 1)
            measured = measured or time.monotonic_ns()
            self.latest = {"sequence": sequence, "source_frame_sequence": sequence * 2,
                "measurement_monotonic_ns": measured, "completed_monotonic_ns": time.monotonic_ns(),
                "owner_generation": self.generation, "geometry_state": "VALID",
                "detections": [{"confidence": 0.9, "box": {"xmin": 0.1, "xmax": 0.3, "ymin": 0.2, "ymax": 0.4}}]
                    if present else []}

    def acquire(self, *, person=False):
        assert person
        token = str(len(self.demands))
        self.demands.add(token)
        self.publish()
        return token

    def release(self, token):
        self.demands.discard(token)

    def control_state(self, generation):
        assert generation == self.generation
        with self.lock:
            return {"detection_status": {"running": self.error is None, "last_error": self.error},
                    "detection": self.latest}

    def status(self):
        return {"camera_state": "ON", "running": True}


def wait_for(predicate, timeout_s=2):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    raise AssertionError("expected edge state did not arrive")


def presence(observer):
    return next((event for event in observer.page()["events"] if event["kind"] == "qualified_presence"), None)


def test_observer_preserves_measurement_and_detects_loss_duplicates_failure():
    owner = DetectorOwner()
    observer = PersonObserver(owner)
    try:
        measured = time.monotonic_ns()
        owner.publish(present=True, sequence=4, measured=measured)
        event = wait_for(lambda: presence(observer))
        assert event["measurement_monotonic_ns"] == measured
        assert event["source_sequence"] == 4 and event["source_frame_sequence"] == 8
        assert event["owner_generation"] == owner.generation
        assert any(item.get("lost_results") == 2 for item in observer.page()["events"])
        # Re-reading the exact producer measurement cannot duplicate a trigger.
        time.sleep(0.05)
        assert len([item for item in observer.page()["events"] if item["kind"] == "qualified_presence"]) == 1
        owner.error = "DETECTOR_CRASH"
        wait_for(lambda: observer.page()["status"] == "FAILED")
        assert any(item["kind"] == "capability_failed" for item in observer.page()["events"])
    finally:
        observer.close()
    assert not owner.demands


@pytest.mark.parametrize("parameters,reason", [
    ({"object_kind": "mouse"}, "DETECTOR_UNAVAILABLE:mouse"),
    ({"object_kind": "door"}, "DETECTOR_UNAVAILABLE:door"),
    ({"region": "kitchen"}, "OBSERVATION_REGION_UNAVAILABLE"),
])
def test_uninstalled_recognition_and_semantic_regions_are_honest(parameters, reason):
    owner = DetectorOwner()
    with pytest.raises(RuntimeError, match=reason):
        PersonObserver(owner, **parameters)
    assert not owner.demands


def test_image_region_is_applied_and_interval_flush_uses_measurement_deadline():
    owner = DetectorOwner()
    observer = PersonObserver(owner, region={"xmin": 0.6, "xmax": 0.9, "ymin": 0.1, "ymax": 0.9})
    try:
        owner.publish(present=True)
        wait_for(lambda: observer.page()["last_measurement_monotonic_ns"] == owner.latest["measurement_monotonic_ns"])
        assert not presence(observer)
        until = time.monotonic_ns()
        # Completion can arrive during draining; its physical time is retained.
        timer = threading.Timer(0.03, lambda: owner.publish(measured=until + 1))
        timer.start()
        result = observer.finish(until_monotonic_ns=until, timeout_s=0.5)
        timer.join()
        assert result["finished"] and not result["partial"]
        assert all(event["measurement_monotonic_ns"] <= until for event in result["events"])
        assert result["latest_sequence"] == 0
    finally:
        observer.close()


def test_interval_timeout_reports_coverage_gap_instead_of_negative_detection():
    owner = DetectorOwner()
    observer = PersonObserver(owner)
    try:
        until = time.monotonic_ns()
        started = time.monotonic()
        result = observer.finish(until_monotonic_ns=until, timeout_s=0.02)
        assert time.monotonic() - started < 0.5
        assert result["partial"] and result["finished"]
        assert result["events"][0]["kind"] == "coverage_gap"
        assert result["events"][0]["measurement_monotonic_ns"] == until
        assert observer.finish(until_monotonic_ns=until, timeout_s=0)["partial"]
    finally:
        observer.close()


def test_finish_delivers_late_completion_measured_before_deadline():
    owner = DetectorOwner()
    observer = PersonObserver(owner)
    try:
        until = time.monotonic_ns()
        event_timer = threading.Timer(0.03, lambda: owner.publish(present=True, measured=until - 1))
        boundary_timer = threading.Timer(0.09, lambda: owner.publish(measured=until + 1))
        event_timer.start()
        boundary_timer.start()
        page = observer.finish(until_monotonic_ns=until, timeout_s=0.5)
        event_timer.join()
        boundary_timer.join()
        event = next(item for item in page["events"] if item["kind"] == "qualified_presence")
        assert event["measurement_monotonic_ns"] == until - 1
        assert event["completed_monotonic_ns"] > until
        assert page["finished"] and not page["partial"]
    finally:
        observer.close()


def test_event_history_is_bounded_with_explicit_consumer_loss():
    class BurstyOwner(DetectorOwner):
        def control_state(self, generation):
            self.publish(present=(self.latest["sequence"] % 4 == 1), sequence=self.latest["sequence"] + 2)
            return super().control_state(generation)

    owner = BurstyOwner()
    observer = PersonObserver(owner)
    try:
        page = wait_for(lambda: observer.page() if observer.page()["lost_events"] else None, timeout_s=7)
        assert 0 < len(page["events"]) <= 32
        assert page["lost_events"] > 0
        assert page["events"][0]["sequence"] > 1
    finally:
        observer.close()


def test_socket_and_direct_semantics_and_crash_cleanup(tmp_path):
    owner = DetectorOwner()
    server = VisionMediaServer(owner, tmp_path / "vision.sock")
    server.start()
    adapter = SkillObservationAdapter(tmp_path, client=VisionClient(tmp_path / "vision.sock", timeout_s=1))
    try:
        opened = adapter.call("observe.open", {}, invocation_id="worker-a")
        measured = time.monotonic_ns()
        owner.publish(present=True, measured=measured)
        def event_page():
            page = adapter.call("observe.events", {"handle": opened["handle"]}, "worker-a")
            return page if any(item["kind"] == "qualified_presence" for item in page["events"]) else None

        result = wait_for(event_page)
        event = next(item for item in result["events"] if item["kind"] == "qualified_presence")
        assert event["measurement_monotonic_ns"] == measured
        assert "image_bytes" not in json.dumps(result)
        assert owner.demands
        adapter.close_invocation("worker-a")
        wait_for(lambda: not owner.demands)
    finally:
        adapter.close()
        server.stop()


def test_recorder_accepts_without_waiting_and_reports_partial_finish(monkeypatch, tmp_path):
    owner = DetectorOwner()
    observer = PersonObserver(owner)
    release_encoder = threading.Event()
    encoded = threading.Event()
    source_sequence = 0

    def capture(stream, generation):
        nonlocal source_sequence
        source_sequence += 1
        now = time.monotonic_ns()
        return VisionJpeg(b"\xff\xd8test\xff\xd9", CameraJpegMetadata(
            source_sequence=source_sequence, sensor_timestamp_ns=now, measurement_monotonic_ns=now,
            completed_monotonic_ns=now, calibration_id="calibration", stream=stream, width=32, height=32,
            rectified_K=((10., 0., 16.), (0., 10., 16.), (0., 0., 1.)), owner_generation=generation))

    def encode(self, clip):
        encoded.set()
        release_encoder.wait(1)
        with self._lock:
            clip.update(status="COMPLETED", output=str(tmp_path / "event.mp4"), frames=len(clip["images"]))

    monkeypatch.setattr("v3.adapters.skill_observation.shutil.which", lambda _: "/test/ffmpeg")
    monkeypatch.setattr(EventRecorder, "_encode", encode)
    recorder = EventRecorder(observer, capture, output_dir=tmp_path, post_s=0, pre_s=0, fps=10)
    try:
        owner.publish(present=True)
        event = wait_for(lambda: presence(observer))
        started = time.monotonic()
        accepted = recorder.start(event)
        assert accepted["status"] == "ACCEPTED" and time.monotonic() - started < 0.1
        assert recorder.start(event)["clip_id"] == accepted["clip_id"]
        assert encoded.wait(1)
        partial = recorder.finish(timeout_s=0.01)
        assert partial["partial"]
        release_encoder.set()
        finished = recorder.finish(timeout_s=1)
        assert not finished["partial"] and finished["clips_completed"] == 1
        assert finished["clips"][0]["event"]["measurement_monotonic_ns"] == event["measurement_monotonic_ns"]
        assert "image_bytes" not in json.dumps(finished)
        result_path = finished["clip_results"]
        wait_for(lambda: Path(result_path).exists())
        assert json.loads(Path(result_path).read_text())["clip_id"] == accepted["clip_id"]
    finally:
        release_encoder.set()
        recorder.close()
        observer.close()


def test_event_clip_encodes_in_edge_and_preserves_frame_lineage(tmp_path):
    encoder = shutil.which("ffmpeg")
    if encoder is None:
        pytest.skip("ffmpeg is not installed")
    jpeg = subprocess.run([encoder, "-hide_banner", "-loglevel", "error", "-f", "lavfi",
        "-i", "color=c=black:s=32x32", "-frames:v", "1", "-f", "image2pipe", "-vcodec", "mjpeg",
        "-threads", "1", "pipe:1"], capture_output=True, check=True, timeout=5).stdout
    owner = DetectorOwner()
    observer = PersonObserver(owner)
    frame_sequence = 0

    def capture(stream, generation):
        nonlocal frame_sequence
        frame_sequence += 1
        now = time.monotonic_ns()
        return VisionJpeg(jpeg, CameraJpegMetadata(source_sequence=frame_sequence,
            sensor_timestamp_ns=now, measurement_monotonic_ns=now, completed_monotonic_ns=now,
            calibration_id="synthetic-calibration", stream=stream, width=32, height=32,
            rectified_K=((10., 0., 16.), (0., 10., 16.), (0., 0., 1.)), owner_generation=generation))

    recorder = EventRecorder(observer, capture, output_dir=tmp_path, pre_s=0.2, post_s=0.2, fps=10)
    try:
        owner.publish(present=True)
        event = wait_for(lambda: presence(observer))
        accepted = recorder.start(event)
        result = recorder.finish(timeout_s=3)
        clip = result["clips"][0]
        assert clip["clip_id"] == accepted["clip_id"]
        assert clip["status"] in {"PARTIAL", "COMPLETED"}, clip
        assert Path(clip["output"]).stat().st_size > 0
        assert clip["frames"] >= 2
        assert clip["first_frame"]["owner_generation"] == owner.generation
        assert clip["last_frame"]["source_sequence"] > clip["first_frame"]["source_sequence"]
        assert clip["event"]["measurement_monotonic_ns"] == event["measurement_monotonic_ns"]
        assert "image_bytes" not in json.dumps(result)
    finally:
        recorder.close()
        observer.close()


def test_proxy_cancellation_does_not_wait_behind_interval_finish(tmp_path):
    owner = DetectorOwner()
    server = VisionMediaServer(owner, tmp_path / "vision.sock")
    server.start()
    adapter = SkillObservationAdapter(tmp_path, client=VisionClient(tmp_path / "vision.sock", timeout_s=2))
    errors = []
    try:
        opened = adapter.call("observe.open", {}, "worker-a")

        def finish():
            try:
                adapter.call("observe.finish", {"handle": opened["handle"],
                    "until_monotonic_ns": time.monotonic_ns(), "timeout_s": 0.5}, "worker-a")
            except (OSError, EOFError):
                errors.append("disconnected")

        thread = threading.Thread(target=finish)
        thread.start()
        time.sleep(0.03)
        started = time.monotonic()
        adapter.close_invocation("worker-a")
        assert time.monotonic() - started < 0.2
        thread.join(timeout=1)
        assert errors == ["disconnected"]
        wait_for(lambda: not owner.demands)
    finally:
        adapter.close()
        server.stop()


def test_concurrent_observer_open_shares_one_invocation_connection(tmp_path):
    owner = DetectorOwner()
    server = VisionMediaServer(owner, tmp_path / "vision.sock")
    server.start()

    class Client(VisionClient):
        connections = 0

        def connect(self, **parameters):
            self.connections += 1
            time.sleep(0.03)
            return super().connect(**parameters)

    client = Client(tmp_path / "vision.sock", timeout_s=2)
    adapter = SkillObservationAdapter(tmp_path, client=client)
    opened = []
    errors = []

    def open_observer():
        try:
            opened.append(adapter.call("observe.open", {}, "worker-a"))
        except Exception as exc:
            errors.append(exc)

    try:
        threads = [threading.Thread(target=open_observer) for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=1)
        assert errors == []
        assert len(opened) == 2 and opened[0]["handle"] != opened[1]["handle"]
        assert client.connections == 1
        adapter.close_invocation("worker-a")
        wait_for(lambda: not owner.demands)
    finally:
        adapter.close()
        server.stop()
