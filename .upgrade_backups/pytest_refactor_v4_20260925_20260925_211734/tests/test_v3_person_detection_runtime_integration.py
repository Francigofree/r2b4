from __future__ import annotations

import threading
import time
from pathlib import Path

from v3.adapters.person_photo_evidence import (
    PersonPhotoEvidenceConfig,
    PersonPhotoEvidenceRecorder,
)
from v3.adapters.picamera2_camera import NativePicamera2Camera, Picamera2CameraConfig
from v3.contracts import (
    AdmittedFrame,
    ActuatorRequest,
    CommandMode,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    LifecycleState,
    MissionConstraints,
    MissionIntent,
    MissionLifecycle,
    Observation,
    SafetyDecision,
    TickContext,
)
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.import_guard import validate_v3_imports
from v3.layers.l12_safety_final import FinalSafetyGate
from v3_process_runtime import load_resident_runtime_config


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


class PhotoPort:
    def __init__(self) -> None:
        self.requests: list[tuple[Path, str]] = []
        self.accept = True

    def request_jpeg(self, output, *, stream_name="lores"):
        self.requests.append((Path(output), stream_name))
        return self.accept


def _mission(tick: int, now_ns: int, mode=CommandMode.EXPLORE):
    from v3_test_fixtures import active_mission

    return active_mission(
        TickContext(tick, now_ns),
        mode=mode,
        mission_id="mission-operator-roomcruise-test",
    )


def _admitted(
    tick: int,
    now_ns: int,
    *,
    sequence: int | None,
    person: bool,
    frame_sequence: int | None = None,
) -> AdmittedFrame:
    context = TickContext(tick, now_ns)
    accepted = ()
    if sequence is not None:
        accepted = (
            Observation(
                kind="person_detection",
                source_device_id="PERSON_DETECTOR_FRONT",
                source_sequence=sequence,
                captured_monotonic_ns=now_ns - 20_000_000,
                values=(
                    DataField("age_ns", 20_000_000),
                    DataField("measurement_timing_valid", True),
                    DataField("measurement_stale", False),
                    DataField(
                        "source_frame_sequence",
                        frame_sequence if frame_sequence is not None else sequence + 10,
                    ),
                    DataField("inference_duration_ns", 31_000_000),
                    DataField("person_count", 1 if person else 0),
                    DataField("person_detected", person),
                ),
            ),
        )
    return AdmittedFrame(context, accepted, (), ())


def test_photo_evidence_counts_only_new_l2_admitted_results_and_explore(tmp_path):
    photo = PhotoPort()
    recorder = PersonPhotoEvidenceRecorder(
        photo,
        PersonPhotoEvidenceConfig(
            enabled=True,
            directory="pic",
            confirm_results=3,
            rearm_misses=2,
            minimum_interval_ns=1_000_000_000,
        ),
        project_root=tmp_path,
    )

    # 50 Hz ticks without a new L2 person observation are neutral: no fake miss.
    for tick in range(1, 6):
        now_ns = tick * 20_000_000
        assert recorder.observe(
            _admitted(tick, now_ns, sequence=None, person=False),
            _mission(tick, now_ns),
        ) is False

    for index, sequence in enumerate((1, 2, 3), start=10):
        now_ns = index * 100_000_000
        requested = recorder.observe(
            _admitted(index, now_ns, sequence=sequence, person=True),
            _mission(index, now_ns),
        )
        assert requested is (sequence == 3)

    assert len(photo.requests) == 1
    output, stream_name = photo.requests[0]
    assert output.parent == tmp_path / "pic"
    assert output.suffix == ".jpg"
    assert "det000003_frame000013" in output.name
    assert stream_name == "main"

    # A non-EXPLORE mission can never create camera evidence.
    now_ns = 2_000_000_000
    assert recorder.observe(
        _admitted(20, now_ns, sequence=4, person=True),
        _mission(20, now_ns, mode=CommandMode.TELEOP),
    ) is False
    assert len(photo.requests) == 1


def test_photo_evidence_rearms_after_real_person_free_detector_results(tmp_path):
    photo = PhotoPort()
    recorder = PersonPhotoEvidenceRecorder(
        photo,
        PersonPhotoEvidenceConfig(
            enabled=True,
            directory="pic",
            confirm_results=1,
            rearm_misses=2,
            minimum_interval_ns=100_000_000,
        ),
        project_root=tmp_path,
    )

    now_ns = 200_000_000
    assert recorder.observe(
        _admitted(1, now_ns, sequence=1, person=True),
        _mission(1, now_ns),
    ) is True

    # One miss is insufficient to rearm.
    now_ns = 400_000_000
    assert recorder.observe(
        _admitted(2, now_ns, sequence=2, person=False),
        _mission(2, now_ns),
    ) is False
    now_ns = 500_000_000
    assert recorder.observe(
        _admitted(3, now_ns, sequence=3, person=True),
        _mission(3, now_ns),
    ) is False

    # Two consecutive real misses rearm the next person episode.
    for tick, sequence, now_ns in (
        (4, 4, 700_000_000),
        (5, 5, 800_000_000),
    ):
        assert recorder.observe(
            _admitted(tick, now_ns, sequence=sequence, person=False),
            _mission(tick, now_ns),
        ) is False
    now_ns = 1_000_000_000
    assert recorder.observe(
        _admitted(6, now_ns, sequence=6, person=True),
        _mission(6, now_ns),
    ) is True
    assert len(photo.requests) == 2


class _SavingRequest:
    def __init__(self, timestamp_ns: int) -> None:
        self.timestamp_ns = timestamp_ns
        self.released = False

    def get_metadata(self):
        return {
            "SensorTimestamp": self.timestamp_ns,
            "ExposureTime": 10_000,
            "FrameDuration": 50_000,
            "AfState": 2,
            "LensPosition": 1.0,
        }

    def make_buffer(self, stream_name: str):
        assert stream_name == "lores"
        return bytes(640 * 360 * 3)

    def save(self, stream_name: str, output: str, *, format=None):
        assert stream_name == "lores"
        assert format == "jpeg"
        Path(output).write_bytes(b"test-jpeg")

    def release(self):
        self.released = True


class _LiveSavingCamera:
    camera_properties = {"Model": "imx708"}

    def __init__(self) -> None:
        self.configuration_kwargs = None
        self.stopped = False
        self.sequence = 0
        self.allow_capture = threading.Event()

    def create_video_configuration(self, **kwargs):
        self.configuration_kwargs = kwargs
        return {"configured": True}

    def configure(self, configuration):
        return None

    def stream_configuration(self, stream_name: str):
        spec = self.configuration_kwargs[stream_name]
        width, height = spec["size"]
        pixel_format = spec["format"]
        stride = width * 3 if pixel_format == "RGB888" else width
        framesize = stride * height if pixel_format == "RGB888" else stride * height * 3 // 2
        return {
            "size": (width, height),
            "format": pixel_format,
            "stride": stride,
            "framesize": framesize,
        }

    def set_controls(self, controls):
        return None

    def start(self):
        return None

    def capture_request(self):
        self.allow_capture.wait(1.0)
        if self.stopped:
            raise OSError("camera stopped")
        time.sleep(0.005)
        self.sequence += 1
        return _SavingRequest(1_000_000_000 + self.sequence * 50_000_000)

    def stop(self):
        self.stopped = True
        self.allow_capture.set()

    def close(self):
        return None


def test_photo_request_uses_existing_camera_owner_and_is_bounded(tmp_path):
    camera = _LiveSavingCamera()
    clock = [2_000_000_000]

    def monotonic_ns():
        clock[0] += 50_000_000
        return clock[0]

    owner = NativePicamera2Camera(
        Picamera2CameraConfig(max_frame_completion_lag_ns=2_000_000_000),
        picamera_factory=lambda index: camera,
        sensor_timestamp_mapper=lambda value: value,
        camera_controls_factory=lambda: {},
        monotonic_ns=monotonic_ns,
    )
    assert owner.start()
    output = tmp_path / "person.jpg"
    assert owner.request_jpeg(output, stream_name="lores") is True
    # Only one pending request is accepted; no unbounded photo queue exists.
    assert owner.request_jpeg(tmp_path / "second.jpg", stream_name="lores") is False
    camera.allow_capture.set()

    deadline = time.monotonic() + 1.0
    while owner.get_photo_status().saved_count < 1 and time.monotonic() < deadline:
        time.sleep(0.005)
    status = owner.get_photo_status()
    assert status.saved_count == 1
    assert status.last_error is None
    assert status.last_output == str(output)
    assert output.read_bytes() == b"test-jpeg"
    assert owner.get_runtime_status().running is True
    owner.stop()


def test_canonical_runtime_config_closes_noncritical_person_capability():
    runtime = load_resident_runtime_config(PROJECT_ROOT)
    sensors = runtime.sensor_inputs
    assert sensors.person_detection_backend is not None
    assert sensors.person_detection_backend.model_path == "models/efficientdet_lite0.tflite"
    assert sensors.inputs.person_detection_source is not None
    assert sensors.inputs.person_detection_source.device_id == "PERSON_DETECTOR_FRONT"
    assert sensors.person_photo_evidence is not None
    assert sensors.person_photo_evidence.directory == "pic"
    assert sensors.person_photo_evidence.stream_name == "main"
    assert "PERSON_DETECTOR_FRONT" not in PRODUCTION_CRITICAL_DEVICE_IDS


class _Writer:
    def __init__(self) -> None:
        self.values = []

    def write(self, value) -> None:
        self.values.append(value)


def test_failed_person_detector_does_not_gain_motor_safety_authority():
    context = TickContext(1, 1_000_000_000)
    writer = _Writer()
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
    health = (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
        DeviceHealth("RPLIDAR_C1", DeviceHealthState.OK),
        DeviceHealth(
            "PERSON_DETECTOR_FRONT",
            DeviceHealthState.FAILED,
            "PERSON_DETECTOR_RUNTIME_ERROR",
        ),
    )
    request = ActuatorRequest(context=context, left_normalized=0.2, right_normalized=0.2)
    result = gate.finalize(context, request, health, LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.ALLOW
    assert result.enabled is True


