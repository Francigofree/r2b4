from __future__ import annotations

import time
from dataclasses import dataclass

from v3.adapters.live_camera import NativeCameraConfig, NativeCameraSource
from v3.adapters.picamera2_camera import (
    CameraFrameSnapshot,
    CameraRuntimeStatus,
    NativePicamera2Camera,
    Picamera2CameraConfig,
)
from v3.contracts import (
    ActuatorRequest,
    DeviceHealth,
    DeviceHealthState,
    LifecycleState,
    SafetyDecision,
    TickContext,
)
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.layers.l12_safety_final import FinalSafetyGate


class _Request:
    def __init__(self) -> None:
        self.released = False

    def get_metadata(self):
        return {
            "SensorTimestamp": 1_000_000_000,
            "ExposureTime": 10_000,
            "FrameDuration": 66_667,
            "AfState": 2,
            "LensPosition": 1.25,
        }

    def make_buffer(self, stream_name: str):
        assert stream_name == "main"
        return b"camera-frame"

    def release(self) -> None:
        self.released = True


class _UnusedCamera:
    pass


def test_camera_frame_keeps_sensor_timestamp_without_unvalidated_exposure_shift():
    owner = NativePicamera2Camera(
        Picamera2CameraConfig(),
        picamera_factory=lambda index: _UnusedCamera(),
        sensor_timestamp_mapper=lambda value: value + 100,
        monotonic_ns=lambda: 2_000_000_000,
    )
    snapshot = owner.capture_once_for_test(_Request())
    assert snapshot.sensor_timestamp_ns == 1_000_000_000
    assert snapshot.exposure_time_ns == 10_000_000
    assert snapshot.measurement_monotonic_ns == 1_000_000_100
    assert snapshot.completed_monotonic_ns == 2_000_000_000
    assert snapshot.image_bytes == b"camera-frame"


@dataclass
class _Port:
    frame: CameraFrameSnapshot | None
    status: CameraRuntimeStatus

    def get_latest_frame(self):
        return self.frame

    def get_runtime_status(self):
        return self.status


def _frame(measurement_ns: int = 900) -> CameraFrameSnapshot:
    return CameraFrameSnapshot(
        sequence=7,
        sensor_timestamp_ns=900,
        measurement_monotonic_ns=measurement_ns,
        completed_monotonic_ns=950,
        exposure_time_ns=10,
        frame_duration_ns=50,
        width=640,
        height=360,
        pixel_format="RGB888",
        focus_state="2",
        lens_position=1.0,
        image_bytes=b"abc",
    )


def test_live_camera_exposes_metadata_but_not_raw_image_in_device_sample():
    source = NativeCameraSource(
        _Port(_frame(), CameraRuntimeStatus(True, 7, 100, None)),
        NativeCameraConfig(maximum_frame_age_ns=250),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.OK
    assert len(snapshot.samples) == 1
    values = {field.key: field.value for field in snapshot.samples[0].values}
    assert values["payload_bytes"] == 3
    assert "image_bytes" not in values
    assert snapshot.samples[0].captured_monotonic_ns == 900


def test_live_camera_marks_stale_frame_degraded_without_rewriting_measurement():
    source = NativeCameraSource(
        _Port(_frame(500), CameraRuntimeStatus(True, 7, 500, None)),
        NativeCameraConfig(maximum_frame_age_ns=250),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == "CAMERA_FRAME_STALE"
    assert snapshot.samples[0].captured_monotonic_ns == 500


class _Writer:
    def __init__(self) -> None:
        self.commands = []

    def write(self, command) -> None:
        self.commands.append(command)


def _request(context: TickContext) -> ActuatorRequest:
    return ActuatorRequest(
        context=context,
        left_normalized=0.2,
        right_normalized=0.2,
    )


def _healthy_with_camera(camera_state=DeviceHealthState.FAILED):
    return (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
        DeviceHealth("RPLIDAR_C1", DeviceHealthState.OK),
        DeviceHealth("CAMERA_FRONT", camera_state, "CAMERA_RUNTIME_ERROR"),
    )


def test_noncritical_failed_camera_does_not_gain_l12_motor_authority():
    context = TickContext(1, 1_000)
    writer = _Writer()
    gate = FinalSafetyGate(
        writer,
        critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS,
    )
    result = gate.finalize(
        context,
        _request(context),
        _healthy_with_camera(),
        LifecycleState.ACTIVE,
        None,
    )
    assert result.safety_decision is SafetyDecision.ALLOW
    assert result.enabled is True


def test_missing_configured_critical_device_stops_fail_closed():
    context = TickContext(1, 1_000)
    writer = _Writer()
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
    health = (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
    )
    result = gate.finalize(context, _request(context), health, LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.STOP
    assert result.reason == "CRITICAL_DEVICE_HEALTH_MISSING"
    assert result.enabled is False


def test_upstream_fault_has_priority_over_missing_critical_health():
    context = TickContext(1, 1_000)
    writer = _Writer()
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
    health = (DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),)
    result = gate.finalize(
        context,
        _request(context),
        health,
        LifecycleState.ACTIVE,
        "L11_ERROR",
    )
    assert result.safety_decision is SafetyDecision.FAULT
    assert result.reason == "L11_ERROR"
    assert gate.fault_latched


def test_failed_critical_device_has_priority_over_other_missing_health():
    context = TickContext(1, 1_000)
    writer = _Writer()
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
    health = (DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.FAILED, "ENCODER_FAIL"),)
    result = gate.finalize(context, _request(context), health, LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.FAULT
    assert result.reason == "CRITICAL_DEVICE_FAILED"
    assert gate.fault_latched


class _FailingCaptureCamera:
    def __init__(self, name: str, events: list[str]) -> None:
        self.name = name
        self.events = events

    def create_video_configuration(self, **kwargs):
        return {}

    def configure(self, configuration) -> None:
        self.events.append(f"configure:{self.name}")

    def start(self) -> None:
        self.events.append(f"start:{self.name}")

    def capture_request(self):
        self.events.append(f"capture:{self.name}")
        raise OSError(f"capture failed {self.name}")

    def stop(self) -> None:
        self.events.append(f"stop:{self.name}")

    def close(self) -> None:
        self.events.append(f"close:{self.name}")


def test_restart_closes_failed_previous_picamera_handle_before_new_open():
    events: list[str] = []
    created = 0

    def factory(index: int):
        nonlocal created
        created += 1
        name = f"camera-{created}"
        events.append(f"factory:{name}")
        return _FailingCaptureCamera(name, events)

    owner = NativePicamera2Camera(
        Picamera2CameraConfig(stop_join_timeout_s=1.0),
        picamera_factory=factory,
        sensor_timestamp_mapper=lambda value: value,
    )
    assert owner.start()
    deadline = time.monotonic() + 1.0
    while owner.get_runtime_status().running and time.monotonic() < deadline:
        time.sleep(0.005)
    assert owner.get_runtime_status().running is False

    assert owner.start()
    assert events.index("close:camera-1") < events.index("factory:camera-2")
    owner.stop()
