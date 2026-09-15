from __future__ import annotations

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


def test_camera_frame_closes_sensor_measurement_and_completion_times():
    owner = NativePicamera2Camera(
        Picamera2CameraConfig(),
        picamera_factory=lambda index: _UnusedCamera(),
        sensor_timestamp_mapper=lambda value: value + 100,
        monotonic_ns=lambda: 2_000_000_000,
    )
    snapshot = owner.capture_once_for_test(_Request())
    assert snapshot.sensor_timestamp_ns == 1_000_000_000
    assert snapshot.exposure_time_ns == 10_000_000
    assert snapshot.measurement_monotonic_ns == 995_000_100
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
        feedforward_left=0.2,
        feedforward_right=0.2,
        correction_left=0.0,
        correction_right=0.0,
    )


def test_noncritical_failed_camera_does_not_gain_l12_motor_authority():
    context = TickContext(1, 1_000)
    writer = _Writer()
    gate = FinalSafetyGate(
        writer,
        critical_device_ids=frozenset({"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}),
    )
    health = (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
        DeviceHealth("RPLIDAR_C1", DeviceHealthState.OK),
        DeviceHealth("CAMERA_FRONT", DeviceHealthState.FAILED, "CAMERA_RUNTIME_ERROR"),
    )
    result = gate.finalize(context, _request(context), health, LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.ALLOW
    assert result.enabled is True


def test_missing_configured_critical_device_stops_fail_closed():
    context = TickContext(1, 1_000)
    writer = _Writer()
    gate = FinalSafetyGate(
        writer,
        critical_device_ids=frozenset({"WHEEL_ENCODERS", "BNO055_IMU", "RPLIDAR_C1"}),
    )
    health = (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
    )
    result = gate.finalize(context, _request(context), health, LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.STOP
    assert result.reason == "CRITICAL_DEVICE_HEALTH_MISSING"
    assert result.enabled is False
