from __future__ import annotations

import time
from dataclasses import dataclass

from v3.adapters.live_camera import NativeCameraConfig, NativeCameraSource
from v3.adapters.picamera2_camera import (
    CameraEdgeSnapshot,
    CameraFrameSnapshot,
    CameraRuntimeStatus,
    NativePicamera2Camera,
    Picamera2CameraConfig,
    picamera2_camera_config_from_mapping,
    raspberry_pi_sensor_timestamp_to_monotonic_ns,
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
    def __init__(self, timestamp=1_000_000_000, data=b"camera-frame") -> None:
        self.released = False
        self.timestamp = timestamp
        self.data = data

    def get_metadata(self):
        return {
            "SensorTimestamp": self.timestamp,
            "ExposureTime": 10_000,
            "FrameDuration": 50_000,
            "AfState": 2,
            "LensPosition": 1.25,
        }

    def make_buffer(self, stream_name: str):
        assert stream_name == "lores"
        return self.data

    def release(self) -> None:
        self.released = True


class _ConfiguredCamera:
    camera_properties = {"Model": "imx708"}

    def __init__(self, requests=None, events=None):
        self.requests = list(requests or [])
        self.events = events if events is not None else []
        self.configuration_kwargs = None
        self.controls = None

    def create_video_configuration(self, **kwargs):
        self.configuration_kwargs = kwargs
        return {"configured": True}

    def configure(self, configuration) -> None:
        self.events.append("configure")

    def set_controls(self, controls) -> None:
        self.controls = dict(controls)
        self.events.append("controls")

    def start(self) -> None:
        self.events.append("start")

    def capture_request(self):
        if self.requests:
            return self.requests.pop(0)
        raise OSError("test capture end")

    def stop(self) -> None:
        self.events.append("stop")

    def close(self) -> None:
        self.events.append("close")


def test_camera_config_mapping_closes_hardware_policy():
    config = picamera2_camera_config_from_mapping(
        {
            "camera_index": 0,
            "expected_model": "imx708",
            "stream_name": "lores",
            "width": 640,
            "height": 360,
            "pixel_format": "RGB888",
            "main_width": 1280,
            "main_height": 720,
            "main_pixel_format": "YUV420",
            "fps": 20.0,
            "buffer_count": 6,
            "queue": False,
            "continuous_autofocus": True,
        }
    )
    assert config.expected_model == "imx708"
    assert config.queue is False
    assert config.buffer_count == 6
    assert config.stream_name == "lores"


def test_camera_config_is_freshness_first_two_stream_policy():
    config = Picamera2CameraConfig()
    camera = _ConfiguredCamera()
    owner = NativePicamera2Camera(
        config,
        picamera_factory=lambda index: camera,
        sensor_timestamp_mapper=lambda value: value,
        camera_controls_factory=lambda: {"AfMode": "CONTINUOUS"},
        monotonic_ns=lambda: 2_000_000_000,
    )
    assert owner.start()
    deadline = time.monotonic() + 1.0
    while owner.get_runtime_status().running and time.monotonic() < deadline:
        time.sleep(0.005)
    kwargs = camera.configuration_kwargs
    assert kwargs["main"] == {"size": (1280, 720), "format": "YUV420"}
    assert kwargs["lores"] == {"size": (640, 360), "format": "RGB888"}
    assert kwargs["buffer_count"] == 6
    assert kwargs["queue"] is False
    assert camera.controls == {"AfMode": "CONTINUOUS"}
    owner.stop()


def test_camera_frame_keeps_sensor_timestamp_without_exposure_shift():
    clock = iter([2_000_000_000])
    owner = NativePicamera2Camera(
        Picamera2CameraConfig(max_frame_completion_lag_ns=2_000_000_000),
        picamera_factory=lambda index: _ConfiguredCamera(),
        sensor_timestamp_mapper=lambda value: value + 100,
        camera_controls_factory=lambda: {},
        monotonic_ns=lambda: next(clock),
    )
    snapshot = owner.capture_once_for_test(_Request())
    assert snapshot.sensor_timestamp_ns == 1_000_000_000
    assert snapshot.exposure_time_ns == 10_000_000
    assert snapshot.measurement_monotonic_ns == 1_000_000_100
    assert snapshot.completed_monotonic_ns == 2_000_000_000
    assert snapshot.image_bytes == b"camera-frame"
    assert snapshot.width == 640
    assert snapshot.height == 360


def test_raspberry_pi_timestamp_mapper_is_identity():
    assert raspberry_pi_sensor_timestamp_to_monotonic_ns(123_456_789) == 123_456_789


@dataclass
class _Port:
    edge: CameraEdgeSnapshot

    def get_edge_snapshot(self):
        return self.edge


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


def _edge(frame=None, *, running=True, error=None):
    frame = _frame() if frame is None and running else frame
    return CameraEdgeSnapshot(
        CameraRuntimeStatus(
            running,
            frame.sequence if frame is not None else 0,
            100 if frame is not None else None,
            error,
            "imx708",
            frame.completion_lag_ns if frame is not None else None,
        ),
        frame,
    )


def test_live_camera_exposes_metadata_but_not_raw_image():
    source = NativeCameraSource(_Port(_edge()), NativeCameraConfig(maximum_frame_age_ns=250))
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.OK
    assert len(snapshot.samples) == 1
    values = {field.key: field.value for field in snapshot.samples[0].values}
    assert values["payload_bytes"] == 3
    assert values["camera_model"] == "imx708"
    assert values["measurement_stale"] is False
    assert "image_bytes" not in values
    assert snapshot.samples[0].captured_monotonic_ns == 900


def test_live_camera_marks_stale_frame_degraded():
    frame = _frame(500)
    source = NativeCameraSource(
        _Port(_edge(frame)),
        NativeCameraConfig(maximum_frame_age_ns=250),
    )
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == "CAMERA_FRAME_STALE"
    values = {field.key: field.value for field in snapshot.samples[0].values}
    assert values["measurement_stale"] is True


def test_camera_port_exception_isolated_as_noncritical_failed_health():
    class BrokenPort:
        def get_edge_snapshot(self):
            raise OSError("camera disconnected")

    source = NativeCameraSource(BrokenPort(), NativeCameraConfig())
    snapshot = source.read(TickContext(1, 1_000))
    assert snapshot.health.state is DeviceHealthState.FAILED
    assert snapshot.health.reason == "CAMERA_PORT_ERROR"
    assert snapshot.samples == ()


class _Writer:
    def __init__(self) -> None:
        self.commands = []

    def write(self, command) -> None:
        self.commands.append(command)


def _request(context: TickContext) -> ActuatorRequest:
    return ActuatorRequest(context=context, left_normalized=0.2, right_normalized=0.2)


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
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
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


class _FailingCaptureCamera(_ConfiguredCamera):
    def __init__(self, name: str, events: list[str]) -> None:
        super().__init__(events=events)
        self.name = name

    def capture_request(self):
        self.events.append(f"capture:{self.name}")
        raise OSError(f"capture failed {self.name}")

    def stop(self) -> None:
        self.events.append(f"stop:{self.name}")

    def close(self) -> None:
        self.events.append(f"close:{self.name}")


def test_restart_closes_old_handle_and_clears_old_frame():
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
        camera_controls_factory=lambda: {},
    )
    assert owner.start()
    deadline = time.monotonic() + 1.0
    while owner.get_runtime_status().running and time.monotonic() < deadline:
        time.sleep(0.005)
    assert owner.get_runtime_status().running is False

    assert owner.start()
    assert events.index("close:camera-1") < events.index("factory:camera-2")
    assert owner.get_latest_frame() is None
    owner.stop()


def test_unexpected_sensor_model_refuses_start():
    camera = _ConfiguredCamera()
    camera.camera_properties = {"Model": "imx219"}
    owner = NativePicamera2Camera(
        Picamera2CameraConfig(expected_model="imx708"),
        picamera_factory=lambda index: camera,
        sensor_timestamp_mapper=lambda value: value,
        camera_controls_factory=lambda: {},
    )
    assert owner.start() is False
    status = owner.get_runtime_status()
    assert status.running is False
    assert "unexpected camera model" in (status.last_error or "")
