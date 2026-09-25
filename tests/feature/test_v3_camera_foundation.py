from __future__ import annotations
import time
from dataclasses import dataclass
from v3.adapters.live_camera import NativeCameraConfig, NativeCameraSource
from v3.adapters.picamera2_camera import CameraEdgeSnapshot, CameraFrameSnapshot, CameraRuntimeStatus, CameraStreamGeometry, NativePicamera2Camera, Picamera2CameraConfig, picamera2_camera_config_from_mapping, raspberry_pi_sensor_timestamp_to_monotonic_ns
from v3.contracts import ActuatorRequest, DeviceHealth, DeviceHealthState, LifecycleState, SafetyDecision, TickContext
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.layers.l12_safety_final import FinalSafetyGate

class _Request:

    def __init__(self, timestamp=1000000000, data=None) -> None:
        self.released = False
        self.timestamp = timestamp
        self.data = bytes(640 * 360 * 3) if data is None else data

    def get_metadata(self):
        return {'SensorTimestamp': self.timestamp, 'ExposureTime': 10000, 'FrameDuration': 50000, 'AfState': 2, 'LensPosition': 1.25}

    def make_buffer(self, stream_name: str):
        assert stream_name == 'lores'
        return self.data

    def release(self) -> None:
        self.released = True

class _ConfiguredCamera:
    camera_properties = {'Model': 'imx708'}

    def __init__(self, requests=None, events=None):
        self.requests = list(requests or [])
        self.events = events if events is not None else []
        self.configuration_kwargs = None
        self.controls = None

    def create_video_configuration(self, **kwargs):
        self.configuration_kwargs = kwargs
        return {'configured': True}

    def configure(self, configuration) -> None:
        self.events.append('configure')

    def stream_configuration(self, stream_name: str):
        spec = self.configuration_kwargs[stream_name]
        width, height = spec['size']
        pixel_format = spec['format']
        if pixel_format == 'RGB888':
            stride = width * 3
            framesize = stride * height
        elif pixel_format == 'YUV420':
            stride = width
            framesize = stride * height * 3 // 2
        else:
            raise AssertionError(pixel_format)
        return {'size': (width, height), 'format': pixel_format, 'stride': stride, 'framesize': framesize}

    def set_controls(self, controls) -> None:
        self.controls = dict(controls)
        self.events.append('controls')

    def start(self) -> None:
        self.events.append('start')

    def capture_request(self):
        if self.requests:
            return self.requests.pop(0)
        raise OSError('test capture end')

    def stop(self) -> None:
        self.events.append('stop')

    def close(self) -> None:
        self.events.append('close')

@dataclass
class _Port:
    edge: CameraEdgeSnapshot

    def get_edge_snapshot(self):
        return self.edge

def _frame(measurement_ns: int=900) -> CameraFrameSnapshot:
    return CameraFrameSnapshot(sequence=7, sensor_timestamp_ns=900, measurement_monotonic_ns=measurement_ns, completed_monotonic_ns=950, exposure_time_ns=10, frame_duration_ns=50, width=2, height=2, pixel_format='RGB888', stride_bytes=6, frame_size_bytes=12, focus_state='2', lens_position=1.0, image_bytes=b'abcdefghijkl')

def _edge(frame=None, *, running=True, error=None):
    frame = _frame() if frame is None and running else frame
    return CameraEdgeSnapshot(CameraRuntimeStatus(running, frame.sequence if frame is not None else 0, 100 if frame is not None else None, error, 'imx708', frame.completion_lag_ns if frame is not None else None), frame)

def test_live_camera_marks_stale_frame_degraded():
    frame = _frame(500)
    source = NativeCameraSource(_Port(_edge(frame)), NativeCameraConfig(maximum_frame_age_ns=250))
    snapshot = source.read(TickContext(1, 1000))
    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == 'CAMERA_FRAME_STALE'
    values = {field.key: field.value for field in snapshot.samples[0].values}
    assert values['measurement_stale'] is True

class _Writer:

    def __init__(self) -> None:
        self.commands = []

    def write(self, command) -> None:
        self.commands.append(command)

def _request(context: TickContext) -> ActuatorRequest:
    return ActuatorRequest(context=context, left_normalized=0.2, right_normalized=0.2)

def _healthy_with_camera(camera_state=DeviceHealthState.FAILED):
    return (DeviceHealth('WHEEL_ENCODERS', DeviceHealthState.OK), DeviceHealth('BNO055_IMU', DeviceHealthState.OK), DeviceHealth('RPLIDAR_C1', DeviceHealthState.OK), DeviceHealth('CAMERA_FRONT', camera_state, 'CAMERA_RUNTIME_ERROR'))

def test_noncritical_failed_camera_does_not_gain_l12_motor_authority():
    context = TickContext(1, 1000)
    writer = _Writer()
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
    result = gate.finalize(context, _request(context), _healthy_with_camera(), LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.ALLOW
    assert result.enabled is True

class _FailingCaptureCamera(_ConfiguredCamera):

    def __init__(self, name: str, events: list[str]) -> None:
        super().__init__(events=events)
        self.name = name

    def capture_request(self):
        self.events.append(f'capture:{self.name}')
        raise OSError(f'capture failed {self.name}')

    def stop(self) -> None:
        self.events.append(f'stop:{self.name}')

    def close(self) -> None:
        self.events.append(f'close:{self.name}')
