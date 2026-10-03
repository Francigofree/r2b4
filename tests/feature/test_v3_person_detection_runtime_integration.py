from __future__ import annotations
import threading
import time
from pathlib import Path
from v3.adapters.person_photo_evidence import PersonPhotoEvidenceConfig, PersonPhotoEvidenceRecorder
from v3.adapters.camera_geometry import camera_geometry_config_from_mapping
from v3.adapters.picamera2_camera import NativePicamera2Camera, Picamera2CameraConfig
from v3.contracts import AdmittedFrame, ActuatorRequest, CommandMode, DataField, DeviceHealth, DeviceHealthState, LifecycleState, MissionConstraints, MissionIntent, MissionLifecycle, Observation, SafetyDecision, TickContext
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.import_guard import validate_v3_imports
from v3.layers.l12_safety_final import FinalSafetyGate
from v3_process_runtime import load_resident_runtime_config
PROJECT_ROOT = Path(__import__('os').environ['R2B4_ROOT']).resolve() if __import__('os').environ.get('R2B4_ROOT') else next((p for p in Path(__file__).resolve().parents if (p / 'conf' / 'hardver.json').is_file() and (p / 'v3').is_dir()), Path.cwd())

class PhotoPort:

    def __init__(self) -> None:
        self.requests: list[tuple[Path, str]] = []
        self.accept = True

    def request_jpeg(self, output, *, stream_name='lores'):
        self.requests.append((Path(output), stream_name))
        return self.accept

def _mission(tick: int, now_ns: int, mode=CommandMode.EXPLORE):
    from v3_test_fixtures import active_mission
    return active_mission(TickContext(tick, now_ns), mode=mode, mission_id='mission-operator-roomcruise-test')

def _admitted(tick: int, now_ns: int, *, sequence: int | None, person: bool, frame_sequence: int | None=None) -> AdmittedFrame:
    context = TickContext(tick, now_ns)
    accepted = ()
    if sequence is not None:
        accepted = (Observation(kind='person_detection', source_device_id='PERSON_DETECTOR_FRONT', source_sequence=sequence, captured_monotonic_ns=now_ns - 20000000, values=(DataField('age_ns', 20000000), DataField('measurement_timing_valid', True), DataField('measurement_stale', False), DataField('source_frame_sequence', frame_sequence if frame_sequence is not None else sequence + 10), DataField('inference_duration_ns', 31000000), DataField('person_count', 1 if person else 0), DataField('person_detected', person))),)
    return AdmittedFrame(context, accepted, (), ())

def _check_photo_evidence_counts_only_new_l2_admitted_results_and_explore(tmp_path):
    photo = PhotoPort()
    recorder = PersonPhotoEvidenceRecorder(photo, PersonPhotoEvidenceConfig(enabled=True, directory='pic', confirm_results=3, rearm_misses=2, minimum_interval_ns=1000000000), project_root=tmp_path)
    for tick in range(1, 6):
        now_ns = tick * 20000000
        assert recorder.observe(_admitted(tick, now_ns, sequence=None, person=False), _mission(tick, now_ns)) is False
    for index, sequence in enumerate((1, 2, 3), start=10):
        now_ns = index * 100000000
        requested = recorder.observe(_admitted(index, now_ns, sequence=sequence, person=True), _mission(index, now_ns))
        assert requested is (sequence == 3)
    assert len(photo.requests) == 1
    output, stream_name = photo.requests[0]
    assert output.parent == tmp_path / 'pic'
    assert output.suffix == '.jpg'
    assert 'det000003_frame000013' in output.name
    assert stream_name == 'main'
    now_ns = 2000000000
    assert recorder.observe(_admitted(20, now_ns, sequence=4, person=True), _mission(20, now_ns, mode=CommandMode.TELEOP)) is False
    assert len(photo.requests) == 1

def _check_photo_evidence_rearms_after_real_person_free_detector_results(tmp_path):
    photo = PhotoPort()
    recorder = PersonPhotoEvidenceRecorder(photo, PersonPhotoEvidenceConfig(enabled=True, directory='pic', confirm_results=1, rearm_misses=2, minimum_interval_ns=100000000), project_root=tmp_path)
    now_ns = 200000000
    assert recorder.observe(_admitted(1, now_ns, sequence=1, person=True), _mission(1, now_ns)) is True
    now_ns = 400000000
    assert recorder.observe(_admitted(2, now_ns, sequence=2, person=False), _mission(2, now_ns)) is False
    now_ns = 500000000
    assert recorder.observe(_admitted(3, now_ns, sequence=3, person=True), _mission(3, now_ns)) is False
    for tick, sequence, now_ns in ((4, 4, 700000000), (5, 5, 800000000)):
        assert recorder.observe(_admitted(tick, now_ns, sequence=sequence, person=False), _mission(tick, now_ns)) is False
    now_ns = 1000000000
    assert recorder.observe(_admitted(6, now_ns, sequence=6, person=True), _mission(6, now_ns)) is True
    assert len(photo.requests) == 2

def _empirical_geometry():
    return camera_geometry_config_from_mapping(
        {
            "factory_profile": "imx708_wide_noir",
            "mount": {
                "x_m": 0.05, "y_m": 0.0, "z_m": 0.19,
                "roll_deg": 0.0, "pitch_deg": 15.0, "yaw_deg": 0.0,
            },
            "intrinsic": {
                "source": "empirical",
                "reference_size": [4608, 2592],
                "K": [[2400.0, 0.0, 2304.0], [0.0, 2400.0, 1296.0], [0.0, 0.0, 1.0]],
                "distortion_model": "opencv_pinhole",
                "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
                "reference_lens_position": 1.0,
                "lens_position_tolerance": 0.05,
            },
        }
    )


class _SavingRequest:

    def __init__(self, timestamp_ns: int) -> None:
        self.timestamp_ns = timestamp_ns
        self.released = False

    def get_metadata(self):
        return {'SensorTimestamp': self.timestamp_ns, 'ExposureTime': 10000, 'FrameDuration': 50000, 'AfState': 2, 'LensPosition': 1.0}

    def make_buffer(self, stream_name: str):
        assert stream_name == 'lores'
        return bytes(640 * 360 * 3)

    def save(self, stream_name: str, output: str, *, format=None):
        assert stream_name == 'lores'
        assert format == 'jpeg'
        Path(output).write_bytes(b'test-jpeg')

    def release(self):
        self.released = True

class _LiveSavingCamera:
    camera_properties = {'Model': 'imx708'}

    def __init__(self) -> None:
        self.configuration_kwargs = None
        self.stopped = False
        self.sequence = 0
        self.allow_capture = threading.Event()

    def create_video_configuration(self, **kwargs):
        self.configuration_kwargs = kwargs
        return {'configured': True}

    def configure(self, configuration):
        return None

    def stream_configuration(self, stream_name: str):
        spec = self.configuration_kwargs[stream_name]
        width, height = spec['size']
        pixel_format = spec['format']
        stride = width * 3 if pixel_format == 'RGB888' else width
        framesize = stride * height if pixel_format == 'RGB888' else stride * height * 3 // 2
        return {'size': (width, height), 'format': pixel_format, 'stride': stride, 'framesize': framesize}

    def set_controls(self, controls):
        return None

    def start(self):
        return None

    def capture_request(self):
        self.allow_capture.wait(1.0)
        if self.stopped:
            raise OSError('camera stopped')
        time.sleep(0.005)
        self.sequence += 1
        return _SavingRequest(1000000000 + self.sequence * 50000000)

    def stop(self):
        self.stopped = True
        self.allow_capture.set()

    def close(self):
        return None

def _check_photo_request_uses_existing_camera_owner_and_is_bounded(tmp_path):
    camera = _LiveSavingCamera()
    clock = [2000000000]

    def monotonic_ns():
        clock[0] += 50000000
        return clock[0]
    owner = NativePicamera2Camera(Picamera2CameraConfig(max_frame_completion_lag_ns=2000000000), camera_geometry_config=_empirical_geometry(), picamera_factory=lambda index: camera, sensor_timestamp_mapper=lambda value: value, camera_controls_factory=lambda: {}, monotonic_ns=monotonic_ns)
    assert owner.start()
    output = tmp_path / 'person.jpg'
    assert owner.request_jpeg(output, stream_name='lores') is True
    assert owner.request_jpeg(tmp_path / 'second.jpg', stream_name='lores') is False
    camera.allow_capture.set()
    deadline = time.monotonic() + 1.0
    while owner.get_photo_status().saved_count < 1 and time.monotonic() < deadline:
        time.sleep(0.005)
    status = owner.get_photo_status()
    assert status.saved_count == 1
    assert status.last_error is None
    assert status.last_output == str(output)
    assert output.read_bytes() == b'test-jpeg'
    assert owner.get_runtime_status().running is True
    owner.stop()

def _check_canonical_runtime_config_closes_noncritical_person_capability():
    runtime = load_resident_runtime_config(PROJECT_ROOT)
    sensors = runtime.sensor_inputs
    assert sensors.person_detection_backend is not None
    assert sensors.person_detection_backend.model_path == 'models/efficientdet_lite0.tflite'
    assert sensors.inputs.person_detection_source is not None
    assert sensors.inputs.person_detection_source.device_id == 'PERSON_DETECTOR_FRONT'
    assert sensors.person_photo_evidence is not None
    assert sensors.person_photo_evidence.directory == 'pic'
    assert sensors.person_photo_evidence.stream_name == 'main'
    assert 'PERSON_DETECTOR_FRONT' not in PRODUCTION_CRITICAL_DEVICE_IDS

class _Writer:

    def __init__(self) -> None:
        self.values = []

    def write(self, value) -> None:
        self.values.append(value)

def _check_failed_person_detector_does_not_gain_motor_safety_authority():
    context = TickContext(1, 1000000000)
    writer = _Writer()
    gate = FinalSafetyGate(writer, critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
    health = (DeviceHealth('WHEEL_ENCODERS', DeviceHealthState.OK), DeviceHealth('BNO055_IMU', DeviceHealthState.OK), DeviceHealth('RPLIDAR_C1', DeviceHealthState.OK), DeviceHealth('PERSON_DETECTOR_FRONT', DeviceHealthState.FAILED, 'PERSON_DETECTOR_RUNTIME_ERROR'))
    request = ActuatorRequest(context=context, left_normalized=0.2, right_normalized=0.2)
    result = gate.finalize(context, request, health, LifecycleState.ACTIVE, None)
    assert result.safety_decision is SafetyDecision.ALLOW
    assert result.enabled is True


def test_person_photo_evidence_uses_admitted_results_and_bounded_camera_request(tmp_path):
    _check_photo_evidence_counts_only_new_l2_admitted_results_and_explore(tmp_path)
    _check_photo_evidence_rearms_after_real_person_free_detector_results(tmp_path)
    _check_photo_request_uses_existing_camera_owner_and_is_bounded(tmp_path)


def _check_camera_demand_belongs_to_follow_mission_and_releases_on_stop_or_failure():
    _check_canonical_runtime_config_closes_noncritical_person_capability()
    _check_failed_person_detector_does_not_gain_motor_safety_authority()
    from dataclasses import replace
    from rig import resolved_config
    from test_v3_bno055_imu_backend import Device
    from test_v3_encoder_ab_direction_robustness import FakeGpio
    from test_v3_latest_lidar_backend import Port
    from v3.adapters.process_vision_port import UnavailableVisionPort
    from v3.contracts import FinalActuation, NavigationPlan, NavigationStatus
    from v3.engine import LayerRecord, TickResult, TickTrace
    from v3_hardware_runtime import NativeHardwareSensorOwner

    class VisionConsumer(UnavailableVisionPort):
        def __init__(self, *args, **kwargs):
            super().__init__('unused offline camera')
            self.demands = []
            self.closed = False

        def set_person_detection_demand(self, active):
            self.demands.append(active)

        def stop(self):
            self.set_person_detection_demand(False)
            self.closed = True

    vision = VisionConsumer()
    sensors = replace(resolved_config().runtime.sensor_inputs, person_photo_evidence=None)
    imu, lidar = Device({}), Port()
    owner = NativeHardwareSensorOwner(
        FakeGpio(), lambda bus: None, lambda pose: lidar, sensors,
        open_imu_device=lambda config: imu,
        open_vision_port=lambda *args, **kwargs: vision,
    )

    def publish(tick, mode, *, reason=None, mission_id='follow-demand'):
        mission = replace(_mission(tick, 1_000_000_000 + tick * 20_000_000), mode=mode, mission_id=mission_id)
        navigation = NavigationPlan(mission.context, mission.mission_id, (), None,
                                    mission.constraints, 0.0, 0.0,
                                    NavigationStatus.INVALIDATED, reason or 'PERSON_TARGET_NOT_AVAILABLE')
        final = FinalActuation(mission.context, 0.0, 0.0, False, SafetyDecision.STOP,
                               'CLEAR', reason or 'NAVIGATION_HOLD')
        owner.publish_tick_result(TickResult(final, TickTrace(mission.context,
                                  (LayerRecord('L5', mission), LayerRecord('L6', navigation)))))

    try:
        assert owner.inputs.camera_frame_port is vision
        assert vision.demands == []  # V3 start creates no active camera demand.
        publish(1, CommandMode.EXPLORE)
        assert vision.demands[-1] is False
        publish(2, CommandMode.FOLLOW_PERSON)
        assert vision.demands[-1] is True  # Acquisition may await its first result.
        publish(3, CommandMode.FOLLOW_PERSON, reason='PERSON_TARGET_LOST')
        assert vision.demands[-1] is False
        publish(4, CommandMode.FOLLOW_PERSON)
        assert vision.demands[-1] is False  # Continued heartbeat cannot reopen a terminal mission.
        publish(5, CommandMode.FACE_PERSON, mission_id='face-demand')
        assert vision.demands[-1] is True
        stop = replace(_mission(6, 1_120_000_000), mode=CommandMode.STOP,
                       lifecycle=MissionLifecycle.CANCELLED, stop_reason='COMMAND_STOP')
        owner.publish_tick_result(TickResult(
            FinalActuation(stop.context, 0.0, 0.0, False, SafetyDecision.STOP, 'CLEAR', 'COMMAND_STOP'),
            TickTrace(stop.context, (LayerRecord('L5', stop),)),
        ))
        assert vision.demands[-1] is False
        publish(7, CommandMode.FOLLOW_PERSON, mission_id='new-follow-demand')
        assert vision.demands[-1] is True
        publish(8, CommandMode.FOLLOW_PERSON, mission_id='new-follow-demand', reason='PERSON_CAPABILITY_FAILED')
        assert vision.demands[-1] is False
    finally:
        owner.close()
    assert vision.closed and vision.demands[-1] is False
    assert imu.close_calls == lidar.stop_calls == 1


def _check_follow_person_closed_camera_health_revokes_motion_and_replays_failure():
    from dataclasses import replace
    from rig import healthy_localization, resolved_config
    from v3.contracts import AcquisitionFrame, DeviceSample, LOCAL_FRAME_ID, NavigationStatus, RobotEstimate
    from v3.layers.l2_admission import InputAdmission
    from v3.layers.l4_world_model import ShadowWorldModel
    from v3.layers.l6_navigation import TrajectoryNavigator

    control = resolved_config().runtime.composition.live_control.control
    admission = InputAdmission(control.admission)
    world_model = ShadowWorldModel(control.world_model)
    navigator = TrajectoryNavigator(control.navigation, async_config=control.async_l6)

    def step(tick, state, *, mission_id='follow-health', mode=CommandMode.FOLLOW_PERSON, nav=navigator):
        context = TickContext(tick, 2_000_000_000 + tick * 20_000_000)
        fields = lambda **values: tuple(DataField(key, value) for key, value in values.items())
        samples = (
            DeviceSample('RPLIDAR_C1', 'lidar_health', tick, context.monotonic_ns,
                         fields(age_ns=0, point_count=80)),
            DeviceSample('PERSON_DETECTOR_FRONT', 'obstacle_track', tick, context.monotonic_ns,
                         fields(track_id='person-1', x_m=1.8, y_m=1.3, radius_m=.2,
                                vx_mps=0.0, vy_mps=0.0, confidence=1.0)),
        )
        health = (DeviceHealth('RPLIDAR_C1', DeviceHealthState.OK),
                  DeviceHealth('PERSON_DETECTOR_FRONT', state,
                               None if state is DeviceHealthState.OK else 'OFFLINE_CAPABILITY_UNAVAILABLE'))
        admitted = admission(AcquisitionFrame(context, samples, health))
        estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 (0.0,) * 25, localization_quality=healthy_localization())
        world = world_model(admitted, estimate)
        assert world.person_detection_state is state
        mission = replace(_mission(tick, context.monotonic_ns), mode=mode, mission_id=mission_id)
        return nav.evaluate(mission, estimate, world), mission, estimate, world

    pending, *_ = step(1, DeviceHealthState.UNKNOWN)
    assert pending.status is NavigationStatus.IDLE and not pending.route
    active, *_ = step(2, DeviceHealthState.OK)
    assert active.status is NavigationStatus.ACTIVE
    stale, *_ = step(3, DeviceHealthState.DEGRADED)
    assert stale.status is NavigationStatus.IDLE and not stale.route
    failed, *_ = step(4, DeviceHealthState.FAILED)
    assert failed.reason == 'PERSON_CAPABILITY_FAILED' and not failed.route
    restored = TrajectoryNavigator(control.navigation, async_config=control.async_l6)
    restored.restore(navigator.checkpoint())
    still_failed, mission, estimate, world = step(5, DeviceHealthState.OK)
    assert still_failed == restored.evaluate(mission, estimate, world)
    assert still_failed.reason == 'PERSON_CAPABILITY_FAILED'
    new_mission, *_ = step(6, DeviceHealthState.OK, mission_id='new-follow-health')
    assert new_mission.status is NavigationStatus.ACTIVE

    # Restart retains physical frame sequence but changes owner identity. An
    # old generation cannot return after the new one has entered input closure.
    from v3.contracts import RejectionReason
    for kind, device_id in (('camera_frame_health', 'CAMERA_FRONT'),
                            ('person_detection', 'PERSON_DETECTOR_FRONT')):
        live = InputAdmission(control.admission)

        def frame(tick, sequence, generation, measured_ns):
            context = TickContext(tick, 3_000_000_000 + tick * 20_000_000)
            return AcquisitionFrame(
                context,
                (DeviceSample(device_id, kind, sequence, measured_ns,
                              (DataField('owner_generation', generation),)),),
                (DeviceHealth(device_id, DeviceHealthState.OK),),
            )

        assert live(frame(1, 31, 'owner-one', 3_000_000_000)).accepted
        restarted = live(frame(2, 1, 'owner-two', 3_030_000_000))
        assert restarted.accepted[0].source_sequence == 1
        assert restarted.accepted[0].captured_monotonic_ns == 3_030_000_000
        replay = InputAdmission(control.admission)
        replay.restore(live.checkpoint())
        delayed = frame(3, 32, 'owner-one', 3_010_000_000)
        rejected = live(delayed)
        assert rejected == replay(delayed)
        assert not rejected.accepted
        assert rejected.rejected[0].reason is RejectionReason.OUT_OF_ORDER
        assert rejected.degraded_sources == (device_id,)
        missing_generation = frame(4, 2, '', 3_060_000_000)
        rejected = live(missing_generation)
        assert rejected == replay(missing_generation)
        assert rejected.rejected[0].reason is RejectionReason.UNTRUSTED
        current = frame(5, 2, 'owner-two', 3_080_000_000)
        assert live(current) == replay(current)
        assert live.checkpoint() == replay.checkpoint()


def _check_follow_person_closed_health_native_replay(tmp_path):
    from rig import ROOT, resolved_config
    from v3.capture import CaptureSink
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandRequest, DeviceSample, RawDeviceBatch
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord
    from v3.replay import replay_capture

    config = resolved_config().runtime.composition.live_control.control
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink('follow-camera-failure', configuration={'production_control': config})
    states = (DeviceHealthState.UNKNOWN, DeviceHealthState.OK,
              DeviceHealthState.FAILED, DeviceHealthState.OK)
    try:
        for tick, state in enumerate(states):
            context = TickContext(tick, 4_000_000_000 + tick * 20_000_000)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick + 1, context.monotonic_ns,
                                    tuple(DataField(key, value) for key, value in values.items()))

            samples = (
                sample('WHEEL_ENCODERS', 'wheel_velocity', left_mps=0.0, right_mps=0.0, trust=1.0),
                sample('BNO055_IMU', 'ekf_heading', yaw_rad=0.0, omega_rad_s=0.0, confidence=1.0),
                sample('RPLIDAR_C1', 'lidar_health', age_ns=0, point_count=80),
                sample('PERSON_DETECTOR_FRONT', 'person_detection', person_count=0,
                       person_detected=False, owner_generation='replay-owner'),
            )
            health = tuple(DeviceHealth(name, DeviceHealthState.OK)
                           for name in sorted(config.critical_device_ids)) + (
                DeviceHealth('PERSON_DETECTOR_FRONT', state,
                             None if state is DeviceHealthState.OK else 'CAMERA_CAPABILITY_UNAVAILABLE'),
            )
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, samples, health),
                CommandRequest(context, 'follow-camera-failure', CommandMode.FOLLOW_PERSON, (), tick),
                LifecycleState.ACTIVE,
            ))
            result = composition.run_tick(inputs)
            layers = {row.layer: row.output for row in result.trace.layers}
            assert result.trace.fault_layer is None
            assert layers['L2'].device_health == health
            assert layers['L4'].person_detection_state is state
            if tick >= 2:
                assert layers['L6'].reason == 'PERSON_CAPABILITY_FAILED'
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.0
            assert result.final_actuation.safety_decision is not SafetyDecision.FAULT
            sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    capture = sink.finalize('PASS', tmp_path / 'follow-camera-failure.json')
    replay = replay_capture(capture, project_root=ROOT)
    assert replay['status'] == 'MATCH', replay['diagnostics']


def test_follow_person_camera_consumer_demand_failure_and_generation_contract(tmp_path):
    _check_camera_demand_belongs_to_follow_mission_and_releases_on_stop_or_failure()
    _check_follow_person_closed_camera_health_revokes_motion_and_replays_failure()
    _check_follow_person_closed_health_native_replay(tmp_path)
