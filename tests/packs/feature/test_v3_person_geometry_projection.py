from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

from v3.adapters.camera_geometry import (
    CameraGeometryStatus,
    SensorCrop,
    camera_geometry_config_from_mapping,
    camera_geometry_status_from_properties,
)
from v3.adapters.live_person_detection import (
    NativePersonDetectionConfig,
    NativePersonDetectionSource,
)
from v3.adapters.person_detection import (
    NativePersonDetector,
    PersonBox,
    PersonDetection,
    PersonDetectionProjection,
    PersonDetectionRuntimeStatus,
    PersonDetectionSnapshot,
)
from v3.config import ConfigResolver
from v3.contracts import DataField, DeviceHealthState, Observation, TickContext
from v3.layers.l4_world_model import ShadowWorldModel


ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()
)


def _geometry():
    hardware = json.loads((ROOT / "conf" / "hardver.json").read_text(encoding="utf-8"))
    return camera_geometry_config_from_mapping(hardware["camera"]["geometry"])


def _world_config():
    conf = ROOT / "conf"
    docs = [
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    ]
    return ConfigResolver.from_documents(*docs).runtime.composition.live_control.control.world_model


@dataclass(frozen=True)
class _Frame:
    sequence: int
    measurement_monotonic_ns: int
    width: int = 640
    height: int = 360
    pixel_format: str = "RGB888"
    stride_bytes: int = 1920
    image_bytes: bytes = b"x"
    sensor_crop: SensorCrop | None = SensorCrop(0, 0, 4608, 2592)


@dataclass(frozen=True)
class _Status:
    running: bool = True
    last_error: str | None = None


@dataclass(frozen=True)
class _Edge:
    status: _Status
    frame: _Frame | None


class _Camera:
    def __init__(self, geometry_status: CameraGeometryStatus) -> None:
        self._condition = threading.Condition()
        self._frame: _Frame | None = None
        self._geometry_status = geometry_status

    def publish(self, frame: _Frame) -> None:
        with self._condition:
            self._frame = frame
            self._condition.notify_all()

    def get_edge_snapshot(self):
        with self._condition:
            return _Edge(_Status(), self._frame)

    def wait_for_new_frame(self, after_sequence=0, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._frame is None or self._frame.sequence <= after_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return _Edge(_Status(), self._frame)
                self._condition.wait(remaining)
            return _Edge(_Status(), self._frame)

    def get_camera_geometry_status(self):
        return self._geometry_status


class _Backend:
    def detect(self, frame):
        return (PersonDetection(0.9, PersonBox(0.2, 0.10, 0.8, 0.30)),)


def _valid_status() -> CameraGeometryStatus:
    config = _geometry()
    return camera_geometry_status_from_properties(
        config,
        {
            "Model": "imx708_wide_noir",
            "PixelArraySize": (4608, 2592),
            "UnitCellSize": (1400, 1400),
            "Rotation": "0",
            "ScalerCropMaximum": (0, 0, 4608, 2592),
        },
    )


def _detect(camera: _Camera):
    detector = NativePersonDetector(
        camera,
        _Backend(),
        camera_geometry_config=_geometry(),
        worker_poll_s=0.01,
    )
    assert detector.start()
    camera.publish(_Frame(1, 1_000_000))
    result = detector.wait_for_new_detection(0, timeout_s=1.0)
    detector.stop()
    assert result is not None
    return result


def _check_vision_owner_projects_bbox_from_camera_geometry_ssot():
    result = _detect(_Camera(_valid_status()))
    assert result.geometry_state == "VALID"
    assert result.geometry_reason is None
    assert len(result.projections) == 1
    projection = result.projections[0]
    # Image-left is robot-left in the canonical base frame.
    assert projection.left_bearing_rad > projection.right_bearing_rad > 0.0
    assert projection.geometry_quality == "empirical"


def _check_invalid_runtime_geometry_keeps_2d_detection_but_blocks_spatial_projection():
    invalid = CameraGeometryStatus(
        "INVALID",
        "UNIT_CELL_MISMATCH",
        "imx708",
        (4608, 2592),
        (999, 999),
        "0",
        SensorCrop(0, 0, 4608, 2592),
    )
    result = _detect(_Camera(invalid))
    assert len(result.detections) == 1
    assert result.geometry_state == "INVALID"
    assert result.geometry_reason == "UNIT_CELL_MISMATCH"
    assert result.projections == ()
    source = NativePersonDetectionSource(_Port(result), NativePersonDetectionConfig())
    snapshot = source.read(TickContext(1, result.measurement_monotonic_ns + 1))
    assert snapshot.health.state.value == "DEGRADED"
    assert snapshot.health.reason == "PERSON_GEOMETRY_INVALID"


class _Port:
    def __init__(self, result, *, last_error=None) -> None:
        self.result = result
        self.last_error = last_error

    def get_detection_snapshot(self):
        return self.result

    def get_detection_status(self):
        return PersonDetectionRuntimeStatus(
            True,
            self.result.sequence,
            self.result.source_frame_sequence,
            0,
            self.last_error,
        )

    def stop(self):
        return None


def _check_projected_bearings_cross_l0_as_bounded_semantic_fields():
    result = _detect(_Camera(_valid_status()))
    source = NativePersonDetectionSource(_Port(result), NativePersonDetectionConfig())
    snapshot = source.read(TickContext(1, result.measurement_monotonic_ns + 1))
    assert len(snapshot.samples) == 1
    values = {item.key: item.value for item in snapshot.samples[0].values}
    assert values["geometry_projection_state"] == "VALID"
    assert values["person_000_bearing_left_rad"] == pytest.approx(
        result.projections[0].left_bearing_rad
    )
    assert values["person_000_bearing_right_rad"] == pytest.approx(
        result.projections[0].right_bearing_rad
    )


def _person_observation(*, state: str, include_bearings: bool) -> Observation:
    values = [
        DataField("person_detected", True),
        DataField("emitted_person_count", 1),
        DataField("person_000_confidence", 0.9),
        DataField("person_000_xmin", 0.1),
        DataField("person_000_ymin", 0.2),
        DataField("person_000_xmax", 0.3),
        DataField("person_000_ymax", 0.8),
        DataField("geometry_projection_state", state),
    ]
    if include_bearings:
        values.extend(
            (
                DataField("person_000_bearing_left_rad", 0.55),
                DataField("person_000_bearing_right_rad", 0.25),
                DataField("person_000_geometry_quality", "factory_nominal"),
            )
        )
    return Observation("person_detection", "PERSON_DETECTOR_FRONT", 1, 1000, tuple(values))


def _check_l4_consumes_projected_bearings_instead_of_reconstructing_fov():
    world = ShadowWorldModel(_world_config())
    detections = world._person_image_detections(
        _person_observation(state="VALID", include_bearings=True)
    )
    assert len(detections) == 1
    assert detections[0].left_bearing_rad == pytest.approx(0.55)
    assert detections[0].right_bearing_rad == pytest.approx(0.25)


def _check_l4_fails_closed_for_explicit_invalid_geometry_projection():
    world = ShadowWorldModel(_world_config())
    assert world._person_image_detections(
        _person_observation(state="INVALID", include_bearings=False)
    ) == ()


def test_person_geometry_calibrated_bearings_and_fail_closed_projection_contract():
    _check_vision_owner_projects_bbox_from_camera_geometry_ssot()
    _check_projected_bearings_cross_l0_as_bounded_semantic_fields()
    _check_l4_consumes_projected_bearings_instead_of_reconstructing_fov()
    _check_invalid_runtime_geometry_keeps_2d_detection_but_blocks_spatial_projection()
    _check_l4_fails_closed_for_explicit_invalid_geometry_projection()


def test_follow_degraded_projection_requires_fresh_lidar_qualified_target_and_replays(tmp_path):
    from rig import ROOT, resolved_config
    from v3.capture import CaptureSink
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import (
        CommandMode, CommandRequest, DeviceHealth, DeviceSample, LifecycleState,
        NavigationStatus, RawDeviceBatch, RejectionReason, SafetyDecision,
    )
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord
    from v3.replay import replay_capture

    config = resolved_config().runtime.composition.live_control.control
    source_config = NativePersonDetectionConfig()
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink("follow-degraded-projection", configuration={"production_control": config})
    # The target lies outside the alignment envelope: a qualified track must
    # produce positive heading guidance even with degraded camera calibration.
    detection = PersonDetection(.9, PersonBox(.1, .2, .3, .8))
    projection = PersonDetectionProjection(.9, .7, "degraded")
    modes = (
        ("DEGRADED", 0, None, False),
        ("DEGRADED", 0, None, False),
        ("DEGRADED", 0, None, True),
        ("INVALID", 0, None, True),
        ("DEGRADED", source_config.maximum_result_age_ns + 1, None, True),
        ("DEGRADED", 0, "inference failed", True),
        ("DEGRADED", 0, None, True),
    )
    try:
        for tick, (geometry, age_ns, error, cluster) in enumerate(modes):
            context = TickContext(tick, 2_000_000_000 + tick * 20_000_000)
            result = PersonDetectionSnapshot(
                sequence=tick + 1, source_frame_sequence=tick + 11,
                measurement_monotonic_ns=context.monotonic_ns - age_ns,
                completed_monotonic_ns=context.monotonic_ns, inference_duration_ns=0,
                detections=(detection,), geometry_state=geometry,
                geometry_reason="INTRINSIC_UNVERIFIED" if geometry == "DEGRADED" else "UNIT_CELL_MISMATCH",
                projections=() if geometry == "INVALID" else (projection,),
                owner_generation="follow-projection-owner",
            )
            source = NativePersonDetectionSource(_Port(result, last_error=error), source_config)
            snapshot = source.read(context)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick + 1, context.monotonic_ns,
                                    tuple(DataField(key, value) for key, value in values.items()))

            # Initial ticks have no matching LiDAR cluster: bearings
            # alone may never create a usable spatial person target.
            samples = (
                sample("WHEEL_ENCODERS", "wheel_velocity", left_mps=0.0, right_mps=0.0, trust=1.0),
                sample("BNO055_IMU", "ekf_heading", yaw_rad=0.0, omega_rad_s=0.0, confidence=1.0),
                sample("RPLIDAR_C1", "lidar_health", age_ns=0, point_count=80),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=1,
                       point_000_x_m=1.8, point_000_y_m=1.8 if cluster else -1.8,
                       point_000_quality=10),
            ) + snapshot.samples
            health = tuple(DeviceHealth(name, DeviceHealthState.OK)
                           for name in sorted(config.critical_device_ids)) + (snapshot.health,)
            command = CommandRequest(context, "follow-projection", CommandMode.FOLLOW_PERSON,
                                     (DataField("target_track_id", "person-1"),), tick)
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, samples, health), command, LifecycleState.ACTIVE,
            ))
            output = composition.run_tick(inputs)
            layers = {row.layer: row.output for row in output.trace.layers}
            assert output.trace.fault_layer is None
            world, navigation = layers["L4"], layers["L6"]
            if tick <= 2:
                assert snapshot.health == DeviceHealth("PERSON_DETECTOR_FRONT", DeviceHealthState.OK)
                values = {item.key: item.value for item in snapshot.samples[0].values}
                assert values["geometry_projection_state"] == "DEGRADED"
                assert values["geometry_projection_reason"] == "INTRINSIC_UNVERIFIED"
                assert values["person_000_geometry_quality"] == "degraded"
                assert snapshot.samples[0].captured_monotonic_ns == result.measurement_monotonic_ns
                assert values["source_frame_sequence"] == result.source_frame_sequence
                assert values["owner_generation"] == result.owner_generation
                assert world.person_detection_state is DeviceHealthState.OK
            if tick < 2:
                assert not world.obstacle_tracks
                assert navigation.reason == "PERSON_TARGET_NOT_AVAILABLE"
                assert not navigation.route
            elif tick == 2:
                target, = world.obstacle_tracks
                assert target.track_id == "person-1" and target.usable_at(context.monotonic_ns)
                assert navigation.status is NavigationStatus.ACTIVE
                assert navigation.route[0].yaw_rad > 0.0
                assert layers["L8"].requested_omega_rad_s > 0.0
            elif tick in (3, 4):
                # Previously qualified track is still usable; closed unhealthy
                # capability must revoke it before any prediction can move.
                assert any(track.usable_at(context.monotonic_ns) for track in world.obstacle_tracks)
                assert snapshot.health.state is DeviceHealthState.DEGRADED
                assert snapshot.health.reason == ("PERSON_GEOMETRY_INVALID" if tick == 3
                                                  else "PERSON_DETECTOR_RESULT_STALE")
                assert navigation.reason == "PERSON_CAPABILITY_UNAVAILABLE" and not navigation.route
                if tick == 4:
                    assert any(row.reason is RejectionReason.STALE for row in layers["L2"].rejected)
            else:
                assert navigation.reason == "PERSON_CAPABILITY_FAILED" and not navigation.route
            if tick != 2:
                assert output.final_actuation.left_output == output.final_actuation.right_output == 0.0
            assert output.final_actuation.safety_decision is not SafetyDecision.FAULT
            sink.write(ExecutionRecord(inputs, output))
    finally:
        composition.close()
    capture = sink.finalize("PASS", tmp_path / "follow-degraded-projection.json")
    replay = replay_capture(capture, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
