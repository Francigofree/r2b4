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
    PersonDetectionRuntimeStatus,
)
from v3.config import ConfigResolver
from v3.contracts import DataField, Observation, TickContext
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


def test_vision_owner_projects_bbox_from_camera_geometry_ssot():
    result = _detect(_Camera(_valid_status()))
    assert result.geometry_state == "VALID"
    assert result.geometry_reason is None
    assert len(result.projections) == 1
    projection = result.projections[0]
    # Image-left is robot-left in the canonical base frame.
    assert projection.left_bearing_rad > projection.right_bearing_rad > 0.0
    assert projection.geometry_quality == "factory_nominal"


def test_invalid_runtime_geometry_keeps_2d_detection_but_blocks_spatial_projection():
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
    def __init__(self, result) -> None:
        self.result = result

    def get_detection_snapshot(self):
        return self.result

    def get_detection_status(self):
        return PersonDetectionRuntimeStatus(
            True,
            self.result.sequence,
            self.result.source_frame_sequence,
            0,
            None,
        )

    def stop(self):
        return None


def test_projected_bearings_cross_l0_as_bounded_semantic_fields():
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


def test_l4_consumes_projected_bearings_instead_of_reconstructing_fov():
    world = ShadowWorldModel(_world_config())
    detections = world._person_image_detections(
        _person_observation(state="VALID", include_bearings=True)
    )
    assert len(detections) == 1
    assert detections[0].left_bearing_rad == pytest.approx(0.55)
    assert detections[0].right_bearing_rad == pytest.approx(0.25)


def test_l4_fails_closed_for_explicit_invalid_geometry_projection():
    world = ShadowWorldModel(_world_config())
    assert world._person_image_detections(
        _person_observation(state="INVALID", include_bearings=False)
    ) == ()
