from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from v3.config import ConfigResolver


ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()
)


def _documents():
    conf = ROOT / "conf"
    return tuple(
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    )


def test_live_world_model_camera_projection_has_no_second_config_authority():
    hardware, physics, speed_map, control = _documents()
    world_document = control["layers"]["world_model"]
    assert "person_camera_horizontal_fov_rad" not in world_document
    assert "person_camera_yaw_offset_rad" not in world_document

    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    geometry = resolved.runtime.sensor_inputs.camera_geometry
    world = resolved.runtime.composition.live_control.control.world_model
    assert geometry is not None
    assert world.person_camera_horizontal_fov_rad == pytest.approx(
        math.radians(geometry.factory.horizontal_fov_deg)
    )
    assert world.person_camera_yaw_offset_rad == pytest.approx(
        math.radians(geometry.mount.yaw_deg)
    )


def test_legacy_live_camera_fov_key_is_rejected_instead_of_becoming_second_ssot():
    hardware, physics, speed_map, control = _documents()
    control = deepcopy(control)
    control["layers"]["world_model"]["person_camera_horizontal_fov_rad"] = math.radians(66.0)
    with pytest.raises(ValueError, match="person_camera_horizontal_fov_rad"):
        ConfigResolver.from_documents(hardware, physics, speed_map, control)
