from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path

import pytest

from v3.config import ConfigResolver


ROOT = (
    Path(os.environ["R2B4_ROOT"]).resolve()
    if os.environ.get("R2B4_ROOT")
    else next(
        (
            parent
            for parent in Path(__file__).resolve().parents
            if (parent / "conf" / "hardver.json").is_file()
            and (parent / "v3").is_dir()
        ),
        Path.cwd(),
    )
)


def _documents():
    conf = ROOT / "conf"
    return tuple(
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    )


def test_production_temporal_invariants_resolve_unchanged():
    resolved = ConfigResolver.from_documents(*_documents())
    assert resolved.lidar.maximum_result_age_ns == resolved.runtime.sensor_inputs.inputs.lidar_backend.maximum_result_age_ns
    for invalid_cadence in ("command_reader", "encoder_closure"):
        _check_cadence_cannot_outlive_its_acceptance_budget(invalid_cadence)


def test_l11_feedback_cache_cannot_outlive_admission_freshness():
    hardware, physics, speed_map, control = _documents()
    candidate = deepcopy(control)
    admission_age = candidate["layers"]["admission"]["max_sample_age_ns"]
    candidate["layers"]["wheel_pi"]["max_feedback_age_ns"] = admission_age + 1
    with pytest.raises(ValueError, match="feedback age exceeds admission freshness"):
        ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def test_world_freshness_has_one_authority_and_rejects_a_second():
    hardware, physics, speed_map, control = _documents()
    candidate = deepcopy(control)
    candidate["layers"]["navigation"]["max_world_freshness_ns"] += 1
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, candidate)
    layers = resolved.runtime.composition.live_control.control
    assert layers.motion_realization.max_world_freshness_ns == layers.navigation.max_world_freshness_ns
    candidate["layers"]["motion_realization"]["max_world_freshness_ns"] = layers.navigation.max_world_freshness_ns
    with pytest.raises(ValueError, match="unknown.*max_world_freshness_ns"):
        ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def test_existing_encoder_estimation_budget_invariant_is_preserved():
    hardware, physics, speed_map, control = _documents()
    candidate = deepcopy(control)
    candidate["sensor_policy"]["encoder_maximum_estimation_window_ns"] = (
        candidate["layers"]["wheel_pi"]["max_feedback_uncertainty_ns"] + 1
    )
    with pytest.raises(ValueError, match="encoder estimation window exceeds actuator feedback uncertainty budget"):
        ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def test_existing_planner_transport_budget_remains_strict():
    hardware, physics, speed_map, control = _documents()
    candidate = deepcopy(control)
    candidate["layers"]["async_l6"]["request_timeout_ns"] = candidate["layers"]["async_l6"]["transport_timeout_ns"]
    with pytest.raises(ValueError):
        ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def _check_cadence_cannot_outlive_its_acceptance_budget(invalid_cadence):
    hardware, physics, speed_map, control = _documents()
    if invalid_cadence == "command_reader":
        ingress = control["runtime"]["command_ingress"]
        ingress["reader_poll_s"] = ingress["maximum_ttl_ns"] / 1e9
        reason = "reader poll.*TTL"
    else:
        multirate = control["runtime"]["multirate"]
        multirate["source_periods"][0]["period_ns"] = multirate["max_snapshot_age_ns"] + 1
        reason = "multirate period exceeds freshness"
    with pytest.raises(ValueError, match=reason):
        ConfigResolver.from_documents(hardware, physics, speed_map, control)
