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
    ConfigResolver.from_documents(*_documents())


def test_l11_feedback_cache_cannot_outlive_admission_freshness():
    hardware, physics, speed_map, control = _documents()
    candidate = deepcopy(control)
    admission_age = candidate["layers"]["admission"]["max_sample_age_ns"]
    candidate["layers"]["wheel_pi"]["max_feedback_age_ns"] = admission_age + 1
    with pytest.raises(ValueError, match="feedback age exceeds admission freshness"):
        ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def test_l6_and_l8_world_freshness_must_match():
    hardware, physics, speed_map, control = _documents()
    candidate = deepcopy(control)
    candidate["layers"]["motion_realization"]["max_world_freshness_ns"] += 1
    with pytest.raises(ValueError, match="navigation/motion world freshness mismatch"):
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
