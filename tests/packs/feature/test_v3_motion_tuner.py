"""Optional native evidence for paired motor-free configuration experiments."""
from copy import deepcopy
import json

import pytest

from tools.tuners import r2b4_motion_tuner as tuner


def test_complete_config_overlay_is_validated_and_does_not_write_authority():
    source = tuner.PROJECT_ROOT / "conf" / "vezerles.json"
    before = source.read_bytes()
    baseline = tuner.resolve_config(tuner.PROJECT_ROOT)
    variant = tuner.resolve_config(tuner.PROJECT_ROOT, {
        "layers": {"motion_realization": {"angular_velocity_gain": .25}},
        "behavior": {"roomcruise": {"preferences": {"local_goal_forward_weight": 0.}}}})
    assert variant.runtime.composition.live_control.control.motion_realization.angular_velocity_gain == .25
    assert variant.roomcruise.preferences.local_goal_forward_weight == 0.
    assert baseline.snapshot_id != variant.snapshot_id
    with pytest.raises(ValueError, match="unknown config override"):
        tuner.resolve_config(tuner.PROJECT_ROOT, {"layers": {"motion_realization": {"typo": 1}}})
    with pytest.raises(ValueError):
        tuner.resolve_config(tuner.PROJECT_ROOT, {"layers": {"motion_realization": {"angular_velocity_gain": -1.}}})
    assert source.read_bytes() == before


@pytest.mark.parametrize("name", ["straight_asymmetric", "follow_occluded"])
def test_native_experiment_moves_stops_and_replays_without_hardware(tmp_path, name):
    resolved = tuner.resolve_config(tuner.PROJECT_ROOT)
    scenario = deepcopy(tuner.scenarios()[name])
    scenario["duration_s"] = 1.6
    if name == "follow_occluded":
        scenario["occlusion_s"] = [.3, 1.5]
    result = tuner.run_experiment(resolved, name, tmp_path/name, scenario=scenario, replay=True)
    assert result["physical_output"] is False
    assert result["fault_layer"] is None, result["first_boundary"]
    assert result["metrics"]["collision_count"] == 0
    assert result["metrics"]["path_length_m"] > .01
    assert result["final_stop"] is True
    assert result["replay"]["status"] == "MATCH"
    assert result["replay"]["determinism"]["repeated_trace_match"] is True
    rows = [json.loads(row) for row in (tmp_path/name/"timeline.ndjson").read_text().splitlines()]
    assert all(row["mission_id"] for row in rows)
    assert rows[-1]["mission_mode"] == "STOP"
    if name == "follow_occluded":
        states = result["metrics"]["follow_state_duration_s"]
        assert "FOLLOW" in states
        assert "OCCLUDED_HOLD" in states or "SEARCH" in states


def test_completed_goal_is_not_reported_as_an_active_stall(tmp_path):
    resolved = tuner.resolve_config(tuner.PROJECT_ROOT)
    scenario = deepcopy(tuner.scenarios()["straight"])
    scenario.update(duration_s=.3, goal={"x_m": .01, "y_m": 0., "yaw_rad": 0.})
    result = tuner.run_experiment(resolved, "straight", tmp_path/"complete", scenario=scenario)
    assert result["termination_reason"] == "GOAL_COMPLETE"
    assert result["metrics"]["active_mission_sample_count"] == 0
    assert result["metrics"]["moving_ratio"] is None
    assert result["metrics"]["longest_stop_s"] == 0.
    assert result["metrics"]["no_translation_progress_2s_duration_s"] == 0.
