"""Focused acceptance for the non-actuating RoomCruise tuner."""
from __future__ import annotations

import math

from tools.tuners import r2b4_roomcruise_tuner as tuner


def test_roomcruise_tuner_search_space_stays_inside_rollout_contract():
    baseline = tuner.Candidate(
        command_max_v_mps=.30,
        command_max_omega_rad_s=.60,
        minimum_planning_speed_mps=.15,
        rollout_linear_samples=6,
        rollout_angular_samples=9,
        localization_degraded_speed_scale=.40,
        local_goal_novelty_weight=.45,
        local_goal_clearance_weight=.10,
        local_goal_forward_weight=.20,
        smoothness_weight=.20,
        novelty_weight=.10,
        localization_observability_weight=.20,
    )
    axes = tuner._axis_specs(baseline)
    assert axes
    assert {axis.name for axis in axes} >= {
        "command_max_omega_rad_s",
        "minimum_planning_speed_mps",
        "rollout_shape",
        "local_goal_forward_weight",
    }
    for axis in axes:
        for changes in axis.variants:
            candidate = tuner._candidate_with(baseline, changes)
            assert 30 <= candidate.rollout_linear_samples * candidate.rollout_angular_samples <= 60
            assert 0 < candidate.command_max_v_mps <= .45
            assert 0 < candidate.command_max_omega_rad_s <= 1.20


def test_roomcruise_tuner_geometry_and_score_helpers_are_deterministic():
    scenario = tuner.scenarios()[1]
    first = tuner._physical_clearance(scenario, *scenario.start[:2], .2)
    second = tuner._physical_clearance(scenario, *scenario.start[:2], .2)
    assert first == second
    assert math.isfinite(first)
    assert tuner._percentile([3.0, 1.0, 2.0, 4.0], .05) == 1.0


def test_roomcruise_tuner_smoke_uses_resolved_headless_l5_l9():
    control = tuner._load_control(tuner.PROJECT_ROOT)
    baseline = tuner._baseline_candidate(
        control,
        tuner.DEFAULT_COMMAND_MAX_V_MPS,
        tuner.DEFAULT_COMMAND_MAX_OMEGA_RAD_S,
    )
    result = tuner.simulate_candidate(
        control,
        baseline,
        tuner.scenarios()[0],
        ticks=25,
    )
    assert result.scenario == "open_room"
    assert result.ticks > 0
    assert math.isfinite(result.score)
    assert 0.0 <= result.moving_ratio <= 1.0
    assert 0.0 <= result.curved_ratio <= 1.0
    assert result.minimum_clearance_m > 0.0
