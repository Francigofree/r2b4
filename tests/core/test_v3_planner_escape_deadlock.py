from __future__ import annotations

import math
from types import SimpleNamespace

from v3.contracts import TrajectoryEvaluation, TrajectoryPose, Waypoint
from v3.layers import l6_navigation as l6
from v3.layers import l7_motion_selection as l7


def _config():
    return SimpleNamespace(
        rollout_horizon_ns=800_000_000,
        rollout_step_count=8,
        footprint_safety_margin_m=0.05,
        local_escape_trigger_clearance_m=0.12,
        clearance_score_cap_m=1.0,
        progress_viability_floor=0.02,
        progress_weight=0.60,
        clearance_weight=0.10,
        smoothness_weight=0.16,
        novelty_weight=0.14,
    )


def _pivot(yaw_rad: float) -> TrajectoryEvaluation:
    return TrajectoryEvaluation(
        candidate_id="trajectory-00-07",
        v_mps=0.0,
        omega_rad_s=0.5,
        horizon_ns=800_000_000,
        samples=(TrajectoryPose(0.0, 0.0, yaw_rad, 800_000_000),),
        collision=False,
        min_clearance_m=0.20,
        progress_score=0.0,
        smoothness_score=0.8,
        novelty_score=0.5,
        total_score=0.0,
        progress_potential_score=0.0,
        progress_viable=False,
    )


def _escape(candidate, goal, monkeypatch, clearance):
    monkeypatch.setattr(l6, "_footprint_clearance", clearance)
    return l6._as_escape_candidate(
        candidate,
        0,
        7,
        0.20,
        goal,
        0.12,
        object(),
        _config(),
        object(),
    )


def test_safe_pivot_is_viable_when_it_opens_clearance_on_the_next_step(monkeypatch):
    def clearance(x_m, y_m, yaw_rad, world, config, scene=None, *, clearance_limit_m=None):
        del x_m, yaw_rad, world, config, scene, clearance_limit_m
        return min(0.50, 0.20 + 2.0 * max(0.0, y_m))

    result = _escape(_pivot(math.pi / 2.0), Waypoint(1.0, 0.0), monkeypatch, clearance)

    assert result.progress_viable is True
    assert result.progress_potential_score >= _config().progress_viability_floor
    # This is the L7 gate that previously produced NO_PROGRESS_VIABLE_TRAJECTORY.
    assert l7._viable_trajectories(SimpleNamespace(trajectory_candidates=(result,))) == (result,)


def test_safe_but_causally_useless_pivot_remains_rejected(monkeypatch):
    def clearance(x_m, y_m, yaw_rad, world, config, scene=None, *, clearance_limit_m=None):
        del x_m, y_m, yaw_rad, world, config, scene, clearance_limit_m
        return 0.20

    result = _escape(_pivot(math.pi / 2.0), Waypoint(1.0, 0.0), monkeypatch, clearance)

    assert result.progress_viable is False
    assert result.progress_potential_score == 0.0
    assert l7._viable_trajectories(SimpleNamespace(trajectory_candidates=(result,))) == ()


def test_pivot_can_unlock_a_reverse_next_step_without_clearance_gain(monkeypatch):
    def clearance(x_m, y_m, yaw_rad, world, config, scene=None, *, clearance_limit_m=None):
        del y_m, yaw_rad, world, config, scene, clearance_limit_m
        # Forward is physically blocked; reverse is safe but has flat clearance.
        return 0.04 if x_m > 0.01 else 0.20

    result = _escape(_pivot(0.0), Waypoint(-1.0, 0.0), monkeypatch, clearance)

    assert result.progress_viable is True
    assert result.progress_potential_score > _config().progress_viability_floor


def test_forward_unlock_probe_respects_existing_escape_trigger_clearance(monkeypatch):
    def clearance(x_m, y_m, yaw_rad, world, config, scene=None, *, clearance_limit_m=None):
        del y_m, yaw_rad, world, config, scene, clearance_limit_m
        if x_m > 0.01:
            # Above hard footprint safety (0.05), but below the existing 0.12 m
            # forward preemption threshold: this must not count as an exit.
            return 0.10 if x_m < 0.04 else 0.30
        return 0.20

    result = _escape(_pivot(0.0), Waypoint(1.0, 0.0), monkeypatch, clearance)

    assert result.progress_viable is False
    assert result.progress_potential_score == 0.0
