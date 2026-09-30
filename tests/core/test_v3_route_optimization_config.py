from __future__ import annotations

import json
from pathlib import Path

import pytest

from v3.config import ConfigResolver
from v3.contracts import TrajectoryEvaluation, TrajectoryPose
from v3.layers import l6_navigation as l6
from v3.layers import l7_motion_selection as l7
from v3.replay import _migrate_legacy_resolved_config_snapshot

ROOT = next(
    (
        path
        for path in Path(__file__).resolve().parents
        if (path / "conf" / "hardver.json").is_file() and (path / "v3").is_dir()
    ),
    Path.cwd(),
)


def _resolved():
    conf = ROOT / "conf"
    documents = [
        json.loads((conf / name).read_text(encoding="utf-8"))
        for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
    ]
    return ConfigResolver.from_documents(*documents)


def _candidate(candidate_id: str, *, clearance: float, score: float) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=0.3,
        omega_rad_s=0.0,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(0.03, 0.0, 0.0, horizon_ns),),
        collision=False,
        min_clearance_m=clearance,
        progress_score=0.5,
        progress_potential_score=0.5,
        progress_viable=True,
        smoothness_score=0.5,
        novelty_score=0.5,
        total_score=score,
    )


def test_production_route_tuning_is_explicit_config_authority():
    resolved = _resolved()
    control = resolved.runtime.composition.live_control.control
    nav = control.navigation
    selection = control.motion_selection

    assert nav.speed_clearance_enabled is True
    assert nav.speed_clearance_low_speed_mps == 0.2
    assert nav.speed_clearance_high_speed_mps == 0.4
    assert nav.speed_clearance_low_m == 0.18
    assert nav.speed_clearance_high_m == 0.3
    assert nav.speed_clearance_penalty_weight == 0.18
    assert nav.rollout_horizon_ns == 1_000_000_000
    assert nav.rollout_step_count == 10
    assert l6._explore_goal_distances(nav) == pytest.approx((0.9, 0.75, 0.6, 0.45))
    assert selection.continuity_score_band == 0.02
    assert selection.continuity_clearance_drop_tolerance_m == 0.04


def test_speed_clearance_soft_barrier_penalizes_fast_tight_path_first():
    nav = _resolved().runtime.composition.live_control.control.navigation
    clearance = nav.speed_clearance_low_m
    slow_score = l6._trajectory_clearance_score(clearance, 0.2, nav)
    fast_score = l6._trajectory_clearance_score(clearance, 0.4, nav)
    slow_penalty = l6._trajectory_clearance_penalty(slow_score, 0.2, nav)
    fast_penalty = l6._trajectory_clearance_penalty(fast_score, 0.4, nav)
    assert slow_score == 1.0
    assert 0.0 <= fast_score < slow_score
    assert slow_penalty == 0.0
    assert fast_penalty > 0.0
    assert l6._trajectory_clearance_score(nav.speed_clearance_high_m, 0.4, nav) == 1.0


def test_l7_continuity_guard_does_not_keep_materially_tighter_near_best_path():
    tight = _candidate("tight", clearance=0.30, score=1.0)
    wide = _candidate("wide", clearance=0.45, score=0.99)
    guarded = l7._clearance_guarded_near_best((tight, wide), 0.04)
    assert guarded == (wide,)
    assert l7._clearance_guarded_near_best((tight, wide), 1_000_000.0) == (tight, wide)


def test_historical_capture_migration_disables_new_route_policy():
    old_l7 = _migrate_legacy_resolved_config_snapshot(
        {"__type__": "MotionSelectionConfig", "continuity_score_band": 0.005}
    )
    assert old_l7["reversal_min_omega_rad_s"] == 0.05
    assert old_l7["continuity_clearance_drop_tolerance_m"] == 1_000_000.0

    old_l6 = _migrate_legacy_resolved_config_snapshot(
        {"__type__": "NavigationConfig", "local_goal_distance_m": 0.6}
    )
    assert old_l6["speed_clearance_enabled"] is False
    assert old_l6["speed_clearance_penalty_weight"] == 0.0
    assert old_l6["explore_local_goal_min_distance_m"] == 0.6
    assert old_l6["explore_local_goal_max_distance_m"] == 0.6
    assert old_l6["explore_local_goal_distance_samples"] == 1
