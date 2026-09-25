from __future__ import annotations
from v3_config_fixtures import configured

from v3.contracts import (
    MissionConstraints,
    MotionObjectiveKind,
    NavigationPlan,
    NavigationStatus,
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
    WorldSnapshot,
)
from v3.contracts.planner import TrajectoryRolloutRequest
from v3.layers.l6_navigation import NavigationConfig, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import select_motion


CONSTRAINTS = MissionConstraints(0.30, 0.60, 0.0, 0.08, 0.10)


def _candidate(
    candidate_id: str,
    *,
    v_mps: float,
    omega_rad_s: float,
    total_score: float,
    potential: float,
    viable: bool,
    collision: bool = False,
) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=v_mps,
        omega_rad_s=omega_rad_s,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(v_mps * 0.1, 0.0, omega_rad_s * 0.1, horizon_ns),),
        collision=collision,
        min_clearance_m=0.50,
        progress_score=potential,
        smoothness_score=0.50,
        novelty_score=0.50,
        total_score=total_score,
        progress_potential_score=potential,
        progress_viable=viable,
    )


def _plan(candidates: tuple[TrajectoryEvaluation, ...]) -> NavigationPlan:
    return NavigationPlan(
        context=TickContext(1, 1_000_000_000),
        mission_id="progress-viability",
        route=(),
        velocity_target=None,
        constraints=CONSTRAINTS,
        corridor_radius_m=0.0,
        progress=0.0,
        status=NavigationStatus.ACTIVE,
        local_goal=Waypoint(0.6, 0.0),
        trajectory_candidates=candidates,
    )


def _estimate(context: TickContext, yaw_rad: float = 0.0) -> RobotEstimate:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    return RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.0,
        0.0,
        yaw_rad,
        0.0,
        0.0,
        covariance,
    )


def _world(context: TickContext) -> WorldSnapshot:
    return WorldSnapshot(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=(),
        freshness_ns=0,
        local_costmap=RollingLocalCostmap(
            "R2B4_BOOT_ROBOT_MAP",
            revision=1,
            resolution_m=0.1,
            radius_m=2.5,
            occupied_cells=(),
            source_sequence=1,
            freshness_ns=0,
        ),
    )


def _rollout(goal: Waypoint):
    context = TickContext(1, 1_000_000_000)
    config = configured(NavigationConfig, 
        rollout_horizon_ns=800_000_000,
        progress_viability_floor=0.02,
    )
    request = TrajectoryRolloutRequest(
        context=context,
        estimate=_estimate(context),
        world=_world(context),
        goal=goal,
        max_v_mps=0.30,
        max_omega_rad_s=0.60,
        coverage=(),
    )
    return TrajectoryRolloutComputer(config).compute(request).trajectory_candidates


def test_progress_viable_candidate_beats_higher_scoring_stationary_candidate():
    stationary = _candidate(
        "stationary", v_mps=0.0, omega_rad_s=0.0,
        total_score=0.95, potential=0.0, viable=False,
    )
    moving = _candidate(
        "moving", v_mps=0.20, omega_rad_s=0.0,
        total_score=0.40, potential=0.10, viable=True,
    )
    objective = select_motion(_plan((stationary, moving)))
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "moving"


def test_candidate_name_never_bypasses_typed_progress_viability():
    pivot = _candidate(
        "escape-00-08", v_mps=0.0, omega_rad_s=0.60,
        total_score=0.80, potential=0.0, viable=False,
    )
    reverse = _candidate(
        "escape-05-04", v_mps=-0.12, omega_rad_s=0.0,
        total_score=0.40, potential=-0.10, viable=False,
    )
    objective = select_motion(_plan((pivot, reverse)))
    assert objective.kind is MotionObjectiveKind.STOP
    assert objective.trajectory is None
    assert objective.selection_reason == "NO_PROGRESS_VIABLE_TRAJECTORY"


def test_straight_goal_keeps_54_candidates_but_stationary_is_not_viable():
    candidates = _rollout(Waypoint(0.60, 0.0))
    assert len(candidates) == 54
    assert all(candidate.candidate_id.startswith("trajectory-") for candidate in candidates)
    stationary = next(item for item in candidates if item.candidate_id == "trajectory-00-04")
    forward = next(item for item in candidates if item.candidate_id == "trajectory-05-04")
    assert stationary.progress_viable is False
    assert stationary.progress_potential_score == 0.0
    assert forward.progress_viable is True
    assert forward.progress_potential_score > 0.02


def test_goal_behind_accepts_productive_pivot_as_progress_viable():
    candidates = _rollout(Waypoint(-0.60, 0.0))
    assert len(candidates) == 54
    assert all(candidate.candidate_id.startswith("trajectory-") for candidate in candidates)
    pivots = tuple(
        item
        for item in candidates
        if abs(item.v_mps) <= 1e-12 and abs(item.omega_rad_s) > 1e-12
    )
    assert any(item.progress_viable for item in pivots)
    assert max(item.progress_potential_score for item in pivots) > 0.02


def test_no_progress_potential_switches_to_existing_escape_family():
    candidates = _rollout(Waypoint(0.0, 0.0))
    assert len(candidates) == 54
    assert all(candidate.candidate_id.startswith("escape-") for candidate in candidates)
