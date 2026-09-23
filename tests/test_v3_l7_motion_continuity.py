from dataclasses import fields, replace

import pytest

from v3.composition.native_control import NativeControlStateCheckpoint
from v3.contracts import (
    MissionConstraints,
    MotionObjectiveKind,
    NavigationPlan,
    NavigationStatus,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
)
from v3.layers.l7_motion_selection import (
    MotionSelectionConfig,
    MotionSelectionStateCheckpoint,
    MotionSelector,
    select_motion,
)


CONSTRAINTS = MissionConstraints(0.35, 1.2, 0.30, 0.08, 0.10)


def _candidate(
    candidate_id: str,
    total_score: float,
    *,
    collision: bool = False,
    omega_rad_s: float = 0.0,
) -> TrajectoryEvaluation:
    horizon_ns = 100_000_000
    return TrajectoryEvaluation(
        candidate_id=candidate_id,
        v_mps=0.20,
        omega_rad_s=omega_rad_s,
        horizon_ns=horizon_ns,
        samples=(TrajectoryPose(0.02, 0.0, 0.0, horizon_ns),),
        collision=collision,
        min_clearance_m=0.50,
        progress_score=0.50,
        smoothness_score=0.50,
        novelty_score=0.50,
        total_score=total_score,
    )


def _plan(
    tick_id: int,
    candidates: tuple[TrajectoryEvaluation, ...],
    *,
    mission_id: str = "mission-a",
) -> NavigationPlan:
    context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
    return NavigationPlan(
        context=context,
        mission_id=mission_id,
        route=(),
        velocity_target=None,
        constraints=CONSTRAINTS,
        corridor_radius_m=0.30,
        progress=0.25,
        status=NavigationStatus.ACTIVE,
        local_goal=Waypoint(1.0, 0.0),
        trajectory_candidates=candidates,
    )


def _selected_id(selector: MotionSelector, plan: NavigationPlan) -> str:
    objective = selector.evaluate(plan)
    assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert objective.trajectory is not None
    return objective.trajectory.candidate_id


def _prime_previous(selector: MotionSelector, candidate_id: str = "previous") -> None:
    other = "best" if candidate_id != "best" else "other"
    first = (
        _candidate(candidate_id, 1.000, omega_rad_s=0.0),
        _candidate(other, 0.900, omega_rad_s=0.3),
    )
    assert _selected_id(selector, _plan(1, first)) == candidate_id


def test_stateless_selector_keeps_existing_canonical_ranking():
    plan = _plan(
        1,
        (
            _candidate("a", 0.900),
            _candidate("b", 1.000),
        ),
    )
    objective = select_motion(plan)
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "b"
    assert objective.selection_reason == "BEST_TRAJECTORY:b"


@pytest.mark.parametrize(
    ("score_gap", "expected_id", "expected_prefix"),
    (
        (0.0049, "previous", "CONTINUITY_HOLD"),
        (0.0050, "previous", "CONTINUITY_HOLD"),
        (0.0051, "best", "BEST_TRAJECTORY"),
    ),
)
def test_id_hold_uses_measured_0_005_score_band(
    score_gap,
    expected_id,
    expected_prefix,
):
    selector = MotionSelector()
    _prime_previous(selector)

    objective = selector.evaluate(
        _plan(
            2,
            (
                _candidate("previous", 1.000 - score_gap, omega_rad_s=0.0),
                _candidate("best", 1.000, omega_rad_s=0.3),
            ),
        )
    )
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == expected_id
    assert objective.selection_reason.startswith(expected_prefix + ":")


def test_zero_band_disables_temporal_hold():
    selector = MotionSelector(MotionSelectionConfig(0.0))
    _prime_previous(selector)
    assert _selected_id(
        selector,
        _plan(
            2,
            (
                _candidate("previous", 1.000),
                _candidate("best", 1.000),
            ),
        ),
    ) == "best"


def test_previous_collision_can_never_be_held():
    selector = MotionSelector()
    _prime_previous(selector)
    objective = selector.evaluate(
        _plan(
            2,
            (
                _candidate("previous", 0.999, collision=True),
                _candidate("best", 1.000),
            ),
        )
    )
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "best"
    assert objective.selection_reason == "BEST_TRAJECTORY:best"


def test_missing_previous_candidate_falls_back_to_current_best():
    selector = MotionSelector()
    _prime_previous(selector)
    assert _selected_id(
        selector,
        _plan(
            2,
            (
                _candidate("new-a", 0.990),
                _candidate("new-b", 1.000),
            ),
        ),
    ) == "new-b"


def test_mission_change_cannot_reuse_old_candidate_identity():
    selector = MotionSelector()
    _prime_previous(selector)
    objective = selector.evaluate(
        _plan(
            2,
            (
                _candidate("previous", 0.999),
                _candidate("best", 1.000),
            ),
            mission_id="mission-b",
        )
    )
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "best"
    assert objective.selection_reason == "BEST_TRAJECTORY:best"


def test_nontrajectory_branch_retains_objective_without_old_candidate_bias():
    selector = MotionSelector()
    _prime_previous(selector)

    base = _plan(2, (_candidate("previous", 1.0),))
    route_plan = replace(
        base,
        route=(Waypoint(1.0, 0.0),),
        local_goal=None,
        trajectory_candidates=(),
    )
    route_objective = selector.evaluate(route_plan)
    assert route_objective.kind is MotionObjectiveKind.TRACK_PLAN
    assert selector.checkpoint().last_valid_objective == route_objective
    assert selector.checkpoint().last_candidate_id is None

    objective = selector.evaluate(
        _plan(
            3,
            (
                _candidate("previous", 0.999),
                _candidate("best", 1.000),
            ),
        )
    )
    assert objective.trajectory is not None
    assert objective.trajectory.candidate_id == "best"


def test_checkpoint_restore_preserves_next_l7_decision():
    original = MotionSelector()
    _prime_previous(original)
    checkpoint = original.checkpoint()

    restored = MotionSelector()
    restored.restore(checkpoint)

    next_plan = _plan(
        2,
        (
            _candidate("previous", 0.998, omega_rad_s=0.0),
            _candidate("best", 1.000, omega_rad_s=0.3),
        ),
    )
    assert original.evaluate(next_plan) == restored.evaluate(next_plan)
    assert original.checkpoint() == restored.checkpoint()


def test_native_control_checkpoint_has_backward_compatible_l7_default():
    field = next(
        item
        for item in fields(NativeControlStateCheckpoint)
        if item.name == "motion_selection"
    )
    assert isinstance(field.default, MotionSelectionStateCheckpoint)
    assert field.default == MotionSelectionStateCheckpoint()


@pytest.mark.parametrize("value", (-0.001, 1.001, float("inf"), float("nan")))
def test_motion_selection_band_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        MotionSelectionConfig(value)
