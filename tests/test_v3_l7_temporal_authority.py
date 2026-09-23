"""Temporal authority, prediction and transitions through the canonical layers."""
from dataclasses import replace

import pytest

from v3.contracts import (
    MissionConstraints, MotionIntent, MotionObjectiveKind, MotionValidity,
    NavigationStatus, TickContext, TrackEstimateStatus, VelocityTarget, Waypoint,
)
from v3.layers.l4_temporal_tracking import TemporalTrackStore
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer
from v3.contracts.planner import PlannerInput
from test_v3_l7_motion_continuity import _candidate, _plan
from test_v3_follow_person import _estimate, _mission, _person, _world


def _guidance(tick=1, kind="trajectory", *, until=1_200_000_000):
    plan = _plan(tick, (_candidate("straight", 1.0),))
    if kind == "route":
        plan = replace(plan, trajectory_candidates=(), local_goal=None,
                       route=(Waypoint(1.0, 0.0),))
    elif kind == "velocity":
        plan = replace(plan, trajectory_candidates=(), local_goal=None,
                       velocity_target=VelocityTarget(0.2, 0.1))
    return replace(plan, motion_validity=MotionValidity(
        plan.context, until, "R2B4_BOOT_ROBOT_MAP", "FOLLOW_PERSON:person-1",
    ))


def _pending(plan, tick, ns=None):
    context = TickContext(tick, ns if ns is not None else 1_000_000_000 + tick * 20_000_000)
    return replace(plan, context=context, status=NavigationStatus.PENDING,
                   route=(), velocity_target=None, local_goal=None, trajectory_candidates=(),
                   reason="PLANNER_PENDING", motion_validity=replace(
                       plan.motion_validity, source_context=context, valid_until_ns=context.monotonic_ns + 350_000_000))


@pytest.mark.parametrize("kind", ["trajectory", "route", "velocity"])
def test_pending_retains_each_objective_without_renewing_lineage_or_expiry(kind):
    selector = MotionSelector()
    plan = _guidance(kind=kind)
    original = selector.evaluate(plan)
    for tick in range(2, 11):
        held = selector.evaluate(_pending(plan, tick))
        assert held.kind is original.kind
        assert held.trajectory == original.trajectory
        assert held.target_waypoint == original.target_waypoint
        assert held.velocity_target == original.velocity_target
        assert held.validity == original.validity
        assert held.context.tick_id == tick
    expired = selector.evaluate(_pending(plan, 11))
    assert expired.kind is MotionObjectiveKind.STOP
    assert selector.checkpoint().last_valid_objective is None
    assert selector.evaluate(_pending(plan, 12)).kind is MotionObjectiveKind.STOP


@pytest.mark.parametrize("change", ["mission", "frame", "scope", "gap", "backwards", "invalid", "idle", "complete", "collision"])
def test_invalidation_never_revives_old_authority(change):
    selector = MotionSelector()
    plan = _guidance()
    selector.evaluate(plan)
    next_plan = _pending(plan, 2)
    if change == "mission":
        next_plan = replace(next_plan, mission_id="new-mission")
    elif change in ("frame", "scope"):
        next_plan = replace(next_plan, motion_validity=replace(
            next_plan.motion_validity, **{("frame_id" if change == "frame" else "scope"): "different"}))
    elif change == "gap":
        next_plan = _pending(plan, 3)
    elif change == "backwards":
        next_plan = _pending(plan, 2, plan.context.monotonic_ns - 1)
    elif change == "collision":
        next_plan = replace(_guidance(2), trajectory_candidates=(_candidate("straight", 1, collision=True),))
    else:
        next_plan = replace(next_plan, status={"invalid": NavigationStatus.INVALIDATED,
                            "idle": NavigationStatus.IDLE, "complete": NavigationStatus.COMPLETE}[change],
                            reason="REVOKED")
    assert selector.evaluate(next_plan).kind is MotionObjectiveKind.STOP
    assert selector.evaluate(_pending(plan, next_plan.context.tick_id + 1)).kind is MotionObjectiveKind.STOP


def test_pending_tightens_limits_and_checkpoint_preserves_original_validity():
    selector = MotionSelector()
    plan = _guidance()
    original = selector.evaluate(plan)
    pending = _pending(plan, 2)
    pending = replace(pending, constraints=replace(pending.constraints, max_v_mps=0.1))
    held = selector.evaluate(pending)
    assert held.constraints.max_v_mps == 0.1
    assert held.validity == original.validity
    restored = MotionSelector()
    restored.restore(selector.checkpoint())
    for tick in (3, 4, 5):
        request = _pending(plan, tick)
        assert selector.evaluate(request) == restored.evaluate(request)
        assert selector.checkpoint() == restored.checkpoint()
    assert selector.checkpoint().last_valid_objective.constraints.max_v_mps == 0.1


def test_l8_rechecks_monotonic_expiry_and_frame_even_with_fresh_tick_token():
    plan = _guidance(until=1_020_000_000)
    objective = MotionSelector().evaluate(plan)
    now = TickContext(2, 1_020_000_001)
    expired = replace(objective, context=now, expiry_tick=100)
    motion = MotionRealizer().evaluate(expired, _estimate(now), _world(now))
    assert motion.stop_reason == "OBJECTIVE_EXPIRED"
    assert motion.requested_v_mps == motion.requested_omega_rad_s == 0
    wrong_frame = replace(objective, validity=replace(objective.validity, frame_id="other"))
    assert MotionRealizer().evaluate(wrong_frame, _estimate(plan.context), _world(plan.context)).stop_reason == "FRAME_MISMATCH"


def test_follow_pivot_to_planner_pending_retains_l7_objective_until_replacement():
    navigator = TrajectoryNavigator(completion_inputs=True)
    selector = MotionSelector()
    ctx = TickContext(1, 1_000_000_000)
    person = _person("person-1", 2.0, 1.5)
    first = navigator.evaluate(_mission(ctx), _estimate(ctx), _world(ctx, person))
    pivot = selector.evaluate(first)
    assert pivot.kind is MotionObjectiveKind.TRACK_PLAN
    ctx = TickContext(2, 1_020_000_000)
    pending = navigator.evaluate(_mission(ctx), _estimate(ctx, yaw=0.6), _world(ctx, person))
    assert pending.status is NavigationStatus.PENDING
    held = selector.evaluate(pending)
    assert held.target_waypoint == pivot.target_waypoint
    assert held.validity == pivot.validity
    request = navigator.pending_rollout_request
    computer = TrajectoryRolloutComputer(NavigationConfig())
    restored_nav, restored_selector = TrajectoryNavigator(completion_inputs=True), MotionSelector()
    restored_nav.restore(navigator.checkpoint())
    restored_selector.restore(selector.checkpoint())
    ctx = TickContext(3, 1_040_000_000)
    completion = PlannerInput(ctx, request.context, computer.compute(request))
    results = []
    for nav, sel in ((navigator, selector), (restored_nav, restored_selector)):
        plan = nav.evaluate(_mission(ctx), _estimate(ctx, yaw=0.6), _world(ctx, person), completion)
        results.append(sel.evaluate(plan))
    assert results[0] == results[1]
    assert results[0].kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert results[0].validity.source_context == request.context


def test_observation_dropout_uses_prediction_then_degraded_hold_without_losing_identity():
    store = TemporalTrackStore(alpha=0.85, beta=0.35, prediction_max_age_ns=350_000_000,
                               max_speed_mps=6.0)
    store.upsert_external(_person("person-1", 2.0, 0.0), 1_000_000_000,
                          observed_ns=1_020_000_000)
    navigator, selector = TrajectoryNavigator(), MotionSelector()
    for tick, now, expected in ((1, 1_020_000_000, "OBSERVED"),
                                (2, 1_120_000_000, "PREDICTED"),
                                (3, 1_350_000_001, "DEGRADED")):
        context = TickContext(tick, now)
        tracks = store.projected_tracks(now)
        assert tracks[0].estimate_status.value == expected
        assert tracks[0].measurement_monotonic_ns == 1_000_000_000
        assert tracks[0].prediction_valid_until_ns == 1_350_000_000
        plan = navigator.evaluate(_mission(context), _estimate(context), _world(context, *tracks))
        objective = selector.evaluate(plan)
        evidence = navigator.follow_person_evidence
        assert evidence.locked_target_uid == "person-1"
        assert evidence.target_estimate_status == expected
        assert evidence.target_visible is (expected == "OBSERVED")
        if expected == "DEGRADED":
            assert evidence.state == "OCCLUDED_HOLD"
            assert objective.kind is MotionObjectiveKind.STOP
        else:
            assert evidence.state == "FOLLOW"
            assert objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY
            assert objective.validity.valid_until_ns <= 1_350_000_000
    # A fresh measurement resumes the same identity, never a renewed prediction.
    store.upsert_external(_person("person-1", 2.05, 0.0), 1_380_000_000)
    context = TickContext(4, 1_380_000_000)
    plan = navigator.evaluate(_mission(context), _estimate(context),
                              _world(context, *store.projected_tracks(context.monotonic_ns)))
    assert selector.evaluate(plan).kind is MotionObjectiveKind.TRACK_TRAJECTORY
    assert navigator.follow_person_evidence.state == "FOLLOW"


@pytest.mark.parametrize("target", [0.0, -0.5, 0.2])
def test_normal_angular_target_changes_have_bounded_deceleration_and_reversal(target):
    layer = OperationalConstraintLayer()
    constraints = MissionConstraints(0.4, 1.2, 0.3, 0.1, 0.1)
    first_context = TickContext(1, 1_000_000_000)
    layer.evaluate(MotionIntent(first_context, 0.0, 0.5, 100_000_000, constraints), _estimate(first_context))
    previous = 0.0
    for tick in range(2, 34):
        context = TickContext(tick, 1_000_000_000 + (tick - 1) * 20_000_000)
        requested = 0.5 if tick < 12 else target
        motion = MotionIntent(context, 0.0, requested, 100_000_000, constraints, transition_allowed=True)
        output = layer.evaluate(motion, _estimate(context))
        assert abs(output.allowed_omega_rad_s - previous) <= 0.0500000001
        previous = output.allowed_omega_rad_s
    assert previous == pytest.approx(target)
    context = TickContext(34, 1_680_000_000)
    stopped = layer.evaluate(MotionIntent(context, 0.0, 0.0, 0, constraints, "INVALIDATED"), _estimate(context))
    assert stopped.allowed_omega_rad_s == 0


def test_arrival_and_guidance_kind_changes_preserve_l9_transition_state():
    selector, realizer, limits = MotionSelector(), MotionRealizer(), OperationalConstraintLayer()
    previous = 0.0
    for tick in range(1, 20):
        kind = "route" if tick < 12 else "velocity" if tick < 16 else "trajectory"
        plan = _guidance(tick, kind=kind, until=2_000_000_000)
        if kind == "route":
            plan = replace(plan, route=(Waypoint(0, 0, 0.6 if tick < 10 else 0.0),))
        elif kind == "velocity":
            plan = replace(plan, velocity_target=VelocityTarget(0, -0.5))
        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, _estimate(plan.context), _world(plan.context))
        assert motion.stop_reason is None
        constrained = limits.evaluate(motion, _estimate(plan.context))
        assert abs(constrained.allowed_omega_rad_s - previous) <= 0.0500000001
        previous = constrained.allowed_omega_rad_s


def test_newly_colliding_previous_command_cannot_authorize_braking_continuity():
    selector = MotionSelector()
    first = _guidance()
    selector.evaluate(first)
    replacement = replace(_guidance(2), trajectory_candidates=(
        _candidate("straight", 0.9, collision=True),
        _candidate("avoid", 1.0, omega_rad_s=0.3),
    ))
    objective = selector.evaluate(replacement)
    assert objective.trajectory.candidate_id == "avoid"
    assert objective.transition_allowed is False


def test_reduced_mission_envelope_and_revoked_transition_take_effect_immediately():
    limits = OperationalConstraintLayer()
    constraints = _guidance().constraints
    for tick in range(1, 20):
        context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
        limits.evaluate(MotionIntent(context, 0.2, 0.6, 100_000_000, constraints,
                                     transition_allowed=True), _estimate(context))
    context = TickContext(20, 1_400_000_000)
    tighter = replace(constraints, max_v_mps=0.02, max_omega_rad_s=0.03)
    output = limits.evaluate(MotionIntent(context, 0.2, 0.6, 100_000_000, tighter,
                                         transition_allowed=True), _estimate(context))
    assert abs(output.allowed_v_mps) <= 0.02
    assert abs(output.allowed_omega_rad_s) <= 0.03
    context = TickContext(21, 1_420_000_000)
    revoked = limits.evaluate(MotionIntent(context, 0, 0, 100_000_000, constraints), _estimate(context))
    assert revoked.allowed_v_mps == revoked.allowed_omega_rad_s == 0
    assert revoked.previous_velocity is None
