import pytest

from v3.composition import MissionNavigationComposition, MissionNavigationInputs
from v3.contracts import (
    CommandMode,
    CommandRequest,
    ConstraintCode,
    DataField,
    MissionLifecycle,
    MotionObjectiveKind,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    TickContext,
    WorldSnapshot,
)


def _covariance(position_variance: float = 0.01, yaw_variance: float = 0.01) -> tuple[float, ...]:
    values = [0.0] * 25
    values[0] = position_variance
    values[6] = position_variance
    values[12] = yaw_variance
    values[18] = 0.01
    values[24] = 0.01
    return tuple(values)


def _frame(
    tick_id: int,
    monotonic_ns: int,
    *,
    mode: CommandMode = CommandMode.NAVIGATE,
    goal: tuple[DataField, ...] = (DataField("x_m", 1.0), DataField("y_m", 0.0)),
    command_id: str = "command-1",
    x_m: float = 0.0,
    y_m: float = 0.0,
    yaw_rad: float = 0.0,
    v_mps: float = 0.0,
    omega_rad_s: float = 0.0,
    position_variance: float = 0.01,
    obstacles: tuple[ObstacleTrack, ...] = (),
) -> MissionNavigationInputs:
    context = TickContext(tick_id, monotonic_ns)
    estimate = RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        x_m,
        y_m,
        yaw_rad,
        v_mps,
        omega_rad_s,
        _covariance(position_variance),
    )
    world = WorldSnapshot(
        context,
        estimate.frame_id,
        map_revision=4,
        obstacle_tracks=obstacles,
        freshness_ns=0,
    )
    command = CommandRequest(context, command_id, mode, goal, tick_id + 2)
    return MissionNavigationInputs(context, command, estimate, world)


def test_navigation_scenario_replay_is_deterministic_and_progresses_l5_to_l9():
    frames = (
        _frame(0, 1_000_000_000, x_m=0.0),
        _frame(1, 1_100_000_000, x_m=0.1),
        _frame(2, 1_200_000_000, x_m=0.2),
    )

    first = MissionNavigationComposition().replay(frames)
    second = MissionNavigationComposition().replay(frames)

    assert first == second
    assert tuple(trace.navigation.progress for trace in first) == pytest.approx((0.0, 0.1, 0.2))
    assert all(trace.mission.lifecycle is MissionLifecycle.ACTIVE for trace in first)
    assert all(trace.navigation.status is NavigationStatus.ACTIVE for trace in first)
    assert all(trace.objective.kind is MotionObjectiveKind.TRACK_PLAN for trace in first)
    assert all(trace.motion.requested_v_mps > 0.0 for trace in first)
    assert first[0].constrained.allowed_v_mps == 0.0
    assert first[1].constrained.allowed_v_mps == pytest.approx(0.06)
    assert ConstraintCode.ACCELERATION_LIMIT in first[1].constrained.active_constraints


def test_navigation_progress_does_not_regress_and_completion_is_latched_per_mission():
    runtime = MissionNavigationComposition()

    traces = runtime.replay(
        (
            _frame(0, 1_000_000_000, x_m=0.0),
            _frame(1, 1_100_000_000, x_m=0.5),
            _frame(2, 1_200_000_000, x_m=0.3),
            _frame(3, 1_300_000_000, x_m=1.0),
            _frame(4, 1_400_000_000, x_m=0.7),
        )
    )

    assert tuple(trace.navigation.progress for trace in traces) == pytest.approx(
        (0.0, 0.5, 0.5, 1.0, 1.0)
    )
    assert traces[-2].navigation.status is NavigationStatus.COMPLETE
    assert traces[-1].navigation.status is NavigationStatus.COMPLETE


def test_blocked_route_stops_before_motion_and_reports_local_clearance_constraint():
    obstacle = ObstacleTrack("blocking", 0.5, 0.0, 0.10, 0.0, 0.0, 0.9)

    trace = MissionNavigationComposition().run_tick(
        _frame(0, 1_000_000_000, obstacles=(obstacle,))
    )

    assert trace.navigation.status is NavigationStatus.NO_PATH
    assert trace.navigation.reason == "ROUTE_BLOCKED"
    assert trace.objective.kind is MotionObjectiveKind.STOP
    assert trace.motion.stop_reason == "ROUTE_BLOCKED"
    assert trace.constrained.allowed_v_mps == 0.0
    assert trace.constrained.allowed_omega_rad_s == 0.0
    assert trace.constrained.active_constraints == (ConstraintCode.LOCAL_CLEARANCE,)


def test_goal_position_and_heading_completion_produce_an_explicit_stop():
    goal = (
        DataField("x_m", 1.0),
        DataField("y_m", 0.0),
        DataField("yaw_rad", 0.25),
    )

    trace = MissionNavigationComposition().run_tick(
        _frame(0, 1_000_000_000, goal=goal, x_m=1.0, yaw_rad=0.25)
    )

    assert trace.navigation.status is NavigationStatus.COMPLETE
    assert trace.navigation.progress == 1.0
    assert trace.objective.kind is MotionObjectiveKind.STOP
    assert trace.motion.requested_v_mps == 0.0
    assert trace.motion.requested_omega_rad_s == 0.0
    assert trace.constrained.allowed_v_mps == 0.0


def test_teleop_scenario_applies_mission_limits_then_stateful_acceleration_limit():
    goal = (
        DataField("v_mps", 1.0),
        DataField("omega_rad_s", 2.0),
        DataField("max_v_mps", 0.2),
        DataField("max_omega_rad_s", 0.5),
    )
    frames = (
        _frame(0, 1_000_000_000, mode=CommandMode.TELEOP, goal=goal),
        _frame(1, 1_100_000_000, mode=CommandMode.TELEOP, goal=goal),
    )

    traces = MissionNavigationComposition().replay(frames)
    constrained = traces[-1].constrained

    assert traces[-1].objective.kind is MotionObjectiveKind.VELOCITY
    assert traces[-1].motion.requested_v_mps == 1.0
    assert traces[-1].motion.requested_omega_rad_s == 2.0
    assert constrained.allowed_v_mps == pytest.approx(0.06)
    assert constrained.allowed_omega_rad_s == pytest.approx(0.25)
    assert constrained.active_constraints == (
        ConstraintCode.MISSION_LIMIT,
        ConstraintCode.ACCELERATION_LIMIT,
    )


def test_degraded_localization_stops_a_valid_motion_objective_at_l9():
    runtime = MissionNavigationComposition()
    runtime.run_tick(_frame(0, 1_000_000_000))

    trace = runtime.run_tick(
        _frame(1, 1_100_000_000, position_variance=0.3)
    )

    assert trace.motion.requested_v_mps > 0.0
    assert trace.constrained.allowed_v_mps == 0.0
    assert trace.constrained.allowed_omega_rad_s == 0.0
    assert trace.constrained.active_constraints == (
        ConstraintCode.LOCALIZATION_DEGRADED,
    )


def test_invalid_command_and_reused_command_id_fail_closed_without_a_motion_target():
    runtime = MissionNavigationComposition()
    invalid = runtime.run_tick(
        _frame(0, 1_000_000_000, goal=(DataField("x_m", 1.0),))
    )
    runtime = MissionNavigationComposition()
    runtime.run_tick(_frame(0, 1_000_000_000))
    reused = runtime.run_tick(
        _frame(
            1,
            1_100_000_000,
            goal=(DataField("x_m", 2.0), DataField("y_m", 0.0)),
        )
    )

    assert invalid.mission.lifecycle is MissionLifecycle.FAILED
    assert invalid.mission.stop_reason == "INVALID_COMMAND"
    assert invalid.navigation.status is NavigationStatus.INVALIDATED
    assert invalid.constrained.allowed_v_mps == 0.0
    assert reused.mission.lifecycle is MissionLifecycle.FAILED
    assert reused.mission.stop_reason == "COMMAND_ID_REUSED"
    assert reused.objective.kind is MotionObjectiveKind.STOP


def test_scenario_replay_rejects_noncontiguous_tick_order():
    runtime = MissionNavigationComposition()
    runtime.run_tick(_frame(0, 1_000_000_000))

    with pytest.raises(ValueError, match="tick order"):
        runtime.run_tick(_frame(2, 1_100_000_000))
