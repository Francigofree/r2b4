"""A changed-geometry hold preserves the host mission and revokes motion."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from rig import resolved_config
from test_v3_cached_geometry import _scene
from r2b4_orchestration.behavior_system import BehaviorLifecycle, BehaviorSnapshot, _MissionProgram
from v3.composition.full_fake import OfflineMotorSink
from v3.contracts import (
    AdmittedFrame, CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, GLOBAL_FRAME_ID, LifecycleState, LOCAL_FRAME_ID, Observation, Pose2D, TickContext,
)
from v3.contracts.planner import PlannerInput
from v3.engine import LayerRecord, TickResult, TickTrace
from v3.finite_navigation import FiniteNavigationExecutor
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer
from v3.layers.l10_chassis_control import DifferentialDriveKinematics
from v3.layers.l11_actuator_control import WheelActuatorController
from v3.layers.l12_safety_final import FinalSafetyGate
from v3.resident_status import _tick_status


def _completed_states(mode=CommandMode.NAVIGATE):
    """Synthetic L3/L4 inputs; real L5-L12 and an in-memory motor sink."""
    config = resolved_config().runtime.composition.live_control.control
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    selector = MotionSelector(config.motion_selection)
    realizer = MotionRealizer(config.motion_realization)
    limiter = OperationalConstraintLayer(config.operational_constraints)
    chassis = DifferentialDriveKinematics(config.chassis_control)
    actuator = WheelActuatorController(config.speed_map, config.wheel_pi)
    sink = OfflineMotorSink()
    safety = FinalSafetyGate(sink, config.lidar_safety, config.critical_device_ids)
    health = tuple(DeviceHealth(device, DeviceHealthState.OK)
                   for device in sorted(config.critical_device_ids))

    def tick(tick_id, *, blocked=False, completion=None, x_m=0.):
        estimate, world = _scene(tick_id, blocked=blocked)
        if tick_id >= 2:
            revision = 2 if blocked else 3
            world = replace(world, map_revision=revision, local_costmap=replace(
                world.local_costmap, revision=revision, source_sequence=tick_id + 1, freshness_ns=0))
        estimate = replace(estimate, x_m=x_m,
            local_pose=Pose2D(LOCAL_FRAME_ID, x_m, 0., 0.),
            global_pose=Pose2D(GLOBAL_FRAME_ID, x_m, 0., 0.),
            map_to_odom=Pose2D(GLOBAL_FRAME_ID, 0., 0., 0.))
        context = estimate.context
        goal = (
            DataField("x_m", 1.), DataField("y_m", 0.), DataField("frame_id", LOCAL_FRAME_ID),
        ) if mode is CommandMode.NAVIGATE else ()
        command = CommandRequest(context, "hold", mode, goal, tick_id)
        mission = manager.evaluate(command)
        plan = navigator.evaluate(mission, estimate, world, completion)
        objective = selector.evaluate(plan)
        motion = realizer.evaluate(objective, estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        wheels = chassis(allowed)
        feedback = Observation("wheel_velocity", "ENCODER", tick_id, context.monotonic_ns,
            (DataField("left_mps", 0.), DataField("right_mps", 0.), DataField("trust", 1.)))
        request = actuator(wheels, AdmittedFrame(context, (feedback,), (), device_health=health))
        samples = (DeviceSample(config.lidar_safety.device_id, "lidar_safety_clearance", tick_id,
            context.monotonic_ns, (DataField("age_ns", 0), *tuple(DataField(f"{sector}_{field}", value)
                for sector in ("front", "rear", "left", "right")
                for field, value in (("clearance_m", 2.), ("observation_count", 10))))),)
        final = safety.finalize(context, request, health, LifecycleState.ACTIVE, None, samples, wheels)
        result = TickResult(final, TickTrace(context, tuple(
            LayerRecord(layer, value) for layer, value in (
                ("L3", estimate), ("L4", world), ("L5", mission), ("L6", plan),
                ("L7", objective), ("L8", motion), ("L9", allowed),
                ("L10", wheels), ("L11", request), ("L12", final),
            )
        )))
        return result

    tick(0)
    pending = navigator.pending_rollout_request
    completion = TrajectoryRolloutComputer(config.navigation).compute(pending)
    active = tick(1, completion=PlannerInput(TickContext(1, 1_020_000_000), pending.context, completion))
    # Keep the old family beyond its max age while a fresh replacement remains
    # pending inside its own original deadline. Every local scan stays fresh.
    last_hold_tick = config.async_l6.max_plan_age_ns // 20_000_000 + 1
    held = tuple(tick(index, blocked=True) for index in range(2, last_hold_tick + 1))
    pending = navigator.pending_rollout_request
    resume_context = TickContext(last_hold_tick + 1, 1_000_000_000 + (last_hold_tick + 1) * 20_000_000)
    assert resume_context.monotonic_ns - pending.context.monotonic_ns < config.async_l6.request_timeout_ns
    replacement = TrajectoryRolloutComputer(config.navigation).compute(pending)
    resumed = tick(last_hold_tick + 1, completion=PlannerInput(resume_context, pending.context, replacement))
    completed = tick(last_hold_tick + 2, x_m=1.)
    return active, held, resumed, completed, sink


@pytest.mark.parametrize("mode", [CommandMode.NAVIGATE, CommandMode.FOLLOW_PERSON])
def test_geometry_hold_revokes_real_motion_tail_and_preserves_host_mission(mode):
    active, held, resumed, completed, sink = _completed_states(mode)
    before, after = (_tick_status(result) for result in (active, resumed))
    paused_states = tuple(_tick_status(result) for result in held)
    assert before["navigation"]["status"] == after["navigation"]["status"] == "ACTIVE"
    for result, paused in zip(held, paused_states):
        assert paused["navigation"]["status"] == "IDLE"
        assert paused["navigation"]["reason"] == "PLANNER_GEOMETRY_HOLD"
        assert paused["safety_decision"] == "ALLOW"
        assert before["mission"]["mission_id"] == paused["mission"]["mission_id"] == after["mission"]["mission_id"]
        values = {layer.layer: layer.output for layer in result.trace.layers}
        assert values["L6"].route == values["L6"].trajectory_candidates == ()
        assert values["L6"].velocity_target is None
        assert values["L7"].trajectory is values["L7"].velocity_target is None
        assert values["L8"].requested_v_mps == values["L8"].requested_omega_rad_s == 0.
        assert values["L9"].allowed_v_mps == values["L9"].allowed_omega_rad_s == 0.
        assert values["L10"].left_mps == values["L10"].right_mps == 0.
        assert values["L11"].left_normalized == values["L11"].right_normalized == 0.
        assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
    assert len(sink.writes) == completed.trace.context.tick_id + 1
    state = BehaviorSnapshot(lifecycle=BehaviorLifecycle.ACTIVE, mission_id=before["mission"]["mission_id"])
    program = _MissionProgram("v3.command.navigate", mode.value)
    for status in (before, *paused_states, after):
        update = program.step(None, state, status)
        assert update.lifecycle is BehaviorLifecycle.ACTIVE


@pytest.mark.parametrize("terminal", ["COMPLETE", "STALLED"])
def test_finite_wait_keeps_geometry_hold_bounded_and_can_resume(terminal):
    _, held, resumed, completed, _ = _completed_states()
    hold_status, aged_hold_status = _tick_status(held[0]), _tick_status(held[-1])
    statuses = iter((hold_status, aged_hold_status, _tick_status(resumed), _tick_status(completed)))
    clock = [0.]
    polls = []

    def status():
        current = next(statuses) if terminal == "COMPLETE" else hold_status
        polls.append(current["navigation"]["status"])
        return current

    controller = SimpleNamespace(snapshot=lambda: SimpleNamespace(runtime_pid=123), live_runtime_status=status)
    executor = FiniteNavigationExecutor(controller, monotonic=lambda: clock[0],
        sleep=lambda delay: clock.__setitem__(0, clock[0] + delay), progress_timeout_s=.2)
    result, reason = executor._wait(hold_status["mission"]["mission_id"], LOCAL_FRAME_ID,
        executor._generation(hold_status), 1., None, runtime_pid=123,
        target={"frame_id": LOCAL_FRAME_ID, "x_m": 1., "y_m": 0.},
        goal_tolerance_m=.1, yaw_tolerance_rad=.1)
    if terminal == "COMPLETE":
        assert reason == "COMPLETE" and polls == ["IDLE", "IDLE", "ACTIVE", "COMPLETE"]
        assert result["navigation"]["mission_id"] == hold_status["mission"]["mission_id"]
    else:
        assert reason == "NAVIGATION_STALLED" and .2 <= clock[0] < .3
        assert all(value == "IDLE" for value in polls)
