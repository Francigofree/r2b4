from dataclasses import dataclass, replace

import pytest

from v3.contracts import (
    AcquisitionFrame,
    ActuatorRequest,
    AdmittedFrame,
    CommandMode,
    CommandRequest,
    ConstrainedMotion,
    ConstraintCode,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    FinalActuation,
    LifecycleState,
    MissionConstraints,
    MissionIntent,
    MissionLifecycle,
    MotionIntent,
    MotionObjective,
    MotionObjectiveKind,
    NavigationPlan,
    NavigationStatus,
    Observation,
    RawDeviceBatch,
    RobotEstimate,
    SafetyDecision,
    TickContext,
    Waypoint,
    WheelVelocitySetpoint,
    WorldSnapshot,
)
from v3.engine import PipelineLayers, TickEngine, TickExecutionError, TickInputs
from v3.layers.l12_safety_final import FinalSafetyGate, LidarSafetyConfig
from v3.replay import first_divergence, run_replay


@dataclass
class RecordingWriter:
    calls: list[FinalActuation]
    fail: bool = False

    def write(self, command: FinalActuation) -> None:
        self.calls.append(command)
        if self.fail:
            raise OSError("fake driver failure")


def _inputs(tick_id: int = 7, monotonic_ns: int = 2_000) -> TickInputs:
    context = TickContext(tick_id, monotonic_ns)
    sample = DeviceSample(
        "encoder-left",
        "wheel_velocity",
        sequence=tick_id,
        captured_monotonic_ns=monotonic_ns - 100,
        values=(DataField("mps", 0.2),),
    )
    health = DeviceHealth("motor-driver", DeviceHealthState.OK)
    return TickInputs(
        context,
        RawDeviceBatch(context, (sample,), (health,)),
        CommandRequest(
            context,
            command_id="command-1",
            mode=CommandMode.NAVIGATE,
            goal=(DataField("x_m", 2.0), DataField("y_m", 0.0)),
            expiry_tick=tick_id + 10,
        ),
        LifecycleState.ACTIVE,
    )


def _layers(
    writer: RecordingWriter,
    *,
    allowed_v_mps: float = 0.15,
    fail_navigation: bool = False,
) -> PipelineLayers:
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    mission_constraints = MissionConstraints(0.3, 1.0, 0.3, 0.08, 0.1)

    def acquisition(raw: RawDeviceBatch) -> AcquisitionFrame:
        return AcquisitionFrame(raw.context, raw.samples, raw.device_health)

    def admission(frame: AcquisitionFrame) -> AdmittedFrame:
        sample = frame.samples[0]
        observation = Observation(
            sample.kind,
            sample.device_id,
            sample.sequence,
            sample.captured_monotonic_ns,
            sample.values,
        )
        return AdmittedFrame(frame.context, (observation,), ())

    def estimation(frame: AdmittedFrame) -> RobotEstimate:
        return RobotEstimate(frame.context, "map", 0.0, 0.0, 0.0, 0.2, 0.0, covariance)

    def world_model(frame: AdmittedFrame, estimate: RobotEstimate) -> WorldSnapshot:
        return WorldSnapshot(frame.context, estimate.frame_id, 1, (), freshness_ns=0)

    def command_mission(command: CommandRequest) -> MissionIntent:
        return MissionIntent(
            command.context,
            "mission-1",
            command.mode,
            Waypoint(2.0, 0.0),
            None,
            mission_constraints,
            MissionLifecycle.ACTIVE,
        )

    def navigation(
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        if fail_navigation:
            raise RuntimeError("test failure")
        return NavigationPlan(
            mission.context,
            mission.mission_id,
            (Waypoint(estimate.x_m, estimate.y_m), Waypoint(2.0, 0.0)),
            None,
            mission.constraints,
            corridor_radius_m=0.3,
            progress=0.0,
            status=NavigationStatus.ACTIVE,
        )

    def motion_selection(plan: NavigationPlan) -> MotionObjective:
        return MotionObjective(
            plan.context,
            "navigation",
            MotionObjectiveKind.TRACK_PLAN,
            priority=10,
            expiry_tick=plan.context.tick_id + 1,
            selection_reason="highest-priority",
            target_waypoint=plan.route[-1],
            velocity_target=None,
            constraints=plan.constraints,
        )

    def motion_realization(
        objective: MotionObjective,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> MotionIntent:
        return MotionIntent(
            objective.context,
            0.2,
            0.0,
            100_000_000,
            objective.constraints,
        )

    def constraints(motion: MotionIntent, estimate: RobotEstimate) -> ConstrainedMotion:
        return ConstrainedMotion(
            motion.context,
            motion.requested_v_mps,
            motion.requested_omega_rad_s,
            allowed_v_mps,
            motion.requested_omega_rad_s,
            (ConstraintCode.SPEED_LIMIT,),
        )

    def chassis_control(motion: ConstrainedMotion) -> WheelVelocitySetpoint:
        return WheelVelocitySetpoint(
            motion.context,
            motion.allowed_v_mps,
            motion.allowed_v_mps,
        )

    def actuator_control(
        wheels: WheelVelocitySetpoint,
        frame: AdmittedFrame,
    ) -> ActuatorRequest:
        return ActuatorRequest(wheels.context, wheels.left_mps, wheels.right_mps)

    return PipelineLayers(
        acquisition,
        admission,
        estimation,
        world_model,
        command_mission,
        navigation,
        motion_selection,
        motion_realization,
        constraints,
        chassis_control,
        actuator_control,
        FinalSafetyGate(writer),
    )


def test_same_input_produces_equal_direct_value_trace_without_hashes():
    first_writer = RecordingWriter([])
    second_writer = RecordingWriter([])

    first = run_replay(TickEngine(_layers(first_writer)), (_inputs(),))
    second = run_replay(TickEngine(_layers(second_writer)), (_inputs(),))

    assert first == second
    assert first_divergence(first, second) is None
    assert len(first_writer.calls) == len(second_writer.calls) == 1


def test_replay_reports_first_diverging_layer():
    expected = run_replay(TickEngine(_layers(RecordingWriter([]))), (_inputs(),))
    actual = run_replay(
        TickEngine(_layers(RecordingWriter([]), allowed_v_mps=0.1)),
        (_inputs(),),
    )

    divergence = first_divergence(expected, actual)

    assert divergence is not None
    assert divergence.tick_id == 7
    assert divergence.layer == "L9"


def test_upstream_exception_commits_one_fail_closed_l12_stop():
    writer = RecordingWriter([])
    engine = TickEngine(_layers(writer, fail_navigation=True))

    result = engine.run_tick(_inputs())

    assert result.trace.fault_layer == "L6"
    assert tuple(record.layer for record in result.trace.layers) == (
        "L1",
        "L2",
        "L3",
        "L4",
        "L5",
        "L12",
    )
    assert result.final_actuation.enabled is False
    assert result.final_actuation.left_output == result.final_actuation.right_output == 0.0
    assert result.final_actuation.reason == "L6_ERROR"
    assert writer.calls == [result.final_actuation]


def test_non_active_lifecycle_stops_at_the_only_writer():
    writer = RecordingWriter([])
    inactive = replace(_inputs(), lifecycle=LifecycleState.IDLE)

    result = TickEngine(_layers(writer)).run_tick(inactive)

    assert result.final_actuation.enabled is False
    assert result.final_actuation.reason == "NOT_ACTIVE"
    assert writer.calls == [result.final_actuation]


def test_invalid_tick_order_skips_control_layers_and_commits_stop_once():
    writer = RecordingWriter([])
    engine = TickEngine(_layers(writer))
    engine.run_tick(_inputs(7, 2_000))

    result = engine.run_tick(_inputs(9, 3_000))

    assert tuple(record.layer for record in result.trace.layers) == ("L12",)
    assert result.trace.fault_layer == "TickEngine"
    assert result.final_actuation.reason == "INVALID_TICK_ORDER"
    assert len(writer.calls) == 2


def test_writer_failure_is_not_retried_and_latches_fault():
    writer = RecordingWriter([], fail=True)
    gate = FinalSafetyGate(writer)
    layers = replace(_layers(writer), final_safety=gate)

    with pytest.raises(TickExecutionError, match="L12"):
        TickEngine(layers).run_tick(_inputs())

    assert len(writer.calls) == 1
    assert gate.fault_latched is True


def test_successful_fail_closed_tick_still_advances_tick_order():
    writer = RecordingWriter([])
    engine = TickEngine(_layers(writer, fail_navigation=True))

    first = engine.run_tick(_inputs(7, 2_000))
    skipped = engine.run_tick(_inputs(9, 3_000))

    assert first.final_actuation.reason == "L6_ERROR"
    assert skipped.trace.fault_layer == "TickEngine"
    assert skipped.final_actuation.reason == "INVALID_TICK_ORDER"
    assert len(writer.calls) == 2


def _safety_sample(
    context: TickContext,
    *,
    front: float = 1.0,
    rear: float = 1.0,
    left: float = 1.0,
    right: float = 1.0,
) -> DeviceSample:
    return DeviceSample(
        "RPLIDAR_C1",
        "lidar_safety_clearance",
        context.tick_id + 1,
        context.monotonic_ns,
        (
            DataField("age_ns", 0),
            DataField("front_clearance_m", front),
            DataField("rear_clearance_m", rear),
            DataField("left_clearance_m", left),
            DataField("right_clearance_m", right),
            DataField("front_observation_count", 10),
            DataField("rear_observation_count", 10),
            DataField("left_observation_count", 10),
            DataField("right_observation_count", 10),
        ),
    )


def test_l12_uses_direct_l1_lidar_clearance_without_changing_device_health():
    context = TickContext(0, 1_000)
    writer = RecordingWriter([])
    gate = FinalSafetyGate(
        writer,
        LidarSafetyConfig("RPLIDAR_C1", minimum_clearance_m=0.2),
    )
    request = ActuatorRequest(context, 0.2, 0.2)

    blocked = gate.finalize(
        context,
        request,
        (DeviceHealth("RPLIDAR_C1", DeviceHealthState.OK),),
        LifecycleState.ACTIVE,
        None,
        (_safety_sample(context, front=0.19),),
    )

    assert blocked.safety_decision is SafetyDecision.STOP
    assert blocked.reason == "LIDAR_CLEARANCE_LOW"
    assert blocked.left_output == blocked.right_output == 0.0


def test_l12_directional_lidar_gate_fails_closed_for_missing_or_unseen_sector():
    context = TickContext(0, 1_000)
    config = LidarSafetyConfig("RPLIDAR_C1", minimum_clearance_m=0.2)

    missing = FinalSafetyGate(RecordingWriter([]), config).finalize(
        context,
        ActuatorRequest(context, 0.2, 0.2),
        (),
        LifecycleState.ACTIVE,
        None,
    )
    unseen = replace(
        _safety_sample(context),
        values=tuple(
            DataField(field.key, 0)
            if field.key == "left_observation_count"
            else field
            for field in _safety_sample(context).values
        ),
    )
    turning = FinalSafetyGate(RecordingWriter([]), config).finalize(
        context,
        ActuatorRequest(context, -0.2, 0.2),
        (),
        LifecycleState.ACTIVE,
        None,
        (unseen,),
    )
    calibrated_straight = FinalSafetyGate(RecordingWriter([]), config).finalize(
        context,
        ActuatorRequest(context, 0.2, 0.3),
        (),
        LifecycleState.ACTIVE,
        None,
        (unseen,),
        WheelVelocitySetpoint(context, 0.1, 0.1),
    )

    assert missing.reason == "LIDAR_SAFETY_MISSING"
    assert turning.reason == "LIDAR_SAFETY_UNOBSERVED"
    assert calibrated_straight.safety_decision is SafetyDecision.ALLOW
