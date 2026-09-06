from dataclasses import FrozenInstanceError, fields, is_dataclass
from typing import Any, get_args, get_type_hints

import pytest

import v3.contracts as contracts
from v3.contracts import (
    AcquisitionFrame,
    ActuatorRequest,
    AdmittedFrame,
    CommandMode,
    CommandRequest,
    ConstrainedMotion,
    ConstraintCode,
    ContractValidationError,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    FinalActuation,
    MissionConstraints,
    MissionIntent,
    MissionLifecycle,
    MotionIntent,
    MotionObjective,
    MotionObjectiveKind,
    NavigationPlan,
    NavigationStatus,
    Observation,
    ObstacleTrack,
    RawDeviceBatch,
    RejectedObservation,
    RejectionReason,
    RobotEstimate,
    SafetyDecision,
    TickContext,
    VelocityTarget,
    Waypoint,
    WheelVelocitySetpoint,
    WorldSnapshot,
)


def contract_instances() -> tuple[object, ...]:
    context = TickContext(7, 2_000)
    sample = DeviceSample(
        device_id="encoder-left",
        kind="wheel_ticks",
        sequence=4,
        captured_monotonic_ns=1_000,
        values=(DataField("ticks", 12), DataField("scale", 1.0)),
    )
    health = DeviceHealth("encoder-left", DeviceHealthState.OK)
    raw = RawDeviceBatch(context, (sample,), (health,))
    acquisition = AcquisitionFrame(context, (sample,), (health,))
    observation = Observation(
        kind="wheel_velocity",
        source_device_id="encoder-left",
        source_sequence=4,
        captured_monotonic_ns=1_000,
        values=(DataField("mps", 0.25),),
    )
    admitted = AdmittedFrame(
        context,
        accepted=(observation,),
        rejected=(
            RejectedObservation("lidar", 3, RejectionReason.STALE, age_ns=500),
        ),
        degraded_sources=("lidar",),
    )
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    estimate = RobotEstimate(context, "map", 1.0, 2.0, 0.1, 0.2, 0.0, covariance)
    world = WorldSnapshot(
        context,
        frame_id="map",
        map_revision=8,
        obstacle_tracks=(ObstacleTrack("track-1", 2.0, 3.0, 0.2, 0.0, 0.0, 0.9),),
        freshness_ns=20_000,
    )
    command = CommandRequest(
        context,
        command_id="command-1",
        mode=CommandMode.NAVIGATE,
        goal=(DataField("x_m", 4.0), DataField("y_m", 5.0)),
        expiry_tick=20,
    )
    constraints = MissionConstraints(0.3, 1.0, 0.3, 0.08, 0.1)
    velocity_target = VelocityTarget(0.2, 0.1)
    mission = MissionIntent(
        context,
        mission_id="mission-1",
        mode=CommandMode.NAVIGATE,
        target_pose=Waypoint(4.0, 5.0),
        velocity_target=None,
        constraints=constraints,
        lifecycle=MissionLifecycle.ACTIVE,
    )
    navigation = NavigationPlan(
        context,
        mission_id="mission-1",
        route=(Waypoint(1.0, 2.0), Waypoint(4.0, 5.0, 0.0)),
        velocity_target=None,
        constraints=constraints,
        corridor_radius_m=0.3,
        progress=0.25,
        status=NavigationStatus.ACTIVE,
    )
    objective = MotionObjective(
        context,
        selected_source="navigation",
        kind=MotionObjectiveKind.TRACK_PLAN,
        priority=10,
        expiry_tick=8,
        selection_reason="highest-priority",
        target_waypoint=Waypoint(4.0, 5.0, 0.0),
        velocity_target=None,
        constraints=constraints,
    )
    motion = MotionIntent(context, 0.2, 0.1, 100_000_000, constraints)
    constrained = ConstrainedMotion(
        context,
        requested_v_mps=0.2,
        requested_omega_rad_s=0.1,
        allowed_v_mps=0.15,
        allowed_omega_rad_s=0.1,
        active_constraints=(ConstraintCode.LOCAL_CLEARANCE, ConstraintCode.SPEED_LIMIT),
    )
    wheels = WheelVelocitySetpoint(context, left_mps=0.13, right_mps=0.17)
    actuator = ActuatorRequest(context, 0.2, 0.25)
    final = FinalActuation(
        context,
        left_output=0.0,
        right_output=0.0,
        enabled=False,
        safety_decision=SafetyDecision.STOP,
        latch_state="STOPPED",
        reason="NOT_ACTIVE",
    )
    return (
        raw,
        acquisition,
        admitted,
        estimate,
        world,
        command,
        constraints,
        velocity_target,
        mission,
        navigation,
        objective,
        motion,
        constrained,
        wheels,
        actuator,
        final,
    )


def test_contract_surface_has_only_minimal_tick_metadata():
    context_fields = tuple(field.name for field in fields(TickContext))
    assert context_fields == ("tick_id", "monotonic_ns")

    removed_administrative_api = (
        "ActuationReceipt",
        "CandidateEvaluation",
        "ContractDecodeError",
        "ContractEnvelope",
        "Validity",
        "canonical_bytes",
        "canonical_sha256",
        "from_canonical_bytes",
    )
    assert not any(hasattr(contracts, name) for name in removed_administrative_api)


def test_contracts_are_frozen_slotted_and_compare_by_direct_value():
    first = contract_instances()
    second = contract_instances()

    assert first == second
    with pytest.raises(FrozenInstanceError):
        first[-1].enabled = True
    for contract in first:
        assert is_dataclass(contract)
        assert getattr(type(contract), "__slots__", None)


def _contains_any(annotation: object) -> bool:
    return annotation is Any or any(_contains_any(item) for item in get_args(annotation))


def test_boundary_annotations_do_not_use_any():
    for contract in contract_instances():
        for annotation in get_type_hints(type(contract)).values():
            assert not _contains_any(annotation), type(contract).__name__


def test_variable_scalar_fields_only_require_unique_keys():
    fields_out_of_alphabetical_order = (DataField("z", 1), DataField("a", 2))
    assert DeviceSample("encoder", "ticks", 1, 1, fields_out_of_alphabetical_order)

    with pytest.raises(ContractValidationError, match="unique"):
        DeviceSample(
            "encoder",
            "ticks",
            1,
            1,
            (DataField("ticks", 1), DataField("ticks", 2)),
        )


def test_domain_validation_keeps_nonfinite_and_expired_values_out():
    context = TickContext(7, 2_000)
    with pytest.raises(ContractValidationError, match="finite"):
        DataField("bad", float("nan"))
    with pytest.raises(ContractValidationError, match="expired"):
        CommandRequest(context, "old", CommandMode.STOP, (), expiry_tick=6)


def test_final_actuation_is_fail_closed():
    context = TickContext(7, 2_000)
    with pytest.raises(ContractValidationError, match="zero output"):
        FinalActuation(
            context,
            left_output=0.1,
            right_output=0.0,
            enabled=False,
            safety_decision=SafetyDecision.STOP,
            latch_state="STOPPED",
            reason="FAULT",
        )
    with pytest.raises(ContractValidationError, match="enabled"):
        FinalActuation(
            context,
            left_output=0.0,
            right_output=0.0,
            enabled=True,
            safety_decision=SafetyDecision.STOP,
            latch_state="STOPPED",
            reason="FAULT",
        )
    with pytest.raises(ContractValidationError, match="requires a reason"):
        FinalActuation(
            context,
            left_output=0.0,
            right_output=0.0,
            enabled=False,
            safety_decision=SafetyDecision.STOP,
            latch_state="STOPPED",
        )
