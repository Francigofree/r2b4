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
    CostmapCell,
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
    RollingLocalCostmap,
    SafetyDecision,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    VelocityTarget,
    Waypoint,
    WheelVelocitySetpoint,
    WorldSnapshot,
)


def test_rejection_identity_preserves_distinct_reasons_for_one_source_revision():
    stale = RejectedObservation("lidar", 60, RejectionReason.STALE, 384_680_281)
    duplicate = RejectedObservation("lidar", 60, RejectionReason.DUPLICATE, 230_789_068)
    frame = AdmittedFrame(TickContext(253, 4298832040810), (), (stale, duplicate))
    assert frame.rejected == (stale, duplicate)
    with pytest.raises(ContractValidationError, match="duplicates"):
        AdmittedFrame(frame.context, (), (stale, stale))


@pytest.mark.parametrize("value", [None, False, True, 60, 1.25, "LIDAR_STALE"])
def test_scalar_capture_leaf_pickle_preserves_value_type_and_immutability(value, monkeypatch):
    import dataclasses
    import pickle

    field = DataField("measurement", value)
    for protocol in range(2, pickle.HIGHEST_PROTOCOL + 1):
        restored = pickle.loads(pickle.dumps(field, protocol=protocol))
        assert restored == field
        assert type(restored.value) is type(value)
        with pytest.raises(FrozenInstanceError):
            restored.value = None
    # Existing list-based dataclass pickles remain readable as well.
    with monkeypatch.context() as old:
        old.setattr(DataField, "__reduce__", object.__reduce__)
        old.setattr(DataField, "__getstate__", dataclasses._dataclass_getstate)
        wire = pickle.dumps(field)
    assert pickle.loads(wire) == field


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
    costmap_cell = CostmapCell(10, 20, 2)
    costmap = RollingLocalCostmap("map", 8, 0.1, 2.5, (costmap_cell,), 7, 20_000)
    world = WorldSnapshot(
        context,
        frame_id="map",
        map_revision=8,
        obstacle_tracks=(ObstacleTrack("track-1", 2.0, 3.0, 0.2, 0.0, 0.0, 0.9),),
        freshness_ns=20_000,
        local_costmap=costmap,
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
    trajectory_pose = TrajectoryPose(1.1, 2.0, 0.1, 100_000_000)
    trajectory = TrajectoryEvaluation(
        "trajectory-00-00",
        0.1,
        0.0,
        100_000_000,
        (trajectory_pose,),
        False,
        0.4,
        0.2,
        0.8,
        1.0,
        0.55,
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
        costmap_cell,
        costmap,
        world,
        command,
        constraints,
        velocity_target,
        mission,
        navigation,
        objective,
        trajectory_pose,
        trajectory,
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


def test_motion_validity_and_track_prediction_require_explicit_valid_lineage():
    from v3.contracts import MotionValidity, TrackEstimateStatus

    context = TickContext(1, 100)
    validity = MotionValidity(context, 200, "map", "FOLLOW_PERSON:person-1")
    assert validity.usable_at(TickContext(2, 200))
    assert not validity.usable_at(TickContext(2, 201))
    assert not validity.usable_at(TickContext(0, 150))
    with pytest.raises(ContractValidationError):
        MotionValidity(context, 99, "map", "follow")
    with pytest.raises(ContractValidationError):
        ObstacleTrack("person-1", 0, 0, 0.3, 0, 0, 0.9,
                      estimate_status=TrackEstimateStatus.PREDICTED)
    with pytest.raises(ContractValidationError):
        ObstacleTrack("person-1", 0, 0, 0.3, 0, 0, 0.9,
                      measurement_monotonic_ns=100, prediction_valid_until_ns=99)


@pytest.mark.parametrize("allowed,previous", [(0.5, 0.4), (-0.1, 0.4), (0.4, 0.4)])
def test_transition_origin_cannot_create_amplify_or_move_away_from_zero(allowed, previous):
    with pytest.raises(ContractValidationError):
        ConstrainedMotion(TickContext(2, 200), 0, 0, 0, allowed,
                          (ConstraintCode.ACCELERATION_LIMIT,),
                          VelocityTarget(0, previous), TickContext(1, 100))


def test_deceleration_requires_explicit_consecutive_transition_origin():
    current = TickContext(2, 200)
    with pytest.raises(ContractValidationError):
        ConstrainedMotion(current, 0, 0, 0, 0.3, (ConstraintCode.ACCELERATION_LIMIT,))
    with pytest.raises(ContractValidationError):
        ConstrainedMotion(current, 0, 0, 0, 0.3, (ConstraintCode.ACCELERATION_LIMIT,),
                          VelocityTarget(0, 0.4), TickContext(0, 100))
    valid = ConstrainedMotion(current, 0, 0, 0, 0.3, (ConstraintCode.ACCELERATION_LIMIT,),
                              VelocityTarget(0, 0.4), TickContext(1, 100))
    assert valid.allowed_omega_rad_s == 0.3
