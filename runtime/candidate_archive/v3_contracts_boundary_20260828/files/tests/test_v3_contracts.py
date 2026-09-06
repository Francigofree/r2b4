import json
from dataclasses import FrozenInstanceError, fields, is_dataclass
from typing import Any, get_args, get_type_hints

import pytest

from v3.contracts import (
    AcquisitionFrame,
    ActuationReceipt,
    ActuatorRequest,
    AdmittedFrame,
    CandidateEvaluation,
    CommandMode,
    CommandRequest,
    ConstrainedMotion,
    ConstraintCode,
    ContractDecodeError,
    ContractEnvelope,
    ContractValidationError,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    FinalActuation,
    MissionIntent,
    MissionLifecycle,
    MotionIntent,
    MotionObjective,
    MotionObjectiveKind,
    MotionSample,
    NavigationPlan,
    NavigationStatus,
    Observation,
    ObstacleTrack,
    RawDeviceBatch,
    RejectedObservation,
    RejectionReason,
    RobotEstimate,
    SafetyDecision,
    SourceTrust,
    SourceWatermark,
    TrustLevel,
    Validity,
    Waypoint,
    WheelVelocitySetpoint,
    WorldSnapshot,
    WriteStatus,
    canonical_bytes,
    canonical_sha256,
    from_canonical_bytes,
)


CONFIG_ID = "a" * 64
OBSERVATION_ID = "b" * 64
REQUEST_ID = "c" * 64
STATE_HASH = "d" * 64
OCCUPANCY_HASH = "e" * 64


def meta(schema_id: str, sequence: int = 1) -> ContractEnvelope:
    return ContractEnvelope(
        schema_id=schema_id,
        schema_version=1,
        session_id="session-001",
        tick_id=7,
        producer_id="test-producer",
        source_sequence=sequence,
        captured_monotonic_ns=1_000,
        published_monotonic_ns=2_000,
        config_set_id=CONFIG_ID,
    )


def contract_instances() -> tuple[object, ...]:
    sample = DeviceSample(
        device_id="encoder-left",
        sample_kind="wheel_ticks",
        device_sequence=4,
        host_monotonic_ns=1_000,
        payload=(
            DataField("available", True),
            DataField("calibration", None),
            DataField("label", "left"),
            DataField("scale", 1.0),
            DataField("ticks", 12),
        ),
    )
    health = DeviceHealth("encoder-left", DeviceHealthState.OK)
    raw = RawDeviceBatch(
        meta("R2B4_V3_RAW_DEVICE_BATCH"),
        samples=(sample,),
        device_health=(health,),
    )
    acquisition = AcquisitionFrame(
        meta("R2B4_V3_ACQUISITION_FRAME", 2),
        samples=(sample,),
        source_watermarks=(SourceWatermark("encoder-left", 4, 1_000),),
        io_health=(health,),
    )
    observation = Observation(
        observation_id=OBSERVATION_ID,
        observation_kind="wheel_velocity",
        source_device_id="encoder-left",
        source_sequence=4,
        captured_monotonic_ns=1_000,
        values=(DataField("mps", 0.25),),
    )
    admitted = AdmittedFrame(
        meta("R2B4_V3_ADMITTED_FRAME", 3),
        accepted=(observation,),
        rejected=(
            RejectedObservation(REQUEST_ID, RejectionReason.STALE, 500, 3),
        ),
        alignment_epoch=2,
        trust_summary=(SourceTrust("encoder-left", TrustLevel.TRUSTED),),
    )
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    estimate = RobotEstimate(
        meta("R2B4_V3_ROBOT_ESTIMATE", 4),
        frame_id="map",
        x_m=1.0,
        y_m=2.0,
        yaw_rad=0.1,
        v_mps=0.2,
        omega_rad_s=0.0,
        covariance_5x5=covariance,
        estimator_generation=3,
    )
    world = WorldSnapshot(
        meta("R2B4_V3_WORLD_SNAPSHOT", 5),
        frame_id="map",
        map_revision=8,
        occupancy_hash=OCCUPANCY_HASH,
        obstacle_tracks=(ObstacleTrack("track-1", 2.0, 3.0, 0.2, 0.0, 0.0, 0.9),),
        freshness_ns=20_000,
    )
    command = CommandRequest(
        meta("R2B4_V3_COMMAND_REQUEST", 6),
        command_id="command-1",
        issuer_id="operator-1",
        authority_lease_id="lease-1",
        mode=CommandMode.NAVIGATE,
        goal=(DataField("x_m", 4.0), DataField("y_m", 5.0)),
        issued_tick=7,
        expiry_tick=20,
    )
    mission = MissionIntent(
        meta("R2B4_V3_MISSION_INTENT", 7),
        mission_id="mission-1",
        mission_revision=1,
        mode=CommandMode.NAVIGATE,
        goal=(DataField("x_m", 4.0), DataField("y_m", 5.0)),
        constraints=(DataField("max_v_mps", 0.3),),
        lifecycle=MissionLifecycle.ACTIVE,
    )
    navigation = NavigationPlan(
        meta("R2B4_V3_NAVIGATION_PLAN", 8),
        plan_id="plan-1",
        plan_revision=1,
        mission_revision=1,
        route=(Waypoint(1.0, 2.0, None), Waypoint(4.0, 5.0, 0.0)),
        corridor_radius_m=0.3,
        progress=0.25,
        status=NavigationStatus.ACTIVE,
        terminal_condition="goal-tolerance",
    )
    objective = MotionObjective(
        meta("R2B4_V3_MOTION_OBJECTIVE", 9),
        selected_candidate_id="navigation",
        kind=MotionObjectiveKind.TRACK_PLAN,
        priority=10,
        expiry_tick=8,
        arbitration_proof=(CandidateEvaluation("navigation", 10, True, ()),),
    )
    motion = MotionIntent(
        meta("R2B4_V3_MOTION_INTENT", 10),
        requested_v_mps=0.2,
        requested_omega_rad_s=0.1,
        horizon_ns=100_000_000,
        reference_samples=(MotionSample(0, 0.2, 0.1), MotionSample(50_000_000, 0.2, 0.0)),
    )
    constrained = ConstrainedMotion(
        meta("R2B4_V3_CONSTRAINED_MOTION", 11),
        requested_v_mps=0.2,
        requested_omega_rad_s=0.1,
        allowed_v_mps=0.15,
        allowed_omega_rad_s=0.1,
        active_constraints=(ConstraintCode.LOCAL_CLEARANCE, ConstraintCode.SPEED_LIMIT),
        limiting_facts=(DataField("clearance_m", 0.4),),
    )
    wheels = WheelVelocitySetpoint(
        meta("R2B4_V3_WHEEL_VELOCITY_SETPOINT", 12),
        left_mps=0.13,
        right_mps=0.17,
        kinematic_model_id="differential-v1",
        source_motion_event_id=motion.meta.event_id,
    )
    actuator = ActuatorRequest(
        meta("R2B4_V3_ACTUATOR_REQUEST", 13),
        left_normalized=0.2,
        right_normalized=0.25,
        controller_state_hash=STATE_HASH,
        saturation_facts=(),
    )
    final = FinalActuation(
        meta("R2B4_V3_FINAL_ACTUATION", 14, ),
        left_output=0.0,
        right_output=0.0,
        enabled=False,
        safety_decision=SafetyDecision.STOP,
        latch_state="idle",
        source_request_event_id=actuator.meta.event_id,
        reason_codes=("NOT_ACTIVATED",),
    )
    receipt = ActuationReceipt(
        meta("R2B4_V3_ACTUATION_RECEIPT", 15),
        requested_actuation_event_id=final.meta.event_id,
        driver_sequence=1,
        requested_left_output=0.0,
        requested_right_output=0.0,
        applied_left_output=0.0,
        applied_right_output=0.0,
        write_status=WriteStatus.APPLIED,
        hardware_faults=(),
    )
    return (
        raw,
        acquisition,
        admitted,
        estimate,
        world,
        command,
        mission,
        navigation,
        objective,
        motion,
        constrained,
        wheels,
        actuator,
        final,
        receipt,
    )


@pytest.mark.parametrize("contract", contract_instances(), ids=lambda item: type(item).__name__)
def test_every_top_level_contract_has_deterministic_canonical_round_trip(contract):
    encoded = canonical_bytes(contract)

    decoded = from_canonical_bytes(encoded, type(contract))

    assert decoded == contract
    assert canonical_bytes(decoded) == encoded
    assert canonical_sha256(decoded) == canonical_sha256(contract)


def test_envelope_event_id_is_deterministic_and_ignores_non_identity_metadata():
    first = meta("R2B4_V3_TEST_EVENT")
    second = ContractEnvelope(
        schema_id=first.schema_id,
        schema_version=first.schema_version,
        session_id=first.session_id,
        tick_id=99,
        producer_id=first.producer_id,
        source_sequence=first.source_sequence,
        captured_monotonic_ns=9_000,
        published_monotonic_ns=9_001,
        config_set_id="f" * 64,
        validity=Validity.DEGRADED,
        reason_codes=("TEST_DEGRADED",),
    )

    assert first.event_id == second.event_id
    assert len(first.event_id) == 64


def test_contracts_are_frozen_and_payload_collections_are_tuples():
    final = contract_instances()[-2]

    with pytest.raises(FrozenInstanceError):
        final.enabled = True

    for contract in contract_instances():
        assert is_dataclass(contract)
        assert getattr(type(contract), "__slots__", None)


def _contains_any(annotation: object) -> bool:
    return annotation is Any or any(_contains_any(item) for item in get_args(annotation))


def test_boundary_annotations_do_not_use_any():
    for contract in contract_instances():
        for annotation in get_type_hints(type(contract)).values():
            assert not _contains_any(annotation), type(contract).__name__
        for field in fields(contract):
            assert field.name != "payload" or isinstance(getattr(contract, field.name), tuple)


@pytest.mark.parametrize(
    "changes, expected",
    (
        ({"config_set_id": "bad"}, "config_set_id"),
        ({"captured_monotonic_ns": 3_000}, "cannot exceed"),
        (
            {
                "validity": Validity.INVALID,
                "reason_codes": (),
            },
            "requires at least one reason code",
        ),
        (
            {
                "causation_ids": ("f" * 64, "e" * 64),
            },
            "sorted and unique",
        ),
    ),
)
def test_envelope_rejects_invalid_metadata(changes, expected):
    values = {
        "schema_id": "R2B4_V3_TEST_EVENT",
        "schema_version": 1,
        "session_id": "session-001",
        "tick_id": 7,
        "producer_id": "test-producer",
        "source_sequence": 1,
        "captured_monotonic_ns": 1_000,
        "published_monotonic_ns": 2_000,
        "config_set_id": CONFIG_ID,
    }
    values.update(changes)

    with pytest.raises(ContractValidationError, match=expected):
        ContractEnvelope(**values)


def test_nonfinite_float_and_nondeterministic_field_order_are_rejected():
    with pytest.raises(ContractValidationError, match="finite"):
        DataField("bad", float("nan"))
    with pytest.raises(ContractValidationError, match="sorted and unique"):
        DeviceSample(
            "encoder-left",
            "ticks",
            1,
            1,
            (DataField("z", 1), DataField("a", 2)),
        )


def test_final_actuation_is_fail_closed():
    actuator = contract_instances()[-3]
    final_meta = meta("R2B4_V3_FINAL_ACTUATION")

    with pytest.raises(ContractValidationError, match="zero output"):
        FinalActuation(
            final_meta,
            left_output=0.1,
            right_output=0.0,
            enabled=False,
            safety_decision=SafetyDecision.STOP,
            latch_state="fault",
            source_request_event_id=actuator.meta.event_id,
            reason_codes=("FAULT",),
        )
    with pytest.raises(ContractValidationError, match="enabled bit"):
        FinalActuation(
            final_meta,
            left_output=0.0,
            right_output=0.0,
            enabled=True,
            safety_decision=SafetyDecision.STOP,
            latch_state="fault",
            source_request_event_id=actuator.meta.event_id,
            reason_codes=("FAULT",),
        )
    with pytest.raises(ContractValidationError, match="stable uppercase"):
        FinalActuation(
            final_meta,
            left_output=0.0,
            right_output=0.0,
            enabled=False,
            safety_decision=SafetyDecision.STOP,
            latch_state="fault",
            source_request_event_id=actuator.meta.event_id,
            reason_codes=("not_uppercase",),
        )


def test_decoder_rejects_type_marker_or_field_set_tampering():
    contract = contract_instances()[0]
    payload = json.loads(canonical_bytes(contract))
    payload["$type"] = "v3.contracts.messages.WorldSnapshot"

    with pytest.raises(ContractDecodeError, match="type marker"):
        from_canonical_bytes(json.dumps(payload).encode(), type(contract))

    payload = json.loads(canonical_bytes(contract))
    payload["unexpected"] = True
    with pytest.raises(ContractDecodeError, match="field set mismatch"):
        from_canonical_bytes(json.dumps(payload).encode(), type(contract))


@pytest.mark.parametrize("payload", (b"NaN", b"Infinity", b"-Infinity"))
def test_decoder_rejects_nonfinite_json_constants(payload):
    with pytest.raises(ContractDecodeError, match="non-finite JSON constant"):
        from_canonical_bytes(payload, float)


def test_schema_id_is_bound_to_message_type():
    with pytest.raises(ContractValidationError, match="metadata schema"):
        RawDeviceBatch(meta("R2B4_V3_WORLD_SNAPSHOT"), (), ())
