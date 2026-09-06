"""Typed immutable contracts for every normative V3 layer boundary."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .base import (
    ContractEnvelope,
    ContractValidationError,
    DataField,
    require_data_fields,
    require_finite,
    require_nonnegative,
    require_reason_codes,
    require_schema,
    require_sha256,
    require_sorted_unique,
    require_token,
    require_unit_interval,
)


def _require_ordered(values: tuple[object, ...], keys: tuple[object, ...], field_name: str) -> None:
    if (
        len(values) != len(keys)
        or keys != tuple(sorted(keys))
        or len(set(keys)) != len(keys)
    ):
        raise ContractValidationError(f"{field_name} must be deterministically ordered and unique")


def _require_normalized(value: float, field_name: str) -> None:
    require_finite(value, field_name)
    if not -1.0 <= float(value) <= 1.0:
        raise ContractValidationError(f"{field_name} must be in [-1, 1]")


class DeviceHealthState(str, Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"
    UNKNOWN = "UNKNOWN"


class RejectionReason(str, Enum):
    DUPLICATE = "DUPLICATE"
    INVALID = "INVALID"
    OUT_OF_ORDER = "OUT_OF_ORDER"
    STALE = "STALE"
    TIME_ALIGNMENT_FAILED = "TIME_ALIGNMENT_FAILED"
    UNTRUSTED = "UNTRUSTED"
    UNKNOWN_SCHEMA = "UNKNOWN_SCHEMA"


class TrustLevel(str, Enum):
    TRUSTED = "TRUSTED"
    DEGRADED = "DEGRADED"
    UNTRUSTED = "UNTRUSTED"


class CommandMode(str, Enum):
    STOP = "STOP"
    TELEOP = "TELEOP"
    NAVIGATE = "NAVIGATE"
    SERVICE = "SERVICE"


class MissionLifecycle(str, Enum):
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"


class NavigationStatus(str, Enum):
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    COMPLETE = "COMPLETE"
    NO_PATH = "NO_PATH"
    INVALIDATED = "INVALIDATED"


class MotionObjectiveKind(str, Enum):
    STOP = "STOP"
    TRACK_PLAN = "TRACK_PLAN"
    VELOCITY = "VELOCITY"


class ConstraintCode(str, Enum):
    ACCELERATION_LIMIT = "ACCELERATION_LIMIT"
    CURVATURE_LIMIT = "CURVATURE_LIMIT"
    LOCAL_CLEARANCE = "LOCAL_CLEARANCE"
    LOCALIZATION_DEGRADED = "LOCALIZATION_DEGRADED"
    MISSION_LIMIT = "MISSION_LIMIT"
    SPEED_LIMIT = "SPEED_LIMIT"


class SafetyDecision(str, Enum):
    ALLOW = "ALLOW"
    SERVICE_ALLOW = "SERVICE_ALLOW"
    STOP = "STOP"
    FAULT = "FAULT"


class WriteStatus(str, Enum):
    APPLIED = "APPLIED"
    REJECTED = "REJECTED"
    FAILED = "FAILED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True, slots=True)
class DeviceHealth:
    device_id: str
    state: DeviceHealthState
    reason_codes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        require_token(self.device_id, "DeviceHealth.device_id")
        require_reason_codes(
            self.reason_codes,
            "DeviceHealth.reason_codes",
            required=self.state is not DeviceHealthState.OK,
        )


@dataclass(frozen=True, slots=True)
class DeviceSample:
    device_id: str
    sample_kind: str
    device_sequence: int
    host_monotonic_ns: int
    payload: tuple[DataField, ...]

    def __post_init__(self) -> None:
        require_token(self.device_id, "DeviceSample.device_id")
        require_token(self.sample_kind, "DeviceSample.sample_kind")
        require_nonnegative(self.device_sequence, "DeviceSample.device_sequence")
        require_nonnegative(self.host_monotonic_ns, "DeviceSample.host_monotonic_ns")
        require_data_fields(self.payload, "DeviceSample.payload")


@dataclass(frozen=True, slots=True)
class RawDeviceBatch:
    meta: ContractEnvelope
    samples: tuple[DeviceSample, ...]
    device_health: tuple[DeviceHealth, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_RAW_DEVICE_BATCH")
        sample_keys = tuple(
            (sample.host_monotonic_ns, sample.device_id, sample.sample_kind, sample.device_sequence)
            for sample in self.samples
        )
        _require_ordered(self.samples, sample_keys, "RawDeviceBatch.samples")
        health_keys = tuple(item.device_id for item in self.device_health)
        _require_ordered(self.device_health, health_keys, "RawDeviceBatch.device_health")


@dataclass(frozen=True, slots=True)
class SourceWatermark:
    source_id: str
    last_sequence: int
    last_capture_monotonic_ns: int

    def __post_init__(self) -> None:
        require_token(self.source_id, "SourceWatermark.source_id")
        require_nonnegative(self.last_sequence, "SourceWatermark.last_sequence")
        require_nonnegative(
            self.last_capture_monotonic_ns,
            "SourceWatermark.last_capture_monotonic_ns",
        )


@dataclass(frozen=True, slots=True)
class AcquisitionFrame:
    meta: ContractEnvelope
    samples: tuple[DeviceSample, ...]
    source_watermarks: tuple[SourceWatermark, ...]
    io_health: tuple[DeviceHealth, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_ACQUISITION_FRAME")
        sample_keys = tuple(
            (sample.host_monotonic_ns, sample.device_id, sample.sample_kind, sample.device_sequence)
            for sample in self.samples
        )
        _require_ordered(self.samples, sample_keys, "AcquisitionFrame.samples")
        watermark_keys = tuple(item.source_id for item in self.source_watermarks)
        _require_ordered(
            self.source_watermarks,
            watermark_keys,
            "AcquisitionFrame.source_watermarks",
        )
        health_keys = tuple(item.device_id for item in self.io_health)
        _require_ordered(self.io_health, health_keys, "AcquisitionFrame.io_health")


@dataclass(frozen=True, slots=True)
class Observation:
    observation_id: str
    observation_kind: str
    source_device_id: str
    source_sequence: int
    captured_monotonic_ns: int
    values: tuple[DataField, ...]

    def __post_init__(self) -> None:
        require_sha256(self.observation_id, "Observation.observation_id")
        require_token(self.observation_kind, "Observation.observation_kind")
        require_token(self.source_device_id, "Observation.source_device_id")
        require_nonnegative(self.source_sequence, "Observation.source_sequence")
        require_nonnegative(self.captured_monotonic_ns, "Observation.captured_monotonic_ns")
        require_data_fields(self.values, "Observation.values")


@dataclass(frozen=True, slots=True)
class RejectedObservation:
    input_event_id: str
    reason: RejectionReason
    observed_age_ns: int
    source_sequence: int

    def __post_init__(self) -> None:
        require_sha256(self.input_event_id, "RejectedObservation.input_event_id")
        require_nonnegative(self.observed_age_ns, "RejectedObservation.observed_age_ns")
        require_nonnegative(self.source_sequence, "RejectedObservation.source_sequence")


@dataclass(frozen=True, slots=True)
class SourceTrust:
    source_id: str
    level: TrustLevel
    reason_codes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        require_token(self.source_id, "SourceTrust.source_id")
        require_reason_codes(
            self.reason_codes,
            "SourceTrust.reason_codes",
            required=self.level is not TrustLevel.TRUSTED,
        )


@dataclass(frozen=True, slots=True)
class AdmittedFrame:
    meta: ContractEnvelope
    accepted: tuple[Observation, ...]
    rejected: tuple[RejectedObservation, ...]
    alignment_epoch: int
    trust_summary: tuple[SourceTrust, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_ADMITTED_FRAME")
        accepted_keys = tuple(item.observation_id for item in self.accepted)
        _require_ordered(self.accepted, accepted_keys, "AdmittedFrame.accepted")
        rejected_keys = tuple((item.input_event_id, item.reason.value) for item in self.rejected)
        _require_ordered(self.rejected, rejected_keys, "AdmittedFrame.rejected")
        require_nonnegative(self.alignment_epoch, "AdmittedFrame.alignment_epoch")
        trust_keys = tuple(item.source_id for item in self.trust_summary)
        _require_ordered(self.trust_summary, trust_keys, "AdmittedFrame.trust_summary")


@dataclass(frozen=True, slots=True)
class RobotEstimate:
    meta: ContractEnvelope
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    v_mps: float
    omega_rad_s: float
    covariance_5x5: tuple[float, ...]
    estimator_generation: int

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_ROBOT_ESTIMATE")
        require_token(self.frame_id, "RobotEstimate.frame_id")
        for name in ("x_m", "y_m", "yaw_rad", "v_mps", "omega_rad_s"):
            require_finite(getattr(self, name), f"RobotEstimate.{name}")
        if len(self.covariance_5x5) != 25:
            raise ContractValidationError("RobotEstimate.covariance_5x5 must contain 25 values")
        for index, value in enumerate(self.covariance_5x5):
            require_finite(value, f"RobotEstimate.covariance_5x5[{index}]")
        if any(self.covariance_5x5[index * 5 + index] < 0.0 for index in range(5)):
            raise ContractValidationError("RobotEstimate covariance diagonal cannot be negative")
        require_nonnegative(self.estimator_generation, "RobotEstimate.estimator_generation")


@dataclass(frozen=True, slots=True)
class ObstacleTrack:
    track_id: str
    x_m: float
    y_m: float
    radius_m: float
    vx_mps: float
    vy_mps: float
    confidence: float

    def __post_init__(self) -> None:
        require_token(self.track_id, "ObstacleTrack.track_id")
        for name in ("x_m", "y_m", "radius_m", "vx_mps", "vy_mps"):
            require_finite(getattr(self, name), f"ObstacleTrack.{name}")
        if self.radius_m < 0.0:
            raise ContractValidationError("ObstacleTrack.radius_m cannot be negative")
        require_unit_interval(self.confidence, "ObstacleTrack.confidence")


@dataclass(frozen=True, slots=True)
class WorldSnapshot:
    meta: ContractEnvelope
    frame_id: str
    map_revision: int
    occupancy_hash: str
    obstacle_tracks: tuple[ObstacleTrack, ...]
    freshness_ns: int

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_WORLD_SNAPSHOT")
        require_token(self.frame_id, "WorldSnapshot.frame_id")
        require_nonnegative(self.map_revision, "WorldSnapshot.map_revision")
        require_sha256(self.occupancy_hash, "WorldSnapshot.occupancy_hash")
        track_keys = tuple(item.track_id for item in self.obstacle_tracks)
        _require_ordered(self.obstacle_tracks, track_keys, "WorldSnapshot.obstacle_tracks")
        require_nonnegative(self.freshness_ns, "WorldSnapshot.freshness_ns")


@dataclass(frozen=True, slots=True)
class CommandRequest:
    meta: ContractEnvelope
    command_id: str
    issuer_id: str
    authority_lease_id: str
    mode: CommandMode
    goal: tuple[DataField, ...]
    issued_tick: int
    expiry_tick: int

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_COMMAND_REQUEST")
        require_token(self.command_id, "CommandRequest.command_id")
        require_token(self.issuer_id, "CommandRequest.issuer_id")
        require_token(self.authority_lease_id, "CommandRequest.authority_lease_id")
        require_data_fields(self.goal, "CommandRequest.goal")
        require_nonnegative(self.issued_tick, "CommandRequest.issued_tick")
        require_nonnegative(self.expiry_tick, "CommandRequest.expiry_tick")
        if self.expiry_tick < self.issued_tick:
            raise ContractValidationError("CommandRequest expiry cannot precede issue tick")


@dataclass(frozen=True, slots=True)
class MissionIntent:
    meta: ContractEnvelope
    mission_id: str
    mission_revision: int
    mode: CommandMode
    goal: tuple[DataField, ...]
    constraints: tuple[DataField, ...]
    lifecycle: MissionLifecycle
    stop_reason: str | None = None

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_MISSION_INTENT")
        require_token(self.mission_id, "MissionIntent.mission_id")
        require_nonnegative(self.mission_revision, "MissionIntent.mission_revision")
        require_data_fields(self.goal, "MissionIntent.goal")
        require_data_fields(self.constraints, "MissionIntent.constraints")
        if self.stop_reason is not None:
            require_token(self.stop_reason, "MissionIntent.stop_reason")
        if self.mode is CommandMode.STOP and self.stop_reason is None:
            raise ContractValidationError("STOP MissionIntent requires stop_reason")


@dataclass(frozen=True, slots=True)
class Waypoint:
    x_m: float
    y_m: float
    yaw_rad: float | None = None

    def __post_init__(self) -> None:
        require_finite(self.x_m, "Waypoint.x_m")
        require_finite(self.y_m, "Waypoint.y_m")
        if self.yaw_rad is not None:
            require_finite(self.yaw_rad, "Waypoint.yaw_rad")


@dataclass(frozen=True, slots=True)
class NavigationPlan:
    meta: ContractEnvelope
    plan_id: str
    plan_revision: int
    mission_revision: int
    route: tuple[Waypoint, ...]
    corridor_radius_m: float
    progress: float
    status: NavigationStatus
    terminal_condition: str

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_NAVIGATION_PLAN")
        require_token(self.plan_id, "NavigationPlan.plan_id")
        require_nonnegative(self.plan_revision, "NavigationPlan.plan_revision")
        require_nonnegative(self.mission_revision, "NavigationPlan.mission_revision")
        require_finite(self.corridor_radius_m, "NavigationPlan.corridor_radius_m")
        if self.corridor_radius_m < 0.0:
            raise ContractValidationError("NavigationPlan corridor radius cannot be negative")
        require_unit_interval(self.progress, "NavigationPlan.progress")
        require_token(self.terminal_condition, "NavigationPlan.terminal_condition")
        if self.status is NavigationStatus.ACTIVE and not self.route:
            raise ContractValidationError("active NavigationPlan requires a route")


@dataclass(frozen=True, slots=True)
class CandidateEvaluation:
    candidate_id: str
    priority: int
    accepted: bool
    reason_codes: tuple[str, ...]

    def __post_init__(self) -> None:
        require_token(self.candidate_id, "CandidateEvaluation.candidate_id")
        if not isinstance(self.priority, int) or isinstance(self.priority, bool):
            raise ContractValidationError("CandidateEvaluation.priority must be an integer")
        require_reason_codes(
            self.reason_codes,
            "CandidateEvaluation.reason_codes",
            required=not self.accepted,
        )


@dataclass(frozen=True, slots=True)
class MotionObjective:
    meta: ContractEnvelope
    selected_candidate_id: str
    kind: MotionObjectiveKind
    priority: int
    expiry_tick: int
    arbitration_proof: tuple[CandidateEvaluation, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_MOTION_OBJECTIVE")
        require_token(self.selected_candidate_id, "MotionObjective.selected_candidate_id")
        if not isinstance(self.priority, int) or isinstance(self.priority, bool):
            raise ContractValidationError("MotionObjective.priority must be an integer")
        require_nonnegative(self.expiry_tick, "MotionObjective.expiry_tick")
        proof_keys = tuple(item.candidate_id for item in self.arbitration_proof)
        _require_ordered(
            self.arbitration_proof,
            proof_keys,
            "MotionObjective.arbitration_proof",
        )
        selected = [
            item
            for item in self.arbitration_proof
            if item.candidate_id == self.selected_candidate_id and item.accepted
        ]
        if len(selected) != 1:
            raise ContractValidationError("selected candidate must have one accepted proof row")


@dataclass(frozen=True, slots=True)
class MotionSample:
    offset_ns: int
    v_mps: float
    omega_rad_s: float

    def __post_init__(self) -> None:
        require_nonnegative(self.offset_ns, "MotionSample.offset_ns")
        require_finite(self.v_mps, "MotionSample.v_mps")
        require_finite(self.omega_rad_s, "MotionSample.omega_rad_s")


@dataclass(frozen=True, slots=True)
class MotionIntent:
    meta: ContractEnvelope
    requested_v_mps: float
    requested_omega_rad_s: float
    horizon_ns: int
    reference_samples: tuple[MotionSample, ...]
    stop_reason: str | None = None

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_MOTION_INTENT")
        require_finite(self.requested_v_mps, "MotionIntent.requested_v_mps")
        require_finite(self.requested_omega_rad_s, "MotionIntent.requested_omega_rad_s")
        require_nonnegative(self.horizon_ns, "MotionIntent.horizon_ns")
        sample_keys = tuple(item.offset_ns for item in self.reference_samples)
        _require_ordered(self.reference_samples, sample_keys, "MotionIntent.reference_samples")
        if len(set(sample_keys)) != len(sample_keys):
            raise ContractValidationError("MotionIntent sample offsets must be unique")
        if self.stop_reason is not None:
            require_token(self.stop_reason, "MotionIntent.stop_reason")
        if self.stop_reason is not None and (
            self.requested_v_mps != 0.0 or self.requested_omega_rad_s != 0.0
        ):
            raise ContractValidationError("stopped MotionIntent must request zero motion")


@dataclass(frozen=True, slots=True)
class ConstrainedMotion:
    meta: ContractEnvelope
    requested_v_mps: float
    requested_omega_rad_s: float
    allowed_v_mps: float
    allowed_omega_rad_s: float
    active_constraints: tuple[ConstraintCode, ...]
    limiting_facts: tuple[DataField, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_CONSTRAINED_MOTION")
        for name in (
            "requested_v_mps",
            "requested_omega_rad_s",
            "allowed_v_mps",
            "allowed_omega_rad_s",
        ):
            require_finite(getattr(self, name), f"ConstrainedMotion.{name}")
        constraint_values = tuple(item.value for item in self.active_constraints)
        if constraint_values != tuple(sorted(set(constraint_values))):
            raise ContractValidationError("active_constraints must be sorted and unique")
        require_data_fields(self.limiting_facts, "ConstrainedMotion.limiting_facts")


@dataclass(frozen=True, slots=True)
class WheelVelocitySetpoint:
    meta: ContractEnvelope
    left_mps: float
    right_mps: float
    kinematic_model_id: str
    source_motion_event_id: str

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_WHEEL_VELOCITY_SETPOINT")
        require_finite(self.left_mps, "WheelVelocitySetpoint.left_mps")
        require_finite(self.right_mps, "WheelVelocitySetpoint.right_mps")
        require_token(self.kinematic_model_id, "WheelVelocitySetpoint.kinematic_model_id")
        require_sha256(self.source_motion_event_id, "WheelVelocitySetpoint.source_motion_event_id")


@dataclass(frozen=True, slots=True)
class ActuatorRequest:
    meta: ContractEnvelope
    left_normalized: float
    right_normalized: float
    controller_state_hash: str
    saturation_facts: tuple[DataField, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_ACTUATOR_REQUEST")
        _require_normalized(self.left_normalized, "ActuatorRequest.left_normalized")
        _require_normalized(self.right_normalized, "ActuatorRequest.right_normalized")
        require_sha256(self.controller_state_hash, "ActuatorRequest.controller_state_hash")
        require_data_fields(self.saturation_facts, "ActuatorRequest.saturation_facts")


@dataclass(frozen=True, slots=True)
class FinalActuation:
    meta: ContractEnvelope
    left_output: float
    right_output: float
    enabled: bool
    safety_decision: SafetyDecision
    latch_state: str
    source_request_event_id: str
    reason_codes: tuple[str, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_FINAL_ACTUATION")
        _require_normalized(self.left_output, "FinalActuation.left_output")
        _require_normalized(self.right_output, "FinalActuation.right_output")
        require_token(self.latch_state, "FinalActuation.latch_state")
        require_sha256(self.source_request_event_id, "FinalActuation.source_request_event_id")
        require_reason_codes(
            self.reason_codes,
            "FinalActuation.reason_codes",
            required=not self.enabled,
        )
        allowing = self.safety_decision in {SafetyDecision.ALLOW, SafetyDecision.SERVICE_ALLOW}
        if self.enabled != allowing:
            raise ContractValidationError("FinalActuation enabled bit must match safety decision")
        if not self.enabled and (self.left_output != 0.0 or self.right_output != 0.0):
            raise ContractValidationError("disabled FinalActuation must command zero output")


@dataclass(frozen=True, slots=True)
class ActuationReceipt:
    meta: ContractEnvelope
    requested_actuation_event_id: str
    driver_sequence: int
    requested_left_output: float
    requested_right_output: float
    applied_left_output: float
    applied_right_output: float
    write_status: WriteStatus
    hardware_faults: tuple[str, ...]

    def __post_init__(self) -> None:
        require_schema(self.meta, "R2B4_V3_ACTUATION_RECEIPT")
        require_sha256(
            self.requested_actuation_event_id,
            "ActuationReceipt.requested_actuation_event_id",
        )
        require_nonnegative(self.driver_sequence, "ActuationReceipt.driver_sequence")
        for name in (
            "requested_left_output",
            "requested_right_output",
            "applied_left_output",
            "applied_right_output",
        ):
            _require_normalized(getattr(self, name), f"ActuationReceipt.{name}")
        require_reason_codes(
            self.hardware_faults,
            "ActuationReceipt.hardware_faults",
            required=self.write_status is not WriteStatus.APPLIED,
        )


__all__ = [
    "AcquisitionFrame",
    "ActuationReceipt",
    "ActuatorRequest",
    "AdmittedFrame",
    "CandidateEvaluation",
    "CommandMode",
    "CommandRequest",
    "ConstrainedMotion",
    "ConstraintCode",
    "DeviceHealth",
    "DeviceHealthState",
    "DeviceSample",
    "FinalActuation",
    "MissionIntent",
    "MissionLifecycle",
    "MotionIntent",
    "MotionObjective",
    "MotionObjectiveKind",
    "MotionSample",
    "NavigationPlan",
    "NavigationStatus",
    "Observation",
    "ObstacleTrack",
    "RawDeviceBatch",
    "RejectedObservation",
    "RejectionReason",
    "RobotEstimate",
    "SafetyDecision",
    "SourceTrust",
    "SourceWatermark",
    "TrustLevel",
    "Waypoint",
    "WheelVelocitySetpoint",
    "WorldSnapshot",
    "WriteStatus",
]
