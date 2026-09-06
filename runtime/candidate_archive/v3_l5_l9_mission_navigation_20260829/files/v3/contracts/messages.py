"""Simple immutable contracts for the V3 layer boundaries."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .base import (
    ContractValidationError,
    DataField,
    TickContext,
    require_data_fields,
    require_finite,
    require_nonnegative,
    require_token,
    require_unit_interval,
)


def _require_unique(values: tuple[object, ...], keys: tuple[object, ...], name: str) -> None:
    if len(values) != len(keys) or len(set(keys)) != len(keys):
        raise ContractValidationError(f"{name} must not contain duplicates")


def _require_normalized(value: float, name: str) -> None:
    require_finite(value, name)
    if not -1.0 <= float(value) <= 1.0:
        raise ContractValidationError(f"{name} must be in [-1, 1]")


def _require_optional_token(value: str | None, name: str) -> None:
    if value is not None:
        require_token(value, name)


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


class LifecycleState(str, Enum):
    BOOTING = "BOOTING"
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    DEGRADED = "DEGRADED"
    FAULT = "FAULT"
    SHUTDOWN = "SHUTDOWN"


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
    STOP = "STOP"
    FAULT = "FAULT"


@dataclass(frozen=True, slots=True)
class DeviceHealth:
    device_id: str
    state: DeviceHealthState
    reason: str | None = None

    def __post_init__(self) -> None:
        require_token(self.device_id, "DeviceHealth.device_id")
        _require_optional_token(self.reason, "DeviceHealth.reason")
        if self.state is not DeviceHealthState.OK and self.reason is None:
            raise ContractValidationError("non-OK DeviceHealth requires a reason")


@dataclass(frozen=True, slots=True)
class DeviceSample:
    device_id: str
    kind: str
    sequence: int
    captured_monotonic_ns: int
    values: tuple[DataField, ...]

    def __post_init__(self) -> None:
        require_token(self.device_id, "DeviceSample.device_id")
        require_token(self.kind, "DeviceSample.kind")
        require_nonnegative(self.sequence, "DeviceSample.sequence")
        require_nonnegative(self.captured_monotonic_ns, "DeviceSample.captured_monotonic_ns")
        require_data_fields(self.values, "DeviceSample.values")


@dataclass(frozen=True, slots=True)
class RawDeviceBatch:
    context: TickContext
    samples: tuple[DeviceSample, ...]
    device_health: tuple[DeviceHealth, ...]

    def __post_init__(self) -> None:
        sample_keys = tuple((item.device_id, item.kind, item.sequence) for item in self.samples)
        _require_unique(self.samples, sample_keys, "RawDeviceBatch.samples")
        health_keys = tuple(item.device_id for item in self.device_health)
        _require_unique(self.device_health, health_keys, "RawDeviceBatch.device_health")


@dataclass(frozen=True, slots=True)
class AcquisitionFrame:
    context: TickContext
    samples: tuple[DeviceSample, ...]
    io_health: tuple[DeviceHealth, ...]

    def __post_init__(self) -> None:
        sample_keys = tuple((item.device_id, item.kind, item.sequence) for item in self.samples)
        _require_unique(self.samples, sample_keys, "AcquisitionFrame.samples")
        health_keys = tuple(item.device_id for item in self.io_health)
        _require_unique(self.io_health, health_keys, "AcquisitionFrame.io_health")


@dataclass(frozen=True, slots=True)
class Observation:
    kind: str
    source_device_id: str
    source_sequence: int
    captured_monotonic_ns: int
    values: tuple[DataField, ...]

    def __post_init__(self) -> None:
        require_token(self.kind, "Observation.kind")
        require_token(self.source_device_id, "Observation.source_device_id")
        require_nonnegative(self.source_sequence, "Observation.source_sequence")
        require_nonnegative(self.captured_monotonic_ns, "Observation.captured_monotonic_ns")
        require_data_fields(self.values, "Observation.values")


@dataclass(frozen=True, slots=True)
class RejectedObservation:
    source_device_id: str
    source_sequence: int
    reason: RejectionReason
    age_ns: int

    def __post_init__(self) -> None:
        require_token(self.source_device_id, "RejectedObservation.source_device_id")
        require_nonnegative(self.source_sequence, "RejectedObservation.source_sequence")
        require_nonnegative(self.age_ns, "RejectedObservation.age_ns")


@dataclass(frozen=True, slots=True)
class AdmittedFrame:
    context: TickContext
    accepted: tuple[Observation, ...]
    rejected: tuple[RejectedObservation, ...]
    degraded_sources: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        accepted_keys = tuple((item.source_device_id, item.kind, item.source_sequence) for item in self.accepted)
        _require_unique(self.accepted, accepted_keys, "AdmittedFrame.accepted")
        rejected_keys = tuple((item.source_device_id, item.source_sequence) for item in self.rejected)
        _require_unique(self.rejected, rejected_keys, "AdmittedFrame.rejected")
        if len(set(self.degraded_sources)) != len(self.degraded_sources):
            raise ContractValidationError("AdmittedFrame.degraded_sources must be unique")
        for source in self.degraded_sources:
            require_token(source, "AdmittedFrame.degraded_sources")


@dataclass(frozen=True, slots=True)
class RobotEstimate:
    context: TickContext
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    v_mps: float
    omega_rad_s: float
    covariance_5x5: tuple[float, ...]

    def __post_init__(self) -> None:
        require_token(self.frame_id, "RobotEstimate.frame_id")
        for name in ("x_m", "y_m", "yaw_rad", "v_mps", "omega_rad_s"):
            require_finite(getattr(self, name), f"RobotEstimate.{name}")
        if len(self.covariance_5x5) != 25:
            raise ContractValidationError("RobotEstimate.covariance_5x5 must contain 25 values")
        for index, value in enumerate(self.covariance_5x5):
            require_finite(value, f"RobotEstimate.covariance_5x5[{index}]")
        if any(self.covariance_5x5[index * 5 + index] < 0.0 for index in range(5)):
            raise ContractValidationError("RobotEstimate covariance diagonal cannot be negative")


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
    context: TickContext
    frame_id: str
    map_revision: int
    obstacle_tracks: tuple[ObstacleTrack, ...]
    freshness_ns: int

    def __post_init__(self) -> None:
        require_token(self.frame_id, "WorldSnapshot.frame_id")
        require_nonnegative(self.map_revision, "WorldSnapshot.map_revision")
        track_keys = tuple(item.track_id for item in self.obstacle_tracks)
        _require_unique(self.obstacle_tracks, track_keys, "WorldSnapshot.obstacle_tracks")
        require_nonnegative(self.freshness_ns, "WorldSnapshot.freshness_ns")


@dataclass(frozen=True, slots=True)
class CommandRequest:
    context: TickContext
    command_id: str
    mode: CommandMode
    goal: tuple[DataField, ...]
    expiry_tick: int

    def __post_init__(self) -> None:
        require_token(self.command_id, "CommandRequest.command_id")
        require_data_fields(self.goal, "CommandRequest.goal")
        require_nonnegative(self.expiry_tick, "CommandRequest.expiry_tick")
        if self.expiry_tick < self.context.tick_id:
            raise ContractValidationError("CommandRequest is already expired")


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
class VelocityTarget:
    v_mps: float
    omega_rad_s: float

    def __post_init__(self) -> None:
        require_finite(self.v_mps, "VelocityTarget.v_mps")
        require_finite(self.omega_rad_s, "VelocityTarget.omega_rad_s")


@dataclass(frozen=True, slots=True)
class MissionConstraints:
    max_v_mps: float
    max_omega_rad_s: float
    corridor_radius_m: float
    goal_tolerance_m: float
    yaw_tolerance_rad: float

    def __post_init__(self) -> None:
        for name in (
            "max_v_mps",
            "max_omega_rad_s",
            "corridor_radius_m",
            "goal_tolerance_m",
            "yaw_tolerance_rad",
        ):
            value = getattr(self, name)
            require_finite(value, f"MissionConstraints.{name}")
            if value < 0.0:
                raise ContractValidationError(f"MissionConstraints.{name} cannot be negative")
        if self.max_v_mps == 0.0 or self.max_omega_rad_s == 0.0:
            raise ContractValidationError("MissionConstraints motion limits must be positive")


@dataclass(frozen=True, slots=True)
class MissionIntent:
    context: TickContext
    mission_id: str
    mode: CommandMode
    target_pose: Waypoint | None
    velocity_target: VelocityTarget | None
    constraints: MissionConstraints
    lifecycle: MissionLifecycle
    stop_reason: str | None = None

    def __post_init__(self) -> None:
        require_token(self.mission_id, "MissionIntent.mission_id")
        _require_optional_token(self.stop_reason, "MissionIntent.stop_reason")
        if self.lifecycle is MissionLifecycle.ACTIVE:
            if self.mode is CommandMode.NAVIGATE and (
                self.target_pose is None or self.velocity_target is not None
            ):
                raise ContractValidationError("active NAVIGATE mission requires only target_pose")
            if self.mode is CommandMode.TELEOP and (
                self.velocity_target is None or self.target_pose is not None
            ):
                raise ContractValidationError("active TELEOP mission requires only velocity_target")
            if self.mode in (CommandMode.STOP, CommandMode.SERVICE):
                raise ContractValidationError("STOP/SERVICE mission cannot be active")
        if self.mode is CommandMode.STOP and self.stop_reason is None:
            raise ContractValidationError("STOP MissionIntent requires a reason")
        if self.stop_reason is not None and (
            self.target_pose is not None or self.velocity_target is not None
        ):
            raise ContractValidationError("stopped MissionIntent cannot carry a motion target")


@dataclass(frozen=True, slots=True)
class NavigationPlan:
    context: TickContext
    mission_id: str
    route: tuple[Waypoint, ...]
    velocity_target: VelocityTarget | None
    constraints: MissionConstraints
    corridor_radius_m: float
    progress: float
    status: NavigationStatus
    reason: str | None = None

    def __post_init__(self) -> None:
        require_token(self.mission_id, "NavigationPlan.mission_id")
        require_finite(self.corridor_radius_m, "NavigationPlan.corridor_radius_m")
        if self.corridor_radius_m < 0.0:
            raise ContractValidationError("NavigationPlan corridor radius cannot be negative")
        require_unit_interval(self.progress, "NavigationPlan.progress")
        _require_optional_token(self.reason, "NavigationPlan.reason")
        if self.status is NavigationStatus.ACTIVE:
            if bool(self.route) == (self.velocity_target is not None):
                raise ContractValidationError(
                    "active NavigationPlan requires exactly one route or velocity target"
                )
        elif self.route or self.velocity_target is not None:
            raise ContractValidationError("inactive NavigationPlan cannot carry an active target")
        if self.status in (NavigationStatus.NO_PATH, NavigationStatus.INVALIDATED) and self.reason is None:
            raise ContractValidationError("failed NavigationPlan requires a reason")


@dataclass(frozen=True, slots=True)
class MotionObjective:
    context: TickContext
    selected_source: str
    kind: MotionObjectiveKind
    priority: int
    expiry_tick: int
    selection_reason: str
    target_waypoint: Waypoint | None
    velocity_target: VelocityTarget | None
    constraints: MissionConstraints

    def __post_init__(self) -> None:
        require_token(self.selected_source, "MotionObjective.selected_source")
        if not isinstance(self.priority, int) or isinstance(self.priority, bool):
            raise ContractValidationError("MotionObjective.priority must be an integer")
        require_nonnegative(self.expiry_tick, "MotionObjective.expiry_tick")
        require_token(self.selection_reason, "MotionObjective.selection_reason")
        if self.kind is MotionObjectiveKind.STOP:
            if self.target_waypoint is not None or self.velocity_target is not None:
                raise ContractValidationError("STOP MotionObjective cannot carry a target")
        elif self.kind is MotionObjectiveKind.TRACK_PLAN:
            if self.target_waypoint is None or self.velocity_target is not None:
                raise ContractValidationError("TRACK_PLAN requires only a target waypoint")
        elif self.target_waypoint is not None or self.velocity_target is None:
            raise ContractValidationError("VELOCITY requires only a velocity target")


@dataclass(frozen=True, slots=True)
class MotionIntent:
    context: TickContext
    requested_v_mps: float
    requested_omega_rad_s: float
    horizon_ns: int
    constraints: MissionConstraints
    stop_reason: str | None = None

    def __post_init__(self) -> None:
        require_finite(self.requested_v_mps, "MotionIntent.requested_v_mps")
        require_finite(self.requested_omega_rad_s, "MotionIntent.requested_omega_rad_s")
        require_nonnegative(self.horizon_ns, "MotionIntent.horizon_ns")
        _require_optional_token(self.stop_reason, "MotionIntent.stop_reason")
        if self.stop_reason is not None and (
            self.requested_v_mps != 0.0 or self.requested_omega_rad_s != 0.0
        ):
            raise ContractValidationError("stopped MotionIntent must request zero motion")


@dataclass(frozen=True, slots=True)
class ConstrainedMotion:
    context: TickContext
    requested_v_mps: float
    requested_omega_rad_s: float
    allowed_v_mps: float
    allowed_omega_rad_s: float
    active_constraints: tuple[ConstraintCode, ...]

    def __post_init__(self) -> None:
        for name in (
            "requested_v_mps",
            "requested_omega_rad_s",
            "allowed_v_mps",
            "allowed_omega_rad_s",
        ):
            require_finite(getattr(self, name), f"ConstrainedMotion.{name}")
        if len(set(self.active_constraints)) != len(self.active_constraints):
            raise ContractValidationError("active_constraints must be unique")
        for requested, allowed, name in (
            (self.requested_v_mps, self.allowed_v_mps, "linear"),
            (self.requested_omega_rad_s, self.allowed_omega_rad_s, "angular"),
        ):
            if requested == 0.0 and allowed != 0.0:
                raise ContractValidationError(f"{name} constraint cannot create motion")
            if requested != 0.0 and (
                requested * allowed < 0.0 or abs(allowed) > abs(requested)
            ):
                raise ContractValidationError(f"{name} constraint cannot reverse or amplify motion")


@dataclass(frozen=True, slots=True)
class WheelVelocitySetpoint:
    context: TickContext
    left_mps: float
    right_mps: float

    def __post_init__(self) -> None:
        require_finite(self.left_mps, "WheelVelocitySetpoint.left_mps")
        require_finite(self.right_mps, "WheelVelocitySetpoint.right_mps")


@dataclass(frozen=True, slots=True)
class ActuatorRequest:
    context: TickContext
    left_normalized: float
    right_normalized: float
    saturated: bool = False

    def __post_init__(self) -> None:
        _require_normalized(self.left_normalized, "ActuatorRequest.left_normalized")
        _require_normalized(self.right_normalized, "ActuatorRequest.right_normalized")


@dataclass(frozen=True, slots=True)
class FinalActuation:
    context: TickContext
    left_output: float
    right_output: float
    enabled: bool
    safety_decision: SafetyDecision
    latch_state: str
    reason: str | None = None

    def __post_init__(self) -> None:
        _require_normalized(self.left_output, "FinalActuation.left_output")
        _require_normalized(self.right_output, "FinalActuation.right_output")
        require_token(self.latch_state, "FinalActuation.latch_state")
        _require_optional_token(self.reason, "FinalActuation.reason")
        if self.enabled != (self.safety_decision is SafetyDecision.ALLOW):
            raise ContractValidationError("enabled must match the ALLOW safety decision")
        if not self.enabled and (self.left_output != 0.0 or self.right_output != 0.0):
            raise ContractValidationError("disabled FinalActuation must command zero output")
        if not self.enabled and self.reason is None:
            raise ContractValidationError("stopped FinalActuation requires a reason")


__all__ = [
    "AcquisitionFrame",
    "ActuatorRequest",
    "AdmittedFrame",
    "CommandMode",
    "CommandRequest",
    "ConstrainedMotion",
    "ConstraintCode",
    "DeviceHealth",
    "DeviceHealthState",
    "DeviceSample",
    "FinalActuation",
    "LifecycleState",
    "MissionIntent",
    "MissionConstraints",
    "MissionLifecycle",
    "MotionIntent",
    "MotionObjective",
    "MotionObjectiveKind",
    "NavigationPlan",
    "NavigationStatus",
    "Observation",
    "ObstacleTrack",
    "RawDeviceBatch",
    "RejectedObservation",
    "RejectionReason",
    "RobotEstimate",
    "SafetyDecision",
    "Waypoint",
    "VelocityTarget",
    "WheelVelocitySetpoint",
    "WorldSnapshot",
]
