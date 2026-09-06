#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Immutable V2.1 contracts at the L6-to-L7A guidance boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping, Optional, Sequence

from controller.motion_platform_contract import (
    CycleContext,
    DriveCapabilities,
)


MOTION_INTENT_CONTRACT_ID = "R2B4_MOTION_INTENT_V2_1"
GUIDANCE_NONE = "NONE"
GUIDANCE_HEADING_HOLD = "HEADING_HOLD"
GUIDANCE_TURN_TO_HEADING = "TURN_TO_HEADING"
GUIDANCE_TRACK_LOCAL_SEGMENT = "TRACK_LOCAL_SEGMENT"
GUIDANCE_TYPES = frozenset(
    {
        GUIDANCE_NONE,
        GUIDANCE_HEADING_HOLD,
        GUIDANCE_TURN_TO_HEADING,
        GUIDANCE_TRACK_LOCAL_SEGMENT,
    }
)


def _immutable_mapping(value: Mapping[str, Any] | None) -> Mapping[str, Any]:
    return MappingProxyType(dict(value or {}))


def _optional_float(value: Any) -> float | None:
    try:
        candidate = float(value)
    except (TypeError, ValueError):
        return None
    return candidate if math.isfinite(candidate) else None


@dataclass(frozen=True, slots=True)
class GuidanceRequest:
    """Typed, immutable upper-motion reference consumed only by L7A."""

    guidance_type: str
    request_id: str
    target_heading_deg: float | None = None
    settle_tolerance_deg: float | None = None
    settle_time_s: float | None = None
    max_duration_s: float | None = None
    speed_level: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "guidance_type", str(self.guidance_type).upper())
        object.__setattr__(self, "request_id", str(self.request_id))
        for field_name in (
            "target_heading_deg",
            "settle_tolerance_deg",
            "settle_time_s",
            "max_duration_s",
        ):
            object.__setattr__(
                self,
                field_name,
                _optional_float(getattr(self, field_name)),
            )
        if self.speed_level is not None:
            try:
                object.__setattr__(self, "speed_level", int(self.speed_level))
            except (TypeError, ValueError):
                object.__setattr__(self, "speed_level", None)

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        fallback_guidance_type: str,
    ) -> "GuidanceRequest":
        source = dict(value or {})
        return cls(
            guidance_type=str(source.get("guidance_type") or fallback_guidance_type),
            request_id=str(source.get("request_id") or ""),
            target_heading_deg=source.get("target_heading_deg"),
            settle_tolerance_deg=source.get("settle_tolerance_deg"),
            settle_time_s=source.get("settle_time_s"),
            max_duration_s=source.get("max_duration_s"),
            speed_level=source.get("speed_level"),
        )


@dataclass(frozen=True, slots=True)
class PoseSnapshot:
    frame_id: str
    pose_id: str
    source_timestamp: float
    x_m: float
    y_m: float
    yaw_rad: float
    v_mps: float
    omega_rad_s: float
    validity: str

    @property
    def valid(self) -> bool:
        return str(self.validity).upper() == "VALID"


@dataclass(frozen=True, slots=True)
class WorldModelSnapshot:
    """Narrow immutable L5/L7A world feedback contract."""

    world_id: str
    source_timestamp: float
    validity: str
    lidar_summary: Mapping[str, Any] = field(default_factory=dict)
    obstacle_status: Mapping[str, Any] = field(default_factory=dict)
    raw_scan: Sequence[Mapping[str, Any]] = field(default_factory=tuple)
    localization_status: Mapping[str, Any] = field(default_factory=dict)
    odometry_mode: str = "UNKNOWN"

    def __post_init__(self) -> None:
        object.__setattr__(self, "lidar_summary", _immutable_mapping(self.lidar_summary))
        object.__setattr__(self, "obstacle_status", _immutable_mapping(self.obstacle_status))
        object.__setattr__(
            self,
            "localization_status",
            _immutable_mapping(self.localization_status),
        )
        object.__setattr__(self, "odometry_mode", str(self.odometry_mode).upper())
        object.__setattr__(
            self,
            "raw_scan",
            tuple(_immutable_mapping(item) for item in self.raw_scan),
        )

    @property
    def valid(self) -> bool:
        return str(self.validity).upper() == "VALID"


@dataclass(frozen=True, slots=True)
class ResolvedMotionIntent:
    contract_id: str
    resolved_id: str
    cycle_id: str
    selected_proposal_id: str
    valid_until_monotonic: float
    nominal_mode: str
    v_mps: float = 0.0
    omega_rad_s: float = 0.0
    left_mps: float = 0.0
    right_mps: float = 0.0
    guidance_type: str = GUIDANCE_NONE
    guidance_request: GuidanceRequest | Mapping[str, Any] | None = None
    trace_metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))
        object.__setattr__(self, "nominal_mode", str(self.nominal_mode).upper())
        object.__setattr__(self, "guidance_type", str(self.guidance_type).upper())
        guidance_request = self.guidance_request
        if isinstance(guidance_request, Mapping):
            guidance_request = GuidanceRequest.from_mapping(
                guidance_request,
                fallback_guidance_type=self.guidance_type,
            )
        if guidance_request is not None and not isinstance(
            guidance_request,
            GuidanceRequest,
        ):
            raise TypeError("guidance_request_must_be_typed")
        object.__setattr__(self, "guidance_request", guidance_request)
        object.__setattr__(
            self,
            "trace_metadata",
            _immutable_mapping(self.trace_metadata),
        )


@dataclass(frozen=True, slots=True)
class MotionSemanticsInput:
    cycle_context: CycleContext
    pose: PoseSnapshot
    resolved_intent: ResolvedMotionIntent
    drive_capabilities: DriveCapabilities
    v_mps: float
    omega_rad_s: float
    requested_left_mps: Optional[float] = None
    requested_right_mps: Optional[float] = None
    executed_left_mps: Optional[float] = None
    executed_right_mps: Optional[float] = None
    actual_linear_mps: Optional[float] = None
    actual_angular_dps: Optional[float] = None


@dataclass(frozen=True, slots=True)
class MotionSemanticsResult:
    cycle_id: str
    v_mps: float
    omega_rad_s: float
    valid: bool
    reason: str
    status: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))
        object.__setattr__(self, "status", _immutable_mapping(self.status))


@dataclass(frozen=True, slots=True)
class MotionPolicyInput:
    cycle_context: CycleContext
    lidar_summary: Mapping[str, Any]
    obstacle_status: Mapping[str, Any]
    raw_scan: Sequence[Mapping[str, Any]]
    effective_v_max_mps: float
    v_mps: float
    omega_rad_s: float
    requested_motion_intent: Mapping[str, Any]
    resolved_motion: Mapping[str, Any]
    active_command_layer: str
    active_command_type: str
    motion_source: str
    execution_mode: str
    turn_primitive_requested: str
    robot_state: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "lidar_summary", _immutable_mapping(self.lidar_summary))
        object.__setattr__(self, "obstacle_status", _immutable_mapping(self.obstacle_status))
        object.__setattr__(
            self,
            "requested_motion_intent",
            _immutable_mapping(self.requested_motion_intent),
        )
        object.__setattr__(
            self,
            "resolved_motion",
            _immutable_mapping(self.resolved_motion),
        )
        object.__setattr__(
            self,
            "raw_scan",
            tuple(_immutable_mapping(item) for item in self.raw_scan),
        )


@dataclass(frozen=True, slots=True)
class MotionGuidanceInput:
    resolved_intent: ResolvedMotionIntent
    pose: PoseSnapshot
    world: WorldModelSnapshot
    cycle_context: CycleContext
    drive_capabilities: DriveCapabilities
    executed_left_mps: Optional[float] = None
    executed_right_mps: Optional[float] = None
    actual_linear_mps: Optional[float] = None
    actual_angular_dps: Optional[float] = None
    measured_left_mps: Optional[float] = None
    measured_right_mps: Optional[float] = None
    gyro_z_rad_s: Optional[float] = None


@dataclass(frozen=True, slots=True)
class MotionGuidanceResult:
    cycle_id: str
    resolved_id: str
    v_mps: float
    omega_rad_s: float
    valid: bool
    reason: str
    semantics_status: Mapping[str, Any] = field(default_factory=dict)
    obstacle_status: Mapping[str, Any] = field(default_factory=dict)
    policy_status: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))
        object.__setattr__(
            self,
            "semantics_status",
            _immutable_mapping(self.semantics_status),
        )
        object.__setattr__(
            self,
            "obstacle_status",
            _immutable_mapping(self.obstacle_status),
        )
        object.__setattr__(
            self,
            "policy_status",
            _immutable_mapping(self.policy_status),
        )
