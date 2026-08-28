#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Immutable public contracts for the V2.1 lower motion platform."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping


MOTION_PLATFORM_CONTRACT_ID = "R2B4_MOTION_PLATFORM_V2_1"
PHYSICAL_MODE_BODY_TWIST = "BODY_TWIST"
PHYSICAL_MODE_WHEEL_VELOCITY = "WHEEL_VELOCITY"
PHYSICAL_MODE_STOP = "STOP"
PHYSICAL_MODES = frozenset(
    {
        PHYSICAL_MODE_BODY_TWIST,
        PHYSICAL_MODE_WHEEL_VELOCITY,
        PHYSICAL_MODE_STOP,
    }
)


def _immutable_mapping(value: Mapping[str, Any] | None) -> Mapping[str, Any]:
    return MappingProxyType(dict(value or {}))


@dataclass(frozen=True, slots=True)
class CycleContext:
    cycle_id: str
    monotonic_time: float
    dt_observed_s: float
    dt_control_s: float
    timing_valid: bool
    timing_reason: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))


@dataclass(frozen=True, slots=True)
class PhysicalMotionCommand:
    contract_id: str
    physical_command_id: str
    resolved_id: str
    cycle_id: str
    valid_until_monotonic: float
    physical_mode: str
    v_mps: float = 0.0
    omega_rad_s: float = 0.0
    left_mps: float = 0.0
    right_mps: float = 0.0
    guidance_reason: str = ""
    trace_metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))
        object.__setattr__(self, "physical_mode", str(self.physical_mode).upper())
        object.__setattr__(
            self,
            "trace_metadata",
            _immutable_mapping(self.trace_metadata),
        )


@dataclass(frozen=True, slots=True)
class MotionEnvelope:
    cycle_id: str
    physical_command_id: str
    stop_required: bool
    stop_reason: str
    max_abs_v_mps: float
    max_abs_omega_rad_s: float
    max_abs_wheel_mps: float
    max_wheel_accel_mps2: float
    max_wheel_decel_mps2: float
    capability_version: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))


@dataclass(frozen=True, slots=True)
class DriveCapabilities:
    track_width_m: float
    calibrated_wheel_min_mps: float
    calibrated_wheel_max_mps: float
    max_wheel_accel_mps2: float
    max_wheel_decel_mps2: float
    capability_version: str


@dataclass(frozen=True, slots=True)
class WheelVelocitySetpoint:
    contract_id: str
    wheel_setpoint_id: str
    physical_command_id: str
    resolved_id: str
    cycle_id: str
    left_target_mps: float
    right_target_mps: float
    feasible: bool
    reason: str
    applied_limits: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))
        object.__setattr__(self, "applied_limits", tuple(self.applied_limits))


@dataclass(frozen=True, slots=True)
class WheelFeedback:
    measurement_id: str
    source_timestamp: float
    left_mps: float
    right_mps: float
    combined_trust: float
    timing_valid: bool
    stale: bool
    timing_reason: str = ""
    aggregation_window_s: float = 0.0


@dataclass(frozen=True, slots=True)
class CandidateMotorOutput:
    contract_id: str
    candidate_output_id: str
    wheel_setpoint_id: str
    physical_command_id: str
    resolved_id: str
    cycle_id: str
    left_pwm: float
    right_pwm: float
    output_reason: str
    wheel_control_diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))
        object.__setattr__(
            self,
            "wheel_control_diagnostics",
            _immutable_mapping(self.wheel_control_diagnostics),
        )


@dataclass(frozen=True, slots=True)
class MotionPlatformStatus:
    cycle_id: str
    resolved_id: str
    physical_command_id: str
    accepted_physical_mode: str
    requested_left_mps: float
    requested_right_mps: float
    executed_left_mps: float
    executed_right_mps: float
    candidate_left_pwm: float
    candidate_right_pwm: float
    final_left_pwm: float
    final_right_pwm: float
    controller_reason: str
    executor_reason: str
    safety_reason: str
    measurement_validity: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "cycle_id", str(self.cycle_id))


@dataclass(frozen=True, slots=True)
class ServiceActuationRequest:
    armed_token: str
    expiry_monotonic: float
    left_pwm: float
    right_pwm: float
    max_abs_pwm: float
    distance_bound_m: float
    time_bound_s: float
    reason: str


@dataclass(frozen=True, slots=True)
class ServiceActuationOutput:
    left_pwm: float
    right_pwm: float
    accepted: bool
    reason: str
