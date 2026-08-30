"""Pure native V3 DRV8871 physical-output planning contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from v3.contracts import FinalActuation, SafetyDecision, TickContext


def _require_nonnegative_int(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")


class PwmDecayMode(str, Enum):
    """Native V3 DRV8871 PWM decay semantics."""

    COAST = "coast"
    BRAKE = "brake"


@dataclass(frozen=True, slots=True)
class MotorChannelPhysicalConfig:
    """Closed physical contract for one native V3 motor channel."""

    in1: int
    in2: int
    invert: bool = False
    pwm_decay_mode: PwmDecayMode = PwmDecayMode.COAST

    def __post_init__(self) -> None:
        _require_nonnegative_int(self.in1, "MotorChannelPhysicalConfig.in1")
        _require_nonnegative_int(self.in2, "MotorChannelPhysicalConfig.in2")
        if self.in1 == self.in2:
            raise ValueError("one motor channel requires two distinct pins")
        if type(self.invert) is not bool:
            raise ValueError("MotorChannelPhysicalConfig.invert must be bool")
        if not isinstance(self.pwm_decay_mode, PwmDecayMode):
            raise ValueError(
                "MotorChannelPhysicalConfig.pwm_decay_mode must be PwmDecayMode"
            )


@dataclass(frozen=True, slots=True)
class Drv8871PwmPlan:
    """I/O-free duty plan for the two DRV8871 input pins."""

    in1_duty_cycle: float
    in2_duty_cycle: float

    def __post_init__(self) -> None:
        for name, duty_cycle in (
            ("in1_duty_cycle", self.in1_duty_cycle),
            ("in2_duty_cycle", self.in2_duty_cycle),
        ):
            if (
                isinstance(duty_cycle, bool)
                or not isinstance(duty_cycle, (int, float))
                or not math.isfinite(duty_cycle)
                or not 0.0 <= duty_cycle <= 100.0
            ):
                raise ValueError(f"{name} must be finite and within [0, 100]")


@dataclass(frozen=True, slots=True)
class Drv8871MotorFrame:
    """One immutable paired physical plan for one L12 decision."""

    context: TickContext
    left: Drv8871PwmPlan
    right: Drv8871PwmPlan
    safety_decision: SafetyDecision

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("context must be TickContext")
        if not isinstance(self.left, Drv8871PwmPlan):
            raise TypeError("left must be Drv8871PwmPlan")
        if not isinstance(self.right, Drv8871PwmPlan):
            raise TypeError("right must be Drv8871PwmPlan")
        if not isinstance(self.safety_decision, SafetyDecision):
            raise TypeError("safety_decision must be SafetyDecision")
        zero = Drv8871PwmPlan(0.0, 0.0)
        if self.safety_decision is not SafetyDecision.ALLOW and (
            self.left != zero or self.right != zero
        ):
            raise ValueError("STOP/FAULT motor frame must be zero on every pin")


def plan_drv8871_pwm(
    config: MotorChannelPhysicalConfig,
    normalized_output: float,
) -> Drv8871PwmPlan:
    """Map one normalized wheel output to native DRV8871 pin duties.

    This function deliberately has no GPIO/backend capability.  It closes the
    physical polarity and coast/brake semantics before a later ACTIVE-ready
    writer is considered.
    """

    if not isinstance(config, MotorChannelPhysicalConfig):
        raise TypeError("config must be MotorChannelPhysicalConfig")
    if (
        isinstance(normalized_output, bool)
        or not isinstance(normalized_output, (int, float))
        or not math.isfinite(normalized_output)
        or not -1.0 <= normalized_output <= 1.0
    ):
        raise ValueError("normalized_output must be finite and within [-1, 1]")

    output = float(normalized_output)
    if config.invert:
        output = -output
    if output == 0.0:
        return Drv8871PwmPlan(0.0, 0.0)

    duty_cycle = abs(output) * 100.0
    if config.pwm_decay_mode is PwmDecayMode.COAST:
        if output > 0.0:
            return Drv8871PwmPlan(duty_cycle, 0.0)
        return Drv8871PwmPlan(0.0, duty_cycle)

    if output > 0.0:
        return Drv8871PwmPlan(100.0, 100.0 - duty_cycle)
    return Drv8871PwmPlan(100.0 - duty_cycle, 100.0)


def plan_final_actuation(
    left_config: MotorChannelPhysicalConfig,
    right_config: MotorChannelPhysicalConfig,
    command: FinalActuation,
) -> Drv8871MotorFrame:
    """Close one L12 decision into one paired, capability-free motor frame."""

    if not isinstance(left_config, MotorChannelPhysicalConfig):
        raise TypeError("left_config must be MotorChannelPhysicalConfig")
    if not isinstance(right_config, MotorChannelPhysicalConfig):
        raise TypeError("right_config must be MotorChannelPhysicalConfig")
    if not isinstance(command, FinalActuation):
        raise TypeError("command must be FinalActuation")
    pins = (
        left_config.in1,
        left_config.in2,
        right_config.in1,
        right_config.in2,
    )
    if len(set(pins)) != len(pins):
        raise ValueError("left and right motor GPIO pins must be unique")

    if command.safety_decision is not SafetyDecision.ALLOW:
        if (
            command.enabled
            or command.left_output != 0.0
            or command.right_output != 0.0
        ):
            raise ValueError("STOP/FAULT FinalActuation must be disabled and zero")
        left = Drv8871PwmPlan(0.0, 0.0)
        right = Drv8871PwmPlan(0.0, 0.0)
    else:
        if not command.enabled:
            raise ValueError("ALLOW FinalActuation must be enabled")
        left = plan_drv8871_pwm(left_config, command.left_output)
        right = plan_drv8871_pwm(right_config, command.right_output)

    return Drv8871MotorFrame(
        context=command.context,
        left=left,
        right=right,
        safety_decision=command.safety_decision,
    )


__all__ = [
    "Drv8871MotorFrame",
    "Drv8871PwmPlan",
    "MotorChannelPhysicalConfig",
    "PwmDecayMode",
    "plan_drv8871_pwm",
    "plan_final_actuation",
]
