from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.motor_pwm import (
    Drv8871MotorFrame,
    Drv8871PwmPlan,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_drv8871_pwm,
    plan_final_actuation,
)
from v3.contracts import FinalActuation, SafetyDecision, TickContext


def _final_actuation(
    *,
    left_output: float,
    right_output: float,
    safety_decision: SafetyDecision,
) -> FinalActuation:
    return FinalActuation(
        context=TickContext(7, 1_000),
        left_output=left_output,
        right_output=right_output,
        enabled=safety_decision is SafetyDecision.ALLOW,
        safety_decision=safety_decision,
        latch_state="CLEAR" if safety_decision is SafetyDecision.ALLOW else "STOPPED",
        reason=None if safety_decision is SafetyDecision.ALLOW else "test-stop",
    )


@pytest.mark.parametrize(
    ("decay_mode", "output", "expected"),
    (
        (PwmDecayMode.COAST, 0.40, Drv8871PwmPlan(40.0, 0.0)),
        (PwmDecayMode.COAST, -0.25, Drv8871PwmPlan(0.0, 25.0)),
        (PwmDecayMode.BRAKE, 0.40, Drv8871PwmPlan(100.0, 60.0)),
        (PwmDecayMode.BRAKE, -0.25, Drv8871PwmPlan(75.0, 100.0)),
    ),
)
def test_native_drv8871_plan_closes_direction_and_decay_semantics(
    decay_mode: PwmDecayMode,
    output: float,
    expected: Drv8871PwmPlan,
):
    config = MotorChannelPhysicalConfig(12, 13, pwm_decay_mode=decay_mode)

    assert plan_drv8871_pwm(config, output) == expected


@pytest.mark.parametrize("decay_mode", tuple(PwmDecayMode))
def test_native_drv8871_plan_applies_channel_polarity_before_pin_mapping(
    decay_mode: PwmDecayMode,
):
    normal = MotorChannelPhysicalConfig(12, 13, pwm_decay_mode=decay_mode)
    inverted = MotorChannelPhysicalConfig(
        12,
        13,
        invert=True,
        pwm_decay_mode=decay_mode,
    )

    assert plan_drv8871_pwm(inverted, 0.35) == plan_drv8871_pwm(normal, -0.35)
    assert plan_drv8871_pwm(inverted, -0.35) == plan_drv8871_pwm(normal, 0.35)


@pytest.mark.parametrize("decay_mode", tuple(PwmDecayMode))
@pytest.mark.parametrize("invert", (False, True))
def test_native_drv8871_stop_is_always_both_pins_low(
    decay_mode: PwmDecayMode,
    invert: bool,
):
    config = MotorChannelPhysicalConfig(
        12,
        13,
        invert=invert,
        pwm_decay_mode=decay_mode,
    )

    assert plan_drv8871_pwm(config, 0.0) == Drv8871PwmPlan(0.0, 0.0)
    assert plan_drv8871_pwm(config, -0.0) == Drv8871PwmPlan(0.0, 0.0)


@pytest.mark.parametrize(
    "invalid_output",
    (True, float("nan"), float("inf"), -1.01, 1.01),
)
def test_native_drv8871_plan_rejects_invalid_output_fail_closed(invalid_output):
    config = MotorChannelPhysicalConfig(12, 13)

    with pytest.raises(ValueError, match="normalized_output"):
        plan_drv8871_pwm(config, invalid_output)


def test_native_drv8871_plan_and_config_are_immutable_typed_values():
    config = MotorChannelPhysicalConfig(12, 13)
    plan = plan_drv8871_pwm(config, 0.5)

    with pytest.raises(FrozenInstanceError):
        config.in1 = 18
    with pytest.raises(FrozenInstanceError):
        plan.in1_duty_cycle = 0.0
    with pytest.raises(TypeError, match="MotorChannelPhysicalConfig"):
        plan_drv8871_pwm(object(), 0.0)


def test_final_actuation_closes_into_one_context_bound_paired_motor_frame():
    left_config = MotorChannelPhysicalConfig(
        12,
        13,
        pwm_decay_mode=PwmDecayMode.COAST,
    )
    right_config = MotorChannelPhysicalConfig(
        18,
        19,
        invert=True,
        pwm_decay_mode=PwmDecayMode.BRAKE,
    )
    command = _final_actuation(
        left_output=0.40,
        right_output=0.25,
        safety_decision=SafetyDecision.ALLOW,
    )

    frame = plan_final_actuation(left_config, right_config, command)

    assert frame == Drv8871MotorFrame(
        context=command.context,
        left=Drv8871PwmPlan(40.0, 0.0),
        right=Drv8871PwmPlan(75.0, 100.0),
        safety_decision=SafetyDecision.ALLOW,
    )


@pytest.mark.parametrize("decision", (SafetyDecision.STOP, SafetyDecision.FAULT))
def test_stop_and_fault_close_into_one_paired_all_pin_zero_frame(decision):
    command = _final_actuation(
        left_output=0.0,
        right_output=0.0,
        safety_decision=decision,
    )

    frame = plan_final_actuation(
        MotorChannelPhysicalConfig(12, 13, invert=True),
        MotorChannelPhysicalConfig(
            18,
            19,
            invert=True,
            pwm_decay_mode=PwmDecayMode.BRAKE,
        ),
        command,
    )

    assert frame.context == command.context
    assert frame.safety_decision is decision
    assert frame.left == Drv8871PwmPlan(0.0, 0.0)
    assert frame.right == Drv8871PwmPlan(0.0, 0.0)


def test_paired_motor_frame_rejects_nonzero_stop_even_when_built_directly():
    with pytest.raises(ValueError, match="STOP/FAULT motor frame"):
        Drv8871MotorFrame(
            context=TickContext(7, 1_000),
            left=Drv8871PwmPlan(10.0, 0.0),
            right=Drv8871PwmPlan(0.0, 0.0),
            safety_decision=SafetyDecision.STOP,
        )


def test_paired_motor_planner_rejects_overlapping_physical_pins():
    command = _final_actuation(
        left_output=0.0,
        right_output=0.0,
        safety_decision=SafetyDecision.STOP,
    )

    with pytest.raises(ValueError, match="GPIO pins must be unique"):
        plan_final_actuation(
            MotorChannelPhysicalConfig(12, 13),
            MotorChannelPhysicalConfig(13, 19),
            command,
        )


def test_paired_motor_frame_is_an_immutable_typed_value():
    frame = plan_final_actuation(
        MotorChannelPhysicalConfig(12, 13),
        MotorChannelPhysicalConfig(18, 19),
        _final_actuation(
            left_output=0.0,
            right_output=0.0,
            safety_decision=SafetyDecision.STOP,
        ),
    )

    with pytest.raises(FrozenInstanceError):
        frame.left = Drv8871PwmPlan(1.0, 0.0)
    with pytest.raises(TypeError, match="FinalActuation"):
        plan_final_actuation(
            MotorChannelPhysicalConfig(12, 13),
            MotorChannelPhysicalConfig(18, 19),
            object(),
        )
