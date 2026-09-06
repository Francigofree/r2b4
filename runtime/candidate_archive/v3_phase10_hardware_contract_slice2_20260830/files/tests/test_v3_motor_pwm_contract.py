from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.motor_pwm import (
    Drv8871PwmPlan,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
    plan_drv8871_pwm,
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


@pytest.mark.parametrize("invalid_output", (True, float("nan"), float("inf"), -1.01, 1.01))
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
