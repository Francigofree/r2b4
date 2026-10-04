from dataclasses import FrozenInstanceError
import pytest
from v3.adapters.motor_pwm import Drv8871MotorFrame, Drv8871PwmPlan, MotorChannelPhysicalConfig, PwmDecayMode, plan_drv8871_pwm, plan_final_actuation
from v3.contracts import FinalActuation, SafetyDecision, TickContext

def _final_actuation(*, left_output: float, right_output: float, safety_decision: SafetyDecision) -> FinalActuation:
    return FinalActuation(context=TickContext(7, 1000), left_output=left_output, right_output=right_output, enabled=safety_decision is SafetyDecision.ALLOW, safety_decision=safety_decision, latch_state='CLEAR' if safety_decision is SafetyDecision.ALLOW else 'STOPPED', reason=None if safety_decision is SafetyDecision.ALLOW else 'test-stop')

@pytest.mark.parametrize('decision', (SafetyDecision.STOP, SafetyDecision.FAULT))
def test_stop_and_fault_close_into_one_paired_all_pin_zero_frame(decision):
    command = _final_actuation(left_output=0.0, right_output=0.0, safety_decision=decision)
    frame = plan_final_actuation(MotorChannelPhysicalConfig(12, 13, invert=True), MotorChannelPhysicalConfig(18, 19, invert=True, pwm_decay_mode=PwmDecayMode.BRAKE), command)
    assert frame.context == command.context
    assert frame.safety_decision is decision
    assert frame.left == Drv8871PwmPlan(0.0, 0.0)
    assert frame.right == Drv8871PwmPlan(0.0, 0.0)
