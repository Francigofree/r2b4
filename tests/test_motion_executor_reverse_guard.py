import dataclasses
import json
from pathlib import Path

import pytest

from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    CycleContext,
    WheelFeedback,
    WheelVelocitySetpoint,
)
from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _speed_map() -> dict:
    return json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))


def _executor(**kwargs) -> MotionExecutor:
    return MotionExecutor(
        pid_config=kwargs.pop(
            "pid_config",
            PIDConfig(kp=0.25, ki=0.08, integrator_limit=0.18, wheel_feedback_trust_min=0.25),
        ),
        max_pwm=kwargs.pop("max_pwm", 0.95),
        speed_map=kwargs.pop("speed_map", _speed_map()),
        **kwargs,
    )


def _cycle(number: int, *, now: float | None = None, valid: bool = True) -> CycleContext:
    return CycleContext(
        cycle_id=str(number),
        monotonic_time=float(number * 0.02 if now is None else now),
        dt_observed_s=0.02,
        dt_control_s=0.02,
        timing_valid=valid,
    )


def _setpoint(number: int, left: float, right: float, *, feasible: bool = True) -> WheelVelocitySetpoint:
    return WheelVelocitySetpoint(
        contract_id=MOTION_PLATFORM_CONTRACT_ID,
        wheel_setpoint_id=f"wheel:{number}",
        physical_command_id=f"physical:{number}",
        resolved_id=f"resolved:{number}",
        cycle_id=str(number),
        left_target_mps=left,
        right_target_mps=right,
        feasible=feasible,
        reason="ACCEPTED" if feasible else "INFEASIBLE",
    )


def _feedback(*, left: float = 0.0, right: float = 0.0) -> WheelFeedback:
    return WheelFeedback(
        measurement_id="encoder:9",
        source_timestamp=1.0,
        left_mps=left,
        right_mps=right,
        combined_trust=1.0,
        timing_valid=True,
        stale=False,
        aggregation_window_s=0.1,
    )


def _compute(executor: MotionExecutor, number: int, left: float, right: float, *, now=None, feedback=None):
    return executor.compute(
        _cycle(number, now=now),
        _setpoint(number, left, right),
        feedback or _feedback(),
    )


def test_wheel_only_executor_applies_feedforward_and_one_pi_update_per_wheel():
    output = _compute(_executor(), 1, 0.20, 0.24)
    diag = dict(output.wheel_control_diagnostics)

    assert output.output_reason == "WHEEL_SPEED_LOOP"
    assert diag["speed_map_lookup_count"] == 2
    assert diag["wheel_pi_update_count"] == 2
    assert diag["left_reference_mps"] == pytest.approx(0.20)
    assert diag["right_reference_mps"] == pytest.approx(0.24)


def test_opposite_wheel_targets_select_independent_direction_curves():
    output = _compute(_executor(), 1, -0.15, 0.15)
    diag = dict(output.wheel_control_diagnostics)

    assert output.left_pwm < 0.0 < output.right_pwm
    assert diag["left_feedforward"]["curve"] == "left_reverse"
    assert diag["right_feedforward"]["curve"] == "right_forward"


def test_one_zero_wheel_stays_zero_without_semantic_special_case():
    output = _compute(_executor(), 1, 0.0, 0.15)

    assert output.left_pwm == 0.0
    assert output.right_pwm > 0.0
    assert output.wheel_control_diagnostics["left_output_reason"] == "inactive"


def test_direction_switch_requires_debounce_and_deadtime():
    executor = _executor(direction_switch_debounce_cycles=3, direction_switch_hold_s=0.08)
    assert _compute(executor, 1, 0.15, 0.15, now=1.00).output_reason == "WHEEL_SPEED_LOOP"
    assert _compute(executor, 2, -0.15, -0.15, now=1.02).output_reason == "DIRECTION_SWITCH_DEBOUNCE"
    assert _compute(executor, 3, -0.15, -0.15, now=1.04).output_reason == "DIRECTION_SWITCH_DEBOUNCE"
    assert _compute(executor, 4, -0.15, -0.15, now=1.06).output_reason == "DIRECTION_SWITCH_HOLD"
    assert _compute(executor, 5, -0.15, -0.15, now=1.10).output_reason == "DIRECTION_SWITCH_HOLD"
    assert _compute(executor, 6, -0.15, -0.15, now=1.15).output_reason == "WHEEL_SPEED_LOOP"


def test_same_direction_reference_change_never_activates_reverse_guard():
    executor = _executor()
    _compute(executor, 1, 0.15, 0.15)
    output = _compute(executor, 2, 0.24, 0.18)

    assert output.output_reason == "WHEEL_SPEED_LOOP"
    assert output.left_pwm > 0.0
    assert output.right_pwm > 0.0


def test_zero_target_is_immediate_and_resets_direction_state():
    executor = _executor()
    _compute(executor, 1, 0.15, 0.15)
    zero = _compute(executor, 2, 0.0, 0.0)
    reverse = _compute(executor, 3, -0.15, -0.15)

    assert (zero.left_pwm, zero.right_pwm) == (0.0, 0.0)
    assert zero.output_reason == "ZERO_TARGET"
    assert reverse.output_reason == "WHEEL_SPEED_LOOP"


def test_feedback_contract_failures_zero_before_speed_map_or_pi():
    executor = _executor()
    failures = (
        (dataclasses.replace(_feedback(), measurement_id="MISSING"), "WHEEL_MEASUREMENT_ID_MISSING"),
        (dataclasses.replace(_feedback(), stale=True), "WHEEL_FEEDBACK_STALE"),
        (dataclasses.replace(_feedback(), timing_valid=False), "WHEEL_FEEDBACK_TIMING_INVALID"),
        (dataclasses.replace(_feedback(), combined_trust=0.0), "WHEEL_FEEDBACK_UNTRUSTED"),
    )
    for number, (feedback, reason) in enumerate(failures, start=1):
        output = _compute(executor, number, 0.15, 0.15, feedback=feedback)
        assert (output.left_pwm, output.right_pwm) == (0.0, 0.0)
        assert output.output_reason == reason


def test_invalid_cycle_and_infeasible_setpoint_fail_closed():
    executor = _executor()
    invalid_cycle = executor.compute(_cycle(1, valid=False), _setpoint(1, 0.15, 0.15), _feedback())
    infeasible = executor.compute(_cycle(2), _setpoint(2, 0.15, 0.15, feasible=False), _feedback())

    assert invalid_cycle.output_reason == "CYCLE_TIMING_INVALID"
    assert infeasible.output_reason == "SETPOINT_INFEASIBLE"
    assert (invalid_cycle.left_pwm, invalid_cycle.right_pwm) == (0.0, 0.0)
    assert (infeasible.left_pwm, infeasible.right_pwm) == (0.0, 0.0)


def test_missing_or_inactive_speed_map_fails_closed():
    output = _compute(_executor(speed_map={}), 1, 0.15, 0.15)

    assert output.output_reason == "WHEEL_SPEED_MAP_UNAVAILABLE"
    assert (output.left_pwm, output.right_pwm) == (0.0, 0.0)


def test_output_is_clamped_by_immutable_runtime_pwm_cap():
    executor = _executor(
        pid_config=PIDConfig(kp=5.0, ki=0.0, integrator_limit=0.0),
        max_pwm=0.22,
    )
    output = _compute(executor, 1, 0.50, 0.50)

    assert abs(output.left_pwm) <= 0.22
    assert abs(output.right_pwm) <= 0.22


def test_executor_diagnostics_contain_no_heading_or_behavior_ownership():
    diag = _compute(_executor(), 1, 0.15, 0.15).wheel_control_diagnostics
    flattened = repr(dict(diag)).lower()

    for forbidden in ("heading", "planner", "behavior", "room_cruise", "command_type"):
        assert forbidden not in flattened
