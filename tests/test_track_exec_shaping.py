import pytest

from controller.motion_controller import MotionController, MotionControllerConfig
from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    PHYSICAL_MODE_STOP,
    PHYSICAL_MODE_WHEEL_VELOCITY,
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    PhysicalMotionCommand,
)


def _cycle(number: int, *, dt: float = 0.1) -> CycleContext:
    return CycleContext(str(number), number * dt, dt, dt, True)


def _command(number: int, left: float, right: float, *, stop: bool = False) -> PhysicalMotionCommand:
    return PhysicalMotionCommand(
        contract_id=MOTION_PLATFORM_CONTRACT_ID,
        physical_command_id=f"physical:{number}",
        resolved_id=f"resolved:{number}",
        cycle_id=str(number),
        valid_until_monotonic=100.0,
        physical_mode=PHYSICAL_MODE_STOP if stop else PHYSICAL_MODE_WHEEL_VELOCITY,
        left_mps=left,
        right_mps=right,
    )


def _envelope(command: PhysicalMotionCommand, *, stop: bool = False) -> MotionEnvelope:
    return MotionEnvelope(
        cycle_id=command.cycle_id,
        physical_command_id=command.physical_command_id,
        stop_required=stop,
        stop_reason="STOP" if stop else "",
        max_abs_v_mps=0.5,
        max_abs_omega_rad_s=2.0,
        max_abs_wheel_mps=0.582,
        max_wheel_accel_mps2=0.6,
        max_wheel_decel_mps2=0.8,
        capability_version="test",
    )


CAPABILITIES = DriveCapabilities(
    track_width_m=0.185,
    calibrated_wheel_min_mps=0.0,
    calibrated_wheel_max_mps=0.582,
    max_wheel_accel_mps2=0.6,
    max_wheel_decel_mps2=0.8,
    capability_version="test",
)


def _compute(controller, number, left, right, *, stop=False):
    command = _command(number, left, right, stop=stop)
    return controller.compute(_cycle(number), command, _envelope(command, stop=stop), CAPABILITIES)


def test_wheel_reference_uses_one_common_physical_slew_state():
    controller = MotionController(config=MotionControllerConfig(enable_slew=True))
    output = _compute(controller, 1, 0.30, 0.30)

    assert output.left_target_mps == pytest.approx(0.06)
    assert output.right_target_mps == pytest.approx(0.06)
    assert "wheel_rate_limit" in output.applied_limits


def test_transition_to_pivot_is_bounded_and_preserves_pair_coherence():
    controller = MotionController(config=MotionControllerConfig(enable_slew=True))
    _compute(controller, 1, 0.30, 0.30)
    pivot = _compute(controller, 2, -0.15, 0.15)

    assert max(abs(pivot.left_target_mps - 0.06), abs(pivot.right_target_mps - 0.06)) <= 0.0800001
    assert "wheel_rate_limit" in pivot.applied_limits


def test_stop_is_immediate_and_resets_slew_state():
    controller = MotionController(config=MotionControllerConfig(enable_slew=True))
    _compute(controller, 1, 0.30, 0.30)
    stop = _compute(controller, 2, 0.0, 0.0, stop=True)
    restarted = _compute(controller, 3, 0.30, 0.30)

    assert (stop.left_target_mps, stop.right_target_mps) == (0.0, 0.0)
    assert restarted.left_target_mps == pytest.approx(0.06)
    assert restarted.right_target_mps == pytest.approx(0.06)


def test_common_scale_preserves_wheel_ratio_at_maximum():
    controller = MotionController(config=MotionControllerConfig(enable_slew=False))
    output = _compute(controller, 1, 0.40, 0.80)

    assert output.right_target_mps == pytest.approx(0.582)
    assert output.left_target_mps == pytest.approx(0.291)
    assert output.right_target_mps / output.left_target_mps == pytest.approx(2.0)


def test_envelope_stop_overrides_nonzero_wheel_request():
    controller = MotionController(config=MotionControllerConfig(enable_slew=False))
    command = _command(1, -0.15, 0.15)
    output = controller.compute(_cycle(1), command, _envelope(command, stop=True), CAPABILITIES)

    assert output.feasible is True
    assert output.reason == "STOP"
    assert (output.left_target_mps, output.right_target_mps) == (0.0, 0.0)
