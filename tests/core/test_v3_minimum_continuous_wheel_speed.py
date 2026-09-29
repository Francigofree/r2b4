from __future__ import annotations

import math

from v3.contracts import ConstrainedMotion, ConstraintCode, TickContext
from v3.layers.l10_chassis_control import (
    ChassisControlConfig,
    DifferentialDriveKinematics,
)


TRACK_WIDTH_M = 0.3557
MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS = 0.15


def _motion(
    v_mps: float,
    omega_rad_s: float,
    *,
    constraints: tuple[ConstraintCode, ...] = (),
) -> ConstrainedMotion:
    context = TickContext(1, 1_000_000_000)
    return ConstrainedMotion(
        context=context,
        requested_v_mps=v_mps,
        requested_omega_rad_s=omega_rad_s,
        allowed_v_mps=v_mps,
        allowed_omega_rad_s=omega_rad_s,
        active_constraints=constraints,
    )


def _kinematics() -> DifferentialDriveKinematics:
    return DifferentialDriveKinematics(
        ChassisControlConfig(
            track_width_m=TRACK_WIDTH_M,
            minimum_continuous_wheel_speed_mps=MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS,
        )
    )


def test_stable_nonzero_wheel_speed_is_floored() -> None:
    wheels = _kinematics()(_motion(0.09, 0.0))
    assert wheels.left_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS
    assert wheels.right_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS


def test_acceleration_transition_may_pass_below_floor() -> None:
    wheels = _kinematics()(
        _motion(
            0.12,
            0.0,
            constraints=(ConstraintCode.ACCELERATION_LIMIT,),
        )
    )
    assert wheels.left_mps == 0.12
    assert wheels.right_mps == 0.12


def test_stop_remains_exact_zero() -> None:
    wheels = _kinematics()(_motion(0.0, 0.0))
    assert wheels.left_mps == 0.0
    assert wheels.right_mps == 0.0


def test_deliberately_stationary_wheel_remains_zero() -> None:
    # left=0.15, right=0.0 exactly in differential-drive kinematics.
    wheels = _kinematics()(
        _motion(
            MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS / 2.0,
            -MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS / TRACK_WIDTH_M,
        )
    )
    assert math.isclose(
        wheels.left_mps,
        MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS,
        rel_tol=0.0,
        abs_tol=1e-12,
    )
    assert wheels.right_mps == 0.0


def test_stable_pure_rotation_uses_wheel_floor() -> None:
    wheels = _kinematics()(_motion(0.0, 0.2))
    assert wheels.left_mps == -MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS
    assert wheels.right_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS


def test_only_subfloor_inner_wheel_is_raised() -> None:
    wheels = _kinematics()(_motion(0.15, 0.2))
    raw_outer = 0.15 + 0.2 * TRACK_WIDTH_M / 2.0
    assert wheels.left_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS
    assert math.isclose(wheels.right_mps, raw_outer, rel_tol=0.0, abs_tol=1e-12)
