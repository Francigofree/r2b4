"""Canonical motion preserves its envelope, curvature and finite transitions."""
from dataclasses import replace
import math

from rig import resolved_config, healthy_localization
from v3.contracts import (
    ConstrainedMotion, ConstraintCode, LocalizationRequirement, MissionConstraints,
    MotionIntent, RobotEstimate, TickContext,
)
from v3.layers.l10_chassis_control import DifferentialDriveKinematics
from v3.layers.l9_operational_constraints import OperationalConstraintLayer


def test_kinematics_preserves_the_authorized_twist_including_live_regressions():
    config = resolved_config().runtime.composition.live_control.control
    kinematics = DifferentialDriveKinematics(config.chassis_control)
    # Tick 88 lost its curvature; 590--592 amplified tiny alternating turns to
    # full opposite wheel commands. L10 may not change any authorized twist.
    for v, omega in ((.119999, -.014936), (0, .00342), (0, -.01491),
                     (0, .00739), (0, 0), (.075, -.15/.3557), (.15, .2)):
        context = TickContext(1, 1_000_000_000)
        motion = ConstrainedMotion(context, v, omega, v, omega, ())
        wheels = kinematics(motion)
        assert math.isclose((wheels.left_mps + wheels.right_mps)/2, v, abs_tol=1e-12)
        assert math.isclose((wheels.right_mps - wheels.left_mps)/config.chassis_control.track_width_m,
                            omega, abs_tol=1e-12)


def test_motion_chain_keeps_wheel_acceleration_bounded_through_arcs_and_reversal():
    config = resolved_config().runtime.composition.live_control.control
    limiter = OperationalConstraintLayer(config.operational_constraints)
    kinematics = DifferentialDriveKinematics(config.chassis_control)
    limits = MissionConstraints(.3, 1.2, .08, .1, .3)
    previous = None
    for tick in range(240):
        context = TickContext(tick, 1_000_000_000 + tick*20_000_000)
        estimate = RobotEstimate(context, 'odom', 0, 0, 0, 0, 0, (0.,)*25,
                                 localization_quality=healthy_localization())
        v, omega = ((.25, .4) if tick < 80 else (-.25, -.4) if tick < 160 else (0., 0.))
        motion = MotionIntent(context, v, omega, 100_000_000, limits,
                              transition_allowed=True,
                              localization_requirement=LocalizationRequirement(True, True, False))
        allowed = limiter.evaluate(motion, estimate)
        wheels = kinematics(allowed)
        if previous is not None:
            for side in ('left_mps', 'right_mps'):
                assert abs(getattr(wheels, side)-getattr(previous, side)) <= .6*.02 + 1e-10
        assert abs(allowed.allowed_v_mps) <= limits.max_v_mps
        assert abs(allowed.allowed_omega_rad_s) <= limits.max_omega_rad_s
        if tick in (79, 159):
            assert min(abs(wheels.left_mps), abs(wheels.right_mps)) >= .15-1e-12
        previous = wheels
    assert wheels.left_mps == wheels.right_mps == 0


def test_unrealizable_steady_motion_is_not_amplified_and_stop_is_exact():
    config = resolved_config().runtime.composition.live_control.control
    kinematics = DifferentialDriveKinematics(config.chassis_control)
    for v, omega in ((.09, 0), (0, .2), (0, .6)):
        limiter = OperationalConstraintLayer(config.operational_constraints)
        for tick in range(20):
            context = TickContext(tick, 1_000_000_000 + tick*20_000_000)
            estimate = RobotEstimate(context, 'odom', 0, 0, 0, 0, 0, (0.,)*25,
                                     localization_quality=healthy_localization())
            motion = MotionIntent(context, v, omega, 100_000_000,
                                  MissionConstraints(.3, .6, .08, .1, .3),
                                  localization_requirement=LocalizationRequirement(True, True, False))
            wheels = kinematics(limiter.evaluate(motion, estimate))
            assert wheels.left_mps == wheels.right_mps == 0
        stop = replace(motion, requested_v_mps=0., requested_omega_rad_s=0., stop_reason='OPERATOR_STOP')
        assert kinematics(limiter.evaluate(stop, estimate)).left_mps == 0
