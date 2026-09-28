"""Regression coverage for rate-only IMU local heading authority."""
from dataclasses import replace

from rig import resolved_config
from v3.contracts import (
    AdmittedFrame,
    CommandMode,
    CommandRequest,
    DataField,
    LOCAL_FRAME_ID,
    NavigationStatus,
    Observation,
    QualityState,
    RollingLocalCostmap,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l3_state_estimation import NativeStateEstimator
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer


def _observation(kind, tick, ns, **values):
    source = (
        "IMU" if kind == "ekf_heading"
        else "ENCODER" if kind == "wheel_velocity"
        else "RPLIDAR_C1"
    )
    return Observation(
        kind,
        source,
        tick,
        ns,
        tuple(DataField(key, value) for key, value in values.items()),
    )


def _frame(tick, *, omega_confidence):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    ns = context.monotonic_ns
    return AdmittedFrame(
        context,
        (
            _observation(
                "wheel_velocity",
                tick,
                ns,
                left_mps=0.0,
                right_mps=0.0,
                trust=1.0,
            ),
            _observation(
                "ekf_heading",
                tick,
                ns,
                yaw_rad=0.0,
                omega_rad_s=0.0,
                confidence=0.0,
                calibration=0,
                omega_confidence=omega_confidence,
                omega_calibration=3,
            ),
            _observation("lidar_health", tick, ns, age_ns=0, point_count=1),
        ),
        (),
    )


def _evaluate(omega_confidence):
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    for tick in range(5):
        frame = _frame(tick, omega_confidence=omega_confidence)
        estimate = estimator(frame)

    costmap = RollingLocalCostmap(
        LOCAL_FRAME_ID,
        frame.context.tick_id,
        0.05,
        2.5,
        (),
        frame.context.tick_id,
        0,
    )
    world = WorldSnapshot(
        frame.context,
        LOCAL_FRAME_ID,
        frame.context.tick_id,
        (),
        0,
        costmap,
    )
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(
        config.navigation,
        async_config=replace(
            config.async_l6,
            enabled=False,
            completion_inputs=False,
        ),
    )
    selector = MotionSelector(config.motion_selection)
    realizer = MotionRealizer(config.motion_realization)
    mission = manager.evaluate(
        CommandRequest(
            frame.context,
            "rate-only-explore",
            CommandMode.EXPLORE,
            (),
            frame.context.tick_id,
        )
    )
    plan = navigator.evaluate(mission, estimate, world)
    motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
    return estimate, plan, motion


def test_rate_only_gyro_preserves_local_heading_and_explore_motion():
    estimate, plan, motion = _evaluate(1.0)

    assert estimate.localization_quality.local_translation is QualityState.GOOD
    assert estimate.localization_quality.heading is not QualityState.LOST
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.reason != "LOCALIZATION_HOLD"
    assert abs(motion.requested_v_mps) + abs(motion.requested_omega_rad_s) > 0.0


def test_missing_absolute_and_rate_heading_authority_still_fails_closed():
    estimate, plan, motion = _evaluate(0.0)

    assert estimate.localization_quality.heading is QualityState.LOST
    assert plan.reason == "LOCALIZATION_HOLD"
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s == 0.0
