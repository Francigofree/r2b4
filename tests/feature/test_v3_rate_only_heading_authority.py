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


def _frame(tick, *, omega_confidence, relative=True):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    ns = context.monotonic_ns
    observations = (
        _observation("lidar_relative_motion", tick // 5, ns - 20_000_000,
                     start_ns=ns - 120_000_000, dx_m=0.0, dy_m=0.0,
                     dyaw_rad=0.0, rmse_m=.002, observability=.9),
    ) if relative and tick >= 6 and tick % 5 == 1 else ()
    return AdmittedFrame(
        context,
        (
            _observation(
                "wheel_velocity",
                tick,
                ns,
                left_mps=0.0,
                right_mps=0.0,
                trust=0.0,
                rejection_code="BASELINE",
                raw_left_distance_m=0.0,
                raw_right_distance_m=0.0,
                left_counter_running=True,
                right_counter_running=True,
                measurement_timing_valid=True,
                measurement_stale=False,
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
        ) + observations,
        (),
    )


def _evaluate(omega_confidence, *, relative=True):
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    restored = None
    for tick in range(3001):
        frame = _frame(tick, omega_confidence=omega_confidence, relative=relative)
        estimate = estimator(frame)
        if restored is not None:
            assert restored(frame) == estimate
        if tick == 2990:
            restored = NativeStateEstimator(config.estimation)
            restored.restore(estimator.checkpoint())
        if omega_confidence and relative:
            assert estimate.localization_quality.heading is QualityState.GOOD
            assert estimate.localization_quality.local_translation is QualityState.GOOD

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


def test_localization_rate_only_gyro_preserves_local_heading_and_explore_motion():
    estimate, plan, motion = _evaluate(1.0)

    assert estimate.localization_quality.local_translation is QualityState.GOOD
    assert estimate.localization_quality.heading is not QualityState.LOST
    assert estimate.localization_quality.global_position is QualityState.LOST
    assert estimate.covariance_5x5[12] > .2  # Map yaw is still uncertain.
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.reason != "LOCALIZATION_HOLD"
    assert abs(motion.requested_v_mps) + abs(motion.requested_omega_rad_s) > 0.0


def test_localization_missing_absolute_and_rate_heading_authority_still_fails_closed():
    estimate, plan, motion = _evaluate(0.0)

    assert estimate.localization_quality.heading is QualityState.LOST
    assert plan.reason == "LOCALIZATION_HOLD"
    assert motion.requested_v_mps == 0.0
    assert motion.requested_omega_rad_s == 0.0

    # Fresh gyro alone cannot renew the validity of an old relative check.
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    for tick in range(101):
        estimator(_frame(tick, omega_confidence=1.0))
    checkpoint = estimator.checkpoint()
    for failure in ("relative_missing", "relative_degenerate", "relative_unaligned",
                    "relative_overlap", "imu_missing",
                    "encoder_stale", "encoder_bad_timing", "counter_stopped",
                    "counter_error", "encoder_missing", "yaw_disagreement"):
        estimator.restore(checkpoint)
        states = []
        for tick in range(101, 501):
            frame = _frame(tick, omega_confidence=1.0, relative=failure != "relative_missing")
            changed = []
            for observation in frame.accepted:
                values = {v.key: v.value for v in observation.values}
                if observation.kind == "ekf_heading" and failure == "imu_missing":
                    continue
                if observation.kind == "wheel_velocity":
                    if failure == "encoder_stale":
                        values["measurement_stale"] = True
                    elif failure == "encoder_bad_timing":
                        values["measurement_timing_valid"] = False
                    elif failure == "counter_stopped":
                        values["left_counter_running"] = False
                    elif failure == "counter_error":
                        values["rejection_code"] = "COUNTER_READ_ERROR_CHANGED"
                    elif failure == "encoder_missing":
                        continue
                if observation.kind == "lidar_relative_motion":
                    if failure == "relative_degenerate":
                        values["observability"] = 0.0
                    elif failure in {"relative_unaligned", "relative_overlap"}:
                        values["start_ns"] = (0 if failure == "relative_unaligned"
                                              else checkpoint.last_relative_ns-100_000_000)
                        values["observability"] = 1.0
                    elif failure == "yaw_disagreement":
                        values["dyaw_rad"] = .03
                changed.append(replace(observation, values=tuple(DataField(k, v) for k, v in values.items())))
            estimate = estimator(replace(frame, accepted=tuple(changed)))
            if failure in {"relative_unaligned", "relative_overlap"}:
                assert estimate.localization_quality.observability == .9
            states.append(estimate.localization_quality.heading)
            if not failure.startswith("relative_") and (
                estimate.localization_quality.heading is QualityState.LOST
                or estimate.localization_quality.local_translation is QualityState.LOST
            ):
                break
        quality = estimate.localization_quality
        if failure.startswith("relative_"):
            assert QualityState.DEGRADED in states
            assert quality.heading is QualityState.LOST
            # A fresh valid interval can reacquire local authority.
            for tick in range(501, 512):
                estimate = estimator(_frame(tick, omega_confidence=1.0))
            assert estimate.localization_quality.heading is QualityState.GOOD
        elif failure in {"imu_missing", "yaw_disagreement"}:
            assert QualityState.DEGRADED in states
            assert quality.heading is QualityState.LOST
        else:
            assert quality.local_translation is QualityState.LOST

    estimator = NativeStateEstimator(config)
    frame = _frame(0, omega_confidence=1.0)
    frame = replace(frame, accepted=tuple(replace(o, values=tuple(
        v for v in o.values if v.key not in {"raw_left_distance_m", "raw_right_distance_m"}
    )) if o.kind == "wheel_velocity" else o for o in frame.accepted))
    assert estimator(frame).localization_quality.local_translation is QualityState.LOST
