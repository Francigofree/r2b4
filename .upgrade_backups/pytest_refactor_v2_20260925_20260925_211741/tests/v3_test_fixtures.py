"""Small shared factories for V3 tests.

These helpers intentionally construct valid public contracts.  They do not hide
production behaviour or provide test-only authority.
"""

from __future__ import annotations

from dataclasses import replace

from v3.contracts import (
    CommandMode,
    MissionConstraints,
    MissionIntent,
    MissionLifecycle,
    TickContext,
    VelocityTarget,
)
from v3_bounded_config import NativeSensorPolicyConfig


_SENSOR_POLICY_DEFAULTS = {
    "encoder_maximum_sample_interval_ns": 100_000_000,
    "encoder_maximum_abs_velocity_mps": 1.5,
    "encoder_minimum_trust": 0.5,
    "imu_maximum_sample_age_ns": 100_000_000,
    "imu_heading_clockwise_positive": True,
    "imu_yaw_rate_axis": 2,
    "imu_yaw_rate_clockwise_positive": False,
    "imu_yaw_offset_rad": 0.0,
    "imu_minimum_confidence": 0.5,
    "imu_minimum_calibration": 2,
    "imu_allow_rate_only": True,
    "lidar_maximum_result_age_ns": 250_000_000,
    "lidar_maximum_future_skew_ns": 10_000_000,
    "lidar_pose_r_scale": 1.0,
    "lidar_minimum_confidence": 0.2,
    "lidar_maximum_measurement_age_ns": 250_000_000,
}


def native_sensor_policy(**overrides) -> NativeSensorPolicyConfig:
    """Return one valid native policy, with explicit per-test overrides."""
    values = dict(_SENSOR_POLICY_DEFAULTS)
    unknown = set(overrides) - set(values) - {
        "encoder_minimum_estimation_pulses",
        "encoder_minimum_estimation_window_ns",
        "encoder_maximum_estimation_window_ns",
        "camera_maximum_frame_age_ns",
    }
    if unknown:
        raise TypeError(
            "unknown NativeSensorPolicyConfig override(s): "
            + ", ".join(sorted(unknown))
        )
    values.update(overrides)
    return NativeSensorPolicyConfig(**values)


def without_optional_perception(runtime):
    """Disable camera/person capabilities as one consistent test capability set."""
    sensors = runtime.sensor_inputs
    if sensors is None:
        raise ValueError("runtime must close native sensor inputs")
    inputs = replace(
        sensors.inputs,
        camera_source=None,
        person_detection_source=None,
    )
    sensors = replace(
        sensors,
        inputs=inputs,
        camera_device=None,
        person_detection_backend=None,
        person_photo_evidence=None,
    )
    return replace(runtime, sensor_inputs=sensors)


def active_mission(
    context: TickContext,
    *,
    mode: CommandMode = CommandMode.EXPLORE,
    mission_id: str = "mission-test-active",
) -> MissionIntent:
    """Build a contract-valid ACTIVE mission for behaviour-facing tests."""
    if mode not in (CommandMode.EXPLORE, CommandMode.TELEOP):
        raise ValueError("active_mission supports EXPLORE or TELEOP")
    velocity_target = (
        VelocityTarget(0.0, 0.0)
        if mode is CommandMode.TELEOP
        else None
    )
    return MissionIntent(
        context=context,
        mission_id=mission_id,
        mode=mode,
        target_pose=None,
        velocity_target=velocity_target,
        constraints=MissionConstraints(
            max_v_mps=0.35,
            max_omega_rad_s=1.2,
            corridor_radius_m=0.30,
            goal_tolerance_m=0.08,
            yaw_tolerance_rad=0.10,
        ),
        lifecycle=MissionLifecycle.ACTIVE,
    )
