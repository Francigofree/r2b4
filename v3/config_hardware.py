"""Pure hardware/sensor config construction used only by ConfigResolver."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from v3.adapters.bno055_device import NativeBno055DeviceConfig
from v3.adapters.bno055_imu import Bno055ImuBackendConfig
from v3.adapters.bounded_command import BoundedExploreProfile, BoundedTeleopProfile
from v3.adapters.counter_encoder import CounterEncoderBackendConfig
from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
)
from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.latest_lidar import LatestLidarBackendConfig
from v3.adapters.live_encoder import NativeEncoderConfig
from v3.adapters.live_imu import NativeImuConfig
from v3.adapters.live_lidar import NativeLidarConfig
from v3.adapters.live_camera import NativeCameraConfig
from v3.adapters.live_person_detection import NativePersonDetectionConfig
from v3.adapters.litert_person_detector import (
    litert_person_detector_config_from_mapping,
)
from v3.adapters.person_photo_evidence import (
    person_photo_evidence_config_from_mapping,
)
from v3.adapters.picamera2_camera import (
    Picamera2CameraConfig,
    picamera2_camera_config_from_mapping,
)
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode
from v3.composition.bounded_live_control import BoundedLiveControlConfig
from v3.composition.bounded_physical_control import BoundedPhysicalControlConfig
from v3.composition.native_control import (
    NativeControlCompositionConfig,
    V3_NAVIGATION_CONTRACT,
    V3NavigationConfig,
)
from v3.composition.native_sensor_inputs import (
    NativeSensorHardwareConfig,
    NativeSensorInputConfig,
)
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.layers.l3_state_estimation import NativeStateEstimatorConfig
from v3.layers.l10_chassis_control import ChassisControlConfig
from v3.layers.l11_actuator_control import WheelSpeedMap
from v3.layers.l12_safety_final import LidarSafetyConfig
from v3.composition.runtime_config import (
    BoundedPhysicalRuntimeConfig,
    NativeEncoderRuntimeConfig,
)


POSE_FRAME_ID = "R2B4_BOOT_ROBOT_MAP"


@dataclass(frozen=True, slots=True)
class NativeSensorPolicyConfig:
    """Explicit native runtime thresholds closed at the configuration edge."""

    encoder_maximum_sample_interval_ns: int
    encoder_maximum_abs_velocity_mps: float
    encoder_minimum_trust: float
    imu_maximum_sample_age_ns: int
    imu_heading_clockwise_positive: bool
    imu_yaw_rate_axis: int
    imu_yaw_rate_clockwise_positive: bool
    imu_yaw_offset_rad: float
    imu_minimum_confidence: float
    imu_minimum_calibration: int
    imu_allow_rate_only: bool
    lidar_maximum_result_age_ns: int
    lidar_maximum_future_skew_ns: int
    lidar_pose_r_scale: float
    lidar_minimum_confidence: float
    lidar_maximum_measurement_age_ns: int
    encoder_minimum_estimation_pulses: int = 4
    encoder_minimum_estimation_window_ns: int = 40_000_000
    encoder_maximum_estimation_window_ns: int = 160_000_000
    camera_maximum_frame_age_ns: int = 250_000_000

    def __post_init__(self) -> None:
        # Construct the downstream immutable contracts now, before any file or
        # hardware can be opened. Their validators remain the single authority.
        CounterEncoderBackendConfig(
            1.0,
            1.0,
            maximum_sample_interval_ns=self.encoder_maximum_sample_interval_ns,
            maximum_abs_velocity_mps=self.encoder_maximum_abs_velocity_mps,
            minimum_estimation_pulses=self.encoder_minimum_estimation_pulses,
            minimum_estimation_window_ns=(
                self.encoder_minimum_estimation_window_ns
            ),
            maximum_estimation_window_ns=(
                self.encoder_maximum_estimation_window_ns
            ),
        )
        NativeEncoderConfig("validation-encoder", self.encoder_minimum_trust)
        Bno055ImuBackendConfig(
            self.imu_maximum_sample_age_ns,
            self.imu_heading_clockwise_positive,
            self.imu_yaw_rate_axis,
            self.imu_yaw_rate_clockwise_positive,
            self.imu_yaw_offset_rad,
        )
        NativeImuConfig(
            "validation-imu",
            self.imu_minimum_confidence,
            self.imu_minimum_calibration,
            self.imu_allow_rate_only,
        )
        LatestLidarBackendConfig(
            self.lidar_maximum_result_age_ns,
            self.lidar_pose_r_scale,
            maximum_future_skew_ns=self.lidar_maximum_future_skew_ns,
        )
        NativeLidarConfig(
            "validation-lidar",
            self.lidar_minimum_confidence,
            self.lidar_maximum_measurement_age_ns,
            POSE_FRAME_ID,
        )
        NativeCameraConfig(
            "CAMERA_FRONT",
            self.camera_maximum_frame_age_ns,
        )


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _positive_float(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _required_bool(value: object, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name} must be bool")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _strict_i2c_address(value: object, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer or hexadecimal string")
    try:
        address = int(value, 0) if isinstance(value, str) else int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer or hexadecimal string") from exc
    if not 0x08 <= address <= 0x77:
        raise ValueError(f"{name} must be a valid seven-bit I2C address")
    return address


def _axis_tuple(value: object, name: str) -> tuple[int, int, int]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must be a three-value JSON array")
    if any(not isinstance(item, int) or isinstance(item, bool) for item in value):
        raise ValueError(f"{name} values must be integers")
    return value[0], value[1], value[2]


def _sensor_hardware_config(
    hardware: Mapping[str, object],
    encoder: NativeEncoderRuntimeConfig,
    policy: NativeSensorPolicyConfig,
    navigation: V3NavigationConfig,
    physics: Mapping[str, object],
    imu_policy: Mapping[str, object],
    danger_zone_m: float,
) -> NativeSensorHardwareConfig:
    imu = _mapping(hardware.get("imu"), "hardware config imu")
    if imu.get("provider") != "bno055":
        raise ValueError("hardware config imu.provider must be bno055")
    bno055 = _mapping(imu.get("bno055"), "hardware config imu.bno055")
    operation_mode = imu_policy["operation_mode"]
    if not isinstance(operation_mode, str):
        raise ValueError("hardware config imu.bno055.operation_mode must be a string")
    imu_device = NativeBno055DeviceConfig(
        bus_number=_nonnegative_int(
            bno055.get("bus"),
            "hardware config imu.bno055.bus",
        ),
        address=_strict_i2c_address(
            bno055.get("address"),
            "hardware config imu.bno055.address",
        ),
        operation_mode=operation_mode.strip().upper(),
        startup_timeout_ns=imu_policy["startup_timeout_ns"],
        startup_poll_interval_ns=imu_policy["startup_poll_interval_ns"],
        axis_order=_axis_tuple(
            physics["imu_axis_order"],
            "hardware config imu.bno055.axis_order",
        ),
        axis_sign=_axis_tuple(
            physics["imu_axis_sign"],
            "hardware config imu.bno055.axis_sign",
        ),
        use_external_crystal=_required_bool(
            bno055.get("use_external_crystal"),
            "hardware config imu.bno055.use_external_crystal",
        ),
    )
    camera_device: Picamera2CameraConfig | None = None
    camera_source: NativeCameraConfig | None = None
    camera_value = hardware.get("camera")
    if camera_value is not None:
        camera = _mapping(camera_value, "hardware config camera")
        enabled = camera.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("hardware config camera.enabled must be bool")
        if enabled:
            if camera.get("provider", "picamera2") != "picamera2":
                raise ValueError("hardware config camera.provider must be picamera2")
            camera_device = picamera2_camera_config_from_mapping(camera)
            camera_source = NativeCameraConfig(
                "CAMERA_FRONT",
                policy.camera_maximum_frame_age_ns,
            )

    person_detection_backend = None
    person_detection_source = None
    person_photo_evidence = None
    person_value = hardware.get("person_detection")
    if person_value is not None:
        person = _mapping(person_value, "hardware config person_detection")
        allowed = {
            "enabled",
            "provider",
            "model_path",
            "person_class_id",
            "score_threshold",
            "max_detections",
            "num_threads",
            "maximum_result_age_ns",
            "photo_evidence",
        }
        unknown = sorted(set(person) - allowed)
        if unknown:
            raise ValueError(
                "unknown hardware person_detection keys: " + ", ".join(unknown)
            )
        person_enabled = person.get("enabled", False)
        if type(person_enabled) is not bool:
            raise ValueError("hardware config person_detection.enabled must be bool")
        if person_enabled:
            if camera_device is None:
                raise ValueError("enabled person_detection requires enabled camera")
            backend_keys = {
                "enabled",
                "provider",
                "model_path",
                "person_class_id",
                "score_threshold",
                "max_detections",
                "num_threads",
            }
            person_detection_backend = litert_person_detector_config_from_mapping(
                {key: person[key] for key in backend_keys if key in person}
            )
            person_detection_source = NativePersonDetectionConfig(
                device_id="PERSON_DETECTOR_FRONT",
                maximum_result_age_ns=_positive_int(
                    person.get("maximum_result_age_ns", 500_000_000),
                    "hardware config person_detection.maximum_result_age_ns",
                ),
            )
            photo_value = person.get("photo_evidence")
            if photo_value is not None:
                parsed_photo = person_photo_evidence_config_from_mapping(photo_value)
                if parsed_photo.enabled:
                    person_photo_evidence = parsed_photo

    inputs = NativeSensorInputConfig(
        encoder_counter=encoder.counter_gpio,
        encoder_backend=encoder.backend_config(
            maximum_sample_interval_ns=policy.encoder_maximum_sample_interval_ns,
            maximum_abs_velocity_mps=policy.encoder_maximum_abs_velocity_mps,
            minimum_estimation_pulses=policy.encoder_minimum_estimation_pulses,
            minimum_estimation_window_ns=(
                policy.encoder_minimum_estimation_window_ns
            ),
            maximum_estimation_window_ns=(
                policy.encoder_maximum_estimation_window_ns
            ),
        ),
        encoder_source=NativeEncoderConfig(
            "WHEEL_ENCODERS",
            policy.encoder_minimum_trust,
        ),
        imu_backend=Bno055ImuBackendConfig(
            policy.imu_maximum_sample_age_ns,
            policy.imu_heading_clockwise_positive,
            policy.imu_yaw_rate_axis,
            policy.imu_yaw_rate_clockwise_positive,
            policy.imu_yaw_offset_rad,
        ),
        imu_source=NativeImuConfig(
            "BNO055_IMU",
            policy.imu_minimum_confidence,
            policy.imu_minimum_calibration,
            policy.imu_allow_rate_only,
        ),
        lidar_backend=LatestLidarBackendConfig(
            policy.lidar_maximum_result_age_ns,
            policy.lidar_pose_r_scale,
            maximum_future_skew_ns=policy.lidar_maximum_future_skew_ns,
        ),
        camera_source=camera_source,
        person_detection_source=person_detection_source,
        lidar_source=NativeLidarConfig(
            "RPLIDAR_C1",
            policy.lidar_minimum_confidence,
            policy.lidar_maximum_measurement_age_ns,
            POSE_FRAME_ID,
            local_perception_min_range_m=navigation.local_perception_min_range_m,
            local_perception_max_range_m=navigation.local_perception_max_range_m,
            local_perception_max_points=navigation.local_perception_max_points,
        ),
    )
    return NativeSensorHardwareConfig(
        imu_device=imu_device,
        inputs=inputs,
        lidar_danger_zone_m=danger_zone_m,
        camera_device=camera_device,
        person_detection_backend=person_detection_backend,
        person_photo_evidence=person_photo_evidence,
    )


def _motor_channel(
    motors: Mapping[str, object],
    side: str,
    decay: str,
) -> MotorChannelPhysicalConfig:
    channel = _mapping(motors.get(side), f"motorok.{side}")
    invert = channel["invert"]
    if type(invert) is not bool:
        raise ValueError(f"motorok.{side}.invert must be bool")
    raw_decay_mode = decay
    if not isinstance(raw_decay_mode, str):
        raise ValueError(f"motorok.{side}.pwm_decay_mode must be a string")
    try:
        decay_mode = PwmDecayMode(raw_decay_mode.strip().lower())
    except ValueError as exc:
        raise ValueError(
            f"motorok.{side}.pwm_decay_mode is invalid"
        ) from exc
    return MotorChannelPhysicalConfig(
        in1=channel.get("gpio_in1"),  # type: ignore[arg-type]
        in2=channel.get("gpio_in2"),  # type: ignore[arg-type]
        invert=invert,
        pwm_decay_mode=decay_mode,
    )


def _encoder_runtime_config(
    hardware: Mapping[str, object],
    physics: Mapping[str, object],
    *,
    gpio_chip: int,
    policy: Mapping[str, object],
) -> NativeEncoderRuntimeConfig:
    encoders = _mapping(hardware.get("encoderek"), "hardware config encoderek")
    count_mode = encoders.get("count_mode")
    if count_mode != "X1_A_RISING":
        raise ValueError("hardware config encoderek.count_mode must be X1_A_RISING")

    _positive_int(physics["encoder_impulzus_per_fordulat"], "physics.encoder_impulzus_per_fordulat")
    forward_b_level = physics["encoder_forward_b_level"]
    debounce_micros = policy["a_debounce_micros"]
    direction_guard_micros = policy["direction_guard_micros"]
    direction_change_confirm_edges = policy["direction_change_confirm_edges"]
    direction_change_confirm_window_micros = policy["direction_change_confirm_window_micros"]
    pull_up = _required_bool(
        encoders.get("input_pull_up"),
        "hardware config encoderek.input_pull_up",
    )
    counter_gpio = GpioCounterPairConfig(
        left=GpioCounterChannelConfig(
            pin_a=encoders.get("bal_a_pin"),  # type: ignore[arg-type]
            pin_b=encoders.get("bal_b_pin"),  # type: ignore[arg-type]
            forward_b_level=forward_b_level,  # type: ignore[arg-type]
            invert=_required_bool(
                encoders.get("invert_bal"),
                "hardware config encoderek.invert_bal",
            ),
            pull_up=pull_up,
            a_debounce_micros=debounce_micros,  # type: ignore[arg-type]
            direction_guard_micros=direction_guard_micros,  # type: ignore[arg-type]
            direction_change_confirm_edges=direction_change_confirm_edges,  # type: ignore[arg-type]
            direction_change_confirm_window_micros=(
                direction_change_confirm_window_micros  # type: ignore[arg-type]
            ),
        ),
        right=GpioCounterChannelConfig(
            pin_a=encoders.get("jobb_a_pin"),  # type: ignore[arg-type]
            pin_b=encoders.get("jobb_b_pin"),  # type: ignore[arg-type]
            forward_b_level=forward_b_level,  # type: ignore[arg-type]
            invert=_required_bool(
                encoders.get("invert_jobb"),
                "hardware config encoderek.invert_jobb",
            ),
            pull_up=pull_up,
            a_debounce_micros=debounce_micros,  # type: ignore[arg-type]
            direction_guard_micros=direction_guard_micros,  # type: ignore[arg-type]
            direction_change_confirm_edges=direction_change_confirm_edges,  # type: ignore[arg-type]
            direction_change_confirm_window_micros=(
                direction_change_confirm_window_micros  # type: ignore[arg-type]
            ),
        ),
        gpio_chip=gpio_chip,
        edge_history_capacity=policy["edge_history_capacity"],
        diagnostic_event_capacity=policy["diagnostic_event_capacity"],
    )

    base_step_distance_m = _positive_float(
        physics.get("lepes_hossz_m"),
        "physics config lepes_hossz_m",
    )
    left_multiplier = _positive_float(
        physics.get("lepes_hossz_bal_szorzo"),
        "physics config lepes_hossz_bal_szorzo",
    )
    right_multiplier = _positive_float(
        physics.get("lepes_hossz_jobb_szorzo"),
        "physics config lepes_hossz_jobb_szorzo",
    )
    return NativeEncoderRuntimeConfig(
        counter_gpio=counter_gpio,
        left_step_distance_m=base_step_distance_m * left_multiplier,
        right_step_distance_m=base_step_distance_m * right_multiplier,
    )


