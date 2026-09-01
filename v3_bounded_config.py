"""Close explicit active JSON sources into one immutable bounded V3 config."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from pathlib import Path

from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.gpio_counter import (
    GpioCounterChannelConfig,
    GpioCounterPairConfig,
)
from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.motor_pwm import MotorChannelPhysicalConfig, PwmDecayMode
from v3.composition.bounded_live_control import BoundedLiveControlConfig
from v3.composition.bounded_physical_control import BoundedPhysicalControlConfig
from v3.composition.native_control import NativeControlCompositionConfig
from v3.layers.l3_state_estimation import NativeStateEstimatorConfig
from v3.layers.l10_chassis_control import ChassisControlConfig
from v3.layers.l11_actuator_control import WheelSpeedMap
from v3_bounded_runtime import (
    BoundedPhysicalRuntimeConfig,
    NativeEncoderRuntimeConfig,
)


POSE_FRAME_ID = "R2B4_BOOT_ROBOT_MAP"


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _load_json_object(path_value: str | Path, name: str) -> Mapping[str, object]:
    path = Path(path_value)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} path must be a regular non-symlink file")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{name} must contain valid UTF-8 JSON") from exc
    return _mapping(value, name)


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


def _motor_channel(
    motors: Mapping[str, object],
    side: str,
) -> MotorChannelPhysicalConfig:
    channel = _mapping(motors.get(side), f"motorok.{side}")
    invert = channel.get("invert", False)
    if type(invert) is not bool:
        raise ValueError(f"motorok.{side}.invert must be bool")
    raw_decay_mode = channel.get(
        "pwm_decay_mode",
        motors.get("pwm_decay_mode", PwmDecayMode.COAST.value),
    )
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
) -> NativeEncoderRuntimeConfig:
    encoders = _mapping(hardware.get("encoderek"), "hardware config encoderek")
    count_mode = encoders.get("count_mode")
    if count_mode != "X1_A_RISING":
        raise ValueError("hardware config encoderek.count_mode must be X1_A_RISING")

    hardware_counts = _positive_int(
        encoders.get("counts_per_revolution"),
        "hardware config encoderek.counts_per_revolution",
    )
    physics_counts = _positive_int(
        physics.get("encoder_impulzus_per_fordulat"),
        "physics config encoder_impulzus_per_fordulat",
    )
    if hardware_counts != physics_counts:
        raise ValueError("hardware and physics encoder counts per revolution differ")

    forward_b_level = encoders.get("forward_b_level")
    debounce_micros = encoders.get("a_debounce_micros")
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
        ),
        gpio_chip=gpio_chip,
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


def load_bounded_physical_runtime_config(
    hardware_path: str | Path,
    physics_path: str | Path,
    speed_map_path: str | Path,
    command_profile: BoundedTeleopProfile,
    *,
    tick_period_ns: int = 20_000_000,
    max_preflight_age_ns: int = 250_000_000,
    gpio_chip: int = 0,
    pwm_frequency_hz: int = 8_000,
) -> BoundedPhysicalRuntimeConfig:
    """Load only the static values required by the bounded native V3 path."""

    if not isinstance(command_profile, BoundedTeleopProfile):
        raise TypeError("command_profile must be BoundedTeleopProfile")

    hardware = _load_json_object(hardware_path, "hardware config")
    physics = _load_json_object(physics_path, "physics config")
    speed_map_value = _load_json_object(speed_map_path, "speed-map config")

    motors = _mapping(hardware.get("motorok"), "hardware config motorok")
    left = _motor_channel(motors, "bal_oldal")
    right = _motor_channel(motors, "jobb_oldal")
    motor_output = GpioMotorFrameSinkConfig(
        left=left,
        right=right,
        gpio_chip=gpio_chip,
        pwm_frequency_hz=pwm_frequency_hz,
    )
    encoder = _encoder_runtime_config(
        hardware,
        physics,
        gpio_chip=gpio_chip,
    )
    if set(motor_output.pins) & set(encoder.counter_gpio.pins):
        raise ValueError("motor and encoder GPIO pins must be unique")
    track_width_m = _positive_float(
        physics.get("nyomtav_szelesseg_m"),
        "physics config nyomtav_szelesseg_m",
    )
    speed_map = WheelSpeedMap.from_mapping(speed_map_value)

    control = NativeControlCompositionConfig(
        speed_map=speed_map,
        estimation=NativeStateEstimatorConfig(
            frame_id=POSE_FRAME_ID,
            track_width_m=track_width_m,
        ),
        chassis_control=ChassisControlConfig(track_width_m=track_width_m),
    )
    physical = BoundedPhysicalControlConfig(
        live_control=BoundedLiveControlConfig(
            command_profile=command_profile,
            control=control,
            max_preflight_age_ns=max_preflight_age_ns,
        ),
        motor_output=motor_output,
    )
    return BoundedPhysicalRuntimeConfig(
        composition=physical,
        tick_period_ns=tick_period_ns,
        encoder=encoder,
    )


__all__ = [
    "POSE_FRAME_ID",
    "load_bounded_physical_runtime_config",
]
