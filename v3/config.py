"""The sole production robot config resolver.

Files are read once, before hardware startup. Every configurable typed field is
required, including fields whose constructors support isolated unit fixtures.
No component receives paths or permission to reopen these documents.
"""
from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
import types
from typing import Union, get_args, get_origin, get_type_hints

from v3.adapters.camera_geometry import camera_geometry_config_from_mapping
from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig
from v3.adapters.multirate_inputs import MultiRateInputConfig
from v3.adapters.native_lidar_port import NativeLidarPortConfig
from v3.adapters.recovering_l6_planner import PlannerRecoveryPolicy
from v3.adapters.rplidar_c1 import RplidarC1Config
from v3.capture_encoding import encode_value
from v3.composition.native_control import NativeControlCompositionConfig, V3NavigationConfig
from v3.composition.resident_live_control import ResidentLiveControlConfig
from v3.composition.resident_physical_control import ResidentPhysicalControlConfig
from v3.config_types import CommandIngressPolicy, EncoderProcessConfig, ImuProcessConfig, LidarProcessConfig, PlannerProcessConfig
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.layers.l10_chassis_control import ChassisControlConfig
from v3.layers.l11_actuator_control import WheelSpeedMap
from v3.lidar_config import LidarMatcherConfig
from v3.runtime_performance import RuntimeAffinityConfig
from v3.config_hardware import NativeSensorPolicyConfig, POSE_FRAME_ID, _encoder_runtime_config, _motor_channel, _sensor_hardware_config
from v3.composition.runtime_config import ResidentPhysicalRuntimeConfig


def _keys(value: object, expected: set[str], path: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    missing, unknown = expected - value.keys(), value.keys() - expected
    if missing or unknown:
        raise ValueError(f"{path}: missing={sorted(missing)}, unknown={sorted(unknown)}")
    return value


def _typed(kind, value, path: str):
    """Strict schema conversion: never consult dataclass defaults or coerce bools."""
    origin, args = get_origin(kind), get_args(kind)
    if origin in (Union, types.UnionType):
        if value is None and type(None) in args:
            return None
        for option in args:
            if option is type(None):
                continue
            try:
                return _typed(option, value, path)
            except (ValueError, TypeError):
                pass
        raise ValueError(f"{path}: invalid value for {kind}")
    if origin in (tuple, frozenset):
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"{path} must be an array")
        if origin is tuple and len(args) > 1 and args[-1] is not Ellipsis:
            if len(args) != len(value):
                raise ValueError(f"{path}: invalid tuple length")
            return tuple(_typed(t, v, f"{path}[{i}]") for i, (t, v) in enumerate(zip(args, value)))
        values = (_typed(args[0], v, f"{path}[{i}]") for i, v in enumerate(value))
        return tuple(values) if origin is tuple else frozenset(values)
    if isinstance(kind, type) and is_dataclass(kind):
        names = {f.name for f in fields(kind) if f.init}
        _keys(value, names, path)
        hints = get_type_hints(kind)
        return kind(**{key: _typed(hints[key], value[key], f"{path}.{key}") for key in names})
    if isinstance(kind, type) and issubclass(kind, Enum):
        return kind(value)
    if kind is float:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{path} must be finite numeric")
        return float(value)
    if kind in (str, int, bool):
        if type(value) is not kind:
            raise ValueError(f"{path} must be {kind.__name__}")
        return value
    raise TypeError(f"Unsupported config type at {path}: {kind}")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate config key: {key}")
        result[key] = value
    return result


def _read(path: Path) -> dict:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{path}: expected regular non-symlink config file")
    try:
        value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object,
                           parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"nonfinite JSON: {value}")))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: invalid UTF-8 JSON") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected JSON object")
    return value


@dataclass(frozen=True, slots=True)
class RuntimeEdgeConfig:
    command_ingress: CommandIngressPolicy
    multirate: MultiRateInputConfig
    planner_recovery: PlannerRecoveryPolicy
    encoder_process: EncoderProcessConfig
    imu_process: ImuProcessConfig
    lidar_process: LidarProcessConfig
    planner_process: PlannerProcessConfig
    pose_history_capacity: int

    def __post_init__(self):
        if type(self.pose_history_capacity) is not int or self.pose_history_capacity <= 0:
            raise ValueError("pose_history_capacity must be a positive integer")


@dataclass(frozen=True, slots=True)
class ResolvedRobotConfig:
    """One immutable effective snapshot shared by live, capture and replay."""
    runtime: ResidentPhysicalRuntimeConfig
    lidar: NativeLidarPortConfig
    affinity: RuntimeAffinityConfig
    edges: RuntimeEdgeConfig

    def __post_init__(self) -> None:
        control = self.runtime.composition.live_control.control
        sensors = self.runtime.sensor_inputs
        if control.lidar_safety is None or control.critical_device_ids != PRODUCTION_CRITICAL_DEVICE_IDS:
            raise ValueError("resolved production config requires canonical safety inputs")
        if control.world_model.local_costmap_max_points_per_scan != sensors.inputs.lidar_source.local_perception_max_points:
            raise ValueError("world-model and LiDAR point budget mismatch")
        if control.lidar_safety.minimum_clearance_m != sensors.lidar_danger_zone_m or self.lidar.danger_zone_m != sensors.lidar_danger_zone_m:
            raise ValueError("LiDAR safety clearance mismatch")
        if control.lidar_safety.maximum_sample_age_ns != sensors.inputs.lidar_source.maximum_measurement_age_ns:
            raise ValueError("LiDAR safety/source freshness mismatch")
        if self.lidar.maximum_result_age_ns != sensors.inputs.lidar_backend.maximum_result_age_ns:
            raise ValueError("LiDAR matcher/source freshness mismatch")
        if control.estimation.frame_id != sensors.inputs.lidar_source.pose_frame_id:
            raise ValueError("localization frame mismatch")
        if (math.hypot(control.navigation.footprint_length_m, control.navigation.footprint_width_m) / 2
                + control.navigation.footprint_safety_margin_m >= control.world_model.local_costmap_radius_m):
            raise ValueError("robot footprint does not fit inside the local costmap")
        if self.edges.encoder_process.sample_period_ns > sensors.inputs.encoder_backend.maximum_sample_interval_ns:
            raise ValueError("encoder process period exceeds sample interval")
        if self.edges.imu_process.sample_period_ns > sensors.inputs.imu_backend.maximum_sample_age_ns:
            raise ValueError("IMU process period exceeds freshness")
        if self.edges.multirate.max_snapshot_age_ns > control.admission.max_sample_age_ns:
            raise ValueError("multirate snapshot freshness exceeds admission limit")

    @property
    def navigation(self) -> V3NavigationConfig:
        control = self.runtime.composition.live_control.control
        source = self.runtime.sensor_inputs.inputs.lidar_source
        return V3NavigationConfig(source.local_perception_min_range_m,
                                  source.local_perception_max_range_m,
                                  source.local_perception_max_points,
                                  control.world_model, control.navigation, control.async_l6)

    def as_dict(self) -> dict:
        return encode_value(self)

    @property
    def snapshot_id(self) -> str:
        return hashlib.sha256(json.dumps(self.as_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

    def bounded(self, command_profile):
        from v3.composition.bounded_live_control import BoundedLiveControlConfig
        from v3.composition.bounded_physical_control import BoundedPhysicalControlConfig
        from v3.composition.runtime_config import BoundedPhysicalRuntimeConfig, NativeEncoderRuntimeConfig
        live = self.runtime.composition.live_control
        sensors = self.runtime.sensor_inputs
        backend = sensors.inputs.encoder_backend
        return BoundedPhysicalRuntimeConfig(
            composition=BoundedPhysicalControlConfig(
                live_control=BoundedLiveControlConfig(command_profile, live.control, live.max_preflight_age_ns),
                motor_output=self.runtime.composition.motor_output),
            tick_period_ns=self.runtime.tick_period_ns,
            encoder=NativeEncoderRuntimeConfig(sensors.inputs.encoder_counter, backend.left_step_distance_m, backend.right_step_distance_m),
            sensor_inputs=sensors)


class ConfigResolver:
    """Single-use resolver; failing validation occurs before any device opens."""
    def __init__(self, hardware_path: str | Path, physics_path: str | Path,
                 speed_map_path: str | Path, control_path: str | Path):
        self._paths = tuple(Path(p) for p in (hardware_path, physics_path, speed_map_path, control_path))
        self._resolved = None

    @classmethod
    def for_project(cls, root: str | Path) -> ConfigResolver:
        conf = Path(root) / "conf"
        return cls(*(conf / name for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")))

    def resolve(self) -> ResolvedRobotConfig:
        if self._resolved is None:
            self._resolved = self.from_documents(*(_read(path) for path in self._paths))
        return self._resolved

    @staticmethod
    def from_documents(hardware: dict, physics: dict, speed_map: dict, control: dict) -> ResolvedRobotConfig:
        h, p, c = hardware, physics, control
        _keys(h, {"motorok", "encoderek", "lidar", "imu", "gpio_chip", "camera", "person_detection"}, "hardver")
        _keys(h["motorok"], {"bal_oldal", "jobb_oldal"}, "hardver.motorok")
        for side in ("bal_oldal", "jobb_oldal"):
            _keys(h["motorok"][side], {"gpio_in1", "gpio_in2", "invert"}, f"hardver.motorok.{side}")
        _keys(h["encoderek"], {"bal_a_pin", "bal_b_pin", "jobb_a_pin", "jobb_b_pin", "count_mode", "input_pull_up", "invert_bal", "invert_jobb"}, "hardver.encoderek")
        _keys(h["lidar"], {"port", "baudrate"}, "hardver.lidar")
        _keys(h["imu"], {"provider", "bno055"}, "hardver.imu")
        _keys(h["imu"]["bno055"], {"bus", "address", "use_external_crystal"}, "hardver.imu.bno055")
        calibration_names = {"imu_heading_clockwise_positive", "imu_yaw_rate_axis", "imu_yaw_rate_clockwise_positive", "imu_yaw_offset_rad"}
        _keys(p, {"nyomtav_szelesseg_m", "encoder_impulzus_per_fordulat", "lepes_hossz_m", "lepes_hossz_bal_szorzo", "lepes_hossz_jobb_szorzo", "footprint_length_m", "footprint_width_m", "encoder_forward_b_level", "imu_axis_order", "imu_axis_sign"} | calibration_names, "fizika")
        _keys(c, {"layers", "local_perception", "sensor_policy", "motor", "encoder", "imu", "lidar_driver", "lidar_pose", "lidar_runtime", "runtime_affinity", "runtime"}, "vezerles")
        runtime = c["runtime"]
        edge_names = {f.name for f in fields(RuntimeEdgeConfig)}
        _keys(runtime, edge_names | {"tick_period_ns", "max_preflight_age_ns", "required_lidar_preflight_revisions"}, "runtime")
        edges = _typed(RuntimeEdgeConfig, {k: runtime[k] for k in edge_names}, "runtime")
        _keys(c["sensor_policy"], {f.name for f in fields(NativeSensorPolicyConfig)} - calibration_names, "sensor_policy")
        policy = _typed(NativeSensorPolicyConfig, {**c["sensor_policy"], **{k:p[k] for k in calibration_names}}, "sensor_policy")
        local = _keys(c["local_perception"], {"min_range_m", "max_range_m", "max_points"}, "local_perception")
        layers = dict(c["layers"])
        for section, derived_names in {
            "estimation": {"frame_id", "track_width_m"},
            "navigation": {"footprint_length_m", "footprint_width_m"},
            "world_model": {
                "local_costmap_max_points_per_scan",
                "person_camera_horizontal_fov_rad",
                "person_camera_yaw_offset_rad",
            },
            "lidar_safety": {"device_id", "maximum_sample_age_ns"},
        }.items():
            layer_type = get_type_hints(NativeControlCompositionConfig)[section]
            if get_origin(layer_type) in (Union, types.UnionType):
                layer_type = next(t for t in get_args(layer_type) if is_dataclass(t))
            _keys(layers[section], {f.name for f in fields(layer_type)} - derived_names, f"layers.{section}")

        camera_geometry = None
        camera_value = h.get("camera")
        if isinstance(camera_value, dict) and camera_value.get("enabled") is True:
            geometry_value = camera_value.get("geometry")
            if not isinstance(geometry_value, dict):
                raise ValueError("enabled camera requires hardver.camera.geometry")
            camera_geometry = camera_geometry_config_from_mapping(geometry_value)
        world_tracking_enabled = layers["world_model"].get("person_tracking_enabled")
        if type(world_tracking_enabled) is not bool:
            raise ValueError("layers.world_model.person_tracking_enabled must be bool")
        if world_tracking_enabled and camera_geometry is None:
            raise ValueError("person tracking requires enabled camera geometry")

        layers["estimation"] = {**layers["estimation"], "track_width_m":p["nyomtav_szelesseg_m"], "frame_id":POSE_FRAME_ID}
        layers["navigation"] = {**layers["navigation"], "footprint_length_m":p["footprint_length_m"], "footprint_width_m":p["footprint_width_m"]}
        layers["world_model"] = {
            **layers["world_model"],
            "local_costmap_max_points_per_scan": local["max_points"],
            # Compatibility-only fallback for historical observations/captures.
            # New live person detections carry frame-specific projected bearings.
            "person_camera_horizontal_fov_rad": (
                None
                if camera_geometry is None
                else math.radians(camera_geometry.factory.horizontal_fov_deg)
            ),
            "person_camera_yaw_offset_rad": (
                None
                if camera_geometry is None
                else math.radians(camera_geometry.mount.yaw_deg)
            ),
        }
        layers["lidar_safety"] = {**layers["lidar_safety"], "device_id":"RPLIDAR_C1", "maximum_sample_age_ns":policy.lidar_maximum_measurement_age_ns}
        derived = {"speed_map", "chassis_control", "critical_device_ids"}
        hints = get_type_hints(NativeControlCompositionConfig)
        _keys(layers, {f.name for f in fields(NativeControlCompositionConfig)} - derived, "layers")
        typed_layers = {name: _typed(hints[name], value, f"layers.{name}") for name, value in layers.items()}
        resolved_control = NativeControlCompositionConfig(**typed_layers,
            speed_map=WheelSpeedMap.from_mapping(speed_map),
            chassis_control=ChassisControlConfig(_typed(float,p["nyomtav_szelesseg_m"],"track_width_m")),
            critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
        if (
            resolved_control.motion_selection.reversal_min_omega_rad_s
            > resolved_control.operational_constraints.max_omega_rad_s
        ):
            raise ValueError("motion-selection reversal threshold exceeds operational omega limit")
        navigation = V3NavigationConfig(_typed(float,local["min_range_m"],"local.min_range_m"),
            _typed(float,local["max_range_m"],"local.max_range_m"),_typed(int,local["max_points"],"local.max_points"),
            resolved_control.world_model, resolved_control.navigation, resolved_control.async_l6)
        motor = _keys(c["motor"], {"pwm_frequency_hz", "pwm_decay_mode"}, "motor")
        _keys(c["encoder"], {"a_debounce_micros", "direction_guard_micros", "direction_change_confirm_edges", "direction_change_confirm_window_micros", "edge_history_capacity", "diagnostic_event_capacity"}, "encoder")
        _keys(c["imu"], {"operation_mode", "startup_timeout_ns", "startup_poll_interval_ns"}, "imu")
        encoder = _encoder_runtime_config(h, p, gpio_chip=h["gpio_chip"], policy=c["encoder"])
        motor_output = GpioMotorFrameSinkConfig(_motor_channel(h["motorok"],"bal_oldal",motor["pwm_decay_mode"]),
            _motor_channel(h["motorok"],"jobb_oldal",motor["pwm_decay_mode"]),h["gpio_chip"],motor["pwm_frequency_hz"])
        if set(motor_output.pins) & set(encoder.counter_gpio.pins):
            raise ValueError("motor and encoder GPIO pins must be unique")
        sensors = _sensor_hardware_config(h, encoder, policy, navigation, p, c["imu"], resolved_control.lidar_safety.minimum_clearance_m)
        pose = _typed(LidarMatcherConfig,c["lidar_pose"],"lidar_pose")
        _keys(c["lidar_driver"], {f.name for f in fields(RplidarC1Config)} - {"port", "baudrate", "minimum_distance_m", "maximum_distance_m"}, "lidar_driver")
        driver = _typed(RplidarC1Config, {**c["lidar_driver"], **h["lidar"], "minimum_distance_m":pose.min_valid_distance_m, "maximum_distance_m":pose.max_valid_distance_m}, "lidar_driver")
        lr = _keys(c["lidar_runtime"], {"matcher_process_start_method", "matcher_process_ready_timeout_s", "matcher_stop_timeout_s", "latest_scan_queue_size", "latest_result_queue_size", "matcher_max_input_age_s", "matcher_max_result_age_s", "driver_poll_hz"}, "lidar_runtime")
        hz = _typed(float,lr["driver_poll_hz"],"lidar_runtime.driver_poll_hz")
        if hz <= 0:
            raise ValueError("driver_poll_hz must be positive")
        lidar = NativeLidarPortConfig(driver, sensors.lidar_danger_zone_m, pose,
            poll_interval_s=1/hz, process_ready_timeout_s=_typed(float,lr["matcher_process_ready_timeout_s"],"matcher ready"),
            process_stop_timeout_s=_typed(float,lr["matcher_stop_timeout_s"],"matcher stop"),
            maximum_input_age_ns=round(_typed(float,lr["matcher_max_input_age_s"],"matcher input age")*1e9),
            maximum_result_age_ns=round(_typed(float,lr["matcher_max_result_age_s"],"matcher result age")*1e9),
            matcher_start_method=lr["matcher_process_start_method"], input_queue_capacity=lr["latest_scan_queue_size"], result_queue_capacity=lr["latest_result_queue_size"])
        tick_ns = _typed(int,runtime["tick_period_ns"],"runtime.tick_period_ns")
        if tick_ns > min(resolved_control.admission.max_sample_age_ns, resolved_control.estimation.max_dt_ns,
                         resolved_control.wheel_pi.max_control_gap_ns, resolved_control.motion_realization.max_control_gap_ns):
            raise ValueError("control period exceeds freshness/control gap")
        if p["nyomtav_szelesseg_m"] > p["footprint_width_m"]:
            raise ValueError("track width exceeds physical footprint width")
        if local["max_range_m"] > pose.max_valid_distance_m or local["min_range_m"] < pose.min_valid_distance_m:
            raise ValueError("local perception range exceeds LiDAR measurement range")
        if resolved_control.async_l6.request_timeout_ns > resolved_control.async_l6.transport_timeout_ns:
            raise ValueError("planner deadline exceeds transport watchdog")
        if lidar.maximum_result_age_ns != policy.lidar_maximum_result_age_ns:
            raise ValueError("LiDAR transport/source freshness limits conflict")
        if policy.encoder_maximum_estimation_window_ns > resolved_control.wheel_pi.max_feedback_uncertainty_ns:
            raise ValueError("encoder estimation window exceeds actuator feedback uncertainty budget")
        physical = ResidentPhysicalRuntimeConfig(ResidentPhysicalControlConfig(
            ResidentLiveControlConfig(resolved_control, runtime["max_preflight_age_ns"],runtime["required_lidar_preflight_revisions"]), motor_output),
            sensors,tick_ns)
        return ResolvedRobotConfig(physical, lidar, _typed(RuntimeAffinityConfig,c["runtime_affinity"],"runtime_affinity"),edges)
