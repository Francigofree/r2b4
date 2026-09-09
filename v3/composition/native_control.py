"""Closed-input native V3 L1-L12 control composition."""

from __future__ import annotations

from dataclasses import dataclass

from v3.contracts import DeviceHealth, LifecycleState, TickContext
from v3.engine import PipelineLayers, TickEngine, TickInputs, TickResult
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import AdmissionConfig, InputAdmission
from v3.layers.l3_state_estimation import (
    NativeStateEstimator,
    NativeStateEstimatorConfig,
)
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig
from v3.layers.l5_command_mission import MissionConfig, MissionManager
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizationConfig, MotionRealizer
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer,
    OperationalConstraintsConfig,
)
from v3.layers.l10_chassis_control import (
    ChassisControlConfig,
    DifferentialDriveKinematics,
)
from v3.layers.l11_actuator_control import (
    WheelActuatorController,
    WheelPiConfig,
    WheelSpeedMap,
)
from v3.layers.l12_safety_final import FinalSafetyGate, LidarSafetyConfig


V3_NAVIGATION_CONTRACT = "R2B4_V3_NAVIGATION_V1"


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _finite_float(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not -float("inf") < value < float("inf")
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


def _positive_float(value: object, name: str) -> float:
    parsed = _finite_float(value, name)
    if parsed <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return parsed


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


@dataclass(frozen=True, slots=True)
class V3NavigationConfig:
    """Closed generic V3 local-perception and navigation configuration."""

    local_perception_min_range_m: float
    local_perception_max_range_m: float
    local_perception_max_points: int
    world_model: WorldModelConfig
    navigation: NavigationConfig

    def __post_init__(self) -> None:
        minimum_range = _finite_float(
            self.local_perception_min_range_m,
            "local_perception_min_range_m",
        )
        maximum_range = _finite_float(
            self.local_perception_max_range_m,
            "local_perception_max_range_m",
        )
        if minimum_range < 0.0 or maximum_range <= minimum_range:
            raise ValueError("local perception range must be finite and increasing")
        _positive_int(
            self.local_perception_max_points,
            "local_perception_max_points",
        )
        if not isinstance(self.world_model, WorldModelConfig):
            raise TypeError("world_model must be WorldModelConfig")
        if not isinstance(self.navigation, NavigationConfig):
            raise TypeError("navigation must be NavigationConfig")


def v3_navigation_config_from_mapping(
    control: dict[str, object],
) -> V3NavigationConfig:
    """Close the explicit native V3 navigation section into layer configs."""

    root = _mapping(control.get("v3_navigation"), "control config v3_navigation")
    if root.get("contract") != V3_NAVIGATION_CONTRACT:
        raise ValueError("control config v3_navigation.contract is invalid")
    local = _mapping(root.get("local_perception"), "v3_navigation.local_perception")
    rolling = _mapping(root.get("rolling_costmap"), "v3_navigation.rolling_costmap")
    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")
    rollout = _mapping(root.get("trajectory_rollout"), "v3_navigation.trajectory_rollout")
    local_min_range_m = _finite_float(
        local.get("min_range_m"),
        "v3_navigation.local_perception.min_range_m",
    )
    local_max_range_m = _positive_float(
        local.get("max_range_m"),
        "v3_navigation.local_perception.max_range_m",
    )
    local_max_points = _positive_int(
        local.get("max_points"),
        "v3_navigation.local_perception.max_points",
    )
    world_model = WorldModelConfig(
        local_costmap_resolution_m=_positive_float(
            rolling.get("resolution_m"),
            "v3_navigation.rolling_costmap.resolution_m",
        ),
        local_costmap_radius_m=_positive_float(
            rolling.get("radius_m"),
            "v3_navigation.rolling_costmap.radius_m",
        ),
        local_costmap_max_cell_age_ns=_positive_int(
            rolling.get("max_cell_age_ns"),
            "v3_navigation.rolling_costmap.max_cell_age_ns",
        ),
        local_costmap_max_cells=_positive_int(
            rolling.get("max_cells"),
            "v3_navigation.rolling_costmap.max_cells",
        ),
        local_costmap_max_points_per_scan=local_max_points,
    )
    navigation = NavigationConfig(
        trajectory_replan_interval_ns=_positive_int(
            rollout.get("replan_interval_ns"),
            "v3_navigation.trajectory_rollout.replan_interval_ns",
        ),
        coverage_cell_size_m=_positive_float(
            exploration.get("coverage_cell_size_m"),
            "v3_navigation.exploration.coverage_cell_size_m",
        ),
        coverage_max_cells=_positive_int(
            exploration.get("coverage_max_cells"),
            "v3_navigation.exploration.coverage_max_cells",
        ),
        local_goal_distance_m=_positive_float(
            exploration.get("local_goal_distance_m"),
            "v3_navigation.exploration.local_goal_distance_m",
        ),
        local_goal_tolerance_m=_positive_float(
            exploration.get("local_goal_tolerance_m"),
            "v3_navigation.exploration.local_goal_tolerance_m",
        ),
        local_goal_max_age_ns=_positive_int(
            exploration.get("local_goal_max_age_ns"),
            "v3_navigation.exploration.local_goal_max_age_ns",
        ),
        local_goal_heading_samples=_positive_int(
            exploration.get("local_goal_heading_samples"),
            "v3_navigation.exploration.local_goal_heading_samples",
        ),
        rollout_linear_samples=_positive_int(
            rollout.get("linear_samples"),
            "v3_navigation.trajectory_rollout.linear_samples",
        ),
        rollout_angular_samples=_positive_int(
            rollout.get("angular_samples"),
            "v3_navigation.trajectory_rollout.angular_samples",
        ),
        rollout_horizon_ns=_positive_int(
            rollout.get("horizon_ns"),
            "v3_navigation.trajectory_rollout.horizon_ns",
        ),
        rollout_step_count=_positive_int(
            rollout.get("step_count"),
            "v3_navigation.trajectory_rollout.step_count",
        ),
        footprint_length_m=_positive_float(
            rollout.get("footprint_length_m"),
            "v3_navigation.trajectory_rollout.footprint_length_m",
        ),
        footprint_width_m=_positive_float(
            rollout.get("footprint_width_m"),
            "v3_navigation.trajectory_rollout.footprint_width_m",
        ),
        footprint_safety_margin_m=_finite_float(
            rollout.get("footprint_safety_margin_m"),
            "v3_navigation.trajectory_rollout.footprint_safety_margin_m",
        ),
        clearance_score_cap_m=_positive_float(
            rollout.get("clearance_score_cap_m"),
            "v3_navigation.trajectory_rollout.clearance_score_cap_m",
        ),
        progress_weight=_finite_float(
            rollout.get("progress_weight"),
            "v3_navigation.trajectory_rollout.progress_weight",
        ),
        clearance_weight=_finite_float(
            rollout.get("clearance_weight"),
            "v3_navigation.trajectory_rollout.clearance_weight",
        ),
        smoothness_weight=_finite_float(
            rollout.get("smoothness_weight"),
            "v3_navigation.trajectory_rollout.smoothness_weight",
        ),
        novelty_weight=_finite_float(
            rollout.get("novelty_weight"),
            "v3_navigation.trajectory_rollout.novelty_weight",
        ),
    )
    return V3NavigationConfig(
        local_perception_min_range_m=local_min_range_m,
        local_perception_max_range_m=local_max_range_m,
        local_perception_max_points=local_max_points,
        world_model=world_model,
        navigation=navigation,
    )


@dataclass(frozen=True, slots=True)
class NativeControlCompositionConfig:
    """Immutable production control configuration with no edge authority."""

    speed_map: WheelSpeedMap
    admission: AdmissionConfig = AdmissionConfig(max_sample_age_ns=250_000_000)
    estimation: NativeStateEstimatorConfig = NativeStateEstimatorConfig(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        track_width_m=0.3557,
    )
    world_model: WorldModelConfig = WorldModelConfig()
    mission: MissionConfig = MissionConfig()
    navigation: NavigationConfig = NavigationConfig()
    motion_realization: MotionRealizationConfig = MotionRealizationConfig()
    operational_constraints: OperationalConstraintsConfig = OperationalConstraintsConfig()
    chassis_control: ChassisControlConfig = ChassisControlConfig(track_width_m=0.3557)
    wheel_pi: WheelPiConfig = WheelPiConfig(
        kp=0.25,
        ki=0.10,
        integrator_limit=0.5,
        max_normalized_output=1.0,
    )
    lidar_safety: LidarSafetyConfig | None = None

    def __post_init__(self) -> None:
        expected_types = (
            ("speed_map", self.speed_map, WheelSpeedMap),
            ("admission", self.admission, AdmissionConfig),
            ("estimation", self.estimation, NativeStateEstimatorConfig),
            ("world_model", self.world_model, WorldModelConfig),
            ("mission", self.mission, MissionConfig),
            ("navigation", self.navigation, NavigationConfig),
            (
                "motion_realization",
                self.motion_realization,
                MotionRealizationConfig,
            ),
            (
                "operational_constraints",
                self.operational_constraints,
                OperationalConstraintsConfig,
            ),
            ("chassis_control", self.chassis_control, ChassisControlConfig),
            ("wheel_pi", self.wheel_pi, WheelPiConfig),
        )
        for name, value, expected_type in expected_types:
            if not isinstance(value, expected_type):
                raise TypeError(f"{name} must be {expected_type.__name__}")
        if self.estimation.track_width_m != self.chassis_control.track_width_m:
            raise ValueError("L3 and L10 must use the same injected track width")
        if self.lidar_safety is not None and not isinstance(
            self.lidar_safety,
            LidarSafetyConfig,
        ):
            raise TypeError("lidar_safety must be LidarSafetyConfig or None")


class NativeControlComposition:
    """Run the canonical control layers over already closed tick inputs.

    The caller owns every edge snapshot and lifecycle value. This composition
    owns only stateful layer instances and never exposes the injected writer.
    """

    __slots__ = ("_engine", "_estimator")

    def __init__(
        self,
        motor_writer: object,
        config: NativeControlCompositionConfig,
    ) -> None:
        if not callable(getattr(motor_writer, "write", None)):
            raise TypeError("motor_writer must provide a callable write method")
        if not isinstance(config, NativeControlCompositionConfig):
            raise TypeError("config must be NativeControlCompositionConfig")

        estimator = NativeStateEstimator(config.estimation)
        self._estimator = estimator
        self._engine = TickEngine(
            PipelineLayers(
                acquisition=acquire,
                admission=InputAdmission(config.admission),
                estimation=estimator,
                world_model=ShadowWorldModel(config.world_model),
                command_mission=MissionManager(config.mission).evaluate,
                navigation=TrajectoryNavigator(config.navigation).evaluate,
                motion_selection=select_motion,
                motion_realization=MotionRealizer(config.motion_realization).evaluate,
                constraints=OperationalConstraintLayer(
                    config.operational_constraints
                ).evaluate,
                chassis_control=DifferentialDriveKinematics(config.chassis_control),
                actuator_control=WheelActuatorController(
                    config.speed_map,
                    config.wheel_pi,
                ),
                final_safety=FinalSafetyGate(motor_writer, config.lidar_safety),
            )
        )

    @property
    def tick_evidence(self) -> tuple[object, ...]:
        """Expose only bounded diagnostic facts produced by the last L3 call."""

        return self._estimator.last_update_evidence

    def run_tick(self, inputs: TickInputs) -> TickResult:
        if not isinstance(inputs, TickInputs):
            raise TypeError("inputs must be TickInputs")
        return self._engine.run_tick(inputs)

    def run_fault_tick(
        self,
        context: TickContext,
        lifecycle: LifecycleState,
        reason: str,
        fault_layer: str,
        critical_health: tuple[DeviceHealth, ...] = (),
    ) -> TickResult:
        """Close one upstream edge failure through the same single L12 call."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if not isinstance(lifecycle, LifecycleState):
            raise TypeError("lifecycle must be LifecycleState")
        if not isinstance(reason, str) or not reason:
            raise ValueError("reason must be a non-empty string")
        if not isinstance(fault_layer, str) or not fault_layer:
            raise ValueError("fault_layer must be a non-empty string")
        return self._engine.run_fault_tick(
            context,
            lifecycle,
            reason,
            fault_layer,
            critical_health,
        )


__all__ = [
    "NativeControlComposition",
    "NativeControlCompositionConfig",
    "V3_NAVIGATION_CONTRACT",
    "V3NavigationConfig",
    "v3_navigation_config_from_mapping",
]
