"""Closed-input native V3 L1-L12 control composition."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace

from v3.contracts.planner import PlannerInput, TrajectoryRolloutRequest
from v3.contracts import DeviceHealth, LifecycleState, TickContext
from v3.engine import PipelineLayers, TickEngine, TickInputs, TickResult
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import (
    AdmissionConfig,
    AdmissionStateCheckpoint,
    InputAdmission,
)
from v3.layers.l3_state_estimation import (
    NativeEstimatorStateCheckpoint,
    NativeStateEstimator,
    NativeStateEstimatorConfig,
)
from v3.layers.l4_world_model import (
    ShadowWorldModel,
    WorldModelConfig,
    WorldModelStateCheckpoint,
)
from v3.layers.l5_command_mission import (
    MissionConfig,
    MissionManager,
    MissionStateCheckpoint,
)
from v3.layers.l6_navigation import (
    AsyncL6PlannerConfig,
    InlineTrajectoryRolloutBackend,
    NavigationConfig,
    NavigationStateCheckpoint,
    TrajectoryNavigator,
    TrajectoryRolloutComputer,
)
from v3.layers.l7_motion_selection import (
    MotionSelectionStateCheckpoint,
    MotionSelector,
)
from v3.layers.l8_motion_realization import (
    MotionRealizationConfig,
    MotionRealizationStateCheckpoint,
    MotionRealizer,
)
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer,
    OperationalConstraintsConfig,
    OperationalConstraintsStateCheckpoint,
)
from v3.layers.l10_chassis_control import (
    ChassisControlConfig,
    DifferentialDriveKinematics,
)
from v3.layers.l11_actuator_control import (
    WheelActuatorController,
    WheelActuatorStateCheckpoint,
    WheelPiConfig,
    WheelSpeedMap,
)
from v3.layers.l12_safety_final import (
    FinalSafetyGate,
    FinalSafetyStateCheckpoint,
    LidarSafetyConfig,
)


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
    async_l6: AsyncL6PlannerConfig = AsyncL6PlannerConfig()

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
        if not isinstance(self.async_l6, AsyncL6PlannerConfig):
            raise TypeError("async_l6 must be AsyncL6PlannerConfig")


def v3_navigation_config_from_mapping(
    control: dict[str, object],
) -> V3NavigationConfig:
    """Close the explicit native V3 navigation section into layer configs."""

    root = _mapping(control.get("v3_navigation"), "control config v3_navigation")
    if root.get("contract") != V3_NAVIGATION_CONTRACT:
        raise ValueError("control config v3_navigation.contract is invalid")
    local = _mapping(root.get("local_perception"), "v3_navigation.local_perception")
    rolling = _mapping(root.get("rolling_costmap"), "v3_navigation.rolling_costmap")
    person_tracking_value = root.get("person_tracking")
    person_tracking = (
        {}
        if person_tracking_value is None
        else _mapping(person_tracking_value, "v3_navigation.person_tracking")
    )
    person_tracking_enabled = person_tracking.get("enabled", True)
    if type(person_tracking_enabled) is not bool:
        raise ValueError("v3_navigation.person_tracking.enabled must be bool")
    face_person_value = root.get("face_person")
    face_person = (
        {}
        if face_person_value is None
        else _mapping(face_person_value, "v3_navigation.face_person")
    )
    follow_person_value = root.get("follow_person")
    follow_person = (
        {}
        if follow_person_value is None
        else _mapping(follow_person_value, "v3_navigation.follow_person")
    )
    exploration = _mapping(root.get("exploration"), "v3_navigation.exploration")
    rollout = _mapping(root.get("trajectory_rollout"), "v3_navigation.trajectory_rollout")
    async_value = root.get("async_l6")
    async_mapping = (
        {}
        if async_value is None
        else _mapping(async_value, "v3_navigation.async_l6")
    )
    async_enabled = async_mapping.get("enabled", False)
    if type(async_enabled) is not bool:
        raise ValueError("v3_navigation.async_l6.enabled must be bool")
    release_delay_value = async_mapping.get("release_delay_ns")
    release_delay_ns = (
        None
        if release_delay_value is None
        else _positive_int(
            release_delay_value,
            "v3_navigation.async_l6.release_delay_ns",
        )
    )
    async_l6 = AsyncL6PlannerConfig(
        enabled=async_enabled,
        completion_inputs=async_mapping.get("completion_inputs", False),
        request_timeout_ns=_positive_int(async_mapping.get("request_timeout_ns", 300_000_000), "async_l6.request_timeout_ns"),
        release_tick_gap=_positive_int(
            async_mapping.get("release_tick_gap", 5),
            "v3_navigation.async_l6.release_tick_gap",
        ),
        max_plan_age_ns=_positive_int(
            async_mapping.get("max_plan_age_ns", 350_000_000),
            "v3_navigation.async_l6.max_plan_age_ns",
        ),
        release_delay_ns=release_delay_ns,
    )
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
        person_tracking_enabled=person_tracking_enabled,
        person_camera_horizontal_fov_rad=_positive_float(
            person_tracking.get("camera_horizontal_fov_rad", 1.1519173063162575),
            "v3_navigation.person_tracking.camera_horizontal_fov_rad",
        ),
        person_camera_yaw_offset_rad=_finite_float(
            person_tracking.get("camera_yaw_offset_rad", 0.0),
            "v3_navigation.person_tracking.camera_yaw_offset_rad",
        ),
        person_lidar_max_skew_ns=_positive_int(
            person_tracking.get("lidar_max_skew_ns", 150_000_000),
            "v3_navigation.person_tracking.lidar_max_skew_ns",
        ),
        person_lidar_angular_margin_rad=_finite_float(
            person_tracking.get("lidar_angular_margin_rad", 0.04),
            "v3_navigation.person_tracking.lidar_angular_margin_rad",
        ),
        person_lidar_cluster_depth_m=_positive_float(
            person_tracking.get("lidar_cluster_depth_m", 0.30),
            "v3_navigation.person_tracking.lidar_cluster_depth_m",
        ),
        person_lidar_min_points=_positive_int(
            person_tracking.get("lidar_min_points", 1),
            "v3_navigation.person_tracking.lidar_min_points",
        ),
        person_track_max_association_distance_m=_positive_float(
            person_tracking.get("max_association_distance_m", 0.75),
            "v3_navigation.person_tracking.max_association_distance_m",
        ),
        person_track_max_speed_mps=_positive_float(
            person_tracking.get("max_speed_mps", 6.0),
            "v3_navigation.person_tracking.max_speed_mps",
        ),
        person_track_radius_m=_positive_float(
            person_tracking.get("track_radius_m", 0.30),
            "v3_navigation.person_tracking.track_radius_m",
        ),
        person_track_max_age_ns=_positive_int(
            person_tracking.get("track_max_age_ns", 500_000_000),
            "v3_navigation.person_tracking.track_max_age_ns",
        ),
    )
    navigation = NavigationConfig(
        face_person_min_confidence=_finite_float(
            face_person.get("minimum_confidence", 0.60),
            "v3_navigation.face_person.minimum_confidence",
        ),
        face_person_align_tolerance_rad=_positive_float(
            face_person.get("align_tolerance_rad", 0.10),
            "v3_navigation.face_person.align_tolerance_rad",
        ),
        face_person_release_tolerance_rad=_positive_float(
            face_person.get("release_tolerance_rad", 0.16),
            "v3_navigation.face_person.release_tolerance_rad",
        ),
        follow_person_min_confidence=_finite_float(
            follow_person.get("minimum_confidence", 0.60),
            "v3_navigation.follow_person.minimum_confidence",
        ),
        follow_person_align_tolerance_rad=_positive_float(
            follow_person.get("align_tolerance_rad", 0.22),
            "v3_navigation.follow_person.align_tolerance_rad",
        ),
        follow_person_release_tolerance_rad=_positive_float(
            follow_person.get("release_tolerance_rad", 0.30),
            "v3_navigation.follow_person.release_tolerance_rad",
        ),
        follow_person_stand_off_m=_positive_float(
            follow_person.get("stand_off_m", 1.05),
            "v3_navigation.follow_person.stand_off_m",
        ),
        follow_person_distance_deadband_m=_finite_float(
            follow_person.get("distance_deadband_m", 0.15),
            "v3_navigation.follow_person.distance_deadband_m",
        ),
        follow_person_min_safe_distance_m=_positive_float(
            follow_person.get("min_safe_distance_m", 0.75),
            "v3_navigation.follow_person.min_safe_distance_m",
        ),
        follow_person_lost_hold_ns=_positive_int(
            follow_person.get("lost_hold_ns", 400_000_000),
            "v3_navigation.follow_person.lost_hold_ns",
        ),
        follow_person_pivot_enter_rad=_positive_float(
            follow_person.get("pivot_enter_rad", 0.55),
            "v3_navigation.follow_person.pivot_enter_rad",
        ),
        follow_person_hold_release_margin_m=_positive_float(
            follow_person.get("hold_release_margin_m", 0.05),
            "v3_navigation.follow_person.hold_release_margin_m",
        ),
        follow_person_slowdown_distance_m=_positive_float(
            follow_person.get("slowdown_distance_m", 0.18),
            "v3_navigation.follow_person.slowdown_distance_m",
        ),
        follow_person_minimum_follow_speed_mps=_positive_float(
            follow_person.get("minimum_follow_speed_mps", 0.10),
            "v3_navigation.follow_person.minimum_follow_speed_mps",
        ),
        follow_person_heading_min_factor=_positive_float(
            follow_person.get("heading_min_factor", 0.75),
            "v3_navigation.follow_person.heading_min_factor",
        ),
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
        async_l6=async_l6,
    )


@dataclass(frozen=True, slots=True)
class NativeControlCompositionConfig:
    """Immutable production control configuration with no edge authority."""

    speed_map: WheelSpeedMap
    admission: AdmissionConfig = AdmissionConfig(
        max_sample_age_ns=250_000_000, max_future_skew_ns=10_000_000
    )
    estimation: NativeStateEstimatorConfig = NativeStateEstimatorConfig(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        track_width_m=0.3557,
    )
    world_model: WorldModelConfig = WorldModelConfig()
    mission: MissionConfig = MissionConfig()
    navigation: NavigationConfig = NavigationConfig()
    async_l6: AsyncL6PlannerConfig = AsyncL6PlannerConfig()
    motion_realization: MotionRealizationConfig = MotionRealizationConfig()
    operational_constraints: OperationalConstraintsConfig = OperationalConstraintsConfig()
    chassis_control: ChassisControlConfig = ChassisControlConfig(track_width_m=0.3557)
    wheel_pi: WheelPiConfig = WheelPiConfig(
        kp=0.25,
        ki=0.60,
        integrator_limit=0.75,
        max_normalized_output=1.0,
        # Live 2026-09-15 captures proved that the 100 ms default can expire
        # before the 40 ms edge-fit window becomes control-grade during motor
        # startup. Keep the generic WheelPiConfig default available to tests,
        # but make the production reacquisition policy explicit here.
        max_feedback_uncertainty_ns=250_000_000,
    )
    lidar_safety: LidarSafetyConfig | None = None
    critical_device_ids: frozenset[str] | None = None

    def __post_init__(self) -> None:
        expected_types = (
            ("speed_map", self.speed_map, WheelSpeedMap),
            ("admission", self.admission, AdmissionConfig),
            ("estimation", self.estimation, NativeStateEstimatorConfig),
            ("world_model", self.world_model, WorldModelConfig),
            ("mission", self.mission, MissionConfig),
            ("navigation", self.navigation, NavigationConfig),
            ("async_l6", self.async_l6, AsyncL6PlannerConfig),
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
        if self.critical_device_ids is not None:
            if not isinstance(self.critical_device_ids, frozenset):
                raise TypeError("critical_device_ids must be frozenset[str] or None")
            if not self.critical_device_ids:
                raise ValueError("critical_device_ids cannot be empty")
            if any(
                not isinstance(device_id, str) or not device_id.strip()
                for device_id in self.critical_device_ids
            ):
                raise ValueError("critical_device_ids must contain non-empty strings")


@dataclass(frozen=True, slots=True)
class NativeControlStateCheckpoint:
    """Bounded post-tick state needed to start a short canonical replay."""

    engine_last_context: TickContext
    admission: AdmissionStateCheckpoint
    estimation: NativeEstimatorStateCheckpoint
    world_model: WorldModelStateCheckpoint
    mission: MissionStateCheckpoint
    navigation: NavigationStateCheckpoint
    operational_constraints: OperationalConstraintsStateCheckpoint
    actuator_control: WheelActuatorStateCheckpoint
    final_safety: FinalSafetyStateCheckpoint
    motion_realization: MotionRealizationStateCheckpoint = MotionRealizationStateCheckpoint()
    motion_selection: MotionSelectionStateCheckpoint = MotionSelectionStateCheckpoint()


class NativeControlComposition:
    """Run the canonical control layers over already closed tick inputs.

    The caller owns every edge snapshot and lifecycle value. This composition
    owns only stateful layer instances and never exposes the injected writer.
    """

    __slots__ = (
        "_actuator_control",
        "_admission",
        "_engine",
        "_estimator",
        "_final_safety",
        "_mission",
        "_motion_selection",
        "_motion_realization",
        "_navigation",
        "_operational_constraints",
        "_rollout_backend",
        "_closed_planner_mode",
        "_planner_config",
        "_transport_request",
        "_transport_id",
        "_transport_error",
        "_transport_started_ns",
        "_transport_timeout_ns",
        "_world_model",
    )

    def __init__(
        self,
        motor_writer: object,
        config: NativeControlCompositionConfig,
        *,
        trajectory_rollout_backend: object | None = None,
    ) -> None:
        if not callable(getattr(motor_writer, "write", None)):
            raise TypeError("motor_writer must provide a callable write method")
        if not isinstance(config, NativeControlCompositionConfig):
            raise TypeError("config must be NativeControlCompositionConfig")

        admission = InputAdmission(config.admission)
        estimator = NativeStateEstimator(config.estimation)
        world_model = ShadowWorldModel(config.world_model)
        mission = MissionManager(config.mission)
        if config.async_l6.enabled:
            backend = trajectory_rollout_backend
            if backend is None:
                backend = InlineTrajectoryRolloutBackend(config.navigation)
            navigation = TrajectoryNavigator(
                config.navigation,
                rollout_backend=None if config.async_l6.completion_inputs else backend,
                completion_inputs=config.async_l6.completion_inputs,
                request_timeout_ns=config.async_l6.request_timeout_ns,
                rollout_release_tick_gap=config.async_l6.release_tick_gap,
                rollout_release_delay_ns=config.async_l6.release_delay_ns,
                max_plan_age_ns=config.async_l6.max_plan_age_ns,
            )
        else:
            if trajectory_rollout_backend is not None:
                raise ValueError("trajectory rollout backend requires async_l6.enabled")
            backend = None
            navigation = TrajectoryNavigator(config.navigation)
        motion_selection = MotionSelector()
        motion_realization = MotionRealizer(config.motion_realization)
        operational_constraints = OperationalConstraintLayer(
            config.operational_constraints
        )
        actuator_control = WheelActuatorController(
            config.speed_map,
            config.wheel_pi,
        )
        final_safety = FinalSafetyGate(
            motor_writer,
            config.lidar_safety,
            critical_device_ids=config.critical_device_ids,
        )
        self._admission = admission
        self._estimator = estimator
        self._world_model = world_model
        self._mission = mission
        self._navigation = navigation
        self._motion_selection = motion_selection
        self._closed_planner_mode = config.async_l6.enabled and config.async_l6.completion_inputs
        self._planner_config = config.navigation
        self._transport_request = None
        self._transport_id = None
        self._transport_error = None
        self._transport_started_ns = None
        self._transport_timeout_ns = config.async_l6.request_timeout_ns
        self._rollout_backend = backend
        self._motion_realization = motion_realization
        self._operational_constraints = operational_constraints
        self._actuator_control = actuator_control
        self._final_safety = final_safety
        self._engine = TickEngine(
            PipelineLayers(
                acquisition=acquire,
                admission=admission,
                estimation=estimator,
                world_model=world_model,
                command_mission=mission.evaluate,
                navigation=navigation.evaluate,
                navigation_with_completion=navigation.evaluate if self._closed_planner_mode else None,
                motion_selection=motion_selection.evaluate,
                motion_realization=motion_realization.evaluate,
                constraints=operational_constraints.evaluate,
                chassis_control=DifferentialDriveKinematics(config.chassis_control),
                actuator_control=actuator_control,
                final_safety=final_safety,
            )
        )

    @property
    def tick_evidence(self) -> tuple[object, ...]:
        """Expose bounded EKF facts plus any caught L1-L11 exception detail."""

        return self._estimator.last_update_evidence + self._engine.fault_evidence

    def set_timing_observer(
        self, observer: Callable[[str, int], None] | None
    ) -> None:
        """Forward passive layer timing to the deterministic engine."""

        self._engine.set_timing_observer(observer)

    def checkpoint(self) -> NativeControlStateCheckpoint:
        context = self._engine.checkpoint()
        if context is None:
            raise RuntimeError("a state checkpoint requires one completed tick")
        return NativeControlStateCheckpoint(
            context,
            self._admission.checkpoint(),
            self._estimator.checkpoint(),
            self._world_model.checkpoint(),
            self._mission.checkpoint(),
            self._navigation.checkpoint(),
            self._operational_constraints.checkpoint(),
            self._actuator_control.checkpoint(),
            self._final_safety.checkpoint(),
            self._motion_realization.checkpoint(),
            self._motion_selection.checkpoint(),
        )

    def restore(self, checkpoint: NativeControlStateCheckpoint) -> None:
        if not isinstance(checkpoint, NativeControlStateCheckpoint):
            raise TypeError("checkpoint must be NativeControlStateCheckpoint")
        if checkpoint.estimation.last_context != checkpoint.engine_last_context:
            raise ValueError("estimator checkpoint is not aligned to the engine")
        self._admission.restore(checkpoint.admission)
        self._estimator.restore(checkpoint.estimation)
        self._world_model.restore(checkpoint.world_model)
        self._mission.restore(checkpoint.mission)
        self._navigation.restore(checkpoint.navigation)
        self._motion_selection.restore(checkpoint.motion_selection)
        self._motion_realization.restore(checkpoint.motion_realization)
        self._operational_constraints.restore(checkpoint.operational_constraints)
        self._actuator_control.restore(checkpoint.actuator_control)
        self._final_safety.restore(checkpoint.final_safety)
        self._engine.restore(checkpoint.engine_last_context)

    def _sync_planner_transport(
        self,
        request: TrajectoryRolloutRequest | None,
        started_ns: int,
    ) -> bool:
        """Synchronize the authority-free worker transport with L6-owned request state."""
        if request == self._transport_request:
            return False
        backend = self._rollout_backend
        if self._transport_id is not None and backend is not None:
            backend.abandon(self._transport_id)
        self._transport_request = request
        self._transport_id = None
        self._transport_error = None
        self._transport_started_ns = None
        if request is None:
            return False
        try:
            if backend is None:
                raise RuntimeError("ASYNC_L6_BACKEND_MISSING")
            self._transport_id = backend.submit(request)
            self._transport_started_ns = started_ns
        except Exception as exc:
            self._transport_error = f"{type(exc).__name__}:{exc}"[:256]
        return True

    def dispatch_pending_planner_request(self, monotonic_ns: int) -> bool:
        """Dispatch a request created by the completed tick without exposing a completion.

        L6 remains the sole navigation-state owner. This method only advances the
        runtime transport edge after L1-L12 has completed; worker output can still
        become visible only through a later ``close_inputs`` call.
        """
        if not self._closed_planner_mode:
            return False
        if (
            not isinstance(monotonic_ns, int)
            or isinstance(monotonic_ns, bool)
            or monotonic_ns < 0
        ):
            raise ValueError("planner dispatch monotonic_ns must be non-negative int")
        request = self._navigation.pending_rollout_request
        if request is not None and monotonic_ns < request.context.monotonic_ns:
            raise ValueError("planner dispatch cannot precede request source time")
        return self._sync_planner_transport(request, monotonic_ns)

    def close_inputs(self, inputs: TickInputs) -> TickInputs:
        """Close async planner completion before L1 without borrowing pre-submit time.

        The transport deadline starts only after the request has actually been
        submitted to the worker. Completion visibility, transport failure and
        deadline expiry are all frozen into ``PlannerInput`` so canonical replay
        never re-evaluates worker scheduling.
        """
        if not self._closed_planner_mode or inputs.planner_input is not None:
            return inputs
        event = PlannerInput(inputs.context)
        request = self._navigation.pending_rollout_request
        # Live production normally dispatches immediately after the tick that
        # created this immutable request. Keep this fallback for replay/tests and
        # any caller that does not own an explicit post-tick runtime edge.
        self._sync_planner_transport(request, inputs.context.monotonic_ns)
        backend = self._rollout_backend
        if request is not None:
            result = None
            if self._transport_error is None:
                started_ns = self._transport_started_ns
                if started_ns is None:
                    self._transport_error = "ASYNC_L6_TRANSPORT_STATE_INVALID"
                elif inputs.context.monotonic_ns - started_ns > self._transport_timeout_ns:
                    if self._transport_id is not None and backend is not None:
                        backend.abandon(self._transport_id)
                    self._transport_id = None
                    self._transport_error = "ASYNC_L6_DEADLINE_MISSED"
                else:
                    try:
                        result = backend.take(self._transport_id)
                    except Exception as exc:
                        self._transport_error = f"{type(exc).__name__}:{exc}"[:256]
            if result is not None or self._transport_error is not None:
                self._transport_id = None
                event = PlannerInput(
                    inputs.context,
                    request.context,
                    result,
                    self._transport_error,
                )
        return replace(inputs, planner_input=event)

    def verify_planner_input(self, inputs: TickInputs) -> None:
        """Offline kernel check independent of recorded arrival scheduling."""
        event = inputs.planner_input
        request = self._navigation.pending_rollout_request
        if event is not None and event.result is not None and request is not None and event.request_context == request.context:
            expected = TrajectoryRolloutComputer(self._planner_config).compute(request)
            if event.result != expected:
                raise ValueError("PLANNER_PURE_RESULT_MISMATCH")

    def run_tick(self, inputs: TickInputs) -> TickResult:
        if not isinstance(inputs, TickInputs):
            raise TypeError("inputs must be TickInputs")
        return self._engine.run_tick(inputs)

    def close(self) -> None:
        backend = self._rollout_backend
        self._rollout_backend = None
        if backend is not None:
            backend.close()

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
    "NativeControlStateCheckpoint",
    "V3_NAVIGATION_CONTRACT",
    "V3NavigationConfig",
    "v3_navigation_config_from_mapping",
]
