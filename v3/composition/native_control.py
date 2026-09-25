# R2B4_FOLLOW_PERSON_P0_V2_20260923
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
    MotionSelectionConfig,
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
    async_l6: AsyncL6PlannerConfig

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


@dataclass(frozen=True, slots=True)
class NativeControlCompositionConfig:
    """Immutable production control configuration with no edge authority."""

    speed_map: WheelSpeedMap
    admission: AdmissionConfig
    estimation: NativeStateEstimatorConfig
    world_model: WorldModelConfig
    mission: MissionConfig
    navigation: NavigationConfig
    async_l6: AsyncL6PlannerConfig
    motion_selection: MotionSelectionConfig
    motion_realization: MotionRealizationConfig
    operational_constraints: OperationalConstraintsConfig
    chassis_control: ChassisControlConfig
    wheel_pi: WheelPiConfig
    lidar_safety: LidarSafetyConfig | None
    critical_device_ids: frozenset[str] | None

    def __post_init__(self) -> None:
        expected_types = (
            ("speed_map", self.speed_map, WheelSpeedMap),
            ("admission", self.admission, AdmissionConfig),
            ("estimation", self.estimation, NativeStateEstimatorConfig),
            ("world_model", self.world_model, WorldModelConfig),
            ("mission", self.mission, MissionConfig),
            ("navigation", self.navigation, NavigationConfig),
            ("async_l6", self.async_l6, AsyncL6PlannerConfig),
            ("motion_selection", self.motion_selection, MotionSelectionConfig),
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
        "_request_timeout_ns",
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
                async_config=config.async_l6,
            )
        else:
            if trajectory_rollout_backend is not None:
                raise ValueError("trajectory rollout backend requires async_l6.enabled")
            backend = None
            navigation = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
        motion_selection = MotionSelector(config.motion_selection)
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
        self._transport_timeout_ns = config.async_l6.transport_timeout_ns
        self._request_timeout_ns = config.async_l6.request_timeout_ns
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

        follow = self._navigation.follow_person_evidence
        follow_evidence = () if follow is None else (follow,)
        return self._estimator.last_update_evidence + self._engine.fault_evidence + follow_evidence

    def set_timing_observer(
        self, observer: Callable[[str, int], None] | None
    ) -> None:
        """Forward passive layer timing to the deterministic engine."""

        self._engine.set_timing_observer(observer)

    def planner_capability_snapshot(self, observed_monotonic_ns: int):
        """Passive transport evidence; never participates in input closure."""
        getter = getattr(self._rollout_backend, "capability_snapshot", None)
        return getter(observed_monotonic_ns) if callable(getter) else None

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

        Worker completion time decides the computation deadline. Collector and
        closure times decide visibility only. A separate transport watchdog
        fails closed on missing delivery. Replay consumes the frozen result/error
        and never re-evaluates worker scheduling.
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
                else:
                    try:
                        take_completion = getattr(backend, "take_completion", None)
                        if callable(take_completion):
                            completion = take_completion(
                                self._transport_id,
                                visible_ns=inputs.context.monotonic_ns,
                                transport_timeout_ns=self._transport_timeout_ns,
                            )
                            if completion is not None:
                                identity = backend.request_identity(self._transport_id, request.context)
                                if completion.identity != identity:
                                    raise RuntimeError("ASYNC_L6_COMPLETION_IDENTITY_MISMATCH")
                                if completion.error is not None:
                                    raise RuntimeError(f"ASYNC_L6_WORKER_FAILED:{completion.error}")
                                if completion.timing.deadline_missed(self._request_timeout_ns):
                                    self._transport_error = "ASYNC_L6_DEADLINE_MISSED"
                                    backend.note_deadline_missed()
                                else:
                                    result = completion.result
                        else:
                            # Pure inline/replay ports have no asynchronous worker
                            # clock. They preserve their explicit result visibility.
                            result = backend.take(self._transport_id)
                        watchdog_started_ns = started_ns
                        current_transport_start = getattr(
                            backend, "transport_started_ns", None
                        )
                        if callable(current_transport_start):
                            restarted_ns = current_transport_start(self._transport_id)
                            if restarted_ns is not None:
                                watchdog_started_ns = restarted_ns
                        transport_restarting = False
                        capability_snapshot = getattr(backend, "capability_snapshot", None)
                        if callable(capability_snapshot):
                            snapshot = capability_snapshot(inputs.context.monotonic_ns)
                            transport_restarting = (
                                getattr(getattr(snapshot, "state", None), "value", None)
                                == "RESTARTING"
                            )
                        if (
                            result is None and self._transport_error is None
                            and not transport_restarting
                            and inputs.context.monotonic_ns - watchdog_started_ns
                            > self._transport_timeout_ns
                        ):
                            backend.abandon(self._transport_id)
                            self._transport_error = "ASYNC_L6_TRANSPORT_TIMEOUT"
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
]
