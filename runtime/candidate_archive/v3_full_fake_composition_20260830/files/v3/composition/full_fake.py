"""Single-process, offline-only composition of the complete V3 layer chain."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

from v3.contracts import FinalActuation, TickContext
from v3.engine import PipelineLayers, TickEngine, TickInputs, TickResult, TickTrace
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import AdmissionConfig, InputAdmission
from v3.layers.l3_state_estimation import ShadowStateEstimator, StateEstimatorConfig
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig
from v3.layers.l5_command_mission import MissionConfig, MissionManager
from v3.layers.l6_navigation import DirectNavigator, NavigationConfig
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
    SpeedMapPoint,
    WheelActuatorController,
    WheelPiConfig,
    WheelSpeedCurve,
    WheelSpeedMap,
)
from v3.layers.l12_safety_final import FinalSafetyGate


_LAYER_NAMES = frozenset(f"L{number}" for number in range(1, 13))


def _offline_speed_map() -> WheelSpeedMap:
    curves = tuple(
        WheelSpeedCurve(
            name=name,
            points=(
                SpeedMapPoint(0.02, 0.15),
                SpeedMapPoint(0.50, 0.85),
            ),
            maintenance_output=0.12,
            startup_output=0.15,
        )
        for name in (
            "left_forward",
            "left_reverse",
            "right_forward",
            "right_reverse",
        )
    )
    return WheelSpeedMap(
        schema="R2B4_WHEEL_SPEED_MAP_V2",
        map_state="ACTIVE",
        curves=curves,
    )


@dataclass(frozen=True, slots=True)
class FullFakeConfig:
    """Immutable configuration closed before the offline composition starts."""

    admission: AdmissionConfig = AdmissionConfig(max_sample_age_ns=250_000_000)
    estimation: StateEstimatorConfig = StateEstimatorConfig(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        track_width_m=0.42,
    )
    world_model: WorldModelConfig = WorldModelConfig()
    mission: MissionConfig = MissionConfig()
    navigation: NavigationConfig = NavigationConfig()
    motion_realization: MotionRealizationConfig = MotionRealizationConfig()
    operational_constraints: OperationalConstraintsConfig = OperationalConstraintsConfig()
    chassis_control: ChassisControlConfig = ChassisControlConfig(track_width_m=0.42)
    speed_map: WheelSpeedMap = field(default_factory=_offline_speed_map)
    wheel_pi: WheelPiConfig = WheelPiConfig(
        kp=0.25,
        ki=0.10,
        integrator_limit=0.5,
        max_normalized_output=1.0,
    )

    def __post_init__(self) -> None:
        if self.estimation.track_width_m != self.chassis_control.track_width_m:
            raise ValueError("L3 and L10 must use the same injected track width")


@dataclass(frozen=True, slots=True, order=True)
class LayerFault:
    """One deterministic fault injected before a selected layer evaluation."""

    tick_id: int
    layer: str

    def __post_init__(self) -> None:
        if not isinstance(self.tick_id, int) or isinstance(self.tick_id, bool) or self.tick_id < 0:
            raise ValueError("fault tick_id must be a non-negative integer")
        if self.layer not in _LAYER_NAMES:
            raise ValueError("fault layer must be one of L1 through L12")


class InjectedLayerFault(RuntimeError):
    """Raised internally at a requested deterministic layer boundary."""


class OfflineMotorSink:
    """In-memory L12 sink with no device handle or physical I/O capability."""

    __slots__ = ("_fail_ticks", "_writes")

    def __init__(self, fail_ticks: frozenset[int] = frozenset()) -> None:
        self._fail_ticks = fail_ticks
        self._writes: list[FinalActuation] = []

    @property
    def writes(self) -> tuple[FinalActuation, ...]:
        return tuple(self._writes)

    def write(self, command: FinalActuation) -> None:
        self._writes.append(command)
        if command.context.tick_id in self._fail_ticks:
            raise InjectedLayerFault(f"injected L12 writer fault at tick {command.context.tick_id}")


class _FaultBoundary:
    __slots__ = ("_fault_ticks", "_layer", "_target")

    def __init__(
        self,
        layer: str,
        target: Callable[..., object],
        fault_ticks: frozenset[int],
    ) -> None:
        self._layer = layer
        self._target = target
        self._fault_ticks = fault_ticks

    def __call__(self, *args: object) -> object:
        if not args:
            raise RuntimeError("layer boundary requires a typed input")
        context = getattr(args[0], "context", None)
        if not isinstance(context, TickContext):
            raise RuntimeError("layer boundary input has no TickContext")
        if context.tick_id in self._fault_ticks:
            raise InjectedLayerFault(
                f"injected {self._layer} fault at tick {context.tick_id}"
            )
        return self._target(*args)


class FullFakeComposition:
    """Wire real V3 L1-L12 implementations to an in-memory final sink.

    The root accepts only already closed ``TickInputs`` values.  It owns no live
    reader, device handle, thread, clock or runtime activation path.
    """

    __slots__ = ("_engine", "_sink")

    def __init__(
        self,
        config: FullFakeConfig = FullFakeConfig(),
        *,
        faults: tuple[LayerFault, ...] = (),
    ) -> None:
        fault_keys = tuple((fault.tick_id, fault.layer) for fault in faults)
        if len(fault_keys) != len(set(fault_keys)):
            raise ValueError("fault injections must be unique")
        fault_ticks = {
            layer: frozenset(fault.tick_id for fault in faults if fault.layer == layer)
            for layer in _LAYER_NAMES
        }

        admission = InputAdmission(config.admission)
        estimation = ShadowStateEstimator(config.estimation)
        world_model = ShadowWorldModel(config.world_model)
        mission = MissionManager(config.mission)
        navigation = DirectNavigator(config.navigation)
        realization = MotionRealizer(config.motion_realization)
        constraints = OperationalConstraintLayer(config.operational_constraints)
        chassis = DifferentialDriveKinematics(config.chassis_control)
        actuator = WheelActuatorController(config.speed_map, config.wheel_pi)
        self._sink = OfflineMotorSink(fault_ticks["L12"])

        def boundary(layer: str, target: Callable[..., object]) -> _FaultBoundary:
            return _FaultBoundary(layer, target, fault_ticks[layer])

        self._engine = TickEngine(
            PipelineLayers(
                acquisition=boundary("L1", acquire),
                admission=boundary("L2", admission),
                estimation=boundary("L3", estimation),
                world_model=boundary("L4", world_model),
                command_mission=boundary("L5", mission.evaluate),
                navigation=boundary("L6", navigation.evaluate),
                motion_selection=boundary("L7", select_motion),
                motion_realization=boundary("L8", realization.evaluate),
                constraints=boundary("L9", constraints.evaluate),
                chassis_control=boundary("L10", chassis),
                actuator_control=boundary("L11", actuator),
                final_safety=FinalSafetyGate(self._sink),
            )
        )

    @property
    def writes(self) -> tuple[FinalActuation, ...]:
        return self._sink.writes

    def run_tick(self, inputs: TickInputs) -> TickResult:
        return self._engine.run_tick(inputs)

    def run_replay(self, inputs: Iterable[TickInputs]) -> tuple[TickTrace, ...]:
        return tuple(self.run_tick(item).trace for item in inputs)


__all__ = [
    "FullFakeComposition",
    "FullFakeConfig",
    "InjectedLayerFault",
    "LayerFault",
    "OfflineMotorSink",
]
