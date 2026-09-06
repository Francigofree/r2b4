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
    WheelActuatorController,
    WheelPiConfig,
    WheelSpeedMap,
)
from v3.layers.l12_safety_final import FinalSafetyGate


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


class NativeControlComposition:
    """Run the canonical control layers over already closed tick inputs.

    The caller owns every edge snapshot and lifecycle value. This composition
    owns only stateful layer instances and never exposes the injected writer.
    """

    __slots__ = ("_engine",)

    def __init__(
        self,
        motor_writer: object,
        config: NativeControlCompositionConfig,
    ) -> None:
        if not callable(getattr(motor_writer, "write", None)):
            raise TypeError("motor_writer must provide a callable write method")
        if not isinstance(config, NativeControlCompositionConfig):
            raise TypeError("config must be NativeControlCompositionConfig")

        self._engine = TickEngine(
            PipelineLayers(
                acquisition=acquire,
                admission=InputAdmission(config.admission),
                estimation=NativeStateEstimator(config.estimation),
                world_model=ShadowWorldModel(config.world_model),
                command_mission=MissionManager(config.mission).evaluate,
                navigation=DirectNavigator(config.navigation).evaluate,
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
                final_safety=FinalSafetyGate(motor_writer),
            )
        )

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


__all__ = ["NativeControlComposition", "NativeControlCompositionConfig"]
