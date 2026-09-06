import json
from pathlib import Path

from v3.capture import CaptureSink
from v3.composition.native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
)
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    RawDeviceBatch,
    TickContext,
)
from v3.engine import TickInputs
from v3.execution import ExecutionBoundary, IterableInputSource
from v3.layers.l10_chassis_control import ChassisControlConfig
from v3.layers.l11_actuator_control import WheelSpeedMap
from v3.layers.l3_state_estimation import NativeStateEstimatorConfig


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RecordingMotorSink:
    def __init__(self) -> None:
        self.commands = []

    def write(self, command) -> None:
        self.commands.append(command)


def configuration_documents() -> dict[str, object]:
    return {
        name: json.loads((PROJECT_ROOT / relative).read_text(encoding="utf-8"))
        for name, relative in (
            ("physics", "conf/fizika.json"),
            ("speed_map", "conf/speed_map.json"),
            ("hardware", "conf/hardver.json"),
        )
    }


def control_config() -> NativeControlCompositionConfig:
    documents = configuration_documents()
    physics = documents["physics"]
    speed_map = documents["speed_map"]
    assert isinstance(physics, dict)
    assert isinstance(speed_map, dict)
    track_width = float(physics["nyomtav_szelesseg_m"])
    return NativeControlCompositionConfig(
        speed_map=WheelSpeedMap.from_mapping(speed_map),
        estimation=NativeStateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=track_width,
        ),
        chassis_control=ChassisControlConfig(track_width),
    )


def tick_inputs(count: int = 5) -> tuple[TickInputs, ...]:
    values = []
    for tick_id in range(count):
        context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
        active = 1 <= tick_id < count - 1
        samples = (
            DeviceSample(
                "WHEEL_ENCODERS",
                "wheel_velocity",
                tick_id,
                context.monotonic_ns,
                (
                    DataField("left_mps", 0.04 if active else 0.0),
                    DataField("right_mps", 0.04 if active else 0.0),
                    DataField("trust", 1.0),
                ),
            ),
            DeviceSample(
                "BNO055_IMU",
                "ekf_heading",
                tick_id,
                context.monotonic_ns,
                (
                    DataField("yaw_rad", 0.0),
                    DataField("omega_rad_s", 0.0),
                    DataField("confidence", 1.0),
                ),
            ),
            DeviceSample(
                "LIDAR_LOCALIZATION",
                "lidar_health",
                tick_id,
                context.monotonic_ns,
                (DataField("age_ns", 0), DataField("confidence", 1.0)),
            ),
        )
        health = tuple(
            DeviceHealth(device_id, DeviceHealthState.OK)
            for device_id in ("WHEEL_ENCODERS", "BNO055_IMU", "LIDAR_LOCALIZATION")
        )
        command = CommandRequest(
            context,
            f"capture.{tick_id}",
            CommandMode.TELEOP if active else CommandMode.STOP,
            (
                (
                    DataField("v_mps", 0.15),
                    DataField("omega_rad_s", 0.0),
                    DataField("max_v_mps", 0.15),
                    DataField("max_omega_rad_s", 0.05),
                )
                if active
                else ()
            ),
            tick_id,
        )
        values.append(
            TickInputs(
                context,
                RawDeviceBatch(context, samples, health),
                command,
                LifecycleState.ACTIVE if active else LifecycleState.IDLE,
            )
        )
    return tuple(values)


def create_general_capture(tmp_path: Path, *, capture_id: str = "general-v3") -> Path:
    motor_sink = RecordingMotorSink()
    production = NativeControlComposition(motor_sink, control_config())
    capture_sink = CaptureSink(
        capture_id,
        configuration=configuration_documents(),
        metadata={"purpose": "general-v3-validation"},
    )
    ExecutionBoundary(production).run(IterableInputSource(tick_inputs()), capture_sink)
    path = tmp_path / f"{capture_id}.json"
    capture_sink.finalize("PASS", path)
    return path


def create_fault_capture(tmp_path: Path, *, capture_id: str = "fault-v3") -> Path:
    """Capture a real production L4 exception closed by the production L12."""

    original = tick_inputs(1)[0]
    fault_input = TickInputs(
        original.context,
        RawDeviceBatch(
            original.context,
            tuple(
                sample
                for sample in original.raw_devices.samples
                if sample.kind != "lidar_health"
            ),
            original.raw_devices.device_health,
        ),
        original.command,
        original.lifecycle,
    )
    motor_sink = RecordingMotorSink()
    production = NativeControlComposition(motor_sink, control_config())
    capture_sink = CaptureSink(
        capture_id,
        configuration=configuration_documents(),
        metadata={"purpose": "fail-closed-v3-validation"},
    )
    ExecutionBoundary(production).run(IterableInputSource((fault_input,)), capture_sink)
    assert len(motor_sink.commands) == 1
    assert motor_sink.commands[0].enabled is False
    path = tmp_path / f"{capture_id}.json"
    capture_sink.finalize("FAULT", path)
    return path
