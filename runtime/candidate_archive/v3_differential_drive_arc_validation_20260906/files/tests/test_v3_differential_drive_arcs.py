import json
from pathlib import Path

import pytest

from v3.adapters.resident_command import (
    AtomicResidentCommandGateway,
    RESIDENT_COMMAND_SCHEMA,
    ResidentCommandMailboxConfig,
)
from v3.capture import CaptureSink
from v3.composition.native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
)
from v3.contracts import (
    ActuatorRequest,
    CommandMode,
    CommandRequest,
    ConstrainedMotion,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
    WheelVelocitySetpoint,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.layers.l10_chassis_control import (
    ChassisControlConfig,
    DifferentialDriveKinematics,
)
from v3.layers.l11_actuator_control import WheelSpeedMap
from v3.layers.l12_safety_final import FinalSafetyGate, LidarSafetyConfig
from v3.layers.l3_state_estimation import NativeStateEstimatorConfig
from v3.replay import replay_capture
from v3_process_runtime import load_resident_runtime_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TRACK_WIDTH_M = 0.3557
LIVE_ARCS = (
    ("mild_left", 0.25, 0.06 / TRACK_WIDTH_M, 0.22, 0.28),
    ("mild_right", 0.25, -0.06 / TRACK_WIDTH_M, 0.28, 0.22),
    ("medium_left", 0.25, 0.12 / TRACK_WIDTH_M, 0.19, 0.31),
    ("medium_right", 0.25, -0.12 / TRACK_WIDTH_M, 0.31, 0.19),
    ("tight_left", 0.25, 0.20 / TRACK_WIDTH_M, 0.15, 0.35),
    ("tight_right", 0.25, -0.20 / TRACK_WIDTH_M, 0.35, 0.15),
)


class _Writer:
    def __init__(self) -> None:
        self.commands = []

    def write(self, command) -> None:
        self.commands.append(command)


def _configuration() -> dict[str, object]:
    return {
        name: json.loads((PROJECT_ROOT / relative).read_text(encoding="utf-8"))
        for name, relative in (
            ("physics", "conf/fizika.json"),
            ("speed_map", "conf/speed_map.json"),
            ("hardware", "conf/hardver.json"),
        )
    }


def _control_config() -> NativeControlCompositionConfig:
    configuration = _configuration()
    physics = configuration["physics"]
    hardware = configuration["hardware"]
    assert isinstance(physics, dict)
    assert isinstance(hardware, dict)
    track_width_m = float(physics["nyomtav_szelesseg_m"])
    return NativeControlCompositionConfig(
        speed_map=WheelSpeedMap.from_mapping(configuration["speed_map"]),
        estimation=NativeStateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=track_width_m,
        ),
        chassis_control=ChassisControlConfig(track_width_m),
        lidar_safety=LidarSafetyConfig(
            "RPLIDAR_C1",
            float(hardware["lidar"]["biztonsagi_zona_m"]),
        ),
    )


def _motion(context: TickContext, v_mps: float, omega_rad_s: float) -> ConstrainedMotion:
    return ConstrainedMotion(
        context=context,
        requested_v_mps=v_mps,
        requested_omega_rad_s=omega_rad_s,
        allowed_v_mps=v_mps,
        allowed_omega_rad_s=omega_rad_s,
        active_constraints=(),
    )


def _safety_sample(context: TickContext, clearance_m: float) -> DeviceSample:
    return DeviceSample(
        device_id="RPLIDAR_C1",
        kind="lidar_safety_clearance",
        sequence=context.tick_id + 1,
        captured_monotonic_ns=context.monotonic_ns,
        values=(
            DataField("age_ns", 0),
            DataField("front_clearance_m", clearance_m),
            DataField("front_observation_count", 8),
            DataField("rear_clearance_m", 1.0),
            DataField("rear_observation_count", 8),
            DataField("left_clearance_m", clearance_m),
            DataField("left_observation_count", 8),
            DataField("right_clearance_m", clearance_m),
            DataField("right_observation_count", 8),
        ),
    )


def _raw(context: TickContext, clearance_m: float = 1.0) -> RawDeviceBatch:
    samples = (
        DeviceSample(
            "WHEEL_ENCODERS",
            "wheel_velocity",
            context.tick_id + 1,
            context.monotonic_ns,
            (
                DataField("left_mps", 0.0),
                DataField("right_mps", 0.0),
                DataField("trust", 1.0),
            ),
        ),
        DeviceSample(
            "BNO055_IMU",
            "ekf_heading",
            context.tick_id + 1,
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
            context.tick_id + 1,
            context.monotonic_ns,
            (DataField("age_ns", 0), DataField("confidence", 1.0)),
        ),
        _safety_sample(context, clearance_m),
    )
    return RawDeviceBatch(
        context,
        samples,
        (
            DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
            DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
            DeviceHealth("LIDAR_LOCALIZATION", DeviceHealthState.OK),
        ),
    )


def _command(
    context: TickContext,
    *,
    command_id: str,
    v_mps: float = 0.0,
    omega_rad_s: float = 0.0,
) -> CommandRequest:
    moving = abs(v_mps) > 1e-12 or abs(omega_rad_s) > 1e-12
    return CommandRequest(
        context=context,
        command_id=command_id,
        mode=CommandMode.TELEOP if moving else CommandMode.STOP,
        goal=(
            (
                DataField("v_mps", v_mps),
                DataField("omega_rad_s", omega_rad_s),
                DataField("max_v_mps", 0.30),
                DataField("max_omega_rad_s", 0.70),
            )
            if moving
            else ()
        ),
        expiry_tick=context.tick_id,
    )


def test_active_runtime_closes_requested_040_m_fail_closed_clearance() -> None:
    runtime = load_resident_runtime_config(PROJECT_ROOT)

    lidar_safety = runtime.composition.live_control.control.lidar_safety
    assert lidar_safety is not None
    assert lidar_safety.minimum_clearance_m == pytest.approx(0.40)


def test_resident_continuous_twist_gateway_accepts_tight_live_arc(
    tmp_path: Path,
) -> None:
    path = (tmp_path / "command.json").resolve()
    now_ns = 1_000_000_000
    omega_rad_s = LIVE_ARCS[-1][2]
    path.write_text(
        json.dumps(
            {
                "schema": RESIDENT_COMMAND_SCHEMA,
                "revision": 1,
                "issued_monotonic_ns": now_ns,
                "expires_monotonic_ns": now_ns + 200_000_000,
                "mode": "TELEOP",
                "v_mps": 0.25,
                "omega_rad_s": omega_rad_s,
                "max_v_mps": 0.30,
                "max_omega_rad_s": 0.70,
            }
        ),
        encoding="utf-8",
    )
    path.chmod(0o600)
    gateway = AtomicResidentCommandGateway(
        ResidentCommandMailboxConfig(path=path),
        monotonic_ns=lambda: now_ns,
    )

    command = gateway.snapshot(TickContext(0, now_ns))

    assert command.mode is CommandMode.TELEOP
    assert {field.key: field.value for field in command.goal}["omega_rad_s"] == pytest.approx(
        omega_rad_s
    )


@pytest.mark.parametrize(
    ("name", "v_mps", "omega_rad_s", "expected_left", "expected_right"),
    LIVE_ARCS,
)
def test_general_differential_drive_arc_targets_stay_in_validated_live_band(
    name: str,
    v_mps: float,
    omega_rad_s: float,
    expected_left: float,
    expected_right: float,
) -> None:
    context = TickContext(0, 1_000_000_000)
    wheels = DifferentialDriveKinematics(ChassisControlConfig(TRACK_WIDTH_M))(
        _motion(context, v_mps, omega_rad_s)
    )

    assert name.endswith("left") == (wheels.right_mps > wheels.left_mps)
    assert (wheels.left_mps, wheels.right_mps) == pytest.approx(
        (expected_left, expected_right)
    )
    assert 0.15 <= wheels.left_mps <= 0.50
    assert 0.15 <= wheels.right_mps <= 0.50


@pytest.mark.parametrize("omega_rad_s", (LIVE_ARCS[0][2], LIVE_ARCS[1][2]))
def test_arc_safety_is_fail_closed_below_040_m(omega_rad_s: float) -> None:
    context = TickContext(0, 1_000_000_000)
    wheels = DifferentialDriveKinematics(ChassisControlConfig(TRACK_WIDTH_M))(
        _motion(context, 0.25, omega_rad_s)
    )
    request = ActuatorRequest(context, 0.3, 0.3)

    clear_writer = _Writer()
    clear = FinalSafetyGate(
        clear_writer,
        LidarSafetyConfig("RPLIDAR_C1", 0.40),
    ).finalize(
        context,
        request,
        (),
        LifecycleState.ACTIVE,
        None,
        (_safety_sample(context, 0.40),),
        wheels,
    )
    blocked_writer = _Writer()
    blocked = FinalSafetyGate(
        blocked_writer,
        LidarSafetyConfig("RPLIDAR_C1", 0.40),
    ).finalize(
        context,
        request,
        (),
        LifecycleState.ACTIVE,
        None,
        (_safety_sample(context, 0.399),),
        wheels,
    )

    assert clear.safety_decision is SafetyDecision.ALLOW
    assert blocked.safety_decision is SafetyDecision.STOP
    assert blocked.reason == "LIDAR_CLEARANCE_LOW"
    assert (blocked.left_output, blocked.right_output) == (0.0, 0.0)


def test_six_free_motion_arcs_capture_and_replay_through_l1_l12(tmp_path: Path) -> None:
    writer = _Writer()
    composition = NativeControlComposition(writer, _control_config())
    capture = CaptureSink(
        "six-free-motion-arcs",
        configuration=_configuration(),
        metadata={"motion_model": "continuous_twist", "motion_primitives": False},
    )
    tick_id = 0

    def run(v_mps: float, omega_rad_s: float, command_id: str) -> None:
        nonlocal tick_id
        context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
        active = abs(v_mps) > 1e-12 or abs(omega_rad_s) > 1e-12
        inputs = TickInputs(
            context,
            _raw(context),
            _command(
                context,
                command_id=command_id,
                v_mps=v_mps,
                omega_rad_s=omega_rad_s,
            ),
            LifecycleState.ACTIVE if active else LifecycleState.IDLE,
        )
        result = composition.run_tick(inputs)
        capture.write(ExecutionRecord(inputs, result))
        assert len(result.trace.layers) == 12
        tick_id += 1

    run(0.0, 0.0, "initial-stop")
    for name, v_mps, omega_rad_s, _, _ in LIVE_ARCS:
        for _ in range(24):
            run(v_mps, omega_rad_s, name)
        for _ in range(3):
            run(0.0, 0.0, f"{name}-stop")

    capture_path = capture.finalize("PASS", tmp_path / "arcs_capture.json")
    replay = replay_capture(capture_path, project_root=PROJECT_ROOT)

    assert replay["status"] == "MATCH"
    assert replay["first_divergence"] is None
    assert replay["determinism"]["production_layers"] == "L1-L12"
    assert replay["determinism"]["repeated_trace_match"] is True
    assert all(
        row["mismatch_count"] == 0
        for row in replay["diagnostics"]["layers"].values()
    )
    assert writer.commands[-1].safety_decision is SafetyDecision.STOP
    assert (writer.commands[-1].left_output, writer.commands[-1].right_output) == (
        0.0,
        0.0,
    )
