import json
from pathlib import Path

import pytest

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
from v3.layers.l10_chassis_control import ChassisControlConfig
from v3.layers.l11_actuator_control import WheelSpeedMap
from v3.layers.l3_state_estimation import NativeStateEstimatorConfig
from v3.replay import (
    V3ReplayError,
    _capture_value,
    inspect_floor_capture,
    replay_floor_capture,
    verify_replay_result,
    write_replay_result,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class _Writer:
    def write(self, _command):
        return None


def _config() -> NativeControlCompositionConfig:
    physics = json.loads((PROJECT_ROOT / "conf/fizika.json").read_text(encoding="utf-8"))
    speed_map = json.loads(
        (PROJECT_ROOT / "conf/speed_map.json").read_text(encoding="utf-8")
    )
    track_width = float(physics["nyomtav_szelesseg_m"])
    return NativeControlCompositionConfig(
        speed_map=WheelSpeedMap.from_mapping(speed_map),
        estimation=NativeStateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=track_width,
        ),
        chassis_control=ChassisControlConfig(track_width),
    )


def _raw(
    context: TickContext,
    measured_mps: float,
    *,
    lidar_state: DeviceHealthState = DeviceHealthState.OK,
) -> RawDeviceBatch:
    samples = (
        DeviceSample(
            "WHEEL_ENCODERS",
            "wheel_velocity",
            context.tick_id,
            context.monotonic_ns,
            (
                DataField("left_mps", measured_mps),
                DataField("right_mps", measured_mps),
                DataField("trust", 1.0),
            ),
        ),
        DeviceSample(
            "BNO055_IMU",
            "ekf_heading",
            context.tick_id,
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
            context.tick_id,
            context.monotonic_ns,
            (
                DataField("age_ns", 0),
                DataField(
                    "confidence",
                    1.0 if lidar_state is DeviceHealthState.OK else 0.1,
                ),
            ),
        ),
    )
    health = (
        DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
        DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
        DeviceHealth(
            "LIDAR_LOCALIZATION",
            lidar_state,
            "LIDAR_LOW_CONFIDENCE"
            if lidar_state is DeviceHealthState.DEGRADED
            else None,
        ),
    )
    return RawDeviceBatch(context, samples, health)


def _command(context: TickContext, active: bool) -> CommandRequest:
    return CommandRequest(
        context=context,
        command_id=f"synthetic.{context.tick_id}",
        mode=CommandMode.TELEOP if active else CommandMode.STOP,
        goal=(
            (
                DataField("v_mps", 0.15),
                DataField("omega_rad_s", 0.0),
                DataField("max_v_mps", 0.15),
                DataField("max_omega_rad_s", 0.05),
            )
            if active
            else ()
        ),
        expiry_tick=context.tick_id,
    )


def _capture(tmp_path: Path) -> Path:
    composition = NativeControlComposition(_Writer(), _config())
    ticks = []
    for tick_id in range(5):
        context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
        active = 1 <= tick_id <= 3
        result = composition.run_tick(
            TickInputs(
                context,
                _raw(context, measured_mps=0.04 if active else 0.0),
                _command(context, active),
                LifecycleState.ACTIVE if active else LifecycleState.IDLE,
            )
        )
        ticks.append(
            {
                "tick_id": tick_id,
                "monotonic_ns": context.monotonic_ns,
                "fault_layer": result.trace.fault_layer,
                "layer_count": len(result.trace.layers),
                "layers": {
                    record.layer: _capture_value(record.output)
                    for record in result.trace.layers
                },
                "final_actuation": _capture_value(result.final_actuation),
                "motor_gpio_events": [],
            }
        )
    path = tmp_path / "capture.json"
    path.write_text(
        json.dumps(
            {
                "schema": "R2B4_V3_NATIVE_FLOOR_TICK_CAPTURE_V1",
                "profile": "synthetic-floor",
                "status": "PASS",
                "tick_count": len(ticks),
                "ticks": ticks,
                "unique_raw_lidar_scan_count": 0,
                "raw_lidar_scans": [],
                "motor_gpio_events_after_last_tick": [],
            }
        ),
        encoding="utf-8",
    )
    return path


def _fault_capture(tmp_path: Path, *, allow_before_fault: bool) -> Path:
    composition = NativeControlComposition(_Writer(), _config())
    ticks = []
    tick_count = 3 if allow_before_fault else 2
    for tick_id in range(tick_count):
        context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
        active = tick_id > 0
        degraded = active and (
            tick_id == 2 if allow_before_fault else tick_id == 1
        )
        result = composition.run_tick(
            TickInputs(
                context,
                _raw(
                    context,
                    measured_mps=0.0,
                    lidar_state=(
                        DeviceHealthState.DEGRADED
                        if degraded
                        else DeviceHealthState.OK
                    ),
                ),
                _command(context, active),
                LifecycleState.ACTIVE if active else LifecycleState.IDLE,
            )
        )
        ticks.append(
            {
                "tick_id": tick_id,
                "monotonic_ns": context.monotonic_ns,
                "fault_layer": result.trace.fault_layer,
                "layer_count": len(result.trace.layers),
                "layers": {
                    record.layer: _capture_value(record.output)
                    for record in result.trace.layers
                },
                "final_actuation": _capture_value(result.final_actuation),
                "motor_gpio_events": [],
            }
        )
    path = tmp_path / "fault-capture.json"
    path.write_text(
        json.dumps(
            {
                "schema": "R2B4_V3_NATIVE_FLOOR_TICK_CAPTURE_V1",
                "profile": "synthetic-fault-floor",
                "status": "FAIL",
                "tick_count": len(ticks),
                "ticks": ticks,
                "unique_raw_lidar_scan_count": 0,
                "raw_lidar_scans": [],
                "motor_gpio_events_after_last_tick": [],
            }
        ),
        encoding="utf-8",
    )
    return path


def test_file_replay_matches_every_production_layer_twice(tmp_path):
    capture = _capture(tmp_path)

    result = replay_floor_capture(
        capture,
        physics_config_path=PROJECT_ROOT / "conf/fizika.json",
        speed_map_config_path=PROJECT_ROOT / "conf/speed_map.json",
        project_root=PROJECT_ROOT,
    )

    assert result["status"] == "MATCH"
    assert result["first_divergence"] is None
    assert result["determinism"]["production_layers"] == "L1-L12"
    assert result["determinism"]["repeated_trace_match"] is True
    assert result["determinism"]["first_run_tick_count"] == 5
    assert result["analysis"]["setpoint_timeline"]["active_tick_count"] == 3
    assert len(result["control_rows"]) == 3

    result_path = write_replay_result(result, tmp_path / "result.json")
    assert verify_replay_result(result_path)["status"] == "PASS"


def test_file_replay_names_the_first_changed_layer(tmp_path):
    capture = _capture(tmp_path)
    payload = json.loads(capture.read_text(encoding="utf-8"))
    payload["ticks"][2]["layers"]["L10"]["left_mps"] += 0.001
    capture.write_text(json.dumps(payload), encoding="utf-8")

    result = replay_floor_capture(
        capture,
        physics_config_path=PROJECT_ROOT / "conf/fizika.json",
        speed_map_config_path=PROJECT_ROOT / "conf/speed_map.json",
        project_root=PROJECT_ROOT,
    )

    assert result["status"] == "MISMATCH"
    assert result["first_divergence"]["tick_id"] == 2
    assert result["first_divergence"]["layer"] == "L10"


def test_file_replay_rejects_noncontiguous_capture(tmp_path):
    capture = _capture(tmp_path)
    payload = json.loads(capture.read_text(encoding="utf-8"))
    payload["ticks"][2]["tick_id"] = 99
    capture.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(V3ReplayError, match="contiguous"):
        replay_floor_capture(
            capture,
            physics_config_path=PROJECT_ROOT / "conf/fizika.json",
            speed_map_config_path=PROJECT_ROOT / "conf/speed_map.json",
            project_root=PROJECT_ROOT,
        )


@pytest.mark.parametrize("allow_before_fault", (False, True))
def test_terminal_failed_capture_replays_fault_transition_and_separates_verdict(
    tmp_path,
    allow_before_fault,
):
    capture = _fault_capture(tmp_path, allow_before_fault=allow_before_fault)

    inspected = inspect_floor_capture(capture)
    result = replay_floor_capture(
        capture,
        physics_config_path=PROJECT_ROOT / "conf/fizika.json",
        speed_map_config_path=PROJECT_ROOT / "conf/speed_map.json",
        project_root=PROJECT_ROOT,
    )

    assert inspected["execution_status"] == "FAIL"
    assert inspected["execution_passed"] is False
    assert result["status"] == "MATCH"
    assert result["capture"]["status"] == "FAIL"
    assert result["execution"]["capture_passed"] is False
    assert result["execution"]["first_non_allow_active_tick_id"] == (
        2 if allow_before_fault else 1
    )
    assert result["execution"]["terminal_lifecycle"] == "ACTIVE"
    assert result["execution"]["terminal_safety_decision"] == "STOP"
    assert result["analysis"]["setpoint_timeline"]["active_tick_count"] == int(
        allow_before_fault
    )


def test_nonterminal_capture_status_remains_invalid(tmp_path):
    capture = _capture(tmp_path)
    payload = json.loads(capture.read_text(encoding="utf-8"))
    payload["status"] = "ACTIVE"
    capture.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(V3ReplayError, match="terminal PASS/FAIL/FAULT"):
        inspect_floor_capture(capture)
