import json
from pathlib import Path
from types import SimpleNamespace

from controller.motion_controller import create_motion_controller_from_config
from controller.motion_resolver import make_motion_proposal
from core.motion.speed_limits import SpeedLimitsRuntime
from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor
from replayer.adapters import (
    ProductionMotionPipelineAdapter,
    executor_contract_from_instance,
    motion_pipeline_contract_from_controller,
)
from replayer.capture import CaptureRecorder, verify_capture
from replayer.contracts import (
    CAPTURE_SCHEMA_V2,
    CAPTURE_STATUS_ACTIVE,
    CAPTURE_STATUS_COMPLETE,
    CAPTURE_STATUS_INVALID,
    PIPELINE_FRAME_SCHEMA_V2,
    PIPELINE_STAGE_ORDER,
    REPLAY_RESULT_SCHEMA_V2,
)
from replayer.replay import replay_capture, verify_replay_result
from replayer import runtime_capture


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_runtime_capture_close_uses_coordinated_drain_timeout(tmp_path, monkeypatch):
    capture_path = tmp_path / "capture_runtime_close"
    capture_path.mkdir()
    (capture_path / "capture_manifest.json").write_text(
        json.dumps({"status": CAPTURE_STATUS_COMPLETE}),
        encoding="utf-8",
    )
    observed = {}

    class _Recorder:
        def close(self, *, timeout_s, invalid_reason):
            observed["timeout_s"] = timeout_s
            observed["invalid_reason"] = invalid_reason
            return {
                "capture_id": "capture_runtime_close",
                "capture_path": str(capture_path),
                "closed": True,
                "close_timing": {"close_duration_s": 1.25},
            }

        def status(self):
            return {"capture_path": str(capture_path)}

    monkeypatch.setattr(runtime_capture, "_recorder", _Recorder())
    monkeypatch.setattr(
        runtime_capture,
        "_initialization_status",
        {"enabled": True, "state": "ACTIVE"},
    )

    result = runtime_capture.close_runtime_capture()

    assert observed["timeout_s"] == runtime_capture.RUNTIME_CAPTURE_CLOSE_TIMEOUT_S
    assert observed["invalid_reason"] == ""
    assert result["state"] == CAPTURE_STATUS_COMPLETE
    assert result["close_timing"]["close_duration_s"] == 1.25
    assert runtime_capture._recorder is None


def _executor_contract():
    executor = MotionExecutor(
        pid_config=PIDConfig(
            kp=0.25,
            ki=0.08,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.2,
            wheel_feedback_trust_min=0.25,
            motor_compensation_enabled=True,
            straight_hold_enabled=False,
        ),
        turn_intensity=0.765,
        max_pwm=0.95,
        track_width=0.3557,
        control_mode="UNIFIED",
        direction_switch_hold_s=0.0,
        direction_switch_debounce_cycles=3,
        inplace_turn_omega_deadband=0.06,
    )
    return executor_contract_from_instance(executor)


def _production_inputs():
    vezerles = json.loads((PROJECT_ROOT / "conf" / "vezerles.json").read_text(encoding="utf-8"))
    fizika = json.loads((PROJECT_ROOT / "conf" / "fizika.json").read_text(encoding="utf-8"))
    speed_map = json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))
    track_width = float(fizika["nyomtav_szelesseg_m"])
    controller = create_motion_controller_from_config(vezerles, track_width=track_width)
    speed_limits = SpeedLimitsRuntime()
    speed_limits.load_from_config(
        vezerles,
        "UNIFIED",
        9,
        0.95,
        wheel_speed_range_mps=(
            float(speed_map["operating_range_min_mps"]),
            float(speed_map["operating_range_max_mps"]),
        ),
        track_width_m=track_width,
    )
    ctrl = SimpleNamespace(motion_controller=controller, speed_limits=speed_limits)
    return vezerles, fizika, speed_map, ctrl


def _feedback(v_cmd, omega_cmd, measured):
    return {
        "v_l": measured,
        "v_r": measured,
        "v_l_encoder": measured,
        "v_r_encoder": measured,
        "v_l_encoder_raw": measured,
        "v_r_encoder_raw": measured,
        "encoder_combined_trust": 1.0,
        "encoder_forward_reliability": 1.0,
        "encoder_snapshot_stale": False,
        "encoder_timing_valid": True,
        "encoder_timing_error": "",
        "encoder_timing_gap_s": 0.02,
        "feedback_velocity_source": "KIT0085_ENCODER",
        "current_yaw": 0.0,
        "ekf_theta_deg": 0.0,
        "active_command_type": "set_twist",
        "active_command_layer": "MOTION_TARGET",
        "active_execution_mode": "TWIST_EXEC",
        "turn_primitive_requested": "STRAIGHT" if abs(omega_cmd) < 1e-9 else "DIFF_ARC_GENTLE",
        "straight_hold_executor_candidate": abs(omega_cmd) < 1e-9,
        "requested_v": v_cmd,
        "requested_omega": omega_cmd,
    }


def _make_v2_capture(
    tmp_path,
    *,
    capture_id="capture_v2_m1",
    wrong_stage=None,
    omit_stage=None,
    external_gate_transition_at=None,
    return_close_status=False,
    close_invalid_reason="",
):
    data_root = tmp_path / "replayer_data"
    vezerles, fizika, speed_map, ctrl = _production_inputs()
    executor_contract = _executor_contract()
    pipeline_contract = motion_pipeline_contract_from_controller(ctrl)
    producer = ProductionMotionPipelineAdapter(
        pipeline_contract=pipeline_contract,
        executor_contract=executor_contract,
        speed_map=speed_map,
        vezerles_config=vezerles,
        fizika_config=fizika,
    )
    recorder = CaptureRecorder(
        project_root=PROJECT_ROOT,
        executor_contract=executor_contract,
        pipeline_contract=pipeline_contract,
        data_root=data_root,
        capture_id=capture_id,
        queue_size=32,
        seal_read_only=False,
    )
    gate_runtime = {}
    commands = [
        (0.15, 0.0, 0.00),
        (0.15, 0.0, 0.10),
        (0.225, -0.2, 0.16),
        (0.225, -0.2, 0.20),
    ]
    for index, (v_target, omega_target, measured) in enumerate(commands, start=1):
        mono_s = 1.0 + index * 0.02
        if index == external_gate_transition_at:
            gate_runtime = {**gate_runtime, "last_mode": "RESETTING"}
        proposal = make_motion_proposal(
            name="m1_set_twist",
            layer="MOTION_TARGET",
            source="STATE",
            command_type="set_twist",
            execution_mode="TWIST_EXEC",
            v_target=v_target,
            omega_target=omega_target,
            priority=900,
        )
        context = {
            "pose": {"x": 0.0, "y": 0.0, "theta_rad": 0.0},
            "velocity": {
                "v_mps": measured,
                "omega_rad_s": 0.0,
                "left_mps": measured,
                "right_mps": measured,
            },
            "front_clearance_m": 2.0,
            "left_clearance_m": 1.0,
            "right_clearance_m": 1.0,
            "emergency": False,
            "target_visible": False,
            "target_distance_m": None,
            "target_bearing_rad": None,
            "lidar_seq": index,
        }
        gate_input = {
            "lidar_odom_status": {
                "localization_health": "TRACKING",
                "localization_health_reason": "tracking",
                "delivery_status": "available",
                "ekf_applied_gap_s": 0.1,
                "recent_apply_available": True,
                "applied": True,
            },
            "now_s": mono_s,
            "moving_command": True,
            "runtime_state": {**gate_runtime, "pose_reset": {}},
            "cfg": {"enabled": True},
            "v_target": v_target,
            "omega_target": omega_target,
            "execution_mode": "TWIST_EXEC",
            "requested_track_reference": {"left_mps": None, "right_mps": None},
            "track_width_m": float(fizika["nyomtav_szelesseg_m"]),
        }
        stages = {
            "requested_motion": {
                "input": {
                    "proposals": [proposal],
                    "active_source": "STATE",
                    "category_caps": {},
                    "max_total": 8,
                },
                "recorded_output": {},
            },
            "resolver": {
                "input": {
                    "motion_tick_context": context,
                    "now_monotonic_s": mono_s,
                    "now_wall_s": 1_700_000_000.0 + mono_s,
                },
                "recorded_output": {},
            },
            "localization_gate": {"input": gate_input, "recorded_output": {}},
            "reference": {
                "input": {
                    "mode": "TWIST",
                    "dt_s": 0.02,
                    "force_zero": False,
                    "clear_motion_controller_state": False,
                    "v_target": v_target,
                    "omega_target": omega_target,
                    "execution_mode": "TWIST_EXEC",
                    "requested_track_reference": {"left_mps": None, "right_mps": None},
                    "ekf_state": {"v": measured, "omega_rad_s": 0.0},
                    "motion_controller_state_before": {
                        "v_prev": float(producer.motion_controller._v_prev),
                        "omega_prev": float(producer.motion_controller._omega_prev),
                    },
                    "speed_limits_state": ctrl.speed_limits.as_runtime_state(),
                    "controller_state": {
                        "motion_command_source": "STATE",
                        "active_motion_command_type": "set_twist",
                        "active_motion_command_layer": "MOTION_TARGET",
                    },
                },
                "recorded_output": {},
            },
            "motion_executor": {"input": {}, "recorded_output": {}},
            "pwm": {"input": {}, "recorded_output": {}},
        }
        call = {
            "method": "compute_pwm",
            "kwargs": {
                "v_cmd": v_target,
                "omega_cmd": omega_target,
                "sensor_feedback": _feedback(v_target, omega_target, measured),
                "dt": 0.02,
                "execution_mode": "TWIST_EXEC",
                "track_reference": {"left_mps": None, "right_mps": None},
            },
        }
        frame = {
            "cycle_id": 100 + index,
            "monotonic_ns": int(mono_s * 1_000_000_000),
            "dt_s": 0.02,
            "executor_reset_generation": 0,
            "executor_call": call,
            "pipeline": {
                "schema": PIPELINE_FRAME_SCHEMA_V2,
                "stage_order": list(PIPELINE_STAGE_ORDER),
                "stages": stages,
                "plant": {
                    "adapter_id": "NONE",
                    "available": False,
                    "boundary": "PWM_TO_PHYSICAL_OBSERVATION",
                },
            },
        }
        produced = producer.replay_frame(frame)
        reference = produced["stage_outputs"]["reference"]
        call["kwargs"]["v_cmd"] = reference["v_cmd"]
        call["kwargs"]["omega_cmd"] = reference["omega_cmd"]
        call["kwargs"]["track_reference"] = reference["track_reference"]
        for stage_name in PIPELINE_STAGE_ORDER:
            stages[stage_name]["recorded_output"] = produced["stage_outputs"][stage_name]
        stages["motion_executor"]["input"] = call
        stages["pwm"]["input"] = produced["executor_output"]
        gate_runtime = dict(
            produced["stage_outputs"]["localization_gate"]["gate_status"].get(
                "runtime_state", {}
            )
        )
        if wrong_stage == "resolver" and index == 3:
            stages["resolver"]["recorded_output"]["resolved_motion"]["omega_target"] = -0.17
        if wrong_stage == "reference" and index == 3:
            stages["reference"]["recorded_output"]["omega_cmd"] = -0.17
        if omit_stage and index == 2:
            stages.pop(omit_stage)
        executor_output = produced["executor_output"]
        assert recorder.record(
            {
                "cycle_id": 100 + index,
                "monotonic_ns": int(mono_s * 1_000_000_000),
                "dt_s": 0.02,
                "executor_reset_generation": 0,
                "executor_call": call,
                "recorded_executor_output": executor_output,
                "final_output": {
                    "pwm_l": executor_output["pwm_l"],
                    "pwm_r": executor_output["pwm_r"],
                },
                "safety_lineage": {
                    "allow": True,
                    "reason": "OK",
                    "final_pwm_zero_reason": executor_output["output_reason"],
                },
                "pipeline": frame["pipeline"],
            }
        )
    close_status = recorder.close(invalid_reason=close_invalid_reason)
    if return_close_status:
        close_status = {
            **close_status,
            "record_after_close_accepted": recorder.record({}),
        }
        return data_root, capture_id, close_status
    return data_root, capture_id


def test_v2_normal_shutdown_stops_intake_and_never_leaves_capture_active(tmp_path):
    data_root, capture_id, close_status = _make_v2_capture(
        tmp_path,
        capture_id="capture_v2_normal_shutdown",
        return_close_status=True,
    )

    manifest = json.loads(
        (data_root / "captures" / capture_id / "capture_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    timing = dict(close_status.get("close_timing") or {})

    assert close_status["closing"] is True
    assert close_status["closed"] is True
    assert close_status["record_after_close_accepted"] is False
    assert manifest["status"] == CAPTURE_STATUS_COMPLETE
    assert manifest["status"] != CAPTURE_STATUS_ACTIVE
    assert timing["state"] == CAPTURE_STATUS_COMPLETE
    assert timing["frames_flush_fsync_complete"] is True
    assert timing["terminal_manifest_written"] is True
    assert timing["close_duration_s"] >= 0.0
    assert timing["writer_drain_s"] >= 0.0


def test_v2_normal_invalid_shutdown_also_writes_terminal_manifest(tmp_path):
    data_root, capture_id, close_status = _make_v2_capture(
        tmp_path,
        capture_id="capture_v2_normal_invalid_shutdown",
        return_close_status=True,
        close_invalid_reason="controlled_shutdown_invalid",
    )

    manifest = json.loads(
        (data_root / "captures" / capture_id / "capture_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    timing = dict(close_status.get("close_timing") or {})

    assert close_status["closed"] is True
    assert manifest["status"] == CAPTURE_STATUS_INVALID
    assert manifest["status"] != CAPTURE_STATUS_ACTIVE
    assert timing["state"] == CAPTURE_STATUS_INVALID
    assert timing["frames_flush_fsync_complete"] is True
    assert timing["terminal_manifest_written"] is True


def test_v2_replays_full_production_stage_chain_and_preserves_m1_command_slices(tmp_path):
    data_root, capture_id = _make_v2_capture(tmp_path)

    verification = verify_capture(capture_id, data_root=data_root)
    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert verification["valid"] is True
    assert verification["manifest"]["schema"] == CAPTURE_SCHEMA_V2
    assert result["schema"] == REPLAY_RESULT_SCHEMA_V2
    assert result["status"] == "MATCH"
    assert result["diff"]["first_divergence"] is None
    assert all(count == 0 for count in result["diff"]["stage_mismatch_counts"].values())
    signatures = {
        (row["command_type"], row["v_target"], row["omega_target"])
        for row in result["diff"]["requested_command_examples"]
    }
    assert ("set_twist", 0.15, 0.0) in signatures
    assert ("set_twist", 0.225, -0.2) in signatures
    assert result["diff"]["plant_model"]["available"] is False
    assert verify_replay_result(capture_id, result["result_id"], data_root=data_root)["valid"] is True


def test_v2_reports_resolver_as_first_candidate_divergence(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v2_resolver_diff",
        wrong_stage="resolver",
    )
    assert verify_capture(capture_id, data_root=data_root)["valid"] is True

    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert result["status"] == "MISMATCH"
    assert result["diff"]["first_divergence"]["stage"] == "resolver"
    assert result["diff"]["first_divergence"]["command_context"]["omega_target"] == -0.17
    assert result["diff"]["stage_mismatch_counts"]["requested_motion"] == 0
    assert result["diff"]["stage_mismatch_counts"]["resolver"] == 1


def test_v2_reports_reference_shaping_as_first_arc_right_divergence(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v2_reference_diff",
        wrong_stage="reference",
    )

    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert result["status"] == "MISMATCH"
    first = result["diff"]["first_divergence"]
    assert first["stage"] == "reference"
    assert first["command_context"]["command_type"] == "set_twist"
    assert first["command_context"]["omega_target"] == -0.2


def test_v2_replays_external_localization_gate_state_transition(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v2_external_gate_transition",
        external_gate_transition_at=2,
    )

    frames = [
        json.loads(line)
        for line in (data_root / "captures" / capture_id / "frames.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    assert frames[1]["pipeline"]["stages"]["localization_gate"]["recorded_output"][
        "gate_status"
    ]["state_transition"] is True

    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert result["status"] == "MATCH"
    assert result["diff"]["first_divergence"] is None


def test_v2_missing_stage_fails_closed_before_replay(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v2_missing_stage",
        omit_stage="localization_gate",
    )

    verification = verify_capture(capture_id, data_root=data_root)
    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert verification["valid"] is False
    assert any("pipeline_stage" in error for error in verification["errors"])
    assert result["status"] == "INVALID_CAPTURE"
