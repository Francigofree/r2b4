import ast
import json
from pathlib import Path
from types import SimpleNamespace

from controller.motion_controller import create_motion_controller_from_config
from controller.motion_platform_contract import CycleContext
from controller.motion_resolver import make_motion_proposal
from core.motion.speed_limits import SpeedLimitsRuntime
from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor
from replayer.adapters import (
    ProductionMotionPipelineAdapter,
    executor_contract_from_instance,
    motion_layer_contract_from_controller,
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


def test_runtime_capture_executor_call_is_built_before_recording():
    tree = ast.parse((PROJECT_ROOT / "cont.py").read_text(encoding="utf-8"))
    run_method = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "run"
    )
    assignments = [
        node.lineno
        for node in ast.walk(run_method)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "replayer_executor_call"
            for target in node.targets
        )
    ]
    capture_calls = [
        node.lineno
        for node in ast.walk(run_method)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "record_runtime_tick"
    ]

    assert len(assignments) == 1
    assert len(capture_calls) == 1
    assert assignments[0] < capture_calls[0]


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


def test_runtime_capture_detaches_immutable_snapshot_values(monkeypatch):
    captured = {}

    class _ImmutableDict(dict):
        def __deepcopy__(self, _memo):
            raise TypeError("immutable_lidar_snapshot")

    class _ImmutableList(list):
        def __deepcopy__(self, _memo):
            raise TypeError("immutable_lidar_snapshot")

    class _Recorder:
        def record(self, frame):
            captured.update(frame)
            return True

        def mark_invalid(self, reason):
            raise AssertionError(reason)

    monkeypatch.setattr(runtime_capture, "_recorder", _Recorder())
    monkeypatch.setattr(runtime_capture, "_last_matcher_evidence_id", None)

    assert runtime_capture.record_runtime_tick(
        cycle_id=7,
        monotonic_ns=123456789,
        dt_s=0.02,
        executor_reset_generation=1,
        executor_call={"method": "compute", "input": _ImmutableDict({"v": 0.0})},
        executor_pwm_l=0.0,
        executor_pwm_r=0.0,
        executor_output_reason="STOP",
        final_pwm_l=0.0,
        final_pwm_r=0.0,
        safety_allow=True,
        safety_reason="OK",
        final_pwm_zero_reason="STOP",
        pipeline={"stages": _ImmutableDict({"executor": _ImmutableList([0.0])})},
        matcher_evidence=_ImmutableDict(
            {
                "matcher_result_id": 11,
                "input": _ImmutableDict({"scan": _ImmutableList([1.0, 2.0])}),
            }
        ),
    )

    assert captured["executor_call"]["input"] == {"v": 0.0}
    assert type(captured["executor_call"]["input"]) is dict
    assert captured["pipeline"]["stages"]["executor"] == [0.0]
    assert type(captured["pipeline"]["stages"]["executor"]) is list
    assert captured["matcher_evidence"]["input"]["scan"] == [1.0, 2.0]
    assert type(captured["matcher_evidence"]["input"]["scan"]) is list


def _executor_contract():
    speed_map = json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))
    executor = MotionExecutor(
        pid_config=PIDConfig(
            kp=0.25,
            ki=0.08,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.2,
            wheel_feedback_trust_min=0.25,
        ),
        max_pwm=0.95,
        speed_map=speed_map,
        control_mode="UNIFIED",
        direction_switch_hold_s=0.0,
        direction_switch_debounce_cycles=3,
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


def _feedback(*, cycle_id, mono_s, measured):
    return {
        "measurement_id": f"encoder:{cycle_id}",
        "source_timestamp": mono_s,
        "left_mps": measured,
        "right_mps": measured,
        "combined_trust": 1.0,
        "timing_valid": True,
        "stale": False,
        "timing_reason": "",
        "aggregation_window_s": 0.1,
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
    v21=False,
):
    data_root = tmp_path / "replayer_data"
    vezerles, fizika, speed_map, ctrl = _production_inputs()
    executor_contract = _executor_contract()
    pipeline_contract = (
        motion_layer_contract_from_controller(ctrl)
        if v21
        else motion_pipeline_contract_from_controller(ctrl)
    )
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
        platform_cycle_id = str(100 + index)
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
        cycle_context = {
            "cycle_id": platform_cycle_id,
            "monotonic_time": mono_s,
            "dt_observed_s": 0.02,
            "dt_control_s": 0.02,
            "timing_valid": True,
            "timing_reason": "",
        }
        drive_capabilities = {
            "track_width_m": float(fizika["nyomtav_szelesseg_m"]),
            "calibrated_wheel_min_mps": float(speed_map["operating_range_min_mps"]),
            "calibrated_wheel_max_mps": float(speed_map["operating_range_max_mps"]),
            "max_wheel_accel_mps2": 0.35,
            "max_wheel_decel_mps2": 0.35,
            "capability_version": "replay-v21",
        }
        requested_stage_input = {
            "proposals": [proposal],
            "active_source": "STATE",
            "category_caps": {},
            "max_total": 8,
        }
        resolver_stage_input = {
            "motion_tick_context": context,
            "now_monotonic_s": mono_s,
            "now_wall_s": 1_700_000_000.0 + mono_s,
        }
        _, resolver_stage_output = producer.replay_resolver(
            requested_input=requested_stage_input,
            resolver_input=resolver_stage_input,
            cycle_context=CycleContext(**cycle_context),
        )
        resolved_intent = dict(resolver_stage_output["resolved_intent"])
        guidance_input = {
            "resolved_intent": resolved_intent,
            "pose": {
                "frame_id": "R2B4_BOOT_ROBOT_MAP",
                "pose_id": f"pose:{platform_cycle_id}",
                "source_timestamp": mono_s,
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_rad": 0.0,
                "v_mps": measured,
                "omega_rad_s": 0.0,
                "validity": "VALID",
            },
            "world": {
                "world_id": f"world:{platform_cycle_id}",
                "source_timestamp": mono_s,
                "validity": "VALID",
                "lidar_summary": {
                    "front_clearance_m": 2.0,
                    "blocked_front": False,
                },
                "obstacle_status": {},
                "raw_scan": [],
            },
            "cycle_context": cycle_context,
            "drive_capabilities": drive_capabilities,
            "executed_left_mps": measured,
            "executed_right_mps": measured,
            "actual_linear_mps": measured,
            "actual_angular_dps": 0.0,
        }
        gate_stop = bool(index == external_gate_transition_at)
        physical_command = {
            "contract_id": "R2B4_MOTION_PLATFORM_V2_1",
            "physical_command_id": f"physical:{platform_cycle_id}",
            "resolved_id": resolved_intent["resolved_id"],
            "cycle_id": platform_cycle_id,
            "valid_until_monotonic": resolved_intent["valid_until_monotonic"],
            "physical_mode": "BODY_TWIST",
            "v_mps": resolved_intent["v_mps"],
            "omega_rad_s": resolved_intent["omega_rad_s"],
            "left_mps": 0.0,
            "right_mps": 0.0,
            "guidance_reason": "GUIDANCE_APPLIED",
            "trace_metadata": {
                "selected_proposal_id": resolved_intent["selected_proposal_id"],
                "guidance_type": resolved_intent["guidance_type"],
                "pose_id": f"pose:{platform_cycle_id}",
                "world_id": f"world:{platform_cycle_id}",
            },
        }
        motion_envelope = {
            "cycle_id": platform_cycle_id,
            "physical_command_id": physical_command["physical_command_id"],
            "stop_required": gate_stop,
            "stop_reason": "LOCALIZATION_GATE" if gate_stop else "",
            "max_abs_v_mps": 0.30,
            "max_abs_omega_rad_s": 1.68,
            "max_abs_wheel_mps": float(speed_map["operating_range_max_mps"]),
            "max_wheel_accel_mps2": 0.35,
            "max_wheel_decel_mps2": 0.35,
            "capability_version": "replay-v21",
        }
        stages = {
            "requested_motion": {
                "input": {
                    **requested_stage_input,
                },
                "recorded_output": {},
            },
            "resolver": {
                "input": {
                    **resolver_stage_input,
                },
                "recorded_output": {},
            },
            "guidance": {"input": guidance_input, "recorded_output": {}},
            "localization_gate": {"input": gate_input, "recorded_output": {}},
            "reference": {
                "input": {
                    "cycle_context": cycle_context,
                    "physical_command": physical_command,
                    "motion_envelope": motion_envelope,
                    "drive_capabilities": drive_capabilities,
                },
                "recorded_output": {},
            },
            "motion_executor": {"input": {}, "recorded_output": {}},
            "pwm": {"input": {}, "recorded_output": {}},
        }
        call = {
            "method": "compute",
            "cycle_context": cycle_context,
            "wheel_setpoint": {
                "contract_id": "R2B4_MOTION_PLATFORM_V2_1",
                "wheel_setpoint_id": f"wheel:{platform_cycle_id}",
                "physical_command_id": physical_command["physical_command_id"],
                "resolved_id": physical_command["resolved_id"],
                "cycle_id": platform_cycle_id,
                "left_target_mps": 0.0,
                "right_target_mps": 0.0,
                "feasible": True,
                "reason": "PLACEHOLDER",
                "applied_limits": [],
            },
            "wheel_feedback": _feedback(
                cycle_id=platform_cycle_id,
                mono_s=mono_s,
                measured=measured,
            ),
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
        call["wheel_setpoint"] = reference
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
        if wrong_stage == "guidance" and index == 3:
            stages["guidance"]["recorded_output"]["physical_command"]["v_mps"] = -0.17
            stages["reference"]["input"]["physical_command"]["v_mps"] = -0.17
        if wrong_stage == "reference" and index == 3:
            stages["reference"]["recorded_output"]["right_target_mps"] = -0.17
        if omit_stage and index == 2:
            stages.pop(omit_stage)
        executor_output = produced["executor_output"]
        accepted = recorder.record(
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
        if v21 and omit_stage and index == 2:
            assert accepted is False
        else:
            assert accepted is True
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
