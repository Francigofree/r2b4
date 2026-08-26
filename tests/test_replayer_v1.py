import json
from pathlib import Path

from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor
from replayer.adapters import ProductionMotionExecutorAdapter, executor_contract_from_instance
from replayer.capture import CaptureRecorder, verify_capture
from replayer.replay import replay_capture, verify_replay_result


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _contract():
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


def _calls():
    base_feedback = {
        "v_l": 0.0,
        "v_r": 0.0,
        "v_l_encoder": 0.0,
        "v_r_encoder": 0.0,
        "v_l_encoder_raw": 0.0,
        "v_r_encoder_raw": 0.0,
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
        "active_command_layer": "MANUAL",
        "active_execution_mode": "TWIST_EXEC",
        "turn_primitive_requested": "ARC",
        "straight_hold_executor_candidate": False,
        "requested_v": 0.0,
        "requested_omega": 0.0,
    }
    rows = []
    for v_cmd, measured in ((0.0, 0.0), (0.18, 0.03), (0.18, 0.08), (0.0, 0.0)):
        feedback = dict(base_feedback)
        feedback.update(
            {
                "v_l": measured,
                "v_r": measured,
                "v_l_encoder": measured,
                "v_r_encoder": measured,
                "requested_v": v_cmd,
            }
        )
        rows.append(
            {
                "method": "compute_pwm",
                "kwargs": {
                    "v_cmd": v_cmd,
                    "omega_cmd": 0.0,
                    "sensor_feedback": feedback,
                    "dt": 0.02,
                    "execution_mode": "TWIST_EXEC",
                    "track_reference": {},
                },
            }
        )
    return rows


def _make_capture(tmp_path, *, capture_id="capture_test", wrong_output=False, times=None):
    data_root = tmp_path / "replayer_data"
    contract = _contract()
    speed_map = json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))
    producer = ProductionMotionExecutorAdapter(contract=contract, speed_map=speed_map)
    recorder = CaptureRecorder(
        project_root=PROJECT_ROOT,
        executor_contract=contract,
        data_root=data_root,
        capture_id=capture_id,
        queue_size=32,
        seal_read_only=False,
    )
    calls = _calls()
    timestamps = times or [1_000_000_000 + idx * 20_000_000 for idx in range(len(calls))]
    for index, (call, mono_ns) in enumerate(zip(calls, timestamps), start=1):
        replay_frame = {
            "monotonic_ns": mono_ns,
            "executor_call": call,
        }
        output = producer.replay_frame(replay_frame)
        if wrong_output and index == 2:
            output = dict(output)
            output["pwm_l"] += 0.01
        assert recorder.record(
            {
                "cycle_id": 100 + index,
                "monotonic_ns": mono_ns,
                "dt_s": 0.02,
                "executor_call": call,
                "recorded_executor_output": output,
                "final_output": {"pwm_l": output["pwm_l"], "pwm_r": output["pwm_r"]},
                "safety_lineage": {
                    "allow": True,
                    "reason": "OK",
                    "final_pwm_zero_reason": output["output_reason"],
                },
            }
        )
    recorder.close()
    return data_root, capture_id


def test_complete_capture_replays_with_production_executor_and_unique_results(tmp_path):
    data_root, capture_id = _make_capture(tmp_path)

    verification = verify_capture(capture_id, data_root=data_root)
    assert verification["valid"] is True
    assert verification["frame_count"] == 4

    first = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)
    second = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert first["status"] == "MATCH"
    assert first["diff"]["mismatch_count"] == 0
    assert first["result_id"] != second["result_id"]
    assert Path(first["result_path"]).parent == Path(second["result_path"]).parent
    assert verify_replay_result(capture_id, first["result_id"], data_root=data_root)["valid"] is True


def test_integrity_valid_but_behavior_wrong_is_mismatch(tmp_path):
    data_root, capture_id = _make_capture(tmp_path, capture_id="capture_mismatch", wrong_output=True)
    assert verify_capture(capture_id, data_root=data_root)["valid"] is True

    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert result["status"] == "MISMATCH"
    assert result["diff"]["mismatch_count"] == 1
    evidence = json.loads(Path(result["evidence_path"]).read_text(encoding="utf-8"))
    assert next(
        gate for gate in evidence["acceptance_gates"] if gate["gate"] == "executor_output_equivalence"
    )["status"] == "FAIL"


def test_executor_reset_generation_replays_production_reset_boundaries(tmp_path):
    data_root = tmp_path / "replayer_data"
    contract = _contract()
    speed_map = json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))
    producer = ProductionMotionExecutorAdapter(contract=contract, speed_map=speed_map)
    recorder = CaptureRecorder(
        project_root=PROJECT_ROOT,
        executor_contract=contract,
        data_root=data_root,
        capture_id="capture_with_executor_resets",
        queue_size=32,
        seal_read_only=False,
    )
    call = _calls()[1]
    cycle_id = 0
    for phase in range(2):
        if cycle_id:
            producer.executor.reset()
        reset_generation = int(producer.executor._replayer_reset_generation)
        assert reset_generation == phase
        for _ in range(5):
            cycle_id += 1
            mono_ns = 1_000_000_000 + cycle_id * 20_000_000
            frame = {
                "monotonic_ns": mono_ns,
                "executor_call": call,
                "executor_reset_generation": reset_generation,
            }
            output = producer.replay_frame(frame)
            assert recorder.record(
                {
                    "cycle_id": cycle_id,
                    "monotonic_ns": mono_ns,
                    "dt_s": 0.02,
                    "executor_reset_generation": reset_generation,
                    "executor_call": call,
                    "recorded_executor_output": output,
                    "final_output": {"pwm_l": output["pwm_l"], "pwm_r": output["pwm_r"]},
                    "safety_lineage": {
                        "allow": True,
                        "reason": "OK",
                        "final_pwm_zero_reason": output["output_reason"],
                    },
                }
            )
    recorder.close()

    verification = verify_capture("capture_with_executor_resets", data_root=data_root)
    result = replay_capture(
        "capture_with_executor_resets",
        data_root=data_root,
        project_root=PROJECT_ROOT,
    )

    assert verification["valid"] is True
    assert verification["timing"]["executor_reset_count"] == 1
    assert result["status"] == "MATCH"
    assert result["diff"]["mismatch_count"] == 0


def test_tampered_capture_can_never_match(tmp_path):
    data_root, capture_id = _make_capture(tmp_path, capture_id="capture_tampered")
    frames = data_root / "captures" / capture_id / "frames.jsonl"
    text = frames.read_text(encoding="utf-8")
    frames.write_text(text.replace('"pwm_l":', '"pwm_l":0.123,"tampered_pwm_l":', 1), encoding="utf-8")

    verification = verify_capture(capture_id, data_root=data_root)
    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)

    assert verification["valid"] is False
    assert result["status"] == "INVALID_CAPTURE"
    evidence = json.loads(Path(result["evidence_path"]).read_text(encoding="utf-8"))
    assert all(gate["status"] != "PASS" for gate in evidence["acceptance_gates"])


def test_active_and_insufficient_capture_fail_closed(tmp_path):
    data_root = tmp_path / "replayer_data"
    recorder = CaptureRecorder(
        project_root=PROJECT_ROOT,
        executor_contract=_contract(),
        data_root=data_root,
        capture_id="capture_incomplete",
        seal_read_only=False,
    )

    active = verify_capture("capture_incomplete", data_root=data_root)
    recorder.close()
    closed = verify_capture("capture_incomplete", data_root=data_root)
    result = replay_capture("capture_incomplete", data_root=data_root, project_root=PROJECT_ROOT)

    assert active["valid"] is False
    assert closed["valid"] is False
    assert "capture_not_complete:INVALID" in closed["errors"]
    assert result["status"] == "INVALID_CAPTURE"


def test_non_monotonic_capture_is_invalid(tmp_path):
    repeated = [1_000_000_000] * len(_calls())
    data_root, capture_id = _make_capture(
        tmp_path,
        capture_id="capture_bad_time",
        times=repeated,
    )
    verification = verify_capture(capture_id, data_root=data_root)

    assert verification["valid"] is False
    assert any("time" in error or "monotonic" in error for error in verification["errors"])


def test_runtime_failure_reason_forces_invalid_even_with_complete_frames(tmp_path):
    data_root = tmp_path / "replayer_data"
    contract = _contract()
    speed_map = json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))
    producer = ProductionMotionExecutorAdapter(contract=contract, speed_map=speed_map)
    recorder = CaptureRecorder(
        project_root=PROJECT_ROOT,
        executor_contract=contract,
        data_root=data_root,
        capture_id="capture_runtime_failure",
        seal_read_only=False,
    )
    for index, call in enumerate(_calls()[:2], start=1):
        mono_ns = 1_000_000_000 + index * 20_000_000
        output = producer.replay_frame({"monotonic_ns": mono_ns, "executor_call": call})
        recorder.record(
            {
                "cycle_id": index,
                "monotonic_ns": mono_ns,
                "dt_s": 0.02,
                "executor_call": call,
                "recorded_executor_output": output,
                "final_output": {"pwm_l": output["pwm_l"], "pwm_r": output["pwm_r"]},
                "safety_lineage": {"allow": True, "reason": "OK", "final_pwm_zero_reason": "NONE"},
            }
        )
    recorder.close(invalid_reason="control_loop_exception")

    verification = verify_capture("capture_runtime_failure", data_root=data_root)

    assert verification["valid"] is False
    assert "capture_not_complete:INVALID" in verification["errors"]


def test_replay_result_integrity_detects_evidence_tamper(tmp_path):
    data_root, capture_id = _make_capture(tmp_path, capture_id="capture_result_tamper")
    result = replay_capture(capture_id, data_root=data_root, project_root=PROJECT_ROOT)
    evidence_path = Path(result["evidence_path"])
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["status"] = "MISMATCH"
    evidence_path.write_text(json.dumps(evidence), encoding="utf-8")

    verification = verify_replay_result(capture_id, result["result_id"], data_root=data_root)

    assert verification["valid"] is False
    assert "evidence_hash_invalid" in verification["errors"]
