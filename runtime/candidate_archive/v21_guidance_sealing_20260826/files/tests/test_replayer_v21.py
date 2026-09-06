import json

from replayer.capture import inspect_capture, verify_capture
from replayer.cli import build_parser, main as replayer_main
from replayer.contracts import (
    CAPTURE_SCHEMA_V21,
    DIAGNOSIS_SCHEMA_V21,
    LAYER_L6_INTENT_RESOLVER,
    LAYER_L7A_MOTION_GUIDANCE,
    LAYER_L8_MOTION_CONTROLLER,
    LAYER_L9_MOTION_EXECUTOR,
    REPLAY_RESULT_SCHEMA_V21,
)
from replayer.replay import replay_capture, verify_replay_result
from tests.test_replayer_v2 import PROJECT_ROOT, _make_v2_capture


def test_v21_cli_advertises_resolver_and_guidance_layer_selection():
    parser = build_parser()
    replay_parser = next(
        action.choices["replay"]
        for action in parser._actions
        if getattr(action, "dest", "") == "command"
    )
    layer_action = next(
        action for action in replay_parser._actions if action.dest == "layer"
    )

    assert "L6" in layer_action.help
    assert "L7A" in layer_action.help


def test_v21_replays_sealed_boundaries_and_seals_diagnosis(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_boundaries",
        v21=True,
    )

    verification = verify_capture(capture_id, data_root=data_root)
    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
    )
    diagnosis = json.loads(
        (data_root / "results" / capture_id / result["result_id"] / "diagnosis.json")
        .read_text(encoding="utf-8")
    )

    assert verification["valid"] is True
    assert verification["manifest"]["schema"] == CAPTURE_SCHEMA_V21
    assert result["schema"] == REPLAY_RESULT_SCHEMA_V21
    assert result["status"] == "MATCH"
    assert result["diff"]["layer_mismatch_counts"] == {
        LAYER_L6_INTENT_RESOLVER: 0,
        LAYER_L7A_MOTION_GUIDANCE: 0,
        LAYER_L8_MOTION_CONTROLLER: 0,
        LAYER_L9_MOTION_EXECUTOR: 0,
        "SERVICE_ACTUATION": 0,
    }
    assert diagnosis["schema"] == DIAGNOSIS_SCHEMA_V21
    assert diagnosis["first_divergence"] is None
    assert verify_replay_result(
        capture_id,
        result["result_id"],
        data_root=data_root,
    )["valid"] is True


def test_v21_targeted_l9_window_uses_complete_prefix_warmup(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_l9_window",
        v21=True,
    )

    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
        start_monotonic_ns=1_060_000_000,
        end_monotonic_ns=1_080_000_000,
        layers=["L9"],
    )

    scope = result["diff"]["replay_scope"]
    assert result["status"] == "MATCH"
    assert scope["layers"] == [LAYER_L9_MOTION_EXECUTOR]
    assert scope["warmup_frame_count"] == 2
    assert scope["warmup_layer_counts"] == {LAYER_L9_MOTION_EXECUTOR: 2}
    assert scope["selected_frame_count"] == 2
    assert scope["layer_evaluation_count"] == 2
    assert result["diff"]["replayed_frame_count"] == 2


def test_v21_replays_resolver_and_guidance_boundaries_before_l8(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_l6_l7a",
        v21=True,
    )

    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
        layers=["L6", "L7A"],
    )

    assert result["status"] == "MATCH"
    assert result["diff"]["replay_scope"]["layers"] == [
        LAYER_L6_INTENT_RESOLVER,
        LAYER_L7A_MOTION_GUIDANCE,
    ]
    assert result["diff"]["layer_mismatch_counts"] == {
        LAYER_L6_INTENT_RESOLVER: 0,
        LAYER_L7A_MOTION_GUIDANCE: 0,
    }


def test_v21_reports_guidance_as_the_first_divergent_layer(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_l7a_diagnosis",
        wrong_stage="guidance",
        v21=True,
    )

    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
        layers=["L7A"],
    )

    assert result["status"] == "MISMATCH"
    first = result["diff"]["first_divergence"]
    assert first["layer"] == LAYER_L7A_MOTION_GUIDANCE
    assert first["expected_output"]["physical_command"]["v_mps"] == -0.17


def test_v21_diagnosis_contains_first_layer_boundary_divergence(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_l8_diagnosis",
        wrong_stage="reference",
        v21=True,
    )

    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
        start_monotonic_ns=1_060_000_000,
        layers=["L8"],
    )
    diagnosis = json.loads(
        (data_root / "results" / capture_id / result["result_id"] / "diagnosis.json")
        .read_text(encoding="utf-8")
    )
    first = diagnosis["first_divergence"]

    assert result["status"] == "MISMATCH"
    assert first["monotonic_ns"] == 1_060_000_000
    assert first["layer"] == LAYER_L8_MOTION_CONTROLLER
    assert first["input"]["physical_command"]["physical_mode"] == "BODY_TWIST"
    assert first["expected_output"]["right_target_mps"] == -0.17
    assert first["actual_output"]["right_target_mps"] != -0.17
    assert first["relevant_state"]["prefix_warmup_frames_applied"] == 2
    assert "layer_state_after_replay" in first["relevant_state"]


def test_v21_inspect_is_fast_manifest_only_and_never_claims_acceptance(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_inspect",
        v21=True,
    )

    summary = inspect_capture(capture_id, data_root=data_root)

    assert summary["capture_schema"] == CAPTURE_SCHEMA_V21
    assert summary["frame_count"] == 4
    assert summary["available_layers"][0] == LAYER_L6_INTENT_RESOLVER
    assert summary["verification_scope"] == "MANIFEST_ONLY"
    assert summary["replay_acceptance"] == "NOT_EVALUATED_USE_VERIFY_OR_REPLAY"
    assert summary["manifest_integrity"] == "VALID"


def test_v21_inspect_cli_exposes_manifest_only_scope(tmp_path, capsys):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_inspect_cli",
        v21=True,
    )

    exit_code = replayer_main(
        ["--data-root", str(data_root), "inspect", capture_id]
    )
    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["capture_schema"] == CAPTURE_SCHEMA_V21
    assert payload["verification_scope"] == "MANIFEST_ONLY"


def test_v21_missing_boundary_cannot_verify_or_replay_as_pass(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_missing_boundary",
        omit_stage="reference",
        v21=True,
    )

    verification = verify_capture(capture_id, data_root=data_root)
    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
    )

    assert verification["valid"] is False
    assert any(
        "layer_boundary_capture_failed" in error
        or "capture_not_complete" in error
        for error in verification["errors"]
    )
    assert result["status"] == "INVALID_CAPTURE"
    assert result["status"] != "MATCH"


def test_v21_empty_target_window_cannot_return_match(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_empty_window",
        v21=True,
    )

    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
        start_monotonic_ns=9_000_000_000,
        layers=["L8"],
    )

    assert result["status"] == "ERROR"
    assert result["status"] != "MATCH"
    assert result["diff"]["expected_frame_count"] == 0


def test_v21_diagnosis_tamper_invalidates_result_integrity(tmp_path):
    data_root, capture_id = _make_v2_capture(
        tmp_path,
        capture_id="capture_v21_diagnosis_integrity",
        v21=True,
    )
    result = replay_capture(
        capture_id,
        data_root=data_root,
        project_root=PROJECT_ROOT,
    )
    diagnosis_path = (
        data_root / "results" / capture_id / result["result_id"] / "diagnosis.json"
    )
    diagnosis_path.write_text("{}\n", encoding="utf-8")

    verification = verify_replay_result(
        capture_id,
        result["result_id"],
        data_root=data_root,
    )

    assert verification["valid"] is False
    assert any("artifact_integrity_mismatch:diagnosis.json" in error for error in verification["errors"])
