import json

import pytest

from v3.replay import ReplaySelection, verify_replay_result
from v3.test_hub import V3TestHubError, validate_run, verify_evidence
from v3_validation_helpers import create_fault_capture, create_general_capture


def test_test_hub_writes_one_run_bound_replay_and_l1_l12_diagnosis(tmp_path):
    capture = create_general_capture(tmp_path)
    output_dir = tmp_path / "run-20260906T090000Z"

    summary = validate_run(
        capture,
        output_dir,
        selection=ReplaySelection(
            start_tick_id=2,
            end_tick_id=3,
            start_layer="L3",
            end_layer="L10",
        ),
    )

    assert summary["status"] == "PASS"
    assert summary["replay_status"] == "MATCH"
    assert set(path.name for path in output_dir.iterdir()) == {
        "diagnosis.json",
        "evidence_index.json",
        "inspect.json",
        "replay_result.json",
    }
    diagnosis = json.loads((output_dir / "diagnosis.json").read_text(encoding="utf-8"))
    evidence = json.loads((output_dir / "evidence_index.json").read_text(encoding="utf-8"))
    assert set(diagnosis["layers"]) == {f"L{index}" for index in range(1, 13)}
    assert diagnosis["layers"]["L1"]["authority"] == "OUT_OF_SCOPE"
    assert diagnosis["layers"]["L3"]["authority"] == "REPLAYER_V3"
    assert diagnosis["scope"]["state_warmup"]["tick_count"] == 2
    assert evidence["run_id"] == output_dir.name
    assert "latest" not in json.dumps(evidence).lower()
    assert verify_replay_result(output_dir / "replay_result.json")["status"] == "PASS"
    assert verify_evidence(output_dir / "evidence_index.json")["status"] == "PASS"


def test_test_hub_refuses_to_overwrite_a_run_directory(tmp_path):
    capture = create_general_capture(tmp_path)
    output_dir = tmp_path / "existing-run"
    output_dir.mkdir()
    (output_dir / "owned.txt").write_text("keep", encoding="utf-8")

    with pytest.raises(V3TestHubError, match="absent or empty"):
        validate_run(capture, output_dir)


def test_evidence_verifier_rejects_a_modified_diagnosis(tmp_path):
    capture = create_general_capture(tmp_path)
    output_dir = tmp_path / "tampered-run"
    validate_run(capture, output_dir)
    diagnosis = output_dir / "diagnosis.json"
    diagnosis.write_text(diagnosis.read_text(encoding="utf-8") + " ", encoding="utf-8")

    result = verify_evidence(output_dir / "evidence_index.json")

    assert result["status"] == "FAIL"
    assert result["artifacts"]["diagnosis.json"]["matches_index"] is False


def test_test_hub_preserves_fault_verdict_but_passes_matching_partial_replay(tmp_path):
    capture = create_fault_capture(tmp_path)
    output_dir = tmp_path / "fault-run"

    summary = validate_run(capture, output_dir)
    diagnosis = json.loads((output_dir / "diagnosis.json").read_text(encoding="utf-8"))

    assert summary["status"] == "PASS"
    assert summary["capture_status"] == "FAULT"
    assert summary["replay_status"] == "MATCH"
    assert diagnosis["capture_execution_status"] == "FAULT"
    assert diagnosis["layers"]["L4"]["not_executed_tick_count"] == 1
    assert diagnosis["layers"]["L12"]["compared_tick_count"] == 1
