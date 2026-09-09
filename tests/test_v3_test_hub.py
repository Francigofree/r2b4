import json
from pathlib import Path

import pytest
import v3.test_hub as test_hub_module

from v3.replay import ReplaySelection, replay_capture, verify_replay_result
from v3.test_hub import V3TestHubError, validate_run, verify_evidence
from v3_validation_helpers import create_explore_capture, create_fault_capture, create_general_capture


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_test_hub_delegates_to_the_canonical_replay_api(tmp_path, monkeypatch):
    capture = create_general_capture(tmp_path)
    calls = []
    canonical = test_hub_module.replay_capture

    def recording_replay(*args, **kwargs):
        calls.append((args, kwargs))
        return canonical(*args, **kwargs)

    monkeypatch.setattr(test_hub_module, "replay_capture", recording_replay)

    summary = validate_run(capture, tmp_path / "canonical-api")

    assert summary["replay_status"] == "MATCH"
    assert len(calls) == 1


def test_test_hub_writes_one_run_bound_replay_and_l1_l12_diagnosis(tmp_path):
    capture = create_explore_capture(tmp_path)
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


def test_test_hub_indexes_source_first_navigation_and_local_perception_coverage(tmp_path):
    capture = create_explore_capture(tmp_path)
    discovery = replay_capture(capture, project_root=PROJECT_ROOT)
    discovered_files = discovery["source_first"]["files"]
    manifest = tmp_path / "source-manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "files": {
                    path: {"sha256": row["sha256"]}
                    for path, row in discovered_files.items()
                }
            }
        ),
        encoding="utf-8",
    )
    output_dir = tmp_path / "source-first-navigation-run"

    summary = validate_run(
        capture,
        output_dir,
        project_root=PROJECT_ROOT,
        capture_source_manifest_path=manifest,
    )
    diagnosis = json.loads((output_dir / "diagnosis.json").read_text(encoding="utf-8"))
    evidence = json.loads((output_dir / "evidence_index.json").read_text(encoding="utf-8"))

    expected = {
        "conf/vezerles.json",
        "v3/adapters/latest_lidar.py",
        "v3/adapters/live_lidar.py",
        "v3/contracts/messages.py",
        "v3/layers/l4_world_model.py",
        "v3/layers/l5_command_mission.py",
        "v3/layers/l6_navigation.py",
        "v3/layers/l7_motion_selection.py",
        "v3/layers/l8_motion_realization.py",
    }
    assert summary["status"] == "PASS"
    assert diagnosis["source_first"]["all_capture_baseline_hashes_match"] is True
    assert expected <= set(diagnosis["source_first"]["files"])
    assert evidence["source_first"] == {
        "all_capture_baseline_hashes_match": True,
        "covered_file_count": len(discovered_files),
    }


def test_test_hub_fails_closed_on_a_declared_source_baseline_mismatch(tmp_path):
    capture = create_explore_capture(tmp_path)
    discovery = replay_capture(capture, project_root=PROJECT_ROOT)
    files = {
        path: {"sha256": row["sha256"]}
        for path, row in discovery["source_first"]["files"].items()
    }
    files["v3/layers/l6_navigation.py"] = {"sha256": "0" * 64}
    manifest = tmp_path / "mismatched-source-manifest.json"
    manifest.write_text(json.dumps({"files": files}), encoding="utf-8")

    summary = validate_run(
        capture,
        tmp_path / "source-mismatch-run",
        project_root=PROJECT_ROOT,
        capture_source_manifest_path=manifest,
    )

    assert summary["replay_status"] == "MATCH"
    assert summary["status"] == "FAIL"
