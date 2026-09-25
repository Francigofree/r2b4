"""Portable/offline Test Hub upgrade tests."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import v3.test_hub as public_hub
import v3.test_hub_next as next_hub
import v3.test_hub_portable as portable
import v3.test_hub_runtime as runtime_handoff

from test_v3_mcap_e2e import capture


def _json_lines(path: Path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def test_portable_bundle_contains_remote_analysis_evidence(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    captured, _ = capture(source)
    destination = tmp_path / "portable.evidence"

    result = next_hub.run_default(
        captured.path,
        output_dir=destination,
        replay_mode="incident",
        replay_sweep_enabled=True,
        sweep_window_ticks=4,
    )

    assert result["status"] not in {"FAIL", "ERROR"}
    assert result["replay_status"] == "MATCH"
    expected = {
        "agent_view.json",
        "portable_manifest.json",
        "overview_10hz.ndjson",
        "timeline.ndjson",
        "lidar_summary.ndjson",
        "runtime_performance.json",
        "replay_sweep.json",
        "inspect.json",
        "triage.json",
        "diagnosis.json",
        "evidence_index.json",
    }
    assert expected <= {path.name for path in destination.iterdir()}

    sweep = json.loads((destination / "replay_sweep.json").read_text(encoding="utf-8"))
    assert sweep["status"] == "MATCH"
    assert sweep["first_tick"] == 0
    assert sweep["last_tick"] == 11
    assert sweep["window_count"] == 3
    assert sweep["covered_tick_span"] == [0, 11]
    assert all(row["status"] == "MATCH" for row in sweep["windows"])

    lidar_rows = _json_lines(destination / "lidar_summary.ndjson")
    assert lidar_rows[0]["point_arrays_in_file"] is False
    scan_rows = [row for row in lidar_rows if row.get("row_type") == "scan"]
    assert len(scan_rows) == 12
    assert all("points" not in row for row in scan_rows)
    assert all("tick_diagnostics" in row for row in scan_rows)

    agent = json.loads((destination / "agent_view.json").read_text(encoding="utf-8"))
    assert agent["authority"]["portable_remote_analysis"] is True
    assert agent["remote_analysis_policy"]["normal_analysis_requires_mcap"] is False
    assert agent["overview"] == "overview_10hz.ndjson"

    manifest = json.loads((destination / "portable_manifest.json").read_text(encoding="utf-8"))
    assert manifest["replay_sweep_status"] == "MATCH"
    assert manifest["capture"]["mcap_required_for_normal_remote_analysis"] is False
    assert "timeline.ndjson" in manifest["artifacts"]
    assert "lidar_summary.ndjson" in manifest["artifacts"]


def test_default_output_is_one_evidence_directory(tmp_path):
    capture_path = tmp_path / "run.mcap"
    assert next_hub.default_output_dir(capture_path) == tmp_path / "run.evidence"
    assert "evidence_next" not in str(next_hub.default_output_dir(capture_path))


def test_compare_works_from_portable_evidence_without_mcap(tmp_path, capsys):
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    first_dir.mkdir()
    second_dir.mkdir()
    first, _ = capture(first_dir)
    second, _ = capture(second_dir)
    first_evidence = tmp_path / "first.evidence"
    second_evidence = tmp_path / "second.evidence"

    next_hub.run_default(
        first.path,
        output_dir=first_evidence,
        replay_mode="off",
        replay_sweep_enabled=False,
    )
    next_hub.run_default(
        second.path,
        output_dir=second_evidence,
        replay_mode="off",
        replay_sweep_enabled=False,
    )
    first.path.unlink()
    second.path.unlink()

    assert next_hub.main(["compare", str(first_evidence), str(second_evidence)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["schema"] == "R2B4_AGENT_RUN_COMPARE_V1"
    assert all(
        value in (0, 0.0, None)
        for value in result["delta_after_minus_before"].values()
    )


def test_pytest_is_a_test_hub_command_not_a_runtime_dependency(monkeypatch, tmp_path):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout="7 passed in 0.10s\n", stderr="")

    monkeypatch.setattr(portable.subprocess, "run", fake_run)
    project_root = Path(__file__).resolve().parents[1]
    result = portable.run_pytest(project_root, scope="testhub")
    assert result["status"] == "PASS"
    assert result["scope"] == "testhub"
    assert result["command"][:4] == [portable.sys.executable, "-m", "pytest", "-q"]
    assert "tests/test_v3_test_hub_portable.py" in result["command"]
    assert calls[0][1]["cwd"] == project_root


def test_public_test_hub_routes_test_command_to_unified_cli(monkeypatch):
    called = []

    def fake_main(arguments):
        called.append(list(arguments))
        return 0

    monkeypatch.setattr(next_hub, "main", fake_main)
    assert public_hub.main(["test", "--scope", "testhub"]) == 0
    assert called == [["test", "--scope", "testhub"]]


def test_runtime_handoff_starts_only_the_offline_public_test_hub(monkeypatch, tmp_path):
    capture_path = tmp_path / "run.mcap"
    capture_path.write_bytes(b"placeholder")
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps({"status": "PASS", "replay_status": "MATCH"}),
            stderr="",
        )

    monkeypatch.setattr(runtime_handoff.subprocess, "run", fake_run)
    result = runtime_handoff.postprocess_capture(capture_path, project_root=tmp_path)
    assert result["status"] == "PASS"
    command = calls[0][0]
    assert command[:4] == [runtime_handoff.sys.executable, "-m", "v3.test_hub", "run"]
    assert "--pytest" in command and command[command.index("--pytest") + 1] == "off"
    assert str(capture_path.with_suffix(".evidence")) in command
    assert calls[0][1]["cwd"] == tmp_path.resolve()
