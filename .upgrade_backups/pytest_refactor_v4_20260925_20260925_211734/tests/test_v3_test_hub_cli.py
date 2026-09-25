"""Test Hub CLI/public entrypoint tests, grouped by responsibility."""

import json
from pathlib import Path

import pytest

from test_v3_mcap_e2e import capture
from v3.test_hub import main
from v3.test_hub_v2 import verify_evidence


@pytest.mark.parametrize(
    ("options", "exit_code", "replay_status"),
    [
        ({}, 0, "MATCH"),
        ({"capacity": 4}, 2, "ERROR"),
        ({"configuration": {}}, 2, "ERROR"),
    ],
)
def test_integrated_entrypoint_native_replay(
    tmp_path,
    capsys,
    options,
    exit_code,
    replay_status,
):
    captured, _ = capture(tmp_path, **options)
    destination = tmp_path / "next"
    assert main([
        "run",
        str(captured.path),
        "--output-dir",
        str(destination),
        "--replay",
        "full",
    ]) == exit_code
    result = json.loads(capsys.readouterr().out)
    assert result["replay_status"] == replay_status
    assert result["evidence_status"] == (
        "PASS" if exit_code == 0 else "FAIL"
    )
    assert verify_evidence(
        destination / "evidence_index.json"
    )["status"] == "PASS"
    agent = json.loads(Path(result["agent_view"]).read_text())
    assert agent["replay_status"] == replay_status
    rows = [
        json.loads(line)
        for line in Path(result["overview"]).read_text().splitlines()
    ]
    assert rows[0]["authority"]["derived_only"] is True
    times = [row.get("t_start_s", row.get("t_s", 0)) for row in rows[1:]]
    assert times == sorted(times)
    assert any(row["row_type"] == "window" for row in rows)
    if exit_code == 0:
        assert any(
            row.get("signal") == "safety_decision"
            for row in rows
        )
        assert rows[0]["data_coverage"]["/r2b4/tick"]["captured"] == 12


def test_no_arguments_selects_latest_capture(tmp_path, monkeypatch, capsys):
    captures = tmp_path / "runtime" / "captures"
    captures.mkdir(parents=True)
    captured, _ = capture(captures)
    monkeypatch.chdir(tmp_path)
    assert main([]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["capture"] == str(captured.path.resolve())
    assert Path(result["agent_view"]).is_file()


@pytest.mark.parametrize("hz", [1, 5, 10])
def test_view_and_compare_cli_preserve_existing_outputs(
    tmp_path,
    capsys,
    hz,
):
    captured, _ = capture(tmp_path)
    overview = tmp_path / "overview.ndjson"
    args = [
        "view",
        str(captured.path),
        "--hz",
        str(hz),
        "--output",
        str(overview),
    ]
    assert main(args) == 0
    assert json.loads(capsys.readouterr().out)["hz"] == hz
    original = overview.read_bytes()
    assert main(args) == 2
    capsys.readouterr()
    assert overview.read_bytes() == original
    assert main([
        "compare",
        str(captured.path),
        str(captured.path),
        "--hz",
        str(hz),
    ]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["verdict_policy"].startswith("No automatic")
    assert all(
        value in (0, None)
        for value in result["delta_after_minus_before"].values()
    )
