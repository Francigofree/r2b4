"""Launcher and common developer-tool integration without hardware/provider I/O."""
from __future__ import annotations

import json

import pytest

from r2b4_orchestration import learning_cli
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_orchestration.software_interface import SoftwareInterfaceAdapter
from v3 import launcher_cli, launcher_extras


def test_learning_help_catalog_completion_use_owner_without_provider(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(launcher_cli, "project_root", lambda: tmp_path)
    from v3 import runtime_performance
    monkeypatch.setattr(runtime_performance, "apply_host_affinity", lambda *args: None)
    monkeypatch.setattr(learning_cli, "run_learning", lambda *args, **kwargs: pytest.fail("help ran learning"))
    assert "learn" in launcher_cli._command_names()
    assert launcher_cli.main(["help", "learn"]) == 0
    assert "--scenario" in capsys.readouterr().out
    assert "learn" in launcher_extras.completion(1, ["lea"], tmp_path)[1]
    assert {"--scenario", "--max-iterations", "--evidence"} <= set(
        launcher_extras.completion(2, ["learn", "--"], tmp_path)[1])


def test_learning_cli_routes_full_goal_explicit_evidence_and_scenario(monkeypatch, tmp_path, capsys):
    scenario = tmp_path / "scenario.json"
    scenario.write_text('{"samples":[{"value":4}]}')
    seen = []

    def run(root, goal, **kwargs):
        seen.append((root, goal, kwargs))
        return {"status": "LEARNED", "skill": "watch", "provider_calls": 2,
                "artifact_directory": "/tmp/proof"}

    monkeypatch.setattr(learning_cli, "run_learning", run)
    assert learning_cli.main(["észleld", "az eseményt", "--scenario", str(scenario),
                              "--parameters", '{"threshold":4}', "--evidence", "trial.evidence",
                              "--capture", "trial.mcap", "--json"], project_root=tmp_path) == 0
    assert seen[0][1] == "észleld az eseményt"
    assert seen[0][2]["parameters"] == {"threshold": 4}
    assert seen[0][2]["scenario"] == {"samples": [{"value": 4}]}
    assert json.loads(capsys.readouterr().out)["provider_calls"] == 2


@pytest.mark.parametrize("status", ["INCOMPLETE", "FAILED", "TIMED_OUT", "CANCELLED"])
def test_unproven_learning_has_nonzero_exit(monkeypatch, tmp_path, capsys, status):
    monkeypatch.setattr(learning_cli, "run_learning", lambda *args, **kwargs: {
        "status": status, "artifact_directory": "/tmp/proof"})
    assert learning_cli.main(["observe", "--json"], project_root=tmp_path) == 1
    assert json.loads(capsys.readouterr().out)["status"] == status


def test_same_experiment_evidence_tools_are_published_to_agent_and_software(tmp_path):
    class Interface:
        pass
    agent = {spec.name for spec, _ in build_default_agent_tools(tmp_path, interface=Interface())}
    software = SoftwareInterfaceAdapter(tmp_path).capabilities()
    names = {"learning.trial", "evi.compile", "replay.run", "evidence.feedback"}
    assert names <= agent
    assert names <= software.keys()
