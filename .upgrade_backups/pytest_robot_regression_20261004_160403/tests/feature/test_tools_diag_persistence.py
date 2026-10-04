from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace


def _fake_cli(monkeypatch, cli, *, tmp_path: Path):
    facts = SimpleNamespace(
        path=tmp_path / "runtime" / "captures" / "run.evidence",
        as_dict=lambda: {"path": "runtime/captures/run.evidence"},
    )
    context = SimpleNamespace(facts=facts)

    class Result:
        admission = SimpleNamespace(applicable=True)

    class Report:
        results = (Result(),)

        def as_dict(self):
            return {
                "schema": "R2B4_DIAG_FULL_REPORT_V1",
                "purpose": "DIAGNOSTIC_DATA_ONLY",
                "source": "EVI_EVIDENCE_ONLY",
                "evidence": facts.as_dict(),
                "analyzers": [],
                "root_cause_inferred": False,
            }

    class Registry:
        def ids(self):
            return ("x",)

        def describe(self):
            return []

        def run_all(self, supplied):
            assert supplied is context
            return Report()

    monkeypatch.setattr(cli, "build_default_registry", lambda: Registry())
    monkeypatch.setattr(cli, "open_context", lambda root, value: context)


def test_persist_payload_atomically_writes_under_runtime_diag(tmp_path: Path):
    from tools.diag.persistence import persist_payload

    payload = {
        "schema": "R2B4_DIAG_FULL_REPORT_V1",
        "purpose": "DIAGNOSTIC_DATA_ONLY",
        "root_cause_inferred": False,
    }
    path = persist_payload(
        tmp_path,
        payload,
        mode="full",
        evidence_path=tmp_path / "runtime" / "captures" / "sample.evidence",
    )

    assert path.parent == tmp_path / "runtime" / "diag"
    assert path.name.startswith("diag_")
    assert "_full_sample_" in path.name
    assert json.loads(path.read_text(encoding="utf-8")) == payload
    assert not list(path.parent.glob(".diag-*.tmp"))


def test_diag_json_stdout_stays_machine_readable_while_default_is_saved(
    monkeypatch, tmp_path: Path, capsys
):
    from tools.diag import cli

    (tmp_path / "v3").mkdir()
    (tmp_path / "tools" / "mcap_evidence").mkdir(parents=True)
    _fake_cli(monkeypatch, cli, tmp_path=tmp_path)

    assert cli.main(["--json"], project_root=tmp_path) == 0
    captured = capsys.readouterr()
    stdout_payload = json.loads(captured.out)
    assert stdout_payload["schema"] == "R2B4_DIAG_FULL_REPORT_V1"
    assert "diag artifact:" in captured.err

    artifacts = list((tmp_path / "runtime" / "diag").glob("diag_*.json"))
    assert len(artifacts) == 1
    assert json.loads(artifacts[0].read_text(encoding="utf-8")) == stdout_payload


def test_diag_no_save_is_explicit_ephemeral_mode(monkeypatch, tmp_path: Path, capsys):
    from tools.diag import cli

    (tmp_path / "v3").mkdir()
    (tmp_path / "tools" / "mcap_evidence").mkdir(parents=True)
    _fake_cli(monkeypatch, cli, tmp_path=tmp_path)

    assert cli.main(["--json", "--no-save"], project_root=tmp_path) == 0
    captured = capsys.readouterr()
    assert json.loads(captured.out)["schema"] == "R2B4_DIAG_FULL_REPORT_V1"
    assert "diag artifact:" not in captured.err
    assert not (tmp_path / "runtime" / "diag").exists()


def test_launcher_surface_exposes_persistence_controls(tmp_path: Path):
    from v3 import host_cli, launcher_cli, launcher_extras

    usage, description = host_cli.COMMAND_HELP["diag"]
    assert "--no-save" in usage
    assert "runtime/diag" in description

    hint, candidates = launcher_extras._host_completion("diag", [], "--", tmp_path)
    assert "--no-save" in candidates
    assert "--json" in candidates
    assert "runtime/diag" in hint
    assert "runtime/diag" in Path(launcher_cli.__file__).read_text(encoding="utf-8")
