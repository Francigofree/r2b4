from __future__ import annotations

import json
from pathlib import Path
import sqlite3
from types import SimpleNamespace

import pytest

from tools.diag.context import DiagEvidenceError, resolve_evidence
from tools.diag.contracts import AnalyzerContract, AnalyzerOutput
from tools.diag.registry import AnalyzerRegistry, build_default_registry
from tools.mcap_evidence.index import DDL
from tools.mcap_evidence.reader import EvidenceBundle


def _indexed_bundle(tmp_path: Path) -> EvidenceBundle:
    root = tmp_path / "run.evidence"
    (root / "normalized" / "layers").mkdir(parents=True)
    view_path = root / "normalized" / "layers" / "L8_00000_00000.ndjson"
    row = {
        "message_id": "msg-1",
        "source_pointer": "/expected/layers/L8",
        "topic": "/r2b4/tick",
        "log_time_ns": 100,
        "payload": {"requested_v_mps": 0.1, "requested_omega_rad_s": 0.2},
    }
    raw = (json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n").encode()
    view_path.write_bytes(raw)
    (root / "source" / "messages").mkdir(parents=True)
    source_path = root / "source" / "messages" / "tick_00000_00000.ndjson"
    source_row = {
        "message_id": "msg-1", "topic": "/r2b4/tick", "channel_id": 1,
        "sequence": 1, "source_offset": 10, "log_time_ns": 100, "publish_time_ns": 100,
        "decode_status": "JSON", "payload_sha256": "x", "payload_base64": "e30=", "payload": {"tick_id": 7},
    }
    source_raw = (json.dumps(source_row, sort_keys=True, separators=(",", ":")) + "\n").encode()
    source_path.write_bytes(source_raw)
    db = sqlite3.connect(root / "index.sqlite")
    db.executescript(DDL)
    db.execute(
        "INSERT INTO messages VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
        ("msg-1", "/r2b4/tick", 1, 1, 10, f"{100:020d}", f"{100:020d}", "7",
         "source/messages/tick_00000_00000.ndjson", 0, len(source_raw), "JSON", "x"),
    )
    db.execute(
        "INSERT INTO views VALUES (?,?,?,?,?)",
        ("msg-1", "/expected/layers/L8", "normalized/layers/L8_00000_00000.ndjson", 0, len(raw)),
    )
    db.execute("INSERT INTO fields VALUES (?,?)", ("/tick_id", "msg-1"))
    db.commit(); db.close()
    return EvidenceBundle(
        root=root,
        manifest={}, coverage={}, integrity={}, verification={"status": "PASS", "source_checked": False},
    )


def test_evidence_reader_iterates_normalized_view_without_mcap(tmp_path: Path):
    bundle = _indexed_bundle(tmp_path)
    assert bundle.view_names() == ("layers/L8",)
    assert bundle.view_count("layers/L8") == 1
    rows = list(bundle.iter_view("layers/L8"))
    assert len(rows) == 1
    assert rows[0].tick_id == 7
    assert rows[0].payload["requested_omega_rad_s"] == 0.2
    assert bundle.view_alignment(("layers/L8",))["common_messages"] == 1


def test_resolve_evidence_defaults_to_latest_bundle(tmp_path: Path):
    capture_dir = tmp_path / "runtime" / "captures"
    older = capture_dir / "a.evidence"
    newer = capture_dir / "b.evidence"
    for bundle in (older, newer):
        bundle.mkdir(parents=True)
        (bundle / "manifest.json").write_text("{}")
    (older / "manifest.json").touch()
    import os
    os.utime(older / "manifest.json", ns=(1, 1))
    os.utime(newer / "manifest.json", ns=(2, 2))
    assert resolve_evidence(tmp_path, None) == newer.resolve()
    assert resolve_evidence(tmp_path, "latest") == newer.resolve()
    with pytest.raises(DiagEvidenceError):
        resolve_evidence(tmp_path, capture_dir / "x.mcap")


def test_default_registry_is_explicit_full_diag_order():
    assert build_default_registry().ids() == (
        "evidence_health",
        "execution_chain",
        "safety",
        "recovery",
        "navigation",
        "localization",
        "drive",
        "world_model",
        "lineage",
        "lifecycle",
    )


def test_tools_diag_has_no_mcap_reader_dependency():
    diag_dir = Path(__file__).resolve().parents[2] / "tools" / "diag"
    for path in diag_dir.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert "from v3.mcap_reader" not in text
        assert "import v3.mcap_reader" not in text
        assert "McapReader(" not in text



def test_registry_rejects_prescriptive_output_keys():
    registry = AnalyzerRegistry()
    contract = AnalyzerContract(analyzer_id="x", description="x")
    registry.register(contract, lambda context, owner: AnalyzerOutput(metrics={"recommendation": "x"}))
    context = SimpleNamespace(
        facts=SimpleNamespace(
            verification_status="PASS", view_counts={}, topics={}
        )
    )
    with pytest.raises(RuntimeError, match="prescriptive DIAG output key"):
        registry.run("x", context)


def test_localization_profiles_only_relevant_sensor_views():
    from tools.diag.analyzers.localization import _localization_sensor_views

    views = (
        "sensors/imu_abc",
        "sensors/encoder_left_def",
        "sensors/camera_rgb_ghi",
        "sensors/lidar_raw_jkl",
        "layers/L3",
    )
    assert _localization_sensor_views(views) == (
        "sensors/imu_abc",
        "sensors/encoder_left_def",
    )

def test_launcher_routes_diag_to_tools_diag_and_evidence_help():
    from v3 import host_cli, launcher_extras

    usage, description = host_cli.COMMAND_HELP["diag"]
    assert "EVIDENCE" in usage
    assert "evidence" in description.lower()
    source = Path(host_cli.__file__).read_text(encoding="utf-8")
    assert '"-m", "tools.diag"' in source
    assert "with hardware_guard(root):" in source
    completion_source = Path(launcher_extras.__file__).read_text(encoding="utf-8")
    assert "from tools.diag.registry import build_default_registry" in completion_source
    assert 'glob("*.evidence")' in completion_source


def test_diag_cli_no_args_means_full_latest(monkeypatch, tmp_path: Path, capsys):
    from tools.diag import cli

    (tmp_path / "v3").mkdir()
    (tmp_path / "tools" / "mcap_evidence").mkdir(parents=True)
    facts = SimpleNamespace(as_dict=lambda: {"path": "latest"})
    context = SimpleNamespace(facts=facts)
    called = {}

    class Result:
        admission = SimpleNamespace(applicable=True)

    class Report:
        results = (Result(),)
        def as_dict(self):
            return {"mode": "full", "evidence": "latest"}

    class Registry:
        def ids(self): return ("x",)
        def run_all(self, supplied):
            called["context"] = supplied
            return Report()
        def describe(self): return []

    monkeypatch.setattr(cli, "build_default_registry", lambda: Registry())
    monkeypatch.setattr(cli, "open_context", lambda root, value: called.update(value=value) or context)
    assert cli.main(["--json"], project_root=tmp_path) == 0
    assert called["value"] is None
    assert json.loads(capsys.readouterr().out)["mode"] == "full"
