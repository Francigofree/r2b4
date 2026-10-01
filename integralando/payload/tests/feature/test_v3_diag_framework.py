from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from v3.diag.admission import evaluate_admission
from v3.diag.context import DiagCaptureError, resolve_capture
from v3.diag.contracts import AnalyzerContract, CaptureFacts, FieldRef
from v3.diag.coverage import collect_capture_coverage
from v3.diag.registry import AnalyzerRegistry, build_default_registry
from v3.diagnostic_contracts import contract_from_dataclass
from v3.mcap_reader import TICK_TOPIC


class FakeReader:
    def __init__(self, ticks):
        self._ticks = list(ticks)

    def iter_json_messages(self, *, topics):
        assert topics == (TICK_TOPIC,)
        for index, tick in enumerate(self._ticks):
            yield SimpleNamespace(log_time_ns=index), tick


def _sample(kind: str, **values):
    return {
        "__type__": "DeviceSample",
        "device_id": kind,
        "kind": kind,
        "sequence": 1,
        "captured_monotonic_ns": 1,
        "values": [{"key": key, "value": value} for key, value in values.items()],
    }


def test_default_registry_is_explicit_and_minimal():
    registry = build_default_registry()
    assert [item.contract.analyzer_id for item in registry] == ["capture", "coverage"]
    assert registry.semantic_owners() == {}
    assert registry.semantic_path_owners() == {}


def test_registry_rejects_duplicate_ids():
    registry = AnalyzerRegistry()
    contract = AnalyzerContract(analyzer_id="x", description="x")
    registry.register(contract, lambda context, owner: None)
    with pytest.raises(ValueError, match="duplicate"):
        registry.register(contract, lambda context, owner: None)


def test_capture_coverage_auto_observes_new_production_dataclass_field():
    @dataclass(frozen=True)
    class FuturePayload:
        left_mps: float
        future_quality: float

    contract = contract_from_dataclass(
        contract_id="future.sensor",
        target="SENSOR_SAMPLE",
        selector="future_kind",
        payload_type=FuturePayload,
        criticality="PRODUCTION_CRITICAL",
    )
    reader = FakeReader([
        {"inputs": {"sensor": _sample("future_kind", left_mps=0.1, future_quality=0.7)}}
    ])
    coverage = collect_capture_coverage(reader, contracts=(contract,))
    assert coverage.tick_count == 1
    assert "inputs.sensor.values[].value" in coverage.generic_tick_paths
    assert coverage.observed_fields("future.sensor") == frozenset({"left_mps", "future_quality"})
    assert "future_quality" in contract.field_names


def test_admission_is_fail_closed_for_missing_topic_rate_raw_and_field():
    facts = CaptureFacts(
        path=Path("capture.mcap"),
        file_size=1,
        mcap_profile="r2b4",
        mcap_library="test",
        tick_sample_hz=10,
        raw_evidence_requested=False,
        raw_evidence_complete=False,
        topics=(TICK_TOPIC,),
        message_count=1,
        message_start_time_ns=1,
        message_end_time_ns=2,
        data_crc_ok=True,
        summary_crc_ok=True,
        integrity_complete=True,
    )
    context = SimpleNamespace(
        facts=facts,
        coverage=lambda: SimpleNamespace(
            observed_fields=lambda contract_id: frozenset(),
            has_path=lambda path: False,
        ),
    )
    contract = AnalyzerContract(
        analyzer_id="deep",
        description="deep",
        required_topics=(TICK_TOPIC, "/r2b4/raw_lidar"),
        min_tick_sample_hz=50,
        requires_raw_evidence=True,
        required_fields=(FieldRef("encoder.wheel_velocity", "left_mps"),),
        required_paths=("expected.layers.L6.future_field",),
    )
    decision = evaluate_admission(contract, context)
    assert decision.state.value == "INSUFFICIENT_EVIDENCE"
    text = " ".join(decision.reasons)
    assert "/r2b4/raw_lidar" in text
    assert "below required 50 Hz" in text
    assert "raw MCAP" in text
    assert "encoder.wheel_velocity.left_mps" in text
    assert "expected.layers.L6.future_field" in text


def test_resolve_capture_accepts_only_mcap(tmp_path: Path):
    (tmp_path / "runtime" / "captures").mkdir(parents=True)
    mcap = tmp_path / "runtime" / "captures" / "x.mcap"
    mcap.write_bytes(b"x")
    assert resolve_capture(tmp_path, "latest") == mcap
    json_path = tmp_path / "runtime" / "captures" / "x.json"
    json_path.write_text("{}")
    with pytest.raises(DiagCaptureError, match="MCAP input only"):
        resolve_capture(tmp_path, json_path)


def test_diag_source_does_not_depend_on_test_hub_or_derived_evidence():
    diag_dir = Path(__file__).resolve().parents[2] / "v3" / "diag"
    for path in diag_dir.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert "test_hub" not in text
        assert ".evidence/" not in text
        assert ".evidence\\" not in text


def test_host_launcher_registers_diag_command():
    from v3 import host_cli

    assert "diag" in host_cli.COMMANDS
    usage, description = host_cli.COMMAND_HELP["diag"]
    assert "ANALYZER" in usage
    assert "MCAP" in description

    from v3 import interface_cli
    assert interface_cli.ALIASES["d"] == "diag"
    assert "d" not in host_cli.COMMANDS

    from v3 import launcher_cli
    robot_names = {item["name"] for item in launcher_cli.command_catalog()["robot"]}
    assert "diag" not in robot_names
    assert "diag" in launcher_cli.command_catalog()["local"]


def test_launcher_completion_uses_diag_registry(tmp_path: Path):
    from v3 import launcher_extras

    hint, candidates = launcher_extras._host_completion("diag", [], "", tmp_path)
    assert "r diag" in hint
    assert {"list", "admission", "capture", "coverage"}.issubset(set(candidates))


def test_diag_refuses_analyzer_execution_while_runtime_is_active(tmp_path: Path, monkeypatch, capsys):
    from contextlib import contextmanager
    from v3.diag import cli as diag_cli

    (tmp_path / "v3").mkdir()

    class Controller:
        def __init__(self, root):
            self.root = root

        @contextmanager
        def operator_transition(self):
            yield

        def status(self):
            return {"runtime_running": True}

    monkeypatch.setattr(diag_cli, "OperatorController", Controller)
    rc = diag_cli.main(["capture"], project_root=tmp_path)
    assert rc == 3
    assert "offline-only" in capsys.readouterr().err
