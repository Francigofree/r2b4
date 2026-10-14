from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
import sqlite3
from types import SimpleNamespace

import pytest

from tools.diag.context import DiagEvidenceError, resolve_evidence
from tools.diag.contracts import AnalyzerContract, AnalyzerOutput
from tools.diag.registry import AnalyzerRegistry
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


def test_compact_navigation_lineage_survives_mcap_evi_and_diag(tmp_path: Path):
    from rig import ROOT, resolved_config
    from test_resident_status_freshness import _navigation_tick
    from test_spatial_service import Clock
    from tools.diag.context import open_context
    from tools.diag.registry import build_default_registry
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify
    from v3.capture_behavior import encode_behavior_record, project_behavior_record
    from v3.contracts import (CommandMode, CommandRequest, LifecycleState, MissionIntent,
                             MissionLifecycle, NavigationStatus, RawDeviceBatch, VelocityTarget)
    from v3.engine import LayerRecord, TickInputs
    from v3.execution import ExecutionRecord
    from v3.layers.l7_motion_selection import MotionSelector
    from v3.layers.l8_motion_realization import MotionRealizer
    from v3.mcap_writer import StdlibMcapWriter
    from v3.resident_status import _tick_status

    # This is passive evidence plumbing, using typed completed downstream
    # values. It does not claim a complete control replay or physical motion.
    clock = Clock()
    config = resolved_config().runtime.composition.live_control.control
    selector, realizer = MotionSelector(config.motion_selection), MotionRealizer(config.motion_realization)
    first = _navigation_tick(clock)
    validity = first.trace.layers[-1].output.motion_validity
    progress = first.trace.layers[-1].output.progress
    assert progress > 0.0
    upper_progress = []
    source = tmp_path / "navigation.mcap"
    with source.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        channel = writer.register_channel("/r2b4/tick")
        for tick_id in (1, 2, 3):
            if tick_id > 1:
                clock.now += 20_000_000
            completed = _navigation_tick(clock, tick=tick_id, validity=validity)
            estimate, world, plan = (layer.output for layer in completed.trace.layers)
            if tick_id > 1:
                world = replace(world, map_revision=world.map_revision + 1,
                    local_costmap=replace(world.local_costmap, revision=8, source_sequence=9))
                plan = replace(plan, goal_selection_reason="GOAL_HANDOFF")
            plan = replace(plan, status=NavigationStatus.ACTIVE if tick_id < 3 else NavigationStatus.PENDING,
                reason=(None, "LOCAL_GUIDANCE_REVALIDATED", "GEOMETRY_HOLD")[tick_id - 1],
                velocity_target=VelocityTarget(.15, 0.0) if tick_id < 3 else None)
            mission = MissionIntent(plan.context, plan.mission_id, CommandMode.EXPLORE,
                                    None, None, plan.constraints, MissionLifecycle.ACTIVE)
            objective = selector.evaluate(plan)
            motion = realizer.evaluate(objective, estimate, world)
            completed = replace(completed, trace=replace(completed.trace, layers=(
                LayerRecord("L3", estimate), LayerRecord("L4", world), LayerRecord("L5", mission),
                LayerRecord("L6", plan), LayerRecord("L7", objective), LayerRecord("L8", motion),
            )))
            inputs = TickInputs(plan.context, RawDeviceBatch(plan.context, (), ()),
                CommandRequest(plan.context, "nav", CommandMode.EXPLORE, (), tick_id), LifecycleState.ACTIVE)
            payload = encode_behavior_record(project_behavior_record(ExecutionRecord(inputs, completed)))
            upper_progress.append(_tick_status(completed)["navigation"]["progress"])
            assert payload["expected"]["layers"]["L6"]["progress"] == upper_progress[-1] == progress
            assert "occupied_cells" not in payload["expected"]["layers"]["L4"]["local_costmap"]
            assert "trajectory_candidates" not in payload["expected"]["layers"]["L6"]
            writer.add_message(channel, log_time_ns=plan.context.monotonic_ns,
                               data=json.dumps(payload).encode(), sequence=tick_id)
        writer.finish()
    original = source.read_bytes()
    compiled = compile_evidence(source, tmp_path / "navigation.evidence", workers=1)
    assert compiled["compiler_status"] == "COMPLETE"
    assert verify(compiled["output"], source=source)["status"] == "PASS"
    indexed = list(query(compiled["output"], field="/expected/layers/L6/motion_validity/geometry_revision"))
    assert len(indexed) == 3
    context = open_context(ROOT, compiled["output"])
    navigation_rows = list(context.bundle.iter_view("layers/L6"))
    assert [row.payload["progress"] for row in navigation_rows] == upper_progress == [progress] * 3
    assert navigation_rows[-1].payload["reason"] == "GEOMETRY_HOLD"
    registry = build_default_registry()
    for analyzer_id in ("navigation", "world_model"):
        analyzed = registry.run(analyzer_id, context)
        assert analyzed.admission.applicable
        profiles = analyzed.metrics["profiles"]
        l4, l6 = profiles["layers/L4"]["fields"], profiles["layers/L6"]["fields"]
        assert l4["/local_costmap/revision"]["change_count"] == 1
        assert l4["/local_costmap/source_sequence"]["change_count"] == 1
        assert l6["/goal_selection_reason"]["change_count"] == 1
        if analyzer_id == "navigation":
            assert l6["/progress"]["change_count"] == 0
            assert l6["/progress"]["numeric"]["min"] == l6["/progress"]["numeric"]["max"] == progress
        assert l6["/motion_validity/geometry_captured_ns"]["change_count"] == 0
        assert l6["/motion_validity/valid_until_ns"]["change_count"] == 0
        reference = l6["/goal_selection_reason"]["examples"][0]["evidence_ref"]
        assert reference["source_pointer"] == "/expected/layers/L6"
        assert reference["field_path"] == "/goal_selection_reason"
    assert source.read_bytes() == original
