from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.diag.admission import evaluate_admission
from tools.diag.basis import view_scan
from tools.diag.contracts import (
    AnalyzerContract,
    DiagProducerFacts,
    DiagnosticObservation,
    EvidenceFacts,
    ObservationKind,
)
from tools.diag.episodes import contiguous_field_episodes
from tools.diag.profiling import profile_view
from tools.diag.registry import build_default_registry
from tools.mcap_evidence.reader import EvidenceViewRow


class FakeBundle:
    def __init__(self, rows: dict[str, list[EvidenceViewRow]]):
        self.rows = rows

    def iter_view(self, view: str, **kwargs):
        del kwargs
        yield from self.rows.get(view, ())

    def has_view(self, view: str) -> bool:
        return view in self.rows

    def view_names(self):
        return tuple(sorted(self.rows))

    def view_count(self, view: str) -> int:
        return len(self.rows.get(view, ()))

    def view_alignment(self, views):
        ids = [{row.message_id for row in self.rows.get(view, ())} for view in views]
        common = set.intersection(*ids) if ids else set()
        return {
            "views": list(views),
            "common_messages": len(common),
            "per_view_messages": {view: len(self.rows.get(view, ())) for view in views},
        }


def row(view: str, message_id: str, tick: int, payload: dict[str, object]) -> EvidenceViewRow:
    return EvidenceViewRow(
        view=view,
        message_id=message_id,
        source_pointer=f"/expected/{view}",
        topic="/r2b4/tick",
        log_time_ns=1_000 + tick * 100,
        tick_id=tick,
        payload=payload,
    )


def test_array_members_do_not_become_temporal_transitions():
    bundle = FakeBundle({
        "runtime": [row("runtime", "m1", 1, {"affinity": {"cpus": [1, 2]}})]
    })
    profile = profile_view(bundle, "runtime", tokens=("cpus",))
    stats = profile["fields"]["/affinity/cpus[]"]
    assert stats["count"] == 2
    assert stats["rows_present"] == 1
    assert stats["change_count"] == 0
    assert stats["change_semantics"] == "ROW_TO_ROW_VALUE_COLLECTION"

    bundle = FakeBundle({
        "runtime": [
            row("runtime", "m1", 1, {"affinity": {"cpus": [1, 2]}}),
            row("runtime", "m2", 2, {"affinity": {"cpus": [1, 3]}}),
        ]
    })
    profile = profile_view(bundle, "runtime", tokens=("cpus",))
    assert profile["fields"]["/affinity/cpus[]"]["change_count"] == 1


def test_recovery_tokenizer_rejects_configuration_type_and_not_active_false_positive():
    from tools.diag.analyzers.recovery import _semantic_tokens, _state_path

    assert _semantic_tokens("PlannerRecoveryPolicy") == set()
    assert "ACTIVE" not in _semantic_tokens("NOT_ACTIVE")
    assert _semantic_tokens("RECOVERY_FAULT") == {"RECOVER", "FAULT"}
    assert not _state_path("/configuration/resolved_robot/edges/planner_recovery/__type__")
    assert _state_path("/reason")


def test_observation_requires_explicit_lineage_basis_or_refs():
    with pytest.raises(ValueError, match="requires evidence lineage"):
        DiagnosticObservation(
            kind=ObservationKind.FACT,
            code="ORPHAN",
            message="orphan claim",
        )
    item = DiagnosticObservation(
        kind=ObservationKind.FACT,
        code="BASED",
        message="based claim",
        evidence_basis=(view_scan("layers/L1"),),
    )
    assert item.as_dict()["evidence_basis"][0]["kind"] == "VIEW_SCAN"


def test_contiguous_episode_has_bounds_duration_and_refs():
    bundle = FakeBundle({
        "layers/L8": [
            row("layers/L8", "m1", 1, {"stop_reason": None}),
            row("layers/L8", "m2", 2, {"stop_reason": "BLOCKED"}),
            row("layers/L8", "m3", 3, {"stop_reason": "BLOCKED"}),
            row("layers/L8", "m4", 4, {"stop_reason": None}),
        ]
    })
    episodes = contiguous_field_episodes(
        bundle,
        "layers/L8",
        "/stop_reason",
        episode_type="STOP_REASON",
        include=lambda value: value is not None,
    )
    assert len(episodes) == 1
    episode = episodes[0]
    assert episode.value == "BLOCKED"
    assert episode.row_count == 2
    assert episode.duration_ns == 100
    assert episode.start_ref.message_id == "m2"
    assert episode.end_ref.message_id == "m3"


def test_admission_separates_required_from_optional_coverage():
    contract = AnalyzerContract(
        analyzer_id="x",
        description="x",
        required_views=("layers/L1",),
        optional_views=("sensors/imu", "sensors/encoder"),
        required_topics=("/r2b4/tick",),
    )
    context = SimpleNamespace(facts=SimpleNamespace(
        verification_status="PASS",
        view_counts={"layers/L1": 1, "sensors/imu": 1},
        topics={"/r2b4/tick": 1},
    ))
    decision = evaluate_admission(contract, context)
    assert decision.applicable
    assert decision.coverage["required"] == "COMPLETE"
    assert decision.coverage["optional"] == "PARTIAL"
    assert decision.coverage["optional_views_missing"] == ["sensors/encoder"]


def _full_context() -> SimpleNamespace:
    payloads = {
        "layers/L1": {"__capture_input_reference__": "RAW_DEVICE_BATCH"},
        "layers/L2": {"pose": {"x": 0.0}},
        "layers/L3": {"localization_quality": {"global_position": "GOOD"}},
        "layers/L4": {"goal": None},
        "layers/L5": {"lifecycle": "ACTIVE", "mode": "EXPLORE"},
        "layers/L6": {"status": "ACTIVE", "reason": None},
        "layers/L7": {"kind": "TRACK_TRAJECTORY", "selected_source": "navigation.trajectory", "selection_reason": "BEST_TRAJECTORY"},
        "layers/L8": {"requested_v_mps": 0.1, "requested_omega_rad_s": 0.1, "stop_reason": None},
        "layers/L9": {"requested_v_mps": 0.1, "requested_omega_rad_s": 0.1, "allowed_v_mps": 0.1, "allowed_omega_rad_s": 0.1, "active_constraints": []},
        "layers/L10": {"left_mps": 0.09, "right_mps": 0.11},
        "layers/L11": {"left_normalized": 0.1, "right_normalized": 0.12, "saturated": False},
        "layers/L12": {"enabled": True, "latch_state": "CLEAR", "left_output": 0.1, "right_output": 0.12, "reason": None, "safety_decision": "ALLOW"},
    }
    rows = {view: [row(view, "m1", 1, payload)] for view, payload in payloads.items()}
    bundle = FakeBundle(rows)
    facts = EvidenceFacts(
        path=Path("/tmp/run.evidence"), source_path="/tmp/run.mcap", source_snapshot_size=1,
        source_sha256="abc", compiler_status="COMPLETE", source_integrity="COMPLETE",
        compiler_git_commit="deadbeef", compiler_git_dirty=False, message_count=1,
        json_decoded_messages=1, quarantined_messages=0, normalized_views=12,
        topics={"/r2b4/tick": 1}, view_counts={view: 1 for view in rows}, field_path_count=20,
        message_start_time_ns=1100, message_end_time_ns=1100, tick_rate_hz_estimate=None,
        verification_status="PASS", verification_source_checked=False,
    )
    producer = DiagProducerFacts(
        generated_at_utc="2026-10-02T00:00:00Z", git_commit="cafebabe",
        diag_source_dirty=False, diag_source_sha256="sourcehash", python_version="3.x",
        platform="test", source_files=("tools/diag/contracts.py",),
    )
    return SimpleNamespace(bundle=bundle, facts=facts, producer=producer)


def test_full_registry_has_lineage_coverage_summary_and_compact_global_provenance():
    report = build_default_registry().run_all(_full_context())
    payload = report.as_dict()
    assert payload["schema"] == "R2B4_DIAG_FULL_REPORT_V2"
    assert payload["producer"]["diag_source_sha256"] == "sourcehash"
    assert payload["summary"]["analyzer_count"] == 10
    assert payload["summary"]["admission_counts"] == {"APPLICABLE": 10}
    for analyzer in payload["analyzers"]:
        assert analyzer["schema"] == "R2B4_DIAG_RESULT_V3"
        assert "evidence" not in analyzer
        assert "producer" not in analyzer
        assert analyzer["coverage"]["required"] == "COMPLETE"
        for observation in analyzer["observations"]:
            assert observation["evidence_refs"] or observation["evidence_basis"]


def test_navigation_counts_have_independent_dimensions():
    from tools.diag.analyzers import navigation

    context = _full_context()
    output = navigation.run(context, None)
    classification = output.metrics["requested_motion_classification"]
    assert classification["motion_mode_counts"]["forward_turning"] == 1
    assert sum(classification["motion_mode_counts"].values()) == 1
    assert classification["omega_sign_counts"]["positive"] == 1
    assert sum(classification["omega_sign_counts"].values()) == 1
