from __future__ import annotations

from v3.diagnostic_contracts import infer_field_contract
from v3.replay import _physical_evidence
from v3.test_hub_analysis import Incident, _priority_evidence_candidate
from v3.test_hub_v2 import _build_diagnosis
from v3.test_hub_motion_quality import analyze_motion_quality_ticks
from v3.test_hub_localization_quality import analyze_localization_quality_ticks


def test_priority_evidence_is_explicitly_noncausal() -> None:
    result = _priority_evidence_candidate(
        (
            Incident(
                "timing-1",
                "HIGH",
                "TIMING",
                10,
                123,
                "Runtime",
                "TICK_INTERVAL_OUTLIER",
                {"delta_ms": 120.0},
            ),
        )
    )
    assert result["claim_class"] == "HEURISTIC_FINDING"
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "HEURISTIC"
    assert "confidence" not in result


def test_direct_production_fault_is_policy_verdict_not_root_cause() -> None:
    result = _priority_evidence_candidate(
        (
            Incident(
                "fault-1",
                "CRITICAL",
                "PRODUCTION_FAULT",
                11,
                124,
                "L11",
                "L11_ERROR",
                {},
            ),
        )
    )
    assert result["claim_class"] == "POLICY_VERDICT"
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "DIRECT"


def test_optional_device_health_is_not_physical_explanation() -> None:
    observation = {
        "reason": "PERSON_GEOMETRY_DEGRADED",
        "evidence": {
            "device_health": [
                {
                    "device_id": "PERSON_DETECTOR_FRONT",
                    "state": "DEGRADED",
                    "reason": "PERSON_GEOMETRY_DEGRADED",
                }
            ]
        },
    }
    result = _physical_evidence({}, observation, None)
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "INSUFFICIENT"
    assert result["observation"] is None
    assert "cause" not in result


def test_critical_device_health_stays_an_indicator_only() -> None:
    observation = {
        "reason": "ENCODER_DEGRADED",
        "evidence": {
            "device_health": [
                {
                    "device_id": "WHEEL_ENCODERS",
                    "state": "DEGRADED",
                    "reason": "ENCODER_DEGRADED",
                }
            ]
        },
    }
    result = _physical_evidence({}, observation, None)
    assert result["claim_class"] == "HEURISTIC_FINDING"
    assert result["causal_claim"] is False
    assert result["evidence_strength"] == "INDICATOR"
    assert "cause" not in result


def test_generation_is_a_counter_not_a_ratio() -> None:
    field = infer_field_contract("generation")
    assert field.semantic_role == "COUNTER"
    assert field.unit is None
    assert "MONOTONICITY_OBSERVATION" in field.generic_checks


def test_quality_findings_are_declared_noncausal() -> None:
    # Empty input intentionally exercises INSUFFICIENT_DATA. Claim semantics
    # are part of the evidence contract even when no threshold can be evaluated.
    motion, _segments = analyze_motion_quality_ticks([])
    localization, _events = analyze_localization_quality_ticks([])
    for payload in (motion, localization):
        assert payload["claim_policy"]["finding_class"] == "HEURISTIC_FINDING"
        assert payload["claim_policy"]["causal_claim"] is False
        assert payload["claim_policy"]["root_cause_inferred"] is False
        assert payload["claim_policy"]["causal_analysis_owner"] == "ANALYZER_LLM"


def test_diagnosis_contract_hands_causality_to_analyzer_llm() -> None:
    inspect = {
        "analysis_profile": {"name": "FORENSIC_LOW_LEVEL"},
        "structure": {"valid": True},
        "final_event": {
            "status": "PASS",
            "integrity": {"complete": True},
        },
    }
    triage = {
        "behavior_status": "NOMINAL",
        "incidents": [],
        "priority_evidence_candidate": {
            "claim_class": "HEURISTIC_FINDING",
            "causal_claim": False,
            "evidence_strength": "HEURISTIC",
            "selection_basis": "SEVERITY_TIME_ORDER",
            "reason": "TICK_INTERVAL_OUTLIER",
            "tick_id": None,
            "layer": "Runtime",
            "kind": "TIMING",
            "evidence_ids": ["timing-1"],
        },
    }
    replay = {
        "status": "MATCH",
        "first_divergence": None,
        "first_live_observation": None,
        "physical_evidence": {
            "claim_class": "FACT",
            "causal_claim": False,
            "evidence_strength": "NONE",
            "observation": None,
            "reason": "NO_PRODUCTION_RELEVANT_LIVE_OBSERVATION_IN_SCOPE",
            "evidence": None,
        },
    }
    result = _build_diagnosis(
        inspect,
        triage,
        replay,
        None,
        replay_requested=True,
    )
    assert "root_cause" not in result
    assert "physical_root_cause" not in result
    assert result["analysis_handoff"]["root_cause_inferred"] is False
    assert result["analysis_handoff"]["causal_analysis_owner"] == "ANALYZER_LLM"
    assert result["priority_evidence"]["causal_claim"] is False
