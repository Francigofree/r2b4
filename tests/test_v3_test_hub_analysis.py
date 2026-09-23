"""Focused Test Hub analysis regressions, grouped by responsibility."""

from types import SimpleNamespace

import pytest

from v3.test_hub_analysis import (
    Incident,
    _append_incident,
    _root_cause_candidate,
    analyze_capture,
)
from v3.test_hub_v2 import _build_diagnosis
from v3.test_hub_views import (
    _suppress_agent_noise,
    _transition_events,
    compare_views,
)


class _Reader:
    def latest_metadata(self, name):
        return None

    def __init__(self, ticks):
        self._ticks = ticks

    def iter_json_messages(self, *, topics, **_kwargs):
        topic = tuple(topics)[0]
        if topic.endswith("/event"):
            yield (
                SimpleNamespace(sequence=0, log_time_ns=1),
                {
                    "event_type": "capture_finalized",
                    "capture_id": "p0",
                    "status": "PASS",
                    "integrity": {"complete": True},
                },
            )
            return
        for sequence, payload in enumerate(self._ticks, 100):
            yield (
                SimpleNamespace(
                    sequence=sequence,
                    log_time_ns=payload["monotonic_ns"],
                ),
                payload,
            )


def _tick(tick_id, *, duplicate=True, constrained=True):
    rejected = (
        [{
            "source_device_id": "RPLIDAR_C1",
            "source_sequence": 7,
            "reason": "DUPLICATE",
            "age_ns": 20_000_000,
        }]
        if duplicate
        else []
    )
    return {
        "tick_id": tick_id,
        "monotonic_ns": tick_id * 20_000_000,
        "expected": {
            "fault_layer": None,
            "layers": {
                "L2": {"rejected": rejected},
                "L3": {
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                    "v_mps": 0.0,
                    "omega_rad_s": 0.0,
                    "covariance_5x5": [
                        0.75, 0, 0, 0, 0,
                        0, 0.75, 0, 0, 0,
                        0, 0, 0.01, 0, 0,
                        0, 0, 0, 0.02, 0,
                        0, 0, 0, 0, 0.0001,
                    ],
                },
                "L5": {"mode": "TELEOP"},
                "L6": {},
                "L8": {
                    "requested_v_mps": 0.15,
                    "requested_omega_rad_s": 0.0,
                },
                "L9": {
                    "allowed_v_mps": 0.0 if constrained else 0.15,
                    "allowed_omega_rad_s": 0.0,
                    "constraints": (
                        ["LOCALIZATION_DEGRADED"] if constrained else []
                    ),
                },
                "L12": {"decision": "ALLOW", "reason": "CLEAR"},
            },
        },
        "inputs": {
            "raw_devices": {
                "device_health": [{
                    "device_id": "RPLIDAR_C1",
                    "state": "OK",
                    "reason": "",
                }]
            }
        },
    }


def test_teleop_navigation_stagnation_is_agent_noise_only():
    incidents = [{
        "id": "navigation-stagnation",
        "reason": "NAVIGATION_PROGRESS_STAGNATION",
        "category": "NAVIGATION",
        "tick_id": 99,
    }]
    rows = [{"mission": {"mode": "TELEOP"}} for _ in range(5)]
    effective, suppressed = _suppress_agent_noise(incidents, rows)
    assert effective == []
    assert len(suppressed) == 1
    assert "TELEOP" in suppressed[0]["suppressed_reason"]


def test_explore_stagnation_is_not_suppressed():
    incidents = [{"reason": "NAVIGATION_PROGRESS_STAGNATION"}]
    effective, suppressed = _suppress_agent_noise(
        incidents,
        [{"mission": {"mode": "EXPLORE"}}],
    )
    assert effective == incidents
    assert suppressed == []


def test_short_safety_transition_is_preserved_as_event():
    before = {
        "tick_id": 10,
        "monotonic_ns": 100,
        "mission": {
            "mode": "TELEOP",
            "lifecycle": "ACTIVE",
            "stop_reason": None,
        },
        "safety_decision": "ALLOW",
        "l12_reason": None,
        "l9_constraints": [],
    }
    after = {
        **before,
        "tick_id": 11,
        "monotonic_ns": 120,
        "safety_decision": "STOP",
        "l12_reason": "NOT_ACTIVE",
    }
    events = _transition_events(before, after)
    assert any(
        item["signal"] == "safety_decision" and item["to"] == "STOP"
        for item in events
    )
    assert any(
        item["signal"] == "l12_reason" and item["to"] == "NOT_ACTIVE"
        for item in events
    )


def test_compare_is_objective_delta_not_verdict():
    before = {
        "ticks": {"duration_s": 10.0},
        "phases": [],
        "effective_incidents": [{"id": "x"}],
        "windows": [{
            "wheel_control": {
                "left_error_mps": {"mean": 0.02},
                "right_error_mps": {"mean": 0.03},
            },
            "safety": {"ALLOW": 10, "STOP": 1, "FAULT": 1},
        }],
    }
    after = {
        "ticks": {"duration_s": 10.0},
        "phases": [],
        "effective_incidents": [],
        "windows": [{
            "wheel_control": {
                "left_error_mps": {"mean": 0.01},
                "right_error_mps": {"mean": 0.01},
            },
            "safety": {"ALLOW": 11, "STOP": 0, "FAULT": 0},
        }],
    }
    result = compare_views(before, after)
    assert result["verdict_policy"].startswith("No automatic")
    assert result["delta_after_minus_before"]["incident_count"] == -1.0
    assert result["delta_after_minus_before"]["safety_fault"] == -1
    assert (
        result["delta_after_minus_before"]["mean_abs_right_wheel_error_mps"]
        < 0
    )


def test_l2_duplicate_cannot_hide_l9_motion_block():
    triage = analyze_capture(
        _Reader([_tick(100), _tick(101)]),
        max_incidents=8,
    )
    assert triage["sensors"]["l2_duplicate_count"] == 2
    assert triage["sensors"]["l2_actionable_rejection_count"] == 0
    assert triage["behavior_status"] == "BLOCKED"
    assert triage["root_cause_candidate"]["layer"] == "L9"
    assert triage["root_cause_candidate"]["reason"] == "LOCALIZATION_DEGRADED"
    assert triage["root_cause_candidate"]["confidence"] == "PROVEN"
    assert all(
        item["category"] != "ADMISSION"
        for item in triage["incidents"]
    )


def test_incident_budget_keeps_later_critical_over_earlier_medium():
    items = []
    for index in range(4):
        _append_incident(
            items,
            Incident(
                f"medium-{index}",
                "MEDIUM",
                "ADMISSION",
                index,
                index,
                "L2",
                "STALE",
                {},
            ),
            4,
        )

    _append_incident(
        items,
        Incident(
            "critical-late",
            "CRITICAL",
            "SAFETY",
            999,
            999,
            "L12",
            "FAULT",
            {},
        ),
        4,
    )
    assert any(item.incident_id == "critical-late" for item in items)


@pytest.mark.parametrize(
    "category,layer",
    [("SAFETY", "L12"), ("PRODUCTION_FAULT", "L3")],
)
@pytest.mark.parametrize("fault_tick", [99, 100, 101])
def test_fault_severity_cannot_override_earlier_proven_motion_block(
    category,
    layer,
    fault_tick,
):
    blocked = Incident(
        "blocked",
        "HIGH",
        "MOTION_BLOCKED",
        100,
        2_000_000_000,
        "L9",
        "LOCALIZATION_DEGRADED",
        {},
    )
    fault = Incident(
        "fault",
        "CRITICAL",
        category,
        fault_tick,
        fault_tick * 20_000_000,
        layer,
        "FAULT",
        {},
    )
    root = _root_cause_candidate([fault, blocked])
    expected = blocked if fault_tick > blocked.tick_id else fault
    assert root["confidence"] == "PROVEN"
    assert root["kind"] == expected.category
    assert root["tick_id"] == expected.tick_id
    assert root["layer"] == expected.layer
    assert root["reason"] == expected.reason
    assert expected.incident_id in root["evidence_ids"]


def test_valid_evidence_plus_blocked_robot_is_finding_not_pass():
    inspect = {
        "integrity_error": None,
        "structure": {"valid": True},
        "final_event": {
            "status": "PASS",
            "integrity": {"complete": True},
        },
    }
    triage = {
        "behavior_status": "BLOCKED",
        "motion": {
            "requested_motion_tick_count": 273,
            "constrained_to_zero_tick_count": 273,
        },
        "incidents": [{
            "severity": "HIGH",
            "category": "MOTION_BLOCKED",
            "layer": "L9",
            "reason": "LOCALIZATION_DEGRADED",
        }],
        "root_cause_candidate": {
            "confidence": "PROVEN",
            "kind": "MOTION_BLOCKED",
            "layer": "L9",
            "reason": "LOCALIZATION_DEGRADED",
            "tick_id": 809,
        },
        "incident_count": 1,
    }
    replay = {
        "status": "MATCH",
        "first_divergence": None,
        "first_live_incident": None,
        "physical_root_cause": {
            "status": "NOT_PROVEN",
            "reason": "NO_LIVE_INCIDENT_IN_SCOPE",
        },
    }
    diagnosis = _build_diagnosis(
        inspect,
        triage,
        replay,
        None,
        replay_requested=True,
    )
    assert diagnosis["evidence_status"] == "PASS"
    assert diagnosis["replay_status"] == "MATCH"
    assert diagnosis["behavior_status"] == "BLOCKED"
    assert diagnosis["status"] == "FINDING"
    assert diagnosis["root_cause"]["layer"] == "L9"
