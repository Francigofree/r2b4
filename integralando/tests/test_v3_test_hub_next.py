from types import SimpleNamespace

from v3.test_hub_views import (
    _suppress_agent_noise,
    _transition_events,
    compare_views,
)


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
        incidents, [{"mission": {"mode": "EXPLORE"}}]
    )
    assert effective == incidents
    assert suppressed == []


def test_short_safety_transition_is_preserved_as_event():
    before = {
        "tick_id": 10,
        "monotonic_ns": 100,
        "mission": {"mode": "TELEOP", "lifecycle": "ACTIVE", "stop_reason": None},
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
    assert any(item["signal"] == "safety_decision" and item["to"] == "STOP" for item in events)
    assert any(item["signal"] == "l12_reason" and item["to"] == "NOT_ACTIVE" for item in events)


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
    assert result["delta_after_minus_before"]["mean_abs_right_wheel_error_mps"] < 0
