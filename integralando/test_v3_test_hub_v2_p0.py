"""P0 regressions for Test Hub V2 motion-block/root-cause semantics."""

from types import SimpleNamespace

from v3.test_hub_analysis import Incident, _append_incident, analyze_capture
from v3.test_hub_v2 import _build_diagnosis


class _Reader:
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
        [
            {
                "source_device_id": "RPLIDAR_C1",
                "source_sequence": 7,
                "reason": "DUPLICATE",
                "age_ns": 20_000_000,
            }
        ]
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
                        0.75,0,0,0,0,
                        0,0.75,0,0,0,
                        0,0,0.01,0,0,
                        0,0,0,0.02,0,
                        0,0,0,0,0.0001,
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
                        ["LOCALIZATION_DEGRADED"]
                        if constrained
                        else []
                    ),
                },
                "L12": {
                    "decision": "ALLOW",
                    "reason": "CLEAR",
                },
            },
        },
        "inputs": {
            "raw_devices": {
                "device_health": [
                    {
                        "device_id": "RPLIDAR_C1",
                        "state": "OK",
                        "reason": "",
                    }
                ]
            }
        },
    }


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
        "incidents": [
            {
                "severity": "HIGH",
                "category": "MOTION_BLOCKED",
                "layer": "L9",
                "reason": "LOCALIZATION_DEGRADED",
            }
        ],
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
