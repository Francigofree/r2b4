from v3.diagnostic_contracts import infer_field_contract
from v3.replay import _physical_evidence

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


