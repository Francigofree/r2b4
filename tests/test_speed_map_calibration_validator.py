import copy
import json

import pytest

from tools.speed_map_calibration_validator import (
    M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
    NON_BLOCKING_M2_PROFILE,
    _hash,
    decide,
    promote_candidate,
    rollback_promotion,
)


def _candidate():
    speeds = (0.15, 0.19, 0.26, 0.35, 0.50, 0.582)
    curves = {}
    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            curves[f"{side}_{direction}"] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": 0.20,
                "maintenance_pwm": 0.16,
                "dead_zone_pwm": 0.16,
                "points": [
                    {"speed_mps": speed, "pwm": 0.05 + speed}
                    for speed in speeds
                ],
            }
    return {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "CANDIDATE",
        "candidate_id": "candidate-test",
        "interpolation": "linear",
        "operating_range_max_mps": 0.582,
        "minimum_common_coverage_mps": 0.58,
        "curves": curves,
    }


def _evidence():
    analysis = {
        "status": "PASS",
        "candidate_qualified": True,
        "candidate_activation_allowed": False,
        "candidate_id": "candidate-test",
        "completed_at_epoch_s": 10.0,
    }
    no_pi = {
        "status": "PASS",
        "candidate_id": "candidate-test",
        "completed_at_epoch_s": 20.0,
    }
    pi = {
        "status": "PASS",
        "candidate_id": "candidate-test",
        "active_map_restored": True,
        "completed_at_epoch_s": 30.0,
    }
    m1 = {
        "status": "PASS",
        "m1_status": "PASS",
        "m1_contract_id": M1_SPEED_MAP_EXECUTION_CONTRACT_ID,
        "candidate_id": "candidate-test",
        "active_map_restored": True,
        "completed_at_epoch_s": 40.0,
    }
    return analysis, no_pi, pi, m1


def test_validator_accepts_only_ordered_same_candidate_evidence():
    analysis, no_pi, pi, m1 = _evidence()

    result = decide(
        candidate=_candidate(),
        analysis=analysis,
        no_pi=no_pi,
        pi=pi,
        m1=m1,
    )

    assert result["status"] == "PASS"
    assert result["decision"] == "ACCEPT"
    assert result["candidate_activation_allowed"] is True
    assert not result["failed_gates"]
    assert result["non_blocking_system_validations"][
        NON_BLOCKING_M2_PROFILE
    ] == {
        "required_for_speed_map_promotion": False,
        "included_in_decision": False,
    }


@pytest.mark.parametrize(
    "mutation,failed_gate",
    [
        (lambda a, n, p, m: p.update(active_map_restored=False), "quick_pi"),
        (lambda a, n, p, m: m.update(m1_status="FAIL"), "full_m1"),
        (lambda a, n, p, m: m.update(m1_contract_id="OLD"), "full_m1"),
        (lambda a, n, p, m: n.update(candidate_id="other"), "candidate_identity"),
        (lambda a, n, p, m: p.update(completed_at_epoch_s=15.0), "validation_order"),
    ],
)
def test_validator_fails_closed_for_missing_gate(mutation, failed_gate):
    analysis, no_pi, pi, m1 = _evidence()
    mutation(analysis, no_pi, pi, m1)

    result = decide(
        candidate=_candidate(),
        analysis=analysis,
        no_pi=no_pi,
        pi=pi,
        m1=m1,
    )

    assert result["status"] == "FAIL"
    assert result["candidate_activation_allowed"] is False
    assert failed_gate in result["failed_gates"]


def test_promotion_requires_exact_decision_and_has_rollback(tmp_path):
    candidate = _candidate()
    analysis, no_pi, pi, m1 = _evidence()
    decision = decide(
        candidate=candidate,
        analysis=analysis,
        no_pi=no_pi,
        pi=pi,
        m1=m1,
    )
    active = copy.deepcopy(candidate)
    active["map_state"] = "ACTIVE"
    active["candidate_id"] = "old"
    active_path = tmp_path / "speed_map.json"
    backup_path = tmp_path / "before.json"
    active_path.write_text(json.dumps(active), encoding="utf-8")

    promoted = promote_candidate(
        candidate=candidate,
        decision=decision,
        active_map_path=active_path,
        backup_path=backup_path,
    )

    current = json.loads(active_path.read_text())
    assert promoted["promoted"] is True
    assert current["map_state"] == "ACTIVE"
    assert current["candidate_id"] == "candidate-test"
    assert current["accepted_by"] == decision["schema"]
    assert json.loads(backup_path.read_text())["candidate_id"] == "old"

    rollback = rollback_promotion(
        active_map_path=active_path,
        backup_path=backup_path,
    )

    assert rollback["rolled_back"] is True
    assert json.loads(active_path.read_text())["candidate_id"] == "old"


def test_promotion_rejects_modified_candidate(tmp_path):
    candidate = _candidate()
    analysis, no_pi, pi, m1 = _evidence()
    decision = decide(
        candidate=candidate,
        analysis=analysis,
        no_pi=no_pi,
        pi=pi,
        m1=m1,
    )
    modified = copy.deepcopy(candidate)
    modified["curves"]["left_forward"]["points"][0]["pwm"] += 0.01
    active = copy.deepcopy(candidate)
    active["map_state"] = "ACTIVE"
    active_path = tmp_path / "speed_map.json"
    active_path.write_text(json.dumps(active), encoding="utf-8")

    with pytest.raises(RuntimeError, match="promotion_not_authorized"):
        promote_candidate(
            candidate=modified,
            decision=decision,
            active_map_path=active_path,
            backup_path=tmp_path / "backup.json",
        )

    assert decision["candidate_sha256"] == _hash(candidate)
