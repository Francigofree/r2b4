import json
from types import SimpleNamespace

import pytest

import tools.speed_map_candidate_live_validator as candidate_validator
from tools.speed_map_candidate_live_validator import (
    _quick_leg_attempt_limit,
    _quick_no_pi_measurement_s,
    _quick_leg_retry_reasons,
    _reload_runtime,
    _selected_speeds,
    temporary_candidate_map,
    validate_quick_rows,
)


def _map(state):
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
                    {"speed_mps": 0.15, "pwm": 0.18},
                    {"speed_mps": 0.65, "pwm": 0.72},
                ],
            }
    return {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": state,
        "candidate_id": "candidate-test" if state == "CANDIDATE" else "",
        "curves": curves,
    }


def test_temporary_candidate_map_restores_exact_active_bytes_on_exception(tmp_path):
    active_path = tmp_path / "speed_map.json"
    backup_path = tmp_path / "rollback.json"
    journal_path = tmp_path / "journal.json"
    original_bytes = (
        json.dumps(_map("ACTIVE"), ensure_ascii=False, indent=1) + "\n"
    ).encode()
    active_path.write_bytes(original_bytes)
    reload_states = []

    def reload_callback():
        reload_states.append(
            json.loads(active_path.read_text(encoding="utf-8"))["map_state"]
        )

    with pytest.raises(RuntimeError, match="synthetic_failure"):
        with temporary_candidate_map(
            candidate=_map("CANDIDATE"),
            active_path=active_path,
            backup_path=backup_path,
            journal_path=journal_path,
            reload_callback=reload_callback,
        ):
            assert json.loads(active_path.read_text())["map_state"] == "ACTIVE"
            assert json.loads(active_path.read_text())["validation_only"] is True
            raise RuntimeError("synthetic_failure")

    assert active_path.read_bytes() == original_bytes
    assert reload_states == ["ACTIVE", "ACTIVE"]
    journal = json.loads(journal_path.read_text())
    assert journal["state"] == "ROLLED_BACK"
    assert journal["rollback_verified"] is True
    assert json.loads(backup_path.read_text())["map_state"] == "ACTIVE"


def test_candidate_reload_uses_runtime_manager_restart(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout="{}", stderr="")

    ready_calls = []
    monkeypatch.setattr(candidate_validator.subprocess, "run", fake_run)
    monkeypatch.setattr(
        candidate_validator,
        "_wait_calibration_ready",
        lambda timeout_s, token: ready_calls.append((timeout_s, token)),
    )

    _reload_runtime("GUI_DEFAULT")

    assert calls[0][0][-2:] == ["tools/agent_runtime_manager.py", "restart"]
    assert calls[0][1]["timeout"] == 90.0
    assert ready_calls == [(90.0, "GUI_DEFAULT")]


def _quick_rows(pi_expected):
    rows = []
    for direction, sign in (("forward", 1.0), ("reverse", -1.0)):
        rows.append(
            {
                "candidate_id": "candidate-test",
                "motion_geometry": "STRAIGHT",
                "measurement_kind": "stable_point",
                "direction": direction,
                "target_speed_mps": 0.26,
                "actual_mps": {"left": sign * 0.25, "right": sign * 0.27},
                "faults": [],
                "safety_intervention_seen": False,
                "encoder_blocking_anomaly_seen": False,
                "distance_target_reached": True,
                "return_distance_error_ratio": 0.04,
                "pi_enabled_observed": pi_expected,
                "candidate_feedforward_observed": pi_expected,
                "executed_target_observed": pi_expected,
                "direct_executor_observed": not pi_expected,
                "pi_disabled_observed": not pi_expected,
                "pi_violation_seen": False,
                "controller_distortion": {},
                "encoder_reliability_health_seen": ["OK"],
                "encoder_reliability_trust_min": 0.9,
                "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
                "distance_limit_triggered": False,
                "stability": {
                    "left": {
                        "sample_count": 10,
                        "moving_sample_ratio": 1.0,
                        "coefficient_of_variation": 0.03,
                        "acceleration_slope_mps2": 0.01,
                        "dropout_transitions": 0,
                        "wrong_direction_samples": 0,
                    },
                    "right": {
                        "sample_count": 10,
                        "moving_sample_ratio": 1.0,
                        "coefficient_of_variation": 0.03,
                        "acceleration_slope_mps2": 0.01,
                        "dropout_transitions": 0,
                        "wrong_direction_samples": 0,
                    },
                },
            }
        )
    return rows


@pytest.mark.parametrize("pi_expected", [False, True])
def test_quick_validator_requires_four_profiles_and_bounded_error(pi_expected):
    result = validate_quick_rows(
        _quick_rows(pi_expected),
        candidate_id="candidate-test",
        pi_expected=pi_expected,
    )

    assert result["status"] == "PASS"
    assert set(result["profile_counts"]) == {
        "left_forward",
        "right_forward",
        "left_reverse",
        "right_reverse",
    }


def test_quick_validator_rejects_safety_intervention():
    rows = _quick_rows(False)
    rows[0]["safety_intervention_seen"] = True

    result = validate_quick_rows(
        rows,
        candidate_id="candidate-test",
        pi_expected=False,
    )

    assert result["status"] == "FAIL"
    assert "safety_or_runtime_fault" in result["failures"]


def test_pi_quick_validator_rejects_command_target_that_was_speed_limited():
    rows = _quick_rows(True)
    rows[0]["executed_target_observed"] = False

    result = validate_quick_rows(
        rows,
        candidate_id="candidate-test",
        pi_expected=True,
    )

    assert result["status"] == "FAIL"
    assert "executed_target_not_observed" in result["failures"]


def test_no_pi_high_speed_band_is_more_tolerant_without_relaxing_strict_band():
    rows = _quick_rows(False)
    for source in _quick_rows(False):
        high = json.loads(json.dumps(source))
        sign = 1.0 if high["direction"] == "forward" else -1.0
        high["target_speed_mps"] = 0.50
        high["actual_mps"] = {
            "left": sign * 0.575,
            "right": sign * 0.575,
        }
        rows.append(high)

    result = validate_quick_rows(
        rows,
        candidate_id="candidate-test",
        pi_expected=False,
    )

    assert result["status"] == "PASS"
    assert result["mean_abs_speed_error_mps"] > 0.035
    assert result["strict_speed_band"]["p90_abs_speed_error_mps"] <= 0.060
    assert result["high_speed_band"]["p90_abs_speed_error_mps"] == pytest.approx(
        0.075
    )


def test_no_pi_high_speed_band_still_has_a_bounded_error_gate():
    rows = _quick_rows(False)
    for source in _quick_rows(False):
        high = json.loads(json.dumps(source))
        sign = 1.0 if high["direction"] == "forward" else -1.0
        high["target_speed_mps"] = 0.50
        high["actual_mps"] = {
            "left": sign * 0.67,
            "right": sign * 0.575,
        }
        rows.append(high)

    result = validate_quick_rows(
        rows,
        candidate_id="candidate-test",
        pi_expected=False,
    )

    assert result["status"] == "FAIL"
    assert "high_speed_p90_abs_speed_error_high" in result["failures"]


def test_quick_leg_retry_is_limited_to_invalid_smallest_phase():
    valid = _quick_rows(False)[0]
    assert _quick_leg_retry_reasons(valid) == []

    invalid = dict(valid)
    invalid["safety_intervention_seen"] = True
    invalid["distance_target_reached"] = False
    assert _quick_leg_retry_reasons(invalid) == [
        "distance_target_not_reached",
        "safety_or_runtime_fault",
    ]

    map_quality_failure = dict(valid)
    map_quality_failure["actual_mps"] = {"left": 0.01, "right": 0.02}
    assert _quick_leg_retry_reasons(map_quality_failure) == []

    limited_pi_leg = dict(valid)
    limited_pi_leg["executed_target_observed"] = False
    assert _quick_leg_retry_reasons(
        limited_pi_leg,
        require_executed_target=True,
    ) == ["executed_target_not_observed"]

    unstable = dict(valid)
    unstable["stability"] = {
        **valid["stability"],
        "left": {
            **valid["stability"]["left"],
            "acceleration_slope_mps2": 0.061,
        },
    }
    assert _quick_leg_retry_reasons(
        unstable,
        require_stable=True,
    ) == ["left:accelerating_sample"]


def test_return_distance_mismatch_is_observation_not_sample_failure():
    rows = _quick_rows(False)
    rows[1]["return_distance_error_ratio"] = 0.95

    result = validate_quick_rows(
        rows,
        candidate_id="candidate-test",
        pi_expected=False,
    )

    assert result["status"] == "PASS"
    assert result["return_distance_mismatch_observation_count"] == 1
    assert _quick_leg_retry_reasons(rows[1]) == []


def test_no_pi_quick_check_uses_measured_reduced_range_endpoint():
    assert _selected_speeds(_map("CANDIDATE"), pi_mode=False) == [
        0.19,
        0.26,
        0.50,
        0.65,
    ]


def test_no_pi_distance_window_keeps_margin_without_raising_distance_cap():
    assert _quick_no_pi_measurement_s(
        target_distance_m=1.485,
        speed_mps=0.582,
    ) == 3.15
    assert _quick_no_pi_measurement_s(
        target_distance_m=0.20,
        speed_mps=0.50,
    ) == 1.5
    assert _quick_no_pi_measurement_s(
        target_distance_m=1.80,
        speed_mps=0.19,
    ) == 3.15


def test_high_speed_gets_two_extra_remeasurements_without_changing_low_speed():
    class Args:
        max_leg_attempts = 3

    assert _quick_leg_attempt_limit(Args(), speed_mps=0.499) == 3
    assert _quick_leg_attempt_limit(Args(), speed_mps=0.50) == 5
    assert _quick_leg_attempt_limit(Args(), speed_mps=0.582) == 5
