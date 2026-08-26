import copy

from tools.speed_map_calibration_analyzer import (
    PROFILE_KEYS,
    analyze,
    sample_rejection_reasons,
)


def _active_map():
    curves = {}
    for key in PROFILE_KEYS:
        side, direction = key.split("_", 1)
        curves[key] = {
            "wheel": side,
            "direction": direction,
            "startup_pwm": 0.20,
            "maintenance_pwm": 0.17,
            "dead_zone_pwm": 0.17,
            "points": [
                {"speed_mps": 0.15, "pwm": 0.20},
                {"speed_mps": 0.30, "pwm": 0.35},
            ],
        }
    return {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "ACTIVE",
        "hardware": "test",
        "curves": curves,
    }


def _row(
    *,
    kind,
    direction,
    pwm,
    speed,
    repeat,
    sweep="ascending",
):
    sign = 1.0 if direction == "forward" else -1.0
    stability = {
        "sample_count": 12,
        "moving_sample_ratio": 1.0,
        "coefficient_of_variation": 0.04,
        "acceleration_slope_mps2": 0.01,
        "dropout_transitions": 0,
        "wrong_direction_samples": 0,
    }
    return {
        "calibration_run_id": "run-test",
        "measurement_kind": kind,
        "motion_geometry": "STRAIGHT",
        "sweep_direction": sweep,
        "repeat": repeat,
        "direction": direction,
        "commanded_pwm": {"left": sign * pwm, "right": sign * pwm},
        "actual_mps": {"left": sign * speed, "right": sign * speed * 1.01},
        "direct_executor_observed": True,
        "pi_disabled_observed": True,
        "pi_violation_seen": False,
        "controller_distortion": {
            "straight_hold_applied": False,
            "feedforward_map_applied": False,
            "startup_floor_applied": False,
            "maintenance_floor_applied": False,
            "planner_correction_applied": False,
        },
        "faults": [],
        "safety_intervention_seen": False,
        "encoder_blocking_anomaly_seen": False,
        "encoder_reliability_health_seen": ["OK"],
        "encoder_reliability_trust_min": 0.9,
        "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
        "distance_target_reached": True,
        "distance_limit_triggered": False,
        "stability": {
            "left": dict(stability),
            "right": dict(stability),
        },
    }


def _rows():
    rows = []
    for direction in ("forward", "reverse"):
        for repeat in range(1, 4):
            rows.append(
                _row(
                    kind="startup_threshold",
                    direction=direction,
                    pwm=0.12,
                    speed=0.005,
                    repeat=repeat,
                )
            )
            rows.append(
                _row(
                    kind="startup_threshold",
                    direction=direction,
                    pwm=0.18,
                    speed=0.06,
                    repeat=repeat,
                )
            )
            rows.append(
                _row(
                    kind="maintenance_threshold",
                    direction=direction,
                    pwm=0.10,
                    speed=0.006,
                    repeat=repeat,
                    sweep="descending",
                )
            )
            rows.append(
                _row(
                    kind="maintenance_threshold",
                    direction=direction,
                    pwm=0.14,
                    speed=0.05,
                    repeat=repeat,
                    sweep="descending",
                )
            )
        for sweep in ("ascending", "descending"):
            for repeat in range(1, 3):
                for pwm, speed in (
                    (0.16, 0.15),
                    (0.22, 0.19),
                    (0.30, 0.26),
                    (0.40, 0.35),
                    (0.52, 0.50),
                    (0.68, 0.70),
                    (0.85, 0.90),
                ):
                    rows.append(
                        _row(
                            kind="stable_point",
                            direction=direction,
                            pwm=pwm,
                            speed=speed,
                            repeat=repeat,
                            sweep=sweep,
                        )
                    )
    return rows


def test_analyzer_builds_four_monotonic_candidate_profiles():
    active = _active_map()
    before = copy.deepcopy(active)

    result, candidate = analyze(active_map=active, rows=_rows())

    assert active == before
    assert result["status"] == "PASS"
    assert result["candidate_qualified"] is True
    assert result["candidate_activation_allowed"] is False
    assert candidate["map_state"] == "CANDIDATE"
    assert set(candidate["curves"]) == set(PROFILE_KEYS)
    assert candidate["requires_validation_order"] == [
        "speed_map_quick_no_pi_live",
        "speed_map_quick_pi_live",
        "speed_map_candidate_M1_live",
    ]
    for curve in candidate["curves"].values():
        assert curve["startup_pwm"] == 0.18
        assert curve["maintenance_pwm"] == 0.14
        assert curve["dead_zone_pwm"] == curve["maintenance_pwm"]
        speeds = [point["speed_mps"] for point in curve["points"]]
        pwms = [point["pwm"] for point in curve["points"]]
        assert 0.19 in speeds
        assert 0.26 in speeds
        assert len(speeds) == 6
        assert speeds == sorted(speeds)
        assert pwms == sorted(pwms)
        assert max(speeds) == 0.582
    assert result["minimum_common_coverage_mps"] == 0.58
    assert result["operating_range_target_max_mps"] == 0.582
    assert candidate["operating_range_max_mps"] == 0.582


def test_accelerating_or_controller_distorted_sample_is_rejected():
    row = _rows()[0]
    row["stability"]["left"]["acceleration_slope_mps2"] = 0.2
    row["controller_distortion"]["straight_hold_applied"] = True

    reasons = sample_rejection_reasons(row, "left", require_stable=True)

    assert "accelerating_sample" in reasons
    assert "straight_hold_applied" in reasons


def test_analyzer_keeps_encoder_reliability_gate_for_threshold_selection():
    row = _row(
        kind="startup_threshold",
        direction="forward",
        pwm=0.06,
        speed=0.02,
        repeat=1,
    )
    row["encoder_blocking_anomaly_seen"] = True
    row["encoder_reliability_trust_min"] = 0.17

    reasons = sample_rejection_reasons(
        row,
        "left",
        require_stable=True,
    )
    acquisition_only_reasons = sample_rejection_reasons(
        row,
        "left",
        require_stable=False,
        require_encoder_reliable=False,
    )

    assert "encoder_anomaly" in reasons
    assert "encoder_trust" in reasons
    assert "encoder_anomaly" not in acquisition_only_reasons
    assert "encoder_trust" not in acquisition_only_reasons


def test_missing_descending_repeat_coverage_fails_candidate():
    rows = [
        row
        for row in _rows()
        if not (
            row["measurement_kind"] == "stable_point"
            and row["direction"] == "reverse"
            and row["sweep_direction"] == "descending"
        )
    ]

    result, candidate = analyze(active_map=_active_map(), rows=rows)

    assert result["status"] == "FAIL"
    assert result["candidate_qualified"] is False
    assert candidate["activation_allowed"] is False
    assert any("stable_response_insufficient" in failure for failure in result["analysis_failures"])
