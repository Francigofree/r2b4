import json
from pathlib import Path

import pytest

from core.control_strategies import (
    SimplePI,
    WheelSpeedPILoop,
    WHEEL_PI_FEEDBACK_FILTER_ALPHA,
    load_control_mode,
    normalize_control_mode,
    save_control_mode,
    wheel_feedback_timing_error,
)
from middleware.ffp import PIDConfig, lookup_wheel_feedforward


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _active_wheel_pi_config():
    payload = json.loads((PROJECT_ROOT / "conf" / "vezerles.json").read_text(encoding="utf-8"))
    return payload["pid_szabalyzo"]


def test_active_wheel_controller_is_fixed_pi_with_bounded_integral():
    cfg = _active_wheel_pi_config()

    assert cfg["aranyos_tag_p"] == pytest.approx(0.25)
    assert cfg["integralo_tag_i"] == pytest.approx(0.08)
    assert "derivalo_tag_d" not in cfg
    assert "holtsav_kuszob" not in cfg
    assert cfg["integralo_limit"] == pytest.approx(0.18)


def test_fixed_pi_integral_output_remains_bounded():
    cfg = _active_wheel_pi_config()
    pi = SimplePI(
        cfg["aranyos_tag_p"],
        cfg["integralo_tag_i"],
        cfg["integralo_limit"],
    )

    for _ in range(2000):
        _, i_term = pi.update(-0.025, 0.02)

    assert i_term == pytest.approx(-0.0144, abs=1e-9)
    assert pi.integrator_clamped is True


def test_fixed_pi_drops_onset_windup_when_tracking_error_reverses():
    cfg = _active_wheel_pi_config()
    pi = SimplePI(
        cfg["aranyos_tag_p"],
        cfg["integralo_tag_i"],
        cfg["integralo_limit"],
    )

    for _ in range(20):
        pi.update(0.05, 0.02)
    p_term, i_term = pi.update(-0.02, 0.02)

    assert p_term == pytest.approx(-0.005, abs=1e-12)
    assert i_term == pytest.approx(-0.000032, abs=1e-12)
    assert (p_term + i_term) < 0.0


def test_wheel_pi_keeps_integral_across_encoder_quantization_zero_crossing():
    cfg = _active_wheel_pi_config()
    pi = SimplePI(
        cfg["aranyos_tag_p"],
        cfg["integralo_tag_i"],
        cfg["integralo_limit"],
        zero_cross_reset_deadband=0.006,
    )

    for _ in range(20):
        pi.update(0.012, 0.02)
    before = pi.integrator_state
    p_term, i_term = pi.update(-0.003, 0.02)

    assert before > 0.0
    assert p_term < 0.0
    assert i_term > 0.0
    assert pi.integrator_state == pytest.approx(before - 0.00006, abs=1e-12)


def test_wheel_pi_still_resets_integral_on_material_overspeed_crossing():
    cfg = _active_wheel_pi_config()
    pi = SimplePI(
        cfg["aranyos_tag_p"],
        cfg["integralo_tag_i"],
        cfg["integralo_limit"],
        zero_cross_reset_deadband=0.006,
    )

    for _ in range(20):
        pi.update(0.05, 0.02)
    p_term, i_term = pi.update(-0.02, 0.02)

    assert p_term == pytest.approx(-0.005, abs=1e-12)
    assert i_term == pytest.approx(-0.000032, abs=1e-12)
    assert (p_term + i_term) < 0.0


def test_wheel_loop_reports_pi_feedback_without_rewriting_raw_tracking_error():
    loop = WheelSpeedPILoop(
        PIDConfig(kp=0.25, ki=0.0, integrator_limit=0.18),
        max_pwm=1.0,
        dead_zone=0.0,
    )
    _, _, first_diag = loop.compute(
        left_reference_mps=0.26057,
        right_reference_mps=0.18943,
        left_measured_mps=0.26057,
        right_measured_mps=0.18943,
        dt_s=0.1,
        feedforward_pwm_l=0.0,
        feedforward_pwm_r=0.0,
        maintenance_floor_pwm_l=0.0,
        maintenance_floor_pwm_r=0.0,
    )
    assert first_diag["left_control_error_mps"] == pytest.approx(0.0)

    pwm_l, pwm_r, diag = loop.compute(
        left_reference_mps=0.26057,
        right_reference_mps=0.18943,
        left_measured_mps=0.14798,
        right_measured_mps=0.10938,
        dt_s=0.1,
        feedforward_pwm_l=0.0,
        feedforward_pwm_r=0.0,
        maintenance_floor_pwm_l=0.0,
        maintenance_floor_pwm_r=0.0,
    )

    expected_l_meas = (
        WHEEL_PI_FEEDBACK_FILTER_ALPHA * 0.14798
        + (1.0 - WHEEL_PI_FEEDBACK_FILTER_ALPHA) * 0.26057
    )
    expected_r_meas = (
        WHEEL_PI_FEEDBACK_FILTER_ALPHA * 0.10938
        + (1.0 - WHEEL_PI_FEEDBACK_FILTER_ALPHA) * 0.18943
    )
    assert diag["left_measured_mps"] == pytest.approx(0.14798)
    assert diag["left_control_error_mps"] == pytest.approx(0.26057 - expected_l_meas)
    assert diag["wheel_pi_enabled"] is True
    assert diag["left_p_pwm"] == pytest.approx(0.25 * (0.26057 - expected_l_meas))
    assert diag["right_p_pwm"] == pytest.approx(0.25 * (0.18943 - expected_r_meas))
    assert pwm_l == pytest.approx(0.25 * (0.26057 - expected_l_meas))
    assert pwm_r == pytest.approx(0.25 * (0.18943 - expected_r_meas))
    assert pwm_l == pytest.approx(0.25 * 0.11259)


def test_wheel_loop_reports_physical_maintenance_floor_application():
    loop = WheelSpeedPILoop(
        PIDConfig(kp=0.0, ki=0.0, integrator_limit=0.18),
        max_pwm=1.0,
        dead_zone=0.0,
    )
    _left, _right, diagnostics = loop.compute(
        left_reference_mps=0.15,
        right_reference_mps=0.15,
        left_measured_mps=0.0,
        right_measured_mps=0.0,
        dt_s=0.02,
        feedforward_pwm_l=0.01,
        feedforward_pwm_r=0.01,
        maintenance_floor_pwm_l=0.12,
        maintenance_floor_pwm_r=0.13,
    )

    assert diagnostics["maintenance_floor_pwm_l"] == pytest.approx(0.12)
    assert diagnostics["maintenance_floor_pwm_r"] == pytest.approx(0.13)
    assert diagnostics["wheel_loop_left_maintenance_floor_applied"] is True
    assert diagnostics["wheel_loop_right_maintenance_floor_applied"] is True


def test_wheel_feedback_timing_gap_has_explicit_l9_reason():
    assert wheel_feedback_timing_error(
        timing_valid=False,
        stale=False,
        timing_reason="encoder_timing_gap",
    ) == "ENCODER_TIMING_GAP"
    assert wheel_feedback_timing_error(
        timing_valid=False,
        stale=False,
        timing_reason="deadline",
    ) == "WHEEL_FEEDBACK_TIMING_INVALID"


def test_wheel_loop_feedback_filter_resets_on_reference_direction_change():
    loop = WheelSpeedPILoop(
        PIDConfig(kp=0.25, ki=0.0, integrator_limit=0.18),
        max_pwm=1.0,
        dead_zone=0.0,
    )
    loop.compute(
        left_reference_mps=0.20,
        right_reference_mps=0.20,
        left_measured_mps=0.20,
        right_measured_mps=0.20,
        dt_s=0.1,
        feedforward_pwm_l=0.0,
        feedforward_pwm_r=0.0,
        maintenance_floor_pwm_l=0.0,
        maintenance_floor_pwm_r=0.0,
    )

    _, _, diag = loop.compute(
        left_reference_mps=-0.15,
        right_reference_mps=-0.15,
        left_measured_mps=-0.08,
        right_measured_mps=-0.08,
        dt_s=0.1,
        feedforward_pwm_l=0.0,
        feedforward_pwm_r=0.0,
        maintenance_floor_pwm_l=0.0,
        maintenance_floor_pwm_r=0.0,
    )

    assert diag["left_control_error_mps"] == pytest.approx(-0.07)


def test_active_wheel_pi_gain_is_below_delayed_map_loop_gain_limit():
    """Keep proportional feedback stable against the 100 ms canonical window.

    The active speed map is the measured inverse plant model.  Its steepest
    speed/PWM segment is therefore the conservative local plant gain.  A
    proportional loop gain at or above one can alternate corrections when
    canonical feedback arrives after a 100-120 ms aggregation window.
    """

    cfg = _active_wheel_pi_config()
    speed_map = json.loads(
        (PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8")
    )
    local_plant_gains = []
    for curve in dict(speed_map.get("curves") or {}).values():
        points = list((curve or {}).get("points") or [])
        for left, right in zip(points, points[1:]):
            delta_pwm = float(right["pwm"]) - float(left["pwm"])
            delta_speed = float(right["speed_mps"]) - float(left["speed_mps"])
            local_plant_gains.append(delta_speed / delta_pwm)

    worst_local_plant_gain = max(local_plant_gains)
    active_loop_gain = float(cfg["aranyos_tag_p"]) * worst_local_plant_gain
    previous_loop_gain = 0.4 * worst_local_plant_gain
    rejected_loop_gain = 0.7 * worst_local_plant_gain

    assert active_loop_gain == pytest.approx(0.385142074632, rel=1e-6)
    assert active_loop_gain < 0.5
    assert previous_loop_gain == pytest.approx(0.616227319411, rel=1e-6)
    assert rejected_loop_gain > 1.0


def test_active_kp_reduces_latest_m1_delayed_error_correction_variation():
    """Replay the independent canonical errors from the latest failing M1.

    This is intentionally a controller-output replay, not sample exclusion or
    a claim that recorded wheel speeds would change retroactively.
    """

    cfg = _active_wheel_pi_config()
    # Worst affected outer/inner ARC windows from the 2026-07-22 12:47Z run.
    error_windows = (
        (0.120601, 0.087635),
        (-0.047977, -0.010855),
        (0.129754, 0.083532),
        (-0.041943, -0.032053),
        (0.003412, -0.002071),
        (0.004123, -0.028550),
        (-0.002143, 0.012501),
        (0.082944, 0.054865),
        (-0.006789, -0.001541),
    )

    def proportional_total_variation(kp):
        outputs = [(float(kp) * left, float(kp) * right) for left, right in error_windows]
        return sum(
            abs(cur_l - prev_l) + abs(cur_r - prev_r)
            for (prev_l, prev_r), (cur_l, cur_r) in zip(outputs, outputs[1:])
        )

    active_tv = proportional_total_variation(cfg["aranyos_tag_p"])
    previous_tv = proportional_total_variation(0.4)
    assert active_tv < previous_tv * 0.65


def test_active_speed_map_feedforward_remains_the_unchanged_primary_output():
    speed_map = json.loads(
        (PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8")
    )
    pwm, diag = lookup_wheel_feedforward(
        speed_map,
        side="left",
        target_mps=0.18943,
        require_active=True,
    )
    assert pwm == pytest.approx(0.238618985, abs=1e-6)
    assert diag["curve"] == "left_forward"


def test_control_mode_is_exact_and_invalid_values_fail_closed(tmp_path):
    assert normalize_control_mode(" unified ") == "UNIFIED"
    for invalid in (None, "", "BASIC", "ENHANCED", "FULL"):
        with pytest.raises(ValueError, match="unsupported_control_mode"):
            normalize_control_mode(invalid)

    missing = tmp_path / "missing-control-mode.json"
    with pytest.raises(FileNotFoundError, match="control_mode_config_missing"):
        load_control_mode(str(missing))

    invalid_path = tmp_path / "invalid-control-mode.json"
    invalid_path.write_text('{"control_mode": "BASIC"}', encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported_control_mode:BASIC"):
        load_control_mode(str(invalid_path))


def test_control_mode_writer_cannot_persist_removed_mode(tmp_path):
    path = tmp_path / "control-mode.json"
    assert save_control_mode(str(path), "UNIFIED") is True
    assert json.loads(path.read_text(encoding="utf-8")) == {"control_mode": "UNIFIED"}
    with pytest.raises(ValueError, match="unsupported_control_mode:BASIC"):
        save_control_mode(str(path), "BASIC")
    assert json.loads(path.read_text(encoding="utf-8")) == {"control_mode": "UNIFIED"}
