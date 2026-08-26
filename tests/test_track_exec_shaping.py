#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from types import SimpleNamespace

import pytest

from controller.motion_controller import MotionController


def _ctrl():
    return SimpleNamespace(
        cfg={"vezerles": {}},
        speed_limits=None,
        motion_command_source="STATE",
        motion_controller_state={},
        motion_ref_v_l=0.0,
        motion_ref_v_r=0.0,
    )


def test_track_reference_uses_common_physical_slew_state():
    ctrl = _ctrl()
    controller = MotionController(
        track_width=0.185,
        v_accel_m_s2=0.6,
        v_decel_m_s2=0.8,
        omega_accel_rad_s2=1.8,
        omega_decel_rad_s2=2.4,
    )

    v_out, omega_out, track = controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=0.3,
        right_target_mps=0.3,
        dt=0.02,
    )

    assert v_out == pytest.approx(0.012)
    assert omega_out == pytest.approx(0.0)
    assert track == pytest.approx({"left_mps": 0.012, "right_mps": 0.012})
    assert ctrl.motion_controller_state["mode"] == "TRACK_REFERENCE_SLEW"
    assert ctrl.motion_controller_state["clamped"] is True


def test_track_transition_to_pivot_is_bounded_and_kinematically_coherent():
    ctrl = _ctrl()
    controller = MotionController(track_width=0.185)
    controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=0.3,
        right_target_mps=0.3,
        dt=0.1,
    )

    v_out, omega_out, track = controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=-0.049,
        right_target_mps=0.049,
        dt=0.1,
    )

    assert v_out == pytest.approx(0.0)
    assert omega_out == pytest.approx(0.18)
    assert track["left_mps"] == pytest.approx(-0.01665)
    assert track["right_mps"] == pytest.approx(0.01665)
    assert (track["right_mps"] - track["left_mps"]) / 0.185 == pytest.approx(omega_out)


def test_track_force_zero_is_immediate_and_resets_slew():
    ctrl = _ctrl()
    controller = MotionController(track_width=0.185)
    controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=0.3,
        right_target_mps=0.3,
        dt=0.1,
    )

    v_out, omega_out, track = controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=0.3,
        right_target_mps=0.3,
        dt=0.1,
        force_zero=True,
    )

    assert (v_out, omega_out) == (0.0, 0.0)
    assert track == {"left_mps": 0.0, "right_mps": 0.0}
    assert ctrl.motion_controller_state["force_zero"] is True


def test_track_reference_floors_nonzero_forward_reverse_and_pivot_targets():
    ctrl = _ctrl()
    controller = MotionController(track_width=0.25, enable_slew=False)

    _, _, pivot = controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=-0.07,
        right_target_mps=0.07,
        dt=0.02,
    )

    assert pivot == pytest.approx({"left_mps": -0.15, "right_mps": 0.15})
    assert ctrl.motion_controller_state["track_minimum_applied"] is True
    assert ctrl.motion_controller_state["track_minimum_mps"] == pytest.approx(0.15)

    controller.reset()
    _, _, reverse = controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=-0.08,
        right_target_mps=-0.08,
        dt=0.02,
    )
    assert reverse == pytest.approx({"left_mps": -0.15, "right_mps": -0.15})


def test_track_reference_preserves_localization_degraded_speed_override():
    ctrl = _ctrl()
    ctrl.localization_gate_status = {
        "apply": {"reason": "localization_gate_speed_limit"}
    }
    controller = MotionController(track_width=0.25, enable_slew=False)

    _, _, track = controller.tick_track_reference(
        ctrl=ctrl,
        left_target_mps=0.07,
        right_target_mps=0.07,
        dt=0.02,
    )

    assert track == pytest.approx({"left_mps": 0.07, "right_mps": 0.07})
    assert ctrl.motion_controller_state["track_minimum_applied"] is False
    assert ctrl.motion_controller_state["track_minimum_bypassed_for_localization"] is True
