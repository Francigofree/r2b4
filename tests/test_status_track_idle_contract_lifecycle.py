#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller.status import build_motion_command_semantics


class _DummyStateMachine:
    def __init__(self, name: str):
        self._name = str(name)

    def get_current_state_name(self):
        return str(self._name)


def _build_ctrl(*, task_updated_ts: float, task_marker: str) -> SimpleNamespace:
    return SimpleNamespace(
        requested_motion_intent={"v": 0.0, "omega": 0.0},
        limited_motion_intent={"v": 0.0, "omega": 0.0},
        requested_track_reference={"left_mps": 0.0, "right_mps": 0.0},
        service_pwm_command={},
        motion_contract_status={},
        motion_public_status={},
        motion_task_status={
            "command_type": "set_track_velocity",
            "execution_state": "succeeded",
            "updated_ts": float(task_updated_ts),
            "details": {"track_idle_transition_contract": str(task_marker)},
        },
        active_motion_command_type="idle",
        active_motion_command_layer="IDLE",
        active_motion_command_source="STATE",
        motion_execution_mode="IDLE_EXEC",
        track_target_left_mps=0.0,
        track_target_right_mps=0.0,
        track_idle_transition_contract_latch={},
        cfg={"fizika": {"nyomtav_szelesseg_m": 0.175}},
        sm=_DummyStateMachine("IDLE"),
    )


class TestTrackIdleContractLifecycleFallback(unittest.TestCase):
    def test_required_is_true_from_recent_terminal_track_task_even_if_active_command_is_idle(self):
        ctrl = _build_ctrl(task_updated_ts=100.5, task_marker="TRACK_ZERO_TO_IDLE")

        with patch("controller.status.time.time", return_value=101.0):
            semantics = build_motion_command_semantics(ctrl)

        contract = dict(semantics.get("track_idle_transition_contract") or {})
        self.assertTrue(bool(contract.get("required", False)))
        self.assertTrue(bool(contract.get("satisfied", False)))
        self.assertEqual(str(contract.get("required_reason", "")), "task_zero_track_lifecycle_transition")

    def test_required_is_not_held_from_stale_terminal_track_task(self):
        ctrl = _build_ctrl(task_updated_ts=100.0, task_marker="TRACK_ZERO_TO_IDLE")

        with patch("controller.status.time.time", return_value=102.5):
            semantics = build_motion_command_semantics(ctrl)

        contract = dict(semantics.get("track_idle_transition_contract") or {})
        self.assertFalse(bool(contract.get("required", True)))
        self.assertEqual(str(contract.get("required_reason", "")), "none")

    def test_required_is_temporarily_sticky_after_latch_has_just_been_satisfied(self):
        ctrl = _build_ctrl(task_updated_ts=90.0, task_marker="")
        ctrl.motion_task_status = {}
        ctrl.track_idle_transition_contract_latch = {
            "pending": False,
            "hold_required_s": 0.25,
            "satisfied_ts": 100.0,
            "requested_track_reference": {"left_mps": 0.0, "right_mps": 0.0},
            "track_targets": {"left_mps": 0.0, "right_mps": 0.0},
        }

        with patch("controller.status.time.time", return_value=100.3):
            semantics = build_motion_command_semantics(ctrl)

        contract = dict(semantics.get("track_idle_transition_contract") or {})
        self.assertTrue(bool(contract.get("required", False)))
        self.assertTrue(bool(contract.get("satisfied", False)))
        self.assertEqual(str(contract.get("required_reason", "")), "recent_latched_zero_track_command")

    def test_follow_arc_semantics_preserve_arc_intent_when_twist_temporarily_zero(self):
        ctrl = _build_ctrl(task_updated_ts=90.0, task_marker="")
        ctrl.active_motion_command_type = "follow_arc"
        ctrl.active_motion_command_layer = "BEHAVIOR"
        ctrl.motion_execution_mode = "ARC_EXEC"
        ctrl.requested_motion_intent = {"v": 0.0, "omega": 0.0}
        ctrl.limited_motion_intent = {"v": 0.0, "omega": 0.0}
        ctrl.requested_track_reference = {"left_mps": None, "right_mps": None}
        ctrl.track_target_left_mps = None
        ctrl.track_target_right_mps = None
        ctrl.behavior_motion_status = {
            "mode": "FOLLOW_ARC",
            "radius_m": 0.25,
            "arc_angle_deg": 60.0,
            "speed_mps": 0.095,
        }

        with patch("controller.status.time.time", return_value=101.0):
            semantics = build_motion_command_semantics(ctrl)

        self.assertNotEqual(str(semantics.get("turn_primitive_requested", "")), "STRAIGHT")
        self.assertNotEqual(str(semantics.get("turn_primitive_limited", "")), "STRAIGHT")
        self.assertNotEqual(str(semantics.get("turn_primitive_executed", "")), "STRAIGHT")
        requested = dict(semantics.get("requested_motion_intent") or {})
        self.assertGreater(abs(float(requested.get("omega", 0.0))), 0.01)

    def test_follow_arc_actual_semantics_remain_measurement_based(self):
        ctrl = _build_ctrl(task_updated_ts=90.0, task_marker="")
        ctrl.active_motion_command_type = "follow_arc"
        ctrl.active_motion_command_layer = "BEHAVIOR"
        ctrl.motion_execution_mode = "ARC_EXEC"
        ctrl.requested_motion_intent = {"v": 0.095, "omega": 0.38}
        ctrl.limited_motion_intent = {"v": 0.095, "omega": 0.38}
        ctrl.requested_track_reference = {"left_mps": 0.045, "right_mps": 0.145}
        ctrl.track_target_left_mps = 0.02
        ctrl.track_target_right_mps = 0.10
        ctrl.motion_public_status = {
            "actual_linear_mps": 0.06,
            "actual_angular_dps": 8.0,
        }
        ctrl.behavior_motion_status = {
            "mode": "FOLLOW_ARC",
            "radius_m": 0.25,
            "arc_angle_deg": 60.0,
            "speed_mps": 0.095,
        }

        with patch("controller.status.time.time", return_value=101.0):
            semantics = build_motion_command_semantics(ctrl)

        self.assertEqual(str(semantics.get("turn_primitive_executed", "")), "DIFF_ARC_SHARP")
        self.assertEqual(str(semantics.get("turn_primitive_actual", "")), "DIFF_ARC_GENTLE")
        source = dict(semantics.get("turn_primitive_source") or {})
        self.assertEqual(str(source.get("actual", "")), "actual_measurement")

    def test_follow_arc_low_v_turn_surface_is_anchored_to_arc_semantics(self):
        ctrl = _build_ctrl(task_updated_ts=90.0, task_marker="")
        ctrl.active_motion_command_type = "follow_arc"
        ctrl.active_motion_command_layer = "BEHAVIOR"
        ctrl.motion_execution_mode = "ARC_EXEC"
        ctrl.requested_motion_intent = {"v": 0.006, "omega": 0.32}
        ctrl.limited_motion_intent = {"v": 0.006, "omega": 0.30}
        ctrl.requested_track_reference = {"left_mps": None, "right_mps": None}
        ctrl.track_target_left_mps = None
        ctrl.track_target_right_mps = None
        ctrl.behavior_motion_status = {
            "mode": "FOLLOW_ARC",
            "radius_m": 0.25,
            "arc_angle_deg": 60.0,
            "speed_mps": 0.095,
        }

        with patch("controller.status.time.time", return_value=101.0):
            semantics = build_motion_command_semantics(ctrl)

        self.assertTrue(str(semantics.get("turn_primitive_requested", "")).startswith("DIFF_ARC_"))
        self.assertTrue(str(semantics.get("turn_primitive_limited", "")).startswith("DIFF_ARC_"))
        self.assertTrue(str(semantics.get("turn_primitive_executed", "")).startswith("DIFF_ARC_"))


if __name__ == "__main__":
    unittest.main()
