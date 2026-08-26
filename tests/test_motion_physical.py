#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from controller.motion_physical import MotionPhysicalTelemetry


def _old_json_signature(*, layer, command_type, source, cmd_linear_mps, cmd_angular_dps, targets):
    payload = {
        "layer": str(layer or ""),
        "command_type": str(command_type or ""),
        "source": str(source or ""),
        "cmd_linear_mps": round(float(cmd_linear_mps), 4),
        "cmd_angular_dps": round(float(cmd_angular_dps), 4),
        "target_distance_m": (
            None
            if targets.get("target_distance_m") is None
            else round(float(targets.get("target_distance_m")), 4)
        ),
        "target_heading_deg": (
            None
            if targets.get("target_heading_deg") is None
            else round(float(targets.get("target_heading_deg")), 4)
        ),
        "target_pose": targets.get("target_pose"),
    }
    return json.dumps(payload, sort_keys=True)


class TestMotionPhysicalTelemetry(unittest.TestCase):
    def test_segment_age_uses_stable_requested_intent_during_shaping(self):
        ctrl = SimpleNamespace(
            motion_execution_mode="TWIST_EXEC",
            requested_track_reference={"left_mps": None, "right_mps": None},
            limited_motion_intent={"v": 0.09, "omega": 0.0},
            requested_motion_intent={"v": 0.09, "omega": 0.0},
            active_motion_command_layer="MOTION_TARGET",
            active_motion_command_type="set_twist",
            active_motion_command_source="STATE",
            motion_executor=SimpleNamespace(track_width=0.3557),
            track_target_left_mps=0.09,
            track_target_right_mps=0.09,
            motion_quality_status={
                "quality_state": "NOMINAL",
                "estimator_consistency": {
                    "confidence": 0.9,
                    "localization_mode": "TRACKING",
                },
            },
            encoder_pipeline_status={
                "canonical_velocity": {"left_mps": 0.085, "right_mps": 0.09},
                "combined_trust": 0.9,
                "snapshot_health": "OK",
                "anomaly_active": False,
            },
            stop_status={},
            motion_public_target={},
            behavior_motion_status={},
            heading_controller_status={},
            target_pose=None,
            sm=SimpleNamespace(get_current_state_name=lambda: "FORWARD"),
        )
        telemetry = MotionPhysicalTelemetry()

        first = telemetry.update(
            ctrl=ctrl,
            ekf_state={
                "x": 0.0,
                "y": 0.0,
                "theta_deg": 0.0,
                "v": 0.08,
                "omega_rad_s": 0.0,
            },
            now=1.0,
        )
        first_segment_id = first["segment"]["segment_id"]
        ctrl.limited_motion_intent = {"v": 0.0917, "omega": 0.012}
        second = telemetry.update(
            ctrl=ctrl,
            ekf_state={
                "x": 0.009,
                "y": 0.0,
                "theta_deg": 0.1,
                "v": 0.09,
                "omega_rad_s": 0.01,
            },
            now=1.12,
        )

        self.assertEqual(second["segment"]["segment_id"], first_segment_id)
        self.assertAlmostEqual(second["segment_age_s"], 0.12)
        self.assertTrue(second["actual_measurement_ready"])

    def test_publishes_segment_age_and_separate_ekf_pose_velocity(self):
        ctrl = SimpleNamespace(
            motion_execution_mode="TRACK_EXEC",
            requested_track_reference={"left_mps": -0.02, "right_mps": 0.02},
            limited_motion_intent={"v": 0.0, "omega": 0.2},
            requested_motion_intent={"v": 0.0, "omega": 0.2},
            active_motion_command_layer="TRACK_REFERENCE",
            active_motion_command_type="set_track_velocity",
            active_motion_command_source="STATE",
            motion_executor=SimpleNamespace(track_width=0.20),
            track_target_left_mps=-0.02,
            track_target_right_mps=0.02,
            motion_quality_status={
                "quality_state": "NOMINAL",
                "estimator_consistency": {
                    "confidence": 0.9,
                    "localization_mode": "TRACKING",
                },
            },
            encoder_pipeline_status={
                "canonical_velocity": {
                    "left_mps": -0.04,
                    "right_mps": 0.04,
                },
                "combined_trust": 0.9,
                "snapshot_health": "OK",
                "anomaly_active": False,
            },
            stop_status={},
            motion_public_target={},
            behavior_motion_status={},
            heading_controller_status={},
            target_pose=None,
            sm=SimpleNamespace(get_current_state_name=lambda: "ROTATE"),
        )
        telemetry = MotionPhysicalTelemetry()

        telemetry.update(
            ctrl=ctrl,
            ekf_state={
                "x": 0.0,
                "y": 0.0,
                "theta_deg": 0.0,
                "v": 0.0,
                "omega_rad_s": 0.0,
            },
            now=1.0,
        )
        out = telemetry.update(
            ctrl=ctrl,
            ekf_state={
                "x": 0.01,
                "y": 0.0,
                "theta_deg": 2.0,
                "v": 0.04,
                "omega_rad_s": 0.3,
            },
            now=1.1,
        )

        self.assertAlmostEqual(out["segment_age_s"], 0.1)
        self.assertAlmostEqual(out["ekf_linear_mps"], 0.04)
        self.assertAlmostEqual(out["pose_linear_mps"], 0.1)
        self.assertAlmostEqual(out["pose_angular_dps"], 20.0)
        self.assertAlmostEqual(
            out["actual_measurement_gate"]["segment_age_s"],
            0.1,
        )
        self.assertTrue(out["actual_primitive_corroboration"]["applied"])
        self.assertAlmostEqual(out["actual_linear_for_primitive_mps"], 0.0)
        self.assertEqual(out["turn_primitive_actual"], "IN_PLACE_ROTATE")

    def test_same_sign_encoder_motion_does_not_override_actual_arc(self):
        ctrl = SimpleNamespace(
            motion_execution_mode="TRACK_EXEC",
            requested_track_reference={"left_mps": 0.02, "right_mps": 0.05},
            limited_motion_intent={"v": 0.035, "omega": 0.15},
            requested_motion_intent={"v": 0.035, "omega": 0.15},
            active_motion_command_layer="TRACK_REFERENCE",
            active_motion_command_type="set_track_velocity",
            active_motion_command_source="STATE",
            motion_executor=SimpleNamespace(track_width=0.20),
            track_target_left_mps=0.02,
            track_target_right_mps=0.05,
            encoder_pipeline_status={
                "canonical_velocity": {
                    "left_mps": 0.02,
                    "right_mps": 0.05,
                },
                "combined_trust": 0.9,
                "snapshot_health": "OK",
                "anomaly_active": False,
            },
            motion_quality_status={
                "quality_state": "NOMINAL",
                "estimator_consistency": {
                    "confidence": 0.9,
                    "localization_mode": "TRACKING",
                },
            },
            stop_status={},
            motion_public_target={},
            behavior_motion_status={},
            heading_controller_status={},
            target_pose=None,
            sm=SimpleNamespace(get_current_state_name=lambda: "FORWARD"),
        )
        telemetry = MotionPhysicalTelemetry()
        telemetry.update(
            ctrl=ctrl,
            ekf_state={
                "x": 0.0,
                "y": 0.0,
                "theta_deg": 0.0,
                "v": 0.03,
                "omega_rad_s": 0.2,
            },
            now=1.0,
        )
        out = telemetry.update(
            ctrl=ctrl,
            ekf_state={
                "x": 0.003,
                "y": 0.0,
                "theta_deg": 1.0,
                "v": 0.03,
                "omega_rad_s": 0.2,
            },
            now=1.1,
        )

        self.assertFalse(out["actual_primitive_corroboration"]["applied"])
        self.assertAlmostEqual(out["actual_linear_for_primitive_mps"], 0.03)
        self.assertEqual(out["turn_primitive_actual"], "DIFF_ARC_SHARP")

    def test_update_does_not_json_serialize_signature_in_control_path(self):
        ctrl = SimpleNamespace(
            motion_execution_mode="TWIST_EXEC",
            requested_track_reference={"left_mps": None, "right_mps": None},
            limited_motion_intent={"v": 0.09, "omega": 0.0},
            requested_motion_intent={"v": 0.09, "omega": 0.0},
            active_motion_command_layer="MOTION_TARGET",
            active_motion_command_type="set_twist",
            active_motion_command_source="STATE",
            motion_executor=SimpleNamespace(track_width=0.3557),
            track_target_left_mps=0.09,
            track_target_right_mps=0.09,
            motion_quality_status={},
            encoder_pipeline_status={},
            stop_status={},
            motion_public_target={},
            behavior_motion_status={},
            heading_controller_status={},
            target_pose={"x": 1.0, "y": 2.0, "theta_deg": 0.0},
            sm=SimpleNamespace(get_current_state_name=lambda: "FORWARD"),
        )
        telemetry = MotionPhysicalTelemetry()

        with patch("json.dumps", side_effect=AssertionError("json serialize")):
            out = telemetry.update(
                ctrl=ctrl,
                ekf_state={
                    "x": 0.0,
                    "y": 0.0,
                    "theta_deg": 0.0,
                    "v": 0.08,
                    "omega_rad_s": 0.0,
                },
                now=1.0,
            )

        self.assertEqual(out["segment"]["segment_id"], 1)

    def test_tuple_signature_preserves_old_json_segment_decisions(self):
        samples = [
            {
                "layer": "MOTION_TARGET",
                "command_type": "set_twist",
                "source": "STATE",
                "cmd_linear_mps": 0.150004,
                "cmd_angular_dps": 0.00004,
                "targets": {
                    "target_distance_m": None,
                    "target_heading_deg": None,
                    "target_pose": None,
                },
            },
            {
                "layer": "MOTION_TARGET",
                "command_type": "set_twist",
                "source": "STATE",
                "cmd_linear_mps": 0.150003,
                "cmd_angular_dps": 0.00003,
                "targets": {
                    "target_distance_m": None,
                    "target_heading_deg": None,
                    "target_pose": None,
                },
            },
            {
                "layer": "MOTION_TARGET",
                "command_type": "set_twist",
                "source": "STATE",
                "cmd_linear_mps": 0.150003,
                "cmd_angular_dps": 0.00003,
                "targets": {
                    "target_distance_m": 1.23444,
                    "target_heading_deg": None,
                    "target_pose": {
                        "x": 1.0,
                        "y": [2.0, {"inner": 3.14159265, "none": None}],
                        "meta": {"b": True, "a": "z"},
                    },
                },
            },
            {
                "layer": "MOTION_TARGET",
                "command_type": "set_twist",
                "source": "STATE",
                "cmd_linear_mps": 0.150003,
                "cmd_angular_dps": 0.00003,
                "targets": {
                    "target_distance_m": 1.23443,
                    "target_heading_deg": None,
                    "target_pose": {
                        "meta": {"a": "z", "b": True},
                        "y": [2.0, {"none": None, "inner": 3.14159265}],
                        "x": 1.0,
                    },
                },
            },
            {
                "layer": "MOTION_TARGET",
                "command_type": "set_twist",
                "source": "STATE",
                "cmd_linear_mps": 0.150003,
                "cmd_angular_dps": 0.00003,
                "targets": {
                    "target_distance_m": 1.23443,
                    "target_heading_deg": None,
                    "target_pose": {
                        "meta": {"a": "z", "b": True},
                        "y": [2.0, {"none": None, "inner": 3.1415926}],
                        "x": 1.0,
                    },
                },
            },
            {
                "layer": "TRACK_REFERENCE",
                "command_type": "set_track_velocity",
                "source": "STATE",
                "cmd_linear_mps": 0.0,
                "cmd_angular_dps": 45.12346,
                "targets": {
                    "target_distance_m": None,
                    "target_heading_deg": 90.00004,
                    "target_pose": {"theta_deg": 90.00004, "nested": [{"v": 0.2}]},
                },
            },
            {
                "layer": "TRACK_REFERENCE",
                "command_type": "set_track_velocity",
                "source": "STATE",
                "cmd_linear_mps": 0.0,
                "cmd_angular_dps": 45.12345,
                "targets": {
                    "target_distance_m": None,
                    "target_heading_deg": 90.00005,
                    "target_pose": {"nested": [{"v": 0.2}], "theta_deg": 90.00004},
                },
            },
        ]

        old_signatures = [_old_json_signature(**sample) for sample in samples]
        new_signatures = [MotionPhysicalTelemetry._signature(**sample) for sample in samples]
        old_decisions = [
            old_signatures[idx] == old_signatures[idx - 1]
            for idx in range(1, len(samples))
        ]
        new_decisions = [
            new_signatures[idx] == new_signatures[idx - 1]
            for idx in range(1, len(samples))
        ]

        self.assertEqual(new_decisions, old_decisions)


if __name__ == "__main__":
    unittest.main()
