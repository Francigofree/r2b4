#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
from unittest import mock

from tools import M3_room_cruise_unified_validator as validator


def _preflight_sample(**overrides):
    sample = {
        "state": "IDLE",
        "control_mode": "UNIFIED",
        "motion_state_mode": "UNIFIED",
        "motion_state_mode_raw": "UNIFIED",
        "camera_enabled": False,
        "peripherals": {
            "camera": False,
            "lidar": True,
            "encoder": True,
            "imu": True,
        },
        "pwm_left": 0.0,
        "pwm_right": 0.0,
        "resolved_v": 0.0,
        "resolved_omega": 0.0,
        "actual_v": 0.0,
        "actual_omega": 0.0,
        "front_m": 1.2,
        "lidar_pose_confidence": 0.8,
        "status_age_s": 0.1,
        "lidar_health": "OK",
        "encoder_reliability_health": "OK",
        "imu_health": "OK",
        "safety_allow": True,
        "watchdog_stop_triggered": False,
        "logger_dropped_messages": 0,
        "logger_write_errors": 0,
        "logger_queue_depth": 0,
        "logger_flush_duration_ms": 1.0,
    }
    sample.update(overrides)
    return sample


def _runtime_sample(**overrides):
    sample = _preflight_sample(state="FORWARD", resolved_v=0.18, actual_v=0.17, front_m=0.9)
    sample.update(
        {
            "room_cruise_v2_active": True,
            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
            "watchdog_freq_hz": 50.0,
            "watchdog_period_s": 0.020,
            "loop_budget_total_ema_ms": 18.0,
            "lidar_raw_scan_rate_hz": 10.0,
            "lidar_matcher_latency_ms": 50.0,
            "lidar_ekf_applied_gap_s": 0.08,
            "slow_tick_count": 1,
            "slow_observed_tick_count": 10,
            "slow_io_event_count": 0,
            "slow_lidar_spike_count": 0,
            "slow_gc_count": 0,
        }
    )
    sample.update(overrides)
    return sample


def _continuous_sample(index, **overrides):
    sample = _runtime_sample(
        ts=float(index),
        dt_s=1.0,
        room_cruise_v2_active=True,
        m3_moving_cmd=True,
        m3_stop_window=False,
        m3_track_exec=True,
        pwm_left=0.22,
        pwm_right=0.22,
        motion_segment_age_s=2.0,
        localization_allow_motion=True,
        localization_mode="TRACKING",
        localization_truth_consistent=True,
        command_owner_conflict=False,
        active_route_count=1,
        primitive_contract_violation=False,
        control_execution_contract_violation=False,
        encoder_direction_switch_recent=False,
        pose={"x": 0.02 * index, "y": 0.0, "theta_deg": 0.0},
    )
    sample.update(overrides)
    return sample


def _continuity_pass():
    return {
        "contract_id": validator.CONTINUOUS_ROOM_CRUISE_CONTRACT_ID,
        "status": "PASS",
        "success": True,
        "failed_gates": [],
        "metrics": {},
    }


class TestM3RoomCruiseUnifiedValidator(unittest.TestCase):
    def test_parse_ts_accepts_structured_system_wall_timestamp(self):
        parsed = validator._parse_ts({"wall_ts": "2026-07-17T19:29:16Z"})

        self.assertEqual(parsed, 1784316556.0)

    def test_collect_system_metrics_reads_wall_timestamp_rows(self):
        class _SessionDir:
            name = "session_unit"

            def __truediv__(self, name):
                self.path_name = name
                return self

            def exists(self):
                return True

            def read_text(self, encoding):
                return (
                    '{"wall_ts":"2026-07-17T19:29:16Z","event_type":"system_health",'
                    '"data":{"cpu_percent":42.0,"cpu_temp_c":55.0,'
                    '"sd_write_latency_ms":12.0,"throttled":"0x0"}}\n'
                )

        with mock.patch.object(validator, "_latest_session_dirs", return_value=[_SessionDir()]):
            metrics = validator._collect_system_metrics(1784316555.0, 1784316557.0)

        self.assertEqual(metrics["sample_count"], 1)
        self.assertEqual(metrics["cpu_percent"]["p95"], 42.0)
        self.assertEqual(metrics["throttled_bad_count"], 0)

    def test_preflight_passes_when_idle_unified_and_camera_off(self):
        result = validator.analyze_preflight([_preflight_sample() for _ in range(8)])

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failed_gates"], [])
        self.assertEqual(result["gates"]["camera_off"]["status"], "PASS")
        self.assertEqual(result["gates"]["unified_motion_mode"]["status"], "PASS")

    def test_preflight_fails_when_camera_is_enabled(self):
        result = validator.analyze_preflight(
            [
                _preflight_sample(
                    camera_enabled=True,
                    peripherals={
                        "camera": True,
                        "lidar": True,
                        "encoder": True,
                        "imu": True,
                    },
                )
                for _ in range(8)
            ]
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("camera_off", result["failed_gates"])

    def test_runtime_allows_old_m3_repeatability_inconclusive_for_one_run(self):
        samples = [_runtime_sample() for _ in range(20)]
        base_result = {"status": "PASS", "summary": {"status": "PASS"}}
        m3_result = {
            "status": "INCONCLUSIVE",
            "failed_gates": [],
            "inconclusive_gates": ["repeatability"],
            "metrics": {
                "integrity": {
                    "forbidden_path_samples": 0,
                    "owner_conflict_samples": 0,
                    "motion_actual_ssot_bad_samples": 0,
                    "m3_track_route_bad_samples": 0,
                },
                "safety": {
                    "safety_event_samples": 0,
                    "nonfinite_command_samples": 0,
                    "min_clearance_m": 0.7,
                },
                "timing": {
                    "frequency_p10_hz": 50.0,
                    "frequency_below_45_ratio": 0.0,
                    "dt_p95_s": 0.020,
                    "loop_budget_total_ema_p95_ms": 18.0,
                },
            },
        }
        system_metrics = {
            "sample_count": 3,
            "cpu_percent": {"p95": 40.0},
            "cpu_temp_c": {"max": 55.0},
            "sd_write_latency_ms": {"p95": 10.0, "max": 12.0},
            "throttled_bad_count": 0,
        }

        result = validator.analyze_runtime(
            samples,
            base_result=base_result,
            m3_result=m3_result,
            system_metrics=system_metrics,
            continuity_result=_continuity_pass(),
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["gates"]["motion_quality_m3"]["status"], "PASS")

    def test_runtime_keeps_out_of_scope_old_quality_failures_diagnostic(self):
        samples = [_runtime_sample() for _ in range(20)]
        m3_result = {
            "status": "FAIL",
            "failed_gates": ["wheel_left_forward"],
            "inconclusive_gates": [],
            "metrics": {
                "integrity": {
                    "forbidden_path_samples": 0,
                    "owner_conflict_samples": 0,
                    "motion_actual_ssot_bad_samples": 0,
                    "m3_track_route_bad_samples": 0,
                },
                "safety": {
                    "safety_event_samples": 0,
                    "nonfinite_command_samples": 0,
                    "min_clearance_m": 0.7,
                },
                "timing": {
                    "frequency_p10_hz": 50.0,
                    "frequency_below_45_ratio": 0.0,
                    "dt_p95_s": 0.020,
                    "loop_budget_total_ema_p95_ms": 18.0,
                },
            },
        }
        system_metrics = {
            "sample_count": 3,
            "cpu_percent": {"p95": 40.0},
            "cpu_temp_c": {"max": 55.0},
            "sd_write_latency_ms": {"p95": 10.0, "max": 12.0},
            "throttled_bad_count": 0,
        }

        result = validator.analyze_runtime(
            samples,
            base_result={"status": "FAIL", "summary": {"status": "FAIL"}},
            m3_result=m3_result,
            system_metrics=system_metrics,
            continuity_result=_continuity_pass(),
        )

        self.assertEqual(result["status"], "PASS")
        self.assertIn("base_room_cruise_v2", result["diagnostic_failed_gates"])
        self.assertIn("motion_quality_m3", result["diagnostic_failed_gates"])
        self.assertFalse(result["gates"]["base_room_cruise_v2"]["required"])

    def test_continuous_room_cruise_passes_one_uninterrupted_minute(self):
        samples = [_continuous_sample(index) for index in range(60)]

        result = validator.analyze_continuous_room_cruise(
            samples,
            base_result={"summary": {"duration_s": 60.5, "progress_m": 1.2}},
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(
            result["contract_id"],
            validator.CONTINUOUS_ROOM_CRUISE_CONTRACT_ID,
        )

    def test_continuous_room_cruise_fails_internal_activation_dropout(self):
        samples = [_continuous_sample(index) for index in range(60)]
        samples[30]["room_cruise_v2_active"] = False

        result = validator.analyze_continuous_room_cruise(
            samples,
            base_result={"summary": {"duration_s": 60.5, "progress_m": 1.2}},
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("single_uninterrupted_run", result["failed_gates"])

    def test_continuous_room_cruise_fails_settled_pwm_loss_episode(self):
        samples = [_continuous_sample(index) for index in range(60)]
        for index in (20, 21):
            samples[index]["pwm_left"] = 0.0
            samples[index]["pwm_right"] = 0.0

        result = validator.analyze_continuous_room_cruise(
            samples,
            base_result={"summary": {"duration_s": 60.5, "progress_m": 1.2}},
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("no_command_or_pwm_loss", result["failed_gates"])

    def test_continuous_room_cruise_allows_obstacle_justified_stop(self):
        samples = [_continuous_sample(index) for index in range(60)]
        for index in (20, 21):
            samples[index].update(
                {
                    "m3_moving_cmd": False,
                    "m3_stop_window": True,
                    "m3_obstacle_near": True,
                    "resolved_v": 0.0,
                    "resolved_omega": 0.0,
                    "pwm_left": 0.0,
                    "pwm_right": 0.0,
                }
            )

        result = validator.analyze_continuous_room_cruise(
            samples,
            base_result={"summary": {"duration_s": 60.5, "progress_m": 1.2}},
        )

        self.assertEqual(result["status"], "PASS")

    def test_continuous_room_cruise_fails_unjustified_internal_stop(self):
        samples = [_continuous_sample(index) for index in range(60)]
        for index in (20, 21):
            samples[index].update(
                {
                    "m3_moving_cmd": False,
                    "m3_stop_window": True,
                    "resolved_v": 0.0,
                    "resolved_omega": 0.0,
                    "pwm_left": 0.0,
                    "pwm_right": 0.0,
                }
            )

        result = validator.analyze_continuous_room_cruise(
            samples,
            base_result={"summary": {"duration_s": 60.5, "progress_m": 1.2}},
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("no_unjustified_stop_start", result["failed_gates"])

    def test_runtime_slow_ratios_use_control_tick_counter_deltas(self):
        samples = [
            _runtime_sample(
                slow_tick_count=100 + index * 4,
                slow_observed_tick_count=1000 + index * 20,
                slow_io_event_count=200 + index,
            )
            for index in range(6)
        ]

        metrics = validator._runtime_sample_metrics(samples)

        self.assertEqual(metrics["slow_tick_delta"], 20)
        self.assertEqual(metrics["slow_observed_tick_delta"], 100)
        self.assertEqual(metrics["slow_io_event_delta"], 5)
        self.assertAlmostEqual(metrics["slow_tick_ratio"], 0.20)
        self.assertAlmostEqual(metrics["slow_io_ratio"], 0.05)

    def test_runtime_slow_ratios_are_not_divided_by_validator_sample_count(self):
        samples = [
            _runtime_sample(
                slow_tick_count=700 + index * 5,
                slow_observed_tick_count=3000 + index * 25,
                slow_io_event_count=280 + index * 2,
            )
            for index in range(5)
        ]

        metrics = validator._runtime_sample_metrics(samples)

        self.assertLess(metrics["slow_tick_ratio"], 0.25)
        self.assertLess(metrics["slow_io_ratio"], 0.12)

    def test_encoder_transition_anomaly_is_reported_but_not_persistent(self):
        samples = [
            _runtime_sample(
                encoder_anomaly_active=True,
                encoder_direction_switch_recent=True,
                encoder_symmetry_fault_active=False,
                motion_segment_age_s=0.18,
            )
            for _ in range(4)
        ]

        metrics = validator._runtime_sample_metrics(samples)

        self.assertEqual(metrics["encoder_anomaly_samples"], 4)
        self.assertEqual(metrics["encoder_transient_anomaly_samples"], 4)
        self.assertEqual(metrics["encoder_persistent_anomaly_samples"], 0)

    def test_encoder_anomaly_after_settle_is_persistent(self):
        samples = [
            _runtime_sample(
                encoder_anomaly_active=True,
                encoder_direction_switch_recent=False,
                encoder_symmetry_fault_active=False,
                motion_segment_age_s=1.2,
            )
        ]

        metrics = validator._runtime_sample_metrics(samples)

        self.assertEqual(metrics["encoder_persistent_anomaly_samples"], 1)


if __name__ == "__main__":
    unittest.main()
