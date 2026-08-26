#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from middleware.peripheral_usage import set_peripherals
from safety import safety_supervisor as safety_supervisor_module
from safety.safety_supervisor import SafetySupervisor
from state import RobotState


class _DummyLogger:
    def warn(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass


class _DummyTelemetry:
    def emit_audit(self, *_args, **_kwargs):
        pass


class _DummySM:
    def __init__(self):
        self.current_enum = RobotState.IDLE


class _DummyIMUService:
    def __init__(self, snapshot):
        self.snapshot = snapshot

    def get_snapshot(self):
        return self.snapshot


class _DummyEncoder:
    def __init__(self):
        self.threshold = 0.0
        self.health = "OK"
        self.pin_a = 1
        self.pin_b = 2


class _DummyController:
    def __init__(self, status_path: str):
        self.status_path = status_path
        self.sm = _DummySM()
        self.logger = _DummyLogger()
        self.telemetry = _DummyTelemetry()
        self.imu_driver = SimpleNamespace(provider="bno055", initialized=True)
        self.imu_service = _DummyIMUService(
            SimpleNamespace(
                timestamp=time.monotonic(),
                health="OK",
                gyro=(0.0, 0.0, 0.0),
            )
        )
        self.enc_l = _DummyEncoder()
        self.enc_r = _DummyEncoder()
        self.service_pwm_command = {}
        self.v_target = 0.0
        self.omega_target = 0.0
        self.motion_command_source = "MANUAL"
        self.recovery_mobility_mode = False
        self.lidar_last_update = time.monotonic()
        self.lidar_health = "OK"
        self.lidar_summary = {
            "scan_count_filtered": 16,
            "matcher_called": True,
            "scan_seq": 50,
        }
        self.lidar_odom_runtime_status = {
            "accepted": 10,
            "candidate_created": 10,
            "candidate_available": True,
            "candidate_age_s": 0.03,
            "candidate_confidence": 0.45,
            "latest_age_s": 0.08,
            "latest_confidence": 0.41,
        }


class TestSafetySupervisorAmrGuard(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.status_path = str(Path(self.tempdir.name) / "status.json")
        set_peripherals(
            {
                "lidar": True,
                "encoder": True,
                "imu": True,
            },
            status_path=self.status_path,
        )
        self.ctrl = _DummyController(self.status_path)
        self.sup = SafetySupervisor(self.ctrl)

    def tearDown(self):
        self.tempdir.cleanup()

    def test_amr_guard_blocks_state_source_when_lidar_quality_bad(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.omega_target = 0.0
        self.ctrl.lidar_summary.update(
            {
                "scan_count_filtered": 1,
                "matcher_called": False,
                "matcher_reason": "TOO_FEW_POINTS",
            }
        )
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "accepted": 0,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.0,
                "latest_age_s": 99.0,
                "latest_confidence": 0.0,
            }
        )
        self.ctrl.lidar_last_update = time.monotonic()

        decision = None
        for _ in range(int(self.sup.amr_lidar_bad_confirm_ticks)):
            self.ctrl.lidar_summary["scan_seq"] += 1
            self.ctrl.lidar_last_update = time.monotonic()
            decision = self.sup.evaluate()

        self.assertIsNotNone(decision)
        self.assertFalse(bool(decision.allow))
        self.assertIn("AMR_LIDAR_GUARD", str(decision.reason))
        self.assertIn("scan", str(decision.reason))

    def test_lidar_enabled_uses_cached_peripheral_state(self):
        calls = []
        original = safety_supervisor_module.get_cached_peripherals

        def fake_get_cached_peripherals(**kwargs):
            calls.append(dict(kwargs))
            return {"lidar": False}

        try:
            safety_supervisor_module.get_cached_peripherals = fake_get_cached_peripherals
            self.assertFalse(safety_supervisor_module._is_lidar_enabled(self.ctrl))
        finally:
            safety_supervisor_module.get_cached_peripherals = original

        self.assertEqual(calls, [{"status_path": self.status_path}])

    def test_amr_guard_does_not_count_one_bad_scan_as_three_control_ticks(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.omega_target = 0.0
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_created": 11,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.21,
                "latest_age_s": 0.02,
                "latest_confidence": 0.21,
            }
        )
        self.ctrl.lidar_last_update = 123.0

        decisions = [self.sup.evaluate() for _ in range(10)]

        self.assertTrue(all(bool(decision.allow) for decision in decisions))
        diag = self.sup.status()["amr_lidar_guard"]
        self.assertEqual(diag["bad_observation_count"], 1)
        self.assertEqual(diag["confirmation_unit"], "distinct_lidar_observation")

    def test_amr_guard_matcher_confidence_ignores_new_raw_scan_identity(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_created": 11,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.21,
                "latest_age_s": 0.02,
                "latest_confidence": 0.21,
            }
        )

        for raw_scan_id in range(60, 70):
            self.ctrl.lidar_summary["scan_seq"] = raw_scan_id
            self.ctrl.lidar_last_update = time.monotonic()
            self.assertTrue(self.sup.evaluate().allow)

        diag = self.sup.status()["amr_lidar_guard"]
        self.assertEqual(diag["bad_observation_count"], 1)
        self.assertEqual(diag["last_observation_id"][0], "quality")
        self.assertIn(("matcher_result", 11), diag["last_observation_id"][1])

    def test_amr_guard_prefers_explicit_evidence_ids_over_legacy_counters(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_summary.update({"raw_scan_id": 501, "scan_seq": 999})
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_id": 701,
                "matcher_result_id": 701,
                "candidate_created": 999,
                "lidar_odometry_measurement_id": 801,
                "accepted": 999,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.21,
                "latest_age_s": 0.02,
                "latest_confidence": 0.10,
            }
        )

        self.assertTrue(self.sup.evaluate().allow)

        observation_id = self.sup.status()["amr_lidar_guard"]["last_observation_id"]
        self.assertIn(("matcher_result", 701), observation_id[1])
        self.assertNotIn(("matcher_result", 999), observation_id[1])

        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_available": False,
                "candidate_age_s": 9.0,
                "candidate_confidence": 0.10,
                "latest_confidence": 0.21,
            }
        )
        self.assertTrue(self.sup.evaluate().allow)
        observation_id = self.sup.status()["amr_lidar_guard"]["last_observation_id"]
        self.assertIn(("lidar_odometry_measurement", 801), observation_id[1])
        self.assertNotIn(("lidar_odometry_measurement", 999), observation_id[1])

    def test_amr_guard_three_distinct_bad_matcher_results_still_stop(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.21,
                "latest_age_s": 0.02,
                "latest_confidence": 0.21,
            }
        )

        decision = None
        for candidate_id in (21, 22, 23):
            self.ctrl.lidar_odom_runtime_status["candidate_created"] = candidate_id
            decision = self.sup.evaluate()

        self.assertIsNotNone(decision)
        self.assertFalse(decision.allow)
        self.assertIn("LOW_CONF", decision.reason)
        self.assertIn("(3 scan)", decision.reason)

    def test_amr_guard_latest_confidence_uses_odometry_measurement_id(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "accepted": 7,
                "candidate_created": 11,
                "candidate_available": False,
                "candidate_age_s": 9.0,
                "candidate_confidence": 0.10,
                "latest_age_s": 0.02,
                "latest_confidence": 0.21,
            }
        )

        self.assertTrue(self.sup.evaluate().allow)
        first_id = self.sup.status()["amr_lidar_guard"]["last_observation_id"]
        self.ctrl.lidar_summary["scan_seq"] += 1
        self.ctrl.lidar_last_update += 0.1
        self.assertTrue(self.sup.evaluate().allow)
        second_id = self.sup.status()["amr_lidar_guard"]["last_observation_id"]

        self.assertEqual(first_id, second_id)
        self.assertIn(("lidar_odometry_measurement", 7), first_id[1])

    def test_amr_guard_stops_multimodal_integrity_despite_good_measurement(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.82,
                "candidate_integrity_score": 0.10,
                "candidate_integrity_state": "MULTIMODAL",
                "latest_age_s": 9.0,
            }
        )

        decision = None
        for candidate_id in (31, 32, 33):
            self.ctrl.lidar_odom_runtime_status["candidate_id"] = candidate_id
            self.ctrl.lidar_odom_runtime_status["matcher_result_id"] = candidate_id
            decision = self.sup.evaluate()

        self.assertIsNotNone(decision)
        self.assertFalse(decision.allow)
        self.assertIn("LOCALIZATION_INTEGRITY_MULTIMODAL", decision.reason)

    def test_amr_guard_accepts_separate_good_measurement_and_integrity(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_id": 41,
                "matcher_result_id": 41,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.68,
                "candidate_integrity_score": 0.91,
                "candidate_integrity_state": "OK",
                "latest_age_s": 9.0,
            }
        )

        self.assertTrue(self.sup.evaluate().allow)
        self.assertEqual(self.sup._check_amr_lidar_quality(), (True, "OK"))

    def test_amr_guard_uses_measurement_confidence_during_tracking_reacquire(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_available": True,
                "candidate_age_s": 0.02,
                # The estimator intentionally emits this promotion sentinel
                # while three high-quality poses are being reacquired.
                "candidate_confidence": 0.179999,
                "candidate_measurement_confidence": 0.91,
                "candidate_integrity_score": 0.97,
                "candidate_integrity_state": "OK",
                "tracking_ready": False,
                "tracking_reacquire_streak": 2,
                "tracking_reacquire_required": 3,
                "localization_status": "tracking_reacquire",
                "latest_age_s": 0.40,
                "latest_confidence": 0.90,
                "latest_measurement_confidence": 0.90,
            }
        )

        for candidate_id in (201, 202, 203):
            self.ctrl.lidar_odom_runtime_status.update(
                {"candidate_id": candidate_id, "matcher_result_id": candidate_id}
            )
            self.assertTrue(self.sup.evaluate().allow)

        evidence = self.sup._amr_lidar_evidence_snapshot()
        self.assertAlmostEqual(evidence["selected_confidence"], 0.91)
        self.assertEqual(
            evidence["selected_confidence_source"],
            "candidate_measurement_confidence",
        )
        self.assertEqual(
            self.sup.status()["amr_lidar_guard"]["bad_observation_count"],
            0,
        )

    def test_amr_guard_low_measurement_confidence_still_stops(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.90,
                "candidate_measurement_confidence": 0.10,
                "candidate_integrity_score": 0.95,
                "candidate_integrity_state": "OK",
                "latest_age_s": 9.0,
            }
        )

        decision = None
        for candidate_id in (301, 302, 303):
            self.ctrl.lidar_odom_runtime_status.update(
                {"candidate_id": candidate_id, "matcher_result_id": candidate_id}
            )
            decision = self.sup.evaluate()

        self.assertIsNotNone(decision)
        self.assertFalse(decision.allow)
        self.assertIn("LOW_CONF(0.100)", decision.reason)
        self.assertIn("(3 scan)", decision.reason)

    def test_amr_guard_never_pairs_fresh_candidate_with_stale_high_measurement(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_id": 701,
                "matcher_result_id": 701,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.10,
                "lidar_odometry_measurement_id": 801,
                "latest_age_s": 9.0,
                "latest_confidence": 0.90,
            }
        )

        ok, reason = self.sup._check_amr_lidar_quality()
        self.assertFalse(ok)
        self.assertEqual(reason, "LOW_CONF(0.100)")

        decision = None
        for candidate_id in (701, 702, 703):
            self.ctrl.lidar_odom_runtime_status.update(
                {"candidate_id": candidate_id, "matcher_result_id": candidate_id}
            )
            decision = self.sup.evaluate()

        self.assertIsNotNone(decision)
        self.assertFalse(decision.allow)
        observation_id = self.sup.status()["amr_lidar_guard"]["last_observation_id"]
        selected = ("matcher_result", 703)
        self.assertEqual(observation_id[1], ("confidence", selected))
        self.assertEqual(observation_id[2], ("freshness", (selected,)))
        self.assertNotIn(("lidar_odometry_measurement", 801), observation_id)

    def test_amr_guard_stale_candidate_cannot_supply_confidence_to_fresh_measurement(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_id": 701,
                "matcher_result_id": 701,
                "candidate_available": True,
                "candidate_age_s": 9.0,
                "candidate_confidence": 0.95,
                "lidar_odometry_measurement_id": 801,
                "latest_age_s": 0.02,
                "latest_confidence": 0.10,
            }
        )

        ok, reason = self.sup._check_amr_lidar_quality()
        self.assertFalse(ok)
        self.assertEqual(reason, "LOW_CONF(0.100)")
        self.assertTrue(self.sup.evaluate().allow)
        observation_id = self.sup.status()["amr_lidar_guard"]["last_observation_id"]
        selected = ("lidar_odometry_measurement", 801)
        self.assertEqual(observation_id[1], ("confidence", selected))
        self.assertEqual(observation_id[2], ("freshness", (selected,)))

    def test_amr_guard_stale_freshness_ignores_unrelated_raw_updates(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "accepted": 7,
                "candidate_created": 11,
                "candidate_available": False,
                "candidate_age_s": 9.0,
                "latest_age_s": 9.0,
            }
        )

        for raw_scan_id in range(60, 70):
            self.ctrl.lidar_summary["scan_seq"] = raw_scan_id
            self.ctrl.lidar_last_update = time.monotonic()
            self.assertTrue(self.sup.evaluate().allow)

        diag = self.sup.status()["amr_lidar_guard"]
        self.assertEqual(diag["bad_observation_count"], 1)
        self.assertEqual(diag["last_quality_reason"], "ODOM_STALE")
        self.assertEqual(diag["last_observation_id"][0], "freshness")

    def test_amr_guard_resets_bad_scan_count_on_distinct_good_scan(self):
        self.ctrl.motion_command_source = "STATE"
        self.ctrl.v_target = 0.12
        self.ctrl.omega_target = 0.0
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_created": 11,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.21,
                "latest_age_s": 0.02,
                "latest_confidence": 0.21,
            }
        )
        self.ctrl.lidar_last_update = 123.0
        self.assertTrue(self.sup.evaluate().allow)
        self.assertEqual(self.sup.status()["amr_lidar_guard"]["bad_observation_count"], 1)

        self.ctrl.lidar_odom_runtime_status.update(
            {
                "candidate_created": 12,
                "candidate_confidence": 0.75,
                "latest_confidence": 0.75,
            }
        )
        self.ctrl.lidar_last_update = 124.0

        self.assertTrue(self.sup.evaluate().allow)
        self.assertEqual(self.sup.status()["amr_lidar_guard"]["bad_observation_count"], 0)

    def test_amr_guard_does_not_block_manual_source(self):
        self.ctrl.motion_command_source = "MANUAL"
        self.ctrl.v_target = 0.12
        self.ctrl.omega_target = 0.0
        self.ctrl.lidar_summary.update(
            {
                "scan_count_filtered": 1,
                "matcher_called": False,
                "matcher_reason": "TOO_FEW_POINTS",
            }
        )
        self.ctrl.lidar_odom_runtime_status.update(
            {
                "accepted": 0,
                "candidate_available": True,
                "candidate_age_s": 0.02,
                "candidate_confidence": 0.0,
                "latest_age_s": 99.0,
                "latest_confidence": 0.0,
            }
        )
        self.ctrl.lidar_last_update = time.monotonic()

        decision = self.sup.evaluate()
        self.assertTrue(bool(decision.allow))

    def test_device_managed_imu_health_uses_cached_service_snapshot(self):
        self.ctrl.imu_service = _DummyIMUService(
            SimpleNamespace(
                timestamp=time.monotonic(),
                health="OK",
                gyro=(0.1, -0.2, 0.3),
            )
        )

        decision = self.sup.evaluate()

        self.assertTrue(bool(decision.allow))
        diag = self.sup.status()["sensor_health"]
        self.assertTrue(bool(diag["ok"]))
        self.assertEqual(diag["source"], "IMU_SERVICE_SNAPSHOT")

    def test_device_managed_imu_stale_snapshot_is_blocked(self):
        self.ctrl.imu_service = _DummyIMUService(
            SimpleNamespace(
                timestamp=time.monotonic() - 1.0,
                health="OK",
                gyro=(0.1, -0.2, 0.3),
            )
        )

        decision = self.sup.evaluate()

        self.assertFalse(bool(decision.allow))
        self.assertIn("SNAPSHOT ELAVULT", str(decision.reason))

    def test_device_managed_imu_recent_degraded_snapshot_uses_stale_gate(self):
        self.ctrl.imu_service = _DummyIMUService(
            SimpleNamespace(
                timestamp=time.monotonic() - 0.1,
                health="DEGRADED",
                gyro=(0.1, -0.2, 0.3),
                consecutive_errors=1,
            )
        )

        decision = self.sup.evaluate()

        self.assertTrue(bool(decision.allow))
        diag = self.sup.status()["sensor_health"]
        self.assertTrue(bool(diag["ok"]))
        self.assertIn("DEGRADED", str(diag["reason"]))

    def test_device_managed_imu_stale_degraded_snapshot_is_blocked(self):
        self.ctrl.imu_service = _DummyIMUService(
            SimpleNamespace(
                timestamp=time.monotonic() - 1.0,
                health="DEGRADED",
                gyro=(0.1, -0.2, 0.3),
                consecutive_errors=8,
            )
        )

        decision = self.sup.evaluate()

        self.assertFalse(bool(decision.allow))
        self.assertIn("SNAPSHOT ELAVULT", str(decision.reason))


if __name__ == "__main__":
    unittest.main()
