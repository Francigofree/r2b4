#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from middleware.lidar_odometry import LidarOdometry


class TestLidarOdometryDiagnostics(unittest.TestCase):
    def test_control_loop_diagnostics_publish_is_non_blocking_when_lock_busy(self):
        odometry = LidarOdometry(config={"enabled": True})
        published = threading.Event()
        payload = {
            "control_loop_lidar_apply_status": "GEN_2",
            "control_loop_lidar_flow": {"generation": 2},
            "initialization_gate": {"generation": 2},
            "localization_health": "GEN_2",
        }

        def writer():
            odometry.publish_control_loop_diagnostics(payload)
            published.set()

        odometry._lock.acquire()
        try:
            thread = threading.Thread(target=writer)
            thread.start()
            self.assertTrue(published.wait(timeout=0.03))
        finally:
            odometry._lock.release()
        thread.join(timeout=1.0)

        self.assertTrue(published.is_set())
        stats = odometry.get_stats()
        self.assertGreaterEqual(stats["control_lock_miss_count"], 1)
        self.assertNotEqual(stats["control_loop_lidar_apply_status"], "GEN_2")

        odometry.publish_control_loop_diagnostics(payload)
        stats = odometry.get_stats()
        self.assertEqual(stats["control_loop_lidar_apply_status"], "GEN_2")
        self.assertEqual(stats["control_loop_lidar_flow"]["generation"], 2)
        self.assertEqual(stats["initialization_gate"]["generation"], 2)
        self.assertEqual(stats["localization_health"], "GEN_2")

    def test_control_loop_reads_do_not_block_when_lidar_lock_busy(self):
        odometry = LidarOdometry(config={"enabled": True})

        odometry._lock.acquire()
        try:
            self.assertIsNone(odometry.get_odometry())
            stats = odometry.get_stats()
        finally:
            odometry._lock.release()

        self.assertTrue(stats["control_lock_busy"])
        self.assertEqual(stats["delivery_status"], "lock_busy")
        self.assertEqual(stats["get_odometry_result"], "lock_busy")
        self.assertGreaterEqual(stats["control_lock_miss_count"], 2)

    def test_published_nested_diagnostics_are_detached_from_caller(self):
        odometry = LidarOdometry(config={"enabled": True})
        flow = {"measurement_contract": {"id": 11}}

        odometry.publish_control_loop_diagnostics(
            {"control_loop_lidar_flow": flow}
        )
        flow["measurement_contract"]["id"] = 99

        self.assertEqual(
            odometry.get_stats()["control_loop_lidar_flow"]["measurement_contract"]["id"],
            11,
        )

    def test_control_loop_cannot_overwrite_matcher_owned_counters(self):
        odometry = LidarOdometry(config={"enabled": True})
        accepted_before = odometry.get_stats()["accepted"]

        odometry.publish_control_loop_diagnostics(
            {
                "accepted": 999,
                "ekf_applied_samples_total": 7,
            }
        )

        stats = odometry.get_stats()
        self.assertEqual(stats["accepted"], accepted_before)
        self.assertEqual(stats["ekf_applied_samples_total"], 7)

    def test_process_resource_diagnostics_are_allowed_control_projection(self):
        odometry = LidarOdometry(config={"enabled": True})

        odometry.publish_control_loop_diagnostics(
            {
                "matcher_process_pid": 321,
                "matcher_process_alive": True,
                "matcher_process_rss_kb": 45678,
                "matcher_process_peak_rss_kb": 56789,
                "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
                "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
                "matcher_integrity_model": "R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1",
                "matcher_process_start_method": "spawn",
                "matcher_input_queue_capacity": 1,
                "matcher_result_queue_capacity": 1,
                "matcher_max_input_age_s": 0.25,
                "matcher_max_result_age_s": 0.25,
                "matcher_queue_depth": 0,
                "matcher_result_queue_depth": 0,
                "matcher_stale_result_drops": 2,
                "matcher_transport": "process_latest_only",
            }
        )

        stats = odometry.get_stats()
        self.assertEqual(stats["matcher_process_pid"], 321)
        self.assertTrue(stats["matcher_process_alive"])
        self.assertEqual(stats["matcher_process_rss_kb"], 45678)
        self.assertEqual(stats["matcher_process_peak_rss_kb"], 56789)
        self.assertEqual(
            stats["matcher_contract_id"],
            "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
        )
        self.assertEqual(
            stats["matcher_confidence_model"],
            "R2B4_SCAN_MATCH_CONFIDENCE_V2",
        )
        self.assertEqual(
            stats["matcher_integrity_model"],
            "R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1",
        )
        self.assertEqual(stats["matcher_process_start_method"], "spawn")
        self.assertEqual(stats["matcher_input_queue_capacity"], 1)
        self.assertEqual(stats["matcher_result_queue_capacity"], 1)
        self.assertEqual(stats["matcher_max_input_age_s"], 0.25)
        self.assertEqual(stats["matcher_max_result_age_s"], 0.25)
        self.assertEqual(stats["matcher_stale_result_drops"], 2)
        self.assertEqual(stats["matcher_transport"], "process_latest_only")

    def test_matcher_quality_snapshot_is_detached_from_result_payload(self):
        odometry = LidarOdometry(
            config={
                "enabled": True,
                "min_confidence": 0.2,
                "max_scan_age_s": 1.0,
                "max_delta_m": 5.0,
                "max_delta_rad": 3.2,
            }
        )
        quality = {
            "confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
            "integrity_model": "R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1",
            "measurement_confidence": 0.81,
            "localization_integrity_score": 0.93,
            "integrity_state": "OK",
            "degenerate": False,
            "degeneracy_reasons": [],
        }
        odometry.on_scan_result(
            {
                "lidar_pose_x": 0.0,
                "lidar_pose_y": 0.0,
                "lidar_pose_theta": 0.0,
                "lidar_pose_confidence": 0.8,
                "matcher_result_id": 1,
                "candidate_id": 1,
                "matcher_source_raw_scan_id": 1,
                "matcher_quality": quality,
                "matcher_degenerate": False,
                "matcher_degeneracy_reasons": [],
            }
        )
        quality["degenerate"] = True

        stats = odometry.get_stats()
        self.assertFalse(stats["matcher_quality"]["degenerate"])
        self.assertEqual(
            stats["matcher_quality"]["confidence_model"],
            "R2B4_SCAN_MATCH_CONFIDENCE_V2",
        )
        self.assertAlmostEqual(stats["candidate_measurement_confidence"], 0.81)
        self.assertAlmostEqual(stats["candidate_integrity_score"], 0.93)
        self.assertEqual(stats["candidate_integrity_state"], "OK")
        self.assertAlmostEqual(stats["latest_integrity_score"], 0.93)
        measurement = odometry.get_odometry()
        self.assertIsNotNone(measurement)
        self.assertAlmostEqual(measurement["measurement_confidence"], 0.81)
        self.assertAlmostEqual(measurement["integrity_score"], 0.93)
        self.assertEqual(measurement["integrity_state"], "OK")


if __name__ == "__main__":
    unittest.main()
