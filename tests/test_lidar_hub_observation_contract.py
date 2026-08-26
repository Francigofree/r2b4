#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from tools.lidar_1m_step import (
    _extract_lidar_observation,
    _summarize_lidar_observation_rows,
)


def _row(measurement_id, *, confidence, applied=True, raw_id=None, matcher_id=None):
    if measurement_id is None:
        return {
            "lidar_odom_applied": bool(applied),
            "lidar_odom_latest_age_s": 0.1,
            "lidar_odom_latest_confidence": float(confidence),
            "lidar_observation": {"lineage_errors": ["applied_measurement_id_missing"]},
        }
    raw_id = int(raw_id if raw_id is not None else measurement_id + 200)
    matcher_id = int(matcher_id if matcher_id is not None else measurement_id + 100)
    return {
        "lidar_odom_applied": bool(applied),
        "lidar_odom_latest_age_s": 0.1,
        "lidar_odom_latest_confidence": float(confidence),
        "lidar_observation": {
            "raw_scan_id": raw_id,
            "matcher_result_id": matcher_id,
            "candidate_id": matcher_id,
            "candidate_source_raw_scan_id": raw_id,
            "lidar_odometry_measurement_id": int(measurement_id),
            "measurement_source_matcher_result_id": matcher_id,
            "measurement_source_raw_scan_id": raw_id,
            "lineage_errors": [],
        },
    }


class TestLidarHubObservationContract(unittest.TestCase):
    def test_extracts_distinct_ids_and_applied_lineage(self):
        status = {
            "lidar_odom_status": {
                "raw_scan_id": 41,
                "matcher_result_id": 31,
                "candidate_id": 31,
                "candidate_source_raw_scan_id": 41,
                "lidar_odometry_measurement_id": 21,
                "measurement_source_matcher_result_id": 31,
                "measurement_source_raw_scan_id": 41,
                "ekf_input_lidar_odometry_measurement_id": 21,
                "ekf_last_processed_lidar_odometry_measurement_id": 21,
                "ekf_last_applied_lidar_odometry_measurement_id": 21,
                "applied": True,
            }
        }

        observation = _extract_lidar_observation(status)

        self.assertEqual(observation["raw_scan_id"], 41)
        self.assertEqual(observation["matcher_result_id"], 31)
        self.assertEqual(observation["lidar_odometry_measurement_id"], 21)
        self.assertEqual(observation["lineage_errors"], [])

    def test_four_polls_with_two_measurements_count_as_two_observations(self):
        rows = [
            _row(1, confidence=0.3),
            _row(1, confidence=0.3),
            _row(2, confidence=0.9),
            _row(2, confidence=0.9),
        ]

        summary = _summarize_lidar_observation_rows(rows)

        self.assertEqual(summary["poll_samples"], 4)
        self.assertEqual(summary["applied_status_samples"], 4)
        self.assertEqual(summary["applied_samples"], 2)
        self.assertEqual(summary["unique_lidar_odometry_measurements"], 2)
        self.assertEqual(summary["confidence_values"], [0.3, 0.9])
        self.assertEqual(summary["observation_contract_errors"], [])

    def test_reused_measurement_id_with_changed_payload_is_rejected(self):
        rows = [
            _row(7, confidence=0.4, raw_id=207, matcher_id=107),
            _row(7, confidence=0.8, raw_id=208, matcher_id=108),
        ]

        summary = _summarize_lidar_observation_rows(rows)

        self.assertEqual(summary["applied_samples"], 1)
        self.assertIn(
            "measurement_id_reused_with_changed_payload:7",
            summary["observation_contract_errors"],
        )

    def test_applied_sample_without_measurement_id_fails_closed(self):
        summary = _summarize_lidar_observation_rows([_row(None, confidence=0.8)])

        self.assertEqual(summary["applied_samples"], 0)
        self.assertEqual(summary["applied_missing_measurement_id_samples"], 1)
        self.assertIn("applied_measurement_id_missing", summary["observation_contract_errors"])


if __name__ == "__main__":
    unittest.main()
