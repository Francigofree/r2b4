#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from control_loop import ControlLoop
from middleware.ekf_manager import EKFManager


class _EmptyService:
    def get_snapshot(self):
        return None


class _State:
    def __init__(self, name="FORWARD"):
        self.name = name
        self.robot = type("Robot", (), {"v_target": 0.0, "omega_target": 0.0})()

    def update(self, _dt):
        return None

    def get_current_state_name(self):
        return self.name


class _Core:
    def tick(self):
        return None


class _Controller:
    def __init__(self, state):
        self.cfg = {"vezerles": {"idozites": {"fo_ciklus_hz": 50.0}}}
        self.v_cmd = 0.15
        self.v_target = 0.15
        self.omega_target = 0.0
        self.speed_level = 0
        self.turn_level = 0
        self.motion_command_source = "STATE"
        self.input_vector = {"x": 0.0, "y": 0.0}
        self.turn_omega_levels = {0: 0.0}
        self.turn_mix = 1.0
        self.recovery_mobility_mode = False
        self.service_motion_active = False
        self.sm = state
        self._prev_pwm_l = 0.0
        self._prev_pwm_r = 0.0
        self.speeds_fwd = {i: 0.1 * i for i in range(10)}
        self.speeds_rev = {i: -0.1 * i for i in range(10)}


class _ReplayLidarOdometry:
    min_confidence = 0.1

    def __init__(self, payloads):
        self._payloads = list(payloads)
        self._index = 0
        self._last_payload = None
        self._delivery_status = "missing"
        self._stats = {}

    def get_odometry(self):
        payload = self._payloads[self._index] if self._index < len(self._payloads) else None
        self._index += 1
        if payload is None:
            self._delivery_status = "missing"
            return None
        self._last_payload = dict(payload)
        self._delivery_status = "available"
        return dict(payload)

    def get_stats(self):
        payload = dict(self._last_payload or {})
        return {
            **dict(self._stats),
            "last_decision": "accepted" if self._last_payload else "missing",
            "delivery_status": self._delivery_status,
            "get_odometry_result": (
                "consumed" if self._delivery_status == "available" else "missing"
            ),
            "latest_age_s": 0.05,
            "candidate_age_s": 0.05,
            "candidate_available": bool(self._last_payload),
            "candidate_confidence": float(payload.get("confidence", 0.0) or 0.0),
            "latest_confidence": float(payload.get("confidence", 0.0) or 0.0),
            "min_confidence": 0.1,
            "max_scan_age_s": 0.25,
            "raw_scan_latest_age_s": 0.05,
            "raw_scan_rate_hz": 10.0,
            "accepted": 1 if self._last_payload else 0,
            "lidar_odometry_measurement_id": payload.get(
                "lidar_odometry_measurement_id"
            ),
            "measurement_source_matcher_result_id": payload.get(
                "measurement_source_matcher_result_id"
            ),
            "measurement_source_raw_scan_id": payload.get(
                "measurement_source_raw_scan_id"
            ),
        }


def _measurement(measurement_id=1):
    payload = {
        "x": 0.25,
        "y": 0.0,
        "theta": 0.0,
        "confidence": 0.9,
        "r_scale": 1.0,
        "timestamp": time.monotonic(),
        "measurement_source_matcher_result_id": 101,
        "measurement_source_raw_scan_id": 91,
    }
    if measurement_id is not None:
        payload["lidar_odometry_measurement_id"] = measurement_id
    return payload


def _make_loop(payloads):
    state = _State("FORWARD")
    odometry = _ReplayLidarOdometry(payloads)
    manager = EKFManager(wheel_base=0.175, live_config={}, shadow_config={})
    loop = ControlLoop(
        encoder_service=_EmptyService(),
        imu_service=_EmptyService(),
        ekf_manager=manager,
        state_machine=state,
        core=_Core(),
        loop_hz=50.0,
        odometry_mode="LIDAR_FIRST",
        lidar_odometry=odometry,
    )
    ctrl = _Controller(state)
    calls = []
    update_lidar = loop.ekf.update_lidar

    def counted_update(*args, **kwargs):
        calls.append((args, kwargs))
        return update_lidar(*args, **kwargs)

    loop.ekf.update_lidar = counted_update
    return loop, ctrl, calls


class TestLidarEkfObservationContract(unittest.TestCase):
    def test_duplicate_measurement_id_is_not_applied_twice(self):
        measurement = _measurement(7)
        loop, ctrl, calls = _make_loop([measurement, measurement])

        first = loop.tick(0.02, ctrl)
        state_after_first = dict(first["ekf_state"])
        second = loop.tick(0.02, ctrl)

        self.assertEqual(len(calls), 1)
        self.assertTrue(first["lidar_odom_status"]["applied"])
        self.assertFalse(second["lidar_odom_status"]["applied"])
        self.assertEqual(
            second["lidar_odom_status"]["ekf_status"],
            "rejected_duplicate_lidar_odometry_measurement",
        )
        self.assertEqual(
            second["lidar_odom_status"]["ekf_duplicate_measurement_rejected_total"],
            1,
        )
        self.assertEqual(
            second["lidar_odom_status"]["ekf_last_applied_lidar_odometry_measurement_id"],
            7,
        )
        self.assertAlmostEqual(second["ekf_state"]["x"], state_after_first["x"], delta=1e-6)

    def test_out_of_order_measurement_id_is_fail_closed(self):
        loop, ctrl, calls = _make_loop([_measurement(9), _measurement(8)])

        loop.tick(0.02, ctrl)
        result = loop.tick(0.02, ctrl)

        self.assertEqual(len(calls), 1)
        self.assertEqual(
            result["lidar_odom_status"]["ekf_status"],
            "rejected_out_of_order_lidar_odometry_measurement",
        )

    def test_missing_measurement_id_is_fail_closed(self):
        loop, ctrl, calls = _make_loop([_measurement(None)])

        result = loop.tick(0.02, ctrl)

        self.assertEqual(calls, [])
        self.assertFalse(result["lidar_odom_status"]["applied"])
        self.assertEqual(
            result["lidar_odom_status"]["ekf_status"],
            "rejected_missing_lidar_odometry_measurement_id",
        )
        self.assertEqual(
            result["lidar_odom_status"]["ekf_missing_measurement_id_rejected_total"],
            1,
        )

    def test_retained_cadence_evidence_never_becomes_an_ekf_measurement(self):
        loop, ctrl, calls = _make_loop([_measurement(12), None])

        first = loop.tick(0.02, ctrl)
        self.assertTrue(first["lidar_odom_status"]["applied"])
        now = time.monotonic()
        loop._lidar_last_delivered_ts = now - 0.40
        loop._lidar_ekf_last_applied_ts = now - 0.40
        # This primes the exact legacy branch that used to reapply the cached
        # payload after the cadence interval.
        loop._lidar_cadence_soft_reapply_last_ts = now - 0.40

        second = loop.tick(0.02, ctrl)

        self.assertEqual(len(calls), 1)
        self.assertFalse(second["lidar_odom_status"]["applied"])
        self.assertFalse(second["lidar_odom_status"]["cadence_soft_reapply"])
        self.assertEqual(second["lidar_odom_status"]["cadence_soft_reapply_total"], 0)
        self.assertTrue(
            second["lidar_odom_status"]["cadence_retained_evidence_available"]
        )
        self.assertEqual(
            second["lidar_odom_status"]["ekf_last_applied_lidar_odometry_measurement_id"],
            12,
        )


if __name__ == "__main__":
    unittest.main()
