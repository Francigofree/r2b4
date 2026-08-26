import unittest

from tools.kit0085_motion_primitives_audit import _case_failures


class Kit0085MotionPrimitiveAuditTests(unittest.TestCase):
    def _base(self, kind):
        return {
            "kind": kind,
            "target_distance_m": 0.30,
            "max_pwm_left": 0.42,
            "max_pwm_right": 0.43,
            "failsafe_seen": False,
            "safety_block_seen": False,
            "normal_stop_confirmed": True,
        }

    def test_reverse_passes_with_signed_negative_encoder_progress(self):
        metrics = {
            **self._base("reverse"),
            "left_distance_delta_m": -0.30,
            "right_distance_delta_m": -0.29,
            "encoder_average_delta_m": -0.295,
            "ekf_chord_m": 0.28,
            "signed_yaw_deg": 3.0,
        }

        self.assertEqual(_case_failures(metrics), [])

    def test_reverse_rejects_unsigned_encoder_progress(self):
        metrics = {
            **self._base("reverse"),
            "left_distance_delta_m": 0.30,
            "right_distance_delta_m": 0.29,
            "encoder_average_delta_m": 0.295,
            "ekf_chord_m": 0.28,
            "signed_yaw_deg": 2.0,
        }

        self.assertIn("encoder_reverse_sign_missing", _case_failures(metrics))

    def test_left_arc_requires_positive_yaw_and_right_outer_distance(self):
        metrics = {
            **self._base("arc_left"),
            "left_distance_delta_m": 0.10,
            "right_distance_delta_m": 0.14,
            "encoder_average_delta_m": 0.12,
            "signed_yaw_deg": 12.0,
            "min_yaw_deg": 5.0,
        }

        self.assertEqual(_case_failures(metrics), [])

    def test_right_arc_requires_negative_yaw_and_left_outer_distance(self):
        metrics = {
            **self._base("arc_right"),
            "left_distance_delta_m": 0.15,
            "right_distance_delta_m": 0.10,
            "encoder_average_delta_m": 0.125,
            "signed_yaw_deg": -11.0,
            "min_yaw_deg": 5.0,
        }

        self.assertEqual(_case_failures(metrics), [])


if __name__ == "__main__":
    unittest.main()
