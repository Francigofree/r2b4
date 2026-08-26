import json
import unittest
from pathlib import Path
from types import SimpleNamespace

from controller.motion_readiness import EncoderReliabilityLayer


STEP_M = 0.0006


def _snapshot(
    *, ts: float, pulses: int, distance_m: float, dp: int, sample_dt: float = 0.05
) -> SimpleNamespace:
    velocity = float(dp) * STEP_M / 0.05 if dp else 0.0
    return SimpleNamespace(
        timestamp=float(ts),
        published_at=float(ts) + 0.001,
        left_velocity_raw=velocity,
        right_velocity_raw=velocity,
        left_velocity_unsigned=abs(velocity),
        right_velocity_unsigned=abs(velocity),
        left_distance=float(distance_m),
        right_distance=float(distance_m),
        left_pulses=int(pulses),
        right_pulses=int(pulses),
        left_pulse_delta=int(dp),
        right_pulse_delta=int(dp),
        left_distance_delta=float(dp) * STEP_M,
        right_distance_delta=float(dp) * STEP_M,
        left_step_distance_m=STEP_M,
        right_step_distance_m=STEP_M,
        sample_dt=float(sample_dt),
        health="OK",
        left_direction=1 if dp else 0,
        right_direction=1 if dp else 0,
        left_direction_source="QUADRATURE_AB" if dp else "QUADRATURE_IDLE",
        right_direction_source="QUADRATURE_AB" if dp else "QUADRATURE_IDLE",
        left_direction_confident=True,
        right_direction_confident=True,
        left_unresolved_pulses=0,
        right_unresolved_pulses=0,
    )


def _update(layer: EncoderReliabilityLayer, snapshot: SimpleNamespace):
    return layer.update(
        enc_snapshot=snapshot,
        pwm_l=0.19,
        pwm_r=0.19,
        v_target=0.15,
        omega_target=0.0,
        motion_state="FORWARD",
        control_mode="UNIFIED",
        now_mono=float(snapshot.published_at),
    )


class EncoderCanonicalWindowTests(unittest.TestCase):
    def test_active_config_declares_canonical_aggregation_window(self):
        root = Path(__file__).resolve().parents[1]
        cfg = json.loads((root / "conf" / "vezerles.json").read_text(encoding="utf-8"))
        rel_cfg = cfg["motion_readiness"]["encoder_reliability"]

        self.assertAlmostEqual(float(rel_cfg["pulse_aggregation_window_s"]), 0.10)
        self.assertAlmostEqual(float(rel_cfg["timing_gap_invalid_s"]), 0.04)
        self.assertNotIn("degraded_timing_s", rel_cfg)

    def test_canonical_velocity_uses_counter_endpoints_when_distance_accumulator_resets(self):
        layer = EncoderReliabilityLayer(
            {
                "wheel_base_m": 0.3557,
                "pulse_aggregation_window_s": 0.10,
                "left_step_distance_m": STEP_M,
                "right_step_distance_m": STEP_M,
                "timing_gap_invalid_s": 0.06,
            }
        )

        _update(layer, _snapshot(ts=10.00, pulses=100, distance_m=0.1000, dp=0))
        _update(layer, _snapshot(ts=10.05, pulses=112, distance_m=0.1072, dp=12))
        result = _update(
            layer,
            # A pose/reset race may re-anchor the distance accumulator, but it
            # must not change the signed Hall counter or canonical velocity.
            _snapshot(ts=10.10, pulses=124, distance_m=0.1000, dp=12),
        )

        velocity = result["canonical_velocity"]
        pulses = result["pulses_delta"]
        self.assertAlmostEqual(velocity["left_mps"], 0.144, places=6)
        self.assertAlmostEqual(velocity["right_mps"], 0.144, places=6)
        self.assertEqual(pulses["left"], 24)
        self.assertEqual(pulses["right"], 24)
        self.assertAlmostEqual(pulses["dt_aggregation_window_s"], 0.10, places=6)
        self.assertAlmostEqual(pulses["window_start_ts"], 10.00, places=6)
        self.assertAlmostEqual(pulses["window_end_ts"], 10.10, places=6)
        self.assertEqual(pulses["left_count_start"], 100)
        self.assertEqual(pulses["left_count_end"], 124)
        self.assertAlmostEqual(result["measurement_timestamp_s"], 10.10, places=6)
        self.assertAlmostEqual(result["publication_timestamp_s"], 10.101, places=6)
        self.assertAlmostEqual(result["publication_delay_s"], 0.001, places=6)

    def test_window_keeps_one_predecessor_for_exact_counter_interval(self):
        layer = EncoderReliabilityLayer(
            {
                "wheel_base_m": 0.3557,
                "pulse_aggregation_window_s": 0.10,
                "left_step_distance_m": STEP_M,
                "right_step_distance_m": STEP_M,
                "timing_gap_invalid_s": 0.06,
            }
        )
        distance = 0.0
        pulses = 0
        result = None
        for ts, dp in ((20.00, 0), (20.04, 10), (20.08, 10), (20.12, 10), (20.16, 10)):
            pulses += dp
            distance += dp * STEP_M
            result = _update(layer, _snapshot(ts=ts, pulses=pulses, distance_m=distance, dp=dp))

        self.assertIsNotNone(result)
        window = result["pulses_delta"]
        self.assertAlmostEqual(window["window_start_ts"], 20.04, places=6)
        self.assertAlmostEqual(window["window_end_ts"], 20.16, places=6)
        self.assertAlmostEqual(window["dt_aggregation_window_s"], 0.12, places=6)
        self.assertEqual(window["left"], 30)
        self.assertAlmostEqual(result["canonical_velocity"]["left_mps"], 0.15, places=6)

    def test_control_rate_samples_form_a_fresh_sliding_counter_window(self):
        layer = EncoderReliabilityLayer(
            {
                "wheel_base_m": 0.3557,
                "pulse_aggregation_window_s": 0.10,
                "left_step_distance_m": STEP_M,
                "right_step_distance_m": STEP_M,
                "timing_gap_invalid_s": 0.06,
            }
        )
        pulses = 0
        distance = 0.0
        result = None
        for index in range(7):
            dp = 0 if index == 0 else 5
            pulses += dp
            distance += dp * STEP_M
            result = _update(
                layer,
                _snapshot(
                    ts=30.0 + 0.02 * index,
                    pulses=pulses,
                    distance_m=distance,
                    dp=dp,
                ),
            )

        self.assertIsNotNone(result)
        window = result["pulses_delta"]
        self.assertAlmostEqual(window["window_start_ts"], 30.02, places=6)
        self.assertAlmostEqual(window["window_end_ts"], 30.12, places=6)
        self.assertAlmostEqual(window["dt_aggregation_window_s"], 0.10, places=6)
        self.assertEqual(window["left_count_end"] - window["left_count_start"], 25)
        self.assertAlmostEqual(result["canonical_velocity"]["left_mps"], 0.15, places=6)

    def test_long_control_gap_is_raw_only_and_rejected_from_canonical_consumers(self):
        layer = EncoderReliabilityLayer(
            {
                "wheel_base_m": 0.3557,
                "pulse_aggregation_window_s": 0.10,
                "left_step_distance_m": STEP_M,
                "right_step_distance_m": STEP_M,
                "timing_gap_invalid_s": 0.04,
            }
        )

        _update(
            layer,
            _snapshot(
                ts=40.000,
                pulses=100,
                distance_m=0.0600,
                dp=0,
                sample_dt=0.02,
            ),
        )
        invalid = _update(
            layer,
            _snapshot(
                ts=40.105,
                pulses=101,
                distance_m=0.0606,
                dp=1,
                sample_dt=0.105,
            ),
        )

        self.assertFalse(invalid["timing_valid"])
        self.assertFalse(invalid["canonical_available"])
        self.assertEqual(invalid["timing_error"], "TIMING_GAP")
        self.assertEqual(invalid["canonical_state"], "TIMING_GAP")
        self.assertEqual(invalid["flags"], ["ENCODER_TIMING_GAP"])
        self.assertIsNone(invalid["canonical_velocity"]["left_mps"])
        self.assertIsNone(invalid["canonical_velocity"]["right_mps"])
        self.assertAlmostEqual(
            invalid["raw_measurement"]["velocity"]["left_mps"],
            STEP_M / 0.105,
            places=6,
        )
        self.assertEqual(invalid["canonical_distance"]["left_delta_m"], 0.0)
        self.assertEqual(invalid["combined_trust"], 0.0)
        self.assertEqual(invalid["ekf_usage_mode"], "REJECT")
        self.assertEqual(invalid["ekf_usage_reason"], "TIMING_GAP")
        self.assertEqual(invalid["timing_gap_count"], 1)
        self.assertEqual(invalid["motion_timing_gap_count"], 1)
        self.assertEqual(invalid["idle_timing_gap_count"], 0)
        self.assertTrue(invalid["last_timing_gap"]["motion_active"])

        recovered = _update(
            layer,
            _snapshot(
                ts=40.125,
                pulses=106,
                distance_m=0.0636,
                dp=5,
                sample_dt=0.02,
            ),
        )
        self.assertTrue(recovered["timing_valid"])
        self.assertTrue(recovered["canonical_available"])
        self.assertAlmostEqual(recovered["canonical_velocity"]["left_mps"], 0.15, places=6)
        self.assertEqual(recovered["timing_gap_count"], 1)
        self.assertEqual(recovered["motion_timing_gap_count"], 1)

    def test_idle_timing_gap_is_invalid_but_not_a_motion_gap(self):
        layer = EncoderReliabilityLayer(
            {
                "wheel_base_m": 0.3557,
                "pulse_aggregation_window_s": 0.10,
                "left_step_distance_m": STEP_M,
                "right_step_distance_m": STEP_M,
                "timing_gap_invalid_s": 0.04,
            }
        )
        layer.update(
            enc_snapshot=_snapshot(
                ts=50.000,
                pulses=100,
                distance_m=0.0600,
                dp=0,
                sample_dt=0.02,
            ),
            pwm_l=0.0,
            pwm_r=0.0,
            v_target=0.0,
            omega_target=0.0,
            motion_state="IDLE",
            control_mode="UNIFIED",
            now_mono=50.001,
        )
        result = layer.update(
            enc_snapshot=_snapshot(
                ts=50.100,
                pulses=100,
                distance_m=0.0600,
                dp=0,
                sample_dt=0.10,
            ),
            pwm_l=0.0,
            pwm_r=0.0,
            v_target=0.0,
            omega_target=0.0,
            motion_state="IDLE",
            control_mode="UNIFIED",
            now_mono=50.101,
        )

        self.assertFalse(result["timing_valid"])
        self.assertEqual(result["timing_gap_count"], 1)
        self.assertEqual(result["motion_timing_gap_count"], 0)
        self.assertEqual(result["idle_timing_gap_count"], 1)
        self.assertFalse(result["last_timing_gap"]["motion_active"])
        self.assertTrue(result["last_timing_gap"]["pwm_zero"])
        self.assertTrue(result["last_timing_gap"]["command_zero"])

    def test_startup_command_gap_with_zero_pwm_is_not_a_motion_gap(self):
        layer = EncoderReliabilityLayer(
            {
                "wheel_base_m": 0.3557,
                "pulse_aggregation_window_s": 0.10,
                "left_step_distance_m": STEP_M,
                "right_step_distance_m": STEP_M,
                "timing_gap_invalid_s": 0.04,
            }
        )
        layer.update(
            enc_snapshot=_snapshot(
                ts=60.000,
                pulses=100,
                distance_m=0.0600,
                dp=0,
                sample_dt=0.02,
            ),
            pwm_l=0.0,
            pwm_r=0.0,
            v_target=0.0,
            omega_target=0.0,
            motion_state="IDLE",
            control_mode="UNIFIED",
            now_mono=60.001,
        )

        result = layer.update(
            enc_snapshot=_snapshot(
                ts=60.055,
                pulses=101,
                distance_m=0.0606,
                dp=1,
                sample_dt=0.055,
            ),
            pwm_l=0.0,
            pwm_r=0.0,
            v_target=0.15,
            omega_target=0.0,
            motion_state="FORWARD",
            control_mode="UNIFIED",
            now_mono=60.056,
        )

        self.assertFalse(result["timing_valid"])
        self.assertEqual(result["timing_gap_count"], 1)
        self.assertEqual(result["motion_timing_gap_count"], 0)
        self.assertEqual(result["idle_timing_gap_count"], 1)
        self.assertFalse(result["last_timing_gap"]["motion_active"])
        self.assertTrue(result["last_timing_gap"]["startup_sync_gap"])
        self.assertEqual(result["last_timing_gap"]["classification"], "STARTUP_SYNC_PWM_ZERO")
        self.assertTrue(result["last_timing_gap"]["pwm_zero"])
        self.assertFalse(result["last_timing_gap"]["command_zero"])
        self.assertIsNone(result["canonical_velocity"]["left_mps"])


if __name__ == "__main__":
    unittest.main()
