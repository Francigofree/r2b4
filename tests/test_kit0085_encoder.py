import time
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from driver.encoder import DFRobotQuadratureEncoder
from middleware.enc_estim import EncoderEstimator
from startup.phases import _encoder_stream_ready


class _FakeCallback:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _FakeLgpio:
    SET_PULL_UP = 32
    RISING_EDGE = 1
    BOTH_EDGES = 3

    def __init__(self):
        self.levels = {23: 0, 24: 1}
        self.read_count = {}
        self.callbacks = {}
        self.claimed_alerts = []
        self.claimed_inputs = []
        self.debounce_calls = []

    def gpiochip_open(self, chip):
        return 7

    def gpio_claim_alert(self, handle, pin, edge, flags=0):
        self.claimed_alerts.append((handle, pin, edge, flags))
        return 0

    def gpio_claim_input(self, handle, pin, flags=0):
        self.claimed_inputs.append((handle, pin, flags))
        return 0

    def gpio_read(self, handle, pin):
        self.read_count[pin] = int(self.read_count.get(pin, 0)) + 1
        return self.levels[pin]

    def callback(self, handle, pin, edge, func):
        callback_obj = _FakeCallback()
        self.callbacks[pin] = (edge, func, callback_obj)
        return callback_obj

    def gpio_set_debounce_micros(self, handle, pin, debounce_micros):
        self.debounce_calls.append((handle, pin, debounce_micros))
        return 0

    def gpio_free(self, handle, pin):
        return 0

    def gpiochip_close(self, handle):
        return 0


class _Counter:
    signed_counts = True
    counts_per_revolution = 663

    def __init__(self):
        self.pulse_count = 0


class _Snapshot:
    health = "OK"

    def __init__(self):
        self.timestamp = time.perf_counter()


class _Service:
    _running = True

    def get_snapshot(self):
        return _Snapshot()


class _Controller:
    def __init__(self, left, right):
        self.enc_l = left
        self.enc_r = right
        self.encoder_service = _Service()


class Kit0085EncoderTests(unittest.TestCase):
    def test_active_physics_matches_kit0085_cpr_and_wheel_radius(self):
        root = Path(__file__).resolve().parents[1]
        physics = json.loads((root / "conf/fizika.json").read_text(encoding="utf-8"))
        hardware = json.loads((root / "conf/hardver.json").read_text(encoding="utf-8"))
        cpr_hardware = int(hardware["encoderek"]["counts_per_revolution"])
        cpr_physics = int(physics["encoder_impulzus_per_fordulat"])
        expected_step = (
            2.0 * math.pi * float(physics["kerek_sugar_m"]) / float(cpr_hardware)
        )

        self.assertEqual(cpr_hardware, 663)
        self.assertEqual(cpr_physics, cpr_hardware)
        self.assertAlmostEqual(float(physics["lepes_hossz_m"]), expected_step, places=12)
        self.assertAlmostEqual(float(physics["nyomtav_szelesseg_m"]), 0.3557, places=6)

    def test_direction_decoder_matches_configured_b_phase(self):
        self.assertEqual(DFRobotQuadratureEncoder.direction_from_b(1, forward_b_level=1), 1)
        self.assertEqual(DFRobotQuadratureEncoder.direction_from_b(0, forward_b_level=1), -1)
        self.assertEqual(DFRobotQuadratureEncoder.direction_from_b(1, forward_b_level=1, invert=True), -1)

    def test_callback_counts_signed_a_rising_edges(self):
        fake = _FakeLgpio()
        with patch.dict("driver.encoder.os.environ", {"R2B4_ENCODER_EDGE_TRACE": "1"}), patch(
            "driver.encoder.lgpio",
            fake,
        ):
            enc = DFRobotQuadratureEncoder(
                23,
                24,
                name="L",
                a_debounce_micros=150,
            )
            enc.start()
            callback_a = fake.callbacks[23][1]
            callback_b = fake.callbacks[24][1]
            with patch("driver.encoder.time.perf_counter", side_effect=[10.0, 10.1]):
                callback_a(0, 23, 1, 100)
                callback_b(0, 24, 0, 150)
                callback_a(0, 23, 1, 200)
            self.assertEqual(enc.pulse_count, 0)
            self.assertEqual(enc.edge_count, 2)
            self.assertEqual(enc.a_edge_count, 2)
            self.assertEqual(enc.b_edge_count, 1)
            self.assertEqual(enc.a_rising_count, 2)
            self.assertEqual(enc.a_falling_count, 0)
            self.assertEqual(enc.b_rising_count, 0)
            self.assertEqual(enc.b_falling_count, 1)
            self.assertEqual(enc.forward_count, 1)
            self.assertEqual(enc.reverse_count, 1)
            trace = enc.recent_a_rising_events()
            self.assertEqual(
                [
                    (
                        row["sequence"],
                        row["gpio_tick"],
                        row["b_level_at_a_rising"],
                        row["direction"],
                        row["signed_pulse_count"],
                    )
                    for row in trace
                ],
                [
                    (1, 100, 1, 1, 1),
                    (2, 200, 0, -1, 0),
                ],
            )
            self.assertEqual(
                [row[1:3] for row in fake.claimed_alerts],
                [(23, fake.RISING_EDGE), (24, fake.BOTH_EDGES)],
            )
            self.assertEqual(fake.claimed_inputs, [])
            self.assertEqual(fake.debounce_calls, [(7, 23, 150)])
            self.assertEqual(fake.read_count.get(24, 0), 1)
            enc.stop()
            self.assertTrue(fake.callbacks[23][2].cancelled)
            self.assertTrue(fake.callbacks[24][2].cancelled)

    def test_edge_trace_is_disabled_by_default(self):
        with patch.dict(
            "driver.encoder.os.environ",
            {"R2B4_ENCODER_EDGE_TRACE": "0"},
        ):
            enc = DFRobotQuadratureEncoder(23, 24)
        enc._running = True
        enc.level_b = 1
        enc._on_a_edge(0, 23, 1, 100)
        self.assertEqual(enc.pulse_count, 1)
        self.assertEqual(enc.recent_a_rising_events(), [])

    def test_b_edge_callback_latches_phase_without_incrementing_x1_count(self):
        enc = DFRobotQuadratureEncoder(23, 24)
        enc._running = True
        enc._on_b_edge(0, 24, 1, 10)
        self.assertEqual(enc.b_edge_count, 1)
        self.assertEqual(enc.b_rising_count, 1)
        self.assertEqual(enc.level_b, 1)
        self.assertEqual(enc.pulse_count, 0)

    def test_a_edge_uses_latched_b_phase_not_delayed_gpio_read(self):
        fake = _FakeLgpio()
        with patch("driver.encoder.lgpio", fake):
            enc = DFRobotQuadratureEncoder(23, 24, name="L")
            enc.start()
            callback_a = fake.callbacks[23][1]
            callback_b = fake.callbacks[24][1]
            callback_b(0, 24, 1, 100)
            fake.levels[24] = 0

            with patch("driver.encoder.time.perf_counter", return_value=10.0):
                callback_a(0, 23, 1, 200)

            self.assertEqual(enc.pulse_count, 1)
            self.assertEqual(enc.forward_count, 1)
            self.assertEqual(fake.read_count.get(24, 0), 1)
            enc.stop()

    def test_driver_snapshot_is_lock_consistent(self):
        enc = DFRobotQuadratureEncoder(23, 24)
        enc._running = True
        with enc._lock:
            enc._pulse_count = 5
            enc.edge_count = 5
            enc.level_a = 1
            enc.level_b = 0
            enc.last_direction = -1

        snap = enc.snapshot()

        self.assertEqual(snap.pulse_count, 5)
        self.assertEqual(snap.edge_count, 5)
        self.assertEqual(snap.level_a, 1)
        self.assertEqual(snap.level_b, 0)
        self.assertEqual(snap.last_direction, -1)

    def test_estimator_uses_signed_counts_without_pwm_direction_hint(self):
        left = _Counter()
        right = _Counter()
        estimator = EncoderEstimator(left, right)
        estimator.step_distance = 0.001
        estimator._refresh_step_distances()
        estimator._last_t = time.perf_counter() - 0.1
        left.pulse_count = 10
        right.pulse_count = -5

        estimator.update(pwm_l=0.0, pwm_r=0.0)

        self.assertAlmostEqual(estimator.left.distance, 0.01, places=6)
        self.assertAlmostEqual(estimator.right.distance, -0.005, places=6)
        self.assertEqual(estimator.left.direction_source, "QUADRATURE_AB")
        self.assertEqual(estimator.right.direction_source, "QUADRATURE_AB")
        self.assertEqual(estimator.left.unresolved_pulses, 0)

    def test_estimator_timestamp_advances_only_for_a_new_measurement(self):
        left = _Counter()
        right = _Counter()
        estimator = EncoderEstimator(left, right)
        estimator._last_t = 100.0
        estimator._measurement_timestamp = 100.0

        with patch("middleware.enc_estim.time.perf_counter", return_value=100.001):
            self.assertFalse(estimator.update())
        self.assertEqual(estimator.measurement_timestamp, 100.0)

        left.pulse_count = 2
        right.pulse_count = 3
        with patch("middleware.enc_estim.time.perf_counter", return_value=100.003):
            self.assertTrue(estimator.update())
        self.assertEqual(estimator.measurement_timestamp, 100.003)

    def test_estimator_reads_driver_snapshots_before_timestamping_measurement(self):
        order = []

        class SnapshotCounter:
            signed_counts = True
            counts_per_revolution = 663

            def __init__(self, name, pulse_count):
                self.name = name
                self._pulse_count = pulse_count

            def snapshot(self):
                order.append(f"{self.name}_snapshot")
                return SimpleNamespace(pulse_count=self._pulse_count)

        left = SnapshotCounter("left", 0)
        right = SnapshotCounter("right", 0)
        estimator = EncoderEstimator(left, right)
        estimator.step_distance = 0.001
        estimator._refresh_step_distances()
        estimator._last_t = 100.0
        estimator._measurement_timestamp = 100.0
        left._pulse_count = 4
        right._pulse_count = -4
        order.clear()

        def fake_perf_counter():
            order.append("timestamp")
            return 100.01

        with patch("middleware.enc_estim.time.perf_counter", side_effect=fake_perf_counter):
            self.assertTrue(estimator.update())

        self.assertEqual(order, ["left_snapshot", "right_snapshot", "timestamp"])
        self.assertEqual(estimator.left.pulses, 4)
        self.assertEqual(estimator.right.pulses, -4)
        self.assertEqual(estimator.measurement_timestamp, 100.01)

    def test_startup_readiness_accepts_stationary_quadrature_stream(self):
        left = DFRobotQuadratureEncoder(23, 24)
        right = DFRobotQuadratureEncoder(25, 16)
        left._running = True
        right._running = True
        left.level_a = left.level_b = 0
        right.level_a = right.level_b = 1

        ready, info = _encoder_stream_ready(_Controller(left, right))

        self.assertTrue(ready, info)
        self.assertEqual(info["pulse_count_l"], 0)
        self.assertEqual(info["pulse_count_r"], 0)


if __name__ == "__main__":
    unittest.main()
