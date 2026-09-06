import unittest
from dataclasses import FrozenInstanceError
from unittest.mock import patch

from driver.motor import AlbaMotor, MotorChannelConfig


class _FakeLgpio:
    def __init__(self):
        self.calls = []

    def gpiochip_open(self, chip):
        self.calls.append(("open", chip))
        return 1

    def gpio_claim_output(self, handle, pin):
        self.calls.append(("claim", handle, pin))

    def gpiochip_close(self, handle):
        self.calls.append(("close", handle))

    def tx_pwm(self, handle, pin, freq, duty):
        self.calls.append(("pwm", handle, pin, freq, duty))


def _config(mode="coast"):
    return MotorChannelConfig(
        side_key="bal_oldal",
        gpio_in1=12,
        gpio_in2=13,
        pwm_decay_mode=mode,
    )


class Drv8871MotorHalTests(unittest.TestCase):
    def test_brake_decay_forward_pwm_uses_drive_brake_pattern(self):
        fake = _FakeLgpio()
        with patch("driver.motor.lgpio", fake):
            motor = AlbaMotor(_config("brake"))
            fake.calls.clear()

            motor.set_pwm(0.40)

        self.assertEqual(fake.calls, [("pwm", 1, 12, 8000, 100.0), ("pwm", 1, 13, 8000, 60.0)])

    def test_brake_decay_reverse_pwm_uses_drive_brake_pattern(self):
        fake = _FakeLgpio()
        with patch("driver.motor.lgpio", fake):
            motor = AlbaMotor(_config("brake"))
            fake.calls.clear()

            motor.set_pwm(-0.25)

        self.assertEqual(fake.calls, [("pwm", 1, 13, 8000, 100.0), ("pwm", 1, 12, 8000, 75.0)])

    def test_coast_decay_keeps_legacy_pwm_pattern(self):
        fake = _FakeLgpio()
        with patch("driver.motor.lgpio", fake):
            motor = AlbaMotor(_config("coast"))
            fake.calls.clear()

            motor.set_pwm(0.40)

        self.assertEqual(fake.calls, [("pwm", 1, 13, 8000, 0), ("pwm", 1, 12, 8000, 40.0)])

    def test_motor_channel_config_is_immutable_and_validated(self):
        config = _config()

        with self.assertRaises(FrozenInstanceError):
            config.gpio_in1 = 18
        with self.assertRaises(ValueError):
            MotorChannelConfig("bal_oldal", 12, 12)
        with self.assertRaises(ValueError):
            MotorChannelConfig("bal_oldal", 12, 13, pwm_decay_mode="invalid")

    def test_motor_requires_explicit_typed_config(self):
        with self.assertRaises(TypeError):
            AlbaMotor("bal_oldal")


if __name__ == "__main__":
    unittest.main()
