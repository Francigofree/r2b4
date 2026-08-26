import unittest
from unittest.mock import patch

from driver.motor import AlbaMotor


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


def _load_config(mode):
    def fake(self, _side_key, _path=None):
        self.in1 = 12
        self.in2 = 13
        self.invert = False
        self.pwm_decay_mode = mode

    return fake


class Drv8871MotorHalTests(unittest.TestCase):
    def test_brake_decay_forward_pwm_uses_drive_brake_pattern(self):
        fake = _FakeLgpio()
        with patch("driver.motor.lgpio", fake), patch.object(AlbaMotor, "_load_config", _load_config("brake")):
            motor = AlbaMotor("bal_oldal")
            fake.calls.clear()

            motor.set_pwm(0.40)

        self.assertEqual(fake.calls, [("pwm", 1, 12, 8000, 100.0), ("pwm", 1, 13, 8000, 60.0)])

    def test_brake_decay_reverse_pwm_uses_drive_brake_pattern(self):
        fake = _FakeLgpio()
        with patch("driver.motor.lgpio", fake), patch.object(AlbaMotor, "_load_config", _load_config("brake")):
            motor = AlbaMotor("bal_oldal")
            fake.calls.clear()

            motor.set_pwm(-0.25)

        self.assertEqual(fake.calls, [("pwm", 1, 13, 8000, 100.0), ("pwm", 1, 12, 8000, 75.0)])

    def test_coast_decay_keeps_legacy_pwm_pattern(self):
        fake = _FakeLgpio()
        with patch("driver.motor.lgpio", fake), patch.object(AlbaMotor, "_load_config", _load_config("coast")):
            motor = AlbaMotor("bal_oldal")
            fake.calls.clear()

            motor.set_pwm(0.40)

        self.assertEqual(fake.calls, [("pwm", 1, 13, 8000, 0), ("pwm", 1, 12, 8000, 40.0)])


if __name__ == "__main__":
    unittest.main()
