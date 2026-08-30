import lgpio
import logging
import threading
from dataclasses import dataclass


# Logolás beállítása
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class MotorChannelConfig:
    side_key: str
    gpio_in1: int
    gpio_in2: int
    invert: bool = False
    pwm_decay_mode: str = "coast"

    def __post_init__(self) -> None:
        if not isinstance(self.side_key, str) or not self.side_key.strip():
            raise ValueError("motor side_key must be a non-empty string")
        for name, pin in (("gpio_in1", self.gpio_in1), ("gpio_in2", self.gpio_in2)):
            if isinstance(pin, bool) or not isinstance(pin, int) or pin < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.gpio_in1 == self.gpio_in2:
            raise ValueError("motor GPIO pins must be distinct")
        if type(self.invert) is not bool:
            raise ValueError("motor invert must be bool")
        if self.pwm_decay_mode not in ("coast", "brake"):
            raise ValueError("motor pwm_decay_mode must be 'coast' or 'brake'")


class AlbaMotor:
    """
    RPi 5 + DRV8871 motor driver explicit, immutable csatornakonfigurációval.
    """
    def __init__(self, config: MotorChannelConfig):
        if not isinstance(config, MotorChannelConfig):
            raise TypeError("config must be MotorChannelConfig")
        self.in1 = config.gpio_in1
        self.in2 = config.gpio_in2
        self.invert = config.invert
        self.pwm_decay_mode = config.pwm_decay_mode
        self.freq = 8000
        self._io_lock = threading.Lock()
        self._closed = False
        self._last_direction = 0  # -1: back, 0: stop, +1: fwd
        self._last_duty = 0.0
        self._duty_epsilon = 0.05

        try:
            self.handle = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self.handle, self.in1)
            lgpio.gpio_claim_output(self.handle, self.in2)
            self._apply_output_locked(0, 0.0)
            logger.info(
                f"Motor inicializálva ({config.side_key}) - IN1: {self.in1}, IN2: {self.in2}, invert={self.invert}"
            )
        except Exception as e:
            logger.error(f"Hiba az lgpio inicializálásakor ({config.side_key}): {e}")
            raise

    def set_pwm(self, pwm: float):
        try:
            pwm = float(pwm)
        except (TypeError, ValueError):
            pwm = 0.0
        pwm = max(-1.0, min(1.0, pwm))
        if bool(getattr(self, "invert", False)):
            pwm = -pwm
        if pwm > 0:
            self._set_output(1, pwm * 100.0)
        elif pwm < 0:
            self._set_output(-1, -pwm * 100.0)
        else:
            self.stop()

    def stop(self):
        self._set_output(0, 0.0)

    def close(self):
        with self._io_lock:
            if self._closed:
                return
            if hasattr(self, "handle"):
                self._apply_output_locked(0, 0.0)
                lgpio.gpiochip_close(self.handle)
            self._closed = True

    def _set_output(self, direction: int, duty_cycle: float):
        duty = max(0.0, min(100.0, float(duty_cycle)))
        direction = 0 if duty <= 1e-6 else (1 if direction > 0 else -1)
        with self._io_lock:
            if self._closed or not hasattr(self, "handle"):
                return
            if (
                direction == self._last_direction
                and abs(duty - self._last_duty) <= self._duty_epsilon
            ):
                return
            self._apply_output_locked(direction, duty)
            self._last_direction = direction
            self._last_duty = duty

    def _apply_output_locked(self, direction: int, duty_cycle: float):
        decay_mode = str(getattr(self, "pwm_decay_mode", "coast") or "coast").lower()
        if direction > 0:
            if decay_mode == "brake":
                lgpio.tx_pwm(self.handle, self.in1, self.freq, 100.0)
                lgpio.tx_pwm(self.handle, self.in2, self.freq, 100.0 - duty_cycle)
            else:
                lgpio.tx_pwm(self.handle, self.in2, self.freq, 0)
                lgpio.tx_pwm(self.handle, self.in1, self.freq, duty_cycle)
        elif direction < 0:
            if decay_mode == "brake":
                lgpio.tx_pwm(self.handle, self.in2, self.freq, 100.0)
                lgpio.tx_pwm(self.handle, self.in1, self.freq, 100.0 - duty_cycle)
            else:
                lgpio.tx_pwm(self.handle, self.in1, self.freq, 0)
                lgpio.tx_pwm(self.handle, self.in2, self.freq, duty_cycle)
        else:
            lgpio.tx_pwm(self.handle, self.in1, self.freq, 0)
            lgpio.tx_pwm(self.handle, self.in2, self.freq, 0)
