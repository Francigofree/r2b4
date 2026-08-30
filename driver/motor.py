import lgpio
import json
import logging
import threading

from config_manager import config as global_config

# Logolás beállítása
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AlbaMotor:
    """
    RPi 5 + DRV8871 motor driver. 
    Konfiguráció a config_manager-ből (conf/hardver.json), hardveres PWM-et használ.
    """
    def __init__(self, side_key: str, config_path: str | None = None):
        """
        side_key: "bal_oldal" vagy "jobb_oldal" (Magyar kulcsok a JSON-ből)
        """
        # Konfiguráció: elsődlegesen config_manager, szükség esetén fallback path
        self._load_config(side_key, config_path)
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
                f"Motor inicializálva ({side_key}) - IN1: {self.in1}, IN2: {self.in2}, invert={self.invert}"
            )
        except Exception as e:
            logger.error(f"Hiba az lgpio inicializálásakor ({side_key}): {e}")
            raise

    def _load_config(self, side_key: str, path: str | None):
        try:
            config = global_config.get("hardver", default={})
            motor_cfg = config.get("motorok", {}).get(side_key)
            if not motor_cfg and path:
                with open(path, "r", encoding="utf-8") as f:
                    config = json.load(f)
                motor_cfg = config.get("motorok", {}).get(side_key)
            if not motor_cfg:
                raise KeyError(f"Motor konfiguráció hiányzik: {side_key}")
            self.in1 = motor_cfg["gpio_in1"]
            self.in2 = motor_cfg["gpio_in2"]
            motors_cfg = config.get("motorok", {}) or {}
            decay_mode = str(
                motor_cfg.get("pwm_decay_mode", motors_cfg.get("pwm_decay_mode", "coast"))
                or "coast"
            ).strip().lower()
            self.pwm_decay_mode = decay_mode if decay_mode in ("coast", "brake") else "coast"
            # Optional per-side polarity flip. This is required on some builds
            # where H-bridge wiring polarity differs between tracks.
            self.invert = bool(motor_cfg.get("invert", False))
        except Exception as e:
            logger.error(f"Konfigurációs hiba (motor): {e}")
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
