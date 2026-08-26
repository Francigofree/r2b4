#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from dataclasses import dataclass
from typing import Optional

from driver.encoder import DFRobotQuadratureEncoder
from middleware.enc_estim import EncoderEstimator


@dataclass(frozen=True)
class EncoderSnapshot:
    """Immutable encoder snapshot stamped at the actual counter measurement."""
    timestamp: float
    left_velocity: float
    right_velocity: float
    left_distance: float
    right_distance: float
    left_pulses: int
    right_pulses: int
    theta_enc: float  # [rad], pulse-based yaw from estimator
    health: str  # "OK", "ERROR"
    # Signed quadrature debug/telemetry mezők.
    sample_dt: float = 0.0
    left_velocity_raw: float = 0.0
    right_velocity_raw: float = 0.0
    left_velocity_unsigned: float = 0.0
    right_velocity_unsigned: float = 0.0
    left_distance_delta: float = 0.0
    right_distance_delta: float = 0.0
    left_unsigned_distance_delta: float = 0.0
    right_unsigned_distance_delta: float = 0.0
    left_pulse_delta: int = 0
    right_pulse_delta: int = 0
    left_direction: float = 0.0
    right_direction: float = 0.0
    left_direction_source: str = "INIT"
    right_direction_source: str = "INIT"
    left_direction_confident: bool = False
    right_direction_confident: bool = False
    left_unresolved_pulses: int = 0
    right_unresolved_pulses: int = 0
    pipeline_model: str = "KIT0085_QUADRATURE"
    published_at: float = 0.0
    left_step_distance_m: float = 0.0
    right_step_distance_m: float = 0.0


class EncoderService:
    """
    Encoder service layer running in its own thread.
    Publishes immutable, timestamped snapshots.
    A PWM csak diagnosztikai kontextus; az irány az A/B encoderből érkezik.
    """
    def __init__(self, enc_l: DFRobotQuadratureEncoder, enc_r: DFRobotQuadratureEncoder):
        self.enc_l = enc_l
        self.enc_r = enc_r
        self.estimator = EncoderEstimator(enc_l, enc_r)

        self._lock = threading.Lock()
        self._current_snapshot: Optional[EncoderSnapshot] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._update_rate_hz = 400.0

        # Utolsó motor PWM (irány a becslőnek) – thread-safe
        self._pwm_lock = threading.Lock()
        self._last_pwm_l: float = 0.0
        self._last_pwm_r: float = 0.0

    def set_last_pwm(self, pwm_l: float, pwm_r: float) -> None:
        """Utolsó alkalmazott motor PWM (control loop hívja). Irány a sebességhez."""
        with self._pwm_lock:
            self._last_pwm_l = pwm_l
            self._last_pwm_r = pwm_r

    def start(self):
        """Start the encoder service thread."""
        if self._running:
            return True
        
        if not (self.enc_l and self.enc_r):
            return False
        
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        """Stop the encoder service thread."""
        self._running = False
        if self._thread and self._thread.is_alive():
            # Thread will exit when _running becomes False
            pass

    def _worker(self):
        """Worker thread: continuously updates estimator and creates snapshot."""
        dt_target = 1.0 / self._update_rate_hz
        next_time = time.perf_counter()

        while self._running:
            try:
                with self._pwm_lock:
                    pwm_l, pwm_r = self._last_pwm_l, self._last_pwm_r
                measurement_updated = self.estimator.update(pwm_l=pwm_l, pwm_r=pwm_r)
                published_at = time.perf_counter()
                if not measurement_updated:
                    next_time += dt_target
                    pause = next_time - time.perf_counter()
                    if pause > 0:
                        time.sleep(pause)
                    else:
                        next_time = time.perf_counter()
                    continue

                left = self.estimator.left
                right = self.estimator.right
                measurement_ts = float(self.estimator.measurement_timestamp)

                snapshot = EncoderSnapshot(
                    timestamp=measurement_ts,
                    left_velocity=left.velocity,
                    right_velocity=right.velocity,
                    left_distance=left.distance,
                    right_distance=right.distance,
                    left_pulses=left.pulses,
                    right_pulses=right.pulses,
                    theta_enc=self.estimator.theta_enc,
                    health="OK",
                    sample_dt=float(getattr(left, "dt", 0.0)),
                    left_velocity_raw=float(getattr(left, "raw_velocity", left.velocity)),
                    right_velocity_raw=float(getattr(right, "raw_velocity", right.velocity)),
                    left_velocity_unsigned=float(getattr(left, "unsigned_velocity", abs(left.velocity))),
                    right_velocity_unsigned=float(getattr(right, "unsigned_velocity", abs(right.velocity))),
                    left_distance_delta=float(getattr(left, "distance_delta", 0.0)),
                    right_distance_delta=float(getattr(right, "distance_delta", 0.0)),
                    left_unsigned_distance_delta=float(getattr(left, "unsigned_distance_delta", 0.0)),
                    right_unsigned_distance_delta=float(getattr(right, "unsigned_distance_delta", 0.0)),
                    left_pulse_delta=int(getattr(left, "dp", 0)),
                    right_pulse_delta=int(getattr(right, "dp", 0)),
                    left_direction=float(getattr(left, "direction", 0.0)),
                    right_direction=float(getattr(right, "direction", 0.0)),
                    left_direction_source=str(getattr(left, "direction_source", "N/A")),
                    right_direction_source=str(getattr(right, "direction_source", "N/A")),
                    left_direction_confident=bool(getattr(left, "direction_confident", False)),
                    right_direction_confident=bool(getattr(right, "direction_confident", False)),
                    left_unresolved_pulses=int(getattr(self.estimator, "_last_unresolved_dp_l", 0)),
                    right_unresolved_pulses=int(getattr(self.estimator, "_last_unresolved_dp_r", 0)),
                    pipeline_model="KIT0085_QUADRATURE",
                    published_at=float(published_at),
                    left_step_distance_m=float(self.estimator.step_distance_left),
                    right_step_distance_m=float(self.estimator.step_distance_right),
                )

                with self._lock:
                    self._current_snapshot = snapshot

                next_time += dt_target
                pause = next_time - time.perf_counter()
                if pause > 0:
                    time.sleep(pause)
                else:
                    next_time = time.perf_counter()

            except Exception:
                now = time.perf_counter()
                snapshot = EncoderSnapshot(
                    timestamp=now,
                    left_velocity=0.0,
                    right_velocity=0.0,
                    left_distance=0.0,
                    right_distance=0.0,
                    left_pulses=0,
                    right_pulses=0,
                    theta_enc=0.0,
                    health="ERROR",
                    published_at=now,
                )
                with self._lock:
                    self._current_snapshot = snapshot
                time.sleep(0.1)

    def get_snapshot(self) -> Optional[EncoderSnapshot]:
        """
        Get the latest immutable snapshot.
        Returns None if no snapshot available yet.
        """
        with self._lock:
            return self._current_snapshot
