#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Loop watchdog: méri a fő ciklus periódusidőt (dt).
- Lassulás < warning_threshold_sec: csak log warning
- Lassulás >= safety_stop_threshold_sec: safety stop (emergency_stop)
Megengedő: warning 0.2-ig, 0.2 felett safety stop.
"""

import time
from typing import Optional, Callable


class LoopWatchdog:
    """
    Egy ciklusonként tick() hívandó. Méri a két tick közötti eltelt időt.
    """

    def __init__(
        self,
        warning_threshold_sec: float = 0.2,
        safety_stop_threshold_sec: float = 0.2,
        maintenance_warning_threshold_sec: float = 1.5,
        maintenance_safety_stop_threshold_sec: float = 5.0,
        on_safety_stop: Optional[Callable[[str], None]] = None,
    ):
        """
        Args:
            warning_threshold_sec: E felett warning log (alatta nincs log).
            safety_stop_threshold_sec: E felett safety stop (on_safety_stop hívás).
            on_safety_stop: callback(reason_string) – pl. ctrl._emergency_stop.
        """
        self.warning_threshold_sec = warning_threshold_sec
        self.safety_stop_threshold_sec = safety_stop_threshold_sec
        self.maintenance_warning_threshold_sec = max(
            float(maintenance_warning_threshold_sec), float(warning_threshold_sec)
        )
        self.maintenance_safety_stop_threshold_sec = max(
            float(maintenance_safety_stop_threshold_sec), self.maintenance_warning_threshold_sec
        )
        self.on_safety_stop = on_safety_stop

        self._last_tick_time: Optional[float] = None
        self._period_sec: float = 0.0
        self._freq_hz: float = 0.0
        self._warn_count: int = 0
        self._stop_triggered: bool = False
        self._policy_mode: str = "normal"
        self._maintenance_reason: str = ""

    def enter_maintenance(self, reason: str = "") -> None:
        self._policy_mode = "maintenance"
        self._maintenance_reason = str(reason or "")
        self.reset()

    def exit_maintenance(self) -> None:
        self._policy_mode = "normal"
        self._maintenance_reason = ""
        self.reset()

    def _active_thresholds(self) -> tuple[float, float]:
        if self._policy_mode == "maintenance":
            return self.maintenance_warning_threshold_sec, self.maintenance_safety_stop_threshold_sec
        return self.warning_threshold_sec, self.safety_stop_threshold_sec

    def tick(self, logger=None) -> bool:
        """
        Ciklus végén hívandó. Vissza: True ha minden oké, False ha safety stop történt.
        """
        now = time.perf_counter()
        if self._last_tick_time is not None:
            warning_threshold_sec, safety_stop_threshold_sec = self._active_thresholds()
            self._period_sec = now - self._last_tick_time
            if self._period_sec > 1e-6:
                self._freq_hz = 1.0 / self._period_sec
            else:
                self._freq_hz = 0.0

            # Safety stop: >= threshold
            if self._period_sec >= safety_stop_threshold_sec:
                if not self._stop_triggered and self.on_safety_stop:
                    self._stop_triggered = True
                    reason = (
                        f"WATCHDOG_LOOP_SLOW period={self._period_sec:.3f}s >= {safety_stop_threshold_sec}s"
                        f" mode={self._policy_mode}"
                    )
                    if logger:
                        logger.error(f"[WATCHDOG] {reason}")
                    try:
                        self.on_safety_stop(reason)
                    except Exception:
                        pass
                self._last_tick_time = now
                return False

            # Warning: > warning_threshold (de < safety)
            if self._period_sec >= warning_threshold_sec and logger:
                self._warn_count += 1
                if self._warn_count <= 5 or self._warn_count % 50 == 0:
                    logger.warn(
                        f"[WATCHDOG] Lassulás: period={self._period_sec:.3f}s "
                        f"(freq={self._freq_hz:.1f} Hz)"
                    )
        else:
            self._period_sec = 0.0
            self._freq_hz = 0.0

        self._last_tick_time = now
        return True

    def status(self) -> dict:
        """Státusz a GUI/status számára: period_sec, freq_hz, warn_count, stop_triggered."""
        return {
            "period_sec": round(self._period_sec, 4),
            "freq_hz": round(self._freq_hz, 2),
            "warn_count": self._warn_count,
            "stop_triggered": self._stop_triggered,
            "warning_threshold_sec": self.warning_threshold_sec,
            "safety_stop_threshold_sec": self.safety_stop_threshold_sec,
            "maintenance_warning_threshold_sec": self.maintenance_warning_threshold_sec,
            "maintenance_safety_stop_threshold_sec": self.maintenance_safety_stop_threshold_sec,
            "policy_mode": self._policy_mode,
            "maintenance_reason": self._maintenance_reason,
        }

    def reset(self) -> None:
        self._last_tick_time = None
        self._period_sec = 0.0
        self._freq_hz = 0.0
        self._warn_count = 0
        self._stop_triggered = False
