#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Startup State Machine – explicit indítási állapotgép.
Minden állapot determinisztikus, timeout-tal védett, logolt, GUI-nak publikált.
"""

from enum import Enum
from dataclasses import dataclass, field
import time


class StartupState(Enum):
    """Indítási fázisok sorrendje."""
    BOOT = 0
    HARDWARE_DISCOVERY = 1
    PERIPHERAL_INIT = 2
    SENSOR_STABILIZATION = 3
    CALIBRATION = 4
    BASELINE_CAPTURE = 5
    SAFETY_VALIDATION = 6
    CONTROL_ARMED = 7
    READY = 8
    FAILSAFE = 9
    DEGRADED = 10


@dataclass
class StartupContext:
    """Startup folyamat kontextusa – megosztott állapot a fázisok között."""
    # Állapot
    current_state: StartupState = StartupState.BOOT
    state_entered_at: float = 0.0
    last_error: str = ""
    degraded_reasons: list = field(default_factory=list)

    # Eredmények (fázisok töltik)
    hardware_discovery: dict = field(default_factory=dict)
    sensor_health: dict = field(default_factory=dict)
    calibration_status: dict = field(default_factory=dict)
    baseline: dict = field(default_factory=dict)

    def set_state(self, state: StartupState):
        self.current_state = state
        self.state_entered_at = time.monotonic()

    def add_degraded_reason(self, reason: str):
        if reason and reason not in self.degraded_reasons:
            self.degraded_reasons.append(reason)


class StartupStateMachine:
    """
    Indítási állapotgép – lépésenként halad, timeout figyeléssel.
    A supervisor hívja a run_step-t, amely visszaadja a következő állapotot vagy hibát.
    """

    def __init__(self, config: dict, logger=None):
        self.config = config
        self.logger = logger
        self.ctx = StartupContext()
        self._timeouts = config.get("timeouts", {})
        self._default_timeout_sec = float(self._timeouts.get("default", 30.0))

    def get_timeout_sec(self, state: StartupState) -> float:
        """Állapot-specifikus timeout (másodperc)."""
        key = state.name.lower()
        return float(self._timeouts.get(key, self._default_timeout_sec))

    def is_timed_out(self) -> bool:
        """Aktuális állapot timeout-jának ellenőrzése."""
        elapsed = time.monotonic() - self.ctx.state_entered_at
        return elapsed >= self.get_timeout_sec(self.ctx.current_state)

    def get_state_for_gui(self) -> dict:
        """GUI-nak publikálható startup státusz."""
        return {
            "startup_state": self.ctx.current_state.name,
            "state_entered_at": self.ctx.state_entered_at,
            "last_error": self.ctx.last_error,
            "degraded_reasons": list(self.ctx.degraded_reasons),
            "sensor_health": dict(self.ctx.sensor_health),
            "calibration_status": dict(self.ctx.calibration_status),
        }

    def log(self, msg: str, prefix: str = "[STARTUP]"):
        if self.logger:
            self.logger.info(f"{prefix} {msg}")
