#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared physical clearance contracts for the single motion pipeline.

The local planner uses these pure calculations to avoid proposing a forward
command that the final SafetyGate would immediately brake or reject.  The
SafetyGate remains the authoritative output filter; this module only keeps its
speed-dependent envelope consistent across the two layers.
"""

from __future__ import annotations


FRONT_SAFE_FLOOR_M = 0.25
FRONT_STOP_GAIN_S = 0.55
FRONT_STOP_MAX_M = 0.60
FRONT_START_EXTRA_M = 0.08
FRONT_START_GAIN_S = 0.25
FRONT_START_MAX_M = 1.00


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def dynamic_front_clearance_thresholds(v_cmd: float) -> tuple[float, float]:
    """Return ``(brake_start_m, hard_stop_m)`` for a forward command.

    The calculation is deliberately stateless and monotonic in forward speed.
    Negative commands use the zero-forward-speed floor; rear clearance remains
    a separate SafetyGate contract.
    """

    speed = max(0.0, float(v_cmd))
    stop_m = FRONT_SAFE_FLOOR_M + (FRONT_STOP_GAIN_S * speed)
    stop_m = _clamp(stop_m, FRONT_SAFE_FLOOR_M, FRONT_STOP_MAX_M)

    start_m = stop_m + FRONT_START_EXTRA_M + (FRONT_START_GAIN_S * speed)
    start_m = _clamp(start_m, stop_m + 0.05, FRONT_START_MAX_M)
    return float(start_m), float(stop_m)
