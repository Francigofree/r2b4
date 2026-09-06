#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compatibility imports for physically separated V2.1 readiness owners.

New composition code must import the owning L2A/L7A modules directly.

The protected L2A trust invariants now live in ``encoder_reliability`` rather
than this façade: ``timing_gap_invalid_s``, ``ENCODER_TIMING_GAP``,
``motion_timing_gap_count``, ``canonical_available`` and
``ekf_usage_reason = "TIMING_GAP"``.  This module intentionally owns no
runtime state or control logic.
"""

from controller.encoder_reliability import (
    EncoderFlowProfile,
    EncoderReliabilityConfig,
    EncoderReliabilityLayer,
)
from controller.heading_turn_controller import (
    HeadingTurnConfig,
    HeadingTurnController,
)
from controller.motion_semantics_engine import MotionSemanticsEngine

__all__ = [
    "EncoderFlowProfile",
    "EncoderReliabilityConfig",
    "EncoderReliabilityLayer",
    "HeadingTurnConfig",
    "HeadingTurnController",
    "MotionSemanticsEngine",
]
