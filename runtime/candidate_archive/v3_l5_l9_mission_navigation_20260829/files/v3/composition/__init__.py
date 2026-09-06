"""Explicit V3 composition roots with no import-time runtime activation."""

from .input_shadow import InputShadowComposition, ZeroOnlyShadowSink
from .mission_navigation import (
    MissionNavigationComposition,
    MissionNavigationInputs,
    MissionNavigationTrace,
)
from .stop_only import LifecycleTransitionError, StopOnlyComposition

__all__ = [
    "InputShadowComposition",
    "LifecycleTransitionError",
    "MissionNavigationComposition",
    "MissionNavigationInputs",
    "MissionNavigationTrace",
    "StopOnlyComposition",
    "ZeroOnlyShadowSink",
]
