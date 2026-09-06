"""Explicit V3 composition roots with no import-time runtime activation."""

from .full_fake import FullFakeComposition, FullFakeConfig, LayerFault, OfflineMotorSink
from .input_shadow import InputShadowComposition, ZeroOnlyShadowSink
from .mission_navigation import (
    MissionNavigationComposition,
    MissionNavigationInputs,
    MissionNavigationTrace,
)
from .read_only_shadow import (
    ReadOnlyShadowConfig,
    ReadOnlyShadowSidecar,
    ShadowSidecarError,
    ShadowTickResult,
)
from .stop_only import LifecycleTransitionError, StopOnlyComposition

__all__ = [
    "FullFakeComposition",
    "FullFakeConfig",
    "InputShadowComposition",
    "LayerFault",
    "LifecycleTransitionError",
    "MissionNavigationComposition",
    "MissionNavigationInputs",
    "MissionNavigationTrace",
    "ReadOnlyShadowConfig",
    "ReadOnlyShadowSidecar",
    "ShadowSidecarError",
    "ShadowTickResult",
    "StopOnlyComposition",
    "OfflineMotorSink",
    "ZeroOnlyShadowSink",
]
