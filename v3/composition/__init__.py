"""Explicit V3 composition roots with no import-time runtime activation."""

from .bounded_live_control import (
    BoundedLiveControlComposition,
    BoundedLiveControlConfig,
)
from .bounded_physical_control import (
    BoundedPhysicalControlComposition,
    BoundedPhysicalControlConfig,
)
from .full_fake import FullFakeComposition, FullFakeConfig, LayerFault, OfflineMotorSink
from .input_shadow import InputShadowComposition, ZeroOnlyShadowSink
from .live_idle import LiveIdleComposition, LiveIdleConfig
from .mission_navigation import (
    MissionNavigationComposition,
    MissionNavigationInputs,
    MissionNavigationTrace,
)
from .motor_output import NativeMotorOutputComposition
from .native_control import NativeControlComposition, NativeControlCompositionConfig
from .read_only_shadow import (
    ReadOnlyShadowConfig,
    ReadOnlyShadowSidecar,
    ShadowSidecarError,
    ShadowTickResult,
)
from .stop_only import LifecycleTransitionError, StopOnlyComposition

__all__ = [
    "BoundedLiveControlComposition",
    "BoundedLiveControlConfig",
    "BoundedPhysicalControlComposition",
    "BoundedPhysicalControlConfig",
    "FullFakeComposition",
    "FullFakeConfig",
    "InputShadowComposition",
    "LayerFault",
    "LifecycleTransitionError",
    "LiveIdleComposition",
    "LiveIdleConfig",
    "MissionNavigationComposition",
    "MissionNavigationInputs",
    "MissionNavigationTrace",
    "NativeMotorOutputComposition",
    "NativeControlComposition",
    "NativeControlCompositionConfig",
    "ReadOnlyShadowConfig",
    "ReadOnlyShadowSidecar",
    "ShadowSidecarError",
    "ShadowTickResult",
    "StopOnlyComposition",
    "OfflineMotorSink",
    "ZeroOnlyShadowSink",
]
