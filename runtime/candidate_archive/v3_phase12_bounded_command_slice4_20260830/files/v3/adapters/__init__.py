"""V3 edge adapters; adapters never own control-layer authority."""

from .bounded_command import BoundedTeleopCommandGateway, BoundedTeleopProfile
from .capture_edges import ReplayerV1CaptureConfig, ReplayerV1InputAdapter
from .live_idle import (
    GpioBackend,
    GpioZeroMotorWriter,
    GpioZeroWriterConfig,
    LiveIdleDeviceReader,
    LiveIdleWriteRejected,
    LockedStopCommandGateway,
    MotorChannelPhysicalConfig,
    PwmDecayMode,
)

__all__ = [
    "BoundedTeleopCommandGateway",
    "BoundedTeleopProfile",
    "GpioBackend",
    "GpioZeroMotorWriter",
    "GpioZeroWriterConfig",
    "LiveIdleDeviceReader",
    "LiveIdleWriteRejected",
    "LockedStopCommandGateway",
    "MotorChannelPhysicalConfig",
    "PwmDecayMode",
    "ReplayerV1CaptureConfig",
    "ReplayerV1InputAdapter",
]
