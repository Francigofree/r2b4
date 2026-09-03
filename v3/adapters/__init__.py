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
from .resident_command import (
    AtomicResidentCommandGateway,
    RESIDENT_COMMAND_SCHEMA,
    ResidentCommandMailboxConfig,
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
    "AtomicResidentCommandGateway",
    "RESIDENT_COMMAND_SCHEMA",
    "ResidentCommandMailboxConfig",
    "ReplayerV1CaptureConfig",
    "ReplayerV1InputAdapter",
]
