"""V3 edge adapters; adapters never own control-layer authority."""

from .capture_edges import ReplayerV1CaptureConfig, ReplayerV1InputAdapter
from .live_idle import (
    GpioBackend,
    GpioZeroMotorWriter,
    GpioZeroWriterConfig,
    LiveIdleDeviceReader,
    LiveIdleWriteRejected,
    LockedStopCommandGateway,
    MotorPinPair,
)

__all__ = [
    "GpioBackend",
    "GpioZeroMotorWriter",
    "GpioZeroWriterConfig",
    "LiveIdleDeviceReader",
    "LiveIdleWriteRejected",
    "LockedStopCommandGateway",
    "MotorPinPair",
    "ReplayerV1CaptureConfig",
    "ReplayerV1InputAdapter",
]
