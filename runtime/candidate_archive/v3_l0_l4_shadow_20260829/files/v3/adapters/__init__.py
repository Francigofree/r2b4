"""V3 edge adapters; adapters never own control-layer authority."""

from .capture_edges import ReplayerV1CaptureConfig, ReplayerV1InputAdapter

__all__ = ["ReplayerV1CaptureConfig", "ReplayerV1InputAdapter"]
