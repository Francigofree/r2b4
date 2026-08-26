"""R2B4 Replayer V2 public API with V1 capture compatibility."""

from replayer.capture import CaptureRecorder, verify_capture
from replayer.replay import replay_capture, verify_replay_result

__all__ = [
    "CaptureRecorder",
    "replay_capture",
    "verify_capture",
    "verify_replay_result",
]
