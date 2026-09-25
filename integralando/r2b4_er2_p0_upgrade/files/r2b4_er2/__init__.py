"""R2B4 integration for Gemini Robotics ER 2 Preview and Streaming."""
ER2_P0_BUILD = "R2B4_ER2_P0_20260925"

from .config import Er2Config, PREVIEW_MODEL, STREAMING_MODEL
from .preview import Er2PreviewClient
from .streaming import Er2StreamingClient
from .tool_bridge import Er2RobotTools

__all__ = [
    "ER2_P0_BUILD",
    "Er2Config",
    "Er2PreviewClient",
    "Er2RobotTools",
    "Er2StreamingClient",
    "PREVIEW_MODEL",
    "STREAMING_MODEL",
]
