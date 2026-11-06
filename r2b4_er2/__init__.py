"""R2B4 integration for Gemini Robotics ER 2 Preview and Streaming."""
ER2_P0_BUILD = "R2B4_ER2_P0_20260925"
ER2_P0R2_BUILD = "R2B4_ER2_P0R2_20260925"

from .config import Er2Config, PREVIEW_MODEL, STREAMING_MODEL
from .evidence import Er2Evidence
from .preview import Er2PreviewClient
from .speech import Er2SpeechReporter
from .streaming import Er2StreamingClient
from .tool_bridge import Er2RobotTools

__all__ = [
    "ER2_P0_BUILD",
    "ER2_P0R2_BUILD",
    "Er2Config",
    "Er2Evidence",
    "Er2PreviewClient",
    "Er2RobotTools",
    "Er2SpeechReporter",
    "Er2StreamingClient",
    "PREVIEW_MODEL",
    "STREAMING_MODEL",
]
