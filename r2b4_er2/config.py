"""Configuration contract for the Gemini Robotics ER 2 integration.

Provider-specific policy lives here, outside the V3 L0-L12 authority chain.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

PREVIEW_MODEL = "gemini-robotics-er-2-preview"
STREAMING_MODEL = "gemini-robotics-er-2-streaming-preview"


class Er2ConfigError(ValueError):
    pass


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        value = float(raw)
    except ValueError as exc:
        raise Er2ConfigError(f"{name} must be numeric") from exc
    if not math.isfinite(value):
        raise Er2ConfigError(f"{name} must be finite")
    return value


@dataclass(frozen=True, slots=True)
class Er2Config:
    preview_model: str = PREVIEW_MODEL
    streaming_model: str = STREAMING_MODEL
    heartbeat_s: float = 1.0
    max_v_mps: float = 0.20
    max_omega_rad_s: float = 0.60
    max_segment_s: float = 1.50
    session_watchdog_s: float = 10.0
    media_timeout_s: float = 1.25

    def __post_init__(self) -> None:
        if not self.preview_model.strip() or not self.streaming_model.strip():
            raise Er2ConfigError("ER2 model names must be non-empty")
        if not 0.90 <= self.heartbeat_s <= 10.0:
            raise Er2ConfigError("heartbeat_s must stay within [0.90, 10.0]")
        if not 0.01 <= self.max_v_mps <= 0.50:
            raise Er2ConfigError("max_v_mps must stay within [0.01, 0.50]")
        if not 0.01 <= self.max_omega_rad_s <= 1.20:
            raise Er2ConfigError("max_omega_rad_s must stay within [0.01, 1.20]")
        if not 0.10 <= self.max_segment_s <= 5.0:
            raise Er2ConfigError("max_segment_s must stay within [0.10, 5.0]")
        if not self.max_segment_s + 1.0 <= self.session_watchdog_s <= 600.0:
            raise Er2ConfigError("session_watchdog_s must exceed max_segment_s and stay <= 600")
        if not 0.25 <= self.media_timeout_s <= 5.0:
            raise Er2ConfigError("media_timeout_s must stay within [0.25, 5.0]")

    @classmethod
    def from_env(cls) -> "Er2Config":
        return cls(
            preview_model=os.environ.get("R2B4_ER2_PREVIEW_MODEL", PREVIEW_MODEL).strip(),
            streaming_model=os.environ.get("R2B4_ER2_STREAMING_MODEL", STREAMING_MODEL).strip(),
            heartbeat_s=_env_float("R2B4_ER2_HEARTBEAT_S", 1.0),
            max_v_mps=_env_float("R2B4_ER2_MAX_V_MPS", 0.20),
            max_omega_rad_s=_env_float("R2B4_ER2_MAX_OMEGA_RAD_S", 0.60),
            max_segment_s=_env_float("R2B4_ER2_MAX_SEGMENT_S", 1.50),
            session_watchdog_s=_env_float("R2B4_ER2_WATCHDOG_S", 10.0),
            media_timeout_s=_env_float("R2B4_ER2_MEDIA_TIMEOUT_S", 1.25),
        )


def api_key_from_env() -> str:
    value = (os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY") or "").strip()
    if not value:
        raise Er2ConfigError("GEMINI_API_KEY (or GOOGLE_API_KEY) is not configured")
    return value


__all__ = [
    "Er2Config",
    "Er2ConfigError",
    "PREVIEW_MODEL",
    "STREAMING_MODEL",
    "api_key_from_env",
]
