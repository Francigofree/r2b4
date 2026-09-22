"""Shared capture sampling policy for the R2B4 V3 runtime."""

from __future__ import annotations

CONTROL_CAPTURE_HZ = 50
DEFAULT_CAPTURE_HZ = 10
CAPTURE_HZ_VALUES = (1, 5, 10, 50)
CAPTURE_HZ_CHOICES = frozenset(CAPTURE_HZ_VALUES)


def validate_capture_hz(value: object) -> int:
    """Return one supported capture rate without silently rounding values."""

    if isinstance(value, bool):
        raise ValueError("capture_hz must be one of: 1, 5, 10, 50")
    if isinstance(value, int):
        hz = value
    elif isinstance(value, str):
        raw = value.strip()
        if not raw.isdigit():
            raise ValueError("capture_hz must be one of: 1, 5, 10, 50")
        hz = int(raw)
    else:
        raise ValueError("capture_hz must be one of: 1, 5, 10, 50")
    if hz not in CAPTURE_HZ_CHOICES:
        raise ValueError("capture_hz must be one of: 1, 5, 10, 50")
    return hz


__all__ = [
    "CAPTURE_HZ_CHOICES",
    "CAPTURE_HZ_VALUES",
    "CONTROL_CAPTURE_HZ",
    "DEFAULT_CAPTURE_HZ",
    "validate_capture_hz",
]
