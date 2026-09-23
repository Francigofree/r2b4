"""Offline analysis scope selected from capture metadata, never view frequency."""

from __future__ import annotations

from .capture_rate import CONTROL_CAPTURE_HZ, validate_capture_hz
from .mcap_reader import McapReader

BEHAVIORAL = "BEHAVIORAL_HIGH_LEVEL"
FORENSIC = "FORENSIC_LOW_LEVEL"


def capture_analysis_profile(reader: McapReader) -> dict[str, object]:
    metadata = reader.latest_metadata("r2b4.capture") or {}
    hz = validate_capture_hz(metadata.get("tick_sample_hz", CONTROL_CAPTURE_HZ))
    sampled = hz <= 10
    return {
        "name": BEHAVIORAL if sampled else FORENSIC,
        "tick_sample_hz": hz,
        "selection_source": (
            "r2b4.capture.tick_sample_hz"
            if "tick_sample_hz" in metadata else "legacy_default_50_hz"
        ),
        "exact_replay_applicable": not sampled,
        "low_level_severity_cap": "WARNING" if sampled else None,
        "limitations": [
            "Counts refer to captured samples, not all control ticks or time occupancy.",
            "Transitions are observed at samples; intervening states may be missing.",
            "Path length is a sampled pose estimate, not physical ground truth.",
            "No exact replay, control jitter, jerk or raw-sensor continuity verdict.",
        ] if sampled else [],
    }
