"""Deterministic fail-safe spoken STOP recognition for the host voice edge."""

from __future__ import annotations

import re
import unicodedata

_STOP_PHRASES = frozenset(
    {
        "allj",
        "allj meg",
        "alba allj",
        "alba allj meg",
        "azonnal allj",
        "azonnal allj meg",
        "alba azonnal allj",
        "alba azonnal allj meg",
        "stop",
        "alba stop",
        "stop alba",
    }
)


def _normalize(text: str) -> str:
    if not isinstance(text, str):
        return ""
    folded = unicodedata.normalize("NFKD", text.casefold())
    folded = "".join(ch for ch in folded if not unicodedata.combining(ch))
    folded = re.sub(r"[^a-z0-9]+", " ", folded)
    return " ".join(folded.split())


def is_stop_intent(text: str) -> bool:
    """Return True only for a small exact STOP phrase set; no fuzzy LLM matching."""

    return _normalize(text) in _STOP_PHRASES


__all__ = ["is_stop_intent"]
