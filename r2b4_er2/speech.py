"""Host-side spoken reporting for ER2 final text responses.

Speech is not a robot authority and is deliberately outside L0-L12. It reuses
the shared local-first TTS adapter and Linux PCM/WAV playback path.
"""
from __future__ import annotations

from typing import Any

from r2b4_voice.tts_provider import build_tts_client
from r2b4_voice.voice_output import PcmWavePlayer

from .evidence import Er2Evidence


class Er2SpeechError(RuntimeError):
    pass


class Er2SpeechReporter:
    def __init__(
        self,
        *,
        api_key: str | None = None,
        evidence: Er2Evidence | None = None,
        tts: Any | None = None,
        player: Any | None = None,
    ) -> None:
        self.evidence = evidence
        self.tts = tts if tts is not None else build_tts_client(gemini_api_key=api_key)
        self.player = player if player is not None else PcmWavePlayer()

    def _emit(self, event_type: str, **fields: object) -> None:
        if self.evidence is not None:
            self.evidence.emit(event_type, **fields)

    def speak(self, text: str) -> dict[str, object]:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("spoken ER2 text must be non-empty")
        spoken = text.strip()
        self._emit("ER2_SPEECH_START", text_chars=len(spoken))
        try:
            speech = self.tts.synthesize(spoken)
            player_name = self.player.play(speech)
        except Exception as exc:
            self._emit("ER2_SPEECH_ERROR", error_type=type(exc).__name__, error=str(exc)[:240])
            raise Er2SpeechError(f"ER2 spoken report failed: {type(exc).__name__}: {exc}") from exc
        result = {
            "player": str(player_name),
            "model": getattr(speech, "model", None),
            "voice": getattr(speech, "voice", None),
            "text_chars": len(spoken),
        }
        self._emit("ER2_SPEECH_COMPLETE", **result)
        return result


__all__ = ["Er2SpeechError", "Er2SpeechReporter"]
