"""Gemini text-to-speech adapter for R2B4 voice output.

Host-side only.  It receives the already-approved spoken_text produced by the
conversation layer and returns immutable mono S16_LE PCM.  It never owns robot
control authority.
"""

from __future__ import annotations

import base64
import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Callable


class GeminiTtsError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class GeneratedSpeech:
    pcm: bytes
    sample_rate_hz: int
    channels: int
    sample_width_bytes: int
    model: str
    voice: str

    def __post_init__(self) -> None:
        if not isinstance(self.pcm, bytes) or not self.pcm:
            raise ValueError("pcm must be non-empty bytes")
        if len(self.pcm) % 2:
            raise ValueError("S16_LE PCM byte count must be even")
        if self.sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be positive")
        if self.channels != 1:
            raise ValueError("R2B4 TTS currently requires mono audio")
        if self.sample_width_bytes != 2:
            raise ValueError("R2B4 TTS currently requires 16-bit PCM")
        if not self.model.strip() or not self.voice.strip():
            raise ValueError("model and voice must be non-empty")


@dataclass(frozen=True, slots=True)
class GeminiTtsConfig:
    model: str = "gemini-3.1-flash-tts-preview"
    voice: str = "Kore"
    timeout_s: float = 25.0
    endpoint_template: str = (
        "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
    )
    max_text_chars: int = 2000

    def __post_init__(self) -> None:
        if not self.model.strip():
            raise ValueError("TTS model must be non-empty")
        if not self.voice.strip():
            raise ValueError("TTS voice must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if not self.endpoint_template.startswith("https://") or "{model}" not in self.endpoint_template:
            raise ValueError("endpoint_template must be HTTPS and contain {model}")
        if self.max_text_chars <= 0:
            raise ValueError("max_text_chars must be positive")


UrlOpen = Callable[..., object]


class GeminiTtsClient:
    def __init__(
        self,
        api_key: str | None = None,
        config: GeminiTtsConfig | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("GEMINI_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise GeminiTtsError("GEMINI_API_KEY is not configured")
        cfg = config or GeminiTtsConfig(
            model=os.environ.get("R2B4_TTS_MODEL", "gemini-3.1-flash-tts-preview"),
            voice=os.environ.get("R2B4_TTS_VOICE", "Kore"),
        )
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._api_key = key.strip()
        self._config = cfg
        self._urlopen = urlopen

    @property
    def model(self) -> str:
        return self._config.model

    @property
    def voice(self) -> str:
        return self._config.voice

    def synthesize(self, text: str) -> GeneratedSpeech:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("TTS text must be non-empty")
        text = text.strip()
        if len(text) > self._config.max_text_chars:
            raise ValueError("TTS text exceeds configured maximum")

        endpoint = self._config.endpoint_template.format(
            model=urllib.parse.quote(self._config.model, safe="")
        )
        body = json.dumps(
            {
                "contents": [{"parts": [{"text": text}]}],
                "generationConfig": {
                    "responseModalities": ["AUDIO"],
                    "speechConfig": {
                        "voiceConfig": {
                            "prebuiltVoiceConfig": {"voiceName": self._config.voice}
                        }
                    },
                },
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        request = urllib.request.Request(
            endpoint,
            data=body,
            method="POST",
            headers={
                "x-goog-api-key": self._api_key,
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-voice-tts/1",
            },
        )
        try:
            response = self._urlopen(request, timeout=self._config.timeout_s)
            payload = response.read()  # type: ignore[attr-defined]
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                raw = exc.read().decode("utf-8", errors="replace")
                decoded = json.loads(raw)
                detail = str(decoded.get("error", {}).get("message", ""))[:300]
            except Exception:
                detail = ""
            suffix = f": {detail}" if detail else ""
            raise GeminiTtsError(f"Gemini TTS HTTP {exc.code}{suffix}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise GeminiTtsError(f"Gemini TTS request failed: {type(exc).__name__}") from exc

        try:
            decoded = json.loads(payload.decode("utf-8"))
            part = decoded["candidates"][0]["content"]["parts"][0]
            inline = part["inlineData"]
            encoded = inline["data"]
            mime_type = str(inline.get("mimeType", "audio/L16;codec=pcm;rate=24000"))
            pcm = base64.b64decode(encoded, validate=True)
        except (UnicodeError, json.JSONDecodeError, KeyError, IndexError, TypeError, ValueError) as exc:
            raise GeminiTtsError("Gemini TTS returned an invalid audio response") from exc
        if not pcm or len(pcm) % 2:
            raise GeminiTtsError("Gemini TTS returned invalid S16_LE PCM")

        rate = 24_000
        match = re.search(r"(?:rate|rate_hz)=([0-9]+)", mime_type, re.IGNORECASE)
        if match:
            rate = int(match.group(1))
        return GeneratedSpeech(
            pcm=pcm,
            sample_rate_hz=rate,
            channels=1,
            sample_width_bytes=2,
            model=self._config.model,
            voice=self._config.voice,
        )


__all__ = [
    "GeneratedSpeech",
    "GeminiTtsClient",
    "GeminiTtsConfig",
    "GeminiTtsError",
]
