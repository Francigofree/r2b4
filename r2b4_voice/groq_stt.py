"""Small standard-library Groq STT adapter for the wake service only."""

from __future__ import annotations

import io
import json
import os
import urllib.error
import urllib.request
import uuid
import wave
from dataclasses import dataclass
from typing import Callable

from .wake_core import WakeUtterance


class WakeTranscriptionError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class GroqWakeSttConfig:
    endpoint: str = "https://api.groq.com/openai/v1/audio/transcriptions"
    model: str = "whisper-large-v3-turbo"
    language: str = "hu"
    prompt: str = "Az ébresztőszó neve: Alba."
    timeout_s: float = 12.0

    def __post_init__(self) -> None:
        if not self.endpoint.startswith("https://"):
            raise ValueError("Groq STT endpoint must use https")
        if not self.model.strip():
            raise ValueError("model must be non-empty")
        if not self.language.strip():
            raise ValueError("language must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")


UrlOpen = Callable[..., object]


class GroqWakeTranscriber:
    __slots__ = ("_api_key", "_config", "_urlopen")

    def __init__(
        self,
        api_key: str | None = None,
        config: GroqWakeSttConfig = GroqWakeSttConfig(),
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("GROQ_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise WakeTranscriptionError("GROQ_API_KEY is not configured")
        if not isinstance(config, GroqWakeSttConfig):
            raise TypeError("config must be GroqWakeSttConfig")
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._api_key = key.strip()
        self._config = config
        self._urlopen = urlopen

    def transcribe(self, utterance: WakeUtterance) -> str:
        if not isinstance(utterance, WakeUtterance):
            raise TypeError("utterance must be WakeUtterance")
        wav_bytes = self._wav_bytes(utterance)
        boundary = "----r2b4wake" + uuid.uuid4().hex
        body = self._multipart_body(boundary, wav_bytes)
        request = urllib.request.Request(
            self._config.endpoint,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": f"multipart/form-data; boundary={boundary}",
                "Accept": "application/json",
                "User-Agent": "r2b4-wake/1",
            },
        )
        try:
            response = self._urlopen(request, timeout=self._config.timeout_s)
            payload = response.read()  # type: ignore[attr-defined]
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise WakeTranscriptionError(f"Groq STT request failed: {type(exc).__name__}") from exc
        try:
            decoded = json.loads(payload.decode("utf-8"))
            text = decoded["text"]
        except (UnicodeError, json.JSONDecodeError, KeyError, TypeError) as exc:
            raise WakeTranscriptionError("Groq STT returned an invalid response") from exc
        if not isinstance(text, str):
            raise WakeTranscriptionError("Groq STT response text is not a string")
        return text.strip()

    def _multipart_body(self, boundary: str, wav_bytes: bytes) -> bytes:
        fields = (
            ("model", self._config.model),
            ("language", self._config.language),
            ("prompt", self._config.prompt),
            ("response_format", "json"),
            ("temperature", "0"),
        )
        chunks: list[bytes] = []
        marker = boundary.encode("ascii")
        for name, value in fields:
            chunks.extend(
                (
                    b"--" + marker + b"\r\n",
                    f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode("utf-8"),
                    value.encode("utf-8") + b"\r\n",
                )
            )
        chunks.extend(
            (
                b"--" + marker + b"\r\n",
                b'Content-Disposition: form-data; name="file"; filename="wake.wav"\r\n',
                b"Content-Type: audio/wav\r\n\r\n",
                wav_bytes,
                b"\r\n--" + marker + b"--\r\n",
            )
        )
        return b"".join(chunks)

    @staticmethod
    def _wav_bytes(utterance: WakeUtterance) -> bytes:
        output = io.BytesIO()
        with wave.open(output, "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(utterance.sample_rate_hz)
            wav.writeframes(utterance.pcm)
        return output.getvalue()


__all__ = [
    "GroqWakeSttConfig",
    "GroqWakeTranscriber",
    "WakeTranscriptionError",
]
