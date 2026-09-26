"""Local Piper text-to-speech adapter for R2B4.

Host-side only. Piper is installed into a dedicated application directory with
``pip --target`` (not into the Debian-managed Python environment and not into a
virtual environment). The voice model is loaded once per client instance and
synthesis returns the existing immutable GeneratedSpeech PCM contract used by
PcmWavePlayer. No network access is performed at runtime.
"""
from __future__ import annotations

import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from .gemini_tts import GeneratedSpeech


class PiperTtsError(RuntimeError):
    pass


def default_piper_data_dir() -> Path:
    configured = os.environ.get("R2B4_PIPER_DATA_DIR", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    xdg_data = os.environ.get("XDG_DATA_HOME", "").strip()
    base = Path(xdg_data).expanduser() if xdg_data else Path.home() / ".local" / "share"
    return (base / "r2b4" / "piper").resolve()


def piper_python_dir() -> Path:
    configured = os.environ.get("R2B4_PIPER_PYTHON_DIR", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    return default_piper_data_dir() / "python"


def ensure_piper_importable(dependency_dir: Path | str | None = None) -> Any:
    """Import Piper, adding only the dedicated application target if needed."""
    try:
        return importlib.import_module("piper")
    except ImportError as first_exc:
        dependency_dir = (
            Path(dependency_dir).expanduser().resolve()
            if dependency_dir is not None
            else piper_python_dir()
        )
        if not dependency_dir.is_dir():
            raise PiperTtsError(
                f"Piper dependency directory not found: {dependency_dir}; rerun the local TTS installer"
            ) from first_exc
        dependency_text = str(dependency_dir)
        if dependency_text not in sys.path:
            # Use append, not insert(0): already-installed R2B4/system packages keep
            # precedence while Piper-only dependencies remain discoverable.
            sys.path.append(dependency_text)
        try:
            return importlib.import_module("piper")
        except ImportError as exc:
            raise PiperTtsError(
                f"Piper is not importable from dedicated target: {dependency_dir}"
            ) from exc


@dataclass(frozen=True, slots=True)
class PiperTtsConfig:
    model_path: Path
    voice: str = "hu_HU-anna-medium"
    max_text_chars: int = 2000

    def __post_init__(self) -> None:
        object.__setattr__(self, "model_path", Path(self.model_path).expanduser().resolve())
        if not self.voice.strip():
            raise ValueError("Piper TTS voice must be non-empty")
        if self.max_text_chars <= 0:
            raise ValueError("max_text_chars must be positive")


VoiceLoader = Callable[[str], Any]


def _default_voice_loader(model_path: str) -> Any:
    module = ensure_piper_importable()
    try:
        voice_class = module.PiperVoice
    except AttributeError as exc:
        raise PiperTtsError("installed Piper package has no PiperVoice API") from exc
    try:
        return voice_class.load(model_path)
    except Exception as exc:
        raise PiperTtsError(
            f"failed to load Piper voice model: {type(exc).__name__}: {exc}"
        ) from exc


class PiperTtsClient:
    """In-process local Piper client with one-time model loading."""

    def __init__(
        self,
        config: PiperTtsConfig,
        *,
        voice_loader: VoiceLoader = _default_voice_loader,
    ) -> None:
        if not isinstance(config, PiperTtsConfig):
            raise TypeError("config must be PiperTtsConfig")
        if not callable(voice_loader):
            raise TypeError("voice_loader must be callable")
        model_path = config.model_path
        config_path = Path(str(model_path) + ".json")
        if not model_path.is_file():
            raise PiperTtsError(f"Piper voice model not found: {model_path}")
        if not config_path.is_file():
            raise PiperTtsError(f"Piper voice config not found: {config_path}")
        self._config = config
        self._voice = voice_loader(str(model_path))

    @property
    def model(self) -> str:
        return "piper-tts"

    @property
    def voice(self) -> str:
        return self._config.voice

    def synthesize(self, text: str) -> GeneratedSpeech:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("TTS text must be non-empty")
        spoken = text.strip()
        if len(spoken) > self._config.max_text_chars:
            raise ValueError("TTS text exceeds configured maximum")

        try:
            chunks = tuple(self._voice.synthesize(spoken))
        except Exception as exc:
            raise PiperTtsError(
                f"Piper synthesis failed: {type(exc).__name__}: {exc}"
            ) from exc
        if not chunks:
            raise PiperTtsError("Piper returned no audio")

        first = chunks[0]
        try:
            sample_rate_hz = int(first.sample_rate)
            sample_width_bytes = int(first.sample_width)
            channels = int(first.sample_channels)
        except (AttributeError, TypeError, ValueError) as exc:
            raise PiperTtsError("Piper returned an invalid audio chunk") from exc

        payloads: list[bytes] = []
        for chunk in chunks:
            try:
                if (
                    int(chunk.sample_rate) != sample_rate_hz
                    or int(chunk.sample_width) != sample_width_bytes
                    or int(chunk.sample_channels) != channels
                ):
                    raise PiperTtsError("Piper changed audio format within one utterance")
                payload = bytes(chunk.audio_int16_bytes)
            except (AttributeError, TypeError, ValueError) as exc:
                raise PiperTtsError("Piper returned an invalid audio chunk") from exc
            if payload:
                payloads.append(payload)

        pcm = b"".join(payloads)
        if not pcm:
            raise PiperTtsError("Piper returned empty audio")
        return GeneratedSpeech(
            pcm=pcm,
            sample_rate_hz=sample_rate_hz,
            channels=channels,
            sample_width_bytes=sample_width_bytes,
            model=self.model,
            voice=self.voice,
        )


__all__ = [
    "PiperTtsClient",
    "PiperTtsConfig",
    "PiperTtsError",
    "default_piper_data_dir",
    "ensure_piper_importable",
    "piper_python_dir",
]
