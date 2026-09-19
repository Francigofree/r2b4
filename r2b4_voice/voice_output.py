"""Blocking host-side PCM->WAV playback for R2B4 conversation speech."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import wave
from pathlib import Path
from typing import Callable

from .gemini_tts import GeneratedSpeech


class VoicePlaybackError(RuntimeError):
    pass


RunCommand = Callable[..., subprocess.CompletedProcess[bytes]]
WhichCommand = Callable[[str], str | None]


class PcmWavePlayer:
    """Play immutable generated speech through the Linux user audio session.

    Playback is deliberately synchronous.  The voice supervisor does not feed
    microphone frames while this method is running, then discards the capture
    tail after playback so Alba cannot transcribe its own speaker output.
    """

    def __init__(
        self,
        *,
        players: tuple[str, ...] = ("pw-play", "paplay", "aplay"),
        run: RunCommand = subprocess.run,
        which: WhichCommand = shutil.which,
    ) -> None:
        if not players or any(not isinstance(item, str) or not item for item in players):
            raise ValueError("players must contain executable names")
        if not callable(run) or not callable(which):
            raise TypeError("run and which must be callable")
        self._players = players
        self._run = run
        self._which = which

    def available_player(self) -> str | None:
        for name in self._players:
            found = self._which(name)
            if found:
                return found
        return None

    def play(self, speech: GeneratedSpeech) -> str:
        if not isinstance(speech, GeneratedSpeech):
            raise TypeError("speech must be GeneratedSpeech")
        player = self.available_player()
        if player is None:
            raise VoicePlaybackError("no supported Linux WAV player found (pw-play/paplay/aplay)")

        fd, temp_name = tempfile.mkstemp(prefix="r2b4_tts_", suffix=".wav")
        os.close(fd)
        path = Path(temp_name)
        try:
            os.chmod(path, 0o600)
            with wave.open(str(path), "wb") as wav:
                wav.setnchannels(speech.channels)
                wav.setsampwidth(speech.sample_width_bytes)
                wav.setframerate(speech.sample_rate_hz)
                wav.writeframes(speech.pcm)

            name = Path(player).name
            argv = [player, str(path)]
            if name == "aplay":
                argv = [player, "-q", str(path)]
            duration_s = len(speech.pcm) / (
                speech.sample_rate_hz * speech.channels * speech.sample_width_bytes
            )
            timeout_s = max(10.0, duration_s + 8.0)
            try:
                result = self._run(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=timeout_s,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise VoicePlaybackError(f"TTS playback failed: {type(exc).__name__}") from exc
            if result.returncode != 0:
                detail = result.stderr.decode("utf-8", "replace").strip() if result.stderr else ""
                raise VoicePlaybackError(
                    f"TTS player exited {result.returncode}{': ' + detail if detail else ''}"
                )
            return name
        finally:
            try:
                path.unlink()
            except FileNotFoundError:
                pass


__all__ = ["PcmWavePlayer", "VoicePlaybackError"]
