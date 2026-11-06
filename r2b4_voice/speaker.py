"""Host-side fixed WAV acknowledgement over the Linux audio stack."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import Callable


class SpeakerError(RuntimeError):
    pass


RunCommand = Callable[..., subprocess.CompletedProcess[bytes]]


class ReadyWaveSpeaker:
    __slots__ = ("_asset", "_players", "_run")

    def __init__(
        self,
        asset: Path | str | None = None,
        *,
        players: tuple[str, ...] = ("pw-play", "paplay", "aplay"),
        run: RunCommand = subprocess.run,
    ) -> None:
        default_asset = Path(__file__).resolve().parent / "assets" / "kesz_vagyok.wav"
        self._asset = Path(asset) if asset is not None else default_asset
        if not players or any(not isinstance(item, str) or not item for item in players):
            raise ValueError("players must contain executable names")
        if not callable(run):
            raise TypeError("run must be callable")
        self._players = players
        self._run = run

    @property
    def asset(self) -> Path:
        return self._asset

    def available_player(self) -> str | None:
        for player in self._players:
            found = shutil.which(player)
            if found:
                return found
        return None

    def play_ready(self) -> str:
        if not self._asset.is_file():
            raise SpeakerError(f"ready WAV is missing: {self._asset}")
        player = self.available_player()
        if player is None:
            raise SpeakerError("no supported Linux WAV player found (pw-play/paplay/aplay)")
        name = Path(player).name
        argv = [player, str(self._asset)]
        if name == "aplay":
            argv = [player, "-q", str(self._asset)]
        try:
            result = self._run(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=10.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise SpeakerError(f"ready WAV playback failed: {type(exc).__name__}") from exc
        if result.returncode != 0:
            detail = result.stderr.decode("utf-8", "replace").strip() if result.stderr else ""
            raise SpeakerError(f"ready WAV player exited {result.returncode}{': ' + detail if detail else ''}")
        return name


__all__ = ["ReadyWaveSpeaker", "SpeakerError"]
