from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import r2b4_voice.speaker as speaker_module
from r2b4_voice.speaker import ReadyWaveSpeaker


def test_ready_speaker_prefers_pipewire_player(monkeypatch, tmp_path: Path):
    asset = tmp_path / "ready.wav"
    asset.write_bytes(b"RIFFfake")
    monkeypatch.setattr(speaker_module.shutil, "which", lambda name: "/usr/bin/pw-play" if name == "pw-play" else None)
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0, stderr=b"")

    output = ReadyWaveSpeaker(asset, run=run).play_ready()
    assert output == "pw-play"
    assert calls == [["/usr/bin/pw-play", str(asset)]]
