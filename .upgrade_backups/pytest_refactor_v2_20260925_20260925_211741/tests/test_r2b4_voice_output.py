import subprocess
import wave

from r2b4_voice.gemini_tts import GeneratedSpeech
from r2b4_voice.voice_output import PcmWavePlayer


def test_pcm_player_writes_valid_wav_and_uses_pw_play():
    observed = {}

    def run(argv, **kwargs):
        observed["argv"] = argv
        with wave.open(argv[-1], "rb") as wav:
            observed["channels"] = wav.getnchannels()
            observed["width"] = wav.getsampwidth()
            observed["rate"] = wav.getframerate()
            observed["frames"] = wav.readframes(wav.getnframes())
        return subprocess.CompletedProcess(argv, 0, b"", b"")

    player = PcmWavePlayer(
        run=run,
        which=lambda name: "/usr/bin/pw-play" if name == "pw-play" else None,
    )
    pcm = (321).to_bytes(2, "little", signed=True) * 100
    backend = player.play(GeneratedSpeech(pcm, 24000, 1, 2, "tts", "Kore"))
    assert backend == "pw-play"
    assert observed["channels"] == 1
    assert observed["width"] == 2
    assert observed["rate"] == 24000
    assert observed["frames"] == pcm
