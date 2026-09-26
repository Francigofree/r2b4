#!/usr/bin/env bash
# R2B4 single human/agent launcher bootstrap.
set -euo pipefail

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SELF_DIR="$(cd "$(dirname "$SELF")" && pwd)"
DEFAULT_ROOT="/home/alba/project_r2b4"

if [[ -n "${R2B4_ROOT:-}" ]]; then
    ROOT="$(readlink -f "$R2B4_ROOT")"
elif [[ -d "$SELF_DIR/v3" && -f "$SELF_DIR/pytest.ini" ]]; then
    ROOT="$SELF_DIR"
elif [[ -d "$DEFAULT_ROOT/v3" && -f "$DEFAULT_ROOT/pytest.ini" ]]; then
    ROOT="$DEFAULT_ROOT"
else
    printf 'ERROR: R2B4 repo not found. Set R2B4_ROOT=/path/to/project_r2b4\n' >&2
    exit 2
fi

export R2B4_ROOT="$ROOT"
cd "$ROOT"

_er2_speak_file() {
    local text_file="$1"
    R2B4_ER2_TTS_TEXT_FILE="$text_file" python3 - <<'PY'
import os
import subprocess
import tempfile
import wave
from pathlib import Path

from google import genai
from google.genai import types

text = Path(os.environ["R2B4_ER2_TTS_TEXT_FILE"]).read_text(
    encoding="utf-8", errors="replace"
).strip()
if not text:
    raise SystemExit(0)

client = genai.Client()
response = client.models.generate_content(
    model="gemini-3.1-flash-tts-preview",
    contents=text,
    config=types.GenerateContentConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(
                    voice_name="Kore"
                )
            )
        ),
    ),
)

part = response.candidates[0].content.parts[0]
audio = part.inline_data.data
mime_type = (part.inline_data.mime_type or "").lower()

with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
    wav_path = handle.name

try:
    if audio[:4] == b"RIFF" or "wav" in mime_type:
        Path(wav_path).write_bytes(audio)
    else:
        with wave.open(wav_path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(24000)
            wav_file.writeframes(audio)
    subprocess.run(["/usr/bin/pw-play", wav_path], check=True)
finally:
    Path(wav_path).unlink(missing_ok=True)
PY
}

# Speak only answer-producing ER2 preview/shorthand calls.
# Keep status and streaming behavior byte-for-byte on the original launcher path.
if [[ "${1:-}" == "er2" && "${2:-}" != "status" && "${2:-}" != "stream" ]]; then
    ER2_TEXT="$(mktemp)"
    trap 'rm -f "$ER2_TEXT"' EXIT

    set +e
    python3 -m v3.launcher_cli "$@" | tee "$ER2_TEXT"
    RC=${PIPESTATUS[0]}
    set -e

    if [[ "$RC" -eq 0 && -s "$ER2_TEXT" ]]; then
        _er2_speak_file "$ER2_TEXT" || \
            printf 'WARNING: ER2 response arrived, but TTS playback failed.\n' >&2
    fi
    exit "$RC"
fi

exec python3 -m v3.launcher_cli "$@"
