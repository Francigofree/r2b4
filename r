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
import re
import subprocess
import tempfile
import wave
from pathlib import Path

from google import genai
from google.genai import types


def _gemini_api_key() -> str | None:
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    if key:
        return key

    root = Path(os.environ.get("R2B4_ROOT", "/home/alba/project_r2b4"))
    env_path = root / "conf" / ".wake.env"
    try:
        for raw_line in env_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            name, value = line.split("=", 1)
            if name.strip() == "GEMINI_API_KEY":
                return value.strip().strip('"').strip("'")
    except OSError:
        pass
    return None


text = Path(os.environ["R2B4_ER2_TTS_TEXT_FILE"]).read_text(
    encoding="utf-8",
    errors="replace",
).strip()
if not text:
    raise SystemExit(0)

api_key = _gemini_api_key()
client = genai.Client(api_key=api_key) if api_key else genai.Client()

response = client.models.generate_content(
    model="gemini-3.1-flash-tts-preview",
    contents=text,
    config=types.GenerateContentConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(
                    voice_name="Kore",
                )
            )
        ),
    ),
)

part = response.candidates[0].content.parts[0]
inline_data = part.inline_data
audio = inline_data.data
mime_type = (inline_data.mime_type or "").lower()

if not audio:
    raise RuntimeError("Gemini TTS returned no audio data")

rate_match = re.search(r"rate=(\d+)", mime_type)
sample_rate = int(rate_match.group(1)) if rate_match else 24000

with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
    wav_path = Path(handle.name)

try:
    if audio[:4] == b"RIFF" or "wav" in mime_type:
        wav_path.write_bytes(audio)
    else:
        with wave.open(str(wav_path), "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio)

    subprocess.run(
        ["/usr/bin/pw-play", str(wav_path)],
        check=True,
    )
finally:
    wav_path.unlink(missing_ok=True)
PY
}

ARGS=("$@")

# Human shorthand:
#   r er2 "mit látsz?"
# becomes the canonical ER2 CLI form:
#   r er2 preview "mit látsz?"
#
# The canonical ER2 commands themselves remain untouched.
if (( ${#ARGS[@]} >= 2 )) && [[ "${ARGS[0]}" == "er2" ]]; then
    case "${ARGS[1]}" in
        status|preview|stream|-h|--help)
            ;;
        *)
            QUESTION="${ARGS[*]:1}"
            ARGS=("er2" "preview" "$QUESTION")
            ;;
    esac
fi

# Only answer-producing ER2 preview calls are captured for speech.
# Terminal output and the original launcher return code are preserved.
if (( ${#ARGS[@]} >= 2 )) \
    && [[ "${ARGS[0]}" == "er2" ]] \
    && [[ "${ARGS[1]}" == "preview" ]]; then

    ER2_TEXT="$(mktemp)"
    trap 'rm -f "$ER2_TEXT"' EXIT

    set +e
    python3 -m v3.launcher_cli "${ARGS[@]}" | tee "$ER2_TEXT"
    RC=${PIPESTATUS[0]}
    set -e

    if [[ "$RC" -eq 0 && -s "$ER2_TEXT" ]]; then
        _er2_speak_file "$ER2_TEXT" || \
            printf 'WARNING: ER2 response arrived, but TTS playback failed.\n' >&2
    fi

    exit "$RC"
fi

exec python3 -m v3.launcher_cli "${ARGS[@]}"
