# Changed files

## Modified by installer

- `v3/launcher_cli.py`
  - adds plain LLM metadata to `command_catalog()`
  - documents `r "PROMPT"` and `r -- "PROMPT"`
  - adds `_plain_prompt()` host-side dispatch helper
  - `--` forces plain-prompt mode
  - unknown non-option text falls back to plain Gemini instead of `_unknown_command()`

## New

- `r2b4_voice/plain_llm.py`
  - one-turn, unstructured Gemini Interactions API adapter
  - reads existing R2B4 API/model configuration
  - prints the answer before TTS playback
  - reuses `build_tts_client()` and `PcmWavePlayer`
  - chunks long text below the existing TTS input limit

- `tests/feature/test_plain_llm_launcher.py`
  - verifies plain Gemini request format
  - verifies output + long-answer speech path
  - verifies unknown-text launcher dispatch
  - verifies `r -- "s"` collision escape
  - verifies known robot commands retain precedence
