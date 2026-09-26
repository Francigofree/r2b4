"""R2B4 TTS provider selection.

Piper is the production default. Gemini TTS remains available only when selected
explicitly with R2B4_TTS_PROVIDER=gemini.
"""
from __future__ import annotations

import os
import stat
from pathlib import Path
from typing import Mapping

from .gemini_tts import GeminiTtsClient, GeminiTtsConfig
from .piper_tts import (
    PiperTtsClient,
    PiperTtsConfig,
    ensure_piper_importable,
    piper_python_dir,
)

DEFAULT_TTS_PROVIDER = "piper"
DEFAULT_PIPER_VOICE = "hu_HU-anna-medium"
DEFAULT_GEMINI_TTS_MODEL = "gemini-3.1-flash-tts-preview"
DEFAULT_GEMINI_TTS_VOICE = "Kore"


def _root(project_root: Path | str | None) -> Path:
    if project_root is not None:
        return Path(project_root).expanduser().resolve()
    configured = os.environ.get("R2B4_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    return Path(__file__).resolve().parents[1]



def _load_project_env(root: Path) -> dict[str, str]:
    path = root / "conf" / ".wake.env"
    if not path.is_file():
        return {}
    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077:
        raise RuntimeError(f"secret file permissions are too open: {oct(mode)}; expected 0o600")
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and value:
            values[key] = value
    return values


def _setting(project_env: Mapping[str, str] | None, name: str) -> str | None:
    value = os.environ.get(name)
    if isinstance(value, str) and value.strip():
        return value.strip()
    if project_env is not None:
        value = project_env.get(name)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def resolve_tts_provider(value: str | None) -> str:
    provider = (value or DEFAULT_TTS_PROVIDER).strip().lower()
    aliases = {"local": "piper", "cloud": "gemini"}
    provider = aliases.get(provider, provider)
    if provider not in {"piper", "gemini"}:
        raise ValueError("R2B4_TTS_PROVIDER must be piper or gemini")
    return provider


def _piper_voice_and_model(
    root: Path,
    project_env: Mapping[str, str] | None,
) -> tuple[str, Path]:
    voice = _setting(project_env, "R2B4_PIPER_VOICE") or DEFAULT_PIPER_VOICE
    explicit_model = _setting(project_env, "R2B4_PIPER_MODEL_PATH")
    if explicit_model:
        return voice, Path(explicit_model).expanduser().resolve()
    data_dir_raw = _setting(project_env, "R2B4_PIPER_DATA_DIR")
    if data_dir_raw:
        data_dir = Path(data_dir_raw).expanduser().resolve()
    else:
        xdg_data = os.environ.get("XDG_DATA_HOME", "").strip()
        base = Path(xdg_data).expanduser() if xdg_data else Path.home() / ".local" / "share"
        data_dir = (base / "r2b4" / "piper").resolve()
    return voice, data_dir / f"{voice}.onnx"


def _piper_dependency_dir(project_env: Mapping[str, str] | None) -> Path:
    explicit = _setting(project_env, "R2B4_PIPER_PYTHON_DIR")
    if explicit:
        return Path(explicit).expanduser().resolve()
    data_dir_raw = _setting(project_env, "R2B4_PIPER_DATA_DIR")
    if data_dir_raw:
        return Path(data_dir_raw).expanduser().resolve() / "python"
    return piper_python_dir()


def build_tts_client(
    project_root: Path | str | None = None,
    project_env: Mapping[str, str] | None = None,
    *,
    gemini_api_key: str | None = None,
):
    root = _root(project_root)
    if project_env is None:
        project_env = _load_project_env(root)
    provider = resolve_tts_provider(_setting(project_env, "R2B4_TTS_PROVIDER"))
    if provider == "piper":
        voice, model_path = _piper_voice_and_model(root, project_env)
        ensure_piper_importable(_piper_dependency_dir(project_env))
        return PiperTtsClient(PiperTtsConfig(model_path=model_path, voice=voice))

    api_key = gemini_api_key or _setting(project_env, "GEMINI_API_KEY") or _setting(project_env, "GOOGLE_API_KEY")
    return GeminiTtsClient(
        api_key=api_key,
        config=GeminiTtsConfig(
            model=_setting(project_env, "R2B4_TTS_MODEL") or DEFAULT_GEMINI_TTS_MODEL,
            voice=_setting(project_env, "R2B4_TTS_VOICE") or DEFAULT_GEMINI_TTS_VOICE,
        ),
    )


def diagnose_tts(
    project_root: Path | str | None = None,
    project_env: Mapping[str, str] | None = None,
    *,
    gemini_api_key: str | None = None,
) -> dict[str, object]:
    root = _root(project_root)
    if project_env is None:
        project_env = _load_project_env(root)
    provider = resolve_tts_provider(_setting(project_env, "R2B4_TTS_PROVIDER"))
    if provider == "piper":
        voice, model_path = _piper_voice_and_model(root, project_env)
        config_path = Path(str(model_path) + ".json")
        dependency_dir = _piper_dependency_dir(project_env)
        try:
            ensure_piper_importable(dependency_dir)
            dependency_ok = True
        except Exception:
            dependency_ok = False
        model_ok = model_path.is_file() and config_path.is_file()
        return {
            "tts_provider": "piper",
            "tts_model": "piper-tts",
            "tts_voice": voice,
            "tts_api_key": "NOT_REQUIRED",
            "tts_dependency": "PASS" if dependency_ok else "FAIL",
            "tts_dependency_dir": str(dependency_dir),
            "tts_model_file": "PASS" if model_ok else "FAIL",
            "tts_model_path": str(model_path),
            "tts_status": "PASS" if dependency_ok and model_ok else "FAIL",
        }

    key = gemini_api_key or _setting(project_env, "GEMINI_API_KEY") or _setting(project_env, "GOOGLE_API_KEY")
    return {
        "tts_provider": "gemini",
        "tts_model": _setting(project_env, "R2B4_TTS_MODEL") or DEFAULT_GEMINI_TTS_MODEL,
        "tts_voice": _setting(project_env, "R2B4_TTS_VOICE") or DEFAULT_GEMINI_TTS_VOICE,
        "tts_api_key": "PASS" if key else "FAIL",
        "tts_status": "PASS" if key else "FAIL",
    }


__all__ = [
    "DEFAULT_GEMINI_TTS_MODEL",
    "DEFAULT_GEMINI_TTS_VOICE",
    "DEFAULT_PIPER_VOICE",
    "DEFAULT_TTS_PROVIDER",
    "build_tts_client",
    "diagnose_tts",
    "resolve_tts_provider",
]
