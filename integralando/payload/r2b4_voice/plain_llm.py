"""Host-side one-turn LLM -> stdout -> TTS path for the root ``r`` launcher.

The selected provider follows ``R2B4_LLM_PROVIDER``. OpenAI/ChatGPT is the
default; Gemini and Groq remain explicit fallbacks. This module owns no
robot/runtime/ER2 state and cannot execute robot actions.
"""
from __future__ import annotations

import json
import os
import stat
import threading
import urllib.error
import urllib.request
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Callable, TextIO

from .gemini_llm import GeminiChatConfig, GeminiRequestError
from .groq_llm import GroqChatConfig, LLMRequestError
from .llm_provider import (
    api_key_env_for,
    default_model_for,
    resolve_llm_provider,
)
from .openai_llm import OpenAIChatConfig, OpenAIPlainClient
from .tts_provider import build_tts_client
from .voice_output import PcmWavePlayer


UrlOpen = Callable[..., object]
_TTS_CHUNK_CHARS = 320


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


def _setting(project_env: Mapping[str, str], name: str) -> str | None:
    value = os.environ.get(name)
    if isinstance(value, str) and value.strip():
        return value.strip()
    value = project_env.get(name)
    return value.strip() if isinstance(value, str) and value.strip() else None


class PlainGeminiClient:
    """One-turn unstructured Gemini Interactions API fallback client."""

    def __init__(
        self,
        api_key: str | None = None,
        config: GeminiChatConfig | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("GEMINI_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise GeminiRequestError("GEMINI_API_KEY (or GOOGLE_API_KEY) is not configured")
        cfg = config or GeminiChatConfig()
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._api_key = key.strip()
        self._config = cfg
        self._urlopen = urlopen

    @property
    def model(self) -> str:
        return self._config.model

    def complete_text(self, prompt: str) -> str:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt must be non-empty text")
        body = {
            "model": self._config.model,
            "input": prompt.strip(),
            "store": False,
            "generation_config": {"thinking_level": self._config.thinking_level},
        }
        request = urllib.request.Request(
            self._config.endpoint,
            data=json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            method="POST",
            headers={
                "x-goog-api-key": self._api_key,
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-plain-llm/2",
            },
        )
        try:
            response = self._urlopen(request, timeout=self._config.timeout_s)
            payload = response.read()  # type: ignore[attr-defined]
        except urllib.error.HTTPError as exc:
            detail = self._http_error_detail(exc)
            suffix = f": {detail}" if detail else ""
            raise GeminiRequestError(f"Gemini HTTP {exc.code}{suffix}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise GeminiRequestError(f"Gemini request failed: {type(exc).__name__}") from exc

        try:
            decoded = json.loads(payload.decode("utf-8"))
            return self._extract_output_text(decoded)
        except (UnicodeError, json.JSONDecodeError, KeyError, IndexError, TypeError, ValueError) as exc:
            raise GeminiRequestError("Gemini returned an invalid plain-text response") from exc

    @staticmethod
    def _extract_output_text(decoded: object) -> str:
        if not isinstance(decoded, Mapping):
            raise ValueError("response is not an object")
        steps = decoded.get("steps")
        if not isinstance(steps, list):
            raise ValueError("response has no steps")
        for step in reversed(steps):
            if not isinstance(step, Mapping) or step.get("type") != "model_output":
                continue
            content = step.get("content")
            if not isinstance(content, list):
                continue
            chunks = [
                item.get("text")
                for item in content
                if isinstance(item, Mapping)
                and item.get("type") == "text"
                and isinstance(item.get("text"), str)
            ]
            text = "".join(chunks).strip()
            if text:
                return text
        raise ValueError("response contains no model text output")

    @staticmethod
    def _http_error_detail(exc: urllib.error.HTTPError) -> str:
        try:
            raw = exc.read().decode("utf-8", errors="replace")
            parsed = json.loads(raw)
            if isinstance(parsed, Mapping):
                error = parsed.get("error")
                if isinstance(error, Mapping):
                    return str(error.get("message", ""))[:300]
        except Exception:
            pass
        return ""


class PlainGroqClient:
    """Small unstructured Groq fallback for parity with provider selection."""

    def __init__(
        self,
        api_key: str | None = None,
        config: GroqChatConfig | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("GROQ_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise LLMRequestError("GROQ_API_KEY is not configured")
        cfg = config or GroqChatConfig()
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._api_key = key.strip()
        self._config = cfg
        self._urlopen = urlopen

    @property
    def model(self) -> str:
        return self._config.model

    def complete_text(self, prompt: str) -> str:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt must be non-empty text")
        body = {
            "model": self._config.model,
            "messages": [{"role": "user", "content": prompt.strip()}],
        }
        request = urllib.request.Request(
            self._config.endpoint,
            data=json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            method="POST",
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-plain-llm/2",
            },
        )
        try:
            response = self._urlopen(request, timeout=self._config.timeout_s)
            payload = response.read()  # type: ignore[attr-defined]
        except urllib.error.HTTPError as exc:
            raise LLMRequestError(f"Groq LLM HTTP {exc.code}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise LLMRequestError(f"Groq LLM request failed: {type(exc).__name__}") from exc
        try:
            decoded = json.loads(payload.decode("utf-8"))
            content = decoded["choices"][0]["message"]["content"]
        except (UnicodeError, json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            raise LLMRequestError("Groq LLM returned an invalid plain response") from exc
        if not isinstance(content, str) or not content.strip():
            raise LLMRequestError("Groq LLM returned empty text")
        return content.strip()


def _speech_chunks(text: str, max_chars: int = _TTS_CHUNK_CHARS) -> tuple[str, ...]:
    if not isinstance(text, str) or not text.strip():
        raise ValueError("speech text must be non-empty")
    if max_chars <= 0:
        raise ValueError("max_chars must be positive")

    remaining = text.strip()
    chunks: list[str] = []
    while len(remaining) > max_chars:
        window = remaining[: max_chars + 1]
        split_at = max(window.rfind("\n\n"), window.rfind(". "), window.rfind("! "), window.rfind("? "))
        if split_at < max_chars // 3:
            split_at = window.rfind(" ")
        if split_at < max_chars // 3:
            split_at = max_chars
        elif window[split_at : split_at + 2] in {". ", "! ", "? "}:
            split_at += 1
        piece = remaining[:split_at].strip()
        if piece:
            chunks.append(piece)
        remaining = remaining[split_at:].strip()
    if remaining:
        chunks.append(remaining)
    return tuple(chunks)


def _start_tts_warmup(tts: Any) -> threading.Thread | None:
    warmup = getattr(tts, "warmup", None)
    if not callable(warmup):
        return None

    def run() -> None:
        try:
            warmup()
        except Exception:
            pass

    thread = threading.Thread(target=run, name="r2b4-plain-tts-warmup", daemon=True)
    thread.start()
    return thread


def _build_plain_client(provider: str, api_key: str | None, model: str):
    if provider == "openai":
        return OpenAIPlainClient(
            api_key=api_key,
            config=OpenAIChatConfig(model=model),
        )
    if provider == "gemini":
        return PlainGeminiClient(
            api_key=api_key,
            config=GeminiChatConfig(model=model),
        )
    return PlainGroqClient(
        api_key=api_key,
        config=GroqChatConfig(model=model),
    )


def run_plain_prompt(
    prompt: str,
    *,
    project_root: Path | str | None = None,
    client: Any | None = None,
    tts: Any | None = None,
    player: Any | None = None,
    stdout: TextIO | None = None,
) -> int:
    """Execute one provider-selected plain turn and speak the same returned text."""
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("prompt must be non-empty text")

    root = _root(project_root)
    project_env = _load_project_env(root)
    provider = resolve_llm_provider(_setting(project_env, "R2B4_LLM_PROVIDER"))
    model = _setting(project_env, "R2B4_LLM_MODEL") or default_model_for(provider)
    api_key = _setting(project_env, api_key_env_for(provider))
    gemini_tts_key = (
        _setting(project_env, "GEMINI_API_KEY")
        or _setting(project_env, "GOOGLE_API_KEY")
    )

    if client is None:
        client = _build_plain_client(provider, api_key, model)
    if tts is None:
        tts = build_tts_client(
            project_root=root,
            project_env=project_env,
            gemini_api_key=gemini_tts_key,
            resident=True,
        )
    warmup_thread = _start_tts_warmup(tts)

    answer = client.complete_text(prompt.strip())
    if not isinstance(answer, str) or not answer.strip():
        raise RuntimeError(f"{provider} returned empty plain-text output")
    answer = answer.strip()

    print(answer, file=stdout, flush=True)

    if player is None:
        player = PcmWavePlayer()

    _ = warmup_thread
    for chunk in _speech_chunks(answer):
        speech = tts.synthesize(chunk)
        player.play(speech)
    return 0


__all__ = ["PlainGeminiClient", "PlainGroqClient", "run_plain_prompt"]
