"""LLM provider selection, authentication discovery and automatic failover."""
from __future__ import annotations

import os
import stat
from collections.abc import Mapping
from pathlib import Path

from .gemini_llm import GeminiChatConfig, GeminiStructuredChatClient
from .groq_llm import GroqChatConfig, GroqStructuredChatClient
from .llm_failover import FailoverLLMClient, LLMProviderCandidate
from .openai_llm import OpenAIChatConfig, OpenAIResponsesChatClient
from .openai_oauth import ChatGPTOAuthTokenProvider, OAuthError, credential_status


DEFAULT_LLM_PROVIDER = "openai"
DEFAULT_OPENAI_MODEL = "gpt-5.6"
DEFAULT_GEMINI_MODEL = "gemini-2.5-flash"
DEFAULT_GROQ_MODEL = "openai/gpt-oss-20b"
SUPPORTED_LLM_PROVIDERS = ("openai", "chatgpt", "gemini", "groq")
_CANONICAL_PROVIDERS = ("openai", "gemini", "groq")


def resolve_llm_provider(provider: str | None = None) -> str:
    value = provider if provider is not None else os.environ.get("R2B4_LLM_PROVIDER", DEFAULT_LLM_PROVIDER)
    normalized = value.strip().lower() if isinstance(value, str) else ""
    if normalized == "chatgpt":
        normalized = "openai"
    if normalized not in _CANONICAL_PROVIDERS:
        raise ValueError(f"unsupported LLM provider: {value!r}")
    return normalized


def api_key_env_for(provider: str) -> str:
    return {
        "openai": "OPENAI_API_KEY",
        "gemini": "GEMINI_API_KEY",
        "groq": "GROQ_API_KEY",
    }[resolve_llm_provider(provider)]


def default_model_for(provider: str) -> str:
    selected = resolve_llm_provider(provider)
    return {
        "openai": DEFAULT_OPENAI_MODEL,
        "gemini": DEFAULT_GEMINI_MODEL,
        "groq": DEFAULT_GROQ_MODEL,
    }[selected]


def _root(project_root: Path | str | None) -> Path:
    if project_root is not None:
        return Path(project_root).expanduser().resolve()
    raw = os.environ.get("R2B4_ROOT", "").strip()
    if raw:
        return Path(raw).expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def load_project_env(project_root: Path | str | None = None) -> dict[str, str]:
    root = _root(project_root)
    path = root / "conf" / ".wake.env"
    if not path.is_file():
        return {}
    mode = stat.S_IMODE(path.stat().st_mode)
    if os.name != "nt" and mode & 0o077:
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


def _key_for(provider: str, project_env: Mapping[str, str], explicit_key: str | None, *, selected: str) -> str | None:
    if provider == selected and isinstance(explicit_key, str) and explicit_key.strip():
        return explicit_key.strip()
    if provider == "gemini":
        return _setting(project_env, "GEMINI_API_KEY") or _setting(project_env, "GOOGLE_API_KEY")
    return _setting(project_env, api_key_env_for(provider))


def model_for(
    provider: str,
    *,
    selected_provider: str,
    explicit_model: str | None,
    project_env: Mapping[str, str],
) -> str:
    canonical = resolve_llm_provider(provider)
    if canonical == selected_provider and isinstance(explicit_model, str) and explicit_model.strip():
        return explicit_model.strip()
    per_provider = _setting(project_env, f"R2B4_{canonical.upper()}_MODEL")
    if per_provider:
        return per_provider
    # Preserve the existing global model only for the selected provider. A
    # fallback must never accidentally send gpt-5.6 to Gemini, or vice versa.
    if canonical == selected_provider:
        global_model = _setting(project_env, "R2B4_LLM_MODEL")
        if global_model:
            return global_model
    return default_model_for(canonical)


def _provider_order(selected: str) -> tuple[str, ...]:
    return tuple([selected] + [name for name in _CANONICAL_PROVIDERS if name != selected])


def llm_auth_summary(
    project_root: Path | str | None = None,
    *,
    project_env: Mapping[str, str] | None = None,
) -> dict[str, object]:
    root = _root(project_root)
    values = dict(project_env) if project_env is not None else load_project_env(root)
    try:
        oauth = credential_status(root)
    except OAuthError as exc:
        oauth = {"status": "INVALID", "error": str(exc), "plan_scope": False}
    openai_key = bool(_setting(values, "OPENAI_API_KEY"))
    gemini_key = bool(_setting(values, "GEMINI_API_KEY") or _setting(values, "GOOGLE_API_KEY"))
    groq_key = bool(_setting(values, "GROQ_API_KEY"))
    oauth_ready = oauth.get("status") in {"READY", "REFRESHABLE"}
    return {
        "openai_oauth": oauth,
        "openai_api_key": "PRESENT" if openai_key else "MISSING",
        "gemini_api_key": "PRESENT" if gemini_key else "MISSING",
        "groq_api_key": "PRESENT" if groq_key else "MISSING",
        "llm_ready": bool(oauth_ready or openai_key or gemini_key or groq_key),
    }


def build_llm_client(
    *,
    provider: str | None = None,
    api_key: str | None = None,
    model: str | None = None,
    project_root: Path | str | None = None,
    project_env: Mapping[str, str] | None = None,
):
    selected = resolve_llm_provider(provider)
    root = _root(project_root)
    values = dict(project_env) if project_env is not None else load_project_env(root)
    failover_enabled = (_setting(values, "R2B4_LLM_FAILOVER") or "1").strip().lower() not in {"0", "false", "no", "off"}
    order = _provider_order(selected) if failover_enabled else (selected,)
    candidates: list[LLMProviderCandidate] = []

    for current in order:
        current_model = model_for(
            current,
            selected_provider=selected,
            explicit_model=model,
            project_env=values,
        )
        if current == "openai":
            oauth_provider = ChatGPTOAuthTokenProvider(root)
            if oauth_provider.available():
                candidates.append(
                    LLMProviderCandidate(
                        "openai_oauth",
                        OpenAIResponsesChatClient(
                            token_provider=oauth_provider,
                            config=OpenAIChatConfig(model=current_model),
                        ),
                    )
                )
            key = _key_for("openai", values, api_key, selected=selected)
            if key:
                candidates.append(
                    LLMProviderCandidate(
                        "openai_api_key",
                        OpenAIResponsesChatClient(api_key=key, config=OpenAIChatConfig(model=current_model)),
                    )
                )
            continue

        key = _key_for(current, values, api_key, selected=selected)
        if not key:
            continue
        if current == "gemini":
            candidates.append(
                LLMProviderCandidate(
                    "gemini",
                    GeminiStructuredChatClient(api_key=key, config=GeminiChatConfig(model=current_model)),
                )
            )
        elif current == "groq":
            candidates.append(
                LLMProviderCandidate(
                    "groq",
                    GroqStructuredChatClient(api_key=key, config=GroqChatConfig(model=current_model)),
                )
            )

    if not candidates:
        summary = llm_auth_summary(root, project_env=values)
        raise RuntimeError(
            "no usable LLM authentication is configured; run './r chatgpt login' "
            "or configure OPENAI_API_KEY / GEMINI_API_KEY / GROQ_API_KEY. "
            f"local_auth={summary}"
        )
    return FailoverLLMClient(candidates)


__all__ = [
    "DEFAULT_GEMINI_MODEL",
    "DEFAULT_GROQ_MODEL",
    "DEFAULT_LLM_PROVIDER",
    "DEFAULT_OPENAI_MODEL",
    "SUPPORTED_LLM_PROVIDERS",
    "api_key_env_for",
    "build_llm_client",
    "default_model_for",
    "llm_auth_summary",
    "load_project_env",
    "model_for",
    "resolve_llm_provider",
]
