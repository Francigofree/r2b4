"""Provider selection for the host-side R2B4 conversation LLM."""

from __future__ import annotations

import os

from .gemini_llm import GeminiChatConfig, GeminiStructuredChatClient
from .groq_llm import GroqChatConfig, GroqStructuredChatClient


DEFAULT_LLM_PROVIDER = "gemini"
DEFAULT_GEMINI_MODEL = "gemini-3.5-flash-lite"
DEFAULT_GROQ_MODEL = "openai/gpt-oss-20b"
SUPPORTED_LLM_PROVIDERS = ("gemini", "groq")


def resolve_llm_provider(provider: str | None = None) -> str:
    value = provider if provider is not None else os.environ.get("R2B4_LLM_PROVIDER", DEFAULT_LLM_PROVIDER)
    normalized = value.strip().lower() if isinstance(value, str) else ""
    if normalized not in SUPPORTED_LLM_PROVIDERS:
        raise ValueError(f"unsupported LLM provider: {value!r}")
    return normalized


def default_model_for(provider: str) -> str:
    provider = resolve_llm_provider(provider)
    return DEFAULT_GEMINI_MODEL if provider == "gemini" else DEFAULT_GROQ_MODEL


def build_llm_client(
    *,
    provider: str | None = None,
    api_key: str | None = None,
    model: str | None = None,
):
    selected = resolve_llm_provider(provider)
    selected_model = model or os.environ.get("R2B4_LLM_MODEL") or default_model_for(selected)
    if selected == "gemini":
        return GeminiStructuredChatClient(
            api_key=api_key,
            config=GeminiChatConfig(model=selected_model),
        )
    return GroqStructuredChatClient(
        api_key=api_key,
        config=GroqChatConfig(model=selected_model),
    )


__all__ = [
    "DEFAULT_GEMINI_MODEL",
    "DEFAULT_GROQ_MODEL",
    "DEFAULT_LLM_PROVIDER",
    "SUPPORTED_LLM_PROVIDERS",
    "build_llm_client",
    "default_model_for",
    "resolve_llm_provider",
]
