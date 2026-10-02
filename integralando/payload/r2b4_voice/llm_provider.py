"""Provider selection for the host-side R2B4 conversation LLM."""

from __future__ import annotations

import os

from .gemini_llm import GeminiChatConfig, GeminiStructuredChatClient
from .groq_llm import GroqChatConfig, GroqStructuredChatClient
from .openai_llm import OpenAIChatConfig, OpenAIResponsesChatClient


DEFAULT_LLM_PROVIDER = "openai"
DEFAULT_OPENAI_MODEL = "gpt-5.6"
DEFAULT_GEMINI_MODEL = "gemini-2.5-flash"
DEFAULT_GROQ_MODEL = "openai/gpt-oss-20b"

# "chatgpt" is accepted as a human-friendly alias, but the canonical provider id
# remains "openai" because authentication and transport use the OpenAI API.
SUPPORTED_LLM_PROVIDERS = ("openai", "chatgpt", "gemini", "groq")


def resolve_llm_provider(provider: str | None = None) -> str:
    value = provider if provider is not None else os.environ.get(
        "R2B4_LLM_PROVIDER", DEFAULT_LLM_PROVIDER
    )
    normalized = value.strip().lower() if isinstance(value, str) else ""
    if normalized == "chatgpt":
        normalized = "openai"
    if normalized not in {"openai", "gemini", "groq"}:
        raise ValueError(f"unsupported LLM provider: {value!r}")
    return normalized


def api_key_env_for(provider: str) -> str:
    selected = resolve_llm_provider(provider)
    return {
        "openai": "OPENAI_API_KEY",
        "gemini": "GEMINI_API_KEY",
        "groq": "GROQ_API_KEY",
    }[selected]


def default_model_for(provider: str) -> str:
    selected = resolve_llm_provider(provider)
    if selected == "openai":
        return DEFAULT_OPENAI_MODEL
    if selected == "gemini":
        return DEFAULT_GEMINI_MODEL
    return DEFAULT_GROQ_MODEL


def build_llm_client(
    *,
    provider: str | None = None,
    api_key: str | None = None,
    model: str | None = None,
):
    selected = resolve_llm_provider(provider)
    selected_model = model or os.environ.get("R2B4_LLM_MODEL") or default_model_for(selected)
    if selected == "openai":
        return OpenAIResponsesChatClient(
            api_key=api_key,
            config=OpenAIChatConfig(model=selected_model),
        )
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
    "DEFAULT_OPENAI_MODEL",
    "SUPPORTED_LLM_PROVIDERS",
    "api_key_env_for",
    "build_llm_client",
    "default_model_for",
    "resolve_llm_provider",
]
