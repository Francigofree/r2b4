from __future__ import annotations

from r2b4_voice.llm_provider import (
    DEFAULT_LLM_PROVIDER,
    DEFAULT_OPENAI_MODEL,
    api_key_env_for,
    build_llm_client,
    default_model_for,
    resolve_llm_provider,
)
from r2b4_voice.openai_llm import OpenAIResponsesChatClient


def test_openai_is_default_provider(monkeypatch) -> None:
    monkeypatch.delenv("R2B4_LLM_PROVIDER", raising=False)
    assert DEFAULT_LLM_PROVIDER == "openai"
    assert resolve_llm_provider() == "openai"
    assert default_model_for("openai") == DEFAULT_OPENAI_MODEL == "gpt-5.6"
    assert api_key_env_for("openai") == "OPENAI_API_KEY"
    assert resolve_llm_provider("chatgpt") == "openai"


def test_openai_client_build_and_existing_fallbacks() -> None:
    client = build_llm_client(provider="openai", api_key="test-key", model="gpt-5.6")
    assert isinstance(client, OpenAIResponsesChatClient)
    assert client.model == "gpt-5.6"
    assert default_model_for("gemini") == "gemini-2.5-flash"
    assert default_model_for("groq") == "openai/gpt-oss-20b"
    assert api_key_env_for("gemini") == "GEMINI_API_KEY"
    assert api_key_env_for("groq") == "GROQ_API_KEY"
