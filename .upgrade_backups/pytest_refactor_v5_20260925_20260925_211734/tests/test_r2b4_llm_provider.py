import pytest

from r2b4_voice.gemini_llm import GeminiStructuredChatClient
from r2b4_voice.groq_llm import GroqStructuredChatClient
from r2b4_voice.llm_provider import build_llm_client, default_model_for, resolve_llm_provider


def test_gemini_is_default_provider(monkeypatch):
    monkeypatch.delenv("R2B4_LLM_PROVIDER", raising=False)
    monkeypatch.delenv("R2B4_LLM_MODEL", raising=False)
    client = build_llm_client(api_key="secret")
    assert isinstance(client, GeminiStructuredChatClient)
    assert client.model == "gemini-3.8-flash"


def test_groq_remains_explicit_fallback(monkeypatch):
    monkeypatch.delenv("R2B4_LLM_MODEL", raising=False)
    client = build_llm_client(provider="groq", api_key="secret")
    assert isinstance(client, GroqStructuredChatClient)
    assert client.model == "openai/gpt-oss-20b"


def test_provider_validation():
    assert resolve_llm_provider("GEMINI") == "gemini"
    assert default_model_for("gemini") == "gemini-3.8-flash"
    with pytest.raises(ValueError):
        resolve_llm_provider("unknown")
