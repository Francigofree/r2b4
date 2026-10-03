from __future__ import annotations

from r2b4_voice.llm_provider import DEFAULT_LLM_PROVIDER, DEFAULT_OPENAI_MODEL, default_model_for, resolve_llm_provider


def test_openai_is_primary_and_alias_is_supported(monkeypatch) -> None:
    monkeypatch.delenv("R2B4_LLM_PROVIDER", raising=False)
    assert DEFAULT_LLM_PROVIDER == "openai"
    assert DEFAULT_OPENAI_MODEL == "gpt-5.6"
    assert resolve_llm_provider() == "openai"
    assert resolve_llm_provider("chatgpt") == "openai"
    assert default_model_for("gemini") == "gemini-2.5-flash"
    assert default_model_for("groq") == "openai/gpt-oss-20b"
