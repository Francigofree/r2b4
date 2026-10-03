from __future__ import annotations

from r2b4_voice.llm_failover import FailoverLLMClient, LLMProviderCandidate


class Client:
    def __init__(self, name: str, outcomes):
        self.model = name
        self._outcomes = list(outcomes)
        self.calls = 0

    def complete_text(self, _prompt: str):
        self.calls += 1
        value = self._outcomes.pop(0)
        if isinstance(value, Exception):
            raise value
        return value


class ProviderError(RuntimeError):
    def __init__(self, text: str, *, status_code=None, retryable=False, code=None):
        super().__init__(text)
        self.status_code = status_code
        self.retryable = retryable
        self.code = code


def test_quota_switches_provider_without_retry() -> None:
    primary = Client("openai", [ProviderError("quota", status_code=429)])
    fallback = Client("gemini", ["ok"])
    chain = FailoverLLMClient(
        [LLMProviderCandidate("openai_oauth", primary), LLMProviderCandidate("gemini", fallback)],
        sleep=lambda _seconds: None,
    )
    assert chain.complete_text("x") == "ok"
    assert primary.calls == 1
    assert fallback.calls == 1
    assert chain.last_provider == "gemini"


def test_one_transient_retry_then_success() -> None:
    primary = Client("openai", [ProviderError("temporary", status_code=503, retryable=True), "ok"])
    fallback = Client("gemini", ["unused"])
    chain = FailoverLLMClient(
        [LLMProviderCandidate("openai_oauth", primary), LLMProviderCandidate("gemini", fallback)],
        sleep=lambda _seconds: None,
    )
    assert chain.complete_text("x") == "ok"
    assert primary.calls == 2
    assert fallback.calls == 0


def test_invalid_response_switches_immediately() -> None:
    primary = Client("openai", [RuntimeError("schema validation failed")])
    fallback = Client("groq", ["ok"])
    chain = FailoverLLMClient(
        [LLMProviderCandidate("openai_oauth", primary), LLMProviderCandidate("groq", fallback)],
        sleep=lambda _seconds: None,
    )
    assert chain.complete_text("x") == "ok"
    assert primary.calls == 1
    assert fallback.calls == 1
