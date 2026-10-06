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


def test_agent_attempt_evidence_counts_failover_without_repeating_tools():
    from r2b4_orchestration.agent_contracts import AgentModelReply
    class Model(Client):
        def complete_agent_step(self, *_args):
            return self.complete_text("unused")
    primary = Model("configured", [ProviderError("temporary", status_code=503, retryable=True),
                                  ProviderError("quota", status_code=429)])
    fallback = Model("fallback", [AgentModelReply("actual-fallback", spoken_text="Kész.")])
    client = FailoverLLMClient([LLMProviderCandidate("primary", primary), LLMProviderCandidate("fallback", fallback)],
                              sleep=lambda _: None)
    reply = client.complete_agent_step([], (), ())
    assert reply.model == "actual-fallback"
    assert dict(reply.inference_metadata) == {"provider": "fallback", "attempt_count": 3}
    assert primary.calls == 2 and fallback.calls == 1


def test_llm_failover_quota_bounded_retry_and_invalid_response_contract() -> None:
    scenarios = (
        ([ProviderError("quota", status_code=429)], "gemini", ["ok"], 1, 1, "gemini"),
        ([ProviderError("temporary", status_code=503, retryable=True), "ok"], "gemini", ["unused"], 2, 0, "openai_oauth"),
        ([RuntimeError("schema validation failed")], "groq", ["ok"], 1, 1, "groq"),
    )
    for primary_outcomes, fallback_name, fallback_outcomes, primary_calls, fallback_calls, final_provider in scenarios:
        primary = Client("openai", primary_outcomes)
        fallback = Client(fallback_name, fallback_outcomes)
        chain = FailoverLLMClient(
            [LLMProviderCandidate("openai_oauth", primary), LLMProviderCandidate(fallback_name, fallback)],
            sleep=lambda _seconds: None,
        )
        assert chain.complete_text("x") == "ok"
        assert primary.calls == primary_calls
        assert fallback.calls == fallback_calls
        assert chain.last_provider == final_provider
