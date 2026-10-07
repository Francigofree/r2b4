"""Groq structured-output adapter for R2B4 conversation/AgentCore."""

from __future__ import annotations

import json
import math
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, replace
from typing import Callable, Mapping, Sequence

from r2b4_orchestration.agent_contracts import (
    AgentModelReply,
    build_agent_step_schema,
    parse_agent_model_reply,
)

from .conversation_contracts import LLMDecision
from .llm_decision import DECISION_SCHEMA, DecisionParseError, build_decision_schema, parse_llm_decision
from .prompting import PromptBudgetError


class LLMRequestError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class GroqChatConfig:
    endpoint: str = "https://api.groq.com/openai/v1/chat/completions"
    model: str = "openai/gpt-oss-20b"
    timeout_s: float = 20.0
    # Per-request admission, not the account's remaining minute quota. The
    # default matches the observed deployment's 8000 TPM tier and may be set
    # independently for another model/account through llm_provider.
    request_token_budget: int = 8000
    max_completion_tokens: int = 1024
    estimated_bytes_per_token: float = 3.0

    def __post_init__(self) -> None:
        if not self.endpoint.startswith("https://"):
            raise ValueError("Groq LLM endpoint must use https")
        if not self.model.strip():
            raise ValueError("model must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        for name in ("request_token_budget", "max_completion_tokens"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.max_completion_tokens + 128 >= self.request_token_budget:
            raise ValueError("Groq completion budget leaves no prompt budget")
        if (not isinstance(self.estimated_bytes_per_token, (int, float))
                or isinstance(self.estimated_bytes_per_token, bool)
                or not math.isfinite(self.estimated_bytes_per_token)
                or not 1.0 <= self.estimated_bytes_per_token <= 4.0):
            raise ValueError("estimated_bytes_per_token must be within [1, 4]")


UrlOpen = Callable[..., object]


class GroqStructuredChatClient:
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
        cfg = config or GroqChatConfig(model=os.environ.get("R2B4_LLM_MODEL", "openai/gpt-oss-20b"))
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._api_key = key.strip()
        self._config = cfg
        self._urlopen = urlopen

    @property
    def model(self) -> str:
        return self._config.model

    def complete(self, messages: Sequence[Mapping[str, str]]) -> LLMDecision:
        return self._complete_decision(messages, action_catalog=None)

    def complete_with_actions(
        self,
        messages: Sequence[Mapping[str, str]],
        action_catalog: Sequence[Mapping[str, object]],
    ) -> LLMDecision:
        return self._complete_decision(messages, action_catalog=action_catalog)

    def complete_agent_step(
        self,
        messages: Sequence[Mapping[str, str]],
        tool_catalog: Sequence[Mapping[str, object]],
        action_catalog: Sequence[Mapping[str, object]],
        *,
        images: Sequence[object] = (),
    ) -> AgentModelReply:
        if images:
            raise LLMRequestError("Groq text model cannot consume canonical image attachments")
        raw, size = self._structured(messages, build_agent_step_schema(tool_catalog, action_catalog), "r2b4_agent_step")
        try:
            reply = parse_agent_model_reply(
                raw,
                model=self._config.model,
                tool_catalog=tool_catalog,
                action_catalog=action_catalog,
            )
            return replace(reply, inference_metadata=tuple(size.items()))
        except (ValueError, DecisionParseError) as exc:
            raise LLMRequestError(str(exc)) from exc

    def _complete_decision(
        self,
        messages: Sequence[Mapping[str, str]],
        *,
        action_catalog: Sequence[Mapping[str, object]] | None,
    ) -> LLMDecision:
        schema = build_decision_schema(action_catalog) if action_catalog is not None else DECISION_SCHEMA
        raw, _size = self._structured(messages, schema, "r2b4_llm_decision")
        try:
            return parse_llm_decision(raw, model=self._config.model, action_catalog=action_catalog)
        except DecisionParseError as exc:
            raise LLMRequestError(str(exc)) from exc

    def _structured(
        self,
        messages: Sequence[Mapping[str, str]],
        schema: Mapping[str, object],
        schema_name: str,
    ) -> tuple[object, dict[str, object]]:
        if not messages:
            raise ValueError("messages must not be empty")
        body = json.dumps(
            {
                "model": self._config.model,
                "max_completion_tokens": self._config.max_completion_tokens,
                "messages": [dict(item) for item in messages],
                "response_format": {
                    "type": "json_schema",
                    "json_schema": {"name": schema_name, "strict": True, "schema": dict(schema)},
                },
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        # No model tokenizer is installed on the robot. The configured UTF-8
        # bytes/token ratio estimates the compact text+JSON prompt; the default
        # is below the 3.2..3.7 ratio measured on the failing deployment requests.
        # not an exact token count or a guarantee about externally shared TPM.
        # Preserve the full current user request, constraints, schema and tool
        # results: refuse oversized requests instead of truncating their meaning.
        token_estimate = (math.ceil(len(body) / self._config.estimated_bytes_per_token)
                          + 128 + self._config.max_completion_tokens)
        if token_estimate > self._config.request_token_budget:
            raise PromptBudgetError(
                f"GROQ_REQUEST_TOKEN_BUDGET_EXCEEDED:{token_estimate}>{self._config.request_token_budget}"
            )
        request = urllib.request.Request(
            self._config.endpoint,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-agent-llm/1",
            },
        )
        try:
            response = self._urlopen(request, timeout=self._config.timeout_s)
            payload = response.read()  # type: ignore[attr-defined]
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                raw = exc.read().decode("utf-8", errors="replace")
                parsed = json.loads(raw)
                detail = str(parsed.get("error", {}).get("message", ""))[:300]
            except Exception:
                detail = ""
            suffix = f": {detail}" if detail else ""
            raise LLMRequestError(f"Groq LLM HTTP {exc.code}{suffix}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise LLMRequestError(f"Groq LLM request failed: {type(exc).__name__}") from exc

        try:
            decoded = json.loads(payload.decode("utf-8"))
            content = decoded["choices"][0]["message"]["content"]
            usage = decoded.get("usage")
            size: dict[str, object] = {
                "provider_request_utf8_bytes": len(body),
                "provider_request_token_estimate": token_estimate,
                "provider_request_token_budget": self._config.request_token_budget,
                "max_completion_tokens": self._config.max_completion_tokens,
                "token_estimate_method": f"utf8_bytes/{self._config.estimated_bytes_per_token:g}+128+max_completion_tokens",
            }
            if isinstance(usage, Mapping):
                for wire, name in (("prompt_tokens", "input_tokens"), ("completion_tokens", "output_tokens")):
                    value = usage.get(wire)
                    if isinstance(value, int) and not isinstance(value, bool):
                        size[name] = value
            return json.loads(content), size
        except (UnicodeError, json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            raise LLMRequestError("Groq LLM returned an invalid structured response") from exc


__all__ = ["DECISION_SCHEMA", "GroqChatConfig", "GroqStructuredChatClient", "LLMRequestError"]
