"""OpenAI Responses API adapter for the R2B4 provider-neutral Agent Core.

Host-side only. R2B4 owns conversation history, tool execution and every robot
authority boundary. This module only translates provider-neutral messages and
strict JSON Schemas to/from the OpenAI Responses API.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Callable

from r2b4_orchestration.agent_contracts import (
    AgentModelReply,
    build_agent_step_schema,
    parse_agent_model_reply,
)

from .conversation_contracts import LLMDecision
from .llm_decision import (
    DECISION_SCHEMA,
    DecisionParseError,
    build_decision_schema,
    parse_llm_decision,
)


class OpenAIRequestError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class OpenAIChatConfig:
    endpoint: str = "https://api.openai.com/v1/responses"
    model: str = "gpt-5.6"
    timeout_s: float = 30.0
    reasoning_effort: str = "low"

    def __post_init__(self) -> None:
        if not self.endpoint.startswith("https://"):
            raise ValueError("OpenAI endpoint must use https")
        if not self.model.strip():
            raise ValueError("model must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if self.reasoning_effort not in {"none", "low", "medium", "high", "xhigh", "max"}:
            raise ValueError("unsupported OpenAI reasoning effort")


UrlOpen = Callable[..., object]


def _validate_messages(messages: Sequence[Mapping[str, str]]) -> list[dict[str, str]]:
    if not messages:
        raise ValueError("messages must not be empty")
    out: list[dict[str, str]] = []
    last_non_system: str | None = None
    for item in messages:
        role = item.get("role")
        content = item.get("content")
        if role not in {"system", "user", "assistant"}:
            raise ValueError("messages contain an unsupported role")
        if not isinstance(content, str) or not content.strip():
            raise ValueError("messages must contain non-empty text")
        out.append({"role": role, "content": content.strip()})
        if role != "system":
            last_non_system = role
    if last_non_system != "user":
        raise ValueError("OpenAI interaction must end with a user message")
    return out


def _http_error_detail(exc: urllib.error.HTTPError) -> str:
    try:
        raw = exc.read().decode("utf-8", errors="replace")
        parsed = json.loads(raw)
        if isinstance(parsed, Mapping):
            error = parsed.get("error")
            if isinstance(error, Mapping):
                return str(error.get("message", ""))[:500]
    except Exception:
        pass
    return ""


def _extract_output_text(decoded: object) -> str:
    if not isinstance(decoded, Mapping):
        raise ValueError("response envelope is not an object")

    status = decoded.get("status")
    if status in {"failed", "cancelled", "incomplete"}:
        detail = decoded.get("error") or decoded.get("incomplete_details") or status
        raise ValueError(f"response status {status}: {detail}")

    direct = decoded.get("output_text")
    if isinstance(direct, str) and direct.strip():
        return direct.strip()

    output = decoded.get("output")
    if not isinstance(output, list):
        raise ValueError("response has no output array")

    chunks: list[str] = []
    refusals: list[str] = []
    for item in output:
        if not isinstance(item, Mapping) or item.get("type") != "message":
            continue
        content = item.get("content")
        if not isinstance(content, list):
            continue
        for part in content:
            if not isinstance(part, Mapping):
                continue
            part_type = part.get("type")
            if part_type == "output_text" and isinstance(part.get("text"), str):
                chunks.append(str(part["text"]))
            elif part_type == "refusal":
                refusal = part.get("refusal")
                if isinstance(refusal, str) and refusal.strip():
                    refusals.append(refusal.strip())

    text = "".join(chunks).strip()
    if text:
        return text
    if refusals:
        raise ValueError("response refused: " + " ".join(refusals)[:500])
    raise ValueError("response contains no output text")


def _post_response(
    *,
    api_key: str,
    config: OpenAIChatConfig,
    body: Mapping[str, object],
    urlopen: UrlOpen,
) -> object:
    request = urllib.request.Request(
        config.endpoint,
        data=json.dumps(dict(body), ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        method="POST",
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
            "Accept": "application/json",
            "User-Agent": "r2b4-openai-llm/1",
        },
    )
    try:
        response = urlopen(request, timeout=config.timeout_s)
        payload = response.read()  # type: ignore[attr-defined]
    except urllib.error.HTTPError as exc:
        detail = _http_error_detail(exc)
        suffix = f": {detail}" if detail else ""
        raise OpenAIRequestError(f"OpenAI HTTP {exc.code}{suffix}") from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise OpenAIRequestError(f"OpenAI request failed: {type(exc).__name__}") from exc

    try:
        return json.loads(payload.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise OpenAIRequestError("OpenAI Responses API returned a non-JSON envelope") from exc


class OpenAIResponsesChatClient:
    """Stateless structured client used by the existing R2B4 AgentCore."""

    def __init__(
        self,
        api_key: str | None = None,
        config: OpenAIChatConfig | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("OPENAI_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise OpenAIRequestError("OPENAI_API_KEY is not configured")
        cfg = config or OpenAIChatConfig(model=os.environ.get("R2B4_LLM_MODEL", "gpt-5.6"))
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
    ) -> AgentModelReply:
        raw = self._structured(
            messages,
            build_agent_step_schema(tool_catalog, action_catalog),
            "r2b4_agent_step",
        )
        try:
            return parse_agent_model_reply(
                raw,
                model=self._config.model,
                tool_catalog=tool_catalog,
                action_catalog=action_catalog,
            )
        except (ValueError, DecisionParseError) as exc:
            raise OpenAIRequestError(
                f"OpenAI Agent step failed R2B4 schema validation: {exc}"
            ) from exc

    def _complete_decision(
        self,
        messages: Sequence[Mapping[str, str]],
        *,
        action_catalog: Sequence[Mapping[str, object]] | None,
    ) -> LLMDecision:
        schema = build_decision_schema(action_catalog) if action_catalog is not None else DECISION_SCHEMA
        raw = self._structured(messages, schema, "r2b4_llm_decision")
        try:
            return parse_llm_decision(
                raw,
                model=self._config.model,
                action_catalog=action_catalog,
            )
        except DecisionParseError as exc:
            raise OpenAIRequestError(
                f"OpenAI decision failed R2B4 schema validation: {exc}"
            ) from exc

    def _structured(
        self,
        messages: Sequence[Mapping[str, str]],
        schema: Mapping[str, object],
        schema_name: str,
    ) -> object:
        body: dict[str, object] = {
            "model": self._config.model,
            "input": _validate_messages(messages),
            "store": False,
            "reasoning": {"effort": self._config.reasoning_effort},
            "text": {
                "format": {
                    "type": "json_schema",
                    "name": schema_name,
                    "strict": True,
                    "schema": dict(schema),
                }
            },
        }
        decoded = _post_response(
            api_key=self._api_key,
            config=self._config,
            body=body,
            urlopen=self._urlopen,
        )
        try:
            content = _extract_output_text(decoded)
        except ValueError as exc:
            raise OpenAIRequestError(f"OpenAI Responses API response invalid: {exc}") from exc
        try:
            return json.loads(content)
        except json.JSONDecodeError as exc:
            preview = content[:240].replace("\n", "\\n")
            raise OpenAIRequestError(
                f"OpenAI structured output was not JSON: {exc.msg} at char {exc.pos}; "
                f"output_prefix={preview!r}"
            ) from exc


class OpenAIPlainClient:
    """One-turn unstructured OpenAI Responses API client for the short ``r`` path."""

    def __init__(
        self,
        api_key: str | None = None,
        config: OpenAIChatConfig | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("OPENAI_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise OpenAIRequestError("OPENAI_API_KEY is not configured")
        cfg = config or OpenAIChatConfig(model=os.environ.get("R2B4_LLM_MODEL", "gpt-5.6"))
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
        body: dict[str, object] = {
            "model": self._config.model,
            "input": [{"role": "user", "content": prompt.strip()}],
            "store": False,
            "reasoning": {"effort": self._config.reasoning_effort},
        }
        decoded = _post_response(
            api_key=self._api_key,
            config=self._config,
            body=body,
            urlopen=self._urlopen,
        )
        try:
            return _extract_output_text(decoded)
        except ValueError as exc:
            raise OpenAIRequestError(f"OpenAI Responses API response invalid: {exc}") from exc


__all__ = [
    "OpenAIChatConfig",
    "OpenAIPlainClient",
    "OpenAIRequestError",
    "OpenAIResponsesChatClient",
]
