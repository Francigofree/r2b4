"""OpenAI Responses API adapters for R2B4, including ChatGPT-plan OAuth.

Host-side only. R2B4 owns conversation history, tool execution and every robot
authority boundary. This module translates provider-neutral messages and strict
JSON Schemas to/from the public Responses API.

For Sign in with ChatGPT plan usage the public flow requires store=false and
stream=true. The same streaming transport is used for API-key fallback so both
authentication paths exercise one response parser.
"""
from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.request
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Callable, Protocol

from r2b4_orchestration.agent_contracts import (
    AgentModelReply,
    build_agent_step_schema,
    parse_agent_model_reply,
)
from v3.adapters.vision_media_contracts import VisionJpeg

from .conversation_contracts import LLMDecision
from .llm_decision import (
    DECISION_SCHEMA,
    DecisionParseError,
    build_decision_schema,
    parse_llm_decision,
)


class OpenAIRequestError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        status_code: int | None = None,
        code: str | None = None,
        retryable: bool = False,
    ) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.code = code
        self.retryable = bool(retryable)


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


class AccessTokenProvider(Protocol):
    auth_mode: str

    def get_access_token(self, *, force_refresh: bool = False) -> str: ...


class _FixedTokenProvider:
    auth_mode = "api_key"

    def __init__(self, token: str) -> None:
        if not isinstance(token, str) or not token.strip():
            raise OpenAIRequestError("OPENAI_API_KEY is not configured")
        self._token = token.strip()

    def get_access_token(self, *, force_refresh: bool = False) -> str:
        _ = force_refresh
        return self._token


UrlOpen = Callable[..., object]


def _prepare_messages(messages: Sequence[Mapping[str, str]]) -> tuple[str | None, list[dict[str, str]]]:
    """Move system blocks to Responses `instructions` for SIWC compatibility."""
    if not messages:
        raise ValueError("messages must not be empty")
    instructions: list[str] = []
    input_messages: list[dict[str, str]] = []
    last_non_system: str | None = None
    for item in messages:
        role = item.get("role")
        content = item.get("content")
        if role not in {"system", "user", "assistant"}:
            raise ValueError("messages contain an unsupported role")
        if not isinstance(content, str) or not content.strip():
            raise ValueError("messages must contain non-empty text")
        content = content.strip()
        if role == "system":
            instructions.append(content)
        else:
            input_messages.append({"role": role, "content": content})
            last_non_system = role
    if last_non_system != "user":
        raise ValueError("OpenAI interaction must end with a user message")
    return ("\n\n".join(instructions) if instructions else None, input_messages)


def _error_fields(value: object) -> tuple[str | None, str | None]:
    """Extract a stable-ish code/message without assuming one response shape."""
    if not isinstance(value, Mapping):
        return None, None
    candidates: list[Mapping[str, object]] = [value]
    for key in ("error", "response", "incomplete_details"):
        child = value.get(key)
        if isinstance(child, Mapping):
            candidates.append(child)
            nested = child.get("error")
            if isinstance(nested, Mapping):
                candidates.append(nested)
    code: str | None = None
    message: str | None = None
    for item in candidates:
        if code is None:
            raw = item.get("code") or item.get("type")
            if isinstance(raw, str) and raw not in {"error", "response.failed", "response.incomplete"}:
                code = raw
        if message is None:
            raw = item.get("message") or item.get("detail") or item.get("reason")
            if isinstance(raw, str) and raw.strip():
                message = raw.strip()
    return code, message


def _retryable_http(status: int, code: str | None) -> bool:
    normalized = (code or "").lower()
    if normalized in {
        "subscription_sharing_usage_limit_exceeded",
        "subscription_sharing_usage_unavailable",
    }:
        return False
    return status in {408, 409, 425, 500, 502, 503, 504}


def _http_error(exc: urllib.error.HTTPError) -> OpenAIRequestError:
    try:
        raw = exc.read().decode("utf-8", errors="replace")
    except Exception:
        raw = ""
    parsed: object = None
    try:
        parsed = json.loads(raw) if raw else None
    except json.JSONDecodeError:
        parsed = None
    code, message = _error_fields(parsed)
    if message is None and raw:
        message = raw[:500]
    suffix = f": {message}" if message else ""
    return OpenAIRequestError(
        f"OpenAI HTTP {exc.code}{suffix}",
        status_code=exc.code,
        code=code,
        retryable=_retryable_http(exc.code, code),
    )


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
            if part.get("type") == "output_text" and isinstance(part.get("text"), str):
                chunks.append(str(part["text"]))
            elif part.get("type") == "refusal" and isinstance(part.get("refusal"), str):
                refusals.append(str(part["refusal"]).strip())
    text = "".join(chunks).strip()
    if text:
        return text
    if refusals:
        raise ValueError("response refused: " + " ".join(refusals)[:500])
    raise ValueError("response contains no output text")


def _iter_sse_json(response: object):
    """Yield JSON data records from a text/event-stream HTTP response."""
    data_lines: list[str] = []
    try:
        iterator = iter(response)  # HTTPResponse is line-iterable.
    except TypeError as exc:
        raise OpenAIRequestError("OpenAI streaming response is not iterable", retryable=True) from exc
    for raw in iterator:
        if isinstance(raw, bytes):
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        else:
            line = str(raw).rstrip("\r\n")
        if not line:
            if data_lines:
                joined = "\n".join(data_lines).strip()
                data_lines.clear()
                if joined and joined != "[DONE]":
                    try:
                        value = json.loads(joined)
                    except json.JSONDecodeError as exc:
                        raise OpenAIRequestError("OpenAI SSE event contains invalid JSON", retryable=True) from exc
                    if isinstance(value, Mapping):
                        yield value
            continue
        if line.startswith(":") or line.startswith("event:") or line.startswith("id:") or line.startswith("retry:"):
            continue
        if line.startswith("data:"):
            data_lines.append(line[5:].lstrip())
    if data_lines:
        joined = "\n".join(data_lines).strip()
        if joined and joined != "[DONE]":
            try:
                value = json.loads(joined)
            except json.JSONDecodeError as exc:
                raise OpenAIRequestError("OpenAI SSE tail contains invalid JSON", retryable=True) from exc
            if isinstance(value, Mapping):
                yield value


def _consume_response_stream(response: object, metadata: dict | None = None) -> str:
    chunks: list[str] = []
    completed_response: Mapping[str, object] | None = None
    completed = False
    for event in _iter_sse_json(response):
        event_type = event.get("type")
        if event_type == "response.output_text.delta":
            delta = event.get("delta")
            if isinstance(delta, str):
                chunks.append(delta)
            continue
        if event_type == "response.completed":
            completed = True
            candidate = event.get("response")
            if isinstance(candidate, Mapping):
                completed_response = candidate
                if metadata is not None:
                    for source, target in (("id", "response_id"), ("model", "actual_model")):
                        value = candidate.get(source)
                        if isinstance(value, str):
                            metadata[target] = value[:256]
                    usage = candidate.get("usage")
                    if isinstance(usage, Mapping):
                        for key in ("input_tokens", "output_tokens"):
                            if type(usage.get(key)) is int:
                                metadata[key] = usage[key]
            continue
        if event_type in {"response.failed", "response.incomplete", "error"}:
            code, message = _error_fields(event)
            suffix = f": {message}" if message else ""
            retryable = code not in {
                "subscription_sharing_usage_limit_exceeded",
                "subscription_sharing_usage_unavailable",
            }
            raise OpenAIRequestError(
                f"OpenAI stream {event_type}{suffix}",
                code=code,
                retryable=retryable,
            )
    if not completed:
        raise OpenAIRequestError("OpenAI stream ended before response.completed", retryable=True)
    text = "".join(chunks).strip()
    if text:
        return text
    if completed_response is not None:
        try:
            return _extract_output_text(completed_response)
        except ValueError as exc:
            raise OpenAIRequestError(f"OpenAI completed response invalid: {exc}") from exc
    raise OpenAIRequestError("OpenAI completed stream contained no output text")


class _OpenAITransport:
    def __init__(
        self,
        *,
        api_key: str | None,
        token_provider: AccessTokenProvider | None,
        config: OpenAIChatConfig,
        urlopen: UrlOpen,
    ) -> None:
        if token_provider is not None and api_key is not None:
            raise ValueError("use either token_provider or api_key, not both")
        if token_provider is None:
            key = api_key if api_key is not None else os.environ.get("OPENAI_API_KEY")
            token_provider = _FixedTokenProvider(key or "")
        if not callable(getattr(token_provider, "get_access_token", None)):
            raise TypeError("token_provider must provide get_access_token()")
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._auth = token_provider
        self._config = config
        self._urlopen = urlopen
        self._resolved_model: str | None = None
        self.last_request_metadata: dict[str, object] = {}

    @property
    def auth_mode(self) -> str:
        return str(getattr(self._auth, "auth_mode", "bearer"))

    def resolve_model(self, preferred: str) -> str:
        """Use the signed-in ChatGPT account's visible model catalog when possible.

        OpenAI documents the OAuth model catalog as account-specific.  We keep
        the configured model when it is available; otherwise we use the first
        visible slug in the server-provided order.  Catalog lookup failure is
        non-fatal: inference still gets a chance with the configured model.
        """
        if self.auth_mode != "oauth":
            return preferred
        if self._resolved_model:
            return self._resolved_model
        try:
            token = self._auth.get_access_token(force_refresh=False)
            request = urllib.request.Request(
                "https://api.openai.com/v1/models",
                method="GET",
                headers={
                    "Authorization": f"Bearer {token}",
                    "Accept": "application/json",
                    "User-Agent": "r2b4-openai-llm/3",
                },
            )
            response = self._urlopen(request, timeout=self._config.timeout_s)
            raw = response.read()  # type: ignore[attr-defined]
            decoded = json.loads(raw.decode("utf-8"))
            rows = decoded.get("models") if isinstance(decoded, Mapping) else None
            visible = [
                str(item.get("slug"))
                for item in rows
                if isinstance(item, Mapping)
                and item.get("visibility") == "list"
                and isinstance(item.get("slug"), str)
                and str(item.get("slug")).strip()
            ] if isinstance(rows, list) else []
            if preferred in visible:
                self._resolved_model = preferred
            elif visible:
                self._resolved_model = visible[0]
        except Exception:
            # Model catalog is advisory for robustness.  The Responses request
            # itself remains the source of truth and participates in failover.
            self._resolved_model = None
        return self._resolved_model or preferred

    def post(self, body: Mapping[str, object]) -> str:
        # One forced token renewal on OAuth 401. Other retry decisions belong to
        # the provider-level failover policy, not this transport.
        self.last_request_metadata = {"actual_model": body.get("model")}
        for auth_attempt in range(2):
            self.last_request_metadata["transport_attempt_count"] = auth_attempt + 1
            force = auth_attempt == 1
            try:
                token = self._auth.get_access_token(force_refresh=force)
            except Exception as exc:
                if isinstance(exc, OpenAIRequestError):
                    raise
                raise OpenAIRequestError(f"OpenAI {self.auth_mode} authentication failed: {type(exc).__name__}: {exc}") from exc
            request = urllib.request.Request(
                self._config.endpoint,
                data=json.dumps(dict(body), ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                method="POST",
                headers={
                    "Authorization": f"Bearer {token}",
                    "Content-Type": "application/json",
                    "Accept": "text/event-stream",
                    "User-Agent": "r2b4-openai-llm/3",
                },
            )
            try:
                response = self._urlopen(request, timeout=self._config.timeout_s)
                headers = getattr(response, "headers", {})
                request_id = headers.get("x-request-id")
                if isinstance(request_id, str):
                    self.last_request_metadata["provider_request_id"] = request_id[:256]
                return _consume_response_stream(response, self.last_request_metadata)
            except urllib.error.HTTPError as exc:
                error = _http_error(exc)
                if error.status_code == 401 and self.auth_mode == "oauth" and auth_attempt == 0:
                    continue
                raise error from exc
            except OpenAIRequestError:
                raise
            except (urllib.error.URLError, TimeoutError, OSError) as exc:
                raise OpenAIRequestError(
                    f"OpenAI request failed: {type(exc).__name__}",
                    retryable=True,
                ) from exc
        raise OpenAIRequestError("OpenAI OAuth authentication remained unauthorized after token refresh", status_code=401)


def _base_body(config: OpenAIChatConfig, messages: Sequence[Mapping[str, str]]) -> dict[str, object]:
    instructions, input_messages = _prepare_messages(messages)
    body: dict[str, object] = {
        "model": config.model,
        "input": input_messages,
        "store": False,
        "stream": True,
        "reasoning": {"effort": config.reasoning_effort},
    }
    if instructions:
        body["instructions"] = instructions
    return body


class OpenAIResponsesChatClient:
    """Stateless structured client used by the existing R2B4 AgentCore."""

    def __init__(
        self,
        api_key: str | None = None,
        config: OpenAIChatConfig | None = None,
        *,
        token_provider: AccessTokenProvider | None = None,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        cfg = config or OpenAIChatConfig(model=os.environ.get("R2B4_OPENAI_MODEL") or os.environ.get("R2B4_LLM_MODEL", "gpt-5.6"))
        self._config = cfg
        self._transport = _OpenAITransport(api_key=api_key, token_provider=token_provider, config=cfg, urlopen=urlopen)

    @property
    def model(self) -> str:
        return self._config.model

    @property
    def auth_mode(self) -> str:
        return self._transport.auth_mode

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
        images: Sequence[VisionJpeg] = (),
    ) -> AgentModelReply:
        raw, used_model = self._structured(messages, build_agent_step_schema(tool_catalog, action_catalog), "r2b4_agent_step", images=images)
        try:
            reply = parse_agent_model_reply(
                raw,
                model=used_model,
                tool_catalog=tool_catalog,
                action_catalog=action_catalog,
            )
            return replace(reply, model=str(self._transport.last_request_metadata.get("actual_model") or used_model),
                           inference_metadata=tuple(self._transport.last_request_metadata.items()))
        except (ValueError, DecisionParseError) as exc:
            raise OpenAIRequestError(f"OpenAI Agent step failed R2B4 schema validation: {exc}") from exc

    def _complete_decision(
        self,
        messages: Sequence[Mapping[str, str]],
        *,
        action_catalog: Sequence[Mapping[str, object]] | None,
    ) -> LLMDecision:
        schema = build_decision_schema(action_catalog) if action_catalog is not None else DECISION_SCHEMA
        raw, used_model = self._structured(messages, schema, "r2b4_llm_decision")
        try:
            return parse_llm_decision(raw, model=used_model, action_catalog=action_catalog)
        except DecisionParseError as exc:
            raise OpenAIRequestError(f"OpenAI decision failed R2B4 schema validation: {exc}") from exc

    def _structured(
        self,
        messages: Sequence[Mapping[str, str]],
        schema: Mapping[str, object],
        schema_name: str,
        *,
        images: Sequence[VisionJpeg] = (),
    ) -> tuple[object, str]:
        body = _base_body(self._config, messages)
        if images:
            inputs = body["input"]
            last = inputs[-1]
            last["content"] = [
                {"type": "input_text", "text": last["content"]},
                *[
                    {
                        "type": "input_image",
                        "image_url": "data:image/jpeg;base64," + base64.b64encode(image.image_bytes).decode("ascii"),
                    }
                    for image in images
                ],
            ]
        used_model = self._transport.resolve_model(self._config.model)
        body["model"] = used_model
        body["text"] = {
            "format": {
                "type": "json_schema",
                "name": schema_name,
                "strict": True,
                "schema": dict(schema),
            }
        }
        content = self._transport.post(body)
        try:
            return json.loads(content), used_model
        except json.JSONDecodeError as exc:
            preview = content[:240].replace("\n", "\\n")
            raise OpenAIRequestError(
                f"OpenAI structured output was not JSON: {exc.msg} at char {exc.pos}; output_prefix={preview!r}"
            ) from exc


class OpenAIPlainClient:
    """One-turn unstructured OpenAI Responses API client for the short ``r`` path."""

    def __init__(
        self,
        api_key: str | None = None,
        config: OpenAIChatConfig | None = None,
        *,
        token_provider: AccessTokenProvider | None = None,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        cfg = config or OpenAIChatConfig(model=os.environ.get("R2B4_OPENAI_MODEL") or os.environ.get("R2B4_LLM_MODEL", "gpt-5.6"))
        self._config = cfg
        self._transport = _OpenAITransport(api_key=api_key, token_provider=token_provider, config=cfg, urlopen=urlopen)

    @property
    def model(self) -> str:
        return self._config.model

    @property
    def auth_mode(self) -> str:
        return self._transport.auth_mode

    def complete_text(self, prompt: str) -> str:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt must be non-empty text")
        used_model = self._transport.resolve_model(self._config.model)
        body: dict[str, object] = {
            "model": used_model,
            "input": [{"role": "user", "content": prompt.strip()}],
            "store": False,
            "stream": True,
            "reasoning": {"effort": self._config.reasoning_effort},
        }
        return self._transport.post(body).strip()


__all__ = [
    "AccessTokenProvider",
    "OpenAIChatConfig",
    "OpenAIPlainClient",
    "OpenAIRequestError",
    "OpenAIResponsesChatClient",
]
