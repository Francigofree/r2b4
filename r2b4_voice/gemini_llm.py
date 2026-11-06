"""Gemini structured-output adapter for R2B4 conversation/AgentCore.

Plain one-turn Gemini text remains on the Interactions API via ``GeminiChatConfig.endpoint``.
Structured conversation/AgentCore turns use ``generateContent`` because Gemini 2.5 structured
output is enforced there with ``responseMimeType`` + ``responseJsonSchema``.
"""

from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Callable

from r2b4_orchestration.agent_contracts import (
    AgentModelReply,
    build_agent_step_schema,
    parse_agent_model_reply,
)
from v3.adapters.vision_media_contracts import VisionJpeg

from .conversation_contracts import LLMDecision
from .llm_decision import DECISION_SCHEMA, DecisionParseError, build_decision_schema, parse_llm_decision


class GeminiRequestError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class GeminiChatConfig:
    # Kept for PlainGeminiClient, which intentionally still uses Interactions.
    endpoint: str = "https://generativelanguage.googleapis.com/v1beta/interactions"
    structured_endpoint_template: str = (
        "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
    )
    model: str = "gemini-2.5-flash"
    timeout_s: float = 20.0
    thinking_level: str = "low"

    def __post_init__(self) -> None:
        if not self.endpoint.startswith("https://"):
            raise ValueError("Gemini endpoint must use https")
        if not self.structured_endpoint_template.startswith("https://"):
            raise ValueError("Gemini structured endpoint must use https")
        if "{model}" not in self.structured_endpoint_template:
            raise ValueError("Gemini structured endpoint must contain {model}")
        if not self.model.strip():
            raise ValueError("model must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if self.thinking_level not in {"low", "medium", "high"}:
            raise ValueError("thinking_level must be low, medium or high")


UrlOpen = Callable[..., object]


class GeminiStructuredChatClient:
    """Stateless Gemini client; R2B4 owns history, tools and robot authority."""

    def __init__(
        self,
        api_key: str | None = None,
        config: GeminiChatConfig | None = None,
        *,
        urlopen: UrlOpen = urllib.request.urlopen,
    ) -> None:
        key = api_key if api_key is not None else os.environ.get("GEMINI_API_KEY")
        if not isinstance(key, str) or not key.strip():
            raise GeminiRequestError("GEMINI_API_KEY is not configured")
        cfg = config or GeminiChatConfig(model=os.environ.get("R2B4_LLM_MODEL", "gemini-2.5-flash"))
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
        images: Sequence[VisionJpeg] = (),
    ) -> AgentModelReply:
        raw = self._structured(messages, build_agent_step_schema(tool_catalog, action_catalog), images=images)
        try:
            return parse_agent_model_reply(
                raw,
                model=self._config.model,
                tool_catalog=tool_catalog,
                action_catalog=action_catalog,
            )
        except (ValueError, DecisionParseError) as exc:
            raise GeminiRequestError(f"Gemini Agent step failed R2B4 schema validation: {exc}") from exc

    def _complete_decision(
        self,
        messages: Sequence[Mapping[str, str]],
        *,
        action_catalog: Sequence[Mapping[str, object]] | None,
    ) -> LLMDecision:
        schema = build_decision_schema(action_catalog) if action_catalog is not None else DECISION_SCHEMA
        raw = self._structured(messages, schema)
        try:
            return parse_llm_decision(raw, model=self._config.model, action_catalog=action_catalog)
        except DecisionParseError as exc:
            raise GeminiRequestError(f"Gemini decision failed R2B4 schema validation: {exc}") from exc

    def _structured(
        self,
        messages: Sequence[Mapping[str, str]],
        schema: Mapping[str, object],
        *,
        images: Sequence[VisionJpeg] = (),
    ) -> object:
        if not messages:
            raise ValueError("messages must not be empty")
        system_instruction, interaction_input = self._convert_messages(messages)
        generation_config: dict[str, object] = {
            "responseMimeType": "application/json",
            # Use the JSON-Schema surface rather than the older OpenAPI-shaped
            # responseSchema. R2B4 schemas use nullable type arrays.
            "responseJsonSchema": dict(schema),
            "thinkingConfig": self._thinking_config(),
        }
        body: dict[str, object] = {
            "contents": [{"role": "user", "parts": [
                {"text": interaction_input},
                *[
                    {"inlineData": {"mimeType": "image/jpeg", "data": base64.b64encode(image.image_bytes).decode("ascii")}}
                    for image in images
                ],
            ]}],
            "generationConfig": generation_config,
        }
        if system_instruction:
            body["systemInstruction"] = {"parts": [{"text": system_instruction}]}

        model = urllib.parse.quote(self._config.model.strip(), safe="")
        endpoint = self._config.structured_endpoint_template.format(model=model)
        request = urllib.request.Request(
            endpoint,
            data=json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            method="POST",
            headers={
                "x-goog-api-key": self._api_key,
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-agent-llm/2",
            },
        )
        try:
            response = self._urlopen(request, timeout=self._config.timeout_s)
            payload = response.read()  # type: ignore[attr-defined]
        except urllib.error.HTTPError as exc:
            detail = self._http_error_detail(exc)
            suffix = f": {detail}" if detail else ""
            raise GeminiRequestError(f"Gemini HTTP {exc.code}{suffix}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise GeminiRequestError(f"Gemini request failed: {type(exc).__name__}") from exc

        try:
            decoded = json.loads(payload.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise GeminiRequestError("Gemini generateContent returned a non-JSON response envelope") from exc

        try:
            content = self._extract_generate_content_text(decoded)
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise GeminiRequestError(f"Gemini generateContent response invalid: {exc}") from exc

        try:
            return json.loads(content)
        except json.JSONDecodeError as exc:
            preview = content[:240].replace("\n", "\\n")
            raise GeminiRequestError(
                f"Gemini structured output was not JSON: {exc.msg} at char {exc.pos}; "
                f"output_prefix={preview!r}"
            ) from exc

    def _thinking_config(self) -> dict[str, object]:
        """Translate the provider-neutral low/medium/high knob to model-family syntax."""
        model = self._config.model.strip().lower()
        level = self._config.thinking_level
        if model.startswith("gemini-2.5"):
            # Gemini 2.5 uses thinkingBudget, not thinkingLevel. 0 is the documented
            # thinking-off/lowest-latency setting; -1 asks for dynamic thinking.
            budget = {"low": 0, "medium": 1024, "high": -1}[level]
            return {"thinkingBudget": budget}
        return {"thinkingLevel": level}

    @staticmethod
    def _convert_messages(messages: Sequence[Mapping[str, str]]) -> tuple[str, str]:
        """Flatten local text history into one stateless Gemini request input."""
        systems: list[str] = []
        transcript: list[str] = []
        last_role: str | None = None
        for item in messages:
            role = item.get("role")
            content = item.get("content")
            if role not in {"system", "user", "assistant"} or not isinstance(content, str) or not content.strip():
                raise ValueError("messages must contain non-empty system/user/assistant text")
            text = content.strip()
            if role == "system":
                systems.append(text)
                continue
            label = "USER" if role == "user" else "ASSISTANT"
            transcript.append(f"{label}:\n{text}")
            last_role = role
        if not transcript or last_role != "user":
            raise ValueError("Gemini interaction must end with a user message")
        return "\n\n".join(systems), "\n\n".join(transcript)

    @staticmethod
    def _extract_generate_content_text(decoded: object) -> str:
        if not isinstance(decoded, Mapping):
            raise ValueError("response envelope is not an object")
        candidates = decoded.get("candidates")
        if not isinstance(candidates, list) or not candidates:
            prompt_feedback = decoded.get("promptFeedback")
            detail = json.dumps(prompt_feedback, ensure_ascii=False, sort_keys=True, default=str)[:500]
            raise ValueError(f"response has no candidates; promptFeedback={detail}")
        candidate = candidates[0]
        if not isinstance(candidate, Mapping):
            raise ValueError("first candidate is not an object")
        content = candidate.get("content")
        if not isinstance(content, Mapping):
            raise ValueError(
                f"candidate has no content; finishReason={candidate.get('finishReason')!r}"
            )
        parts = content.get("parts")
        if not isinstance(parts, list):
            raise ValueError("candidate content has no parts")
        chunks = [
            item.get("text")
            for item in parts
            if isinstance(item, Mapping)
            and item.get("thought") is not True
            and isinstance(item.get("text"), str)
        ]
        text = "".join(chunks).strip()
        if text:
            return text
        raise ValueError(
            f"candidate contains no non-thought text; finishReason={candidate.get('finishReason')!r}"
        )

    @staticmethod
    def _http_error_detail(exc: urllib.error.HTTPError) -> str:
        try:
            raw = exc.read().decode("utf-8", errors="replace")
            parsed = json.loads(raw)
            if isinstance(parsed, Mapping):
                error = parsed.get("error")
                if isinstance(error, Mapping):
                    return str(error.get("message", ""))[:300]
        except Exception:
            pass
        return ""


__all__ = ["GeminiChatConfig", "GeminiRequestError", "GeminiStructuredChatClient"]
