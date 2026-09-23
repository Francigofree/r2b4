"""Gemini Interactions API structured-output adapter for R2B4 conversation."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Callable

from .conversation_contracts import LLMDecision
from .llm_decision import DECISION_SCHEMA, DecisionParseError, build_decision_schema, parse_llm_decision


class GeminiRequestError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class GeminiChatConfig:
    endpoint: str = "https://generativelanguage.googleapis.com/v1beta/interactions"
    model: str = "gemini-3.8-flash"
    timeout_s: float = 20.0
    thinking_level: str = "low"

    def __post_init__(self) -> None:
        if not self.endpoint.startswith("https://"):
            raise ValueError("Gemini endpoint must use https")
        if not self.model.strip():
            raise ValueError("model must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if self.thinking_level not in {"low", "medium", "high"}:
            raise ValueError("thinking_level must be low, medium or high")


UrlOpen = Callable[..., object]


class GeminiStructuredChatClient:
    """Stateless Gemini client; R2B4 remains owner of conversation history."""

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
        cfg = config or GeminiChatConfig(model=os.environ.get("R2B4_LLM_MODEL", "gemini-3.8-flash"))
        if not callable(urlopen):
            raise TypeError("urlopen must be callable")
        self._api_key = key.strip()
        self._config = cfg
        self._urlopen = urlopen

    @property
    def model(self) -> str:
        return self._config.model

    def complete(self, messages: Sequence[Mapping[str, str]]) -> LLMDecision:
        return self._complete(messages, action_catalog=None)

    def complete_with_actions(
        self,
        messages: Sequence[Mapping[str, str]],
        action_catalog: Sequence[Mapping[str, object]],
    ) -> LLMDecision:
        return self._complete(messages, action_catalog=action_catalog)

    def _complete(
        self,
        messages: Sequence[Mapping[str, str]],
        *,
        action_catalog: Sequence[Mapping[str, object]] | None,
    ) -> LLMDecision:
        if not messages:
            raise ValueError("messages must not be empty")
        system_instruction, interaction_input = self._convert_messages(messages)
        body: dict[str, object] = {
            "model": self._config.model,
            "input": interaction_input,
            "store": False,
            "response_format": {
                "type": "text",
                "mime_type": "application/json",
                "schema": build_decision_schema(action_catalog) if action_catalog is not None else DECISION_SCHEMA,
            },
            "generation_config": {"thinking_level": self._config.thinking_level},
        }
        if system_instruction:
            body["system_instruction"] = system_instruction
        request = urllib.request.Request(
            self._config.endpoint,
            data=json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            method="POST",
            headers={
                "x-goog-api-key": self._api_key,
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-voice-llm/2",
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
            content = self._extract_output_text(decoded)
            decision_raw = json.loads(content)
        except (UnicodeError, json.JSONDecodeError, KeyError, IndexError, TypeError, ValueError) as exc:
            raise GeminiRequestError("Gemini returned an invalid structured response") from exc
        try:
            return parse_llm_decision(decision_raw, model=self._config.model, action_catalog=action_catalog)
        except DecisionParseError as exc:
            raise GeminiRequestError(str(exc)) from exc

    @staticmethod
    def _convert_messages(messages: Sequence[Mapping[str, str]]) -> tuple[str, str]:
        """Flatten local text history into one stateless Gemini input.

        Interactions ``store=false`` requires exact model-generated steps when
        replaying native Interaction history, including thought signatures.
        R2B4 intentionally owns only bounded text history, so we do not forge
        model_output steps. Instead the local text history is supplied as one
        explicit transcript in the current user input.
        """
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
    def _extract_output_text(decoded: object) -> str:
        if not isinstance(decoded, Mapping):
            raise ValueError("response is not an object")
        steps = decoded.get("steps")
        if not isinstance(steps, list):
            raise ValueError("response has no steps")
        for step in reversed(steps):
            if not isinstance(step, Mapping) or step.get("type") != "model_output":
                continue
            content = step.get("content")
            if not isinstance(content, list):
                continue
            chunks = [
                item.get("text")
                for item in content
                if isinstance(item, Mapping) and item.get("type") == "text" and isinstance(item.get("text"), str)
            ]
            text = "".join(chunks).strip()
            if text:
                return text
        raise ValueError("response contains no model text output")

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
