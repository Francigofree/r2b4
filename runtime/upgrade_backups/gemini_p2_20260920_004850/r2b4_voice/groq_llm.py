"""Groq structured-output LLM adapter for R2B4 conversation decisions."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

from .conversation_contracts import LLMDecision, RobotAction


class LLMRequestError(RuntimeError):
    pass


DECISION_SCHEMA: dict[str, object] = {
    "type": "object",
    "properties": {
        "spoken_text": {"type": ["string", "null"]},
        "action_name": {
            "type": ["string", "null"],
            "enum": [
                None,
                "v3.command.stop",
                "v3.command.face_person",
                "v3.command.follow_person",
            ],
        },
        "action_parameters": {
            "type": "object",
            "properties": {
                "max_v_mps": {"type": ["number", "null"]},
                "max_omega_rad_s": {"type": ["number", "null"]},
            },
            "required": ["max_v_mps", "max_omega_rad_s"],
            "additionalProperties": False,
        },
    },
    "required": ["spoken_text", "action_name", "action_parameters"],
    "additionalProperties": False,
}


@dataclass(frozen=True, slots=True)
class GroqChatConfig:
    endpoint: str = "https://api.groq.com/openai/v1/chat/completions"
    model: str = "openai/gpt-oss-20b"
    timeout_s: float = 20.0

    def __post_init__(self) -> None:
        if not self.endpoint.startswith("https://"):
            raise ValueError("Groq LLM endpoint must use https")
        if not self.model.strip():
            raise ValueError("model must be non-empty")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")


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
        if not messages:
            raise ValueError("messages must not be empty")
        body = json.dumps(
            {
                "model": self._config.model,
                "messages": [dict(item) for item in messages],
                "response_format": {
                    "type": "json_schema",
                    "json_schema": {
                        "name": "r2b4_llm_decision",
                        "strict": True,
                        "schema": DECISION_SCHEMA,
                    },
                },
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        request = urllib.request.Request(
            self._config.endpoint,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Bearer {self._api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "r2b4-voice-llm/1",
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
            decision_raw = json.loads(content)
        except (UnicodeError, json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            raise LLMRequestError("Groq LLM returned an invalid structured response") from exc
        return self._parse_decision(decision_raw)

    def _parse_decision(self, raw: object) -> LLMDecision:
        if not isinstance(raw, Mapping):
            raise LLMRequestError("LLM decision is not an object")
        allowed_keys = {"spoken_text", "action_name", "action_parameters"}
        if set(raw) != allowed_keys:
            raise LLMRequestError("LLM decision has unexpected fields")
        spoken_text = raw.get("spoken_text")
        if spoken_text is not None and not isinstance(spoken_text, str):
            raise LLMRequestError("spoken_text must be string or null")
        action_name = raw.get("action_name")
        if action_name is not None and not isinstance(action_name, str):
            raise LLMRequestError("action_name must be string or null")
        params_raw = raw.get("action_parameters")
        if not isinstance(params_raw, Mapping):
            raise LLMRequestError("action_parameters must be an object")
        if set(params_raw) != {"max_v_mps", "max_omega_rad_s"}:
            raise LLMRequestError("action_parameters has unexpected fields")

        action: RobotAction | None = None
        if action_name is not None:
            params: list[tuple[str, float]] = []
            for key in ("max_v_mps", "max_omega_rad_s"):
                value = params_raw.get(key)
                if value is None:
                    continue
                if not isinstance(value, (int, float)) or isinstance(value, bool):
                    raise LLMRequestError(f"{key} must be numeric or null")
                params.append((key, float(value)))
            action = RobotAction(action_name, tuple(params))
        elif any(params_raw.get(key) is not None for key in ("max_v_mps", "max_omega_rad_s")):
            raise LLMRequestError("action_parameters must be null-valued when action_name is null")

        try:
            return LLMDecision(spoken_text, action, self._config.model)
        except ValueError as exc:
            raise LLMRequestError(str(exc)) from exc


__all__ = [
    "DECISION_SCHEMA",
    "GroqChatConfig",
    "GroqStructuredChatClient",
    "LLMRequestError",
]
