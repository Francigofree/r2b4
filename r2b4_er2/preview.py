"""Gemini Robotics ER 2 Preview client using the Interactions API."""
from __future__ import annotations

import base64
import json
from collections.abc import Mapping
from dataclasses import dataclass

from .config import Er2Config, api_key_from_env
from .evidence import Er2Evidence
from .tool_bridge import PHYSICAL_TOOLS, Er2RobotTools, Er2SafetyError


class Er2PreviewError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class Er2PreviewResult:
    interaction_id: str | None
    text: str
    tool_rounds: int


def _interaction_steps(interaction: object) -> tuple[object, ...]:
    """Support the current SDK ``steps`` field and the older ``outputs`` alias."""
    for name in ("steps", "outputs"):
        value = getattr(interaction, name, None)
        if value is not None:
            try:
                return tuple(value or ())
            except TypeError:
                continue
    return ()


def _function_call_arguments(call: object) -> Mapping[str, object] | None:
    for name in ("arguments", "args"):
        value = getattr(call, name, None)
        if isinstance(value, Mapping):
            return value
    return None


class Er2PreviewClient:
    def __init__(
        self,
        config: Er2Config | None = None,
        *,
        api_key: str | None = None,
        client: object | None = None,
        evidence: Er2Evidence | None = None,
    ) -> None:
        self.config = config or Er2Config.from_env()
        self.evidence = evidence
        if client is None:
            try:
                from google import genai
            except ImportError as exc:
                raise Er2PreviewError("google-genai is not installed") from exc
            client = genai.Client(api_key=api_key or api_key_from_env())
        self.client = client

    def _emit(self, event_type: str, **fields: object) -> None:
        if self.evidence is not None:
            self.evidence.emit(event_type, provider="gemini", endpoint="preview", **fields)

    def run(
        self,
        prompt: str,
        *,
        image_bytes: bytes | None = None,
        mime_type: str = "image/jpeg",
        tools: Er2RobotTools | None = None,
        max_tool_rounds: int = 8,
    ) -> Er2PreviewResult:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt must be non-empty")
        if image_bytes is not None and not isinstance(image_bytes, bytes):
            raise TypeError("image_bytes must be bytes or None")
        if max_tool_rounds < 0:
            raise ValueError("max_tool_rounds must be non-negative")

        inputs: list[dict[str, object]] = []
        if image_bytes is not None:
            inputs.append(
                {
                    "type": "image",
                    "data": base64.b64encode(image_bytes).decode("ascii"),
                    "mime_type": mime_type,
                }
            )
        inputs.append({"type": "text", "text": prompt.strip()})
        tool_declarations = tools.interaction_tools() if tools is not None else None
        generation_config = {"thinking_level": "high"}
        kwargs: dict[str, object] = {
            "model": self.config.preview_model,
            "input": inputs,
            "generation_config": generation_config,
        }
        if tool_declarations is not None:
            kwargs["tools"] = tool_declarations

        self._emit(
            "ER2_PREVIEW_START",
            model=self.config.preview_model,
            prompt_chars=len(prompt.strip()),
            image_present=image_bytes is not None,
            tools_enabled=tools is not None,
        )
        try:
            interaction = self.client.interactions.create(**kwargs)  # type: ignore[attr-defined]
            rounds = 0

            while tools is not None:
                calls = [
                    step
                    for step in _interaction_steps(interaction)
                    if getattr(step, "type", None) == "function_call"
                ]
                if not calls:
                    break
                if rounds >= max_tool_rounds:
                    raise Er2PreviewError("ER2 Preview exceeded bounded tool-call rounds")
                previous_id = getattr(interaction, "id", None)
                if not isinstance(previous_id, str) or not previous_id:
                    raise Er2PreviewError("ER2 Preview tool call is missing interaction id")

                function_results: list[dict[str, object]] = []
                physical_tool_executed = False
                for call in calls:
                    name = getattr(call, "name", None)
                    arguments = _function_call_arguments(call)
                    call_id = getattr(call, "id", None)
                    if not isinstance(name, str) or arguments is None or not isinstance(call_id, str) or not call_id:
                        raise Er2PreviewError("ER2 returned malformed function call")
                    if name in PHYSICAL_TOOLS and physical_tool_executed:
                        # Never execute two physical goals selected from the same
                        # model turn. The model must observe the first blocking tool
                        # result and choose the next goal in a fresh interaction.
                        result = {
                            "status": "ERROR",
                            "error": "ONE_PHYSICAL_TOOL_PER_ROUND: wait for the prior mission result, then request the next goal in a new tool round",
                        }
                    else:
                        if name in PHYSICAL_TOOLS:
                            physical_tool_executed = True
                        try:
                            result = tools.execute(name, arguments)
                        except Er2SafetyError:
                            raise
                        except Exception as exc:
                            result = {"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}
                    # Interactions API function results are content blocks. Keep the
                    # robot result JSON-encoded inside a text block, matching the
                    # provider contract and avoiding SDK-specific object coercion.
                    function_results.append(
                        {
                            "type": "function_result",
                            "name": name,
                            "call_id": call_id,
                            "result": [
                                {
                                    "type": "text",
                                    "text": json.dumps(
                                        result,
                                        ensure_ascii=False,
                                        separators=(",", ":"),
                                        default=str,
                                    ),
                                }
                            ],
                        }
                    )
                rounds += 1
                # tools and generation_config are interaction-scoped in the
                # Interactions API, so they must be re-specified on every turn.
                interaction = self.client.interactions.create(  # type: ignore[attr-defined]
                    model=self.config.preview_model,
                    previous_interaction_id=previous_id,
                    input=function_results,
                    tools=tool_declarations,
                    generation_config=generation_config,
                )

            text = getattr(interaction, "output_text", None)
            if not isinstance(text, str):
                text_parts = [
                    getattr(step, "text", "")
                    for step in _interaction_steps(interaction)
                    if isinstance(getattr(step, "text", None), str)
                ]
                text = "".join(text_parts)
            result = Er2PreviewResult(
                interaction_id=getattr(interaction, "id", None),
                text=text,
                tool_rounds=rounds,
            )
            self._emit(
                "ER2_PREVIEW_COMPLETE",
                model=self.config.preview_model,
                interaction_id=result.interaction_id,
                tool_rounds=result.tool_rounds,
                output_chars=len(result.text),
            )
            return result
        except Exception as exc:
            self._emit(
                "ER2_PREVIEW_ERROR",
                model=self.config.preview_model,
                error_type=type(exc).__name__,
                error=str(exc)[:300],
            )
            if isinstance(exc, Er2PreviewError):
                raise
            raise Er2PreviewError(f"ER2 Preview request failed: {type(exc).__name__}: {exc}") from exc


__all__ = ["Er2PreviewClient", "Er2PreviewError", "Er2PreviewResult"]
