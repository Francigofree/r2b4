"""Gemini Robotics ER 2 Preview client using the GA Interactions API."""
from __future__ import annotations

import base64
from collections.abc import Mapping
from dataclasses import dataclass

from .config import Er2Config, api_key_from_env
from .tool_bridge import Er2RobotTools


class Er2PreviewError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class Er2PreviewResult:
    interaction_id: str | None
    text: str
    tool_rounds: int


class Er2PreviewClient:
    def __init__(
        self,
        config: Er2Config | None = None,
        *,
        api_key: str | None = None,
        client: object | None = None,
    ) -> None:
        self.config = config or Er2Config.from_env()
        if client is None:
            try:
                from google import genai
            except ImportError as exc:
                raise Er2PreviewError("google-genai is not installed") from exc
            client = genai.Client(api_key=api_key or api_key_from_env())
        self.client = client

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
        kwargs: dict[str, object] = {
            "model": self.config.preview_model,
            "input": inputs,
            "generation_config": {"thinking_level": "high"},
        }
        if tools is not None:
            kwargs["tools"] = tools.interaction_tools()
        interaction = self.client.interactions.create(**kwargs)  # type: ignore[attr-defined]
        rounds = 0

        while tools is not None:
            calls = [out for out in tuple(getattr(interaction, "outputs", ()) or ()) if getattr(out, "type", None) == "function_call"]
            if not calls:
                break
            if rounds >= max_tool_rounds:
                raise Er2PreviewError("ER2 Preview exceeded bounded tool-call rounds")
            function_results: list[dict[str, object]] = []
            for call in calls:
                name = getattr(call, "name", None)
                arguments = getattr(call, "arguments", {}) or {}
                call_id = getattr(call, "id", None)
                if not isinstance(name, str) or not isinstance(arguments, Mapping) or not isinstance(call_id, str):
                    raise Er2PreviewError("ER2 returned malformed function call")
                try:
                    result = tools.execute(name, arguments)
                except Exception as exc:
                    result = {"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}
                function_results.append(
                    {
                        "type": "function_result",
                        "name": name,
                        "call_id": call_id,
                        "result": result,
                    }
                )
            rounds += 1
            interaction = self.client.interactions.create(  # type: ignore[attr-defined]
                model=self.config.preview_model,
                previous_interaction_id=getattr(interaction, "id", None),
                input=function_results,
            )

        text = getattr(interaction, "output_text", None)
        if not isinstance(text, str):
            text_parts = [
                getattr(out, "text", "")
                for out in tuple(getattr(interaction, "outputs", ()) or ())
                if isinstance(getattr(out, "text", None), str)
            ]
            text = "".join(text_parts)
        return Er2PreviewResult(
            interaction_id=getattr(interaction, "id", None),
            text=text,
            tool_rounds=rounds,
        )


__all__ = ["Er2PreviewClient", "Er2PreviewError", "Er2PreviewResult"]
