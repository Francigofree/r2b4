"""Gemini Robotics ER 2 Preview client using the Interactions API."""
from __future__ import annotations

import base64
import json
import math
import queue
import threading
import time
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

    def _create(self, deadline: float, cancel_event: threading.Event | None, **kwargs):
        # Only a model request runs in this daemon thread. Late responses cannot
        # dispatch tools; the caller owns all tool admission and cancellation.
        def remaining() -> float:
            if cancel_event is not None and cancel_event.is_set():
                raise TimeoutError("ER2 preview cancelled")
            value = deadline - time.monotonic()
            if value <= 0:
                raise TimeoutError("ER2 preview deadline reached")
            return value
        remaining()
        response: queue.Queue = queue.Queue(maxsize=1)
        def request() -> None:
            try:
                result = self.client.interactions.create(timeout=remaining(), **kwargs)
                response.put_nowait((True, result))
            except Exception as exc:
                response.put_nowait((False, exc))
        threading.Thread(target=request, name="er2-preview-request", daemon=True).start()
        while True:
            try:
                ok, value = response.get(timeout=min(0.02, remaining()))
            except queue.Empty:
                continue
            remaining()
            if not ok:
                raise value
            return value

    def run(
        self,
        prompt: str,
        *,
        image_bytes: bytes | None = None,
        mime_type: str = "image/jpeg",
        tools: Er2RobotTools | None = None,
        max_tool_rounds: int = 8,
        timeout_s: float = 30.0,
        cancel_event: threading.Event | None = None,
    ) -> Er2PreviewResult:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt must be non-empty")
        if image_bytes is not None and not isinstance(image_bytes, bytes):
            raise TypeError("image_bytes must be bytes or None")
        if max_tool_rounds < 0:
            raise ValueError("max_tool_rounds must be non-negative")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        deadline = time.monotonic() + timeout_s

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
            self._emit(
                "ER2_PREVIEW_REQUEST_TX",
                model=self.config.preview_model,
                input_items=len(inputs),
                image_bytes=len(image_bytes) if image_bytes is not None else 0,
                tool_declarations=len(tool_declarations or ()),
            )
            interaction = self._create(deadline, cancel_event, **kwargs)
            steps = _interaction_steps(interaction)
            self._emit(
                "ER2_PREVIEW_RESPONSE_RX",
                model=self.config.preview_model,
                interaction_id=getattr(interaction, "id", None),
                step_count=len(steps),
                function_call_count=sum(1 for step in steps if getattr(step, "type", None) == "function_call"),
            )
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
                    if (cancel_event is not None and cancel_event.is_set()) or time.monotonic() >= deadline:
                        raise TimeoutError("ER2 preview cancelled or expired")
                    name = getattr(call, "name", None)
                    arguments = _function_call_arguments(call)
                    call_id = getattr(call, "id", None)
                    if not isinstance(name, str) or arguments is None or not isinstance(call_id, str) or not call_id:
                        raise Er2PreviewError("ER2 returned malformed function call")
                    self._emit(
                        "ER2_PREVIEW_TOOL_CALL_RX",
                        interaction_id=previous_id,
                        tool_name=name,
                        call_id=call_id,
                        argument_count=len(arguments),
                        physical=name in PHYSICAL_TOOLS,
                    )
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
                            result = tools.execute(name, arguments, cancel_event=cancel_event, deadline=deadline)
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
                self._emit(
                    "ER2_PREVIEW_TOOL_RESULTS_TX",
                    previous_interaction_id=previous_id,
                    tool_round=rounds,
                    result_count=len(function_results),
                )
                interaction = self._create(deadline, cancel_event,
                    model=self.config.preview_model,
                    previous_interaction_id=previous_id,
                    input=function_results,
                    tools=tool_declarations,
                    generation_config=generation_config,
                )
                steps = _interaction_steps(interaction)
                self._emit(
                    "ER2_PREVIEW_RESPONSE_RX",
                    model=self.config.preview_model,
                    interaction_id=getattr(interaction, "id", None),
                    step_count=len(steps),
                    function_call_count=sum(1 for step in steps if getattr(step, "type", None) == "function_call"),
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
