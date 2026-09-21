"""Prompt assembly for robot-aware R2B4 conversation turns."""

from __future__ import annotations

import json
from pathlib import Path

from .conversation_contracts import ConversationMemoryTurn, RobotContextSnapshot, UserTextTurn


PROMPT_VERSION = "R2B4_VOICE_LLM_SYSTEM_V2"


class PromptAssembler:
    def __init__(self, system_prompt_path: Path | str, *, max_history_turns: int = 8) -> None:
        path = Path(system_prompt_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        if not isinstance(max_history_turns, int) or isinstance(max_history_turns, bool) or max_history_turns < 0:
            raise ValueError("max_history_turns must be a non-negative integer")
        text = path.read_text(encoding="utf-8").strip()
        if not text:
            raise ValueError("system prompt is empty")
        self.path = path
        self.system_prompt = text
        self.max_history_turns = max_history_turns

    def build_messages(
        self,
        turn: UserTextTurn,
        context: RobotContextSnapshot,
        history: tuple[ConversationMemoryTurn, ...],
    ) -> list[dict[str, str]]:
        context_json = json.dumps(
            context.to_jsonable(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        messages: list[dict[str, str]] = [
            {"role": "system", "content": self.system_prompt},
            {
                "role": "system",
                "content": (
                    f"PROMPT_VERSION={PROMPT_VERSION}\n"
                    "Az alábbi ROBOT_CONTEXT_JSON friss, csak olvasható robotállapot. "
                    "Kizárólag ebből állíts tényt a robot aktuális állapotáról. "
                    "A runtime.state=STOPPED normál leállított állapot, nem hiba. "
                    "A runtime.state=UNAVAILABLE csak azt jelenti, hogy nincs friss V3 live státusz; "
                    "önmagában nem FAULT és nem üzemképtelenség. Hibát csak explicit FAULT/fault_layer alapján állíts.\n"
                    f"ROBOT_CONTEXT_JSON={context_json}"
                ),
            },
        ]
        selected = history[-self.max_history_turns :] if self.max_history_turns else ()
        for item in selected:
            messages.append({"role": "user", "content": item.user_text})
            messages.append({"role": "assistant", "content": item.assistant_text})
        messages.append({"role": "user", "content": turn.text})
        return messages


__all__ = ["PROMPT_VERSION", "PromptAssembler"]
