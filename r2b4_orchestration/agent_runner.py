"""One-shot Agent Core runner used by the root r launcher."""
from __future__ import annotations

import os
import stat
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Mapping

from .brain_hri import adopt_brain_result, wait_for_brain_goal
from v3.action_catalog import action_descriptor
from r2b4_voice.conversation_interface import build_voice_interface
from r2b4_voice.llm_provider import api_key_env_for, default_model_for, resolve_llm_provider


@dataclass(frozen=True, slots=True)
class AgentRunResult:
    text: str
    action_status: str | None = None


def _load_env(root: Path) -> dict[str, str]:
    path = root / "conf" / ".wake.env"
    if not path.is_file():
        return {}
    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077:
        raise RuntimeError(f"secret file permissions are too open: {oct(mode)}; expected 0o600")
    out: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip(); value = value.strip().strip('"').strip("'")
        if key and value:
            out[key] = value
    return out


def _setting(values: Mapping[str, str], name: str) -> str | None:
    value = os.environ.get(name)
    if isinstance(value, str) and value.strip():
        return value.strip()
    value = values.get(name)
    return value.strip() if isinstance(value, str) and value.strip() else None


def _receipt(status: str, executed: bool) -> str:
    if executed:
        return "Rendben."
    if status.startswith("FAILED:"):
        return "A kért robotművelet nem fejeződött be."
    if status.startswith("REJECTED:"):
        return "A kért robotműveletet most nem tudom biztonságosan végrehajtani."
    return "A robotművelet nem indult el."


def _finite_plan(result: Mapping[str, object]) -> bool:
    plan = result.get("proposed_plan")
    if isinstance(plan, Mapping):
        steps = plan.get("steps")
    else:
        action = result.get("proposed_action")
        steps = [{"action": action.get("name")}] if isinstance(action, Mapping) else None
    if not isinstance(steps, (list, tuple)) or not steps:
        return False
    has_finite_action = False
    for step in steps:
        if not isinstance(step, Mapping):
            return False
        name = step.get("action")
        if name == "vision.observe":
            continue
        descriptor = action_descriptor(name) if isinstance(name, str) else None
        if descriptor is None or not descriptor.completion_required:
            return False
        has_finite_action = True
    return has_finite_action


def run_agent_prompt(
    prompt: str,
    *,
    project_root: str | Path,
    wait_s: float = 90.0,
    developer_mode: bool = False,
) -> AgentRunResult:
    root = Path(project_root).expanduser().resolve()
    env = _load_env(root)
    provider = resolve_llm_provider(_setting(env, "R2B4_LLM_PROVIDER"))
    model = _setting(env, "R2B4_LLM_MODEL") or default_model_for(provider)
    key_name = api_key_env_for(provider)
    key = _setting(env, key_name)
    with build_voice_interface(
        root, api_key=key, provider=provider, model=model, developer_mode=developer_mode,
    ) as bundle:
        accepted = bundle.interface.execute("conversation.submit_text", text=prompt, source="launcher")
        turn_id = accepted.get("turn_id") if isinstance(accepted, Mapping) else None
        if not isinstance(turn_id, str):
            raise RuntimeError("conversation.submit_text returned no turn_id")
        result = bundle.conversation.wait_for_turn(turn_id, timeout_s=wait_s)
        if result is None:
            raise TimeoutError("Agent Core turn timed out")
        if result.get("error"):
            raise RuntimeError(str(result["error"]))
        proposed = result.get("proposed_action")
        if proposed is not None or result.get("proposed_plan") is not None:
            adoption = adopt_brain_result(bundle.interface, result)
            if adoption.status == "ACTIVE" and _finite_plan(result):
                adoption = wait_for_brain_goal(bundle.interface, adoption, timeout_s=wait_s)
            return AgentRunResult(adoption.text, adoption.status)
        text = result.get("spoken_text")
        if not isinstance(text, str) or not text.strip():
            raise RuntimeError("Agent Core returned no final text")
        return AgentRunResult(text.strip(), str(result.get("action_status") or "NONE"))


__all__ = ["AgentRunResult", "run_agent_prompt"]
