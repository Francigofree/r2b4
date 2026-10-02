#!/usr/bin/env python3
"""Network smoke for the fixed Gemini structured Agent step. Does not execute robot actions."""
from __future__ import annotations

import os
import stat
from pathlib import Path

from r2b4_voice.gemini_llm import GeminiChatConfig, GeminiStructuredChatClient


def load_env(root: Path) -> dict[str, str]:
    path = root / "conf" / ".wake.env"
    if not path.is_file():
        return {}
    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077:
        raise RuntimeError(f"unsafe secret permissions: {oct(mode)}")
    result: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip().strip('"').strip("'")
    return result


root = Path.cwd().resolve()
env = load_env(root)
key = os.environ.get("GEMINI_API_KEY") or env.get("GEMINI_API_KEY")
model = os.environ.get("R2B4_LLM_MODEL") or env.get("R2B4_LLM_MODEL") or "gemini-2.5-flash"
if not key:
    raise SystemExit("GEMINI_API_KEY missing")
client = GeminiStructuredChatClient(api_key=key, config=GeminiChatConfig(model=model))
reply = client.complete_agent_step(
    [
        {"role": "system", "content": "Return a final answer only. Do not request tools or robot actions."},
        {"role": "user", "content": "Miért kék az ég? Egy rövid mondatban válaszolj."},
    ],
    (),
    (),
)
if reply.spoken_text is None:
    raise SystemExit(f"unexpected non-final reply: {reply}")
print("PASS")
print(reply.spoken_text)
