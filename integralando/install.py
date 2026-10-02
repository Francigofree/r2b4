#!/usr/bin/env python3
"""Install the R2B4 ChatGPT/OpenAI-primary LLM upgrade.

Deliberately simple:
- locate project root
- validate a few source anchors
- make one backup
- copy payload
- apply exact text patches
- switch the local LLM provider/model
- syntax-check changed Python files

No git state, hash or network checks are required.
"""
from __future__ import annotations

import os
import shutil
import sys
from datetime import datetime
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
PAYLOAD_DIR = PACKAGE_DIR / "payload"
DEFAULT_MODEL = "gpt-5.6"


def project_root() -> Path:
    if len(sys.argv) > 2:
        raise SystemExit("usage: python3 install.py [PROJECT_ROOT]")
    if len(sys.argv) == 2:
        return Path(sys.argv[1]).expanduser().resolve()

    cwd = Path.cwd().resolve()
    if (cwd / "r2b4_voice").is_dir():
        return cwd

    default = Path("/home/alba/project_r2b4")
    if default.is_dir():
        return default.resolve()

    raise SystemExit("R2B4 project root not found; pass it as the only argument")


def replace_once_or_installed(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one source anchor, found {count}")
    return text.replace(old, new, 1)


def patch_source(root: Path) -> dict[Path, str]:
    staged: dict[Path, str] = {}

    p = root / "r2b4_voice" / "conversation_cli.py"
    text = p.read_text(encoding="utf-8")
    text = replace_once_or_installed(
        text,
        "from .llm_provider import (\n    SUPPORTED_LLM_PROVIDERS,\n",
        "from .llm_provider import (\n    SUPPORTED_LLM_PROVIDERS,\n    api_key_env_for,\n",
        str(p),
    )
    old = 'key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"'
    text = text.replace(old, "key_name = api_key_env_for(provider)")
    if old in text:
        raise RuntimeError(f"{p}: provider key routing patch incomplete")
    staged[p] = text

    p = root / "r2b4_voice" / "voice_service.py"
    text = p.read_text(encoding="utf-8")
    text = replace_once_or_installed(
        text,
        "    -> RobotInterface conversation.submit_text -> Gemini/Groq LLM\n",
        "    -> RobotInterface conversation.submit_text -> OpenAI/Gemini/Groq LLM\n",
        str(p),
    )
    text = replace_once_or_installed(
        text,
        "from .llm_provider import default_model_for, resolve_llm_provider\n",
        "from .llm_provider import api_key_env_for, default_model_for, resolve_llm_provider\n",
        str(p),
    )
    text = replace_once_or_installed(
        text,
        '    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"\n',
        "    key_name = api_key_env_for(provider)\n",
        str(p),
    )
    staged[p] = text

    p = root / "r2b4_orchestration" / "agent_runner.py"
    text = p.read_text(encoding="utf-8")
    text = replace_once_or_installed(
        text,
        "from r2b4_voice.llm_provider import default_model_for, resolve_llm_provider\n",
        "from r2b4_voice.llm_provider import api_key_env_for, default_model_for, resolve_llm_provider\n",
        str(p),
    )
    text = replace_once_or_installed(
        text,
        '    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"\n',
        "    key_name = api_key_env_for(provider)\n",
        str(p),
    )
    staged[p] = text

    p = root / "R2B4_SYSTEM_BEHAVIOR_CONTRACT.md"
    text = p.read_text(encoding="utf-8")
    replacements = [
        (
            "Normál beszélgetési kérésnél az **alapértelmezett beszélgető partner a Gemini**.",
            "Normál beszélgetési kérésnél az **alapértelmezett beszélgető partner a ChatGPT/OpenAI provider**.",
        ),
        (
            "- **ER2 stream**;\n- **ER2 preview**;",
            "- **Gemini fallback**;\n- **ER2 stream**;\n- **ER2 preview**;",
        ),
        (
            "A Gemini, az ER2 stream és az ER2 preview nem külön robotikai authority-k.",
            "A ChatGPT/OpenAI provider, a Gemini, az ER2 stream és az ER2 preview nem külön robotikai authority-k.",
        ),
        (
            "Attól, hogy egy választ Gemini, ER2 stream, ER2 preview vagy más későbbi végrehajtó állít elő,",
            "Attól, hogy egy választ ChatGPT/OpenAI, Gemini, ER2 stream, ER2 preview vagy más későbbi végrehajtó állít elő,",
        ),
        (
            "        ├─ Gemini / conversation\n        ├─ ER2 stream / ER2 preview",
            "        ├─ ChatGPT/OpenAI / conversation\n        ├─ Gemini / fallback conversation\n        ├─ ER2 stream / ER2 preview",
        ),
        (
            "**Voice, Gemini, ER2 és más agent/végrehajtó csak kliens/orchestrator lehet, nem motor-authority.**",
            "**Voice, ChatGPT/OpenAI, Gemini, ER2 és más agent/végrehajtó csak kliens/orchestrator lehet, nem motor-authority.**",
        ),
        (
            "**Az alapértelmezett beszélgető partner Gemini; szükség esetén a végrehajtási mód választó ER2 streamet, ER2 preview-t vagy későbbi más végrehajtót választhat.**",
            "**Az alapértelmezett beszélgető partner ChatGPT/OpenAI; szükség esetén a végrehajtási mód választó Gemini fallbacket, ER2 streamet, ER2 preview-t vagy későbbi más végrehajtót választhat.**",
        ),
        (
            "- a Gemini, ER2 stream, ER2 preview vagy későbbi végrehajtók konkrét routing algoritmusát;",
            "- a ChatGPT/OpenAI, Gemini, ER2 stream, ER2 preview vagy későbbi végrehajtók konkrét routing algoritmusát;",
        ),
        (
            "normál beszélgetésnél alapértelmezetten Geminivel dolgozzon, szükség esetén a végrehajtási mód választó válthasson ER2 streamre, ER2 preview-ra vagy későbbi más végrehajtóra;",
            "normál beszélgetésnél alapértelmezetten ChatGPT/OpenAI providerrel dolgozzon, szükség esetén a végrehajtási mód választó válthasson Gemini fallbackre, ER2 streamre, ER2 preview-ra vagy későbbi más végrehajtóra;",
        ),
    ]
    for old, new in replacements:
        text = replace_once_or_installed(text, old, new, str(p))
    staged[p] = text

    return staged


def set_env_value(lines: list[str], key: str, value: str) -> list[str]:
    prefix = key + "="
    out: list[str] = []
    replaced = False
    for line in lines:
        if line.strip().startswith(prefix):
            if not replaced:
                out.append(prefix + value)
                replaced = True
            continue
        out.append(line)
    if not replaced:
        out.append(prefix + value)
    return out


def update_env(root: Path) -> tuple[Path, str, bool]:
    path = root / "conf" / ".wake.env"
    current = path.read_text(encoding="utf-8") if path.is_file() else ""
    lines = current.splitlines()
    lines = set_env_value(lines, "R2B4_LLM_PROVIDER", "openai")
    lines = set_env_value(lines, "R2B4_LLM_MODEL", DEFAULT_MODEL)
    new = "\n".join(lines).rstrip() + "\n"
    key_present = bool(os.environ.get("OPENAI_API_KEY", "").strip())
    for line in lines:
        if line.strip().startswith("OPENAI_API_KEY=") and line.split("=", 1)[1].strip().strip('"').strip("'"):
            key_present = True
            break
    return path, new, key_present


def backup_files(root: Path, paths: list[Path]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"chatgpt_primary_{stamp}"
    for src in paths:
        if not src.exists():
            continue
        rel = src.relative_to(root)
        dst = backup / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    return backup


def syntax_check(paths: list[Path]) -> None:
    for path in paths:
        if path.suffix != ".py":
            continue
        source = path.read_text(encoding="utf-8")
        compile(source, str(path), "exec")


def main() -> int:
    root = project_root()
    required = [
        root / "r2b4_voice",
        root / "r2b4_orchestration",
        root / "R2B4_SYSTEM_BEHAVIOR_CONTRACT.md",
    ]
    if not all(path.exists() for path in required):
        raise RuntimeError(f"not an R2B4 project root: {root}")

    # Validate all exact patches before writing anything.
    staged = patch_source(root)

    payload_map = {
        PAYLOAD_DIR / "r2b4_voice" / "openai_llm.py": root / "r2b4_voice" / "openai_llm.py",
        PAYLOAD_DIR / "r2b4_voice" / "llm_provider.py": root / "r2b4_voice" / "llm_provider.py",
        PAYLOAD_DIR / "r2b4_voice" / "plain_llm.py": root / "r2b4_voice" / "plain_llm.py",
        PAYLOAD_DIR / "tests" / "core" / "test_openai_responses_transport.py":
            root / "tests" / "core" / "test_openai_responses_transport.py",
        PAYLOAD_DIR / "tests" / "core" / "test_openai_primary_provider.py":
            root / "tests" / "core" / "test_openai_primary_provider.py",
    }
    for src in payload_map:
        if not src.is_file():
            raise RuntimeError(f"package payload missing: {src}")

    env_path, env_text, key_present = update_env(root)
    touched = list(staged) + list(payload_map.values()) + [env_path]
    backup = backup_files(root, touched)

    for path, content in staged.items():
        path.write_text(content, encoding="utf-8")

    for src, dst in payload_map.items():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    env_path.parent.mkdir(parents=True, exist_ok=True)
    env_path.write_text(env_text, encoding="utf-8")
    os.chmod(env_path, 0o600)

    syntax_check([p for p in touched if p.exists()])

    print("R2B4 ChatGPT/OpenAI primary LLM upgrade: INSTALLED")
    print(f"project: {root}")
    print(f"backup:  {backup}")
    print("provider: openai")
    print(f"model:    {DEFAULT_MODEL}")
    if key_present:
        print("OPENAI_API_KEY: present")
    else:
        print("OPENAI_API_KEY: MISSING")
        print(f"Add OPENAI_API_KEY=... to {env_path} before starting the LLM/voice service.")
    print("No robot runtime was started.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"INSTALL FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
