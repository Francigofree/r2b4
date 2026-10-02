#!/usr/bin/env python3
"""Install the R2B4 ChatGPT OAuth + automatic LLM failover upgrade.

Source expectations: current Francigofree/r2b4 main or the previous
ChatGPT-primary upgrade applied on top of it.

Deliberately simple:
- locate project root
- validate/patch a small set of source anchors
- make one timestamped backup
- copy only the changed/new modules and tests
- enable OpenAI primary + failover in conf/.wake.env
- create a stable ChatGPT host ID if missing
- syntax-check changed Python files

No git state, hash, API-key or network checks are required. No robot runtime is
started by this installer.
"""
from __future__ import annotations

import os
import shutil
import sys
import uuid
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
    if (cwd / "r2b4_voice").is_dir() and (cwd / "v3").is_dir():
        return cwd
    default = Path("/home/alba/project_r2b4")
    if default.is_dir():
        return default.resolve()
    raise SystemExit("R2B4 project root not found; pass it as the only argument")


def _replace_optional(text: str, old: str, new: str) -> str:
    if old in text:
        return text.replace(old, new)
    return text


def _replace_required_or_done(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one source anchor, found {count}")
    return text.replace(old, new, 1)


def _ensure_parenthesized_import(text: str, prefix: str, names: tuple[str, ...], label: str) -> str:
    start = text.find(prefix)
    if start < 0:
        raise RuntimeError(f"{label}: import block not found: {prefix!r}")
    end = text.find("\n)", start)
    if end < 0:
        raise RuntimeError(f"{label}: import block terminator not found")
    block = text[start:end]
    insertion = ""
    for name in names:
        if f"    {name}," not in block:
            insertion += f"    {name},\n"
    if insertion:
        text = text[: start + len(prefix)] + insertion + text[start + len(prefix) :]
    return text


def _patch_conversation_cli(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    prefix = "from .llm_provider import (\n"
    text = _ensure_parenthesized_import(text, prefix, ("api_key_env_for", "llm_auth_summary"), str(path))
    text = _replace_optional(
        text,
        '    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"\n',
        "    key_name = api_key_env_for(provider)\n",
    )
    start = text.find("def _check(")
    end = text.find("def main(", start)
    if start < 0 or end < 0:
        raise RuntimeError(f"{path}: _check/main anchors not found")
    replacement = '''def _check(root: Path, provider: str, model: str, key: str | None) -> int:\n    prompt = root / "conf" / "r2b4_agent_system.md"\n    secret = root / "conf" / ".wake.env"\n    project_env = _load_project_env(root)\n    auth = llm_auth_summary(root, project_env=project_env)\n    result = {\n        "project_root": str(root),\n        "system_prompt": "PASS" if prompt.is_file() else "FAIL",\n        "llm_provider": provider,\n        "llm_model": model,\n        "llm_api_key": "OPTIONAL:PRESENT" if key else "OPTIONAL:MISSING",\n        "llm_auth": auth,\n        "secret_file": str(secret),\n        "action_mode": "AGENT_PROPOSAL_ONLY",\n        "motor_action_execution": False,\n        "groq_stt_key_present": "PASS" if _setting(project_env, "GROQ_API_KEY") else "FAIL",\n    }\n    print(json.dumps(result, ensure_ascii=False, indent=2))\n    return 0 if result["system_prompt"] == "PASS" and auth.get("llm_ready") is True else 1\n'''
    text = text[:start] + replacement + "\n" + text[end:]
    main_start = text.find("def main(")
    key_guard = text.find("\n    if not key:\n", main_start)
    if key_guard >= 0:
        guard_end = text.find("\n    if not args.text", key_guard)
        if guard_end < 0:
            raise RuntimeError(f"{path}: LLM API-key guard end not found")
        text = text[:key_guard] + text[guard_end:]
    return text


def _patch_voice_service(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    text = _replace_optional(text, "    -> RobotInterface conversation.submit_text -> Gemini/Groq LLM\n", "    -> RobotInterface conversation.submit_text -> ChatGPT OAuth/OpenAI API key/Gemini/Groq LLM\n")
    text = _replace_optional(text, "    -> RobotInterface conversation.submit_text -> OpenAI/Gemini/Groq LLM\n", "    -> RobotInterface conversation.submit_text -> ChatGPT OAuth/OpenAI API key/Gemini/Groq LLM\n")
    lines = text.splitlines(keepends=True)
    found = False
    for idx, line in enumerate(lines):
        if line.startswith("from .llm_provider import "):
            lines[idx] = "from .llm_provider import api_key_env_for, default_model_for, llm_auth_summary, resolve_llm_provider\n"
            found = True
            break
    if not found:
        raise RuntimeError(f"{path}: llm_provider import not found")
    text = "".join(lines)
    text = _replace_optional(
        text,
        '    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"\n',
        "    key_name = api_key_env_for(provider)\n",
    )

    diag_start = text.find("def _diagnostic_check(")
    diag_end = text.find("\n\ndef _acquire_instance_lock", diag_start)
    if diag_start < 0 or diag_end < 0:
        raise RuntimeError(f"{path}: diagnostic function anchors not found")
    diag = text[diag_start:diag_end]
    if "auth = llm_auth_summary(" not in diag:
        marker = '    gemini_key = _setting(project_env, "GEMINI_API_KEY")\n'
        if marker not in diag:
            raise RuntimeError(f"{path}: diagnostic Gemini key anchor not found")
        diag = diag.replace(marker, marker + "    auth = llm_auth_summary(root, project_env=project_env)\n", 1)
    diag = diag.replace('        "llm_api_key": "PASS" if llm_key else "FAIL",\n', '        "llm_api_key": "OPTIONAL:PRESENT" if llm_key else "OPTIONAL:MISSING",\n        "llm_auth": auth,\n')
    diag = diag.replace("            llm_key,\n", '            auth.get("llm_ready") is True,\n')
    text = text[:diag_start] + diag + text[diag_end:]

    main_start = text.find("def main(")
    if main_start < 0:
        raise RuntimeError(f"{path}: main not found")
    main = text[main_start:]
    anchor = "        provider, model, llm_key = _resolved_llm(project_env)\n"
    if "        auth = llm_auth_summary(root, project_env=project_env)\n" not in main:
        if anchor not in main:
            raise RuntimeError(f"{path}: main LLM resolution anchor not found")
        main = main.replace(anchor, anchor + "        auth = llm_auth_summary(root, project_env=project_env)\n", 1)
    old_guard = '        if not llm_key:\n            raise RuntimeError(f"missing API key for LLM provider {provider}")\n'
    new_guard = '        if auth.get("llm_ready") is not True:\n            raise RuntimeError("no usable LLM authentication; run ./r chatgpt login or configure a fallback API key")\n'
    if old_guard in main:
        main = main.replace(old_guard, new_guard, 1)
    elif new_guard not in main:
        raise RuntimeError(f"{path}: main LLM auth guard anchor not found")
    text = text[:main_start] + main
    return text


def _patch_agent_runner(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    found = False
    for idx, line in enumerate(lines):
        if line.startswith("from r2b4_voice.llm_provider import "):
            lines[idx] = "from r2b4_voice.llm_provider import api_key_env_for, default_model_for, resolve_llm_provider\n"
            found = True
            break
    if not found:
        raise RuntimeError(f"{path}: llm_provider import not found")
    text = "".join(lines)
    text = _replace_optional(
        text,
        '    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"\n',
        "    key_name = api_key_env_for(provider)\n",
    )
    old = '    if not key:\n        raise RuntimeError(f"{key_name} is not configured")\n\n'
    text = text.replace(old, "", 1)
    return text


def _patch_conversation_interface(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    old = "    llm = build_llm_client(provider=provider, api_key=api_key, model=model)\n"
    new = "    llm = build_llm_client(provider=provider, api_key=api_key, model=model, project_root=root)\n"
    return _replace_required_or_done(text, old, new, str(path))


def _patch_launcher(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    text = _replace_optional(text, '            "provider": "gemini",\n', '            "provider": "chatgpt_oauth+failover",\n')
    text = _replace_optional(text, '        "  r -- \"s\"                   Kényszerített sima Gemini prompt\\n"\n', '        "  r -- \\"s\\"                   Sima LLM prompt (ChatGPT OAuth + failover)\\n"\n')
    # The exact help line contains nested quotes; handle the source form too.
    text = text.replace('        "  r -- \\"s\\"                   Kényszerített sima Gemini prompt\\n"\n', '        "  r -- \\"s\\"                   Sima LLM prompt (ChatGPT OAuth + failover)\\n"\n')
    text = text.replace("# is treated as a plain Gemini prompt.", "# is treated as a plain LLM prompt.")
    text = text.replace(
        'known |= set(host_cli.COMMANDS) | {"er2", "voice", "wake", "help", "commands", "route", "evi"}',
        'known |= set(host_cli.COMMANDS) | {"chatgpt", "er2", "voice", "wake", "help", "commands", "route", "evi"}',
    )
    voice_handler = '''        if args[0] == "voice" or args[0] == "wake":\n            return launcher_extras.voice_command(args[1:], root)\n'''
    chat_handler = '''        if args[0] == "chatgpt":\n            from r2b4_voice.openai_oauth import cli_main as chatgpt_oauth_main\n            return chatgpt_oauth_main(list(args[1:]), project_root=root)\n'''
    if chat_handler not in text:
        if voice_handler not in text:
            raise RuntimeError(f"{path}: voice command handler anchor not found")
        text = text.replace(voice_handler, chat_handler + voice_handler, 1)
    help_anchor = '        "  r voice restart|check      Voice service újraindítás / diagnosztika\\n"\n'
    help_line = '        "  r chatgpt status|login     ChatGPT OAuth állapot / böngészős belépés\\n"\n'
    if help_line not in text:
        if help_anchor not in text:
            raise RuntimeError(f"{path}: help anchor not found")
        text = text.replace(help_anchor, help_anchor + help_line, 1)
    catalog_anchor = '''        "voice": {\n            "command": "voice",\n'''
    catalog = '''        "chatgpt": {\n            "command": "chatgpt",\n            "operations": ["status", "login", "host-id", "import", "logout"],\n            "auth": "OAUTH_PRIMARY_API_KEY_OPTIONAL",\n            "runtime": "NONE",\n        },\n'''
    if catalog not in text:
        if catalog_anchor not in text:
            raise RuntimeError(f"{path}: catalog voice anchor not found")
        text = text.replace(catalog_anchor, catalog + catalog_anchor, 1)
    return text


def _patch_contract(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "Normál beszélgetési kérésnél az **alapértelmezett beszélgető partner a Gemini**.",
        "Normál beszélgetési kérésnél az **alapértelmezett beszélgető partner a ChatGPT/OpenAI provider**.",
    )
    section = '''\n### LLM hitelesítés és automatikus fallback\n\nA ChatGPT/OpenAI elsődleges hitelesítési útja a **Sign in with ChatGPT OAuth**. A mentett OAuth access token automatikusan használható, lejárat előtt vagy lejáratkor a hozzá tartozó rotating refresh tokennel megújítandó. Az `OPENAI_API_KEY` opcionális fallback, nem kötelező előfeltétel.\n\nAz LLM provider-lánc normál sorrendje: **ChatGPT/OpenAI OAuth → OpenAI API key → Gemini → Groq**, kizárólag azokból az ágakból, amelyekhez érvényes helyi hitelesítés van. Kvóta-, rate-limit-, provider-, transport- vagy érvénytelen LLM-válasz hiba esetén csak minimális, indokolt újrapróbálkozás engedett; ezután a következő konfigurált LLM-re kell váltani. Kvóta/rate-limit és szemantikai/strukturált válaszhiba nem indokol ismételt azonos-provider próbálkozásokat.\n\nA provider-váltás kizárólag a magas szintű LLM-válasz előállítását érinti. Nem ismételhet meg már végrehajtott toolt vagy robotakciót, nem hoz létre új robotikai authorityt, és nem kerülheti meg a canonical RobotInterface/V3 command- és safety-utat.\n\n'''
    if "### LLM hitelesítés és automatikus fallback" not in text:
        anchor = "### Hallható válasz\n"
        if anchor not in text:
            raise RuntimeError(f"{path}: Hallható válasz section anchor not found")
        text = text.replace(anchor, section + anchor, 1)
    text = text.replace(
        "**Az alapértelmezett beszélgető partner Gemini; szükség esetén a végrehajtási mód választó ER2 streamet, ER2 preview-t vagy későbbi más végrehajtót választhat.**",
        "**Az alapértelmezett beszélgető partner ChatGPT/OpenAI; LLM-hibánál a hitelesített fallback-lánc automatikusan válthat OpenAI API key, Gemini vagy Groq providerre; ER2 stream/preview továbbra is külön végrehajtási mód.**",
    )
    text = text.replace(
        "**Az alapértelmezett beszélgető partner ChatGPT/OpenAI; szükség esetén a végrehajtási mód választó Gemini fallbacket, ER2 streamet, ER2 preview-t vagy későbbi más végrehajtót választhat.**",
        "**Az alapértelmezett beszélgető partner ChatGPT/OpenAI; LLM-hibánál a hitelesített fallback-lánc automatikusan válthat OpenAI API key, Gemini vagy Groq providerre; ER2 stream/preview továbbra is külön végrehajtási mód.**",
    )
    text = text.replace(
        "normál beszélgetésnél alapértelmezetten Geminivel dolgozzon, szükség esetén a végrehajtási mód választó válthasson ER2 streamre, ER2 preview-ra vagy későbbi más végrehajtóra;",
        "normál beszélgetésnél alapértelmezetten ChatGPT/OpenAI OAuth-pal dolgozzon, LLM-hibánál automatikusan válthasson a konfigurált fallback providerre, és szükség esetén a végrehajtási mód választó külön ER2 streamre vagy ER2 preview-ra válthasson;",
    )
    text = text.replace(
        "normál beszélgetésnél alapértelmezetten ChatGPT/OpenAI providerrel dolgozzon, szükség esetén a végrehajtási mód választó válthasson Gemini fallbackre, ER2 streamre, ER2 preview-ra vagy későbbi más végrehajtóra;",
        "normál beszélgetésnél alapértelmezetten ChatGPT/OpenAI OAuth-pal dolgozzon, LLM-hibánál automatikusan válthasson a konfigurált fallback providerre, és szükség esetén a végrehajtási mód választó külön ER2 streamre vagy ER2 preview-ra válthasson;",
    )
    return text


def patch_source(root: Path) -> dict[Path, str]:
    patchers = {
        root / "r2b4_voice" / "conversation_cli.py": _patch_conversation_cli,
        root / "r2b4_voice" / "voice_service.py": _patch_voice_service,
        root / "r2b4_voice" / "conversation_interface.py": _patch_conversation_interface,
        root / "r2b4_orchestration" / "agent_runner.py": _patch_agent_runner,
        root / "v3" / "launcher_cli.py": _patch_launcher,
        root / "R2B4_SYSTEM_BEHAVIOR_CONTRACT.md": _patch_contract,
    }
    staged: dict[Path, str] = {}
    for path, patcher in patchers.items():
        if not path.is_file():
            raise RuntimeError(f"required source file missing: {path}")
        staged[path] = patcher(path)
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


def update_env(root: Path) -> tuple[Path, str]:
    path = root / "conf" / ".wake.env"
    current = path.read_text(encoding="utf-8") if path.is_file() else ""
    lines = current.splitlines()
    lines = set_env_value(lines, "R2B4_LLM_PROVIDER", "openai")
    lines = set_env_value(lines, "R2B4_LLM_MODEL", DEFAULT_MODEL)
    lines = set_env_value(lines, "R2B4_LLM_FAILOVER", "1")
    return path, "\n".join(lines).rstrip() + "\n"


def ensure_host_id(root: Path) -> tuple[Path, str, bool]:
    path = root / "conf" / ".chatgpt_host_id"
    if path.is_file():
        value = path.read_text(encoding="utf-8").strip()
        if not value:
            raise RuntimeError(f"empty ChatGPT host ID: {path}")
        return path, value, False
    return path, "urn:uuid:" + str(uuid.uuid4()), True


def backup_files(root: Path, paths: list[Path]) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"chatgpt_oauth_failover_{stamp}"
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
        if path.suffix != ".py" or not path.is_file():
            continue
        compile(path.read_text(encoding="utf-8"), str(path), "exec")


def main() -> int:
    root = project_root()
    required = [root / "r2b4_voice", root / "r2b4_orchestration", root / "v3", root / "R2B4_SYSTEM_BEHAVIOR_CONTRACT.md"]
    if not all(path.exists() for path in required):
        raise RuntimeError(f"not an R2B4 project root: {root}")

    staged = patch_source(root)  # validate all source anchors before any write
    payload_map = {
        PAYLOAD_DIR / "r2b4_voice" / "openai_oauth.py": root / "r2b4_voice" / "openai_oauth.py",
        PAYLOAD_DIR / "r2b4_voice" / "openai_llm.py": root / "r2b4_voice" / "openai_llm.py",
        PAYLOAD_DIR / "r2b4_voice" / "llm_failover.py": root / "r2b4_voice" / "llm_failover.py",
        PAYLOAD_DIR / "r2b4_voice" / "llm_provider.py": root / "r2b4_voice" / "llm_provider.py",
        PAYLOAD_DIR / "r2b4_voice" / "plain_llm.py": root / "r2b4_voice" / "plain_llm.py",
        PAYLOAD_DIR / "tests" / "core" / "test_openai_oauth.py": root / "tests" / "core" / "test_openai_oauth.py",
        PAYLOAD_DIR / "tests" / "core" / "test_llm_failover.py": root / "tests" / "core" / "test_llm_failover.py",
        PAYLOAD_DIR / "tests" / "core" / "test_openai_responses_transport.py": root / "tests" / "core" / "test_openai_responses_transport.py",
        PAYLOAD_DIR / "tests" / "core" / "test_openai_primary_provider.py": root / "tests" / "core" / "test_openai_primary_provider.py",
    }
    for src in payload_map:
        if not src.is_file():
            raise RuntimeError(f"package payload missing: {src}")

    env_path, env_text = update_env(root)
    host_path, host_id, create_host = ensure_host_id(root)
    touched = list(staged) + list(payload_map.values()) + [env_path, host_path]
    backup = backup_files(root, touched)

    for path, content in staged.items():
        path.write_text(content, encoding="utf-8")
    for src, dst in payload_map.items():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    env_path.parent.mkdir(parents=True, exist_ok=True)
    env_path.write_text(env_text, encoding="utf-8")
    os.chmod(env_path, 0o600)
    if create_host:
        host_path.write_text(host_id + "\n", encoding="utf-8")
    os.chmod(host_path, 0o600)

    syntax_check([path for path in touched if path.exists()])

    credential = root / "conf" / ".chatgpt_oauth.json"
    print("R2B4 ChatGPT OAuth + LLM failover upgrade: INSTALLED")
    print(f"project: {root}")
    print(f"backup:  {backup}")
    print("primary: ChatGPT/OpenAI OAuth")
    print("fallback: OpenAI API key -> Gemini -> Groq (only when configured)")
    print(f"model: {DEFAULT_MODEL}")
    print(f"host-id: {host_id}")
    print("OAuth credential: " + ("present" if credential.is_file() else "not signed in yet"))
    print("OPENAI_API_KEY: optional")
    print("Next local-browser step: ./r chatgpt login")
    print("Headless Pi: ./r chatgpt host-id, then use chatgpt_login.py on the browser PC and import the result.")
    print("No robot runtime was started.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"INSTALL FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
