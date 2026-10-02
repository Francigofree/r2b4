"""Motor-free CLI for validating R2B4 conversation/LLM integration."""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
from pathlib import Path

from .conversation_interface import build_voice_interface
from .llm_provider import (
    SUPPORTED_LLM_PROVIDERS,
    default_model_for,
    resolve_llm_provider,
)


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_project_env(root: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    path = root / "conf" / ".wake.env"
    if not path.is_file():
        return values
    try:
        mode = stat.S_IMODE(path.stat().st_mode)
        if mode & 0o077:
            raise RuntimeError(f"secret file permissions are too open: {oct(mode)}; expected 0o600")
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and value:
                values[key] = value
    except OSError as exc:
        raise RuntimeError(f"cannot read {path}: {exc}") from exc
    return values


def _setting(project_env: dict[str, str], name: str) -> str | None:
    value = os.environ.get(name)
    if isinstance(value, str) and value.strip():
        return value.strip()
    value = project_env.get(name)
    return value.strip() if isinstance(value, str) and value.strip() else None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="r2b4-conversation")
    parser.add_argument("--text", help="submit one text turn and print the completed result")
    parser.add_argument("--source", default="cli")
    parser.add_argument("--wait", type=float, default=25.0)
    parser.add_argument("--check", action="store_true", help="check local config without contacting the LLM provider")
    parser.add_argument("--interactive", action="store_true", help="interactive text-only conversation")
    parser.add_argument("--provider", choices=SUPPORTED_LLM_PROVIDERS)
    parser.add_argument("--model")
    return parser


def _resolved_llm(args: argparse.Namespace, project_env: dict[str, str]) -> tuple[str, str, str | None]:
    provider = resolve_llm_provider(args.provider or _setting(project_env, "R2B4_LLM_PROVIDER"))
    model = args.model or _setting(project_env, "R2B4_LLM_MODEL") or default_model_for(provider)
    key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"
    return provider, model, _setting(project_env, key_name)


def _check(root: Path, provider: str, model: str, key: str | None) -> int:
    prompt = root / "conf" / "voice_llm_system.md"
    secret = root / "conf" / ".wake.env"
    result = {
        "project_root": str(root),
        "system_prompt": "PASS" if prompt.is_file() else "FAIL",
        "llm_provider": provider,
        "llm_model": model,
        "llm_api_key": "PASS" if key else "FAIL",
        "secret_file": str(secret),
        "action_mode": "SHADOW",
        "motor_action_execution": False,
        "groq_stt_key_present": "PASS" if _setting(_load_project_env(root), "GROQ_API_KEY") else "FAIL",
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["system_prompt"] == "PASS" and result["llm_api_key"] == "PASS" else 1


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = _project_root()
    try:
        project_env = _load_project_env(root)
        provider, model, key = _resolved_llm(args, project_env)
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.check:
        return _check(root, provider, model, key)
    if not key:
        key_name = "GEMINI_API_KEY" if provider == "gemini" else "GROQ_API_KEY"
        print(f"ERROR: {key_name} is not configured in environment or conf/.wake.env", file=sys.stderr)
        return 2
    if not args.text and not args.interactive:
        print("ERROR: use --text TEXT, --interactive, or --check", file=sys.stderr)
        return 2

    try:
        with build_voice_interface(root, api_key=key, provider=provider, model=model) as bundle:
            interface = bundle.interface
            if args.text:
                accepted = interface.execute("conversation.submit_text", text=args.text, source=args.source)
                turn_id = accepted["turn_id"]
                result = bundle.conversation.wait_for_turn(turn_id, timeout_s=args.wait)
                if result is None:
                    print(json.dumps({"status": "TIMEOUT", "turn_id": turn_id}, ensure_ascii=False, indent=2))
                    return 3
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 0 if result.get("error") is None else 4

            while True:
                try:
                    text = input("you> ").strip()
                except EOFError:
                    return 0
                if not text:
                    continue
                if text.lower() in {"quit", "exit", "kilep", "kilép"}:
                    return 0
                accepted = interface.execute("conversation.submit_text", text=text, source="cli")
                result = bundle.conversation.wait_for_turn(accepted["turn_id"], timeout_s=args.wait)
                if result is None:
                    print("alba> [timeout]")
                    continue
                if result.get("error"):
                    print(f"alba> [ERROR] {result['error']}")
                else:
                    print(f"alba> {result.get('spoken_text') or '[nincs beszédválasz]'}")
                    if result.get("proposed_action"):
                        print(f"intent[SHADOW]> {json.dumps(result['proposed_action'], ensure_ascii=False)}")
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
