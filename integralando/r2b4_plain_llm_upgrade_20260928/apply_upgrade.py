#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

BASE_COMMIT = "b44b183ed9c626b7df8fa6b6d12d5ed3fce93560"
EXPECTED_LAUNCHER_GIT_BLOB = "98f885727add163f21181eda005910fa88a40ab3"


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"patch anchor {label!r} expected once, found {count}")
    return text.replace(old, new, 1)


def patch_launcher(source: str) -> str:
    source = replace_once(
        source,
        '''        "er2": {\n            "commands": ["status", "preview", "stream"],\n            "default_command": "stream",\n            "stream_defaults": ["camera", "tools", "speak", "json"],\n            "stream_options": ["--camera", "--tools", "--speak", "--json", "--seconds"],\n            "execution": "REAL_ONLY",\n        },\n        "voice": {''',
        '''        "er2": {\n            "commands": ["status", "preview", "stream"],\n            "default_command": "stream",\n            "stream_defaults": ["camera", "tools", "speak", "json"],\n            "stream_options": ["--camera", "--tools", "--speak", "--json", "--seconds"],\n            "execution": "REAL_ONLY",\n        },\n        "plain_llm": {\n            "usage": 'r "PROMPT"',\n            "escape_usage": 'r -- "PROMPT"',\n            "provider": "gemini",\n            "tts": "default",\n            "runtime": "NONE",\n        },\n        "voice": {''',
        "catalog",
    )
    source = replace_once(
        source,
        '''        "\\nAI, voice, fejlesztés és gépállapot:\\n"\n        "  r er2 \\\"FELADAT\\\"            ER2 stream; camera/tools/speak/json bekapcsolva\\n"''',
        '''        "\\nAI, voice, fejlesztés és gépállapot:\\n"\n        "  r \\\"KÉRDÉS\\\"                Sima Gemini válasz + alapértelmezett TTS\\n"\n        "  r -- \\\"s\\\"                   Prompt akkor is, ha a szöveg r-parancs neve\\n"\n        "  r er2 \\\"FELADAT\\\"            ER2 stream; camera/tools/speak/json bekapcsolva\\n"''',
        "help",
    )
    source = replace_once(
        source,
        '''    print("\\nER2: r er2 status|preview|stream; röviden: r er2 \\\"FELADAT\\\"")\n    print("Voice wake: r voice status|on|off|restart|check  (alias: r wake ...)")''',
        '''    print("\\nER2: r er2 status|preview|stream; röviden: r er2 \\\"FELADAT\\\"")\n    print('Plain LLM: r "PROMPT"; ütköző parancsnév esetén: r -- "PROMPT"')\n    print("Voice wake: r voice status|on|off|restart|check  (alias: r wake ...)")''',
        "commands-help",
    )
    source = replace_once(
        source,
        '''\ndef _unknown_command(command: str) -> int:\n''',
        '''\ndef _plain_prompt(argv: Sequence[str], root: Path) -> int:\n    prompt = " ".join(argv).strip()\n    if not prompt:\n        raise LauncherError('Használat: r "PROMPT" vagy r -- "PROMPT"')\n    from r2b4_voice.plain_llm import run_plain_prompt\n    return run_plain_prompt(prompt, project_root=root)\n\n\ndef _unknown_command(command: str) -> int:\n''',
        "plain-helper",
    )
    source = replace_once(
        source,
        '''        if args and args[0] == "__complete":\n            return launcher_extras.emit_completion(args[1:], root)\n\n        if not args or args[0] in {"help", "-h", "--help"}:''',
        '''        if args and args[0] == "__complete":\n            return launcher_extras.emit_completion(args[1:], root)\n\n        # Explicit escape: after `--`, even a text equal to an existing r command\n        # is treated as a plain Gemini prompt.\n        if args and args[0] == "--":\n            return _plain_prompt(args[1:], root)\n\n        if not args or args[0] in {"help", "-h", "--help"}:''',
        "double-dash-dispatch",
    )
    source = replace_once(
        source,
        '''        if not args[0].startswith("-"):\n            known = {item["name"] for item in _robot_catalog()} | set(interface_cli.ALIASES)\n            if args[0] not in known:\n                return _unknown_command(args[0])\n        return interface_cli.main(args, project_root=root)''',
        '''        if not args[0].startswith("-"):\n            known = {item["name"] for item in _robot_catalog()} | set(interface_cli.ALIASES)\n            if args[0] not in known:\n                return _plain_prompt(args, root)\n        return interface_cli.main(args, project_root=root)''',
        "fallback-dispatch",
    )
    return source


def run(cmd: list[str], cwd: Path) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Install R2B4 plain LLM launcher upgrade")
    parser.add_argument("--root", default="/home/alba/project_r2b4")
    parser.add_argument("--skip-tests", action="store_true")
    args = parser.parse_args()

    package = Path(__file__).resolve().parent
    root = Path(args.root).expanduser().resolve()
    launcher = root / "v3" / "launcher_cli.py"
    plain_target = root / "r2b4_voice" / "plain_llm.py"
    test_target = root / "tests" / "feature" / "test_plain_llm_launcher.py"
    if not launcher.is_file() or not (root / "pytest.ini").is_file():
        raise RuntimeError(f"not an R2B4 repo root: {root}")

    current_blob = git_blob_sha(launcher)
    if current_blob != EXPECTED_LAUNCHER_GIT_BLOB:
        raise RuntimeError(
            "launcher baseline mismatch; upgrade was built source-first for "
            f"{BASE_COMMIT} / blob {EXPECTED_LAUNCHER_GIT_BLOB}, current blob is {current_blob}"
        )
    if plain_target.exists() or test_target.exists():
        raise RuntimeError("plain LLM upgrade target already exists; refusing to overwrite")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"plain_llm_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)
    shutil.copy2(launcher, backup / "launcher_cli.py")

    created: list[Path] = []
    try:
        patched = patch_launcher(launcher.read_text(encoding="utf-8"))
        launcher.write_text(patched, encoding="utf-8")

        plain_target.parent.mkdir(parents=True, exist_ok=True)
        test_target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(package / "files" / "r2b4_voice" / "plain_llm.py", plain_target)
        created.append(plain_target)
        shutil.copy2(package / "files" / "tests" / "feature" / "test_plain_llm_launcher.py", test_target)
        created.append(test_target)

        run([sys.executable, "-m", "py_compile", str(launcher), str(plain_target), str(test_target)], root)
        if not args.skip_tests:
            run([sys.executable, "-m", "pytest", "-q", "tests/feature/test_plain_llm_launcher.py"], root)
    except Exception:
        shutil.copy2(backup / "launcher_cli.py", launcher)
        for path in created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        raise

    print(f"PASS: plain LLM upgrade installed; backup: {backup}")
    print('Try: ./r "Miért kék az ég?"')
    print('Escape collision: ./r -- "s"')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
