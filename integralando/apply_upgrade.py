#!/usr/bin/env python3
"""Transactional installer for R2B4 canonical action-catalog P0/P1 upgrade."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

UPGRADE = "r2b4_action_catalog_p0_p1_20260923"
BASE_COMMIT = "6cfc56b69df84adcfe185f410d7d046f790877f9"
PACKAGE_ROOT = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_ROOT / "payload"

REPLACEMENTS = {
    "r2b4_voice/robot_context.py": "r2b4_voice/robot_context.py",
    "r2b4_voice/action_validation.py": "r2b4_voice/action_validation.py",
    "r2b4_voice/action_executor.py": "r2b4_voice/action_executor.py",
    "r2b4_voice/llm_decision.py": "r2b4_voice/llm_decision.py",
    "conf/voice_llm_system.md": "conf/voice_llm_system.md",
}
NEW_FILES = {
    "v3/action_catalog.py": "v3/action_catalog.py",
    "tests/test_action_catalog_ssot.py": "tests/test_action_catalog_ssot.py",
}
TRANSFORMED = (
    "v3/adapters/v3_control.py",
    "v3/robot_interface.py",
    "r2b4_voice/gemini_llm.py",
    "r2b4_voice/groq_llm.py",
    "r2b4_voice/conversation_service.py",
    "r2b4_voice/prompting.py",
    "tests/test_voice_action_executor.py",
)
EXISTING = tuple(REPLACEMENTS) + TRANSFORMED

TARGETED_TESTS = (
    "tests/test_action_catalog_ssot.py",
    "tests/test_voice_action_executor.py",
    "tests/test_r2b4_conversation_service.py",
    "tests/test_r2b4_groq_llm.py",
    "tests/test_r2b4_gemini_llm.py",
    "tests/test_r2b4_conversation_contracts.py",
    "tests/test_r2b4_conversation_interface.py",
    "tests/test_interface_p0_hardening.py",
    "tests/test_voice_hri_p0.py",
    "tests/test_r2b4_voice_conversation_service.py",
)

class UpgradeError(RuntimeError): pass

def run(root: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    proc = subprocess.run(args, cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if check and proc.returncode:
        raise UpgradeError(f"command failed ({' '.join(args)}): {proc.stderr.strip() or proc.stdout.strip()}")
    return proc

def git_blob(root: Path, path: str) -> str:
    return run(root, "git", "hash-object", path).stdout.strip()

def base_blob(root: Path, path: str) -> str:
    return run(root, "git", "rev-parse", f"{BASE_COMMIT}:{path}").stdout.strip()

def replace_once(text: str, old: str, new: str, path: str) -> str:
    count = text.count(old)
    if count != 1:
        raise UpgradeError(f"{path}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)

def transform_v3_control(text: str) -> str:
    path = "v3/adapters/v3_control.py"
    text = replace_once(text, "from collections.abc import Mapping\n\nfrom v3.capture_rate", "from collections.abc import Mapping\n\nfrom v3.action_catalog import ACTION_CATALOG, ActionDescriptor\nfrom v3.capture_rate", path)
    old_names = """    capability_names = frozenset({\n        "v3.status", "v3.pose", "v3.safety", "v3.health",\n        "v3.command.stop", "v3.command.forward", "v3.command.backward",\n        "v3.command.teleop", "v3.command.wheels", "v3.command.explore",\n        "v3.command.face_person", "v3.command.follow_person",\n    })"""
    new_names = """    capability_names = frozenset({"v3.status", "v3.pose", "v3.safety", "v3.health"}) | frozenset(ACTION_CATALOG)"""
    text = replace_once(text, old_names, new_names, path)
    start = text.find('        return {\n            "v3.status":')
    end_marker = '\n        }\n\n    @staticmethod\n    def _action'
    end = text.find(end_marker, start)
    if start < 0 or end < 0:
        raise UpgradeError(f"{path}: capability return block anchor missing")
    replacement = """        result: dict[str, dict[str, object]] = {\n            "v3.status": {\n                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,\n                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",\n            },\n            "v3.pose": {\n                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,\n                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",\n            },\n            "v3.safety": {\n                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,\n                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",\n            },\n            "v3.health": {\n                "kind": "read", "supported": True, "available": read_ready, "ready": read_ready,\n                "reason": None if read_ready else "NO_LIVE_RUNTIME_STATUS",\n            },\n        }\n        for name, descriptor in ACTION_CATALOG.items():\n            ready = runtime_running if name == "v3.command.stop" else True\n            reason = "STOP_IS_SAFE_NOOP_WHEN_IDLE" if name == "v3.command.stop" else current\n            result[name] = self._action(descriptor, True, ready, reason)\n        return result\n\n    @staticmethod\n    def _action"""
    text = text[:start] + replacement + text[end + len('\n        }\n\n    @staticmethod\n    def _action'):]
    old = """(available: bool, ready: bool, reason: str | None) -> dict[str, object]:\n        return {\n            "kind": "action",\n            "supported": True,\n            "available": available,\n            "ready": ready,\n            "reason": reason,\n        }"""
    new = """(descriptor: ActionDescriptor, available: bool, ready: bool, reason: str | None) -> dict[str, object]:\n        result = descriptor.to_jsonable()\n        result.update({\n            "kind": "action",\n            "supported": True,\n            "available": available,\n            "ready": ready,\n            "reason": reason,\n        })\n        return result"""
    text = replace_once(text, old, new, path)
    return text

def transform_robot_interface(text: str) -> str:
    path = "v3/robot_interface.py"
    text = replace_once(text, "from v3.interface_adapters import build_adapters", "from v3.action_catalog import ACTION_CATALOG_SCHEMA, action_catalog_jsonable\nfrom v3.interface_adapters import build_adapters", path)
    text = replace_once(text, 'ROBOT_INTERFACE_SCHEMA = "R2B4_ROBOT_INTERFACE_V1"', 'ROBOT_INTERFACE_SCHEMA = "R2B4_ROBOT_INTERFACE_V2"', path)
    text = replace_once(
        text,
        "    Capability data is generated from the live adapters on every request.  No\n    capability registry file or duplicated robot state is maintained here.",
        "    Live capability state is generated from adapters on every request. Static\n    canonical v3.command action contracts come from v3.action_catalog; no duplicated\n    live robot state is maintained here.",
        path,
    )
    old = """        return {\n            "schema": ROBOT_INTERFACE_SCHEMA,\n            "capabilities": dict(sorted(items.items())),\n        }"""
    new = """        return {\n            "schema": ROBOT_INTERFACE_SCHEMA,\n            "action_catalog_schema": ACTION_CATALOG_SCHEMA,\n            "action_catalog": action_catalog_jsonable(),\n            "capabilities": dict(sorted(items.items())),\n        }"""
    return replace_once(text, old, new, path)

def transform_provider(text: str, path: str, error_name: str) -> str:
    text = replace_once(text, "from .llm_decision import DECISION_SCHEMA, DecisionParseError, parse_llm_decision", "from .llm_decision import DECISION_SCHEMA, DecisionParseError, build_decision_schema, parse_llm_decision", path)
    sig = '    def complete(self, messages: Sequence[Mapping[str, str]]) -> LLMDecision:\n        if not messages:\n'
    repl = """    def complete(self, messages: Sequence[Mapping[str, str]]) -> LLMDecision:\n        return self._complete(messages, action_catalog=None)\n\n    def complete_with_actions(\n        self,\n        messages: Sequence[Mapping[str, str]],\n        action_catalog: Sequence[Mapping[str, object]],\n    ) -> LLMDecision:\n        return self._complete(messages, action_catalog=action_catalog)\n\n    def _complete(\n        self,\n        messages: Sequence[Mapping[str, str]],\n        *,\n        action_catalog: Sequence[Mapping[str, object]] | None,\n    ) -> LLMDecision:\n        if not messages:\n"""
    text = replace_once(text, sig, repl, path)
    text = replace_once(text, '"schema": DECISION_SCHEMA,', '"schema": build_decision_schema(action_catalog) if action_catalog is not None else DECISION_SCHEMA,', path)
    text = replace_once(text, 'return parse_llm_decision(decision_raw, model=self._config.model)', 'return parse_llm_decision(decision_raw, model=self._config.model, action_catalog=action_catalog)', path)
    return text

def transform_conversation_service(text: str) -> str:
    path = "r2b4_voice/conversation_service.py"
    old = '            decision = self._llm.complete(messages)'
    new = """            complete_with_actions = getattr(self._llm, "complete_with_actions", None)\n            if callable(complete_with_actions):\n                decision = complete_with_actions(messages, context.available_actions)\n            else:\n                # Compatibility for simple/fake LLM ports; production providers\n                # use the dynamic catalog-aware method above.\n                decision = self._llm.complete(messages)"""
    return replace_once(text, old, new, path)

def transform_prompting(text: str) -> str:
    return replace_once(text, 'PROMPT_VERSION = "R2B4_VOICE_LLM_SYSTEM_V3"', 'PROMPT_VERSION = "R2B4_VOICE_LLM_SYSTEM_V4"', "r2b4_voice/prompting.py")

def transform_executor_test(text: str) -> str:
    path = "tests/test_voice_action_executor.py"
    old = '    assert unknown.status == "REJECTED:ACTION_NOT_ALLOWLISTED"'
    new = '    assert unknown.status == "REJECTED:ACTION_NOT_ADVERTISED"'
    return replace_once(text, old, new, path)

TRANSFORMS = {
    "v3/adapters/v3_control.py": transform_v3_control,
    "v3/robot_interface.py": transform_robot_interface,
    "r2b4_voice/gemini_llm.py": lambda t: transform_provider(t, "r2b4_voice/gemini_llm.py", "GeminiRequestError"),
    "r2b4_voice/groq_llm.py": lambda t: transform_provider(t, "r2b4_voice/groq_llm.py", "LLMRequestError"),
    "r2b4_voice/conversation_service.py": transform_conversation_service,
    "r2b4_voice/prompting.py": transform_prompting,
    "tests/test_voice_action_executor.py": transform_executor_test,
}

def stage(root: Path) -> dict[str, bytes]:
    if not (root / ".git").exists(): raise UpgradeError(f"not a git repo: {root}")
    anc = run(root, "git", "merge-base", "--is-ancestor", BASE_COMMIT, "HEAD", check=False)
    if anc.returncode != 0: raise UpgradeError(f"expected base commit is not an ancestor of HEAD: {BASE_COMMIT}")
    for path in EXISTING:
        target = root / path
        if not target.is_file(): raise UpgradeError(f"missing source file: {path}")
        current = git_blob(root, path)
        expected = base_blob(root, path)
        if current != expected:
            raise UpgradeError(f"source drift before write: {path} (current {current}, expected base {expected})")
    for path in NEW_FILES:
        if (root / path).exists(): raise UpgradeError(f"new target already exists: {path}")

    staged: dict[str, bytes] = {}
    for target, payload_rel in REPLACEMENTS.items():
        staged[target] = (PAYLOAD / payload_rel).read_bytes()
    for target, payload_rel in NEW_FILES.items():
        staged[target] = (PAYLOAD / payload_rel).read_bytes()
    for target, fn in TRANSFORMS.items():
        source = (root / target).read_text(encoding="utf-8")
        staged[target] = fn(source).encode("utf-8")

    for path, data in staged.items():
        if path.endswith(".py"):
            try: ast.parse(data.decode("utf-8"), filename=path)
            except SyntaxError as exc: raise UpgradeError(f"staged syntax error {path}: {exc}") from exc
    return staged

def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data); handle.flush(); os.fsync(handle.fileno())
        os.replace(tmp, path)
    finally:
        try: os.unlink(tmp)
        except FileNotFoundError: pass

def restore(root: Path, backup: Path) -> None:
    manifest = json.loads((backup / "manifest.json").read_text(encoding="utf-8"))
    for path in manifest["original_files"]:
        atomic_write(root / path, (backup / "original" / path).read_bytes())
    for path in manifest["new_files"]:
        try: (root / path).unlink()
        except FileNotFoundError: pass

def install(root: Path, staged: dict[str, bytes], *, run_tests: bool) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / "runtime" / "upgrade_backups" / f"action_catalog_p0p1_{stamp}"
    for path in EXISTING:
        dst = backup / "original" / path
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / path, dst)
    manifest = {"upgrade": UPGRADE, "base_commit": BASE_COMMIT, "original_files": list(EXISTING), "new_files": list(NEW_FILES)}
    (backup / "manifest.json").parent.mkdir(parents=True, exist_ok=True)
    (backup / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    try:
        for path, data in staged.items(): atomic_write(root / path, data)
        if run_tests:
            cmd = [sys.executable, "-m", "pytest", "-q", *TARGETED_TESTS]
            proc = subprocess.run(cmd, cwd=root, text=True)
            if proc.returncode: raise UpgradeError(f"targeted pytest failed with exit {proc.returncode}")
    except BaseException:
        restore(root, backup)
        print(f"ROLLBACK restored from {backup}", file=sys.stderr)
        raise
    return backup

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="/home/alba/project_r2b4")
    parser.add_argument("--check", "--check-only", action="store_true", dest="check_only")
    parser.add_argument("--no-tests", action="store_true")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    try:
        staged = stage(root)
        if args.check_only:
            print(f"{UPGRADE}: CHECK OK; {len(staged)} files would be installed; no files written")
            return 0
        backup = install(root, staged, run_tests=not args.no_tests)
        print(f"{UPGRADE}: UPGRADE PASS")
        print(f"backup: {backup}")
        return 0
    except Exception as exc:
        print(f"{UPGRADE}: FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

if __name__ == "__main__": raise SystemExit(main())
