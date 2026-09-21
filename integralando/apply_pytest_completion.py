#!/usr/bin/env python3
"""Finish the partially applied R2B4 pytest/Test Hub refactor.

This completion patch is intentionally narrow and idempotent. It:
1) fixes the portable Test Hub pytest runner test using the real repo root,
2) inserts the pytest profile policy into AGENTS.md,
3) repairs integralando/apply_pytest_refactor.py so it no longer uses an
   ambiguous global cwd assertion anchor.

It does not modify production control code, runtime/capture data, or the V3
import-guard policy. It does not commit or push.
"""

from __future__ import annotations

import argparse
from pathlib import Path


POLICY = (
    "A pytest célzott scope-jainak egyetlen forrása a `v3/pytest_profiles.py`; "
    "ugyanezeket a profilokat használja a Test Hub. A `gate` legyen az első gyors kapu, "
    "majd a változás természetének megfelelő `contract`/`async`/`runtime`/`replay`/"
    "`control`/`perception` profil következzen. A `full` nem helyettesíti a célzott tesztet.\n\n"
)


def _replace_once(text: str, old: str, new: str, *, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def _function_span(text: str, function_name: str, *, label: str) -> tuple[int, int]:
    anchor = f"def {function_name}("
    start = text.find(anchor)
    if start < 0:
        raise RuntimeError(f"{label}: function {function_name} not found")
    end = text.find("\ndef ", start + len(anchor))
    if end < 0:
        end = len(text)
    return start, end


def _patch_portable_test(repo: Path) -> None:
    path = repo / "tests/test_v3_test_hub_portable.py"
    text = path.read_text(encoding="utf-8")
    start, end = _function_span(
        text,
        "test_pytest_is_a_test_hub_command_not_a_runtime_dependency",
        label=str(path),
    )
    block = text[start:end]

    old_call = '    result = portable.run_pytest(tmp_path, scope="testhub")\n'
    new_call = (
        '    project_root = Path(__file__).resolve().parents[1]\n'
        '    result = portable.run_pytest(project_root, scope="testhub")\n'
    )
    if old_call in block:
        block = _replace_once(block, old_call, new_call, label=f"{path}: target test call")
    elif new_call not in block:
        raise RuntimeError(f"{path}: target test call is neither old nor expected new form")

    old_cwd = '    assert calls[0][1]["cwd"] == tmp_path.resolve()\n'
    new_cwd = '    assert calls[0][1]["cwd"] == project_root\n'
    if old_cwd in block:
        block = _replace_once(block, old_cwd, new_cwd, label=f"{path}: target test cwd")
    elif new_cwd not in block:
        raise RuntimeError(f"{path}: target test cwd is neither old nor expected new form")

    text = text[:start] + block + text[end:]
    path.write_text(text, encoding="utf-8")


def _patch_agents(repo: Path) -> None:
    path = repo / "AGENTS.md"
    text = path.read_text(encoding="utf-8")
    if POLICY in text:
        return
    anchor = "A célzott teszt az alapértelmezett.\n\n"
    text = _replace_once(text, anchor, anchor + POLICY, label=str(path))
    path.write_text(text, encoding="utf-8")


def _patch_original_installer(repo: Path) -> None:
    path = repo / "integralando/apply_pytest_refactor.py"
    if not path.is_file():
        return
    text = path.read_text(encoding="utf-8")

    old = '''    portable = repo / "tests/test_v3_test_hub_portable.py"\n    text = portable.read_text(encoding="utf-8")\n    old_call = '    result = portable.run_pytest(tmp_path, scope="testhub")\\n'\n    new_call = '    project_root = Path(__file__).resolve().parents[1]\\n    result = portable.run_pytest(project_root, scope="testhub")\\n'\n    if old_call in text:\n        text = _replace_once(text, old_call, new_call, label=str(portable))\n    elif new_call not in text:\n        raise RuntimeError(f"{portable}: portable pytest runner call anchor not found")\n    old_cwd = '    assert calls[0][1]["cwd"] == tmp_path.resolve()\\n'\n    new_cwd = '    assert calls[0][1]["cwd"] == project_root\\n'\n    if old_cwd in text:\n        text = _replace_once(text, old_cwd, new_cwd, label=str(portable))\n    elif new_cwd not in text:\n        raise RuntimeError(f"{portable}: portable pytest cwd assertion anchor not found")\n    portable.write_text(text, encoding="utf-8")\n'''

    new = '''    portable = repo / "tests/test_v3_test_hub_portable.py"\n    text = portable.read_text(encoding="utf-8")\n    function_anchor = "def test_pytest_is_a_test_hub_command_not_a_runtime_dependency("\n    start = text.find(function_anchor)\n    if start < 0:\n        raise RuntimeError(f"{portable}: portable pytest runner test function not found")\n    end = text.find("\\ndef ", start + len(function_anchor))\n    if end < 0:\n        end = len(text)\n    block = text[start:end]\n\n    old_call = '    result = portable.run_pytest(tmp_path, scope="testhub")\\n'\n    new_call = '    project_root = Path(__file__).resolve().parents[1]\\n    result = portable.run_pytest(project_root, scope="testhub")\\n'\n    if old_call in block:\n        block = _replace_once(block, old_call, new_call, label=f"{portable}: target test call")\n    elif new_call not in block:\n        raise RuntimeError(f"{portable}: portable pytest runner call anchor not found")\n\n    old_cwd = '    assert calls[0][1]["cwd"] == tmp_path.resolve()\\n'\n    new_cwd = '    assert calls[0][1]["cwd"] == project_root\\n'\n    if old_cwd in block:\n        block = _replace_once(block, old_cwd, new_cwd, label=f"{portable}: target test cwd")\n    elif new_cwd not in block:\n        raise RuntimeError(f"{portable}: portable pytest cwd assertion anchor not found")\n\n    text = text[:start] + block + text[end:]\n    portable.write_text(text, encoding="utf-8")\n'''

    if old in text:
        text = _replace_once(text, old, new, label=str(path))
        path.write_text(text, encoding="utf-8")
        return

    # Already repaired: accept the scoped implementation. Anything else is a
    # changed installer and must be reviewed instead of guessed at.
    if (
        'function_anchor = "def test_pytest_is_a_test_hub_command_not_a_runtime_dependency("' in text
        and 'label=f"{portable}: target test cwd"' in text
    ):
        return
    raise RuntimeError(f"{path}: expected old or repaired portable-test patch block not found")


def _assert_refactor_core(repo: Path) -> None:
    required = (
        repo / "AGENTS.md",
        repo / "pytest.ini",
        repo / "v3/pytest_profiles.py",
        repo / "v3/test_hub_portable.py",
        repo / "v3/test_hub_next.py",
        repo / "tests/conftest.py",
        repo / "tests/test_v3_pytest_profiles.py",
        repo / "tests/test_v3_test_hub_portable.py",
    )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError(
            "pytest refactor core is not present; missing: " + ", ".join(missing)
        )


def apply(repo: Path) -> None:
    repo = repo.resolve()
    _assert_refactor_core(repo)
    _patch_portable_test(repo)
    _patch_agents(repo)
    _patch_original_installer(repo)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".", help="R2B4 repository root")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    apply(repo)
    print("R2B4 pytest completion patch applied. No commit or push was performed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
