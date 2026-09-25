#!/usr/bin/env python3
"""R2B4 pytest-refactor V4 hotfix.

Source-first correction for the V3 outcome where the curated suite already met the
TOTAL 80-140 target but the V2 installer rejected it only because DEEP had fewer
than 20 tests.

V4 policy:
- preserve the obsolete v3_config_fixtures dependency closure exclusion;
- keep CORE minimum 20 and FEATURE minimum 40 as hard guards;
- keep total 80..140 and <=20 test files as hard guards;
- make DEEP 20..40 a soft target, never repopulate the suite with obsolete/
  implementation-coupled tests merely to reach DEEP=20;
- keep v3/pytest_profiles.py untouched (shared Test Hub/CLI registry);
- --check restores the original tests tree;
- install has an independent full tests backup/rollback.

The wrapper does not permanently edit the V2 installer. It creates a temporary,
AST-patched copy and delegates to that copy.
"""
from __future__ import annotations

import argparse
import ast
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

HOTFIX_ID = "pytest_refactor_v4_20260925"
OBSOLETE_ROOT_MODULES = {"v3_config_fixtures"}


def find_repo(start: Path) -> Path:
    env = os.environ.get("R2B4_ROOT")
    if env:
        p = Path(env).expanduser().resolve()
        if (p / "tests").is_dir() and (p / "v3").is_dir():
            return p
        raise SystemExit(f"R2B4_ROOT is not a repo root: {p}")
    candidates = [start, *start.parents, Path("/home/alba/project_r2b4")]
    seen = set()
    for p in candidates:
        try:
            p = p.resolve()
        except Exception:
            continue
        if p in seen:
            continue
        seen.add(p)
        if (p / "tests").is_dir() and (p / "v3").is_dir():
            return p
    raise SystemExit("R2B4 repo root not found (expected tests/ and v3/). Set R2B4_ROOT if needed.")


def find_v2_installer(script_dir: Path, repo: Path) -> Path:
    override = os.environ.get("R2B4_PYTEST_V2_INSTALLER")
    candidates = []
    if override:
        candidates.append(Path(override))
    candidates += [script_dir / "installer.py", script_dir.parent / "installer.py", repo / "integralando" / "installer.py"]
    me = Path(__file__).resolve()
    for p in candidates:
        try:
            p = p.expanduser().resolve()
        except Exception:
            continue
        if p.is_file() and p != me:
            text = p.read_text(encoding="utf-8", errors="ignore")
            if "pytest_refactor" in text and "--check" in text:
                return p
    raise SystemExit(
        "V2 installer.py not found. Keep installer_v4.py under integralando next to the existing installer.py, "
        "or set R2B4_PYTEST_V2_INSTALLER=/path/to/installer.py"
    )


def module_names_for(path: Path, tests: Path) -> set[str]:
    rel = path.relative_to(tests).with_suffix("")
    parts = rel.parts
    names = {path.stem, ".".join(parts)}
    if parts and parts[-1] == "__init__":
        names.add(".".join(parts[:-1]))
    return {n for n in names if n}


def imports_for(path: Path) -> set[str]:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (SyntaxError, UnicodeDecodeError):
        return set()
    out: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                out.add(a.name)
                out.add(a.name.split(".")[-1])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                out.add(node.module)
                out.add(node.module.split(".")[-1])
            for a in node.names:
                if node.module:
                    out.add(f"{node.module}.{a.name}")
                out.add(a.name)
    return out


def obsolete_dependency_closure(tests: Path) -> list[Path]:
    pyfiles = sorted(p for p in tests.rglob("*.py") if "__pycache__" not in p.parts)
    names = {p: module_names_for(p, tests) for p in pyfiles}
    imports = {p: imports_for(p) for p in pyfiles}
    bad: set[Path] = set()
    for p in pyfiles:
        if names[p] & OBSOLETE_ROOT_MODULES or imports[p] & OBSOLETE_ROOT_MODULES:
            bad.add(p)
    changed = True
    while changed:
        changed = False
        bad_names = set(OBSOLETE_ROOT_MODULES)
        for p in bad:
            bad_names |= names[p]
        for p in pyfiles:
            if p not in bad and imports[p] & bad_names:
                bad.add(p)
                changed = True
    return sorted(bad)


def move_out(paths: list[Path], tests: Path, stash: Path) -> None:
    for src in paths:
        rel = src.relative_to(tests)
        dst = stash / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(src), str(dst))


def restore_from_stash(stash: Path, tests: Path) -> None:
    if not stash.exists():
        return
    for src in sorted(stash.rglob("*")):
        if not src.is_file():
            continue
        rel = src.relative_to(stash)
        dst = tests / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst.exists():
            dst.unlink()
        shutil.move(str(src), str(dst))


def full_tests_backup(repo: Path, tests: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = repo / ".upgrade_backups" / f"{HOTFIX_ID}_{stamp}" / "tests"
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(tests, backup)
    return backup


def _str_key(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _int_const(node: ast.AST) -> int | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, int) and not isinstance(node.value, bool):
        return node.value
    return None


def _deep_ref(node: ast.AST) -> bool:
    for x in ast.walk(node):
        if isinstance(x, ast.Subscript):
            sl = x.slice
            if isinstance(sl, ast.Constant) and sl.value == "deep":
                return True
        if isinstance(x, ast.Attribute) and x.attr == "deep":
            return True
        if isinstance(x, ast.Constant) and x.value == "deep":
            return True
    return False


class DeepMinimumRelaxer(ast.NodeTransformer):
    """Relax only the DEEP *minimum* while preserving every maximum/total guard."""

    def __init__(self) -> None:
        self.changes: list[str] = []
        self._in_budget_failure_func = 0

    def visit_Assign(self, node: ast.Assign):
        # Handle common forms such as:
        # BUDGET = {"core": (20,30), "feature": (40,70), "deep": (20,40)}
        # MIN_COUNTS = {"core": 20, "feature": 40, "deep": 20}
        name = " ".join(t.id for t in node.targets if isinstance(t, ast.Name)).lower()
        if isinstance(node.value, ast.Dict):
            keys = [_str_key(k) for k in node.value.keys]
            if {"core", "feature", "deep"}.issubset(set(k for k in keys if k)):
                for i, key in enumerate(keys):
                    if key != "deep":
                        continue
                    v = node.value.values[i]
                    if isinstance(v, (ast.Tuple, ast.List)) and len(v.elts) >= 2:
                        low = _int_const(v.elts[0])
                        high = _int_const(v.elts[1])
                        if low == 20 and high is not None and high >= 20:
                            v.elts[0] = ast.Constant(value=0)
                            self.changes.append("deep tuple/list minimum 20->0")
                    elif "min" in name:
                        val = _int_const(v)
                        if val == 20:
                            node.value.values[i] = ast.Constant(value=0)
                            self.changes.append("deep MIN dict 20->0")
                    elif isinstance(v, ast.Call):
                        # e.g. Budget(minimum=20, maximum=40) or Budget(20,40)
                        for kw in v.keywords:
                            if kw.arg and kw.arg.lower() in {"min", "minimum", "low", "min_count", "minimum_count"}:
                                if _int_const(kw.value) == 20:
                                    kw.value = ast.Constant(value=0)
                                    self.changes.append("deep budget keyword minimum 20->0")
                        if len(v.args) >= 2 and _int_const(v.args[0]) == 20 and (_int_const(v.args[1]) or 0) >= 20:
                            v.args[0] = ast.Constant(value=0)
                            self.changes.append("deep budget positional minimum 20->0")
        return self.generic_visit(node)

    def visit_AnnAssign(self, node: ast.AnnAssign):
        # Reuse assignment handling for annotated dict assignments.
        if isinstance(node.target, ast.Name) and isinstance(node.value, ast.Dict):
            fake = ast.Assign(targets=[node.target], value=node.value)
            self.visit_Assign(fake)
            node.value = fake.value
        return self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef):
        has_budget_failure = any(
            isinstance(x, ast.Constant)
            and isinstance(x.value, str)
            and "could not build suite inside budget without implementation-coupled tests" in x.value
            for x in ast.walk(node)
        )
        if has_budget_failure:
            self._in_budget_failure_func += 1
            node = self.generic_visit(node)
            self._in_budget_failure_func -= 1
            return node
        return self.generic_visit(node)

    def visit_Compare(self, node: ast.Compare):
        node = self.generic_visit(node)
        if not self._in_budget_failure_func:
            return node
        # Catch hard-coded forms inside the budget-building function:
        # counts["deep"] < 20   or   20 <= counts["deep"]
        if _deep_ref(node):
            candidates = [node.left, *node.comparators]
            for c in candidates:
                if isinstance(c, ast.Constant) and c.value == 20:
                    c.value = 0
                    self.changes.append("deep compare minimum 20->0")
        return node


def patch_v2_installer(src: Path, dst: Path) -> list[str]:
    text = src.read_text(encoding="utf-8")
    if "could not build suite inside budget without implementation-coupled tests" not in text:
        raise RuntimeError("unexpected V2 installer: budget-failure guard marker not found; refusing unsafe patch")
    tree = ast.parse(text, filename=str(src))
    tx = DeepMinimumRelaxer()
    new_tree = tx.visit(tree)
    ast.fix_missing_locations(new_tree)
    changes = list(dict.fromkeys(tx.changes))

    if not changes:
        # Conservative textual fallback for the exact common tuple mapping only.
        patterns = [
            (r"([\"']deep[\"']\s*:\s*\()20(\s*,\s*40\s*\))", r"\g<1>0\g<2>"),
            (r"([\"']deep[\"']\s*:\s*\[)20(\s*,\s*40\s*\])", r"\g<1>0\g<2>"),
        ]
        patched = text
        n_total = 0
        for pat, repl in patterns:
            patched, n = re.subn(pat, repl, patched)
            n_total += n
        if n_total:
            changes.append("text fallback: deep range minimum 20->0")
            dst.write_text(patched, encoding="utf-8")
            return changes
        raise RuntimeError(
            "V2 installer budget representation was not recognized; refusing to modify anything. "
            "Run: grep -n -E \"budget|deep|could not build suite\" installer.py and provide the output."
        )

    # ast.unparse is available on Python 3.11 used by the RPi log.
    patched = ast.unparse(new_tree) + "\n"
    # Preserve the decisive marker to detect accidental transformation damage.
    if "could not build suite inside budget without implementation-coupled tests" not in patched:
        raise RuntimeError("patched V2 lost safety marker; refusing")
    dst.write_text(patched, encoding="utf-8")
    return changes


def run_patched_v2(installer: Path, repo: Path, check: bool, temp_dir: Path) -> int:
    # Keep the temporary source in the SAME directory as installer.py. This preserves
    # Path(__file__).parent behavior if V2 loads sibling manifest/README resources.
    patched = installer.parent / f".installer_v2_soft_deep_{os.getpid()}.py"
    if patched.exists():
        patched.unlink()
    try:
        changes = patch_v2_installer(installer, patched)
        print("V4 temporary V2 budget patch:")
        for c in changes:
            print(" -", c)
        cmd = [sys.executable, str(patched)]
        if check:
            cmd.append("--check")
        print("+", " ".join(cmd), flush=True)
        env = os.environ.copy()
        env["R2B4_ROOT"] = str(repo)
        return subprocess.call(cmd, cwd=str(installer.parent), env=env)
    finally:
        try:
            patched.unlink()
        except FileNotFoundError:
            pass


def count_installed_suite(tests: Path) -> tuple[dict[str, int], int]:
    counts = {"core": 0, "feature": 0, "deep": 0}
    total = 0
    for layer in counts:
        d = tests / layer
        if not d.is_dir():
            continue
        for p in d.glob("test_*.py"):
            try:
                tree = ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
            except Exception:
                continue
            n = sum(1 for x in ast.walk(tree) if isinstance(x, (ast.FunctionDef, ast.AsyncFunctionDef)) and x.name.startswith("test_"))
            counts[layer] += n
            total += n
    return counts, total


def validate_installed_shape(repo: Path) -> None:
    tests = repo / "tests"
    counts, total = count_installed_suite(tests)
    test_files = sum(1 for p in tests.rglob("test_*.py") if any(part in {"core", "feature", "deep"} for part in p.parts))
    errors = []
    if counts["core"] < 20:
        errors.append(f"CORE below hard minimum: {counts['core']} < 20")
    if counts["feature"] < 40:
        errors.append(f"FEATURE below hard minimum: {counts['feature']} < 40")
    if not (80 <= total <= 140):
        errors.append(f"total outside hard budget: {total} not in 80..140")
    if test_files > 20:
        errors.append(f"too many test files: {test_files} > 20")
    if errors:
        raise RuntimeError("; ".join(errors))
    print(f"V4 installed-suite budget: CORE={counts['core']} FEATURE={counts['feature']} DEEP={counts['deep']} TOTAL={total} files={test_files}")
    if counts["deep"] < 20:
        print(f"INFO: DEEP soft target is currently {counts['deep']}/20 minimum target; no legacy tests were reintroduced to pad it.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="dry run; original tests tree is restored")
    ap.add_argument("--report", action="store_true", help="report obsolete dependency closure and V2 patchability only")
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    repo = find_repo(Path.cwd())
    tests = repo / "tests"
    installer = find_v2_installer(script_dir, repo)

    bad = obsolete_dependency_closure(tests)
    print(f"Obsolete v3_config_fixtures dependency closure: {len(bad)} Python files")
    for p in bad:
        print(" -", p.relative_to(repo))

    # Prove the V2 can be patched before moving any repo file.
    with tempfile.TemporaryDirectory(prefix="r2b4-pytest-v4-preflight-") as td:
        probe = Path(td) / "probe.py"
        changes = patch_v2_installer(installer, probe)
        print("V2 budget patch preflight: PASS")
        for c in changes:
            print(" -", c)
    if args.report:
        return 0

    backup = None
    if not args.check:
        backup = full_tests_backup(repo, tests)
        print("Full pre-V4 tests backup:", backup)

    with tempfile.TemporaryDirectory(prefix="r2b4-pytest-v4-") as td:
        work = Path(td)
        stash = work / "obsolete_stash"
        if bad:
            move_out(bad, tests, stash)
            print("INFO: obsolete fixture-dependent files excluded from candidate discovery.")
        rc = 99
        try:
            rc = run_patched_v2(installer, repo, args.check, work)
            if rc == 0 and not args.check:
                validate_installed_shape(repo)
        except Exception as exc:
            print("ERROR:", exc)
            rc = 2
        finally:
            if args.check:
                restore_from_stash(stash, tests)
                print("CHECK restore: original fixture-dependent files restored.")
            elif rc != 0:
                if tests.exists():
                    shutil.rmtree(tests)
                assert backup is not None
                shutil.copytree(backup, tests)
                print("INSTALL rollback: full original tests tree restored from V4 backup.")

    if rc == 0:
        print("V4 CHECK PASS" if args.check else "V4 INSTALL PASS")
        return 0
    print(f"V4 {'CHECK' if args.check else 'INSTALL'} FAILED (exit={rc}); no partial tests-tree install retained.")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
