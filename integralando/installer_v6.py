#!/usr/bin/env python3
"""R2B4 pytest refactor V6 recovery/fix.

Fixes the V4/V5 false rollback caused by counting Python test function
*definitions* instead of pytest-collected test *items*. Parameterized tests make
those numbers different (real R2B4 evidence: 70 definitions vs 83 pytest items).

V6 also repairs the only plausible partial-install residue from the failed V5
chain before retrying: V2 may already have changed `r` and `v3/test_runner.py`
while V4/V5 guaranteed rollback only for `tests/`. If the latest V2 backup
contains those files, V6 restores them first, then runs V5 against a temporary
V4 copy whose AST-based TOTAL lower-bound check is disabled. V6 itself performs
the authoritative budget check with pytest collection counts.

Hard acceptance after install:
- core pytest items >= 20
- feature pytest items >= 40
- total pytest items 80..140
- <= 20 curated test files
- full curated pytest PASS
- ./r test PASS
- ./r test full PASS
DEEP 20..40 remains a soft target.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

UPGRADE_ID = "pytest_refactor_v6_20260925"


def find_repo(start: Path) -> Path:
    env = os.environ.get("R2B4_ROOT")
    if env:
        p = Path(env).expanduser().resolve()
        if (p / "tests").is_dir() and (p / "v3").is_dir() and (p / "conf").is_dir():
            return p
        raise SystemExit(f"R2B4_ROOT is not a repo root: {p}")
    for p in [start, *start.parents, Path("/home/alba/project_r2b4")]:
        try:
            p = p.resolve()
        except Exception:
            continue
        if (p / "tests").is_dir() and (p / "v3").is_dir() and (p / "conf").is_dir():
            return p
    raise SystemExit("R2B4 repo root not found")


def find_existing(script_dir: Path, repo: Path, name: str, env_name: str) -> Path:
    candidates = []
    if os.environ.get(env_name):
        candidates.append(Path(os.environ[env_name]))
    candidates += [script_dir / name, script_dir.parent / name, repo / "integralando" / name]
    for p in candidates:
        try:
            p = p.expanduser().resolve()
        except Exception:
            continue
        if p.is_file() and p != Path(__file__).resolve():
            return p
    raise SystemExit(f"{name} not found; keep V6 under integralando next to existing {name}")


def config_preflight(repo: Path) -> None:
    required = ["hardver.json", "fizika.json", "speed_map.json", "vezerles.json"]
    bad = []
    for name in required:
        p = repo / "conf" / name
        if p.is_symlink() or not p.is_file():
            bad.append(str(p))
    if bad:
        raise RuntimeError("canonical production config missing/non-regular: " + ", ".join(bad))
    print("Production config preflight: PASS")


class Snapshot:
    """Independent rollback for files/dirs V2/V5 are known to touch."""
    def __init__(self, repo: Path):
        self.repo = repo
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.root = repo / ".upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
        self.root.mkdir(parents=True, exist_ok=True)
        self.paths = [Path("tests"), Path("r"), Path("v3/test_runner.py"), Path("pytest.ini")]
        self.present: set[Path] = set()
        for rel in self.paths:
            src = repo / rel
            if not src.exists() and not src.is_symlink():
                continue
            self.present.add(rel)
            dst = self.root / "snapshot" / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            if src.is_dir():
                shutil.copytree(src, dst, symlinks=True)
            else:
                shutil.copy2(src, dst, follow_symlinks=False)
        # Preserve any unit-config file regardless of its exact historical location.
        self.unit_configs = []
        for p in repo.rglob("v3_unit_config.json"):
            if ".upgrade_backups" in p.parts or "integralando" in p.parts:
                continue
            rel = p.relative_to(repo)
            self.unit_configs.append(rel)
            dst = self.root / "snapshot" / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, dst, follow_symlinks=False)
        print("Full V6 rollback snapshot:", self.root)

    def restore(self) -> None:
        # Known mutable paths: restore original presence/absence exactly.
        for rel in self.paths:
            dst = self.repo / rel
            src = self.root / "snapshot" / rel
            if dst.is_dir() and not dst.is_symlink():
                shutil.rmtree(dst)
            elif dst.exists() or dst.is_symlink():
                dst.unlink()
            if rel in self.present:
                dst.parent.mkdir(parents=True, exist_ok=True)
                if src.is_dir():
                    shutil.copytree(src, dst, symlinks=True)
                else:
                    shutil.copy2(src, dst, follow_symlinks=False)
        # Remove any newly created unit config then restore pre-V6 copies.
        for p in list(self.repo.rglob("v3_unit_config.json")):
            if ".upgrade_backups" not in p.parts and "integralando" not in p.parts:
                try:
                    p.unlink()
                except OSError:
                    pass
        for rel in self.unit_configs:
            src = self.root / "snapshot" / rel
            dst = self.repo / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst, follow_symlinks=False)
        print("V6 rollback: pre-V6 mutable surfaces restored.")


def latest_v2_backup(repo: Path) -> Path | None:
    base = repo / ".upgrade_backups"
    if not base.is_dir():
        return None
    cands = [p for p in base.iterdir() if p.is_dir() and p.name.startswith("pytest_refactor_v2_20260925_")]
    return max(cands, key=lambda p: p.stat().st_mtime) if cands else None


def find_backup_file(backup: Path, rel: Path) -> Path | None:
    direct = backup / rel
    if direct.is_file():
        return direct
    suffix = rel.as_posix()
    cands = [p for p in backup.rglob(rel.name) if p.is_file() and p.as_posix().endswith(suffix)]
    if not cands:
        return None
    return min(cands, key=lambda p: len(p.parts))


def repair_partial_v2_residue(repo: Path) -> None:
    """Restore non-tests files V2 may have changed before V4/V5 rolled tests back."""
    b = latest_v2_backup(repo)
    if not b:
        print("Partial-state repair: no V2 backup found; nothing restored")
        return
    print("Partial-state repair source:", b)
    restored = 0
    for rel in (Path("r"), Path("v3/test_runner.py")):
        src = find_backup_file(b, rel)
        if not src:
            print(" - no backed-up", rel)
            continue
        dst = repo / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst, follow_symlinks=False)
        if rel == Path("r"):
            try:
                dst.chmod(src.stat().st_mode)
            except OSError:
                pass
        print(" - restored", rel)
        restored += 1
    if not restored:
        print("Partial-state repair: V2 backup had no known non-tests mutable files")


def patch_v4_total_lower_bound(src: Path, dst: Path) -> None:
    text = src.read_text(encoding="utf-8")
    marker = 'errors.append(f"total outside hard budget: {total} not in 80..140")'
    if marker not in text:
        # tolerate single quotes / formatting but refuse broad unsafe edits
        if "total outside hard budget" not in text or "80 <= total <= 140" not in text:
            raise RuntimeError("unexpected V4 validator shape; refusing patch")
    patched, n = re.subn(r"if\s+not\s*\(80\s*<=\s*total\s*<=\s*140\s*\):", "if not (0 <= total <= 140):", text, count=1)
    if n != 1:
        raise RuntimeError(f"expected exactly one V4 AST-total guard, patched {n}")
    dst.write_text(patched, encoding="utf-8")
    print("V4 temporary patch: AST function-count TOTAL minimum 80 -> 0; max 140 retained")
    print("Authoritative 80..140 check will use pytest-collected item counts after install")


def run_v5(v5: Path, patched_v4: Path, repo: Path, check: bool) -> int:
    cmd = [sys.executable, str(v5)]
    if check:
        cmd.append("--check")
    env = os.environ.copy()
    env["R2B4_ROOT"] = str(repo)
    env["R2B4_PYTEST_V4_INSTALLER"] = str(patched_v4)
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=str(v5.parent), env=env)


def collect_count(repo: Path, target: Path) -> int:
    cmd = [sys.executable, "-m", "pytest", "--collect-only", "-q", str(target)]
    p = subprocess.run(cmd, cwd=str(repo), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        print(p.stdout)
        raise RuntimeError(f"pytest collection failed for {target}")
    out = p.stdout
    # Common pytest summaries.
    matches = re.findall(r"(?:collected\s+(\d+)\s+items?|(?:(\d+)\s+tests?\s+collected))", out)
    if matches:
        a, b = matches[-1]
        return int(a or b)
    # -q may print only nodeids on some versions.
    nodeids = [ln.strip() for ln in out.splitlines() if "::" in ln and not ln.lstrip().startswith(("=", "_"))]
    if nodeids:
        return len(nodeids)
    raise RuntimeError(f"could not parse pytest collection count for {target}; output:\n{out[-2000:]}")


def curated_file_count(repo: Path) -> int:
    n = 0
    for layer in ("core", "feature", "deep"):
        d = repo / "tests" / layer
        if d.is_dir():
            n += sum(1 for _ in d.glob("test_*.py"))
    return n


def authoritative_validate(repo: Path) -> dict[str, int]:
    counts = {}
    for layer in ("core", "feature", "deep"):
        d = repo / "tests" / layer
        counts[layer] = collect_count(repo, d) if d.is_dir() else 0
    total = sum(counts.values())
    files = curated_file_count(repo)
    print(f"Authoritative pytest-item budget: CORE={counts['core']} FEATURE={counts['feature']} DEEP={counts['deep']} TOTAL={total} files={files}")
    errors = []
    if counts["core"] < 20:
        errors.append(f"CORE {counts['core']} < 20")
    if counts["feature"] < 40:
        errors.append(f"FEATURE {counts['feature']} < 40")
    if not (80 <= total <= 140):
        errors.append(f"TOTAL {total} outside 80..140")
    if files > 20:
        errors.append(f"test files {files} > 20")
    if errors:
        raise RuntimeError("; ".join(errors))
    if counts["deep"] < 20:
        print(f"INFO: DEEP soft target currently {counts['deep']}/20; no obsolete tests added for padding")
    return counts


def run_checked(cmd: list[str], cwd: Path, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(map(str, cmd)), flush=True)
    rc = subprocess.call([str(x) for x in cmd], cwd=str(cwd), env=env)
    if rc != 0:
        raise RuntimeError(f"command failed ({rc}): {' '.join(map(str, cmd))}")


def final_validation(repo: Path) -> None:
    authoritative_validate(repo)
    env = os.environ.copy(); env.pop("R2B4_ROOT", None)
    targets = [repo / "tests" / x for x in ("core", "feature", "deep") if (repo / "tests" / x).is_dir()]
    run_checked([sys.executable, "-m", "pytest", "-q", *map(str, targets)], repo, env)
    r = repo / "r"
    if not r.is_file():
        raise RuntimeError("./r missing after install")
    run_checked([str(r), "test"], repo, env)
    run_checked([str(r), "test", "full"], repo, env)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()

    repo = find_repo(Path.cwd())
    script_dir = Path(__file__).resolve().parent
    v5 = find_existing(script_dir, repo, "installer_v5.py", "R2B4_PYTEST_V5_INSTALLER")
    v4 = find_existing(script_dir, repo, "installer_v4.py", "R2B4_PYTEST_V4_INSTALLER_SOURCE")
    try:
        config_preflight(repo)
    except Exception as exc:
        print("ERROR:", exc)
        return 2

    if args.report:
        b = latest_v2_backup(repo)
        print("Latest V2 backup:", b or "NONE")
        print("Current collect command:")
        subprocess.call([sys.executable, "-m", "pytest", "--collect-only", "-q"], cwd=str(repo))
        return 0

    snap = None if args.check else Snapshot(repo)
    rc = 99
    temp_v4 = repo / "integralando" / f".installer_v4_pytest_items_{os.getpid()}.py"
    try:
        if not args.check:
            repair_partial_v2_residue(repo)
        patch_v4_total_lower_bound(v4, temp_v4)
        rc = run_v5(v5, temp_v4, repo, args.check)
        if rc != 0:
            raise RuntimeError(f"V5/V4/V2 chain failed with exit {rc}")
        if not args.check:
            final_validation(repo)
        print("V6 CHECK PASS" if args.check else "V6 INSTALL PASS")
        return 0
    except Exception as exc:
        print("ERROR:", exc)
        if snap is not None:
            snap.restore()
        return 2
    finally:
        try:
            temp_v4.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
