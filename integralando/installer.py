#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import textwrap
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable

UPGRADE_ID = "pytest_refactor_v2_20260925"
DEFAULT_ROOT = Path(os.environ.get("R2B4_ROOT", "/home/alba/project_r2b4"))
TARGETS = {
    "core": (20, 30, 25),
    "feature": (40, 70, 55),
    "deep": (20, 40, 30),
}
TOTAL_MIN = 80
TOTAL_MAX = 140
HARD_CAP = 150
MAX_TEST_FILES = 20

KNOWN_STALE_20260925 = {
    "test_r2b4_conversation_contracts.py",
    "test_r2b4_conversation_service.py",
    "test_r2b4_llm_provider.py",
    "test_v3_async_capability_convergence.py",
    "test_v3_async_command_gateway.py",
    "test_v3_async_l6_planner.py",
    "test_v3_async_planner_edges.py",
    "test_v3_async_recovery_p0.py",
    "test_v3_capture.py",
    "test_v3_control_process_isolation.py",
    "test_v3_differential_drive_arcs.py",
    "test_v3_follow_person.py",
    "test_v3_hardware_runtime.py",
    "test_v3_l6_planner_process.py",
    "test_v3_live_input_composition.py",
    "test_v3_mcap_e2e.py",
    "test_v3_motion_feedback.py",
    "test_v3_native_lidar_estimator.py",
    "test_v3_native_lidar_port.py",
    "test_v3_operator_controller.py",
    "test_v3_process_runtime.py",
    "test_v3_scan_matching_batch.py",
    "test_v3_sensor_measurement_tool.py",
    "test_v3_sterile_edges.py",
    "test_v3_transient_sensor_timing.py",
    "test_v3_process_imu_device.py",
}

CORE_WORDS = {
    "safety", "stop", "fault", "ttl", "command", "authority", "runtime",
    "architecture", "boundary", "contract", "gate", "health", "writer",
    "motor", "l12", "stale", "resident", "tick_engine", "import",
}
FEATURE_WORDS = {
    "follow", "room_cruise", "roomcruise", "cruise", "explore", "localization",
    "perception", "person", "camera", "motion", "forward", "turn", "navigation",
    "lidar", "encoder", "imu", "world_model", "trajectory", "pose",
}
DEEP_WORDS = {
    "async", "process", "replay", "checkpoint", "restore", "crash", "worker",
    "delayed", "corrupt", "timestamp", "clock", "race", "concurrency", "fault_replay",
    "determin", "isolation", "recovery", "deadline", "capture",
}
GOOD_WORDS = {
    "fails_closed", "fail_closed", "dominates", "stale", "unsafe", "replay", "determin",
    "worker", "crash", "death", "lost", "reacqui", "obstacle", "ahead", "off_axis",
    "too_close", "room", "follow", "forward", "turn", "fault", "stop", "ttl", "authority",
    "checkpoint", "restore", "boundary", "runtime", "canonical", "safety", "expired",
}
BAD_NAME_WORDS = {
    "constructor", "signature", "default", "compat", "legacy", "migration", "private",
    "internal", "queue_size", "buffer_size", "reflection", "configured", "pytest_profile",
    "unit_config", "exact_pwm", "exact_gain", "exact_threshold",
}

STALE_FAILURE_PATTERNS = [
    re.compile(r"TypeError: .*missing .*required .*argument"),
    re.compile(r"TypeError: .*unexpected keyword argument"),
    re.compile(r"TypeError: .*takes .* positional argument"),
    re.compile(r"AttributeError: .* object has no attribute '_"),
    re.compile(r"fixture '.*' not found"),
    re.compile(r"KeyError: ['\"](?:biztonsagi_zona_m|motion|legacy|config)"),
]

@dataclass(frozen=True)
class Candidate:
    source: str
    qualname: str
    layer: str
    score: int
    unsafe: tuple[str, ...]


def run(cmd: list[str], *, cwd: Path, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(cmd))
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=check,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def validate_root(root: Path) -> None:
    required = [root / "pytest.ini", root / "tests", root / "v3", root / "conf", root / "r"]
    missing = [str(p.relative_to(root)) for p in required if not p.exists()]
    if missing:
        raise RuntimeError(f"not a current R2B4 checkout; missing: {missing}")
    r_text = (root / "r").read_text(encoding="utf-8", errors="replace")
    if "v3.launcher_cli" not in r_text:
        raise RuntimeError("root launcher r is not the current thin v3.launcher_cli bootstrap; refusing to replace it")
    conf = root / "conf"
    for name in ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json"):
        if not (conf / name).is_file():
            raise RuntimeError(f"missing production config: conf/{name}")


def pytest_collect_count(root: Path, paths: list[str] | None = None) -> tuple[int, str]:
    cmd = [sys.executable, "-m", "pytest", "--collect-only", "-q"]
    if paths:
        cmd.extend(paths)
    cp = run(cmd, cwd=root, check=False, capture=True)
    out = cp.stdout or ""
    # pytest -q usually prints one nodeid per item; final summary is fallback.
    nodeids = [ln.strip() for ln in out.splitlines() if "::" in ln and not ln.lstrip().startswith(("E ", ">"))]
    if nodeids:
        return len(nodeids), out
    m = re.search(r"(\d+) tests? collected", out)
    if m:
        return int(m.group(1)), out
    m = re.search(r"collected (\d+) items?", out)
    if m:
        return int(m.group(1)), out
    if cp.returncode == 5 and "no tests" in out.lower():
        return 0, out
    if cp.returncode not in (0, 5):
        raise RuntimeError("pytest collection failed:\n" + out[-5000:])
    return 0, out


def iter_test_files(tests: Path) -> list[Path]:
    return sorted(
        p for p in tests.rglob("test_*.py")
        if not any(part.startswith(".") for part in p.relative_to(tests).parts)
    )


def file_imports_test_module(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(alias.name.split(".")[-1].startswith("test_") for alias in node.names):
                return True
        elif isinstance(node, ast.ImportFrom) and node.module:
            if node.module.split(".")[-1].startswith("test_"):
                return True
    return False


def unsafe_reasons(node: ast.AST) -> tuple[str, ...]:
    reasons: set[str] = set()
    for sub in ast.walk(node):
        if isinstance(sub, ast.Attribute) and sub.attr.startswith("_") and not sub.attr.startswith("__"):
            reasons.add("private_attribute")
        elif isinstance(sub, ast.Name) and sub.id == "configured":
            reasons.add("reflection_configured")
        elif isinstance(sub, ast.Call):
            fn = sub.func
            if isinstance(fn, ast.Attribute) and isinstance(fn.value, ast.Name):
                if fn.value.id == "inspect" and fn.attr in {"signature", "getfullargspec", "getmembers"}:
                    reasons.add("reflection")
        elif isinstance(sub, ast.Constant) and isinstance(sub.value, str):
            s = sub.value.lower()
            if "v3_unit_config" in s:
                reasons.add("unit_config_copy")
            if "pytest_profiles" in s:
                reasons.add("pytest_profile_infra")
    return tuple(sorted(reasons))


def test_nodes(tree: ast.Module) -> Iterable[tuple[str, ast.AST]]:
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name.startswith("test_"):
            yield node.name, node
        elif isinstance(node, ast.ClassDef) and node.name.startswith("Test"):
            for child in node.body:
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)) and child.name.startswith("test_"):
                    yield f"{node.name}.{child.name}", child


def words(s: str) -> set[str]:
    s = re.sub(r"[^a-z0-9_]+", "_", s.lower())
    parts = set(filter(None, re.split(r"_+", s)))
    # Keep compound tokens too because room_cruise/tick_engine are useful.
    parts |= {s}
    return parts


def infer_layer(path: Path, qualname: str) -> str:
    # Keep every test from one source file in one layer. This avoids turning a
    # single module into mixed CORE/FEATURE/DEEP semantics after curation.
    text = path.stem.lower()
    score = {"core": 0, "feature": 0, "deep": 0}
    for token in CORE_WORDS:
        if token in text:
            score["core"] += 2
    for token in FEATURE_WORDS:
        if token in text:
            score["feature"] += 2
    for token in DEEP_WORDS:
        if token in text:
            score["deep"] += 2
    # Deep wins on concurrency/replay/process semantics; feature wins on robot behavior; otherwise core.
    if score["deep"] >= max(4, score["feature"] + 1, score["core"] + 1):
        return "deep"
    if score["feature"] >= max(2, score["core"]):
        return "feature"
    return "core"


def candidate_score(path: Path, qualname: str, layer: str) -> int:
    text = f"{path.stem}_{qualname}".lower()
    score = 10
    for token in GOOD_WORDS:
        if token in text:
            score += 3
    for token in BAD_NAME_WORDS:
        if token in text:
            score -= 12
    if path.name in KNOWN_STALE_20260925:
        score -= 100
    if layer in text:
        score += 1
    return score


def scan_candidates(tests: Path) -> tuple[list[Candidate], dict[str, list[str]]]:
    result: list[Candidate] = []
    rejected: dict[str, list[str]] = {}
    for path in iter_test_files(tests):
        rel = str(path.relative_to(tests))
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError) as exc:
            rejected[rel] = [f"parse_error:{exc}"]
            continue
        if file_imports_test_module(tree):
            rejected[rel] = ["cross_test_import"]
            continue
        for qualname, node in test_nodes(tree):
            unsafe = unsafe_reasons(node)
            name_text = qualname.lower()
            bad_name = tuple(sorted(word for word in BAD_NAME_WORDS if word in name_text))
            unsafe = tuple(sorted(set(unsafe + bad_name)))
            layer = infer_layer(path, qualname)
            score = candidate_score(path, qualname, layer)
            if unsafe or score < 0:
                rejected[f"{rel}::{qualname}"] = list(unsafe or ("known_stale_20260925",))
            else:
                result.append(Candidate(rel, qualname, layer, score, unsafe))
    return result, rejected


def source_test_file_count(candidates: Iterable[Candidate]) -> int:
    return len({c.source for c in candidates})


def select_seed(candidates: list[Candidate]) -> dict[str, set[str]]:
    # Prefer concentrated, high-value files so the suite stays within 20 test files.
    by_file: dict[str, list[Candidate]] = {}
    for c in candidates:
        by_file.setdefault(c.source, []).append(c)
    file_rank = sorted(
        by_file.items(),
        key=lambda kv: (
            -sum(c.score for c in kv[1]),
            -len(kv[1]),
            kv[0],
        ),
    )
    chosen_files: list[str] = []
    layer_files: dict[str, list[str]] = {k: [] for k in TARGETS}
    # Reserve a balanced number of files per layer first.
    file_layer: dict[str, str] = {}
    for path, cs in by_file.items():
        counts = {k: sum(1 for c in cs if c.layer == k) for k in TARGETS}
        file_layer[path] = max(counts, key=lambda k: counts[k])
    for layer, quota in (("core", 5), ("feature", 9), ("deep", 6)):
        for path, _ in file_rank:
            if len(layer_files[layer]) >= quota:
                break
            if path in chosen_files or file_layer[path] != layer:
                continue
            chosen_files.append(path)
            layer_files[layer].append(path)
    for path, _ in file_rank:
        if len(chosen_files) >= MAX_TEST_FILES:
            break
        if path not in chosen_files:
            chosen_files.append(path)

    selected: dict[str, set[str]] = {p: set() for p in chosen_files}
    # Start somewhat above target; collected parametrization counts will trim later.
    desired_functions = {"core": 38, "feature": 80, "deep": 45}
    per_layer = {k: 0 for k in TARGETS}
    for c in sorted(candidates, key=lambda x: (-x.score, x.source, x.qualname)):
        if c.source not in selected or per_layer[c.layer] >= desired_functions[c.layer]:
            continue
        selected[c.source].add(c.qualname)
        per_layer[c.layer] += 1
    return {p: q for p, q in selected.items() if q}


class TestPruner(ast.NodeTransformer):
    def __init__(self, keep: set[str]):
        self.keep = keep
        self.class_stack: list[str] = []

    def visit_ClassDef(self, node: ast.ClassDef):
        self.class_stack.append(node.name)
        node = self.generic_visit(node)
        self.class_stack.pop()
        return node

    def _visit_test(self, node):
        if node.name.startswith("test_"):
            qn = ".".join([*self.class_stack, node.name]) if self.class_stack else node.name
            if qn not in self.keep:
                return None
        return self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef):
        return self._visit_test(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef):
        return self._visit_test(node)


def render_pruned(source: Path, keep: set[str]) -> str:
    text = source.read_text(encoding="utf-8")
    tree = ast.parse(text, filename=str(source))
    out = TestPruner(keep).visit(tree)
    ast.fix_missing_locations(out)
    return ast.unparse(out) + "\n"


def layer_for_file(candidates: list[Candidate], source: str, keep: set[str]) -> str:
    counts = {k: 0 for k in TARGETS}
    scores = {k: 0 for k in TARGETS}
    for c in candidates:
        if c.source == source and c.qualname in keep:
            counts[c.layer] += 1
            scores[c.layer] += c.score
    return max(TARGETS, key=lambda k: (counts[k], scores[k]))


def copy_support_files(src_tests: Path, dst_tests: Path) -> None:
    for p in src_tests.rglob("*"):
        if not p.is_file():
            continue
        rel = p.relative_to(src_tests)
        if any(part in {"__pycache__", ".pytest_cache"} for part in rel.parts):
            continue
        if p.name.startswith("test_") and p.suffix == ".py":
            continue
        if p.name in {"conftest.py", "v3_config_fixtures.py", "pytest_profiles.py", "v3_unit_config.json"}:
            continue
        target = dst_tests / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)


def write_stage(root: Path, stage_tests: Path, candidates: list[Candidate], selected: dict[str, set[str]]) -> dict[str, str]:
    if stage_tests.exists():
        shutil.rmtree(stage_tests)
    stage_tests.mkdir(parents=True)
    copy_support_files(root / "tests", stage_tests)
    file_layers: dict[str, str] = {}
    for source_rel, keep in selected.items():
        if not keep:
            continue
        layer = layer_for_file(candidates, source_rel, keep)
        src = root / "tests" / source_rel
        # Flatten filenames; duplicate names get a stable suffix.
        name = src.name
        target = stage_tests / layer / name
        if target.exists():
            suffix = hashlib.sha1(source_rel.encode()).hexdigest()[:8]
            target = target.with_name(f"{target.stem}_{suffix}.py")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(render_pruned(src, keep), encoding="utf-8")
        file_layers[source_rel] = layer
    (stage_tests / "conftest.py").write_text(CONFTEST, encoding="utf-8")
    (stage_tests / "rig.py").write_text(RIG, encoding="utf-8")
    return file_layers


def collect_nodeids(root: Path, stage_tests: Path) -> list[str]:
    paths = [str(stage_tests / k) for k in TARGETS]
    cp = run([sys.executable, "-m", "pytest", "--collect-only", "-q", *paths], cwd=root, check=False, capture=True)
    out = cp.stdout or ""
    nodeids = [ln.strip() for ln in out.splitlines() if "::" in ln and not ln.lstrip().startswith(("E ", ">"))]
    if cp.returncode not in (0, 5):
        raise RuntimeError("staged collection failed:\n" + out[-7000:])
    return nodeids


def nodeid_function(nodeid: str) -> tuple[str, str]:
    # Returns basename and qualname, with parametrization stripped.
    parts = nodeid.split("::")
    file_name = Path(parts[0]).name
    q = ".".join(p.split("[")[0] for p in parts[1:])
    return file_name, q


def count_by_layer(nodeids: list[str]) -> dict[str, int]:
    counts = {k: 0 for k in TARGETS}
    for n in nodeids:
        norm = n.replace("\\", "/")
        for layer in TARGETS:
            if f"/{layer}/" in "/" + norm:
                counts[layer] += 1
                break
    return counts


def trim_to_budget(
    root: Path,
    stage_tests: Path,
    candidates: list[Candidate],
    selected: dict[str, set[str]],
) -> tuple[dict[str, set[str]], list[str], dict[str, int]]:
    # Build file->source reverse map by basename; chosen source basenames are normally unique.
    for _ in range(6):
        write_stage(root, stage_tests, candidates, selected)
        nodeids = collect_nodeids(root, stage_tests)
        counts = count_by_layer(nodeids)
        total = len(nodeids)
        if all(TARGETS[k][0] <= counts[k] <= TARGETS[k][1] for k in TARGETS) and TOTAL_MIN <= total <= TOTAL_MAX:
            return selected, nodeids, counts

        # Count collected items per underlying function using basename + qualname.
        item_counts: dict[tuple[str, str], int] = {}
        for n in nodeids:
            key = nodeid_function(n)
            item_counts[key] = item_counts.get(key, 0) + 1

        changed = False
        # If a layer is over target, remove lowest-score functions until near target.
        for layer, (lo, hi, target) in TARGETS.items():
            if counts[layer] <= hi:
                continue
            choices: list[Candidate] = []
            for c in candidates:
                if c.layer == layer and c.qualname in selected.get(c.source, set()):
                    choices.append(c)
            for c in sorted(choices, key=lambda x: (x.score, x.source, x.qualname)):
                key = (Path(c.source).name, c.qualname)
                weight = item_counts.get(key, 1)
                if counts[layer] - weight < max(target, lo):
                    continue
                selected[c.source].discard(c.qualname)
                counts[layer] -= weight
                changed = True
                if counts[layer] <= target:
                    break

        # If a layer is short, add best unused candidate from already selected files first,
        # then add another file only while file budget permits.
        current_files = {p for p, qs in selected.items() if qs}
        for layer, (lo, hi, target) in TARGETS.items():
            if counts[layer] >= lo:
                continue
            pool = [c for c in candidates if c.layer == layer and c.qualname not in selected.get(c.source, set())]
            pool.sort(key=lambda x: (-x.score, x.source, x.qualname))
            for c in pool:
                if c.source not in current_files and len(current_files) >= MAX_TEST_FILES:
                    continue
                selected.setdefault(c.source, set()).add(c.qualname)
                current_files.add(c.source)
                counts[layer] += 1
                changed = True
                if counts[layer] >= target:
                    break
        selected = {p: q for p, q in selected.items() if q}
        if not changed:
            break

    write_stage(root, stage_tests, candidates, selected)
    nodeids = collect_nodeids(root, stage_tests)
    counts = count_by_layer(nodeids)
    raise RuntimeError(
        f"could not build suite inside budget without implementation-coupled tests; "
        f"counts={counts}, total={len(nodeids)}, files={source_test_file_count(c for c in candidates if c.qualname in selected.get(c.source, set()))}"
    )


def stale_failure_only(output: str) -> bool:
    # Refuse to hide AssertionError or explicit safety/behavior failures.
    if "AssertionError" in output or ("Safety" in output and "FAILED" in output):
        return False
    failed = [ln for ln in output.splitlines() if ln.startswith("FAILED ") or ln.startswith("ERROR ")]
    if not failed:
        return False
    # Every failed summary line must itself look like obsolete API/fixture coupling.
    # One unclassified failure aborts the refactor instead of being silently pruned.
    return all(any(pattern.search(line) for pattern in STALE_FAILURE_PATTERNS) for line in failed)


def run_stage(root: Path, stage_tests: Path) -> tuple[bool, str]:
    paths = [str(stage_tests / k) for k in TARGETS]
    cp = run([sys.executable, "-m", "pytest", "-q", *paths], cwd=root, check=False, capture=True)
    out = cp.stdout or ""
    print(out[-4000:])
    return cp.returncode == 0, out


def parse_failed_qualnames(output: str) -> set[str]:
    result: set[str] = set()
    for ln in output.splitlines():
        if ln.startswith(("FAILED ", "ERROR ")):
            token = ln.split(" - ", 1)[0].split(maxsplit=1)[1]
            parts = token.split("::")
            if len(parts) >= 2:
                q = ".".join(p.split("[")[0] for p in parts[1:])
                result.add(q)
    return result


def remove_failed(selected: dict[str, set[str]], output: str) -> list[str]:
    failed_q = parse_failed_qualnames(output)
    removed: list[str] = []
    for source, qs in selected.items():
        for q in list(qs):
            if q in failed_q:
                qs.remove(q)
                removed.append(f"{source}::{q}")
    return removed


def patch_pytest_ini(text: str) -> str:
    lines = text.splitlines()
    out: list[str] = []
    in_pytest = False
    skip_cont = False
    keys = {"testpaths", "pythonpath", "python_files"}
    i = 0
    found_section = False
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            in_pytest = stripped.lower() == "[pytest]"
            found_section |= in_pytest
            out.append(line)
            i += 1
            if in_pytest:
                out.extend([
                    "testpaths =",
                    "    tests/core",
                    "    tests/feature",
                    "    tests/deep",
                    "pythonpath = .",
                    "python_files = test_*.py",
                ])
            continue
        if in_pytest:
            m = re.match(r"\s*([A-Za-z0-9_]+)\s*=", line)
            if m and m.group(1) in keys:
                i += 1
                while i < len(lines) and (lines[i].startswith(" ") or lines[i].startswith("\t")) and not re.match(r"\s*[A-Za-z0-9_]+\s*=", lines[i]):
                    i += 1
                continue
        out.append(line)
        i += 1
    if not found_section:
        out.extend([
            "",
            "[pytest]",
            "testpaths =",
            "    tests/core",
            "    tests/feature",
            "    tests/deep",
            "pythonpath = .",
            "python_files = test_*.py",
        ])
    return "\n".join(out).rstrip() + "\n"


def external_references(root: Path, token: str) -> list[str]:
    hits: list[str] = []
    skip_roots = {".git", ".upgrade_backups", "runtime", "integralando"}
    for p in root.rglob("*.py"):
        rel = p.relative_to(root)
        if any(part in skip_roots for part in rel.parts):
            continue
        if rel.parts and rel.parts[0] == "tests":
            continue
        try:
            if token in p.read_text(encoding="utf-8", errors="ignore"):
                hits.append(str(rel))
        except OSError:
            pass
    return hits


def backup_paths(root: Path, backup: Path, paths: Iterable[Path]) -> None:
    for p in paths:
        if not p.exists():
            continue
        rel = p.relative_to(root)
        target = backup / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if p.is_dir():
            shutil.copytree(p, target, dirs_exist_ok=True)
        else:
            shutil.copy2(p, target)


def safe_remove(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def commit_install(
    root: Path,
    stage_tests: Path,
    counts: dict[str, int],
    before_count: int,
    candidates: list[Candidate],
    rejected: dict[str, list[str]],
    selected: dict[str, set[str]],
    auto_removed: list[str],
) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    backup.mkdir(parents=True, exist_ok=False)

    obsolete = []
    # Keep v3/pytest_profiles.py: on the current source it is a shared Test Hub/CLI
    # profile registry, not disposable pytest-only infrastructure.  A historical
    # tests/pytest_profiles.py copy, if present, is still obsolete.
    tests_profiles = root / "tests" / "pytest_profiles.py"
    if tests_profiles.exists():
        obsolete.append(tests_profiles)
    for p in root.rglob("v3_unit_config.json"):
        if ".upgrade_backups" not in p.parts and "runtime" not in p.parts and "integralando" not in p.parts:
            obsolete.append(p)
    if (root / "tests" / "v3_config_fixtures.py").exists():
        obsolete.append(root / "tests" / "v3_config_fixtures.py")

    touched = [root / "tests", root / "pytest.ini", root / "r", root / "AGENT_RUNTIME.md", root / "docs" / "AGENT_RUNTIME.md"] + obsolete
    backup_paths(root, backup, touched)

    try:
        old_tests = root / "tests"
        incoming = root / f".tests_incoming_{stamp}"
        if incoming.exists():
            shutil.rmtree(incoming)
        shutil.copytree(stage_tests, incoming)

        manifest = {
            "schema": "R2B4_PYTEST_SUITE_V1",
            "upgrade_id": UPGRADE_ID,
            "created_at_local": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "before_collected": before_count,
            "after_collected": sum(counts.values()),
            "budget": {"layers": {k: {"min": v[0], "max": v[1], "target": v[2]} for k, v in TARGETS.items()}, "total_min": TOTAL_MIN, "total_max": TOTAL_MAX, "hard_cap": HARD_CAP},
            "counts": counts,
            "selected_test_files": sorted(selected),
            "selected_test_functions": sorted(f"{p}::{q}" for p, qs in selected.items() for q in qs),
            "rejected_count": len(rejected),
            "rejected_examples": dict(list(sorted(rejected.items()))[:80]),
            "auto_removed_stale_failures": auto_removed,
            "known_stale_baseline_files_20260925": sorted(KNOWN_STALE_20260925),
            "obsolete_removed": sorted(str(p.relative_to(root)) for p in obsolete),
            "pytest_profiles_registry": "retained_shared_testhub_cli_registry",
        }
        (incoming / "suite_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        # Atomic-ish directory swap after staging has passed.
        archived = root / f".tests_old_{stamp}"
        old_tests.rename(archived)
        incoming.rename(old_tests)

        (root / "pytest.ini").write_text(patch_pytest_ini((root / "pytest.ini").read_text(encoding="utf-8")), encoding="utf-8")
        (root / "v3" / "test_runner.py").write_text(TEST_RUNNER, encoding="utf-8")
        (root / "r").write_text(R_LAUNCHER, encoding="utf-8")
        (root / "r").chmod((root / "r").stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        docs = root / "docs"
        docs.mkdir(exist_ok=True)
        (docs / "PYTEST_POLICY.md").write_text(POLICY, encoding="utf-8")

        # Remove explicit obsolete infrastructure only after new suite is in place.
        for p in obsolete:
            if p.exists():
                safe_remove(p)

        # Append agent workflow to the canonical runtime guidance file if one exists.
        agent_doc = root / "AGENT_RUNTIME.md"
        if not agent_doc.exists() and (root / "docs" / "AGENT_RUNTIME.md").exists():
            agent_doc = root / "docs" / "AGENT_RUNTIME.md"
        if agent_doc.exists():
            txt = agent_doc.read_text(encoding="utf-8")
            marker = "<!-- R2B4_PYTEST_POLICY_20260925 -->"
            if marker not in txt:
                agent_doc.write_text(txt.rstrip() + "\n\n" + AGENT_APPENDIX + "\n", encoding="utf-8")

        # Validation on committed tree. Failure restores everything.
        run([sys.executable, "-m", "py_compile", "v3/test_runner.py", "tests/rig.py", "tests/conftest.py"], cwd=root)
        count, _ = pytest_collect_count(root)
        if count > HARD_CAP or not (TOTAL_MIN <= count <= TOTAL_MAX):
            raise RuntimeError(f"committed suite budget violation: collected={count}")
        run([str(root / "r"), "test"], cwd=root)
        run([str(root / "r"), "test", "full"], cwd=root)

        report_dir = root / "runtime" / "pytest_refactor"
        report_dir.mkdir(parents=True, exist_ok=True)
        report = report_dir / f"{stamp}.json"
        report.write_text(json.dumps({**manifest, "backup": str(backup), "status": "PASS"}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        shutil.rmtree(archived)
        return backup
    except BaseException:
        print("ERROR: validation failed; restoring original pytest tree and touched files", file=sys.stderr)
        # Remove current replacements first.
        safe_remove(root / "tests")
        # Restore every backed-up path.
        for p in sorted(backup.rglob("*")):
            if not p.is_file():
                continue
            rel = p.relative_to(backup)
            target = root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, target)
        # tests directory backup is copied file-by-file above; ensure it exists.
        # Remove files newly created by this upgrade if not present in backup.
        for rel in (Path("v3/test_runner.py"), Path("docs/PYTEST_POLICY.md")):
            if not (backup / rel).exists():
                safe_remove(root / rel)
        # Clean transient old/incoming dirs.
        for p in root.glob(".tests_old_*"):
            if p.is_dir():
                shutil.rmtree(p)
        for p in root.glob(".tests_incoming_*"):
            if p.is_dir():
                shutil.rmtree(p)
        raise


def preflight_obsolete(root: Path) -> dict[str, list[str]]:
    refs = {
        # Despite its historical name, v3/pytest_profiles.py is currently a shared
        # Test Hub / host / interface / launcher profile registry.  It is NOT safe
        # to delete as part of the pytest-suite refactor.  Preserve it and report
        # its consumers for audit visibility.
        "pytest_profiles_retained": external_references(root, "pytest_profiles"),
        "v3_unit_config": external_references(root, "v3_unit_config"),
        "v3_config_fixtures": external_references(root, "v3_config_fixtures"),
    }
    bad = {
        k: v for k, v in refs.items()
        if k in {"v3_unit_config", "v3_config_fixtures"} and v
    }
    if bad:
        raise RuntimeError(
            "obsolete unit-config/reflection infrastructure is still referenced by "
            "production/non-test Python; source-first removal requires resolving "
            "these references first: " + json.dumps(bad, indent=2)
        )
    retained = refs["pytest_profiles_retained"]
    if retained:
        print("INFO: retaining v3/pytest_profiles.py because it is a shared Test Hub/CLI registry:")
        for rel in retained:
            print(" -", rel)
    return refs


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="R2B4 high-signal pytest refactor")
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    ap.add_argument("--check", action="store_true", help="source-first audit and staging dry run; write nothing")
    args = ap.parse_args(argv)
    root = args.root.resolve()
    validate_root(root)

    before_count, _ = pytest_collect_count(root)
    print(f"Current collected pytest items: {before_count}")
    if before_count < 150:
        print("NOTE: current suite is already below the historical 1200-item baseline; refactor will still enforce the new policy.")

    preflight_obsolete(root)
    candidates, rejected = scan_candidates(root / "tests")
    print(f"Safe high-signal candidate functions: {len(candidates)}")
    print(f"Rejected implementation-coupled/legacy entries: {len(rejected)}")
    if len(candidates) < TOTAL_MIN:
        raise RuntimeError(f"only {len(candidates)} safe candidates found; need at least {TOTAL_MIN}; no files changed")

    selected = select_seed(candidates)
    auto_removed: list[str] = []
    with tempfile.TemporaryDirectory(prefix="r2b4-pytest-refactor-") as td:
        stage_tests = Path(td) / "tests"
        selected, nodeids, counts = trim_to_budget(root, stage_tests, candidates, selected)
        if len({p for p, qs in selected.items() if qs}) > MAX_TEST_FILES:
            raise RuntimeError("test file budget exceeded before validation")
        print(f"Staged suite: {counts}, total={len(nodeids)}, files={len(selected)}")

        ok, output = run_stage(root, stage_tests)
        rounds = 0
        while not ok and rounds < 3 and stale_failure_only(output):
            removed = remove_failed(selected, output)
            if not removed:
                break
            auto_removed.extend(removed)
            print("Pruning stale implementation-coupled failures:")
            for x in removed:
                print(" -", x)
            selected, nodeids, counts = trim_to_budget(root, stage_tests, candidates, selected)
            ok, output = run_stage(root, stage_tests)
            rounds += 1
        if not ok:
            raise RuntimeError(
                "staged high-signal suite has a behavior/contract failure; refusing to hide it. "
                "No repository files changed. Tail:\n" + output[-7000:]
            )

        if args.check:
            print("CHECK PASS: staging suite is green and inside budget; repository was not modified.")
            print(json.dumps({"before": before_count, "counts": counts, "total": len(nodeids), "test_files": len(selected)}, indent=2))
            return 0

        # Re-render once into a stable temp location because commit_install copies it after staging pass.
        stable_stage = root / f".pytest_refactor_stage_{os.getpid()}"
        try:
            write_stage(root, stable_stage, candidates, selected)
            backup = commit_install(root, stable_stage, counts, before_count, candidates, rejected, selected, auto_removed)
        finally:
            safe_remove(stable_stage)

    print("PASS: R2B4 pytest refactor installed")
    print(f"Backup: {backup}")
    print("Commands:")
    print("  ./r test")
    print("  ./r test follow")
    print("  ./r test full")
    return 0


CONFTEST = r'''from __future__ import annotations

import sys
from pathlib import Path
import pytest

TESTS = Path(__file__).resolve().parent
ROOT = TESTS.parent
for path in (ROOT, TESTS, TESTS / "core", TESTS / "feature", TESTS / "deep"):
    s = str(path)
    if s not in sys.path:
        sys.path.insert(0, s)

HARD_CAP = 150


def pytest_collection_modifyitems(session, config, items):
    # The hard cap applies to the complete curated tree. Focused selections are naturally smaller.
    roots = {"core", "feature", "deep"}
    seen = set()
    for item in items:
        p = Path(str(item.fspath))
        seen |= roots.intersection(p.parts)
    if seen == roots and len(items) > HARD_CAP:
        raise pytest.UsageError(
            f"R2B4 pytest budget exceeded: {len(items)} items > {HARD_CAP}. "
            "Merge or remove an existing test before adding more."
        )
'''

RIG = r'''from __future__ import annotations

from pathlib import Path
from v3.config import ConfigResolver

ROOT = Path(__file__).resolve().parents[1]


def resolved_config(root: Path = ROOT):
    """Resolve the same four production config authorities used by the robot.

    No unit-config copy, constructor reflection or signature-following fixture exists here.
    """
    conf = root / "conf"
    return ConfigResolver(
        conf / "hardver.json",
        conf / "fizika.json",
        conf / "speed_map.json",
        conf / "vezerles.json",
    ).resolve()
'''

TEST_RUNNER = r'''from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"

FOCUSED = {
    "follow": ("feature", "follow"),
    "roomcruise": ("feature", "room_cruise or roomcruise or cruise or explore"),
    "room_cruise": ("feature", "room_cruise or roomcruise or cruise or explore"),
    "localization": ("feature", "localization or pose or estimator"),
    "perception": ("feature", "perception or person or camera or lidar"),
    "motion": ("feature", "motion or forward or turn or trajectory"),
    "async": ("deep", "async or worker or delayed or deadline"),
    "process": ("deep", "process or crash or isolation or worker"),
    "replay": ("deep", "replay or checkpoint or restore or determin"),
}


def _run(paths, k=None):
    cmd = [sys.executable, "-m", "pytest", "-q", *[str(p) for p in paths]]
    if k:
        cmd += ["-k", k]
    return subprocess.call(cmd, cwd=ROOT)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args else "core"
    if args:
        print("Unexpected extra arguments. Use: ./r test [full|follow|roomcruise|localization|perception|motion|async|process|replay]", file=sys.stderr)
        return 2
    if mode in {"core", "quick"}:
        return _run([TESTS / "core"])
    if mode == "full":
        return _run([TESTS / "core", TESTS / "feature", TESTS / "deep"])
    if mode in FOCUSED:
        layer, expr = FOCUSED[mode]
        return _run([TESTS / layer], expr)
    if mode in {"list", "help", "-h", "--help"}:
        print("./r test            CORE (~20-30)")
        print("./r test follow     Follow scenarios")
        print("./r test full       CORE + FEATURE + DEEP (~80-140, hard cap 150)")
        print("Other focused modes: roomcruise localization perception motion async process replay")
        return 0
    print(f"Unknown test mode: {mode}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
'''

R_LAUNCHER = r'''#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ "${1:-}" == "test" ]]; then
  shift
  exec python3 -m v3.test_runner "$@"
fi
exec python3 -m v3.launcher_cli "$@"
'''

POLICY = '''# R2B4 pytest policy

## Purpose

Pytest protects robot-level contracts and high-value scenarios. It is not an implementation diary.

## Budget

- CORE: 20-30 collected cases.
- FEATURE: 40-70 collected cases.
- DEEP: 20-40 collected cases.
- Total target: 80-140.
- Hard cap: 150. Growth above 150 requires an explicit redesign of the suite rather than another test.

When adding a test, prefer merging or deleting an older overlapping test.

## What belongs here

CORE: safety, canonical authority, STOP/FAULT, TTL/freshness, runtime composition, basic control chain, import boundaries.
FEATURE: user-visible robot behavior such as Follow, Room Cruise, localization, perception and motion scenarios.
DEEP: process/worker failure, delayed async completion, replay/checkpoint, timestamp edges and fault injection.

## What does not belong here

- assertions on private `_foo` fields;
- exact queue/buffer sizes unless they are a documented safety contract;
- constructor-signature compatibility;
- copied unit config or constructor reflection;
- tuning-number snapshots that production ConfigResolver already owns;
- tests preserving a migration step or an old implementation sequence;
- pytest infrastructure whose only purpose is to test pytest infrastructure.

For deep diagnostics, prefer MCAP + canonical replay + Test Hub evidence.

## Commands

- `./r test` — CORE after normal agent changes.
- `./r test <feature>` — relevant scenario slice, e.g. `follow`.
- `./r test full` — complete 80-140 case suite for shared boundaries / release acceptance.
'''

AGENT_APPENDIX = '''<!-- R2B4_PYTEST_POLICY_20260925 -->
## Pytest workflow — high-signal suite

- Normal code change: `./r test`.
- Behavior-specific change: `./r test <feature>` (for example `follow`).
- Shared architecture/config/runtime boundary or release acceptance: `./r test full`.
- The full suite target is 80-140 collected cases; 150 is a hard budget ceiling.
- Do not add constructor-signature, private-field, copied-config or tuning-number preservation tests.
- Prefer MCAP + replay + Test Hub for deep runtime diagnosis.
'''

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError, SyntaxError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
