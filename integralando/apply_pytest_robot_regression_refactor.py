#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

EXPECTED_HEAD = "00181897229c6857abb17fbd46891f8540d0b610"

EXPECTED_BLOBS = {
    "tests/core/test_openai_primary_provider.py": "97b391a085794dd05a5784c5bb97c1cdceb24d28",
    "tests/core/test_v3_planner_escape_deadlock.py": "21595f2b98062e596df2fe25dbf40feac5fc8e27",
    "tests/core/test_v3_config_p0_authority.py": "cc8bfc95d8dbbc7df831ac07673116b255cb03f0",
    "tests/core/test_v3_route_optimization_config.py": "5b754ee4b04bec659cc618cb1bf1c87f058108c1",
    "tests/core/test_agent_prompt_hierarchy.py": "cefeb785b37663d4ee18229f93f7edee014f4299",
    "tests/feature/test_v3_roomcruise_tuner.py": "e28e51d4084fd4f0cd2cf5828f28fe0fffac95e1",
    "tests/feature/test_tools_diag_evidence.py": "e56fe4c3e00d2da8be35c3e7e61be321f728807f",
    "tests/feature/test_tools_diag_persistence.py": "b54865f8985f62cfbe5d766e064adb99c2e80b64",
    "tests/feature/test_voice_orchestration_p0.py": "69b60ba4966fd5b75fb7569f37820a9839dee460",
    "docs/PYTEST_POLICY.md": "a94767deb2e55590fa144287171122867b457c27",
    "tests/README.md": "7e5b59fb64a7e983a997cfa8d0fcced34faadbac",
}

DELETE_FILES = {
    "tests/core/test_openai_primary_provider.py",
    "tests/core/test_v3_planner_escape_deadlock.py",
}

REMOVE_DEFS = {
    "tests/core/test_v3_route_optimization_config.py": {
        "_resolved",
        "_candidate",
        "test_speed_clearance_soft_barrier_penalizes_fast_tight_path_first",
        "test_l7_continuity_guard_does_not_keep_materially_tighter_near_best_path",
        "test_historical_capture_migration_disables_new_route_policy",
    },
    "tests/core/test_agent_prompt_hierarchy.py": {
        "_assert_embodied_observation_and_multistep_contract",
    },
    "tests/feature/test_v3_roomcruise_tuner.py": {
        "test_roomcruise_tuner_search_space_stays_inside_rollout_contract",
        "test_roomcruise_tuner_geometry_and_score_helpers_are_deterministic",
        "test_roomcruise_tuner_default_artifacts_live_under_runtime_tunes",
        "test_roomcruise_tuner_is_exposed_as_launcher_host_command",
        "test_roomcruise_tuner_direct_file_execution_can_import_v3",
    },
    "tests/feature/test_tools_diag_evidence.py": {
        "test_default_registry_is_explicit_full_diag_order",
        "test_tools_diag_has_no_mcap_reader_dependency",
        "test_localization_profiles_only_relevant_sensor_views",
        "test_launcher_routes_diag_to_tools_diag_and_evidence_help",
    },
    "tests/feature/test_tools_diag_persistence.py": {
        "test_launcher_surface_exposes_persistence_controls",
    },
    "tests/feature/test_voice_orchestration_p0.py": {
        "test_contract_defaults_are_robot_figyelek_and_ten_seconds",
    },
}

CONFIG_P0_REPLACEMENT = '''from __future__ import annotations
from copy import deepcopy
import json
from pathlib import Path

import pytest

from v3.config import ConfigResolver

ROOT = Path(__import__('os').environ['R2B4_ROOT']).resolve() if __import__('os').environ.get('R2B4_ROOT') else next((p for p in Path(__file__).resolve().parents if (p / 'conf' / 'hardver.json').is_file() and (p / 'v3').is_dir()), Path.cwd())


def _documents():
    conf = ROOT / 'conf'
    return tuple(
        json.loads((conf / name).read_text(encoding='utf-8'))
        for name in ('hardver.json', 'fizika.json', 'speed_map.json', 'vezerles.json')
    )


def test_camera_freshness_is_explicit_resolved_authority():
    hardware, physics, speed_map, control = _documents()
    expected = control['sensor_policy']['camera_maximum_frame_age_ns']
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    camera = resolved.runtime.sensor_inputs.inputs.camera_source
    assert camera is not None
    assert camera.maximum_frame_age_ns == expected

    # Missing physical freshness authority must fail before hardware opens.
    invalid = deepcopy(control)
    invalid['sensor_policy'].pop('camera_maximum_frame_age_ns')
    with pytest.raises(ValueError, match='camera_maximum_frame_age_ns'):
        ConfigResolver.from_documents(hardware, physics, speed_map, invalid)


def test_affinity_masks_are_normalized_and_keep_control_exclusive():
    hardware, physics, speed_map, control = _documents()
    control = deepcopy(control)
    control['runtime_affinity']['capture_cpus'] = [2, 1]
    control['runtime_affinity']['lidar_matcher_cpus'] = [1, 2]
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    affinity = resolved.affinity

    assert affinity.capture_cpus == tuple(sorted(control['runtime_affinity']['capture_cpus']))
    assert affinity.lidar_matcher_cpus == tuple(sorted(control['runtime_affinity']['lidar_matcher_cpus']))
    control_set = set(affinity.control_cpus)
    assert control_set
    for name, cpus in affinity.cpu_roles().items():
        if name != 'control_cpus':
            assert control_set.isdisjoint(cpus), (name, cpus)

    for field, invalid_value in (
        ('capture_cpus', []), ('capture_cpus', [1, 1]), ('capture_cpus', [-1]),
        ('capture_cpus', [True]), ('capture_cpus', [1.5]), ('capture_cpus', ['1']),
        ('capture_cpus', 1), ('capture_cpus', [2, 3]), ('control_cpus', [2, 3]),
        ('voice_cpus', [3]), ('l0_imu_cpus', [3]),
    ):
        candidate = deepcopy(control)
        candidate['runtime_affinity'][field] = invalid_value
        with pytest.raises(ValueError):
            ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def test_l7_reversal_threshold_conflict_fails_closed():
    hardware, physics, speed_map, control = _documents()
    invalid = deepcopy(control)
    invalid['layers']['motion_selection']['reversal_min_omega_rad_s'] = (
        invalid['layers']['operational_constraints']['max_omega_rad_s'] + 0.1
    )
    with pytest.raises(ValueError, match='reversal threshold exceeds operational omega limit'):
        ConfigResolver.from_documents(hardware, physics, speed_map, invalid)
'''


def git_blob_sha(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode()
    return hashlib.sha1(header + data).hexdigest()


def current_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return None


def remove_top_level_defs(text: str, names: set[str]) -> str:
    tree = ast.parse(text)
    ranges: list[tuple[int, int]] = []
    found: set[str] = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in names:
            start = min([node.lineno] + [d.lineno for d in node.decorator_list]) - 1
            end = node.end_lineno or node.lineno
            ranges.append((start, end))
            found.add(node.name)
    missing = names - found
    if missing:
        raise RuntimeError(f"missing expected definitions: {sorted(missing)}")
    lines = text.splitlines(keepends=True)
    for start, end in sorted(ranges, reverse=True):
        while end < len(lines) and lines[end].strip() == "":
            end += 1
        del lines[start:end]
    return "".join(lines)


def normalize_blank_lines(text: str) -> str:
    while "\n\n\n\n" in text:
        text = text.replace("\n\n\n\n", "\n\n\n")
    return text.rstrip() + "\n"


def transform(relative: str, text: str) -> str:
    if relative == "tests/core/test_v3_config_p0_authority.py":
        return CONFIG_P0_REPLACEMENT

    if relative in REMOVE_DEFS:
        text = remove_top_level_defs(text, REMOVE_DEFS[relative])

    if relative == "tests/core/test_v3_route_optimization_config.py":
        text = text.replace("from v3.contracts import TrajectoryEvaluation, TrajectoryPose\n", "")
        text = text.replace("from v3.layers import l6_navigation as l6\n", "")
        text = text.replace("from v3.layers import l7_motion_selection as l7\n", "")
        text = text.replace("from v3.replay import _migrate_legacy_resolved_config_snapshot\n", "")

    elif relative == "tests/core/test_agent_prompt_hierarchy.py":
        text = text.replace(
            "def test_agent_prompt_hierarchy_and_observation_authority_contract(tmp_path: Path) -> None:\n",
            "def test_agent_prompt_hierarchy_contract(tmp_path: Path) -> None:\n",
        )
        text = text.replace("    _assert_embodied_observation_and_multistep_contract()\n", "")

    elif relative == "tests/feature/test_v3_roomcruise_tuner.py":
        for line in (
            "import subprocess\n",
            "import sys\n",
            "from pathlib import Path\n",
            "from v3 import host_cli\n",
        ):
            text = text.replace(line, "")

    elif relative == "tests/feature/test_tools_diag_evidence.py":
        text = text.replace(
            "from tools.diag.registry import AnalyzerRegistry, build_default_registry\n",
            "from tools.diag.registry import AnalyzerRegistry\n",
        )

    return normalize_blank_lines(text)


def append_policy(text: str) -> str:
    marker = "## Robot-level maintenance rule"
    if marker in text:
        return text
    addition = '''\n## Robot-level maintenance rule\n\nThe curated regression suite protects externally meaningful robot contracts and failure boundaries, not implementation history. Prefer one public-path scenario that crosses the owning boundary over several tests of private helpers. Provider/model defaults, CLI help/completion text, artifact filenames, private `_foo` helpers, and retired config/capture migration steps are not regression contracts unless they are explicitly declared supported compatibility surfaces.\n\nWhen a lower-level unit assertion duplicates a public L5-L12, sensor-to-state, host-to-runtime, replay, or evidence scenario, keep the public scenario and remove the duplicate. Historical replay compatibility that must remain supported requires an explicit named replay fixture/evidence artifact; otherwise it does not stay in the permanent pytest gate.\n'''
    return text.rstrip() + addition + "\n"


def append_readme(text: str) -> str:
    marker = "## Karbantartási elv"
    if marker in text:
        return text
    addition = '''\n## Karbantartási elv\n\nA regressziós suite robot-szintű szerződéseket véd. Nem tartunk külön pytestet csak azért, hogy egy privát helper, CLI-help szöveg, fájlnév, provider/model default vagy lezárt migrációs lépés változatlan maradjon. Ha ugyanazt a kockázatot egy publikus L5-L12, sensor→state, host→runtime, replay vagy evidence scenario már bizonyítja, az alacsonyabb szintű duplikátum törlendő.\n'''
    return text.rstrip() + addition + "\n"


def write_backup(root: Path, backup_root: Path, relative: str) -> None:
    source = root / relative
    if not source.exists():
        return
    target = backup_root / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def verify_sources(root: Path, allow_head_mismatch: bool) -> None:
    head = current_head(root)
    if head != EXPECTED_HEAD and not allow_head_mismatch:
        raise RuntimeError(
            f"HEAD mismatch: expected {EXPECTED_HEAD}, got {head or 'UNKNOWN'}. "
            "Refresh the package or pass --allow-head-mismatch only after reviewing the diff."
        )
    mismatches = []
    for relative, expected in EXPECTED_BLOBS.items():
        path = root / relative
        if not path.is_file():
            mismatches.append(f"{relative}: missing")
            continue
        actual = git_blob_sha(path.read_bytes())
        if actual != expected:
            mismatches.append(f"{relative}: expected {expected}, got {actual}")
    if mismatches and not allow_head_mismatch:
        raise RuntimeError("source mismatch:\n  " + "\n  ".join(mismatches))
    if mismatches:
        print("WARNING: source mismatch accepted by --allow-head-mismatch:", file=sys.stderr)
        for row in mismatches:
            print(f"  {row}", file=sys.stderr)


def plan_changes(root: Path) -> dict[str, tuple[str, str | None]]:
    changes: dict[str, tuple[str, str | None]] = {}
    for relative in sorted(DELETE_FILES):
        if not (root / relative).exists():
            raise RuntimeError(f"expected file missing: {relative}")
        changes[relative] = ("DELETE", None)

    transform_files = set(REMOVE_DEFS) | {"tests/core/test_v3_config_p0_authority.py"}
    for relative in sorted(transform_files):
        path = root / relative
        original = path.read_text(encoding="utf-8")
        updated = transform(relative, original)
        if updated == original:
            raise RuntimeError(f"no change produced for {relative}")
        ast.parse(updated)
        changes[relative] = ("CHANGE", updated)

    policy_path = root / "docs/PYTEST_POLICY.md"
    original = policy_path.read_text(encoding="utf-8")
    updated = append_policy(original)
    if updated != original:
        changes["docs/PYTEST_POLICY.md"] = ("CHANGE", updated)

    readme_path = root / "tests/README.md"
    original = readme_path.read_text(encoding="utf-8")
    updated = append_readme(original)
    if updated != original:
        changes["tests/README.md"] = ("CHANGE", updated)

    return changes


def apply(root: Path, *, dry_run: bool, allow_head_mismatch: bool, verify: bool) -> int:
    verify_sources(root, allow_head_mismatch)
    changes = plan_changes(root)
    for relative, (kind, _) in changes.items():
        print(f"{kind} {relative}")
    if dry_run:
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_root = root / ".upgrade_backups" / f"pytest_robot_regression_{stamp}"
    for relative in changes:
        write_backup(root, backup_root, relative)

    for relative, (kind, content) in changes.items():
        path = root / relative
        if kind == "DELETE":
            path.unlink()
        else:
            assert content is not None
            path.write_text(content, encoding="utf-8")

    manifest = {
        "upgrade": "pytest_robot_regression_refactor",
        "source_head": current_head(root),
        "expected_head": EXPECTED_HEAD,
        "backup": str(backup_root.relative_to(root)),
        "changes": {k: v[0] for k, v in changes.items()},
    }
    (backup_root / "APPLY_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    py_files = [str(root / rel) for rel, (kind, _) in changes.items() if kind == "CHANGE" and rel.endswith(".py")]
    if py_files:
        subprocess.run([sys.executable, "-m", "py_compile", *py_files], cwd=root, check=True)

    if verify:
        env = os.environ.copy()
        env.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
        commands = [
            [sys.executable, "-m", "pytest", "-q", "--tb=short", "--collect-only", "tests/core", "tests/feature", "tests/deep"],
            [str(root / "r"), "test"],
            [str(root / "r"), "test", "roomcruise"],
        ]
        for command in commands:
            print("VERIFY", " ".join(command))
            subprocess.run(command, cwd=root, env=env, check=True)

    print(f"backup: {backup_root}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Refactor R2B4 pytest into a lower-maintenance robot-level regression suite.")
    parser.add_argument("--root", default="/home/alba/project_r2b4", help="R2B4 repository root")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    parser.add_argument("--verify", action="store_true", help="Run collect-only, quick gate and RoomCruise validation after apply")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    try:
        return apply(root, dry_run=args.dry_run, allow_head_mismatch=args.allow_head_mismatch, verify=args.verify)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
