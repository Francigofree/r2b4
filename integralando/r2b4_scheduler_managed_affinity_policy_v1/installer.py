#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

UPGRADE_ID = "scheduler_managed_affinity_policy_v1_20260927"
BASE_COMMIT = "1333b49a2917f46753249f73599b6f7a98dc4f42"

EXPECTED_BLOBS = {
    "README.md": "0d882150cf554a12a65b7d5080d4a5222b3a0e04",
    "tests/core/test_v3_config_p0_authority.py": "181acb531de351cb92deb0a85d500dbcba8625d2",
    "tests/deep/test_v3_process_affinity.py": "8b7d314e6f8b5ba6c03dbbfe7e6b114ccef6f58a",
    "tools/v3_performance_audit.py": "b0c214c06b6b231e8a4afbdc91446d4371ce1004",
    "tools/v3_p0_async_acceptance.py": "f9bc9dda9302ad8995f6565f5ec0d71da4389ded",
}

README_MARKER = "SCHEDULER_MANAGED_AFFINITY_POLICY_V1"


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def project_root(explicit: str | None) -> Path:
    if explicit:
        root = Path(explicit).expanduser().resolve()
    elif os.environ.get("R2B4_ROOT"):
        root = Path(os.environ["R2B4_ROOT"]).expanduser().resolve()
    else:
        here = Path(__file__).resolve()
        candidates = [here.parent, here.parent.parent, Path("/home/alba/project_r2b4")]
        root = next(
            (p.resolve() for p in candidates if (p / "v3").is_dir() and (p / "conf").is_dir()),
            candidates[-1].resolve(),
        )
    if not (root / "v3").is_dir() or not (root / "conf" / "vezerles.json").is_file():
        raise SystemExit(f"ERROR: invalid R2B4 root: {root}")
    return root


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def patch_readme(text: str) -> str:
    if README_MARKER in text:
        return text
    start = text.index("## CPU-kiosztás\n")
    end = text.index("## Használat\n", start)
    new = """## CPU-ütemezés

<!-- SCHEDULER_MANAGED_AFFINITY_POLICY_V1 -->
A `conf/vezerles.json` `runtime_affinity` szekciója két üzemmódot támogat.

- `enabled: true`: az R2B4 szerepenként explicit Linux CPU-affinity maszkokat
  alkalmaz és ellenőriz. A `control_cpus` pontosan egyelemű, és egyik másik
  szerep maszkja sem fedheti át.
- `enabled: false`: az R2B4 **nem alkalmaz saját CPU-pinninget**. A runtime,
  matcher, LiDAR, planner, capture, szenzor- és szolgáltatásfolyamatok az
  örökölt host/cgroup CPU-készleten maradnak, elhelyezésüket a Linux scheduler
  végzi.

A jelenlegi production baseline `enabled: false`, tehát scheduler-managed mód.
A konfigurációban megmaradó `*_cpus` mezők ilyenkor nem aktív elhelyezési
parancsok; schema/replay kompatibilitási értékek. A schema ettől még szigorú:
`control_cpus` egyetlen CPU, a többi szerep vele diszjunkt, a CPU-listák nem
lehetnek üresek, duplikáltak, negatívak vagy nem egészek. Ez lehetővé teszi,
hogy a policy később egyetlen `enabled` váltással ismét aktiválható legyen.

`strict: true` csak bekapcsolt affinity-policy mellett kényszeríti az OS/cgroup
maszkok alkalmazását és visszaellenőrzését. `enabled: false` mellett nincs
R2B4-affinity alkalmazás. Ez nem jelent OS-szintű CPU-izolációt: más Linux
folyamatok, kernel threadek és IRQ-k ütemezését az R2B4 nem szabályozza.

Futó rendszer ellenőrzése: `python3 tools/v3_performance_audit.py live`.
Scheduler-managed módban az affinity-rész `NOT_APPLICABLE` /
`SCHEDULER_MANAGED` eredménnyel, sikeres exit kóddal tér vissza; `enabled: true`
mellett a tényleges `/proc` affinity-layoutot auditálja.

"""
    return text[:start] + new + text[end:]


def patch_core_test(text: str) -> str:
    if "def test_active_affinity_policy_is_scheduler_managed_but_schema_valid" in text:
        return text
    anchor = "def test_affinity_masks_keep_control_exclusive_and_replay_historical_policy():\n"
    new_test = """def test_active_affinity_policy_is_scheduler_managed_but_schema_valid():
    # scheduler-managed affinity baseline: masks remain schema-valid but are dormant
    hardware, physics, speed_map, control = _documents()
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    affinity = resolved.affinity
    assert affinity.enabled is False
    assert affinity.control_cpus == (3,)
    control_set = set(affinity.control_cpus)
    for name, cpus in affinity.cpu_roles().items():
        if name != 'control_cpus':
            assert control_set.isdisjoint(cpus), (name, cpus)

"""
    return replace_once(text, anchor, new_test + anchor, "core affinity test")


def patch_deep_test(text: str) -> str:
    if "def test_disabled_affinity_layout_is_scheduler_noop" in text:
        return text
    anchor = "def _mask_probe(connection, cpus):\n"
    new_test = """def test_disabled_affinity_layout_is_scheduler_noop():
    # scheduler-managed affinity baseline: disabled policy must not narrow the task cpuset
    if not hasattr(os, "sched_getaffinity"):
        pytest.skip("Linux affinity inspection required")
    before = set(os.sched_getaffinity(0))
    config = RuntimeAffinityConfig(enabled=False)
    assert apply_process_affinity_layout(config) == ()
    assert set(os.sched_getaffinity(0)) == before


"""
    return replace_once(text, anchor, new_test + anchor, "deep affinity test")


def patch_performance_audit(text: str) -> str:
    if '"mode": "SCHEDULER_MANAGED"' in text:
        return text
    old = """def live(args: argparse.Namespace) -> int:
    pid = _resolve_pid(args)
    root = Path(__file__).resolve().parents[1]
    import sys
    sys.path.insert(0, str(root))
    from v3.runtime_performance import load_runtime_affinity_config, apply_host_affinity
    from v3.affinity_diagnostics import audit_affinity
    apply_host_affinity(root, "diagnostics")
    result = audit_affinity(load_runtime_affinity_config(root / "conf" / "vezerles.json"),
                            pid, project_root=root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1
"""
    new = """def live(args: argparse.Namespace) -> int:
    pid = _resolve_pid(args)
    root = Path(__file__).resolve().parents[1]
    import sys
    sys.path.insert(0, str(root))
    from v3.runtime_performance import load_runtime_affinity_config, apply_host_affinity
    from v3.affinity_diagnostics import audit_affinity
    config = load_runtime_affinity_config(root / "conf" / "vezerles.json")
    if not config.enabled:
        result = {
            "status": "NOT_APPLICABLE",
            "mode": "SCHEDULER_MANAGED",
            "reason": "runtime_affinity.enabled=false; R2B4 applies no CPU pinning",
            "runtime_pid": pid,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    apply_host_affinity(root, "diagnostics")
    result = audit_affinity(config, pid, project_root=root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1
"""
    return replace_once(text, old, new, "performance live audit")


def patch_async_acceptance(text: str) -> str:
    if '"mode": "SCHEDULER_MANAGED"' in text and "runtime_affinity.enabled must be true for live acceptance" not in text:
        return text
    text = text.replace(
        "For both sessions it requires IDLE/SAFE-LOW behavior, verifies the current\nprocess-isolation CPU layout, reuses the repository's canonical timing audit,\n",
        "For both sessions it requires IDLE/SAFE-LOW behavior, verifies the configured\nCPU policy when affinity is enabled (or records scheduler-managed mode when disabled),\nreuses the repository's canonical timing audit,\n",
        1,
    )
    old = """    config = load_runtime_affinity_config(root / "conf" / "vezerles.json")
    if not config.enabled:
        raise AcceptanceError("runtime_affinity.enabled must be true for live acceptance")
    roles = ("encoder", "imu", "lidar_owner", "lidar_matcher", "vision", "planner",
             "status", "command", "l0_encoder", "l0_imu", "l0_lidar", "l0_aux")
"""
    new = """    config = load_runtime_affinity_config(root / "conf" / "vezerles.json")
    if not config.enabled:
        return {
            "status": "NOT_APPLICABLE",
            "mode": "SCHEDULER_MANAGED",
            "reason": "runtime_affinity.enabled=false; R2B4 applies no CPU pinning",
            "runtime_pid": runtime_pid,
            "capture_required": require_capture,
        }
    roles = ("encoder", "imu", "lidar_owner", "lidar_matcher", "vision", "planner",
             "status", "command", "l0_encoder", "l0_imu", "l0_lidar", "l0_aux")
"""
    return replace_once(text, old, new, "P0 async affinity gate")


PATCHERS = {
    "README.md": patch_readme,
    "tests/core/test_v3_config_p0_authority.py": patch_core_test,
    "tests/deep/test_v3_process_affinity.py": patch_deep_test,
    "tools/v3_performance_audit.py": patch_performance_audit,
    "tools/v3_p0_async_acceptance.py": patch_async_acceptance,
}


def already_patched(rel: str, text: str) -> bool:
    return {
        "README.md": README_MARKER in text,
        "tests/core/test_v3_config_p0_authority.py": "test_active_affinity_policy_is_scheduler_managed_but_schema_valid" in text,
        "tests/deep/test_v3_process_affinity.py": "test_disabled_affinity_layout_is_scheduler_noop" in text,
        "tools/v3_performance_audit.py": '"mode": "SCHEDULER_MANAGED"' in text,
        "tools/v3_p0_async_acceptance.py": '"mode": "SCHEDULER_MANAGED"' in text and "must be true for live acceptance" not in text,
    }[rel]


def atomic_write(path: Path, data: bytes) -> None:
    fd, tmp = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(data)
            fh.flush()
            os.fsync(fh.fileno())
        os.chmod(tmp, path.stat().st_mode)
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def main() -> int:
    ap = argparse.ArgumentParser(description="Install scheduler-managed affinity policy docs/tests alignment")
    ap.add_argument("--root")
    ap.add_argument("--skip-tests", action="store_true")
    args = ap.parse_args()
    root = project_root(args.root)

    originals: dict[str, bytes] = {}
    for rel, expected in EXPECTED_BLOBS.items():
        path = root / rel
        data = path.read_bytes()
        text = data.decode("utf-8")
        if already_patched(rel, text):
            continue
        actual = git_blob_sha(data)
        if actual != expected:
            raise SystemExit(
                f"ERROR: source drift: {rel}\n"
                f" expected blob {expected}\n actual   blob {actual}\n"
                "Refusing to overwrite a modified/newer file."
            )
        originals[rel] = data

    if not originals:
        print(f"{UPGRADE_ID}: already installed")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    for rel, data in originals.items():
        target = backup / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)

    try:
        for rel, patcher in PATCHERS.items():
            path = root / rel
            old = path.read_text(encoding="utf-8")
            new = patcher(old)
            if new != old:
                atomic_write(path, new.encode("utf-8"))
                print(f"PATCHED {rel}")
            else:
                print(f"OK      {rel} (already aligned)")

        for rel in (
            "tools/v3_performance_audit.py",
            "tools/v3_p0_async_acceptance.py",
            "tests/core/test_v3_config_p0_authority.py",
            "tests/deep/test_v3_process_affinity.py",
        ):
            subprocess.run([sys.executable, "-m", "py_compile", str(root / rel)], cwd=root, check=True)

        if not args.skip_tests:
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    "-q",
                    "tests/core/test_v3_config_p0_authority.py",
                    "tests/deep/test_v3_process_affinity.py",
                ],
                cwd=root,
                check=True,
            )

        print(f"PASS {UPGRADE_ID}")
        print(f"backup: {backup}")
        print("production runtime code: unchanged")
        print("conf/vezerles.json: unchanged")
        return 0
    except BaseException as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        print("Rolling back changed files...", file=sys.stderr)
        for rel, data in originals.items():
            atomic_write(root / rel, data)
        print(f"rollback complete; backup retained: {backup}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
