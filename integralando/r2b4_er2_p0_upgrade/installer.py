#!/usr/bin/env python3
"""Install R2B4 Gemini Robotics ER2 P0 against the verified 2026-09-25 main source.

Source-first package: only per-file Git blob hashes are checked; no full-repo SHA gate.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

BUILD = "R2B4_ER2_P0_20260925"
GOOGLE_GENAI_VERSION = "2.25.0"
EXPECTED_BLOBS = {
    "requirements.txt": "0c43de4bc2d66810da8206059647ad2905e5fd92",
    "v3/adapters/process_vision_port.py": "a1987026763bb0b8220a4ddacb4f4603da179e7f",
    "v3/external_gateway.py": "f0d39cf1e55ddaff47c57e64eb320a8196bf49a2",
    "v3/interface_cli.py": "6b3798db9b5d3da58ec0ddc9c3675c6be5c621d9",
    "v3/launcher_cli.py": "2e819b8751cf688b88209abfa08778e15212b9dd",
}
NEW_PATHS = (
    "r2b4_er2/__init__.py",
    "r2b4_er2/__main__.py",
    "r2b4_er2/config.py",
    "r2b4_er2/media.py",
    "r2b4_er2/tool_bridge.py",
    "r2b4_er2/preview.py",
    "r2b4_er2/streaming.py",
    "r2b4_er2/cli.py",
    "v3/adapters/vision_media_socket.py",
    "tools/validate_er2_p0.py",
)


def run(cmd: list[str], *, cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, check=check)


def git_blob(root: Path, relative: str) -> str:
    result = subprocess.run(
        ["git", "hash-object", relative], cwd=root, text=True, capture_output=True
    )
    if result.returncode != 0:
        raise RuntimeError(f"cannot hash {relative}: {result.stderr.strip()}")
    return result.stdout.strip()


def check_source(root: Path, *, force: bool) -> None:
    if not (root / "v3").is_dir() or not (root / "pytest.ini").is_file():
        raise RuntimeError(f"not an R2B4 repository: {root}")
    marker = root / "r2b4_er2" / "__init__.py"
    if marker.is_file() and BUILD in marker.read_text(encoding="utf-8", errors="replace"):
        print(f"{BUILD}: already installed")
        return
    mismatches = []
    for relative, expected in EXPECTED_BLOBS.items():
        path = root / relative
        if not path.is_file():
            mismatches.append(f"{relative}: missing")
            continue
        actual = git_blob(root, relative)
        if actual != expected:
            mismatches.append(f"{relative}: expected {expected}, got {actual}")
    if mismatches and not force:
        raise RuntimeError(
            "source drift detected; package targets the verified main state.\n  "
            + "\n  ".join(mismatches)
            + "\nRebuild the upgrade source-first for the new repo state; --force is only for a reviewed equivalent tree."
        )


def ensure_dependency(*, skip: bool) -> None:
    if skip:
        print("dependency install: SKIPPED")
        return
    try:
        current = importlib.metadata.version("google-genai")
    except importlib.metadata.PackageNotFoundError:
        current = None
    if current == GOOGLE_GENAI_VERSION:
        print(f"google-genai {current}: already installed")
        return
    spec = f"google-genai=={GOOGLE_GENAI_VERSION}"
    candidates = [
        [sys.executable, "-m", "pip", "install", "--user", "--break-system-packages", spec],
        [sys.executable, "-m", "pip", "install", "--user", spec],
    ]
    last = None
    for cmd in candidates:
        print("dependency:", " ".join(cmd))
        last = subprocess.run(cmd)
        if last.returncode == 0:
            return
    raise RuntimeError(
        f"could not install {spec}. Install it for the same Python interpreter, then rerun with --skip-deps."
    )


def patch_requirements(text: str) -> str:
    line = f"google-genai=={GOOGLE_GENAI_VERSION}"
    if line not in text.splitlines():
        if not text.endswith("\n"):
            text += "\n"
        text += "\n# Gemini Robotics ER 2 Preview + Streaming\n" + line + "\n"
    return text


def patch_external_gateway(text: str) -> str:
    if BUILD in text:
        return text
    needle = "from typing import Protocol\n"
    if needle not in text:
        raise RuntimeError("external_gateway import anchor changed")
    text = text.replace(
        needle,
        needle + "\nfrom v3.action_catalog import action_descriptor\n",
        1,
    )
    pattern = re.compile(
        r"\n_SESSION_OWNED_ACTIONS = frozenset\(\n    \{.*?\n    \}\n\)\n",
        re.DOTALL,
    )
    text, count = pattern.subn("\n", text, count=1)
    if count != 1:
        raise RuntimeError("external_gateway session-action anchor changed")
    old = """            if parsed.name in _SESSION_OWNED_ACTIONS:\n                parameters[\"session_owner_pid\"] = self._policy.session_owner_pid\n                parameters[\"session_watchdog_s\"] = self._policy.session_watchdog_s\n"""
    new = """            # R2B4_ER2_P0_20260925: session semantics come from the canonical\n            # action catalog, not from a second gateway-maintained action list.\n            descriptor = action_descriptor(parsed.name)\n            if descriptor is not None and descriptor.session_watchdog:\n                parameters[\"session_owner_pid\"] = self._policy.session_owner_pid\n                parameters[\"session_watchdog_s\"] = self._policy.session_watchdog_s\n"""
    if old not in text:
        raise RuntimeError("external_gateway execution anchor changed")
    return text.replace(old, new, 1)


def patch_process_vision(text: str) -> str:
    if BUILD in text:
        return text
    import_anchor = "from .person_detection import (\n"
    if import_anchor not in text:
        raise RuntimeError("process_vision import anchor changed")
    # Place the tiny producer-side socket adapter next to the vision imports.
    text = text.replace(
        import_anchor,
        "from .vision_media_socket import VisionMediaServer\n" + import_anchor,
        1,
    )
    old = """    camera: NativePicamera2Camera | None = None\n    detector: NativePersonDetector | None = None\n    try:\n"""
    new = """    camera: NativePicamera2Camera | None = None\n    detector: NativePersonDetector | None = None\n    media_server: VisionMediaServer | None = None\n    try:\n"""
    if old not in text:
        raise RuntimeError("process_vision owner anchor changed")
    text = text.replace(old, new, 1)
    old = """        if not camera.start():\n            raise RuntimeError(\"camera worker did not start\")\n        if detector_config is not None:\n"""
    new = """        if not camera.start():\n            raise RuntimeError(\"camera worker did not start\")\n        # R2B4_ER2_P0_20260925: large JPEG bytes leave directly from the\n        # vision owner process. They never re-enter the 50 Hz control interpreter.\n        media_server = VisionMediaServer(camera)\n        media_server.start()\n        if detector_config is not None:\n"""
    if old not in text:
        raise RuntimeError("process_vision camera-start anchor changed")
    text = text.replace(old, new, 1)
    old = """    finally:\n        if detector is not None:\n"""
    new = """    finally:\n        if media_server is not None:\n            try:\n                media_server.stop()\n            except BaseException:\n                pass\n        if detector is not None:\n"""
    if old not in text:
        raise RuntimeError("process_vision finalization anchor changed")
    text = text.replace(old, new, 1)
    return text


def patch_launcher(text: str) -> str:
    if BUILD in text:
        return text
    old = '        "local": sorted(set(host_cli.COMMANDS) - set(host_cli.ALIASES)),\n'
    new = (
        '        "er2": {"commands": ["status", "preview", "stream"], '
        '"execution": "REAL_ONLY"},\n'
        + old
    )
    if old not in text:
        raise RuntimeError("launcher catalog anchor changed")
    text = text.replace(old, new, 1)
    old = '        "  r version                     repo revision + dirty state\\n"\n'
    new = (
        old
        + '        "  r er2 status|preview|stream   Gemini Robotics ER 2 (real execution only)\\n"\n'
    )
    if old not in text:
        raise RuntimeError("launcher help anchor changed")
    text = text.replace(old, new, 1)
    old = """        if args[0] == \"commands\":\n            return _commands(args[1:])\n        if args[0] in host_cli.COMMANDS:\n"""
    new = """        if args[0] == \"commands\":\n            return _commands(args[1:])\n        if args[0] == \"er2\":\n            # R2B4_ER2_P0_20260925: provider integration is a consumer of the\n            # canonical RobotInterface/ExternalRobotGateway, not a robot layer.\n            from r2b4_er2.cli import main as er2_main\n            return er2_main(args[1:], project_root=root)\n        if args[0] in host_cli.COMMANDS:\n"""
    if old not in text:
        raise RuntimeError("launcher routing anchor changed")
    return text.replace(old, new, 1)


_TIMED_MOTION = '''def _run_timed_motion(\n    interface: RobotInterface,\n    args: argparse.Namespace,\n    *,\n    capture_mode: str,\n    capture_hz: int,\n    no_trigger: bool,\n) -> dict[str, object]:\n    command = _canonical(args.command)\n    seconds = _validate_seconds(args.seconds)\n    action, parameters = _motion_request(args)\n\n    # R2B4_ER2_P0_20260925: a short CLI motion owns its motion producer, not an\n    # already-running resident runtime. Reuse the resident capture contract too,\n    # so a client command can never restart another client's runtime just to\n    # change capture mode/rate.\n    before_raw = interface.read("operator.status")\n    before = before_raw if isinstance(before_raw, Mapping) else {}\n    runtime_preexisting = before.get("runtime_running") is True\n    effective_mode = capture_mode\n    effective_hz = capture_hz\n    if runtime_preexisting:\n        resident_mode = before.get("capture_mode")\n        resident_hz = before.get("capture_hz")\n        if isinstance(resident_mode, str):\n            effective_mode = resident_mode\n        if isinstance(resident_hz, int) and not isinstance(resident_hz, bool):\n            effective_hz = resident_hz\n        if effective_mode != capture_mode or effective_hz != capture_hz:\n            print(\n                f"runtime already active: inherit capture {effective_mode} @ {effective_hz} Hz "\n                f"(requested {capture_mode} @ {capture_hz} Hz)",\n                flush=True,\n            )\n\n    parameters.update({\n        "capture": not no_trigger,\n        "capture_mode": effective_mode,\n        "capture_hz": effective_hz,\n    })\n    if seconds > 0.0:\n        parameters["session_owner_pid"] = os.getpid()\n        parameters["session_watchdog_s"] = seconds + 5.0\n\n    print(\n        f"R2B4: {command} | {'continuous' if seconds == 0 else f'{seconds:g} s'}"\n        f" | capture {effective_mode} @ {effective_hz} Hz"\n    )\n    try:\n        handle = interface.execute(action, **parameters)\n    except BaseException:\n        # If this CLI created a resident runtime during setup, clean up only that\n        # owned runtime. A pre-existing runtime belongs to the wider robot session.\n        if seconds > 0 and not runtime_preexisting:\n            try:\n                _shutdown_with_testhub_progress(interface, capture_mode=effective_mode)\n            except Exception:\n                pass\n        raise\n\n    if seconds == 0:\n        print("command ACTIVE; runtime remains running.  STOP: ./r x   shutdown: ./r sd")\n        return {"status": "ACTIVE", "command": command, "handle": str(handle)}\n\n    deadline = time.monotonic() + seconds\n    last_bucket: int | None = None\n    interrupted = False\n    try:\n        while True:\n            remaining = deadline - time.monotonic()\n            if remaining <= 0:\n                break\n            bucket = int(math.ceil(remaining / 5.0) * 5)\n            if bucket != last_bucket and seconds >= 10.0:\n                print(f"running: {max(0, int(math.ceil(remaining)))} s remaining", flush=True)\n                last_bucket = bucket\n            time.sleep(min(0.25, remaining))\n    except KeyboardInterrupt:\n        interrupted = True\n        print("\\nCtrl+C -> STOP", file=sys.stderr, flush=True)\n    finally:\n        interface.stop()\n\n    if runtime_preexisting:\n        final: Mapping[str, object] = {\n            "state": "RUNTIME_REUSED",\n            "runtime_kept_running": True,\n        }\n        print("runtime kept running (pre-existing resident session)")\n    else:\n        final = _shutdown_with_testhub_progress(interface, capture_mode=effective_mode)\n\n    return {\n        "status": "INTERRUPTED" if interrupted else "FINISHED",\n        "command": command,\n        "seconds": seconds,\n        "testhub": final,\n    }\n'''


def patch_interface_cli(text: str) -> str:
    if BUILD in text:
        return text
    start = text.find("def _run_timed_motion(\n")
    end = text.find("\ndef _shutdown_with_testhub_progress(\n", start)
    if start < 0 or end < 0:
        raise RuntimeError("interface_cli timed-motion function anchor changed")
    return text[:start] + _TIMED_MOTION + text[end:]


def patched_contents(root: Path) -> dict[str, str]:
    source = {rel: (root / rel).read_text(encoding="utf-8") for rel in EXPECTED_BLOBS}
    return {
        "requirements.txt": patch_requirements(source["requirements.txt"]),
        "v3/adapters/process_vision_port.py": patch_process_vision(source["v3/adapters/process_vision_port.py"]),
        "v3/external_gateway.py": patch_external_gateway(source["v3/external_gateway.py"]),
        "v3/interface_cli.py": patch_interface_cli(source["v3/interface_cli.py"]),
        "v3/launcher_cli.py": patch_launcher(source["v3/launcher_cli.py"]),
    }


def install_files(root: Path, package_root: Path, patches: dict[str, str]) -> tuple[Path, list[str]]:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = root / ".upgrade_backups" / f"er2_p0_{stamp}"
    created: list[str] = []
    backup.mkdir(parents=True, exist_ok=False)

    targets = list(patches) + list(NEW_PATHS)
    for relative in targets:
        target = root / relative
        if target.exists():
            dest = backup / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            if target.is_file():
                shutil.copy2(target, dest)
        else:
            created.append(relative)

    for relative, content in patches.items():
        target = root / relative
        target.write_text(content, encoding="utf-8")

    files_root = package_root / "files"
    for relative in NEW_PATHS:
        src = files_root / relative
        if not src.is_file():
            raise RuntimeError(f"upgrade payload missing: {relative}")
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)

    return backup, created


def rollback(root: Path, backup: Path, created: list[str]) -> None:
    for relative in created:
        target = root / relative
        try:
            if target.is_file() or target.is_symlink():
                target.unlink()
        except OSError:
            pass
    for source in backup.rglob("*"):
        if not source.is_file():
            continue
        relative = source.relative_to(backup)
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def validate(root: Path) -> None:
    compile_targets = [str(root / rel) for rel in NEW_PATHS if rel.endswith(".py")]
    compile_targets += [
        str(root / "v3/adapters/process_vision_port.py"),
        str(root / "v3/external_gateway.py"),
        str(root / "v3/interface_cli.py"),
        str(root / "v3/launcher_cli.py"),
    ]
    run([sys.executable, "-m", "py_compile", *compile_targets], cwd=root)
    run([sys.executable, "tools/validate_er2_p0.py"], cwd=root)
    run([sys.executable, "-m", "r2b4_er2", "status", "--json"], cwd=root)


def main() -> int:
    parser = argparse.ArgumentParser(description="Install R2B4 ER2 P0 upgrade")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(os.environ.get("R2B4_ROOT", "/home/alba/project_r2b4")),
    )
    parser.add_argument("--force", action="store_true", help="allow reviewed per-file source drift")
    parser.add_argument("--skip-deps", action="store_true", help="do not install google-genai")
    parser.add_argument("--skip-validation", action="store_true")
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    package_root = Path(__file__).resolve().parent

    check_source(root, force=args.force)
    already = root / "r2b4_er2" / "__init__.py"
    if already.is_file() and BUILD in already.read_text(encoding="utf-8", errors="replace"):
        if not args.skip_validation:
            validate(root)
        return 0

    # Dependency failure must not leave a half-applied source upgrade.
    ensure_dependency(skip=args.skip_deps)
    patches = patched_contents(root)
    backup, created = install_files(root, package_root, patches)
    try:
        if not args.skip_validation:
            validate(root)
    except BaseException:
        print(f"validation failed; restoring source from {backup}", file=sys.stderr)
        rollback(root, backup, created)
        raise

    print(f"{BUILD}: INSTALLED")
    print(f"backup: {backup}")
    print("models: gemini-robotics-er-2-preview / gemini-robotics-er-2-streaming-preview")
    print("commands: ./r er2 status | ./r er2 preview ... | ./r er2 stream ...")
    print("execution: REAL_ONLY through ExternalRobotGateway -> RobotInterface -> canonical V3 command path")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
