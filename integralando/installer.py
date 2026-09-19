#!/usr/bin/env python3
"""Install the R2B4 RobotInterface + Test Hub test-integration fix.

Scope:
  NEW  v3/interface_adapters.py
  MOD  tests/test_v3_test_hub_quality.py
  MOD  tests/test_v3_test_hub_behavior.py

No runtime/control/capture/replay/L0-L12 files are modified.
No whole-repository SHA check is used.
"""

from __future__ import annotations

import py_compile
import subprocess
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_DIR / "payload"

NEW_INTERFACE = Path("v3/interface_adapters.py")
QUALITY_TEST = Path("tests/test_v3_test_hub_quality.py")
BEHAVIOR_TEST = Path("tests/test_v3_test_hub_behavior.py")

TARGETED_TESTS = (
    "tests/test_v3_interface_cli.py",
    "tests/test_v3_robot_interface.py",
    "tests/test_v3_test_hub_quality.py",
    "tests/test_v3_test_hub_behavior.py",
    "tests/test_v3_test_hub_cli.py",
    "tests/test_v3_test_hub_portable.py",
    "tests/test_v3_test_hub_analysis.py",
)


def find_root() -> Path:
    candidates = [Path.cwd(), PACKAGE_DIR, *PACKAGE_DIR.parents]
    for candidate in candidates:
        if (
            (candidate / "v3/robot_interface.py").is_file()
            and (candidate / "v3/adapters").is_dir()
            and (candidate / "tests").is_dir()
        ):
            return candidate.resolve()
    raise RuntimeError("R2B4 repo root not found")


def atomic_write(path: Path, data: bytes) -> None:
    temporary = path.with_name(f".{path.name}.upgrade.tmp")
    temporary.write_bytes(data)
    temporary.replace(path)


def require_current_layout(root: Path) -> None:
    robot_interface = root / "v3/robot_interface.py"
    text = robot_interface.read_text(encoding="utf-8")
    marker = "from v3.interface_adapters import build_adapters"
    if marker not in text:
        raise RuntimeError(
            "unexpected v3/robot_interface.py: canonical interface_adapters import missing"
        )

    expected = {
        "v3/adapters/camera.py": "class CameraInterfaceAdapter",
        "v3/adapters/operator.py": "class OperatorInterfaceAdapter",
        "v3/adapters/system.py": "class SystemInterfaceAdapter",
        "v3/adapters/testhub.py": "class TestHubInterfaceAdapter",
        "v3/adapters/v3_control.py": "class V3ControlInterfaceAdapter",
    }
    for relative, marker in expected.items():
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"required existing adapter missing: {relative}")
        if marker not in path.read_text(encoding="utf-8"):
            raise RuntimeError(f"required adapter class missing from {relative}: {marker}")

    for relative in (QUALITY_TEST, BEHAVIOR_TEST):
        if not (root / relative).is_file():
            raise RuntimeError(f"required test file missing: {relative}")


def patch_quality_test(text: str) -> str:
    fixed_marker = '    behavior_source = behavior.read_text(encoding="utf-8")\n'
    if fixed_marker in text:
        return text

    old = """    assert behavior.is_file(), "Quality upgrade must be installed after the Behavior upgrade"
    for required in ("behavior_summary.json", "behavior_episodes.ndjson", "behavior_timeline.ndjson"):
        assert required in next_source
"""
    new = """    assert behavior.is_file(), "Quality upgrade must be installed after the Behavior upgrade"
    behavior_source = behavior.read_text(encoding="utf-8")
    assert "build_behavior_evidence" in next_source
    for required in ("behavior_summary.json", "behavior_episodes.ndjson", "behavior_timeline.ndjson"):
        assert required in behavior_source
"""
    if old not in text:
        raise RuntimeError(
            "tests/test_v3_test_hub_quality.py is neither the expected old form nor the fixed form"
        )
    return text.replace(old, new, 1)


def patch_behavior_test(text: str) -> str:
    class_start = text.find("class FakeReader:")
    if class_start < 0:
        raise RuntimeError("FakeReader not found in tests/test_v3_test_hub_behavior.py")
    next_def = text.find("\ndef _tick(", class_start)
    if next_def < 0:
        raise RuntimeError("FakeReader boundary not found in tests/test_v3_test_hub_behavior.py")

    class_text = text[class_start:next_def]
    if "def first_json(self, topic):" in class_text:
        return text

    old = """    def sha256(self):
        return "0" * 64
"""
    new = """    def first_json(self, topic):
        del topic
        return None

    def sha256(self):
        return "0" * 64
"""
    if old not in class_text:
        raise RuntimeError(
            "FakeReader sha256 anchor missing in tests/test_v3_test_hub_behavior.py"
        )
    return text[:class_start] + class_text.replace(old, new, 1) + text[next_def:]


def rollback(backups: dict[Path, bytes | None]) -> None:
    for path, original in reversed(tuple(backups.items())):
        if original is None:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        else:
            atomic_write(path, original)


def install(root: Path) -> tuple[dict[Path, bytes | None], list[Path]]:
    backups: dict[Path, bytes | None] = {}
    touched: list[Path] = []

    try:
        source = PAYLOAD / NEW_INTERFACE
        if not source.is_file():
            raise RuntimeError(f"package payload missing: {NEW_INTERFACE}")
        target = root / NEW_INTERFACE
        wanted = source.read_bytes()

        if target.exists():
            if target.is_symlink() or not target.is_file():
                raise RuntimeError(f"refusing non-regular target: {NEW_INTERFACE}")
            current = target.read_bytes()
            if current != wanted:
                raise RuntimeError(
                    f"{NEW_INTERFACE} already exists with different content; refusing overwrite"
                )
        else:
            backups[target] = None
            target.parent.mkdir(parents=True, exist_ok=True)
            atomic_write(target, wanted)
            touched.append(target)

        for relative, patcher in (
            (QUALITY_TEST, patch_quality_test),
            (BEHAVIOR_TEST, patch_behavior_test),
        ):
            path = root / relative
            original = path.read_bytes()
            updated = patcher(original.decode("utf-8")).encode("utf-8")
            if updated != original:
                backups[path] = original
                atomic_write(path, updated)
                touched.append(path)

    except Exception:
        rollback(backups)
        raise

    return backups, touched


def validate_source(root: Path) -> None:
    for relative in (NEW_INTERFACE, QUALITY_TEST, BEHAVIOR_TEST):
        py_compile.compile(str(root / relative), doraise=True)

    interface_text = (root / NEW_INTERFACE).read_text(encoding="utf-8")
    for marker in (
        "CameraInterfaceAdapter",
        "OperatorInterfaceAdapter",
        "SystemInterfaceAdapter",
        "TestHubInterfaceAdapter",
        "V3ControlInterfaceAdapter",
        "def build_adapters(",
    ):
        if marker not in interface_text:
            raise RuntimeError(f"interface composition marker missing: {marker}")

    quality_text = (root / QUALITY_TEST).read_text(encoding="utf-8")
    if "assert required in behavior_source" not in quality_text:
        raise RuntimeError("Quality test fix is not installed")

    behavior_text = (root / BEHAVIOR_TEST).read_text(encoding="utf-8")
    if "def first_json(self, topic):" not in behavior_text:
        raise RuntimeError("Behavior FakeReader first_json fix is not installed")


def run_tests(root: Path) -> None:
    command = [sys.executable, "-m", "pytest", "-q", *TARGETED_TESTS]
    print("+", " ".join(command), flush=True)
    completed = subprocess.run(command, cwd=root, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"targeted tests failed with exit code {completed.returncode}")


def main() -> int:
    root = find_root()
    print(f"repo: {root}")
    print("upgrade: RobotInterface composition + Test Hub test integration fix")

    backups: dict[Path, bytes | None] = {}
    try:
        require_current_layout(root)
        backups, touched = install(root)
        validate_source(root)
        run_tests(root)
    except Exception as exc:
        if backups:
            rollback(backups)
            print("INSTALL FAILED - changes rolled back", file=sys.stderr)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print("INSTALL PASS")
    if touched:
        print("installed/modified:")
        for path in touched:
            print(" -", path.relative_to(root))
    else:
        print("fix was already installed; validation/tests passed")

    print("futtasd a full pytest-et: cd /home/alba/project_r2b4 && python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
