#!/usr/bin/env python3
"""Apply the 2026-09-23 R2B4 launcher convergence upgrade.

The upgrade intentionally changes only host/operator interface files and tests.
It does not touch runtime captures, logs, configs, control layers, motor/GPIO code,
or the resident runtime-session implementation.
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path
import shutil
import stat
import tempfile


PACKAGE_ROOT = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_ROOT / "payload"


class UpgradeError(RuntimeError):
    pass


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Apply R2B4 launcher convergence upgrade")
    p.add_argument(
        "root",
        nargs="?",
        default="/home/alba/project_r2b4",
        help="R2B4 repo root (default: /home/alba/project_r2b4)",
    )
    return p


def validate_root(root: Path) -> None:
    required = [
        root / "r",
        root / "v3" / "interface_cli.py",
        root / "v3" / "operator_controller.py",
        root / "v3" / "pytest_profiles.py",
        root / "README.md",
        root / "pytest.ini",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise UpgradeError("not an R2B4 repo or required files missing: " + ", ".join(missing))


def backup_paths(root: Path, paths: list[Path]) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = Path(tempfile.mkdtemp(prefix=f"r2b4_launcher_upgrade_backup_{stamp}_"))
    for path in paths:
        if not path.exists():
            continue
        relative = path.relative_to(root)
        target = backup / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if path.is_dir():
            shutil.copytree(path, target)
        else:
            shutil.copy2(path, target)
    return backup


def write_payload(root: Path, relative: str, *, executable: bool = False) -> None:
    source = PAYLOAD / relative
    destination = root / relative
    if not source.is_file():
        raise UpgradeError(f"payload missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.launcher-upgrade.tmp")
    shutil.copy2(source, temporary)
    if executable:
        mode = temporary.stat().st_mode
        temporary.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    temporary.replace(destination)


def replace_once(text: str, old: str, new: str, *, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise UpgradeError(f"{label}: expected exactly one patch anchor, found {count}")
    return text.replace(old, new, 1)


def transform_interface_cli(text: str) -> str:

    import_anchor = "from v3.operator_controller import CAPTURE_MODES, DEFAULT_CAPTURE_MODE, OperatorError, OperatorEvent\n"
    import_new = import_anchor + "from v3.pytest_profiles import pytest_profile_names\n"
    text = replace_once(text, import_anchor, import_new, label="interface_cli pytest profile import")

    parser_anchor = '    testhub.add_argument("--no-sweep", action="store_true")\n'
    parser_new = parser_anchor + (
        '    testhub.add_argument(\n'
        '        "--pytest", "--pytest-scope", dest="pytest_scope",\n'
        '        choices=("off", *pytest_profile_names()), default="off",\n'
        '        help="run one shared pytest profile inside Test Hub",\n'
        '    )\n'
    )
    text = replace_once(text, parser_anchor, parser_new, label="interface_cli Test Hub pytest argument")

    signature_anchor = "    replay: str,\n    no_sweep: bool,\n) -> object:\n"
    signature_new = "    replay: str,\n    no_sweep: bool,\n    pytest_scope: str,\n) -> object:\n"
    text = replace_once(text, signature_anchor, signature_new, label="interface_cli Test Hub helper signature")

    execute_anchor = "                replay=replay,\n                no_sweep=no_sweep,\n            ))\n"
    execute_new = "                replay=replay,\n                no_sweep=no_sweep,\n                pytest_scope=pytest_scope,\n            ))\n"
    text = replace_once(text, execute_anchor, execute_new, label="interface_cli Test Hub execute parameters")

    call_anchor = "                    replay=args.replay,\n                    no_sweep=args.no_sweep,\n                )\n"
    call_new = "                    replay=args.replay,\n                    no_sweep=args.no_sweep,\n                    pytest_scope=args.pytest_scope,\n                )\n"
    text = replace_once(text, call_anchor, call_new, label="interface_cli Test Hub call")

    return text


def transform_interface_test(text: str) -> str:
    old = '    clean, mode, no_trigger = _extract_capture_selector(["rc", "35", "c", "full", "nc"])\n'
    new = '    clean, mode, hz, no_trigger = _extract_capture_selector(["rc", "35", "c", "full", "nc"])\n'
    if old in text:
        text = text.replace(old, new, 1)
        text = text.replace('    assert mode == "full"\n    assert no_trigger is True\n', '    assert mode == "full"\n    assert hz == 10\n    assert no_trigger is True\n', 1)

    old = '    clean, mode, no_trigger = _extract_capture_selector(["--capture=nincs", "f", "4", "0.15"])\n'
    new = '    clean, mode, hz, no_trigger = _extract_capture_selector(["--capture=nincs", "f", "4", "0.15"])\n'
    if old in text:
        text = text.replace(old, new, 1)
        text = text.replace('    assert mode == "nincs"\n    assert no_trigger is False\n', '    assert mode == "nincs"\n    assert hz == 10\n    assert no_trigger is False\n', 1)

    text = text.replace('with pytest.raises(ValueError, match="capture mode"):', 'with pytest.raises(ValueError):')
    return text


def transform_readme(text: str) -> str:
    start = text.find("## Használat\n")
    end = text.find("Közvetlen production entrypoint:", start)
    if start < 0 or end < 0:
        raise UpgradeError("README: launcher section anchors not found")

    section = '''## Használat

Az egyetlen ajánlott ember/agent belépő a gyökér `r` launcher. A `r` nem robotikai
authority: a robotparancsokat változtatás nélkül a `v3.interface_cli` felé delegálja,
a host/developer segédek pedig külön launcher-infrastruktúrában maradnak.

```bash
./r help
./r commands
./r commands --json

./r s
./r d
./r rc 30
./r fp 20
./r f 10 0.15
./r x
./r sd
```

A timed mozgásparancsok a szükséges runtime-ot automatikusan elindítják, majd STOP,
runtime shutdown és Test Hub finalizálás következik. `0` másodperc folyamatos módot
jelent, ilyenkor a runtime futva marad explicit STOP/shutdown kérésig.

Capture mintavétel alapértelmezése 10 Hz. Választható: `c 50`, `c 10`, `c 5`,
`c 1`; a capture mód továbbra is `c alap`, `c full` vagy `c nincs`. Példák:

```bash
./r rc 30 c 10
./r rc 30 c 50
./r fp 20 c nincs
./r cap status
```

Test Hub és közös pytest-profilok:

```bash
./r th
./r th run
./r th run --pytest control
./r test
./r test async
./r test --list
```

A pytest-profilok egyetlen forrása a `v3/pytest_profiles.py`; a launcher és a Test Hub
ezt a közös listát használja. Nyers pytest továbbra is elérhető: `./r pytest ...`.

Fejlesztő/host segédek például: `r git`, `r gitre`, `r tools`, `r tool NAME`,
`r cpu`, `r cpu2`, `r disc`, `r mem`, `r temp`, `r ps`, `r net`, `r usb`, `r i2c`,
`r host`, `r version`. A teljes aktuális felület agent-barát JSON formában:
`r commands --json`; az élő RobotInterface capability-k: `r caps`.

A régi gyökér `r2b4` launcher megszűnt. A belső `v3.operator_cli` modul megmarad,
mert a resident runtime-session technikai child-process entrypointja használja; ez
nem második felhasználói launcher.

'''
    return text[:start] + section + text[end:]


def main() -> int:
    args = parser().parse_args()
    root = Path(args.root).expanduser().resolve()
    validate_root(root)

    tracked = [
        root / "r",
        root / "r2b4",
        root / "v3" / "interface_cli.py",
        root / "v3" / "launcher_cli.py",
        root / "v3" / "host_cli.py",
        root / "tests" / "test_v3_launcher_cli.py",
        root / "tests" / "test_v3_interface_cli.py",
        root / "README.md",
    ]
    # Compute every source patch before writing anything, so an unexpected
    # source shape cannot leave a half-applied upgrade.
    interface_path = root / "v3" / "interface_cli.py"
    readme_path = root / "README.md"
    interface_text = transform_interface_cli(interface_path.read_text(encoding="utf-8"))
    readme_text = transform_readme(readme_path.read_text(encoding="utf-8"))
    interface_test_path = root / "tests" / "test_v3_interface_cli.py"
    interface_test_text = (
        transform_interface_test(interface_test_path.read_text(encoding="utf-8"))
        if interface_test_path.is_file() else None
    )

    backup = backup_paths(root, tracked)

    write_payload(root, "r", executable=True)
    write_payload(root, "v3/launcher_cli.py")
    write_payload(root, "v3/host_cli.py")
    write_payload(root, "tests/test_v3_launcher_cli.py")
    interface_path.write_text(interface_text, encoding="utf-8")
    readme_path.write_text(readme_text, encoding="utf-8")
    if interface_test_text is not None:
        interface_test_path.write_text(interface_test_text, encoding="utf-8")

    legacy = root / "r2b4"
    if legacy.exists() or legacy.is_symlink():
        legacy.unlink()

    print("R2B4 launcher upgrade applied.")
    print(f"root:   {root}")
    print(f"backup: {backup}")
    print("changed: r, v3/launcher_cli.py, v3/host_cli.py, v3/interface_cli.py, README.md, launcher tests")
    print("removed: r2b4 (legacy user launcher)")
    print("unchanged: v3/operator_cli.py internal runtime-session entrypoint")
    print("validate: ./r commands && ./r test gate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
