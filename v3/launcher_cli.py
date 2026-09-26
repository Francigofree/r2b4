"""Single human/agent launcher facade for R2B4.

The launcher owns no robot state. Robot-facing syntax is delegated unchanged to
``v3.interface_cli``; host/developer helpers live in ``v3.host_cli``.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from collections.abc import Sequence

from v3 import host_cli, interface_cli
from v3.capture_rate import CAPTURE_HZ_VALUES, DEFAULT_CAPTURE_HZ
from v3.test_runner import FOCUSED


class LauncherError(RuntimeError):
    pass


def project_root() -> Path:
    raw = os.environ.get("R2B4_ROOT")
    root = Path(raw).expanduser() if raw else Path(__file__).resolve().parents[1]
    root = root.resolve()
    if not (root / "v3").is_dir() or not (root / "pytest.ini").is_file():
        raise LauncherError(f"invalid R2B4 root: {root}")
    return root


def _robot_catalog() -> list[dict[str, object]]:
    parser = interface_cli._parser()
    subparsers = next(
        action for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    aliases = dict(interface_cli.ALIASES)
    canonical = sorted(set(subparsers.choices) - set(aliases))
    return [
        {
            "name": name,
            "aliases": sorted(alias for alias, target in aliases.items() if target == name),
            "motion": name in interface_cli.MOTION_COMMANDS,
        }
        for name in canonical
    ]


def command_catalog() -> dict[str, object]:
    return {
        "launcher": "r",
        "robot": _robot_catalog(),
        "capture": {
            "default_hz": DEFAULT_CAPTURE_HZ,
            "hz": list(CAPTURE_HZ_VALUES),
            "modes": ["alap", "full", "nincs"],
        },
        "tests": {
            "modes": ["core", *FOCUSED, "full"],
        },
        "testhub": {
            "view_hz": [1, 5, 10],
        },
        "er2": {"commands": ["status", "preview", "stream"], "execution": "REAL_ONLY"},
        "local": sorted(set(host_cli.COMMANDS) - set(host_cli.ALIASES)),
    }


def print_help() -> None:
    print("R2B4 — single human/agent launcher\n")
    print(interface_cli._parser().format_help().rstrip())
    print(
        "\nLauncher/development:\n"
        "  r commands [--json]          discover current command surface\n"
        "  r test [MODE]                curated pytest mode; default core\n"
        "  r pytest [ARGS...]            raw pytest passthrough\n"
        "  r git | gitre | tools | tool  repo/developer helpers\n"
        "  r version                     repo revision + dirty state\n"
        "  r er2 status|preview|stream   Gemini Robotics ER 2 (real execution only)\n"
        "\nHost diagnostics:\n"
        "  r cpu | cpu2 | disc | mem | temp | ps\n"
        "  r net | usb | i2c | host\n"
        "\nSetup:\n"
        "  r install | where | root\n"
    )


def _commands(argv: list[str]) -> int:
    if argv not in ([], ["--json"]):
        raise LauncherError("usage: r commands [--json]")
    catalog = command_catalog()
    if argv == ["--json"]:
        print(json.dumps(catalog, ensure_ascii=False, indent=2, sort_keys=True))
        return 0
    print("Robot commands:")
    for item in catalog["robot"]:
        aliases = f" ({', '.join(item['aliases'])})" if item["aliases"] else ""
        motion = " [motion]" if item["motion"] else ""
        print(f"  {item['name']}{aliases}{motion}")
    capture = catalog["capture"]
    print(
        f"\nCapture: default {capture['default_hz']} Hz | rates "
        + "/".join(map(str, capture["hz"]))
        + " | modes " + "/".join(capture["modes"])
    )
    print("Test modes: " + ", ".join(catalog["tests"]["modes"]))
    print("Local: " + ", ".join(catalog["local"]))
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    try:
        root = project_root()
        if not args or args[0] in {"help", "-h", "--help"}:
            print_help()
            return 0
        if args[0] == "commands":
            return _commands(args[1:])
        if args[0] == "er2":
            # R2B4_ER2_P0_20260925: provider integration is a consumer of the
            # canonical RobotInterface/ExternalRobotGateway, not a robot layer.
            from r2b4_er2.cli import main as er2_main
            return er2_main(args[1:], project_root=root)
        if args[0] in host_cli.COMMANDS:
            return host_cli.execute(args[0], args[1:], root)
        return interface_cli.main(args)
    except KeyboardInterrupt:
        return 130
    except (LauncherError, host_cli.HostCliError, OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["command_catalog", "main", "project_root"]
