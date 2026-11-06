"""Single human/agent launcher facade for R2B4.

The launcher owns no robot state. Robot-facing syntax is delegated unchanged to
``v3.interface_cli``; host/developer helpers live in ``v3.host_cli``.
"""

from __future__ import annotations

import argparse
import difflib
import json
import os
from pathlib import Path
import sys
from collections.abc import Sequence

from v3 import host_cli, interface_cli, launcher_extras
from v3.capture_rate import CAPTURE_HZ_VALUES, DEFAULT_CAPTURE_HZ


class LauncherError(RuntimeError):
    pass


def project_root() -> Path:
    raw = os.environ.get("R2B4_ROOT")
    root = Path(raw).expanduser() if raw else Path(__file__).resolve().parents[1]
    root = root.resolve()
    if not (root / "v3").is_dir() or not (root / "pytest.ini").is_file():
        raise LauncherError(f"invalid R2B4 root: {root}")
    return root


def _normalize_er2_args(args: list[str]) -> list[str]:
    """Map human ER2 shorthand onto the canonical streaming command.

    Examples:
      r er2 "fordulj 90 fokot"
      r er2 "mit látsz?" --camera --tools --speak --json

    Canonical status/preview/stream invocations are preserved unchanged.
    """
    if len(args) < 2 or args[0] != "er2":
        return args
    from r2b4_er2.cli import _parser
    if args[1] in launcher_extras._subparsers(_parser()).choices or args[1] in {"-h", "--help"}:
        return args
    return ["er2", "stream", *args[1:]]


def _robot_catalog() -> list[dict[str, object]]:
    parser = interface_cli._parser()
    subparsers = next(
        action for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    aliases = dict(interface_cli.ALIASES)
    # These routes are owned by the offline host DIAG and standalone camera
    # facade. Runtime diagnostics remains explicitly discoverable as r d.
    canonical = sorted((set(subparsers.choices) - set(aliases)) - {"diag", "camera"})
    rows = [
        {
            "name": name,
            "aliases": sorted(alias for alias, target in aliases.items() if target == name),
            "motion": name in interface_cli.MOTION_COMMANDS,
            "description": subparsers.choices[name].description,
            "usage": subparsers.choices[name].format_usage().strip(),
        }
        for name in canonical
    ]
    child = subparsers.choices["d"]
    rows.append({"name": "d", "aliases": [], "motion": False,
                 "description": "Részletes élő runtime-diagnosztika.",
                 "usage": child.format_usage().strip().replace("r diag", "r d", 1)})
    return sorted(rows, key=lambda row: row["name"])


def command_entries() -> list[dict[str, object]]:
    """Effective launcher routes shared by help, catalog and completion.

    Robot additions come from the owning parser, host additions from COMMAND_HELP.
    Only launcher-specific routes are declared here; no plugin discovery/import.
    """
    from r2b4_er2.cli import _parser as er2_parser
    from r2b4_voice.openai_oauth import cli_parser as chatgpt_parser
    from tools.mcap_evidence.cli import parser as evidence_parser
    from r2b4_orchestration.learning_cli import parser as learning_parser

    rows = [{**item, "usage": item["usage"].removeprefix("usage: "), "group": "robot"}
            for item in _robot_catalog()]
    rows.extend({"name": name, "aliases": sorted(alias for alias, target in host_cli.ALIASES.items()
                                                 if target == name),
                 "usage": f"r {name} {usage}".rstrip(), "description": description,
                 "group": "host"}
                for name, (usage, description) in host_cli.COMMAND_HELP.items())
    for name, aliases, parser in (
        ("er2", [], er2_parser()), ("chatgpt", [], chatgpt_parser()),
        ("evi", [], evidence_parser()), ("cam", ["camera"], launcher_extras.camera_parser()),
        ("learn", [], learning_parser()),
    ):
        rows.append({"name": name, "aliases": aliases, "usage": parser.format_usage().strip().removeprefix("usage: "),
                     "description": parser.description, "group": "launcher"})
    rows.extend({"name": name, "aliases": aliases, "usage": usage,
                 "description": description, "group": "launcher"}
                for name, aliases, usage, description in (
                    ("voice", ["wake"], "r voice [" + "|".join(launcher_extras.VOICE_OPERATIONS) + "]",
                     "Voice wake user-service kezelése; alap: status."),
                    ("route", [], 'r route "KÉRÉS" [--json]', "Belépési route megtekintése."),
                    ("help", [], "r help [PARANCS]", "Áttekintés vagy a parancs saját részletes súgója."),
                    ("commands", [], "r commands [--json]", "Teljes, source-ból követett parancskatalógus."),
                ))
    return sorted(rows, key=lambda row: row["name"])


def _catalog_root(root: Path | str | None) -> Path:
    return Path(root).expanduser().resolve() if root is not None else project_root()


def command_catalog(*, project_root: Path | str | None = None) -> dict[str, object]:
    from r2b4_er2.cli import _parser as er2_parser
    from r2b4_voice.openai_oauth import cli_parser as chatgpt_parser
    from tools.mcap_evidence.cli import OPERATIONS
    root = _catalog_root(project_root)
    entries = command_entries()
    return {
        "launcher": "r",
        "commands": entries,
        "robot": _robot_catalog(),
        "capture": {
            "default_hz": DEFAULT_CAPTURE_HZ,
            "hz": list(CAPTURE_HZ_VALUES),
            "modes": ["alap", "full", "nincs"],
        },
        "tests": {
            "modes": launcher_extras.test_modes(),
            "packs": list(launcher_extras.test_runner._pack_names()),
        },
        "evidence": {"command": "evi", "operations": list(OPERATIONS), "default_source": "latest"},
        "public_robot": launcher_extras.public_catalog(root),
        "er2": {
            "commands": sorted(launcher_extras._subparsers(er2_parser()).choices),
            "default_command": "stream",
            "stream_defaults": ["camera", "tools", "speak", "json"],
            "stream_options": sorted(option for action in launcher_extras._subparsers(er2_parser()).choices["stream"]._actions
                                     for option in action.option_strings if option not in {"-h", "--help"}),
            "execution": "REAL_ONLY",
        },
        "camera": {
            "command": "cam",
            "aliases": ["camera"],
            "operations": sorted(launcher_extras._subparsers(launcher_extras.camera_parser()).choices),
            "runtime": "NONE",
            "images": "CALIBRATED_ONLY",
        },
        "plain_llm": {
            "usage": 'r "PROMPT"',
            "escape_usage": 'r -- "PROMPT"',
            "provider": "chatgpt_oauth+failover",
            "tts": "default",
            "runtime": "NONE",
        },
        "execution_router": {
            "usage": 'r "REQUEST"',
            "dry_run": 'r route "REQUEST" --json',
            "modes": ["AGENT", "DIRECT_V3"],
            "evidence": "runtime/execution_routes.ndjson",
        },
        "chatgpt": {
            "command": "chatgpt",
            "operations": sorted(launcher_extras._subparsers(chatgpt_parser()).choices),
            "auth": "OAUTH_PRIMARY_API_KEY_OPTIONAL",
            "runtime": "NONE",
        },
        "voice": {
            "command": "voice",
            "aliases": ["wake"],
            "operations": list(launcher_extras.VOICE_OPERATIONS),
            "service": launcher_extras.VOICE_UNIT,
            "authority": "USER_SYSTEMD_SERVICE",
        },
        "completion": {
            "shell": "bash",
            "source": "deploy/bash-completion/r",
            "install": "r install",
            "ssot": "command_entries + owning parsers/capability catalogs",
        },
        "local": sorted(host_cli.COMMAND_HELP),
        "local_details": [item for item in entries if item["group"] == "host"],
    }


def print_help() -> None:
    print(
        "R2B4 — robot, diagnosztika, fejlesztés\n"
        "Használat: r PARANCS [ARGUMENTUMOK]\n"
        "\nGyakori parancsok:\n"
        "  r s / r d                 Rövid állapot / részletes élő runtime-diagnosztika\n"
        "  r x / r sd                STOP / STOP és runtime-leállítás\n"
        "  r skill robot_skills/terkepesz_r2b4.py   Mentett Python-skill indítása\n"
        "  r skill                   Mentett skillek listája\n"
        "  r evi                     Legutolsó lezárt MCAP → evidence\n"
        "  r diag                    Legutolsó evidence → teljes offline diagnosztika\n"
        "  r help PARANCS            A parancs saját részletes súgója\n"
        "  r commands [--json]       Teljes parancs- és capability-katalógus\n"
        "\nMozgás — az első szám az idő másodpercben:\n"
        "  r rc 30 / r fp 20 / r fa 20\n"
        "  r f 10 0.15 / r b 10 0.15  Előre / hátra [m/s]\n"
        "  r t 10 0.15 -0.20          Haladás [m/s], fordulás [rad/s]\n"
        "  r m 8 0.10 0.20            Bal/jobb keréksebesség-cél [m/s]\n"
        "  0 s = folyamatos; leállítás: r x / r sd. Időzített futás végén STOP;\n"
        "  a parancs saját runtime-ja leáll, a már futó runtime megmarad.\n"
        f"  Capture: c HZ ({'/'.join(map(str, CAPTURE_HZ_VALUES))}; alap: {DEFAULT_CAPTURE_HZ} Hz)\n"
        "  c alap / c full / c nincs; nc = mozgás-trigger kihagyása.\n"
        "\nSzöveges kérés: r \"KÉRÉS\"; sima LLM: r -- \"PROMPT\"\n"
        "TAB: kontextusfüggő súgó és kiegészítés az r install után."
    )
    print("\nElérhető parancsok (az aktuális source alapján):")
    for item in command_entries():
        aliases = f" ({', '.join(item['aliases'])})" if item["aliases"] else ""
        print(f"  {item['name'] + aliases:<30} {item['description']}")


def _commands(argv: list[str]) -> int:
    if argv in (["-h"], ["--help"]):
        print("Használat: r commands [--json]\nTeljes parancslista rövidítésekkel és leírással.")
        return 0
    if argv not in ([], ["--json"]):
        raise LauncherError("usage: r commands [--json]")
    catalog = command_catalog()
    if argv == ["--json"]:
        print(json.dumps(catalog, ensure_ascii=False, indent=2, sort_keys=True))
        return 0
    print("R2B4 parancsok:")
    for item in catalog["commands"]:
        aliases = f" ({', '.join(item['aliases'])})" if item["aliases"] else ""
        motion = " [motion]" if item.get("motion") else ""
        print(f"  {item['usage']}{aliases}{motion}\n    {item['description']}")
    capture = catalog["capture"]
    print(
        f"\nCapture: default {capture['default_hz']} Hz | rates "
        + "/".join(map(str, capture["hz"]))
        + " | modes " + "/".join(capture["modes"])
    )
    print("Test modes: " + ", ".join(catalog["tests"]["modes"]))
    print("Evidence: r evi [MCAP|latest] | r evi verify BUNDLE | r evi query BUNDLE")
    print("\nER2: r er2 " + "|".join(catalog["er2"]["commands"]) + '; röviden: r er2 "FELADAT"')
    print("Camera: r cam " + "|".join(catalog["camera"]["operations"]) + "; r help cam")
    print('Agent route: r "REQUEST"; dry-run: r route "REQUEST" --json')
    print('Plain LLM escape: r -- "PROMPT"')
    print("Voice wake: r voice " + "|".join(catalog["voice"]["operations"]) + " (alias: r wake ...)")
    print("TAB help: r install telepíti a Bash completiont")
    public = catalog["public_robot"]
    print("\nPublikus capability-k (élő elérhetőség: r caps):")
    print("  r read RESOURCE --json\n    " + ", ".join(public["resources"]))
    print("  r execute ACTION --parameters JSON --json\n    " + ", ".join(public["actions"]))
    print("  Mentett Python-skillek: " + (", ".join(item["skill_name"] for item in public["python_skills"]) or "(nincs)"))
    return 0


def _plain_prompt(argv: Sequence[str], root: Path) -> int:
    prompt = " ".join(argv).strip()
    if not prompt:
        raise LauncherError('Használat: r "PROMPT" vagy r -- "PROMPT"')
    from r2b4_voice.plain_llm import run_plain_prompt
    return run_plain_prompt(prompt, project_root=root)


def _auto_prompt(argv: Sequence[str], root: Path) -> int:
    prompt = " ".join(argv).strip()
    if not prompt:
        raise LauncherError('Használat: r "KÉRÉS"')
    from r2b4_orchestration.executor import execute_text
    return execute_text(prompt, project_root=root, source="launcher")


def _route(argv: Sequence[str], root: Path) -> int:
    args = list(argv)
    if any(arg in {"-h", "--help"} for arg in args):
        print('Használat: r route "KÉRÉS" [--json]\nBelépési route megtekintése a kérés végrehajtása nélkül.')
        return 0
    json_output = "--json" in args
    args = [item for item in args if item != "--json"]
    prompt = " ".join(args).strip()
    if not prompt:
        raise LauncherError('Használat: r route "KÉRÉS" [--json]')
    from r2b4_orchestration.execution_mode import ExecutionModeSelector, RouteEvidenceJournal
    plan = ExecutionModeSelector().select(prompt, source="launcher-dry-run")
    RouteEvidenceJournal(root).emit("ROUTE_DRY_RUN", plan)
    payload = plan.to_jsonable()
    if json_output:
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(
            f"mode={payload['mode']} requires_v3={payload['requires_v3']} "
            f"capability={payload['capability']} reason={payload['reason']}"
        )
    return 0


def _command_names() -> set[str]:
    return {name for item in command_entries() for name in (item["name"], *item["aliases"])}


def _looks_like_command_typo(command: str) -> bool:
    # Keep short/free text such as `r "hello"` on the plain-LLM path, while
    # obvious near-misses such as `statuz` fail locally instead of reaching Gemini.
    return bool(
        difflib.get_close_matches(
            command, sorted(_command_names()), n=1, cutoff=0.8
        )
    )


def _unknown_command(command: str) -> int:
    matches = difflib.get_close_matches(
        command, sorted(_command_names()), n=3, cutoff=0.6
    )
    print(f"Ismeretlen parancs: {command!r}.", file=sys.stderr)
    if matches:
        print("Erre gondoltál? " + ", ".join(f"r {match}" for match in matches), file=sys.stderr)
    print("Súgó: r help | Teljes lista: r commands", file=sys.stderr)
    return 2


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    try:
        root = project_root()
        from v3.runtime_performance import apply_host_affinity
        role = "diagnostics" if args and args[0] in {"pytest", "tests", "test", "tune", "tool", "cpu", "cpu2", "diag", "evi", "learn"} else "operator"
        apply_host_affinity(root, role)

        # Private, read-only shell completion transport. It is intentionally not
        # listed as a user command and must run before any command normalization.
        if args and args[0] == "__complete":
            return launcher_extras.emit_completion(args[1:], root)

        # Explicit escape: after `--`, even a text equal to an existing r command
        # is treated as a plain LLM prompt.
        if args and args[0] == "--":
            return _plain_prompt(args[1:], root)

        if not args or args[0] in {"help", "-h", "--help"}:
            if len(args) <= 1:
                print_help()
                return 0
            # Ask the owning parser for help; never execute a help topic.
            args = args[1:]
            if args[0] not in _command_names():
                return _unknown_command(args[0])
            if args[0] == "route":
                return _route(["--help"], root)
            if args[0] in {"voice", "wake"}:
                return launcher_extras.voice_command(["--help"], root)
            if host_cli.ALIASES.get(args[0], args[0]) in host_cli.COMMAND_HELP:
                if len(args) != 1:
                    raise LauncherError("Használat: r help PARANCS")
                if args[0] == "diag":
                    from tools.diag.cli import main as diag_main
                    return diag_main(["--help"], project_root=root)
                if args[0] == "install":
                    print(
                        "Használat: r install\n\n"
                        "Telepíti az r launcher symlinket, a Bash TAB help/completiont és "
                        "a voice wake user-systemd unitot. A voice wake-et nem engedélyezi "
                        "automatikusan; bekapcsolás: r voice on."
                    )
                else:
                    host_cli.print_help(args[0])
                return 0
            args.append("--help")
        args = _normalize_er2_args(args)
        if args[0] in {"th", "testhub"}:
            raise LauncherError("A Test Hub megszűnt. Offline evidence: r evi CAPTURE.mcap")
        if args[0] == "evi":
            from tools.mcap_evidence.cli import main as evidence_main
            return evidence_main(args[1:], project_root=root)
        if args[0] == "learn":
            from r2b4_orchestration.learning_cli import main as learning_main
            return learning_main(args[1:], project_root=root)
        if args[0] == "commands":
            return _commands(args[1:])
        if args[0] == "route":
            return _route(args[1:], root)
        if args[0] == "chatgpt":
            from r2b4_voice.openai_oauth import cli_main as chatgpt_oauth_main
            return chatgpt_oauth_main(list(args[1:]), project_root=root)
        if args[0] == "voice" or args[0] == "wake":
            return launcher_extras.voice_command(args[1:], root)
        if args[0] in {"cam", "camera"}:
            return launcher_extras.camera_command(args[1:], root)
        if args[0] == "er2":
            # R2B4_ER2_P0_20260925: provider integration is a consumer of the
            # canonical RobotInterface/ExternalRobotGateway, not a robot layer.
            from r2b4_er2.cli import main as er2_main
            return er2_main(args[1:], project_root=root)
        if args[0] == "install":
            rc = host_cli.execute(args[0], args[1:], root)
            if rc == 0 and not args[1:]:
                return launcher_extras.install_extras(root)
            return rc
        if host_cli.ALIASES.get(args[0], args[0]) in host_cli.COMMAND_HELP:
            return host_cli.execute(args[0], args[1:], root)
        if not args[0].startswith("-"):
            known = _command_names()
            if args[0] not in known:
                if _looks_like_command_typo(args[0]):
                    return _unknown_command(args[0])
                return _auto_prompt(args, root)
        return interface_cli.main(args, project_root=root)
    except SystemExit as exc:
        # argparse help and usage errors also behave as return codes for callers.
        return int(exc.code or 0)
    except KeyboardInterrupt:
        return 130
    except (LauncherError, host_cli.HostCliError, OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["command_catalog", "main", "project_root", "_normalize_er2_args"]
