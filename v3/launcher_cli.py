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
from v3.test_runner import FOCUSED


class LauncherError(RuntimeError):
    pass


_ER2_CANONICAL_COMMANDS = frozenset({"status", "preview", "stream", "-h", "--help"})


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
    if args[1] in _ER2_CANONICAL_COMMANDS:
        return args
    return ["er2", "stream", *args[1:]]


def _robot_catalog() -> list[dict[str, object]]:
    parser = interface_cli._parser()
    subparsers = next(
        action for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    aliases = dict(interface_cli.ALIASES)
    # Top-level r diag is owned by the offline DIAG facade; the existing
    # detailed runtime status remains reachable through the d alias.
    canonical = sorted((set(subparsers.choices) - set(aliases)) - {"diag"})
    return [
        {
            "name": name,
            "aliases": sorted(alias for alias, target in aliases.items() if target == name),
            "motion": name in interface_cli.MOTION_COMMANDS,
            "description": subparsers.choices[name].description,
            "usage": subparsers.choices[name].format_usage().strip(),
        }
        for name in canonical
    ]


def command_catalog() -> dict[str, object]:
    from r2b4_orchestration.person_skills import PERSON_SKILLS
    from r2b4_orchestration.robot_runtime import READS, QUERIES, ACTIONS
    return {
        "launcher": "r",
        "robot": _robot_catalog(),
        "capture": {
            "default_hz": DEFAULT_CAPTURE_HZ,
            "hz": list(CAPTURE_HZ_VALUES),
            "modes": ["alap", "full", "nincs"],
        },
        "tests": {
            "modes": ["quick", *FOCUSED, "full", "core", "feature", "deep"],
        },
        "evidence": {"command": "evi", "operations": ["compile", "verify", "query"]},
        "public_robot": {
            "resources": sorted(READS), "queries": sorted(QUERIES), "actions": sorted(ACTIONS),
            "person_skills": [skill.to_jsonable() for skill in PERSON_SKILLS],
        },
        "er2": {
            "commands": ["status", "preview", "stream"],
            "default_command": "stream",
            "stream_defaults": ["camera", "tools", "speak", "json"],
            "stream_options": ["--camera", "--tools", "--speak", "--json", "--seconds"],
            "execution": "REAL_ONLY",
        },
        "camera": {
            "command": "cam",
            "aliases": ["camera"],
            "operations": list(launcher_extras.CAMERA_OPERATIONS),
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
            "operations": ["status", "login", "host-id", "import", "logout"],
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
            "ssot": "launcher/interface/ER2 parsers",
        },
        "local": sorted(set(host_cli.COMMANDS) - set(host_cli.ALIASES)),
        "local_details": [
            {
                "name": name,
                "usage": f"r {name} {usage}".rstrip(),
                "description": (
                    "Az r launcher, Bash TAB help/completion és voice user-service unit telepítése; "
                    "a voice wake-et nem kapcsolja be."
                    if name == "install" else description
                ),
                "aliases": sorted(alias for alias, target in host_cli.ALIASES.items() if target == name),
            }
            for name, (usage, description) in sorted(host_cli.COMMAND_HELP.items())
        ],
    }


def print_help() -> None:
    print(
        "R2B4 — robot, diagnosztika, fejlesztés\n"
        "Használat: r PARANCS [ARGUMENTUMOK]\n"
        "\nÁllapot és leállítás:\n"
        "  r s / r d                  Rövid állapot / részletes diagnosztika\n"
        "  r x                        Mozgás STOP, runtime megmarad\n"
        "  r sd / r panic             STOP és runtime-leállítás\n"
        "\nMozgás — az első szám mindig az idő másodpercben:\n"
        "  r rc 30                    Room Cruise, 30 s\n"
        "  r fp 20 / r fa 20          Személykövetés / személy felé fordulás\n"
        "  r f 10 0.15 / r b 10 0.15  Előre / hátra, 10 s, 0.15 m/s\n"
        "  r t 10 0.15 -0.20          Haladás [m/s] és fordulás [rad/s]\n"
        "  r m 8 0.10 0.20            Bal/jobb keréksebesség-cél [m/s]\n"
        "  r pr                       Integrált fizikai mozgásteszt\n"
        "  0 s = folyamatos. Időzített futás végén STOP; a saját runtime leáll,\n"
        "  a már futó runtime megmarad. Folyamatos mód leállítása: r x / r sd.\n"
        "\nRuntime, capture és elemzés:\n"
        "  r rt start|stop|status|diag Kézi runtime-kezelés\n"
        "  r cap start|stop|status     Capture-kezelés\n"
        f"  r rc 30 c 10               Capture Hz: {' / '.join(map(str, CAPTURE_HZ_VALUES))}; alap: {DEFAULT_CAPTURE_HZ} Hz\n"
        "  c alap / c full / c nincs  Capture-mód; nc = mozgás-trigger kihagyása\n"
        "  r evi CAPTURE.mcap         Offline MCAP Evidence Compiler\n"
        "  r diag [full|ANALYZER]     EVI diagnosztikai adat; alap: full/latest; artifact: runtime/diag/\n"
        "  r cam status|on|off        Kameraállapot / manuális igény be / ki; V3 nélkül\n"
        "  r cam photo OUTPUT         Kalibrált JPEG; videó: r cam video OUTPUT [SECONDS]\n"
        "\nAI, voice, fejlesztés és gépállapot:\n"
        "  r \"KÉRÉS\"                 Agent Core: LLM + R2B4 toolok; exact STOP lokális\n"
        "  r route \"KÉRÉS\" --json    Belépési route: STOP vagy AGENT\n"
        "  r -- \"s\"                   Sima LLM prompt (ChatGPT OAuth + failover)\n"
        "  r er2 \"FELADAT\"            ER2 stream; camera/tools/speak/json bekapcsolva\n"
        "  r er2 status|preview|stream ER2 részletes parancsok\n"
        "  r voice status|on|off      Voice wake állapot / bekapcsolás / kikapcsolás\n"
        "  r voice restart|check      Voice service újraindítás / diagnosztika\n"
        "  r chatgpt status|login     ChatGPT OAuth állapot / böngészős belépés\n"
        "  r test [MODE]              Tesztek; alap: kis gate; release: r test release\n"
        "  r tune roomcruise [OPCIÓK] Offline RoomCruise hangolás; runtime/tunes/\n"
        "  r pytest [ARGS...]         Nyers pytest\n"
        "  r git / r gitre            Git-segédek\n"
        "  r tools / r tool NAME      Python segédprogramok\n"
        "  r host / r sys             Gépállapot\n"
        "  r cpu / cpu2 / disc / mem / temp / ps / net / usb / i2c\n"
        "  r install / where / root / version\n"
        "\nSegítség és gépi használat:\n"
        "  TAB                        Kontextusfüggő gyors help + kiegészítés (r install után)\n"
        "  r help PARANCS             Célzott súgó; például: r help fp\n"
        "  r commands [--json]        Teljes parancslista és rövidítések\n"
        "  r caps                     Élő robotképességek\n"
        "  r read brain.history --json Korrelált részfeladat- és céleredmények\n"
        "  r read world.snapshot --json Tudás, binding, tanulás és megőrzési keretek\n"
        "  r execute person.teach --parameters JSON --json Explicit név–track tanítás\n"
        "  r s --json                 Robotparancs JSON-eredménye; folyamatjelzés stderr-en"
    )


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
    print("Robot commands:")
    for item in catalog["robot"]:
        aliases = f" ({', '.join(item['aliases'])})" if item["aliases"] else ""
        motion = " [motion]" if item["motion"] else ""
        print(f"  {item['name']}{aliases}{motion}\n    {item['description']}")
    capture = catalog["capture"]
    print(
        f"\nCapture: default {capture['default_hz']} Hz | rates "
        + "/".join(map(str, capture["hz"]))
        + " | modes " + "/".join(capture["modes"])
    )
    print("Test modes: " + ", ".join(catalog["tests"]["modes"]))
    print("Evidence: r evi MCAP | r evi verify BUNDLE | r evi query BUNDLE")
    print("\nER2: r er2 status|preview|stream; röviden: r er2 \"FELADAT\"")
    print("Camera: r cam status|on|off|photo OUTPUT|video OUTPUT [SECONDS]; V3 nélkül")
    print('Agent route: r "REQUEST"; dry-run: r route "REQUEST" --json')
    print('Plain LLM escape: r -- "PROMPT"')
    print("Voice wake: r voice status|on|off|restart|check  (alias: r wake ...)")
    print("TAB help: r install telepíti a Bash completiont")
    print("\nHost / fejlesztés:")
    for item in catalog["local_details"]:
        aliases = f" ({', '.join(item['aliases'])})" if item["aliases"] else ""
        print(f"  {item['usage']}{aliases}\n    {item['description']}")
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
    known = {item["name"] for item in _robot_catalog()} | set(interface_cli.ALIASES)
    known |= set(host_cli.COMMANDS) | {"chatgpt", "er2", "voice", "wake", "help", "commands", "route", "evi"}
    return known


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
        role = "diagnostics" if args and args[0] in {"pytest", "tests", "test", "tune", "tool", "cpu", "cpu2", "diag", "evi"} else "operator"
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
            if args[0] in host_cli.COMMANDS:
                if len(args) != 1:
                    raise LauncherError("Használat: r help PARANCS")
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
            return evidence_main(args[1:])
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
        if args[0] in host_cli.COMMANDS:
            return host_cli.execute(args[0], args[1:], root)
        if not args[0].startswith("-"):
            known = {item["name"] for item in _robot_catalog()} | set(interface_cli.ALIASES)
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
