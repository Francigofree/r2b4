"""Launcher-only completion and host service helpers for :mod:`v3.launcher_cli`.

This module owns no robot state and never talks to motor/safety paths.  TAB
completion introspects the existing CLI parsers; voice wake lifecycle is delegated
to the existing user-systemd unit which starts ``r2b4_voice.voice_service``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from collections.abc import Sequence

from v3 import host_cli, interface_cli
from v3 import test_runner

VOICE_UNIT = "r2b4-wake.service"
VOICE_OPERATIONS = ("status", "on", "off", "restart", "check")
CAMERA_OPERATIONS = ("status", "on", "off", "photo", "video")
VOICE_ALIASES = {
    "be": "on",
    "enable": "on",
    "start": "on",
    "ki": "off",
    "disable": "off",
    "stop": "off",
}


def _subparsers(parser: argparse.ArgumentParser) -> argparse._SubParsersAction:
    return next(
        action for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )


def _canonical_robot_commands() -> list[str]:
    sub = _subparsers(interface_cli._parser())
    aliases = set(interface_cli.ALIASES)
    return sorted(set(sub.choices) - aliases)


def _top_level_commands() -> list[str]:
    from v3.launcher_cli import command_entries
    return sorted({name for item in command_entries() for name in (item["name"], *item["aliases"])})


def test_modes() -> list[str]:
    return sorted(set(test_runner.command_modes()))


def public_catalog(root: Path) -> dict[str, object]:
    """Supported public names from their owners, without runtime/device queries.

    Local skill discovery parses source metadata only. Live availability remains
    the explicit r caps operation; completion never starts or queries an owner.
    """
    from r2b4_orchestration.person_skills import PERSON_SKILLS
    from r2b4_orchestration.robot_runtime import READS, QUERIES, ACTIONS, SKILL_READS, skill_capabilities
    from r2b4_orchestration.skill_library import SkillLibrary
    from r2b4_orchestration.skill_observation import SkillObservationAdapter
    from r2b4_orchestration.software_interface import SoftwareInterfaceAdapter
    from v3.action_catalog import ACTION_CATALOG
    from v3.interface_adapters import build_adapters

    descriptors = skill_capabilities(SkillLibrary(root / "robot_skills"))
    descriptors.update(SoftwareInterfaceAdapter(root).capabilities())
    actions = set(ACTIONS) | set(ACTION_CATALOG) | SkillObservationAdapter.capability_names
    actions |= {name for name, item in descriptors.items() if item["kind"] == "action"}
    reads = set(READS | SKILL_READS)
    # The existing composition only constructs thin adapters. Inspect their
    # static names, never capabilities(), which would query live owner state.
    for adapter in build_adapters(None, root):
        reads.update(adapter.read_capability_names)
        actions.update(adapter.capability_names - adapter.read_capability_names)
    return {
        "resources": sorted(reads), "queries": sorted(QUERIES),
        "actions": sorted(actions),
        "person_skills": [skill.to_jsonable() for skill in PERSON_SKILLS],
        "python_skills": [item for item in descriptors.values() if "skill_name" in item],
        "descriptors": descriptors,
        "discovery": "LOCAL_SOURCE_ONLY; live availability: r caps",
    }


def _filter(candidates: Sequence[str], current: str) -> list[str]:
    return sorted({value for value in candidates if value.startswith(current)})


def _parser_candidates(
    parser: argparse.ArgumentParser,
    before: Sequence[str],
    current: str,
    *,
    extra: Sequence[str] = (),
) -> list[str]:
    option_actions: dict[str, argparse.Action] = {}
    option_strings: list[str] = []
    positional_choices: list[str] = []
    for action in parser._actions:
        for option in action.option_strings:
            option_actions[option] = action
            if option not in {"-h"}:
                option_strings.append(option)
        if not action.option_strings and action.choices is not None:
            positional_choices.extend(str(value) for value in action.choices)

    if before:
        previous = before[-1]
        action = option_actions.get(previous)
        if action is not None and action.choices is not None:
            return _filter([str(value) for value in action.choices], current)

    if current.startswith("--capture-hz="):
        prefix = "--capture-hz="
        return _filter([prefix + str(v) for v in interface_cli.CAPTURE_HZ_VALUES], current)
    if current.startswith("--capture-mode="):
        prefix = "--capture-mode="
        return _filter([prefix + value for value in ("alap", "full", "nincs")], current)

    candidates: list[str] = list(extra)
    if current.startswith("-") or not current:
        candidates.extend(option_strings)
    if not current.startswith("-"):
        candidates.extend(positional_choices)
    return _filter(candidates, current)


def _subcommand_completion(
    parser: argparse.ArgumentParser, before: Sequence[str], current: str,
) -> tuple[str, list[str]]:
    """Follow selected subparsers so their own operations/options stay current."""
    tail = list(before)
    while True:
        sub = next((action for action in parser._actions
                    if isinstance(action, argparse._SubParsersAction)), None)
        if sub is None or not tail or tail[0] not in sub.choices:
            return parser.format_usage().strip(), _parser_candidates(parser, tail, current)
        parser = sub.choices[tail.pop(0)]


def _robot_completion(command: str, before: Sequence[str], current: str,
                      *, root: Path | None = None) -> tuple[str, list[str]]:
    canonical = interface_cli.ALIASES.get(command, command)
    parser = interface_cli._parser()
    child = _subparsers(parser).choices.get(canonical)
    if child is None:
        return "", []

    extra: list[str] = []
    if canonical in {"read", "execute"} and not before:
        catalog = public_catalog(root or Path(__file__).resolve().parents[1])
        extra.extend(catalog["resources" if canonical == "read" else "actions"])
    if canonical == "skill" and not before:
        catalog = public_catalog(root or Path(__file__).resolve().parents[1])
        extra.extend(item["skill_name"] for item in catalog["python_skills"])
    capture_commands = set(interface_cli.MOTION_COMMANDS) | {"runtime", "capture", "proba"}
    if canonical in capture_commands:
        if before and before[-1] in {"c", "--capture"}:
            values = [*(str(v) for v in interface_cli.CAPTURE_HZ_VALUES), "alap", "full", "nincs"]
            return child.format_usage().strip(), _filter(values, current)
        if before and before[-1] == "--capture-hz":
            return child.format_usage().strip(), _filter([str(v) for v in interface_cli.CAPTURE_HZ_VALUES], current)
        if before and before[-1] == "--capture-mode":
            return child.format_usage().strip(), _filter(["alap", "full", "nincs"], current)
        extra.extend(("c", "nc", "--capture-hz", "--capture-mode", "--no-trigger"))

    return child.format_usage().strip(), _parser_candidates(child, before, current, extra=extra)


def _er2_completion(before: Sequence[str], current: str) -> tuple[str, list[str]]:
    from r2b4_er2.cli import _parser as er2_parser

    parser = er2_parser()
    sub = _subparsers(parser)
    commands = tuple(sub.choices)
    if not before:
        return parser.format_usage().strip() + ' | r er2 "FELADAT"', _filter(commands, current)

    first = before[0]
    if first in commands:
        child = sub.choices[first]
        tail = before[1:]
    else:
        # Human shorthand: r er2 "TASK" is normalized to stream by launcher_cli.
        child = sub.choices["stream"]
        tail = before[1:]
    return child.format_usage().strip(), _parser_candidates(child, tail, current)


def camera_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="r cam",
        description="Önálló kamera: igényvezérelt, kalibrált kép; V3 és LLM nélkül.",
    )
    sub = parser.add_subparsers(dest="operation")
    for operation, description in (
        ("status", "Kamera- és consumerállapot; nem kapcsolja be a kamerát."),
        ("on", "Manuális kameraigény bekapcsolása."),
        ("off", "Manuális kameraigény elengedése; más consumer megmarad."),
    ):
        child = sub.add_parser(operation, help=description, description=description)
        child.add_argument("--json", action="store_true")
    photo = sub.add_parser("photo", help="Friss kalibrált JPEG készítése.")
    photo.add_argument("output", help="JPEG célfájl (.jpg / .jpeg)")
    photo.add_argument("--json", action="store_true")
    video = sub.add_parser("video", help="Kalibrált videó készítése.")
    video.add_argument("output", help="Videó célfájl (.mp4 / .h264)")
    video.add_argument("seconds", type=float, nargs="?", default=10.0)
    video.add_argument("--json", action="store_true")
    return parser


def camera_command(argv: Sequence[str], root: Path) -> int:
    from v3.adapters.camera_media import capture_h264_video, capture_photo
    from v3.adapters.vision_media_socket import VisionClient

    args = camera_parser().parse_args(list(argv) if argv else ["status"])
    client = VisionClient(root=root)
    if args.operation == "status":
        result = client.status()
    elif args.operation in {"on", "off"}:
        result = client.set_manual_demand(args.operation == "on")
    elif args.operation == "photo":
        result = capture_photo(args.output, root=root)
    elif args.operation == "video":
        result = capture_h264_video(args.output, args.seconds, root=root)
    else:
        raise ValueError("usage: r cam status|on|off|photo OUTPUT|video OUTPUT [SECONDS]")
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def _camera_completion(before: Sequence[str], current: str) -> tuple[str, list[str]]:
    parser = camera_parser()
    if not before:
        return parser.format_usage().strip(), _filter(tuple(_subparsers(parser).choices), current)
    child = _subparsers(parser).choices.get(before[0])
    if child is None:
        return parser.format_usage().strip(), []
    return child.format_usage().strip(), _parser_candidates(child, before[1:], current)


def _host_completion(command: str, before: Sequence[str], current: str, root: Path) -> tuple[str, list[str]]:
    canonical = host_cli.ALIASES.get(command, command)
    usage, description = host_cli.COMMAND_HELP.get(canonical, ("", ""))
    hint = f"r {canonical} {usage}".rstrip()
    if description:
        hint += f" — {description}"

    if canonical == "test":
        if before and before[0] == "pack" and len(before) == 1:
            return hint, _filter(test_runner._pack_names(), current)
        return hint, _filter(test_modes(), current) if not before else []
    if canonical == "tool" and not before:
        names: set[str] = set()
        for parent in (root / "tools", root):
            if parent.is_dir():
                names.update(path.stem for path in parent.glob("*.py"))
        return hint, _filter(sorted(names), current)
    if canonical == "tune":
        if not before:
            return hint, _filter(["roomcruise", "simulate"], current)
        if before[0] == "simulate":
            from tools.tuners.r2b4_motion_tuner import parser as simulation_parser
            return "r tune simulate [OPCIÓK]", _parser_candidates(
                simulation_parser(), before[1:], current
            )
        if before[0] == "roomcruise":
            from tools.tuners.r2b4_roomcruise_tuner import parser as tuner_parser
            return "r tune roomcruise [OPCIÓK]", _parser_candidates(
                tuner_parser(), before[1:], current
            )
        return hint, []
    if canonical == "diag":
        # Registry-driven completion is evidence-only and does not open a bundle.
        from tools.diag.registry import build_default_registry

        analyzers = list(build_default_registry().ids())
        if not before:
            return hint, _filter(["full", "list", "admission", "--json", "--no-save", *analyzers], current)
        if before[0] == "admission" and len(before) == 1:
            return hint, _filter(analyzers, current)
        evidence_position = (
            before[0] == "full" and len(before) == 1
        ) or (
            before[0] in analyzers and len(before) == 1
        ) or (
            before[0] == "admission" and len(before) == 2 and before[1] in analyzers
        )
        if evidence_position:
            evidence = ["latest", "--json", "--no-save"]
            capture_dir = root / "runtime" / "captures"
            if capture_dir.is_dir():
                evidence.extend(
                    str(path.relative_to(root))
                    for path in sorted(capture_dir.glob("*.evidence"))
                    if path.is_dir() and not path.is_symlink() and (path / "manifest.json").is_file()
                )
            return hint, _filter(evidence, current)
    return hint, []


def completion(cword: int, words: Sequence[str], root: Path) -> tuple[str, list[str]]:
    """Return a context hint and safe completion candidates.

    ``cword`` is Bash ``COMP_CWORD`` (which includes the command itself), while
    ``words`` contains ``COMP_WORDS[1:]``.  This function only introspects parsers
    and files; it never executes a robot, host diagnostic, ER2 request or service.
    """
    relative = max(0, int(cword) - 1)
    values = list(words)
    while len(values) <= relative:
        values.append("")
    current = values[relative]
    before = values[:relative]

    if not before:
        hint = "R2B4 parancsok — súgó: r help PARANCS | teljes lista: r commands"
        return hint, _filter(_top_level_commands(), current)

    command = before[0]
    tail = before[1:]

    if command == "help":
        if not tail:
            return "r help PARANCS — célzott, végrehajtás nélküli súgó", _filter(_top_level_commands(), current)
        if tail[0] == "er2":
            return _er2_completion(tail[1:], current)
        if tail[0] in {"cam", "camera"}:
            return _camera_completion(tail[1:], current)
        if tail[0] == "chatgpt":
            from r2b4_voice.openai_oauth import cli_parser
            return _subcommand_completion(cli_parser(), tail[1:], current)
        return "r help PARANCS", []

    if command == "commands":
        return "r commands [--json] — teljes parancskatalógus", _filter(["--json"], current)

    if command == "route":
        return 'r route "KÉRÉS" [--json]', _filter(["--json"], current)

    if command == "learn":
        from r2b4_orchestration.learning_cli import parser
        selected = parser()
        return selected.format_usage().strip(), _parser_candidates(selected, tail, current)

    if command == "chatgpt":
        from r2b4_voice.openai_oauth import cli_parser
        return _subcommand_completion(cli_parser(), tail, current)

    if command in {"voice", "wake"}:
        if not tail:
            return "r voice status|on|off|restart|check — voice wake user service", _filter(VOICE_OPERATIONS, current)
        return "r voice status [--json]", _filter(["--json"], current) if tail[0] == "status" else []

    if command == "er2":
        return _er2_completion(tail, current)

    if command in {"cam", "camera"}:
        return _camera_completion(tail, current)

    if command == "evi":
        from tools.mcap_evidence.cli import OPERATIONS, VERIFY_OPERATIONS, parser
        operation = tail[0] if tail and tail[0] in {*OPERATIONS, *VERIFY_OPERATIONS} else None
        selected = parser(operation)
        hint = selected.format_usage().strip() if operation else "r evi [MCAP|latest] | compile | verify BUNDLE | query BUNDLE"
        return hint, _parser_candidates(selected, tail[1:] if operation else tail, current,
                                        extra=() if operation else (*OPERATIONS, "latest"))
    if host_cli.ALIASES.get(command, command) in host_cli.COMMAND_HELP:
        return _host_completion(command, tail, current, root)

    canonical = interface_cli.ALIASES.get(command, command)
    if canonical in _canonical_robot_commands():
        return _robot_completion(command, tail, current, root=root)

    return "", []


def emit_completion(argv: Sequence[str], root: Path) -> int:
    """Emit a tiny line protocol consumed by deploy/bash-completion/r."""
    try:
        if not argv:
            return 0
        cword = int(argv[0])
        hint, candidates = completion(cword, argv[1:], root)
        if hint:
            print("HINT\t" + " ".join(hint.splitlines()))
        for candidate in candidates:
            print("CAND\t" + candidate)
    except Exception:
        # Completion must never perturb an interactive shell.
        return 0
    return 0


def _safe_symlink(source: Path, destination: Path, *, keep_regular: bool = True) -> str:
    source = source.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink():
        try:
            if destination.resolve() == source:
                return "present"
        except OSError:
            pass
        destination.unlink()
    elif destination.exists():
        if keep_regular:
            return "existing-file-kept"
        destination.unlink()
    destination.symlink_to(source)
    return "installed"


def _systemctl(argv: Sequence[str], *, capture: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["systemctl", "--user", *argv],
        check=False,
        text=True,
        capture_output=capture,
    )


def install_extras(root: Path) -> int:
    """Install TAB completion and expose the existing voice user-service unit.

    The voice service is deliberately not enabled here; ``r voice on`` is the
    explicit opt-in that enables and starts it.
    """
    completion_source = root / "deploy" / "bash-completion" / "r"
    if not completion_source.is_file():
        print(f"WARNING: completion source missing: {completion_source}", file=sys.stderr)
    else:
        completion_dest = Path.home() / ".local" / "share" / "bash-completion" / "completions" / "r"
        result = _safe_symlink(completion_source, completion_dest)
        print(f"completion: {result}: {completion_dest} -> {completion_source}")

    unit_source = root / "deploy" / "systemd" / VOICE_UNIT
    if unit_source.is_file():
        unit_dest = Path.home() / ".config" / "systemd" / "user" / VOICE_UNIT
        result = _safe_symlink(unit_source, unit_dest, keep_regular=True)
        print(f"voice unit: {result}: {unit_dest}")
        try:
            reload_result = _systemctl(["daemon-reload"])
        except OSError as exc:
            print(f"WARNING: systemctl unavailable: {exc}; voice control is not installed in this session", file=sys.stderr)
        else:
            if reload_result.returncode != 0:
                print("WARNING: systemctl --user daemon-reload failed; voice control may require a login session", file=sys.stderr)
    else:
        print(f"WARNING: voice unit source missing: {unit_source}", file=sys.stderr)

    print("TAB completion: open a new Bash shell, or source ~/.local/share/bash-completion/completions/r")
    print("Voice wake remains unchanged/off unless explicitly requested: r voice on")
    return 0


def _voice_unit_path() -> Path:
    return Path.home() / ".config" / "systemd" / "user" / VOICE_UNIT


def _ensure_voice_unit(root: Path) -> None:
    source = root / "deploy" / "systemd" / VOICE_UNIT
    if not source.is_file():
        raise RuntimeError(f"voice unit source missing: {source}")
    result = _safe_symlink(source, _voice_unit_path(), keep_regular=True)
    if result != "present":
        _systemctl(["daemon-reload"])


def _systemctl_value(*argv: str) -> tuple[str, int]:
    result = _systemctl(argv, capture=True)
    value = (result.stdout or result.stderr or "").strip().splitlines()
    return (value[0] if value else "unknown"), result.returncode


def voice_status(root: Path, *, as_json: bool = False) -> int:
    load, _ = _systemctl_value("show", VOICE_UNIT, "--property=LoadState", "--value")
    enabled, _ = _systemctl_value("is-enabled", VOICE_UNIT)
    active, _ = _systemctl_value("is-active", VOICE_UNIT)
    status_path = root / "runtime" / "wake_status.json"
    snapshot: object | None = None
    if status_path.is_file():
        try:
            snapshot = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            snapshot = None
    payload = {
        "service": VOICE_UNIT,
        "load_state": load,
        "enabled": enabled,
        "active": active,
        "status_file": str(status_path),
        "voice": snapshot,
    }
    if as_json:
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(f"voice wake: active={active} enabled={enabled} load={load}")
        if isinstance(snapshot, dict):
            print(
                "voice state: "
                f"{snapshot.get('state', '?')} pid={snapshot.get('pid', '?')} "
                f"mic={snapshot.get('microphone_state', '?')}"
            )
        print("control: r voice on | off | restart | check")
    return 0


def voice_command(argv: Sequence[str], root: Path) -> int:
    args = list(argv)
    if args in (["-h"], ["--help"]):
        print(
            "Használat: r voice [status|on|off|restart|check] [--json]\n"
            "  status   systemd + voice service állapot (alapértelmezett)\n"
            "  on       user service enable + start (újraindítás után is engedélyezett)\n"
            "  off      user service stop + disable\n"
            "  restart  voice service újraindítása\n"
            "  check    mikrofon/API/LLM/TTS/speaker diagnosztika, service-indítás nélkül\n"
            "Alias: r wake ...; on/off esetén be/ki, enable/disable, start/stop is elfogadott."
        )
        return 0

    operation = args[0] if args else "status"
    operation = VOICE_ALIASES.get(operation, operation)
    rest = args[1:] if args else []
    if operation not in VOICE_OPERATIONS:
        raise ValueError("usage: r voice [status|on|off|restart|check] [--json]")

    if operation == "status":
        if rest not in ([], ["--json"]):
            raise ValueError("usage: r voice status [--json]")
        return voice_status(root, as_json=rest == ["--json"])

    if rest:
        raise ValueError(f"usage: r voice {operation}")

    if operation == "check":
        return subprocess.run(
            [sys.executable, "-m", "r2b4_voice.voice_service", "--check"],
            cwd=root,
            check=False,
        ).returncode

    if operation in {"on", "restart"}:
        _ensure_voice_unit(root)
    if operation == "on":
        return _systemctl(["enable", "--now", VOICE_UNIT]).returncode
    if operation == "off":
        return _systemctl(["disable", "--now", VOICE_UNIT]).returncode
    return _systemctl(["restart", VOICE_UNIT]).returncode


__all__ = [
    "CAMERA_OPERATIONS",
    "VOICE_OPERATIONS",
    "VOICE_UNIT",
    "completion",
    "camera_parser",
    "camera_command",
    "emit_completion",
    "install_extras",
    "voice_command",
    "voice_status",
]
