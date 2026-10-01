"""Human-friendly launcher CLI over :mod:`v3.robot_interface`.

Designed for the short ``./r`` launcher.  Timed motion commands own the whole
operator session: runtime auto-start, motion, STOP, runtime shutdown, then visible
capture finalization.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from v3.action_catalog import (
    FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S,
    FOLLOW_PERSON_DEFAULT_MAX_V_MPS,
)
from v3.capture_rate import CAPTURE_HZ_VALUES, DEFAULT_CAPTURE_HZ, validate_capture_hz
from v3.operator_controller import CAPTURE_MODES, DEFAULT_CAPTURE_MODE, OperatorError, OperatorEvent
from v3.robot_interface import RobotInterface, RobotInterfaceError


ALIASES = {
    "s": "status",
    "d": "diag",
    "ls": "caps",
    "f": "forward",
    "b": "backward",
    "t": "teleop",
    "m": "wheels",
    "mozog": "wheels",
    "rc": "roomcruise",
    "explore": "roomcruise",
    "fa": "faceperson",
    "fp": "followperson",
    "pr": "proba",
    "x": "stop",
    "sd": "shutdown",
    "rt": "runtime",
    "cap": "capture",
    "cam": "camera",
    "sys": "system",
}

MOTION_COMMANDS = frozenset({
    "forward", "backward", "teleop", "wheels", "roomcruise", "faceperson", "followperson"
})


def _event_printer(event: OperatorEvent) -> None:
    stream = sys.stderr if event.kind in {"error", "warning"} else sys.stdout
    print(event.message, file=stream, flush=True)


def _extract_capture_selector(argv: Sequence[str]) -> tuple[list[str], str, int, bool]:
    clean: list[str] = []
    mode = DEFAULT_CAPTURE_MODE
    capture_hz = DEFAULT_CAPTURE_HZ
    no_trigger = False
    mode_explicit = False
    hz_explicit = False
    i = 0
    values = list(argv)

    def set_selector(value: str) -> None:
        nonlocal mode, capture_hz, mode_explicit, hz_explicit
        if value in CAPTURE_MODES:
            if mode_explicit:
                raise ValueError("capture mode may be specified only once")
            mode = value
            mode_explicit = True
            return
        if hz_explicit:
            raise ValueError("capture Hz may be specified only once")
        capture_hz = validate_capture_hz(value)
        hz_explicit = True

    while i < len(values):
        token = values[i]
        if token in {"c", "--capture", "--capture-hz", "--capture-mode"}:
            if i + 1 >= len(values):
                raise ValueError("capture selector requires: 50, 10, 5, 1, alap, full or nincs")
            value = values[i + 1]
            if token == "--capture-hz":
                if hz_explicit:
                    raise ValueError("capture Hz may be specified only once")
                capture_hz = validate_capture_hz(value)
                hz_explicit = True
            elif token == "--capture-mode":
                if mode_explicit or value not in CAPTURE_MODES:
                    raise ValueError("capture mode must be one of: alap, full, nincs")
                mode = value
                mode_explicit = True
            else:
                set_selector(value)
            i += 2
            continue
        if token.startswith("--capture-hz="):
            if hz_explicit:
                raise ValueError("capture Hz may be specified only once")
            capture_hz = validate_capture_hz(token.split("=", 1)[1])
            hz_explicit = True
            i += 1
            continue
        if token.startswith("--capture-mode="):
            value = token.split("=", 1)[1]
            if mode_explicit or value not in CAPTURE_MODES:
                raise ValueError("capture mode must be one of: alap, full, nincs")
            mode = value
            mode_explicit = True
            i += 1
            continue
        if token.startswith("--capture="):
            set_selector(token.split("=", 1)[1])
            i += 1
            continue
        if token in {"nc", "nocapture", "--no-trigger"}:
            no_trigger = True
            i += 1
            continue
        clean.append(token)
        i += 1
    return clean, mode, capture_hz, no_trigger


class _HelpFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
    pass


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="r",
        description="R2B4 robotparancsok. Részletes súgó: r help PARANCS.",
        formatter_class=_HelpFormatter,
        epilog=(
            "Timed examples (first number = seconds):\n"
            "  ./r rc 35              Room Cruise 35 s\n"
            "  ./r fp 20              follow person 20 s\n"
            "  ./r f 10 0.15          forward 10 s at 0.15 m/s\n"
            "  ./r m 8 0.10 0.20      wheel targets for 8 s\n"
            "  ./r t 12 0.15 -0.20    TELEOP 12 s\n\n"
            "Use 0 seconds to leave a command running: ./r rc 0\n"
            f"Capture Hz: {' | '.join(f'c {hz}' for hz in CAPTURE_HZ_VALUES)}   (default: {DEFAULT_CAPTURE_HZ} Hz)\n"
            "Legacy capture mode: c alap | c full | c nincs\n"
            "Skip movement trigger: nc   (runtime shutdown may still finalize its capture slot)\n"
            "Useful: s=status, d=diag, x=STOP, cap=capture, rt=runtime."
        ),
    )
    parser.add_argument("--json", action="store_true", help="egy JSON-eredmény stdout-on; folyamatjelzés stderr-en")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_command(name: str, *, help: str) -> argparse.ArgumentParser:
        child = sub.add_parser(
            name,
            aliases=[alias for alias, target in ALIASES.items() if target == name],
            help=help, description=help, formatter_class=_HelpFormatter,
        )
        child.add_argument("--json", action="store_true", default=argparse.SUPPRESS,
                           help="egy JSON-eredmény stdout-on; folyamatjelzés stderr-en")
        if name in MOTION_COMMANDS or name in {"runtime", "capture", "proba"}:
            child.epilog = (
                f"Capture: c HZ / --capture-hz HZ ({', '.join(map(str, CAPTURE_HZ_VALUES))}; alap: {DEFAULT_CAPTURE_HZ} Hz)\n"
                f"Mód: c MODE / --capture-mode MODE ({', '.join(sorted(CAPTURE_MODES))}; alap: {DEFAULT_CAPTURE_MODE})"
            )
        if name in MOTION_COMMANDS:
            child.epilog += (
                "\nAz első szám az idő másodpercben; 0 = folyamatos. STOP: r x; leállítás: r sd.\n"
                "Időzített futás végén STOP; a parancs által indított runtime leáll, a már futó megmarad.\n"
                "nc / --no-trigger: mozgás-trigger kihagyása; a runtime capture-je még lezárulhat."
            )
        return child

    add_command("status", help="Rövid robot- és runtime-állapot.")
    add_command("diag", help="Részletes diagnosztika.")
    add_command("caps", help="Élő RobotInterface-képességek.")
    add_command("stop", help="Mozgás STOP; a runtime futva marad, ha jelen van.")
    add_command("shutdown", help="STOP + capture-finalizálás + runtime-leállítás.")
    add_command("panic", help="Fail-safe STOP és runtime-leállítás.")
    add_command("proba", help="Integrált fizikai mozgásteszt.")

    runtime = add_command("runtime", help="Kézi runtime-indítás, leállítás és állapot.")
    runtime.add_argument("operation", choices=("start", "stop", "status", "diag"))

    capture = add_command("capture", help="Capture indítása, leállítása és állapota.")
    capture.add_argument("operation", choices=("start", "stop", "status"))

    forward = add_command("forward", help="Előre: r f [SECONDS] [SPEED_MPS]. Példa: r f 10 0.15")
    forward.add_argument("seconds", type=float, nargs="?", default=0.0, help="idő [s]; 0 = folyamatos")
    forward.add_argument("speed", type=float, nargs="?", default=0.15, help="sebesség [m/s]")

    backward = add_command("backward", help="Hátra: r b [SECONDS] [SPEED_MPS]. Példa: r b 10 0.15")
    backward.add_argument("seconds", type=float, nargs="?", default=0.0, help="idő [s]; 0 = folyamatos")
    backward.add_argument("speed", type=float, nargs="?", default=0.15, help="sebesség [m/s]")

    teleop = add_command("teleop", help="Testsebesség: r t SECONDS V_MPS OMEGA_RAD_S")
    teleop.add_argument("seconds", type=float, help="idő [s]; 0 = folyamatos")
    teleop.add_argument("v", type=float, help="haladási sebesség [m/s]")
    teleop.add_argument("omega", type=float, help="szögsebesség [rad/s]")
    teleop.add_argument("--max-v", type=float, default=0.50, help="sebességkorlát [m/s]")
    teleop.add_argument("--max-omega", type=float, default=1.20, help="szögsebességkorlát [rad/s]")

    wheels = add_command("wheels", help="Keréksebesség-célok: r m SECONDS LEFT_MPS RIGHT_MPS")
    wheels.add_argument("seconds", type=float, help="idő [s]; 0 = folyamatos")
    wheels.add_argument("left", type=float, help="bal kerék célsebessége [m/s]")
    wheels.add_argument("right", type=float, help="jobb kerék célsebessége [m/s]")

    room = add_command("roomcruise", help="Room Cruise: r rc [SECONDS]. Példa: r rc 30 c 10")
    room.add_argument("seconds", type=float, nargs="?", default=0.0, help="idő [s]; 0 = folyamatos")

    face = add_command("faceperson", help="Személy felé fordulás: r fa [SECONDS]")
    face.add_argument("seconds", type=float, nargs="?", default=0.0, help="idő [s]; 0 = folyamatos")
    face.add_argument("--max-omega", type=float, default=0.50, help="szögsebességkorlát [rad/s]")

    follow = add_command("followperson", help="Személykövetés: r fp [SECONDS]. Példa: r fp 20")
    follow.add_argument("seconds", type=float, nargs="?", default=0.0, help="idő [s]; 0 = folyamatos")
    follow.add_argument("--max-v", type=float, default=FOLLOW_PERSON_DEFAULT_MAX_V_MPS, help="sebességkorlát [m/s]")
    follow.add_argument("--max-omega", type=float, default=FOLLOW_PERSON_DEFAULT_MAX_OMEGA_RAD_S, help="szögsebességkorlát [rad/s]")

    camera = add_command("camera", help="Exkluzív kameradiagnosztika: photo / video OUTPUT [SECONDS].")
    camera.add_argument("operation", choices=("photo", "video"))
    camera.add_argument("output")
    camera.add_argument("seconds", type=float, nargs="?", default=10.0)

    add_command("system", help="RPi/Linux rendszerállapot.")
    return parser


def _canonical(command: str) -> str:
    return ALIASES.get(command, command)


def _validate_seconds(value: float) -> float:
    seconds = float(value)
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError("seconds must be finite and >= 0")
    return seconds


def _print_json(value: object) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, default=str))


def _status_line(interface: RobotInterface) -> dict[str, object]:
    data = interface.read("operator.status")
    assert isinstance(data, Mapping)
    runtime_running = bool(data.get("runtime_running"))
    runtime = "RUNNING" if runtime_running else "STOPPED"
    status = None
    if runtime_running:
        try:
            live = interface.read("v3.status")
        except RobotInterfaceError:
            live = None
        if isinstance(live, Mapping):
            status = live
    result: dict[str, object] = {
        "runtime": runtime,
        "pid": data.get("runtime_pid"),
        "capture_mode": data.get("capture_mode"),
        "capture_hz": data.get("capture_hz"),
    }
    if isinstance(status, Mapping):
        result.update({
            "state": status.get("state"),
            "ready": status.get("ready_for_active"),
            "tick": status.get("tick_id"),
            "safety": status.get("safety_decision"),
            "safety_reason": status.get("safety_reason"),
            "fault_layer": status.get("fault_layer"),
        })
    return result


def _print_short_status(interface: RobotInterface) -> dict[str, object]:
    result = _status_line(interface)
    print(
        f"runtime {result['runtime']}"
        f" | state {result.get('state') or '-'}"
        f" | ready {'YES' if result.get('ready') is True else 'NO' if result.get('ready') is False else '-'}"
        f" | safety {result.get('safety') or '-'}"
        f" | tick {result.get('tick') if result.get('tick') is not None else '-'}"
    )
    print(
        f"capture {result.get('capture_mode') or '-'}"
        f" @ {result.get('capture_hz') if result.get('capture_hz') is not None else '-'} Hz"
        f" | pid {result.get('pid') or '-'}"
    )
    return result


def _motion_request(args: argparse.Namespace) -> tuple[str, dict[str, object]]:
    command = _canonical(args.command)
    if command == "forward":
        return "v3.command.forward", {"speed_mps": args.speed}
    if command == "backward":
        return "v3.command.backward", {"speed_mps": args.speed}
    if command == "teleop":
        return "v3.command.teleop", {
            "v_mps": args.v,
            "omega_rad_s": args.omega,
            "max_v_mps": args.max_v,
            "max_omega_rad_s": args.max_omega,
        }
    if command == "wheels":
        return "v3.command.wheels", {"left_mps": args.left, "right_mps": args.right}
    if command == "roomcruise":
        return "v3.command.explore", {}
    if command == "faceperson":
        return "v3.command.face_person", {"max_omega_rad_s": args.max_omega}
    if command == "followperson":
        return "v3.command.follow_person", {
            "max_v_mps": args.max_v,
            "max_omega_rad_s": args.max_omega,
        }
    raise ValueError(f"not a motion command: {command}")


def _run_timed_motion(
    interface: RobotInterface,
    args: argparse.Namespace,
    *,
    capture_mode: str,
    capture_hz: int,
    no_trigger: bool,
) -> dict[str, object]:
    command = _canonical(args.command)
    seconds = _validate_seconds(args.seconds)
    action, parameters = _motion_request(args)

    # R2B4_ER2_P0_20260925: a short CLI motion owns its motion producer, not an
    # already-running resident runtime. Reuse the resident capture contract too,
    # so a client command can never restart another client's runtime just to
    # change capture mode/rate.
    before_raw = interface.read("operator.status")
    before = before_raw if isinstance(before_raw, Mapping) else {}
    runtime_preexisting = before.get("runtime_running") is True
    effective_mode = capture_mode
    effective_hz = capture_hz
    if runtime_preexisting:
        resident_mode = before.get("capture_mode")
        resident_hz = before.get("capture_hz")
        if isinstance(resident_mode, str):
            effective_mode = resident_mode
        if isinstance(resident_hz, int) and not isinstance(resident_hz, bool):
            effective_hz = resident_hz
        if effective_mode != capture_mode or effective_hz != capture_hz:
            print(
                f"runtime already active: inherit capture {effective_mode} @ {effective_hz} Hz "
                f"(requested {capture_mode} @ {capture_hz} Hz)",
                flush=True,
            )

    parameters.update({
        "capture": not no_trigger,
        "capture_mode": effective_mode,
        "capture_hz": effective_hz,
    })
    if seconds > 0.0:
        parameters["session_owner_pid"] = os.getpid()
        parameters["session_watchdog_s"] = seconds + 5.0

    print(
        f"R2B4: {command} | {'continuous' if seconds == 0 else f'{seconds:g} s'}"
        f" | capture {effective_mode} @ {effective_hz} Hz"
    )
    try:
        handle = interface.execute(action, **parameters)
    except BaseException:
        # If this CLI created a resident runtime during setup, clean up only that
        # owned runtime. A pre-existing runtime belongs to the wider robot session.
        if seconds > 0 and not runtime_preexisting:
            try:
                _shutdown(interface)
            except Exception:
                pass
        raise

    if seconds == 0:
        print("command ACTIVE; runtime remains running.  STOP: ./r x   shutdown: ./r sd")
        return {"status": "ACTIVE", "command": command, "handle": str(handle)}

    deadline = time.monotonic() + seconds
    last_bucket: int | None = None
    interrupted = False
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            bucket = int(math.ceil(remaining / 5.0) * 5)
            if bucket != last_bucket and seconds >= 10.0:
                print(f"running: {max(0, int(math.ceil(remaining)))} s remaining", flush=True)
                last_bucket = bucket
            time.sleep(min(0.25, remaining))
    except KeyboardInterrupt:
        interrupted = True
        print("\nCtrl+C -> STOP", file=sys.stderr, flush=True)
    finally:
        interface.stop()

    if runtime_preexisting:
        final: Mapping[str, object] = {
            "state": "RUNTIME_REUSED",
            "runtime_kept_running": True,
        }
        print("runtime kept running (pre-existing resident session)")
    else:
        final = _shutdown(interface)

    return {
        "status": "INTERRUPTED" if interrupted else "FINISHED",
        "command": command,
        "seconds": seconds,
        "runtime": final,
    }

def _shutdown(interface: RobotInterface) -> Mapping[str, object]:
    interface.execute("operator.runtime.stop")
    return {"state": "STOPPED"}


def _execute(
    interface: RobotInterface,
    args: argparse.Namespace,
    *,
    capture_mode: str,
    capture_hz: int,
    no_trigger: bool,
) -> object:
    command = _canonical(args.command)
    if command in MOTION_COMMANDS:
        output = _run_timed_motion(
            interface,
            args,
            capture_mode=capture_mode,
            capture_hz=capture_hz,
            no_trigger=no_trigger,
        )
    elif command == "status":
        output = _status_line(interface) if args.json else _print_short_status(interface)
    elif command == "diag":
        output = interface.read("operator.diagnostics")
        if not args.json:
            _print_json(output)
    elif command == "caps":
        output = interface.capabilities()
        if not args.json:
            _print_json(output)
    elif command == "stop":
        output = interface.stop()
        print("robot: IDLE (runtime kept running if present)")
    elif command == "shutdown":
        output = _shutdown(interface)
    elif command == "panic":
        output = interface.execute("operator.panic")
        print("robot: STOP + runtime shutdown requested")
    elif command == "proba":
        output = interface.execute(
            "operator.proba", capture_mode=capture_mode, capture_hz=capture_hz
        )
    elif command == "runtime":
        if args.operation == "start":
            output = interface.execute(
                "operator.runtime.start", capture_mode=capture_mode, capture_hz=capture_hz
            )
        elif args.operation == "stop":
            output = _shutdown(interface)
        elif args.operation == "status":
            output = _status_line(interface) if args.json else _print_short_status(interface)
        else:
            output = interface.read("operator.diagnostics")
            if not args.json:
                _print_json(output)
    elif command == "capture":
        if args.operation == "start":
            output = interface.execute(
                "capture.start", capture_mode=capture_mode, capture_hz=capture_hz
            )
        elif args.operation == "stop":
            output = interface.execute("capture.stop")
        else:
            output = interface.read("capture.status")
        if not args.json:
            _print_json(output)
    elif command == "camera":
        if args.operation == "photo":
            output = interface.execute("camera.photo", output=args.output)
        else:
            output = interface.execute("camera.video", output=args.output, duration_s=args.seconds)
        if not args.json:
            _print_json(output)
    elif command == "system":
        output = interface.read("system.status")
        if not args.json:
            _print_json(output)
    else:  # pragma: no cover - argparse guarantees this
        raise ValueError(f"unsupported command: {command}")
    return output


def main(
    argv: Sequence[str] | None = None,
    *,
    project_root: str | Path | None = None,
) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        clean, capture_mode, capture_hz, no_trigger = _extract_capture_selector(raw)
        args = _parser().parse_args(clean)
        # Progress, including shutdown/capture messages, stays visible
        # on stderr in JSON mode. stdout contains exactly one final document.
        with contextlib.redirect_stdout(sys.stderr if args.json else sys.stdout):
            interface = RobotInterface(project_root=project_root, event_sink=_event_printer)
            output = _execute(
                interface, args, capture_mode=capture_mode,
                capture_hz=capture_hz, no_trigger=no_trigger,
            )
        if args.json:
            _print_json(output)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OperatorError, RobotInterfaceError, OSError, RuntimeError, TypeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["ALIASES", "MOTION_COMMANDS", "main"]
