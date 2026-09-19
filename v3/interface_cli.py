"""Human-friendly launcher CLI over :mod:`v3.robot_interface`.

Designed for the short ``./r`` launcher.  Timed motion commands own the whole
operator session: runtime auto-start, motion, STOP, runtime shutdown, then visible
capture/Test Hub finalization.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from v3.operator_controller import CAPTURE_MODES, DEFAULT_CAPTURE_MODE, OperatorError, OperatorEvent
from v3.robot_interface import RobotInterface, RobotInterfaceError


TEST_HUB_STATUS_POLL_S = 0.5
TEST_HUB_PROGRESS_EVERY_S = 5.0
TEST_HUB_WAIT_LIMIT_S = 15.0 * 60.0


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
    "th": "testhub",
    "cam": "camera",
    "sys": "system",
}

MOTION_COMMANDS = frozenset({
    "forward", "backward", "teleop", "wheels", "roomcruise", "faceperson", "followperson"
})


def _event_printer(event: OperatorEvent) -> None:
    if event.message.startswith("TEST_HUB_AUTORUN"):
        # The progress loop below provides a less cryptic, continuously visible view.
        return
    stream = sys.stderr if event.kind in {"error", "warning"} else sys.stdout
    print(event.message, file=stream, flush=True)


def _extract_capture_selector(argv: Sequence[str]) -> tuple[list[str], str, bool]:
    clean: list[str] = []
    mode = DEFAULT_CAPTURE_MODE
    no_trigger = False
    explicit = False
    i = 0
    values = list(argv)
    while i < len(values):
        token = values[i]
        if token in {"c", "--capture"}:
            if explicit:
                raise ValueError("capture selector may be specified only once")
            if i + 1 >= len(values):
                raise ValueError("capture selector requires: alap, full or nincs")
            mode = values[i + 1]
            if mode not in CAPTURE_MODES:
                raise ValueError("capture mode must be one of: alap, full, nincs")
            explicit = True
            i += 2
            continue
        if token.startswith("--capture="):
            if explicit:
                raise ValueError("capture selector may be specified only once")
            mode = token.split("=", 1)[1]
            if mode not in CAPTURE_MODES:
                raise ValueError("capture mode must be one of: alap, full, nincs")
            explicit = True
            i += 1
            continue
        if token in {"nc", "nocapture", "--no-trigger"}:
            no_trigger = True
            i += 1
            continue
        clean.append(token)
        i += 1
    return clean, mode, no_trigger


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="r",
        description="R2B4 short interface launcher. Timed motion = auto runtime + STOP + Test Hub.",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Timed examples (first number = seconds):\n"
            "  ./r rc 35              Room Cruise 35 s\n"
            "  ./r fp 20              follow person 20 s\n"
            "  ./r f 10 0.15          forward 10 s at 0.15 m/s\n"
            "  ./r m 8 0.10 0.20      wheel targets for 8 s\n"
            "  ./r t 12 0.15 -0.20    TELEOP 12 s\n\n"
            "Use 0 seconds to leave a command running: ./r rc 0\n"
            "Capture anywhere: c alap | c full | c nincs   (or --capture MODE)\n"
            "Skip movement trigger: nc   (runtime shutdown may still finalize its capture slot)\n"
            "Useful: s=status, d=diag, x=STOP, th=Test Hub, cap=capture, rt=runtime."
        ),
    )
    parser.add_argument("--json", action="store_true", help="machine-readable final output where applicable")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", aliases=["s"], help="short robot/runtime status")
    sub.add_parser("diag", aliases=["d"], help="diagnostics")
    sub.add_parser("caps", aliases=["ls"], help="live external capabilities")
    sub.add_parser("stop", aliases=["x"], help="STOP motion; keep runtime if manually managed")
    sub.add_parser("shutdown", aliases=["sd"], help="STOP + runtime shutdown + Test Hub progress")
    sub.add_parser("panic", help="fail-safe STOP + runtime shutdown")
    sub.add_parser("proba", aliases=["pr"], help="existing integrated physical test")

    runtime = sub.add_parser("runtime", aliases=["rt"], help="manual runtime lifecycle")
    runtime.add_argument("operation", choices=("start", "stop", "status", "diag"))

    capture = sub.add_parser("capture", aliases=["cap"], help="capture control")
    capture.add_argument("operation", choices=("start", "stop", "status"))

    forward = sub.add_parser("forward", aliases=["f"], help="forward: SECONDS [SPEED_MPS]")
    forward.add_argument("seconds", type=float, nargs="?", default=0.0)
    forward.add_argument("speed", type=float, nargs="?", default=0.15)

    backward = sub.add_parser("backward", aliases=["b"], help="backward: SECONDS [SPEED_MPS]")
    backward.add_argument("seconds", type=float, nargs="?", default=0.0)
    backward.add_argument("speed", type=float, nargs="?", default=0.15)

    teleop = sub.add_parser("teleop", aliases=["t"], help="teleop: SECONDS V_MPS OMEGA_RAD_S")
    teleop.add_argument("seconds", type=float)
    teleop.add_argument("v", type=float)
    teleop.add_argument("omega", type=float)
    teleop.add_argument("--max-v", type=float, default=0.50)
    teleop.add_argument("--max-omega", type=float, default=1.20)

    wheels = sub.add_parser("wheels", aliases=["m", "mozog"], help="wheels: SECONDS LEFT_MPS RIGHT_MPS")
    wheels.add_argument("seconds", type=float)
    wheels.add_argument("left", type=float)
    wheels.add_argument("right", type=float)

    room = sub.add_parser("roomcruise", aliases=["rc", "explore"], help="Room Cruise: [SECONDS]")
    room.add_argument("seconds", type=float, nargs="?", default=0.0)

    face = sub.add_parser("faceperson", aliases=["fa"], help="face person: [SECONDS]")
    face.add_argument("seconds", type=float, nargs="?", default=0.0)
    face.add_argument("--max-omega", type=float, default=0.50)

    follow = sub.add_parser("followperson", aliases=["fp"], help="follow person: [SECONDS]")
    follow.add_argument("seconds", type=float, nargs="?", default=0.0)
    follow.add_argument("--max-v", type=float, default=0.15)
    follow.add_argument("--max-omega", type=float, default=0.30)

    testhub = sub.add_parser("testhub", aliases=["th"], help="Test Hub status/run/batch")
    testhub.add_argument("operation", nargs="?", choices=("status", "run", "batch"), default="status")
    testhub.add_argument("capture", nargs="?")
    testhub.add_argument("--replay", choices=("off", "incident", "full"), default="incident")
    testhub.add_argument("--hz", type=int, choices=(1, 5, 10), default=5)
    testhub.add_argument("--no-sweep", action="store_true")

    camera = sub.add_parser("camera", aliases=["cam"], help="exclusive camera diagnostics")
    camera.add_argument("operation", choices=("photo", "video"))
    camera.add_argument("output")
    camera.add_argument("seconds", type=float, nargs="?", default=10.0)

    sub.add_parser("system", aliases=["sys"], help="RPi/Linux host status")
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
    status = data.get("status")
    runtime = "RUNNING" if data.get("runtime_running") else "STOPPED"
    result: dict[str, object] = {
        "runtime": runtime,
        "pid": data.get("runtime_pid"),
        "capture_mode": data.get("capture_mode"),
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
    print(f"capture {result.get('capture_mode') or '-'} | pid {result.get('pid') or '-'}")
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
    no_trigger: bool,
) -> dict[str, object]:
    command = _canonical(args.command)
    seconds = _validate_seconds(args.seconds)
    action, parameters = _motion_request(args)
    parameters.update({"capture": not no_trigger, "capture_mode": capture_mode})

    print(f"R2B4: {command} | {'continuous' if seconds == 0 else f'{seconds:g} s'} | capture {capture_mode}")
    try:
        handle = interface.execute(action, **parameters)
    except BaseException:
        # Motion setup can start the runtime before rejecting the command.  The
        # short launcher owns timed-session cleanup, so do not leave it behind.
        if seconds > 0:
            try:
                _shutdown_with_testhub_progress(interface, capture_mode=capture_mode)
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
        try:
            interface.stop()
        finally:
            final = _shutdown_with_testhub_progress(interface, capture_mode=capture_mode)

    return {
        "status": "INTERRUPTED" if interrupted else "FINISHED",
        "command": command,
        "seconds": seconds,
        "testhub": final,
    }


def _shutdown_with_testhub_progress(
    interface: RobotInterface,
    *,
    capture_mode: str | None = None,
) -> Mapping[str, object]:
    result: dict[str, object] = {}
    error: list[BaseException] = []

    def worker() -> None:
        try:
            value = interface.execute("operator.runtime.stop")
            if isinstance(value, Mapping):
                result.update(value)
        except BaseException as exc:  # carried back to the main thread
            error.append(exc)

    thread = threading.Thread(target=worker, name="r2b4-interface-shutdown", daemon=False)
    thread.start()
    started = time.monotonic()
    last_print = 0.0
    last_state: str | None = None
    while thread.is_alive():
        now = time.monotonic()
        if capture_mode == "nincs":
            state = "OFF"
            detail: Mapping[str, object] = {"state": state}
        else:
            raw = interface.read("testhub.status")
            detail = raw if isinstance(raw, Mapping) else {"state": "UNKNOWN"}
            state = str(detail.get("state") or "UNKNOWN")
        if state != last_state or now - last_print >= TEST_HUB_PROGRESS_EVERY_S:
            elapsed = int(now - started)
            if state == "OFF":
                print(f"shutdown: runtime stopping | Test Hub OFF | {elapsed}s", flush=True)
            else:
                print(f"shutdown/Test Hub: {state} | {elapsed}s", flush=True)
            last_state = state
            last_print = now
        thread.join(TEST_HUB_STATUS_POLL_S)
    thread.join()
    if error:
        raise error[0]

    if capture_mode == "nincs":
        print("runtime STOPPED | capture/Test Hub OFF")
        return {"state": "OFF", "status": None}

    # The runtime process can stop before its non-authoritative Test Hub
    # supervisor finishes.  Continue observing the evidence directory so the
    # human is never left staring at a silent terminal for minutes.
    return _wait_testhub_finished(interface, started=started)


def _wait_testhub_finished(
    interface: RobotInterface,
    *,
    started: float | None = None,
) -> Mapping[str, object]:
    began = started if started is not None else time.monotonic()
    last_print = 0.0
    last_state: str | None = None
    while True:
        raw = interface.read("testhub.status")
        status = raw if isinstance(raw, Mapping) else {"state": "UNKNOWN"}
        state = str(status.get("state") or "UNKNOWN")
        now = time.monotonic()
        if state != last_state or now - last_print >= TEST_HUB_PROGRESS_EVERY_S:
            elapsed = int(now - began)
            if state == "FINISHED":
                print(
                    "Test Hub: FINISHED"
                    f" | status {status.get('status') or '-'}"
                    f" | replay {status.get('replay_status') or '-'}"
                    f" | {elapsed}s",
                    flush=True,
                )
            else:
                print(f"Test Hub: {state} | {elapsed}s", flush=True)
            last_state = state
            last_print = now

        if state == "FINISHED":
            evidence = status.get("evidence")
            if evidence:
                print(f"evidence: {evidence}")
            return status
        if state == "IDLE":
            return status
        if now - began >= TEST_HUB_WAIT_LIMIT_S:
            print(
                f"Test Hub: still {state} after {int(TEST_HUB_WAIT_LIMIT_S)} s; analysis may be inspected with ./r th",
                file=sys.stderr,
            )
            return status
        try:
            time.sleep(TEST_HUB_STATUS_POLL_S)
        except KeyboardInterrupt:
            # Robot/runtime is already stopped here.  Do not kill an offline
            # evidence process merely because the user stops watching it.
            print("\nTest Hub wait cancelled; robot is already stopped.  Status: ./r th", file=sys.stderr)
            return status


def _run_testhub_with_progress(
    interface: RobotInterface,
    *,
    capture: str | None,
    hz: int,
    replay: str,
    no_sweep: bool,
) -> object:
    result: list[object] = []
    error: list[BaseException] = []

    def worker() -> None:
        try:
            result.append(interface.execute(
                "testhub.run",
                capture=capture,
                hz=hz,
                replay=replay,
                no_sweep=no_sweep,
            ))
        except BaseException as exc:
            error.append(exc)

    thread = threading.Thread(target=worker, name="r2b4-testhub-manual", daemon=False)
    thread.start()
    start = time.monotonic()
    last = -TEST_HUB_PROGRESS_EVERY_S
    while thread.is_alive():
        elapsed = time.monotonic() - start
        if elapsed - last >= TEST_HUB_PROGRESS_EVERY_S:
            print(f"Test Hub: RUNNING | {int(elapsed)}s", flush=True)
            last = elapsed
        thread.join(TEST_HUB_STATUS_POLL_S)
    thread.join()
    if error:
        raise error[0]
    value = result[0] if result else {"status": "ERROR", "error": "no Test Hub result"}
    if isinstance(value, Mapping):
        print(
            f"Test Hub: {value.get('status') or 'DONE'}"
            f" | replay {value.get('replay_status') or '-'}"
            f" | {int(time.monotonic() - start)}s"
        )
        if value.get("output_dir"):
            print(f"evidence: {value['output_dir']}")
    return value


def main(argv: Sequence[str] | None = None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        clean, capture_mode, no_trigger = _extract_capture_selector(raw)
        args = _parser().parse_args(clean)
        command = _canonical(args.command)
        interface = RobotInterface(event_sink=_event_printer)

        if command in MOTION_COMMANDS:
            output = _run_timed_motion(
                interface,
                args,
                capture_mode=capture_mode,
                no_trigger=no_trigger,
            )
        elif command == "status":
            output = _print_short_status(interface)
        elif command == "diag":
            output = interface.read("operator.diagnostics")
            _print_json(output)
        elif command == "caps":
            output = interface.capabilities()
            _print_json(output)
        elif command == "stop":
            output = interface.stop()
            print("robot: IDLE (runtime kept running if present)")
        elif command == "shutdown":
            output = _shutdown_with_testhub_progress(interface, capture_mode=None)
        elif command == "panic":
            output = interface.execute("operator.panic")
            print("robot: STOP + runtime shutdown requested")
        elif command == "proba":
            output = interface.execute("operator.proba", capture_mode=capture_mode)
        elif command == "runtime":
            if args.operation == "start":
                output = interface.execute("operator.runtime.start", capture_mode=capture_mode)
            elif args.operation == "stop":
                output = _shutdown_with_testhub_progress(interface, capture_mode=None)
            elif args.operation == "status":
                output = _print_short_status(interface)
            else:
                output = interface.read("operator.diagnostics")
                _print_json(output)
        elif command == "capture":
            if args.operation == "start":
                output = interface.execute("capture.start", capture_mode=capture_mode)
            elif args.operation == "stop":
                output = interface.execute("capture.stop")
            else:
                output = interface.read("capture.status")
            _print_json(output)
        elif command == "testhub":
            if args.operation == "status":
                output = interface.read("testhub.status")
                _print_json(output)
            elif args.operation == "run":
                output = _run_testhub_with_progress(
                    interface,
                    capture=args.capture,
                    hz=args.hz,
                    replay=args.replay,
                    no_sweep=args.no_sweep,
                )
            else:
                output = interface.execute("testhub.batch", hz=args.hz, replay=args.replay)
                _print_json(output)
        elif command == "camera":
            if args.operation == "photo":
                output = interface.execute("camera.photo", output=args.output)
            else:
                output = interface.execute("camera.video", output=args.output, duration_s=args.seconds)
            _print_json(output)
        elif command == "system":
            output = interface.read("system.status")
            _print_json(output)
        else:  # pragma: no cover - argparse guarantees this
            raise ValueError(f"unsupported command: {command}")

        if args.json and command not in {"diag", "caps", "capture", "testhub", "camera", "system"}:
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
