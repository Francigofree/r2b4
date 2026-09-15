"""Human-friendly CLI adapter for :mod:`v3.operator_controller`."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
from pathlib import Path

from v3.operator_controller import (
    CAPTURE_MODES,
    DEFAULT_CAPTURE_MODE,
    OperatorController,
    OperatorError,
    OperatorEvent,
)


def _event_printer(event: OperatorEvent) -> None:
    stream = sys.stderr if event.kind in {"error", "warning"} else sys.stdout
    print(event.message, file=stream, flush=True)


def _extract_capture_selector(argv: list[str]) -> tuple[list[str], str, bool]:
    clean: list[str] = []
    mode = DEFAULT_CAPTURE_MODE
    explicit = False
    i = 0
    while i < len(argv):
        token = argv[i]
        if token == "c" or token == "--capture":
            if explicit:
                raise OperatorError("capture selector may be specified only once")
            if i + 1 >= len(argv):
                raise OperatorError("capture selector requires: alap, full or nincs")
            mode = argv[i + 1]
            if mode not in CAPTURE_MODES:
                raise OperatorError("capture mode must be one of: alap, full, nincs")
            explicit = True
            i += 2
            continue
        if token.startswith("--capture="):
            if explicit:
                raise OperatorError("capture selector may be specified only once")
            mode = token.split("=", 1)[1]
            if mode not in CAPTURE_MODES:
                raise OperatorError("capture mode must be one of: alap, full, nincs")
            explicit = True
            i += 1
            continue
        clean.append(token)
        i += 1
    return clean, mode, explicit


def _normalize_legacy(argv: list[str]) -> list[str]:
    args = list(argv)
    if len(args) >= 2 and args[0] in {"forward", "mozog", "wheels", "roomcruise", "explore"} and args[1] == "start":
        del args[1]
    args = ["--no-trigger" if token == "nocapture" else token for token in args]
    return args


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="r2b4",
        description="R2B4 operator remote control. Short commands are preferred; legacy forms remain accepted.",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Examples:\n"
            "  ./r2b4 status\n"
            "  ./r2b4 forward 0.15\n"
            "  ./r2b4 backward 0.15 c full\n"
            "  ./r2b4 wheels 0.10 0.20\n"
            "  ./r2b4 teleop 0.15 -0.20\n"
            "  ./r2b4 roomcruise\n"
            "  ./r2b4 proba c full\n"
            "  ./r2b4 stop\n"
            "  ./r2b4 shutdown\n\n"
            "Capture: c alap | c full | c nincs (or --capture MODE).\n"
            "Legacy forms such as 'forward start' and 'roomcruise start' still work."
        ),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", help="robot/runtime status")
    sub.add_parser("diag", help="detailed status plus recent logs")
    sub.add_parser("start", help="start resident runtime")
    sub.add_parser("stop", help="stop motion, keep runtime running")
    sub.add_parser("shutdown", help="stop motion and resident runtime")
    sub.add_parser("proba", help="run the integrated 10-phase motion test")

    runtime = sub.add_parser("runtime", help="runtime lifecycle (legacy/advanced)")
    runtime.add_argument("operation", choices=("start", "stop", "status", "diag"))

    capture = sub.add_parser("capture", help="capture control")
    capture.add_argument("operation", choices=("start", "stop", "status"))

    def motion_flags(p: argparse.ArgumentParser) -> None:
        p.add_argument("--no-trigger", action="store_true", help="do not arm the ALAP movement-stop trigger")

    forward = sub.add_parser("forward", help="drive straight forward")
    forward.add_argument("speed", type=float, nargs="?", default=0.15)
    motion_flags(forward)

    backward = sub.add_parser("backward", help="drive straight backward")
    backward.add_argument("speed", type=float, nargs="?", default=0.15)
    motion_flags(backward)

    teleop = sub.add_parser("teleop", help="generic body target: v [m/s], omega [rad/s]")
    teleop.add_argument("v", type=float)
    teleop.add_argument("omega", type=float)
    teleop.add_argument("--max-v", type=float, default=0.50)
    teleop.add_argument("--max-omega", type=float, default=1.20)
    motion_flags(teleop)

    wheels = sub.add_parser("wheels", aliases=["mozog"], help="left/right wheel-speed target [m/s]")
    wheels.add_argument("left", type=float)
    wheels.add_argument("right", type=float)
    motion_flags(wheels)

    room = sub.add_parser("roomcruise", aliases=["explore"], help="autonomous Room Cruise / EXPLORE")
    motion_flags(room)

    panic = sub.add_parser("panic", help=argparse.SUPPRESS)
    panic.add_argument("--quiet", action="store_true")

    worker = sub.add_parser("__runtime-session", help=argparse.SUPPRESS)
    worker.add_argument("--capture-path", required=True)
    worker.add_argument("--capture-mode", choices=tuple(sorted(CAPTURE_MODES)), required=True)
    return parser


def _print_status(controller: OperatorController, *, diagnostic: bool) -> None:
    data = controller.diagnostics() if diagnostic else controller.status()
    status = data.get("status")
    running = bool(data.get("runtime_running"))
    print(f"runtime: {'RUNNING' if running else 'STOPPED'}")
    if data.get("runtime_pid"):
        print(f"pid:     {data['runtime_pid']}")
    print(f"capture: {data.get('capture_mode') or '-'}")
    if isinstance(status, dict):
        print(f"state:   {status.get('state', '-')}")
        if status.get("state") == "RUNNING":
            ready = status.get("ready_for_active")
            print(f"ready:   {'YES' if ready is True else 'NO' if ready is False else '-'}")
            print(f"tick:    {status.get('tick_id', '-')}")
            print(f"safety:  {status.get('safety_decision', '-')} / {status.get('safety_reason', '-')}")
            print(f"motors:  {status.get('left_output', '-')} / {status.get('right_output', '-')}")
            estimate = status.get("estimate")
            if diagnostic and isinstance(estimate, dict):
                yaw = estimate.get("yaw_rad")
                yaw_deg = math.degrees(yaw) if isinstance(yaw, (int, float)) else None
                print(f"x:       {estimate.get('x_m', '-')} m")
                print(f"y:       {estimate.get('y_m', '-')} m")
                print(f"yaw:     {yaw_deg:.2f} deg" if yaw_deg is not None else "yaw:     -")
                print(f"v:       {estimate.get('v_mps', '-')} m/s")
                print(f"omega:   {estimate.get('omega_rad_s', '-')} rad/s")
        else:
            report = status.get("report") or {}
            if isinstance(report, dict):
                print(f"reason:  {report.get('final_reason') or report.get('exit_reason') or status.get('error') or '-'}")
                print(f"fault:   {report.get('fault_layer') or '-'}")
    if diagnostic:
        runtime_tail = data.get("runtime_log_tail") or []
        command_tail = data.get("command_log_tail") or []
        if runtime_tail:
            print("\nLast runtime log:")
            print("\n".join(runtime_tail))
        if command_tail:
            print("\nLast command log:")
            print("\n".join(command_tail))


def _print_capture_status(data: dict[str, object]) -> None:
    print(f"capture: {data.get('state', '-')}")
    print(f"mode:    {data.get('mode', '-')}")
    print(f"file:    {data.get('path') or '-'}")
    if "status" in data:
        print(f"status:  {data.get('status') or '-'}")
        print(f"ticks:   {data.get('ticks') if data.get('ticks') is not None else '-'}")
        print(f"trigger: {data.get('trigger') or '-'}")
        print(f"complete:{' YES' if data.get('complete') else ' NO'}")
        print(f"evidence:{' ' + str(data['evidence']) if data.get('evidence') else ' -'}")


def main(argv: list[str] | None = None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        clean, capture_mode, capture_explicit = _extract_capture_selector(raw)
        clean = _normalize_legacy(clean)
        args = _parser().parse_args(clean)
        controller = OperatorController(event_sink=_event_printer)

        if args.command == "__runtime-session":
            return controller.run_runtime_session(Path(args.capture_path), args.capture_mode)

        if args.command == "status":
            _print_status(controller, diagnostic=False)
        elif args.command == "diag":
            _print_status(controller, diagnostic=True)
        elif args.command == "start":
            controller.runtime_start(capture_mode)
        elif args.command == "stop":
            controller.stop()
            print("robot: IDLE (runtime kept running if present)")
        elif args.command == "shutdown":
            controller.shutdown()
        elif args.command == "panic":
            controller.panic()
            if not args.quiet:
                print("robot: STOP + runtime shutdown requested")
        elif args.command == "runtime":
            if args.operation == "start":
                controller.runtime_start(capture_mode)
            elif args.operation == "stop":
                controller.runtime_stop()
            elif args.operation == "status":
                _print_status(controller, diagnostic=False)
            else:
                _print_status(controller, diagnostic=True)
        elif args.command == "capture":
            if args.operation == "start":
                controller.capture_start(capture_mode if capture_explicit else None)
            elif args.operation == "stop":
                _print_capture_status(controller.capture_stop())
            else:
                _print_capture_status(controller.capture_status())
        elif args.command == "forward":
            controller.forward(args.speed, capture=not args.no_trigger, capture_mode=capture_mode)
        elif args.command == "backward":
            controller.backward(args.speed, capture=not args.no_trigger, capture_mode=capture_mode)
        elif args.command == "teleop":
            controller.start_teleop(
                v_mps=args.v,
                omega_rad_s=args.omega,
                max_v_mps=args.max_v,
                max_omega_rad_s=args.max_omega,
                capture=not args.no_trigger,
                capture_mode=capture_mode,
            )
        elif args.command in {"wheels", "mozog"}:
            controller.wheels(args.left, args.right, capture=not args.no_trigger, capture_mode=capture_mode)
        elif args.command in {"roomcruise", "explore"}:
            controller.roomcruise(capture=not args.no_trigger, capture_mode=capture_mode)
        elif args.command == "proba":
            old_term = signal.getsignal(signal.SIGTERM)

            def interrupt(_signum: int, _frame: object) -> None:
                raise KeyboardInterrupt

            signal.signal(signal.SIGTERM, interrupt)
            try:
                controller.run_proba(capture_mode=capture_mode)
            finally:
                signal.signal(signal.SIGTERM, old_term)
        else:  # pragma: no cover - argparse guarantees this
            raise OperatorError(f"unsupported command: {args.command}")
        return 0
    except KeyboardInterrupt:
        return 130
    except OperatorError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
