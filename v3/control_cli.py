"""Thin machine-oriented CLI over the resident V3 command mailbox."""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
import time
import uuid
from pathlib import Path
from typing import Callable

from v3.adapters.resident_command import (
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROL_CLI_RESULT_SCHEMA = "R2B4_V3_CONTROL_CLI_RESULT_V1"
RESIDENT_PROCESS_STATUS_SCHEMA = "R2B4_V3_RESIDENT_PROCESS_STATUS_V1"
DEFAULT_TTL_NS = 200_000_000
DEFAULT_HEARTBEAT_NS = 100_000_000
_STATUS_MAXIMUM_BYTES = 65_536


def _runtime_path(value: str, project_root: Path | None = None) -> Path:
    if project_root is None:
        project_root = PROJECT_ROOT
    path = Path(value)
    if not path.is_absolute():
        path = project_root / path
    path = path.resolve(strict=False)
    try:
        path.relative_to((project_root / "runtime").resolve())
    except ValueError as exc:
        raise ValueError("CLI paths must stay below the canonical runtime directory") from exc
    return path


def _read_status(path: Path) -> dict[str, object]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise ValueError("resident status cannot be opened safely") from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError("resident status must be a regular file")
        if metadata.st_uid != os.geteuid():
            raise ValueError("resident status owner is not trusted")
        if stat.S_IMODE(metadata.st_mode) & 0o022:
            raise ValueError("resident status must not be group/world writable")
        if metadata.st_size > _STATUS_MAXIMUM_BYTES:
            raise ValueError("resident status exceeds the CLI size limit")
        raw = os.read(descriptor, _STATUS_MAXIMUM_BYTES + 1)
    finally:
        os.close(descriptor)
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("resident status must contain valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("resident status root must be an object")
    if payload.get("schema") != RESIDENT_PROCESS_STATUS_SCHEMA:
        raise ValueError("resident status schema is invalid")
    return payload


def _logical_id(mode: str) -> str:
    return f"v3-cli.{mode}.{uuid.uuid4().hex}"


def _result(status: str, **values: object) -> dict[str, object]:
    return {
        "schema": CONTROL_CLI_RESULT_SCHEMA,
        "status": status,
        **values,
    }


def _active_preflight(status_path: Path) -> None:
    status = _read_status(status_path)
    if status.get("state") != "RUNNING" or status.get("ready_for_active") is not True:
        raise ValueError("resident runtime is not ready for an ACTIVE command")


def _run_active(
    client: ResidentCommandClient,
    publish: Callable[[str], int],
    *,
    command_id: str,
    ttl_ns: int,
    heartbeat_ns: int,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    if heartbeat_ns <= 0 or heartbeat_ns >= ttl_ns:
        raise ValueError("heartbeat interval must be positive and shorter than TTL")
    try:
        revision = publish(command_id)
        print(
            json.dumps(
                _result(
                    "ACTIVE",
                    command_id=command_id,
                    revision=revision,
                ),
                sort_keys=True,
            ),
            flush=True,
        )
        while True:
            sleep(heartbeat_ns / 1e9)
            publish(command_id)
    except KeyboardInterrupt:
        stop_id = _logical_id("stop")
        revision = client.publish_stop(stop_id, ttl_ns=ttl_ns)
        print(
            json.dumps(
                _result(
                    "STOP_PUBLISHED",
                    command_id=stop_id,
                    revision=revision,
                    stopped_command_id=command_id,
                    reason="KEYBOARD_INTERRUPT",
                ),
                sort_keys=True,
            ),
            flush=True,
        )
        return 130
    except Exception:
        try:
            client.publish_stop(_logical_id("stop"), ttl_ns=ttl_ns)
        except Exception:
            pass
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control the native resident V3 runtime")
    parser.add_argument("--command-path", default="runtime/v3_command.json")
    parser.add_argument("--status-path", default="runtime/v3_status.json")
    parser.add_argument("--ttl-ms", type=int, default=DEFAULT_TTL_NS // 1_000_000)
    parser.add_argument(
        "--heartbeat-ms",
        type=int,
        default=DEFAULT_HEARTBEAT_NS // 1_000_000,
    )
    subcommands = parser.add_subparsers(dest="operation", required=True)
    subcommands.add_parser("status", help="print the resident status JSON")

    stop = subcommands.add_parser("stop", help="publish one STOP command")
    stop.add_argument("--command-id")

    teleop = subcommands.add_parser("teleop", help="heartbeat a bounded TELEOP command")
    teleop.add_argument("--command-id")
    teleop.add_argument("--v-mps", type=float, required=True)
    teleop.add_argument("--omega-rad-s", type=float, required=True)
    teleop.add_argument("--max-v-mps", type=float, default=0.50)
    teleop.add_argument("--max-omega-rad-s", type=float, default=1.20)

    explore = subcommands.add_parser(
        "explore",
        help="heartbeat a generic EXPLORE mission (Room Cruise)",
    )
    explore.add_argument("--command-id")
    explore.add_argument("--max-v-mps", type=float, default=0.30)
    explore.add_argument("--max-omega-rad-s", type=float, default=0.60)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        command_path = _runtime_path(args.command_path)
        status_path = _runtime_path(args.status_path)
        if command_path == status_path:
            raise ValueError("command and status paths must differ")
        if args.operation == "status":
            print(json.dumps(_read_status(status_path), sort_keys=True))
            return 0

        ttl_ns = args.ttl_ms * 1_000_000
        heartbeat_ns = args.heartbeat_ms * 1_000_000
        config = ResidentCommandMailboxConfig(path=command_path)
        client = ResidentCommandClient(config)
        command_id = args.command_id or _logical_id(args.operation)
        if args.operation == "stop":
            revision = client.publish_stop(command_id, ttl_ns=ttl_ns)
            print(
                json.dumps(
                    _result(
                        "STOP_PUBLISHED",
                        command_id=command_id,
                        revision=revision,
                    ),
                    sort_keys=True,
                )
            )
            return 0

        _active_preflight(status_path)
        if args.operation == "teleop":
            publish = lambda logical_id: client.publish_teleop(
                logical_id,
                v_mps=args.v_mps,
                omega_rad_s=args.omega_rad_s,
                max_v_mps=args.max_v_mps,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
        else:
            publish = lambda logical_id: client.publish_explore(
                logical_id,
                max_v_mps=args.max_v_mps,
                max_omega_rad_s=args.max_omega_rad_s,
                ttl_ns=ttl_ns,
            )
        return _run_active(
            client,
            publish,
            command_id=command_id,
            ttl_ns=ttl_ns,
            heartbeat_ns=heartbeat_ns,
        )
    except Exception as exc:
        print(
            json.dumps(
                _result(
                    "ERROR",
                    error_type=type(exc).__name__,
                    error=str(exc),
                ),
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "CONTROL_CLI_RESULT_SCHEMA",
    "DEFAULT_HEARTBEAT_NS",
    "DEFAULT_TTL_NS",
    "PROJECT_ROOT",
    "main",
]
