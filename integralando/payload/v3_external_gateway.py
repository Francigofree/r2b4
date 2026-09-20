#!/usr/bin/env python3
"""JSONL/stdio transport for the protocol-neutral R2B4 ExternalRobotGateway."""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import TextIO

from v3.external_gateway import EXTERNAL_GATEWAY_SCHEMA, ExternalRobotGateway, GatewayPolicy
from v3.robot_interface import RobotInterface

_MAX_LINE_CHARS = 65_536


def _json_default(value: object) -> object:
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, set | frozenset):
        return sorted(value)
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def handle_json_line(gateway: ExternalRobotGateway, line: str) -> str:
    request_id = None
    try:
        if not isinstance(line, str):
            raise ValueError("request line must be text")
        if len(line) > _MAX_LINE_CHARS:
            raise ValueError("request line exceeds 65536 characters")
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError("request must be a JSON object")
        raw_request_id = value.get("request_id")
        request_id = raw_request_id if isinstance(raw_request_id, str) else None
        response = gateway.handle(value)
        return json.dumps(
            response.to_jsonable(),
            ensure_ascii=False,
            sort_keys=True,
            default=_json_default,
        )
    except (ValueError, json.JSONDecodeError) as exc:
        return json.dumps(
            {
                "schema": EXTERNAL_GATEWAY_SCHEMA,
                "request_id": request_id,
                "status": "REJECTED",
                "result": None,
                "error": f"{type(exc).__name__}: {exc}",
            },
            ensure_ascii=False,
            sort_keys=True,
            default=_json_default,
        )


def serve(gateway: ExternalRobotGateway, source: TextIO = sys.stdin, sink: TextIO = sys.stdout) -> int:
    for raw in source:
        line = raw.strip()
        if not line:
            continue
        sink.write(handle_json_line(gateway, line) + "\n")
        sink.flush()
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="v3_external_gateway")
    parser.add_argument("--root", default=None)
    parser.add_argument(
        "--allow-execute",
        action="store_true",
        help="allow positive RobotInterface actions; read/capabilities/STOP work without this flag",
    )
    parser.add_argument("--watchdog-s", type=float, default=30.0)
    parser.add_argument("--request", default=None, help="handle exactly one JSON request and exit")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = Path(args.root).expanduser().resolve() if args.root else Path(__file__).resolve().parent
    interface = RobotInterface(project_root=root)
    gateway = ExternalRobotGateway(
        interface,
        policy=GatewayPolicy(
            allow_execute=bool(args.allow_execute),
            session_owner_pid=os.getpid(),
            session_watchdog_s=args.watchdog_s,
        ),
    )
    if args.request is not None:
        print(handle_json_line(gateway, args.request), flush=True)
        return 0
    return serve(gateway)


if __name__ == "__main__":
    raise SystemExit(main())
