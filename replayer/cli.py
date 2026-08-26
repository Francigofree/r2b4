"""Command-line interface for offline Replayer operations."""

from __future__ import annotations

import argparse
import json
from typing import Any, Dict, Optional, Sequence

from replayer.capture import verify_capture
from replayer.replay import replay_capture, verify_replay_result
from replayer.storage import list_ids


def _print(payload: Dict[str, Any]) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m replayer",
        description=(
            "R2B4 Replayer V2: stage-by-stage production motion replay "
            "with fail-closed V1 compatibility."
        ),
    )
    parser.add_argument("--data-root", default=None, help="Default: <project>/replayer_data")
    sub = parser.add_subparsers(dest="command", required=True)

    verify = sub.add_parser("verify", help="Verify one immutable capture and its frame chain")
    verify.add_argument("capture_id")

    replay = sub.add_parser(
        "replay",
        help="Replay every captured production stage (V2) or MotionExecutor (V1); never dispatches motors",
    )
    replay.add_argument("capture_id")
    replay.add_argument("--result-id", default=None)
    replay.add_argument("--absolute-tolerance", type=float, default=1e-9)

    verify_result = sub.add_parser("verify-result", help="Verify replay result and evidence integrity")
    verify_result.add_argument("capture_id")
    verify_result.add_argument("result_id")

    sub.add_parser("list", help="List capture IDs and their independent replay result IDs")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "verify":
        payload = verify_capture(args.capture_id, data_root=args.data_root)
        _print(payload)
        return 0 if payload["valid"] else 2
    if args.command == "replay":
        payload = replay_capture(
            args.capture_id,
            data_root=args.data_root,
            result_id=args.result_id,
            absolute_tolerance=args.absolute_tolerance,
        )
        _print(payload)
        return 0 if payload["status"] == "MATCH" else 2
    if args.command == "verify-result":
        payload = verify_replay_result(
            args.capture_id,
            args.result_id,
            data_root=args.data_root,
        )
        _print(payload)
        return 0 if payload["valid"] else 2
    if args.command == "list":
        _print(list_ids(args.data_root))
        return 0
    return 2
