"""Offline MCAP-only Test Hub CLI."""
from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from .test_hub_compiler import (
    compile_evidence, inspect_mcap, latest_capture, query_capture,
    run_pending, verify_evidence,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("capture", nargs="?")
    run.add_argument("--output-dir")
    run.add_argument("--replay", choices=("off", "auto", "full"), default="auto")
    batch = commands.add_parser("batch")
    batch.add_argument("--capture-dir", default="runtime/captures")
    batch.add_argument("--replay", choices=("off", "auto", "full"), default="auto")
    inspect = commands.add_parser("inspect")
    inspect.add_argument("capture")
    verify = commands.add_parser("verify-evidence")
    verify.add_argument("index")
    query = commands.add_parser("query")
    query.add_argument("capture")
    query.add_argument("--topic", required=True)
    query.add_argument("--start-ns", type=int)
    query.add_argument("--end-ns", type=int)
    query.add_argument("--max-rows", type=int, default=200)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(list(sys.argv[1:] if argv is None else argv))
    try:
        if args.command == "run":
            capture = Path(args.capture).resolve() if args.capture else latest_capture().resolve()
            output = Path(args.output_dir).resolve() if args.output_dir else capture.with_suffix(".evidence")
            result = compile_evidence(
                capture, output, replay_mode=args.replay,
                project_root=Path(__file__).resolve().parents[1],
            )
        elif args.command == "batch":
            result = run_pending(Path(args.capture_dir), replay_mode=args.replay)
        elif args.command == "inspect":
            result = inspect_mcap(args.capture, deep=True)
        elif args.command == "verify-evidence":
            result = verify_evidence(args.index)
        else:
            result = query_capture(
                args.capture, topic=args.topic, start_ns=args.start_ns,
                end_ns=args.end_ns, max_rows=args.max_rows,
            )
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, default=str))
        return 0
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "ERROR", "error": str(exc)}, indent=2), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
