"""Canonical public entry point for the R2B4 Test Hub.

All human, agent and internal Test Hub calls use ``python3 -m v3.test_hub``.
Implementation modules are intentionally named by responsibility instead of
migration generation.
"""

from __future__ import annotations

import sys
from collections.abc import Sequence

from . import test_hub_app as _app
from . import test_hub_backend as _backend

DEFAULT_HZ = _app.DEFAULT_HZ
default_output_dir = _app.default_output_dir
latest_capture = _app.latest_capture
resolve_capture = _app.resolve_capture
run_default = _app.run_default
run_pending = _app.run_pending

inspect_mcap = _backend.inspect_mcap
diagnose_run = _backend.diagnose_run
build_agent_brief_only = _backend.build_agent_brief_only
query_capture = _backend.query_capture
verify_evidence = _backend.verify_evidence

APP_COMMANDS = frozenset({"run", "batch", "view", "compare", "test"})
EVIDENCE_COMMANDS = frozenset({"inspect", "diagnose", "agent", "query", "verify-evidence"})


def _print_help(*, file: object = None) -> None:
    stream = sys.stdout if file is None else file
    print(
        "R2B4 Test Hub — canonical offline MCAP analysis entrypoint\n\n"
        "usage: python3 -m v3.test_hub [COMMAND] [OPTIONS]\n\n"
        "High-level commands:\n"
        "  run              analyze one finished MCAP (default command)\n"
        "  view             build a cheap derived run view\n"
        "  batch            process pending MCAP captures\n"
        "  compare          objective before/after comparison\n"
        "  test             run a curated pytest profile\n\n"
        "Precise evidence commands:\n"
        "  inspect          MCAP/integrity summary\n"
        "  diagnose         build canonical run-bound evidence\n"
        "  agent            bounded diagnosis on stdout\n"
        "  query            precise bounded evidence extraction\n"
        "  verify-evidence  verify evidence index and artifacts\n\n"
        "Use: python3 -m v3.test_hub COMMAND --help",
        file=stream,
    )


def main(argv: Sequence[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments:
        return _app.main([])
    command = arguments[0]
    if command in {"-h", "--help", "help"}:
        _print_help()
        return 0
    if command in APP_COMMANDS:
        return _app.main(arguments)
    if command in EVIDENCE_COMMANDS:
        return _backend.main(arguments)
    print(f"unknown Test Hub command: {command}", file=sys.stderr)
    _print_help(file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "APP_COMMANDS",
    "EVIDENCE_COMMANDS",
    "DEFAULT_HZ",
    "build_agent_brief_only",
    "default_output_dir",
    "diagnose_run",
    "inspect_mcap",
    "latest_capture",
    "main",
    "query_capture",
    "resolve_capture",
    "run_default",
    "run_pending",
    "verify_evidence",
]
