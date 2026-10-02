#!/usr/bin/env python3
"""Desktop helper for a headless R2B4 Sign in with ChatGPT flow.

Use the exact host ID printed by `./r chatgpt host-id` on the Pi, then transfer
the output file back to the Pi and import it with `./r chatgpt import FILE`.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MODULE = ROOT / "payload" / "r2b4_voice" / "openai_oauth.py"
spec = importlib.util.spec_from_file_location("r2b4_openai_oauth_standalone", MODULE)
if spec is None or spec.loader is None:
    raise SystemExit(f"cannot load {MODULE}")
oauth = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = oauth
spec.loader.exec_module(oauth)


def main() -> int:
    parser = argparse.ArgumentParser(description="Create R2B4 ChatGPT OAuth credentials on the browser computer")
    parser.add_argument("--host-id", required=True, help="exact value from: ./r chatgpt host-id")
    parser.add_argument("--output", default="r2b4_chatgpt_oauth.json")
    parser.add_argument("--no-open", action="store_true", help="print URL but do not open browser")
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    existing = None
    if output.is_file():
        try:
            value = json.loads(output.read_text(encoding="utf-8"))
            if isinstance(value, dict):
                existing = value
        except Exception:
            existing = None
    try:
        record = oauth.perform_login(
            host_id=args.host_id,
            output_path=output,
            existing=existing,
            open_browser=not args.no_open,
        )
    except oauth.OAuthError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"Saved protected credential: {output}")
    print(f"Account: {record.get('email') or record.get('subject')}")
    print("Next: copy this file to the Pi and run: ./r chatgpt import <FILE>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
