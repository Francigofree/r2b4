#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Strict Human Follow v2 live gate through the camera FOLLOW path."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import person_follow_camera_live as live  # noqa: E402


live.RESULT_PATH = live.AGENT_TESTS_DIR / "latest_person_follow_camera_live_v2.json"
live.SUMMARY_PATH = live.AGENT_TESTS_DIR / "latest_person_follow_camera_live_v2_summary.json"
live.HISTORY_PATH = live.AGENT_TESTS_DIR / "person_follow_camera_live_v2_samples.jsonl"


def main() -> int:
    parser = argparse.ArgumentParser(description="Strict bounded live Human Follow v2 camera gate.")
    parser.add_argument("--test-name", default="person_follow_camera_live_v2")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--sample-rate-hz", type=float, default=5.0)
    parser.add_argument("--speed-scale", type=float, default=0.8)
    parser.add_argument("--follow-distance-m", type=float, default=1.0)
    parser.add_argument("--search-pivot-omega-rad-s", type=float, default=0.08)
    parser.add_argument("--control-mode", default="UNIFIED")
    parser.add_argument("--fresh-target-omega-max-rad-s", type=float, default=None)
    parser.add_argument("--omega-p90-max-rad-s", type=float, default=None)
    parser.add_argument("--command-delta-omega-p90-max-rad-s", type=float, default=None)
    parser.add_argument("--command-delta-omega-max-rad-s", type=float, default=None)
    parser.add_argument("--status-timeout-s", type=float, default=5.0)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--no-strict-v2", dest="strict_v2", action="store_false")
    parser.set_defaults(strict_v2=True)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    result = live.run(args)
    print(
        json.dumps(
            result if not args.compact else {"status": result["status"], "errors": result["errors"]},
            ensure_ascii=False,
        )
    )
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
