#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-/home/alba/project_r2b4}"
cd "$ROOT"

python3 -m pytest -q \
  tests/feature/test_er2_navigation_motion.py::test_navigate_motion_closed_loop_turn_translation_and_deterministic_replay \
  tests/feature/test_er2_navigation_motion.py::test_navigate_motion_async_terminal_heading_discards_late_rollout_and_restores

if [[ "${2:-}" == "--full" ]]; then
  r test full
fi
