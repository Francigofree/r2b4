#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-.}"
PKG_DIR="$(cd "$(dirname "$0")" && pwd)"

cd "$REPO"

python3 -m py_compile v3/layers/l6_navigation.py
python3 "$PKG_DIR/smoke_replan_cooldown.py" "$REPO"

python3 -m pytest -q \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_l6_planning_scene.py \
  tests/test_v3_replay.py
