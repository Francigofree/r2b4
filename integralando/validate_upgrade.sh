#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-/home/alba/project_r2b4}"
MODE="${2:-touched}"

cd "$ROOT"

echo "== R2B4 fast gate =="
python3 -m pytest -q tests/test_v3_gate.py -x

echo "== Refactor-touched suites =="
python3 -m pytest -q \
  tests/test_v3_architecture_boundaries.py \
  tests/test_v3_async_l6_planner.py \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_replay.py \
  tests/test_v3_hardware_runtime.py \
  tests/test_v3_person_detection_runtime_integration.py \
  tests/test_v3_bounded_runtime_config.py \
  tests/test_v3_active_robot_config.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_test_hub_analysis.py \
  tests/test_v3_test_hub_evidence.py \
  tests/test_v3_test_hub_cli.py

if [[ "$MODE" == "--full" || "$MODE" == "full" ]]; then
  echo "== Full pytest =="
  python3 -m pytest -q
fi
