#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-/home/alba/project_r2b4}"
cd "$REPO"

echo "[1/5] compile"
python3 -m py_compile \
  v3/layers/l6_navigation.py \
  tests/test_v3_async_l6_planner.py

echo "[2/5] P0 async-L6 regression"
python3 -m pytest -q tests/test_v3_async_l6_planner.py

echo "[3/5] architecture/import boundary"
python3 -m pytest -q tests/test_v3_architecture_boundaries.py

echo "[4/5] replay/control focused regression"
python3 -m pytest -q \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_native_control_composition.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_fault_replay.py \
  tests/test_v3_mcap_e2e.py

echo "[5/5] full regression"
python3 -m pytest -q

echo "PASS: async-L6 P0 handoff freshness fix validated"
