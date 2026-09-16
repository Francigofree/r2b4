#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-/home/alba/project_r2b4}"
cd "$REPO"

echo "[1/6] compile"
python3 -m py_compile \
  v3/layers/l6_navigation.py \
  v3/composition/native_control.py \
  tests/test_v3_async_l6_planner.py

echo "[2/6] architecture/import boundary"
python3 -m pytest -q tests/test_v3_architecture_boundaries.py

echo "[3/6] async L6 timebase + compatibility"
python3 -m pytest -q tests/test_v3_async_l6_planner.py

echo "[4/6] navigation/composition/runtime"
python3 -m pytest -q \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_l5_l9_mission_navigation.py \
  tests/test_v3_native_control_composition.py \
  tests/test_v3_resident_runtime.py

echo "[5/6] replay/capture regression"
python3 -m pytest -q \
  tests/test_v3_fault_replay.py \
  tests/test_v3_mcap_e2e.py \
  tests/test_v3_triggered_capture.py \
  tests/test_v3_execution.py

echo "[6/6] full regression"
python3 -m pytest -q

echo "PASS: async-L6 monotonic handoff upgrade validated"
