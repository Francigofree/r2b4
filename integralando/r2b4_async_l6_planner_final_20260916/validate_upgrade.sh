#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-/home/alba/project_r2b4}"
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

echo "[1/6] source compile"
python3 -m py_compile \
  v3/layers/l6_navigation.py \
  v3/adapters/l6_planner_process.py \
  v3/composition/native_control.py \
  v3/composition/resident_live_control.py \
  v3/composition/resident_physical_control.py \
  v3_runtime.py v3_hardware_runtime.py v3_bounded_config.py v3/replay.py \
  tests/test_v3_async_l6_planner.py

echo "[2/6] architecture/import boundary"
python3 -m pytest -q tests/test_v3_architecture_boundaries.py

echo "[3/6] async L6 focused regression"
python3 -m pytest -q \
  tests/test_v3_async_l6_planner.py \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_l5_l9_mission_navigation.py \
  tests/test_v3_native_control_composition.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_fault_replay.py

echo "[4/6] real spawn-process deterministic smoke"
python3 "$PKG/smoke_async_l6_process.py" "$REPO"

echo "[5/6] replay/capture focused regression"
python3 -m pytest -q \
  tests/test_v3_mcap_e2e.py \
  tests/test_v3_triggered_capture.py \
  tests/test_v3_execution.py

echo "[6/6] full regression"
if [[ "${R2B4_SKIP_FULL:-0}" == "1" ]]; then
  echo "SKIP: full regression requested via R2B4_SKIP_FULL=1"
else
  python3 -m pytest -q
fi

echo "PASS: asynchronous L6 upgrade validation complete"
