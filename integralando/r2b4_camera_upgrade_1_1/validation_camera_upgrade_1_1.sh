#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-/home/alba/project_r2b4}"
cd "$REPO"
python3 -m pytest -q \
  tests/test_v3_camera_foundation.py \
  tests/test_v3_device_health_policy.py \
  tests/test_v3_tick_engine.py \
  tests/test_v3_bounded_live_control_composition.py \
  tests/test_v3_resident_runtime.py

python3 -m py_compile \
  v3/device_health_policy.py \
  v3/adapters/picamera2_camera.py \
  v3/adapters/live_camera.py \
  v3/layers/l12_safety_final.py \
  v3/composition/bounded_live_control.py \
  v3/composition/resident_live_control.py \
  v3/replay.py \
  v3_bounded_config.py

echo "camera upgrade 1.1 targeted validation: PASS"
