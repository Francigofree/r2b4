#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-.}"
cd "$REPO"

python3 -m py_compile \
  v3/adapters/picamera2_camera.py \
  v3/adapters/live_camera.py \
  tools/v3_camera_test.py \
  tests/test_v3_camera_foundation.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_hardware_runtime.py

git diff --check -- \
  v3/adapters/picamera2_camera.py \
  v3/adapters/live_camera.py \
  tools/v3_camera_test.py \
  tests/test_v3_camera_foundation.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_hardware_runtime.py

python3 -m pytest -q \
  tests/test_v3_camera_foundation.py \
  tests/test_v3_device_health_policy.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_hardware_runtime.py \
  tests/test_v3_native_sensor_inputs.py \
  tests/test_v3_bounded_live_control_composition.py

echo "camera upgrade 2.0.2 targeted validation: PASS"
echo "Next gate: python3 -m pytest -q"
echo "Only after full regression is green: python3 tools/v3_camera_test.py status --seconds 30"
