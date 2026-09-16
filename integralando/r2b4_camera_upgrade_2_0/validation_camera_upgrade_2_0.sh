#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-.}"
cd "$REPO"

python3 -m py_compile \
  v3/adapters/picamera2_camera.py \
  v3/adapters/live_camera.py \
  v3/adapters/camera_media.py \
  v3/composition/native_sensor_inputs.py \
  v3/composition/live_inputs.py \
  v3/composition/bounded_live_control.py \
  v3/composition/resident_live_control.py \
  v3/composition/bounded_physical_control.py \
  v3/composition/resident_physical_control.py \
  v3_bounded_runtime.py \
  v3_runtime.py \
  v3_hardware_runtime.py \
  v3_bounded_config.py \
  tools/v3_camera_test.py

python3 -m pytest -q \
  tests/test_v3_camera_foundation.py \
  tests/test_v3_device_health_policy.py \
  tests/test_v3_native_sensor_inputs.py \
  tests/test_v3_bounded_live_control_composition.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_hardware_runtime.py \
  tests/test_v3_bounded_runtime_config.py

echo "camera upgrade 2.0 targeted validation: PASS"
echo "Recommended before physical motion: python3 -m pytest -q"
