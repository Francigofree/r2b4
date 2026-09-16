#!/usr/bin/env bash
set -e
cd "${1:-.}"
python3 -m pytest -q \
  tests/test_v3_person_detection.py \
  tests/test_v3_person_detection_runtime_integration.py \
  tests/test_v3_person_spatial_tracking.py \
  tests/test_v3_l4_world_model.py \
  tests/test_v3_native_sensor_inputs.py \
  tests/test_v3_bounded_runtime_config.py
python3 -m pytest -q tests/test_v3_import_guard.py
