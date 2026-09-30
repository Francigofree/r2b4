#!/usr/bin/env bash
set -euo pipefail
cd "${1:-/home/alba/project_r2b4}"

python3 -m pytest -q \
  tests/feature/test_v3_l11_reversal_reacquisition.py \
  tests/feature/test_v3_motion_feedback_quality.py \
  tests/feature/test_v3_counter_encoder_backend.py \
  tests/feature/test_v3_encoder_ab_direction_robustness.py

git diff --check
