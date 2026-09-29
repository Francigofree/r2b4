#!/usr/bin/env bash
set -euo pipefail
python -m pytest -q \
  tests/feature/test_v3_lidar_world_model.py \
  tests/feature/test_v3_roomcruise_localization_motion.py \
  tests/feature/test_v3_dual_frame_localization.py \
  tests/feature/test_v3_rate_only_heading_authority.py
