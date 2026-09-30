#!/usr/bin/env bash
set -euo pipefail
cd "${1:-/home/alba/project_r2b4}"

python3 -m pytest -q   tests/feature/test_v3_stationary_covariance.py   tests/feature/test_v3_dual_frame_localization.py

git diff --check
