#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-/home/alba/project_r2b4}"
cd "$ROOT"

git diff --check
python3 -m pytest -q \
  tests/test_v3_pytest_profiles.py \
  tests/test_v3_test_hub_portable.py \
  tests/test_v3_test_hub_behavior.py \
  tests/test_v3_test_hub_quality.py
python3 -m v3.test_hub test --scope testhub
python3 -m v3.test_hub test --scope async
