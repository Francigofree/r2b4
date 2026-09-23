#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-/home/alba/project_r2b4}"
cd "$ROOT"
python3 -m compileall -q v3/launcher_cli.py v3/interface_cli.py
bash -n r
python3 -m pytest -q \
  tests/test_v3_launcher_cli.py \
  tests/test_v3_interface_cli.py \
  tests/test_v3_capture_hz.py
