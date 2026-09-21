#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-/home/alba/project_r2b4}"
cd "$ROOT"

python3 -m py_compile \
  v3/capture_ipc.py \
  v3/process_sidecars.py \
  v3/adapters/process_lidar_port.py \
  v3/mcap_capture.py \
  v3/mcap_reader.py \
  v3/mcap_replay_bridge.py \
  v3/test_hub_portable.py \
  v3_process_runtime.py \
  tests/test_v3_capture_refactor_p0.py

run_profile() {
  local profile="$1"
  echo "== pytest profile: $profile =="
  mapfile -t targets < <(python3 - "$profile" <<'PY'
import sys
from v3.pytest_profiles import resolve_pytest_targets
for item in resolve_pytest_targets(".", sys.argv[1]):
    print(item)
PY
)
  if [ "${#targets[@]}" -eq 0 ]; then
    pytest -q -m "$profile"
  else
    pytest -q "${targets[@]}"
  fi
}

run_profile gate
run_profile contract
run_profile async
run_profile replay
run_profile testhub

pytest -q \
  tests/test_v3_capture_refactor_p0.py \
  tests/test_v3_control_process_isolation.py \
  tests/test_v3_sterile_edges.py \
  tests/test_v3_mcap_e2e.py

echo "OFFLINE P0 ACCEPTANCE: PASS"
echo "Remaining physical gate: capture OFF vs ON live timing/jitter comparison."
