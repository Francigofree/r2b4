#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-/home/alba/project_r2b4}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

for rel in \
  v3/adapters/counter_encoder.py \
  v3/adapters/gpio_counter.py \
  v3/layers/l11_actuator_control.py; do
  src="$HERE/$rel"
  dst="$ROOT/$rel"
  if [[ ! -f "$src" ]]; then
    echo "missing replacement: $src" >&2
    exit 2
  fi
  if [[ -f "$dst" ]]; then
    cp -a "$dst" "$dst.before_encoder_fix_$STAMP"
  fi
  install -m 0644 "$src" "$dst"
done

install -m 0644 \
  "$HERE/tests/test_v3_encoder_robustness_regressions.py" \
  "$ROOT/tests/test_v3_encoder_robustness_regressions.py"

python3 -m py_compile \
  "$ROOT/v3/adapters/counter_encoder.py" \
  "$ROOT/v3/adapters/gpio_counter.py" \
  "$ROOT/v3/layers/l11_actuator_control.py"

echo "Encoder robustness replacement installed. Backups suffix: before_encoder_fix_$STAMP"
echo "Run: pytest -q tests/test_v3_encoder_robustness_regressions.py"
