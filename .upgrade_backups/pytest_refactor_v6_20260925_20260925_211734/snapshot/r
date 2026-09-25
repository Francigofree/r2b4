#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ "${1:-}" == "test" ]]; then
  shift
  exec python3 -m v3.test_runner "$@"
fi
exec python3 -m v3.launcher_cli "$@"
