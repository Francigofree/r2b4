#!/usr/bin/env bash
# R2B4 short RobotInterface launcher.  The existing ./r2b4 remains untouched.
set -u
SRC="${BASH_SOURCE[0]}"
ROOT="$(cd "$(dirname "$SRC")" && pwd)"
cd "$ROOT" || exit 1
exec python3 -m v3.interface_cli "$@"
