#!/usr/bin/env bash
# R2B4 unified human/agent launcher.
# Robot commands stay on the canonical v3.interface_cli path.
# Host/developer helpers are intentionally launcher-only and do not enter V3 control authority.
set -u

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SELF_DIR="$(cd "$(dirname "$SELF")" && pwd)"
DEFAULT_ROOT="/home/alba/project_r2b4"

resolve_root() {
    if [[ -n "${R2B4_ROOT:-}" ]]; then
        printf '%s\n' "$R2B4_ROOT"
        return 0
    fi
    if [[ -d "$SELF_DIR/v3" && -f "$SELF_DIR/pytest.ini" ]]; then
        printf '%s\n' "$SELF_DIR"
        return 0
    fi
    if [[ -d "$DEFAULT_ROOT" ]]; then
        printf '%s\n' "$DEFAULT_ROOT"
        return 0
    fi
    printf 'ERROR: R2B4 repo not found. Set R2B4_ROOT=/path/to/project_r2b4\n' >&2
    return 2
}

ROOT="$(resolve_root)" || exit $?

need_root() {
    if [[ ! -d "$ROOT" || ! -f "$ROOT/pytest.ini" ]]; then
        printf 'ERROR: invalid R2B4 root: %s\n' "$ROOT" >&2
        return 2
    fi
}

print_help() {
    cat <<'HELP'
R2B4 launcher

Robot (canonical V3 interface; existing commands unchanged):
  r s | status                 robot/runtime short status
  r d | diag                   diagnostics
  r rc 30                      Room Cruise 30 s
  r fp 20                      follow person 20 s
  r f 10 0.15                  forward 10 s at 0.15 m/s
  r x                           STOP
  r sd                          STOP + runtime shutdown
  r th [status|run|batch]      Test Hub
  r system                      existing V3 system.status
  ...                           every existing v3.interface_cli command still works

Developer tools:
  r gitre [commit message]      run gitre.py (alias: r gittre)
  r git <args...>               run git in the R2B4 repo
  r pytest [args...]            run python3 -m pytest in the repo
  r tools                       list repo Python tools
  r tool NAME [args...]         run tools/NAME.py or ROOT/NAME.py

RPi / Linux helpers:
  r cpu [SECONDS] [INTERVAL]    per-core CPU monitor; default 30 s / 1 s
                                log: runtime/cpu/cpu_YYYYmmdd_HHMMSS.log
  r cpu2 [SECONDS] [INTERVAL]   deep /proc runtime diag; default 30 s / 0.5 s
                                writes cpu2 NDJSON + factual summary JSON
  r disc                        free disk + runtime/capture sizes (alias: disk)
  r mem                         memory usage
  r temp                        CPU temperature
  r ps                          top processes with CPU core (PSR)
  r net                         addresses + default route
  r usb                         USB devices
  r i2c                         I2C bus 1 scan (if i2cdetect exists)
  r host                        compact host summary
  r root                        print resolved repo path

Global launcher:
  ./r install                   install/symlink as "r" in a user-writable PATH dir
  r where                       show launcher and repo paths

Repo root resolution:
  1) R2B4_ROOT environment variable
  2) directory containing this r file, if it is the repo root
  3) /home/alba/project_r2b4
HELP
}

install_global() {
    local candidates=()
    local dir dest=""

    # Prefer an already active user PATH directory so the command works immediately.
    IFS=':' read -r -a candidates <<< "${PATH:-}"
    for dir in "${candidates[@]}"; do
        [[ -z "$dir" ]] && continue
        case "$dir" in
            "$HOME/.local/bin"|"$HOME/bin")
                mkdir -p "$dir"
                if [[ -w "$dir" ]]; then
                    dest="$dir/r"
                    break
                fi
                ;;
        esac
    done

    if [[ -z "$dest" ]]; then
        mkdir -p "$HOME/.local/bin"
        dest="$HOME/.local/bin/r"
    fi

    ln -sfn "$SELF" "$dest"
    chmod +x "$SELF"
    printf 'Installed: %s -> %s\n' "$dest" "$SELF"

    case ":${PATH:-}:" in
        *":$(dirname "$dest"):"*)
            printf 'Ready. Use from any directory: r help\n'
            ;;
        *)
            printf 'NOTE: %s is not in the current PATH.\n' "$(dirname "$dest")"
            printf 'For this shell run: export PATH="%s:$PATH"\n' "$(dirname "$dest")"
            printf 'A new login shell on Debian/Raspberry Pi OS usually adds ~/.local/bin automatically.\n'
            ;;
    esac
}

list_tools() {
    need_root || return $?
    printf 'Repo Python tools:\n'
    local found=0 f
    if [[ -d "$ROOT/tools" ]]; then
        for f in "$ROOT"/tools/*.py; do
            [[ -e "$f" ]] || continue
            printf '  %-30s  tools/%s\n' "$(basename "$f" .py)" "$(basename "$f")"
            found=1
        done
    fi
    for f in "$ROOT"/*.py; do
        [[ -e "$f" ]] || continue
        printf '  %-30s  %s\n' "$(basename "$f" .py)" "$(basename "$f")"
        found=1
    done
    [[ $found -eq 1 ]] || printf '  (none)\n'
}

find_tool() {
    local name="$1"
    local base="$name"
    [[ "$base" == *.py ]] || base="${base}.py"
    if [[ -f "$ROOT/tools/$base" ]]; then
        printf '%s\n' "$ROOT/tools/$base"
        return 0
    fi
    if [[ -f "$ROOT/$base" ]]; then
        printf '%s\n' "$ROOT/$base"
        return 0
    fi
    return 1
}

is_hardware_tool() {
    local base="${1##*/}"
    base="${base%.py}"
    case "$base" in
        r2b4_voice_mic_test|v3_camera_test|v3_encoder_ab_probe|v3_person_detection_test|v3_sensor_measurement)
            return 0 ;;
    esac
    return 1
}

hardware_guard() {
    need_root || return $?
    command -v flock >/dev/null 2>&1 || { printf 'ERROR: flock is required.\n' >&2; return 2; }
    mkdir -p "$ROOT/runtime"
    exec 9>"$ROOT/runtime/.r2b4_operator.lock"
    chmod 600 "$ROOT/runtime/.r2b4_operator.lock"
    flock -x 9 || return 2
    cd "$ROOT" || return 2
    python3 -c 'from pathlib import Path; from v3.operator_controller import OperatorController; raise SystemExit(3 if OperatorController(Path.cwd()).status().get("runtime_running") else 0)'
    local rc=$?
    if [[ $rc -eq 3 ]]; then
        printf 'REFUSED: resident V3 runtime owns physical hardware. Stop it before this diagnostic.\n' >&2
        flock -u 9 || true; exec 9>&-; return 3
    fi
    if [[ $rc -ne 0 ]]; then
        printf 'ERROR: hardware ownership check failed.\n' >&2
        flock -u 9 || true; exec 9>&-; return $rc
    fi
}

release_hardware_guard() {
    flock -u 9 2>/dev/null || true
    exec 9>&-
}

run_tool() {
    need_root || return $?
    local name="${1:-}"
    [[ -n "$name" ]] || { printf 'ERROR: tool name required. Use: r tools\n' >&2; return 2; }
    shift || true
    local path
    path="$(find_tool "$name")" || {
        printf 'ERROR: tool not found: %s\n' "$name" >&2
        printf 'Use: r tools\n' >&2
        return 2
    }
    cd "$ROOT" || return 2
    if is_hardware_tool "$name"; then
        hardware_guard || return $?
        python3 "$path" "$@"
        local rc=$?
        release_hardware_guard
        return $rc
    fi
    exec python3 "$path" "$@"
}

show_disc() {
    need_root || return $?
    printf 'Disk filesystem:\n'
    df -h "$ROOT"
    printf '\nR2B4 data sizes:\n'
    if [[ -d "$ROOT/runtime" ]]; then
        du -sh "$ROOT/runtime" 2>/dev/null || true
    fi
    if [[ -d "$ROOT/runtime/captures" ]]; then
        du -sh "$ROOT/runtime/captures" 2>/dev/null || true
    fi
    if [[ -d "$ROOT/runtime/cpu" ]]; then
        du -sh "$ROOT/runtime/cpu" 2>/dev/null || true
    fi
}

show_temp() {
    local p="/sys/class/thermal/thermal_zone0/temp"
    if [[ -r "$p" ]]; then
        awk '{v=$1; if (v>1000) v=v/1000; printf "CPU temperature: %.1f C\n", v}' "$p"
    else
        printf 'CPU temperature: unavailable\n'
    fi
}

show_host() {
    printf 'Host: %s\n' "$(hostname 2>/dev/null || printf '?')"
    printf 'Kernel: %s\n' "$(uname -srmo 2>/dev/null || uname -a)"
    printf 'Uptime/load: '
    uptime 2>/dev/null || true
    show_temp
    printf '\nMemory:\n'
    free -h 2>/dev/null || true
    printf '\nDisk:\n'
    df -h "$ROOT" 2>/dev/null || true
}

cpu_monitor() {
    need_root || return $?
    local seconds="${1:-30}"
    local interval="${2:-1}"

    python3 - "$seconds" "$interval" "$ROOT" <<'PY'
from __future__ import annotations

import datetime as dt
import os
import pathlib
import subprocess
import sys
import time

try:
    duration = float(sys.argv[1])
    interval = float(sys.argv[2])
except ValueError:
    raise SystemExit("ERROR: SECONDS and INTERVAL must be numbers")
root = pathlib.Path(sys.argv[3])
if duration <= 0 or interval <= 0:
    raise SystemExit("ERROR: SECONDS and INTERVAL must be > 0")

out_dir = root / "runtime" / "cpu"
out_dir.mkdir(parents=True, exist_ok=True)
stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = out_dir / f"cpu_{stamp}.log"


def read_cpu():
    result = {}
    with open("/proc/stat", "r", encoding="utf-8") as fh:
        for line in fh:
            if not line.startswith("cpu"):
                break
            parts = line.split()
            name = parts[0]
            if name == "cpu":
                continue
            values = [int(x) for x in parts[1:]]
            total = sum(values)
            idle = (values[3] if len(values) > 3 else 0) + (values[4] if len(values) > 4 else 0)
            result[name] = (total, idle)
    return result


def temperature():
    p = pathlib.Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        value = float(p.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return None
    return value / 1000.0 if value > 1000 else value


def loadavg():
    try:
        return os.getloadavg()
    except OSError:
        return (0.0, 0.0, 0.0)


def top_processes():
    try:
        cp = subprocess.run(
            ["ps", "-eo", "pid,psr,pcpu,pmem,etime,comm,args", "--sort=-pcpu"],
            check=False,
            capture_output=True,
            text=True,
        )
        lines = cp.stdout.splitlines()[:13]
        return "\n".join(lines)
    except OSError:
        return "ps unavailable"

print(f"CPU monitor: {duration:g}s, interval {interval:g}s")
print(f"Log: {log_path}")

previous = read_cpu()
started = time.monotonic()
next_ps = 0.0
with log_path.open("w", encoding="utf-8", buffering=1) as log:
    log.write(f"R2B4 CPU monitor start={dt.datetime.now().isoformat()} duration_s={duration} interval_s={interval}\n")
    while True:
        elapsed = time.monotonic() - started
        if elapsed >= duration:
            break
        time.sleep(min(interval, max(0.0, duration - elapsed)))
        current = read_cpu()
        pieces = []
        for name in sorted(current, key=lambda s: int(s[3:]) if s[3:].isdigit() else 999):
            if name not in previous:
                continue
            total_delta = current[name][0] - previous[name][0]
            idle_delta = current[name][1] - previous[name][1]
            usage = 0.0 if total_delta <= 0 else 100.0 * (total_delta - idle_delta) / total_delta
            pieces.append(f"{name}={usage:5.1f}%")
        previous = current
        la = loadavg()
        temp = temperature()
        now = dt.datetime.now().strftime("%H:%M:%S")
        temp_text = "temp=n/a" if temp is None else f"temp={temp:.1f}C"
        line = f"{now} t={time.monotonic()-started:5.1f}s load={la[0]:.2f}/{la[1]:.2f}/{la[2]:.2f} {temp_text} | " + " ".join(pieces)
        print(line)
        log.write(line + "\n")

        elapsed = time.monotonic() - started
        if elapsed >= next_ps:
            log.write("\nTOP PROCESSES (PID PSR CPU% MEM% ELAPSED COMM ARGS)\n")
            log.write(top_processes() + "\n\n")
            next_ps = elapsed + 5.0

    log.write(f"END {dt.datetime.now().isoformat()}\n")

print(f"CPU log saved: {log_path}")
PY
}

if [[ $# -eq 0 ]]; then
    print_help
    exit 0
fi

CMD="$1"
shift || true

case "$CMD" in
    help|-h|--help)
        print_help
        ;;
    install)
        install_global
        ;;
    where)
        printf 'launcher: %s\nrepo:     %s\n' "$SELF" "$ROOT"
        ;;
    root)
        printf '%s\n' "$ROOT"
        ;;
    gitre|gittre)
        need_root || exit $?
        cd "$ROOT" || exit 2
        exec python3 "$ROOT/gitre.py" "$@"
        ;;
    git)
        need_root || exit $?
        exec git -C "$ROOT" "$@"
        ;;
    pytest|tests)
        need_root || exit $?
        cd "$ROOT" || exit 2
        exec python3 -m pytest "$@"
        ;;
    tools)
        list_tools
        ;;
    tool|script)
        run_tool "$@"
        ;;
    cpu)
        cpu_monitor "$@"
        ;;
    cpu2)
        need_root || exit $?
        exec python3 "$ROOT/tools/r2b4_cpu2.py" --root "$ROOT" "$@"
        ;;
    disc|disk)
        show_disc
        ;;
    mem|memory)
        free -h
        ;;
    temp|temperature)
        show_temp
        ;;
    ps|processes)
        ps -eo pid,psr,pcpu,pmem,etime,comm,args --sort=-pcpu | head -n 26
        ;;
    net|network)
        printf 'Addresses:\n'
        ip -brief address 2>/dev/null || ip addr
        printf '\nRoutes:\n'
        ip route 2>/dev/null || true
        ;;
    usb)
        if command -v lsusb >/dev/null 2>&1; then
            lsusb
        else
            printf 'lsusb is not installed.\n' >&2
            exit 1
        fi
        ;;
    i2c)
        hardware_guard || exit $?
        if command -v i2cdetect >/dev/null 2>&1; then
            i2cdetect -y 1; rc=$?
        else
            printf 'i2cdetect is not installed (package: i2c-tools).\n' >&2; rc=1
        fi
        release_hardware_guard
        exit $rc
        ;;
    host)
        show_host
        ;;
    *)
        # Unknown launcher words always go to the canonical robot interface.
        # Repo tools are explicit via: r tool NAME ...
        need_root || exit $?
        cd "$ROOT" || exit 2
        exec python3 -m v3.interface_cli "$CMD" "$@"
        ;;
esac
