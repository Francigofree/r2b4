#!/usr/bin/env bash
# Install four-channel Pi 5 PWM pinmux. Never applies positive motor output.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
python3 - <<'PY'
from v3.robot_interface import RobotInterface
if RobotInterface().read('operator.status').get('runtime_running'):
    raise SystemExit('Stop the canonical runtime before PWM installation')
PY
work_dir="$(mktemp -d "$repo_dir/tmp/rp1_install_XXXXXX")"
dtc -@ -I dts -O dtb -o "$work_dir/r2b4-pwm.dtbo" deploy/r2b4-pwm-overlay.dts
cp /boot/firmware/config.txt "$work_dir/config.txt.before"
python3 - "$work_dir" <<'PY'
from pathlib import Path
import sys
work = Path(sys.argv[1])
text = (work/'config.txt.before').read_text()
old = 'dtoverlay=pwm-2chan'
if old in text.splitlines():
    text = '\n'.join('dtoverlay=r2b4-pwm' if line == old else line for line in text.splitlines()) + '\n'
elif 'dtoverlay=r2b4-pwm' not in text.splitlines():
    raise SystemExit('Expected the existing pwm-2chan overlay; inspect boot config before changing it')
(work/'config.txt').write_text(text)
PY
sudo -n install -m 644 "$work_dir/r2b4-pwm.dtbo" /boot/firmware/overlays/r2b4-pwm.dtbo
sudo -n install -m 644 "$work_dir/config.txt" /boot/firmware/config.txt
# Existing boot overlay already enables PWM0. Bring up the two additional
# pin functions now, without reboot or any duty write. Boot uses the overlay.
sudo -n pinctrl set 12 a0 pd
sudo -n pinctrl set 13 a0 pd
sudo -n pinctrl set 18 a3 pd
sudo -n pinctrl set 19 a3 pd
pinctrl get 12-13
pinctrl get 18-19
printf 'Boot config backup: %s/config.txt.before\n' "$work_dir"
