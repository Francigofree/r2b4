# R2B4 launcher convergence upgrade — 2026-09-23

Cél: egyetlen egyszerű `r` ember/agent launcher, amely nem duplikál robotikai authorityt.

## Alkalmazás

```bash
cd /home/alba/project_r2b4
python3 /path/to/r2b4_launcher_upgrade_20260923/apply_upgrade.py
```

A script source fájlokról `/tmp` alatt készít backupot, runtime/capture/log adatot nem módosít.
A régi gyökér `r2b4` launcher törlődik. A belső `v3.operator_cli` megmarad, mert a
resident runtime session technikai child-process entrypointja továbbra is ezt használja.

## Validáció

```bash
/path/to/r2b4_launcher_upgrade_20260923/validate.sh /home/alba/project_r2b4
```

Nem indít fizikai robotmozgást.

## Struktúra

- `r`: ~0.6 KB bootstrap
- `v3.launcher_cli`: dispatch + help + agent discovery
- `v3.host_cli`: host/developer helpers
- `v3.interface_cli`: canonical robot-facing CLI

