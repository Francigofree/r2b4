# R2B4 DIAG upgrade — 2026-10-01

Source-first base: `Francigofree/r2b4` main commit `4bc8f757a476c3438c775c5ab8d53264316a90a7`.

Installs the on-demand MCAP-only DIAG framework, registry, admission/coverage contracts, two foundation analyzers (`capture`, `coverage`), documentation, tests and the `r diag` launcher facade.

## Apply

From the extracted package:

```bash
python3 apply_diag_upgrade.py --root /home/alba/project_r2b4
```

If HEAD has advanced but the guarded source anchors still match:

```bash
python3 apply_diag_upgrade.py --root /home/alba/project_r2b4 --allow-head-mismatch
```

The installer is fail-closed: it checks repository shape, git HEAD (unless explicitly relaxed), exact patch anchors and Python compilation before modifying files. It creates a timestamped backup under `.upgrade_backups/diag_subsystem_*`.

After install:

```bash
r diag list
r diag capture latest
r diag coverage latest
python3 -m pytest -q tests/feature/test_v3_diag_framework.py
```
