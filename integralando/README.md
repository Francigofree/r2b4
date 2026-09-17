# R2B4 test refactor upgrade

Baseline analysed: `7c623998ead6a8ddd148a49793aee0b1be91378e`

This package changes only the test system. It intentionally modifies no production
`v3/` source file.

## Apply

```bash
cd /home/alba/project_r2b4
python3 /PATH/TO/r2b4_test_refactor_upgrade_20260917/apply_upgrade.py /home/alba/project_r2b4
```

The apply script makes a one-time backup of the previous touched test files under
`/tmp/r2b4_test_refactor_backup_<timestamp>` and then applies the upgrade.

## Validate

Fast agent gate:

```bash
python3 -m pytest -q tests/test_v3_gate.py -x
```

Touched suites:

```bash
bash /PATH/TO/r2b4_test_refactor_upgrade_20260917/validate_upgrade.sh /home/alba/project_r2b4
```

Full pytest:

```bash
bash /PATH/TO/r2b4_test_refactor_upgrade_20260917/validate_upgrade.sh /home/alba/project_r2b4 --full
```

## Design

- central `import_guard` is the architecture dependency authority;
- architecture tests guard capabilities/authority, not complete import whitelists;
- L6 tests derive candidate/sample expectations from config instead of hard-coding `54/8`;
- replay no longer requires a plan handoff on one exact control tick;
- hardware fixtures disable camera + person detection as one coherent optional capability;
- ACTIVE TELEOP test missions are always contract-valid;
- active robot wiring/tuning snapshot is isolated in one obvious test file;
- Test Hub tests are grouped by responsibility instead of historical `next/v2/p0` names;
- `test_v3_gate.py` is the fast high-signal agent-first gate;
- full pytest remains the final regression authority.
