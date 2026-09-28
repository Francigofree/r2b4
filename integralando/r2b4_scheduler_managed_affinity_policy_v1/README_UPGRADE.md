# R2B4 scheduler-managed affinity policy V1

Source-first baseline: `1333b49a2917f46753249f73599b6f7a98dc4f42` (main, 2026-09-27).

## Mit változtat

- `README.md`: az aktív `runtime_affinity.enabled=false` scheduler-managed módot dokumentálja.
- `tools/v3_performance_audit.py`: kikapcsolt affinity mellett `NOT_APPLICABLE / SCHEDULER_MANAGED`, exit 0; bekapcsolt módban a régi `/proc` affinity audit változatlan.
- `tools/v3_p0_async_acceptance.py`: kikapcsolt affinity többé nem acceptance-hiba; evidence-ben scheduler-managed állapotként jelenik meg.
- `tests/core/test_v3_config_p0_authority.py`: rögzíti, hogy az aktív baseline scheduler-managed, miközben a CPU-mask schema továbbra is szigorú.
- `tests/deep/test_v3_process_affinity.py`: bizonyítja, hogy `enabled=false` esetén az affinity-layout no-op és nem szűkíti az örökölt Linux cpusetet.

## Nem változik

- production runtime/control kód;
- safety-, motor-, sensor-, matcher- és planner-logika;
- `conf/vezerles.json`;
- az `enabled=true` explicit affinity mód és annak guardjai.

## Telepítés

```bash
cd /home/alba/project_r2b4/integralando/r2b4_scheduler_managed_affinity_policy_v1
python3 installer.py
```

Az installer per-file Git blob SHA-val ellenőrzi a source-first baseline-t, backupot készít, atomikusan patch-el, majd `py_compile`-t és két célzott pytest fájlt futtat. Teszthiba esetén automatikus rollback történik.

Teszt nélküli telepítés csak szükség esetén:

```bash
python3 installer.py --skip-tests
```
