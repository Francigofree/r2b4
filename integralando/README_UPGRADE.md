# R2B4 pytest fast refactor — 2026-10-03

Source snapshot: `Francigofree/r2b4` main at `4f12c19d5aae047bcb581d03dcafba8188daec42`.

## Mit változtat?

- `r test` többé nem a teljes CORE réteget futtatja, hanem **11 nagy jelértékű, hardvermentes QUICK esetet**, `--maxfail=1` módban.
- `r test core` explicit módon futtatja a teljes CORE réteget.
- `r test full` változatlanul a teljes CORE + FEATURE + DEEP regresszió.
- A fókuszált módok (`follow`, `roomcruise`, `localization`, `perception`, `motion`, `async`, `process`, `replay`) explicit fájl-listát kapnak. A pytest nem gyűjti/importálja előbb az egész FEATURE vagy DEEP réteget.
- A nyers `r pytest ...` útvonal változatlan.
- Production robotkódhoz, V3 contracthoz, config authorityhoz és hardverkezeléshez nem nyúl.

## Telepítés Raspberry Pi-n

A ZIP-et bontsd ki, majd a projekt gyökeréből:

```bash
cd /home/alba/project_r2b4
python3 /PATH/TO/apply_pytest_fast_refactor.py --root /home/alba/project_r2b4 --dry-run
python3 /PATH/TO/apply_pytest_fast_refactor.py --root /home/alba/project_r2b4
```

A telepítő source-lockolt. Ha a repo HEAD vagy az öt érintett fájl már eltér a csomag készítésekor látott állapottól, **nem ír felül semmit**.

## Ellenőrzés

A telepítő automatikusan futtat:

1. `py_compile` az érintett Python fájlokra;
2. `python3 -m v3.test_runner list`;
3. a QUICK célok `pytest --collect-only` ellenőrzését.

Ezután:

```bash
./r test
./r test core
./r test roomcruise
./r test full
```

## Rollback

A telepítő automatikusan backupot készít `.upgrade_backups/pytest_fast_refactor_20261003_*` alatt.

```bash
python3 /PATH/TO/apply_pytest_fast_refactor.py --root /home/alba/project_r2b4 --rollback
```
