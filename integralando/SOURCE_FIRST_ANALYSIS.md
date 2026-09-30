# Source-first Test Hub lezárási elemzés

Baseline: `Francigofree/r2b4` main, commit `90417581bea1ba64db761a4889d85fb409756c6f`.

## Megállapítás

A Test Hub funkcionálisan már az új MCAP/portable-evidence architektúrán dolgozik, de a migráció szerkezetileg nincs lezárva:

- `v3/test_hub.py` egyszerre régi Test Hub implementáció és dispatcher.
- A magas szintű tényleges orchestration `v3/test_hub_next.py` alatt él.
- Az MCAP diagnose/query/verify backend `v3/test_hub_v2.py` alatt él.
- A README még közvetlenül ajánlja a verziózott modulokat és régi kompatibilitási parancsokat.
- Aktív toolok/tesztek közvetlenül importálnak a verziózott modulokból.
- Az újonnan generált agent brief is verziózott backend CLI-parancsot tud ajánlani.

Ez nem elsősorban diagnosztikai képességhiány, hanem lezáratlan migráció és SSOT-probléma.

## Amit nem szabad legacy-ként törölni

- MCAP capture authority.
- Evidence schema verzióazonosítók (`..._V2`): ezek tárolt adatformátum-verziók.
- Régi MCAP-ok metaadat nélküli 50 Hz kompatibilitási értelmezése.
- `v3.replay` canonical replay core.
- MCAP -> bounded ideiglenes JSON replay bridge. Ez a jelenlegi canonical replayhez szükséges, és az ideiglenes fájlt törli.
- Történeti `runtime/` evidence és `.upgrade_backups/` tartalom.

## Lezáró architektúra

```text
python3 -m v3.test_hub              <- egyetlen publikus facade
          |
          +-- test_hub_app          <- run/view/batch/compare/test
          |
          +-- test_hub_backend      <- inspect/diagnose/agent/query/verify
          |
          +-- analyzerek / views / portable / replay bridge
```

A verziózott migrációs modulnevek megszűnnek az aktív rendszerből.

## Anti-drift állapot

A rendszer már rendelkezik jó, organikus fejlődést segítő mechanizmussal:

- `diagnostic_contracts.py` production dataclassokból képez diagnosztikai field contractot és fingerprintet.
- `test_hub_diagnostic_coverage.py` az ismeretlen új scalar mezőket is automatikusan összesíti és schema driftet jelez.
- A CORE/FEATURE/DEEP pytest profilok pattern-alapúak, ezért az új megfelelő helyre tett teszt automatikusan bekerül a profilba.

A lezáró refaktor ezt dokumentálja és contract teszttel védi, nem épít párhuzamos registryt.

## Upgrade műveletek

1. `v3/test_hub_next.py` tartalma -> `v3/test_hub_app.py`.
2. `v3/test_hub_v2.py` tartalma -> `v3/test_hub_backend.py`.
3. `v3/test_hub.py` -> vékony kanonikus facade és egységes CLI-dispatcher.
4. Aktív belső importok átállítása az új felelősségi modulokra.
5. Publikus caller/import/CLI példák átállítása kizárólag `v3.test_hub`-ra.
6. A régi verziózott modulok törlése.
7. README Test Hub szakasz aktualizálása.
8. `docs/TEST_HUB.md` létrehozása.
9. `docs/PYTEST_POLICY.md` és `STRUKTURALIS_RETEGEK_V3.md` rövid boundary-hivatkozásának hozzáadása, ha a fájlok jelen vannak.
10. Feature contract teszt hozzáadása, amely megakadályozza a verziózott modulok visszanövését.

## Installer policy

- HEAD mismatch: warning és folytatás alapból.
- `--strict-head`: exact baseline szükséges.
- Strukturális anchor eltérés: fail-fast.
- Minden érintett meglévő fájl tranzakciós backupot kap.
- Írás atomikusan történik.
- Kötelező CLI/import/legacy-reference smoke check.
- Validációs hiba esetén automatikus rollback.
- `runtime/` és `.upgrade_backups/` soha nincs átírva.
