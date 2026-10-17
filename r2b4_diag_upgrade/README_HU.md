# R2B4 DIAG-only fejlesztőcsomag

Forrás: `Francigofree/r2b4`, 2026-10-17, `tools/diag/registry.py`, `contracts.py`, `navigation.py`, `motion_performance.py` alapján.

## Hatókör

ÚJ, csak olvasó `motion_forensics` offline DIAG analyzer. A meglévő EVI bundle-ből számol; nincs produkciós vezérlési, motor-, navigációs, capture- vagy konfigurációs változtatás.

## Telepítés a Raspberry Pi-n

A ZIP kitömörítése után a projekt gyökerében:

```bash
python3 /a/kicsomagolt/r2b4_diag_upgrade/install_diag_only.py ~/project_r2b4
cd ~/project_r2b4
python3 -m pytest -q /a/kicsomagolt/r2b4_diag_upgrade/tests/packs/feature/test_motion_forensics_upgrade.py
r diag motion_forensics latest --json
r diag full latest --json
```

A telepítő a regisztráció előtt ellenőrzi a forrásalakot, és `.py.motion_forensics_backup` fájlba menti az eredeti registryt. Nem indít robotmozgást, és nem futtat automatikusan DIAG-ot. DIAG futtatása a robot leállítása után történjen.

## Funkciók és korlátok

- P0 beragadás: megállási lánc időszakaszok, terv/safety kezdeti tények, EVI referenciák (első 32 szakasz).
- P0 célválasztás: rögzített döntések és mért célmegközelítés (csak azonos lokális koordinátarendszerben).
- P0 encoder–IMU–LiDAR–motor: releváns L3 konzisztencia-, frissesség- és slip-jelzők forrásreferenciával. **Nem** állít elő független, négyágú mérést puszta L3 fúzióból.
- P1 50 Hz: csak akkor `PRESENT`, ha explicit nagyfrekvenciás forrásesemény van; 5 Hz-ből nem rekonstruálja.
- P1 L6 jelöltek: megőrzött score-tartományok és kiválasztott azonosító; score irányát nem feltételezi.
- P1 safety/localization: forrásidőhöz kapcsolt állapotváltozás minták.
- P1 ismételt terv: közvetlen egymást követő ismétlődés számláló; **nem** bizonyított escape-sikeresség.
- P1 térképfrissesség: döntéskori L4 freshness és L6 reason, legfeljebb 32 minta.
- P1 CPU: **hiányzó adatként** jelzi, ha nincs 1 Hz-es processzor-esemény; oksági következtetést nem ad.

A jelenlegi változat **részleges megvalósítás**, nem teljes P0/P1 acceptance. A kibővített DIAG éles EVI-adaton való lefutása nem volt végrehajtható ebben a környezetben; a mellékelt minimális tesztek ellenőrzik az alaphelyzeteket, nem helyettesítik a teljes repo regressziós tesztcsomagot.
