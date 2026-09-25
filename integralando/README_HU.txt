R2B4 pytest refaktor – 2026-09-25
=================================

Cél
---
A jelenlegi ~1200 pytest-eset helyett 80–140 magas jelértékű, robot-szintű teszt.
CORE 20–30, FEATURE 40–70, DEEP 20–40. Hard cap: 150.

Fontos
------
A telepítő source-first módon a HELYI, aktuális /home/alba/project_r2b4 repót elemzi.
Nem tartalmaz lefagyasztott régi tesztlistát, és nem módosít production robot-logikát azért,
hogy régi pytest elvárások átmenjenek.

A telepítő először stagingben építi fel az új suite-ot. Csak akkor ír a repóba, ha:
- a production/non-test source nem hivatkozik a törlendő pytest-infrastruktúrára;
- 80–140 tesztesetből álló, rétegenként is budgeten belüli suite építhető;
- legfeljebb 20 test_*.py fájl kell;
- a staging suite 0 FAIL;
- nincs elrejtett magas szintű AssertionError / safety-contract hiba.

Amit eltávolít
--------------
- pytest_profiles.py (ha nincs production hivatkozása)
- v3_unit_config.json
- tests/v3_config_fixtures.py reflection/config-copy rendszer
- privát mezőket, constructor signature-t, régi migrációt vagy exact tuningértéket védő tesztek

Amit létrehoz
-------------
tests/core/
tests/feature/
tests/deep/
tests/rig.py                    production ConfigResolverből olvas
tests/suite_manifest.json       pontos kiválasztási/budget riport
v3/test_runner.py
docs/PYTEST_POLICY.md

Telepítés
---------
cd /home/alba/project_r2b4/integralando
unzip r2b4_pytest_refactor_v2_20260925.zip
cd r2b4_pytest_refactor_v2_20260925

# Kötelező első lépés: teljes source-first dry-run, nem ír semmit
python3 installer.py --check

# Csak ha CHECK PASS
python3 installer.py

Használat
---------
./r test
./r test follow
./r test full

További fókusz módok:
./r test roomcruise
./r test localization
./r test perception
./r test motion
./r test async
./r test process
./r test replay

Biztonság
---------
A telepítő timestampes backupot készít .upgrade_backups/ alatt és post-install validáció
hiba esetén automatikusan visszaállítja az eredeti tests/ fát és a módosított fájlokat.
Production control/motor/config algoritmust nem módosít.

FONTOS V2 PONTOSITAS
--------------------
A v3/pytest_profiles.py NEM torlodik. A jelenlegi source-ban ezt a Test Hub,
host_cli, interface_cli es launcher_cli is hasznalja, tehat ez mar kozos
diagnosztikai/CLI profil-registry, nem egyszeru pytest-segedfajl. A pytest
ritkitashoz nem kell hozzanyulni.

Tovabbra is kikerul:
- v3_unit_config.json
- tests/v3_config_fixtures.py / configured() reflection
- regi implementacio-reszlet tesztek
- a nagy, ~1200 elemu tesztfa helyett a kuralt 80-140-es suite
