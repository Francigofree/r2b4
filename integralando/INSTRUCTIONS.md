# R2B4 FACE_PERSON + L6 P0-A upgrade

## Telepítés

```bash
cd /home/alba/project_r2b4
./r2b4 stop

python3 /tmp/r2b4_faceperson_upgrade/apply_faceperson_upgrade.py \
  --check /home/alba/project_r2b4
```

Ha PASS:

```bash
python3 /tmp/r2b4_faceperson_upgrade/apply_faceperson_upgrade.py \
  --apply /home/alba/project_r2b4
```

Az installer backupot készít:

```text
runtime/upgrade_backups/faceperson_p0a_YYYYMMDD_HHMMSS/
```

## Kötelező tesztek

```bash
cd /home/alba/project_r2b4

python3 -m pytest -q \
  tests/test_v3_face_person.py \
  tests/test_v3_async_l6_planner.py

python3 -m pytest -q tests/test_v3_architecture_boundaries.py
python3 -m pytest -q
```

Csak teljes PASS után élő teszt.

## Raised-stand teszt

```bash
./r2b4 faceperson c full
```

Elvárt:
- ember balra -> bal fordulási kérés;
- ember jobbra -> jobb fordulási kérés;
- ember középen -> nulla mozgás;
- ember eltűnik -> nulla mozgás;
- nincs előre/hátra haladás.

Leállítás:

```bash
./r2b4 stop
```

## Padlóteszt

Tág, tiszta területen, ember kb. 1-2 m-re, kezdetben 30-60 fok oldalirányban:

```bash
./r2b4 faceperson c full
```

Elvárt:
- helyben fordul;
- `v=0`;
- beállás után leáll;
- 0.10 / 0.16 rad hiszterézis miatt nem remeg;
- target loss -> nulla mozgás;
- új person track -> reacquire.

FULL capture lezárás:

```bash
./r2b4 stop
./r2b4 shutdown
```

## P0-A ellenőrzés

A csomag nem írhatja felül más implementációval a reggeli P0-A-t. Ha az L6-ban
már van `_require_fresh_cached_plan`, de nem pontosan a reggeli változat, az installer
hibával megáll.

A megőrzött szabályok:
- monotonic production handoff: késő worker esetén friss cached plan maradhat;
- tick-gated legacy/replay: hard `ASYNC_L6_DEADLINE_MISSED`;
- 350 ms-nál öregebb cached plan: `ASYNC_L6_PLAN_STALE`.
