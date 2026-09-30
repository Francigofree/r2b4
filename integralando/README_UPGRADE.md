# R2B4 Test Hub finalization upgrade

Source-first baseline: `Francigofree/r2b4` main @ `90417581bea1ba64db761a4889d85fb409756c6f` (2026-09-30).

## Cél

A csomag lezárja a Test Hub migrációt:

- `v3.test_hub` lesz az egyetlen publikus Python entrypoint.
- `v3/test_hub_next.py` -> belső `v3/test_hub_app.py`.
- `v3/test_hub_v2.py` -> belső `v3/test_hub_backend.py`.
- A régi, JSON-capture alapú Test Hub facade/CLI eltűnik; a canonical replay core nem törlődik.
- Az aktív források és dokumentációk verziózott Test Hub modulhivatkozásai kanonikus névre állnak át.
- A történeti `runtime/` és `.upgrade_backups/` tartalom érintetlen marad.
- Új `docs/TEST_HUB.md` rögzíti az authority-, evidence-, anti-drift- és bővítési szerződést.
- Új feature contract teszt őrzi, hogy a legacy modulok ne nőjenek vissza.

## Telepítés

A repo gyökeréből:

```bash
python3 /path/to/r2b4_testhub_finalization_upgrade_20260930/install.py --root .
```

Előzetes ellenőrzés módosítás nélkül:

```bash
python3 install.py --root /home/alba/project_r2b4 --dry-run
```

A telepítő alapból **engedékeny HEAD-eltérésnél**: figyelmeztet, majd strukturális preflight alapján folytatja. Pontosan a source-first baseline commit kikényszerítéséhez:

```bash
python3 install.py --root /home/alba/project_r2b4 --strict-head
```

## Biztonság / rollback

A telepítő minden érintett meglévő fájlt a repo alatti
`.upgrade_backups/testhub_finalization_<timestamp>/` könyvtárba ment, és manifestet ír.
Kötelező smoke validation hibánál automatikusan rollbackel.

Kézi rollback az utolsó upgrade-ről:

```bash
python3 install.py --root /home/alba/project_r2b4 --rollback-last
```

## Validáció

A telepítő kötelezően ellenőrzi:

- módosított Python források szintaxisát;
- a legacy aktív fájlok hiányát;
- aktív forrás/doksi legacy modulreferenciáit;
- `python3 -m v3.test_hub --help` működését;
- `run` és `query` parancs láthatóságát.

Ha `pytest` importálható, célzottan lefuttatja a finalization contract tesztet is. A pytest hiánya önmagában nem blokkolja a telepítést; a kötelező strukturális és CLI smoke check igen.
