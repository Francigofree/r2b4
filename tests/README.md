# R2B4 pytest rendszer

A pytest fejlesztési bizonyíték, nem production authority. A robot authorityja továbbra is a V3 contractok és a canonical source; capture/replay/Test Hub pedig offline diagnosztikai capability.

## Rétegek

A canonical pytest policy: `docs/PYTEST_POLICY.md`. A tesztfájlok három rétegben vannak:

| Réteg | Mire való |
|---|---|
| `core` | safety, canonical authority, STOP/FAULT, TTL/freshness, runtime composition, basic control chain, import boundaries |
| `feature` | user-visible robot behavior: Follow, Room Cruise, localization, perception, motion |
| `deep` | process/worker failure, delayed async completion, replay/checkpoint, timestamp edges, fault injection |

A canonical futtató a `v3/test_runner.py`.

## Használat

A célzott validáció az alapértelmezett:

```bash
./r test
./r test core
./r test follow
./r test roomcruise
./r test localization
./r test perception
./r test motion
./r test async
./r test process
./r test replay
./r test evidence
./r test agent
./r test voice
./r test providers
./r test full
./r test endurance
./r test list
```

A `./r test` egy 20–30 esetes, hardvermentes, fail-fast CORE kapu. A `./r test core` futtatja a teljes CORE könyvtárat. A fókuszált módok explicit fájlokat vagy teszteseteket gyűjtenek. A `./r test full` a teljes, időben korlátozott CORE + FEATURE + DEEP regresszió; a tízperces szimuláció külön `endurance` kérésre fut. Ugyanannak a szimulációnak egy rövid, global-fix loss/recovery és checkpoint replay változata a teljes regresszió része.

Egy konkrét hiba, a kiválasztás és a cache-elt hibák a canonical launcherrel is vizsgálhatók:

```bash
./r test tests/feature/test_v3_motion_feedback_quality.py
./r test motion --collect-only
./r test motion --lf
```

A `--lf` javítás közbeni segítség; átvételhez a teljes érintett scope kell. A nem gyors kapus futások jelzik az öt leglassabb, 0,5 s feletti esetet. A shared fake-ek a `v3_test_fixtures.py` modulban vannak; a fake kamera nem foglalhatja le a valódi eszközzárat, és assertion-hibánál is le kell állnia.

Nyers pytest továbbra is elérhető:

```bash
./r pytest -q tests/core
./r pytest -q tests/feature
./r pytest -q tests/deep
python -m pytest -q
```

## Szabályok

A célzott pytest nem helyettesíti a szükséges evidence-t. Async/process módosításnál szükség szerint replay kell; timing/GIL javításnál mérési bizonyíték is kell. Fizikai robotmozgást pytest nem indíthat automatikusan. A teljes regresszió közös contract, TickEngine/execution boundary, composition root, aktív config, L12/motor-edge vagy több réteg érintésekor indokolt.

A tesztfájlok canonical helye `tests/core/`, `tests/feature/` és `tests/deep/`. A teljes fa hard capje 200 collected case, az endurance változatot is beleszámítva. A tesztkiválasztás authorityja a launcher és a policy; az esetszámot a collector számolja.
