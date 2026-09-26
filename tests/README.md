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
./r test follow
./r test roomcruise
./r test localization
./r test perception
./r test motion
./r test async
./r test process
./r test replay
./r test full
./r test list
```

A `./r test` CORE-t futtat. A fókuszált módok a megfelelő FEATURE vagy DEEP szeletet futtatják. A `./r test full` a teljes CORE + FEATURE + DEEP suite.

Nyers pytest továbbra is elérhető:

```bash
./r pytest -q tests/core
./r pytest -q tests/feature
./r pytest -q tests/deep
python -m pytest -q
```

## Szabályok

A célzott pytest nem helyettesíti a szükséges evidence-t. Async/process módosításnál szükség szerint replay kell; timing/GIL javításnál mérési bizonyíték is kell. Fizikai robotmozgást pytest nem indíthat automatikusan. A teljes regresszió közös contract, TickEngine/execution boundary, composition root, aktív config, L12/motor-edge vagy több réteg érintésekor indokolt.

A tesztfájlok canonical helye `tests/core/`, `tests/feature/` és `tests/deep/`. A teljes suite célmérete 80–140 collected case, hard cap 150.
