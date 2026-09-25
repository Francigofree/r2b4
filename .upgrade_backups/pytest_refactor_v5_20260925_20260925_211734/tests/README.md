# R2B4 pytest rendszer

A pytest fejlesztési bizonyíték, nem production authority. A robot authorityja továbbra is a V3 contractok és a canonical source; capture/replay/Test Hub pedig offline diagnosztikai capability.

## Profilok

A profilok egyetlen forrása: `v3/pytest_profiles.py`. Ugyanezt használja a pytest marker-kiosztás és a Test Hub pytest runner.

| Profil | Mire való |
|---|---|
| `gate` | leggyorsabb, nagy jelértékű első ellenőrzés |
| `contract` | V3 authority, layer-határok, isolation, safety contract |
| `async` | async edge, completion/input closure, process, multirate |
| `runtime` | TickEngine, composition, resident/process runtime |
| `replay` | capture, MCAP, determinisztikus replay/evidence |
| `control` | mission → planner → motion → safety → motor |
| `perception` | szenzorok, localization, L4/world-model, perception |
| `testhub` | Test Hub és portable evidence saját tesztjei |
| `voice` | ember–robot kommunikáció |
| `providers` | külső AI/provider adapterek; nem robot-core gate |
| `full` | teljes regresszió |

## Használat

A célzott validáció az alapértelmezett. Test Hubon keresztül például:

```bash
python -m v3.test_hub test --scope gate
python -m v3.test_hub test --scope async
python -m v3.test_hub test --scope replay
python -m v3.test_hub test --scope full
```

Nyers pytestnél a központilag kiosztott markerek is használhatók:

```bash
python -m pytest -q -m gate
python -m pytest -q -m async_boundary
python -m pytest -q -m "contract and not replay"
```

## Szabályok

A profil nem helyettesíti a bizonyíték típusát. Async/process módosításnál a célzott pytest mellett szükség szerint replay kell; timing/GIL javításnál mérési bizonyíték is kell. Fizikai robotmozgást pytest nem indíthat automatikusan. A `full` regresszió közös contract, TickEngine/execution boundary, composition root, aktív config, L12/motor-edge vagy több réteg érintésekor indokolt.

A tesztfájlok most szándékosan maradnak a lapos `tests/` könyvtárban. A profilrendszer leválasztja a tesztek jelentését a fájlhelyről, így a Test Hub integráció közben nem kell egyszerre több tucat importot és project-root számítást eltörni.
