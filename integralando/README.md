# R2B4 speed-aware route optimization upgrade

Source-first baseline: `Francigofree/r2b4` main commit
`aa882aa3a2202b665ae0be54858336e8cc1099c0`.

## Cél

Room Cruise / canonical local navigation nagy szobában tartson nagyobb átlagsebességet
anélkül, hogy rendszeresen ráfutna az L12 safety hard zónájára. A planner a safety
beavatkozása előtt válasszon tágasabb, simább vagy szükség esetén lassabb ívet.

## Mit módosít

- `v3/layers/l6_navigation.py`
  - sebességfüggő soft-clearance envelope;
  - konfigurálható barrier + nagysebességű clearance penalty;
  - EXPLORE-only adaptív 0.45–0.90 m lokális köztescél;
  - ugyanaz a scoring a live és a pure async rollout kernelben.
- `v3/layers/l7_motion_selection.py`
  - szélesebb score-hiszterézis;
  - continuity csak clearance-ekvivalens near-best pályák között.
- `conf/vezerles.json`
  - minden új paraméter production config authority;
  - 1.0 s / 10 lépés rollout;
  - új progress/clearance/smoothness/novelty súlyok.
- `v3/replay.py`
  - régi capture-ekhez explicit legacy migration, hogy az új policy ne írja át a
    történeti replay szemantikát.
- `tests/core/test_v3_route_optimization_config.py`
  - új config/scoring/continuity/replay regressziók.

## Biztonsági határok

Az upgrade **nem módosítja**:

- L12 LiDAR hard safety minimumot (`0.4 m`),
- L12 fail-closed működést,
- L9 platform sebesség-, gyorsulás- vagy curvature limiteket,
- wheel speed map / motor authority útvonalat.

## Alkalmazás

A repository gyökerében:

```bash
python3 /path/to/apply_route_optimization_upgrade.py --check-only
python3 /path/to/apply_route_optimization_upgrade.py
```

Az installer csak a fenti source-first HEAD-en fut alapértelmezetten. Ha a HEAD már
eltért, **ne** használd automatikusan az `--allow-head-mismatch` kapcsolót; előbb új
source-first ellenőrzés kell.

Sikeres alkalmazáskor az installer:

1. ellenőrzi a HEAD-et, dirty targeteket és source anchorokat;
2. backupot készít `.upgrade_backups/route_optimization_<timestamp>/` alatt;
3. végrehajtja a módosításokat;
4. `py_compile` validációt futtat;
5. lefuttatja a célzott config/planner/harmony/Room Cruise pytesteket;
6. bármely hiba esetén automatikusan visszaállítja a backupot.

## Rollback

```bash
python3 /path/to/apply_route_optimization_upgrade.py \
  --root /home/alba/project_r2b4 \
  --rollback /home/alba/project_r2b4/.upgrade_backups/route_optimization_YYYYMMDD_HHMMSS
```

## Ajánlott validáció telepítés után

```bash
./r test full
```

Ezután azonos szobában két Room Cruise futás javasolt: először rövid 15–20 s,
majd 40–60 s. A Test Hubban elsősorban ezeket érdemes összevetni:

- EXPLORE actual/allowed linear speed;
- selected minimum clearance eloszlás;
- selected candidate changes/s;
- angular acceleration / angular jerk;
- L12 STOP `LIDAR_CLEARANCE_LOW` előfordulás;
- `PLANNER_STALE_HOLD`, `NO_PROGRESS_VIABLE_TRAJECTORY`;
- planner/tick p95 és p99 timing.
