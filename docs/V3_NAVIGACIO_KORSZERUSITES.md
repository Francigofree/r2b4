# V3 helyi navigáció: célzott korszerűsítés

2026-10-14. A réteg- és async authority változatlan; a normatív szabályok a
`STRUKTURALIS_RETEGEK_V3.md` és `ASZINKRON_RUNTIME_CONTRACT_V3.md` dokumentumokban
maradnak. Ez a végrehajtott algoritmus-refaktor és a további robotikai munka terve.

## Kiindulás és megvalósított kör

Az aktív `conf/vezerles.json` production útja lezárt sensor/completion inputból
dolgozik. L4 helyi odometria-frame térképet, temporal occupancyt és bounded
structural memoryt tart; L6 a missionből lokális célt és pályajelölteket választ.
A visszatérő teljes rolloutot külön planner processz számolja. L7 választja a
mozgást; L8–L12 realizálja, korlátozza és véglegesíti. A felső logika canonical
commandon ad célt/preferenciát, a Public World completed statusból tanul.

| Probléma | Végrehajtott változás | Megőrzött szemantika |
| --- | --- | --- |
| Ugyanaz az új LiDAR scan két L4 ágban ismételten parse-olódott | Egyszeri kiválasztás/validáció; a robot-relative és pose-aligned ág közös validált pontokat használ | Measurement time, sequence, LOST alatti független robot-relative lineage, pending retry és checkpoint |
| Azonos megvalósítható sebességek többször kaptak teljes rolloutot | Kérésen belüli pontos sebesség-újrahasználat, közös direct/process pure evaluator | Minden normal/escape ID és elutasítási evidence, score, minták, L7 választási bemenet |
| Fiatal régi terv új akadály vagy nagyobb uncertainty mellett tovább élhetett | Új geometry/revision, track-változás vagy növekvő local sigma esetén egyetlen friss bounded manőver ellenőrzése | Régi guidance élettartama nem hosszabbodik; sikertelen újraigazolás nulla mozgású mission-preserving HOLD |
| A status nem mutatta a meglévő guidance/célválasztási lineage-et | Kompakt célválasztási ok/idő és MotionValidity; navigation/world DIAG scalar field coverage | Nincs raw térkép vagy rollout a statusban; observation passzív marad |

A revalidation nem teljes keresés és nem NO_PATH bizonyítás. Egy elutasított
javaslat alatt a worker tovább kereshet alternatívát; a felső mission ezért
`IDLE / PLANNER_GEOMETRY_HOLD` állapotot lát. L7 eközben visszavonja a korábbi
objective-et. Azonos geometria és nem növekvő uncertainty mellett a jelöltcsalád
és eredeti guidance-expiry tovább használható. A friss single-proposal döntés
változó geometria mellett szűkebb választékot ad L7-nek; ezt a korlátot a
folytonossági és escape-scenáriók ellenőrzik.

## Felső logikák és AMR működés

L4 operational worldje és a felső Public World külön szerepet tart meg. A felső
LocalTaskPlanner nem ír costmapet, coverage-et vagy navigation state-et; a cél és
RoomCruise preferenciák a CommandGatewayen át érkeznek. A lokális manőver,
feasibility, waypoint-handoff és progress L6 tulajdona marad. Mission identity,
odom-generation, target identity és measurement-lineage nem változhat status
polling, worker replacement vagy prediction miatt.

A változtatás a meglévő lokális AMR-végrehajtást korszerűsíti. Globális topológiai
útvonaltervező, map persistence, dinamikus akadályok időfüggő predikciója és teljes
AMR-rendszerszintű megfelelés nem következik az offline regresszióból.

## Következő konkrét robotikai lépések

1. **Tartalékból való recovery.** A jelenlegi induló safety-margin átfedés továbbra
   is fail-closed. Az irányfüggetlen `LocalMotionProof` közeli akadálynál nem tudja
   igazolni a nulláról induló hátrameneti gyorsulási rámpát. A legszűkebb teljes
   megoldás az existing escape családhoz kötött, kompakt irányfüggő bizonyítás,
   amelyet a tényleges L8/L9 kimenet fogyaszt. Csak nem átfedő kibővített testből,
   friss geometriával, a teljes átmenet szabad útját és a tartalék visszanyerését
   igazolva engedhető. Hátsó akadály, romló uncertainty, stale vagy megszakadt
   frame esetén STOP. E közös motion boundary változásához teljes release,
   direct/process-egyezés, native replay és encode/decode evidence szükséges.
2. **Megelőző hangolás.** Igazolt capture alapján együtt mérendő a rollout
   horizon, friss terv age, helyi cél handoff, kerékminimum, fékezési út és
   speed-clearance envelope. A margin csökkentése vagy a motion minimum
   átállítása nem helyettesíti a recovery bizonyítást. Aktív tuningérték most nem
   változott.
3. **Minőségmérés.** Azonos pályákon előtte/utána: befejezési arány, progress nélküli
   idő, goal-selection gyakoriság/ok, reverse/pivot váltások, L4/L6 thread CPU,
   planner process CPU, tick wall p95/p99, completion age és L12 STOP-ok. A
   wall–CPU különbségből önmagában nem állítható GIL-gyökérok.

## Evidence és elfogadás

A normal/escape családok törlése helyett a számítás osztozik: 54 normal ID mögött
40, 108 normal+escape ID mögött 45 különböző geometriai számítás volt a célzott
szintetikus próbákban. A request-local cache nem él át új requestet; más pose,
map, coverage vagy uncertainty új számítást kap.

Helyi offline CPU-mérésben, azonos inputokkal:

| Mérési scope | Előtte → utána | Eredmény |
| --- | --- | --- |
| L4: 96 pontos, 5 Hz scan / 50 Hz control, 7 kör | 352,82 → 320,49 µs/tick | 9,16% kevesebb process CPU |
| L6: sűrű szoba, 25 rollout | 55,59 → 43,46 ms/request | 21,8% kevesebb process CPU |
| L6: escape, 25 rollout | 14,02 → 10,28 ms/request | 26,7% kevesebb process CPU |
| L6: induló margin HOLD, 25 rollout | 11,97 → 9,25 ms/request | 22,8% kevesebb process CPU |

4 800 L4 tick kimenete, három sűrű rollout-scenario és 80 változatos rollout
kimenete egyezett a módosítás előtti source-szal. Ezek réteg-/compute-mérések,
nem teljes robot CPU- vagy live jitter-bizonyítások. A fiatal terv friss
geometriai ellenőrzése szándékosan hozzáad egy bounded ellenőrzést a controlhoz;
annak költsége külön mérendő a worker megtakarításától.

Öt, 100 tickes, késleltetett completionnel futó szintetikus control-körben a
változatlan geometria median thread CPU-ideje 0,110 → 0,110 ms volt. Új, 10 Hz-es
térképrevíziókkal ez 0,119 → 0,723 ms lett: a régi terv új geometriával való
ellenőrzése többe kerül, mint a korábbi ellenőrizetlen cache-olvasás. A bounded
reverse/pivot újraigazolás kis geometriai próbában 0,177/0,148 ms volt. Ezek nem
worst-case vagy hard realtime garanciák; a control költségét és a planner
megtakarítását külön kell látni.

Az új lineage mezők a sekély behavior capture → MCAP → explicit EVI compiler →
DIAG útvonalon is megmaradnak. Az eredeti geometry-captured idő, revision,
source-sequence, goal-selected idő és guidance-expiry együtt vizsgálandó.

```bash
./r test
./r test release
./r test tests/packs/feature/test_v3_lidar_world_model.py
./r test tests/packs/feature/test_v3_rollout_reuse.py
./r test tests/packs/feature/test_v3_cached_geometry.py
./r test tests/packs/feature/test_tools_diag_evidence.py
python3 -m v3.replay replay capture.mcap --output /tmp/v3-navigation-replay.json
./r evi capture.mcap --output /tmp/v3-navigation.evidence --workers 1
./r evi query /tmp/v3-navigation.evidence --field 'expected.layers.L6.motion_validity.*' --limit 20
./r diag navigation /tmp/v3-navigation.evidence --json --no-save
./r diag world_model /tmp/v3-navigation.evidence --json --no-save
```

Mintavett behavior capture nem exact replay evidence; ehhez teljes lezárt input,
completion-láthatóság és megfelelő prefix/checkpoint kell. A megváltozott
geometry-elfogadási döntést érintő történeti capture-nél eltérés várható a régi
kódtól; az algoritmus-újrahasználat változatlan compute-szemantikája ettől külön
ellenőrizhető. Live mozgás vagy fizikai recovery-minőség ezen refaktor offline
validációjában nincs bizonyítva.

A végleges source-on a `./r test release` 23 esetben sikeres volt, a robot-gate-et
is beleértve. Célzott evidence: 37 L4-es eset; 13 rollout-reuse eset; 18 cache/
HOLD/felső-logikai eset; 21 status/MCAP/EVI/DIAG eset; további native L4 checkpoint
replay. A 80 tickes szintetikus, hardware nélküli MCAP teljes, raw evidence-je
hiánytalan, a teljes és checkpointból induló native replay egyaránt `MATCH`.
Az explicit EVI compiler 99 message-et exportált `COMPLETE` státusszal;
source-ellenőrzése `PASS`, a navigation és world_model DIAG alkalmazható, teljes
required coverage-dzsel. Fizikai robotmozgás nem történt.

A helyi mérési/evidence artifactok:

- `/tmp/r2b4-l4-scan-bench-UhJuZOYI/report.md`
- `/tmp/r2b4-l6-rollout-mRO9Eq7H/before.json`, `after.json`
- `/tmp/r2b4-nav-geometry-2g35pmfw/benchmark.json`
- `/tmp/r2b4-nav-geometry-2g35pmfw/navigation_complete.mcap`
- `/tmp/r2b4-nav-geometry-2g35pmfw/replay_full.json`, `replay_checkpoint.json`
- `/tmp/r2b4-nav-geometry-2g35pmfw/navigation.evidence/`
- `/tmp/r2b4-nav-geometry-2g35pmfw/diag_navigation.json`, `diag_world_model.json`

Ezek fejlesztői diagnosztikai outputok; a `/tmp` tartalma nem tartós regressziós
követelmény vagy új workflow authority.
