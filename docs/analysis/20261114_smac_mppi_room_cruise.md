# Smac/MPPI Room Cruise — 2026-11-14

A CPU- és Smac/MPPI-integráció elkészült, de a tartós önálló cirkálás célja
**nem teljesült**. Egy engedélyezett `./r rc 60 c full` élő futás történt.
A Nav2 lifecycle-readiness elvesztése és a timing-audit hibája után nem indult
újabb élő futás. A későbbi javítások motormentes és szoftveres validációt kaptak.

Az eredeti evidence a
[v3_20261114_211600_6819_capture.mcap](../../runtime/captures/v3_20261114_211600_6819_capture.mcap).
SHA-256: `23076579a46db690c319370b7c065fdb3b27cc46ede5e59f4852c83a437262e4`.
Az [elemzés és logkivágatok](../../tmp/nav2_mppi_20261114_sl3i878z/),
a [számított mérőszámok](../../tmp/nav2_mppi_20261114_sl3i878z/run1_summary.json),
az [útvonal és navigációs idővonal](../../tmp/nav2_mppi_20261114_sl3i878z/run1_navigation.png)
és a [verifikált EVI-bundle](../../tmp/nav2_mppi_20261114_sl3i878z/run1.evidence/manifest.json)
ugyanehhez az egy futáshoz tartoznak.

Változások:

- A SLAM/Nav2 supervisorok a canonical affinity-policyt követik. Az aktív
  `runtime_affinity.enabled=false` mellett nincs egymagos szűkítés; a vizsgált
  élő ROS-szálak maszkja kivétel nélkül `0–3` volt. Az engedélyezett policy
  konfigurált szerepmaszkjai, a nice 10 és a bounded native threadlimitek megmaradtak.
- Navfn helyett Jazzy SmacPlanner2D működik: ismert szabad tér, 150 ms-os
  keresési keret, bounded iterációk, meglévő 2 Hz-es BT újratervezés.
- RPP helyett Jazzy MPPI működik a meglévő native kerékadapterben: 20 Hz,
  400 rollout, egy iteráció, 32 × 50 ms-os horizont, visualization nélkül.
  A fizikai kerékkorlátokra vetített parancs pontos körívét a Nav2 costmapjével
  újra ellenőrzi; az ismeretlen/térképen kívüli út nem fogadható el.
  Az inflation-sugár 0,55 m; a robot külső sugara 0,456 m.
- A launcher MPPI-paramétereket származtat a resolved fizikai konfigurációból.
  A mission-sebességkorlát nem emelheti meg a konfigurált normál sebességet.
  A V3 50 Hz-es célperiódusa, L9 fizikai gyorsulása, freshness és final safety
  authority változatlan. Az optimizer gyorsabb javaslati modellje és a tényleges
  fizikai gyorsulás közti eltérés továbbra is hangolási kockázat.
- Az izolált ROS-prefixhez Smac/MPPI 1.3.13 került, a meglévő csomagok
  frissítése nélkül. A deklaráció és az explicit URL/MD5 lock frissült.
- Az 5 Hz-es Nav2-observation külön tárolja a vetítés előtti MPPI-javaslatot
  eredeti source-idővel és számlálóval. A meglévő controller-, odometria-,
  coverage-, recovery-, lifecycle- és runtime-timing evidence megmaradt.
- Az élő futás után javult a host eredményjelentése: átmeneti HOLD közben
  folytatható a mission, de a határidőnél nem aktív Room Cruise navigáció
  `REQUESTED_EXECUTION_UNPROVEN` hibát adhat. A vizsgált régi viselkedés ezt
  tévesen sikeres időtartamnak minősítette.
- Szintén az élő futás után a ROS bridge egyszer regisztrálja node-ját az
  executorhoz. Elmarad a minden spin-turnben végzett add/remove és wait-set
  újraépítés. Ez nem lifecycle-timeout emelés vagy automatikus restart.

| Az egy élő futás ténye | Mért eredmény |
|---|---|
| Capture | 5 Hz, 360 mintavett tick; 71,76 s teljes, startupot/shutdownt is tartalmazó ablak |
| Capture-integritás | CRC és requested-scope integrity PASS; EVI 3408 üzenet, COMPLETE |
| Lokalizált haladás | 3,919 m; footprint-rács 0,16 → 1,63 m², növekmény 1,47 m² |
| Mission alatti L6-minták | 87 ACTIVE; 18 PENDING; 8 STALE; 1 SUPERSEDED; 200 UNAVAILABLE |
| Mintákból becsült idő | ACTIVE 17,39 s; UNAVAILABLE 39,80 s; nem teljes 50 Hz-es állapottörténet |
| Célok | Egy frontier-cél; utána egy COVERAGE-cél, elért coverage-cél: 0 |
| Recovery | Egy Wait recovery sikeresen lezárult, utána újra mérhető odometriai haladás következett |
| MPPI | 345 nyers javaslat; aktív, friss controller-ablakok medián source-rátája 19,47 Hz |
| Lifecycle-hiba | `NAV2_LIFECYCLE_STATUS_STALE`; last-response age 1,658 s; legnagyobb poll-köz 1,235 s |
| Első L6 UNAVAILABLE | Tick 1559, mission-mintaidőhöz képest +22,800 s |
| Kiesés alatti actuation | A 200 UNAVAILABLE mintában nincs nem nulla final motor-output |
| Safety | Mintavett production FAULT nincs; az ACTIVE missionben minden mintavett safety ALLOW; induláskor kilenc DEGRADED miatti STOP-minta |
| Mintavett LiDAR-body clearance | Minimum 0,192 m; ez nem teljes kontaktusmentességi bizonyíték |
| V3 teljes futású timing | 3575 normál tick; átlagperiódus 20,078 ms, p95 22,4 ms, p99 26,0 ms, max 51,199 ms |
| Timing-kimaradás | 49 periódus >25 ms, három >40 ms; hat work-idő >20 ms; timing-audit FAIL |
| Control / work p99 | 10,6 / 12,4 ms; pipeline CPU p99 3,4 ms, pipeline wall p99 9,7 ms |
| Host a navigációs időablakban | CPU átlag 70,94%, max 78,85%; hőmérséklet max 54,55 °C |
| Runtime leállítás | SHUTDOWN_SAFE_LOW; ez külön tény a Nav2 navigáció hibájától |

A legkorábbi bizonyított terminal Nav2-hiba a bridge lejárt lifecycle-readiness
állapota. A manager a supervisor későbbi leállításáig aktív maradt; előzetes
bond-loss nem szerepel a logban. A két query-timeout még startup alatt történt,
a terminal hibáig nem nőtt tovább. A nagy bridge poll-köz és a TF/szenzoradatok
együttes öregedése bizonyított, de az elakadást okozó konkrét callback/native
várakozás nincs az élő capture-ben azonosítva. CPU/GIL/transport root cause
ebből nem állítható. A control wall/CPU különbsége sem választja szét ezeket.

Az eredeti 60 s-os parancs exit code 0-val és runtime PASS státusszal zárult,
miközben az L6 már tartósan UNAVAILABLE volt. Emiatt ez **nem sikeres 60 s-os
navigációs bizonyíték**. A későbbi host-javítás pontosan ezt a hamis sikerutat
zárja le; annak élő újraigazolása nem történt meg.

Validáció:

- Végleges `./r test release`: **24 passed**.
- Nav2 navigation/lifecycle/frontier/coverage célzott tesztek: **82 passed**.
- Behavior és runtime-navigation lifecycle célzott tesztek: **45 passed**;
  külön bizonyítják a HOLD → resume és a kiesés melletti befejezés hibáját.
- Native C++ kerékprojekció, valódi costmap-collision/unknown/map-exit és
  változatlan fizikai L9-ramp: **3 passed**.
- CPU-policy és 5 Hz-es diagnosztika: **16 passed**.
- Motormentes frontier-próba a végleges executorral: **PASS**, pozitív
  javaslat, canonical cancel, zero-motion cancel és heartbeat-expiry.
- Motormentes, ismert szobás COVERAGE-próba a végleges executorral: **PASS**,
  25 s vizsgált cruise-idő, két gyermekcél, egy sikeres cél, friss lifecycle.
  Azonos profilozott próbákban a függvényhívások száma 33,80 M → 24,53 M,
  a profilozott idő 18,49 → 16,02 s lett. Ez motormentes fejlesztői evidence,
  nem bizonyítja az élő hiba megszűnését vagy a V3 jitter javulását.
- Végleges motormentes pontos NAVIGATE-próba: **FAILED**, 86,38 s után
  Nav2 terminal FAILED. A kis cél-/yaw-tolerancia és a kerékholtsáv mellett
  elakadás marad; az obstacle és későbbi fázisokig ez a próba nem jutott el.
- A canonical Replayer ténylegesen futott: `CAPTURE_INCOMPLETE: replay core
  is incomplete`. Az 5 Hz-es behavior-capture szándékosan nem tartalmaz teljes
  TickInputs/checkpoint evidence-et, így élő MATCH nincs. Ez nem MISMATCH.
  A release tesztek külön synthetic persisted Nav2-replay evidence-et adnak.

Fennmaradó problémák és korlátok:

1. Az élő lifecycle-poll elakadása nincs konkrét upstream műveletre visszavezetve;
   az executor-optimalizálás élő hatása nincs bizonyítva.
2. A V3 50 Hz-es célfrekvencia megmaradt, de a szigorú jitter-audit nem teljesült;
   teljesítménybeli nem-regressziót kontrollált élő A/B nélkül nem állítunk.
3. A pontos cél/yaw konvergenciája a végleges MPPI-beállítással sem megbízható.
   A nyers javaslat és a kerékprojekció eltérése további adapterhangolást igényel.
4. A live futásban megjelent a frontier → coverage váltás körüli BT halt-result
   hiba és később TF-too-old; utóbbi a lifecycle-poll kihagyásával együtt látszik,
   önálló upstream oka nem bizonyított. A frontier új céljainak visszautasítása
   részben a meglévő aktív cél megtartásának szándékos policyja.
5. A Smac2D configure során fennmaradt egy inflation-optimalizálási ERROR-log;
   a plugin ettől aktiválódott. Az MPPI konkrét külsősugár/inflation eltérése
   a 0,55 m-re növelt sugárral megszűnt; a Smac-log nem lett elrejtve.
6. A szoba tartós, emberi segítség nélküli bejárása, ismételt akadály-/sarok-
   helyzetek önálló megoldása és teljes fizikai kontaktusmentesség nincs bizonyítva.

Az integráció konfigurációs indoklását és a hivatalos Jazzy-forráshivatkozásokat
a [Nav2 runtime-leírás](../NAV2_PLANNER.md) tartalmazza. Architekturális contract
nem változott; új command-, safety- vagy motor-authority nem került a rendszerbe.
