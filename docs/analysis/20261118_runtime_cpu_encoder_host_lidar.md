# Élő CPU-refaktor: encoder, host és LiDAR

2026-11-18. Az induló source `e835030`; munka közben a felhasználó
`dc040e2` commitja már tartalmazta az encoder- és hostmódosításokat is.
Agent commit/push nem történt. A vizsgálat és a további módosítás közvetlenül
az aktuális munkafában készült.

## Élő evidence és a megmaradt hiba

| CPU2 ablak | Host % | Magátlagok % | Encoder, egy mag % | Nav2 bridge, egy mag % |
| --- | ---: | --- | ---: | ---: |
| 19:32:33 | 77,63 | 76,0 / 80,8 / 72,6 / 81,2 | 35,50 | 35,30 |
| 19:53:13 | 73,33 | 74,2 / 75,5 / 72,6 / 71,0 | 22,90 | 39,00 |
| 19:55:50 | 76,57 | 75,9 / 77,5 / 79,5 / 73,4 | 21,72 | 36,60 |

Forrás: a megfelelő `runtime/cpu/cpu2_20261118_*_summary.json` és NDJSON.
Az encoder owner a production wiring szerint a `pipe_handle=35` child;
nem a LiDAR owner. A későbbi két encoder-processz mért CPU-ja 35–39%-kal
kisebb. Ez eltérő élő mozgású ablakok összehasonlítása, nem kontrollált A/B.
A host 50%-os aktív célja nincs teljesítve/bizonyítva.

A magas terhelés önmagában nem bizonyítja a leállás gyökerét. A tartós
STOP-hoz vezető konkrét út:

* `v3_20261118_193159_20397_capture.mcap`: 2892. tick L12
  `CRITICAL_DEVICE_DEGRADED`; a következő teljes mintában encoder
  `ENCODER_RESTARTING`. A runtime logban
  `ENCODER_NOTIFICATION_DELIVERY_STALLED:right`.
* `v3_20261118_195247_23956_capture.mcap`: 1577. tick kritikus STOP;
  1582. mintában encoder restart. Log: ugyanilyen bal oldali delivery-hiba.
* `v3_20261118_195354_25148_capture.mcap`: encoder restart nem látható;
  a végén canonical cancel/shutdown.
* `v3_20261118_195505_26868_capture.mcap`: tartós kritikus STOP 2967. tick,
  2970. teljes mintában encoder restart. Log:
  `last_callback_ns=4909820867957`, `mismatch_since_ns=4909999781033`,
  `current_b=0`, `delivered_b=1`. Egy külön, egytickes kritikus STOP már
  1567-nél is van, de annak device-health oka az 5 Hz-es mintából nem
  állapítható meg; 1568-nál már ismét engedélyezett a mozgás.

A watchdog nem változott. A GPIO-szint és a kézbesített callback
ellentmondása utáni latch akkor is szükséges, ha a szint később visszaáll.
Sem a CPU-okozat, sem az értesítésvesztés fizikai/native-library oka nem
bizonyított. Scheduler runqueue-várakozás nem azonos GIL-várakozással.

A 19:55-ös capture explicit EVI compile: COMPLETE, source integrity COMPLETE,
3886 message; külön source-os verify PASS, 0 quarantined.
`tmp/cpu_20261118_live/new_evidence`, `new_evi_verify.json`.
Canonical replay: BLOCKED / `CAPTURE_INCOMPLETE: replay core is incomplete`.
Az 5 Hz-es system-behavior capture nem teljes replay-input.

## Megvalósított szűk változtatások

1. `gpio_counter.py`: a szigorúan növekvő B-historyban a fizikai A-idő
   szomszédait a legújabb végétől keressük. Az egyenlő idő, guard-prioritás,
   debounce, irányigazolás, számlálók és watchdog azonosak. Nincs új cache,
   I/O vagy callback/thread. A 60 000 callbackes, backendolvasásokat is
   tartalmazó A–B–B–A mérés CPU-ja 1,118 → 0,532 s; minden 500 reading és
   a teljes végső history azonos. Ez 52,4% az izolált munkán, nem a hoston.
2. `RobotInterface._resolve`: az egyértelmű owner kiválasztása után a három
   érintett adapter célzott read-descriptort adhat. Nincs katalógus-cache;
   V3-read esetén továbbra is friss `live_runtime_status()` dönti el az
   availabilityt, és a tényleges read ismét ellenőrzi azt. A régi adapterek,
   actionök és dinamikus skill-readek a teljes katalógusútvonalon maradnak.
   Public host-state olvasáskor elmarad a független skill-discovery,
   capability-RPC és vision-probe; operator readnél a fölös második status.
   Az ambiguity/kind/availability hibák és STOP-admission megmaradnak.
3. `rplidar_c1.py`: a státusz a kiválasztott/megkísérelt portot jelenti;
   nem végez minden health-pollnál `/dev/serial/by-id` globot. Connect és
   reconnect továbbra is újra választ. Az acquisition a dekódolt mezőkből
   egyszer épít validált, receipt-idővel ellátott `RplidarPoint`-ot; a
   standalone public decoder változatlan eredményt ad.
4. `native_lidar_port.py`, `process_lidar_port.py`: az 5 ms-os producer
   explicit revision-filterrel kér új scan-t. Azonos revision esetén nincs
   új health-read, snapshotmásolás és teljes pontlista-validáció. A
   hagyományos argumentum nélküli read friss health/observation-időt ad.
   Az új scan, pending geometry, matcher-only publication, heartbeat és
   teljes latest-state transport megmarad. Source-időt nem újítunk meg.

Az érintett teszt-fixture-ök követik az opcionális revision-paramétert.
Két már elavult fixture-feltételt is helyesbítettünk: a fizikai body-clearance
nem az optikai sugártáv; a szintetikus scan freshness-vizsgálatához ugyanaz
az explicit óra kell. A retime-helper a navigation input contextjét is viszi.

## Izolált mérés és profil

`tmp/cpu_20261118_live/` tartalmazza a reprodukciós programokat és eredményeket.

* `encoder_bench.py`, `encoder_comparison.json`: a fenti teljes callback/
  backend-egyezés és CPU-arány, eltett eredeti kereséssel.
* `interface_bench.py`: az eredeti `_resolve` az **e835030** source-ból jön,
  nem a munka közben megváltozott HEAD-ből. Száz `behavior.state` read
  eredeti CPU-ja 3,47–3,92 s, célzott útja 0,00028–0,00030 s. Az RPC read
  itt konstans válaszú; a különbség a kivett discovery/probe munka,
  nem hálózati vagy teljes élő service-CPU mérés. Száz unavailable V3-read:
  1,20–1,43 → 0,64–0,72 s. Nem aktív mozgású host A/B.
* `lidar_bench.py`: 2000 producer-poll, 100 új, 600 pontos scan;
  CPU 0,1474 → 0,0111 s. Az összes fogyasztott snapshot, health,
  measurement- és observation-idő azonos. Ez csak e poll munkájának 92,5%-a.
* `lidar_ingest_bench.py`: 120 000 packet, fragmentált serial byte-ok,
  hibás prefixek; CPU-átlag 0,678 → 0,493 s (27,3%). Az összes scan,
  point receipt, framing-buffer és invalid counter azonos.
* `idle_runtime_profile.raw`, `idle_nav_profile.raw`: motor nélküli
  production runtime, 25 Hz-es mintavétel, 12 s. A Nav2 bridge Python
  wait-set overheadje és a LiDAR-poll fenti ismétlései közvetlenül láthatók.

A 3185-ös public daemon még a régi importokat tartotta. Inaktív behavior/
brain ellenőrzése és canonical STOP után újraindult, az új PID 30230,
health AVAILABLE. A következő kliens így már az új host-kódot használja.

## Natív executor kísérlet: elutasítva

A telepített ROS Python 3.12 `rclpy 7.1.12` tartalmaz experimental natív
EventsExecutort. Az azonos endpoint-topológiájú üres A–B–B–A próba CPU-ja
4 s alatt 0,3374 → 0,0327 s lett. A teljes room/Nav2 szimuláció viszont
`SystemError: null argument to internal routine` hibával kilépett a
bridge `spin_once` hívásából, pozitív mozgási minta nélkül.

Ez a változat **nincs a production source-ban**. Az üres executor benchmark
nem bizonyít action-, callback-, burst-, cancel- vagy queue-ekvivalenciát.
Az upstream implementáció vizsgálatához használt elsődleges source:
https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/events_executor/events_executor.cpp
Ez nem az installed binary byte-azonosságának bizonyítéka. Nincs fallback-
scheduler, új framework vagy harmadik fél dependency-módosítás.
Kísérlet: `native_wait_bench.py`, `native_wait_results.json`,
`tmp/cpu_20261118/native_room/bridge.log`.

## Validáció és határok

* Encoder/backend + notification recovery: 37 passed.
* Public interface/runtime/observation: 52 passed.
* A végleges sensor-fájlokon: 73 passed.
* LiDAR transport/recovery/map vizsgálat: először 36 passed + 6 hibás
  fixture; azok javítása után mind a 6 érintett eset PASS.
* `./r test release`: 28 passed, a kis robot-contract gate-tel együtt.
* Végleges source, natív room/NAVIGATE: PASS / GOAL_COMPLETE, 3,56 cm célhiba,
  nulla ütközés, végső STOP, 423 tick replay MATCH.
  `tmp/cpu_20261118/final_room/`. A szimuláció nem a fizikai GPIO/serial
  acquisition költségét méri. Timing-nemregresszió ebből az egy futásból
  nincs bizonyítva (max tick wall 51,2 ms).
  A külön EVI compile COMPLETE / source integrity COMPLETE, 638 message;
  a source-hash-t is ellenőrző verify PASS, 0 quarantined:
  `tmp/cpu_20261118_live/final_room_evidence`, `final_room_evi_verify.json`.

Az agent 19:48-as canonical `rc 60 c full` próbája már az admissionnél
meghiúsult, fizikai mozgás nélkül: `roomcruise was not acknowledged`.
A capture-ben nincs ACTIVE command, a command log üres; a pontos host/
producer-indulási ok nincs bizonyítva. Az utólagos egy ns TTL-es STOP
fallback nem az elveszett ACTIVE command bizonyítéka. Nem változott TTL,
admission timeout vagy safety-gate, és nem történt automatikus mozgási retry.

A későbbi két agent hardveres mérés canonical runtime-start/STOP mellett,
álló robottal történt. A végső CPU2 ablak
`cpu2_20261118_201007_229177`: host 38,54%, V3 16,20%, Nav2 11,80%,
SLAM 6,62%, public daemon 2,80% egy-mag CPU; nincs throttling.
Az `idle_after_start.log`/`idle_after_stop.log` szerint finalizálás integrity
PASS és runtime PASS. Ez **nem** aktív Room Cruise eredmény, nem alkalmas
a 73–77%-os aktív ablakokkal közvetlen CPU A/B arány képzésére.

Nyitott maradt: aktív futás magonként kb. 50%-os terhelése és hosszú
mozgásminősége; az encoder notification-delivery kiesés fizikai/native oka;
a ROS executor natív alternatívájának production-kompatibilitása. A control
50 Hz, Nav2 controller, szenzorfrekvenciák és freshness/safety policy nem
változtak. A robotruntime leállt, motor-output nincs.
