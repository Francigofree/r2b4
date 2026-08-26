# LIDAR adatút audit — 2026-07-22, történeti pre-fix bizonyíték

- Aktualis allapot: `RESOLVED / SUPERSEDED BY STABLE CONTRACT`
- Stabil matcher contract: `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1`
- Aktualis implementacios riport: `docs/SCAN_MATCHER_MODERNIZATION.md`
- Stabil architekturalis SSOT: `STRUKTURALIS_RETEGEK.md`, `D012`

Ez a dokumentum a javitas elotti hibaallapot bizonyitekait orzi. Jelen ideju
allitasai nem irhatjak felul az aktualis forraskodot, a protected baseline-t
vagy a fenti stabil szerzodest.

## Szerződés és hatókör

Az auditált lánc:

`driver/raw scan -> timestamp/ID -> LidarService -> matcher -> LidarOdometry -> EKF/map -> SafetySupervisor -> MotionExecutor/room-cruise/follow -> Test Hub validator`

Az observation-azonosítók nem felcserélhetők:

- `raw_scan_id`: a driver által lezárt egyedi fordulat;
- `matcher_result_id` / `candidate_id`: egy teljes matcher-futtatás eredménye,
  forrása pontosan egy `raw_scan_id`;
- `lidar_odometry_measurement_id`: a gate által elfogadott odometriai mérés,
  forrása pontosan egy candidate/matcher result.

Az ID-k között leszármazást kell őrizni. Freshness mindig a vizsgált
adat akvizíciós vagy eredményidejéből származik; publikálási/poll idő nem
tehet egy régi observationt újjá.

## 2026-07-23-i lezárás

Az audit 1–14. pontja lezart:

- a raw snapshot a parent driver workerben, a matcher IPC enqueue elott
  frissul; raw safety nem var matcherre;
- a matcher kulon `spawn` processzben fut, `1/1` bounded latest-only
  input/result queue-val;
- a snapshot immutable, raw/matcher/measurement ID es timestamp lineage
  canonical;
- estimator reset generationt valt, queued es in-flight regi result nem
  publikalhato;
- stale, nem aktualis raw ID-ju, regi generationu vagy contract-ID nelkuli
  result fail-closed eldobodik;
- LidarOdometry es EKF measurement-ID alapon deduplikal; rolling map, safety,
  localization es Hub observation-ID alapon szamol;
- confidence es freshness ugyanazon matcher evidence-bol szarmazik;
- a control loop csak atomikus snapshotot olvas, matcherre nem blokkol;
- az implicit `danger_zone` konstruktorut mukodik.

A vegleges IPC minden scan/result csomagban
`R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1` azonosítot visz. A confidence modell
`R2B4_SCAN_MATCH_CONFIDENCE_V2`. A protected baseline es bootstrap guard tiltja
a threades matcher, backlog, stale apply es residual-only confidence
visszahozasat.

## Történeti bizonyítékok a javítás előtti állapotról

A forráskód mellett hardvermentes determinisztikus replay futott hibás,
ismételt, késő és részlegesen frissült observationökkel. Fő eredmények:

- default queue-méret 2 mellett az 1,2 packetek együtt maradtak: nem
  latest-only;
- blokkolt matcher alatt a raw ID 42 snapshotja `None` maradt, és csak a
  matcher feloldása után jelent meg;
- ugyanaz a raw ID 7 `timestamp=10.0`, majd `11.0` értékkel publikálódott;
- nested raw pont és a consumernek visszaadott summary mutációja a service
  aktuális snapshotját megváltoztatta;
- 90.01 s forrás-age mellett a LidarOdometry candidate/latest age 0.01 s lett,
  és a kiadott mérésben nem volt lineage ID;
- ugyanaz az EKF-mérés kétszer `applied=True`; `P00 0.01 -> 0.008333 ->
  0.007143`, tehát a második alkalmazás új információként súlyozott;
- ugyanaz a raw scan a rolling map observation countot `1 -> 2` növelte;
- a follow ugyanazon, valójában 80 s-os raw scant 0.05 s frissnek látta;
- periodikusan frissített publikálási idő mellett öt STALE safety poll mind
  `allow=True` maradt: raw timestamp-age 0.05–0.09 s, odom age 2.0 s;
- estimator reset után a már generation-checken átjutott régi raw ID 12
  újra publikálódott és callbacket adott;
- friss candidate confidence 0.10 + stale accepted confidence 0.90 esetén a
  SafetySupervisor `OK`-t adott: a freshness és confidence eltérő forrásból
  keveredett;
- azonos raw ID 88 blocked-front flag két control tickben elérte a jelenlegi
  explicit 2-tick obstacle confirmot;
- a friss M0/M1 JSONL-ben a poll sorok 60–70%-a ismételt status-version/
  accepted érték. Például M1 forward: 68 poll, 22 egyedi status-version,
  22 egyedi accepted; 18 `applied` poll, de csak 5 egyedi applied
  status/accepted.

A futó strukturált status az audit alatt további aktuális bizonyítékot adott:
raw rate kb. 10 Hz, matcher queue depth 2, queue delay 165–190 ms, matcher
runtime 174 ms, teljes latency 339–364 ms. A completion-alapú candidate age
0.07–0.13 s volt, miközben a forrás teljes kora már kb. 0.41–0.49 s, az aktív
`max_scan_age_s=0.25` felett.

## Történeti PROVEN gyökérok — valamennyi lezárva

1. **Safety matcher observation-ID szerződéssértés — JAVÍTVA.** A korábbi
   ID a `candidate_created` mellé a periodikusan változó
   `lidar_last_update`-ot tette. A safety most az értékelt raw, matcher-result
   vagy accepted-odometry evidence tagelt ID-ját használja. Célzott 12/12,
   teljes 997/997, M0 PASS.
2. **A service queue nem latest-value.** `latest_scan_queue_size` aktív defaultja
   2; `_queue_latest()` csak full queue esetén dob egy elemet, ezért egy régi
   köztes scan is várhat a legfrissebb előtt. Az aktuális queue depth 2 és
   165–190 ms queue delay ezt productionben is kimutatta.
3. **A raw publikálást a matcher blokkolja.** `_driver_worker()` csak queue-ba
   tesz; `_current_snapshot` kizárólag a matcher workerben frissül.
4. **Publikálási idő raw timestampként való használata.** Queue timeoutkor a
   service ugyanazt a scan/summary párt új `time.monotonic()` timestamp-pel
   publikálja. Ez elfedi a raw stalenesst a controller, SafetySupervisor és
   follow elől.
5. **A snapshot nem immutable.** A frozen dataclass listet/dictet tart, csak
   sekély másolat készül, a `get_snapshot()` pedig ugyanazt az objektumot adja
   vissza. A driver meta listamásolata is megosztja a pont-dicteket.
6. **Nincs explicit lineage és helyes timestamp-átvitel.** A driver
   `scan_ts_mono` mezőjét a service eldobja és poll-időre cseréli; az estimator
   a kapott `raw_meta`-t nem teszi a resultba; a LidarOdometry callback-completion
   időt használ, a kiadott candidate/measurement nem tartalmaz source ID-ket.
7. **Reset/publish TOCTOU.** A generation check és a snapshot+callback között
   reset futhat; így reset után régi frame-ből származó result jelenhet meg.
8. **Ugyanazon odometriai mérés EKF reapply.** A control loop
   `_lidar_last_delivered_odom` másolatát `cadence_soft_reapply` ágon ismét
   `update_lidar()`-nak adja. Az EKF nem kap measurement ID-t és nem deduplikál.
9. **Rolling-map duplicate/rejuvenation.** `cont.py` snapshot timestampból
   képez `new_scan`-t, a RollingLocalMap pedig minden update-nél új, `now`
   idejű observationöket ad hozzá. Ugyanaz a scan többször számolódik és
   TTL-je mesterségesen megújul.
10. **Safety confidence/freshness source-keverés.** A quality gate a
    candidate/latest freshness OR-ját, de a két confidence freshness-független
    maximumát használja. Stale nagy confidence elfedheti a friss rossz
    confidence-ot.
11. **Nem atomikus controller adapter.** `lidar_summary`, `lidar_last_update` és
    `lidar_health` három külön assignment; a safety nem ugyanazon lock alatt
    olvassa őket. A control loop ezen felül lock nélkül ír a LidarOdometry
    privát `_stats` dictjébe, ezért telemetry snapshot részlegesen frissülhet.
12. **LocalizationGate forráskeverés.** A `latest_candidate_recent` a
    `candidate_available` flaget az accepted measurement `latest_age_s`
    értékével kombinálja, nem a candidate saját age/ID-jával.
13. **Hub observation-vakság és poll-súlyozás.** A live validator nem ment
    raw/candidate/measurement ID-t. `applied_samples`, confidence median és age
    poll-soronként számolódik; azonos status többszörösen súlyozódik.
14. **Implicit LidarService konstruktorhiba.** Explicit `danger_zone` nélkül a
    config `get()` hívás argumentumütközéssel `TypeError`-t ad. A production
    explicit értéket ad, ezért ez nem a jelenlegi live hiba gyökere.

Az obstacle confirm ugyanazon raw flaget tickenként számolja, de a konfigurált
szerződés neve és a reason explicit `tick`. A viselkedés bizonyított; hogy ezt
observation-confirmra kell-e migrálni, a confidence/freshness szerződéstől
külön döntés. A confirm érték nem változhat.

## Történeti HIGHLY PLAUSIBLE megállapítások

- A 2026-07-22 14:06Z jobb-pivot LOW_CONF stopot a javított safety-ID hiba
  okozta. Azonos 0.2138 confidence ismétlődött, de az artefakt nem őrzött
  candidate ID-t.
- A default queue és completion-alapú freshness hozzájárul a jelenlegi
  relocalization/accepted-measurement stalenesshez. A backlog bizonyított, de
  az M1 keréksebesség- vagy encoder timing hibáinak nem bizonyított oka.
- A service/driver `stop()` rövid vagy hiányzó joinja gyors stop/start esetén
  régi worker továbbfutását engedheti. A reset TOCTOU reprodukált; kettős
  worker production-előfordulása nem.

## Történeti NOT PROVEN megállapítások

- Matcher/GIL a control- vagy encoder-gap gyökéroka. A korábbi 5 ms/1 ms
  switch-interval A/B nem különítette el; a friss M0 PASS volt. Az M1 egy
  arc-left timing-gapje korreláció, nem kauzális bizonyíték.
- Az EKF `cadence_soft_reapply` lefutott a friss M0/M1-ben: a forrásút és a
  kétszeri EKF-hatás bizonyított, de az aktuális runtime counter 0.
- A friss M1 FAIL LIDAR-safety gyökere. Az incident kizárólag backward
  tracking MAE-t, arc-left encoder timing-gapet és stop/start suspectet nevez.

## Javítás előtti pozitív invariánsok és magasabb rétegek

- A driver egy lezárt fordulatnál egyszer növeli a `scan_seq`-t, a service
  driver worker pedig csak szigorúan növekvő seq-et queue-z.
- A LidarOdometry normál `get_odometry()` útja `_consumed` flaggel egy
  accepted latest mérést egyszer ad ki. Ezt a control-loop reapply kerüli meg.
- A room-cruise stuck/exit evidence `scan_seq`-et használ; ugyanazt a seq-et
  nem tekinti új mintának. Közvetett érintettsége a rolling-map duplicate és
  a késleltetett raw publikálás.
- Follow közvetlenül a snapshot raw scanját és `timestamp` freshnessét
  használja, ezért a false-fresh republish bizonyítottan érinti.
- MotionExecutor nem hoz létre új LIDAR observationt; az upstream
  localization/safety eredményt hajtja végre az egyetlen motion úton.
- M1 FAIL miatt room-cruise/follow/M4 élő kapu nem indítható.

## Lezárt javítási egységek

Az alabbi lista a vegrehajtott migracio eredeti sorrendje. Nem nyitott
fejlesztesi terv; valamennyi pont celzott regresszioval lezart.

1. **Latest-value queue:** kizárólag a queue minden korábbi várakozó elemének
   eldobása az új latest packet előtt; saját replay/unit, teljes pytest,
   bootstrap, majd M0 -> M1.
2. **Raw/matcher snapshot, timestamp, immutability és lineage:** azonnali raw
   snapshot a driver `raw_scan_id/raw_scan_timestamp` adataival; külön atomikus
   matcher result saját ID-val és source raw ID-val; odometry measurement ID és
   lineage; reset-generation publish atomikussá tétele. Nincs párhuzamos
   LIDAR-út.
3. **Freshness/consumer konzisztencia:** rolling map raw ID-t használjon,
   follow/safety/localization a saját evidence timestamp/ID-ját; controller
   adapter atomikus legyen; stale és részleges snapshot replayek.
4. **EKF duplicate/reapply:** a mérés-ID end-to-end átvitele után ugyanaz a
   `lidar_odometry_measurement_id` legfeljebb egyszer juthat `update_lidar()`-ba;
   a cadence monitor nem gyárthat pszeudomérést.
5. **Hub validator:** ID-k mentése, observation-alapú aggregáció és explicit
   duplicate/lineage gate. Pollismétlés diagnosztika maradhat, mérésként nem.
6. **Obstacle tick-hiszterézis:** csak külön szerződésdöntéssel;
   confirm count/küszöb nem változik.

Minden runtime-egység után: célzott replay, teljes offline pytest, bootstrap,
friss preflight, M0; kizárólag M0 PASS után M1. Magasabb kapu csak M1 PASS
után.
