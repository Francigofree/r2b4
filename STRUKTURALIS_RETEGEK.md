# R2B4 stabil rendszerbaseline

- Szerep: az architektura, a tartos szerzodesek es a hosszu tavu dontesek ember altal olvashato SSOT-ja.
- Utoljara forrasbol ellenorizve: `2026-07-29`.
- Nem esemenynaplo. Az aktualis task gepi SSOT-ja:
  `project_rules/current_change.json`; tartos lezarasi bizonyiteka a
  `logs/agent_tasks/<task-id>/receipt.json`.
- A gepileg ellenorzott vedett azonosito-keszlet: `project_rules/protected_baseline.json`.

## Igazsagforrasok es prioritasi rend

1. A tenyleges runtime viselkedes igazsagforrasa a forraskod es az aktiv konfiguracio.
2. Egy konkret validacio eredmenyenek SSOT-ja a Test Hub compact summary-ja, hiba eseten az incident bundle.
3. Ez a dokumentum a vedett architekturalis szerzodesek es dontesek SSOT-ja.
4. `project_rules/current_change.json` az aktualis task gepi allapota.
5. `docs/AGENT_RUNTIME.md` rovid validacios routing utmutato; a profilok
   registry-SSOT-ja `tools/r2b4_test_hub.py`.
6. Torteneti tervek es riportok nem irhatjak felul a fenti forrasokat.

Ellentmondasnal az agent nem talalgat: megnezi a kodot, konfiguraciot es a legfrissebb relevans strukturalt artefaktot, majd vagy frissiti az elavult dokumentumot, vagy rogziti a bizonytalansagot az aktualis allapotban.

## Vedett rendszerazonositok

| Szerzodes | Aktualis ertek | Implementacios SSOT |
|---|---|---|
| Vezerlesi mod | `UNIFIED` es csak ez | `core/control_strategies.py`, `conf/control_mode.json` |
| Odometry mod | `LIDAR_FIRST` | `conf/vezerles.json` |
| Pose SSOT | `EKF_POSE_ODOMETRY_SSOT` | `middleware/ekf.py`, `controller/motion_physical.py` |
| Globalis frame | `R2B4_BOOT_ROBOT_MAP` | `middleware/robot_frame.py` |
| Frame tengelyek | `+X` indulasi robot-elore, `+Y` indulasi robot-balra, yaw `CCW_POSITIVE_LEFT` | `middleware/robot_frame.py` |
| Scan matcher runtime | `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1` | `middleware/scan_matcher_contract.py`, `sensors/lidar_service.py` |
| Scan matcher confidence | `R2B4_SCAN_MATCH_CONFIDENCE_V2` | `middleware/scan_matcher_contract.py`, `middleware/scan_matching.py` |
| Scan matcher pose-integrity | `R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1` | `middleware/scan_matcher_contract.py`, `middleware/scan_matching.py`, `safety/safety_supervisor.py` |
| Scan matcher transport | `process_latest_only` | `middleware/scan_matcher_contract.py`, `sensors/lidar_matcher_process.py` |
| Motorvegrehajto | egyetlen `MotionExecutor` | `motion_executor.py`, `cont.py` |
| Speed map schema/allapot | `R2B4_WHEEL_SPEED_MAP_V2`, `ACTIVE` | `middleware/ffp.py`, `conf/speed_map.json` |
| Aktiv kerek-gorbek | `left_forward`, `left_reverse`, `right_forward`, `right_reverse` | `conf/speed_map.json` |
| Teszt orchestration | Test Hub | `tools/r2b4_test_hub.py` |

## Aktiv vegrehajtasi lancok

### Mozgas

`GUI/API/allapot/follow/room-cruise keres -> controller/commands -> retegjavaslatok -> MotionResolver -> motion shaping es policy -> localization/safety gate -> MotionExecutor -> motor driver`

- A felso retegek fizikai celokat adnak: `v`, `omega` vagy bal/jobb cel-kereksebesseg.
- A resolver egy tickben pontosan egy vegrehajtando parancsot valaszt.
- A safety reteg korlatozhat vagy nullazhat, de nem hoz letre parhuzamos mozgasi szandekot.
- A `MotionExecutor` egyszer vegzi el a diff-drive felbontast es az egyetlen kerek-szintu PWM eloallitast.
- A negyiranyu speed map feed-forward pontosan a kozos kerek-hurokban fut; a PI maradekkorrekcio diagnosztikailag kulon lathato.
- Az elso bolygokerek passziv, valtozo iranybeallasi es gordulesi ellenallasa
  mechanikai zavaras. Emiatt a hajtott kerek encodersebessege onmagaban nem
  bizonyit fizikai haladast: a trust kapu LIDAR/EKF vegpontot is kovetel, es
  az atmeneti ellenallas nem kompenzalhato fix trim vagy masodik vezerlesi
  uttal.
- Kalibracional letezik kulon armozott, idoben, tavolsagban es PWM-ben
  korlatozott executor-ag. Ez nem normal mozgasi bypass: a PI, straight-hold,
  speed-map, startup/maintenance floor es planner korrekcio ilyenkor ki van
  kapcsolva, a SafetySupervisor/LIDAR-veszvedelem es a runtime PWM-cap
  downstream megmarad.
- A wheel-map `startup_pwm` erteke csak megbizhato encoderrel bizonyitott
  megindulasig aktiv, majd ugyanabban a menetiranyban nem elesztheto ujra.
  A `maintenance_pwm` a map nyilt hurku fenntartasi kuszobe; a PI negativ
  maradekkorrekcioja bizonyitott tulsebessegnel ez ala csokkentheti a vegso
  PWM-et.

### Room cruise es emberkovetes

`NavigationIntent / FollowRequest -> CruiseLayerV2 -> RollingLocalMap -> LocalNavigationLayer -> resolver -> kozos mozgaslanc`

- Room cruise es follow nem irhat kozvetlen PWM-et.
- Pivot, egyenes, hatramenet es ARC ugyanazt a kerek-szintu vegrehajtot es speed mapet hasznalja.
- A kamera csak celpontmegfigyelo; nem motorvezerlo es nem pose-tulajdonos.

### Poziciobecsles

`IMU predict + kapuzott encoder update + kapuzott LIDAR update -> EKF -> x,y,theta,v,omega`

- A globalis koordinatarendszert nem szenzor hozza letre: a boot/reset robot-frame elore rogzitett.
- Az IMU yaw elojel, az encoder yaw es a LIDAR relatív yaw ugyanazt a CCW-pozitiv konvenciot hasznalja.
- A LIDAR scan-to-map kereses aktualis EKF pose seedbol indul; a LIDAR meres marad, nem frame-tulajdonos.
- A publikus pose es a mozgasi tenyadat az EKF kimenete. Szenzor nem vezerelhet kozvetlenul mozgast.

### LIDAR raw safety es scan matcher

`driver teljes scan -> parent raw snapshot -> raw safety/map consumer`

`ugyanaz a teljes scan -> single-slot IPC -> kulon matcher processz -> single-slot result IPC -> LidarOdometry gate -> EKF`

- A raw snapshot a parent driver workerben, a matcher IPC enqueue elott
  publikalodik. Raw safety, obstacle evidence es a scan elerese nem varhat a
  matcherre.
- A `blocked_front/back`, `min_dist`, `min_dist_narrow`, `min_back`,
  `avg_left/right` es `bounce_dir` mezok minden friss scanben a parent
  processz aktualis raw pontjaibol keszulnek
  (`PARENT_CURRENT_RAW_SCAN`). Kesve erkezo vagy befagyott matcher-summary
  ezeket nem irhatja felul; a matcher csak pose/odometry es minosegi mezok
  tulajdonosa.
- A teljes stateful `LidarEstimator` csak a kulon `spawn` processzben vegezhet
  scan matchinget. Runtime-threades matcher worker tiltott.
- Az input es result queue kapacitasa pontosan `1`; enqueue es dequeue
  latest-only. Backlog nem epitheto.
- Minden scan es result a
  `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1` contract-ID-t viszi. Hianyzo vagy
  eltero contract, regi estimator-generation, nem aktualis raw scan ID vagy
  `0.25 s`-nel regebbi input/result fail-closed eldobando.
- A measurement confidence modell az `R2B4_SCAN_MATCH_CONFIDENCE_V2`: robust
  residual, inlier, angular coverage es lokalis megfigyelhetoseg alapjan az
  EKF-meres minoseget jelzi. A globalis pose-integrity kulon
  `R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1` jel: alternativ poz csak teljes SE(2)
  coarse seedbol finomitott, a nyertestol elvalt es koztes koltsegbarrierrel
  igazolt basin lehet. Scan-only es prior-aware uniqueness kulon publikalodik.
  A safety mind a measurement confidence-et, mind az integrity allapotot
  fail-closed ellenorzi; budget-tullepes egyik jelben sem fedheto el.
- A canonical `raw_scan_id -> matcher_result_id/candidate_id ->
  lidar_odometry_measurement_id -> EKF applied measurement ID` lineage
  megszakitas nelkul, deduplikalva ervenyes. Egy poll nem uj observation.
- Reset/reanchor estimator-generationt valt es minden regi queued/in-flight
  resultot ervenytelenit. Ez teszi biztonsagossa az IDLE, illetve deklaralt
  M0/M1 szunet alatti manualis robotmozgatast.

### Runtime, safety es telemetry

- Entrypoint: `os.py`; controller eletciklus: `cont.py`; 50 Hz cel control loop.
- Startup es health: `startup/`, watchdog, localization gate es `safety/`.
- Normal motor PWM csak a `cont.py` vegso executor-kimeneten irhato. Mas kozvetlen motoriras csak nullazo stop/init/emergency muvelet lehet.
- A periodikus logger-housekeeping a control threaden csak RAM-snapshotot
  frissithet. A `runtime_stats.json` periodikus fajlirasa az egyetlen
  `AsyncLogger` flush worker tulajdona; szinkron vegso iras csak startup/shutdown
  lifecycle-ponton engedett.
- A GUI command-lifecycle poll az append-only `command_status.jsonl` fajlt csak
  a fajlveg felol, meretben korlatozott blokkokkal olvashatja. Teljes-fajl
  `readlines()` ugyanabban a CPython processben tiltott.
- A LIDAR minosegi safety-confirmacio fuggetlen matcher observationok szamat
  jelenti; egyetlen, tobb control tickig megmarado scan nem szamolhato tobbszor.
- A matcher contract-ID, confidence- es integrity-model, kulon PID, queue-kapacitas/melyseg,
  stale/drop/error, CPU, latency es RSS a strukturalt LIDAR odometry status
  resze. Ezek diagnosztikak; az EKF/safety gate tulajdonjogat nem valtoztatjak.
- Opt-in Replayer capture alatt minden uj matcher-result egyszer rogzitett,
  hash-lancolt raw scan + pontos local-map pontfelho + seed + aktiv matcher
  konfiguracio evidence-et kap. Az offline matcher adapter ezt ugyanazon
  production `match_scan_to_map` fuggvennyel ujrafuttatja; az azonos resultot
  hordozó control tickek csak ID-referenciat tarolnak.
- Strukturalt runtime allapot: `runtime/status.json`.
- Teszteredmeny SSOT: `logs/latest/latest_hub_summary.json`; hiba: `latest_hub_incident.json`.

## Bizonyitott aktualis baseline

- A scan matcher vegleges contractja
  `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1`. A teljes matcher-fejlesztes
  `1065` offline regressziot, M3 no-motion PASS-t es elo M0 PASS-t adott. A
  ket ezt koveto M1-ben matcher-, active timing-, safety- es lineage-hiba `0`;
  a FAIL-ok kulon settled wheel-speed motion-quality kapuk voltak.
- Ismetlodo geometria regi, `0.9994` confidence melletti nagy pose-hibajat a
  V2 modell `0.082` confidence-szel, `ambiguous_alternative` degeneraciokent
  jelolte. Kalibracios Brier `0.152`.
- Raspberry Pi 5-on a kulon matcher processz current RSS-e kb. `53 MiB`,
  10 Hz-en `0.447` CPU-mag; a vegleges contract-kodon a no-motion control-loop
  EMA p50 `4.501 ms`, p95 `5.394 ms`, LIDAR slow-spike `0`. Queue-backlog es stale EKF-alkalmazas nem
  tortent.
- A logger-jitter javitas elotti azonos tartalmu disk/RAM M0 A/B-ben a
  `logger_housekeeping` maximum `59.770 -> 3.408 ms`, a teljes tick maximum
  `72.637 -> 39.225 ms` volt. A vegleges aszinkron snapshot production SD-n
  `0.073 ms`, a friss M0 alatt `0.163 ms` housekeeping maximumot adott.
- A vegleges loggerforrassal az M0 (`2026-07-22T11:28Z`) es egy M1
  (`2026-07-22T11:41Z`) osszesen 12 mozgo eseteben az aktiv encoder timing-gap
  `0`, a logger drop es write error `0`; ez a logger-I/O gyokerok javitasat
  bizonyitja, de nem orokli at a PASS-t kesobbi kodallapotra.
- Az intermittalo encoder timing-gap fejlesztes lezart. A vegleges X1/snapshot
  javitas utani M1-miniben az aktiv motion-gap delta `0` volt; a maradek ritka
  canonical `TIMING_GAP` minta nem jut PI- vagy EKF-fogyasztasba. A validator
  ezt esetszammal es maximum idovel `WARNING`-kent tartja meg: csak a vele
  fuggetlenul bizonyitott safety-, sensor-truth- vagy mozgasminosegi hiba ad
  FAIL-t. A warning trendje minden kesobbi M0/M1/Mx artefaktban figyelendo.
- A KIT0085 A-rising driver-alert konfiguralt `150 us` stable-level
  debounce-ot hasznal. A `20260729T193307Z` pivot-onset trace a bal
  encoderben kb. `125 us` periodusu, motor-PWM-hez kotheto teves
  A-esemenysorozatot bizonyitott; a `20260729T193825Z` ismetlesben ez
  eltunt, mind a negy pivot `WHEEL_FEEDBACK_UNAVAILABLE` es aktiv
  PWM-null nelkul futott. A kuszob kisebb a bemert `0.582 m/s` uzemi
  plafon kb. `276 us` A-to-next-quadrature elvalasztasanal. Az
  `R2B4_ENCODER_EDGE_TRACE=1` csak diagnosztikai, bounded RAM/status
  felulet; alapertelmezetten inaktiv, motor-, PI-, EKF- vagy safety-utat
  nem valtoztat.
- A GUI lifecycle poll korabban 50 ms-onkent beolvasta a teljes, 92 MB-os
  command-status journalt. Egy valos lookup `128.444 ms` volt; a korlatozott
  reverse-tail olvasas mean `0.100 ms`, max `0.227 ms`, az utolso 4000 soros
  szemantika valtozatlan.
- A ket legutobbi teljes M1 (`2026-07-23T18:54Z`, `18:56Z`) mind a nyolc
  primitivet safety-stop, aktiv encoder timing-gap, lineage-hiba, logger
  drop/write error es motion-GC nelkul vegrehajtotta. Mindket Hub verdict
  `FAIL`, kizarolag a valtozatlan settled wheel-speed tracking kapu miatt.
- Az M1-be epitett korabbi `R2B4_M0_MINI_FIRST_MOTION_V1` kapu elo hardveren
  bizonyitottan fail-closed: harom teljes jelolt M1 elott PASS utan
  folytatta a profilokat, egy frissebb mini FAIL utan pedig pontosan egy
  mozgasi esetnel megallt. A FAIL artefakt nem jelolte teljes M0-val
  egyenertekunek az eredmenyt.
- A forras- es futasazonos candidate-logok bizonyitottak, hogy a V1
  `0.30 s` settled kezdete az elso passziv bolygokerek fizikai befordulasat
  is belemérte. Az utod `R2B4_M0_MINI_FIRST_MOTION_V2` az elso `1.0 s`
  tranzienst kulon tartja: `3+3` fuggetlen encoderablak, teljes settled
  `<=0.030 m/s`, legfeljebb `1.0 s` beallasi ido es valtozatlan post-caster
  `<=0.015 m/s` kotelezo. Safety-, sensor-truth-, timing-, endpoint- es
  fail-closed folytatasi kapu nem valtozott; a V2 elo PASS meg nyitott.
- Az aktiv kerek-PI `Kp=0.25`, `Ki=0.08`, valtozatlan `0.18`
  integratorlimittel. A korabbi `0.40/0.04` futashoz kepest harom teljes
  jelolt M1 ot linearis esetere vett settled MAE atlaga
  `0.01792 -> 0.01362 m/s` (`24.0%` javulas). Ez hangolasi bizonyitek, nem
  teljes M1 PASS: a jobb ARC es a mechanikai allapotfuggo szoras nyitott.
- Az elso M1.1 elo paros caster-futas
  (`hub_M1_1_caster_orientation_live_20260723T213421Z`) M0-mini PASS utan
  mind a hat aligned/reversed-180 part vegrehajtotta. Forward, backward es
  mindket pivotpar raw M1 PASS volt. Bal ARC aligned `0.01943 m/s` miatt,
  jobb ARC reversed `0.03832 m/s`, post-1s `0.05538 m/s`, aktiv timing-gap
  es stop/start szakadas miatt FAIL. Tranziens-waiver nem volt alkalmazhato;
  a teljes M1.1 es igy az M1 bolygokerek-alapu lezárasa nyitott.
- A speed-map es a teljes alvazdinamika bizonyitasi hatokore kulon
  verziozott. A promotion-blokkoló M1
  `R2B4_M1_SPEED_MAP_EXECUTION_V1`: embedded M0-mini, normal wheel-map + PI,
  wheel-reference tracking, safety, sensor-truth, timing, tavolsag,
  stop/start es normal stop. A passziv caster, fizikai yaw/gorbulet,
  csuszas, effektív nyomtav es pivotdinamika verdictje az onallo
  `R2B4_M2_CHASSIS_MOTION_DYNAMICS_V1` tulajdona. Az M2 sajat hatokoreben
  fail-closed rendszervalidacio, de nem tagja es nem bemenete a speed-map
  ACCEPT/promotion lancnak; mapet, PID-et es control logikat nem irhat.
- Az alapertelmezett 5 ms es 1 ms CPython thread-switch interval azonos,
  matcher-terheleses offline A/B-ben egyarant `0` darab 40 ms feletti periodust
  adott. Processz-szintu switch-interval hangolas nem kerult productionbe.
- Az egyetlen `UNIFIED` mod, az ures legacy command tipusok, az executor-only normal motorut es a negyiranyu speed map kod- es tesztszinten jelen van.
- Elo pivot parban a bal es jobb fizikai forgasi irany, encoder, IMU, LIDAR es EKF yaw elojel egyezett.
- Pivot eredmeny: bal `+33.065 deg` (cel `+30`), jobb `-29.209 deg` (cel `-30`). A jobb eset PASS; a bal fizikai kapui teljesultek, de a runtime-frekvencia kapu FAIL volt.
- A frame runtime telemetriaban `R2B4_BOOT_ROBOT_MAP`, tulajdonosa `EKF_POSE_ODOMETRY_SSOT`.
- A preflight a legutobbi M3 room-cruise es pivot futasban PASS volt; emergency vagy tiltott normal command ut nem jelent meg.

## Nyitott, nem stabilnak minositett pontok

- A legutobbi 60 s room-cruise (`2026-07-16 21:44Z`) a frame/yaw javitas elotti kodon futott es FAIL: wheel tracking P90 `0.0675 m/s`; a regi magnetometer-alapu endpoint comparator is hibazott. A jelenlegi kodot uj 60 s futasnak kell majd validalnia.
- Az uj PI harom teljes M1 ismetlese kozul egyik sem adott teljes PASS-t:
  a legjobb futasban csak a jobb ARC volt `0.00072 m/s`-mal a valtozatlan
  kapu felett, egy masikban harom wheel-tracking eset, a harmadikban ket
  wheel-tracking eset mellett ket aktiv encoder timing-gap is megjelent.
  A negyedik futas sajat mini kapuja `0.01829 m/s` MAE miatt helyesen
  leallitotta az M1-et. A hiba iranya es nagysaga tovabbra sem stabil; a
  bolygokerek szabad forgasa, iranybeallasa, terhelese es a felulet fizikai
  ellenorzese nelkul tovabbi speed-map vagy fix trim valtoztatas nem
  indokolt.
- A ket `2026-07-23T20:06Z` es `20:33Z` M0 ugyanazt az elso-egyenes aktiv
  encoder timing-gapet, majd felvaltva bal/jobb ARC encoder-LIDAR
  vegpontelterest mutatta. A bolygokerek valtozo ellenallasa es az ebbol
  kovetkezo hajtokerek-csuszas mechanikai gyokerok lehet, de tovabbi
  kontrollalt bizonyitek nelkul nem vegleges diagnozis.
- A speed map aktiv ertekei sematikus migraciobol szarmaznak. Teljes tartomanyu, anomaliamentes candidate kalibracio meg nincs elfogadva.
- A tavolsagvezerelt speed-map meres/analyzer/validator fejlesztes offline
  regresszioja PASS (`160` celzott, `1149` teljes). Az uj lanc negy profilt,
  kulon startup/maintenance kuszobot, novekvo/csokkeno sweepet, automatikus
  mintaujramerest es candidate rollbacket kenyszerit ki. Ez toolbizonyitek,
  nem elo map-kalibracio: candidate es friss no-PI/PI/M1 ACCEPT meg nincs.
- Emberkovetes nem release-ready: a legutobbi M3 follow es camera-follow summary FAIL.
- A hardware performance gate nehany futasban host metrika hianyaban `INCONCLUSIVE`.

## Vegleg megszuntetett vagy tiltott utak

- `BASIC`, `ENHANCED`, `FULL` valaszthato vezerlesi modok.
- Normal runtime `set_motor_pwm` GUI/API parancs.
- Normal runtime `set_tank` / legacy tank adapter vegrehajtas.
- Nem ures legacy command-type registry.
- Pivot-specifikus speed map, fix bal-jobb trim vagy masodik feed-forward ut.
- Resolver utani masodik sebessegszabalyzo vagy parhuzamos motion-shaping ag.
- Kulon motion-semantics clearance-governor: soft clearance tulajdonos a
  planner/policy, a vegso hard output-szuro a `SafetyGate`.
- Kerek-D tag vagy straight-hold derivalt korrekcio; az aktiv kerek-hurok PI,
  a straight-hold P/deadband/filter/slew/saturacio.
- Szenzor altal kozvetlenul kiadott motorparancs.
- Runtime-processzen beluli scan matcher worker vagy masodik matcher-ut.
- Egynel nagyobb scan/result matcher queue, FIFO backlog vagy stale result
  alkalmazasa.
- Contract-ID nelkuli matcher IPC/result, generation/ID/freshness ellenorzes
  megkerulese, illetve residual-only confidence modell visszahozasa.
- Safety, PASS vagy minosegi kuszob lazitasa egy futas sikeresse teteleeert.
  A canonical encoder `TIMING_GAP` non-blocking WARNING-szemantikaja nem
  kuszoblazitas: a 40 ms meresi kuszob, a mintak kizárasa es minden kulon
  safety/minosegi kapu valtozatlan.

## Tartos dontesi nyilvantartas

| ID | Dontes | Ujranyitashoz kotelezo bizonyitek |
|---|---|---|
| `D001` | Egy mozgasrendszer van: `UNIFIED`. | Ismetelheto regresszio, amely bizonyitja, hogy a kozos ut nem javithato; kulon felhasznaloi jovahagyas. |
| `D002` | Egy tickben egy motion intent, egy pose es egy motor output lehet. | Ownership manifest es legalabb harom osszehasonlithato elo futas. |
| `D003` | A vegso pose tulajdonosa az EKF. | Szenzor- es frame-szerzodeses meres, replay es elo mozgasteszt. |
| `D004` | A globalis frame fix boot/reset robot-frame, nem szenzor-frame. | Dokumentalt frame-migracio, teljes lokalizacios regresszio es elo validacio. |
| `D005` | Egy kozos negyiranyu kerek-speed-map hasznalhato. | Legalabb harom anomaliamentes, iranyonkent osszehasonlithato meres es candidate regresszio. |
| `D006` | Classifier/readiness/hiszterezis csak ismetlodo meresi tevesztesre valtozhat. | Tobb egyedi statuszverzios pivotban azonos, bizonyitott tevesztes. |
| `D007` | Uj runtime muszerezes csak akkor keszul, ha a meglevo fazisadat nem nevezi meg a koltseges blokkot. | Ismetelt runtime FAIL es dokumentalt diagnosztikai hiany. |
| `D008` | Kalibracios direct PWM csak armozott, legfeljebb a normal runtime PWM-capig, 4 s command-timeouttal es 1.80 m shuttle-lablimittel engedett executor-agban. | Safety review, armozas-, timeout-, runtime-cap-, tavolsag- es zero-output regresszio. |
| `D009` | Periodikus logger snapshot fajl-I/O nem futhat a control threaden; a queue-to-flush-worker lanc vedett. | Azonos tartalmu disk/RAM A/B, fazismeres es friss M0+M1, amely bizonyitja, hogy a workerre helyezes rosszabb timingot vagy adatvesztest okoz. |
| `D010` | A command lifecycle lookup csak korlatozott reverse-tail olvasast hasznalhat; teljes journal-beolvasas tiltott. | Valos meretu journallal A/B, amelyben a teljes olvasas nem okoz GIL- vagy control-kesest. |
| `D011` | A LIDAR safety hiszterezis fuggetlen observationt, nem control ticket szamol. | Harom kulon scan nelkul igazolt veszelykimaradas, vagy observation-ID hibaja. |
| `D012` | A scan matcher csak `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1` szerint, kulon processzben, single-slot latest-only IPC-vel es V2 confidence-szel futhat. | Reprodukalhato geometriai vagy timing-regresszio, offline A/B, teljes lineage/safety review, friss M0 es ket egymast koveto M1; kulon felhasznaloi jovahagyas. |
| `D013` | Az M1 elso mozgasa a `R2B4_M0_MINI_FIRST_MOTION_V2` fail-closed kapu; az ismeretlen kezdeti bolygokerek-allashoz az elso `1.0 s` kulon fizikai tranziens, `3+3` fuggetlen encoderablak, teljes settled `<=0.030 m/s`, legfeljebb `1.0 s` beallas es valtozatlan post-caster `<=0.015 m/s` kotelezo. PASS teljes M0 PASS-szal egyenerteku, FAIL utan tovabbi M1 mozgas tiltott. | Uj measurement-truth szerzodes, amely legalabb azonos szenzor-, safety-, timing-, endpoint- es post-caster wheel-tracking bizonyitekot ad, celzott regresszio es kulon felhasznaloi jovahagyas. |
| `D014` | Az egyetlen aktiv kerek-PI hangolasa `Kp=0.25`, `Ki=0.08`, integratorlimit `0.18`; a kisebb P a kesleltetett, bolygokerek-terhelesre ingadozo encoderablakok kergeteset csokkenti, a nagyobb I a tartos terhelesi korrekciot tartja meg. | Harom osszehasonlithato, mini-PASS elo M1, amely az elozo `0.40/0.04` hangolasnal rosszabb aggregalt wheel-trackinget mutat, vagy celzott stabilitasi regresszio; fix trim es speed-map atiras tovabbra sem helyettesiti ezt. |
| `D015` | A kontrollalt bolygokerek-orientacio hatasat a `R2B4_M1_1_CASTER_ORIENTATION_V1` paros validator meri: aligned/reversed-180 sorrend, 10 s operatori ablak, teljes fazisu valtozatlan kapuk; csak reversed linearis, elso 1.0 s-ra bizonyitott wheel-tracking tranziens kaphat szuk waiver-t. A canonical `TIMING_GAP` minden Mx-ben kulon, nem blokkoló WARNING; timing-contract-hiany nem waiverelheto. Ez az orientacios bizonyitek nem irja felul sem az M1 speed-map, sem az M2 chassis-dynamics verdictet. | Futasazonos caster-szog szenzor vagy uj mechanikai szerzodes, illetve legalabb harom kontrollalt paros futas, amely bizonyitja, hogy az 1.0 s ablak vagy a szuk failure-lista nem helyes; safety/timing-contract/szenzor/stop kapu kulon jovahagyassal sem waivelheto. |
| `D016` | Speed-map alapmeres a regi aktiv map M0 PASS-a nelkul indulhat, mert armozott direct-PWM uton sajat safety/LIDAR/encoder/stabilitasi kapukkal, map es PI nelkul mer; a regi map okozta M0-korfuggest nem szabad visszahozni. Az egyenes ingajarati palyahossz-preflight friss teljes parent raw scanbol a robot tengelye koruli konzervativ `+/-0.30 m` teglalap folyosot meri; az oldalso fal nem szamithat elore falnak, mikozben az angularis safety-szektorok es az `1.80 m` kapu valtozatlanok. A `214` elfogadott mintas, profilonkent `16` felso ismetleses meres utan a kezelo a bizonyitott kozos stabil tartomanyt veglegesitette: uzemi plafon `0.582 m/s`, meressel bizonyitando minimum `0.58 m/s`, acquisition PWM-cap `0.64`; mas minosegi es safety-kapu valtozatlan. Teljes PASS acquisition utan csak analyzerrel konkretan bizonyitott threshold-sorrendi vagy felso-lefedettsegi hiany potolhato fix, eredeti mintat hash-elve megorzo supplement profillal; az elso supplement utani akkori `0.60 m/s` lefedettsegi FAIL analyzer/run/source/hash-hez kotott masodik fix blokkban negy novekvo es negy csokkeno `0.64` PWM-es part mert. Egyik supplement sem irhat candidate-et vagy aktiv mapet, minden korabbi minta es ugyanazok a meresi kapuk megmaradnak. Promotion csak a `speed_map_calibration` sequence sorrendhelyes, azonos candidate-ID-ju analyzer, no-PI, PI es candidate alatti teljes `R2B4_M1_SPEED_MAP_EXECUTION_V1` PASS-a utan lehet; az M1 elso embedded `R2B4_M0_MINI_FIRST_MOTION_V2` kapuja kotelezo. A PI/M1 candidate csere utan pontos fajlhash- es runtime-reload rollback kotelezo; a decision profil magatol nem aktival. Az M2 nem promotion-input. | Uj meresi szerzodes, amely legalabb azonos negyprofilos threshold/sweep/stabilitasi, safety-, candidate-M0/M1-, rollback- es teljes M1 bizonyitekot ad. |
| `D017` | A `R2B4_M1_SPEED_MAP_EXECUTION_V1` kizarolag a speed-map normal wheel-map + PI vegrehajtasat, embedded M0-minit, wheel trackinget, safety/szenzor/timing/tavolsag/stop bizonyitekot minositi es promotion-blokkoló. A passziv caster, fizikai yaw/gorbulet, csuszas, effektív nyomtav es pivotdinamika onallo `R2B4_M2_CHASSIS_MOTION_DYNAMICS_V1` rendszervalidatorhoz tartozik. Az M2 fail-closed a sajat hatokoreben, de `speed_map_promotion_blocking=false`, nem resze a speed-map decisionnek, es map/PID/control logikat nem modosithat. | A ket hatokor ujraegyesitesehez olyan futasazonos bizonyitek kell, amely igazolja, hogy a chassis-dinamika metric kozvetlen speed-map vegrehajtasi hiba, tovabba kulon felhasznaloi jovahagyas es teljes candidate rollback/promotion regresszio. |

## Valtoztatasi szabaly

Vedett reteget csak konkret hibabizonyitek es celzott teszt mellett szabad
modositani. A task-, hash-, lease- es tesztallapotot az `agentctl` gepileg
rogziti; kezzel szerkesztett Markdown task-state nem authority. A stabil
baseline csak a dontes tartossa valasakor frissul.
