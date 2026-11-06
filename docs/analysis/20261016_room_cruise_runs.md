# Room Cruise: 2026-10-16 19:21 és 19:23

A két futásban az `angular_velocity_gain` rögzített értéke **0.0**. Az emberi tapasztalat szerint az egyenesmeneti oszcilláció megszűnt, a sebesség és az RC mozgásminőség javult. A capture ezt a konfigurációt igazolja; a korábbi 0.25-ös beállítással szembeni kontrollált fizikai összehasonlítást ezekből a futásokból nem lehet bizonyítani.

A hosszú megállások két külön problémát fednek fel: az első futás **L11 encoder-feedback FAULT** miatt véget ér; a másodikban először **command-renewal/reader kiesés**, később **egy átmeneti navigation hold téves host-terminalizálása** állítja meg a viselkedést. Nem 120 másodperces mission-completion történt.

## Felhasznált evidence és határok

Az authority a két végleges MCAP, a már meglévő explicit MCAP Evidence Compiler bundle és a natív Replayer. Nem készült új capture, nem indult élő robotmozgás, a meglévő bundle-ök nem változtak.

- `runtime/captures/v3_20261016_192129_2645_capture.mcap` és az azonos nevű `.evidence` könyvtár. SHA256: `78ce846d7da89c88df1899fca1dc563d1da3b5764b8e5ffb8fb3511635397ee5`.
- `runtime/captures/v3_20261016_192341_2645_capture.mcap` és az azonos nevű `.evidence` könyvtár. SHA256: `fda773c33497e118abf1d0dd4585ed1fd02959c47c5b63e3a3ad684b15bedd1d`.
- Meglévő teljes DIAG: `runtime/diag/diag_20261016T172714.881276Z_full_v3_20261016_192129_2645_capture_y51y7ww6.json`, illetve `diag_20261016T174249.483448Z_full_v3_20261016_192341_2645_capture_peqxzihd.json`.
- CPU2: **mindkét futással párhuzamosan** készült `runtime/cpu/cpu2_20261016_192132*`, illetve `cpu2_20261016_192343*`.
- Kiegészítő, nem capture-authority runtime termination/timing riport: `/tmp/r2b4-runtime.vjopzcb8.log`, `/tmp/r2b4-runtime.zmd37k0u.log`.

Mindkét EVI `compiler_status=COMPLETE`, `source_integrity=COMPLETE`, quarantined/unrecoverable üzenet **0**. Ez a mentett adatok integritását jelenti, nem automatikusan teljes replayt vagy sikeres robotfeladatot. A 10 Hz szándékosan mintavételezett, a production runtime célja mindkettőben **50 Hz**.

| Mérőszám | 19:21 / 10 Hz capture | 19:23 / 50 Hz capture |
| --- | ---: | ---: |
| Tick0 → utolsó captured tick | 31.577 s | 79.724 s |
| Production utolsó tick / mentett tickek | 1522 / 317 | 3518 / 3519 |
| Teljes MCAP méret | 5,470,827 byte | 235,294,381 byte |
| Checkpoint / raw LiDAR | 0 / 0 | 79 / 790 |
| L12 ALLOW / STOP / FAULT minták | 285 / 31 / 1 | 1953 / 1566 / 0 |
| L11 saturation minták | 0 | 0 |
| Becsült lokális úthossz | 7.913 m | 15.367 m |
| L9→következő L3 lineáris RMSE | 0.0368 m/s | 0.0434 m/s |
| L9→következő L3 szögsebesség RMSE | 0.1131 rad/s | 0.1201 rad/s |

A DIAG RMSE a mintázott L3 becslést hasonlítja a korábbi L9 kéréshez; nem független ground truth, nem motorhibadiagnózis. A fordulások, gyorsítások és a különböző mintavétel miatt a két átlag nem használható önmagában gain-hangolási döntésre.

## Első hibás boundary és a megállások idővonala

Az alábbi `+t` a capture első production tickjéhez viszonyított monoton idő, nem a behavior-start faliórája.

### 19:21: friss encoder snapshot mellett megszűnő fizikai edge-evidence

`tick 156 / +3.118 s`: L5 `EXPLORE/ACTIVE`. `tick 1522 / +31.577 s`: az első és utolsó captured FAULT, **L11**, `ValueError: L11 feedback remained uncertain too long: MISSING_EDGE:left;MISSING_EDGE:right`.

A konkrét `WheelFeedbackEvidence`:

- ACCEPTED encoder sequence **1603**, measurement timestamp **886763066697 ns**, a tick **886765971702 ns**; tehát az input **2.905 ms** régi, `measurement_stale=false`, `measurement_timing_valid=true`, trust mindkét oldalon 1.0.
- Utolsó elfogadott bal/jobb edge **886507574283 / 886506859041 ns**, így az edge-ek kora **258.397 / 259.113 ms**. Mindkét `motion_live=false`, `velocity_feedback_ready=false`, timebase `TICK_SNAPSHOT`.
- L10 target **0.188215 / 0.568001 m/s**, a feedback bizonytalansági határ **886756859041 ns**, az aktuális tick ezt **9.113 ms**-mal túllépi. A captured maximum uncertainty **250 ms**.
- L12 helyesen `FAULT`, disabled, mindkét motor-output 0. A host `BEHAVIOR_FAILED/RUNTIME_FAULT` eseményt küld. Runtime termination: `FAULT_SAFE_LOW`, `operator_stopped=false`.

Ez **nem frissességi/TTL hiba**: a friss encoder-snapshot jelentése az, hogy a fizikai élbizonyíték nem folytatódott. A 10 Hz compact capture nem tartalmaz raw encoder/pulse diagnostics-t, ezért a tényleges mechanikai megakadás, GPIO edge-vesztés és az encoder owner késése ebből nem választható szét. A safety watchdog lazítása nem megalapozott.

### 19:23: átmeneti command-renewal/reader kiesés

`tick 171 / +3.438 s`: L5 `EXPLORE/ACTIVE`. `tick 1276 / +30.863 s`: már a lezárt command input **STOP**, ID `resident.mailbox.expired.272.1276`; ebből L5 `CANCELLED`, majd `IDLE`, L12 `STOP/NOT_ACTIVE` következik. Az első boundary tehát a canonical command-ingress, nem L8/L11/L12 mozgáskorlátozás.

Az **aktív captured TTL 500 ms**, heartbeat célütem 100 ms. Rev272 issue **1016883620393 ns**, reader receipt **1016907446489 ns**. A később észlelt rev274 issue **1017083699265 ns**, receipt **1022046999332 ns**: **4.963 s** késés. Rev275 issue **1022083437985 ns**, vagyis a rev274→275 issue-gap **5.000 s**. A tickek eközben tovább futnak. `tick 1509 / +35.539 s` azonos room-cruise commanddal újra ACTIVE: a command miatti állás **4.675 s**.

Ez egyszerre bizonyít hosszú időrést az észlelt producer issue-időkben és a reader-visible deliveryben; a `reader_received_ns` nem különíti el a publisher atomic replace előtti elakadást, filesystem I/O-t, reader schedulinget és GIL-várakozást. A producer/reader syscall-trace és külön scheduler-evidence hiányában nem állítható, hogy kizárólag CPU, IPC vagy egy konkrét Python layer okozta. A TTL-t megőrizve kell a host/command-runtime oldalt mérni.

### 19:23: rövid freshness-holdból tartós host-leállás

`tick 2316 / +55.539 s`: L6 `IDLE/LOCALIZATION_HOLD`; `tick 2333 / +55.877 s`: L6 `INVALIDATED/LOCAL_COSTMAP_STALE`. Ez **6 tick / kb. 0.12 s**, a mission mindvégig ACTIVE. `tick 2339 / +55.999 s`: L6 már újra `ACTIVE/LOCAL_GUIDANCE_REVALIDATED`.

Az eredeti `r2b4_orchestration/behavior_system.py::_MissionProgram.step` az `INVALIDATED/LOCAL_COSTMAP_STALE` státuszt terminal FAILED-ra fordította. A capture `msg-000000004467` eseménye **+57.535 s**-nél `BEHAVIOR_FAILED/LOCAL_COSTMAP_STALE`. A host a heartbeatet leállítja és canonical STOP-okat küld. Már `tick 2359 / +56.448 s`-nél lejár rev479 (`resident.mailbox.expired.479.2359`), később explicit `operator-stop-*` parancsok érkeznek. Az utolsó **23.276 s** mission-inaktív; ez kívülről úgy látszik, hogy a robot a 120 s előtt végleg abbahagyta.

Az esemény publication-ideje, a status mérési ideje és a host action ideje különbözik: a hibás host FAILED tény és az utána tartós STOP igazolt, de az első lejáró heartbeathez képesti pontos host scheduler-sorrendhez nincs producer execution trace. A két okot nem szabad egyetlen „navigation deadlock” állítássá összevonni.

A source-javítás kizárólag a **WORLD_STALE / LOCAL_COSTMAP_STALE** recoverable INVALIDATED eredményeket tartja host ACTIVE állapotban, változatlan mission identityvel. L6 továbbra sem kér mozgást stale geometriával; a safety, terminal mission/target/contract hiba, host duration limit és explicit STOP továbbra is érvényes. Ez nem automatikus újraindítás.

## CPU, GIL, transport és megfigyelői terhelés

| Runtime / CPU2 mérés | 10 Hz capture + CPU2 | 50 Hz capture + CPU2 |
| --- | ---: | ---: |
| Runtime period átlag / p99 / maximum | 20.747 / 39.1 / 78.296 ms | 22.667 / 51.9 / 122.822 ms |
| Period >40 ms | 8 / 1522 | 129 / 3517 |
| Control wall p99 | 24.0 ms | 33.9 ms |
| L1–L12 pipeline wall p99 / thread-CPU p99 | 23.8 / 14.4 ms | 33.6 / 14.0 ms |
| L0 snapshot wall maximum | 0.153 ms | 1.269 ms |
| Command snapshot wall maximum | 2.039 ms | 3.604 ms |
| CPU2 main-task runqueue wait | 2.389 s / 22.05% | 10.765 s / 37.32% |
| CPU2 által mért runtime process CPU átlaga, egy maghoz mérve | 43.45% | 49.52% |
| CPU2 magok átlagos busy tartománya | 36.0–41.9% | 56.4–60.1% |
| Maximum hőmérséklet / throttle flags | 52.9°C / none | 56.75°C / none |

Mindkét configban `affinity.enabled=false`, a control és diagnosztika az összes magon futhatott. Az 50 Hz capture saját process CPU-ja **55.338 s / 89.045 s elapsed** (kb. 62% egy mag), 235 MB outputtal. Nincs capture-loss: required subscription high-water 194/4096, lost 0, raw loss 0.

A nagyobb megfigyelési terhelés mellett lényegesen több wall-time/scheduler késés látható, miközben a pipeline thread-CPU p99 nem nőtt. Ez összhangban van observer contentionnel; **nem izolált A/B teszt**, mert mindkét futás CPU2-vel ment, a pálya és a missionidőtartam is eltér. A CPU2 monitor saját CPU-/I/O-költségét ezek a runtime-process mutatók nem mérik külön; a monitor nélküli kontroll hiányzik. A wall-minus-thread-CPU nem GIL-specifikus mérés. L0 nem végez több másodperces blokkoló sensor I/O-t ebben az evidence-ben; a motor-write 50 Hz wall maximum 10.905 ms, thread-CPU maximum 0.229 ms. A jelentés nem tulajdonítja ezt automatikusan GPIO/IPC/GIL oknak.

Az EVI/DIAG fordítási riportok **a futások lezárása után** keletkeztek (19:27, illetve 19:38–19:42 körül), ezért nem magyarázzák a 19:24 közbeni megállást. A futás alatti capture és CPU2 költsége ettől külön kérdés.

## Navigáció és hangolási sorrend

A 10 Hz capture L8 mintái: 59 egyenes előre, 184 kanyarodó előre, 42 pivot. Az 50 Hz futásban 341 / 1508 / 83, plusz 1587 stop. Aktív mission alatt mindkettő sokkal többet kanyarodik, mint egyenesen megy; a hosszú 50 Hz stopok jelentős része viszont **inaktív mission**, ezért nem navigációs holtpontként értékelendő.

50 Hz DIAG: lokális úthossz **15.367 m**, nettó elmozdulás **2.420 m** (arány 0.157), 46 különböző mentett local-goal koordináta. Ez jelzi a sok visszakanyarodást/terület újrajárását, de szobafelfedezésnél a nettó elmozdulás nem azonos a lefedettségi hatékonysággal, és L3 becslés, nem külső helymeghatározás. Capture végén L6 progress 0.0508, a korábbi mission-reset miatt a két futás progress értéke sem közvetlenül összevethető.

Javasolt, elkülönített offline kísérletek:

1. Mozgás: `angular_velocity_gain=0.0` baseline; azonos egyenes, pivot és ív szcenárióban a heading/cross-track korrekciót, omega-limitet, L9 gyorsuláskorlátot és kerékmodell-aszimmetriát egyszerre egy dimenzióban változtatni. Encoder watchdog és safety limit változtatása nem hangolási megoldás a fenti FAULT-ra.
2. Room Cruise: a jelenlegi local-goal tartomány **0.40–0.55 m**, novelty/clearance/forward/distance súly **0.30/0.25/0.05/0.05**. Offline összehasonlításban forward súly 0.05→0.10/0.20 és goal-táv 0.55→0.75/1.00 m kísérletezhető, a clearance és fresh-geometry ellenőrzések megtartásával. Ez kísérleti rács, nem bizonyított új alapkonfiguráció. Mérni kell a coverage-t, visszatéréseket, célváltásokat, aktív mission-stallt, pivot-időt és gyorsulási ugrásokat; ugyanazzal a világgal/seeddel.
3. Follow Person: külön fresh→occluded→lost→reacquired és target-generation/id váltó szcenáriók, távolság-, szög-, sebesség- és minimum-keréksebesség-határokkal. Ezek a capture-ök nem tartalmaznak futó person-képességet: a kamera/detektor UNKNOWN/degraded volt, ezért Follow fizikai követési minősége nem értékelhető.

## Elvégzett validáció és nyitott kérdések

- Natív Replayer: 10 Hz capture **elutasítva**, `CAPTURE_INCOMPLETE: replay core is incomplete`. Szándékos mintavételi hiány és checkpoint nélküli compact input; ezt nem jelöljük MATCH-nek.
- Natív Replayer: 50 Hz **1270–1280**, L1–L12, warmup 1261–1269: **MATCH**, `first_divergence=null`.
- Natív Replayer: 50 Hz **2310–2370**, L1–L12, warmup 2275–2309: **MATCH**, `first_divergence=null`. Lefedi a localization/stale holdot, resume-ot és a második ingress-expiry kezdetét. Nem teljes 79.7 s replay.
- A bővített `motion_performance` DIAG mindkét meglévő bundle teljes ellenőrzésével **PASS**. A canonical L5 lifecycle és a tickenként változó expired STOP identity szerinti csoportosítás pontosítása után az unchanged sealed bundle-ökön csak manifest verificationnel ismételtük a számítást. A forrásidőből integrált 10 Hz stage-idők: final motor-output aktív **28.302 s**, inaktív mission **3.118 s**, nincs kért mozgás **0.099 s**. Az 50 Hz értékek: **47.840 / 31.389 / 0.460 s**, plusz **0.035 s** L9-nullázás. A leghosszabb egybefüggő inaktív szakasz **23.241 s**, a leghosszabb aktív mission alatti nulla kérés **0.460 s**; az epizódlista nem csonkolt. A CANCELLED/FAILED/COMPLETED is inaktív lifecycle, nem új navigation hold. A 10 Hz utolsó terminal FAULT mintájának nincs utána captured időintervalluma, ezért annak időtartama nem integrálható. Eredeti teljes ellenőrzések: `/tmp/r2b4_20261016_192129_motion_diag.json`, `/tmp/r2b4_20261016_192341_motion_diag.json`; végső számítás: `/tmp/r2b4-continuity-3wvc6wke/192129.json`, illetve `192341.json`.
- Javított host-lifecycle: canonical launcherrel `./r test tests/packs/feature/test_behavior_system.py tests/packs/feature/test_v3_geometry_hold_integration.py`: **37 passed**. Bizonyítja a hold→resume-ot, változatlan mission identityt, duration boundot és a terminal hiba kezelését; élő heartbeat-jitter javulást nem bizonyít.

Nyitott: az első 50 Hz renewal/reader ~5 s kiesés syscall/GIL/scheduler/file-I/O szétválasztása; a 10 Hz fizikai edge-megszűnés oka; külső ground truth szerinti egyenesmeneti és útvonalminőség; a javított host-viselkedés új élő futása; Follow Person valódi szenzoros evidence. Új live mozgás nem történt.

## Elkészült hangolási út és záró validáció

A [paramétertérkép](../MOTION_TUNING.md) megnevezi a konfigurációs ownereket,
a capeket és az árnyékolt minimumokat. A Follow Person host-envelope közös
`behavior.followperson` profilból származik; a mai 0.25/0.30 érték változatlan.
Az L8 heading/cross-track/yaw-rate feedback és correction cap külön-külön
nullázható offline ablationhöz, a production tuningértékek változtatása nélkül.

A [natív szimulációs út](../ROBOT_MOTION_EXPERIMENTS.md#natív-konfigurációs-kísérletek)
`./r tune simulate` paranccsal hat rögzített scenario-t, validált baseline/override
párokat, teljes config snapshotot, MCAP-ot és measurement-time timeline-t ad.
A meglévő canonical command/V3/PWM plantet használja. A coverage mércéje
független a hangolt planner-gridtől; a nyers szintetikus szenzor ugyanaz marad,
miközben a production pontszűrés a konfigurációt fogyasztja. A szimuláció
nem méri a live GIL/IPC/jittert vagy a fizikai szenzorok modellhibáját.

Záró szoftveres evidence:

- `./r test release`: **23 passed**, a kis gate-et és a canonical scenario-kat együtt futtatva.
- DIAG motion-performance: **22 passed**; kapcsolódó DIAG evidence/quality: **12 passed**.
- Host behavior/geometry hold: **37 passed**.
- Új konfiguráció/follow/gain esetek: **19 passed**; kapcsolódó meglévő config/interface esetek **24 passed**, javított route/follow esetek **6 passed**.
- Végleges szimulátor/korábbi tuner/plant célzott ellenőrzés: **10 passed**, benne motion és occluded FOLLOW natív replay **MATCH**.
- Valódi CLI smoke: három terület baseline/0.25 angular-gain változat, **6/6 replay MATCH**, fault/collision nélkül, végső nulla outputtal. Artifact: `/tmp/r2b4-motion-sla_ky_j/comparison.json`. Ez rövid integrációs smoke, nem teljes hosszúságú behavior-hangolási eredmény.
- `git diff --check`: tiszta. Új fizikai robotmozgás nem történt.
