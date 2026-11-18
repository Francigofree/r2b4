# 2026-11-18: encoder-jelcsend, hibás nulla sebesség és recovery

A mai MCAP-okból a leállító feltétel bizonyítható. Az encoder-impulzusok
eredeti megszűnésének fizikai/driver oka még nem állapítható meg biztosan.
Két külön szoftverhiba javítva: a jelcsend megbízható nulla sebességgé
minősítése, valamint az explicit encoder-recovery terminális L11-hibává
alakítása. Élő teszt, robotmozgás, tuning- és speed-map-módosítás nem történt.

**Vizsgált eredeti felvételek** (`runtime/captures/`, minden fájlnév végén
`_capture.mcap`). A tick az eredeti production tickazonosító.

| Fájlnév eleje | Hz | Utolsó tick | Terminális eredmény |
|---|---:|---:|---|
| `v3_20261118_105743_2996` | 50 | 3345 | L11: `MISSING_EDGE:left;MISSING_EDGE:right`, TELEOP |
| `v3_20261118_110107_3676` | 50 | 1721 | Normál STOP / `NOT_ACTIVE` |
| `v3_20261118_110930_4576` | 50 | 1084 | L11: `MISSING_EDGE:left;MISSING_EDGE:right`, TELEOP |
| `v3_20261118_111414_9657` | 5 | 1995 | L3: `cumulative encoder displacement exceeds physical bound` |
| `v3_20261118_111509_10825` | 5 | 1513 | L11: `MISSING_EDGE:left;MISSING_EDGE:right`, EXPLORE |
| `v3_20261118_111617_11911` | 50 | 1691 | L11: `MISSING_EDGE:left;MISSING_EDGE:right`, EXPLORE |
| `v3_20261118_112957_16531` | 5 | 3104 | L11: `MISSING_EDGE:left;MISSING_EDGE:right`, EXPLORE |

Mind a hét eredeti fájl deep CRC- és szemantikai capture-integritás ellenőrzése
PASS; a kért scope teljes. Ez külön állítás a robot FAULT eredményétől.
Az `111617` és `111414` felvétel explicit MCAP Evidence Compiler exportja
`COMPLETE`, forrásintegritása `COMPLETE`, a kötelező verification sikeres.
Az 5 Hz-es behavior-log nem tartalmaz teljes tick-inputot/checkpointot;
exact replayre nem jogosít. A CLI `c full` alapértelmezésben 5 Hz-es folyamatos
behavior recorder; a teljes sensor-debug/exact-replay scope az explicit `c 50`.

**A szakvélemény ellenőrzése.** A `112957` felvétel pontos értékei:

| Tick | L10 bal/jobb cél, m/s | L3 lokális v, m/s | L3 lokális omega, rad/s | L11 bal edge-kor | Elfogadott EKF-korrekció |
|---:|---|---:|---:|---:|---|
| 3084 | 0,540 / 0,285 | 0,518434 | −0,761411 | 110 ms | e tickben nincs új korrekció |
| 3094 | 0,422 / 0,237 | 1,28e−8 | −0,661054 | 306 ms | VELOCITY és ZUPT |
| 3104 | 0,338 / 0,249 | 1,43e−16 | −0,508338 | 506 ms | VELOCITY és ZUPT |

Mindhárom tick L11-evidence-e `TICK_SNAPSHOT` időalapot jelöl. A két utolsó
tickben a jelentős gyro-alapú forgás mellett elfogadott ZUPT igazolja a
szakvélemény szemantikai hibára vonatkozó állítását. A haladási sebesség nulla
volta önmagában összeegyeztethető középpont körüli forgással; az encoder-csend
nem igazolta ezt a nulla mérést, a forgás pedig nem igazolt chassis-állóhelyzetet.

A részletesebb `111617` felvételben a 1666. ticktől a két count 7993/8286
értéken marad. Az utolsó elfogadott fizikai élek ideje
2197154642012/2197154579141 ns; a két utolsó callback receipt ideje
2197155859184/2197155848073 ns. Az acquisition sequence tovább nő, a generation
0, a health OK, a timing valid, L12 ALLOW és pozitív motor-output marad a
watchdogig. Az utolsó callbackek késése kb. 0,9 ms. Az 1691. tickben az élek
kora kb. 516 ms, ekkor következik az L11-hiba és a nulla final output.

Ez korábbi safety STOP nélkül bekövetkező encoder/callback-jelcsend. A rögzített
control-period maximuma e futásban kb. 79 ms; a timing-adatok nem igazolnak
500 ms-os összefüggő control-kiesést. A callback-adatút külön torlódása vagy
hibája ettől még lehetséges. Az I/O, GIL-várakozás és CPU/scheduler-verseny
közül egyik sem nevezhető bizonyított gyökérokúnak e capture alapján.
Nincs a PWM fizikai jeléről, hajtástápról vagy független kerékforgásról olyan
időben illesztett mérés, amely a jelcsendet egyértelműen megmagyarázná.

**Source-first ok és változtatás.** A production út:
`_encoder_process_main` → `NativeCounterEncoderBackend` →
`NativeEncoderSource` → L2 → L3/L11 → L12.

- `counter_encoder.py`: az egykor `_stationary_estimate` nevű ág új impulzus
  nélkül idővel `trust=1` értéket adott a nulla skalárnak. A javított
  `_no_edge_estimate` `trust=0` értéket ad; a nulla unavailable placeholder,
  a rejection `BASELINE`. A count, fizikai edge-idő, acquisition-idő és sequence
  megmarad. A valid count-observation és a velocity-fit minősége külön tény.
- `l3_state_estimation.py`: a nem GPIO-edge alapú minta acquisition-ideje többé
  nem minősül új velocity evidence-nek. Native sebességkorrekcióhoz mindkét
  fizikai fit-végpontnak előre kell lépnie. Hiányos native metadata nem kerülhet
  a velocity-only kompatibilitási ágra. ZUPT-hoz friss, megbízható, az eddigi
  stationary küszöb alatti gyro-rate is kell. Jelcsendnél predikció és annak
  bizonytalansága marad; nem készül kitalált nulla mérés.
- `l11_actuator_control.py`: a PI eddig is elutasította a `TICK_SNAPSHOT`
  nullát nem nulla igénynél; ez a viselkedés és az eredeti, fizikai edge-időből
  számított véges watchdog megmarad. Az explicit
  `DEGRADED / ENCODER_RESTARTING`, sample nélküli állapot azonban most nulla
  kérést ad, és törli a régi feedbacket, PI-t és actuation-episode állapotot.
  Korábban a cached feedback invalidálása itt azonnali kivételt/L11_ERROR-t
  okozott. L12 ugyanebből a lezárt healthből fizikailag inaktív STOP-ot ad,
  L5 megtartja a missiont. Restarting health és egyidejű sample ellentmondása
  továbbra is FAULT.
- `process_encoder_backend.py`: az owner az lgpio notification thread tényleges
  életét ellenőrzi mintavétel előtt/után. Bizonyított thread-halál explicit EIO,
  majd a már létező, legfeljebb három próbás, új mailboxot/generationt létrehozó
  recovery következik. Élő thread puszta jelcsendje nem reset-indok. Readiness
  az új counter-acquisition érvényessége; nem követel kitalált, teljes bizalmú
  nulla velocity-fitet. Az ismeretlen lgpio thread API fail-closed hiba. A
  telepített `/usr/lib/python3/dist-packages/lgpio.py` `_notify_thread` objektuma
  a notificationt futtató Python Thread; a production owner továbbra is spawn
  folyamatban importálja a drivert.
- Passzív evidence: L11 countok, callback-receipt/késés, thread-liveness és
  source-health skalárokat is rögzít. Az owner egy bounded hibasort ír saját
  stderr-jére a supervision értesítése után. Az L3 physical-bound hiba most
  tartalmazza az előző mérés idejét, időközt, két elmozdulást és limitet.

Az `111414` külön L3-hibájának konkrét delta/időértékei az eredeti 5 Hz-es
felvételben hiányoznak. A fizikai bound lazítása nem indokolt. A callback-thread
halálának korábbi bekövetkezése sem bizonyított; a felügyeleti hiány source-ból
igazolt és hibainjektálással ellenőrzött javítás.

**Validáció és replay.**

- `./r test release`: 28 PASS, a kötelező kis robot-contract gate-et is tartalmazza.
- A célzott encoder, L3/L11, stationary, recovery, capture/evidence és natív
  replay tesztcsoport: 78 PASS. A végső metadata/gyro guardokhoz tartozó célzott
  csoport: 32 PASS; a sample + restarting negatív eset is PASS. Nincs új
  permanent gate-/scenario-bejegyzés.
- Motor nélküli hibainjektálás: halott notification thread → DEGRADED, sample
  nélkül → új owner, új source sequence/generation; régi mailbox nem jut át.
  Recovery-exhaustion terminális marad. A quiet, élő owner nem indul újra csak
  azért, mert nincs kerékimpulzus.
- A canonical aktív mission 720 ms-os encoder-recovery alatt STOP-ot ad,
  megtartja az identityt, régi feedback nélkül folytat friss inputból; a
  rögzített recovery-sorozat natív replayje MATCH. Az új szintetikus
  pulse-loss/capture/replay út is MATCH; a hamis VELOCITY/ZUPT hiánya és az
  L11-watchdog megmaradása külön tesztelt.
- Az eredeti `111617` capture 1640–1691 tickes szelete a javítás előtt MATCH.
  A recovery/diagnosztikai változtatás után még MATCH. A szemantikai javítással
  MISMATCH: az első eltérés tick 1674, L3, `covariance_5x5[0]`
  (rögzített 0,03319221186261877, javított 0,033272710538152124).
  Ez a régi téves nulla korrekció elhagyásának szándékos következménye.
  Ugyanebben a szeletben L4–L12 összehasonlított outputjain nincs eltérés;
  a végső L11_ERROR megmarad. Az offline ismételt trace determinisztikus.
  A régi hibás trace-re nem állítunk javítás utáni exact MATCH-et.

Az offline artifactok: `tmp/stop_rootcause_20261118/`: `integrity.json`,
`111617.evidence/`, `111414.evidence/`, `expert_claim_evidence.json`, a két
replay előtti/utáni eredmény és a tesztnaplók. Eredeti MCAP nem lett átírva.
A közben módosult `runtime/*` állományok megmaradtak; runtime-indító parancsot
e vizsgálat nem adott ki.

**A még nyitott gyökérok felderítésének terve; most nem végrehajtva.**

1. Először motor-output nélküli, canonical STOP/IDLE állapotú vizsgálat kell:
   a teljes sensor-debug recorder mellett külső, ellenőrzött A/B-jellel vagy
   kézi kerékforgatással megfigyelni a fizikai jelet és a callbacket. A motor
   leválasztása és a jelforrás bekötése kézi hardveres művelet. Ezt külön kell
   előkészíteni; tetszőleges GPIO-/motor-kerülőút nem használható.
2. A megszűnés legelső időpontján egymás mellé kell illeszteni a fizikai A/B
   jelet, `notification_thread_alive`, callback receipt/késés, raw count,
   counter generation, measurement sequence/idő és L12 final output értékét.
   Fizikai jel megléte callback nélkül a driver/notification ágra mutat.
   Callback mellett változatlan signed count a quadrature-admission ágra
   mutat. Mindkét fizikai jel megszűnésekor független kerékforgás-, PWM- és
   hajtástáp-mérés választja el a megálló hajtást a hibás encoder-jeltől.
   Élő, de elakadt notification threadhez produceroldali thread-stack és
   scheduler/CPU mérés kell; a control wall-minus-CPU idő önmagában nem elég.
3. Az L3 bound következő előfordulásánál a most kibővített első hibasorból
   ellenőrizni kell a `delta / dt` értéket és a GPIO fizikai edge-/receipt-időket.
   Késve egyszerre beérkező count, irányváltáskor commitolt count, kvantálás és
   ténylegesen irreális jel külön hipotézis. Csak a konkrét első hibás
   interval ismeretében választható javítás.
4. Későbbi, külön engedélyezett mozgásos acceptance: azonos speed map és
   beállítások mellett rövid ellenőrzés, majd egy megszakítatlan
   `./r rc 300 c 50` futás. Elfogadás: végigfutó mission, új FAULT/process-death
   nélkül, nem kívánt STOP/HOLD/megtorpanás nélkül, megőrzött mozgásminőséggel
   és teljes MCAP-pal. Váratlan safety/health/timing eredménynél nincs automatikus
   ismétlés. A speed-map sweep csak ezután következzen; az továbbra is megszakad
   valós safety STOP/FAULT esetén, és a hibás szakaszból nem készülhet kalibráció.

Az ötperces fizikai room cruise és a sikeres live speed-map regenerálás még
nem bizonyított. A javítás nem helyettesíti az impulzusok eredeti eltűnésének
felderítését. Valós állóhelyzetben is maradhat velocity-bizonytalanság, ha az
encoder-csenden kívül nincs független állóhelyzeti evidence; ezt a rendszer
többé nem tünteti el mesterséges ZUPT-tal.
