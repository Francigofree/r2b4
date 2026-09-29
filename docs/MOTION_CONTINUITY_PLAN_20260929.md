# Mozgásfolytonosság — a 2026-09-29-i Room Cruise alapján

Állapot: source- és capture-alapú javítási terv; production változtatás még nincs.
Az irányadó contractok a gyökérben található rendszer-viselkedési, strukturális
és async-runtime contractok. Ez a dokumentum nem új authority.

## Bizonyíték és hatókör

- Room Cruise: `runtime/captures/v3_20260929_155847_15181_capture.mcap`,
  tick 0–783, 22,588 s rögzített tick-időtartomány; 680 EXPLORE tick.
- Idle: `runtime/captures/v3_20260929_160620_16083_capture.mcap`,
  tick 0–541, 10,965 s rögzített tick-időtartomány.
- Mindkettőn új teljes natív `replay_mcap()` futott, a jelenlegi production
  L1–L12 kóddal, a capture konfigurációjával, két determinisztikus végrehajtással:
  **MATCH**, első eltérés nincs; 784, illetve 542 tick.
- Mindkét artifact teljes raw-evidence státusza hiányos:
  `RAW_LIDAR_TRANSPORT_END_MISSING`. A CRC/struktúra érvényes, a natív reader
  szerint a replay-core teljes és replayre jogosult. A MATCH erre a döntési
  scope-ra érvényes; nem bizonyít teljes raw-szenzor-adatfolyamot.
- A replay eredmények és az ideiglenes elemző script helye:
  `/tmp/r2b4-motion-analysis-j6it1x0l/`. Az eredeti helyi capture és evidence
  fájlokat az elemzés nem módosította.
- A terv elkészítése után `./r test`: **42 passed**. Ez a jelenlegi CORE
  baseline; a még meg nem valósított javítások helyességét nem bizonyítja.

Az alábbi sebességek parancsolt vagy becsült értékek. A replay nem igazolja,
hogy a robot fizikailag pontosan ezeket a sebességeket érte el.

## Bizonyított megszakítások és torzítások

| Első releváns tick / szakasz | Konkrét tény | Legkorábbi azonosított határ |
| --- | --- | --- |
| 75 | Encoder–LiDAR konzisztenciahiba 0,035546 m, a 0,035 m küszöb felett; lokális minőség DEGRADED, heading és globális minőség GOOD | L3 minősítés |
| 88 | L9: v=0,119999 m/s, ω=−0,014936 rad/s. L10: két 0,15 m/s-os kerék, ebből v=0,15 és ω=0 | L10 kerékenkénti minimum |
| 257–262, 411–414, 644–647 | `resident.mailbox.expired` → STOP → rearm; ugyanaz a Room Cruise command később visszatér | CommandGateway / command-delivery |
| 266 | Konzisztenciahiba 0,120293 m > 0,12 m; local_translation=LOST, heading=GOOD, lokális pose folytonos, nincs slip jelzés | L3 → L6 recovery |
| 267–272 | LOST alatt az L4 costmap nem frissül. Tick 272-ben már DEGRADED a minőség, a robot-relative scan 173,8 ms, de a costmap 273,1 ms → `LOCAL_COSTMAP_STALE` | L4 integráció → L6 freshness |
| 328 | A tick 316-ból származó terv 350 ms-os élettartama lejár; replacement csak tick 329-ben érkezik | L6 planner completion / guidance lifetime |
| 590–592 | L9 ω: +0,00342 → −0,01491 → +0,00739 rad/s. L10 kerékcélok: (−0,15;+0,15) → (+0,15;−0,15) → (−0,15;+0,15) | L10 az L9 simítás után |
| 687 | A world és a friss scan kora 261,94 ms > 250 ms → `WORLD_STALE` | Szenzorfrissesség → L6 |

Az aktív EXPLORE tickekben L12 végig ALLOW; ez nem jelent folyamatos mozgást.
Az upstream rétegek ALLOW mellett is kérnek nullát:

- 41 tick: `PLANNER_PENDING_NO_VALID_OBJECTIVE`;
- 6 tick: `PLANNER_STALE_HOLD`;
- 14 tick: `LOCAL_COSTMAP_STALE`;
- 1 tick: `WORLD_STALE`;
- további 1 tick: nulla L11-kimenet az irányváltási átmenetben (573).

A három menet közbeni command-kiesés további 14 STOP tick, az aktív EXPLORE
számláláson kívül. A kezdeti és lezáró STOP-okat nem kell mozgáshibának tekinteni.
Tick 733-ban szintén expiry látható, de utána operator STOP és shutdown következik;
ennek operátori szándéka a capture-ből nem dönthető el.

53 aktív tick `LOCALIZATION_REACQUIRE`: előremenet helyett helyben fordulási cél.
A fordulás stabil, 0,2 rad/s-os L9 célja 0,3557 m nyomtáv mellett
±0,03557 m/s kerékcélt jelentene. Az L10 ezt ±0,15-re emeli, ami **0,8434 rad/s**.
Így a minimum már a mission/recovery sebességkorlátokat és az L9 átmenetét is
felülírja. A 590–592 tickekben egy kerék célja egyetlen lépésben 0,30 m/s-ot vált.

Összesen az aktív tickek között:

- 95 tickben az L10 eltérő body twistet valósít meg az L9 kimenetéhez képest;
- 55 tickben a kért kanyarodásból egyenes haladás lesz;
- 33 tickben a kerékcélokból számított |ω| meghaladja a Room Cruise 0,6 rad/s-os capjét;
- 506 tickben az L8 nemnulla előremeneti célja 0,15 m/s alatti;
- 494 tickben legalább egy nemnulla L10 kerékcél 0,15 m/s alatti.

Az `ACCELERATION_LIMIT` jelző jelenléte kikapcsolja az L10 minimumot. Ez a jelző
szögsebesség-korrekció miatt is aktív lehet, miközben a lineáris cél tartósan
0,12 m/s. Emiatt a minimum sem folytonosságot, sem stabil alsó sebességhatárt
nem biztosít.

## Az encoder-korlát következménye

A felhasználói fizikai megfigyelés szerint kb. 0,13–0,15 m/s alatt a keréksebesség
becslése nem megbízható. A jelenlegi lánc ezt nem kezeli megfelelően:

- L6 a 0,30 m/s-os mission maximumot DEGRADED állapotban 0,4-del szorozza:
  **0,12 m/s** lesz a rollout felső sebessége; a minták ennél kisebbek is lehetnek.
- L9 külön platformkorlátozást alkalmaz (0,45 × 0,4 = 0,18 m/s), ami nem szünteti
  meg az L6-ból érkező túl alacsony célokat.
- Tick 140-ben az encoder 0,11979 / 0,08728 m/s-ot közöl, mindkét kerékre
  `measurement_trust=1`, `GPIO_EDGE_HISTORY` alappal. L11 ezt PI-visszacsatolásként
  elfogadja. A pulse/window-alapú trust nem bizonyít sebességtartomány szerinti
  mérési megbízhatóságot.

Külön kell kezelni a sebességbecslés minőségét, a signed kumulatív pulzusszám
integritását, az időzítést és a frissességet. A rossz kis-sebességű becslésből
nem következik automatikusan, hogy a displacement-adat használhatatlan.
A jelenlegi L3 már tartalmaz kumulatív encoder-odometriát és külön local/global
becslést; ezeket kell javítani, nem új odometriai alrendszert felépíteni.

A konzisztenciahiba növekedése bizonyított. A fizikai encoderhiba, slip,
LiDAR-regisztrációs hiba és időillesztési/predikciós hiba közötti oksági megoszlás
ebből még nem bizonyított. A 0,12 m küszöb egyszerű megemelése ezt elfedné.

## Runtime: amit az idle összehasonlítás bizonyít

| Tick start–start mérőszám | Room Cruise | Idle |
| --- | ---: | ---: |
| Átlagos frekvencia | 34,66 Hz | 49,34 Hz |
| Medián periódus | 23,29 ms | 20,00 ms |
| p95 periódus | 57,15 ms | 22,72 ms |
| Maximum periódus | 176,87 ms | 39,77 ms |

A capture aktív affinity konfigurációja `enabled=false`; a jelenlegi configban
is ez szerepel. Ez önmagában nem bizonyítja a késések okát.

A capture `completion_latency` adata publication-késleltetést is tartalmaz;
nem szabad L0–L12 CPU-időként értelmezni. Az aktív/idle különbség nem választja
szét a device I/O-t, control-interpreter/GIL munkát, process transportot,
CPU schedulinget és layer-végrehajtást.

A jelenlegi production wiring process edge-eket és aszinkron command readert
használ. A command TTL alapértéke 200 ms, a heartbeat 100 ms, a reader poll 5 ms,
a megengedett maximum TTL 250 ms. A capture bizonyítja az expiry következményét,
de nem tartalmaz elegendő producer/read/closure időpontot annak eldöntéséhez,
hogy hol késett a következő command revision.

## Javítási sorrend és felelősség

### 1. A realizálható mozgás tervezése és az L10 torzítás megszüntetése

Érintett: `v3/layers/l6_navigation.py`, `l8_motion_realization.py`,
`l9_operational_constraints.py`, `l10_chassis_control.py`, `v3/config.py`.

Egy összefüggő változtatásban kell megszüntetni a késői, kerékenként felfelé
clampelést és feljebb biztosítani a realizálható célokat. A clamp puszta törlése
meghagyná a tartós 0,12 m/s-os mozgást.

- A normál, tartós haladási cél legalább 0,15 m/s. A rollout és az L8 korrekció
  ugyanazt a fizikai keréktartományt és mission/platform envelope-ot használja.
- Előremeneti ívnél, ha mindkét kerék folyamatosan gördül:
  `v - abs(ω) * track_width / 2 >= 0.15`. Például ω=0,2-höz
  v legalább 0,18557 m/s. A 0,15 m/s-os body cél önmagában nem elég a kanyarhoz.
- A planner eleve válasszon ezen belül ütközésmentes v/ω célt. Tartson helyet
  az L8 menet közbeni iránykorrekciójának is; a minimumon tartott egyenes
  célhoz nem marad két gördülő kerékkel kanyarodási tartalék.
- L9 birtokolja a sebesség- és átmeneti korlátokat, beleértve a kerekeken
  jelentkező gyorsulás következményét. L10 legyen pontos differenciálkinematika;
  downstream ne emeljen sebességet és ne változtasson görbületet.
- Indulás, fékezés, irányváltás és célhoz érkezés véges átmenete áthaladhat
  0–0,15 között. Ezt az átmeneti state és a kerékenkénti változás igazolja;
  egy általános `ACCELERATION_LIMIT` bit nem elegendő.
- Explicit STOP és szándékos álló kerék maradjon pontos nulla. Egy 0,2 rad/s-os
  helyben fordulás és két legalább 0,15 m/s-os kerék fizikailag nem egyeztethető
  össze ezen a nyomtávon. A planner válasszon engedélyezett szélesebb ívet,
  vagy csak megfelelő explicit envelope esetén realizálható pivotot.
  Ha nincs biztonságos realizálható mozgás, ne lépje túl a korlátot.

A `conf/speed_map.json` kalibrált pontjai, köztük 0,08 és 0,12 m/s, az átmenetek
feed-forward interpolációjához megőrzendők. A maintenance/startup PWM nem
helyettesíti a sebesség-authorityt. Az `integralando/` csomag jelenlegi tesztjei
éppen a hibás clamp viselkedését is elvárják; nem alkalmazandók újra javításként.

### 2. Sebességfüggő mérési minőség az encoder edge-en, L3-ban és L11-ben

Érintett: `v3/adapters/counter_encoder.py`, `live_encoder.py`, a meglévő typed
encoder evidence, `l3_state_estimation.py`, `l11_actuator_control.py`.

- Konzervatív, kezdetben 0,15 m/s-os megbízhatósági határ; a 0,13–0,15 sáv
  bizonytalan átmeneti sáv. A be- és kilépés kapjon mérési evidence-hez kötött
  hiszterézist, hogy a zaj ne kapcsolgassa a szabályozási módot.
- A kis-sebességű velocity estimate ne kaphasson teljes control-grade minőséget
  pusztán az ablak hossza/pulzusszáma miatt. Az irány, pulse-integritás,
  source time, window és uncertainty maradjon külön vizsgálható.
- L3 a friss és ép kumulatív displacementet továbbra is felhasználhatja;
  a bizonytalan velocity update súlya csökkenjen vagy maradjon ki.
  A mérési intervallumot a hozzá tartozó IMU/LiDAR intervallummal kell összevetni.
- L11 a bizonytalan sebességen ne integráljon PI-hibát és ne értelmezze azt
  biztos állásnak. Véges indulási/fékezési átmenetben maradjon kalibrált
  feed-forward, folytonos visszakapcsolással a megbízható PI-tartományba.
- A normál átmenetet el kell különíteni a hiányzó, stale vagy hibás encodertől.
  Az utóbbi meglévő bounded watchdogja és fail-closed viselkedése megmarad;
  nem lehet korlátlan feed-forward módot létrehozni.

### 3. Lokális minőség és friss geometria folytonossága

Érintett: `l3_state_estimation.py`, `l4_world_model.py`, az L6 recovery.

- A tick 75 és 266 előtti első eltérésig össze kell vetni a nem átfedő
  encoder-displacement, lokális predikció és relatív LiDAR intervallumokat.
  Előbb az esetleges időillesztési/becslési hibát kell javítani.
- Az összegzett konzisztenciahiba, a pillanatnyi lokális megfigyelhetőség és
  a valós folytonosságvesztés külön minősítési evidence legyen. A kritikus
  freshness/integrity/heading-loss azonnali korlát; a puha minőségváltás
  kapjon indokolt belépési/kilépési hiszterézist és friss méréses recoveryt.
- Globális X/Y bizonytalanság ne szennyezze a lokális minősítést. Ez a
  szétválasztás már létezik; EXPLORE és FOLLOW lokális frame-ben fut.
- A friss robot-relative scan feldolgozása maradjon független a globális
  korrekciótól. A jelenlegi `RobotRelativeGeometry` csak scan-lineage token,
  nem teljes ütközésellenőrzési geometria: nem elegendő önmagában navigálni.
- Lokális recovery után az L4 a bounded, megőrzött friss ROBOT_BASE adatból
  és érvényes measurement-time lokális pose-ból építse újra a használható
  geometriát, ha ezek rendelkezésre állnak. Az L2 duplikációszűrését ne
  kerülje meg, és ne dátumozza újra a régi costmapet. Valós lokális LOST
  alatt ne folytasson megbízhatatlan pose-ra épülő előremenetet.

### 4. Replanning közben is frissen igazolt lokális mozgás

Érintett: L6 guidance-élettartam és meglévő planner edge; L7 objective-ownership;
L8 friss lokális realizáció. A nehéz rollout továbbra is workerben marad.

- A jelenlegi L7 megtartja az érvényes objective-et pending alatt. Ezt nem
  kell új cache-sel helyettesíteni. Tick 328-ban ténylegesen lejár az authority.
- A terv optimalizálásának életkora és a jelenlegi lokális végrehajthatóság
  igazolása különüljön el. Ha a korábbi irány/útfolyosó továbbra is használható,
  L6 friss lokális odometria és geometria alapján, kis determinisztikus
  work-budgettel új rövid guidance-ot állíthasson elő a replacement előtt.
- Ez új, friss mérésből igazolt guidance, saját source-idővel és lineage-dzsel;
  nem lejárt trajectory/objective timestampjének meghosszabbítása. Az előző
  terv önmagában nem lehet új pozitív authority.
- A vizsgálat egy rövid lokális folyosóra/manőverre korlátozódjon. Ha ez sem
  fér a control budgetbe, a meglévő capability-specifikus planner edge-en
  számítandó; nem kell új általános scheduler vagy IPC-rendszer.
- Mission/odom-generation váltás, explicit STOP, friss akadály vagy valódi
  lokális evidence-vesztés továbbra is megszakítja a mozgásengedélyt.
- Globális NAVIGATE esetén a meglévő lokálisan végrehajtható szakasz és a
  globális útfrissítés külön kezelendő. A mai globális minőségkapu recoveryt
  kér; a kívánt továbblépéshez célzott contract-módosítás is szükséges.
  Bizonytalan global transformból nem szabad új lokális célt kitalálni.

### 5. Command delivery és control budget célzott rendezése

Érintett: `v3/control_cli.py`, `v3/adapters/resident_command.py`,
`v3/composition/resident_live_control.py`, a production runtime/edge wiring.

Első lépésként kis scalar időmérések kellenek az alábbi meglévő határokon:

- command kiadás/revision, reader receipt, closure-kori láthatóság és TTL-maradék;
- planner submit, worker start/finish, collector receipt, closure;
- sensor measurement → publication → closure age;
- L0 closure, L1–L12 wall/CPU duration, final motor-write, observation enqueue.

Ezek különítsék el a fizikai I/O-, GIL-, transport- és scheduler-költséget.
Nagy új payload vagy a control útban történő logformázás ne kerüljön be.

Mérés alapján kell a szűk edge-et javítani: heartbeat ütemezés/delivery,
bounded serialization, vagy indokolt CPU-elhelyezés. A producer már külön
folyamat, a reader kis thread; a thread puszta processbe költöztetése és az
affinity bekapcsolása önmagában nem igazolt javítás. TTL-emelés és stale
command automatikus megújítása nem helyettesíti a delivery rendezését.

A 257/411/644 kieséseknek meg kell szűnniük a támogatott üzemi terhelés alatt;
valódi producerhalálnál/command-expiry esetén a canonical STOP megmarad.

## Elfogadási és validációs terv

1. Az L10 minimumot rögzítő teszteket mozgáslánc-szintű invariánsokkal kell
   kiváltani: L9→L10 twist-egyezés, mission/platform/kerékhatárok, ívkövetés,
   folytonos indulás/fékezés/irányváltás, pontos STOP. A 88 és 590–592 mintájára
   legyen regressziós eset. Ne növekedjen mechanikusan a pytest esetszám.
2. Encoder 0 / 0,08 / 0,12 / 0,13–0,15 / 0,19 m/s evidence, sparse pulses,
   reversal, stale és counter error: a gyenge velocity nem teljes PI authority,
   de az ép displacement nem vész el; az uncertainty időkerete nem nullázható
   váltakozó kerékcélokkal.
3. Room Cruise és Follow: DEGRADED/global LOST mellett friss lokális odometria
   és geometria esetén folyamatos, realizálható legalább 0,15 m/s-os tartós
   haladás. Follow érkezés, túl közeli személy, elveszett target, akadály,
   explicit STOP és valós lokális/heading LOST továbbra is helyesen lassít/áll.
4. Planner késés a régi 350 ms-os határon túl: friss lokális igazolás mellett
   nincs pusztán planner miatti zero gap. Friss igazolás hiányában nincs
   lejárt pozitív actuation. Worker crash/restart, supersede, generation és
   measurement lineage direct/process úton is egyezzen.
5. A mostani két MATCH baseline megőrzendő. A javított viselkedés történeti
   replayben szándékosan eltérhet a capture régi outputjától; az első eltérés
   rétege és oka legyen elvárt. A megváltozott döntés nem nevezhető régi-output
   MATCH-nek. Az új döntéssor ismételhetősége és checkpoint/replay-egyezése
   külön bizonyítandó.
6. Implementáció után először `./r test`, majd az érintett `motion`,
   `localization`, `roomcruise`, `follow`, `async`, `process`, `replay` módok.
   A közös motion contract/config/composition változások végén `./r test full`.
7. A teljesítmény elfogadásához azonos terhelésű motor-output nélküli mérés,
   majd külön felhasználói engedéllyel canonical live próba szükséges.
   Cél a konfigurált 50 Hz üzemi tartása, a 20 ms-os control-budgetből való
   hosszú kiesések megszüntetése, nulla terhelés miatti command-expiry, és
   friss geometria mellett nulla planner-várakozásból eredő megállás.
   A történeti replay nem bizonyít live timingot vagy fizikai simaságot.

## Mi maradt nyitva

A legnagyobb bizonyított vezérlési hiba az L10 utólagos sebességemelése.
A 0,12 m/s-os degradált tervezés, a command-expiry, a konzisztenciaküszöb
körüli recovery-váltás és a planner-élettartam lejárata további konkrét
javítandó pont. A késések komponensenkénti oka és a lokális konzisztenciahiba
fizikai oka még célzott mérést igényel. Új live futás és fizikai robotmozgás
ebben az elemzésben nem történt.
