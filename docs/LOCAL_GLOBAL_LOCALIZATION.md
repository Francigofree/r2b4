# Lokális–globális lokalizáció: fejlesztési állapot

2026-09-28. Ez megvalósítási terv és validációs leírás; az architekturális
authority továbbra is a három gyökérkönyvtári V3/rendszercontract.

## Kiinduló hibák és megvalósított lépés

| Terület | Konkrét probléma | Megvalósítás |
| --- | --- | --- |
| P0, L3 heading | Friss gyro és 10 Hz-es relatív LiDAR mellett 6,02 s után `LOST`, mert az abszolút yaw kovarianciája nő. | A relatív heading minősége külön becslést kap a regisztráció megfigyelhetőségéből, az időben halmozott yaw-eltérésből és az utolsó érvényes mérés óta növő bizonytalanságból. |
| P0, L3 encoder | A sebességillesztés `BASELINE` eredménye az ép számlálóalapú odometriát is letiltotta; az aszimmetrikus részleges sebesség hamis slipet jelezhetett. | A friss, működő számlálók kumulatív elmozdulása önálló helyi bizonyíték. Csak megbízható sebesség használható sebességkorrekcióhoz, wheel-rate fallbackhez és wheel/gyro slip-vizsgálathoz. |
| P1, L4 memória | A lokális strukturális memória globális `GOOD` állapotot követelt. | Tanulása és felidézése a helyi transzlációtól, headingtől, folytonosságtól és varianciától függ. A globális coverage külön marad. |
| P1, L6 folytonosság | Egy replacement timeout az előző, még érvényes trajectoryt is törölte. | Csak a sikertelen replacement szűnik meg. A régi pálya eredeti lejáratáig folytatható; a timeout nem hosszabbítja meg. |

Az abszolút EKF-kovariancia és a lokális pose nem kap mesterséges korrekciót a
relatív ellenőrzéstől. A heading bizonytalansági modell az eddigi konfigurált
yaw/rate/process zajokat használja; fizikai kalibrációja még szükséges.
Kis, tartós yaw-eltérés előbb `DEGRADED`, majd `LOST` állapothoz vezet.
Hiányzó vagy degenerált relatív regisztráció nem frissíti az érvényességet;
az öregedés measurement time alapján történik. Az új yaw-konzisztenciaállapot
az L3 checkpoint része.

A `BASELINE` felmentéshez explicit kumulatív elmozdulás és két működő számláló
kell. Hibás timing, stale mérés, device-degradáció és hard rejection továbbra
is megszünteti a helyi odometriai authorityt. A bizonytalanság a tényleges
elmozdulással nő akkor is, ha még nincs használható sebességillesztés.
Az L11 kerékenkénti feedback watchdog változatlanul önálló követelmény.

## Acceptance

Validáció: `./r test` 32/32, `./r test localization` 21/21,
`./r test roomcruise` 5/5, `./r test replay` 2/2,
végső `./r test full` **193/193 sikeres** (75,07 s).

- 60 s rate-only gyro, késleltetve érkező 10 Hz-es relatív LiDAR és végig
  `BASELINE` encoder, checkpointból azonos folytatással.
- IMU-/encoder-kiesés, stale és hibás timing, leállt számláló, hard rejection,
  hiányzó kumulatív bizonyíték, degenerált/hiányzó relatív regisztráció és
  halmozódó yaw-eltérés; relatív evidence visszatérésekor helyreállás.
- Natív counter backend → encoder adapter → L3 ellenőrzés: a részleges
  sebességillesztés alatt is helyes kumulatív elmozdulás; a hosszú, ellenőrizetlen
  elmozdulás bizonytalansága nem tűnhet el a hiányzó sebesség miatt.
- L4 memória tanulása és felidézése globális fix nélkül, globális minőségváltás,
  helyi minőségromlás, lejárat és checkpoint.
- EXPLORE/FOLLOW_PERSON/relatív NAVIGATE: pending és timeout alatt érvényes
  trajectory, változatlan lejárat, lejáratkor stop, checkpointból azonos kimenet.
- 601 s zárt, 50 Hz-es L1–L12 szimuláció: rate-only gyro, kvantált kumulatív
  encoder, ismétlődő sebességillesztési szünet, 10 Hz-es helyi scan, ritka globális
  korrekció; a végső szakasz natív replaye checkpointból.

Mért szintetikus eredmény: 30 051 tickből 30 048 mozgással, 29 423 globális
`LOST` tick, 1204 encoder `BASELINE` tick, **0 lokalizáció miatti stop**,
3 elfogadott globális fix. Maximális lokális lépés 7,56 mm, maximális
encoder/LiDAR-eltérés 7,73 mm. Az utolsó 50 tick checkpoint-replaye `MATCH`,
ismételt trace-egyezéssel. Fizikai mozgást ez a fejlesztési kör nem indított.

A meglévő `v3_20260928_210045_16471_capture.mcap` régi kimeneteihez képest
az első eltérés a 14. tick `L3.localization_quality.yaw_sigma_rad` értéke
(0,130275 → 0,100173). A 0–210. tick ismételt natív replaye determinisztikus,
de a rögzített baseline-hoz `MISMATCH`. A teljes régi capture replaye a 211.
ticknél `PLANNER_PURE_RESULT_MISMATCH` miatt megáll: a módosult L3/L4 inputból
számolt planner-eredmény nem egyezik a régi rögzített completionnel. Ez nem
teljes fizikai acceptance és nem régi-capture `MATCH` bizonyíték.

## Következő, méréssel megalapozandó lépések

1. **LiDAR latency/freshness:** a helyi scan/safety már a raw scan ágból jön,
   a relatív eredmény publikációja nem követel globális pose-sikert. A relatív
   regisztráció és a globális matcher viszont ugyanazon workerben számol
   (`v3/lidar_matcher_process.py`). A meglévő fizikai capture 36/346 matcher
   timeoutot jelez. Külön kell mérni a scan mérési korát, worker-futásidőt,
   queue/delivery időt és control-closure késést, mielőtt deadline-t vagy
   végrehajtási elhelyezést változtatunk.
2. **Lokalizációt megőrző tervezés:** L6-ban már van `DEGRADED` sebességskálázás,
   uncertainty inflation és bounded reacquire. A következő bővítés az irányfüggő
   megfigyelhetőség szerinti trajectory-scoring; ehhez előbb hiteles geometriai
   evidence és folyosó/fordulás acceptance szükséges. Heading `LOST` mellett a
   vak forgás nem elfogadható helyreállítás.
3. **Fizikai acceptance:** külön engedélyezett mozgással kell igazolni a zajmodell,
   slip-küszöbök és L11 feedback-tolerancia megfelelőségét, valamint a valós
   stop–start gyakoriság és control timing javulását. A szintetikus teszt nem
   bizonyít abszolút térképi pontosságot vagy közös szenzorbias elleni védelmet.

## 2026-09-29: a 08:40 és 08:42 Room Cruise megszakításai

A vizsgálat authorityja a `v3_20260929_084011_9120_capture.mcap` és
`v3_20260929_084207_9850_capture.mcap`, illetve az aktuális production source.
A meglévő capture/evidence fájlok változatlanok maradtak. A `DEVICE_HEALTH_NON_OK`
incidentek optional kamera/person-detector állapotai nem magyarázzák a
megtorpanást: az aktív szakaszokban L12 `ALLOW` mellett L6/L8 kért nullát.

| Rögzített tény | 08:40 | 08:42 |
| --- | ---: | ---: |
| Aktív tick | 453 | 846 |
| Mozgást kérő tick | 236 | 228 |
| `PLANNER_STALE_HOLD` | 18 | 112 |
| `LOCALIZATION_HOLD` | 185 | 464 |
| Átlagos control frekvencia | 44,36 Hz | 41,83 Hz |
| Legnagyobb tick-kezdésköz | 97,38 ms | 243,89 ms |

### Gyökérokok és az elvégzett javítás

1. **L6 számítási költség és a terv élettartama.** Az első planner-megállás
   a 119., illetve 82. tick. A 08:42-es induló kérés a 67. tickből a 78.-ra,
   228,6 ms alatt érkezik meg. A következő eredmény további 173,0 ms múlva,
   a 84. tickben látható; közben a régi terv eredeti 350 ms-os érvényessége lejár.
   A profilban a rollout ismételt rectangle/disc clearance-számítása dominál.
   A módosított L6 a pálya addigi minimumát használja a következő térbeli
   keresés felső korlátjaként. Távolabbi akadály nem csökkentheti ezt a minimumot.
   A direct és worker algoritmus is ezt használja; a pontszám, collision,
   escape-szemantika és a teljes result változatlan. A 14 capture-checkpointból
   származó kérésen a régi és az optimalizált számítás eredménye pontosan egyezik.
   Ezen a fejlesztői gépen az átlagos pure-compute idő 41,70 → 18,87 ms,
   illetve 41,95 → 14,55 ms volt. Ez nem a roboton mért process-latencia.

2. **A relatív LiDAR-regisztráció felületi mintavételezési hibája.** A régi
   point-to-point illesztés a fordulatonként változó sugárfázist és pontszámot
   részben elmozdulásként értelmezte. L3 a sok kicsi eltérést helyesen összegezte,
   de a hibás evidence miatt a 286., illetve 323. tickben átlépte a 12 cm-es
   consistency-határt. Ezután L4 nem integrált új geometriát, és a costmap
   lejárata a recovery forgást is leállította. A javítás a LiDAR workerben
   felületi normálisokra illesztő SE(2) regisztráció. Sarkok/szórt pontok nem
   kapnak megbízható normálist; a trim, fizikai sebességkorlát és measurement
   identity megmarad. Az observability a transzláció mellett a forgási rangot
   is ellenőrzi: párhuzamos folyosó és körfal nem adhat hamis teljes authorityt.
   Encoder/gyro prior nem kerül az ettől független ellenőrző mérésbe.

3. **L3 időben eltérő sebességek összehasonlítása.** A 08:42-es 539. tickben
   a bal oldali régebbi −0,259 m/s-os fit és a jobb oldali +0,123 m/s-os fit
   a friss 0,343 rad/s gyro mellett slipet jelzett. A bal keréknél közben nem
   volt új impulzus. Kumulatív encoder esetén az új ellenőrzés a két kerék
   elmozdulását és a gyro által vezetett helyi heading változását ugyanazon
   mérési intervallumban hasonlítja össze. A szükséges anchor és pose history
   már checkpointolt L3-state; új state/transport nem kell. A velocity-only
   régi inputok meglévő ellenőrzése megmarad. Valódi eltérés továbbra is `LOST`.

A javítás nem növeli a planner-, world-, sensor- vagy safety-lejáratokat.
A meglévő L7 ownership, L8 realizáció, L9 gyorsuláskorlát, L10–L11 és az
egyetlen L12 writer változatlan. Nincs új processz, scheduler vagy authority.

### Ellenőrzés és a bizonyítás határa

A raw scanpárokból az új regisztrációt **az eredeti intervallumokon** újraszámolva,
az eredeti L2 admissiont és closure-időket megtartva, a production L3/L4-en
végzett offline kísérletben egyik futás aktív szakaszában sincs helyi translation
vagy heading `LOST`. A maximális helyi sigma 0,1304 → 0,1053 m és
0,1313 → 0,0881 m; a 08:42-es maximális yaw sigma 0,3188 → 0,2173 rad.
A 08:42-es futásban egy valóban megöregedett costmap-tick megmarad.
Ez megváltoztatott inputú kísérlet, nem az eredeti felvétel exact replaye,
és nem a más mozgásból következő fizikai szenzoradatok előrejelzése.

A regressziós esetek valódi, újramintavételezett sugárgeometriát használnak
azonos landmarkpontok eltolása helyett. Lefedik az egyenes/íves mozgást,
felhalmozódó mérési hibát, hiányos/degenerált regisztrációt, időben eltérő
kerékfiteket és valódi slipet, valamint a direct/process eredményegyezést és
stale inputot. Egy új 601 tickes natív L1–L12 szimulációban a regisztráció,
120 ms késleltetett planner completion, ritka 40 ms tick-köz és a helyi
akadálytérkép együtt fut; az indulás után folyamatos haladást és íves fordulást,
majd 50 tickes checkpoint-replay egyezést követel.

A tick-kezdésköz nem azonos az L0–L12 futásidejével. A capture completion-latency
az observation/capture kézbesítés mérése; nem planner- vagy motorlatencia.
A felvételek nem adnak elegendő per-phase evidence-et a fizikai I/O,
GIL/serializáció, process transport és CPU/scheduler contention számszerű
szétválasztásához. Az ezek közötti oksági megoszlás és a roboton elérhető
új timing/motorfolytonosság külön, engedélyezett live mérésre marad.

## 2026-09-29: a 09:29-es futás fennmaradó darabossága

Evidence: `v3_20260929_092919_16279_capture.mcap`, teljes/integritáshelyes
capture, 903 tick. A 798 aktív tickből 620 kér mozgást. A planner korábbi
problémája jelentősen csökkent: egyetlen `PLANNER_STALE_HOLD` maradt (177. tick).
A fő megszakító ok 128 `LOCALIZATION_HOLD`, további 36 reacquire és
11 stale-costmap tick. Az aktív szakaszban L12 végig `ALLOW`.

Az első helyi lokalizációvesztés a 398. tick: az encoder–LiDAR konzisztencia
0,119838-ról 0,120347 m-re nő, heading `GOOD`, slip nincs. L6 reacquire-t kér,
L4 közben nem integrál új pontokat; a megöregedő helyi geometria ezután HOLD-hoz
vezet. Megállás közben az eltérés csökken, majd az újrainduláskor ismét nő.
Ez konkrét lokalizációs stop–start ciklus; a 25,04 ms átlagos és 129,80 ms
maximális tick-kezdésköz önmagában nem bizonyít housekeeping/GIL gyökérokot.

### Igazolt regisztrációs hiba és javítása

A point-to-plane regisztráció minden iterációban a normálirányú residualok
rögzített hányadát tartotta meg. Ha a scanenkénti elmozdulás összemérhető a
mérési zajjal, ez a kezdeti nulla elmozdulással eleve jobban egyező pontokat
válogatja ki, és lefelé torzítja a megtett utat. Ismert, 1 méteres szintetikus
úton 10 mm zajjal 0,916 m, 20 mm zajjal 0,857 m adódott.

`RelativeLidarOdometry` most a geometriai inlierek normálirányú hibájából
becsült MAD zajskálával Huber-súlyozott illesztést végez. Nem dobja el minden
iterációban a pontok előre rögzített hányadát; a zavaró pontok befolyása
korlátozott. A pont-/iterációszám, geometriai távolságkapu, rank,
megfigyelhetőség, RMSE, sebesség- és frissességkorlát változatlan.
A számítás továbbra is a LiDAR workerben történik, encoder/pose prior nélkül.

A regressziós eset 15 mm zajt, változó sugármintavételt és mozgó előtérfoltot
tartalmaz: az ismert 1 m út régen 0,9040 m, javítva 1,0059 m;
a halmozott yaw-hiba 0,01764 → 0,00369 rad. A meglévő direct/process és teljes
L1–L12 Room Cruise/checkpoint-replay teszt is kapott zajos scaneket.

### Eredmény és fennmaradó kérdések

A capture 156 nyers scanpárját az eredeti intervallumokban újraszámolva,
az eredeti L2 admissiont és measurement/closure-időket megtartva:

| Production L3/L4 offline eredmény, aktív tickek | Rögzített input | Új relatív input |
| --- | ---: | ---: |
| Lokális translation `LOST` | 164 | 20 |
| Heading `LOST` | 0 | 0 |
| Maximális lokális sigma | 0,13432 m | 0,12149 m |

Az új input mellett 16 tickben továbbra is a konzisztenciahatár lép át
(532–534, 706–711, 828–834), négy tickben kerék–gyro slipjelzés marad
(630–633). A mérések közti maradék eltérés fizikai okát ez a capture nem
választja szét: kalibrációt vagy valódi csúszást nem szabad automatikusan
kijavítottnak tekinteni. A védelmi küszöbök változatlanok.

Folytonos L10 cél mellett L11 PWM-csökkenés is látszik: a 360. tickben
a cél bal/jobb 0,0706/0,1694 m/s, az encoder 0,1589/0,3306 m/s, a kimenet
0,0082/0,0314. A PI túlfutást mér és visszavesz; ebből önmagában nem
következik hibás PI-algoritmus. A tényleges keréksebesség/PWM kapcsolat,
kalibráció és terhelés élő bizonyítása hiányzik; L11-et nem hangoltuk át
evidence nélküli minimum-PWM vagy integrátor-reset bevezetésével.

Az offline, azonos 156 scanpáros számítás átlagos ideje 6,41 → 5,89 ms
(maximum 7,75 → 6,76 ms) egy azonos gépen végzett összevetésben. Ez pure
compute mérés, nem teljes live worker-/control-latencia.

A teljes eredeti capture natív replaye **903/903 tick `MATCH`**, ismételt
trace-egyezéssel. Ez az eredetileg rögzített relatív inputot játssza vissza;
a fenti újraszámolási kísérlet külön bizonyíték, nem exact replay és nem
új fizikai futás. Új robotmozgás nem indult. A teljesen folyamatos élő
mozgás így még nincs igazolva, és a maradék 20 localization `LOST` tick
miatt a javítás nem tekinthető a teljes jelenség lezárásának.

Validáció: `./r test` 32/32, `./r test localization` 22/22,
`./r test roomcruise` 6/6, `./r test process` 6/6,
`./r test replay` 3/3; `./r test full` **195/195 sikeres** (76,99 s).
A zajos teljes láncban indulás után folyamatos haladás és íves fordulás,
majd az utolsó 50 tick checkpoint-replaye is megfelelt.
