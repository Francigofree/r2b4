# R2B4 V3 robotarchitektúra — egyszerű rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló, fejlesztési napló,
roadmap vagy formális bizonyítási rendszer.

**Cél:** egyszerű, determinisztikus, jól tesztelhető és hosszú távon
karbantartható robot-runtime. A szerkezet a stabil fizikai működést, a
source-first fejlesztést, a gyors hibakeresést és a determinisztikus replayt
szolgálja.

A pillanatnyi implementációs készültség és hardver-evidence
nem ennek a dokumentumnak a feladata.

Authority sorrend:

```text
source + aktív config
→ ez a canonical V3 contract
→ Replayer + Test Hub V3 futásazonos capture/replay/diagnosis evidence
→ stabil baseline
→ történeti dokumentáció vagy nyers log
```

A dokumentum a V3 stabil architekturális határait rögzíti. Nem ír elő
indokolatlan jövőbeli frameworköt, konkrét algoritmust vagy olyan readiness
kaput, amelyre az aktuális funkciónak nincs szüksége.

## 1. Minimum, nem alkuképes garanciák

* Minden állapotnak és rétegnek pontosan egy owner-e van.
* Nincs legacy shared state, rejtett singleton vagy kerülő authority.
* A réteghatárok immutable, konkrét Python típusok; `dict[str, Any]` nem
  réteghatár-contract.
* Egyetlen, szekvenciális `TickEngine` zárja az input snapshotot, majd rögzített
  sorrendben legfeljebb egyszer hívja a szükséges rétegeket.
* Egy tickből pontosan egy L12 final döntés és legfeljebb egy normál motor-write
  születik.
* Az L12 hiányzó, hibás vagy bizonytalan, az adott mozgáshoz ténylegesen
  kritikus inputnál fail-closed STOP/FAULT döntést és fizikai inaktív motorállapotot
  eredményez.
* Azonos input, konfiguráció és kód azonos typed layer-outputokat eredményez.
* Replay eltérésnél a legelső eltérő tick és réteg közvetlen
  érték-összehasonlítással megnevezhető.
* Egy fizikai szenzor több, egymástól független szemantikai adatot
  szolgáltathat. Egy felhasználási ág hibája nem teheti automatikusan
  érvénytelenné a többi, önmagában érvényes ágat.
* A V3 production- és validációs út nem függ legacy source-tól, API-tól,
  runtime-tól vagy compatibility rétegtől.
* Nincs alternatív normál motorút, safety-bypass vagy tool-specifikus control
  authority.

Ezeket a garanciákat nem szabad adminisztratív könnyítés címén lazítani.
Minden más mechanizmus csak akkor kerülhet be, ha konkrét runtime-, safety-,
replay- vagy hibakeresési igényt egyszerűbben old meg, mint nélküle.

## 2. Determinisztikus végrehajtás

A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló
folyamatban nincs rétegenkénti thread, sleep, falióra, rejtett I/O vagy
modulglobális mutable state.

Egy tick menete:

1. A composition root létrehozza a `TickContext(tick_id, monotonic_ns)` értéket.
2. Lezárja a tick device- és command-inputját.
3. L1-től L11-ig minden szükséges réteget legfeljebb egyszer, rögzített
   sorrendben hív.
4. Bármely upstream hiba esetén a normál lánc megszakad, és az L12 pontosan
   egyszer explicit fault okkal kerül meghívásra.
5. Az L12 dönt és birtokolja az egyetlen normál `MotorWriter` capabilityt.
6. A tick typed trace-e diagnosztikához kiolvasható, de nem hat vissza a
   controlra.

A döntési idő kizárólag az injektált monoton idő. Randomizált algoritmus csak a
replay inputjában rögzített seedet használhat. CPU-idős deadline helyett fix
iteráció-, minta- vagy csomópont-budget kell, ahol ez releváns.

A live, capture, replay és későbbi szimuláció ugyanarra a kis végrehajtási
határra illeszthető:

```text
input source → production V3 → output sink
```

Az input source lezárt `TickInputs` értéket ad, a production elem a canonical
V3 `run_tick`, az output sink pedig passzív eredményt fogyaszt. A sink nem adhat
vissza control state-et, és nem kaphat motor-, GPIO-, lifecycle- vagy safety-
authorityt. A boundary újrafelhasználhatósága nem jelent új végrehajtási utat.

Aszinkron fizikai input, driver vagy nagyobb szenzorfeldolgozás használható, ha
annak eredménye a tick számára egyértelműen lezárt, immutable, időbélyegzett
inputként jelenik meg. Az aszinkron producer nem válhat control-authorityvá.

## 3. Egyszerű contractmodell

Minden top-level layer output egy frozen, slotted dataclass. A közös metadata
minimuma:

```text
TickContext
  tick_id: int
  monotonic_ns: int
```

Nincs kötelező közös schema envelope, schema registry, producer/session
provenance, causation chain, config hash, event hash vagy boundarynkénti
canonical serializer.

A Python típusdefiníció maga a belső contract. Inkompatibilis változást a hívó
kód és a célzott contractteszt együtt követ.

Validáció csak ott kötelező, ahol közvetlen értéke van:

* véges számok, fizikai tartományok és nemnegatív idő/sorszám;
* egy ticken belüli azonos `TickContext`;
* measurement/source idő egyértelmű jelentése;
* freshness, ordering és trust;
* domain-invariánsok;
* STOP/FAULT esetén null logikai final output és fizikailag inaktív motor-edge;
* kritikus azonosítók és gyűjteménykulcsok egyértelműsége.

A fizikai measurement idő és a feldolgozási/result idő nem keverhető.
Freshness alapértelmezés szerint annak a fizikai mérésnek az idejéből számítandó,
amelyre az adott eredmény vonatkozik.

A diagnosztikai ok rövid stabil `reason`, nem általános proof- vagy reason-code
gráf. A capture edge használhat egyszerű verziózott fájlformátumot, de a
serializáció nem része minden runtime contractnak.

## 4. Rétegek és state-ownership

| Réteg                      | Egyetlen felelősség és owned state                                       | Typed output            |
| -------------------------- | ------------------------------------------------------------------------ | ----------------------- |
| L0 Device HAL              | eszközhandle, busz, fizikai read/write                                   | `RawDeviceBatch`        |
| L1 Acquisition             | polling/stream, source sequence, I/O health, lezárt typed szenzermérések | `AcquisitionFrame`      |
| L2 Admission               | freshness, sorrend, duplikáció, trust/alignment history                  | `AdmittedFrame`         |
| L3 State Estimation        | pose, twist, covariance                                                  | `RobotEstimate`         |
| L4 World Model             | rolling local costmap, térképi világállapot, revision és akadályhistory | `WorldSnapshot`         |
| L5 Command & Mission       | validált command- és mission-lifecycle                                   | `MissionIntent`         |
| L6 Navigation              | route/corridor/progress, coverage/local goal és trajectory evaluation    | `NavigationPlan`        |
| L7 Motion Selection        | prioritás és egyetlen kiválasztott trajectory/cél                        | `MotionObjective`       |
| L8 Motion Realization      | kiválasztott guidance pillanatnyi kinematikai célja                      | `MotionIntent`          |
| L9 Operational Constraints | dinamikai/környezeti korlátozás state                                    | `ConstrainedMotion`     |
| L10 Chassis Control        | chassis-kinematika                                                       | `WheelVelocitySetpoint` |
| L11 Actuator Control       | wheel-loop integrátor, feed-forward, calibration map                     | `ActuatorRequest`       |
| L12 Safety & Final         | safety latch, final döntés, egyetlen normál `MotorWriter`                | `FinalActuation`        |
| Composition root           | tick, lifecycle, config snapshot és wiring                               | `TickTrace`             |

Egy réteg nem módosíthat másik réteg state-jét. Az output új immutable érték;
nem adhat át controllert, GUI objektumot, device handlet vagy mutable
collectiont.

A rétegezés nem jelenti azt, hogy egy fizikai szenzorhoz pontosan egy
réteghatár-output tartozhat. Egyetlen acquisition forrásból több, eltérő célú
typed sample zárható, ha azok ownershipje és jelentése egyértelmű.

## 5. Engedélyezett adat-élek

```text
L0  -> L1
L1  -> L2, L12
L2  -> L3, L4, L11
L3  -> L4, L6, L8, L9
L4  -> L6, L8
L5  -> L6
L6  -> L7
L7  -> L8
L8  -> L9
L9  -> L10
L10 -> L11
L11 -> L12
L12 -> L0/MotorWriter
CommandGateway -> L5
CompositionRoot -> minden layer konstrukciója és lifecycle-ja
```

Az engedélyezett élek fan-outot is jelentenek. Ugyanaz a fizikailag megszerzett
szenzorinformáció több, szemantikailag különálló typed eredményt táplálhat a
megengedett célrétegek felé.

Layer implementation nem importálhat másik layer implementationt. Kapcsolás
csak a composition rootban, typed callable/port konstrukcióval történhet.

## 6. Final safety és motorírás

Az L12 a normál motor-write capability egyetlen tulajdonosa. Nincs alternatív
pozitív PWM-, service-, GUI-, tool- vagy külső writer.

Az L12 kötelező viselkedése:

* upstream exception, kritikus device failure vagy hiányzó actuator request:
  fail-closed `FAULT`, null logikai output;
* ismeretlen vagy a konkrét mozgás biztonságos végrehajtásához szükséges
  kritikus input hiánya: `STOP` vagy indokolt esetben `FAULT`;
* érvényes safety observation közvetlenül korlátozhat vagy megtilthat actuationt;
* csak érvényes L11 request, megfelelő lifecycle és minden ténylegesen szükséges
  safety feltétel esetén `ALLOW`;
* egy normál döntés után legfeljebb egy atomi writer-hívás;
* write exception után nincs automatikus második normál írás, a fault latch
  beáll;
* STOP/FAULT contract nem tartalmazhat nem nulla logikai outputot.

A STOP/FAULT nem pusztán `duty=0` szoftveres értéket jelent. A motor-edge
shutdown contractjának a hardver szempontjából fizikailag inaktív, igazolható
állapotot kell eredményeznie.

A `DeviceHealth` és a szenzor által megfigyelt veszély két külön fogalom.

Például:

```text
LiDAR device/scan health = OK
front safety clearance = veszélyesen kicsi
→ STOP az akadály miatt
```

nem pedig:

```text
akadály látható
→ LiDAR device DEGRADED
```

Hasonlóan egy magasabb szintű feldolgozás hibája nem változtathat működő
hardverforrást automatikusan hibás device-zá.

Külön általános `ActuationReceipt` rendszer nem kell. Ha a hardver alkalmazott
érték-visszaolvasást igényel, az egyszerű typed device feedbackként kerül
L0/L1-be.

## 7. Többcélú szenzoradat és capability-függetlenség

Egy fizikai szenzor nem egyetlen algoritmus tulajdona.

A szenzorrendszernek külön kell választania legalább:

1. a fizikai eszköz/stream működőképességét;
2. az adott mérés érvényességét és freshness-ét;
3. az egyes feldolgozási/felhasználási ágak minőségét.

Egy ág failure/degradation állapota csak akkor terjedhet másik ágra, ha közös
fizikai vagy contract-szintű oka van.

A safety számára önmagában használható mérés nem válhat használhatatlanná pusztán
azért, mert ugyanabból a forrásból egy lokalizációs, térképezési vagy más
magasabb szintű algoritmus nem tudott megfelelő eredményt előállítani.

A capability-k szétválasztása nem jelent párhuzamos control authorityt. A
különböző szenzorágak csak typed adatot szolgáltatnak; a végső actuation
authority továbbra is az L12.

### 7.1 Natív encoder sebesség és RAW út

A natív encoder edge minden ticken pontosan egyszer olvassa ki a két signed
impulzusszámlálót. A kumulatív számláló, a tickenkénti signed delta és az ezekből
közvetlenül számolt kumulatív/tick távolság RAW evidence: nem simítható, nem
becsülhető vissza PWM-ből, és nem írható át a sebességszűrő eredményével.

Csak a keréksebesség becslése időablakos. A két kerék külön a legrövidebb friss
ablakot választja, amely legalább a konfigurált impulzusszámot és minimális időt
tartalmazza; ha ez nem teljesül, az ablak a konfigurált maximumig nő. Az aktív
policy négy impulzust, 40 ms minimumot és 160 ms maximumot használ. Így nagy
sebességnél az ablak rövid, kis sebességnél hosszabb, de mindig véges és
determinisztikus.

A measurement `trust` és a kerékenkénti egyimpulzusos sebességbizonytalanság a
kiválasztott ablak minőségét írja le. Baseline, kevés impulzus, stale minta vagy
hibás tick-idő önmagában nem encoder device-hiba. Device health csak konkrét
counter-hibából, leállt counterből vagy fizikailag lehetetlen pulse-rate-ből
romolhat. L2 a mérés timing/freshness állapotát ettől külön kezeli, L3 pedig az
ablakolt sebességet velocity measurementként, a RAW tick-távolságot pedig
elmozdulásként használja. Régi capture raw távolságmezők nélkül a meglévő
sebességintegrálás fallbackje replay-kompatibilitási célból megmarad.

## 8. Natív LiDAR adatút

A natív V3 LiDAR egy fizikai RPLIDAR acquisitionből több, egymástól független
typed eredményt szolgáltathat.

Normatív logikai céladatút:

```text
RPLIDAR
   ↓
natív V3 LiDAR acquisition
   ↓
L1
   ├── safety perception ─────────────→ L12
   │
   ├── local perception ─→ L2 ───────→ L4
   │
   └── localization pose ─→ L2 ──────→ L3
```

A diagram a szemantikai adatágakat rögzíti; nem ír elő fölösleges frameworköt
vagy konkrét algoritmust a feldolgozás belső megvalósítására.

### 8.1 Safety ág

A safety ág a fizikai környezet közvetlenül collision-releváns információját
adja L12 felé.

Ez lehet például irány-/szektoralapú clearance vagy más minimális typed
collision-safety observation. A contract nem épülhet `FORWARD`, `ARC`, `PIVOT`
vagy más motion-primitive fogalomra.

A safety adat akkor használható, ha a saját forrásmérése friss, fizikailag
érvényes és a saját safety contractját teljesíti.

A safety ág nem függhet:

* scan matcher sikerétől;
* globális localization pose meglététől;
* localization confidence-től;
* térkép meglététől;
* L3 EKF LiDAR-update sikerétől;
* L4 local-perception eredményétől.

Ha a localization vagy local perception nem használható, de a safety
measurement önmagában érvényes, a safety ág továbbra is működik.

### 8.2 Local-perception ág

A local-perception ág a robot körüli környezet olyan typed reprezentációját
szolgáltatja L4 felé, amely navigation és motion realization számára közvetlenül
hasznos.

A reprezentáció a következő konkrét funkcióhoz szükséges legegyszerűbb
clearance-, geometry-, obstacle- vagy track-adat legyen.

Nem kell előre általános SLAM-, occupancy-, scene-graph- vagy trajectory
frameworköt építeni.

A jelenlegi `ObstacleTrack` használható, ahol természetes reprezentáció, de nem
kötelező minden LiDAR-geometriát obstacle trackké alakítani.

A lokális geometria L4-owned rolling costmapként zárható. A costmap ownershipje
nem kerülhet a LiDAR adapterbe vagy egy magasabb szintű viselkedésbe; az adapter
csak bounded, immutable local-perception adatot szolgáltat.

### 8.3 Localization ág

A LiDAR localization opcionális measurement az L3 state estimator számára.

```text
LiDAR localization
→ érvényes pose measurement
→ L2
→ L3 EKF correction
```

Ha nincs megbízható localization eredmény:

```text
nincs új lidar_pose
```

Ez önmagában nem jelenti azt, hogy:

```text
LiDAR device hibás
safety perception hibás
local perception hibás
```

A localization minősége pose-measurement quality/uncertainty kérdés. A device
health ettől külön állapot.

A localization mérés confidence/quality/covariance adata használható L3
measurement weightingre vagy validity-döntésre, de ugyanaz az egy scalar
quality nem válhat automatikusan teljes LiDAR device-health authorityvá.

Az encoder és IMU által fenntartott állapotbecslés LiDAR pose hiányában is
folytatódhat. Ha a teljes state estimate bizonytalansága később túl nagy lesz,
annak motion-korlátozása L9/state-estimation policy kérdés, nem hamis LiDAR
device failure.

### 8.4 Idő és raw scan

A LiDAR measurement timestamp a fizikai scan mérési idejét jelenti. A matcher
vagy más feldolgozás befejezési ideje külön diagnosztikai adat lehet, de nem
helyettesíti a source measurement időt.

A raw LiDAR scan nem kötelező top-level L1–L4 boundary payload.

A natív LiDAR capability device-specifikus packet decode-ot, scan assemblyt,
quality-validációt és bounded source-local feldolgozást végezhet, majd a V3
pipeline számára csak a ténylegesen szükséges immutable typed eredményeket
zárhatja.

A raw scan capture/replay célra külön rögzíthető, ha konkrét diagnosztikai
értéke van.

### 8.5 Natív ownership

A production V3 LiDAR adatút kizárólag natív V3 driver-, port-, matcher- és
typed-contract elemekből állhat. Nem függhet külső shared-state- vagy más
runtime-authoritytól.

### 8.6 Aktív natív V3 LiDAR-zárás

Az aktív egyszerű production megvalósítás egyetlen ownership-lánc:

```text
NativeRplidarC1
→ NativeLidarPort (bounded latest-only matcher process)
→ NativeLatestLidarBackend
→ NativeLidarSource
```

Az owner ugyanabból a teljes fizikai scanből az alábbi külön immutable L1
mintákat zárja:

* `lidar_health`: fizikai stream/scan revision, freshness és pontszám;
* `lidar_safety_clearance`: front/rear/left/right clearance és szektoronkénti
  observation count;
* `lidar_localization_health`: a matcher eredmény saját freshness-, confidence-
  és usability-állapota;
* `lidar_pose`: csak használható localization eredménynél;
* `lidar_matcher_diagnostics`: passzív matcher lineage és quality evidence.

Az L12 a `lidar_safety_clearance` mintát közvetlenül az L1 outputból kapja. A
szükséges irányt a logikai L10 kerék-setpointból vezeti le, nem a kalibráció
miatt oldalanként eltérő fizikai PWM-ből. Hiányzó, stale, nem megfigyelt vagy a
konfigurált clearance-határ alatti szükséges szektor esetén nem enged
actuationt.

A stateful scan-matching matematika a natív `v3.lidar_estimator` komponensben,
a hozzá tartozó tiszta keresési matematika pedig a `v3.scan_matching` modulban
él. A production út nem importál legacy donort, drivert, service-t vagy
matcher-processzt, és nem vesz át legacy runtime/shared-state authorityt.

A V3 capture a teljes lezárt `TickInputs` contractot generikusan és veszteség
nélkül rögzíti, a V3 Replayer pedig ugyanazt a production V3 utat futtatja
vissza. A replay egyetlen natív capture sémát fogad, és kötelezően visszazárja a
capture natív `lidar_safety_clearance` device identityjét az L12 safety gate-be.
Tesztprofil-specifikus replay schema vagy alternatív diagnosztikai út nincs.

## 9. Motion és command szabadság

A robot normál mozgatásának egyetlen canonical V3 útja van:

```text
CommandGateway
→ L5
→ L6
→ L7
→ L8
→ L9
→ L10
→ L11
→ L12
→ MotorWriter
```

Test Hub, teleop, script, későbbi follow/AI vagy más külső command source nem
kaphat saját robotmozgató-, motor- vagy safety-útvonalat.

A motion rendszer nem épülhet `FORWARD`, `ARC`, `PIVOT`, `MOVE_1M` vagy más
rögzített motion-primitive fogalmakra.

Külső command source általános kinematikai célt, például `v_mps` és
`omega_rad_s`, vagy magasabb szintű mission/navigation célt adhat a canonical
command út felé.

Egy konkrét tesztszcenárió, például „1 m előre 0.15 m/s sebességgel”, nem
runtime primitive, hanem egy általános command source időben változó kimenete.

Egy motion request csak azokhoz a capability-khez köthető kötelező gate-ként,
amelyek az adott mozgás szemantikájához vagy biztonságos végrehajtásához
ténylegesen szükségesek.

Egy opcionális vagy még fejlesztés alatt álló magasabb szintű capability nem
válhat pusztán a létezése miatt minden motion globális blokkoló feltételévé.

Ez nem jogosít meglévő safety-, PASS- vagy quality gate megkerülésére vagy
lazítására. Ha egy meglévő gate indokolatlan couplingot okoz, a forrás és a
fizikai evidence alapján a capability-határt kell kijavítani, nem a safety
követelményt megkerülni.

### 9.1 Általános lokális navigáció és trajectory guidance

A footprint-aware, fix mintabudgetű `(v, omega)` trajectory rollout és annak
collision-, clearance-, progress-, smoothness- és novelty-értékelése L6
navigációs felelősség. Az L6 az értékelt jelölteket adja át, de nem hoz létre
második motion-selection authorityt. Az L7 választ közülük pontosan egy
`MotionObjective` értéket, az L8 pedig a kiválasztott guidance pillanatnyi
`v_mps, omega_rad_s` célját realizálja.

Ez általános V3 navigációs capability: `NAVIGATE`, `EXPLORE`, későbbi `FOLLOW`
és más felső viselkedés ugyanazt az L4--L8 adat- és authority-utat használhatja.
A Room Cruise ennek egy `EXPLORE` missiont adó felső fogyasztója, nem külön
planner-, primitive-, motion- vagy safety-authority.

## 10. V3 Test Hub elv

A V3 Test Hub külső fejlesztői/test eszköz, nem production runtime-, planner-,
state-, motion- vagy safety-authority.

Normatív szerepe:

```text
V3 source + aktív config
→ általános V3 capture
→ offline Replayer V3
→ futásazonos L1–L12 diagnosis/evidence
→ csak szükség esetén live hardware
```

A V3 Test Hub:

* ugyanazt a canonical V3 motion- és safety-utat használja, mint a robot normál
  működése;
* nem hozhat létre második motor-, safety- vagy lifecycle-authorityt;
* nem vezethet be motion-primitive alapú runtime-architektúrát;
* nem kerülheti meg az L12-t;
* paraméterezett tesztszcenáriót adhat általános command source-ként;
* a capture-t és replayt első osztályú fejlesztési eszközként használja;
* nem tart fenn kötelező profile-, scenario-, admin- vagy latest-pointer
  infrastruktúrát;
* nem lehet production V3 runtime dependency.

A live adapter, az általános capture sink, az offline replay mag és a Test Hub
evidence-csomag külön modul és külön authority. Mindegyik kizárólag natív V3
contractot használhat.

Élő mozgás csak explicit felhasználói keret, friss preflight, jóváhagyott Test
Hub út és igazolt végső biztonságos motorállapot mellett indulhat.

## 11. Konfiguráció, command és külső I/O

A composition root validált, immutable konfigurációt injektál. Layer nem olvas
fájlt, environment variable-t vagy globális config managert.

Actuationt érintő konfiguráció csak explicit biztonságos lifecycle-határon és
fizikailag inaktív motorállapot mellett cserélhető.

GUI, CLI, LLM és tool csak a `CommandGateway` kliensén keresztül adhat typed
`CommandRequest` értéket. A gateway kezeli a külső hitelesítést és lejáratot;
ezek teljes provenance-ének minden belső üzenetben való ismétlése nem
szükséges.

A TTL-es resident command mailbox ephemerális, atomikusan cserélt edge-adat,
nem tartós napló. Heartbeat ütemezése abszolút monotonic határidőhöz igazodik,
így az előző publish késleltetése nem adódhat hozzá minden periódushoz; a
biztonságot változatlanul a gateway TTL-ellenőrzése és a fail-closed STOP adja.

A runtime headless. Külső I/O kizárólag edge/device adapterben történik, és az
adapter nem válhat state- vagy control-authorityvá.

## 12. V3-only source szabály

A V3 modulok csak a V3 csomagot, a standard libraryt és az explicit aktív
numerikus függőségeket importálhatják. Legacy source-import, compatibility
adapter, donor-allowlist, shared state és alternatív runtime authority tilos.

Új capability minimális natív V3 implementációként, a typed V3 contractokból
indulva készülhet.

## 13. Replay és hibakeresés

A replay célja reprodukálni a döntést és gyorsan megtalálni az első hibás
réteget.

Az alapértelmezett live capture-stratégia konfigurálható, bounded RAM ring
buffer. Fault vagy passzív process-szintű manuális trigger esetén a kijelölt
pre-event és post-event időablak kerül tartós fájlba. A teljes futás append-only
streamelése csak explicit opcionális mód lehet, és nem lehet Test Hub
függőség. A capture observer kizárólag nem blokkoló enqueue boundary: encoding,
szerializálás és fájl-I/O nem futhat a production tick loopban. Queue overflow
vagy capture-worker hiba nem módosíthat motor-outputot.

A minimális replay input:

* tickenként a lezárt `RawDeviceBatch`, `CommandRequest`, lifecycle és
  `TickContext`;
* a futáshoz ténylegesen használt konfiguráció egyszer, tartalom szerint;
* randomizált algoritmus esetén a seed.

A capture két tickalakot különböztet meg. A `closed_input_tick` a lezárt
`TickInputs` értéket és a production eredményt hordozza. Az inputlezárás előtti
L0-, command-, preflight- vagy shutdown-hiba `edge_fault_tick`: `TickContext`,
lifecycle, fault reason/layer, rendelkezésre álló health/raw input evidence és
az ugyanazon production L12 által lezárt eredmény. L12 writer-failure esetén a
megkísérelt final actuation és a sikertelen edge commit explicit evidence.

A terminális futási verdict és a replay verdict külön fogalom.
Strukturálisan érvényes `FAIL/FAULT` capture ugyanúgy replayelendő, mint a
`PASS`, mert a fail-closed döntési lánc elsődleges hibakeresési evidence. A
„strukturálisan érvényes” fault tick nem jelent mesterségesen teljes L1–L12
trace-t: a hiba előtt sikeresen lezárt folytonos L1-prefixet, az explicit
`fault_layer` értéket és a kötelező, utolsó L12 eredményt jelenti.

Nem terminális, hiányos, ingress dropot vagy tickrést tartalmazó capture nem
kaphat `MATCH` eredményt. Terminális futásnál a post-event ablak szabályosan
lehet rövidebb; ezt `post_window_complete=false` és
`terminal_short_post_window=true` értékkel explicit jelölni kell, és ez önmagában
nem hamisítja meg a rendelkezésre álló tickek replay-verdictjét.

Readiness vagy arming kapu csak egymástól valóban független, friss
forrásevidence-t számolhat új bizonyítéknak. Ugyanazon latest-only source
revízió ismételt tickes kiolvasása nem számíthat több független mérésnek.

Preflight nélküli első ACTIVE kérés fail-closed terminális fault. Egy már
szabályosan ACTIVE session rövid command-kimaradása viszont STOP-pal rearm
állapotot nyit: a preflight teljességéig érkező ACTIVE heartbeat-ek ugyanazon
production L1–L12 úton zéró IDLE tickként záródnak, és nem válhatnak motor-
authorityvá vagy önmagukban terminális faulttá. Input-, gateway-, layer- vagy
writer-hiba ettől függetlenül továbbra is faultot latch-el.

A readiness feltétel csak azt a szenzor- vagy capability-ágat teheti kötelezővé,
amelyre az adott funkció biztonságos végrehajtásához ténylegesen szükség van.
Egy opcionális localization ág nem válhat pusztán architekturális megszokásból
az egész robot globális arming feltételévé.

Normál tick `TickTrace` értéke L1–L12 typed outputokat tartalmaz. Upstream hiba
esetén csak a sikeresen lezárt folytonos L1-prefix és az L12 létezhet; a hibás
és az utána következő rétegekhez tilos fiktív outputot gyártani. Például L4
exception helyes trace-e `L1,L2,L3,L12`, `fault_layer=L4`. Két futás közvetlen
dataclass-egyenlőséggel hasonlítható össze; az első eltérő rekord megadja a
`tick_id`-t, a layer nevet, az első eltérő mezőútvonalat, valamint az expected és
actual értéket. A replay számítás első eltérése (`first_divergence`), az eredeti
live futás első fault/degradation eseménye (`first_live_incident`) és a fizikai
root cause evidence-verdictje (`PROVEN`, `INDICATED`, `NOT_PROVEN`) külön adat.

A replay inclusive tick-, monotonidő- és összefüggő L1–L12 rétegtartományra
szűkíthető. Stateful réteg esetén a kiválasztott első tick előtti capture-prefix
kötelező warmupként végigfut ugyanazon production példányon; csak a kijelölt
tickek és rétegek kapnak diagnosztikai verdictet. Bounded ring capture esetén a
prefixet a production composition ritka, bounded, post-tick typed state
checkpointja indíthatja: a Replayer ugyanazon `NativeControlComposition`
példányba restore-olja, majd a checkpoint utáni rögzített tickeket futtatja.
Checkpoint nélkül csak a tényleges kezdeti state-ből, első ticktől induló
capture lehet `MATCH`-eligible; a checkpoint nem helyettesíthet inputot vagy
layer-outputot, és nem válhat live control authorityvá.

A capture a konkrét source-first diagnosztikához szükséges evidence-et őrizze
meg, de nincs általános „mindent logoljunk” követelmény.

A raw LiDAR scan trigger előtt csak a bounded RAM ringben élhet. Mentéskor csak
a kiválasztott tickablak által hivatkozott fizikai scan revisionök és a matcher
lineage által közvetlenül hivatkozott source scanek kerülhetnek a capture-be,
revisionönként egyszer. Hiányzó vagy kapacitás miatt kiesett hivatkozott scan
explicit `missing_revisions` evidence, és a fizikai verdict legalább
`NOT_PROVEN`. Capture-local általános objektumtábla vagy provenance-gráf csak
valós capture-mérés által igazolt igényre vezethető be.

A live capture-mérés által igazolt, pontos L0→L1 és L0→L2 payload-duplikáció
szűk input-reference reprezentációval elhagyható. Ez kizárólag akkor érvényes,
ha az L1/L2 érték mezőre pontosan levezethető ugyanazon lezárt `TickInputs`
értékből; a Replayer összehasonlítás előtt a teljes typed értéket visszaállítja.
Ez nem általános capture-local objektumtábla.

Nem kötelező:

* minden event vagy payload SHA-256 hash-e;
* schema registry és mezőnkénti runtime schema validation;
* causation/provenance gráf;
* build/source fingerprint a control contractban;
* layerenkénti receipt vagy külön proof objektum;
* canonical round-trip teszt minden belső üzenethez;
* teljes fizikai robot- vagy sensor-driver szimulátor minden replayhez.

Capture-fájl checksum használható egyszerű fájlsérülés-ellenőrzésre, de nem
válik runtime identityvé vagy döntési inputtá.

A trace/log/evidence hibája soha nem módosíthat control outputot. A Test Hub
csak inspectet, a canonical Replayer API-t, diagnózis-generálást és
evidence-indexelést orchestrálhat; saját decoder, layer-futtatás vagy
szenzorspecifikus replay tilos.

## 14. Kötelező, célzott tesztkapuk

Egy V3 source minimum tesztjei:

* import guard: nincs cross-layer implementation import, külső shared-state,
  GUI/tool authority vagy V3-on kívüli project-import;
* contractteszt: immutable típusok és a közvetlen domain/safety invariánsok;
* TickEngine teszt: lezárt snapshot, rögzített sorrend, rétegenként legfeljebb
  egy értékelés és egyetlen L12 final döntés;
* fail-closed teszt: upstream exception, invalid tick/lifecycle, missing/failed
  ténylegesen kritikus input és writer failure;
* replayteszt: azonos inputra azonos trace, módosított layer-outputnál helyes
  első divergáló réteg;
* natív algoritmusonként célzott offline contract/karakterizációs teszt;
* többcélú szenzorág esetén annak bizonyítása, hogy egy független magasabb
  szintű ág failure-je nem teszi érvénytelenné az önmagában használható safety
  ágat;
* motor-edge contract változásnál célzott bizonyíték arra, hogy STOP/FAULT
  fizikailag inaktív állapotot eredményez.

Nincs általános kötelező schema-, hash-, provenance-, receipt- vagy formális
proof-kapu.

A célzott teszt az alapértelmezett. Teljes regressziót a repo agent-infra
scope/contract/risk szabályai kérnek, vagy indokolt diagnosztikai esetben kell
futtatni.

## 15. Fejlesztési szabály

Ez a dokumentum nem tartalmaz részletes fejlesztési történetet vagy előre
felépített roadmapet.

A rövid távú V3 fejlesztés alapfolyamata:

```text
source + aktív config
→ canonical V3 contract
→ Replayer + Test Hub V3 capture/replay/diagnosis evidence
→ legkisebb szükséges architecture-compatible változás
→ célzott teszt
→ replay
→ csak akkor új élő mérés, ha a következő értékes információ már hardverből jöhet
```

A replay-first nem jelent hardware-avoidance-ot. Ha a következő bizonyító
információ csak a valódi robotból szerezhető meg, a lehető legkisebb biztonságos
élő futást kell elvégezni, majd annak capture-jéből tovább dolgozni.

Ne épüljön komponens azért, mert később esetleg hasznos lehet.

Stabil réteg csak konkrét source/evidence által igazolt igény miatt változzon.

Egy új capability a lehető legkisebb olyan vertikális szelet legyen, amely már
valódi funkcionális értéket ad, de nem vezet be szükségtelen általános
frameworköt.

Fejlesztési fázisban új readiness-, quality- vagy capability-gate csak konkrét
fizikai vagy replay evidence alapján váljon kötelezővé.

Meglévő safety-, PASS- vagy quality gate-et nem szabad pusztán azért lazítani,
hogy egy teszt átmenjen. Ha egy gate tévesen több független capabilityt köt
össze, a coupling gyökerét kell kijavítani.

A pillanatnyi implementációs állapot authorityja a canonical source, az aktív
config, a tesztek és a run-bound evidence.
