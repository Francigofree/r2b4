# R2B4 V3 robotarchitektúra — egyszerű rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló, fejlesztési napló,
roadmap vagy formális bizonyítási rendszer.

**Cél:** egyszerű, determinisztikus, jól tesztelhető és hosszú távon
karbantartható robot-runtime. A szerkezet a stabil fizikai működést, a
source-first fejlesztést, a gyors hibakeresést és a determinisztikus replayt
szolgálja.

A pillanatnyi implementációs készültség, promotion-állapot és hardver-evidence
nem ennek a dokumentumnak a feladata.

Authority sorrend:

```text
source + aktív config
→ ez a canonical V3 contract
→ futásazonos replay/capture/Test Hub evidence
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
* Legacy komponens csak tiszta donor vagy explicit, vékony compatibility edge
  lehet; legacy state- vagy control-authority és mutable objektumreferencia nem
  kerülhet V3-ba.
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
| L4 World Model             | lokális/térképi világállapot, revision és akadályhistory                 | `WorldSnapshot`         |
| L5 Command & Mission       | validált command- és mission-lifecycle                                   | `MissionIntent`         |
| L6 Navigation              | route, corridor és progress                                              | `NavigationPlan`        |
| L7 Motion Selection        | prioritás és egyetlen kiválasztott cél                                   | `MotionObjective`       |
| L8 Motion Realization      | guidance és pillanatnyi kinematikai cél                                  | `MotionIntent`          |
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
pozitív PWM-, service-, GUI-, tool- vagy donor-writer.

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

### 8.5 Natív ownership és legacy kivágás

A production V3 LiDAR adatút nem függhet legacy LiDAR driver-, service-,
matcher-process-, shared-state- vagy runtime-authoritytól.

Legacy LiDAR kód csak:

* algoritmikus referencia;
* karakterizációs donor;
* vagy explicit, izolált átmeneti compatibility edge

lehet.

Az átmeneti compatibility edge nem válhat a V3 architecture authority részévé,
és a natív adatút elkészülésével eltávolíthatónak kell lennie.

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

## 10. V3 Test Hub elv

A V3 Test Hub külső fejlesztői/test eszköz, nem production runtime-, planner-,
state-, motion- vagy safety-authority.

Normatív szerepe:

```text
command source
→ canonical V3 runtime
→ capture
→ V3 replay
→ diagnosztika
```

A V3 Test Hub:

* ugyanazt a canonical V3 motion- és safety-utat használja, mint a robot normál
  működése;
* nem hozhat létre második motor-, safety- vagy lifecycle-authorityt;
* nem vezethet be motion-primitive alapú runtime-architektúrát;
* nem kerülheti meg az L12-t;
* paraméterezett tesztszcenáriót adhat általános command source-ként;
* a capture-t és replayt első osztályú fejlesztési eszközként használja;
* nem lehet production V3 runtime dependency.

A production V3 runtime nem függhet legacy Test Hubtól. Átmeneti külső wrapper
csak a canonical V3 runtime meghívására használható; V3-specifikus runtime-,
motion-, safety- vagy replay-authority nem maradhat benne hosszú távon.

A konkrét V3 Test Hub CLI-, profil-, UI- és artifact-struktúrája nem része ennek
az architecture contractnak; azt az aktuális source és a következő konkrét
fejlesztési igény határozza meg.

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

A runtime headless. Külső I/O kizárólag edge/device adapterben történik, és az
adapter nem válhat state- vagy control-authorityvá.

## 12. Legacy donor szabály

Legacy import csak explicit donor/compatibility határon és pontos allowlisttel
engedélyezett.

Donor akkor használható, ha:

1. nincs `ctrl`, global state, GUI vagy legacy authority függése;
2. idejét, konfigurációját és I/O-ját argumentumból/portból kapja;
3. offline fake-kel determinisztikusan fut;
4. bemenete és kimenete V3 contracttá alakítható legacy referencia nélkül;
5. célzott karakterizációs teszt fedi a megtartani kívánt algoritmust;
6. nem nyit motor- vagy safety-bypass útvonalat.

Ha ehhez nagy adapter vagy legacy orchestration kellene, csak a tiszta
algoritmikus mag emelhető át.

Felső szintű legacy komponens alapértelmezés szerint nem portolandó. GUI, CLI,
Test Hub, legacy Replayer, state machine, Core/Task/AI orchestration,
Follow/Search orchestration és legacy command/status infrastruktúra csak egy
konkrét következő V3 funkció közvetlen igényére kap megfelelő V3 elemet.

Az új elem minimális native V3 implementáció, amely a V3 contractokból indul
ki. Legacy API-, viselkedés- vagy struktúraparitás nem követelmény.

## 13. Replay és hibakeresés

A replay célja reprodukálni a döntést és gyorsan megtalálni az első hibás
réteget.

A minimális replay input:

* tickenként a lezárt `RawDeviceBatch`, `CommandRequest`, lifecycle és
  `TickContext`;
* a futáshoz ténylegesen használt konfiguráció egyszer, tartalom szerint;
* randomizált algoritmus esetén a seed.

A terminális futási verdict és a replay verdict külön fogalom.
Strukturálisan teljes `FAIL/FAULT` capture ugyanúgy replayelendő, mint a `PASS`,
mert a fail-closed döntési lánc elsődleges hibakeresési evidence.

Nem terminális vagy integritáshibás capture nem kaphat `MATCH` eredményt.

Readiness vagy arming kapu csak egymástól valóban független, friss
forrásevidence-t számolhat új bizonyítéknak. Ugyanazon latest-only source
revízió ismételt tickes kiolvasása nem számíthat több független mérésnek.

A readiness feltétel csak azt a szenzor- vagy capability-ágat teheti kötelezővé,
amelyre az adott funkció biztonságos végrehajtásához ténylegesen szükség van.
Egy opcionális localization ág nem válhat pusztán architekturális megszokásból
az egész robot globális arming feltételévé.

A `TickTrace` tickenként L1–L12 typed outputokat tartalmaz. Két futás közvetlen
dataclass-egyenlőséggel hasonlítható össze; az első eltérő rekord megadja a
`tick_id`-t és a layer nevet.

A capture a konkrét source-first diagnosztikához szükséges evidence-et őrizze
meg, de nincs általános „mindent logoljunk” követelmény.

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

A trace/log/evidence hibája soha nem módosíthat control outputot.

## 14. Kötelező, célzott tesztkapuk

Egy V3 source candidate minimum tesztjei:

* import guard: nincs cross-layer implementation import, legacy shared-state,
  GUI/tool authority vagy donor-allowlist bypass;
* contractteszt: immutable típusok és a közvetlen domain/safety invariánsok;
* TickEngine teszt: lezárt snapshot, rögzített sorrend, rétegenként legfeljebb
  egy értékelés és egyetlen L12 final döntés;
* fail-closed teszt: upstream exception, invalid tick/lifecycle, missing/failed
  ténylegesen kritikus input és writer failure;
* replayteszt: azonos inputra azonos trace, módosított layer-outputnál helyes
  első divergáló réteg;
* donoronként külön offline karakterizációs teszt;
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
→ meglévő capture/replay/hardver evidence
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

A pillanatnyi implementációs állapotot nem ez a dokumentum tartja nyilván.
Ennek authorityja a canonical source, az aktív config, a tesztek és a
run-bound evidence.

Korábbi részletes slice-leírások, promotion-történet és történeti evidence
megőrizhető külön, explicit **NON-NORMATIVE** történeti dokumentumban, például:

```text
docs/V3_IMPLEMENTATION_HISTORY.md
```

A történeti dokumentum nem része a normál `robot_v3` source-route authoritynak,
és nem írhatja felül ezt a contractot vagy a canonical source-ot.
