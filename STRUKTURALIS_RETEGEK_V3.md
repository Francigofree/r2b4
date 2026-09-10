# R2B4 V3 robotarchitektúra — egyszerű rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló, fejlesztési napló,
roadmap, tuningjegyzet vagy formális bizonyítási rendszer.

**Cél:** egyszerű, determinisztikus, jól tesztelhető és hosszú távon
karbantartható robot-runtime. A szerkezet a stabil fizikai működést, a
source-first fejlesztést, a gyors hibakeresést és a determinisztikus replayt
szolgálja.

A pillanatnyi implementációs készültség, konkrét tuningérték és hardver-evidence
nem ennek a dokumentumnak a feladata.

Authority sorrend:

```text
source + aktív config
→ ez a canonical V3 contract
→ Replayer + Test Hub V3 futásazonos evidence
→ történeti dokumentáció vagy nyers log
```

A dokumentum a V3 stabil architekturális határait rögzíti. Nem ír elő
indokolatlan jövőbeli frameworköt vagy konkrét algoritmust.

## 1. Minimum, nem alkuképes garanciák

* Minden állapotnak és rétegnek pontosan egy owner-e van.
* Nincs legacy shared state, rejtett singleton vagy kerülő authority.
* A réteghatárok immutable, konkrét Python típusok; `dict[str, Any]` nem
  réteghatár-contract.
* Egyetlen, szekvenciális TickEngine egy már lezárt TickInputs snapshotból, rögzített   sorrendben legfeljebb egyszer hívja a szükséges rétegeket.
* Egy tickből pontosan egy L12 final döntés és legfeljebb egy normál motor-write
  születik.
* Az L12 hiányzó, hibás vagy bizonytalan, az adott mozgáshoz ténylegesen
  kritikus inputnál fail-closed STOP/FAULT döntést és fizikailag inaktív
  motorállapotot eredményez.
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
Minden más mechanizmus csak akkor indokolt, ha konkrét runtime-, safety-,
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

A döntési idő kizárólag az injektált monoton idő. Randomizált algoritmus csak
rögzített, replayelhető seedből dolgozhat. CPU-idős deadline helyett bounded,
determinisztikus munka-budget szükséges ott, ahol a döntésnek reprodukálhatónak
kell maradnia.

A live, capture, replay és szimuláció ugyanarra a kis végrehajtási határra
illeszthető:

```text
input source → production V3 → output sink
```

Az input source lezárt `TickInputs` értéket ad, a production elem a canonical
V3 végrehajtást futtatja, az output sink pedig passzív eredményt fogyaszt. A
sink nem adhat vissza control state-et és nem kaphat motor-, lifecycle- vagy
safety-authorityt.

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

Validáció ott kötelező, ahol közvetlen runtime-, safety- vagy determinisztikai
értéke van, például:

* véges számok, fizikai tartományok és nemnegatív idő/sorszám;
* egy ticken belüli azonos `TickContext`;
* measurement/source idő egyértelmű jelentése;
* freshness, ordering és trust;
* domain-invariánsok;
* STOP/FAULT esetén null logikai final output és fizikailag inaktív motor-edge;
* kritikus azonosítók és gyűjteménykulcsok egyértelműsége.

A fizikai measurement idő és a feldolgozási/result idő nem keverhető.
Freshness annak a fizikai mérésnek az idejéből számítandó, amelyre az eredmény
vonatkozik.

A diagnosztikai ok rövid stabil `reason`, nem általános proof- vagy reason-code
gráf. A capture edge használhat egyszerű verziózott fájlformátumot, de a
serializáció nem része minden runtime contractnak.

## 4. Rétegek és state-ownership

| Réteg | Egyetlen felelősség és owned state | Typed output |
| --- | --- | --- |
| L0 Device HAL | eszközhandle, busz, fizikai read/write, lezárt device snapshot | `RawDeviceBatch` |
| L1 Acquisition | a lezárt device snapshotból source sample-ek és I/O health zárása | `AcquisitionFrame` |
| L2 Admission | freshness, sorrend, duplikáció, trust/alignment history | `AdmittedFrame` |
| L3 State Estimation | pose, twist, covariance | `RobotEstimate` |
| L4 World Model | rolling lokális világállapot, costmap, revision és akadályhistory | `WorldSnapshot` |
| L5 Command & Mission | validált command- és mission-lifecycle | `MissionIntent` |
| L6 Navigation | route/progress, coverage/local goal és trajectory evaluation | `NavigationPlan` |
| L7 Motion Selection | prioritás és pontosan egy kiválasztott trajectory/cél | `MotionObjective` |
| L8 Motion Realization | a kiválasztott guidance pillanatnyi kinematikai célja | `MotionIntent` |
| L9 Operational Constraints | dinamikai/környezeti korlátozás state | `ConstrainedMotion` |
| L10 Chassis Control | chassis-kinematika | `WheelVelocitySetpoint` |
| L11 Actuator Control | wheel-loop integrátor, feed-forward, calibration map | `ActuatorRequest` |
| L12 Safety & Final | safety latch, final döntés, egyetlen normál `MotorWriter` | `FinalActuation` |
| Composition root | tick, lifecycle, config snapshot és wiring | `TickTrace` |

Egy réteg nem módosíthat másik réteg state-jét. Az output új immutable érték;
nem adhat át controllert, GUI objektumot, device handlet vagy mutable
collectiont.

A rétegezés nem jelenti azt, hogy egy fizikai szenzorhoz pontosan egy
réteghatár-output tartozhat. Egyetlen acquisition forrásból több, eltérő célú
typed sample zárható, ha ownershipjük és jelentésük egyértelmű.

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
* érvényes safety observation közvetlenül korlátozhat vagy megtilthat
  actuationt;
* csak érvényes L11 request, megfelelő lifecycle és minden ténylegesen
  szükséges safety feltétel esetén `ALLOW`;
* egy normál döntés után legfeljebb egy atomi writer-hívás;
* write exception után nincs automatikus második normál írás, a fault latch
  beáll;
* STOP/FAULT contract nem tartalmazhat nem nulla logikai final outputot.

A STOP/FAULT nem pusztán `duty=0` szoftveres értéket jelent. A motor-edge
shutdown contractjának a hardver szempontjából fizikailag inaktív, igazolható
állapotot kell eredményeznie.

A `DeviceHealth` és a szenzor által megfigyelt veszély két külön fogalom. Egy
magasabb szintű feldolgozás hibája nem változtathat működő hardverforrást
automatikusan hibás device-zá.

Külön általános `ActuationReceipt` rendszer nem kell. Ha a hardver alkalmazott
érték-visszaolvasást igényel, az egyszerű typed device feedbackként kerül
L0/L1-be.

## 7. Szenzoradat és capability-függetlenség

Egy fizikai szenzor nem egyetlen algoritmus tulajdona.

A szenzorrendszer külön kezeli:

1. a fizikai eszköz/stream működőképességét;
2. az adott mérés érvényességét, freshness-ét és trustját;
3. az egyes feldolgozási/felhasználási ágak minőségét.

Egy ág failure/degradation állapota csak akkor terjedhet másik ágra, ha közös
fizikai vagy contract-szintű oka van. A safety számára önmagában használható
mérés nem válhat használhatatlanná pusztán azért, mert ugyanabból a forrásból
egy localization-, world-model- vagy más magasabb szintű ág nem tudott megfelelő
eredményt előállítani.

A capability-k szétválasztása nem jelent párhuzamos control authorityt. A
különböző szenzorágak csak typed adatot szolgáltatnak; a végső actuation
authority továbbra is az L12.

### 7.1 Encoder invariánsok

A fizikai számláló/delta és az abból közvetlenül származó elmozdulás RAW
measurement; nem írható át sebességszűrő vagy control-output alapján.

A keréksebesség becslése lehet stateful és időablakos, de bounded és
determinisztikus marad. A measurement trust/timing minősége külön fogalom a
fizikai encoder device health állapotától. L2 a measurement admissiont, L3 az
állapotbecslést birtokolja.

A konkrét pulse-window, debounce, CPR, velocity limit és egyéb tuning az aktív
config és source része, nem architekturális contract.

### 7.2 LiDAR szemantikai ágak

Egy fizikai LiDAR acquisition több, egymástól független typed eredményt
szolgáltathat:

```text
LiDAR acquisition
   ↓
L1
   ├── collision/safety measurement ─────────→ L12
   ├── local perception ───────────→ L2 ─────→ L4
   └── localization measurement ───→ L2 ─────→ L3
```

A safety ág a közvetlen collision-releváns mérésből dolgozik. Nem függhet scan
matcher sikerétől, localization pose-tól, térképtől vagy magasabb szintű
perception eredménytől, ha a saját safety measurement önmagában érvényes.

A local-perception ág az L4 számára szükséges legegyszerűbb bounded, immutable
lokális környezet-reprezentációt szolgáltatja. A world state/costmap ownershipje
L4-ben marad, nem kerülhet a device adapterbe.

A localization ág opcionális L3 measurement. Hiánya vagy alacsony minősége nem
jelent automatikusan LiDAR device failure-t, és nem érvényteleníti a független
safety/local-perception ágat.

A measurement timestamp a fizikai mérés idejét jelenti. A feldolgozás
befejezési ideje külön diagnosztikai adat lehet. Raw scan csak akkor kerül
magasabb szintű vagy capture adatba, ha annak konkrét funkcionális vagy
diagnosztikai értéke van.

Egy teljes LiDAR scan scan-szintű időcontractja a mért
`scan_start_monotonic_ns`, `scan_end_monotonic_ns` és az ezekből kizárólag
`start + (end - start) // 2` képlettel képzett `measurement_monotonic_ns`.
A megtartott `captured_monotonic_ns` mező scan-completion timestamp; matcher
pose-illesztésre nem használható. Ez a contract nem pontonkénti deskew.

A matcher packet source scan revisiont, measurement timestampet és pontosan
arra az időre vonatkozó pose reference-et visz; a pose-reference és measurement
monoton időbélyege kötelezően azonos. A már elkészült `RobotEstimate` értékek
scan-időre illesztéséhez a production hardware feedback edge egyetlen ownere
bounded, monoton rendezett pose-historyt tarthat fenn exact lookupkal,
lineáris x/y és wrap-safe yaw interpolációval. Túl régi, jövőbeli vagy nem
interpolálható kérés fail-closed. Ez a hardware-edge state nem része a
`NativeControlComposition` state-authorityjának.

A production szenzorút natív V3 adapterekből és typed contractokból áll; nem
függhet legacy runtime/shared-state authoritytól.

## 8. Motion és command szabadság

Az L7→L12→MotorWriter lánc a stabil canonical motion core. Új command-, mission- vagy behavior capability nem kerülheti meg és nem duplikálhatja; ha a meglévő motion contract elegendő, az új funkció a CommandGateway/L5/L6 oldalán kapcsolódjon be.

Production motion live teszt csak a teljes CommandGateway→L5→L6→L7→L8→L9→L10→L11→L12→MotorWriter láncon keresztül végezhető. Célzott actuator-, motor-edge- vagy hardvervalidáció használhat szűkebb utat, de nem tekinthető production motion tesztnek és nem bizonyítja a teljes motion láncot.

Az L7–L12 mag stabil by default: csak akkor változzon, ha a konkrét funkció vagy igazolt source/replay/live evidence ténylegesen ezt igényli; pusztán új command, mission vagy behavior hozzáadása nem indok az átépítésére.

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
rögzített motion-primitive fogalmakra. Külső command source általános
kinematikai célt vagy magasabb szintű mission/navigation célt adhat a canonical
command út felé.

Egy konkrét tesztszcenárió nem runtime primitive, hanem egy általános command
source kimenete.

Egy motion request csak azokhoz a capability-khez köthető kötelező gate-ként,
amelyek az adott mozgás szemantikájához vagy biztonságos végrehajtásához
ténylegesen szükségesek. Opcionális magasabb szintű capability nem válhat
pusztán a létezése miatt minden motion globális blokkoló feltételévé.

A navigation L6 felelőssége a bounded trajectory-jelöltek előállítása és
értékelése. Az L7 pontosan egy `MotionObjective` értéket választ, az L8 pedig a
kiválasztott guidance pillanatnyi kinematikai célját realizálja. Külön
behavior/planner nem hozhat létre második motion-selection vagy safety
authorityt.

## 9. Konfiguráció, command és külső I/O

A composition root validált, immutable konfigurációt injektál. Layer nem olvas
fájlt, environment variable-t vagy globális config managert.

Actuationt érintő konfiguráció csak biztonságos lifecycle-határon és
fizikailag inaktív motorállapot mellett cserélhető.

GUI, CLI, LLM és tool csak a `CommandGateway` kliensén keresztül adhat typed
`CommandRequest` értéket. A gateway kezeli a külső command érvényességét és
lejáratát; ezek adminisztratív részletei nem terjednek végig a belső layer
contractokon.

Readiness vagy arming kapu csak valóban friss, egymástól független
forrásevidence-et számolhat új bizonyítéknak; ugyanazon latest-only source
revízió ismételt kiolvasása nem új measurement. Kötelező gate csak olyan
capability lehet, amely az adott funkció biztonságos végrehajtásához ténylegesen
szükséges.

Resident ACTIVE csak érvényes preflight után indulhat. Command lejárat vagy
rövid command-kimaradás fail-closed STOP-ot eredményez; visszaaktiválás a normál
readiness/preflight úton történik, alternatív motor- vagy safety-út nélkül.

A runtime headless. Külső I/O kizárólag edge/device adapterben történik, és az
adapter nem válhat state- vagy control-authorityvá.

## 10. V3-only source szabály

A V3 modulok csak a V3 csomagot, a standard libraryt és az explicit aktív
numerikus/hardver függőségeket importálhatják. Legacy source-import,
compatibility adapter, donor-allowlist, shared state és alternatív runtime
authority tilos.

Layer implementation más layer implementationt nem importálhat; a wiring a
composition root felelőssége.

Új capability minimális natív V3 implementációként, a typed V3 contractokból
indulva készülhet.

## 11. Capture, Replay és Test Hub

A capture és replay fejlesztői/diagnosztikai capability, nem control authority.
A capture hibája vagy lassúsága nem módosíthatja a robot döntését vagy
motor-outputját.

A production tick útból történő capture megfigyelés bounded és nem blokkoló.
Encoding és tartós fájl-I/O nem lehet a control tick kritikus útjának része.

A replay minimuma:

* a futtatandó tickek lezárt `TickInputs` értékei;
* a ténylegesen használt konfiguráció;
* minden további olyan determinisztikus state/input, amely nélkül a kivágott
  futásrész nem reprodukálható.

A Replayer ugyanazt a production `NativeControlComposition` utat futtatja
offline. Nem imitálhat layer-viselkedést saját algoritmussal. Az eredményeket
közvetlen typed érték-összehasonlítással vizsgálja, és az első eltérő ticket,
réteget és mezőt jelöli meg.

FAIL/FAULT futás ugyanúgy replayelhető evidence, mint PASS. Upstream hiba esetén
a capture/replay nem gyárthat fiktív outputot a nem futott rétegekhez; a valós
lezárt prefixet és a final safety eredményt őrzi meg.

Stateful futásrész csak akkor kaphat pontos MATCH verdictet, ha a Replayer a
kivágás előtti szükséges állapotból indul. Ez származhat a capture-ben meglévő
prefixből vagy bounded production-state checkpointból. A checkpoint kizárólag
replay-indulóállapot; nem lehet live control authority és nem helyettesíthet
hiányzó tick inputot.

A capture csak a konkrét diagnosztikához szükséges evidence-et őrizze meg.
Nincs általános „mindent logoljunk” követelmény. Raw szenzoradat csak indokolt
esetben szükséges, és hiánya esetén a fizikai root cause nem állítható
bizonyítottnak.

Scan-matcher temporal diagnosztikánál a capture közvetlenül őrzi a source scan
revisiont, scan start/end/measurement időt, pose-reference időt és azok tényleges
időkülönbségét. A processing latency külön mező; nem helyettesítheti a
pose-reference timestampet.

Nem kötelező:

* minden event vagy payload SHA-256 hash-e;
* schema registry és mezőnkénti runtime schema validation;
* causation/provenance gráf;
* build/source fingerprint a control contractban;
* layerenkénti receipt vagy külön proof objektum;
* canonical round-trip teszt minden belső üzenethez;
* teljes fizikai robot- vagy sensor-driver szimulátor minden replayhez.

Egyszerű capture/result checksum használható fájlsérülés észlelésére, de nem
válhat runtime identityvé vagy döntési inputtá.

A V3 Test Hub vékony külső eszköz a canonical Replayer körül. Nem futtathat
saját layer-logikát, nem tarthat fenn alternatív decodert vagy motion/safety
utat, és nem lehet production runtime dependency.

## 12. Kötelező, célzott tesztkapuk

A V3 architektúra alapkapui:

* import guard: nincs cross-layer implementation import, külső shared-state,
  GUI/tool authority vagy V3-on kívüli project-import;
* contractteszt: immutable típusok és közvetlen domain/safety invariánsok;
* TickEngine teszt: lezárt snapshot, rögzített sorrend, rétegenként legfeljebb
  egy értékelés és egyetlen L12 final döntés;
* fail-closed teszt: upstream exception, invalid tick/lifecycle, kritikus
  input-hiba és writer failure;
* replayteszt: azonos input/state esetén azonos trace, eltérésnél helyes első
  divergáló réteg;
* motor-edge változásnál annak célzott bizonyítása, hogy STOP/FAULT fizikailag
  inaktív állapotot eredményez.

Szenzor- vagy algoritmusspecifikus változás a saját közvetlen invariánsait
célzott teszttel bizonyítja. Többcélú szenzorág esetén különösen fontos, hogy
egy független magasabb szintű ág hibája ne tegye érvénytelenné az önmagában
használható safety ágat.

Nincs általános kötelező schema-, hash-, provenance-, receipt- vagy formális
proof-kapu.

A célzott teszt az alapértelmezett; széles regresszió csak a ténylegesen érintett
közös boundary vagy kockázat miatt indokolt.

## 13. Fejlesztési határ

Ez a dokumentum nem fejlesztési workflow.

A stabil architektúrát csak konkrét source-, replay- vagy fizikai evidence által
igazolt igény miatt kell módosítani. Új capability a legkisebb teljes natív V3
szelet legyen; ne épüljön általános framework feltételezett későbbi igényre.

Meglévő safety- vagy quality gate-et nem szabad pusztán azért lazítani, hogy egy
teszt átmenjen. Ha egy gate tévesen több független capabilityt köt össze, a
coupling gyökerét kell kijavítani.

A pillanatnyi implementációs állapot authorityja a canonical source, az aktív
config, a célzott tesztek és a run-bound evidence.
