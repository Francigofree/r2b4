# R2B4 V3 robotarchitektúra — egyszerű rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló és nem formális
bizonyítási rendszer.

**Cél:** egyszerű, determinisztikus, jól tesztelhető és hosszú távon
karbantartható AMR runtime. A szerkezet a stabil működést és a gyors
hibakeresést szolgálja; nem cél ipari minőségű provenance- vagy auditlánc.

**Runtime-aktiválás:** `IDLE_ONLY_CUTOVER_CANDIDATE`. Az explicit emberi `GO`
egy külön headless V3 candidate megépítését engedélyezi, amely kizárólag
`BOOTING -> IDLE/FAULT/SHUTDOWN` állapotot és fizikai PWM-null kimenetet tud.
Ez nem engedélyez `ACTIVE` átmenetet, live motiont, automatikus promotiont vagy
az `os.py` legacy defaultjának importkori kiváltását.

## 1. Minimum, nem alkuképes garanciák

- Minden állapotnak és rétegnek pontosan egy owner-e van.
- Nincs legacy shared state, rejtett singleton vagy kerülő authority.
- A réteghatárok immutable, konkrét Python típusok; `dict[str, Any]` nem
  réteghatár-contract.
- Egyetlen, szekvenciális `TickEngine` zárja az input snapshotot, majd rögzített
  sorrendben egyszer hívja a rétegeket.
- Egy tickből pontosan egy L12 final döntés és legfeljebb egy atomi motor-write
  születik.
- Az L12 final safety hiányzó, hibás vagy bizonytalan kritikus inputnál STOP/FAULT
  döntést és null outputot ad.
- Azonos input, konfiguráció és kód azonos typed layer-outputokat eredményez.
- Replay eltérésnél a legelső eltérő tick és réteg közvetlen érték-összehasonlítással
  megnevezhető.
- Legacy komponens csak tiszta donor vagy vékony adapter lehet; legacy authority
  és objektumreferencia nem kerülhet V3-ba.

Ezeket a garanciákat nem szabad adminisztratív könnyítés címén lazítani. Minden
más mechanizmus csak akkor kerülhet be, ha egy konkrét runtime-, safety-, replay-
vagy hibakeresési igényt egyszerűbben megold, mint nélküle.

## 2. Determinisztikus végrehajtás

A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló
folyamatban nincs rétegenkénti thread, sleep, falióra, rejtett I/O vagy
modulglobális mutable state.

Egy tick menete:

1. A composition root létrehozza a `TickContext(tick_id, monotonic_ns)` értéket.
2. Lezárja a tick nyers device- és command-inputját.
3. L1-től L11-ig minden réteget legfeljebb egyszer, rögzített sorrendben hív.
4. Bármely upstream hiba esetén a normál lánc megszakad, és az L12 pontosan
   egyszer explicit fault okkal kerül meghívásra.
5. Az L12 dönt és birtokolja az egyetlen normál `MotorWriter` capabilityt.
6. A tick typed trace-e diagnosztikához kiolvasható, de nem hat vissza a controlra.

A döntési idő kizárólag az injektált monoton idő. Randomizált algoritmus csak a
replay inputjában rögzített seedet használhat. CPU-idős deadline helyett fix
iteráció-, minta- vagy csomópont-budget kell.

## 3. Egyszerű contractmodell

Minden top-level layer output egy frozen, slotted dataclass. A közös metadata
mindössze:

```text
TickContext
  tick_id: int
  monotonic_ns: int
```

Nincs kötelező közös schema envelope, schema registry, producer/session
provenance, causation chain, config hash, event hash vagy boundarynkénti canonical
serializer. A Python típusdefiníció maga a belső contract. Inkompatibilis
változást a hívó kód és a célzott contractteszt együtt követ.

Validáció csak ott kötelező, ahol közvetlen értéke van:

- véges számok, fizikai tartományok és nemnegatív idő/sorszám;
- egy ticken belüli azonos `TickContext`;
- domain-invariánsok, például aktív tervhez nem üres route;
- STOP/FAULT esetén null final output;
- kritikus azonosítók és gyűjteménykulcsok egyértelműsége.

A diagnosztikai ok egy rövid stabil `reason`, nem általános proof- vagy
reason-code gráf. A capture edge később használhat egyszerű verziózott
fájlformátumot, de a serializáció nem része minden runtime contractnak.

## 4. Rétegek és state-ownership

| Réteg | Egyetlen felelősség és owned state | Typed output |
|---|---|---|
| L0 Device HAL | eszközhandle, busz, fizikai read/write | `RawDeviceBatch` |
| L1 Acquisition | polling/stream, source sequence, I/O health | `AcquisitionFrame` |
| L2 Admission | freshness, sorrend, duplikáció, trust/alignment history | `AdmittedFrame` |
| L3 State Estimation | pose, twist, covariance | `RobotEstimate` |
| L4 World Model | map revision, occupancy és akadálytrack history | `WorldSnapshot` |
| L5 Command & Mission | validált command- és mission-lifecycle | `MissionIntent` |
| L6 Navigation | route, corridor és progress | `NavigationPlan` |
| L7 Motion Selection | prioritás és egyetlen kiválasztott cél | `MotionObjective` |
| L8 Motion Realization | guidance és pillanatnyi kinematikai cél | `MotionIntent` |
| L9 Operational Constraints | dinamikai/környezeti korlátozás state | `ConstrainedMotion` |
| L10 Chassis Control | chassis-kinematika | `WheelVelocitySetpoint` |
| L11 Actuator Control | wheel-loop integrátor, feed-forward, calibration map | `ActuatorRequest` |
| L12 Safety & Final | safety latch, final döntés, egyetlen `MotorWriter` | `FinalActuation` |
| Composition root | tick, lifecycle, config snapshot és wiring | `TickTrace` |

Egy réteg nem módosíthat másik réteg state-jét. Az output új immutable érték; nem
adhat át controllert, GUI objektumot, device handlet vagy mutable collectiont.

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

Layer implementation nem importálhat másik layer implementationt. Kapcsolás
csak a composition rootban, typed callable/port konstrukcióval történhet.

## 6. Final safety és motorírás

Az L12 a normál motor-write capability egyetlen tulajdonosa. Nincs alternatív
PWM-, service-, GUI-, tool- vagy donor-writer.

Az L12 kötelező viselkedése:

- upstream exception, kritikus device failure vagy hiányzó actuator request:
  latch-elt `FAULT`, null output;
- ismeretlen kritikus device állapot vagy nem `ACTIVE` lifecycle: `STOP`, null
  output;
- csak érvényes L11 request és `ACTIVE` lifecycle esetén `ALLOW`;
- egy döntés után pontosan egy atomi writer-hívás; write exception után nincs
  automatikus második írás, a fault latch beáll;
- STOP/FAULT contract nem tartalmazhat nem nulla outputot.

Külön `ActuationReceipt` esemény nem kell. A sikeres atomi write a metódus normál
visszatérése; write exception azonnali hiba, a driver/device health pedig a
következő input snapshot része. Ha a hardver később valóban igényel alkalmazott
érték-visszaolvasást, az egyszerű typed device feedbackként kerül L0/L1-be, nem
általános receipt-rendszerként.

## 7. Konfiguráció, command és külső I/O

A composition root validált, immutable konfigurációt injektál. Layer nem olvas
fájlt, environment variable-t vagy globális config managert. Actuationt érintő
konfiguráció csak tick-határon, `IDLE` és null output mellett cserélhető.

GUI, CLI, LLM és tool csak a `CommandGateway` kliensén keresztül adhat typed
`CommandRequest` értéket. A gateway kezeli a külső hitelesítést és lejáratot; ezek
teljes provenance-ének minden belső üzenetben való ismétlése nem szükséges.

A runtime headless. Külső I/O kizárólag edge adapterben történik, és az adapter
nem válhat state-authorityvá.

## 8. Legacy donor szabály

Legacy import csak `v3.adapters.legacy_donors` alatt és pontos allowlisttel
engedélyezett. Donor akkor használható, ha:

1. nincs `ctrl`, global state, GUI vagy legacy authority függése;
2. idejét, konfigurációját és I/O-ját argumentumból/portból kapja;
3. offline fake-kel determinisztikusan fut;
4. bemenete és kimenete V3 contracttá alakítható legacy referencia nélkül;
5. célzott karakterizációs teszt fedi a megtartani kívánt algoritmust;
6. nem nyit motor- vagy safety-bypass útvonalat.

Ha ehhez nagy adapter vagy legacy orchestration kellene, csak a tiszta
algoritmikus mag emelhető át.

## 9. Replay és hibakeresés

A replay célja reprodukálni a döntést és gyorsan megtalálni az első hibás
réteget. A minimális replay input:

- tickenként a lezárt `RawDeviceBatch`, `CommandRequest`, lifecycle és
  `TickContext`;
- a futáshoz ténylegesen használt konfiguráció egyszer, tartalom szerint;
- randomizált algoritmus esetén a seed.

A `TickTrace` tickenként L1–L12 typed outputokat tartalmaz. Két futás közvetlen
dataclass-egyenlőséggel hasonlítható össze; az első eltérő rekord megadja a
`tick_id`-t és a layer nevet. Nem kötelező:

- minden event vagy payload SHA-256 hash-e;
- schema registry és mezőnkénti runtime schema validation;
- causation/provenance gráf;
- build/source fingerprint a control contractban;
- layerenkénti receipt vagy külön proof objektum;
- canonical round-trip teszt minden belső üzenethez.

Capture-fájl checksum használható egyszerű fájlsérülés-ellenőrzésre, de nem válik
runtime identityvé vagy döntési inputtá. A trace/log/evidence hibája soha nem
módosíthat control outputot.

## 10. Kötelező, célzott tesztkapuk

Egy V3 source candidate minimum tesztjei:

- import guard: nincs cross-layer implementation import, legacy shared-state,
  GUI/tool authority vagy donor-allowlist bypass;
- contractteszt: immutable típusok és a közvetlen domain/safety invariánsok;
- TickEngine teszt: lezárt snapshot, rögzített sorrend, rétegenként legfeljebb egy
  értékelés és egyetlen L12 commit;
- fail-closed teszt: upstream exception, invalid tick/lifecycle, missing/failed
  kritikus input és writer failure;
- replayteszt: azonos inputra azonos trace, módosított layer-outputnál helyes első
  divergáló réteg;
- donoronként külön offline karakterizációs teszt.

Nincs általános kötelező schema-, hash-, provenance-, receipt- vagy formális
proof-kapu.

## 11. Fejlesztési terv

1. **Egyszerű foundation — megvalósítva a jelen candidate-ben:** `TickContext`,
   immutable typed contractok, import guard, determinisztikus `TickEngine`, typed
   `TickTrace`, közvetlen first-divergence és fake-writeres L12.
2. **STOP-only vertikális szelet — megvalósítva a jelen candidate-ben:** valós
   composition root, injektált fake HAL és command gateway, valamint explicit
   `BOOTING -> IDLE/FAULT/SHUTDOWN` lifecycle. Nincs `ACTIVE` átmenet. Az L1-L11
   lánc végig typed, determinisztikus STOP/null értékeket ad; L0- vagy gateway-
   snapshot hiba is pontosan egy L12 fail-closed döntéshez fut, és minden
   elérhető normál/fault állapot null final outputtal zárul.
3. **L10–L11 donor slice — megvalósítva a jelen candidate-ben:** injektált
   nyomtávból számolt differenciálhajtás-kinematika, immutable és induláskor
   validált `R2B4_WHEEL_SPEED_MAP_V2`, valamint L11-owned kétcsatornás PI state.
   A nem nulla offline L11 hívás pontosan egy admitted `wheel_velocity`
   observationt fogad `left_mps` és `right_mps` mezőkkel; a `dt` kizárólag az
   egymást követő `TickContext.monotonic_ns` értékekből származik. A tiszta V2.1
   kinematika-, speed-map- és PI donorral célzott karakterizációs teszt hasonlítja
   össze. A STOP-only composition továbbra is az explicit null L10/L11 utat és
   fake writert használja, nincs `ACTIVE` átmenet vagy új motorút.
4. **Input és becslés shadow slice — megvalósítva a jelen candidate-ben:** a
   read-only L0 edge a canonical Replayer V1 `sensor_feedback` frame-jeit
   immutable encoder-, EKF-heading- és lidar-health mintákká zárja; hiányzó
   kritikus mező typed `UNKNOWN` healthként, sample nélkül jut a fail-closed
   láncba. Az L2 saját sequence/freshness/alignment historyt, az L3 injektált
   geometriával determinisztikus pose/covariance state-et, az L4 pedig
   lidar-revision/freshness és typed obstacle-track historyt birtokol. A külön
   `InputShadowComposition` a teljes L1–L12 `TickEngine` láncot `IDLE`/STOP
   módban, kizárólag zero-only non-physical sinkkel replayeli. A canonical valós
   capture-ök és a repo-ba zárt post-promotion excerpt azonos inputra azonos
   trace-t adnak; nincs `ACTIVE` átmenet, live reader vagy fizikai motorút.
5. **Mission/navigation slice — megvalósítva a jelen candidate-ben:** az L5 a
   gateway `NAVIGATE`, `TELEOP`, `STOP` és tiltott `SERVICE` bemenetét validálja,
   majd typed célra, limitekre és mission-lifecycle-ra zárja. Az L6 közvetlen
   route-ot, corridor-ellenőrzést és monoton mission-progresszt birtokol; az L7
   pontosan egy route-, velocity- vagy STOP-objective-et választ. Az L8 kizárólag
   a kiválasztott objective, a friss pose és world snapshot alapján képez
   pillanatnyi kinematikai célt. Az L9 saját előző-output state-tel mission-,
   platform-, curvature-, lokalizáció- és gyorsuláskorlátot alkalmaz. A külön
   `MissionNavigationComposition` L5–L9 scenario replayeket futtat writer,
   lifecycle-aktiválás és fizikai motorút nélkül; azonos input azonos trace-t ad,
   hibás command, blokkolt route, célba érés és degradált lokalizáció null
   constrained motionnel zárul.
6. **Teljes fake-only composition — megvalósítva a jelen candidate-ben:** az
   immutable konfigurációból huzalozott `FullFakeComposition` az L1–L12 valós V3
   implementációit egyetlen szekvenciális `TickEngine`-ben futtatja, kizárólag
   előre lezárt typed inputokkal és device handle nélküli, memóriabeli L12
   sinkkel. A 256 tickes ACTIVE scenario két friss composition példányon
   közvetlen dataclass-egyenlőséggel azonos trace-et és write-sorozatot ad. Az
   L1–L11 minden rétege tickhez kötötten fault-injektálható és pontosan egy null
   L12 FAULT döntéshez fut; a következő tick a stateful rétegeket determinisztikusan
   újrahorgonyozza. Az injektált L12 writer-hiba egyetlen write-kísérlet után
   latch-el, retry nélkül. Nincs reader, thread, falióra, live lifecycle-aktiválás
   vagy fizikai motorút.
7. **Read-only shadow — megvalósítva a jelen candidate-ben:** a spawnolt
   `ReadOnlyShadowSidecar` kizárólag már lezárt, immutable `RawDeviceBatch`
   értékeket vesz át typed IPC-n. A child processz saját stateful
   `InputShadowComposition` példányt és zero-only memóriasinket birtokol, majd
   immutable `ShadowTickResult` értékben adja vissza a trace-t és a null final
   döntést. A sidecar API nem fogad device readert, motor writert, legacy runtime
   objektumot vagy shared state-et; a hibás tick-sorrend a childban egyetlen null
   L12 FAULT döntéssel zárul. Nincs live lifecycle-aktiválás vagy fizikai motorút.
8. **Külön emberi cutover — IDLE-only candidate megvalósítva, promotionra vár:**
   a `v3_idle_runtime.py` külön, explicit headless entrypointként immutable
   hardverkonfigurációt zár, majd egyetlen `LiveIdleComposition` owner loopot
   futtat. A saját GPIO edge-adapter nem importál legacy drivert vagy globális
   config managert, induláskor nullázza és egy handle alatt birtokolja a négy
   motorpint, majd minden nem nulla vagy `ALLOW` final parancsot hard-faillel
   elutasít. A writer nem kerül ki az L12-t huzalozó compositionből; a command
   gateway csak STOP-ot ad, nincs `activate` API. A canonical `os.py` default és
   a legacy runtime ebben a candidate-ben nem változik, ezért source-promotion
   önmagában nem indít fizikai I/O-t. Tényleges futtatás csak újabb friss
   preflight, egyetlen runtime/writer ellenőrzés és külön explicit promotion
   után történhet.
9. **Legacy eltávolítás:** GUI/tool/shared-state és régi motorutak külön, célzott
   taskokban.

Minden fázis csak a következő fázishoz közvetlenül szükséges kódot és tesztet
adja hozzá. Új evidence- vagy metadata-mechanizmust konkrét diagnosztikai hiba
nélkül nem vezetünk be.
