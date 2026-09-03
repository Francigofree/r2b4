# R2B4 V3 robotarchitektúra — egyszerű rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló és nem formális
bizonyítási rendszer.

**Cél:** egyszerű, determinisztikus, jól tesztelhető és hosszú távon
karbantartható AMR runtime. A szerkezet a stabil működést és a gyors
hibakeresést szolgálja; nem cél ipari minőségű provenance- vagy auditlánc.

**Runtime-aktiválás:** `IDLE_ONLY_CUTOVER_COMPLETE`. Az auditált, külön headless
V3 runtime kizárólag `BOOTING -> IDLE/FAULT/SHUTDOWN` állapotot és fizikai
PWM-null kimenetet tud. Az explicit source-promotion és az élő IDLE start/stop
cutover lezárult. Ez továbbra sem engedélyez `ACTIVE` átmenetet, live motiont,
automatikus runtime-indítást vagy az `os.py` legacy defaultjának importkori
kiváltását.

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

Felső szintű legacy komponens alapértelmezés szerint nem portolandó. GUI, CLI,
Test Hub, Replayer, state machine, Core/Task/AI orchestration, Follow/Search
orchestration és legacy command/status infrastruktúra csak egy konkrét következő
V3 funkció közvetlen igényére kap megfelelő V3 elemet. Az új elem ilyenkor
minimális native V3 implementáció, amely a V3 contractokból indul ki; a legacy
kód legfeljebb algoritmikus referencia vagy tiszta donor. Legacy API-,
viselkedés- vagy struktúraparitás nem követelmény.

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
8. **Külön emberi cutover — IDLE-only lezárva:** a `v3_idle_runtime.py` külön,
   explicit headless entrypointként immutable hardverkonfigurációt zár, majd
   egyetlen `LiveIdleComposition` owner loopot futtat. A saját GPIO edge-adapter
   nem importál legacy drivert vagy globális config managert, induláskor nullázza
   és egy handle alatt birtokolja a négy motorpint, majd minden nem nulla vagy
   `ALLOW` final parancsot hard-faillel elutasít. A writer nem kerül ki az L12-t
   huzalozó compositionből; a command gateway csak STOP-ot ad, nincs `activate`
   API. Az auditált source-promotion, a friss mozgásmentes Test Hub preflight és
   az egyetlen runtime/writer melletti élő IDLE start/stop ellenőrzés lezárult;
   a kimenetek végig nullák maradtak, a szabályos leállítás pedig elengedte a
   GPIO-owner handle-t. A canonical `os.py` default és a legacy runtime nem
   változott. Minden későbbi élő újraindítás ismét friss preflightot, egyetlen
   runtime/writer ellenőrzést és külön explicit emberi engedélyt igényel.
9. **Legacy containment — evidence-bound módon folyamatban:** az első célzott
   szelet kivette a
   `driver/motor.py` önállóan futtatható, közvetlen 50%-os PWM-mozgást indító
   tesztbelépési pontját. A második célzott szelet eltávolította a repositoryban
   fogyasztó nélküli `AlbaMotor.forward()` és `AlbaMotor.backward()` közvetlen
   write API-kat; a jelenlegi legacy default által használt `set_pwm`, valamint a
   fail-closed `stop` és `close` felület változatlan maradt. Az `AlbaMotor`
   harmadik szeletben már nem olvas globális `config_manager` state-et vagy
   configfájlt: frozen, slotted `MotorChannelConfig` értéket kap a legacy startup
   composition ownertől, indulás előtt közvetlen mezővalidációval. Az
   `AlbaMotor` negyedik szelete minden lifecycle-, startup-, emergency- és
   kalibrációs nullázást a stop-only `stop()` capabilityre szűkít; a
   motion-capable `set_pwm()` egyetlen production call site-ja a `cont.py`
   bal/jobb final control-loop commitja. A közvetlen GPIO-null emergency fallback
   változatlanul megmarad, mert nem normál motion writer és driverhiba esetén
   fail-closed leállítási út. Az
   `AlbaMotor` egyelőre könyvtári legacy dependency a canonical `os.py` default
   számára; egyik szelet sem kapcsol át runtime-ot vagy indít élő mozgást.
   További legacy kivonás csak egy konkrét V3 funkció vagy bizonyított
   safety-bypass miatt indul.
10. **V3 hardening és fizikai contract-paritás — lezárva:** az első native
    szelet a motorcsatornák pinjei mellett immutable contractba zárja az aktív
    polaritás- és DRV8871 decay-szemantikát is, ismeretlen értéknél fail-closed
    konfigurációhibával. A zero-only GPIO owner minden pint közvetlenül a claim
    után nulláz, még a következő pin claimje előtt, így részleges inicializálási
    hiba esetén is csak már nullázott pin maradhat a handle-en. Nincs `ACTIVE`,
    nem nulla output vagy új felső szintű komponens. A második native szelet az
    immutable csatornakonfigurációból tiszta, I/O-mentes `Drv8871PwmPlan` értékre
    zárja a normalizált kerékkimenet polaritás-, irány- és coast/brake
    leképezését. A STOP mindkét pinre nulla, a nem véges vagy tartományon kívüli
    bemenet fail-closed hibát ad. A planner nincs a zero-only GPIO writerre
    huzalozva, nem birtokol hardvercapabilityt és nem nyit `ACTIVE` vagy fizikai
    motion útvonalat. A harmadik native szelet egyetlen immutable L12
    `FinalActuation` döntést azonos tick-contextet és safety döntést őrző, páros
    `Drv8871MotorFrame` értékre zár. A négy fizikai pin egyedisége kötelező,
    STOP/FAULT esetén mindkét csatorna minden pinje nulla, `ALLOW` esetén pedig
    pontosan a két tiszta csatornaplan alkotja a frame-et. Ez a frame-képzés is
    I/O- és writer-capability nélküli, és nincs a live GPIO edge-re huzalozva. A
    negyedik szelet külön offline karakterizációs kapuban, a teljes előre/hátra/
    nulla tartomány reprezentatív rácsán, mindkét decay móddal és mindkét invert
    állapottal közvetlenül összehasonlítja a native plant a canonical legacy
    `AlbaMotor` pin-duty eredményével. A legacy import kizárólag a tesztben van;
    a production planner továbbra is csak immutable V3 contractot importál. A
    phase-10 lezárása nem aktiválja a phase-11 runtime-ot vagy fizikai motiont.
11. **Live input és ACTIVE-ready runtime — folyamatban:** a phase-10 kapuk
    lezárása után az első szelet egy injektált, natív L0 aggregációs határt ad.
    A `NativeLiveInputReader` induláskor egyedi device-ID-kre zárt, rögzített
    sorrendű source-listát birtokol; tickenként mindegyik source-ot pontosan
    egyszer, kizárólag a composition által kapott `TickContext` értékkel olvassa,
    majd csak azonos contextű és device-ID-jű immutable `LiveDeviceSnapshot`
    értékeket zár egyetlen `RawDeviceBatch` értékbe. Source-hibánál nincs retry
    vagy részleges batch. A szelet nem tartalmaz konkrét szenzordrivert,
    falióraolvasást, threadet, legacy state-et, live composition-wiringot,
    `activate` API-t vagy nem nulla writer utat; az ACTIVE-ready továbbra sem
    jelent automatikus ACTIVE engedélyezést.
    A második szelet egy `NativeEncoderSource` edge-et ad: az injektált backend
    tickenként egy immutable, explicit sequence/capture-time, bal/jobb m/s,
    trust, stale és timing-valid mezőjű `EncoderVelocityReading` értéket ad. Az
    edge ebből pontosan egy `wheel_velocity` sample-t és OK/DEGRADED/FAILED
    device health értéket zár; timing hiba FAILED, stale vagy alacsony trust
    DEGRADED. A counter backend opcionális immutable `EncoderEdgeDiagnostics`
    rekordja ugyanebben a sample-ben kizárólag megfigyelési célú nyers countot,
    count-deltát, időközt, counter állapotot, read-error/invalid-alert összesítőt
    és deltát, a reject előtt számított sebességet, a plausibility limitet és egy
    stabil rejection code-ot ad; ezek egyike sem írhatja felül a trust-, health-
    vagy safety-döntést. A device-ID és trust küszöb immutable configból érkezik. A legacy
    KIT0085 driver csak forrásreferencia marad: nincs production import, counter-
    vagy reliability-orchestration port, konkrét backend, live wiring vagy
    hardverhozzáférés.
    A harmadik szelet egy `NativeImuSource` edge-et ad: az injektált backend
    tickenként egy immutable, explicit sequence/capture-time, V3 pose-frame-ben
    értelmezett CCW-pozitív yaw és szögsebesség, confidence, 0–3 kalibráció,
    stale és timing-valid mezőjű `ImuHeadingReading` értéket ad. Az edge ebből
    pontosan egy `ekf_heading` sample-t és OK/DEGRADED/FAILED device health
    értéket zár; timing hiba FAILED, stale, alacsony kalibráció vagy alacsony
    confidence DEGRADED. A device-ID és a két küszöb immutable configból
    érkezik. A legacy BNO055 driver csak forrásreferencia marad: nincs
    production import, tengely- vagy mértékegység-konverziós rejtett authority,
    konkrét backend, live wiring vagy hardverhozzáférés.
    A negyedik szelet egy `NativeLidarSource` edge-et ad: az injektált backend
    tickenként egy immutable matcher-result revíziót, capture-time-ot,
    forrásmérés-kort, confidence, stale és timing-valid mezőket tartalmazó
    `LidarHealthReading` értéket ad. Az edge ebből pontosan a meglévő L4
    contracttal egyező `lidar_health` sample-t és OK/DEGRADED/FAILED device
    health értéket zár; timing hiba FAILED, explicit vagy küszöb feletti stale,
    illetve alacsony confidence DEGRADED. A device-ID, confidence-küszöb és
    maximális méréskor immutable configból érkezik. A legacy lidar driver,
    service és scan-matcher contract csak forrásreferencia marad: nincs
    production import, scan-payload, process, queue, thread, konkrét backend,
    live wiring vagy hardverhozzáférés.
    Az ötödik szelet egy `LiveInputComposition` rootot ad, amely pontosan egy
    encoder-, IMU- és lidar-source-ból, ebben a rögzített sorrendben épít
    `NativeLiveInputReader` példányt. Minden kívülről kapott `TickContext`
    értéknél source-onként legfeljebb egy read történik, majd a lezárt batch a
    stateful L1–L4 és a teljes L1–L12 láncon fut át rögzített IDLE lifecycle és
    belső zero-only sink mellett. L0/source hiba retry és részleges batch nélkül
    pontosan egy L12 FAULT/null commitra zárul. A compositionnek nincs
    `activate` API-ja, saját órája, owner loopja, konkrét backendje, hardver- vagy
    nem nulla writer-capabilityje.
    A hatodik szelet a teljes legacy EKF algoritmikus mag és a tényleges V3 L3
    inputcontract összevetése után native `NativeStateEstimator` implementációt
    ad. Az L3-owned öt állapot (`x`, `y`, `yaw`, `v`, `gyro_bias`) tiszta Python
    nonlinear predict/Jacobian, kovariancia-propagáció, wrapped encoder/heading
    Kalman-update, trust/confidence R-skála, NIS gate, stationary ZUPT/bias
    korrekció és kovariancia-stabilizálás útján készít immutable `RobotEstimate`
    értéket. A `dt` kizárólag az egymást követő `TickContext` értékekből ered; gap
    esetén nincs stale motion integráció. A `LiveInputComposition` ezt a native
    estimatort injektálja, ezért a `ShadowStateEstimator` csak offline replay
    szerepben marad. A szeletben még nem létező raw acceleration,
    command-context és abszolút lidar-pose ágat nem pótolja kitalált bemenettel
    vagy legacy hozzáféréssel.
    A hetedik szelet a `LIDAR_FIRST` mód következő legkisebb előfeltételeként az
    abszolút lidar-pose ágat explicit V3 inputtá teszi. Ugyanaz az egy immutable
    matcher-result hordozza a health adatokat és az opcionális `LidarPoseReading`
    értéket; a native edge azonos revízióval és capture-time-mal külön
    `lidar_health` és `lidar_pose` sample-t zár. Pose sample csak timing-valid,
    friss és a konfigurált confidence-küszöböt elérő eredményből készül, explicit
    `R2B4_BOOT_ROBOT_MAP` frame-ID-val. Az L2 kindonkénti monoton historyja a
    duplicate vagy out-of-order revíziót nem engedi vissza az L3-ba; az azonos
    contract-szintű rejection diagnosztikát egyszer zárja, mert a
    `RejectedObservation` nem hordoz sample-kindot. A `NativeStateEstimator` a
    friss pose-t confidence- és matcher `r_scale`-függő mérési kovarianciával,
    wrapped yaw-innovációval és közös háromtengelyes NIS-gate-tel korrigálja. A
    tiszta Python magot közvetlen legacy EKF karakterizáció, extrém outlier,
    yaw-wrap és measurement-reuse regresszió fedi. A raw acceleration és
    command-context továbbra is későbbi typed inputbővítés. Nincs production
    `middleware`, NumPy, shared state, API/runtime dependency, falióra, thread,
    processz, hardverimport, `ACTIVE` vagy nem nulla writer út.
12. **Első kontrollált V3 fizikai motion és navigation — folyamatban:** az első,
    még mozgásmentes előkészítő szelet egy hardverfüggetlen `NativeMotorWriter`
    határt ad. Az immutable bal/jobb fizikai konfigurációval egyetlen lezárt L12
    `FinalActuation` döntésből pontosan egy validált `Drv8871MotorFrame` készül,
    majd pontosan egy injektált `MotorFrameSink.write` hívás történik. Sink-hiba
    változatlanul továbbterjed, nincs retry vagy fallback write. A szelet nem ad
    GPIO backendet, device handle-t, owner loopot, lifecycle-/composition-
    bekötést, `ACTIVE` engedélyezést vagy élő mozgást. Ezek csak külön emberi
    kapuval, bounded profillal, friss preflighttal és végső PWM-nullal jöhetnek.
    A második, továbbra is runtime-bekötés nélküli szelet egy natív
    `GpioMotorFrameSink` edge-et ad. Egyetlen injektált GPIO backend-handle alatt
    claimeli a négy immutable konfigurációjú motorpint explicit LOW kezdőszinttel.
    A claimet PWM-busy ellenőrzés, explicit LOW írás és LOW readback követi.
    Minden `Drv8871MotorFrame` előtt determinisztikus, visszaolvasott
    break-before-make LOW történik; csak `ALLOW` frame után kerülhetnek ki a
    validált nem nulla duty értékek. A STOP, FAULT és close minden aktív
    software-PWM rekordot explicit leállít, mind a négy DRV8871 bemenetet LOW-ra
    hajtja és visszaolvassa, legalább 2 ms-ig LOW-on tartja, majd újra ellenőrzi;
    a GPIO-handle csak ezután engedhető el. Busy/cancel backend-hibánál a
    `gpio_free` + LOW-on újraclaim vészút szinkron elveszi a PWM authorityt.
    Más fizikai hibánál ugyanazon sink-híváson belül best-effort hard-low és
    handle-zárás történik, az eredeti exception továbbterjed, és a capability
    többé nem írhat; ez nem második L12 döntés vagy normál retry.
    A sink nincs compositionhöz, command gatewayhez vagy konkrét `lgpio`
    importhoz kötve; nincs owner loop, óra, `ACTIVE` átmenet, runtime cutover,
    hardverfuttatás vagy élő motion.
    A harmadik szelet egy runtime- és layer-pipeline nélküli
    `NativeMotorOutputComposition` rootban zárja össze az előző két határt. Az
    egyetlen immutable `GpioMotorFrameSinkConfig` azonos bal/jobb objektumai
    vezérlik a normalizált output fizikai leképezését és a sink pin-ownolását,
    így a planner és a GPIO edge konfigurációja nem térhet el. A root pontosan
    egy privát `NativeMotorWriter` és egy privát `GpioMotorFrameSink` példányt
    birtokol; kifelé csak a V3 `FinalActuation.write` portot, valamint az
    idempotens, nullázás utáni handle-release `close` műveletet adja. Nincs
    layer, reader, command gateway, lifecycle, `activate`, tick, owner loop,
    runtime entrypoint, hardverfuttatás vagy élő motion.
    A negyedik szelet nem duplikálja az L5 mission- és az L9 platform-,
    gyorsulás- vagy lokalizációs limitjeit, hanem a még hiányzó időbeli korlátot
    zárja explicit inputcontractba. Az immutable `BoundedTeleopProfile` egy
    abszolút kezdő tickből, véges aktív tick-számból, véges velocity targetből és
    annak explicit mission-limitjeiből áll. A hardvermentes
    `BoundedTeleopCommandGateway` kizárólag ebben a zárt tick-ablakban ad az L5
    által közvetlenül elfogadott typed `TELEOP` commandot; előtte és utána mindig
    tick-lokális `STOP` értéket ad. Az abszolút ablak nem tolódik el késői start
    vagy újraolvasás miatt, a gateway nem birtokol órát, számlálót, lifecycle-t,
    `activate` API-t, külső I/O-t, layer- vagy motor-authorityt. Nincs runtime-
    bekötés, hardverfuttatás vagy élő motion.
    Az ötödik szelet egy zárt inputú `NativeControlComposition` core-ban köti
    össze a canonical L1–L12 implementációkat. Az L3 a natív estimatort használja,
    az L11 speed map kötelezően injektált, immutable és `ACTIVE`-validált; az L3
    és L10 nyomtávja ugyanabban a configban egyezésre zárt. A caller már lezárt
    `TickInputs` értéket vagy explicit fault tick-et ad, a core pedig pontosan egy
    injektált writer-porton commitol. A bounded TELEOP ablak memory writerrel
    végigfut az összes rétegen, az ablak után nulla outputra zár, és tickenként
    pontosan egy commit történik; writer-hibánál nincs retry. A core nem birtokol
    readert, command gatewayt, lifecycle-átmenetet, `activate` API-t, output-
    handle-t, GPIO-t, órát vagy owner loopot, és nincs runtime-bekötés,
    hardverfuttatás vagy élő motion.
    A hatodik, nagyobb integrációs szelet egy manual-tick
    `BoundedLiveControlComposition` rootban zárja össze az ordered encoder/IMU/
    lidar live readert, a bounded command gatewayt és a natív control core-t.
    Pontosan az immutable abszolút command-ablak idején választ `ACTIVE`
    lifecycle-értéket; ehhez közvetlenül megelőző, konfigurált kornál frissebb,
    minden source-on OK healthű, teljes L1–L12 IDLE tick és sikeres null commit
    szükséges. Hiányzó vagy stale preflight, részleges/source hiba, upstream
    FAULT vagy aktív ablak közbeni safety STOP az egyszeri sessiont fault-latchbe
    zárja. Az ablak végén a root újra IDLE és null output; writer-hibánál nincs
    retry, a későbbi tickek még source-t sem pollolnak. A rootnak nincs public
    `activate`, saját óra, owner loop, external command API, fizikai writer-
    default, GPIO/output-composition wiring, runtime entrypoint, hardverfuttatás
    vagy élő motion.
    A hetedik szelet egyetlen `BoundedPhysicalControlComposition` ownerben köti
    össze a bounded live-control rootot és a natív motor-output compositiont.
    Minden source és immutable config még GPIO-claim előtt validált; ezután
    pontosan egy output owner és egy handle jut az L12 kizárólagos writer-útjára.
    A teljes source→L1–L12→DRV8871 pin mapping fake szenzor- és GPIO backenddel
    bizonyított: a friss preflight előtt és a véges ablak után minden pin nulla,
    source-fault FAULT/null frame-et ad, fizikai write-hiba ugyanazon sink-hívás
    vésznullázásával zárja a handle-t és nincs retry. Az idempotens root `close`
    aktív, IDLE vagy fault állapottól függetlenül végső nullázás után engedi el a
    sole handle-t. A root fizikailag ready, de nincs concrete `lgpio` import,
    external command API, óra, owner loop, thread/process, runtime entrypoint,
    automatikus start, valós hardverfuttatás vagy élő motion.
    A nyolcadik szelet egy explicit, véges `run_bounded_physical_control`
    owner loopot ad a fizikai root köré. Az immutable runtime-config a tick-
    periódust a preflight freshness korlátján belül zárja; a callbackek és az
    első monotonic clock-érték GPIO-claim előtt validáltak, kezdeti stop kérés
    pedig handle-t sem nyit. A loop tick nullától az abszolút command-ablakot
    követő első IDLE tickig készít `TickContext` értékeket, visszafelé vagy két
    tick között nem haladó óránál fail-closed exceptionnel áll le. Committált
    FAULT után nem pollol vagy tickel újra; normál vég, operator-stop, fault és
    exception esetén is egyetlen `finally` ág nullázza és engedi el a sole GPIO
    handlet. Fake source-, clock- és GPIO-evidence bizonyítja a preflight →
    bounded ACTIVE → post-window IDLE/null sorrendet, a korai stopot, source-
    faultot, writer-failure emergency close-t és clock-regressziót. A modul nem
    importál concrete `lgpio`-t vagy legacy drivert, és nincs signal handler,
    `main` entrypoint, concrete szenzorbackend, automatikus start, valódi
    hardverfuttatás vagy élő motion.
    A kilencedik szelet külön, side-effect-free config edge-en zárja az explicit
    regular, nem symlink hardware-, fizika- és speed-map JSON forrásokat egy
    `BoundedPhysicalRuntimeConfig` értékbe. Csak a motorpin/polaritás/decay
    contract, a közös nyomtáv és az `ACTIVE` `R2B4_WHEEL_SPEED_MAP_V2` kerül át;
    az L3 és L10 ugyanazt az egyszer beolvasott geometriát kapja. A bounded
    command profil, tick-periódus és preflight-kor explicit typed V3 input marad,
    ezért a loader nem teszi authorityvá a legacy config manager vagy control-
    orchestration mezőit. A canonical aktív config mappinget, a path/symlink,
    schema/state, geometria és motorcontract rejectiont célzott teszt fedi. A
    modul nem importál concrete hardvert, szenzorbackendet, legacy drivert,
    `config_manager`-t, CLI-t, signal handlert vagy `main` entrypointot; nem nyit
    GPIO-t, nem futtat owner loopot vagy élő motiont.
    A tizedik szelet a concrete inputlánc első szükséges native magjaként egy
    hardverfüggetlen `NativeCounterEncoderBackend` implementációt ad. Két
    injektált signed pulse-counterből az első `read(TickContext)` ugyanahhoz a
    tickhez köti mindkét counter snapshotját és a teljes idő-baseline-t; a
    konstruktor nem végez idő nélküli counter readet. Minden további tick
    counterenként pontosan egy immutable snapshotot olvas. A bal/jobb sebességet
    kizárólag a signed count-delta, az immutable oldalankénti step distance és a
    tick monotonic ideje adja; nincs PWM- vagy command-context. `trust=1` és
    esetleges nem nulla velocity kizárólag növekvő idejű, friss, futó counterű,
    diagnosztikailag tiszta és a fizikai sebességhatáron belüli delta esetén
    készülhet. A baseline, timing-invalid, stale, diagnosztikailag hibás vagy
    fizikailag lehetetlen minta mindig `trust=0` és mindkét oldalon zéró
    velocity; a nyers count/delta, időköz, counter-diagnosztikai változás, reject
    előtt számított sebesség és a pontos rejection code ettől elkülönített,
    passzív typed diagnosztika. Minden növekvő tick újrahorgonyozza a következő
    deltát. A contractot
    direction, baseline binding, reanchor, stale recovery, diagnosztikai,
    timing- és velocity rejection, valamint `NativeEncoderSource` integráció
    fedi. A backend nem importál legacy drivert/service-t vagy global configot,
    nem nyit GPIO-t, és nem birtokol callbacket, threadet, órát, runtime-ot vagy
    writert. A valódi GPIO counter-owner, native IMU/LiDAR backend és élő teszt
    továbbra is külön későbbi kapu.
    A tizenegyedik szelet a velocity backend közvetlen fizikai előfeltételeként
    egy páros natív GPIO signed pulse-counter ownert ad. Az immutable config a
    két oldal négy egyedi A/B pinjét, a B irányszintet, invertálást, pull-upot és
    A debounce-ot még GPIO-nyitás előtt zárja. Az owner pontosan egy injektált
    lgpio-style backend-handlet nyit, mind a négy input alertet birtokolja,
    az aszinkron callback gpiochip-sorszámát (nem az opaque open handlet) és
    pinjét ellenőrzi, oldalanként a B callbacket latch-eli, és csak az A rising
    callbacket számolja előjelesen ugyanazon lock alatt. Kifelé két capability-szűkített,
    lock-konzisztens immutable `SignedPulseCounterSnapshot` nézetet ad; sebesség-,
    tick-, PWM- vagy command-policyt nem vesz át. Részleges konstrukciós hiba
    minden már létrehozott callbacket, pint és a sole handlet felszabadítja; a
    `close` előbb leállítja a callback-mutációt, majd idempotensen takarít. Fake
    GPIO evidence bizonyítja az egyhandle-es ownershipet, direction/invert
    viselkedést, malformed callback diagnosztikát, snapshot-immutabilitást,
    partial-failure cleanupot és a `NativeCounterEncoderBackend` integrációt. A
    modul nem importál concrete `lgpio`-t vagy legacy drivert/configot, nem indít
    threadet, runtime-ot, writert, valódi hardvert vagy élő motiont; a concrete
    backend-kötés, native IMU/LiDAR backend és élő teszt későbbi külön kapu.
    A tizenkettedik szelet a counter-owner bekötése előtti side-effect-free
    config-határt zárja. Az explicit regular, nem symlink hardware- és fizika-
    JSON forrásból immutable `NativeEncoderRuntimeConfig` készül: a négy A/B
    pin, közös forward B-szint, oldalankénti invert, pull-up, A debounce és GPIO
    chip a `GpioCounterPairConfig`, az alap step-distance és bal/jobb szorzói
    pedig külön fizikai geometria. Kötelező az `X1_A_RISING` count mode, a
    hardware/physics counts-per-revolution egyezése, a véges pozitív step-
    geometria, valamint a motor- és encoder-owner összes pinjének kölcsönös
    egyedisége. A loader által visszaadott bounded runtime config ezt a typed
    encoder értéket is hordozza, de a freshness/sample-age és maximum velocity
    csak explicit caller-paraméterrel képezhet `CounterEncoderBackendConfig`
    értéket; a legacy `snapshot_hz` nem V3 authority. Canonical mapping- és
    rejection-tesztek fedik a teljes határt. Nincs concrete hardverimport,
    `config_manager`, legacy driver/service, GPIO-open, runtime-bekötés, writer
    vagy élő motion; a concrete backend és source ownership következő külön kapu.
    A tizenharmadik szelet a typed GPIO counter-configot, az explicit velocity-
    mintapolicyt és a native source health-configot egyetlen
    `NativeGpioEncoderSource` ownerben köti össze. A három immutable config még
    GPIO-open előtt típusellenőrzött; ezután pontosan egy
    `NativeGpioSignedCounterPair`, annak két capability-szűkített nézete és egy
    `NativeCounterEncoderBackend` kerül a meglévő `NativeEncoderSource` port
    mögé. A caller `TickContext` értékén kívül nincs óra vagy sampling policy,
    a baseline és minden delta továbbra is tick-bound. Részleges GPIO-
    inicializálási hiba minden megszerzett erőforrást felszabadít, a source
    `close` idempotens, utána read nem lehetséges. Fake GPIO evidence bizonyítja
    az egyhandle-es ownershipet, a teljes counter→velocity→typed snapshot utat,
    a config-before-open kaput és a cleanupot. A modul nem importál concrete
    GPIO-t vagy legacy kódot, nincs IMU/LiDAR binding, runtime-/motor-wiring,
    owner loop, `ACTIVE` engedélyezés, hardverfuttatás vagy élő motion.
    A tizennegyedik szelet az encoder után az IMU és LiDAR teljes native V3
    input-bekötését zárja. Az injektált BNO055 fused-sample portot egy explicit
    freshness-, tengely-, előjel- és yaw-offset config alakítja tick-bound,
    `CCW_POSITIVE_LEFT` radiános `ImuHeadingReading` értékké; minden tick pontosan
    egy forced atomic sample readet végez, falióra- vagy cache-authority nélkül.
    Az injektált latest matcher port tickenként pontosan egy eredményt és egy
    runtime-statust olvas, majd kötelezően ellenőrzi a
    `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1`,
    `R2B4_SCAN_MATCH_CONFIDENCE_V2`, `process_latest_only`, `LIDAR_FIRST`,
    `R2B4_BOOT_ROBOT_MAP`, `EKF_POSE_ODOMETRY_SSOT` és
    `CCW_POSITIVE_LEFT` azonosságokat, mielőtt pose sample készülhet. Az egyetlen
    `NativeSensorInputOwner` a már promotált GPIO encoder source-szal együtt
    birtokolja mindhárom typed source élettartamát; a bounded runtime wrapper
    normál, stop-, fault- vagy exception-kilépésnél is idempotensen lezárja a
    LiDAR-, IMU- és encoder-capabilityt. Fake evidence fedi a unit/sign/idő-
    konverziót, protected matcher-contract rejectiont, egy-read-per-tick utat,
    config-before-GPIO ownershipet és teljes cleanupot. Nincs concrete
    `smbus2`, LiDAR driver/service vagy legacy import a V3 modulokban, nincs
    automatikus device/process start, runtime entrypoint, hardverfuttatás,
    robotmozgás vagy új motorút.
    A tizenötödik, első concrete mérési szelet natív, injektált SMBus-portos
    BNO055 device-ownert ad, amely a fused burstöt közvetlenül a caller
    `TickContext` monoton idődoménjéhez köti; nincs több `perf_counter`/tick-clock
    keverés vagy cache-authority. Az aktív hardware JSON a busz-, cím-, fusion-
    és tengelyconfigot, valamint a LiDAR safety-zónát is immutable V3 configba
    zárja, míg encoder/IMU/LiDAR freshness, trust és confidence policy továbbra
    is explicit typed caller-input. A véges `run_finite_sensor_measurement`
    kizárólag a zero-only `LiveInputComposition` útján fut, minden ticken teljes
    L1–L12 trace-et és L3 estimate-et ad, signal/stop, exception és normál vég
    után pedig idempotensen zárja a LiDAR-, SMBus- és encoder GPIO-ownert.
    A protected matcher csak az előző lezárt L3 pose immutable, thread-safe
    nézetét olvashatja; boot előtt a canonical map-origin `(0, 0, 0)`, aktuális
    tickes vagy visszafelé mutató control-feedback nincs. Az aszinkron matcher
    ugyanazon source-read ablakában mért legfeljebb 5.004 ms jövőbeli capture-
    skewja explicit 10 ms boundon belül a tick zárási idejére clampelődik; ezen
    túl timing-invalid marad, a 250 ms stale korlát változatlan. LIDAR_FIRST
    módban az IMU heading- és gyro-rate confidence külön contract: rate-only OK
    kizárólag pozitív LiDAR confidence-kapu mellett engedett, így a kalibrált
    gyro használható akkor is, ha a mag/accel-függő abszolút heading még nem
    trusted; LiDAR hiba továbbra is kritikus L12 STOP/FAULT. A concrete raised-
    stand fizikai wrapper csak az egyetlen meglévő bounded owner loopot és L12
    writert hívhatja az explicit approval token után; automatikus start vagy
    második PWM-út nincs. Az élő zero-output encoder/IMU/LiDAR→L3 mérési kapu
    lezárult. Az első raised-stand ACTIVE próba 200. tickje engedélyezett bounded
    kimenetet adott, a 201. tick encoder low-trust hibája pedig L12 FAULT-ot.
    A GPIO recorder null duty-kat és handle-close-t látott, de a human fizikai
    megfigyelés szerint a kerekek csak a motor-táp kézi megszakításakor álltak
    meg. Ez bizonyította, hogy a duty=0 kérés önmagában nem fizikai stop-authority.
    Az eset óta powered ACTIVE futás tiltott; a hard-low javítás motor-táp nélküli
    próbája mind a négy pin explicit LOW írását, LOW readbackjét és close utáni
    `op dl` állapotát igazolta. Új ACTIVE futás csak a javítás promotionje, friss
    native V3 Test Hub profil, preflight és explicit human kapu után történhet.
13. **Minimális V3 toolchain-függetlenítés — tervezett:** csak a tényleges V3
    üzemeltetési igényhez szükséges CLI/status, `REPLAYER_V3` és mini Test Hub;
    legacy API-paritás nélkül.
14. **Native V3 FOLLOW — tervezett:** új V3 contractokból, a legacy Follow/Search
    orchestration portolása nélkül.
15. **Emberkövetés fizikai hangolása — tervezett:** kizárólag a native FOLLOW és
    az előző fizikai kapuk után.

Minden fázis csak a következő fázishoz közvetlenül szükséges kódot és tesztet
adja hozzá. Új evidence- vagy metadata-mechanizmust konkrét diagnosztikai hiba
nélkül nem vezetünk be.
