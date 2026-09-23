# R2B4 V3 robotarchitektúra — egyszerű rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló, roadmap, tuningjegyzet, fájlformátum-leírás vagy formális proof rendszer.

**Cél:** egyszerű, determinisztikus, jól tesztelhető és hosszú távon karbantartható robot-runtime, amelyben a diagnosztikai eszközök nem válnak a control rendszer részévé.

Konkrét algoritmus, tuningérték, queue-méret, fájlformátum, CLI/GUI és pillanatnyi hardver-evidence nem ennek a dokumentumnak a feladata.

## Authority — kérdéstípus szerint

* **Architekturális ownership, réteghatár, control/safety authority és engedélyezett adatél:** ez a dokumentum.
* **Pillanatnyi algoritmus, típusmező, konfiguráció és tuning:** canonical source + aktív config, e contract korlátain belül.
* **Egy konkrét futás tényei:** a végleges, integritás-ellenőrzött capture és az abból dolgozó canonical Replayer/Test Hub evidence.
* **Történeti dokumentáció, nyers log, fejlesztési jegyzet:** háttéranyag.

Source-first hibakeresésnél a tényleges viselkedést a source-ból kell megérteni, de véletlen source-drift nem írhatja felül ezt a contractot. Ha source és e dokumentum architekturális szabályban ellentmond, az eltérés hiba mindaddig, amíg a contractot tudatosan és az érintett source-szal együtt nem módosítják.

## 1. Minimum, nem alkuképes garanciák

* Minden stateful felelősségnek pontosan egy owner-e van.
* Nincs legacy shared state, rejtett singleton vagy kerülő authority.
* A production réteghatárok immutable, konkrét Python típusok; `dict[str, Any]` nem réteghatár-contract.
* Egyetlen szekvenciális TickEngine dolgozik egy már lezárt `TickInputs` snapshotból, rögzített sorrendben, rétegenként legfeljebb egyszer.
* Egy tickből pontosan egy L12 final döntés és legfeljebb egy normál motor-write születik.
* Kritikus hiány, hiba vagy bizonytalanság nem eredményezhet pozitív actuationt. Recoverable esetben typed SAFE HOLD/STOP, nem recoverable safety-, contract- vagy recovery-exhaustion esetben FAULT alkalmazható; minden tiltó final döntés fizikailag inaktív motorállapotot eredményez.
* Azonos lezárt input + konfiguráció + szükséges induló state + kód azonos typed layer-outputot ad.
* Replay eltérésnél a legelső eltérő tick, réteg és mező megnevezhető.
* Egy fizikai szenzor több független szemantikai capabilityt szolgáltathat; egy ág hibája nem érvénytelenítheti automatikusan a többit.
* Nincs alternatív normál motorút, safety-bypass vagy tool-specifikus control authority.
* Passzív observation/capture/telemetry consumer fogyasztói szerepében nem lehet production döntési input vagy control authority. Ugyanaz a GUI, agent vagy külső eszköz külön command kliensként kizárólag a canonical `CommandGateway` úton kérhet robotműveletet. Explicit lifecycle/health gate fail-safe STOP/SHUTDOWN-t kérhet, de pozitív actuation authorityt nem kaphat.

Ezeket a garanciákat adminisztratív egyszerűsítés, diagnosztikai kényelmi funkció vagy tesztátvezetés miatt sem szabad lazítani.

## 2. Determinisztikus végrehajtás és passzív observation sík

A composition root egyetlen `TickEngine`-t futtat. A motor-döntést befolyásoló **authority és owned layer-state** nem költözhet rétegenkénti threadbe/processzbe, és layer-kódban továbbra sincs sleep, falióra, rejtett I/O vagy modulglobális mutable state. Drága, determinisztikus **pure computation** külön worker-processzbe tehető kizárólag a runtime/adapter szélen, composition-root által injektált typed compute-port mögött. A worker csak lezárt immutable snapshotból számolhat; nem birtokolhat command-, mission-, navigation-, lifecycle-, safety-, motor- vagy GPIO-authorityt. Az aszinkron completion **nem közvetlen layer-input**: a runtime/composition root a tick production végrehajtása előtt a pillanatnyilag látható completion eredményt, annak hiányát vagy completion/transport állapotát immutable, typed `TickInputs` részévé zárja. L1–L12 kizárólag ezt a lezárt inputot láthatja; layer nem kérdezheti le közvetlenül a workert, completion queue-t vagy faliórát, és egy completion nem válhat ugyanazon tick közben utólag láthatóvá. Az owning layer a lezárt completiont saját typed contractja és owned state-je szerint elfogadhatja vagy elutasíthatja. Átmeneti `PENDING`/`STALE`/`DEGRADED`/`RESTARTING` vagy capability-specifikus ekvivalens állapot önmagában nem kötelező terminal FAULT: az owning layer fail-closed SAFE HOLD/STOP eredményt adhat és ugyanazt a mission/state identityt megtarthatja, miközben az edge bounded recoveryt végez. Stale, superseded, late vagy régi worker-generationből származó eredmény nem használható pozitív actuationhoz. Lehetetlen/malformed context- vagy contract-invariáns sérülés, nem recoverable safety-critical failure, recovery-exhaustion vagy váratlan upstream exception továbbra is fail-closed FAULT lehet. Stateful replay-slice esetén a checkpointnak a szükséges pending immutable kérést és production state-et kell rögzítenie; a completion adott tickbeli láthatóságát, capability/recovery állapotát vagy hibáját nem checkpoint-időzítési szabályból kell újraszámítani, hanem a lezárt replay-inputból kell visszaadni.

```text
async completion / capability state / completion error
              ↓
runtime input closure
              ↓
immutable typed TickInputs
              ↓
single TickEngine → L1 → … → L12
```

Egy normál tick:

1. `TickContext(tick_id, monotonic_ns)` létrejön.
2. A device-, command-, engedélyezett delayed-feedback- és runtime/adapter completion-input egy immutable typed `TickInputs` snapshotba lezárul.
3. L1–L11 legfeljebb egyszer, rögzített sorrendben fut.
4. Váratlan upstream exception vagy explicit hard-fault megszakítja a normál láncot; L12 egyszer, explicit fault okkal fut. Expected recoverable capability-unavailability nem exceptionnel jelzendő csak azért, hogy FAULT keletkezzen.
5. L12 dönt és birtokolja az egyetlen normál `MotorWriter` capabilityt.
6. A completed typed eredmény kifelé megfigyelhető. Production controlba csak e contractban explicit engedélyezett, typed, késleltetett feedback térhet vissza egy későbbi tick input-closure pontján; same-tick visszaél tilos.

A döntési idő az injektált monoton idő. Randomizált algoritmus csak replayelhető seedből dolgozhat; reprodukálható döntésnél bounded, determinisztikus work-budget kell.

A live, replay és szimuláció közös végrehajtási határa:

```text
input source → production V3 → output sink
```

Az input source lezárt `TickInputs` értéket ad; a sink passzív fogyasztó, motor-, lifecycle- és safety-authority nélkül. Aszinkron driver vagy feldolgozás megengedett, ha eredménye — beleértve a completion elérhetőségét, capability/recovery állapotát vagy hibáját — a production rétegek futása előtt lezárt, immutable, időbélyegzett typed inputként jelenik meg.

A fizikai live runtime-ban a szenzor I/O multi-rate edge ownerben futhat. A control tick L0 `DeviceReader.read()` útja nem végez blokkoló fizikai sensor I/O-t: kizárólag korábban publikált, bounded historyból választ olyan immutable snapshotot, amely a tick `monotonic_ns` idején már látható volt. A safety-kritikus és auxiliary acquisition külön worker lane-ben futhat; auxiliary késés vagy hiba nem blokkolhatja a kritikus acquisition lane-t és önmagában nem adhat egész-robot fault authorityt. A worker lane-ek CPU-affinityja operational runtime policy, nem production authority, és nem sértheti a control CPU izolációját. Replay/szimuláció továbbra is közvetlenül lezárt `RawDeviceBatch`/`TickInputs` értékből dolgozik, worker nélkül.

### 2.0 Control-process sterility

Production ACTIVE futásban a control processz szenzoroldali feladata bounded, már elkészült snapshotok átvétele és tick-zárása. Szenzor GPIO-callback, CPU-intenzív Python feldolgozás, raw kamera-frame payload, teljes raw LiDAR-geometria ismételt felépítése, illetve nagy payload serializálás/deserializálás nem konkurálhat az L1–L12 végrehajtással ugyanabban a Python interpreterben. Ezek runtime/adapter edge worker-process tulajdonai. A final L12 motor writer ettől függetlenül továbbra is az egyetlen engedélyezett normál actuation capability. A processhatár nem változtat measurement-időt, source sequence/revisiont, freshness/trust jelentést vagy layer authorityt; a control oldal kizárólag bounded, immutable, már elkészült szemantikai snapshotot zárhat `TickInputs`-ba.

### 2.1 Passzív observation/fan-out

A productionból kifelé vezethet passzív observation/fan-out capture, telemetry, GUI vagy metrics felé. Ez **nem L13 és nem control layer**.

Kötelező invariánsok:

* csak completed vagy önállóan immutable production/edge értéket figyel meg;
* bounded és production-publikáció szempontjából nem blokkoló;
* a generikus observation fan-out publication útja nem végez serializációt vagy fájl-I/O-t, és nem futtat tetszőleges downstream consumer-munkát szinkron módon; a publication legfeljebb bounded delivery/enqueue és saját minimális bookkeeping lehet;
* a fan-out nem értelmezi és nem módosítja a payloadot;
* consumer-ek izoláltak egymástól;
* latest-only consumer szándékosan supersede-elhet telemetryt;
* required evidence consumer adatvesztése explicit, tartós integrity failure.

Observation publication time csak transport/diagnosztikai idő; nem helyettesíti a fizikai measurement időt vagy a domain saját revision/sequence jelentését. Filtered fan-outnál globális publication sequence kihagyása önmagában nem bizonyít loss-t: az evidence-loss authority annak a konkrét consumer-delivery élnek az integrity állapota, amely az adat kézbesítéséért felel.

### 2.2 Recoverable capability-unavailability és recovery

A SAFE HOLD e contractban **nem új numbered layer, nem új `LifecycleState` és nem új L12 `SafetyDecision`**. Azt jelenti, hogy az owning production layer megtartja a releváns mission/state identityt, de pozitív mozgást nem kér, amíg a szükséges capability újra nem válik frisssé és érvényessé. Safety-critical evidence hiányánál L12 ettől függetlenül STOP-ot kényszeríthet ki.

Recovery szabályok:

* expected, átmeneti capability-unavailability (`PENDING`, `STALE`, `DEGRADED`, `RESTARTING` vagy typed ekvivalens) elsődlegesen adat/state, nem exception;
* stale, late, superseded vagy régi generationből érkező result elutasítható anélkül, hogy önmagában terminal FAULT keletkezne;
* worker/device supervisor csak technikai lifecycle-t, transportot, generation/request identityt és bounded restartot birtokolhat; mission/navigation/motion/safety authorityt nem;
* recovery alatt nincs pozitív actuation stale vagy hiányzó evidence alapján;
* recovery csak friss, újra validált typed evidence után folytathatja ugyanazt a missiont/state-et;
* recovery attempt/idő budget bounded; exhaustion explicit FAILED/FAULT policyhoz vezet;
* hard contract/context corruption, motor-writer failure, nem helyreállítható safety-critical failure vagy váratlan layer exception nem maszkolható recoveryként;
* a recovery döntéshez szükséges capability state, generation/request identity, accepted/rejected completion és recovery-exhaustion állapot replayelhető input/state legyen.

A konkrét retry-szám, timeout, backoff, generation formátum és capability-specifikus readiness/revalidation algoritmus source + config kérdés, amíg a fenti authority- és safety-invariánsok megmaradnak.

## 3. Egyszerű contractmodell és időszemantika

Minden production réteghatár explicit typed és a fogyasztó számára immutable érték. A jelenlegi Python implementáció használhat frozen/slotted dataclassot; nagy payload esetén read-only vagy egyértelmű ownershipű immutable buffer/view is használható, ha nincs kifelé szivárgó írható shared state. Közös metadata minimum:

```text
TickContext
  tick_id: int
  monotonic_ns: int
```

Nincs kötelező univerzális schema envelope, schema registry, provenance/causation gráf, config hash, event hash vagy boundarynkénti serializer. A Python típusdefiníció a belső production contract.

Validáció ott kötelező, ahol runtime-, safety- vagy determinisztikai értéke van: fizikai tartományok, idő/sorrend, azonos `TickContext`, freshness/trust, domain-invariánsok, STOP/FAULT null output és kritikus azonosítók.

A fizikai measurement idő, source completion idő, processing/result idő és observation publication idő külön fogalom. Freshness a releváns fizikai measurement idejéből számítandó.

Multi-rate live inputnál a sample-ben acquisitionkor rögzített `age_ns` nem lehet L12 freshness authority: ugyanaz az immutable snapshot egy későbbi control tickben természetesen idősebb. A közvetlen LiDAR safety ág aktuális korát L12 a final `TickContext.monotonic_ns` és a safety sample `captured_monotonic_ns` különbségéből számítja. A sample `age_ns` mezője source-read-time diagnosztikai lineage lehet és típus/tartomány szerint validálandó, de az aktuális L12-age-dzsel való pontos egyenlősége nem követelhető meg. A freshness limitet mindig az L12-ben újraszámolt aktuális korra kell alkalmazni.

A capture edge használhat verziózott külső serializációt, de az nem válik minden runtime contract részévé.

## 4. Rétegek és state-ownership

A numbered L0–L12 réteg **authority-, ownership- és typed contract-határ, nem fájl-, osztály-, függvény-, thread-, processz- vagy algoritmusszám**. Egy réteg lehet több belső modulból álló subsystem, és használhat ugyanahhoz a layerhez tartozó pure/helper/domain modulokat. Ezek egymást importálhatják, amennyiben kifelé egyetlen canonical layer contract, egyértelmű state owner és az engedélyezett production adatél marad. Nagy authority-free compute továbbra is a 2. szakasz szerint edge workerbe szervezhető; ettől a layer authority nem költözik ki.

| Réteg | Egyetlen felelősség és owned state | Typed output |
| --- | --- | --- |
| L0 Input Snapshot | edge-owner által már megszerzett measurementek determinisztikus tick-zárása, bounded published history és device/stream health | `RawDeviceBatch` |
| L1 Acquisition | source sample-ek és I/O health zárása | `AcquisitionFrame` |
| L2 Admission | freshness, sorrend, duplikáció, trust/alignment history | `AdmittedFrame` |
| L3 State Estimation | pose, twist, covariance | `RobotEstimate` |
| L4 World Model | spatial/temporal környezetállapot, lokális operational world/costmap, tracked entity-k és navigationhöz szükséges bounded structural/persistent knowledge; nincs külső I/O- vagy navigation authority | `WorldSnapshot` |
| L5 Command & Mission | command-validáció, aktív mission identity és mission-lifecycle; szükség esetén korábbi completed tickből closure-on át érkező typed execution feedback fogyasztása | `MissionIntent` |
| L6 Navigation | navigation plan, progress és szükséges lokális/global navigation state | `NavigationPlan` |
| L7 Motion Selection | pontosan egy érvényes objective és annak időbeli motion-authority ownershipje | `MotionObjective` |
| L8 Motion Realization | guidance → pillanatnyi kinematikai cél | `MotionIntent` |
| L9 Operational Constraints | motion/platform/gyorsulás/lokalizáció korlátozás | `ConstrainedMotion` |
| L10 Chassis Control | chassis-kinematika | `WheelVelocitySetpoint` |
| L11 Actuator Control | wheel-loop, feed-forward, calibration map | `ActuatorRequest` |
| L12 Safety & Final | safety latch, final döntés, egyetlen normál writer | `FinalActuation` |
| Composition/runtime root | tick, production lifecycle state, config snapshot, wiring, input closure és az explicit engedélyezett delayed-feedback zárása | `TickResult` / completed execution record |

Egy réteg nem módosíthat másik réteg state-jét és nem adhat át más komponens által írható shared mutable state-et. Read-only vagy egyértelmű ownershipű bounded buffer/view megengedett. Egy fizikai acquisition több szemantikailag különálló typed eredményt adhat, ha ownershipjük egyértelmű.

A fizikai device handle/busz/driver lifetime az edge/device owner felelőssége; ez nem külön control layer. L0 csak a már megszerzett, publikált input bounded történetét és determinisztikus tick-zárását birtokolhatja.

A host/operator controller a production L0–L12 és a composition/runtime root fölötti orchestration komponens; nem L13 és nem production robotréteg. Saját state-je kizárólag host/session state lehet, például process supervision, parancsproducer-életciklus, capture-session, tesztfázis vagy felhasználói művelet állapota. Production lifecycle-, navigation-, motion-, actuator- vagy safety-state-et nem birtokolhat.

## 5. Engedélyezett production adat-élek

```text
L0  -> L1
L0  -> L12        (kritikus device health)
L1  -> L2, L12    (közvetlen safety observation)
L2  -> L3, L4, L11
L3  -> L4, L6, L8, L9
L4  -> L6, L8
L5  -> L6
L6  -> L7
L7  -> L8
L8  -> L9
L9  -> L10
L10 -> L11, L12   (L12 felé safetyhez szükséges motion context)
L11 -> L12
L12 -> MotorWriter -> motor-edge/device I/O
CommandGateway -> L5
CompositionRoot -> minden layer konstrukciója, production lifecycle-ja és wiringja
CompletedExecution[tick N] -> runtime input closure -> L5[tick > N]
    (opcionális typed MissionExecutionFeedback; nincs same-tick L6 -> L5 visszaél)
```

Az L5 delayed execution-feedback él kizárólag korábban **completed** production eredményből, a composition/runtime root input-closure pontján át vezethet. L6 vagy bármely downstream layer nem hívhatja vissza közvetlenül L5-öt és nem módosíthatja annak state-jét. A feedback immutable, mission/command identityhoz kötött és replay-input; hiánya nem jogosít fel implicit lifecycle-átmenetre. A konkrét feedback-típus és az, hogy mely mission módok igénylik, canonical source kérdés.

A host/operator nem része a production döntési él-listának. Control irányban kizárólag a canonical command ingressen keresztül kapcsolódhat:

host/operator → canonical command producer/ingress → CommandGateway → L5

A runtime status/observation kifelé olvasható host/operator orchestration számára. Ez használható például readiness-, IDLE- vagy ALLOW-várásra, tesztfázis befejezésének felismerésére és fail-safe megszakításra, de nem vezethető vissza kerülő production adatélként valamely L0–L12 rétegbe. A fent explicit engedélyezett delayed mission feedback nem observation-visszaút, hanem canonical production input-closure él.

Különböző numbered layer implementationök nem importálhatják egymást; wiring közöttük csak composition rootban, typed callable/porttal történhet. **Azonos numbered layerhez tartozó belső helper/domain/subsystem modulok egymást importálhatják**, ha ezzel nem jön létre második state owner, kerülő authority vagy új cross-layer adatél.

A lista a production döntési élekre vonatkozik. Completed typed értékből vagy meglévő edge-owner immutable snapshotjából kifelé vezető passzív observation/capture él megengedett, ha nem tér vissza a döntési láncba és megfelel a 2.1 szakasznak. Kivétel kizárólag az e contractban explicit felsorolt typed delayed-feedback él, amely minden esetben egy későbbi `TickInputs` closure része.

## 6. Final safety és motorírás

L12 a normál motor-write capability egyetlen tulajdonosa. Nincs alternatív pozitív PWM-, service-, GUI-, tool- vagy külső writer.

Kötelező viselkedés:

* váratlan upstream exception, hard contract/invariant violation, kritikus device failure vagy hiányzó actuator request → fail-closed `FAULT`;
* expected recoverable capability-unavailability nem alakítandó exceptionnel terminal FAULT-tá pusztán azért, mert az aktuális tickben nincs használható friss eredmény; az owning layer typed SAFE HOLD/STOP szemantikát használhat a 2.2 szerint;
* ismeretlen/bizonytalan, a mozgáshoz ténylegesen kritikus safety input → `STOP` vagy indokolt `FAULT`;
* érvényes safety observation közvetlenül korlátozhat vagy tilthat actuationt;
* `ALLOW` csak érvényes L11 request + megfelelő lifecycle + szükséges safety feltételek mellett;
* döntésenként legfeljebb egy atomi writer-hívás;
* write exception után nincs automatikus második normál write, fault latch beáll;
* STOP/FAULT logikai final outputja nulla.

STOP/FAULT hardveroldalon fizikailag inaktív, igazolható állapotot jelent. `DeviceHealth` és a szenzor által megfigyelt veszély külön fogalom. Általános `ActuationReceipt` rendszer nem szükséges; valódi hardware feedback egyszerű typed L0/L1 adat.

## 7. Szenzoradat és capability-függetlenség

Külön kezelendő: (1) fizikai device/stream health, (2) measurement validity/freshness/trust, (3) egyes feldolgozási ágak minősége. Egy ág degradationje csak közös fizikai vagy contract-szintű ok esetén terjedhet másik ágra.

Live multi-rate acquisitionnél a source `TickContext` az acquisition saját monoton időpontja, nem egy későbbi control tick kölcsönvett ideje. A measurement timestamp és a snapshot publication/visibility idő külön fogalom; egy measurement csak olyan control tickben válhat láthatóvá, amelynek döntési ideje nem korábbi a publikációnál.

A multi-rate út event-szemantikájú: ugyanazon `(device_id, kind, sequence)` ismételt snapshotja ugyanaz a measurement, ezért L2 `DUPLICATE` és nem alkalmazható újra csak azért, hogy kitöltse a control tick frekvenciáját. A natív L3 EKF bootstrapkor egy friss `wheel_velocity` + `ekf_heading` párból indul; bootstrap után control tickenként forrásonként `0..1` friss admitted measurement érkezhet. L3 minden érvényes control tickben predikál, de encoder-, IMU- és LiDAR measurement correctiont kizárólag friss admitted observationre végez. Egyetlen tickben hiányzó új measurement önmagában nem exception/fault; a tartós hiány, stale állapot és fizikai device health továbbra is L1/L2/L12 freshness/safety contract szerint fail-closed kezelendő.

L4 a track observationt és a track estimate-et külön kezeli: az utolsó fizikai measurement ideje, a bounded prediction lejárata és az observed/predicted/degraded minőség a typed track része. Új observation nélküli tick nem új measurement és önmagában nem target-LOST. Érvényes predikciót L6 felhasználhat; lejárt/degraded becslés megőrizhet identityt és indokolhat HOLD/SEARCH állapotot, de nem adhat követési motion-authorityt. A target behavior (acquire/follow/hold/search/lost) L6 owned state, nem az observation-elérhetőség másolata.

### 7.1 Encoder

A fizikai count/delta és közvetlen elmozdulás RAW measurement; control-output vagy velocity filter nem írhatja át. Velocity estimation lehet stateful/időablakos, de bounded és determinisztikus. Pulse-window, debounce, CPR és tuning source/config, nem architektúra.

### 7.2 LiDAR

```text
LiDAR acquisition
   ↓
L1
   ├── collision/safety ─────────→ L12
   ├── local perception ─→ L2 ──→ L4
   └── localization ─────→ L2 ──→ L3
```

A safety ág nem függhet matcher-, localization- vagy map-sikertől, ha saját measurementje érvényes. World/costmap ownership L4-ben marad. Localization minőségromlás nem jelent automatikusan LiDAR device failure-t.

A scan fizikai időtartama és canonical measurement ideje legyen egyértelmű; completion/capture/publication time külön adat. Pose-reference ugyanarra a measurement időre vonatkozzon. A konkrét measurement-idő képlet, pose-history kapacitás és interpoláció source + config + célzott teszt felelőssége; ezek nem architektúraváltások, amíg az idő- és ownership-invariáns megmarad.

Ha scan-időre pose feedback kell, annak egy bounded, determinisztikus ownere lehet az edge/composition határon; nem válhat második L3 vagy control authorityvá. Raw scan csak konkrét funkcionális/diagnosztikai értékkel kerüljön capture-be.

## 8. Motion és command szabadság

Az L7→L12→MotorWriter lánc stabil canonical motion core. Új command/mission/behavior ne kerülje meg; ha a meglévő motion contract elég, CommandGateway/L5/L6 oldalon kapcsolódjon be.

Production motion live teszt a teljes CommandGateway→L5→L6→L7→L8→L9→L10→L11→L12→MotorWriter láncon fut. Szűk actuator/hardware teszt nem bizonyítja a teljes production motion utat.

L7–L12 stabil by default; csak konkrét funkció vagy igazolt source/replay/live evidence miatt változzon. Test Hub, teleop, script, follow/AI vagy más command source nem kaphat saját motor- vagy safety-utat.

`FORWARD`, `ARC`, `PIVOT`, `MOVE_1M` vagy hasonló primitive nem kaphat külön control-, motor- vagy safety-útvonalat. Recovery, docking, calibration vagy más behavior használhat ilyen mozgásszemantikát, ha azt a canonical CommandGateway→L5→…→L12→MotorWriter út realizálja. Külső command általános kinematikai vagy magasabb szintű mission/navigation célt adjon. Kötelező gate csak olyan capability lehet, amely az adott funkcióhoz ténylegesen szükséges.

Host/UI convenience adapter elfogadhat emberbarát primitive-et vagy bal/jobb keréksebesség-célt, ha azt még a CommandGateway előtt általános kinematikai commanddá alakítja. Ez nem L10 ownership: külső komponens nem injektálhat közvetlen `WheelVelocitySetpoint`, `ActuatorRequest`, PWM vagy más downstream control értéket. A tényleges chassis-, actuator- és safety-realizáció továbbra is kizárólag a canonical L9→L10→L11→L12 úton történik.

L6 birtokolja a navigation planninget és progress state-et, és egy érvényes `NavigationPlan` értéket állít elő. A terv konkrét reprezentációja és planner algoritmusa layeren belüli implementációs döntés. L7 a tervből pontosan egy `MotionObjective` értéket választ, és birtokolja az utolsó még érvényes objective időbeli folytonosságát. L6 a guidance eredetét, frame/scope-ját és monoton lejáratát adja át; az async planner-pending nem önálló motion-visszavonás. Azonos mission/frame/scope mellett L7 a még érvényes korábbi objective-et megtarthatja, de sem a pending, sem az új tick-context nem hosszabbíthatja meg az eredeti élettartamot. Lejárat, explicit invalidálás, STOP, mission/frame/scope-váltás vagy megszakadt control-folytonosság után korábbi objective nem éleszthető vissza. Ez minden guidance-reprezentációra vonatkozik, nem behavior-specifikus cache.

L7 csak folytonos, azonos mission/frame/scope és még érvényes előző objective mellett engedélyez normál átmenetet; a friss tervben explicit elutasított korábbi trajectory sem maradhat átmeneti motion-alap. L8 az objective-et pillanatnyi kinematikai céllá realizálja és a monoton érvényességet is ellenőrzi. A normál objective-váltás, nulla célra lassítás és irányváltás folytonosságát L9 meglévő sebesség/gyorsulás-korlátja biztosítja, a guidance fajtájától függetlenül. Az aktuális cél pillanatnyi nagyságát meghaladó vagy még az előző irányban fékező L9 kimenet csak az előző folytonos tick engedélyezett sebességéhez kötött typed transition-originnel megengedett: annak nagyságát nem növelheti, és az aktuális cél felé kell haladnia. Origin nélkül a pillanatnyi non-amplification szabály marad érvényben. Explicit HOLD/STOP, érvényességvesztés és safety-tiltás nem simítható át pozitív motionné; a szigorodó motion-envelope azonnal érvényes. Második motion-selection vagy safety authority tilos.

## 9. Konfiguráció, command, host/operator és külső I/O

A composition root validált, immutable configot injektál; layer nem olvas fájlt, environment variable-t vagy globális config managert. Actuationt érintő config csak biztonságos production lifecycle-határon, fizikailag inaktív motor mellett cserélhető.

GUI, CLI, LLM, tool vagy host/operator control irányban csak a canonical command ingress kliensén keresztül kérhet robotműveletet a `CommandGateway` felé. A host/operator command-request authorityval rendelkezhet, de production actuation-, motion-, safety- vagy motor-authorityval nem.

A host/operator controller birtokolhatja a felhasználói és host-oldali session orchestrationt: runtime process start/stop/shutdown, command-producer process lifecycle, readiness/IDLE/ALLOW várás, capture-session koordináció, Test Hub indítás, diagnosztika és többfázisú fizikai tesztszekvencia. Ezek host-side state-ek; nem válhatnak L0–L12 production state-té.

A production readiness, lifecycle és safety truth a runtime tulajdona. A host/operator olvashat runtime statust és használhatja azt orchestration döntésekhez, például ACTIVE parancs előtti readiness-váráshoz, valódi ALLOW visszaigazolásához, IDLE felismeréséhez, tesztfázis mérési végpontjához vagy fail-safe megszakításhoz. Nem szintetizálhat saját readiness/ALLOW truth-ot, nem írhatja felül a runtime STOP/FAULT döntését, és nem valósíthat meg kerülő navigation-, motion-, actuator- vagy safety controllert.

Egy logikai aktív command liveness mechanizmusának pontosan egy ownere lehet. A heartbeat ütemezés, command revision folytonosság, TTL-frissítés és atomi command-publikálás a canonical command producer/ingress stack felelőssége. A host/operator elindíthatja, megfigyelheti és leállíthatja ezt a command producert, de ugyanazt a heartbeat/revision/TTL/mailbox mechanizmust nem implementálhatja párhuzamosan.

A resident `CommandGateway` önállóan validálja a command trustot, sorrendet, freshness/TTL-t és process limiteket. Command expiry, hiány vagy ingress-hiba fail-closed STOP/FAULT marad; külső orchestration ezt nem kerülheti meg.

Capture, replay és Test Hub host/operator által koordinálható, de továbbra is observation/evidence capability. Eredményük nem válhat a futó production control pozitív actuation inputjává. Integritási vagy diagnosztikai hiba megszakíthat egy host-oldali tesztszekvenciát vagy indokolhat fail-safe STOP/SHUTDOWN kérést, de nem hozhat létre ALLOW-t és nem módosíthat production layer-outputot.

A runtime headless. Külső I/O megfelelő edge/device, canonical command ingress vagy passzív observation/capture adapterben történik. Adapter vagy edge komponens birtokolhat a saját I/O-, timing-, buffering-, trust- vagy protocol-felelősségéhez szükséges bounded technikai state-et. Nem birtokolhat azonban más réteghez tartozó szemantikai production state-et, és nem válhat kerülő command-, motion-, safety- vagy motor-authorityvá.

## 10. V3-only source és dependency szabály

A védett V3 production és canonical validációs source csak V3-at, standard libraryt és explicit jóváhagyott production dependencyket importálhat. Egy új dependency nem válhat control/state authorityvá, nem sértheti a bounded működést, és a replay/determinizmus módjának egyértelműnek kell maradnia.

A host/operator dependency iránya egyirányú: host/operator függhet a canonical command ingress, runtime status/lifecycle edge, capture és Test Hub felületektől, de production layer, TickEngine, composition/runtime root vagy CommandGateway nem függhet vissza launcher- vagy host/operator-implementációtól. Az, hogy egy host/operator modul technikailag a `v3` Python package-ben található, önmagában nem teszi production réteggé.

Különböző numbered layer implementationök nem importálják egymást. Azonos numbered layer belső subsystem/helper/domain moduljai importálhatják egymást, ha a canonical layer boundary, az egyetlen state owner és a cross-layer dependency irány változatlan marad. A generikus observation/fan-out komponens data-blind marad: nem függ layer implementationtől, engine-től, capture-format logikától, hardware-I/O-tól vagy consumer-specifikus serializációtól.

Külső GUI/vizualizáció/agent kliens használhat saját függőségeket a V3 csomagon kívül, de production V3 nem függhet vissza ezektől.

## 11. Capture, Replay, Test Hub és evidence

Capture, replay és Test Hub diagnosztikai capability, nem control authority. Hibájuk vagy lassúságuk nem módosíthat robotdöntést vagy motor-outputot. A persistent container formátuma implementációs döntés; a követelmény az ellenőrizhető integrity és a bounded visszakereshetőség.

A Test Hub közvetlen capture-tényt, artifact-integrity eredményt és canonical replay verdictet csak a saját pontos scope-jában tekinthet authority evidence-nek. A Test Hub nem általános truth authority. Triage, root-cause rangsor, anomáliaértelmezés és javítási javaslat derived diagnosis; ezek csak közvetlen kauzális evidence esetén minősíthetők PROVEN-nek, egyébként INDICATED, NOT_PROVEN vagy EVIDENCE_BLOCKED maradnak.

### 11.1 Capture integrity

Production observation bounded és nem blokkoló; encoding/tartós I/O nem lehet control-kritikus úton.

Required evidence loss esetén az artifact megőrizhető, de az érintett scope-on nem nevezhető teljesnek és nem lehet exact-replay MATCH-re jogosult. Egy delivery edge integrityjének egy authorityja legyen; downstream ne rekonstruáljon más, nem ekvivalens counterből második loss truth-ot.

Normál shutdown: production publikáció vége → required backlog feldolgozható/drainelhető → integrity lezárul → artifact finalizálódik. Partial és canonical final artifact legyen egyértelműen megkülönböztethető.

Capture csak indokolt replay/diagnosztikai evidence-et tartson; nincs „mindent logoljunk” követelmény. A persisted capture szemantikájának egy canonical encoding/értelmezése legyen; container/bridge ne hozzon létre versengő encoder/decoder authorityt.

### 11.2 Canonical replay

Replayhez kell a futtatandó lezárt `TickInputs`, tényleges config és minden olyan determinisztikus state/input, amely nélkül a scope nem reprodukálható. Ebbe explicit beletartozik az adott live tick input-closure pontján láthatóvá tett aszinkron completion eredmény, annak hiánya, capability/recovery állapota, valamint completion/transport hibája abban a typed formában, ahogy az a lezárt `TickInputs` részévé vált. A completion **láthatósága, capability/recovery állapota és hibaállapota replay-input**. Ugyanez igaz az explicit engedélyezett delayed `MissionExecutionFeedback` értékre, ha az adott tickben L5 inputja volt.

A Replayer ugyanazt a canonical production composition/TickEngine utat futtatja offline; saját layer-logika tilos. Replay nem futtathat vagy időzíthet újra aszinkron workert azért, hogy utólag kikövetkeztesse, melyik tickben lett volna látható az eredmény vagy mikor állt volna helyre a worker: az adott tick lezárt `TickInputs` értéke authority a completion láthatóságára, capability/recovery állapotára és hibájára. Az első eltérő tick/réteg/mező közvetlenül megnevezendő.

Production FAIL/FAULT futás is lehet teljes, MATCH replay evidence. Stateful slice csak megfelelő prefixből vagy bounded production-state checkpointból kaphat MATCH-et; checkpoint nem live authority és nem pótol hiányzó tick inputot.

Ha validation scope replayt kér, csak tényleges `MATCH` teljesítheti a replay gate-et. `MISMATCH`, replay error vagy el nem végzett kért replay nem PASS. Explicit replay-off esetén replay-egyezésre nem tehető állítás.

Capture/evidence hiány nem software divergence. Software divergence csak elegendő evidence/state mellett, tényleges canonical replay-eltérésből állítható. Bounded, rövid életű conversion/bridge artifact megengedett, de nem válik authorityvá; az eredeti final capture marad authority.

### 11.3 Test Hub és agentikus diagnosztika

A Test Hub offline evidence/diagnosztikai orchestrator a canonical capture és Replayer körül; **nem köteles minden kérdéshez replayt futtatni**. Replay nélkül végezhet integrity ellenőrzést, indexelt/bounded evidence-kiválasztást, leíró metrikát, incident triage-ot és agent/CLI/GUI derived nézeteket.

Nem futtathat saját production layer-, motion- vagy safety-logikát, és heurisztikát nem nevezhet canonical replaynek. Agent, GUI és CLI ugyanazt az authority- és verdict-szemantikát fogyassza.

Agentikus fejlesztés default útja: **kis összefoglaló → célzott evidence slice/query → csak szükség esetén nagy raw adat vagy széles replay**. Teljes capture automatikus betöltése nem alapértelmezett hibakeresés.

Brief, timeline, manifest, query-result és más derived artifact nem írhatja felül a canonical capture-t. Ha evidence-index authority capture-hez kötést állít, a verification az authority artifactot is ellenőrizze.

### 11.4 Verdict és root-cause szemantika

Külön fogalom:

1. production run outcome;
2. capture/evidence integrity;
3. replay verdict;
4. diagnosztikai bizonyosság.

Ezek nem olvaszthatók össze. Production FAULT mellett lehet teljes capture + MATCH replay; normál run capture-je lehet hiányos.

Root-cause szabályok:

* capture-integrity hiba elsősorban **evidence blocker**, nem automatikusan robotikai/software ok;
* megfigyelt esemény bizonyítható, de upstream oka csak kauzális evidence mellett nevezhető `PROVEN`-nek;
* heurisztika/korreláció nem emelhető bizonyított okká;
* a legkorábbi kauzálisan upstream eltérés fontosabb, mint egy későbbi súlyosabb következmény; severity önmagában nem root-cause sorrend;
* hiányzó evidence esetén az ok maradjon bizonyítatlan/evidence-blocked.

Temporal diagnosztikánál az állításhoz szükséges source revision és measurement/reference idő őrzendő; processing/publication latency nem helyettesítheti a fizikai measurement időt.

Nem kötelező általánosan: minden payload hash-e, schema registry, provenance graph, build fingerprint a control contractban, layerenkénti receipt/proof, minden belső üzenet round-trip tesztje vagy teljes fizikai szimulátor. Hash/checksum használható artifact-integrityhez, de nem lehet control input.

## 12. Kötelező, célzott tesztkapuk

Alapkapuk:

* import guard;
* immutable/domain/safety contractteszt;
* TickEngine sorrend/egyszeri evaluation/egyetlen L12 döntés;
* fail-closed upstream/lifecycle/input/writer-failure teszt;
* determinisztikus replay és első divergence;
* motor-edge változásnál fizikai STOP/FAULT inaktivitás bizonyítása.

Recoverable capability/recovery változásnál célzottan bizonyítandó: expected transient state nem használ stale/invalid eredményt pozitív actuationhoz; HOLD/STOP alatt a fizikai output inaktív; bounded recovery után csak friss, revalidált evidence enged resume-ot; old/superseded generation completion nem fogadható el; recovery-exhaustion determinisztikus terminal policyhoz vezet; live és replay ugyanazon recovery-state sorozatból azonos layer-outputot ad.

Mission-lifecycle delayed-feedback változásnál bizonyítandó: nincs same-tick downstream→L5 callback vagy közvetlen state-write; a feedback korábbi completed tickből származik, immutable és mission/command identityhoz kötött; a feedback ugyanazon későbbi tick `TickInputs` része live/replay módban; L5 marad a mission lifecycle egyetlen ownere.

Host/operator boundary változásnál célzottan bizonyítandó: minden pozitív motion request és STOP a canonical command ingressen halad; aktív command livenessnek egyetlen logikai ownere van; a host/operator runtime readiness/ALLOW/FAULT truth-ot fogyaszt és nem gyárt második authorityt; command-producer hiba vagy megszakítás fail-safe útra jut; production V3 nem függ vissza az operator implementációtól; capture/Test Hub orchestration nem tér vissza production control inputként.

Szenzor/algoritmus-változás saját közvetlen invariánsait célzottan bizonyítja.

Observation/capture boundary változásnál bizonyítandó: producer oldalon nincs consumer I/O/serializáció; required loss explicit; latest-only coalescing nem rontja required capture-t; payload szemantikája nem változik; close/drain/finalize alatt elfogadott required evidence nem vész el.

Capture/Replay/Test Hub közös boundary változásnál legyen valódi, stub nélküli end-to-end teszt typed production record → persisted capture → canonical replay → Test Hub verdict útvonalon, valamint negatív integrity-loss és replay-failure eset.

Nincs általános kötelező schema/hash/provenance/receipt/formális proof-kapu. Formátum-specifikus interoperability, storage- és performance teszt a konkrét implementáció felelőssége.

Célzott teszt az alapértelmezett; széles regresszió csak tényleges közös boundary/kockázat miatt indokolt. Változatlan, már sikeres validációt nem kell mechanikusan újrafuttatni.

## 13. Fejlesztési határ és mikor változik ez a contract

Ez a dokumentum nem workflow és nem implementációs leltár. Stabil architektúra csak bizonyított meglévő rendszerigény/hiba vagy explicit elfogadott új capability-követelmény miatt változzon. Feltételezett, konkrét igény nélküli jövőbeli lehetőség önmagában nem indok általános framework vagy új authority bevezetésére. Új capability a legkisebb teljes natív V3 szelet legyen. Safety/quality gate-et nem szabad csak azért lazítani, hogy teszt átmenjen.

**Ezt a dokumentumot módosítani kell**, ha változik: ownership; production control/data edge; motor/safety/lifecycle/command authority; layer felelősségi határ; fail-closed safety invariáns; determinisztikus execution/replay alapfeltétel; production–observation authority-határ; a capture/replay evidence alapvető completeness/MATCH/root-cause szemantikája; vagy V3 dependency boundary.

**Nem kell módosítani csak azért**, mert változik: layeren belüli algoritmus; ugyanazon numbered layer belső subsystem/modulstruktúrája; tuning/threshold/config; queue-kapacitás; capture konténerformátum/topic/chunkolás; azonos szemantikájú encoder-optimalizálás; CLI/GUI/agent brief mező; ideiglenes replay bridge; fájlnév/modulon belüli refaktor; vagy authorityt nem változtató diagnosztikai tool.

A pillanatnyi implementáció authorityja a canonical source + aktív config; a konkrét futásé a run-bound evidence. Mindkettőnek e dokumentum architekturális korlátain belül kell maradnia.
