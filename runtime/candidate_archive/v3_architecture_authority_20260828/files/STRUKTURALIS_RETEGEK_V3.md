# R2B4 V3 robotarchitektúra — szigorú rétegcontract

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V3`

**Szerep:** normatív V3 architektúra-SSOT. Nem eseménynapló.

**Hatókör:** minden `v3/` production source, annak composition rootjai, portjai,
adapterei, konfigurációi és replay-evidence útvonalai.

**Runtime-aktiválás:** `NOT_AUTHORIZED`. Ez a dokumentum a V3 source
architektúráját teszi normatívvá; önmagában nem engedélyez V3 composition rootot,
motorírást, live motiont vagy legacy runtime-kiváltást.

## 1. Authority és átmeneti együttélés

1. A `v3/` scope-ban ez a dokumentum az egyetlen normatív réteg-authority.
2. A legacy production source-ra továbbra is a
   `STRUKTURALIS_RETEGEK_V2_1_STRICT.md` vonatkozik.
3. A V2.1 rétegsorrendje, shared state-je, contractjai, GUI/tool API-jai és
   regressziós elvárásai nem terjednek át automatikusan a V3-ra.
4. Legacy komponens V3-ban csak explicit donor-adapter mögött használható. A donor
   nem válik V3 authorityvá.
5. V3 és legacy runtime ugyanabban a processzben csak későbbi, külön jóváhagyott
   shadow adapteren keresztül futhat. Közös mutálható állapotuk nem lehet.
6. Egy adott chassis fölött egyszerre pontosan egy final-actuation authority lehet
   aktív. V3 aktiválás előtt a legacy writer kizárását gépileg bizonyítani kell.

## 2. Nem alkuképes invariánsok

- `V3_ONE_TICK_ONE_INPUT_SNAPSHOT_ONE_DECISION_ONE_COMMIT`
- `V3_L12_ONLY_NORMAL_MOTOR_WRITER`
- `V3_FAIL_CLOSED_STOP`
- `V3_NO_LEGACY_SHARED_STATE_OR_AUTHORITY`
- `V3_REPLAYABLE_DECISIONS_REQUIRED`
- `V3_MONOTONIC_TIME_ONLY_IN_DECISIONS`
- `V3_IMMUTABLE_VERSIONED_CONTRACTS`
- `V3_SINGLE_OWNER_PER_STATE`
- `V3_EVIDENCE_CANNOT_AFFECT_CONTROL_OUTPUT`
- `V3_SERVICE_PATH_MUTUALLY_EXCLUSIVE_WITH_NORMAL_CONTROL`

Hiányzó, elavult, sorrendtelen, ismeretlen sémájú vagy érvénytelen input nem
hallgatólagos defaultot, hanem explicit degradációt vagy STOP döntést eredményez.

## 3. Determinisztikus végrehajtási modell

A V3 composition root egyetlen `TickEngine`. A `TickEngine` tulajdonolja:

- a monoton tick-azonosítót és a tickhez tartozó logikai időt;
- az input snapshot lezárását;
- a rétegek rögzített sorrendű meghívását;
- az egyetlen output commitot;
- a konfigurációváltás tick-határát;
- a lifecycle állapotot: `BOOTING`, `IDLE`, `ACTIVE`, `DEGRADED`, `FAULT`,
  `SHUTDOWN`.

Egy tick alatt minden réteg legfeljebb egyszer értékelhető ki. Réteg nem indíthat
saját control threadet, nem alhat, nem olvashat faliórát, nem végezhet rejtett I/O-t,
és nem módosíthat más réteg állapotát. Külső I/O csak edge adapterben történhet.

A döntést befolyásoló idő kizárólag injektált monoton idő. Randomizált algoritmus
csak rögzített, capture-ben tárolt seedből dolgozhat. CPU-időhöz kötött keresési
deadline nem megengedett; a munka determinisztikus iteráció-, minta- vagy
csomópont-budgettel korlátozandó.

## 4. Közös contract-envelope

Minden réteghatáron áthaladó érték immutable. A közös envelope kötelező mezői:

| Mező | Contract |
|---|---|
| `schema_id` | stabil, globálisan egyedi contract-azonosító |
| `schema_version` | pozitív egész; inkompatibilis változáskor nő |
| `session_id` | egy boot/capture stabil azonosítója |
| `tick_id` | monoton, sessionön belül egyedi egész |
| `producer_id` | a kibocsátó port stabil azonosítója |
| `source_sequence` | produceregyedi monoton sorszám |
| `captured_monotonic_ns` | az első fizikai vagy logikai észlelés ideje |
| `published_monotonic_ns` | a tick logikai ideje; nem falióra |
| `config_set_id` | az immutable aktív konfiguráció hash-azonosítója |
| `causation_ids` | rendezett tuple a közvetlen input-event azonosítóiból |
| `validity` | `VALID`, `DEGRADED` vagy `INVALID` |
| `reason_codes` | rendezett, stabil enumértékek; szabad szöveg nem döntési input |

Az `event_id` determinisztikusan képzendő a
`(session_id, producer_id, source_sequence, schema_id, schema_version)` tuple-ből.
Lebegőpontos payload canonical normalizálása és hash-elése kötelező az evidence-ben.

## 5. Normatív rétegrend és ownership

### V3-L0 — Device HAL

Tulajdon: eszközhandle-ek, buszok, motor-driver fizikai write és fizikai read.

- Input: kizárólag L12 `FinalActuation` vagy L12 által engedélyezett service write.
- Output: `RawDeviceBatch`, valamint az írás eredménye `ActuationReceipt`.
- Tiltott: trust-döntés, fúzió, mission, planner, saját retry-policy a contracton
  kívül, közvetlen GUI/tool hívás.

### V3-L1 — Acquisition

Tulajdon: polling/stream adapterek, source sequence, capture timestamp, I/O health.

- Input: L0 `RawDeviceBatch`.
- Output: `AcquisitionFrame`.
- Tiltott: mérés elfogadása/elutasítása, pose-frissítés, control döntés.

### V3-L2 — Admission & Time Alignment

Tulajdon: freshness, sorrend, duplikáció, trust, time alignment és watermarks.

- Input: L1 `AcquisitionFrame`.
- Output: `AdmittedFrame` és explicit `RejectedObservation` rekordok.
- Tiltott: pose/world state birtoklása, planner vagy actuator state módosítása.

### V3-L3 — State Estimation

Tulajdon: robot kinematikai állapota és kovarianciája.

- Input: L2 `AdmittedFrame`.
- Output: pontosan egy `RobotEstimate` tickenként.
- Tiltott: map ownership, mission state, actuator write, globális pose singleton.

### V3-L4 — World Model

Tulajdon: térkép-revízió, lokális occupancy, akadálytrackek és szemantikus világkép.

- Input: L2 `AdmittedFrame`, L3 `RobotEstimate`.
- Output: pontosan egy immutable `WorldSnapshot` tickenként.
- Tiltott: motion kiválasztása, controller-integrátor, GUI-state.

### V3-L5 — Command & Mission

Tulajdon: validált command lifecycle, mission state és operátori authority lease.

- Input: `CommandGateway` által validált külső `CommandRequest`.
- Output: pontosan egy `MissionIntent` tickenként, szükség esetén explicit STOP.
- Tiltott: közvetlen planner/controller/motor hívás, fájl-journal pollolása a rétegen
  belül, GUI objektum referenciája.

### V3-L6 — Navigation

Tulajdon: route/corridor/trajectory terv és progress state.

- Input: L5 `MissionIntent`, L3 `RobotEstimate`, L4 `WorldSnapshot`.
- Output: `NavigationPlan`.
- Tiltott: actuator limit, PWM, safety PASS vagy közvetlen viselkedés-mellékhatás.

### V3-L7 — Motion Selection

Tulajdon: jelöltek prioritása, arbitration proof és egyetlen kiválasztott cél.

- Input: L6 `NavigationPlan` és explicit, typed motion-candidate portok.
- Output: pontosan egy `MotionObjective` tickenként.
- Tiltott: shared task/FSM objektum, több párhuzamos aktív intent, motorparancs.

### V3-L8 — Motion Realization

Tulajdon: geometriai guidance és a kiválasztott cél kinematikai megvalósítása.

- Input: L7 `MotionObjective`, L3 `RobotEstimate`, L4 `WorldSnapshot`.
- Output: `MotionIntent` (`v`, `omega`, horizon és mintázott referencia).
- Tiltott: végső safety döntés, PWM, sensor trust felülírás.

### V3-L9 — Operational Constraints

Tulajdon: nem végső dinamikai, komfort-, stabilitási és környezeti envelope.

- Input: L8 `MotionIntent`, L3 `RobotEstimate`.
- Output: `ConstrainedMotion` az eredeti és engedett referencia együtt.
- Tiltott: safety PASS tanúsítása, hardver-write, rejtett clamp indoklás nélkül.

### V3-L10 — Chassis Control

Tulajdon: chassis-kinematika és keréksebesség-referencia.

- Input: L9 `ConstrainedMotion`.
- Output: `WheelVelocitySetpoint`.
- Tiltott: PWM, motor-driver, mission/planner state.

### V3-L11 — Actuator Control

Tulajdon: wheel-loop integrátorok, feed-forward és kalibrációs map alkalmazása.

- Input: L10 `WheelVelocitySetpoint`, L2-ből az elfogadott wheel feedback.
- Output: `ActuatorRequest`.
- Tiltott: fizikai motor-write, safety bypass, globális konfiguráció olvasása.

### V3-L12 — Safety & Final Actuation

Tulajdon: safety latch, final enable/disable, hard limit és az egyetlen normál
motor-write capability.

- Input: L11 `ActuatorRequest`, L1 kritikus nyers device-factek, lifecycle state,
  valamint az exkluzív `ServiceGateway` request.
- Output: pontosan egy `FinalActuation` tickenként; L0 után `ActuationReceipt`.
- Tiltott: PASS feltételezés hiányzó evidence-ből, nem auditált bypass, második writer,
  post-gate clamp vagy post-gate parancsmódosítás.

## 6. Réteghatár-contractok

| Contract | Kötelező payload |
|---|---|
| `RawDeviceBatch` | rendezett `(device_id, sample_kind, device_sequence, host_monotonic_ns, payload)` tuple; device health |
| `AcquisitionFrame` | rendezett sample tuple; source watermarks; I/O health snapshot |
| `AdmittedFrame` | accepted observation tuple; rejection tuple; alignment epoch; trust summary |
| `RejectedObservation` | input event id; stabil reason enum; observed age/order/trust facts |
| `RobotEstimate` | frame id; `x_m`, `y_m`, `yaw_rad`, `v_mps`, `omega_rad_s`; covariance; estimator generation |
| `WorldSnapshot` | frame id; map revision; immutable occupancy reference/hash; obstacle tracks; freshness |
| `CommandRequest` | command id; issuer; authority token; requested mode/goal; issued/expiry tick |
| `MissionIntent` | mission id/revision; mode; goal; constraints; lifecycle; stop reason |
| `NavigationPlan` | plan id/revision; mission revision; route/corridor; progress; terminal condition |
| `MotionObjective` | selected candidate id; kind; priority; expiry tick; arbitration proof |
| `MotionIntent` | requested `v_mps`, `omega_rad_s`; horizon; ordered reference samples; stop reason |
| `ConstrainedMotion` | requested és allowed referencia; aktív constraint enumok; limiting facts |
| `WheelVelocitySetpoint` | bal/jobb `m/s`; kinematic model id; source motion event id |
| `ActuatorRequest` | bal/jobb normalizált actuator kérés; controller-state hash; saturation facts |
| `FinalActuation` | bal/jobb végső output; enable bit; safety decision; latch state; source request id |
| `ActuationReceipt` | requested/applied output; driver sequence; write status; hardware fault facts |

Minden contracthoz canonical serializer, schema validation és round-trip teszt
kötelező. `dict[str, Any]` réteghatárként nem elfogadott.

## 7. Engedélyezett adat-élek

Az alábbi whitelist teljes; minden más közvetlen layer-to-layer adatél tiltott:

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
L12 -> L0
CommandGateway -> L5
ServiceGateway -> L12
L0..L12 -> EvidencePlane
CompositionRoot -> minden layer port-construction/lifecycle
```

Az `EvidencePlane` write-only megfigyelő a control rétegek felől. Válasza,
queue-telítettsége, fájl-I/O-ja vagy hibája nem módosíthat control outputot.

## 8. Import- és függőségi szabályok

Egy layer implementation csak az alábbiakat importálhatja:

- Python standard library és jóváhagyott determinisztikus third-party library;
- `v3.contracts` és a saját layer package-e;
- constructorban injektált portok típusdefiníciói;
- tiszta, state nélküli `v3.math` modulok.

Layer implementation nem importálhat másik layer implementationt. Az
implementációkat kizárólag a composition root kapcsolhatja össze.

A teljes `v3/` production scope-ban tiltott közvetlen import:

```text
cont
control_loop
state
robot_state
controller.commands
controller.components
controller.routines
controller.status
fastgui
ai
tools
tests
config_manager
```

Tilos továbbá GUI widget, tool process, JSONL command journal, mutable `ctrl`
objektum vagy modulglobális singleton átadása V3 contractként.

Legacy donor import kizárólag `v3.adapters.legacy_donors` alatt engedélyezhető,
gépi allowlisttel. Az adapter V3 portot valósít meg, minden input/outputot typed V3
contracttá alakít, és nem adhat tovább legacy objektumreferenciát.

## 9. State- és konfiguráció-ownership

| Állapot | Egyetlen owner |
|---|---|
| source sequence és I/O health | L1 |
| admission history és watermark | L2 |
| pose/twist/covariance | L3 |
| map és obstacle history | L4 |
| command/mission lifecycle | L5 |
| route/progress | L6 |
| arbitration history | L7 |
| guidance progress | L8 |
| constraint state | L9 |
| chassis kinematic state | L10 |
| controller integrátor | L11 |
| safety latch és final decision | L12 |
| tick, lifecycle és wiring | CompositionRoot |

Konfigurációt a composition root validált, immutable `ConfigSet` formában injektál.
Réteg nem olvashat közvetlenül fájlt, environment variable-t vagy globális config
managert. Konfigurációcsere kizárólag tick-határon, `ConfigChanged` evidence mellett
történhet. Actuationt érintő configváltás csak `IDLE` és null output mellett
engedélyezett.

## 10. Command, GUI és tool boundary

GUI, CLI, LLM és külső tool csak versionált `CommandGateway` kliens lehet. A
gateway hitelesít, deduplikál, lejáratot és authority lease-t ellenőriz, majd
`CommandRequest` contractot publikál. Nem érhet el layert, mutable state-et vagy
motor API-t.

A production V3 runtime headless. GUI nélkül minden normál és diagnosztikai
funkciónak működnie kell. GUI nem lehet composition root és nem birtokolhat
konfigurációt, calibration state-et, test subprocess-t vagy safety döntést.

A `ServiceGateway` külön, explicit maintenance authority. Normál control mellett
nem aktív, fizikai preflightot és lease-t igényel, és minden requestje/receiptje
capture-be kerül. Service request sem kerülheti meg L12-t.

## 11. Replay és evidence contract

A V3 elsődleges fejlesztési bizonyítéka a determinisztikus replay. Egy teljes
capture kötelezően tartalmazza:

- a tickelt külső input contractokat és canonical byte-hashüket;
- az aktív `ConfigSet` tartalmát és hashét;
- a lifecycle- és authority-változásokat;
- L2–L12 minden döntési outputját;
- a final actuation requestet és receiptet;
- schema registry és build/source fingerprintet.

Replay nem használhat live eszközt, faliórát, GUI-t, legacy shared state-et vagy
nem rögzített konfigurációt. Azonos capture és build esetén minden döntési event
canonical hashének egyeznie kell. Eltérés első divergáló réteggel, tickkel,
input-hash-sel és reason code-dal diagnosztizálandó.

A V3 `agentctl diagnose` backend elkészültéig V3 scope-ban a legacy Replayer V2.1
automatikus használata tilos és fail-closed hibát kell adjon.

## 12. Donor-felhasználási contract

A jól működő alsó komponens KEEP/DONOR jelölt, de csak akkor köthető be, ha:

1. nincs rejtett `ctrl`/global state függése;
2. időt és konfigurációt injektált porton kap;
3. I/O-ja leválasztható és offline fake-kel futtatható;
4. input/outputja V3 contracttá alakítható referencia-szivárgás nélkül;
5. determinisztikus replay-paritása és célzott karakterizációs tesztje PASS;
6. nem nyit alternatív motor- vagy safety-bypass útvonalat.

Elsődleges donorjelöltek: wheel motion controller, wheel executor/kinematika,
PI/feed-forward mag, speed-map görbék, encoder/BNO/LiDAR device adapterek. Ha egy
jelölt a fenti kapun nem vihető át kis adapterrel, csak a bizonyított tiszta
algoritmikus mag emelhető ki; a legacy orchestration nem.

## 13. Kötelező gépi kapuk

V3 source candidate csak az alábbi minimumokkal zárható:

- V3 authority route és hash-valid bootstrap;
- import-graph guard: tiltott legacy/GUI/tool/shared-state import nincs;
- contract schema és canonical round-trip tesztek;
- determinisztikus kétszeres replay vagy — a replayer elkészültéig — azonos
  `TickEngine` inputra azonos event-hash teszt;
- minden nem STOP final actuation előtt L12 property/fault-injection tesztek;
- composition root teszt, amely bizonyítja az egy tick / egy commit invariánst;
- runtime activation előtt külön human promotion, preflight és IDLE/PWM-null kapu.

## 14. Biztonságos bevezetési sorrend

1. Contractok, import guard és fake-only deterministic `TickEngine`.
2. V3 capture/replay skeleton, fizikai I/O nélkül.
3. STOP-only L12–L0 final-actuation slice fake HAL-lal.
4. L10–L11 donorok offline karakterizációja.
5. Acquisition/admission adapterek és estimator shadow replayben.
6. World model, navigation és motion-selection vertikális szeletek.
7. Headless composition root; legacy runtime mellett csak read-only shadow.
8. Külön engedélyezett, korlátozott live cutover egyetlen writer bizonyítással.
9. Legacy GUI/tool/authority és shared-state útvonalak eltávolítása külön taskokban.

Sem e dokumentum, sem egy korai V3 candidate nem engedélyezi a 7–9. pont
automatikus végrehajtását.
