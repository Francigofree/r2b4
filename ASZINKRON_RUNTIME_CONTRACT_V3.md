# R2B4 V3 — Aszinkron runtime contract

**Contract:** `R2B4_ASYNC_RUNTIME_CONTRACT_V3`

**Szerep:** a `STRUKTURALIS_RETEGEK_V3.md` rövid normatív kiegészítése a live runtime végrehajtási elhelyezéséhez. Nem új rétegrend, nem L0–L12 helyettesítő, nem IPC-framework és nem processz-topológia leírás.

**Authority:** layer/state ownership, safety, engedélyezett production adatél, időszemantika, replay és observation-alapszabályok továbbra is a `STRUKTURALIS_RETEGEK_V3.md` authorityja. Konkrét adapter, algoritmus, queue, CPU-affinity, processzszám és konfiguráció source-first kérdés.

## 1. V3 végrehajtási modell

```text
                  ASYNC EDGE WORLD

        device owners       compute workers
        encoder / IMU       localization / SLAM
        LiDAR / camera      vision / AI / planner
               \             /
                completed typed results
                         │
                         ▼
              COMPLETION / INPUT CLOSURE
                         │
                         ▼
                  immutable TickInputs
                         │
═════════════════════════╪════════════════════════
                         ▼
                   CONTROL ISLAND
                      L0 → L12
                         │
                         ▼
                       MOTOR

COMMAND INGRESS ───────► canonical CommandGateway ─► closure / L5

completed edge/control values ───────► OBSERVATION EGRESS
                                      capture / telemetry / GUI
```

Az **Async Edge World**, a **Command Ingress** és az **Observation Egress** nem új production layer. Nincs `A0–A5`, nincs command `L13`, és nincs observation-layer sorozat. Az egyetlen számozott production rétegrend az L0–L12.

### Async Edge World

A fizikai I/O és a változó vagy nagy futásidejű számítás alapértelmezett helye a control interpreteren kívüli edge.

Külön edge/processz vagy bizonyítottan GIL-független native végrehajtás szükséges, ha egy munka:

- blokkoló vagy változó késleltetésű fizikai I/O-t végez;
- nagy frekvenciájú Python callbacket futtat;
- CPU-intenzív Python számítás;
- nagy raw payloadot épít, másol, serializál vagy deserializál;
- egy control tick budgetjéhez képest nem szigorúan kicsi és determinisztikusan bounded.

Python thread használható kis, bounded I/O-ra, latest-value proxyra vagy minimális collector-munkára, de **a thread és a CPU-affinity önmagában nem GIL-izoláció**.

Az edge owner/worker saját device- vagy algoritmus-lokális állapotot tarthat, de nem kaphat L0–L12 production authorityt. A layer-owned state és a végső döntés a V3 rétegekben marad.

Ha a controlnak csak származtatott eredmény kell, a nagy raw adat maradjon a producer oldalán. Példa: kép → detection, teljes LiDAR scan → safety/localization/local-perception eredmény. Raw evidence közvetlenül mehet passzív observation/capture irányba anélkül, hogy a control interpreterbe belépne.

### Completion / Input Closure

Ez az **egyetlen engedélyezett async → deterministic control kapu**.

A closure pillanatában:

- csak már elkészült eredmény válhat láthatóvá;
- az eredmény, a hiány, a stale állapot és a transport/worker hiba explicit typed input;
- a measurement/source identity és idő nem írható át parent-side „most” időre;
- az adott tick `TickInputs` értéke lezárás után nem változhat;
- L0–L12 nem pollol workert, queue-t, device-ot vagy faliórát;
- késő completion legkorábban egy későbbi tickben válhat láthatóvá.

A transportnak control-szempontból boundednak és nem blokkolónak kell lennie. Állapotjellegű adatoknál a **latest/supersede** az alapértelmezett; sorhelyes/reliable szállítás csak ott szükséges, ahol a domain szemantikája vagy evidence-integritás ezt ténylegesen igényli. Végtelen backlog nem megengedett.

A processz-szétválasztás nem tekinthető kész izolációnak, ha a nagy payload pickle/unpickle, objektum-rekonstrukció vagy hasonló Python-munka továbbra is a control interpreterben történik.

### Control Island

A control island a lezárt inputból futó determinisztikus L0–L12 és az egyetlen final motor-write.

Itt csak olyan munka maradhat, amely:

1. production layer authorityhoz vagy layer-owned state-hez tartozik; és
2. szigorúan bounded, determinisztikus és a control budgethez méretezett.

A motor output **szándékosan közvetlen control capability**; nem kell csak az aszinkronitás kedvéért külön motor-processzbe tenni.

Ha egy jelenlegi layer számítása a robot fejlődésével túl nagyra nő, nem a layer authorityt kell kiköltöztetni. A drága, authority-free pure computation kerül edge workerbe, majd typed completionként tér vissza a closure-höz.

## 2. Oldalélek és fejlődési szabály

### Command Ingress

CLI, GUI, voice, AI, remote API vagy más külső komponens nem új production layer. Robotműveletet kizárólag a canonical `CommandGateway` útvonalon kérhet.

Az ingress I/O lehet aszinkron, de az L5 csak a tickhez lezárt immutable command inputot láthatja. Külső komponens, LLM vagy interface nem írhat közvetlenül mission-, navigation-, motion-, actuator- vagy safety-state-et.

### Observation Egress

Capture, telemetry, status, GUI és diagnosztika kifelé vezető passzív oldalél. A részletes integrity és fan-out szabályok a V3 observation contractban maradnak.

Kiegészítő runtime-szabály: **nagy edge-evidence ne térjen vissza a control interpreterbe csak azért, hogy onnan továbbmenjen capture/telemetry felé.** Ahol lehetséges, producer → observation consumer/sidecar közvetlen, bounded út legyen.

Ugyanez vonatkozik status/capture publicationre: a control ne serializáljon vagy adjon át teljes nagy production objektumgráfot, ha a fogyasztónak egy kicsi, explicit snapshot vagy közvetlen evidence-út elegendő.

### Új capability admission

Új szenzor, SLAM/VSLAM, semantic perception, AI, globális planner, map optimizer, GPU/NPU vagy más nagy compute bevezetésekor az alapértelmezett döntési sorrend:

```text
Van fizikai I/O vagy változó/nagy compute?
    igen ─► Async Edge World

A controlnak raw payload kell?
    nem ─► csak compact typed completion léphet át

Befolyásolhatja a robot döntését?
    igen ─► kizárólag closure → TickInputs → owning L-layer

Csak megfigyelés/evidence?
    igen ─► Observation Egress, control-visszaút nélkül
```

Új generic event bus, IPC-framework, scheduler vagy process-orchestrator nem építhető feltételezett jövőbeli igényre. Először capability-specifikus typed edge és konkrét mérési indok szükséges.

### V3 / V4 határ

További processzek, új szenzorok, SLAM, vision/AI, gyorsító vagy nagyobb compute **önmagában nem V4**.

A rendszer addig V3, amíg megmarad:

```text
async edge
    ↓
completed typed result
    ↓
input closure
    ↓
immutable TickInputs
    ↓
single deterministic L0–L12
    ↓
single final motor authority
```

V4 csak akkor indokolt, ha ennek valamely alapvető authority- vagy végrehajtási invariánsa tudatosan megváltozik.

## Rövid conformance gate

Production változtatás akkor illeszkedik ehhez a contracthoz, ha mind igaz:

- nincs új blokkoló device I/O vagy nagy/variable Python compute a control interpreterben;
- nincs indokolatlan raw/nagy payload a control útvonalon;
- minden async eredmény bounded transporton, closure előtt válik typed inputtá;
- worker/edge nem kap production layer vagy motor authorityt;
- command kizárólag canonical ingressen jut be;
- observation/evidence nem hat vissza és nem terheli nagy serializációval a control islandet;
- az L0–L12 sorrend, replayelhetőség és egyetlen final motor authority változatlan.


## 3. Async capability convergence

A külön edge adapterek közös **szemantikát**, nem közös runtime-frameworköt
követnek. Ez a fejezet nem vezet be event bust, schedulert vagy processz-
orchestrátort.

### Capability állapot

Az edge állapot az alábbi fogalmakra vezethető vissza: `NO_DATA`, `FRESH`,
`PENDING`, `STALE`, `DEGRADED`, `FAILED`, továbbá lifecycle-ként
`STARTING`, `RESTARTING`, `STOPPED`.

`STALE` önmagában nem automatikus terminal session fault. A stale érték nem
használható tovább döntésre; az owning layer és L12 a capability szerepe szerint
választ zero-motion holdot, STOP-ot vagy FAULT-ot. Explicit worker/device
failure, transport deadline, identity mismatch és safety-érvénytelenség továbbra
is fail-closed.

### Három transport-szemantika

1. `LATEST_STATE`: állapotjellegű szenzor/perception érték. Backlog helyett a
   legújabb teljes snapshot authoritative.
2. `REQUEST_RESULT`: drága authority-free compute. Request/result identity
   explicit, a result csak closure-on át válhat láthatóvá.
3. `EVIDENCE_STREAM`: raw/passzív evidence. Nem tér vissza a control
   interpreteren keresztül csak azért, hogy capture/telemetry fogyassza.

### Worker identity és restart

Request/result worker logikai azonossága:
`worker_generation + request_id + source_context`.

Régi generationből vagy superseded requestből későn érkező completion nem
fogadható el. Worker restart nem írhatja át a source identityt.

### Continuity és supersede

Állapotjellegű compute esetén a bounded célminta:
`1 running + legfeljebb 1 latest pending replacement`.
Végtelen queue és sorban kiszámolt elavult state nem megengedett.

### Lightweight production evidence

A capability edge kis scalar evidence-et tarthat: generation, request/revision,
pending age, accepted/superseded/stale/error/late-rejected számláló. Ez passzív
diagnosztika; nem timing authority.
