

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
