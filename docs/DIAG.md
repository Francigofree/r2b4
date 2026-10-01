# R2B4 DIAG

A canonical DIAG implementation helye: `tools/diag/`.

## Adatlánc

```text
ObservationHub -> capture -> MCAP -> r evi -> .evidence -> tools/diag -> analyzers
```

A DIAG közvetlen bemenete kizárólag a `tools/mcap_evidence` által publikált, sealed
`.evidence` bundle. A DIAG nem nyitja meg az MCAP-ot és nem épít saját második
capture-parsert.

Az EVI lossless evidence compiler. A DIAG ennek szemantikai fogyasztója.

## Kimeneti szerződés

A DIAG feladata diagnosztikai adatok szolgáltatása ember és LLM számára:

- forrásból származó tények,
- számlálók és eloszlások,
- numerikus statisztikák,
- állapot- és mezőváltozások,
- rétegek közötti mért kapcsolatok,
- adatminőségi és coverage adatok,
- `message_id` / `view` / `source_pointer` / `tick_id` evidence-reference-ek.

A DIAG **nem** ad javítási javaslatot, ajánlást, következő lépést vagy automatikus
root-cause verdictet. A JSON schema ezt `purpose=DIAGNOSTIC_DATA_ONLY`,
`source=EVI_EVIDENCE_ONLY`, `root_cause_inferred=false` mezőkkel jelzi.

## CLI

```bash
r diag
r diag --json
r diag full latest
r diag full path/to/run.evidence --json
r diag list
r diag admission safety latest
r diag safety latest
r diag navigation latest --json
```

A `r diag` argumentum nélkül teljes DIAG-ot futtat a
`runtime/captures/*.evidence` legújabb sealed bundle-jén.

A top-level `r diag` az offline evidence DIAG. A resident runtime részletes
állapotának rövid aliasa továbbra is `r d`; a runtime útvonalon `r rt diag` is
megmarad.

A launcher a `r diag` futást ugyanazzal a cross-process operator lockkal védi,
így aktív resident V3 runtime mellett nem indul el a Pi-t terhelő offline DIAG.
A `tools.diag` Python package önmagában robot-runtime dependency nélkül, offboard
környezetben is futtatható.

## Analyzer registry

A registry explicit; nincs filesystem discovery vagy implicit event bus. A full
DIAG stabil sorrendje:

1. `evidence_health`
2. `execution_chain`
3. `safety`
4. `recovery`
5. `navigation`
6. `localization`
7. `drive`
8. `world_model`
9. `lineage`
10. `lifecycle`

Az analyzer contract required/optional evidence view-kat és source topicokat
deklarál. Hiányzó required view/topic esetén az admission eredménye
`INSUFFICIENT_EVIDENCE`; nincs MCAP fallback és nincs hiányzó adatból következtetés.

## Evidence API

A `tools/mcap_evidence/reader.py` a consumer API. A DIAG ezen keresztül olvas:

- sealed manifest + coverage + integrity,
- `index.sqlite` message/field/view index,
- normalized EVI view-k,
- source-message lineage.

Az API nem nyitja meg a source MCAP-ot. A bundle `full` verification kapun megy
át a DIAG context megnyitásakor. Az EVI verifier ellenőrzi a payload hash-eket,
message identity-t, field censust, normalized projection lineage-et és coverage
accountingot.

## Analyzer jelentés

Egy analyzer eredménye:

```text
R2B4_DIAG_RESULT_V2
  purpose = DIAGNOSTIC_DATA_ONLY
  source = EVI_EVIDENCE_ONLY
  evidence
  admission
  metrics
  observations[]
  root_cause_inferred = false
```

Observation típusok:

- `FACT`
- `DERIVED_MEASUREMENT`
- `RELATIONSHIP`
- `DATA_QUALITY`

Nincs recommendation/suggestion output channel. A registry az ilyen saját DIAG
output kulcsokat futás közben is tiltja.

## Analyzer scope

### evidence_health

Compiler státusz, source-integrity, message/JSON/quarantine számok, topic/view
inventory, field census és EVI verification állapot.

### execution_chain

L5-L12 state/command/constraint/output mezők, layer message-countok és közös
message identity intersection. Nem nevez meg root cause-t.

### safety

L8-L12 requested/allowed velocity kapcsolatok, active constraint számok, L11
saturation, L12 decision/reason/latch/enabled/output eloszlások.

### recovery

Recovery/fault/stop/resume/degraded/lifecycle string-state előfordulások és a
kapcsolódó event/runtime/L5-L12 mezőprofilok. A matching szabály a kimenetben
explicit.

### navigation

L4-L8 goal/route/candidate/selection/progress/velocity mezők. L8 requested v/omega
eloszlás explicit küszöbökkel: stopped/pivot/forward-turning/forward-straight/
reverse kategóriák. Ezek mérési kategóriák, nem minősítések.

### localization

L1-L4 és a localization szempontból releváns materializált sensor view-k
(IMU/encoder/wheel/odometry/pose/heading/matcher/motion)
pose/heading/odometry/quality/freshness/revision mezőprofiljai. A teljes sensor-view
inventory külön megmarad a kimenetben; nagy, nem localization-jellegű sensor payloadot
a full DIAG nem jár végig indokolatlanul.

### drive

L8-L12 és encoder/wheel/motor sensor view-k. Wheel target, normalized command és
final output zero/nonzero/sign számlálók.

### world_model

L3-L6 world/occupancy/obstacle/clearance/corridor/person/goal mezőprofilok. A raw-LiDAR
materializálás darabszáma látszik, de a full DIAG nem járja végig újra az összes LiDAR
pontot; a részletes LiDAR evidence továbbra is az EVI bundle-ben marad lekérdezhetően.

### lineage

Index/coverage normalized-view accounting, layer/sensor view counts, source
snapshot identity. Expliciten jelzi, hogy a DIAG nem nyitotta újra az MCAP-ot.

### lifecycle

Runtime/event/L5/L6/L12 lifecycle/state/status/mode/reason mezők időrendi
value-change számlálói.

## Anti-drift

Az EVI minden JSON leaf-et indexel, és normalized view-kat lineage-dzsel publikál.
A DIAG profiling path-alapú és nem field-whitelist alapú: új mezők a releváns
view-kban automatikusan láthatóvá válnak, ha a domain token-scope-ba esnek. A
full field census az `evidence_health` / `lineage` rétegben ettől függetlenül is
megmarad.
