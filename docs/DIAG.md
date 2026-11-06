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

A DIAG nem ad javítási javaslatot, ajánlást vagy production parancsot.
A `navigation_root_cause` automatikus, evidence-határolt okértékelést ad:
explicit rögzített döntési mechanizmus, hipotézis és bizonyítékhiány külön marad.
A JSON `purpose=DIAGNOSTIC_DATA_ONLY`, `source=EVI_EVIDENCE_ONLY` mezői mellett
a `root_cause_inferred` azt jelzi, hogy készült-e ilyen automatikus értékelés;
önmagában nem bizonyossági, capture-integrity vagy replay verdict.
A többi analyzer továbbra is `root_cause_inferred=false` mérést szolgáltat.

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
r diag navigation_root_cause latest --json
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
6. `navigation_root_cause`
7. `localization`
8. `drive`
9. `motion_performance`
10. `world_model`
11. `lineage`
12. `lifecycle`

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
R2B4_DIAG_RESULT_V3
  purpose = DIAGNOSTIC_DATA_ONLY
  source = EVI_EVIDENCE_ONLY
  evidence
  admission
  metrics
  observations[]
  root_cause_inferred = false | true
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

A navigációs Flight Recorder P0 a mintavételtől független döntési eseményeket
is összegzi a meglévő `events` view-ból. A felvételi scope, kvóták,
lineage és a hosszú futások mérési korlátai: [FLIGHT_RECORDER_P0.md](FLIGHT_RECORDER_P0.md).

### navigation_root_cause

Automatikusan összekapcsolja az aktív NAVIGATE/EXPLORE/FOLLOW_PERSON mission legalább
1 másodperces megállási vagy mintavett no-progress szakaszát az L3–L12
ugyanazon source tickjének döntéseivel. A `metrics.episodes` részletes evidence-et
és okértékelést ad, a közös `episodes` API tömör epizódot közöl; a full report és
a CLI epizódszáma ezért szintén tartalmazza ezeket.

Az explicit L6 → L7 → L8 stop-reason lánc és nulla L9/L12 értékek alapján
megkülönbözteti az elavult világ-/akadályadatot, a lokalizációs HOLD-ot,
a planner várakozását, illetve az elérhető cél/út hiányát. Az L7 explicit
`NO_COLLISION_FREE_TRAJECTORY` vagy `NO_PROGRESS_VIABLE_TRAJECTORY` döntése
az összes pálya elutasítását azonosítja. A külön L9 constraint és L12 safety
gát megmarad. A legkorábbi bizonyított controlling boundary számít, nem a
későbbi stop severity. Pozitív downstream kimenet vagy nem egyező reason
nem bizonyítja az upstream stop propagációját.

A `PROVEN_RECORDED_DECISION` kizárólag ezt a rögzített döntési mechanizmust
bizonyítja. Nem bizonyít fizikai akadályt, rossz térképet, hibás lokalizációt
vagy software-divergence-t. A kiválasztott cél és no-progress közötti kapcsolat
`BAD_GOAL_SELECTION` hipotézis marad; a mélyebb producerhiba oka
`EVIDENCE_BLOCKED`. Az eredmény minden epizódhoz külön megőrzi a bizonyosságot,
az evidence gapeket és a lehetséges hipotéziseket. Az ismeretlen ok nem válik
alapértelmezett célpontválasztási hibává.

A minták közötti `v3.navigation_decision` esemény korábbi hibás tickje akkor
kapcsolható az epizódhoz, ha mission identityje és explicit döntési oka egyezik.
A korábban kiválasztott cél, event sequence, map/costmap revision és source
sequence, valamint a mérési/reference idő külön megmarad. A costmap
`measurement_reference_ns` az eredeti source tick ideje mínusz a domain saját
`freshness_ns` értéke; publication/log time nem freshness authority.

Az időtartam az előző mintától a következő mintáig tart, az utolsó minta
után nincs extrapoláció. 250 ms feletti gap, hibás forrásidő vagy másik mission
megszakítja az epizódot. A no-progress méréshez jó vagy degraded, explicit
folytonos lokális pose, változatlan frame/generation, álló sebességbecslés és
elmozdulás/yaw is kell. A rögzített motor-HOLD-hoz nem kell jó pose.
Ez 5 Hz-es sampled evidence; fizikai beragadást és a minták közötti mozgás
hiányát nem bizonyítja. Az első 64 epizód marad meg, a pontos összesített
darabszám és a leghosszabb epizód csonkolás esetén is elérhető.

EVI-verification, MCAP strukturális integritás, a capture saját delivery
integritása és a robotikai ok külön fogalom. A capture-loss/hiányzó completeness
`evidence_blockers` adat, nem robotikai ok. Az analyzer nem nyitja meg az MCAP-ot,
nem futtat replayt, és nem állít exact-replay MATCH-et.

### localization

L1-L4 és a localization szempontból releváns materializált sensor view-k
(IMU/encoder/wheel/odometry/pose/heading/matcher/motion)
pose/heading/odometry/quality/freshness/revision mezőprofiljai. A teljes sensor-view
inventory külön megmarad a kimenetben; nagy, nem localization-jellegű sensor payloadot
a full DIAG nem jár végig indokolatlanul.

### drive

L8-L12 és encoder/wheel/motor sensor view-k. Wheel target, normalized command és
final output zero/nonzero/sign számlálók.

### motion_performance

Időben összekapcsolt AMR mozgásteljesítmény- és mozgásminőség-mutatók az
L3/L8/L9/L11/L12 mintavett kimeneteiből. Az előző L9 allowed command és a
következő L3 local estimate összevetéséből idővel súlyozott sebességhiba,
RMSE/bias, valamint az első küszöbtúllépés konkrét tickje, ideje, értéke és
source-reference-e készül. Az L3 becslés, nem független fizikai ground truth.

A `temporal_coverage` megőrzi a source `TickContext` időt, a mintavételi frekvencia
becslését és a kizárt intervallumokat. A persisted `{$ref: input_context}`
layer-context kizárólag azonos source-message-hez tartozó `ticks` nézet
`/inputs/context` értékéből oldható fel. Az eredeti top-level tick/idő,
input-context és layer-context egyezése továbbra is kötelező; hiányzó vagy hibás
reference nem pótolható publication/log time-mal. A DIAG ehhez nem nyitja meg
az MCAP-ot, és nem expandál raw payloadot.

Explicit `DiagContext.measurement_time_interval_ns=(start,end)` megadásakor
csak az inclusive source-időablakon belüli mintákból képez intervallumot. Az
ablak előtti parancs és az első belső estimate nem alkot mért response-párt.
A kizárt minták száma és az alkalmazott időablak a coverage-ben szerepel;
scope nélkül továbbra is a teljes bundle az elemzés bemenete.

A `performance` requested/allowed mozgás-, constraint-, safety-block- és
saturation-időt, commanded distance-et, valamint lokális becsült chord-path és
szegmensenkénti net displacement értékeket ad. A `motion_quality` allowed
gyorsulás-RMS, irányváltás és stop-start számlálókat tartalmaz. A `goal_progress`
azonos célú L5 mission endpoint hibáját és az elérhető L6 progress értéket őrzi,
legfeljebb hat, source-reference-ekkel ellátott szegmensben.

A `motion_continuity` azonos source-idő szerint elkülöníti az inaktív missiont,
a nulla L8 kérést, a nulla L9 allowed mozgást, az L12 tiltást és a végső
motor-outputot. Az időtartam az előző mintában látható állapothoz tartozik,
a következő minta idejéig; az utolsó mintához nem feltételez további időt.
Az első 64 összefüggő epizód kezdeti L5/L6/L8/L9/L12 evidence-e és source
reference-e megmarad. A leghosszabb megfigyelt megállás és a leghosszabb
aktív mission alatti megállás a csonkolástól függetlenül elérhető.
Az aktív szakaszt az L5 canonical `lifecycle=ACTIVE` jelöli; a CANCELLED,
COMPLETED és FAILED is inaktív. A tickenként új ID-t kapó expired STOP nem
darabolja fel az azonos inaktív lifecycle/mode szakaszt. Aktív missionnél
az identity változása továbbra is megszakítja az epizódot.
Hiányzó mező nem jelent nulla kimenetet; nem nulla végső kimenet inaktív
mission mellett is látható marad. A megállási intervallumokhoz nem kell
érvényes lokális pose, de ugyanúgy megszakadnak recording-gap vagy hibás
source-context esetén. Ezek kimeneti kapcsolatok, nem fizikai beragadás
vagy automatikus okmegállapítás.

250 ms-nál nagyobb gap, hiányzó vagy nem monoton source idő esetén nincs
intervallum-integráció. Lokális távolság és response-pár csak explicit
pose-continuity, megfelelő localization quality, azonos frame és generation
mellett készül. A mintavett evidence nem bizonyít köztes fizikai mozgást,
stop-distance/settling értéket vagy skill-failure okot.

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

## Tartós DIAG artifact

A diagnosztikai futások alapértelmezésben a terminálkimenet mellett a teljes
machine-readable payloadot is elmentik:

```text
runtime/diag/diag_<UTC timestamp>_<mode>_<evidence>_<unique>.json
```

Az írás ugyanabban a könyvtárban létrehozott ideiglenes fájlon keresztül történik,
`flush` + `fsync` után atomi rename publikálja a végleges JSON-t, majd a könyvtár is
`fsync`-et kap. Sikeres `r diag` visszatérésnél ezért a kiírt `diag artifact:` útvonal
már tartósan publikált reportot jelöl.

A mentés minden tényleges diagnosztikai futásra érvényes (`full`, egyedi analyzer,
`admission`). A `list` és help nem hoz létre artifactot.

Explicit ephemeral futás:

```bash
r diag --no-save
r diag safety latest --no-save
```

A `--json` továbbra is tiszta JSON-t ír stdout-ra; a mentett artifact útvonala
stderr-re kerül, ezért a következő változat gépileg továbbra is közvetlenül
feldolgozható:

```bash
r diag --json | jq .
```

A `runtime/diag/` generált lokális runtime-adat és gitből ignorált.

## DIAG P0-P1 quality contract

A `R2B4_DIAG_FULL_REPORT_V2` report producer-provenance-t, report-szintű tömör
summary-t, analyzer coverage-state-et, explicit observation `evidence_basis` lineage-et
és időben határolt `episodes` adatokat tartalmaz. A nested analyzer result schema
`R2B4_DIAG_RESULT_V3`; a full report nem ismétli analyzerenként az azonos evidence és
producer metaadatot.

A profiler `change_count` szemantikája `ROW_TO_ROW_VALUE_COLLECTION`: egy JSON tömb
különböző elemei egyazon evidence row-ban nem számítanak időbeli transitionnek. A
recovery state számlálás state-bearing mezőkre korlátozott, configuration/metadata
mezőket és type/reference metaadatot nem értelmez recovery eseményként.

Részletes design: `docs/DIAG_P0_P1_DESIGN.md`.

