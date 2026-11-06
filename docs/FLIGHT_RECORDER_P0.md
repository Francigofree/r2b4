# Navigációs Flight Recorder P0

A meglévő Observer → Capture → MCAP → explicit EVI → DIAG út bővítése.
Az architekturális authority továbbra is a két V3 contract; ez a dokumentum
az implementáció és használat leírása. Nincs új control-, naplózó- vagy
diagnosztikai rendszer, nincs production döntési input az observation felől.

A jelenlegi alapértelmezés folyamatos `full`, 5 Hz; a kapcsolódó automatikus
hibakeresés és offline hangolás használata: [Experience Development](EXPERIENCE_DEVELOPMENT.md).

## Felvételi terv és megvalósítás

1. Az állapotfelvétel a meglévő időalapú mintavétel marad, hosszú futásra
   `full`, 5 Hz beállítással. A vezérlési ütemező és L0–L12 döntési lánc nem
   változik. A tap minden completed tick után figyel, de csak változáskor
   küld kis immutable eseményt ugyanabba a bounded capture transportba.
2. A `v3.capture_navigation.NavigationDecisionRecorder` kizárólag passzív
   összehasonlítási state-et tart: mission, cél, terv source-context/status/ok,
   L7 választás, L8 stop/L9 constraint, L12 safety/latch, lokalizációs
   quality/generation és L4 cellalátogatás. Inaktív STOP tickenként változó
   adminisztratív mission-ID-je nem generál önálló eseményáramot.
3. A `v3.navigation_decision` observation az MCAP meglévő `/r2b4/event`
   csatornájába kerül. A payload schema `R2B4_NAVIGATION_DECISION_V1`:
   source `tick_id`, `monotonic_ns`, session-local `event_sequence`, `changes`,
   kompakt `inputs` és `expected.layers.L3`–`L12`. Az MCAP log ideje a source
   tick ideje; a publication idő külön marad. A fizikai measurement-idők,
   source sequence-ek, map/transform revisionök az eredeti mezőkben maradnak.
4. Az esemény L6 route-preview-ja legfeljebb 16 waypoint, a trajectory
   alternatives legfeljebb 32 scalar összegzés: ID, command, score, clearance,
   collision és progress-viability. Darabszám és truncation flag jelzi a
   lefedettséget. Rollout sample-geometria nem kerül át a control→capture
   processhatáron. A kiválasztás és elutasítás csak a source-ban létező
   mezőkből bizonyítható; külön célalternatíva-lista jelenleg nincs az L6
   completed contractban.
5. Az L4 minták és események costmap-cellák és teljes bejártcellalista helyett
   darabszámot, map/costmap revisiont, lineage-et és az utolsó
   `(grid_x, grid_y, visits)` cellalátogatást tartják. A cellaváltás esemény
   mintavételtől független. A cellafelbontás és a retained coverage kapacitása
   a felvétel konfigurációjában szerepel. Ez mozgás–cella–döntés korreláció,
   nem teljes térképrekonstrukció vagy élettartam alatti egyedi területmérés.

## Pufferek, kvóták és integrity

A production út a meglévő capture sidecar bounded, nem blokkoló queue-ját
használja. JSON-encoding, chunkolás és fájl-I/O a sidecar feladata. Az
append-only mód nem tart teljes sessiont RAM-ban; az MCAP writer chunkokban
ír, indexeit a session kvóták korlátozzák. A mintavételi tick-kihagyások
egyetlen skalárszámlálót növelnek; valódi tick-anomáliából legfeljebb 64 példa
marad, a teljes anomáliaszám külön megőrződik.

Alapkvóta: 1 000 000 ingress observation és 4 GiB végleges/partial MCAP-fájl.
Az advanced runtime CLI `--capture-max-session-records` és
`--capture-max-session-bytes` paramétereivel állítható. Ez nem garantált
felvételi idő: az eseménysűrűség és payloadméret számít. A sessionbyte-kvóta
konzervatív accountingja 64 KiB lezárási tartalékot hagy, a fájl-write guard
a tényleges fájlméretet is korlátozza.

Kvótakimerülésnél a már rögzített adatok megmaradnak, az integrity hiányos:
`SESSION_RECORD_LIMIT` vagy `SESSION_BYTE_LIMIT`, eldobott observation-szám,
első/utolsó eldobás publication ideje. Transport-loss marker kvóta után is
feldolgozódik. Required queue overflow tartós integrity failure. Writerhiba,
lemezhiba vagy a lezárási metadata számára is elégtelen kvóta explicit hibát
és megőrzött `.partial` fájlt eredményez. Nincs automatikus retry vagy
régi capture-t törlő rotáció.

A meglévő `alap`/triggered mód rövid, bounded előzménygyűrűben tartja a
kompakt mintákat és a részletes döntési eseményeket, majd a trigger körüli
ablakot menti. Hosszú, teljes navigációs sessionhöz `full` kell. A P0 nem
kapcsol automatikusan 50 Hz-es raw sensor-debug felvételre.

## Offline kiértékelés

Az operátori út meglévő capture-választójával teljes, 5 Hz-es session kérhető:

```bash
./r start c full c 5
```

Ez önmagában nem navigációs mozgásparancs. A mozgás és a leállítás továbbra
is a canonical operátori/command út feladata; a P0 nem indít tesztmozgást.

A capture lezárása után explicit:

```bash
./r evi path/to/run.mcap --output /tmp/run.evidence
./r diag navigation /tmp/run.evidence --json
./r diag motion_performance /tmp/run.evidence --json
```

A meglévő EVI `events` view változtatás nélkül, source-message lineage-dzsel
őrzi az eseményeket. A navigation analyzer döntés-/ok-/replan- és
cellalátogatás-számlálókat, source-időre normalizált gyakoriságokat,
alternatíva/csonkolási mutatókat és korlátozott számú konkrét evidence-refet
ad. A teljes source az EVI-ből továbbra is visszakereshető. Sequence-rés
önmagában megfigyelés; a delivery integrity a capture loss-authority.

A motion_performance meglévő command/estimate, lokális út, tracking,
gyorsulás-, irányváltás-, goal-progress- és megállási mutatói mellett
aktív, engedélyezett, nem nulla mozgáskérés alatti stacionárius L3-becslés
epizódokat is mér. Ez megfigyelt no-progress, nem fizikai beragadás vagy
automatikus root-cause verdict. A folytonossági, frame-, generation-,
quality- és recording-gap korlátok kötelezőek. A mintavételi periódus becslése
legfeljebb az utolsó 1024 intervallumot tartja. A cross-layer flow elemzés
rétegenként egy cursort használ a teljes futás RAM-ba töltése helyett.

A runtime leállásakor a már létező 50 Hz-es timing accumulator összegzése
`v3.runtime_timing` eseményként mentődik. Ez különbözik az 5 Hz-es minták
periódusától és a sidecar saját CPU/RSS/írási metrikáitól. A wall/CPU
fázisidők nem választják külön a GIL-várást és a scheduler-preemptiont.
Report nélkül megszakadt futásból ez az összegzés hiányozhat.

A production capture sidecar a saját interpreterében 1 Hz-enként mintázza
a CPU magonkénti frekvenciáját (legfeljebb 64 mag), összterhelését, loadot,
memóriát, első thermal zone hőmérsékletét és a capture processz CPU/RSS
értékeit. Minden kernelolvasás byte-korlátos; késés után nincs pótló burst.
A `v3.host_telemetry` source topic megőrzi a measurement/completion/publication
időt és a sample sequence-et. Hiányzó vagy hibás kerneladat explicit unavailable.
Ez host evidence, nem V3 health-authority és nem GIL/scheduler szétválasztás.
Virtuális idejű direct szimuláció nem kever hozzá valós host-időt.

A felsőrétegek public-runtime journalja a session alatti új eseményeket
őrzi; nincs capture-kezdéskori Brain/skill checkpoint. A HRI follower boot-azonos,
bounded 60 másodperces előzményt is követ. A felvételt a feladat előtt kell
megnyitni, ha annak teljes felsőrétegi eseménysora szükséges.

A futási JSON reportok source identityt, explicit küszöböket és coverage-et
tartalmaznak, így azonos konfiguráció és coverage mellett összevethetők.
Az L3-mozgás becslés; az 5 Hz-es felvétel nem bizonyít köztes fizikai
trajektóriát, exact replay-t vagy teljes térképet. A teljes 50 Hz-es
sensor-debug és a canonical Replayer külön, változatlan capability.

## Validációs határ

Offline tesztek ellenőrzik a köztes tick eseményének megőrzését, direct/process
egyezést, MCAP→EVI→DIAG lineage-et, payloadkorlátokat, adatvesztést,
kvótakimerülést és virtuális kétórás 5 Hz-es sessiont. A teljes replay-út
meglévő MATCH/loss negatív tesztje is megmarad. Ez nem helyettesít Pi-n
végzett 30–120 perces CPU/RSS/I/O/jitter mérést vagy élő navigációs evidence-et.
