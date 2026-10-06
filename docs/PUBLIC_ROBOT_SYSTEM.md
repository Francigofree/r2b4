# Publikus robotrendszer

A rendszer működési authorityja a `R2B4_SYSTEM_BEHAVIOR_CONTRACT.md`.
Ez a dokumentum a megvalósított felszínt és annak bizonyítási határát írja le.

## Tulajdonosok és adatút

```text
Human / CLI / Voice / autonomous event
          │
       HRI gate ── STOP ──► canonical STOP + upper revocation
          │
          ▼
 PublicRobotRuntime: BrainCore + PublicWorldModel + BehaviorSystem
          │             │
     Agent proposal ◄── Brain goal owner
          │             │
          └─────► Brain admission / subtask lifecycle
                        │
                 RobotInterface ──────► kamera/vision owner
          │
          └──────────► OperatorController
                              │
                         CommandGateway
                              │
                           L0–L12
```

`v3.robot_interface.RobotInterface` a canonical host felszín. A normál
production kompozíció ugyanahhoz a projektazonosítóhoz tartozó host ownerhez
kapcsolja a külön CLI/voice/agent processzeket. A host owner első publikus
world/behavior/Brain kérésre indul; ettől sem V3, sem kamera, sem motor nem indul.
V3 nem importálja a felső owner implementációját. A régi `v3.command.explore`
és `v3.command.follow_person` külső nevek a canonical host behaviorhoz vezetnek.
A behavior által injektált RobotInterface a meglévő V3 execution primitive-et
hívja, így nem jön létre rekurzív behavior-indítás.

L4 továbbra is a friss operational local world és tracked entity-k ownere.
L6 birtokolja a lokális navigationt, coverage/progress, recovery és feasibility
döntéseket. Room Cruise hostoldali
életciklusa és kért profilja ezen felül él; a lokális EXPLORE algoritmus
megmarad. Follow Person ugyanezt az életciklust használja, az aktuális fizikai
személytrack kezelése pedig V3 feladat. A szemantikus SearchPerson a publikus
világmodellből választ következő területet, és a publikus navigációt használja.

## API

| Művelet | Jelentés |
| --- | --- |
| `capabilities()` | Statikus descriptor + aktuális availability/readiness/owner |
| `read("robot.state")` | Közös temporal world, active behavior, mission és felső health |
| `read("world.snapshot")` | Immutable szemantikai facts, confidence, kor, freshness, lineage |
| `read("world.history")` | Bounded observation history; loss/eviction a snapshotban explicit |
| `read("behavior.state")` | Canonical lifecycle, deadline, command/mission identity, reason |
| `read("behavior.history")` | Intentek, állapotváltások, preemption, eredmény |
| `read("brain.state")` | R2B4 identity/RUNTIME role, egy primary goal, pending és background goalok |
| `read("brain.history")` | Bounded Brain-döntések, részfeladat-, behavior- és command lineage |
| `execute("brain.submit", text=..., source=...)` | PENDING input HUMAN/AUTONOMOUS/SAFETY prioritással; még nincs fizikai preemption |
| `execute("brain.adopt", goal_id=..., plan=...)` | Constraint/capability-validáció, priority/preemption és bounded terv indítása |
| `execute("brain.fail", goal_id=..., reason=..., pending_only=True)` | Lezáratlan értelmezési input lezárása; ANSWERED válasz, egyéb ok failure |
| `execute("brain.cancel", reason=...)` | Goal és felső intent visszavonása, canonical STOP |
| `read("v3.status")` | Friss, élő V3 execution státusz; nem public world authority |
| `execute("world.observe", ...)` | Explicit semantic observation, saját measurement/source/lineage |
| `execute("behavior.room_cruise", ...)` | Bounded host Room Cruise |
| `execute("behavior.follow_person", ...)` | Bounded host Follow Person |
| `execute("behavior.search_person", ...)` | Névhez/entitáshoz kötött, világmodellből dolgozó keresés |
| `execute("behavior.search_any_person", ...)` | Friss anonim személytrack keresése, korrelált target eredménnyel |
| `execute("behavior.start", name=..., ...)` | Telepített, regisztrált normál Python behavior indítása |
| `execute("behavior.cancel", reason=...)` | Explicit cancellation és canonical STOP |
| `execute("vision.observe", stream="lores")` | Kalibrált JPEG és eredeti producer metadata, V3 nélkül |
| `stop()` | Felső revocation + prioritásos canonical STOP; késő start nem éledhet újra |

Az új CLI felszín ugyanazt az API-t használja:

```bash
./r read robot.state --json
./r read world.snapshot --json
./r read behavior.state --json
./r caps --json
```

Példa robotműveletet kérő használatra, kizárólag operátori mozgásengedéllyel:

```bash
./r execute behavior.room_cruise --parameters '{"max_duration_s":30}' --json
./r execute behavior.search_person --parameters '{"entity_id":"Laci","candidate_places":["nappali","konyha"]}' --json
./r x
```

Normál HRI-turn a Brainnél PENDING inputot hoz létre, majd az Agent publikus
state/vision adatból action vagy bounded terv javaslatot ad. A HRI ezt a
`brain.adopt` capabilitynek adja vissza; a Brain birtokolja a célt és Agent
periodikus hívása nélkül lépteti a részfeladatokat. Tisztán beszélgetési turn
nem preemptál aktív fizikai goalt. ER2 explicit triggerrel engedélyezett,
`tools=false` reasoning specialist; normál canonical feladat nem igényli.
Source/config/evidence fejlesztőtool csak explicit
`python -m r2b4_voice.conversation_cli --developer-mode --text ...` módban érhető
el. A runtime turn és a provider nem változtathatja meg ezt a módot.
Keréksebesség-parancs nem Voice/Agent capability; az operátori CLI meglévő
kinematikai diagnosztikája továbbra is a canonical command úton működik.

## Idő, bizonytalanság és memória

A `PublicWorldModel.observe` érték mellett domaint, measurement/observation
monotonic időt, confidence-et, source-t, lineage-et, opcionális producer
sequence/revisiont és clock epochot rögzít. A publikus age a measurementtől
számított kor. A domain policy választ KNOWN/LIKELY/STALE/UNKNOWN/CONFLICTING
állapotot; a snapshot olvasása nem frissít adatot. Régebbi, duplikált vagy
producer-sorrendben visszalépő observation nem írja felül az újabbat.

V3 statusból csak completed, compact estimate/mission/health és fizikai
person observation kerül ide. A pose ideje L3 estimate referenciaidőként
jelölt. PREDICTED/DEGRADED track nem válik új physical person observationné;
measurement timestamp és runtime/frame lineage megmarad. Az anonim track
entitása runtime-sessionhöz kötött; ember neve nem következik a track ID-ből.

Facts és history külön count- és összesített JSON-byte budgetet kapnak.
Snapshot/event értékek mélyen immutable-ek; raw kép, scan vagy tetszőleges
Python objektum nem world fact. Az eviction és history loss explicit.

A host owner saját helyi állományai: `runtime/public_world/state.json` és
`events.ndjson`. A state bounded, atomikusan frissített snapshot/history.
Más boot epochból visszatöltött tény STALE/CLOCK_MISMATCH marad az eredeti
idővel; frissességet csak új megfigyelés adhat. Sérült storage esetén az owner
DEGRADED healthöt közöl; a V3 helyi action és STOP függetlenül működik.
Meglévő capture/log/diagnosztikai állományokat ez az út nem dolgoz fel és nem
módosít. Evidence fájlba írás a host observation oldalon történik; STOP nem
vár ilyen fájlírásra. A behavior world-input snapshotjai és intentjei
revision/epoch alapján összekapcsolhatók.

Brain döntés és downstream behavior/command esemény az ObservationHubon
passzívan látható. Goal/subtask/decision/behavior/command/mission identity
kapcsolja össze a lineage-et. Az observation sink, capture, MCAP és helyi
evidence-írás rendelkezésre állása nem előfeltétele Brain adoptionnek,
lifecycle-léptetésnek vagy STOP-nak. Raw kép és sensor payload nem kerül a
Brain döntési eventjébe vagy a V3 control interpreterbe.

## Behavior és STOP

A Brain egyszerre egy primary fizikai goalt birtokol. Normál prioritás
SAFETY > HUMAN > AUTONOMOUS; háttér/maintenance metadata nem kap önálló
fizikai dispatch authorityt. Egy terv legfeljebb 16 részfeladatból áll,
retry legfeljebb 2; safety/stale/crash/identity failure nem retryolható
automatikusan. Explicit időtartam nem rövidülhet csendben caller watchdogra.
Időzített Room Cruise/Follow csak a kért időtartam bizonyított elérésével
complete; általános hard deadline exhaustion failure marad.

`BehaviorSystem` egy aktív programot kezel. A program publikus robot/world
interfészt kap, normál `start` és bounded `step` metódussal. Nincs DSL, behavior
tree, event bus, új numbered layer vagy párhuzamos motorvezérlés. A véges
idő/step budget, az ingress owner/watchdog és a canonical safety együtt
határolja a végrehajtást. Regisztrált program hozzáadása host source- és
tesztváltoztatás, nem V3-változtatás. Python programok megbízható, fejlesztői
telepítések; a rendszer nem futtat runtime LLM által tetszőlegesen beküldött
forráskódot és nem állít sandbox-garanciát tetszőleges Python importokra.

Új motion kérés explicit preemptiont végez. STARTING és korábban sorba állt
indítás is visszavonható; a generation ellenőrzés és a lezáró canonical STOP
meggátolja a késő visszatérésből származó újraaktiválást. A physical STOP a
host drain-várás előtt megkapja prioritását. World/capability olvasás nem
előfeltétel. Host transport hiba esetén a saját, ellenőrzött owner leállítható;
az autonóm command producer ezt a PID-t és saját időbudgetjét figyeli.

Brain/runtime restart után a world memory visszatölthető, a korábbi aktív
goal INTERRUPTED lesz; fizikai terv nem folytatódik automatikusan. A
runtime owner/process és command watchdog elvesztése a canonical fail-safe
útra vezet. A pending HRI értelmezések cancel/timeout/close esetén lezáródnak;
késő model proposal nem adhat új fizikai authorityt.

## Személykeresés és bizonyítási határ

SearchPerson a `room_topology` domainben publikált, ismert helyek `location`
factjeit használja (`frame_id`, `x_m`, `y_m`). Régi `person_position/location`
fact `place_id` vagy koordináta alapján csak sorrendi preferenciát adhat.
Matching mission completion után a program vision observationt kér, majd
bounded ideig vár friss semantic identity evidence-re és szükség esetén a
következő területet választja. Minden mozgást a V3 realizál és helyi akadály
esetén megtagadhat.

A jelenlegi V3 frame-ek runtime-sessionhöz kötöttek. Navigálható helyhez
`location.value.runtime_pid` is szükséges; ezt a publikus navigate admission
az operator transition alatt ellenőrzi. A keresés örökli az aktív capture
módot/frekvenciát és `capture=False` értéket használ, ezért nem re-armolhat
capture-t és nem indíthat új frame epochot a kiválasztott koordináta után.
Leállított V3 vagy régi sessionből származó helyadat esetén a keresés fail
closed; tartós térkép és új frame közötti szemantikai relokalizáció nincs
hozzáadva. Aktív FULL capture tovább rögzít; ALAP per-move capture trigger
helyett a keresés host decision evidence-je áll rendelkezésre.

A „megtaláltam” eredményhez a kért entitás friss, megfelelő confidence-ű,
source/lineage-dzsel rendelkező `person_position/location` observationje kell.
Kamerakép és név nélküli detector-track nem személyazonosítás. A jelenlegi
detector nem tanulja meg automatikusan Laci arcát; a névhez kötött semantic
observationhez külön azonosító producer vagy explicit külső observation kell.
Keresési exhaustion, camera hiba, stale status és invalid navigation explicit
FAILED/STOP. SearchAnyPerson friss anonim személytrackből adhat target
identityt; név szerinti azonosságot továbbra sem bizonyít. A Brain ezt a
runtime/sessionhez kötött targetet adja a következő follow részfeladatnak;
idegen track nem helyettesítheti csendben. A voice a Brain teljes goal
eredményét passzív observerrel közli. Kamera-JPEG megszerzése observation
evidence; önmagában nem bizonyít például egy lámpa állapotára vonatkozó
szemantikai következtetést.

Szoftveres evidence: idő/clock/sequence/conflict/storage bounds, facade-egység,
preemption/STOP race, runtime/developer határ és többterületes SearchPerson
szimuláció. Kötelező validáció: kis `./r test` gate; közös boundary-változás
miatt `./r test release` is. Új fejlesztőtesztek a feature packben maradnak,
a permanent manifest budgetje változatlan. Valós szemantikai azonosítás,
élő navigáció, felső host timing és hardveres elfogadás külön evidence-et
igényel; ebből a változtatásból önmagában nem következnek.
