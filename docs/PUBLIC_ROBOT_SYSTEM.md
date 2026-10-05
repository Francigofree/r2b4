# Publikus robotrendszer

A rendszer működési authorityja a `R2B4_SYSTEM_BEHAVIOR_CONTRACT.md`.
Ez a dokumentum a megvalósított felszínt és annak bizonyítási határát írja le.

## Tulajdonosok és adatút

```text
CLI / Voice / ER2 / Agent
          │
          ▼
    RobotInterface ──────► kamera/vision owner (közvetlen képút)
          │
          ├──► PublicRobotRuntime: PublicWorldModel + BehaviorSystem
          │             │                         │
          │             ▲                         ▼
          │       compact V3 status         publikus robot intent
          │                                       │
          └──────────► OperatorController ◄────────┘
                              │
                         CommandGateway
                              │
                           L0–L12
```

`v3.robot_interface.RobotInterface` a canonical host felszín. A normál
production kompozíció ugyanahhoz a projektazonosítóhoz tartozó host ownerhez
kapcsolja a külön CLI/voice/agent processzeket. A host owner első publikus
world/behavior kérésre indul; ettől sem V3, sem kamera, sem motor nem indul.
V3 nem importálja a felső owner implementációját. A régi `v3.command.explore`
és `v3.command.follow_person` külső nevek a canonical host behaviorhoz vezetnek.
A behavior által injektált RobotInterface a meglévő V3 execution primitive-et
hívja, így nem jön létre rekurzív behavior-indítás.

L4 továbbra is a friss operational local world, a lokális navigation/tracking,
coverage/progress, recovery és feasibility ownere. Room Cruise hostoldali
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
| `read("v3.status")` | Friss, élő V3 execution státusz; nem public world authority |
| `execute("world.observe", ...)` | Explicit semantic observation, saját measurement/source/lineage |
| `execute("behavior.room_cruise", ...)` | Bounded host Room Cruise |
| `execute("behavior.follow_person", ...)` | Bounded host Follow Person |
| `execute("behavior.search_person", ...)` | Névhez/entitáshoz kötött, világmodellből dolgozó keresés |
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

Normál Agent-turn publikus state/vision és deklarált robot capabilityk
használója. Source/config/evidence fejlesztőtool csak explicit
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

## Behavior és STOP

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

## Személykeresés és bizonyítási határ

SearchPerson a `room_topology` domainben publikált, ismert helyek `location`
factjeit használja (`frame_id`, `x_m`, `y_m`). Régi `person_position/location`
fact `place_id` vagy koordináta alapján csak sorrendi preferenciát adhat.
Matching mission completion után a program vision observationt kér, majd
bounded ideig vár friss semantic identity evidence-re és szükség esetén a
következő területet választja. Minden mozgást a V3 realizál és helyi akadály
esetén megtagadhat.

A „megtaláltam” eredményhez a kért entitás friss, megfelelő confidence-ű,
source/lineage-dzsel rendelkező `person_position/location` observationje kell.
Kamerakép és név nélküli detector-track nem személyazonosítás. A jelenlegi
detector nem tanulja meg automatikusan Laci arcát; a névhez kötött semantic
observationhez külön azonosító producer vagy explicit külső observation kell.
Keresési exhaustion, camera hiba, stale status és invalid navigation explicit
FAILED/STOP. Behavior siker a közös állapotban és evidence-ben jelenik meg;
automatikus új beszélgetési turn/TTS értesítés nincs hozzáadva.

Szoftveres evidence: idő/clock/sequence/conflict/storage bounds, facade-egység,
preemption/STOP race, runtime/developer határ és többterületes SearchPerson
szimuláció. Kötelező validáció: kis `./r test` gate; közös boundary-változás
miatt `./r test release` is. Új fejlesztőtesztek a feature packben maradnak,
a permanent manifest budgetje változatlan. Valós szemantikai azonosítás,
élő navigáció, felső host timing és hardveres elfogadás külön evidence-et
igényel; ebből a változtatásból önmagában nem következnek.
