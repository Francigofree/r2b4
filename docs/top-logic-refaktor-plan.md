



┌──────────────────────────────────────────────┐
│                 GOAL INPUT                   │
│                                              │
│ user / schedule / autonomous events          │
│ LLM/VLM may PROPOSE, never own authority     │
└─────────────────────┬────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────┐
│                BRAIN CORE                    │
│                                              │
│ single goal owner                            │
│ priority / cancellation / preemption         │
│ task lifecycle / lineage                     │
└─────────────────────┬────────────────────────┘
                      │
                 PENDING GOAL
                      │
                      ▼
┌──────────────────────────────────────────────┐
│          LOCAL TASK PLANNER                  │
│                                              │
│ hierarchical skill methods                   │
│ preconditions                                │
│ expected effects                             │
│ ACTION_CATALOG                               │
│ Public World queries                         │
│                                              │
│ stateless / no robot authority               │
└─────────────────────┬────────────────────────┘
                      │
              cannot resolve?
              ┌───────┴───────┐
              │               │
             no              yes
              │               │
              │          LLM / VLM
              │          specialist
              │               │
              └───────┬───────┘
                      ▼
┌──────────────────────────────────────────────┐
│              TYPED TASK GRAPH                │
│                                              │
│ dependencies / conditions                    │
│ priority / timeout                           │
│ retry / recovery                             │
│ cancellation / bounded loops                 │
└─────────────────────┬────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────┐
│          BRAIN EXECUTIVE                     │
│          = evolved BrainCore                 │
│                                              │
│ event-driven                                 │
│ asynchronous                                 │
│ preemptive                                   │
│ observable                                   │
└──────┬──────────────┬─────────────┬──────────┘
       │              │             │
       ▼              ▼             ▼
 canonical       BehaviorSystem   conditions/
 actions          long skills      World waits
       │              │             │
       └──────────────┴─────────────┘
                      │
                      ▼
                RobotInterface
                      │
                      ▼
                  V3 L5
                      │
                      ▼
             V3 L6 Navigation
                      │
                      ▼
                 L7–L11
                      │
                      ▼
                 L12 Safety




Fejlesszük a meglévő Braint valódi event-driven typed-task-graph executive-vá, tegyünk elé egy könnyű hierarchikus lokális plannert, és használjuk alatta változatlanul a már meglévő BehaviorSystemet, RobotInterface-t és V3 intelligenciát.


-----------------------------------


## Végső robotikai célállapot

A refaktor után az R2B4 már nem úgy működne, hogy:

> „kap egy mondatot → elküldi LLM-nek → kap egy listát → végrehajtja”.

Hanem úgy, mint egy **lokális autonóm robot-executive rendszer**, amelynek az LLM/VLM csak külső kognitív specialistája.

A fő működési lánc:

```text
CÉL
 ↓
Brain goal ownership
 ↓
lokális értelmezés
 ↓
lokális feladatbontás
 ↓
typed TaskGraph
 ↓
event-driven végrehajtás
 ↓
World / Behavior / V3
 ↓
fizikai végrehajtás
 ↓
eredmény / recovery / újratervezés
```

A robot a feladat jelentős részét **internet és LLM nélkül** végigviszi.

---

# 1. Többlépéses feladatokat önállóan megért és végrehajt

Ma egy ilyen:

> „Menj előre 1 métert, fordulj jobbra 90 fokot, menj még fél métert.”

LLM-függő.

A célállapotban:

```text
Goal
 ├─ MOVE_RELATIVE 1.0 m
 ├─ TURN_BY -90°
 └─ MOVE_RELATIVE 0.5 m
```

A teljes értelmezés, tervezés és végrehajtás lokális.

**Új képesség:** általános, determinisztikus többutasításos robotfeladatok LLM nélkül.

---

# 2. Nem csak lineáris terveket, hanem feltételes feladatokat kezel

Ez az egyik legnagyobb minőségi ugrás.

Például:

> „Menj az ajtóhoz. Ha nem lehet odajutni, próbálj másik utat. Ha az sem sikerül, szólj.”

TaskGraph:

```text
NAVIGATE door
      │
   success
      ▼
   COMPLETE

      │ failure: NO_PATH
      ▼
WAIT / REFRESH WORLD
      │
      ▼
RETRY NAVIGATE
      │
   failure
      ▼
REPORT FAILURE
```

**Új képesség:** feltételes végrehajtás, branch, recovery és kontrollált retry.

---

# 3. Hierarchikus feladatokat tud lebontani

Nem kell minden emberi kérést közvetlenül primitív akciókra fordítani.

Példa:

> „Keresd meg Pétert és kövesd.”

Lokálisan:

```text
FIND_AND_FOLLOW(Péter)

 ├─ resolve known person
 ├─ determine candidate locations
 ├─ search_person
 ├─ bind exact runtime target
 └─ follow_person
```

A `SearchPerson`, target binding és `FollowPerson` jelentős része már ma megvan. A refaktor után ezeket a planner **önállóan komponálja**.

**Új képesség:** komplex képességek összerakása meglévő egyszerűbb képességekből.

---

# 4. Valódi World-aware viselkedése lesz

A planner nem pusztán szöveget alakít parancsokká.

Figyelembe veszi például:

```text
hol van a robot
melyik room ismert
hol láttunk utoljára egy személyt
milyen objektum ismert
melyik ajtó / hely / cél létezik
mely capability érhető el
frissek-e az adatok
fut-e már másik task
```

Példa:

> „Menj a konyhába.”

Nem az LLM talál ki koordinátát.

```text
"konyha"
   ↓
Public World
   ↓
room_topology / location
   ↓
NAVIGATE(location)
```

**Új képesség:** a robot saját világmodelljére alapozott feladattervezés.

---

# 5. Tud keresni, nem csak reagálni

Például:

> „Keress valakit a lakásban.”

Ha a Worldben ismert:

```text
nappali
konyha
előszoba
hálószoba
```

akkor:

```text
nappali
 ↓
observe
 ↓ no person
konyha
 ↓
observe
 ↓ no person
előszoba
 ↓
person found
 ↓
COMPLETE
```

Ehhez nem kell LLM.

Ez már valódi **autonóm keresési feladat**.

---

# 6. Körül tud nézni céllal

Új általános skill lehet például:

```text
LOOK_AROUND
```

amely:

- több irányba fordul;
- minden nézethez friss képet kér;
- megőrzi a frame/pose lineage-et;
- a képeket további helyi vagy VLM feldolgozásnak adja.

Példa:

> „Nézz körül.”

Teljesen lokális.

> „Nézz körül, és keresd meg a piros széket.”

A mozgatás és képkészítés lokális; csak a **„piros szék” vizuális szemantikája** kerülhet VLM-hez.

Ez lényegesen jobb, mint az egész taskot LLM-re bízni.

---

# 7. LLM nélkül is működőképes robot marad

Ez az egyik fő cél.

Internet/LLM kiesés esetén továbbra is működnek például:

- STOP;
- előre/hátra;
- fordulás;
- több lépéses motion;
- navigáció ismert célhoz;
- Room Cruise;
- emberkeresés;
- emberkövetés;
- keresés → követés;
- kamera observation;
- look-around;
- feltételes retry;
- World-query;
- ismert objektumhoz/helyhez navigáció;
- task státusz;
- ismert hiba magyarázata.

Az LLM kiesése így:

```text
ma:
robot funkciók jelentős része elveszhet

célállapot:
csak a nyílt nyelvi / magas szemantikai képességek esnek ki
```

---

# 8. Az LLM valódi escalation layer lesz

Az LLM csak akkor kerül elő, amikor a lokális rendszer nem tudja biztonságosan feloldani a célt.

Például:

> „Menj oda.”

Ha nincs egyértelmű referens:

```text
LOCAL_RESOLUTION_FAILED
       ↓
clarification / LLM
```

Vagy:

> „Nézd meg, hogy rendetlenség van-e a szobában.”

Ez nyílt vizuális szemantika:

```text
local LOOK_AROUND
       ↓
fresh images
       ↓
VLM semantic analysis
```

Viszont az LLM/VLM **nem kap végrehajtási authorityt**.

Csak javasolhat:

```text
proposal
   ↓
Brain validation
   ↓
execution
```

---

# 9. Saját hibáiból task-szinten képes reagálni

Nem „tanulásról” van szó, hanem intelligens runtime recoveryről.

Példák:

```text
person temporarily lost
→ V3 local reacquire

path temporarily blocked
→ V3 local replan

final NO_PATH
→ Brain task recovery

camera unavailable
→ capability failure

stale World fact
→ refresh / query again

runtime restarted
→ invalidate target identity
```

A rétegek nem veszik át egymás munkáját.

Ez különösen fontos.

---

# 10. Tudni fog különbséget tenni hibatípusok között

Ma sok felső hiba egyszerűen string reason.

A célállapotban például:

```text
NO_PATH
TARGET_LOST
STALE_WORLD
CAPABILITY_UNAVAILABLE
SAFETY_STOP
RUNTIME_RESTART
IDENTITY_INVALID
TIMEOUT
CONTRACT_FAILURE
EXECUTION_FAILURE
```

Ez lehetővé teszi, hogy:

```text
NO_PATH
→ esetleg retry

TARGET_LOST
→ search

STALE_WORLD
→ refresh

SAFETY_STOP
→ NINCS automatikus retry

RUNTIME_RESTART
→ target invalidáció
```

Ez már valódi executive viselkedés.

---

# 11. Eseményre reagáló robot lesz

A Brainnek nem csak időnként kell lekérdeznie:

> „történt valami?”

Hanem például:

```text
PERSON_FOUND
MISSION_COMPLETE
NAVIGATION_FAILED
WORLD_FACT_UPDATED
BEHAVIOR_COMPLETED
CAMERA_FAILED
SAFETY_STOP
```

felébreszti a megfelelő taskot.

Fontos: az esemény **nem lesz authority**. Csak trigger.

Utána a Brain újra lekéri a canonical state-et.

---

# 12. Jobban kezeli a megszakítást és prioritást

Például fut:

```text
keresd meg Pétert
```

majd érkezik:

> „STOP”

Azonnal:

```text
task cancel
behavior revoke
canonical STOP
```

Vagy:

```text
autonomous exploration
```

közben érkezik:

> „gyere ide”

A magasabb prioritású emberi goal preemptálhatja az autonóm goal-t.

A Brain ennek az alapját már most tudja; a TaskGraph ezt általánosítja.

---

# 13. Képes lesz hosszabb feladatok koherens végrehajtására

Például:

> „Menj a nappaliba, nézz körül, keress valakit. Ha találsz valakit, kövesd két percig, aztán gyere vissza.”

Ez már lehet:

```text
NAVIGATE living_room
       ↓
LOOK_AROUND
       ↓
SEARCH_ANY_PERSON
       ↓ found
BIND_TARGET
       ↓
FOLLOW_PERSON 120s
       ↓
NAVIGATE origin
       ↓
COMPLETE
```

Ehhez nincs szükség folyamatos LLM-beszélgetésre.

Az LLM legfeljebb az első mondat értelmezésében segít, ha a lokális parser nem elég biztos.

---

# 14. Képes lesz saját korábbi taskját értelmezni

Példa:

> „Miért nem találtad meg Pétert?”

Nem feltétlen kell LLM.

Brain/World/evidence alapján:

```text
goal: find Peter
searched:
  living_room ✓
  kitchen ✓
  hallway ✓

result:
SEARCH_PLACES_EXHAUSTED

camera:
available

navigation:
successful

person identity:
no fresh matching observation
```

Válasz:

> „A három ismert keresési helyet végigjártam, de egyik helyen sem volt friss Péter-azonosítás.”

Ez fontos új robotikai képesség: **introspekció saját execution history alapján**.

---

# 15. Jobban tud majd autonóm eseményekből goalokat indítani

A Goal Layer nem csak user input lehet.

Például később:

```text
schedule
timer
battery event
person detected
door state change
external automation
system event
```

indíthat goal-t.

Például:

```text
18:00
→ patrol living room

person detected
→ observe / report

scheduled task
→ navigate + inspect
```

Ugyanaz a Brain kezeli őket prioritással.

---

# Milyen „intelligenciaszintre” kerül ezzel?

Nem általános mesterséges intelligencia.

Hanem egy sokkal értékesebb robotikai kategória:

## **goal-driven autonomous mobile robot**

amely rendelkezik:

- world modellel;
- goal lifecycle-lal;
- hierarchikus task decompositionnel;
- typed task graphgal;
- reactive skill-ekkel;
- event-driven executive-val;
- condition handlinggel;
- bounded recoveryvel;
- capability awareness-szel;
- task introspectionnel;
- LLM/VLM escalationnel.

---

# Konkrét képességszint a refaktor végén

| Kérés | Lokális | LLM | VLM |
|---|---:|---:|---:|
| Menj előre 1 m | ✅ | – | – |
| 1 m → jobb 90° → 0,5 m | ✅ | – | – |
| Menj a konyhába | ✅ ha World ismert | – | – |
| Menj oda, készíts képet, gyere vissza | ✅ | – | – |
| Nézz körül | ✅ | – | – |
| Keress valakit | ✅ | – | – |
| Keresd meg és kövesd | ✅ | – | – |
| Menj az ajtóhoz, kezeld a blokkolást | ✅ | – | – |
| Miért állt meg az előző task? | ✅ legtöbbször | opcionális | – |
| Hol van a piros szék? | részben | – | ✅ |
| Mi történik ezen a képen? | részben | – | ✅ |
| „Rakj rendet itt” | részben | ✅ | valószínű |
| teljesen nyílt, új feladat | planner fallback | ✅ | szükség szerint |

---

# A legfontosabb változás

A robot **nem több capabilityt kap attól, hogy új motor- vagy navigation-kódot írunk**.

A nagy ugrás az lesz, hogy a már meglévő képességeket:

```text
navigate
turn
move
observe
search
follow
explore
World query
```

képes lesz **önállóan, célorientáltan és feltételesen összerakni**.

Ez jelenleg az R2B4 legnagyobb kihasználatlan potenciálja.

## A végső állapot egy mondatban

> **Az R2B4 egy lokálisan gondolkodó, world-aware, goal-driven mobil robot lesz, amely több lépéses és feltételes feladatokat önállóan megtervez, végrehajt, monitoroz és korlátozottan újratervez, miközben az LLM/VLM csak akkor lép be, amikor valóban nyelvi vagy vizuális szemantikai intelligenciára van szükség.**






           COGNITIVE SYSTEM
 Brain / Planner / TaskGraph
              │
              ▼
        RobotInterface
        /      |      \
       /       |       \
 Public     Spatial      V3
 World      Service    Runtime
              │          │
              │          ▼
              │      Local World
              │      Navigation
              │      Motion
              │      Safety
              │
              ▲
        passive V3 evidence



        