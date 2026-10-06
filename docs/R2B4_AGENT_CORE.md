# R2B4 Agent Core

`R2B4_AGENT_SYSTEM_V3` a provider-neutral Agent hierarchikus prompt-alapja.
A Brain Core a robot primary goaljának és teljes lifecycle-jának tulajdonosa;
az Agent csak igény szerint értelmez, válaszol és tervet javasol.

## Prompt hierarchy

```text
SYSTEM_CORE
  stable robot identity + authority + behavior policy
        |
        +-- ROBOT_CONTEXT
        |     fresh runtime state + canonical available actions
        |
        +-- CAPABILITY_CATALOG
        |     current Agent Core tools
        |
        +-- SELF_KNOWLEDGE
        |     optional bounded read-only hints
        |
        +-- TOOL_RESULT
        |     bounded result data from requested tools
        |
        +-- conversation history
        |
        `-- current user request
```

A `SYSTEM_CORE` az egyetlen stabil policy/personality réteg. A dinamikus system
blokkok adat-rétegek: friss állapotot és capability-ket közölnek, de nem írhatják
felül a core policyt.

## Execution architecture

```text
human text
   |
   +-- exact STOP --> canonical v3.command.stop
   |
   `-- normal input --> Brain PENDING goal --> AgentCore / configured LLM
                                               |
                                               +-- final answer
                                               +-- public state/vision reads
                                               +-- canonical action / bounded plan proposal
                                               `-- explicitly triggered ER2 reasoning specialist
                                                          |
                                      Brain validates/adopts proposal
                                               |
                                      action / BehaviorSystem
                                               |
                                         RobotInterface
                                               |
                                      CommandGateway --> V3 L0–L12
```

Normál HRI input a közös `PublicRobotRuntime` Brain ownerénél létrehoz egy
PENDING goalt a modellhívás előtt. A tisztán beszélgetési válasz ezt lezárja,
és nem preemptálja az aktív fizikai feladatot. Robotműveletnél a HRI a
`brain.adopt` capabilityn át adja vissza a javaslatot. Az Agent nem hív fizikai
commandot, nem léptet behaviort és nem tart fenn külön robotvilágot.

Egy sikeres adoption nem completion. A Brain Agent-hívás nélkül lépteti a
részfeladatokat, megőrzi a target bindingot és a felhasználói constraintet,
majd korrelált mission/behavior/observation eredmény alapján zárja le a goalt.
A voice végső feedbackje a Brain-goal lifecycle-t olvassa, nem az első mission
completionjéből következtet a teljes kérés sikerére.

## V3 behavioral contract

A V3 prompt explicit embodied identityt ad a modellnek: a felhasználó felé az
Agent Core az R2B4 fizikai robot reasoning/kommunikációs rétege. Ettől nem kap új
authorityt. Érzékelést vagy végrehajtást csak friss context, action vagy tool
eredmény alapján állíthat.

A prompt külön kezeli a teljes több-lépéses feladatot és a robot saját
percepcióját. Vizuális kéréseket — például „mit látsz?”, „mi van előtted?”,
„mi megy a TV-ben?” — a robot környezetére kell groundolni, ha a capability
catalog ehhez vizuális utat biztosít.

Egyszerű canonical action maradjon canonical action proposal. Több lépéses
feladat `kind=plan` és bounded `plan_json` formában javasolható: legfeljebb
16 canonical lépés, explicit completion, constraint, target binding és
legfeljebb 2 retry. A Brain a static capability contractot és dispatchkor a
live availabilityt ellenőrzi. Nem támogatott feltételt nem dob el.

ER2 kizárólag a jelenlegi user turn explicit `ER2` triggerével érhető el.
A specialist `tools=false` mellett reasoning/vision eredményt ad; fizikai
toolt nem kap és nem lesz második goal owner. Source/config/evidence
fejlesztőtool továbbra is külön, host által explicit választott
`--developer-mode` felszín. A modell és a szöveges input ezt nem emelheti meg.

## Compatibility

A meglévő egy-action `RobotAction` proposal és közvetlen executor API
kompatibilitása megmarad; a normál HRI fizikai executionbe csak Brain adoption
után lép. Az Agent reply új `kind=plan`/`plan_json` mezője és a conversation
`goal_id`/`proposed_plan` eredménye a provider-közös contractban él.
Régi egy-action provider-válasz a parserben továbbra is elfogadható.

A `PROMPT_VERSION` és a system-prompt header együtt V3-ra emelkedik. A prompt
assembler fail-closed módon elutasítja a régi headerrel betöltött system promptot,
így a journalban szereplő prompt-verzió és a tényleges prompt nem csúszhat szét.
