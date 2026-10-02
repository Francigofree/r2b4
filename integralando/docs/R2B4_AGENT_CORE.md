# R2B4 Agent Core

`R2B4_AGENT_SYSTEM_V2` a provider-neutral Agent Core hierarchikus prompt-alapja.
Nem robot-authority és nem hoz létre alternatív vezérlési utat.

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
   `-- everything else --> AgentCore --> configured LLM
                                  |
                                  +-- final answer
                                  +-- source/docs/config/EVI/DIAG tools
                                  +-- policy-gated config.patch
                                  +-- canonical RobotAction proposal
                                  `-- ER2 delegation
```

Robot actions továbbra is proposalok, amíg a meglévő fresh-state validator és
`RobotInterface` út el nem fogadja őket. ER2 mozgás továbbra is kizárólag az
`ExternalRobotGateway -> RobotInterface -> V3` útvonalon történhet.

## V2 behavioral contract

A V2 prompt explicit embodied identityt ad a modellnek: a felhasználó felé az
Agent Core az R2B4 fizikai robot reasoning/kommunikációs rétege. Ettől nem kap új
authorityt. Érzékelést vagy végrehajtást csak friss context, action vagy tool
eredmény alapján állíthat.

A prompt külön kezeli a teljes több-lépéses feladatot és a robot saját
percepcióját. Vizuális kéréseket — például „mit látsz?”, „mi van előtted?”,
„mi megy a TV-ben?” — a robot környezetére kell groundolni, ha a capability
catalog ehhez vizuális utat biztosít.

Egyszerű canonical action maradjon canonical action. Több lépéses
mozgás+megfigyelés feladat ne redukálódjon az első rész-actionre; ha a teljes
feladat ER2-vel reprezentálható megfelelően, a teljes célt kell delegálni.

## Compatibility

A refaktor nem módosítja:
- `RobotAction` sémát,
- fresh-state action validációt,
- action catalogot,
- `RobotInterface` authorityt,
- V3 safety útvonalat,
- ER2 gateway authorityt,
- Gemini/Groq provider transportot.

A `PROMPT_VERSION` és a system-prompt header együtt V2-re emelkedik. A prompt
assembler fail-closed módon elutasítja a régi headerrel betöltött system promptot,
így a journalban szereplő prompt-verzió és a tényleges prompt nem csúszhat szét.
