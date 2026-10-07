# R2B4 prompt-budget upgrade

**Source-first base:** `66ef376e90f1d3b54028f9e8a28be2d28f84440a`

## Mit változtat?

1. `RobotContextBuilder` nem olvassa és nem injektálja automatikusan a teljes `world.snapshot`-ot és a duplikáló `robot.state`-et.
2. A `RobotContextSnapshot` prompt/journal serializációja csak az actionök élő státuszát küldi (`name`, `available`, `ready`, `reason`). A teljes canonical descriptor továbbra is memóriában marad validációhoz és structured-output schema generáláshoz.
3. `PromptAssembler` réteg- és teljes szöveges hard budgetet érvényesít, valamint determinisztikus méret-telemetriát ad.
4. `AgentCore` minden provider-hívás előtt méri a message promptot + a tényleges structured-output schema méretét. 64k karakter felett a request **provider-hívás előtt** fail-closed módon leáll.
5. A HRI evidence/journal új prompt-méreteket kap: `prompt_text_chars`, `prompt_system_chars`, `response_schema_chars`, `provider_request_chars_estimate`, stb.

A csomag **nem** módosít L0–L12 robotikai authorityt, mozgásvezérlést, safety-t vagy hardver drivert.

## Telepítés

A kicsomagolt csomag könyvtárából:

```bash
python3 apply_upgrade.py /home/alba/project_r2b4
```

A telepítő először ellenőrzi az érintett source fájlok Git blob hashét. Ha a repo érintett source-a közben változott, **nem patch-el vakon**, hanem leáll.

A telepítés végén automatikusan fut:

```bash
python3 -m py_compile ...
python3 -m pytest -q tests/packs/core/test_prompt_budget_upgrade.py
python3 -m pytest -q tests/packs/core/test_agent_prompt_hierarchy.py
python3 -m pytest -q tests/packs/feature/test_agent_core.py
python3 -m pytest -q tests/packs/feature/test_public_robot_interface.py
```

Csak fájltelepítéshez, tesztek nélkül:

```bash
python3 apply_upgrade.py /home/alba/project_r2b4 --no-tests
```

## Rollback

A telepítő mentést készít ide:

`runtime/upgrade_backups/prompt_budget_66ef376/`

Visszaállítás:

```bash
python3 rollback_upgrade.py /home/alba/project_r2b4
```

## Várt hatás a bizonyított 15:35-ös mintán

A korábbi `ROBOT_CONTEXT_JSON` mérete **143 997 karakter** volt. Ugyanezen payloadon az új szabályokkal a default context **6 685 karakter**, azaz **95,4% kisebb**. A teljes provider input ennél nagyobb marad a system prompt, tool catalog és structured-output schema miatt, de a robot hosszú távú Public World memóriájának növekedése többé nem növeli lineárisan az initial LLM promptot.

## Új budgetek

- system core: 24 000 karakter
- default robot context: 16 000 karakter
- self knowledge: 16 000 karakter
- conversation history: 16 000 karakter
- assembled text prompt: 48 000 karakter
- provider request estimate (text + response schema): 64 000 karakter

A provider által visszaadott `input_tokens` továbbra is a tényleges tokenhasználat forrása; a fenti karakterértékek host-oldali determinisztikus admission limitek.
