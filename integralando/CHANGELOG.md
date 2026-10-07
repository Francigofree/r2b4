# Change summary

## P1 — default semantic-memory dump removed
- `R2B4_ROBOT_CONTEXT_V5` → `V6`.
- nincs automatikus `world.snapshot` read a default LLM contexthez.
- nincs automatikus `robot.state` read a default LLM contexthez.
- `world.query` és `robot.read` változatlanul elérhető on-demand.

## P1 — prompt growth bounded
- `PromptBudgetError`.
- per-layer és assembled prompt hard limits.
- provider előtti teljes request-size gate az AgentCore-ban.
- minden Agent round külön mérve, tehát tool-result felhalmozódás sem tud korlátlanul nőni.

## P2 — capability duplication removed
- promptban csak runtime action status.
- canonical descriptor / parameter schema csak a structured-output/action-catalog útvonalon marad.

## Evidence
- 15:35 capture/conversation default context: 143 997 char.
- azonos payload új compact projectionnel: 6 685 char.
- csökkenés: 137 312 char / 95,4%.
