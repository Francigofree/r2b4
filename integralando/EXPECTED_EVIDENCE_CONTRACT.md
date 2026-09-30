# Expected TestHub evidence contract after refactor

## Authority boundary

- MCAP remains the raw evidence authority.
- Replay proves deterministic reconstruction/divergence only.
- TestHub may emit:
  - `FACT`
  - `POLICY_VERDICT`
  - `HEURISTIC_FINDING`
- Every TestHub-derived claim is non-causal unless it is merely restating an
  explicit production result. The exported contract still sets
  `causal_claim=false`.
- Root-cause synthesis belongs to `ANALYZER_LLM`.

## Removed from newly generated evidence

- `root_cause_candidate`
- `root_cause`
- `physical_root_cause`
- `replayer_physical_root_cause`

Legacy-input compatibility may still recognize old field names internally, but
new evidence must not publish them as conclusions.

## Added/reframed

- `priority_evidence_candidate` in triage
- `priority_evidence` in diagnosis/agent-facing outputs
- `first_live_observation` in replay
- `physical_evidence` in replay/diagnosis
- `analysis_handoff.root_cause_inferred = false`
- `analysis_handoff.causal_analysis_owner = ANALYZER_LLM`
- `evidence_compiler_policy = NO_AUTOMATIC_ROOT_CAUSE_V1`
- incident/finding `claim_class`
- incident/finding `causal_claim = false`

## Noise control

Repeated actionable L2 admission states are edge-collapsed by the sorted
`(source_device_id, reason)` signature. Raw rejection counters remain available,
so compression removes repeated incident objects rather than evidence.

## Metadata correctness fixes

- `LocalizationQuality.generation` is a `COUNTER`, not `QUALITY/ratio`.
- zero raw-LiDAR missing scans means `raw_lidar_loss_within_tolerance = true`.

## Fresh-evidence acceptance checks

A newly generated `.evidence` bundle should satisfy:

1. `analysis_handoff.root_cause_inferred == false`.
2. No exported automatic root-cause object.
3. `priority_evidence.causal_claim == false` when priority evidence exists.
4. `physical_evidence` contains observations/evidence strength, not a `cause`.
5. Motion/localization findings declare `HEURISTIC_FINDING` and
   `causal_claim == false`.
6. Optional camera/person-detector degradation cannot be promoted into a
   physical explanation solely because it appears first.
7. Replay MATCH/MISMATCH semantics remain unchanged.
8. Capture integrity semantics remain unchanged except the zero-loss tolerance
   boolean correction.
