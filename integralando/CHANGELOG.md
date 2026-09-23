# Changelog

## r2b4_progress_viability_20260923

- Adds `TrajectoryEvaluation.progress_potential_score` and `progress_viable` as backward-compatible defaulted fields.
- Adds `NavigationConfig.progress_viability_floor` (default/production `0.02`).
- Computes generic local progress from max(distance improvement, heading-alignment improvement).
- Changes normal-vs-escape rollout fallback to use progress viability rather than merely `v > 0`.
- Keeps full 54-candidate normal rollout when any collision-free progress-viable candidate exists.
- Makes L7 ignore non-progress-viable normal candidates while preserving the existing escape family.
- Adds progress potential as a deterministic L7 tie-break after `total_score`.
- Adds focused regression tests for stationary local optima, productive pivots, escape fallback and candidate-count preservation.
