# Acceptance criteria

A new live run is considered successful for this encoder change only if:

1. `left/right_invalid_alert_delta` remain zero during normal motion.
2. Isolated `*_quadrature_rejection_delta` may occur, but do not create an
   opposite signed edge or L11 fault.
3. A real commanded reversal produces `direction_changes_confirmed` and the
   signed count changes direction only after 3 consistent candidates.
4. `direction_change_candidates` without `direction_changes_confirmed` are
   visible in MCAP and do not create a `direction_boundary` in the estimator.
5. Persistent signal loss/ambiguity still ends in the existing bounded L11
   uncertainty fault (250 ms); this package does not weaken fail-closed safety.
6. `proba` phases with one stationary wheel remain allowed by the existing
   per-wheel L11 validity policy.
