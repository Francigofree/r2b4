# Validator encoder count-window identity audit

The wheel-tracking validator previously defined an independent feedback sample
from `(left reference, right reference, left measurement, right measurement,
window end)`. A deterministic replay proved that changing only the controller
reference while retaining the same canonical encoder count window incremented
`independent_feedback_windows`. One physical measurement could therefore be
weighted more than once.

The canonical independent-observation identity is now the immutable count
window:

`(window_start_ts, window_end_ts, left_count_start, left_count_end,
right_count_start, right_count_end)`.

Reference and derived velocity remain payload used to calculate tracking error;
they are not observation identity. A set, rather than adjacent-row comparison,
also prevents a late repeated window from being counted again. If the complete
identity is missing or partially published, the phase conservatively treats all
such rows as at most one unproven window. Poll-weighted diagnostics remain
available and are explicitly separate from the independent-window gate.

This change modifies no motor, encoder, timing, motion-quality or safety
threshold. It does not reinterpret the current M0 hardware/timing failures.

## Verification

- Deterministic repeated/late-window replay: the same count window with a
  changed reference is counted once; two distinct windows are counted twice.
- Partial-window simulation: incomplete identity returns no canonical window
  ID and follows the conservative missing-identity bucket.
- Targeted validator regression: `39 passed`.
- Replay of `latest_M1_motion_baseline_live_samples.jsonl`: every case retained
  its previous independent-window count and settled MAE.
- Full offline regression: `1041 passed`.
- Bootstrap guard: `PASS`.

No live profile was run for this validator-only change because the immediately
preceding M0 proved a right-side physical actuation blocker. A fresh M0 remains
mandatory after that blocker is physically resolved; M1 remains gated.
