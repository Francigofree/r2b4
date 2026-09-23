# Source-first implementation report

## Baseline

Repository: `Francigofree/r2b4`
Baseline commit: `04b5cb704b1f532028c49058f4fc416a1c5deb9d`

The two full captures used to define this P0 were:

- `runtime/captures/v3_20260923_132806_7777_capture.mcap` — EXPLORE control run; the shared upper/lower motion path remained active and moved normally.
- `runtime/captures/v3_20260923_133019_8838_capture.mcap` — FOLLOW_PERSON; target acquisition worked, but identity continuity and timer-driven SEARCH failed.

## Proven source/runtime mismatch fixed

### L4 identity lifecycle

Current source removed an active person after `person_track_max_age_ns=500 ms` and allocated future person IDs monotonically. L6 correctly refused to silently replace a locked target with another UID. Together, those correct local rules made same-person reacquisition impossible after active-track expiry.

Implementation: active/dormant split inside L4. Dormant state is private and checkpointed. It can reactivate the same UID only inside a bounded reacquisition window and association gate.

### L6 search dynamics

Current SEARCH changed target yaw every `400 ms` while the live mission maximum was `0.3 rad/s` and the sweep was `±0.45 rad`. The full capture showed requested omega repeatedly reversing while measured omega was still moving in the previous direction.

Implementation: search phase advances only after the current yaw target is reached within a bounded tolerance. The existing overall search timeout remains the fail-closed bound.

### Evidence gap

The production capture already contained `resolved_runtime`, but `test_hub_task_evidence.py` only searched legacy/top-level navigation config locations. It therefore reported capture-time FOLLOW config as unavailable.

Implementation: decode the existing resolved runtime path; do not duplicate configuration in capture. Add direct passive L6 follow evidence so target UID/state are observed facts instead of inferred state.

## Files touched by installer

- `v3/layers/l4_temporal_tracking.py`
- `v3/layers/l4_world_model.py`
- `v3/layers/l6_navigation.py`
- `v3/composition/native_control.py`
- `v3/test_hub_task_evidence.py`
- `conf/vezerles.json`
- `tests/test_v3_follow_person_modernization.py`
- `tests/test_v3_follow_person_p0_v2.py` (new)

No new numbered layer is introduced. L4 remains identity/world owner; L6 remains FOLLOW behavior/navigation owner; telemetry remains passive.
