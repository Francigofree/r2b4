# R2B4 Async Capability Convergence — P0 + P1 + P2

Base HEAD: `378d42b98ce5f5d893561f007698eb7d768d9e4a`

## P0
- stale cached L6 plan + replacement pending -> `PLANNER_STALE_HOLD`
- zero-motion hold, fresh completion után resume
- deadline/worker/context/stale-completed-result továbbra is fail-closed

## P1
- `v3/async_capability.py`
- `LATEST_STATE`, `REQUEST_RESULT`, `EVIDENCE_STREAM`
- LiDAR + vision capability snapshots
- planner request/result capability evidence
- capture explicit evidence-stream
- async contract kiegészítés

## P2
- `worker_generation + request_id + source_context`
- late/abandoned result rejection
- lifecycle state vocabulary
- supersede/error/late-result counters
- restart-safe identity foundation

Automatikus restart nincs bekapcsolva; worker-halál továbbra is fail-closed.

## Installer policy
Az installer szándékosan nem végez Git/head/dirty-tree ellenőrzést, tesztet,
gate-et, preflightot, backupot vagy rollbacket. Csak módosít.

```bash
cd /home/alba/project_r2b4
python3 /PATH/r2b4_async_capability_convergence_p0_p1_p2_20260921/installer.py
```
