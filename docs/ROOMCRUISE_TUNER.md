# RoomCruise tuner

`tools/tuners/r2b4_roomcruise_tuner.py` is an offline, non-actuating tuner for the current V3 RoomCruise/EXPLORE navigation path.

## Authority boundary

The tool reads the canonical production configuration with `ConfigResolver` and runs candidate variants through `MissionNavigationComposition` (L5-L9). It never starts the resident runtime, opens hardware, writes `conf/vezerles.json`, or bypasses the canonical motor/safety path. The simulator is therefore a navigation-tuning aid, not a replacement for live MCAP/evidence acceptance.

The tuner deliberately does **not** vary `speed_map.json`, encoder reliability thresholds, wheel calibration, L10-L12 safety, or hardware configuration.

## Why the command envelope is reported separately

The current operator RoomCruise path launches `v3.control_cli explore`, whose defaults are `max_v_mps=0.30` and `max_omega_rad_s=0.60`. Those values can cap the effective RoomCruise envelope below `vezerles.json`. They are therefore included in the search and report, but they are returned as `command_ingress_patch`; they are not silently written into `vezerles.json`.

## Search

The default search is deterministic coordinate descent. It tunes one bounded axis at a time and keeps the best current candidate, avoiding an exponential Cartesian sweep. The main axes are:

- RoomCruise command max linear/angular speed;
- `minimum_planning_speed_mps`;
- rollout shape (`6x9` vs `5x11`, both inside the production 30-60 candidate contract);
- degraded localization speed scale;
- EXPLORE local-goal forward/novelty/clearance weights;
- trajectory smoothness/novelty weights;
- localization observability weight.

Each candidate runs the same deterministic living-room-like scenarios. Metrics include collision, moving/curved/straight/pivot/reverse/stop ratios, heading diversity, coverage, physical clearance, no-progress events, and steering reversals per metre.

## Usage

Quick pass:

```bash
python3 tools/tuners/r2b4_roomcruise_tuner.py --profile quick
```

More thorough pass:

```bash
python3 tools/tuners/r2b4_roomcruise_tuner.py --profile full --output /tmp/roomcruise_full.json
```

Write a **separate candidate config** without touching production config:

```bash
python3 tools/tuners/r2b4_roomcruise_tuner.py \
  --profile full \
  --emit-config /tmp/vezerles.roomcruise-tuned.json
```

Focused validation:

```bash
python3 -m pytest -q tests/feature/test_v3_roomcruise_tuner.py
./r test roomcruise
```

## Acceptance

A tuner winner is only a simulation candidate. Before promoting values to production:

1. inspect `config_patch` and `command_ingress_patch` separately;
2. run the focused RoomCruise regression;
3. run a short live RoomCruise session through the canonical operator path;
4. compile MCAP evidence;
5. compare curve/straight/reverse/stop/no-progress behavior and safety/localization evidence with the baseline.
