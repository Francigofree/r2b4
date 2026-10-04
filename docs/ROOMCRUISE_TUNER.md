# RoomCruise tuner

`tools/tuners/r2b4_roomcruise_tuner.py` is an offline, non-actuating tuner for the current V3 RoomCruise/EXPLORE navigation path. The canonical human/agent entry point is `r tune roomcruise`.

## Authority boundary

The tool reads the canonical production configuration with `ConfigResolver` and runs candidate variants through `MissionNavigationComposition` (L5-L9). It never starts the resident runtime, opens hardware, writes `conf/vezerles.json`, or bypasses the canonical motor/safety path. The simulator is therefore a navigation-tuning aid, not a replacement for live MCAP/evidence acceptance.

The tuner deliberately does **not** vary `speed_map.json`, encoder reliability thresholds, wheel calibration, L10-L12 safety, or hardware configuration.

## Configuration authority

`conf/vezerles.json` → `behavior.roomcruise` owns the requested envelope and EXPLORE local-goal preferences. OperatorController resolves the profile and submits it explicitly through `v3.control_cli`; the developer CLI uses the same resolved profile when arguments are omitted. The tuner reads that profile, the same ingress acceptance policy, calibrated wheel limits and `runtime.tick_period_ns`. This document does not define tuning defaults.

`config_patch` separates `behavior.roomcruise` recommendations from shared `layers.navigation` execution recommendations. An emitted candidate config includes both and must pass ConfigResolver before it is written. Production config is never overwritten. Explicit baseline envelope options are experiments and are checked against production ingress acceptance.

`python -m v3.control_cli config` prints the resolved configuration, snapshot ID, derived centered-spin floor and inactive/unrealizable knob diagnostics without opening hardware. Status-sidecar output carries the same passive diagnostics. An unrealizable recovery speed is reported without changing the existing HOLD behavior.

The simulator closes EXPLORE preferences into the same CommandRequest/L5 mission contract. Its synchronous L5-L9 path shares production algorithms and physical limits, while worker timing, sensor dynamics and L10-L12 execution remain outside the simulation. It is not evidence of full live equivalence.

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

## Launcher usage

Quick pass:

```bash
r tune roomcruise
```

Full pass:

```bash
r tune roomcruise --profile full
```

By default every result is written below:

```text
runtime/tunes/roomcruise_tune_<timestamp>.json
```

Write a separate candidate `vezerles.json` beside the report:

```bash
r tune roomcruise --profile full --emit-config
```

This creates:

```text
runtime/tunes/roomcruise_tune_<timestamp>.vezerles.json
```

An explicit path can still be supplied:

```bash
r tune roomcruise --output runtime/tunes/manual.json \
  --emit-config runtime/tunes/manual.vezerles.json
```

The direct script entry point remains supported as a developer fallback:

```bash
python3 tools/tuners/r2b4_roomcruise_tuner.py --profile quick
```

## Validation

```bash
./r test tests/feature/test_v3_roomcruise_tuner.py
./r test roomcruise
r tune roomcruise --list-scenarios
```

## Acceptance

A tuner winner is only a simulation candidate. Before promoting values to production:

1. inspect behavior and execution changes within `config_patch` separately;
2. run the focused RoomCruise regression;
3. with explicit user authorization, run a short live RoomCruise session through the canonical operator path;
4. compile MCAP evidence;
5. compare curve/straight/reverse/stop/no-progress behavior and safety/localization evidence with the baseline.
