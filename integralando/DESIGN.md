# Fejlesztési terv és végrehajtott design

## 1. Kiinduló source contract

A production config resolver szigorúan a dataclass összes init mezőjét megköveteli,
ezért az új paraméterek ugyanabban a `conf/vezerles.json -> resolved config -> capture`
authority útban maradnak. Nincs runtime hardcode tuning.

A jelenlegi L6 54 trajectory candidate-et értékel, collision/progress viability után
progress + clearance + smoothness + novelty pontozással. Az L7 near-best score sávon
belül fizikai command continuityt preferál. Az L12 ettől független, közvetlen raw
LiDAR safety gate.

## 2. Új L6 soft-clearance policy

Új config mezők:

```json
"speed_clearance_enabled": true,
"speed_clearance_low_speed_mps": 0.2,
"speed_clearance_high_speed_mps": 0.4,
"speed_clearance_low_m": 0.18,
"speed_clearance_high_m": 0.3,
"speed_clearance_barrier_power": 2.0,
"speed_clearance_penalty_weight": 0.18
```

A kívánt clearance 0.2 és 0.4 m/s között lineárisan interpolál. A kívánt clearance
fölött nincs extra jutalom; alatta a score négyzetesen esik. 0.2 m/s fölött külön
sebességarányos penalty is nő. Reverse/pivot escape megőrzi a legacy lineáris
clearance scoringot.

Ez soft planner-policy, nem safety limit.

## 3. Trajectory score tuning

Régi:

```text
progress=0.60 clearance=0.10 smoothness=0.16 novelty=0.14
```

Új:

```text
progress=0.46 clearance=0.24 smoothness=0.22 novelty=0.08
```

A progress marad a legnagyobb pozitív komponens, de a clearance és a smoothness már
érdemben versenyez vele.

## 4. Lookahead

```text
rollout_horizon: 0.8 s -> 1.0 s
rollout_step_count: 8 -> 10
```

A térbeli/időbeli mintavételi lépés nagyságrendje így változatlan (~100 ms), miközben
0.3–0.4 m/s sebességnél hosszabb jövőbeli ív kerül ellenőrzésre.

## 5. EXPLORE-only adaptív lokális cél

A shared `local_goal_distance_m=0.6` változatlan, így NAVIGATE és FOLLOW nem kap
indokolatlan waypoint-hossz változást.

EXPLORE új tartomány:

```json
"explore_local_goal_min_distance_m": 0.45,
"explore_local_goal_max_distance_m": 0.9,
"explore_local_goal_distance_samples": 4,
"explore_local_goal_distance_weight": 0.15
```

Minden headinghez far-to-near 4 távolság vizsgálható. Nyílt térben a távolabbi,
folyamatosabb cél kap előnyt; ha az nem fér el, rövidebb lokális cél maradhat.

Goal score:

```text
novelty=0.40 clearance=0.35 forward=0.10 distance=0.15
```

## 6. L7 trajectory continuity

```json
"continuity_score_band": 0.02,
"continuity_clearance_drop_tolerance_m": 0.04
```

A near-best score sáv szélesebb, ezért kevesebb indokolatlan steering-váltás várható.
Viszont continuity csak olyan near-best jelölt között választhat, amely legfeljebb
4 cm-rel rosszabb minimum clearance-t ad a near-best család legtágasabb jelöltjénél.
Így a hiszterézis nem tarthat meg lényegesen szűkebb útvonalat.

## 7. Replay kompatibilitás

Régi resolved capture-ekben az új mezők nem léteznek. Replay migration ezért:

- speed-aware clearance-et kikapcsol;
- penalty weightet 0-ra állít;
- EXPLORE min/max goal distance-et a történeti `local_goal_distance_m` értékre
  összecsukja, 1 mintával;
- L7 clearance guard tolerance-t nagyon nagy értékre állítja.

Ez megőrzi a történeti policy-t, miközben új live capture már az új resolved configot
rögzíti.

## 8. Acceptance criteria

A fejlesztés célja nem egyetlen „PASS” szám, hanem egy együttes javulás:

- nyílt/átlagos szakaszon 0.20–0.40 m/s használható sebesség;
- safety-közeli normál navigációban L12 hard STOP ritka kivétel;
- selected candidate changes/s csökken a 18:31/18:35 baseline ~7/s értékéhez képest;
- minimum selected clearance alsó széle távolodik a ~0.15 m footprint-clearance
  tartománytól nagy sebességnél;
- angular jerk és steering sign-flip nem romolhat;
- planner p95/p99 timing nem romolhat lényegesen az 1.0 s / 10 step rollout miatt;
- nincs új `NO_PROGRESS_VIABLE_TRAJECTORY` vagy hosszú `PLANNER_STALE_HOLD` regresszió.
