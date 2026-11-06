# Mozgás-, Room Cruise- és Follow Person-hangolás

A production konfigurációt a `ConfigResolver` oldja fel a `conf/hardver.json`,
`conf/fizika.json`, `conf/speed_map.json` és `conf/vezerles.json` fájlokból.
A layer-, lifecycle- és safety-szabályok authorityja továbbra is a
[V3 contract](../STRUKTURALIS_RETEGEK_V3.md) és az
[async runtime contract](../ASZINKRON_RUNTIME_CONTRACT_V3.md).
Ez a lap a meglévő paraméterek használati térképe.

A feloldott értékek és az árnyékolt tartományok hardverindítás nélkül lekérhetők:

```bash
python -m v3.control_cli config
```

A `configuration_diagnostics` megkülönbözteti a fizikai kerékminimumot,
a tervezett kerékminimum-tartalékot, a planner transzlációs minimumát,
és a behavior kérése után ténylegesen rendelkezésre álló sebességtartományt.
A szimuláció konfigurációs overlayt kap; nem írja át a production fájlt.
A pontos parancsok és a mérési artifactok a
[szimuláció használati leírásában](ROBOT_MOTION_EXPERIMENTS.md#natív-konfigurációs-kísérletek)
szerepelnek.

## Hangolási térkép

Az alábbi utak a `conf/vezerles.json` fájlra vonatkoznak, kivéve ahol másik
fájl neve szerepel. Az `_ns` végződés nanoszekundumot jelent.

| Terület / owner | Elsődleges paraméterek | Hatás, korlát |
| --- | --- | --- |
| Egyenes- és ívkövetés / L8 | `layers.motion_realization.heading_gain`, `cross_track_gain`, `angular_velocity_gain`, `max_tracking_correction_rad_s` | Arányos heading-, térbeli oldalhiba- és yaw-rate-korrekció, közös korrekciós cap. A négy érték külön-külön lehet `0.0`. |
| Útpont-realizáció / L8 | `layers.motion_realization.cruise_v_mps`, `distance_gain`, `heading_stop_threshold_rad`, `max_requested_omega_rad_s` | A cruise és distance gain waypoint mozgást hangol; Room Cruise/FOLLOW rollout sebességét L6 választja. A heading stop threshold az egyenes/ív követésben is érvényes. |
| Sebesség és folytonosság / L9 | `layers.operational_constraints.max_v_mps`, `max_omega_rad_s`, `max_curvature_rad_per_m`, `max_acceleration_mps2`, `max_angular_acceleration_rad_s2` | Body capek, gyorsulás és kerékenkénti gyorsulási budget. A kerékpár fizikai realizálhatósága közösen korlátoz; a lassítás nem két független kerék-clamp. |
| Gyengébb lokalizáció / L9 | `layers.operational_constraints.degraded_velocity_scale`, `degraded_acceleration_scale` | Csökkentett cél és rámpa; a tényleges tartomány a kerékminimum és a hard capek metszete. |
| Kerék-visszacsatolás / L11 | `layers.wheel_pi.kp`, `ki`, `integrator_limit`, `max_normalized_output` | Kalibrált feed-forward melletti PI. `kp=0` / `ki=0` is támogatott. Control-grade encoder sebesség hiányában a PI csökkentett vagy kikapcsolt lehet; ebből önmagában nem következik rossz gain. |
| Fizikai kalibráció | `conf/fizika.json` nyomtáv és encoder lépéshossz; `conf/speed_map.json` görbék és continuous minimum | L3/L6/L8/L9/L10/L11 közös származtatott geometriája és keréktartománya. A viselkedési gain nem újrakalibrálás. |
| Tervezési minimum / L9 → L6 | `layers.operational_constraints.minimum_target_wheel_speed_mps`; `layers.navigation.minimum_planning_speed_mps` | Az első a mozgó kerekek célminimum-tartaléka, a második a planner transzlációs mintavételi minimuma. Egyik sem módosítja a fizikai continuous minimumot. |
| Room Cruise kérés / host → L5 | `behavior.roomcruise.max_v_mps`, `max_omega_rad_s`, `preferences.*` | Ugyanez a feloldott profil jut az operator/CLI és a canonical command útjára; a CLI/tool explicit override-ot adhat. |
| Room Cruise helyi cél / L6 | `behavior.roomcruise.preferences.local_goal_novelty_weight`, `local_goal_clearance_weight`, `local_goal_forward_weight`, `explore_local_goal_min_distance_m`, `explore_local_goal_max_distance_m`, `explore_local_goal_distance_weight`; `layers.navigation.explore_local_goal_distance_samples`, `local_goal_heading_samples` | A célpont kiválasztása: lefedettség, szabad hely, előrehaladás és céltávolság. A preferences értékeinek nincs második `layers.navigation` konfigurációs authorityja. |
| Helyi pálya / L6 | `layers.navigation.progress_weight`, `clearance_weight`, `smoothness_weight`, `novelty_weight`, `localization_observability_weight`, `progress_viability_floor`, `rollout_linear_samples`, `rollout_angular_samples`, `rollout_horizon_ns`, `rollout_step_count` | A kiválasztott helyi célhoz vezető pályák pontozása és mintavételi költsége. Nagyobb sample-budget több CPU-munkát is jelent. |
| Holtpont és újratervezés / L6 | `layers.navigation.local_goal_max_age_ns`, `local_goal_tolerance_m`, `trajectory_replan_interval_ns`, `local_escape_trigger_clearance_m`, `local_escape_reverse_max_v_mps` | Mért előrehaladás hiányának időablaka, célváltás/újratervezés és bounded escape paraméterei. Az ismételten kimerített holtpont HOLD lehet; kisebb timeout önmagában nem bizonyít jobb útvonalat. |
| Sebesség–clearance / L6 | `layers.navigation.speed_clearance_enabled`, `speed_clearance_low_speed_mps`, `speed_clearance_high_speed_mps`, `speed_clearance_low_m`, `speed_clearance_high_m`, `speed_clearance_barrier_power`, `speed_clearance_penalty_weight` | Soft navigation envelope és pontozás; a végső L12 safety ettől független. |
| Pályaváltás / L7 | `layers.motion_selection.continuity_score_band`, `continuity_clearance_drop_tolerance_m`, `reversal_min_omega_rad_s` | Közeli pontszámú pályák közti folytonosság. A tényleges gyorsulást L9 szabályozza. |
| Follow Person kérés / host → L5 | `behavior.followperson.max_v_mps`, `max_omega_rad_s` | Az operator, `r fp`, developer CLI és RobotInterface alapértelmezett kérése. Ugyanazok a mission/platform/fizikai capek szűkítik, mint más mozgást. |
| Follow Person távolság / L6 | `layers.navigation.follow_person_stand_off_m`, `follow_person_distance_deadband_m`, `follow_person_min_safe_distance_m`, `follow_person_hold_release_margin_m`, `follow_person_slowdown_distance_m` | Kívánt távolság, megállási/újraindulási hiszterézis, közelségi tiltás és lassítás. |
| Follow Person irány / L6 | `layers.navigation.follow_person_align_tolerance_rad`, `follow_person_release_tolerance_rad`, `follow_person_pivot_enter_rad`, `follow_person_heading_min_factor`, `follow_person_minimum_follow_speed_mps` | Követési ív, nagy irányhiba esetén pivot és a heading miatti lassítás. A minimum follow speed nem írja felül a távolság miatti fékezést vagy a planner/kerékminimumot. |
| Follow Person target/search / L6 | `layers.navigation.follow_person_min_confidence`, `follow_person_retention_min_confidence`, `follow_person_lost_hold_ns`, `follow_person_search_timeout_ns`, `follow_person_search_sweep_rad`, `follow_person_search_yaw_tolerance_rad`, `follow_person_search_max_duration_ns` | Acquisiton/retention, takarás utáni HOLD és időben bounded szögszegmenses keresés. `follow_person_search_step_ns` történeti kompatibilitási mező, nem aktív keresési ütem. |

## Árnyékolt tartományok és értelmezés

A kérés felső határát a command ingress fogadja el, a mission default cap
szűkítheti, az L9 platform envelope és a kalibrált keréktartomány pedig tovább
korlátozza. A nagyobb host vagy L8 cap nem emelheti át a kisebb downstream
határt. A freshness/watchdog beállítások nem mozgásminőségi gain-ek.

A production fizikai continuous minimum jelenleg `0.15 m/s`, a célminimum
`0.19 m/s`. A nyomtávból származtatott helyben fordulási fizikai minimum
kb. `0.8434 rad/s`, a tervezett minimum kb. `1.0683 rad/s`. A változatlan
Follow Person alap angular cap `0.30 rad/s`: ez helyben pivotra és kereső
fordulásra elégtelen. A diagnosztika ezt `PARTIAL_CAPABILITY` jelzéssel
közli; a konfigurálhatóvá tétel nem növeli automatikusan a sebességet.

A completion-input planner útban a `trajectory_replan_min_tick_gap`, valamint
az async `release_tick_gap` / `release_delay_ns` nem a production ütemezés
hangolói; ezt a feloldott diagnosztika külön jelzi. A történeti mezők a régi
capture/replay és más explicit módok miatt megmaradnak.

## Gain-kísérletek

A `heading_gain=0` a proporcionális heading feedbacket kapcsolja ki,
a `cross_track_gain=0` az oldalhiba heading-korrekcióját, az
`angular_velocity_gain=0` a mért yaw-rate visszacsatolást. A
`max_tracking_correction_rad_s=0` az egyenes/ív/pivot követés közös feedback
korrekcióját nullázza. A geometriai feed-forward, waypoint terminal-yaw
minimum, mission/kerékcapek, L9 és L12 ettől megmaradnak. Negatív vagy
nem véges értékek továbbra is hibásak.

Egy tényező változzon azonos scenario, seed és plant mellett. Először az
egyenes/aszimmetrikus egyenes, utána a Room Cruise holtpontok/útvonalak,
majd a FOLLOW távolság-, irány- és targetvesztési helyzetek hasonlíthatók
össze. A modellbeli tracking, megállási idő és útminőség nem igazolja a
fizikai robot pontosságát; a natív replay azonos lezárt input/config
determinisztikus eredményét ellenőrzi.
