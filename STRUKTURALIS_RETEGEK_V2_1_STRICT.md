# R2B4 szigorú réteg- és contract baseline V2.1

**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V2_1`
**Szerep:** normatív architektúra-SSOT. Nem eseménynapló.
**Forrásalap:** 2026-07-30 source snapshot + jelenlegi `STRUKTURALIS_RETEGEK.md` + `AGENTS.md`.
**Állapot:** célarchitektúra; a legacy source eltérései `CONTRACT_VIOLATION`-ök, nem precedensek.

## 0. Értelmezési szabály

* **MUST** = kötelező; **MUST NOT** = tilos; **MAY** = megengedett.
* Source/config = **runtime truth**: megmondja, mi történik most.
* Ez a dokumentum = **architecture truth**: megmondja, mi megengedett.
* Source és contract ütközésekor az agent MUST a source-t `CONTRACT_VIOLATION`-ként kezelni.
* Az agent MUST NOT a contractot a legacy implementációhoz igazítani.
* Contract-módosítás csak külön architekturális döntés + explicit felhasználói jóváhagyás.
* **LAYER != CLASS != FILE != PROCESS.** A logical layer ownership-határ. Új osztály/fájl/processz/absztrakció csak akkor MAY létrejönni, ha szükséges egy contract-határ kikényszerítéséhez vagy bizonyíthatóan csökkenti a couplingot. A minimális implementáció kötelező.

## 1. Nem alkuképes invariánsok

`C001` Egy normál motion út van: `MotionProposal -> IntentResolver -> ResolvedMotionIntent -> MotionGuidance -> PhysicalMotionCommand -> MotionEnvelope -> MotionController -> MotionExecutor -> SafetyGate -> MotorHAL`.

`C002` Egy control tick = egy kiválasztott motion command + egy EKF pose + egy final bal/jobb motoroutput.

`C003` A publikus pose egyetlen ownere az EKF, frame: `R2B4_BOOT_ROBOT_MAP`, yaw: `CCW_POSITIVE_LEFT`.

`C004` Normál wheel target -> PWM átalakítás egyetlen ownere a `MotionExecutor`; speed-map egyszer, wheel PI egyszer.

`C005` Raw LiDAR obstacle truth független a matchertől, térképtől, plannertől és localization confidence-tól.

`C006` Réteghatáron teljes `ctrl`/`AlbaController`, globális mutable runtime objektum vagy tetszőleges közös control-dict MUST NOT átmenni.

`C007` Diagnosztikai/status felület MUST NOT rejtett control-buszként működni.

`C008` Hiányzó kötelező contractmezőnél nincs implicit fallback; fail-closed vagy explicit invalid eredmény kell.

`C009` Az `IntentResolver` után kizárólag a kiválasztott intent végrehajtásához szükséges `MotionGuidance` MAY használni pose/heading/world-model feedbacket és módosítani a fizikai v/omega vagy wheel-target parancsot. `PhysicalMotionCommand` létrejötte után nincs új planning, trajectory-választás, behavior-special-case vagy obstacle-avoidance döntés.

`C010` Felső réteg nem ír PWM-et és nem ismeri a speed-map/PI belső állapotát.

## 2. Kanonikus adatfolyam

```text
SENSOR:
Driver -> Measurement Snapshot -> Measurement Trust -> EKF -> PoseSnapshot
   |                 |                |
   |                 +--------------> RollingLocalMap
   +--------------------------------> Raw Safety truth

MOTION:
Behavior / Follow / RoomCruise / AMR
        -> Navigation / proposal production
        -> MotionProposal(s)
        -> IntentResolver
        -> ResolvedMotionIntent
        -> MotionGuidance / GlobalMotionPolicy
        -> PhysicalMotionCommand
        -> OperationalMotionEnvelope
        -> MotionController
        -> WheelVelocitySetpoint
        -> MotionExecutor
        -> CandidateMotorOutput
        -> SafetyGate
        -> FinalMotorOutput
        -> MotorHAL
```

`cont.py` a lánc **orchestratora**, nem motion-réteg.

## 3. Rétegek

### L0 Hardware / MotorHAL

**Input:** `FinalMotorOutput`; hardverkonfiguráció.
**Output:** fizikai motorjel; nyers szenzorminta.
**MAY:** hardware clamp/fault, shutdown-zero.
**MUST NOT:** planner, EKF, speed-map, PI, arbitration, behavior.
Normál nemnulla PWM csak a SafetyGate final outputból jöhet.

### L1 Measurement Acquisition

**Owner:** encoder/IMU/LiDAR service/driver.
**Output kötelező mag:** `measurement_id`, `source_timestamp_monotonic`, `source_id`, `validity`, értékek.
**MUST:** eredeti timestamp + lineage megőrzése; poll nem új observation.
**MUST NOT:** motion target/PWM/pose ownership.

### L2 Measurement Trust + State Estimation

**L2A Measurement Trust:** encoder reliability, LiDAR admission, freshness/lineage gate. Output: validity/confidence/freshness; motiont nem hozhat létre.
**L2B EKF:** IMU predict + gated encoder + gated LiDAR -> `PoseSnapshot`. Az EKF nem planner és nem Safety.

`PoseSnapshot` mag:

```text
frame_id, pose_id, source_timestamp, x_m, y_m, yaw_rad, v_mps, omega_rad_s, validity
```

### L3 World Model

**Owner:** RollingLocalMap és navigációs világmodell.
**Input:** canonical pose + időazonos measurement.
**Output:** immutable navigation snapshot, upstream source ID/timestamp megőrzéssel.
**MUST NOT:** EKF/matcher/raw-Safety truth felülírása; motorvezérlés.

### L4 Behavior / Mission

**Owner:** Follow, Room Cruise mission, waypoint mission, GUI/API szemantikai parancs.
**Output:** behavior intent (`GO_TO_POSE`, `FOLLOW_TARGET`, `ROOM_CRUISE`, `ROTATE_TO_HEADING`, `STOP`, ...).
**MUST NOT:** PWM, speed-map, PI, executor-state, hard-safety override.

### L5 Navigation / Proposal Production

Ez a réteg készíti elő a lehetséges motion intenteket, de **nem ő dönti el, melyik nyer**.

**Owner:** `LocalPlanner`, `LocalNavigationLayer`, obstacle avoidance, trajectory/path geometry, waypoint/local-path kiválasztás, valamint minden olyan policy, amely több lehetséges motion intentet állít elő a Resolver számára.

**Input:** behavior intent + `PoseSnapshot` + world model + soft clearance + read-only `DriveCapabilities`.

**Output:** `MotionProposal`.

**MAY:** trajectory/curvature/v/omega/pivot/arc/reverse geometria választása; több egymással versengő proposal létrehozása; proposal priority/validity kitöltése.

**MUST NOT:** PWM, speed-map, PI, Safety bypass, Resolver selection ownership.

### L6 MotionProposal + IntentResolver

Felső rendszer normál mozgást csak `MotionProposal` porton kérhet.

`MotionProposal`:

```text
contract_id
proposal_id
cycle_id
valid_until_monotonic
priority
active
nominal_mode = BODY_TWIST | WHEEL_VELOCITY | STOP
nominal_payload
guidance_request
trace_metadata
```

`nominal_payload`:

```text
BODY_TWIST:      v_mps, omega_rad_s
WHEEL_VELOCITY: left_mps, right_mps
STOP:           nincs nemnulla target
```

`guidance_request` opcionális, tipizált felső motion-szemantika, amelyet kizárólag L7A fogyaszthat. Példák:

```text
NONE
HEADING_HOLD
TURN_TO_HEADING
TRACK_LOCAL_SEGMENT
```

A `guidance_request` MUST csak a kiválasztott intent végrehajtásához szükséges referenciaadatot tartalmazni. `RoomCruise`, `Follow`, M-szint vagy producer-név MAY trace metadata lenni, de downstream physical control branch feltétel nem lehet.

`trace_metadata` MAY tartalmazni `source/behavior/command_type/planner_reason` adatot; L8/L9 számára ez **csak diagnosztika**, control input nem lehet.

`IntentResolver` input: `MotionProposal[]`; output pontosan egy `ResolvedMotionIntent`:

```text
contract_id
resolved_id
cycle_id
selected_proposal_id
valid_until_monotonic
nominal_mode
nominal_payload
guidance_request
trace_metadata
```

Resolver MAY választani, validálni és priorizálni. MUST NOT headinget zárthurkúan korrigálni, trajectory-t újratervezni, obstacle-side döntést hozni, speed-mapet vagy PWM-et számítani.

A Resolver ownershipja kizárólag:

```text
MELYIK intent nyer ebben a tickben?
```

### L7 Motion Guidance + Operational Motion Envelope

#### L7A Motion Guidance – post-resolution closed-loop guidance

Ez az **egyetlen** Resolver utáni réteg, amely még ismerheti a kiválasztott motion intent jelentését és használhat pose/heading/world-model feedbacket annak fizikai végrehajtásához.

**Input:**

```text
ResolvedMotionIntent
PoseSnapshot
szükséges navigation/world-model snapshot
read-only DriveCapabilities
CycleContext
```

**Output:** pontosan egy `PhysicalMotionCommand`:

```text
contract_id
physical_command_id
resolved_id
cycle_id
valid_until_monotonic
physical_mode = BODY_TWIST | WHEEL_VELOCITY | STOP
payload
guidance_reason
trace_metadata
```

Payload:

```text
BODY_TWIST:      v_mps, omega_rad_s
WHEEL_VELOCITY: left_mps, right_mps
STOP:           nincs nemnulla target
```

**MAY kizárólag:** heading-turn; straight heading hold; selected local-segment/trajectory követése; pose/heading/omega feedback alapján corrected v/omega vagy wheel-target előállítása; a kiválasztott intent fizikai végrehajtásához szükséges zárthurkú korrekció.

**MUST:**

* csak a Resolver által kiválasztott `ResolvedMotionIntent`-et hajthatja végre;
* control-rate vagy explicit meghatározott feedback-rate mellett MAY futni;
* fresh/valid `PoseSnapshot`/world-model adatot használni;
* a kimeneten minden felső szemantikát `PhysicalMotionCommand`-dá redukálni.

**MUST NOT:**

* másik proposalra váltani;
* új producer-prioritást/arbitrationt végezni;
* új behavior intentet létrehozni;
* PWM-et, speed-mapet vagy wheel PI-t számítani;
* Safety authority-t felülírni.

`MotionController` és `MotionExecutor` EKF headinget, trajectory-t, `guidance_request`-et vagy GlobalMotionPolicy belső state-et MUST NOT látni.

**`controller/motion_policy.py` cél-ownership:** a fájl jelenleg vegyes felelősséget tartalmazhat. A proposal/trajectory **jelöltképzés** L5; a Resolver által kiválasztott intent fizikai closed-loop shapingje/korrekciója L7A. Egy függvény pontosan egy ownershiphoz tartozhat; a kettő nem keverhető ugyanazon control pathban.

#### L7B Operational Motion Envelope

**Owner:** runtime speed limits + localization gate + fizikai drive capability.

**Input:** `PhysicalMotionCommand` + aktuális constraint/health állapot.

**Output:**

```text
MotionEnvelope:
cycle_id
physical_command_id
stop_required, stop_reason
max_abs_v_mps, max_abs_omega_rad_s, max_abs_wheel_mps
max_wheel_accel_mps2, max_wheel_decel_mps2
capability_version
```

MUST csak szűkíteni; MUST NOT nemnulla mozgást létrehozni, előjelet megfordítani, trajectory-t választani, headinget korrigálni vagy source-specifikus kivételt létrehozni.

Localization limitet downstream minimum-speed floor MUST NOT visszagyorsítani.

`PhysicalMotionCommand` létrejötte után annak fizikai jelentése immutable. Downstream csak limitálhat, lassíthat, nullázhat vagy `UNREPRESENTABLE`-ként elutasíthat; új trajectory-t vagy guidance-correctiont nem hozhat létre.

### L8 MotionController – lezárt fizikai platformhatár

**Egyetlen feladat:**

```text
PhysicalMotionCommand + MotionEnvelope + DriveCapabilities
    -> WheelVelocitySetpoint
```

Public input:

```text
cycle_context {
  cycle_id,
  monotonic_time,
  dt_observed_s,
  dt_control_s,
  timing_valid,
  timing_reason
}
physical_command
motion_envelope
drive_capabilities {track_width_m, calibrated wheel range, accel/decel capability}
```

Output:

```text
WheelVelocitySetpoint:
contract_id, cycle_id, left_target_mps, right_target_mps,
feasible, reason, applied_limits
```

**MAY kizárólag:** BODY_TWIST -> diff-drive wheel kinematika; fizikai max/rate limit; track target capability-be illesztése; stop/nulla; diagnosztika.

**MUST:** normál executed twist->wheel konverziót pontosan egyszer végezni; nullából nullát adni; invalid inputnál fail-closed; limitet explicit jelezni.

**MUST NOT látni/olvasni:**

```text
ctrl / AlbaController
command source/layer/type
Room Cruise / Follow / M3/M4/M5
LocalPlanner / GlobalMotionPolicy internals
motion_resolution_status
EKF pose/heading
LiDAR / map / obstacle clearance
Safety state
PWM / speed-map internals / wheel PI state
```

Public MotionController API-ban `ctrl`/teljes controller paraméter tilos. `getattr(ctrl,...)` runtime config tilos.

**Wheel-speed floor:** L8-ban rejtett per-wheel minimum speed/floor, amely megváltoztatja a kért geometriát, tilos. Ha a request a kalibrált tartományban geometry-tartó közös skálázással sem reprezentálható: `feasible=false` + `0/0`. Controller nem alakíthat egyenest kanyarrá, ARC-ot más ARC-cá vagy pivotot más motion-classzá.

**Nem ugyanaz, mint az actuator deadband compensation:** a motor fizikai PWM-holtzónájának `startup/maintenance actuator floor` kompenzációja L9 MotionExecutor ownership és kifejezetten engedélyezett. Ez MUST NOT nullából mozgást létrehozni, wheel targetet megváltoztatni vagy felső geometriát újradefiniálni.

### L9 MotionExecutor – wheel-only actuator controller

**Egyetlen feladat:**

```text
WheelVelocitySetpoint + WheelFeedback -> CandidateMotorOutput
```

Input:

```text
cycle_context
wheel_setpoint
WheelFeedback {
  measurement_id, source_timestamp,
  left_mps, right_mps,
  combined_trust, timing_valid, stale
}
```

Output:

```text
CandidateMotorOutput {
  contract_id, cycle_id,
  left_pwm, right_pwm,
  output_reason, wheel_control_diagnostics
}
```

**Egyetlen owner:** négyirányú speed-map lookup; wheel feed-forward; canonical wheel PI; startup/maintenance actuator floor; actuator direction-switch deadtime; PWM clamp.

**MUST:** speed-map egyszer; PI egyszer; zero wheel target -> same-tick `0/0`; invalid/stale/timing-invalid kötelező feedback -> fail-closed `0/0`; explicit PI reset/hold; final candidate clamp.

**Actuator deadband compensation:** L9-ben MAY és kizárólag itt. Csak nemnulla wheel target mellett alkalmazható; előjeltartó; calibrated/immutable speed-map vagy actuator config alapján működik; zero targetből MUST NOT nemnulla PWM-et létrehozni; a `WheelVelocitySetpoint` értékét MUST NOT átírni; behavior/command-type alapján MUST NOT változni.

**MUST NOT látni/olvasni:**

```text
v/omega semantic command
ARC_EXEC / HEADING_EXEC
Room Cruise / Follow / LocalPlanner / GlobalMotionPolicy
command source/layer/type
turn primitive
EKF pose/heading/omega
LiDAR/confidence/map/localization confidence
Safety obstacle state
planner clearance / behavior state
```

**MUST NOT:** straight-heading hold; trajectory/curvature policy; LocalPlanner pivot special-case; Follow/RoomCruise/source-specific guard.

**Determinism:** compute path alatt saját `time.*`, filesystem, global config, hálózat vagy thread tilos. Idő a `cycle_context`-ból, config explicit immutable snapshotból jön. A PI és direction-switch timing kizárólag `dt_control_s`/`monotonic_time` alapján működhet.

### L10 Safety Authority

**L10A SafetySupervisor:** raw LiDAR safety + health + localization safety + fizikai motion direction/magnitude -> `SafetyDecision`. Külön observationt számol, nem control ticket. Nem planner; alternatív motiont nem hozhat létre.

**L10B SafetyGate:** utolsó software output filter a MotorHAL előtt.

```text
CandidateMotorOutput + SafetyDecision + RawSafetySnapshot
    -> FinalMotorOutput
```

Safety block -> same-tick `0/0`. Braking csak csökkenthet; új steering commandot nem hozhat létre. Kötelező invariáns:

```text
abs(final_left_pwm)  <= abs(candidate_left_pwm)
abs(final_right_pwm) <= abs(candidate_right_pwm)
zero candidate -> zero final
```

### L11 Diagnostics / QA / Replayer

**Owner:** `MotionQAMonitor`, telemetry/status, Test Hub, Replayer.
MAY megfigyelni/tárolni contract ID/cycle ID/lineage-et.
MUST NOT ugyanabban a tickben control outputot visszaírni vagy PASS érdekében runtime thresholdot módosítani.

### L12 `cont.py` – Composition Root / Orchestrator

`cont.py` **nem motion layer**.

MAY: komponensek létrehozása/injection; control tick sorrend; immutable contractok továbbítása; lifecycle; diagnostics; final SafetyGate output -> MotorHAL.

Kívánt minta:

```text
proposals  = producers.tick(...)
resolved   = resolver.resolve(proposals)
physical   = motion_guidance.compute(resolved, pose, world, ...)
envelope   = constraints.build(physical, ...)
wheel_ref  = motion_controller.compute(physical, envelope, ...)
candidate  = motion_executor.compute(wheel_ref, feedback, ...)
final_pwm  = safety_gate.filter(candidate, safety, raw_safety)
motor_hal.apply(final_pwm)
```

MUST NOT saját v/omega policy-t, curvature/side/heading-hold döntést, wheel minimumot, primitive special-case-et, speed-mapet vagy PI-t számítani. Executor számára felső semantic mezőket tartalmazó ad-hoc `sensor_feedback` control-dict építése tilos.

## 4. `motion_readiness.py` – kötelező ownership-szétválasztás

A jelenlegi fájl nem egy réteg; öt ownershipot kever:

| Jelenlegi osztály         | V2.1 ownership                                                          |
| ------------------------- | ----------------------------------------------------------------------- |
| `EncoderReliabilityLayer` | L2A Measurement Trust                                                   |
| `HeadingTurnController`   | L7A Motion Guidance                                                     |
| `MotionSemanticsEngine`   | L7A Guidance, illetve ha proposal-képzést végez, az a rész L5 ownership |
| `MotionQAMonitor`         | L11 Diagnostics                                                         |
| `BehaviorMotionInterface` | L4 Behavior/API adapter                                                 |

Átmenetileg egy fájlban maradhatnak, de a V2.1 **SEALED** állapot előtt külön modulba MUST kerülniük, hogy statikus dependency-contracttal ellenőrizhetők legyenek.

## 5. Alsó MotionPlatform publikus határa

Felső AMR/Behavior MUST NOT közvetlenül MotionControllerhez vagy MotionExecutorhoz kapcsolódni.

**Publikus belépés:** `MotionProposal`.
**Publikus read-only visszajelzés:** `MotionPlatformStatus`:

```text
cycle_id, resolved_id, physical_command_id, accepted_physical_mode,
requested/executed wheel targets,
candidate_pwm, final_pwm,
controller_reason, executor_reason, safety_reason,
measurement_validity
```

Felső réteg MAY status alapján új következő-tick proposal-t készíteni; belső PI/speed-map state-be nem nyúlhat.

## 6. Alsó platform által ismert fizikai módok

Kizárólag:

```text
BODY_TWIST
WHEEL_VELOCITY
STOP
```

`ARC_EXEC`, `HEADING_EXEC`, `ROOM_CRUISE`, `FOLLOW`, `LOCAL_PLANNER_SEGMENT`, `ROTATE_TO_HEADING`, `STRAIGHT_HOLD` felső szemantika; MotionController/Executor határt nem léphetik át.

Példák:

```text
ROTATE_TO_HEADING -> MotionProposal -> IntentResolver -> Guidance + EKF -> PhysicalMotionCommand -> platform
ARC               -> Planner MotionProposal -> IntentResolver -> Guidance -> PhysicalMotionCommand -> platform
STRAIGHT_HOLD     -> MotionProposal -> IntentResolver -> Guidance + EKF -> PhysicalMotionCommand -> platform
```

## 7. Config- és state-ownership

Control-releváns confignak egy SSOT-ja van; Controller/Executor konstrukciókor validált immutable config snapshotot kap. Tick közbeni `global_config.get`, filesystem reload vagy `ctrl.cfg`-olvasás tilos.

| Mutable state              | Egyetlen owner        |
| -------------------------- | --------------------- |
| EKF pose                   | EKF                   |
| RollingLocalMap            | RollingLocalMap       |
| proposal lifecycle         | producer              |
| resolver selection         | IntentResolver        |
| guidance closed-loop state | MotionGuidance        |
| slew/rate state            | MotionController      |
| wheel PI integrator        | MotionExecutor        |
| speed-map snapshot         | MotionExecutor config |
| safety hysteresis          | SafetySupervisor      |
| final filter state         | SafetyGate            |
| history/evidence           | Diagnostics/Replayer  |

Más layer az owner belső state-jét közvetlenül MUST NOT módosítani.

## 8. Timing / lineage

Control contract kötelező:

```text
cycle_id
monotonic_time
dt_observed_s
dt_control_s
timing_valid
timing_reason
```

`dt_observed_s` = az orchestrator által monotonic clockból mért tényleges tick-idő.
`dt_control_s` = a runtime timing-policy által validált, szükség esetén explicit clampelt szabályozási idő. A clamp/validáció határa config-SSOT.
Névleges/fix `dt` MUST NOT helyettesítheti a tényleges mért időt normál runtime-ban.
`timing_valid=false` esetén a motion pipeline MUST fail-closed módon viselkedni az adott contract szerint.

Measurement contract kötelező: `measurement_id`, `source_timestamp`, freshness/validity.
Poll timestamp nem measurement timestamp.
Controller/Executor saját alternatív control-időt nem hozhat létre. Replay MUST ugyanazt a rögzített `CycleContext`-et adni a komponenseknek.

## 9. Fail-closed szemantika

| Hiba                                                   | Kötelező eredmény                                 |
| ------------------------------------------------------ | ------------------------------------------------- |
| invalid/stale proposal                                 | Resolver reject                                   |
| nincs valid proposal                                   | STOP                                              |
| stale `ResolvedMotionIntent`                           | STOP                                              |
| MotionGuidance invalid/stale input vagy invalid output | `PhysicalMotionCommand=STOP`                      |
| invalid `CycleContext` / `timing_valid=false`          | downstream fail-closed `0/0`                      |
| envelope `stop_required`                               | Controller `0/0`                                  |
| Controller invalid/unrepresentable input               | `feasible=false`, `0/0`                           |
| wheel feedback invalid/stale/timing invalid            | Executor `0/0`, explicit reason                   |
| speed-map invalid                                      | normal runtime `0/0`, fallback tiltott            |
| Safety block                                           | final same-tick `0/0`                             |
| pipeline exception                                     | final `0/0`; korábbi nemnulla PWM nem ismételhető |

## 10. Service/calibration út

Direct PWM nem normál motion. Külön `ServiceActuationRequest` kell:

```text
armed_token, expiry, left_pwm, right_pwm,
max_abs_pwm, distance/time bound, reason
```

MUST: explicit arm, timeout, hard cap, distance bound, final SafetyGate, session evidence.
MUST NOT: normál MotionProposalnak álcázni, normál motionnal együtt aktív lenni, speed-map/PI state-et módosítani.

## 11. Statikus contract-tesztek – V2.1 lezáráshoz kötelező

`T001` MotionController/Executor public API-ban `ctrl`, `controller`, `AlbaController` paraméter nincs.
`T002` AST forbidden-import teszt a fenti layer-dependencykre.
`T003` Controller/Executor control branch-ben nincs `command_type`, `command_layer`, `motion_source`, `room_cruise`, `follow`, `local_planner`, `M3/M4/M5`, `turn_primitive`, `heading_exec`, `arc_exec`.
`T004` Normal speed-map lookup egyetlen owner: MotionExecutor.
`T005` Normal wheel PI egyetlen owner: MotionExecutor.
`T006` `PhysicalMotionCommand.BODY_TWIST` -> wheel conversion egyetlen owner: MotionController.
`T007` Zero command -> zero wheel -> zero candidate PWM -> zero final PWM.
`T008` Safety monotonicity: final abs PWM nem nőhet.
`T009` Azonos config + input sequence -> determinisztikus Controller/Executor output.
`T010` Controller/Executor compute path nem olvas fájlt/global configot/saját clockot.
`T011` `IntentResolver` után kizárólag L7A MotionGuidance MAY pose/heading/world-model feedback alapján módosítani a kiválasztott intent fizikai végrehajtását; `PhysicalMotionCommand` után nincs planning/trajectory/obstacle-side/behavior special-case.
`T012` Minden nonzero final motoroutput lineage: `proposal_id -> resolved_id -> physical_command_id -> wheel_setpoint -> executor_output -> safety_output`.

## 12. Jelenlegi fájlok cél-ownershipja

| Fájl                              | V2.1 szerep                                         | Szükséges változás                                                                                                    |
| --------------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `controller/motion_controller.py` | L8                                                  | név marad; teljesen szűk physical contract                                                                            |
| `motion_executor.py`              | L9                                                  | név marad; wheel-only executor                                                                                        |
| `controller/motion_policy.py`     | L5 és/vagy L7A, függvényenként egyértelmű ownership | proposal/trajectory jelöltképzés L5; post-resolution closed-loop shaping L7A; shared `ctrl` dependency megszüntetendő |
| `controller/motion_readiness.py`  | több layer keveréke                                 | L2A/L4/L7A/L11 ownership szerint logikai, seal előtt fizikai split                                                    |
| `controller/local_planner.py`     | L5                                                  | réteg marad                                                                                                           |
| `cont.py`                         | L12 orchestration                                   | motion döntési logika kiveendő                                                                                        |
| `controller/motion_resolver.py`   | L6 IntentResolver                                   | selection/arbitration ownership marad; output `ResolvedMotionIntent`                                                  |
| `safety_gate.py`                  | L10B                                                | final filter marad                                                                                                    |
| `safety/safety_supervisor.py`     | L10A                                                | safety authority marad; shared-controller dependency később szűkítendő                                                |
| `core/control_strategies.py`      | L9 belső wheel control                              | csak Executor ownership alatt                                                                                         |
| `middleware/ffp.py`               | L9 speed-map/feed-forward                           | csak Executor ownership alatt                                                                                         |

## 13. Jelenleg forrásból bizonyított contract-sértések

`V001` `controller/motion_controller.py`: `_runtime_limits(ctrl)` ~102; `active_motion_command_type` ~400; `motion_resolution_status` ~529; LocalPlanner/follow/reverse semantic branch ~536–590. **Violation:** L8 felső szemantikát és shared controllert olvas.

`V002` `motion_executor.py`: EKF heading straight-hold ~214; `active_command_type` ~435; heading special-case ~468; `local_planner_segment` ~578; command-type guard ~694. **Violation:** L9 nem wheel-only.

`V003` `cont.py`: Resolver ~2097; MotionSemantics ~2305; GlobalMotionPolicy apply ~2501; Controller calls ~2759/2772; semantic `sensor_feedback` ~2969/~3033; Executor call ~3114. **Violation:** a post-resolution guidance/policy és az orchestration közös `ctrl`/ad-hoc state-en keresztül keveredik. A Resolver utáni Guidance önmagában nem violation; annak ownershipját külön L7A komponensbe kell zárni.

`V004` `controller/motion_readiness.py`: `EncoderReliabilityLayer` ~154; `HeadingTurnController` ~1309; `MotionSemanticsEngine` ~2219; `MotionQAMonitor` ~2634; `BehaviorMotionInterface` ~2878. **Violation:** L2A/L4/L7A/L11 ownership keveredik egy modulban és shared runtime state-en.

`V005` `controller/motion_policy.py`: `select_local_trajectory` ~1552; `build_context(..., ctrl, ...)` shared runtime input; `GlobalMotionPolicy.apply()` post-resolution használat. **Violation:** L5 proposal/trajectory jelöltképzés és L7A post-resolution guidance ugyanazon shared runtime felületen érhető el. Függvényenként explicit ownership + typed input szükséges; fizikai fájlsplit csak akkor kötelező, ha ezt statikus dependency-contract másképp nem tudja kikényszeríteni.

## 14. Megőrzendő jelenlegi döntések

A V2.1 önmagában MUST NOT újranyitni:

```text
UNIFIED control mode
EKF pose SSOT
R2B4_BOOT_ROBOT_MAP
raw LiDAR Safety függetlenség
latest-only külön scan-matcher processz
single-slot matcher IPC
egy közös négyirányú wheel speed-map
egy canonical wheel PI
final SafetyGate közvetlenül motoralkalmazás előtt
armed + bounded calibration path
Test Hub mint live validation SSOT
```

## 15. MotionPlatform SEALED kritérium

A platform csak akkor `SEALED`, ha:

1. MotionController API-ban nincs shared controller/global runtime dependency.
2. MotionExecutor API wheel-only.
3. Executor nem lát EKF/LiDAR/planner/behavior semantic adatot.
4. IntentResolver után csak L7A MotionGuidance használhat pose/heading/world-model feedbacket a kiválasztott intent fizikai végrehajtásához; `PhysicalMotionCommand` után nincs planner/policy/semantic branch.
5. `cont.py` csak orchestrál.
6. Controller/Executor determinisztikusan replayelhető.
7. `T001–T012` PASS.
8. Teljes offline regresszió PASS.
9. Replayer contract-validáció PASS.
10. M0 PASS, M1 PASS, M3 PASS.
11. M5 live futásban nincs ownership/command-chain contradiction.
12. Minden stop után bizonyított final `PWM=0/0`.

SEALED után új AMR/Follow/RoomCruise logika csak `MotionProposal` contracton keresztül csatlakozhat; MotionController/Executor módosítása ehhez normál esetben nem szükséges.

## 16. Agent-kötelezettség motion-refaktornál

Kötelező sorrend:

```text
bootstrap/infra guard
-> ezt a V2.1 contractot normatív célként olvasni
-> source-t runtime truthként olvasni
-> eltérések = CONTRACT_VIOLATION
-> legacyhoz contractot nem igazítani
-> ownership szerint implementálni
-> statikus contract-teszt
-> célzott regresszió
-> teljes offline regresszió közös contract-változásnál
-> replay
-> csak ezután bounded live Test Hub
```

Legacy semantic mező új API-ba csak azért, mert a régi kód használja, MUST NOT átkerülni.

Minden új MotionController/Executor mezőre kötelező kérdés:

```text
Ki az owner?
Miért fizikailag ezen a rétegen szükséges?
Mi romlik el, ha az alsó réteg nem látja?
```

Ha a válasz `Follow`, `Room Cruise`, planner phase, command source, turn primitive, M-szint, EKF heading vagy LiDAR jelentés, a mező az alsó contractban tilos.

## 17. Egy mondatos ownership-definíciók

```text
Felső rendszer:       HOVÁ és MILYEN NOMINÁLIS GEOMETRIÁVAL akar mozogni.
IntentResolver:       MELYIK motion intent nyer ebben a tickben.
MotionGuidance:       HOGYAN kövesse fizikailag a kiválasztott intentet a friss pose/heading/world feedback alapján.
MotionController:     MILYEN végrehajtható keréksebesség tartozik a kész PhysicalMotionCommandhoz.
MotionExecutor:       MILYEN PWM kell a kért keréksebességhez.
SafetyGate:           EBBŐL MENNYI juthat ténylegesen a motorra.
EKF:                  HOL van a robot.
Szenzor/measurement:  MIT mért a robot.
```

**Egyik réteg sem veheti át a másik mondatának ownershipját.**
