# ER2 navigációs interfész

Az egyszerű metrikus mozgás canonical V3 action: `v3.command.move_relative`
és `v3.command.turn_by`. Mindkettő voice-exposed, session-watchdoggal védett,
és csak a mission végén, a végső STOP után tér vissza. Ezekhez nem szükséges ER2.
Az ER2 relatív tooljai ugyanazokra az actionökre delegálnak; a pose-olvasás,
célképzés és véges navigation lifecycle ownere a hostoldali
`v3/finite_navigation.py`, nem az ER2 bridge vagy a control tick.

Az ER2 alapfelülete öt tool: `robot_status`, `robot_stop`,
`robot_navigate_to_pose`, `robot_move_relative`, `robot_turn_by`.
A Preview és Streaming ugyanazt a bridge-et használja. A fizikai toolok
befejezésig vagy hibáig várnak, majd kanonikus STOP-ot és annak visszaigazolását
kérik. A Streaming deklarációjuk `BLOCKING`; egy tool-körben legfeljebb egy
fizikai művelet indulhat.

| Tool | Cél jelentése |
| --- | --- |
| `robot_navigate_to_pose(x_m, y_m, yaw_rad?)` | Abszolút metrikus cél az aktuális lokalizációs frame-ben; a kihagyott yaw szabad végső irányt jelent. |
| `robot_move_relative(forward_m, left_m=0, final_yaw_rad?)` | Az induló pose-hoz viszonyított cél; pozitív `left_m` balra levő célpont, nem oldalazás. A megadott végső yaw az induló irányhoz képesti radiáneltérés. |
| `robot_turn_by(angle_deg)` | Helyben fordulás, pozitív szög balra, negatív jobbra. Tartomány: −180…180°, a félfordulat a kanonikus normalizált irányt követi. |

A navigációs toolok opcionális `max_v_mps` és `max_omega_rad_s` korlátot
fogadnak; a turn csak szögsebességkorlátot. Ezek nem emelhetik az ER2
konfigurált korlátait. A toleranciákat a meglévő L5 konfiguráció adja.
A `robot_drive` csak explicit `Er2RobotTools(..., debug_drive=True)` esetén
kerülhet a modell tooljai közé; a közvetlen metódus diagnosztikához megmarad.

A végrehajtási út:

`Agent → RobotInterface → V3 finite action → OperatorController.navigate → v3.control_cli navigate → resident CommandGateway → L5–L12`

`ER2 → Er2RobotTools → ExternalRobotGateway → RobotInterface → ugyanaz a V3 finite action`

Az abszolút `robot_navigate_to_pose` a `v3.command.navigate` hostoldali
`wait_for_completion=True` módjával használja ugyanazt a finite executort.
A sima `v3.command.navigate` továbbra is acceptance után tér vissza és nem
voice-exposed. A `cancel_event` és `finite_timeout_s` hostoldali lifecycle
paraméterek, nem LLM action-paraméterek; utóbbi csak rövidítheti a watchdog keretét.

A CLI producer birtokolja a heartbeatet, command-revisiont, TTL-t és a
process-owner/watchdog ellenőrzést. Az ER2 nem publikál külön heartbeatet.
Az OperatorController az adott parancsból képzett mission megjelenését várja:
a már elért cél elfogadásához nem szükséges pozitív motor-output.

A V3 finite executor csak az adott commandhoz tartozó **mission és navigation identity**
egyezése mellett fogad el `COMPLETE` eredményt. A completion után sem
marad aktív command producer. Timeout, cancel, elvesztett/frissesség miatt
elutasított státusz, mission-váltás, frame-váltás, navigation-hiba vagy safety
STOP/FAULT megszakítja a műveletet. A completion természetes `NOT_ACTIVE`
STOP-ja megengedett; egyéb safety STOP nem jelent sikert. Sikertelen STOP
esetén a provider session hibával megszakad, nem kap sikeres tool-eredményt.
A finite relative action admissionje egyetlen hostoldali state/action
tranzakció: capture/runtime stabilizálás és szükséges re-arm → STOP/IDLE →
friss pose → relatív target → ugyanazon resident runtime-session NAVIGATE
admission. A pose és a target ezért nem élhet túl egy capture által kiváltott
runtime restartot. A runtime PID/session változása `RUNTIME_SESSION_CHANGED`,
a lokális odometria generation váltása `LOCALIZATION_FRAME_CHANGED` okkal
fail-closed megszakítást jelent.

A visszatérés `frame_provenance` mezőben megőrzi a frame ID-t, a resident
runtime PID-t, a lokalizáció generationt és a pose-státusz monotonic
időbélyegét. Ez fejlesztési/evidence adat; nem motor-authority.

A visszatérés tartalmazza a command és mission ID-t, a kért paramétereket,
induló pose-t, tényleges target pose-t, végső pose-t, progress-t és a
navigation/safety okot. A `COMPLETED` csak `COMPLETE` és sikeres STOP után
lehetséges; megszakításkor `INTERRUPTED` és explicit reason érkezik.
A voice executor a finite actiont kizárólag `COMPLETED` eredménnyel fogadja el,
és lezárt finite missionhöz nem indít új tartós behavior-observert.

A várakozási keret a meglévő `R2B4_ER2_WATCHDOG_S` (alapértelmezetten 10 s)
értékénél 0,5 másodperccel rövidebb, a tool indításától számítva. Az önálló
producer watchdog továbbra is a végső liveness-korlát. A közvetlen canonical
action a host session watchdogját használja (a voice/launcher alapértéke 30 s),
szintén 0,5 s tartalékkal. Hosszabb feladat
explicit nagyobb host-konfigurációt igényel; a modell nem emelheti a keretet.

A végső yaw realizációjához L6 az elért x/y tolerancián belül a meglévő
waypoint-utakkal adja át a célirányt L8-nak. A szükséges friss world/costmap
ellenőrzések megmaradnak, a korábbi trajectory/pending eredmény érvényét
veszti. L8–L12 és a motor/safety authority nem változik.

Az Agent Core `er2.delegate` contractja kötelező `reason` mezőt kér:
`visual_observation`, `multi_step_physical`, `continuous_feedback` vagy
`open_ended_spatial`. Az indok az agent tool-request evidence-ben és a tool
eredményében is megmarad. Egyetlen canonical motion nem delegálási indok.
Egyetlen kamerás megfigyeléshez a közvetlen `vision.observe` tool használható.

Az API/ownership refaktor nem módosítja az L5–L12 működését vagy az aktív
konfigurációt. Külön nyitott navigációs probléma: motor nélküli szimulációban
az 1,2 m-es cél előtt 0,14411 m-rel L6 nem ad progress-viable trajectoryt,
és L7 `NO_PROGRESS_VIABLE_TRAJECTORY` okkal megállít. A −0,5 m-es relatív cél
az induló tickben ugyanígy trajectory nélkül marad a jelenlegi envelope mellett.

A kért alapértelmezett 0,60 rad/s cap megtartott API-érték. Az aktív
0,15 m/s kerékminimum és 0,3557 m nyomtáv mellett egy helyben fordulás
minimuma kb. 0,8434 rad/s; a kisebb capet L8 nem emelheti meg, ezért
nulla mozgást ad. E korlátok rendezése külön navigation/envelope feladat,
nem a finite API második controllere. A finite action ilyen helyzetben sem
jelenthet sikeres completiont.

Élő LLM-routing, fizikai COMPLETE és cél-tolerancia nincs bizonyítva.
Fizikai validációhoz a felhasználó kifejezett mozgásengedélye szükséges.
