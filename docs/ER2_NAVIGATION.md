# ER2 navigációs interfész

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

`ER2 → Er2RobotTools → ExternalRobotGateway → RobotInterface → V3ControlInterfaceAdapter → OperatorController.navigate → v3.control_cli navigate → resident CommandGateway → L5–L12`

A CLI producer birtokolja a heartbeatet, command-revisiont, TTL-t és a
process-owner/watchdog ellenőrzést. Az ER2 nem publikál külön heartbeatet.
Az OperatorController az adott parancsból képzett mission megjelenését várja:
a már elért cél elfogadásához nem szükséges pozitív motor-output.

Az ER2 csak az adott commandhoz tartozó **mission és navigation identity**
egyezése mellett fogad el `COMPLETE` eredményt. A completion után sem
marad aktív command producer. Timeout, cancel, elvesztett/frissesség miatt
elutasított státusz, mission-váltás, frame-váltás, navigation-hiba vagy safety
STOP/FAULT megszakítja a műveletet. A completion természetes `NOT_ACTIVE`
STOP-ja megengedett; egyéb safety STOP nem jelent sikert. Sikertelen STOP
esetén a provider session hibával megszakad, nem kap sikeres tool-eredményt.
A visszatérés tartalmazza a mission ID-t, végső pose-t, progress-t és a
navigation/safety okot. Ez a meglévő resident státuszból származik.

A várakozási keret a meglévő `R2B4_ER2_WATCHDOG_S` (alapértelmezetten 10 s)
értékénél 0,5 másodperccel rövidebb, a tool indításától számítva. Az önálló
producer watchdog továbbra is a végső liveness-korlát. Hosszabb feladat
explicit nagyobb host-konfigurációt igényel; a modell nem emelheti a keretet.

A végső yaw realizációjához L6 az elért x/y tolerancián belül a meglévő
waypoint-utakkal adja át a célirányt L8-nak. A szükséges friss world/costmap
ellenőrzések megmaradnak, a korábbi trajectory/pending eredmény érvényét
veszti. L8–L12 és a motor/safety authority nem változik.

Motor nélküli validáció: `./r test`, `./r test motion`,
`./r pytest -q tests/feature/test_er2_p0r2.py`, `./r test full`.
Az új tesztek a mailbox-validációt és TTL-t, a teljes host/CLI bekötést,
mission-azonosságot, megszakítást és STOP-hibát, valamint a zárt hurkú
L5–L9 fordulást/elmozdulást, determinisztikus visszajátszást és késői planner
completion eldobását ellenőrzik. Valós capture-replay még nem futott. Fizikai validációhoz a felhasználó
kifejezett mozgásengedélye szükséges.
