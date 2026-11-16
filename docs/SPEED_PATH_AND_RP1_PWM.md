# Sebességút, RP1 PWM és kalibráció — 2026-11-16

Ez megvalósítási és mérési jegyzet. Az architekturális authority továbbra is
`STRUKTURALIS_RETEGEK_V3.md` és `ASZINKRON_RUNTIME_CONTRACT_V3.md`.

## Eredmény és még hiányzó elfogadás

A production motorút most RP1 hardveres PWM-et használ. Az L8 Nav2 esetén
változatlanul adja tovább a controller javaslatát; a független fizikai korlát és
a végrehajtott gyorsulási rámpa az L9-é. Az encoder fit bizalma az időlefedés
mellett a ténylegesen felhasznált pulzusok számától is függ. Elkészült a
canonical parancsokkal mérő, natív MCAP-ból Speed Mapet generáló fejlesztőeszköz.

**A 0–1,5 m/s teljes fizikai tartomány, az új négygörbés térkép és a PI
újrahangolása még nincs elfogadva.** A meglévő map/config nem kapott kitalált
1,5 m/s-os pontokat vagy mérés nélküli gainváltozást. Az első beltéri sweep
LiDAR safety STOP miatt megszakadt. Új felhasználói engedéllyel, áthelyezés és
mindkét irányban legalább 2 m szabad út visszaigazolása után ismételtünk:
a második hátrameneti lépés fizikai megállással és L11 missing-edge FAULT-tal
végződött. Ezután nem indult újabb mozgáspróba.

## Source-first sebességút

| Határ / owner | Sebességre ható művelet, aktív érték | STOP / indulás |
|---|---|---|
| Közös fizikai envelope: `v3/config.py`, `conf/speed_map.json` | Négy előre/hátra görbe közös maximuma a legkisebb végpont: **0,582 m/s**. Tartós fizikai minimum **0,15 m/s**; L9 target reserve **0,19 m/s**. Nyomtáv **0,3557 m**. L6/L8/L9 ugyanazt a `WheelMotionLimits` értéket kapja; L10 ebből kap geometriát. | Nulla megengedett; 0 és a minimum közötti sebesség csak véges L9 átmenet. |
| Nav2 MPPI: `conf/nav2_planner.yaml`, `tools/nav2_planner.py::_write_geometry` | 20 Hz controller, 0,05 s modelllépés, 1,6 s horizont. Generált `vx_max=0,45`, `vx_min=-0,20`, `wz_max=1,4`. Javaslatképző `ax_max=4,18`, `az_max≈23,503`: tudatos holtsávátlépés, nem a végrehajtott robotgyorsulás. A wheel critic a minimum alatti tartós jelölteket bünteti. | A nagy proposal acceleration egyszerű visszavétele 0,6-ra induláskor 0,03 m/s-os, az adapter által elutasított célt eredményezhet. Nem lett vakon átállítva. |
| Wheel Adapter: `deploy/nav2_wheel_adapter/{controller.cpp,wheel_projection.hpp,wheel_critic.cpp}` | Közös wheel min/max, platform/mission body cap, görbületkorlát. MPPI súlyozott átlag után realizálható wheel familyre vetít. A megváltozott parancsot újra ellenőrzi a padded footprinttel, 1 s konstans parancshorizonton. Ütközésnél csak újra ellenőrzött lassabb `.75/.5/.25` jelöltek vagy más realizálható family használható. | Nem realizálható parancs nulla; nem igazolható ütközésmentesség controller-elutasítás. A safety/collision ellenőrzés nem lett eltávolítva. |
| L6: `v3/layers/_l6_nav2.py::accept_nav2_guidance` | Nav2 sebességhez nem ad új rámpát. Mission-, generation-, sequence-, request/frame-lineage, lokalizáció, eredeti odom/scan idő és TTL alapján fogad. | Hiányzó/stale/superseded/hibás javaslat HOLD, nulla mozgás. Receipt nem hosszabbít TTL-t. |
| L7: `v3/layers/l7_motion_selection.py` | Egy velocity objective; a Nav2 célérték változatlan. Csak folytonos mission/frame/scope és még élő authority mellett ad transition permissiont. | STOP/lejárat megszünteti a megtartott objective-et. |
| L8: `v3/layers/l8_motion_realization.py` | Nav2 path trackinget nem zár újra. **Eltávolítva a Nav2-ra ismételten alkalmazott steady wheel projection.** A többi objective saját geometriai realizációja megmaradt. | Context/frame/TTL/explicit STOP továbbra is azonnali nulla; L9 az invalid Nav2 envelope-ot is önállóan korlátozza. |
| L9: `v3/layers/l9_operational_constraints.py` | Platform **0,45 m/s**, **1,4 rad/s**; body és coupled wheel acceleration **0,6 m/s²**, angular **2,5 rad/s²**; curvature **32 rad/m**. DEGRADED célsebességskála **0,4**, acceleration-skála **0,3**. Egyetlen stateful fizikai rámpa; explicit cap azonnali hard limit. | HOLD/STOP nem simítható pozitív mozgássá. Subfloor epizód határideje nem megújítható; jelenleg legfeljebb `2*.582/(.6*.3)≈6,467 s`. |
| L10: `v3/layers/l10_chassis_control.py` | Pontos `left=v−ω*track/2`, `right=v+ω*track/2`. Nincs kerékenkénti felfelé clamp vagy második minimum. | Átadja a revocation/transition metadata-t. |
| L11: `v3/layers/l11_actuator_control.py` | Mért Speed Map interpoláció + kerékenkénti PI: `kp=.25`, `ki=.6`, integrator limit `.75`, output cap `.75`. A feedback-gain belépése 0,1 s; anti-windup és irányváltási reset. Ez duty-korrekció, nem második sebességrámpa. `startup_pwm` metadata, nem induló boost. | Nulla cél/revocation PI-reset és nulla output. Gyenge velocity-fitnél csak igazolt saját accepted edge-liveness mellett FF; bizonytalan fitből nincs PI. Hiányzó edge watchdog 0,5 s, feedback age 0,25 s. |
| L12: `v3/layers/l12_safety_final.py` | Egyetlen final döntés/writer. Friss, közvetlen irányfüggő LiDAR body-clearance minimum **0,10 m**; kritikus health/fault gate. Nem módosít normál sebességprofilt. | STOP/FAULT minden fizikai duty nulla. A final STOP a pillanatnyi upstream setpointot fizikailag felülbírálja. |
| Motor: `v3/composition/motor_output.py`, `v3/adapters/rp1_motor.py` | **8 kHz RP1 PWM0**, GPIO 12→0, 13→1, 18→2, 19→3. A meglévő jobb oldali inversion és DRV8871 brake duty-semantika megmaradt. Nincs lgpio `tx_pwm` a kiválasztott production motorágon. | Irányváltás csak az érintett kerék nullázásával, legalább egy PWM-periódus szünettel; normál duty-frissítés nem állítja újra a hullámformát. STOP-ticken nincs korábbi kötelező 2 ms sleep. |

A közvetlen command út további tudatos korlátai: command ingress **0,5 m/s**,
publikus forward/backward/teleop/wheels API-k **0,5 m/s**, illetve egyes
forgási API-k **1,2 rad/s**. A Room Cruise/mission/platform cap 0,45 m/s;
FOLLOW 0,25 m/s. Ezeket és a Nav2 −0,20 m/s reverse-policyját külön kell
összehangolni az új, **ténylegesen megmért** felső tartománnyal. A külön
mission/behavior cap hasznos policy, nem új kalibrációs authority.

## RP1 telepítés és fail-safe működés

`bash deploy/install_rp1_pwm.sh` leállított canonical runtime-ot követel;
lefordítja/telepíti a négycsatornás overlayt, menti a boot konfigurációt a
projekt `tmp/rp1_install_*` könyvtárába, és motor-output nélkül állítja a
négy pinmuxot. A jelen gép boot konfigurációjában a `pwm-2chan` helyére
`r2b4-pwm` került. A gép nem lett újraindítva; a boot utáni pinmux külön
elfogadási pont marad. A meglévő Raspberry Pi udev PWM/gpio-group szabályt
használjuk.

A sink DT identity alapján találja meg a header PWM0-t (`pwm@98000`), nem
`pwmchip0` névre támaszkodik. A másik RP1 PWM a hűtőventilátoré, nem nyúl hozzá.
Inicializáláskor ellenőrzi a pinmuxot, kizárólagos ownership lockot tart,
előre megnyitja a duty FD-ket, és nulla állapotot olvas vissza. Nemnulla korábbi
duty esetén nulláz és elutasítja az indulást. STOP/close: normál polaritás,
**enabled=true, duty=0**; a csatornák ebben az állapotban exportálva maradnak.

A sysfs hardware PWM túléli a vezérlőprocessz halálát. Emiatt egy külön,
kizárólag nullázni képes watchdog védi az outputot: a friss frame-heartbeat
elvesztése 250 ms után vagy owner-halál végleges tiltást és nullázást okoz.
Az időkorlát csak nemnulla pin-duty mellett aktív. Mind a négy output
nullázása után disarmolódik, így egy hosszú, már biztonságosan álló terminal
capture/device cleanup nem öli meg tévesen az ownert. A következő írás előtt
ismét élesedik; owner-halálkor a guardian ettől függetlenül nulláz.
Megakadt ownernél pidfd-vel azonosított processzleállítás előzi meg a nullázást,
hogy egy későn felébredő writer ne írhasson ismét pozitívat. A guardian a
nullázásig megtartja az ownership lockot. Normál frame csak az L12-től érkezhet.

A software watchdog nem helyettesít külső motor-enable/hardveres watchdogot:
a control és guardian egyidejű kényszerleállításának biztonságát nem bizonyítja.
A live kernel-state/pinmux ellenőrzött; független oszcilloszkópos hullámforma-,
irányváltási tranziensteszt és reboot utáni elfogadás nem történt.

Elsődleges hardware források:
[RP1 pinmux](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/pinctrl/pinctrl-rp1.c),
[RP1 PWM kernel driver](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/pwm/pwm-rp1.c),
[Linux PWM inactive-state contract](https://www.kernel.org/doc/html/latest/driver-api/pwm.html).

## Automatikus Speed Map eszköz

```bash
python3 -m tools.wheel_speed_calibration plan \
  --speeds .15 .19 .26 .35 .45 --duration-s 4 \
  --track-length-m 3 --direction both --output tmp/speed_plan_UNIQUE

# Fizikai mérés; kizárólag alkalmas, szabad pályán, aktuális mozgásengedéllyel:
python3 -m tools.wheel_speed_calibration run \
  --speeds .15 .19 .26 .35 .45 --duration-s 4 \
  --track-length-m 3 --direction both --output tmp/speed_run_UNIQUE

python3 -m tools.wheel_speed_calibration generate \
  runtime/captures/MEASURED_capture.mcap \
  --required-maximum .45 --output tmp/speed_map_UNIQUE
```

`both` esetén a megadott hossz **mindkét irányban** szabad kell legyen a
kiindulási helyzettől. Egyirányú futásnál az eszköz az összes lépés kumulált
útját ellenőrzi. A pályaszámítás tartalmaz fékutat és elhelyezési tartalékot;
a tényleges szabad területet és L12 safety-t nem helyettesíti.

A `run` csak leállított runtime-ból indul, saját FULL 50 Hz capture-t készít,
a publikus véges `r forward/backward` parancsokat használja, majd STOP/shutdown
után offline illeszt. Safety STOP/fault/státuszhiba/deadline megszakítja a sort,
nincs retry. A saját runtime megszakítása a canonical runtime STOP/shutdown
útját használja; FAULT esetén nem vár lehetetlen egészséges IDLE-ra. Egy
másodlagos shutdown-hiba nem írja felül az eredeti mérési hibát.
Közvetlen PWM-, GPIO-, `WheelVelocitySetpoint`-injekció nincs.

A generátor csak sértetlen, replay-képes canonical live 50 Hz MCAP-ot fogad;
azonos PWM/wiring/encoder-geometriájú capture-öket lehet egyesíteni.
L2-ben már elfogadott, változatlan ismételt sample eredeti idejét/sequence-ét
őrzi és deduplikálja. L9 ramp, saturation, revocation, STOP, hiányzó/hibás
encoder kizárt. A felhasználó szerint az első passzív, függőleges tengely körül
önbeálló kerék irányváltáskor 180°-ot fordul, lassú menetben kb. 1 s alatt.
Ezért az alapértelmezett settle **1,25 s**, utána legalább **1 s/40 új
measurement** szükséges. A `--settle-s` ezt explicit szabályozza; a terv a
L9 rámpájára, a settle-re és a mérési ablakra együtt túl rövid lépést elutasítja.
Az első/második félablak sebessége nem térhet el jelentősen; későbbi stabil
suffix csak ugyanilyen idő-/sampleszám-kapuval fogadható el. A kerékenkénti
kumulatív quadrature-rejection számláló változása megszakítja a plateau-t:
a control által átugrott acquisition sample nulla aktuális delta mögött sem
rejthet el zajos intervallumot. A jelentés külön számlálja ezeket a kizárásokat.
A sebesség signed raw displacement / **mérési idő**, a duty az ugyanazon időablakra integrált
final L12 output. A kért cél nem címkézheti át a ténylegesen mért sebességet.

Négy monoton, megfelelő tartományú görbe nélkül `INSUFFICIENT_EVIDENCE` és
részletes mérési jelentés készül; nincs extrapolált/automatikusan aktivált map.
Az első steady pont nem stiction/startup mérés: a külön maintenance/startup
küszöbök megmaradnak, nem állnak át az első pont duty-jára.
Az aktuális API/map/platform cap fölé a mérőeszköz sem kerülhet. Az 1,5 m/s-os
bootstrap és végpont-elfogadás további, külön megtervezett fizikai munka.

## Validáció és élő evidence

- `r test release`: **28 passed**. Az RP1 STOP és owner-death/hang invariáns
  tudatosan bekerült a kis gate-be; a manifest 14 logikai bejegyzés.
- A legutolsó kalibrátorjavítás után `r test`: **18 passed**.
- PWM + kalibrátor célzott csomag: **17 passed**; encoder célzott csomag:
  **25 passed**; Nav2/L8/L9/minimum célzott ellenőrzés: **21 passed**.
- Motor nélküli teljes Nav2/SLAM room baseline és azonos scene regression:
  mindkettő PASS, cél elérve, nulla collision, a comparison nem jelzett regressziót.
  Mindkét szimuláció natív replayje MATCH. Ez rövid szintetikus AMR evidence;
  az első tracking kerék konkrét mechanikai ellenállását nem validálja.
- Első live: `runtime/captures/v3_20261116_184931_4720_capture.mcap`.
  2 s / 0,15 m/s előre, 106 mozgó tick, nincs layer fault. 1438 tick,
  teljes raw/debug evidence; EVI COMPLETE, natív replay MATCH.
- Sweep: `runtime/captures/v3_20261116_185533_5655_capture.mcap`.
  Csak előre, 0,15 / 0,26 / 0,35 m/s célok, legfeljebb 3,5 s/lépés.
  Első eltérés **tick 1034, L12, `LIDAR_CLEARANCE_LOW`**:
  front/left body-clearance **0,09947349281116619 m < 0,10 m**.
  A megállított kerekek accepted edge-einek hiánya miatt **tick 1074, L11**
  watchdog FAULT következett. Minden tiltó final output nulla volt; a kernel
  mind a négy PWM csatornán nulla duty-t mutatott.
  1075 tick, teljes integrity, EVI COMPLETE, natív replay MATCH.
- A hibából következő javítás: a mérőhost már az aktív TELEOP első safety
  STOP-jánál canonical STOP-ot küld és nem folytatja a sort. Ezt célzott
  teszt igazolja.
- Új engedéllyel végzett motor nélküli preflight:
  `runtime/captures/v3_20261116_190508_6723_capture.mcap`, 675 tick,
  minden final output nulla, teljes integrity. A felhasználó megerősítette az
  áthelyezést és a mindkét irányban legalább 2 m szabad pályát.
- Ismételt kétirányú sweep:
  `runtime/captures/v3_20261116_190934_7144_capture.mcap`.
  Tervezett célok 0,15 / 0,19 / 0,26 / 0,35 m/s, 4 s/lépés, előre/hátra.
  Az első három lépés lefutott; a **második hátramenet 1397. tickjén L11
  `MISSING_EDGE:left;MISSING_EDGE:right`** FAULT keletkezett. Az elfogadott
  saját élek kora 512/514 ms volt. Előtte az L12 ALLOW és kb. −0,24665
  normalized output maradt kiadva, nem előzetes L12 safety STOP történt.
  A felhasználó fizikai megállást, nem forgó motort látott, és külön jelezte
  az első tracking kerék mechanikai hatását; a hajtott kerekek is álltak.
  A tracking kerék kb. 1 s-os átfordulása lehetséges terhelési ok,
  **nem bizonyított gyökérok**: a driveráram, motorbusz-feszültség és a
  megállás alatti fizikai PWM-hullámforma nincs mérve. A LiDAR relatív
  mozgásbecslése eközben nem egyezik teljesen az encoderrel; ebből nem
  állítható bizonyított folyamatos haladás vagy tisztán encoderes hiba.
  Az első hátramenet alatt a bal quadrature-rejection számláló **11 548**-cal
  nőtt, így az érintett bal intervallumok kalibrációra nem alkalmasak.
  1398 tick, teljes integrity, EVI COMPLETE, natív replay MATCH.
  Evidence: `tmp/speed_bidir_first_fault_20261116.json`,
  `tmp/speed_bidir_legs_20261116.json`,
  `tmp/speed_bidir_replay_20261116_2`, `tmp/speed_bidir_evi_20261116_2`.
  A runtime STOPPED; a négy kernel-PWM duty nulla, 125000 ns periódus,
  enabled=true, normal polarity. A Pi `get_throttled` értéke `0x0`;
  ez nem motorbusz-feszültségmérés.

A sweepből elfogadott részpontok pl. left-forward **0,16471 m/s ↔ 0,17445 duty**,
right-forward **0,28290 m/s ↔ 0,23959 duty**. Ezek különböző plateau-k;
nem teljes kalibráció. Az 1,25 s settle-lel újraértékelt ismétlésből egy-egy előremeneti és egy jobb
hátrameneti stabil részpont maradt. A szigorított quadrature-kapu 190 bal és
4 jobb measurementet kizárt; bal hátrameneti pont nincs. Az offline jelentés
`tmp/speed_bidir_caster_settle_20261116/report.json` továbbra is
`INSUFFICIENT_EVIDENCE`; nem készült aktiválható térkép. A régi map és a
PI-gain emiatt változatlan.

A sweepben a motorírás mért wall mean **0,690 ms**, max **10,212 ms**;
thread-CPU mean **0,123 ms**, max **0,308 ms**. A teljes pipeline wall max
**19,701 ms**. Ez region timing; wall−CPU nem választja külön a kernel I/O-t,
GIL-várást és scheduler preemptiont. Azonos feltételű régi PWM live baseline
hiányában performance/jitter javulás nem állítható.

Következő lépés először a tracking kerék szabad gördülésének/irányváltási
viselkedésének és a bal encoder zajának tisztázása, motor-output nélkül.
A tracking kerék indulási/irányváltási terhelését külön kell kezelni az
állandósult Speed Maptől: automatikus duty-boost, PI-emelés vagy a missing-edge
watchdog lazítása nem következik a mostani evidence-ből. Új motoros próba
előtt az első megállás körülményét kell tisztázni, majd megfelelő oldal- és
hosszirányú szabad pályán stabil, ismételt négygörbés mérés; Speed Map
validálás; ebből plant
step-response és PI-hangolás; fokozatos felsőhatár-emelés és fékút/LiDAR/nav
envelope újramérés; végül teljes-range AMR elfogadás. Az L12 STOP miatti
upstream wheel-loop uncertainty és az újraindulási rámpa általános
összehangolása nyitott: nem lett safety-bypass vagy új L12→L11 rejtett adatél.
