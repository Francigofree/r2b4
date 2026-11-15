# Navigációs stabilitás — 2026-11-15

Motormentes, source-first vizsgálat. Az architekturális és async contract nem
változott. A szimulátor izolált ROS-domainen a production V3/SLAM/Nav2 utat
futtatta; fizikai robotmozgás nem indult. Az alábbi számok szintetikus plantből
származnak, nem fizikai kalibrációból.

## Az élő capture tényei és korlátai

Forrás: `runtime/captures/v3_20261115_111329_7584_capture.mcap`, SHA-256
`c0bb370f10293cb0680f7ac21b739f6bf00fa16c792dfb4f21f11a8f42a0bb2f`.

- Natív deep integrity: PASS; explicit, egy workerrel futó MCAP Evidence
  Compiler: COMPLETE. [Elemzés](../../tmp/nav_fix_live_analysis/analysis.json).
- 397 mintavett tick, 79,083 s megfigyelt tartomány, kb. 5 Hz. A becsült globális
  pózból számolt mintavett út 10,980 m; ez nem fizikai ground truth.
- Nincs rögzített FAULT. A DIAG a 70,200 s aktív navigációból 22,221 s nulla
  mozgáskérésű HOLD-ot mér, 92 szakaszban. A HOLD nem bizonyít fizikai beszorulást.
  [Diagnózis](../../tmp/nav_fix_live_analysis/navigation_root_cause.json).
- A megtartott MPPI-evidence-ben 95 `no_nonzero_wheel_feasible_command`,
  122 `wheel_projection` és egy `lethal_cell` esemény van. A kerékadapter
  nullázása tényszerű; az összes nullázás károssága nincs bizonyítva.
- A recovery a `5859858411044 ns` időpontban indul, controller
  `NO_VALID_CONTROL=106` után. Az azt megelőző `5859793765798 ns` esemény
  `R2B4 projected trajectory: lethal_cell`. Az eredeti célhoz visszatérő
  navigáció később négyszer `NO_VALID_PATH=208` eredményt ad. Spin, hátrálás és
  detour önmagában nem igazolta az eredeti navigáció helyreállását; az epizód
  `5870304564051 ns` időpontban kimerül.
  [Események](../../tmp/nav_fix_live_analysis/recovery_events.json).
- Az útvonalhiány fizikai/térképi gyökérokát nem bizonyítja ez a capture.
  Nem változtattam emiatt safety-korlátot vagy planner-felelősséget.
- A natív Replayer **BLOCKED / CAPTURE_INCOMPLETE**: a kért capture mintavett,
  nem tartalmaz teljes 50 Hz-es lezárt inputot/raw LiDAR-t.
  [Replay](../../tmp/nav_fix_live_replay/replay.json).
- A capture-ből generált, 61 foglalt cellás világ csak közelítés; provenance-je
  explicit `causal_reproduction_proven=false`. Nem az incidens pontos replaye.
  [Szcenárió](../../tmp/nav_fix_live_scenario/scenario.json).

## Bizonyított és javított hibák

### Szimulátor: odometria-minősítés miatti önfenntartó megállás

Az éles `ProcessLidarPort.publish_slam_odometry` a folytonos, nem LOST lokális
odometriát exportálja. A szimulátor csak GOOD-ot fogadott el. Mozgás közben,
relatív scan-odometria nélkül L3 DEGRADED-ra váltott; ez a szimulátorban
érvénytelenítette a SLAM-odometriát és a scanek illesztését. A Nav2 scanideje
ezután valóban elöregedett: a kontrollált alapfutás 183. tickjében 260 ms-os
scan-age miatt lejár a 250 ms-os bound. A következő fékezés/újraindulás ugyanazt
a hibakört ismételte.

A szimuláció most az éles export minőség- és continuity-feltételeit követi;
LOST/discontinuity továbbra is tiltó. Hat minőségi kombináció célzottan
összehasonlítja a szimulált csomagot az éles export tényleges skalárjaival.

Azonos szobában a javítás előtt: 20 s, 0,116 m út, 0,884 m célhiba, FAIL.
Utána: **6,38 s, 0,960 m út, 0,0505 m célhiba, PASS**, 498 tick replay MATCH.
Ezt a szimulátor hibájának javítása bizonyítja; nem állítás az élő robot
ugyanilyen hibájáról.
[Előtte](../../tmp/nav_fix_room/run/result.json),
[utána](../../tmp/nav_fix_room_qualified_odom/run/result.json).

Az első, külön megszakadt futásban a diszkes parancspublikálás 1104 ms-ot
töltött a szimulátor inputszélén. A rövid életű mailbox ezért futásonként külön
`/dev/shm` könyvtárba kerül, majd lezáráskor törlődik. A canonical
ResidentCommandClient/Gateway, validáció, revision és TTL változatlan.
Ez nem bizonyít általános GIL-, fizikai I/O- vagy L0–L12 timing-javulást.

### Nav2 completion: láthatóság a tick lezárásához képest

A kliens a context létrehozása és a socket kiolvasása között keletkezett,
egyébként érvényes választ `NAV2_INVALID_PACKET` hibával eldobta. Most egyetlen
latest completiont tart vissza a következő alkalmas tickig. Az előző friss
javaslat csak eredeti lejáratáig maradhat használható; a visszatartás nem ír át
measurement-, source-, sequence-, generation- vagy TTL-adatot. Missionváltás,
cancel, transporthiba és malformed input törli a függő eredményt.

A célzott socket-tesztek bizonyítják a tickhatárt, a régi javaslat megőrzését,
a missionváltást és azt, hogy egy újabb késői completion nem éhezteti ki a már
láthatóvá vált korábbit. Az élő capture konkrét STOP-ját nem tulajdonítom ennek.

### Lifecycle és recovery: completion-idő és véges ellenőrzés

A lifecycle monitor előbb ellenőrizte a poll időpontját, mint a befejezett
future-t. Egy határidőn belüli válasz így késői polling esetén elveszhetett.
Most a ROS-edge callback eredeti monoton completion-ideje dönti el a request
határidejét, és ugyanebből számolódik a readiness frissessége. A ténylegesen
késői/nem érkező válasz továbbra is lejár; a timeoutértékek nem változtak.
Tesztelt az ACTIVE, INACTIVE és kivételes válasz is.

A recovery első feedbackje új öt másodperces határidőt indíthatott, akár az
eredeti hét másodperc lejárta után. Most csak szűkítheti a meglévő budgetet;
lejárt ellenőrzés késői progresszel sem éleszthető újra. A bridge a lejáratot
friss póz/feedback hiányában is feldolgozza és a futó gyermeket megszakítja.
Ezek forrásból reprodukált hibák; az élő capture-ben való előfordulásuk nincs
bizonyítva.

### Pozíciós cél és action-ACK

A pozíciós, `yaw_rad=None` NAVIGATE célhoz a bridge kitalált végső yaw-t és
szigorú yaw-toleranciát kért. Most ilyen missionnél a Nav2 yaw-tolerancia π;
explicit yaw esetén a korábbi szigorú érték marad. L6 változatlanul csak friss,
missionhöz kötött lokalizáció és a kért célfeltételek alapján fogad el sikert.

A fél pozíciós tolerancia **megmaradt**. A teljes mission-toleranciával végzett
kísérletben Nav2 túl korán jelentett sikert, és L6 helyesen
`NAV2_SUCCESS_UNCONFIRMED` HOLD-ot adott. Ez a kísérlet FAIL és nem került be a
megoldásba. [Kísérlet](../../tmp/nav_fix_controller_pause_ack/run/result.json).

## Éles konfiguráció változása

Kizárólag a `conf/nav2_navigate.xml` FollowPath action node kap explicit
`server_timeout="500"` értéket, a korábbi általános 200 ms helyett. Ez a
**goal-ACK protokollhatár**, nem velocity-, szenzor-, cancellation- vagy fizikai
safety-timeout. A 350 ms-os controller-felfüggesztés 200 ms mellett konkrét
`Timed out ... acknowledge goal request for follow_path` hibát és terminal
navigációs kudarcot okozott; a controller később még tovább dolgozott, de a
bridge mozgásjavaslatát már nem engedte át.

500 ms mellett ugyanaz a véges szcenárió PASS: a lejárt velocity helyett nulla
kimenet maradt, a controller folytatása után **2,38 s-nál** újra volt mozgás
(a pause 2,35 s-nál ér véget), és **13,94 s-nál** L6 igazolta a célt.
Végső célhiba 0,0569 m, nulla ütközés, végső STOP, 914 tick replay MATCH.
[Előtte](../../tmp/nav_fix_controller_pause/run/result.json),
[utána](../../tmp/nav_fix_controller_pause_final/run/result.json).

A fizikai kerékminimumok, testgeometria/padding, L9 gyorsuláskorlát, L12 safety,
a 200 ms-os velocity TTL és a 250 ms-os Nav2 szenzorfrissesség változatlan.
A processz- és action-élettartam továbbra is bounded; tartós providerhiba nem
lesz végtelen retry. A szoftveres konfigurációváltozás a következő indításkor
hat, élő runtime-ot nem indítottam újra.

## További validáció

- 600 ms Nav2 transportkiesés: PASS, ugyanaz a mission, mind a 30 fault-tickben
  nulla kimenet, 2,84 s-nál resume, célba érés, 943 tick replay MATCH.
  [Futás](../../tmp/nav_fix_nav2_loss/run/result.json).
- A végleges simulator/transport/lifecycle/recovery és Nav2 integrációs
  célzott tesztcsoport: **83 PASS**. Parancs:
  `./r test tests/packs/feature/test_nav2_navigation.py tests/packs/feature/test_nav2_lifecycle.py tests/packs/feature/test_room_cruise_recovery.py tests/packs/feature/test_local_navigation_simulator.py tests/packs/feature/test_nav2_control_integration.py`.
- Aktív BT-config változása miatt canonical `./r test release`: **24 PASS**.

## Hangolási kísérlet

A javítás utáni explicit keresés COMPLETE. Azonos room/seed/production
konfiguráció mellett a PathFollowCritic súlya 5,0 → 4,0 vagy 6,0 volt, minden
futás teljes capture-rel és MATCH replayjel. Mindhárom célteszt PASS, de
**nincs elfogadható nyertes**, ezért nincs automatikusan vagy kézzel alkalmazott
critic-súlyváltozás. Győztes hiányában a kereső nem indított holdout-kiválasztást.
[Teljes keresés](../../tmp/nav_fix_tuning_final/autotune.json).

| PathFollowCritic | Célba érés | Célhiba | Leghosszabb állás | Virtuális max. jerk |
|---|---:|---:|---:|---:|
| 5,0 baseline | 5,28 s | 0,0502 m | 0,36 s | 114,0 m/s³ |
| 4,0 | 5,20 s | 0,0586 m | 0,34 s | 121,2 m/s³ |
| 6,0 | 7,48 s | 0,0598 m | 1,02 s | 119,6 m/s³ |

A 4,0-s változatnál nagyobb tick-wall maximum is jelentkezett, de ez nem
bizonyítja a critic közvetlen CPU/GIL-hatását. A 6,0-s változat lassabb volt és
többet állt. Ezek egyszeri ROS-mérések: a számszerű különbségekből nem állítok
fizikai javulást vagy általános algoritmusrangsort. A jerk a virtuális plant
véges különbsége; emergency STOP és a szintetikus kerékválasz is befolyásolja.

## Nyitott bizonyítás

Az élő `NO_VALID_PATH` ismétlődésének pontos térképi oka, a teljes live incidens
counterfactual reprodukciója, fizikai mozgásminőség, többórás stabilitás és a
faultok összes kombinációja nincs bizonyítva. A virtuális jerk/scheduler-idő
nem fizikai kalibráció vagy izolált control-timing. A paraméterkeresés eredménye
csak a ténylegesen lefutott, replayelt train/holdout esetekre vonatkozhat.

## Eltérő geometria: megmaradt kudarcok és elutasított kerülőmegoldás

A 0,9 m széles folyosó próba **FAIL**, 0,00073 m út, nulla ütközés, teljes
1209 tickes capture és MATCH replay. Az első érdemi kudarc a
`7579333122537 ns` időpontban a kerékadapter collision recheckje: a
`v=0,2268157005 m/s`, `omega=0,2800139487 rad/s` állandó twist 1 s-os
kivetítésének 15. mintája a `[38,37]`, 254 költségű cellát érinti. Ez a
testre/paddingre vonatkozó jogos tiltás konkrét bizonyítéka; nem bizonyítja,
hogy az egész folyosó járhatatlan, sem hogy melyik MPPI-beállítás oldaná meg.
[Futás](../../tmp/nav_fix_corridor/run/result.json).

Külön kísérletben a Nav2 `failure_tolerance=0,5 s` bounded újraszámítása is
ugyanezt a tiltást ismételte, majd `Controller patience exceeded` hibával
lezárult. Nem oldotta meg a gyökérokot, ezért a változtatást **visszaállítottam**
az eredeti 0,0-ra. A collision recheck/padding/fizikai korlát nem lazult.
A Nav2 e policy esetén nulla parancsot ad és csak `NoValidControl` esetén
próbálkozik újra; ezt az azonos 1.3.13 verzió
[controller source-a](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_controller/src/controller_server.cpp#L597)
is rögzíti. [Sikertelen kísérlet](../../tmp/nav_fix_corridor_recovery/run/result.json).

A capture-ből közelített, 61 cellás világ első próbája szintén FAIL:
0 m út, 30 s állás, nulla ütközés, MATCH replay. A szintetikus geometria
számítása 29,35 s thread-CPU-t és 32,37 s tick-wall időt használt; a frissességi
gátak mellett ez nem tekinthető használható navigációs kísérletnek.
[Futás](../../tmp/nav_fix_capture_approximation/run/result.json).

A célzott profilban a test–téglalap távolságszámítás a távoli cellák minden
élét feleslegesen vizsgálta. Most a test AABB-jének távolság-lower-boundja
kizárja azokat a cellákat, amelyek az aktuális minimumot nem csökkenthetik;
a lehetséges ütközéseken és közelebbi cellákon ugyanaz az exact SAT/edge
vizsgálat marad. 500 rögzített seedű pózon a maximum eltérés **0**, a mért
thread-CPU 0,932 → 0,284 s volt (3,28×). Ez offline geometriai microbenchmark,
nem control- vagy ROS-ütemezési állítás. Külön regresszió ellenőrzi, hogy a
több falas világ clearance-e megegyezik a független egyfalas lekérdezések
minimumával, elforgatott és ütköző pózokon is.

Ugyanennek a közelített világnak a javítás utáni 30 s-os futása **PASS**:
2,297 m virtuális út, 12 meglátogatott 0,25 m-es truth-cella, nulla ütközés,
0,189 m minimum test-clearance, 8,76 s leghosszabb állás; 1665 tick natív
replay MATCH. A végső canonical STOP után a virtuális test is megállt.
[Futás](../../tmp/nav_fix_capture_geometry/run/result.json).

A teljes szimulált tickek thread-CPU ideje 29,35 → 22,87 s, wall ideje
32,37 → 24,67 s, maximum pacing-késése 424 → 82,7 ms. A 329 darab 20 ms feletti
tick és a 102 ms maximum megmaradt; ezek a szenzorszintézist és observationt
is tartalmazzák. Ez használhatóbb szimulációs evidence, de nem 50 Hz-es
hard-real-time garancia és nem az izolált L0–L12 futásideje.

Az újraindított MPPI-súlykeresés és a fenti folyosó/MCAP-közelítés külön
kísérletek. Az utóbbi hibáiból nem választottam utólag másik paraméternyertest.
A `conf/nav2_planner.yaml` végső tartalma megőrizte az eredeti beállításokat.
