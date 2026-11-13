# Room Cruise stabilizálás és életciklus — 2026-11-13, esti folytatás

Az operátor 18:56-os, 60 másodperces futása a megőrzött összehasonlítási alap:
`runtime/captures/v3_20261113_185619_2767_capture.mcap`.
Az új recovery fizikai javulása még nincs bizonyítva. Az egyetlen új mozgásos
próbát a monitor az első cél és minden nem nulla motorkimenet előtt megszakította,
mert a Nav2 bridge státuszában 1,55 s-os publication gap volt. Automatikus újabb
mozgáspróba nem indult; a további Nav2-vizsgálatok szintetikus ROS-bemenettel,
motorút nélkül futottak.

## Mit mutat a 18:56-os futás?

A mission 60 s-ig aktív, 300 mintavett control tickkel. A heading mindegyikben
GOOD; L12 safety stop/fault nincs. Öt child goal és frontier → coverage váltás
látható. A döntési átmenetekből 38,30 s nem nulla, 21,78 s nulla kimenet adódik;
utóbbiban a kezdeti 5,12 s admission is benne van. A lokalizált út becslése
7,32 m, az érintett footprint-raster 2,03 m². Ezek lokalizációból származó
mérőszámok, nem független pályamérések vagy a teljes szoba lefedettsége.

295 használható Nav2 observationből 286 scan–SLAM-map összevetés elérhető.
A legfeljebb 96 eredeti return végpontjának egycellás occupied-szomszédsági
egyezése mediánban 1,00, alsó decilise 0,976. Ez belső konzisztencia,
nem ground truth. A tényleges costmap és a Nav2 út a saroknál ismételt
collision-elutasítást mutat; a jó térképegyezés nem bizonyítja a terv
kinematikai végrehajthatóságát. A mintavett costmap/út ábrája:

![A golden futás costmapje és útjai](../../tmp/nav-stability-followup-2888b851/golden_costmap_paths.png)

A controller naplójában a mission körül 18 collision abort és öt
transform-hiba látszik; a végső action-hibák 106 (`NO_VALID_CONTROL`).
Két coverage cél sikertelen, elért coverage cél nincs. A harmadik/negyedik
cél körül a robot közel 20 s-ig ugyanabban a sarokban próbálkozik. A rövid
hátrálás után az új FollowPath-próba visszaviszi az előző helyzetbe.

A SLAM-késés továbbra is konkrét fennmaradó probléma. A mission végi 64 pózos
ablak acceptance késése p50 139 ms, p95 429 ms, maximum 523 ms. A DDS transport
p95 1,38 ms, a callback queue p95 2,19 ms; a nagyobb késés a producer előtti
és scan-publication → pose-send szakaszokban van. A futás teljes addigi
1467 pózából 48 stale elutasítás történt. A 250 ms-os acceptance-határ változatlan;
a késési tüskék megszűnése nem állítható.

## Szűk javítások

- A SLAM map→odom elérhetőségi horizontja 0,25 s helyett 0,50 s. A scan
  kora, az elfogadott scanek közti idő és a TF publication együtt korábban
  a friss odometria mögött hagyhatta a TF fejét. Az eredeti mérési idők,
  `restamp_tf=false` és a V3 freshness-kapuk megmaradnak.
- A BT harmadik recovery lépése most egyetlen `BackUpThenTurn` Sequence:
  0,30 m hátrálás 0,19 m/s sebességgel, majd 0,7854 rad elfordulás.
  A két lépés közé nem ékelődik FollowPath. A külön Wait és Spin, a hat
  recovery retry, a collision-check és a V3 motor/safety út megmarad.
- Az 5 Hz-es observation külön rögzíti a map→odom TF-horizontot és az
  odometriához viszonyított headroomot; ez nem fizikai mérési kor.
  Legfeljebb 16 eredeti idejű controller error, hibafajta-számlálók,
  goal/cancel állapot és a frontier indítás cleanup/subscription/launch
  időtartamai is láthatók. A nagy térképadat továbbra sem kerül a controlba.

## Életciklus-audit és egyszerű indítás

Az `./r rc 60 c full` előkészíti a konfigurált SLAM → resident runtime → Nav2
láncot, majd a canonical behavior/command úton kéri a cirkálást. A 60 s a
mission tényleges végrehajtási ideje; a hidegindítás és admission külön várás.
A CLI ellenőrzi a saját behavior terminális állapotát, hibát nem jelent
sikeres befejezésnek. Az alap capture-frekvencia 5 Hz.

A meglévő runtime host-supervisor takarítja a saját maga indította Nav2- és
SLAM-függőségeket, a resident bezárása után. A már külön futó szolgáltatásokat
nem veszi át. A hidegindítás megszakításának és hibájának takarítása is ezen
a meglévő host-életcikluson marad; új orchestration framework nincs.

A Nav2 host auditja két hibát javított: az ismételt start korábban bármely
élő supervisort késznek tekinthetett; a stop külső határideje pedig rövidebb
volt a gyermekenként összeadódó takarításnál. Most közös leállítási határidő,
azonosított bridge + friss canonical lifecycle-készállapot és közös
`status.ready` van. Régi élő gyermek mellett nincs párhuzamos új stack;
csak az igazoltan saját, megszűnt session socketje távolítható el.
Az egyetlen ROS lifecycle owner és a frissességi határidők változatlanok.

## Evidence és korlátok

A motor nélküli TF-összehasonlítás valódi Nav2 node-okkal, 5 Hz-es,
135 ms korú scanből képzett map-TF-fel futott. A 0,25 s-os szakaszban a
parancsintervallum p95 205,7 ms, a mért ráta 13,63 Hz; a 0,50 s-os szakaszban
51,3 ms és 19,83 Hz. A szakaszok eltérő navigációs helyzeteket tartalmaznak,
ezért ez a TF-várakozási mechanizmus bizonyítéka, nem fizikai performance-garancia.
A szintetikus fixture eredeti időbélyeg-objektum megosztási hibája miatt a korábbi
`tf_smoke`/`startup_smoke` eredmény nem acceptance-evidence; a javított,
külön időbélyeget használó `tf_comparison_valid`/`startup_valid` az értékelhető út.

A valódi Behavior Server motor nélküli recovery tesztjében mind a négy node
ACTIVE, a BackUp → Spin → Sequence SUCCESS sorrend teljesült. Hátrálás és a
második Spin közben külön cancel után nulla parancs és késői nem nulla parancs
hiánya igazolt. A motor nélküli host-próbában start → azonos PID-es ismételt
start → stop → új PID-es restart → stop sikeres; az első start 5,27 s,
az első stop 0,75 s. Mindig nulla navigációs sebességjavaslat maradt.

Az új megszakadt fizikai capture:
`runtime/captures/v3_20261113_191546_6030_capture.mcap`.
Az első hibás observation-határ `1682435565377 → 1683990850998 ns`,
1555,286 ms. A canonical lifecycle manager ACTIVE maradt, terminális bridge
hiba nem volt. A bridge később ismét publikált, de a monitor már visszavonta
a próbát. A control max tick 20,42 ms, p99 7,10 ms, végső eredmény
`PASS / SHUTDOWN_SAFE_LOW`, minden mintavett motorkimenet nulla.
Az egyszeri megakadás gyökéroka még nincs bizonyítva. A javított motor nélküli
indításban cleanup 0,72 ms, subscription 3,01 ms, launch 4,83 ms,
maximális lifecycle poll-köz 97,0 ms volt; ez nem reprodukciója az 1,55 s-nak.

Mindkét fizikai MCAP explicit Evidence Compiler útja COMPLETE, source-tal
ellenőrzött verify PASS, nulla quarantine: golden 8908, új próba 587 message.
Az 5 Hz-es capture nem replay-complete, ezért pontos fizikai L1–L12 replay
MATCH nem állítható. A release a persistált NAVIGATE/EXPLORE replay-t is futtatja.

A végső `./r test release` és az érintett Nav2/frontier/runtime/CLI/host
tesztfájlok együtt **300 passed**, 23,72 s alatt; a kötelező gate és a natív
persistált NAVIGATE/EXPLORE replay is sikeres. A source diff whitespace-ellenőrzése
tiszta; a felhasználó korábbi runtime-adatváltozásait megőriztük.
A végső CLI-hibaág javítása a folyamatos cirkálás elutasított hidegindításánál
is felszabadítja az előkészített stacket; sikeres folyamatos futás továbbra is fut.

Az ezután végzett tényleges hardveres, kizárólag IDLE indítás–leállítás próba
PASS: egyetlen `operator.runtime.start(full, 5 Hz)` hívás indította a teljes
konfigurált stacket, 12,54 s alatt. Tizenöt egymást követő ellenőrzésben a
kritikus eszközök egészségesek, Nav2 ready, minden motorkimenet nulla.
Az egyetlen runtime stop a saját Nav2-t és SLAM-et is lezárta; a végső report
`PASS / SHUTDOWN_SAFE_LOW`. Control max 12,01 ms, p99 7,30 ms.
Capture: `runtime/captures/v3_20261113_193202_9321_capture.mcap`, natív integrity
complete/PASS, explicit Evidence Compiler COMPLETE és source-verify PASS,
456 message, nulla quarantine. A capture 34 érvényes Nav2 observationje
tartalmazza az új TF-headroom és controller-error mezőket. Ez a próba nem
kért mozgást, frontier indítást vagy fizikai kiszabadulást.

Részletes származtatott adatok és reprodukciós scriptek:
`tmp/nav-stability-followup-2888b851/`;
`golden_metrics.json`, `baseline_recovery.json`, `tf_comparison_valid/`,
`startup_valid/`, `recovery_smoke/`, `lifecycle_valid/`, `tf_run_1/`, `automatic_idle/`.

Fennmaradó fizikai acceptance: az új lifecycle és recovery együtt végzett,
teljes 60 s-os bejárása, javuló kiszabadulási arány és a SLAM-késési tüskék
megszűnése. A 18:56-os capture megmarad összehasonlítási alapnak.
