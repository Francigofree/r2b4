# Nav2 lifecycle, diagnosztika és Room Cruise — 2026-11-11

## Megvalósítás és authority

A meglévő Nav2 process-supervisor indítja és felügyeli a Planner Servert,
Controller Servert, BT Navigatort, lifecycle managert és bridge-et. A Nav2 ROS
életciklusának egyetlen ownere a `nav2_lifecycle_manager`: rendezett autostart,
bondok, kieséskor deactivation. A bridge-ek nem hívnak `ChangeState`-et vagy
`ManageLifecycleNodes`-ot. A manager saját respawn-reconnection köre kikapcsolt;
a meglévő process-felügyelet megmaradt.

A `diagnostic_updater` a ROS edge-ben figyeli a komponensállapotot, a tényleges
cmd_vel frekvenciát és kimaradásokat, a scan/odom/TF időket és navigációs hibákat.
A `/diagnostics` legfeljebb 16 státuszát és státuszonként 8 értékét tartjuk meg,
explicit truncation számlálóval. A meglévő atomikus bridge-status 5 Hz-es,
bounded latest snapshotját az observation/capture sidecar olvassa, és az
ObservationHub `v3.nav2_observation` topicjára küldi. A `/diagnostics` nem kerül
a control inputjába vagy nagy payloadként a control interpreteren keresztül.
A kiesés, snapshot-kihagyás és friss adat visszatérése explicit jelölést kap.

A V3 minden normál ciklusából konstans méretű számlálók készülnek. Az 5 Hz-es
capture tick az összes köztes periódus számát/összegét/maximális idejét, valamint
külön control- és pipeline-wall idejét rögzíti. A végső részablak is kiürül.
Az 5 Hz-es minták közti ~200 ms önmagában nem a robot vezérlési ciklusideje.

Két kért komponens települt az izolált Jazzy prefixbe: lifecycle manager
1.3.13 és diagnostic_updater 4.2.7, a diagnostic_msgs 5.3.8 függőséggel.
Meglévő csomag nem frissült. Az environment YAML és az explicit aarch64 lock
szinkronban van; egyéb új navigációs komponens nem került telepítésre.

## Fizikai geometria és safety

A felhasználó által megadott test 36 × 36 cm. A hajtott tengely az origó:
`x ∈ [-0.10, 0.26]`, `y ∈ [-0.18, 0.18]` méter. A test középpontja
`x=0.08 m`, a LiDAR `x=-0.005 m, y=0`; ezek már helyesen szerepeltek a
`fizika.json`-ban. Független új fizikai méretmérés nem történt.

Az L12 hivatalos clearance-e most **0.10 m a fizikai testtől**, a
`layers.lidar_safety.minimum_clearance_m` authority alatt. A fizikai scan edge
az eredeti nyers returnökből, a LiDAR-extrinsic figyelembevételével számolja a
téglalaptól mért euklideszi távolságot. `BODY_OUTLINE` mértékjelölés halad a
typed safety mintával; hiányzó/eltérő mértékalap fail-closed. A sarok-return
mindkét érintett testoldalhoz tartozik; fordulásnál L12 minden testoldalt vizsgál.
A source sequence, scan-midpoint és freshness nem változik. A korábbi
capture-ök default mértékalapja `LIDAR_ORIGIN`, így történeti replayjük nem
értelmeződik át utólag test-clearance-re.

A Nav2 a feloldott fizikai poligont és legalább 10 cm paddinget kapja mindkét
costmaphez. A korábbi generált kör sugara ~0.366 m volt, inflációja ~0.466 m;
ez főleg oldalt és hátul jelentősen túlbecsülte a testet. A poligon körül a
padding után az oldalirányú teljes szélesség 0.56 m. A lokális Nav2 collision
check továbbra is a teljes footprintet vizsgálja, a V3 L9 és L12 változatlanul
végső fizikai korlátot érvényesít. A STOP nulla output és a canonical motorlánc
megmaradt.

Az inflation radius mindkét costmapen 0.45 m, a cost scaling 6.0. Ugyanez a
scaling kerül RPP-be; a cost-based lassítás távolsága 0.45 m. A régi 0.30 m-es
lassítási távolság kisebb volt a generált kör inscribed sugaránál: a költség
alapú lassítás és az ütközési határ nem alkotott használható átmenetet.
A sebességfüggő lookahead 1.2 s, 0.30–0.50 m korláttal; a kanyarlassítás
sugara 0.50 m. Ezek kezdeti, fizikailag konzisztens hangolások, nem minden
szobára igazolt optimumok. A controller 20 Hz, local costmap 10 Hz maradt:
a motormentes mérés bizonyította, hogy a 10 Hz-es costmap önmagában nem
korlátozza 10 Hz-re a vezérlőt.

## Felderítés és folyamatos szobabejárás

A canonical EXPLORE mission a ROS edge-ben két külön fázist használ.
`FRONTIER` alatt az explore_lite keresi az új területeket; az elfogadott aktív
célt egy új centroid-javaslat nem szakítja meg. A tényleges Nav2 success/failure,
explicit cancel és missionváltás feloldja a cél megtartását.

`No frontiers found` / `exploration_complete` után `COVERAGE` következik.
A célválasztó a meglévő Nav2 globális costmap legfeljebb 1024 tile-jából,
tile-onként legfeljebb 9 költséglekérdezéssel választ. Útvonalat nem keres:
a reachability és collision továbbra is a Nav2 feladata. Bounded bejárási,
friss cél- és sikertelen cél-history alapján előnyben részesíti az újabb,
változatosabb és enyhén előre eső területeket. Map-frissítés nem cserél aktív
child goalt. A köztes célok toleranciája 18 cm, végső yaw-beállítás nélkül;
a konkrét NAVIGATE pontossági követelménye változatlan. Sikeres érkezés után
nincs mesterséges 150 ms-os várakozás. A tényleges action terminal és az új
action admission közötti kötelező handshake továbbra is rövid STOP-ot okozhat.

## Gyökérokok és bizonyítékok

Az előző fizikai capture célzott elemzése:
`tmp/prechange-root-cause-vdhen3ok/summary.json`.

| Jelenség | Konkrét evidence / korlát |
| --- | --- |
| Felesleges STOP–START | Tick1601 L6 ACTIVE→PENDING, előző javaslat még 29.540 ms-ig érvényes, L12 ALLOW és quality GOOD. Tick1602 ugyanaz a child folytatódik, L9 csak 0.00741 m/s-ról indul. A bridge 100 ms-os PENDING-fallbackje forrásalapú magyarázat; a pontos UDP packet nincs rögzítve. |
| Célváltási akadozás | Öt cancellation kb. 2 másodpercenként; az első három új cél csak 4–5 cm-rel változott. Aktív frontier commitment szünteti meg ezt. |
| Eltűnő cirkálás | Frontier-elfogyás után NAV2_NO_PATH; most normál COVERAGE fázis. |
| Sikertelen utak | Két Navfn cél sikertelen. Az eredeti costmap cellái hiányoznak a baseline capture-ből; konkrét inflation-cellát vagy fizikai átjárhatatlanságot nem bizonyítunk. |
| Régi L12 0.40 m | A beállítás valóban létezett, de a baseline aktív megállásait nem ez a safety-határ okozta. A geometriai túlbiztosítás és a ténylegesen rögzített STOP oka külön állítás. |
| Nav2 ~10 Hz | A baseline napló 92 elmulasztott 20 Hz-es ciklusa, figyelmeztetési medián 10.061 Hz. Ez nem a teljes command stream becslése. |

A PENDING-fallback most nem írja felül a még érvényes, ugyanazon aktív child
eredeti sebességjavaslatát. Nem küld újra és nem hosszabbít timestampet/TTL-t.
Lejárat, terminal, cancel, szenzorhiba vagy child-váltás továbbra is HOLD/STOP.

A valódi, motormentes Nav2 TF-kontrasztmérés:
`tmp/room-tf-horizon-smoke-20261111/result.json`.
Az aktív SLAM 5 Hz körüli scan-admissionje és a scan-időhöz adott régi +100 ms
TF-horizont a legújabb odometria transzformálásánál szinkron várakozást okozott.
Ezt modellezve a controller ~10–11 Hz lett; a CPU-keret bővítése nem oldotta meg.
Ugyanezen 5 Hz-es forrás +250 ms véges horizonttal ~20.001 Hz-et adott.
Ezért a SLAM `transform_timeout` most 0.25 s, `restamp_tf=false` maradt.
A fizikai scan/odom/global-fix időbélyegek és a külön V3 freshness gate-ek
nem újulnak meg a TF-publikálással.

A kerékminimum további fizikai korlát: a kalibrált minimum 0.15 m/s, a target
reserve 0.19 m/s. Például a `.19 m/s, .5 rad/s` javaslat wheel-feasible
vetülete `.19 m/s, ~.225 rad/s`: az alacsony belső keréksebességet nem
amplifikáljuk jogosulatlanul. Ez laposabb ívet eredményezhet. A Nav2 adapter
az egzakt vetített ívet újra collision-checkeli; L9 továbbra is a fizikai
gyorsítási és mission-korlátok ownere. A minimumnál lassabb folyamatos haladás
nem állítható be pusztán egy RPP szám csökkentésével.

## Szoftveres és motormentes validáció

- A lifecycle-, body-corner safety-, navigation- és diagnosztikai javítások
  utáni release és célzott tesztek: **83 passed**. A végső native
  sebességlimit-javítás után `./r test release
  tests/packs/feature/test_nav2_wheel_adapter.py`: **27 passed**. A release
  a kötelező kis robot-contract gate-et is tartalmazza; a két native
  NAVIGATE/EXPLORE persistált replay teszt egyezett.
- Valódi lifecycle manager: planner/controller/BT ACTIVE, mindhárom bond,
  `/diagnostics`; managervesztésre fail-closed bridge. Evidence:
  `tmp/nav2-lifecycle-full-f4baizbo/result.json`.
- Valódi, motormentes coverage: 3 különböző cél, 2 elérés, nulla hiba és
  cancellation. Evidence: `tmp/room-coverage-smoke-20261111/result.json`.
- Valódi frontier admission: eltérő új cél rejected az aktív megtartásával;
  success/cancel után következő cél admitted. Evidence:
  `tmp/frontier-commitment-smoke-20261111/result.json`.

## Fizikai evidence

Az első ideiglenes preflight-monitor a nem igényelt kamera/detektor UNKNOWN
állapotára is leállt; ez monitorhiba volt. A canonical kritikus eszközlista
szerinti javítás után 50 állóhelyzeti státuszminta egészséges volt, motor-output
végig nulla. Forrás: `v3_20261111_101605_11413_capture.mcap`.
Átlagos ciklus 20.019 ms (~49.953 Hz), p99 23.8 ms, max 35.36 ms.
Control p99 7.4 ms; egy 20.557 ms-os outlier volt. Az L3 wall max 17.302 ms,
az L3 thread-CPU max 4.104 ms: legalább ~13 ms nem CPU-végrehajtás volt.
Ebből önmagában a GIL és az OS scheduling nem választható szét.

A preflight MCAP natív integritása PASS, replay_complete=true. Natív replay
850–860: **MATCH**, 11 kiválasztott tick, 41 bemelegítő tick, ismételt L1–L12
egyezés. Explicit Evidence Compiler COMPLETE, 1696 üzenet, egy worker.
Artifactok: `tmp/nav2_room_20261111.ffISVv/preflight_confirmed/`.

A kért 30 s / FULL5 Hz első mozgáspróbáját a passzív monitor megszakította
`NAV2_LIFECYCLE_STATUS_STALE` miatt. A manager a napló szerint aktív maradt,
a bridge kiesését a process-supervisor felismerte. Canonical STOP és shutdown
végrehajtva; automatikus mozgásismétlés nem történt. A felhasználó ezután
kifejezetten új engedélyt adott a live tesztek folytatására.

Forrás: `v3_20261111_101904_11737_capture.mcap`;
artifactok: `tmp/nav2_room_20261111.ffISVv/cruise/`.
A V3 1556 normál tickje átlagosan 20.024 ms (~49.939 Hz), max 34.761 ms;
control p99 10.8 ms, max 15.019 ms; work-over-period nulla.

Az offline döntésintervallumok szerint az első próba aktív missziója 15.660 s,
ebből nem nulla motorkimenet 8.554 s. Az első mozgásig 5.280 s admission várás
volt. Az első mozgás és az utolsó megállás közötti szakaszon kilenc rövid
megszakítás összesen 445.8 ms, a nem nulla command aránya ~95.05%.
Egy child goal maradt aktív, négy új frontier-javaslat rejected; periodikus
célváltási akadozás nem volt. A mért controllerfrekvencia mediánja 19.945 Hz.
Az odometriából/TF-ből becsült út 2.319 m, a raszterezett testnyom 1.02 m².
Ez nem külső fizikai ground truth. Az első belső STALE a korábbi érvényesség
valódi, 17.363 ms-os túllépése volt; nem a már javított PENDING-felülírás.
Az eredeti teljes navigation input hiányában a konkrét scan/odom/TF határ
nem különíthető el bizonyítottan. Részletek:
`tmp/room-motion-analysis-y9x4wbwi/report.md`.

A terminális kiesés előtt a bridge ~1 s-ig nem dolgozott fel friss bemenetet;
az utána olvasott heartbeat-ek egy része már lejárt. A kritikus szenzorok és
a V3 lokalizáció eközben GOOD/OK maradtak. Az installed FastDDS service-response
útja a válaszolvasó discovery-egyezésére szinkron módon várhat. A tényleges
RMW ugyanazon ágának motormentes, injektált discovery-állapotú reprodukciója
100.145 ms-os várást adott; a service XML-profil 10.118 ms-ra korlátozta.
Ez az adott blokkolási mechanizmust bizonyítja, a teljes fizikai 1 s-os stall
minden részét nem tulajdonítja egyetlen service hívásnak. A javítás csak a
Nav2 edge szolgáltatásválasz-íróinak blokkolási idejét korlátozza; nem vált
DDS-implementációt és nem lazítja a lifecycle/szenzor deadline-okat.

A későbbi folytatáskor a gép új bootban volt, a forrásfa új HEAD-re került.
A fenti két eredeti MCAP ekkor már nem volt elérhető a captures könyvtárban;
a korábban elkészült inspect/replay riportok és explicit EVI-bundle-ök
megmaradtak. Ezek történeti bizonyítékok, az eredeti fájlok friss, ismételt
hash-ellenőrzése ebben az állapotban nem lehetséges. Monoton időket különböző
bootok között nem hasonlítunk össze. A felhasználó későbbi working-tree és
konfigurációs változásait a folytatás megőrzi.

## Megújított engedély utáni 30 s / 5 Hz próba

A `30e2140c93ef4390bd6b4beb625c3f2f` skill `SUCCEEDED`, eredménye
`COMPLETED / REQUESTED_DURATION_REACHED`. A rögzített végrehajtási ablak
29.994994 s; az L5 mission 34.440594 s aktív ideje a kezdeti 4.204734 s
admission-várást is tartalmazza. Forrás és megőrzött másolata:

`tmp/nav2_room_20261111.ffISVv/cruise_after_dds_fix/v3_20261111_152328_4069_capture.mcap`.

| Mérőszám | Eredmény |
| --- | --- |
| Nav2 command rate, gördülő ablakok mediánja | 19.448 Hz; p95 ~19.999 Hz; nincs tartós 10 Hz-es korlátozás |
| Lifecycle query timeout | 0; legnagyobb bridge-poll kimaradás 148.955 ms |
| Frontier → coverage | Fizikailag megtörtént; 2 coverage cél, 1 elérés, 0 sikertelen cél |
| Becslésből számított út / testnyom-unió | 5.001 m / 2.03 m², 10 cm raszter |
| Mintavett legkisebb test-clearance | 0.1742 m; a 0.10 m-es fizikai STOP-határ közeli, független tesztje nem történt |
| Belső nulla-output epizódok | 14 epizód, összesen 1.3935 s; egy részük valódi stale/terminal/új goal handshake |
| Tartós végső HOLD | 11.6797 s, NAV2_LOCALIZATION_HOLD |
| V3 teljes capture periódus átlaga | 20.1315 ms, ~49.673 Hz |
| V3 periódus p99 / max | 26.0 ms / 41.9139 ms; kemény 20 ms deadline nem bizonyított |
| Control wall p99 / max | 12.7 ms / 25.408 ms; 7 work-over-period esemény |
| Nav2 action error / upstream command-stop / mért stop | 0 / 4 / 3 |
| Nav2 observation mintaköz medián / max | 201.366 ms / 222.586 ms |
| Becslésből számított sebességcsúcs / forgási sebességcsúcs | 0.5925 m/s / 2.4520 rad/s; ezek becslések, a motoros követés külön vizsgálandó |

A CPU-affinity az aktuális, felhasználói konfigurációban ki volt kapcsolva;
ezt a folytatás megőrizte. A control így nincs dedikált CPU-ra rögzítve.
A pipeline thread-CPU max ~4.82 ms, wall max ~22.78 ms; ez nem CPU-n töltött
időt is bizonyít, de önmagában nem választja szét a motor-I/O-t, GIL-várást
és OS preemptiont. A motor-write és a többi régió külön wall/CPU mérhető.
A korábbi boot/konfiguráció méréseivel ebből nem állítunk szigorú performance
nem-regressziót.

A tartós HOLD első pontos decision eventje **tick1630**,
`1038261072238 ns`: a local/global position GOOD és a források frissek,
a heading DEGRADED. Az accumulated local-vs-relative yaw residual
`0.10855824 rad`, ami nagyobb a `relative_yaw_slip_rad * 0.5 = 0.10 rad`
küszöbnél. A friss interval residual csak `0.01237998 rad`, yaw variance
`~0.02168 rad²`, IMU-age ~8 ms; az aktuális covariance/freshness ezért nem
magyarázza ezt a DEGRADED ágat. L6 erre explicit localization HOLD-ot ad,
L8/L9 nulla mozgást, L12 ALLOW/zero outputot rögzít. Nem costmap- vagy
L12-távolság miatti STOP, és nem lifecycle-kiesés.

Az eredeti MCAP 545 egyesített tick/decision sorának és 316 eltérő accepted
relative sequence-ének célzott ellenőrzése nem mutatott ismételt
újraszámolást vagy megfigyelt intervallumátfedést. A 192 közvetlenül egymást
követő sequence-pár mindegyikén pontosan teljesült a source-ban szereplő
`accum_new = exp(-interval_s / 30) * accum_old + interval_error` összefüggés.
A küszöbátmenet: seq313 `0.07947688`, seq314 `0.09649940`, seq315
`0.10855824 rad`; a két utolsó intervallum ~100 ms. Az összeg ezután még
`0.16531120 rad`-ig nőtt, és az álló szakaszon is csak a 30 s-os memória
szerint csökkent. Ez megmagyarázza a tartós HOLD-ot. A megfigyelt párokon
nincs duplázásra vagy receipt-time téves használatára utaló jel; a scope-ból
hiányzó köztes nyers mérésekre ez nem állít teljes ellenőrzést.
Artifact: `tmp/nav2_room_20261111.ffISVv/cruise_after_dds_fix/heading_accumulation_check.json`.

A physical gyro-bias, helyi odometria és LiDAR-regisztráció hozzájárulása a
5 Hz-es behavior scope raw inputjai nélkül nem választható szét biztosan.
A quality-gate megkerülése vagy küszöbének lazítása nem történt; folyamatos,
egyenletes 30 s-es haladás ezért még **nem elfogadott**. Az elsődleges további
probléma az összegzett yaw-eltérés és az emiatt tartós HOLD.

A sebességprofil külön adapterhibát is feltárt: a 0.30 m/s névleges RPP targetet
a bridge 0.45 m/s mission-ceiling üzenete upstream abszolút targetként emelte
meg. A native adapter korrigálja ezt: abszolút limit legfeljebb a névleges
sebességet engedi, százalékos limit legfeljebb 100%, NO_SPEED_LIMIT visszaállítja
a névleges targetet. A kerékprojekció, collision-check és L9 fizikai rámpa
változatlan. A fenti fizikai mérés ezt az utolsó adapterkorrekciót megelőzi;
utána az adapter újrafordult, a valódi, motormentes ROS-controller ellenőrzése
sikeres: 0.45-ös abszolút limit mellett 0.30 m/s, 0.20-as limit mellett
0.20 m/s, restore után 0.30 m/s; 50%-nál 0.15 m/s, 150%-nál 0.30 m/s.
Összesen 66 command minta készült. Evidence:
`tmp/nav2-speed-limit-controller-9ph4elti/result.json`.
Az utolsó javítás utáni fizikai sebességkövetés nincs igazolva. A fenti
becsült sebességcsúcsok nem független mozgásmérésből származnak; az
odometria/iránybecslés és a motoros követés hozzájárulását külön kell mérni.

Az MCAP natív integritása PASS. Az explicit Evidence Compiler COMPLETE,
2311 üzenet, nulla quarantine; a megőrzött eredeti source-fájllal az explicit
verify PASS (`source_checked=true`). A normál 5 Hz scope nem rendelkezik raw
input/checkpoint/replay-complete tickstreammel, ezért teljes fizikai replay
MATCH nem állítható. A korábbi IDLE replay-MATCH külön bizonyíték marad.

Részletes export és mérési riport: `tmp/room-final-analysis-i1gwpeqx/`.
A mintavett sebességprofil CSV-je és önálló ábrája:
`tmp/nav2_room_20261111.ffISVv/cruise_after_dds_fix/speed_profile.csv`,
`tmp/nav2_room_20261111.ffISVv/cruise_after_dds_fix/motion_profile.png`.

![Sebesség, távolság és ciklusidő; sárga: lokalizációs HOLD](../../tmp/nav2_room_20261111.ffISVv/cruise_after_dds_fix/motion_profile.png)

## Elsődleges külső források

- [Nav2 1.3.13 lifecycle manager forrás](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_lifecycle_manager/src/lifecycle_manager.cpp)
- [Nav2 Jazzy RPP beállítások](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [Nav2 1.3.13 controller loop](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_controller/src/controller_server.cpp)
- [SLAM Toolbox 2.8.5 TF-publikáció](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)
