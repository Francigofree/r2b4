# Nav2 runtime-edge és életciklus

Az aktuális teljes Nav2 út külön Planner Server, Controller Server, Behavior
Server és BT Navigator folyamatokat használ. A Nav2 kizárólag navigációs javaslatot ad;
a mission és a motor-authority a canonical V3 láncban marad. Az aktív
`runtime.nav2_navigation` és a korábbi `runtime.nav2_planner` mód kizárják
egymást. A Room Cruise frontier-felderítés után a már ismert szobában is
folytathatja a térképalapú célválasztást.

## Egyetlen ROS lifecycle owner

A `tools/nav2_planner.py` meglévő process-supervisora továbbra is indítja,
azonosítja és leállítja a gyermekfolyamatokat, valamint gyermekhalálnál lezárja
a teljes Nav2 stacket. A ROS configure/activate/deactivate műveletek egyetlen
ownere a felügyelt `nav2_lifecycle_manager` folyamat, `autostart=true` mellett.
Teljes módban a kezelt sorrend `planner_server`, `controller_server`,
`behavior_server`, `bt_navigator`; planner-only módban csak `planner_server`. A 2 s bond-timeout
aktív; az önálló manager-respawn/reconnection kikapcsolt, ezért nincs második
helyreállítási vezérlő a process-supervisor mellett.

Egyik Python bridge sem hív lifecycle `ChangeState` vagy `manage_nodes`
műveletet. Kizárólag a manager `/lifecycle_manager_navigation/is_active`
`std_srvs/Trigger` szolgáltatását olvassa, 2 Hz-en, legfeljebb egy függő kéréssel
és 1 s válasz-határidővel. Induláskor 25 s az aktiválási határidő. Korábban
aktív stack inaktív válasza vagy 1,5 s óta nem friss státusza bezárja a bridge-et,
amit a meglévő process-supervisor gyermekhibaként kezel. A régi sikeres startup
nem marad korlátlan ideig readiness-bizonyíték. A supervisor és bridge státusza
`lifecycle_owner=nav2_lifecycle_manager` mezőt közöl.

Az `./r rc 60 c full` maga készíti elő a konfigurált stacket: SLAM → resident
runtime → Nav2, friss készállapotra várva, majd a canonical behavior/command
úton kéri a 60 másodperces cirkálást, 5 Hz-es capture-rel. A hidegindítás nem
fogyasztja a végrehajtási időt. A runtime host-supervisora kilépéskor a saját
Nav2-, majd SLAM-folyamatát is leállítja. Korábban külön indított szolgáltatást
nem vesz át és nem állít le. A `./r rt start c full c 5` ugyanezt a stacket
készíti elő mozgáskérés nélkül; a `./r sd` leállítja a runtime-ot és saját
függőségeit. A külön `slam`/`nav2` parancsok diagnosztikai használatra megmaradnak.

Az ismételt Nav2-indítás is friss, azonosított bridge- és lifecycle-készállapotot
vár. A gyermekek leállítására közös határidő vonatkozik; kényszerített leállítás
után csak a bizonyítottan saját, már nem élő session socketje takarítható el.

A Nav2 edge külön `conf/nav2_fastdds.xml` service-writer profilt tölt be.
A reliable válasz továbbra is reliable; a még fel nem fedezett vagy megszűnő
action-válaszolvasóra várás maximuma 10 ms. Az installed RMW 8.4.4
[szinkron válaszútja](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)
ezt a writer-limitet használja. A motormentes, hiányzó response-reader állapotot
reprodukáló mérésben az alap 100 ms-os várás 100,145 ms-ot, a profil 10,116 ms-ot
vett igénybe. A profil csak a Nav2 processzkörnyezetére hat; a lifecycle és
command freshness határidők nem változtak. A bridge-státusz és az MCAP
`nav2_observation.lifecycle_status` fix skalármezői külön mutatják a query idejét,
az utolsó válasz korát, a timeoutokat és a legnagyobb poll-közt.

A manager saját `diagnostic_updater` publishere 5 Hz-en ad `Nav2 Health`
státuszt a `/diagnostics` topicra. Az alkalmazott szolgáltatás és bond-viselkedés
az [installed verzióval azonos Nav2 1.3.13 forrásban](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_lifecycle_manager/src/lifecycle_manager.cpp)
ellenőrizhető.

## Függőségek és reprodukálhatóság

Az izolált `runtime/deps/nav2-planner` prefixhez a Jazzy/RoboStack környezetből
`ros2-nav2-lifecycle-manager=1.3.13`, `ros2-diagnostic-updater=4.2.7` és az
utóbbi interfészfüggősége, `ros2-diagnostic-msgs=5.3.8` került hozzá. A telepítés
a meglévő csomagokat befagyasztva futott; nem cserélt le vagy frissített csomagot.
A `deploy/nav2-planner-environment.yml` deklarálja a kérést, a
`deploy/nav2-planner-linux-aarch64.lock` a teljes telepítést URL és MD5 alapján
rögzíti. Friss prefix reprodukálása:

```bash
micromamba create --prefix runtime/deps/nav2-planner --file deploy/nav2-planner-linux-aarch64.lock
```

A launcher a diagnosztikai fizikai körvonalat és LiDAR-extrinsiceket ugyanabból
a resolved konfigurációs snapshotból adja a bridge-nek, mint amelyből a Nav2
geometry-paramétereket készíti. A ROS2-modulok és a teljes nyers térkép nem
lépnek be a control interpreterbe.

## Korábbi planner-only integráció és evidence

Az alábbi eredeti leírás a korábbi waypoint-planner módra és annak 2026-11-07/08
bizonyítására vonatkozik; nem a jelenlegi teljes Nav2 production wiringja.

A Nav2 kizárólag globális útjavaslatot számol. A robotot továbbra is a V3
CommandGateway → L5 → L6 → L7 → … → L12 → MotorWriter lánc vezérli.
Nincs Nav2 Controller Server, BT Navigator vagy ROS `cmd_vel` motorbekötés.
A Room Cruise (`EXPLORE`) a meglévő lokális V3 tervezőt használja.

## Telepítés és indítás

A ROS-függőségek külön Python/prefix környezetben maradnak:

```bash
micromamba create --prefix runtime/deps/nav2-planner --file deploy/nav2-planner-linux-aarch64.lock
./r slam start
./r runtime start c full c 50
# Várd meg az IDLE readiness-t és a friss /map + map→base_link TF-et.
./r nav2 start
./r nav2 status
./r nav2 stop
./r sd
./r slam stop
```

A `start` a Planner Server és a ROS bridge életciklusát kezeli, mozgásparancsot
nem küld. A SLAM-től érkező `/map` és map → base_link TF szükséges a
Planner Server aktiválásához. A planner globális costmapje static és inflation
rétegből áll; a robot sugarát és az inflation sugarát a launcher a resolved
V3 footprintből származtatja. A planner a feltérképezetlen cellákon nem tervez.
A fenti runtime-indítás explicit mozgásparancs nélkül IDLE marad, motormentes
map/TF inputot szolgáltat. A Nav2 indítása előtt a readiness szükséges;
map/TF hiányában az aktiválás 15 s után időtúllépéssel leáll.
A ROS supervisor és gyermekei nice 10 prioritáson, egy control CPU-n kívüli
háttér CPU-ra korlátozva futnak.

## V3 konfiguráció

Az aktív `conf/vezerles.json` bekapcsolja a globális NAVIGATE Nav2-adapterét.
A `runtime.nav2_planner` értéke:

```json
{
  "socket_path": "/home/alba/project_r2b4/runtime/nav2_planner.sock",
  "request_timeout_s": 0.18,
  "max_map_age_s": 6.0
}
```

A sidecart a fenti `./r nav2 start` indítja. Elérhetetlen Planner Server mellett
a globális NAVIGATE HOLD-ot ad; Room Cruise ettől függetlenül a helyi V3
tervezőt használja. Az adapter `null` értékkel kikapcsolható.

Ehhez SLAM Toolbox backend és aszinkron, lezárt completion-inputos L6 szükséges.
A resolver a globális planner elérhetőségi követelményét a replayhez is rögzített
navigation policyba származtatja. A `request_timeout_s` kisebb az L6
számítási határidejénél; a teljes worker futásidőbe a helyi V3 rollout is
beleszámít. A bridge legfeljebb 1 s kéréshatáridőt és 30 s térkép-életkort enged.

Csak map-frame `NAVIGATE` küld külső tervezési kérést. A meglévő L6 worker
legfeljebb egy datagram-válaszra vár, határidővel; a control interpreter nem
importál ROS-t és nem vár a ROS actionre. A teljes térkép és ROS út a bridge-ben
marad. A V3 egyetlen odom-frame waypointot, request/source contextet,
odom-generationt, transform-revisiont és kis státuszadatot kap.

A waypoint az út első, korlátozott hosszúságú szakaszából származik. A V3
ugyanazzal a lezárt lokális geometriával, teljes footprinttel és meglévő
kinematikai korlátokkal számolja újra a helyi mozgásjelölteket. A Nav2-javaslat
nem igazolja önmagában az akadálymentességet vagy a globális célbaérést.

Az első globális tervig HOLD következik. Hiányzó szolgáltatás, késés, hibás
frame/lineage, elavult térkép vagy út hiánya nem vált közvetlen egyenes
tervezésre. Már elfogadott lokális szakasz csak a meglévő bounded és friss
lokális geometriai igazolás szabályai szerint tartható meg pending alatt.

## Idő és visszajátszás

A request source contextje és a V3 geometriai measurement lineage változatlan.
A `map_stamp_ns` a ROS térkép-publisher bélyege, nem fizikai LiDAR-mérési idő;
a `map_revision` a bridge által a kéréskor látott map-üzenet sorszáma. Ezek
passzív külső source-adatok, nem V3 lokális geometriai bizonyítékok.

A lezárt `GlobalPlanGuidance` része a planner completionnek. Az offline
`compute_recorded_planner_result` ebből ugyanazt a V3 rolloutot és collision
ellenőrzést számolja újra, ROS vagy változó globális térkép nélkül. Ez a V3
helyi döntés reprodukálhatóságát bizonyítja; a Nav2 algoritmus változó
térképből történő újbóli globális tervezését nem.

## Elvégzett bizonyítás

2026-11-07-én a külön ROS domain 75-ben futó valódi Jazzy Planner Server
szintetikus OccupancyGrid+TF bemenetről kerülő utat adott; a V3 helyi rollout
négy akadálymentes és progress-képes jelöltet igazolt. A teljes kliens+rollout
19,7 ms volt ezen az egy példán, a végleges CPU-korláttal és footprint-sugárral.
Foglalt cél `NO_PATH`, elavult térkép `STALE`
eredményt adott, mindkettőnél nulla érvényes mozgásjelölttel. A lezárt
javaslatokból a helyi pure újraszámolás azonos volt. Motor-output nem történt.
Ez nem bizonyít valós helyiségben teljesített Nav2 NAVIGATE missiont vagy
általános timing határértéket. A célzott fejlesztői teszt:

```bash
./r test tests/packs/feature/test_nav2_planner.py
```

2026-11-08-án a valós robot motormentes IDLE állapotában, a tényleges SLAM
18 000 cellás térképével és a V3 valódi GOOD pozícióminőségével a Nav2 READY
útjavaslatot adott. Az izolált worker és a helyi geometriai kernel együtt
50,14 ms alatt végzett, 26 ütközésmentes jelölttel; a lezárt guidance-ból
végzett helyi újraszámolás egyezett. A diagnosztikai lokális geometria a SLAM
gridből, bounded környezetben készült, mert a státusz nem exportálja a teljes
production L4-et. A mérési freshness ismeretlenként maradt, ezért ez nem
production L6 motion-authority vagy végrehajtott NAVIGATE mission bizonyítása.
A [mérési jelentés](analysis/20261108_nav2_live_roomcruise.md) ezt külön jelöli.

A Planner Server action és Jazzy konfiguráció elsődleges forrásai:
[ComputePathToPose](https://api.nav2.org/actions/jazzy/computepathtopose.html),
[hivatalos Jazzy paraméterek](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/params/nav2_params.yaml).
