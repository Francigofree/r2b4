# Nav2 Planner Server első integráció

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
