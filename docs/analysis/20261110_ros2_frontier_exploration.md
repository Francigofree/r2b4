# ROS2 frontier exploration — 2026-11-10

A Room Cruise saját `FrontierGoalSelector` implementációját az upstream
[`m-explore-ros2` explore_lite](https://github.com/robo-friends/m-explore-ros2/tree/326cf8a0b487c34246bb8f3326afbcd69576dc60)
váltja fel. A régi Python célválasztó és annak implementációtesztje törölve lett.
A rögzített forrásverzió telepítése: `deploy/build_frontier_exploration.sh`.

Az explorer a Nav2 globális costmapjét dolgozza fel külön ROS-folyamatban.
Saját `NavigateToPose` kliense kizárólag a bridge
`/r2b4/frontier/navigate_to_pose` céljavaslat-actionjéhez kapcsolódik. A valódi
Nav2 actiont továbbra is a navigációs bridge hívja, friss map/scan/odom/TF és az
alkalmazott mission-korlátok ellenőrzése után. A cél frame-je, eredeti időbélyege
és síkbeli geometriája ellenőrzött; idegen behavior tree nem fogadható el.

A canonical EXPLORE kérés elindítja az explorer inicializálását, így a ROS
discovery nem vár a célküldési kapukra. Missionváltás, canonical cancel vagy
heartbeat-kiesés visszavonja a Nav2 actiont és megszünteti az explorer folyamatát.
Új generation csak az előző processz reapje után indulhat. A státusztopic
generationönként külön névteret kap. Az upstream preemption az előző cél
sebességét azonnal érvényteleníti; az új cél csak a régi Nav2 action lezárása
után küldhető. Processzhiba és a bounded stop/cancel timeout fail-closed.

A V3 felé ugyanaz a kis, immutable `Nav2VelocityGuidance` jut: mission/goal,
provider generation, sequence, scan/odom mérésidő, transform/odom lineage,
kiválasztott cél, map revision és child goal identity. A control interpreterbe
nem kerül ROS, costmap vagy frontier-keresés. Az L6–L12 és a canonical command
és motor authority nem változik. A `global_costmap.always_send_full_costmap`
biztosítja a később feliratkozó explorer számára is a teljes grid publikálását.

Az explorer frontierhiány esetén befejezi a keresést. A korábbi célválasztó
teljesen feltérképezett szobában alkalmazott coverage fallbackje megszűnt.

## Motormentes evidence

- Build/install, dinamikus függőségek és ROS message type support: sikeres.
- `./r test release tests/packs/feature/test_ros2_frontier.py`: 60 sikeres teszt.
- A legutóbbi célzott ingress/frontier teszt: 52 sikeres teszt.
- `python3 -m tools.nav2_navigation_probe --frontier-only`: valódi upstream
  explorer és Nav2, külön ROS domainben. A legutóbbi eredmény PASS,
  `tmp/nav2_probe__zmi4cva/summary.json`; frontier-cél és lineage, pozitív ACTIVE
  guidance, canonical cancel, heartbeat STALE, nulla szimulált mozgás és teljes
  processzreap bizonyítva. Ez nem fizikai mozgásevidence.

Az első szintetikus próba a kezdeti teljes costmap hiányán várakozott; a teljes
grid periodikus publikálása után a próbák sikeresek voltak. A fizikai IDLE
read-only vizsgálat (`tmp/frontier_ros_readonly.THNlbd/diagnostic.json`) aktív
Nav2 node-okat, kompatibilis QoS-t, friss TF-et és közel 1 Hz-es teljes costmapet
igazolt. Általános live costmap-publikációs hiba ebből nem állítható.

## Fizikai próbák

Az első 10 s kérés `REQUESTED_EXECUTION_UNPROVEN` eredménnyel, Nav2 mozgásjavaslat
nélkül zárult. Az eredeti capture:
`runtime/captures/v3_20261110_192251_8053_capture.mcap`.
A kérés +0,844 s-nál már megjelent; +4,024–5,964 s között global-position
DEGRADED miatti lokalizációs HOLD volt. Az explorer csak körülbelül +6,23 s-nál
indult, és az admission lejártáig 3,96 s maradt inicializálni. Az akkori explorer
costmap-kézbesítésének pontos oka nem bizonyított. Az inicializálás korábbi
indítása mellett a bridge a request frissességét immár a tényleges fogadás utáni
monoton idővel ellenőrzi, a korábban mintavett loop-idő helyett.

A következő IDLE mérésben L3 fault jelentkezett; ennek vizsgálata és a végleges
fizikai elfogadás eredménye még kiegészítendő. Sikeres 10 s fizikai végrehajtás
ebben az állapotban nem állítható.
