# Navigációs stabilitás — 2026-11-13

A heading történeti drift miatti tiltása javítva; a Nav2 Behavior Server
integrálva és motorút nélküli ROS-próbával ellenőrizve. A fizikai elfogadás
**nincs kész**: az első 30 s / 5 Hz room cruise kérés még mozgás előtt
LiDAR-hibával megszakadt. Automatikus fizikai ismétlés nem történt.

A változtatás az L3 minősítésre és a meglévő runtime-edge adapterekre
korlátozódik. A V3 authority-contractok, CommandGateway, L6 lokalizációs
admission és L7–L12 motor/safety út változatlanok.

## Heading

A korábbi fizikai evidence tick1630 során a 30 s memóriájú yaw-összeg
0.108558 rad volt, a friss interval residual csak 0.012380 rad. Az L3
DEGRADED ág tévesen az összegzett értéket hasonlította a 0.10 rad félküszöbhöz;
az L6 jogosan HOLD-ot adott az így kapott minőségre.

Az L3 most az elfogadott, nem átfedő mérési intervallum yaw residualját
használja ehhez a minősítéshez. A történeti összeg külön
`accumulated_yaw_bias_warning`, amely 5 Hz-en és döntési átmenetkor is látható.
A 30 s diagnosztikai memóriát nem kellett csökkenteni. Az IMU/relative
freshness, covariance, observability, discontinuity és retained failure
kapuk megmaradtak. Jó új intervallum állíthatja helyre a headinget; pusztán
egy friss IMU vagy méréskimaradás nem törli a rossz interval evidence-et.

A célzott tesztek bizonyítják a történeti warning melletti GOOD minőséget,
a fél-/teljesküszöb körüli DEGRADED/LOST viselkedést, a helyreállást és a
checkpoint-egyezést. A mostani fizikai admission 41 heading mintája GOOD,
de mozgás közbeni történeti driftet ez a futás nem vizsgált.

## SLAM késés

A `/pose` DDS history 10 helyett 1 elem: a consumer a legújabb önálló
globális fixet igényli. Ez backlogkorlátozás, nem bizonyított teljes
késésjavulás. Az eredeti scan midpoint, sequence, exact-stamp association,
generation és a **250 ms** elfogadási határ megmaradt.

A híd 5 Hz-en közli a producer receipt, ROS scan publication, DDS pose send,
DDS receive, callback és return időket. A legutóbbi elfogadott és stale
elutasított mérés külön megmarad; a legutóbbi legfeljebb 64 póz szakaszonkénti
eloszlása az elutasítottakat is tartalmazza. DDS epoch időből a callback
epoch/monoton órakapcsolatával számolunk; hiányzó vagy érvénytelen DDS idő
explicit unavailable. Ingress-drain, map callback és spin maximumok is láthatók.
A státuszt a capture sidecar közvetlenül olvassa, a control processzt elkerülve.

A megszakadt admission 33 ROS-pózából 32 elfogadott, egy stale volt.
Az összesített késés mediánja 122.381 ms, p95 211.092 ms.
A konkrét elutasított **source sequence 30**:

| Szakasz | Idő |
| --- | ---: |
| Eredeti mérés → bridge ingress | 135.678 ms |
| Ingress → ROS scan publication | 26.098 ms |
| ROS scan publication → DDS pose send | 194.359 ms |
| DDS transport | 0.137 ms |
| DDS receive → callback | 0.486 ms |
| Összesen az acceptance pontig | **356.758 ms** |

Ebben az eseményben a bridge callback queue nem a fő késésforrás.
A publish→send szakasz TF-várakozást, upstream feldolgozást és schedulinget
egyaránt tartalmazhat; tiszta SLAM CPU-időnek nem nevezhető. A producer előtti
út és e szakasz belső gyökérokai még nincsenek külön bizonyítva. A QoS-változás
hatására kontrollált előtte/utána performance-állítás nincs, és a késési kiugrás
**nem tekinthető megszüntetettnek**.

![A mintavett pózok késési szakaszai](../../tmp/navigation_stability_20261113/cruise_1/slam_latency.png)

## Recovery és az 5 Hz-es mérhetőség

A korábbi BT kizárólag újratervezéssel próbálkozott. Most ugyanazon lifecycle
manager kezeli a planner/controller/behavior/BT szervereket. A rögzített
Nav2 1.3.13 Behavior Server stamped javaslatai ugyanazon ROS topic → bridge →
L6–L12 úton haladnak. A BT legfeljebb hat recovery kísérletet tesz, Wait 1 s,
Spin 0.7854 rad / 5 s, BackUp 0.15 m / 0.19 m/s / 5 s körforgással.
A valós costmap és a fizikai footprint collision-check megmarad; akadályt
nem törlünk. A spin minimum a feloldott fizikai kerékhatárból származik.
A mission korlátain túli javaslatot a meglévő bridge továbbra is elutasítja.

A végső motormentes ROS-próba négy ACTIVE lifecycle node-ot, három automatikus
recoveryt, tényleges spin/hátrameneti stamped parancsokat és cancel utáni
nullát igazolt. A V3 teszt a rear/oldalsó akadály és az unobserved szektor
miatti nulla motorkimenetet is bizonyítja. Fizikai kiszabadulás nincs igazolva.

Az 5 Hz-es observation most megőrzi:

- a SLAM és tényleges Nav2 costmap méretét, eredeti idejét, free/unknown/occupied
  cellaszámait, és legfeljebb 41×41 tényleges globális costmap cellát;
- a Nav2 út legfeljebb 32 eredeti pontját, eredeti idejét és truncation jelét;
- a scan eredeti mérési idejére lekért TF-fel számított, legfeljebb 96 returnből
  képzett scan–SLAM-map egyezést; ez nem független ground truth;
- bounded recovery átmeneteket, source node UID/időt, truncationt,
  goal UUID/child/mission azonosítós feedbacket és recovery számlálót;
- a már meglévő döntési átmeneteket, kért/engedett mozgást, kerék-/motorkimenetet,
  clearance-t, coverage-t és minden control ciklust összegző timing ablakokat.

Nagy map/ROS/raw payload nem került a control interpreterbe. A mostani 5 Hz
artifactban 31 costmap nézet és 16 érvényes, állóhelyzeti scan-map összevetés
van; útvonal és fizikai navigáció még nem volt. Ezekből menet közbeni
térképminőséget vagy körüljárási sikert nem állítunk.

## Fizikai akadály és validáció

Az első, motormentes preflight 50 egészséges státuszmintát adott, végig nulla
outputtal. Az utána kért room cruise **tick396**, `4453671790001 ns` pontján
RPLIDAR_C1 `LIDAR_TIMING_INVALID` → L12 `CRITICAL_DEVICE_FAILED` FAULT-tal
állt le, még IDLE állapotban. A capture normál tickjei és döntési átmenetei
egyetlen nem nulla motorkimenetet sem tartalmaznak. A skill/STOP felszíni
`ready IDLE` hibája downstream tünet. Minden külső folyamat leállítva;
további automatikus fizikai próbát nem indítottunk.

A korábbi generikus health reason nem őrizte meg, hogy owner-, driver- vagy
konkrét időhiba állt-e mögötte. Ezért a végső diagnosztikai javítás a bounded
edge/driver hibaokot is átviszi a meglévő health reasonbe, whitespace-et
tokenizálva. A tiltás változatlan. A fizikai LiDAR-hiba oka és megszűnése
ezzel még nem bizonyított; a következő fizikai próba előtt tisztázandó.

- Végső `./r test release` + célzott localization/LiDAR/Nav2/capture tesztek:
  **122 passed**. A release tartalmazza a kis kötelező gate-et és a natív
  NAVIGATE/EXPLORE persistált replay tesztjeit.
- Végső, motorút nélküli valódi Nav2 recovery:
  `tmp/navigation_stability_20261113/recovery_final/result.json`, success.
- Preflight: `v3_20261113_181108_5307_capture.mcap`, natív integrity PASS;
  natív L1–L12 replay 850–860: **MATCH**, 11 tick + 42 warmup tick.
  Ez a fizikai replay a végső health-reason diagnosztikai bővítés előtt futott.
- Megszakadt admission: `v3_20261113_181257_5807_capture.mcap`, kért 5 Hz scope
  integrity PASS, navigation-map transport 39/39, nincs loss. Mintavett scope,
  ezért teljes fizikai replay MATCH nem állítható.
- Mindkét explicit MCAP Evidence Compiler COMPLETE; source ellenőrzéssel
  **verify PASS**, 1214 és 223 message, nulla quarantine.

Részletes artifactok:
`tmp/navigation_stability_20261113/{preflight,cruise_1,recovery_final}/`.
Az eredeti lezárt capture és a natív reader az authority; a summary és ábra
származtatott diagnózis. A sikeres 30 s önálló cirkálás, menet közbeni
térkép-/mozgásminőség és fizikai recovery továbbra is hiányzó acceptance.

Forrásellenőrzéshez:
[Nav2 Behavior Server 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_behaviors/src/behavior_server.cpp),
[TimedBehavior stamped/cancel út](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp),
[SLAM Toolbox 2.8.5 pózpublikáció](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp).
