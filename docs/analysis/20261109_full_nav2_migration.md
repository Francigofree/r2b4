# Teljes Nav2 átállás — 2026-11-09

## Cél és végrehajtási határ

A SLAM Toolbox térképet és map→odom transzformációt készít. A Nav2 Planner
Server, Controller Server, globális/lokális costmap és BT Navigator külön
folyamatokban számol. A Room Cruise ugyanitt, a SLAM-grid szabad celláiból kér
következő felfedezési célt. A ROS-oldal nem ír motort és nem ad ki új V3 missiont.

A canonical CommandGateway/L5 NAVIGATE vagy EXPLORE missionje kis kérésként
jut a navigációs edge-be. Az edge eredménye legfeljebb 200 ms élettartamú,
mission-, cél-, provider-generation-, sequence- és measurement-lineage-et
hordozó sebességjavaslat. A runtime ezt a tick előtt zárja le. L6 validál,
L7/L8 realizál, L9 korlátoz, L10/L11 kerékvezérlést végez, L12 marad az
egyetlen final motor-authority. Hiány/stale/azonosságvesztés: nulla mozgás.

Teljes Nav2 módban L4 nem integrál saját occupancy/structural térképet,
L6 nem futtat saját útkeresést vagy trajectory rolloutot. A régi út kizárólag
korábbi konfigurációk/replay és külön, nem Nav2-s képességek támogatására
marad elérhető; az aktív robotkonfiguráció választja ki a szolgáltatót.

## Elfogadási sorrend

1. Célzott szerződés-/transport-/mission-/lejárati tesztek és `./r test release`.
2. Valódi Nav2 lánc motormentesen: friss map/TF/odom/scan, elfogadott action,
   Controller sebességkimenet, cancel és heartbeat/stale STOP. Szintetikus ROS
   bemenet csak a szoftveres lánc működését igazolja, fizikai pontosságot nem.
3. Fizikai szenzoros motormentes readiness és 50 Hz timing/capture/replay.
4. Az engedélyezett 10 s Room Cruise, majd csak sikeres kiértékelése után
   120 s Room Cruise; canonical STOP minden hibánál. Váratlan health-, safety-,
   process- vagy timing-hiba után nincs automatikus új mozgáspróba.
5. Megadott cél elérése, akadály miatti újratervezés/kikerülés, mozgásminőség
   és terhelt 50 Hz külön evidence. A térkép fizikai mérethűségéhez függetlenül
   kimért fal-/landmark-távolság szükséges.

## Függőségek

A meglévő izolált Jazzy 1.3.13 környezethez a Controller, BT Navigator és
Regulated Pure Pursuit Controller, továbbá két nav-2d függőség került hozzá.
Meglévő csomag nem változott. A teljes explicit aarch64 lock frissült.
A BT replanning pipeline csak Planner/Controller actionöket használ;
külön recovery-mozgás nincs a V3 mögé kötve.

A konfiguráció alapja a [Nav2 Jazzy BT Navigator](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/)
és a [Regulated Pure Pursuit Controller](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
dokumentációja; a tényleges paramétereket az installált Jazzy implementációval
és a robot fizikai korlátaival kell ellenőrizni.

## Eredmények

A fizikai referencia a felhasználó mérése szerint: a robot előtt lévő fal kb.
2,8 m, a bal oldali fal kb. 0,7 m távolságban van. Ezek a LIDAR középpontjától mért, közelítő értékek.

A fejlesztés és a validáció folyamatban van; elfogadási siker még nincs igazolva.
