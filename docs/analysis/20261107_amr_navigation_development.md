# Pozícióbecslés, SLAM és Nav2 – 2026-11-07

## Fejlesztési terv és elfogadás

A meglévő V3 réteg- és async contract változatlan authority. A fejlesztés az
aktuális SLAM/LiDAR adaptert, az L3 becslést és az L6 planner edge-et érinti;
a command-, motion- és motorút a canonical V3 marad.

1. **Pozícióbecslés:** a lokális, folytonos odometria és a globális SLAM-korrekció
   külön frame-jének megőrzése; az IMU heading, a késleltetett mérés és a mérési
   covariance helyes kezelése. Elfogadás: független globális korrekció után a
   lokális odometria és akadálytérkép folytonos, az ismételt IMU nem törli a
   map–odom iránykorrekciót, a stale vagy hibás mérés elutasított marad.
2. **SLAM/LiDAR:** scan-mérési idő, pose-reference és forgás közbeni geometria
   összehangolása az acquisition edge-en. Elfogadás: a javítás a helyi
   akadálytérképet kiszolgáló pontokra is vonatkozik, és nem tesz nagy raw
   payloadot a control interpreterbe. Becsült sugáridő nem állítható be fizikai
   mérésként igazolt acquisition-modell nélkül.
3. **Nav2 első lépés:** külön ROS Planner Server globális útjavaslata a meglévő
   process planner porton keresztül, kezdetben globális NAVIGATE célokra.
   A V3 a javasolt lokális szakasz teljes footprintjét továbbra is ellenőrzi.
   Hiányzó, késő, hibás vagy út nélküli Nav2 eredmény HOLD-ot jelent.
   Controller Server és közvetlen ROS motorparancs nem része a változásnak.
4. **Helyi mozgás:** a Room Cruise friss lokális geometriára építő tervezése és
   az L7–L12 folytonossági korlátai megmaradnak. A mérésben külön vizsgálandó a
   haladás, a fordulás, a HOLD/STOP oka, a planner késése és az L4 térkép.
5. **Bizonyítás:** célzott fejlesztői tesztek, a kötelező robot-contract gate és
   az érintett közös boundaryk miatt release gate; utána motor-output nélküli
   readiness, majd egy 120 s Room Cruise a felhasználó engedélyével. A lezárt
   FULL 50 Hz MCAP-ból explicit Evidence Compiler és natív replay következik.
   Váratlan safety-, health-, process- vagy timing-hiba megszakítja a próbát;
   nincs automatikus újraindítás.

## Kiinduló állapot

A Git working tree tiszta volt. A robot runtime és a SLAM sidecar nem futott.
A production config SLAM Toolbox globális backendet és processes helyi
tervezőt választott. Nav2 csomag nem volt telepítve. A korábbi SLAM-status
session-váltás miatti leállást tartalmazott; ez történeti adat, nem a jelen
fejlesztés élő eredménye.

Az élő elfogadás, a replay és a független fizikai pontosság a munka kezdetén
**nem bizonyított**. A végleges változások és eredmények alább kerülnek rögzítésre.
