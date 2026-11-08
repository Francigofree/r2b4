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

## Elkészült változások

Az L3 a globális EKF-hez az IMU abszolút headingjét a megőrzött map–odom
forgatással igazítja. Az elfogadott SLAM yaw-korrekciót a következő IMU-mérés
így nem vonja vissza. A SLAM három külön mérési varianciája eljut az L3-ig;
a késleltetett SE(2) pózprojekció konzervatív covariance-envelope-ot is továbbvisz.
A lokális odometria, az L4 térkép és a mérés eredeti időpontja megmarad.

A LiDAR edge host packet-receipt időket őriz, és a sugarakat a lokális heading
alapján a scan midpointjába forgatja, a szenzoreltolással együtt. A hiányzó
heading-előzményre legfeljebb az eredeti midpointból számolt, már létező input- és
result-age limitek minimumáig vár; egy új raw scan felülírja a függő régit.
A raw safety azonnali;
hiányzó előzménynél a lokális geometria explicit unavailable. A kompenzált
geometria a relatív matcherhez, a helyi térképhez és a SLAM-exporthoz is eljut.
A packet receipt nem hitelesített fizikai sugáridő; a soros átvitel késése
korlátozza a kompenzáció pontosságát.

A SLAM scanritkítása valódi, mérési időből számolt 200 ms távolságot használ;
a bridge külön elutasítási okokat jelent. A nagy raw payload továbbra is a
producer/observation oldalon marad.

A [Nav2 Planner Server](../NAV2_PLANNER.md) külön ROS-prefixből indul. Csak
globális NAVIGATE útjavaslatot ad; a meglévő V3 rollout teljes footprinttel
ellenőrzi a helyi szakaszt. Pending, hiányzó guidance, hibás lineage, no-path és
stale eredmény nem kerülhet közvetlen native seed vagy megmaradt escape mögé.
Az aktív konfigurációban a Nav2 egyelőre explicit opt-in (`null`).

## A motormentes élő mérésben talált és javított cache-hiba

Forrás: `runtime/captures/v3_20261107_194309_7793_capture.mcap`.
Az első bizonyított elakadás seq14 / tick108: friss safety mellett nem érkezett
lokális geometria. Tick112-n a seq13 térkép kora már 250 771 791 ns lett.
A producer azonos fizikai scan-azonosítóval küldte a pending után elkészült
geometriát, de a `ProcessLidarPort._drain_state` cache csak új scan-id-t vett át.

A fogadó most változatlan fizikai start/end/midpoint mellett átveszi a
pending → ready completiont. Ugyanazon identity átírt mérési ideje tartós,
fail-closed hibát okoz. A célzott teszt a producer → collector → latest source →
L2 → L4 teljes átadást és az egyszeri admissiont ellenőrzi. A javítás utáni
8 s motormentes mérésben a térkép sequence 6-ról 86-ra haladt; a lokális és
heading minőség GOOD volt, a bemelegedés utáni mért térképkor legfeljebb177 ms.

## Szoftveres és motormentes bizonyítás

| Ellenőrzés | Eredmény és határ |
| --- | --- |
| `./r test` | 14 passed. |
| `./r test release`, collector-javítás után | 23 passed; késleltetett planneres Room Cruise és natív replay is szerepel benne. |
| Nav2 + config authority célzott kör | 24 passed; process/direct ekvivalencia és hibás completion fail-closed kezelése. |
| Forgáskompenzáció és teljes collector-route célzott kör, 2026-11-08 | 8 passed. |
| Valódi Nav2 Planner Server, motor nélkül | Kerülő út READY / 4 használható jelölt / 19,68 ms; NO_PATH és STALE / 0 használható jelölt; helyi pure újraszámolás egyezik. Egyetlen motormentes példa nem általános timing-garancia. |
| LiDAR edge microbenchmark | 720 pont, 100 futás: p95 2,15 ms, max 2,84 ms; nincs fizikai timing-pontossági igazolás. |
| Első motormentes MCAP explicit compiler + source verify | COMPLETE / PASS, 2476 üzenet, 0 quarantine. |
| Ugyanezen MCAP natív replay | MATCH, 1985 tick kétszeri azonos L1–L12 végrehajtással; minden final STOP. Ez nem mozgásbizonyíték. |

## A legfrissebb mozgásfelvétel ellenőrzése – 2026-11-08

Forrás: `runtime/captures/v3_20261107_224208_3009_capture.mcap`.
SHA256: `e6d7475f4e8ba112b3f9a619439696dc6eefa262dae154325eb96ba51b106cde`.
Explicit Evidence Compiler: COMPLETE; source verify PASS, 6911 üzenet,
0 quarantine. A capture 5 Hz-es, sample-complete, exact replayre nem teljes.

A tényleges EXPLORE/ACTIVE mozgás tick101 / 710075254147 ns-nál kezdődött.
Tick4210 / 800614784126 ns-nál L11 FAULT zárta: `MISSING_EDGE:left;MISSING_EDGE:right`.
Az aktív idő **90,53953 s**; a 120 s teljesítése **nem bizonyított**.
A terminális behavior `BEHAVIOR_FAILED / RUNTIME_FAULT`; L12 disabled és
mindkét motor-output nulla. A felhasználó megfigyelése: mozgott, majd magától megállt.

A legutolsó elfogadott encoderélek 800347563239 / 800347378554 ns-nál voltak.
Utána a célok és a tényleges L11/L12 outputok a faultig pozitívak maradtak;
nem egy korábbi STOP, wheel-cap túllépés vagy kimeneti clamp indította az epizódot.
A source csomagok frissek voltak, de az elfogadott élek több mint 250 ms-ig hiányoztak.
A watchdog helyesen letiltott. A mechanikai megállás és a GPIO/callback-adatvesztés
között ez a scope nem dönt: nincs nyers impulzuselőzmény, IMU-gyro és relatív
LiDAR-elmozdulás a kritikus utolsó intervallumhoz. Emiatt a watchdog vagy a
trust/freshness limitek lazítása nem indokolt.

[A leállás részletes scalar evidence-e](/home/alba/project_r2b4/tmp/amr_20261108_3fKFHm/terminal_stop_summary.json).

## Helyi akadálytérkép és megmaradó mozgásminőségi probléma

A felvétel 457 tényleges L4 cellageometriát tartalmaz. A tick/context, frame,
sequence, revision és measurement-time egyezik; nincs regresszió vagy truncation.
A lokális frame/generation stabil, nincs rögzített pózugrás. A 457 külön
sensor-map observation mindegyike APPLIED deskew-t jelez. A 203 közös sensor/L4
sequence mérési ideje pontosan egyezik. Ez a lokális térképre vonatkozó evidence,
nem csak a mentett SLAM-raster ellenőrzése.

A térképkor p50 149,30 ms, p95 194,60 ms, maximum 288,97 ms. Az eseményalapú
decision capture 29 `LOCAL_COSTMAP_STALE` tickjében a végső motor-output nulla.
Az első tick 153 / t=3,33877 s: seq 23 / 250,78083 ms; tick 155-nél friss seq 25
mellett helyreáll. Ez megmaradó, konkrét akadozási ok. Az 5 Hz-es scope nem
választja szét a fizikai I/O, edge compute, transport és admission késését.

[Négy helyi térképábra és lineage-metrikák](/home/alba/project_r2b4/tmp/amr_20261108_3fKFHm/local_map_final/README.md).
Külső referencia nélkül a falgeometria fizikai pontossága és a javulás mértéke
nem állítható. Teljes fizikai Nav2 NAVIGATE mission sem volt igazolva.

## Engedélyezett FULL 50 Hz próba – 2026-11-08

A felhasználó új, egyszeri 120 s Room Cruise próbát engedélyezett, 0,20 m/s és
1,10 rad/s korláttal. A canonical skill/CommandGateway út indította.
A motormentes readiness 40 mintában sikeres volt: a térkép sequence 5→84,
a legnagyobb térképkor 203,90 ms, a lokális és heading minőség GOOD.

Forrás: `runtime/captures/v3_20261108_091129_3949_capture.mcap`.
SHA256: `84c19b62e01269c7ed093090b7f22142650b8eacb76dca623613fad5cc581f04`.
A futás L11 `MISSING_EDGE:left;MISSING_EDGE:right` faulttal megszakadt,
tick3557 / 1189333569187 ns-nál. A 120 s elfogadás **nem teljesült**.
A runtime és a SLAM leállítva; automatikus újrapróbálás nem történt.
A skill későbbi STATUS_ERROR-ja a runtime-fault következménye.

Az első pontosan visszakövetett boundary tick3538/3539: az L4 seq773 térkép
252,30 / 268,83 ms kora `LOCAL_COSTMAP_STALE`-t, majd két nulla motor-outputot
okozott. Tick3540-nél a seq775 friss geometriájával újraindult a rámpa.
Az encoderélek lassultak, majd mindkét oldalon elmaradtak; a 250 ms-os
élhiány-védelem L11 faultot okozott. Friss IMU-gyro és relatív LiDAR-eredmény is
van, de az utóbbi elmozdulása a registration RMSE nagyságrendjében van.
A végső mechanikai megállás, tapadás és elektromos/callback-élhiány ebből
nem különíthető el biztosan. A watchdog és a motorút változatlan.

### A seq774 lokális geometria kimaradásának szűk javítása

A raw seq774 459 pontja érvényes. Az utolsó pont receipt ideje
1188818367760 ns; az utolsó elérhető L3-context3534 még 2,32 ms-mal korábbi.
A következő L3-context3535 már körülhatárolja a teljes scan headingjét.
A korábbi scan-end +40 ms határ előtt azonban csak 4,62 ms maradt az új
context producerhez jutására. A felvétel nem tartalmazza a producer pontos
retry/pose-publication idejét, ezért a timeout közvetlen bekövetkezése nem
állítható. A rögzített raw scan és heading-előzmény tiszta újraszámolása viszont
3534-ig `HEADING_BRACKET_UNAVAILABLE`, 3535-tel `APPLIED`, mind a 459 pontra.
L2-elutasítás nincs; seq774 pozitív lokális geometria nem jutott el L4-ig.

A process és a relatív matcher edge most a még friss eredeti mérésre vár,
fix 40 ms helyett a meglévő input/result freshness budgeten belül. Legfeljebb
egy legfrissebb scan függő; új raw scan felülírja. Nem nőtt freshness-limit,
nem változott mérési idő vagy sequence, és a raw safety továbbra is azonnali.
A lejárt mérés akkor sem kerül workerbe vagy pozitív geometriaként tovább,
ha közben megérkezett a heading. Ez még nem a javítás utáni fizikai bizonyítás.

A teljes session natív replaye a canonical materializációs budget miatt nem
futott; a korlát nem lett lazítva. A checkpointból indított hibakörnyezeti
replay **MATCH**: 37 kiválasztott tick, 86 végrehajtott tick kétszer, L1–L12
eltérés nélkül. Ez a rögzített bemenetek döntési egyezését bizonyítja;
az adapter javítása utáni fizikai működést nem.

[Leállási scalar evidence](/home/alba/project_r2b4/tmp/amr_20261108_3fKFHm/cruise50_terminal_summary.json),
[seq774 újraszámolás](/home/alba/project_r2b4/tmp/amr_20261108_3fKFHm/cruise50_scan774_deskew_check.json),
[hibakörnyezeti replay](/home/alba/project_r2b4/tmp/amr_20261108_3fKFHm/cruise50_incident_replay.json).
