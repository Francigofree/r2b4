# CPU-refaktor: a két production ROS bridge executor-költsége

2026-11-18. Összehasonlítási alap: `722b151`. A munkafa induláskor tiszta volt.
A két eredeti bridge eltett példányának Git blob hash-e egyezik a HEAD-del.
A refaktor közvetlenül az aktuális munkafában készült.

A terv és megvalósítása: a megadott CPU-mérések processz/szál szerinti
szétválasztása; a production bekötés és a telepített rclpy source ellenőrzése;
motor-output nélküli profil és izolált összehasonlítás; a bizonyítottan
felesleges executor-entitások elhagyása; natív navigáció, replay és tesztkapuk.

A `runtime/cpu` három részletes, 10 másodperces mintájának összgépátlaga
42,45%, 75,18% és 68,66%. A magátlagok rendre 40,7–43,8%, 71,3–78,6% és
66,4–70,7%. A külön egyszerű CPU-log körülbelül 77%-os összgépátlagot mutat.
A részletes minták nem jelölik a futó commandot vagy capture-módot, ezért nem
rendelhetők bizonyítottan a `c nincs` és `c full` változatokhoz.

A Nav2 Python bridge 41,6–47,1% egy-mag CPU-t használ; az utolsó ablakban
ebből 45,3% a főszál. Az MPPI controller ugyanott 11,6%, a control processz
31,5%, annak főszála 17,3%. A LiDAR-szerű processzfa 26,9% + 11,6%, de a
CPU-eszköz az általános multiprocessing parancssorból nem bizonyítja a worker
konkrét szerepét. A nagyobb mintákban a verified robotcsoportokon kívüli
processzek további 15–19 összgép-százalékpontot jelentenek; ez nem automatikusan
roboton kívüli terhelés. Például a 3185-ös PID és 24605-ös start identity ma a
`r2b4_orchestration.robot_runtime` folyamaté, míg a korábbi rekord csak
`python3` parancsot tudott hozzárendelni.

Ezek Linux CPU- és scheduler-adatok. A control főszál utolsó ablakbeli
1,014 s runqueue-várakozása nem GIL-mérés, és a processz CPU-ja nem azonos
az L0–L12 CPU-idejével. A fizikai I/O ideje, a GIL-várakozás és az egyes
rétegek költsége e mintákból nem különíthető el teljesen.

A production út a `tools/nav2_planner.py` által indított
`v3.nav2_navigation_bridge`, illetve a `tools/slam_toolbox.py` által indított
`v3.slam_toolbox_bridge`. Mindkettő saját ROS dependency interpreterben fut.
A natív room profil Nav2 bridge-ében 110 334 `NumberOfEntities.__add__` és
193 830 context-stack bookkeeping hívás volt. A telepített rclpy
`event_handler.py` default incompatibility handlerei kizárólag figyelmeztetést
logolnak; az executor minden wait-set építésénél bejárja őket. Az izolált,
azonos ROS-topológiájú, 4 s-os idle mérésben a default log-handlerek elhagyása
0,701/0,712 s-ról 0,484 s-ra csökkentette a processz CPU-idejét, 36-ról 4-re
a waitable-ek számát. A külön kipróbált periodikus timeres változat több CPU-t
használt; az nem került a production source-ba.

A két bridge most saját, lokális Node-alosztályban hagyja el az implicit
publisher/subscription incompatibility log-handlereket, a TF és diagnostics
által létrehozott endpointokon is. A hívó explicit `event_callbacks` értéke
megmarad, az explicit `None` is. A bridge-ek nem indítanak külső
paraméterszolgáltatást; helyi parameter declaration és a controller felé menő
parameter client tovább működik. A production source-ban nem találtunk e két
node paraméterszolgáltatásához tartozó fogyasztót.

Ez konkrét diagnosztikai kompromisszum: e két node implicit incompatible
QoS/type figyelmeztetése és külső paraméterlekérdezése/-állítása megszűnik.
A DDS QoS-kompatibilitás, üzenetkezelés, explicit logok és callbackek,
readiness, source-időből számolt freshness és fail-closed kezelés megmarad.
A 2/5 ms-os spin timeout, a fizikai szenzor- és 50 Hz-es control frekvencia,
a 20 Hz-es MPPI, a rolloutok, a geometria és a döntési algoritmusok azonosak.
Új thread, processz, transport, authority vagy contract nem készült.

A natív szimulátor 12 s-os EXPLORE jelenetét sorosan A–B–B–A sorrendben
mértük, profilozó nélkül, külön ROS domainben, virtuális motor sinkkel.
A CPU-ablak az első readiness-től a fizikai eszközt nem használó stack
leállítása előtti pillanatig tart, körülbelül 12,4 s. Az A változat a két
eltett eredeti bridge-et futtatja; a második A esetén a munkafa már az új
source-ot tartalmazta. A V3, ROS binárisok és konfiguráció mindvégig azonosak.

| Mérés | Régi A1 | Új B1 | Új B2 | Régi A2 |
| --- | ---: | ---: | ---: | ---: |
| Nav2 bridge, egy-mag CPU % | 47,67 | 35,31 | 37,27 | 48,13 |
| SLAM bridge, egy-mag CPU % | 12,60 | 9,87 | 9,98 | 12,83 |
| Teljes host, négy mag átlaga % | 51,28 | 45,64 | 51,23 | 51,51 |
| Megtett út, m | 3,18 | 2,14 | 3,01 | 2,73 |
| Leghosszabb álló szakasz, s | 2,48 | 5,58 | 2,56 | 4,56 |

A Nav2 bridge átlaga 47,90 → 36,29%, **24,2% csökkenés**; a SLAM bridge-é
12,72 → 9,93%, **21,9% csökkenés**. A két bridge együtt 14,39 egy-mag
százalékpontot takarít meg, ami négy magon 3,60 összgép-százalékpontnak felel
meg. Ez a mért mechanizmus nagyságrendje, nem a korábbi 69–75%-os élő futások
50%-ra csökkentésének bizonyítéka. A host kétmintás átlaga 51,39 → 48,44%; a
hostterhelés és ROS időzítés változása miatt ezt nem tulajdonítjuk egészében
a refaktornak, és nem állítjuk, hogy minden külön mérésben minden mag 50%
alatt volt.

Mind a négy EXPLORE futás PASS, végső STOP-pal és nulla ütközéssel.
A1 és B1 teljes capture-re natív replay MATCH; B2 és A2 a startup- és
CPU-szórás ellenőrzésére készült, új replay nélkül. B1 az első mozgást
5,26 s-nál, A1 2,22 s-nál kapta; B1 frontier-logjában a stock explorer
körülbelül 5 s-ig várt a costmapra. A második párban a régi változat állt
hosszabban. Az első A/B összehasonlító verdict TRADEOFF, ezért a Room Cruise
mozgásminőség teljes nemregresszióját ezekből nem állítjuk.

A végleges source külön room/NAVIGATE futása PASS, GOAL_COMPLETE, 2,73 cm
célhiba, nulla ütközés, final STOP és natív replay MATCH. A kezdeti profilozott
room futás is PASS/GOAL_COMPLETE/MATCH volt; abból nem képzünk CPU A/B arányt.

Reprodukciós programok, régi source és nyers eredmények:
`tmp/cpu_20261118/{profile_sim.py,ros_wait_bench.py,simulation_bench.py,
wait_results.json,profile,before,after,after2,before2,after_room}`.
A `cpu.json` fájlok az owning processzek `/proc` tick-deltáit tartalmazzák,
nem csak a Python főszálat. A szimulátor szenzor-előállítása eltér a fizikai
acquisitiontől; a raw LiDAR driver és a teljes hardveres V3 processzfa CPU-ja
nem tekinthető ezzel megmértnek.

Validáció: `./r test release` **28 passed**, a kötelező kis robot gate-tel
együtt; a célzott Nav2/SLAM bridge-, lifecycle- és diagnostics-tesztek
**100 passed**. Az explicit MCAP Evidence Compiler a B1 capture-ből
**COMPLETE**, source integrity **COMPLETE**, 1134 üzenet. Bundle:
`tmp/cpu_20261118/after_evidence`; az eredeti capture hash-ét is ellenőrző
külön verify **PASS**, 0 quarantined, `source_checked=true`:
`tmp/cpu_20261118/evi_verify.json`.

Fizikai robotmozgás és új élő hardveres runtime-mérés nem történt.
Az élő `rc 60 c nincs` és `rc 60 c full` futások magonként körülbelül 50%-os
CPU-ja, hosszú navigációs minősége és hardveres jitter-nemregressziója még
nem bizonyított. A megmaradt konkrét kérdés az élő elfogadási cél és a
Room Cruise discovery/startup szórása; utóbbit a CPU-refaktor nem javítja.
