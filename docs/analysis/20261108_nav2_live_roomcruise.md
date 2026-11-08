# Nav2 és Room Cruise élő ellenőrzés – 2026-11-08

Az aktuális fejlesztés a meglévő V3 / async contractot követi. A globális
NAVIGATE az izolált Nav2 Planner Server javaslatát használja; a helyi
footprint-ellenőrzés és a végső motor-authority a V3-ban marad. A Room Cruise
EXPLORE továbbra is a helyi V3 tervezőt használja.

## Változás és mérési terv

Az aktív `conf/vezerles.json` most bekapcsolja a meglévő Nav2-adaptert:
0,18 s kéréshatáridő, 6 s globális térkép-publikációs korlát. A sidecar
külön, a `./r nav2 start` paranccsal indul. Elérhetetlen globális planner
esetén a NAVIGATE HOLD-ot ad. A teljes térkép és globális út a ROS edge-ben
marad; a control kizárólag kis, lezárt guidance-completiont kap.

A korábbi LiDAR-javítás végellenőrzése igazolta a deskew utáni measurement-age
guardot mindkét adatúton, a hiányzó midpoint-póz lejáratát, az egyetlen latest
pending scan határát és a változatlan scan-azonosságot. Az első két próbakörben
a freshness- és encoder-watchdog limitek változatlanok voltak. A felhasználó szerint az encoder kis,
0,15 m/s körüli sebességnél bizonytalan; emiatt az új Room Cruise próba
0,30 m/s / 0,80 rad/s felső korláttal készül. A tényleges sebességet az
akadályok és a V3 továbbra is csökkenthetik; a gyorsítási átmenet nem kerülhető meg.

## Szoftveres ellenőrzés

| Ellenőrzés | Eredmény |
| --- | --- |
| LiDAR deskew és relatív budget, két célzott canonical file | 20 passed / 0,65 s |
| `./r test release`, aktív Nav2-konfigurációval | 23 passed / 23,19 s |
| Nav2 és config authority célzott canonical tesztek | 24 passed / 1,42 s |
| Módosított config és dokumentáció whitespace-check | PASS |

Az előzetes FULL 50 Hz IDLE capture:
`runtime/captures/v3_20261108_142919_29046_capture.mcap`; canonical finalizálás
integrity PASS, mozgásparancs nélkül. Az első Nav2 actual-map diagnosztika
quality-guardnál megállt, még a tervezési kérés előtt. A későbbi státusz GOOD
volt; a próba nem állított elő hamis qualityt. A CPU-igényes archíválás és
az IDLE session lezárása után új readiness következett.

Az új session 40 motormentes mintájában local / heading / global GOOD,
a térkép sequence 493→572, a bemelegedés utáni legnagyobb térképkor
125,89 ms. A runtime és mindkét ROS sidecar elindult.

Az új motion session FULL 5 Hz-es viselkedési scope-ot rögzít. Ez kisebb
erőforrásigényű, de nem teljes tick-pontos replay-scope. A korábbi ideiglenes
1,4 GB-os evidence-bundle veszteségmentesen tömörített archívumba került:
`tmp/amr_20261108_3fKFHm/cruise50.evidence.tar.gz`; mind az 5901 fájl végig
olvasható volt a könyvtár felszabadítása előtt. A korábbi verify/replay
riportok megmaradtak. Ez azért lényeges, mert a korábbi MCAP már nem volt
elérhető az aktuális working tree-ben.

Az új mérés artifactjai:
`tmp/amr_live_20261108_2VMQ4t/`.

## Valós SLAM → Nav2 → V3 geometriai próba

A motormentes, új actual-map próba sikeres: 144×125 cellás, 0,05 m felbontású
tényleges SLAM-grid, mindhárom valódi quality GOOD. A Nav2 READY lokális
útjavaslatot adott; a canonical process planner és a V3 geometriai kernel
50,14 ms alatt végzett, 26 ütközésmentes, teljes footprinttel a diagnosztikai
ROI-ban maradó jelölttel. A lezárt guidance-ból végzett pure újraszámolás egyezik.

Ez nem production L4-rekonstrukció: a lokális geometria a tényleges SLAM-grid
konzervatív vetülete, 1,316 m-es ROI-val; azon kívül ismeretlen. A globális
térkép 1,748 s publikációs kora nem fizikai measurement-time. A diagnosztikai
world freshness ezért explicit ismeretlen, és a production L6 motion-gate
számára alkalmatlan. Nem történt command vagy motor-output, teljes globális
NAVIGATE mission továbbra sem bizonyított.

Artifact: `nav2_actual_map/summary.json`, `closed_request.json`,
`closed_result.json`, `actual_slam_map.json` a fenti mérési könyvtárban.

## Room Cruise eredmény és korlát

Forrás: `runtime/captures/v3_20261108_143335_29813_capture.mcap`.
Az engedélyezett 120 s skill aktív intervalluma 115,661 s-ig jutott; a passzív
monitor a lokális minőség LOST állapotánál canonical cancel/STOP-ot kért.
Terminális skill-state CANCELLED / AMR_VALIDATION_ABORT. Encoder-fault nem volt.
A host SDK runtime-stop hívása később timeoutolt; a külön canonical shutdown
már STOPPED runtime-ot igazolt. A Nav2 és a SLAM is leállt; nincs automatikus
új mozgáspróba.

A futás kb. első 42 másodperce után hosszú NO_PATH_EXHAUSTED_HOLD alakult ki.
A diagnosztika 88,363 s zero-request HOLD-ot rögzített; a leghosszabb összefüggő
HOLD 73,761 s. A minősített, mintavett lokális pózokból becsült út 8,165 m;
ez nem külső ground truth vagy folyamatos haladás bizonyítása.

A választott 0,80 rad/s tesztkorlát hibás volt a helyben forduló meneküléshez:
a 0,3557 m nyomtáv és a 0,19 m/s kerékcél-minimum mellett legalább
`2 * 0.19 / 0.3557 = 1.0683 rad/s` szükséges. A 0,30 m/s haladási cap
megtartható; a következő próba 1,40 rad/s cap mellett készítendő. A nagyobb
cap önmagában még nem bizonyít footprint-szempontból járható menekülési utat.

A rögzített HOLD tick12129 friss L3/L4 snapshotján, azonos 128 tényleges
akadálycellával, seq2473 / revision4839 / 80,76 ms térképkorral készült
geometriai ellenpróba. 0,80 rad/s mellett nulla ütközésmentes, progress-viable
jelölt és nulla elérhető exit-bin; 1,40 rad/s mellett két ilyen jelölt és
14 elérhető exit-bin. A tiszta pivot kerékcéljai ±0,24899 m/s,
legkisebb footprint-clearance 0,17698 m. Ez a corrected envelope geometriai
lehetőségét igazolja. Nem exact replay: a coverage, failed-goal és exit-history
állapot nincs rekonstruálva. Artifact: `hold_omega_comparison.json`.

A terminális eltérés nem igazolt álló-robot ICP-zaj: a gyro előbb +2,43 rad/s-ig
nőtt, majd a helyi yaw több mint 2 radiánnal változott, nulla kerékcél és
régi encoderélek mellett. A relatív LiDAR is elfordulást jelzett. A felhasználó
megerősítette, hogy kézzel elfordította a robotot a végén, és előtte valóban
hosszabban állt. Ez megmagyarázza a kerékodometria nélküli testelfordulást;
nem indokolt encoder-, trust- vagy lokalizációs küszöböt módosítani.
A részletes idővonal:
`terminal_yaw_source_review.json`.

Az explicit compiler COMPLETE: 13 522 üzenet, source verify PASS,
0 quarantine. Az 5 Hz scope miatt a mozgásfutás nem exact replay-evidence.
A korábbi motormentes FULL 50 Hz felvétel checkpointos rövid szakaszának
natív replaye MATCH: 19 kiválasztott tick, 36 végrehajtott tick kétszer,
L1–L12 eltérés nélkül. Ez motormentes determinisztikai bizonyítás.

**Nem bizonyított:** 120 s elfogadás, folyamatos AMR-szerű haladás,
teljes fizikai globális NAVIGATE mission, külső referenciához mért térképpontosság.

## Javított fordulási korlátú élő próba

A felhasználó új, egyszeri próbát engedélyezett 0,30 m/s / 1,40 rad/s
korlátokkal. Motormentes readiness PASS, maximális térképkor 141,33 ms.
Forrás: `runtime/captures/v3_20261108_144925_31926_capture.mcap`.
A próba 19,940 s aktív idő után L11 faulttal leállt. A minősített mintavett
lokális pózokból becsült út 6,766 m; ez becslés, nem fizikai ground truth.

Az első fault tick3142 / 15850216225690 ns:
`MISSING_EDGE:left;MISSING_EDGE:right`. A kerékcélok +0,2725 / +0,2725 m/s,
a legutolsó elfogadott élek kora 255,06 / 253,91 ms. Az encoder-snapshot friss
és érvényes. A megelőző másodperc 37 navigációs eseményében nincs STOP,
zéró motor-output, irányváltás vagy L9/L10/L11 átmeneti határidő.
Ez a megállás ezért nem a 0,15 m/s körüli sebességfit vagy egy újrainduló rámpa
következményeként bizonyított. A felhasználó szerint a robot szabadon haladt
a leállásig. A fizikai megállás és a közös GPIO-értesítési út kiesése továbbra
sem különíthető el biztosan; konkrét encoder-hardverhibát nem állítunk.
Artifact: `corrected/terminal_l11_summary.json` és `terminal_l11_boundary.json`.

## Kért 500 ms-os élvédelem

A felhasználó kifejezetten kérte a 250 ms-os védelem 500 ms-ra emelését és
az élő teszt folytatását. A `wheel_pi.max_feedback_uncertainty_ns` 500 ms lett.
Az L11 accepted-edge liveness most kizárólag ebből a fizikai élidőből számolt
budgetből dolgozik. A korábbi `min(max_feedback_age, max_feedback_uncertainty)`
250 ms-nál továbbra is elvette volna az élbizonyítékot, még friss acquisition
esetén is. Az acquisition-age külön ellenőrzésének 250 ms-os határa és az L2
admission freshness megmarad. Snapshot-publikálás nem újítja meg az él határidejét.
