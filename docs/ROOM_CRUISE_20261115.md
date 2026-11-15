# Room-cruise mérés és javítás, 2026-11-15

Az AMR-szerű, tartós fizikai mozgás még nincs igazolva. A célválasztás egy
starvation-hibája és egy recoveryt blokkoló geometriai eltérés javítva;
120 s szimuláció és egy rövid fizikai próba sikeres. A következő hosszú
fizikai próbát új, mindkét kereket érintő encoder-edge-kiesés szakította meg.
A robot STOPPED; további mozgás nem indult e fault után.

## Kiinduló élő evidence

Forrás: `runtime/captures/v3_20261115_151622_67486_capture.mcap` és
`v3_20261115_151822_69925_capture.mcap`. Mindkettő mély integritásellenőrzése
PASS, explicit MCAP Evidence Compiler eredménye COMPLETE. Mintázott,
replay-core nélküli capture-ök: a native Replayer jogosan
`CAPTURE_INCOMPLETE` eredményt ad. Ez nem replay-egyezés.

| Mért érték, 5 Hz viselkedési minta | 15:16:22 | 15:18:22 |
|---|---:|---:|
| EXPLORE idő | 70,4 s | 128,4 s |
| Közel nulla haladás, abs(v) < 0,03 m/s | 59,8% | 50,9% |
| Hátramenet, v < -0,03 m/s | 5,9% | 16,8% |
| Pozitív haladási sebesség mediánja | 0,271 m/s | 0,246 m/s |
| Sebességből becsült jerk RMS, 5 Hz | 2,38 m/s³ | 2,50 m/s³ |
| Natív controller `lethal_cell` hibák | 4 | 7 |
| Elért coverage-célok | 3 | 4 |
| Végső cél nélküli szakasz | 33,6 s | kb. 51,8 s |

A nulla haladás fordulást is tartalmaz; nem azonos a teljes megállással.
A mintázott sebességből számított jerk nem közvetlen gyorsulásmérő-adat.

## Igazolt gyökérokok és változtatások

1. **Célválasztási starvation.** A `RoomCruiseGoals.select` minden próbánál
   ugyanazt a legjobb 32 jelöltet ellenőrizte. Ezek lehettek a grid határán
   kívül, ismeretlen vagy lethal cellában; a lejjebb rangsorolt szabad célokhoz
   nem jutott el. A javítás legfeljebb 1024 rangsorolt koordinátán folytatja
   a keresést, fordulónként legfeljebb 32 ellenőrzéssel. Minden jelölt az
   aktuális térképen vizsgálandó; térképfrissítés nem indítja újra a sort.

2. **MPPI-átlag és kerékmegvalósíthatóság.** A natív adapter az eredeti
   kerékprojekció ütközésekor korlátos számú, megvalósítható alternatívát
   vizsgál. Minden alternatíva ugyanazon teljes 1 s horizonton ellenőrzött;
   akadályos, ismeretlen vagy pályán kívüli mozgást nem enged át. A már szabad
   eredeti parancs változatlan marad.

3. **Indokolatlan normál hátramenet.** EXPLORE módban az MPPI lineáris
   minimuma 0; a PathAngleCritic előremeneti preferenciát kapott. A külön
   natív, costmap-ellenőrzött BackUp recovery továbbra is használható.
   NAVIGATE reverse-képessége megmarad.

4. **A body-padding geometriája.** Az első új 120 s élő futás a kezdeti
   javításokkal funkcionálisan megbukott (`REQUESTED_EXECUTION_UNPROVEN`).
   Három elért cél után a recovery-jelöltek már a kezdőpózban elutasultak.
   Egy konkrét retained cella: `[3.35,3.40] × [1.95,2.00]`, póz
   `(3.628266586, 2.350352139, -1.422011923)`. A teljes cellanégyzet és a
   fizikai test pontos euklideszi távolsága **0,1109625675 m**.
   A korábbi tengelyenkénti 0,10 m-es téglalapbővítés a sarkoknál 0,1414 m
   ráhagyást követelt, ezért ezt a cellát is tiltotta.

   A Nav2 és a Python goal/recovery ellenőrzés most ugyanazt a fizikai testet
   és a 0,10 m apotémájú, körülírt szabályos 16-szög Minkowski-összegét
   használja. A plusz közelítési tartalék legfeljebb 1,96 mm; a margó minden
   irányban legalább 0,10 m. A már kibővített explicit footprint mellé a
   Nav2 `footprint_padding` 0, így nem számoljuk kétszer a margót.

   A natív adapter teljes konvex polygon–cellanégyzet SAT-ellenőrzést végez.
   A folyamatos ívhez negyed cellányi maximális pontelmozdulás és a tényleges
   mintaköz felével számolt konzervatív geometriai guard tartozik. Kezdő- és
   végpóz ellenőrzött; 128 intervallum felett fail-closed az eredmény.

   A teljes megőrzött lokális térképen a 25 cm hátrálás és a +0,70 rad balra
   fordulás szabad; független, sűrű pontos test–cellatávolság-mérésük minimuma
   0,1110 m. A jobbra fordulás továbbra is elutasított. Ez geometriai
   ellenpélda és retained-scene bizonyíték; a robot odakerülésének minden
   követési/latenciahatását önmagában nem magyarázza.

5. **Szenzor-fogadási sor.** A Nav2 ROS edge scan/odom fogadója az alap
   SensorData depth=5 helyett BEST_EFFORT depth=1, legfrissebb értékű sort
   használ. Külön motor nélküli ROS-próba ugyanazon tíz üzenet után depth=5
   esetén a 6–10. üzenetet, depth=1 esetén csak a 10. üzenetet adta át.
   A javítás nem ír át measurement stampet és nem hosszabbít érvényességet.
   A fizikai frissességi megszakítások megszűnése még nincs bizonyítva.

6. **Passzív evidence-kiesés.** A bounded motion/recovery ringekkel és két
   costmapablakkal a status időnként meghaladta a capture reader 128 KiB-os
   korlátját (`NAV2_STATUS_EXCEEDS_BOUND`). Csak a capture-sidecar Nav2
   olvasási kerete nőtt 256 KiB-ra; oversize továbbra is elutasított.
   Ez nem control interpreterbeli payload vagy actuation-authority.

A fizikai kerékminimum, maximum, gyorsulási korlát, a 200 ms command-TTL,
a 250 ms szenzorfrissesség és az L12 0,10 m STOP-clearance változatlan.
PI-gain módosítást az evidence nem igazolt, ezért nem történt.

## Szimuláció

A virtuális szenzorútból eltávolított truth-relative/pose minták helyett a
szintetikus sugarakon a production RelativeLidarOdometry fut. A virtuális
encoder tényleges megtett kerékútból, az aktív méter/pulzus kalibrációval ad
iránymegerősített élbizonyítékot. Álló helyzetben az élidő nem újul meg.
Ez ideális szenzormodell; fizikai GPIO-, kalibrációs és kerékcsúszási bizonyíték
nem következik belőle. A simulator 120 s-ig enged futást, a generikus
motion-scenario 30 s alapkorlátja változatlan.

Az első hosszú szimulációk hamis L11 encoder-watchdog hibáját a hiányos
virtuális élbizonyíték okozta; a watchdogot nem lazítottuk.

| 120 s production V3 + SLAM + Nav2 szimuláció | Elért/kiadott cél | Úthossz | Collision / fault | Replay |
|---|---:|---:|---:|---|
| Előremeneti preferencia előtt | 13 / 14 | 28,83 m | 0 / 0 | 6219 tick, 7 slice MATCH |
| Előremeneti preferenciával | 14 / 15 | 35,39 m | 0 / 0 | 6235 tick, 7 slice MATCH |
| Euklideszi footprinttel | 14 / 15 | 36,59 m | 0 / 0 | 6392 tick, 7 slice MATCH |

Az első két kísérletben indulás után a hátramenet ideje 47,0%-ról 1,47%-ra
esett, de a teljes megállás 2,08%-ról 3,54%-ra, az 5 Hz-es jerk RMS
1,29-ről 1,35 m/s³-re nőtt. Ez egyértelmű kompromisszum, nem teljes
mozgásminőségi siker. Az euklideszi footprint futásában a pozitív sebesség
mediánja 0,445 m/s, a minimális valódi testtávolság 0,237 m volt. A ROS
ütemezése nem determinisztikus, ezek nem sokismétléses statisztikai eredmények.

A hosszú capture-ök a natív memóriahatárok emelése nélkül, minden döntési
ticket lefedő, külön canonical checkpointból indított szeletekkel replayeltek.
A forrás SHA256, a szelet határa és elemszáma ellenőrzött. Ez **nem egyetlen
folyamatos állapot-összehasonlítás**. A simulator ehhez tényleges canonical
checkpointokat ír; hiányzó checkpoint vagy bármely nem MATCH szelet nem
minősülhet összesített MATCH-nek.

## Új fizikai evidence

Az első 120 s futás (`v3_20261115_173225_8679_capture.mcap`) integritása és
runtime-státusza PASS, de a navigációs kimenet FAIL. Ezt a geometriajavítás
előtti sikertelen kontrollkísérletként tartjuk meg. A full EVI-export a
2 GiB tmpfs kapacitásán megbukott; nem COMPLETE. Az eredeti MCAP-bájtok
SHA256-ellenőrzött gzip archívumban megmaradtak; a két felhasználói kiinduló
capture változatlan.

A geometriajavítás utáni 15 s próba capture-je:
`runtime/captures/v3_20261115_180347_13809_capture.mcap`. A robot a korábbi
fizikai helyről elindult, 3,62 m odometriai utat tett meg, egy célt elért,
a következő felé haladt. Aktív szakaszon nincs fault vagy safety-beavatkozás.
Az indulás utáni teljes állás 2,93%, hátramenet 1,54%; a pozitív sebesség
mediánja **0,233 m/s**, így a 0,45 m/s cél még nem teljesült.
Mind az **1513 tick folyamatos native replayje MATCH**, explicit EVI COMPLETE.

A következő hosszú próba függőségi indítása mozgás előtt megbukott: a ROS
környezet aktiválása túllépte a 10 s host-timeoutot, Nav2-child még nem indult.
Az önálló, motor nélküli környezeti előellenőrzés utána 0,165 s alatt PASS.
Az átmeneti lassulás oka nem bizonyított; nem tekintjük kijavított runtime-
vagy navigációs hibának.

STOP-only runtime-indítás után a canonical státusz RUNNING/ready, fault nélküli
állapotot igazolt, aktív Nav2 lifecycle-lal; a scan/odom/TF kora 92/36/15 ms
volt. Ebből indult a következő `r rc 120`. Capture:
`runtime/captures/v3_20261115_181125_15022_capture.mcap`.

**Ez a 120 s kísérlet sem sikeres.** A capture utolsó tickje 8192; 19,80 s
EXPLORE után L11 FAULT, majd verified safe-low leállás történt. A kezdeti
STOP-only előkészítéssel együtt 8193 tick szerepel a capture-ben. A Nav2
addig egy célt elért, coverage/recovery-hibát nem jelzett. A mély integritás
PASS, a lezárt capture runtime-státusza FAULT, replay-core-ja teljes.

Az első hibás boundary konkrét értékei:

- Utolsó elfogadott bal/jobb él: `5667124462168` / `5667126107121` ns.
- Mindkét A/B callback-receipt és számláló ugyanott megáll; bal/jobb
  számláló 3490 / 3739. Az új, 50 Hz-es snapshotok ezt követően is
  friss timestampet és nulla impulzusdeltát közölnek, olvasási hiba nélkül.
- Közben a nyers IMU headingje változik; a yaw-rate kb. 0,9–1,5 rad/s.
  Az L12 még ALLOW, a jobb kerék outputja kb. 0,30-ról 0,44-re nő.
- A fault tick: `5667641481476` ns, encoder-acquisition age 6,1 ms,
  az utolsó fizikai él kora több mint 500 ms. Hiba:
  `L11 feedback remained uncertain too long: MISSING_EDGE:left;MISSING_EDGE:right`.
- A 8100–8192 tickre indított canonical checkpoint replay mind a 93 ticken
  **MATCH**, a faulttal együtt. Ez az L11 helyes determinisztikus leállását
  igazolja; az encoder inputkiesés hardveres vagy natív callback/transport
  eredetét nem dönti el. Watchdog- és frissességlazítás nem történt.

Motor és GPIO-claim nélküli, privát könyvtáras kétprocesszes lgpio-próba
ellenőrizte a közös FIFO-név hipotézisét. Az egyforma `.lgd-nfy0` név mögött
eltérő nyitott FD-inode volt (594102 / 594104), tehát a feltételezett közös
FIFO-olvasás nem igazolódott. Erre nem épült production módosítás.
A fizikai elakadás/táp/kézi beavatkozás és a callback-thread tényleges
állapota a capture-ből nem dönthető el; további mozgás helyett ez a következő
szükséges megkülönböztető mérés.

## Validáció és artifactok

- Release gate: `./r test release`, 24 PASS a végső ROS-QoS módosítás után.
- Goal starvation, aktuális map revalidation, wheel projection és recovery
  célzott pack-tesztek; natív adapter build sikeres.
- Euklideszi geometria/diagnosztika: 27 célzott teszt PASS; további 100
  rögzített seedű native/Python sweep-eset azonos. Független sűrű pontos
  távolságtesztben minden 0,10 m alatti eset elutasított.
- Simulator pack: 21 PASS, beleértve a hamis replay-MATCH és a hiányzó
  encoder/relative-lineage regressziókat.
- `git diff --check -- v3 tools conf tests docs` PASS. Az élő ROS-futások
  generált logjai változatlan nyers evidence-ként megmaradtak.

Részletes nyers eredmények és scriptjeik:
[`tmp/cruise_1516_1518_analysis`](../tmp/cruise_1516_1518_analysis/).
A nagy, kizárólag e feladatban generált evidence- és szimulációs könyvtárak
veszteségmentes, minden fájlra hash-ellenőrzött `.tar.gz` archívumban vannak;
a kis result/replay/config JSON-ok külön is olvashatók.

Nyitott: hosszú fizikai room-cruise ismételhetősége, 0,45 m/s körüli fizikai
sebesség, a megmaradt frissességi megszakítások első késő boundaryja,
a tényleges kerékkövetés/jerk és a szokásos többakadályos recovery-helyzetek.
Az új közvetlen akadály a két encoder egyidejű fizikai edge-/callback-kiesése;
ennek oka és javítása még nincs igazolva. A kívánt kétperces fizikai
sikerkritériumot nem értük el.
