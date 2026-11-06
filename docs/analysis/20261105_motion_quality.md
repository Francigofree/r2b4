# Room Cruise és 5 Hz-es 2D helyzetkép – 2026-11-05

A `20:22:25` futás teljesítette a kért időtartamot (**120,060 s** acknowledged
végrehajtás), de a navigációs mozgásminőség **FAIL**. A robot a futás második
részében tartósan nulla mozgást kért. A runtime `PASS / SHUTDOWN_SAFE_LOW`
eredménnyel, fault nélkül zárult; ez nem teszi sikeressé az AMR-elfogadást.
Az újabb javítás és annak külön 120 s-os élő validációja még függőben van.

## Tartós, futásból készített nézet

- [2D akadálymegfigyelések és becsült robotút](../../runtime/atlas2d/v3_20261105_202225_10912/atlas2d.svg)
- [Mintavett útvonal CSV](../../runtime/atlas2d/v3_20261105_202225_10912/path.csv)
- [Vetítési lineage és scope](../../runtime/atlas2d/v3_20261105_202225_10912/report.json)
- [Rövid validációs összesítés](../../runtime/atlas2d/v3_20261105_202225_10912/validation_summary.json)

A szürke pontok a rögzített, legfeljebb 96 pontos LiDAR-megfigyelések történeti
unióját, a sötét pontok az utolsó vetíthető scant mutatják. A kék vonal mintavett
L3-helyzetbecslés. A lila H-jelölések a leghosszabb **mintavett planner hold**
sorozatok kezdetét jelölik, tickkel, capture-relatív idővel és okkal; önmagukban
nem bizonyítanak fizikai beszorulást vagy a minták közötti teljes mozdulatlanságot.

Ez megfigyelt visszaverődési pontfelhő, **nem a planner occupancy térképe**.
Fehér területből nem következik szabad hely. A vetítés a scan measurement idejére,
azonos frame/generation alatt interpolált, minőség szerint elfogadható lokális
pózt használ; a legnagyobb bracket 250 ms. Nincs extrapoláció, scan-deskew,
új SLAM, történeti Atlasz-illesztés vagy második LiDAR-extrinsic alkalmazás.

## Capture és offline evidence

Authority: [v3_20261105_202225_10912_capture.mcap](../../runtime/captures/v3_20261105_202225_10912_capture.mcap).
SHA256: `31001fbd68a268461ea7f2f4bcfed3fc03b31c7107ec73603c5d6f79937d3869`.

Az explicit MCAP Evidence Compiler **COMPLETE**, a source-hash ellenőrzés
**PASS**: 6015 message, quarantine 0. FULL 5 Hz scope: **624 mintavett tick**;
**618/618** map observation beérkezett, superseded 0, transport end megérkezett,
`navigation_map_complete=true`. Raw és exact replay **NOT_REQUESTED**.

Mind a 624 lokális póz minőség szerint elfogadható, generation 0. A renderer
615 scanből **59 040** pontot vetített. Két scan ugyanabba a 256,781 ms-os
pózbracketbe esett, egy a legutolsó rögzített póz után érkezett; e három scan
vetítését a tool kihagyta. Ez különbözik a veszteségmentesen lezárt map transporttól.

## Az első hosszú hold

Az acknowledged start `4189750401469 ns`. Az első hosszú nulla-kérésű sorozat
natív boundaryja **tick 1781 / 4226390411635 ns**, azaz **+36,640 s**:
`L6 PLANNER_EXECUTION_HOLD` → L8 nulla kérés → L9 nulla engedély → L12 nulla output.
A belépéskor nincs jelölt; `motion_revoked=true`.

A dense decision események ugyanebben a sorozatban **83,521 s** folyamatos
nulla-kérést rögzítenek. A `NO_PROGRESS_EXHAUSTED_HOLD` **tick 2671 / +68,661 s**
mellett kezdődik, és az utolsó 51,401 s-os okepizódot adja. Az elemző öt hosszú
okepizódja ugyanennek a holdnak az ok-/célváltásait bontja szét; közöttük nincs
rögzített pozitív újraindulás.

Az aktív küldetés kvalifikált mintavett becsült útja **10,388 m**. A teljes
capture mintavett allowed-motion ideje **34,542 s**, lineáris/szögsebesség tracking
RMSE **0,0472 m/s / 0,1175 rad/s**. A 83,401 s-os mintavett gyenge-transzlációs
sorozatban csak 0,0030 m becsült elmozdulás van. Ezek L3-becslések, nem független
fizikai ground truth. Lokális/heading LOST megfigyelés 0; legnagyobb host status-kor
205 ms. Native control p99/max **38,2 / 104,629 ms**; periódus p99/max
**56,4 / 116,502 ms**.

## Ellenőrzés és nyitott munka

A capture előtti release gate **23 PASS**; map capture teszt **8 PASS**, benne
a raw 50 Hz-es canonical STOP replay **MATCH**. A kapcsolódó geometria- és
reselection teszt **6 / 8 PASS**, a kis gate **14 PASS**. Az offline renderer
közvetlen tesztje **22 PASS**, valódi MCAP → EVI → SVG/CSV úttal, minőségi és
frame/generation határokkal, extrinsic- és yaw-interpolációval, valamint
mintavett hold-jelöléssel.

A pontfelhő és útvonal segíti a konkrét akadály-cellára visszavezetett elemzést;
a costmap felbontás és a navigációs liveness szűk javítása külön folyamatban van.
E javítások végleges tesztje és új élő futása nélkül nincs új AMR-minőségi PASS.
A történeti Atlasz/current-frame illesztés és a kiindulópontra visszatérés továbbra
sem bizonyított.
