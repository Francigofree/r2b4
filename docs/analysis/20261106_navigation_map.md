# Navigáció és térképi bizonytalanság – 2026-11-06

A november 5-i `20:22:25` futás SVG-je történeti LiDAR-pontunió, nem a planner
pillanatnyi akadálytérképe. Az egyes scanek a 83,5 másodperces megállás alatt
stabilak maradtak (egymást követő pontfelhők medián eltérése 6,82 mm). A mozgó
időablakokban sincs monoton romlás. A jobb oldali kaotikus rajz ezért önmagában
nem bizonyít folyamatos szenzor- vagy occupancy-romlást.

Van viszont konkrét lokalizációs probléma: az L3 által jelzett felhalmozott
encoder–LiDAR transzlációs eltérés elérte a **0,433 m** értéket
(`tick 1407`, +27,705 s). Ez konzisztenciajelzés, nem független ground truth.
Az L4 korábban a még GOOD rövid távú minőség mellett tovább használhatta a
30 másodperces strukturális falemlékezetet. Közvetlen production L4 teszttel
reprodukálható, hogy ilyen figyelmeztetés mellett egy régi fal visszakerül a
helyi térképbe. Az eredeti futás konkrét akadályrácsa nem volt rögzítve, ezért
annak teljes 83,5 másodperces holdja nem vezethető vissza kizárólag erre az okra.

## Változások

- Felhalmozott transzlációs eltérés jelzésénél az L4 ugyanabban a tickben
  törli és kizárja a hosszabb strukturális emlékezetet; a friss, 0,75 s-os
  térkép és a robothoz viszonyított érzékelés megmarad.
- A helyi térkép 5 cm-es cellái és az L6 végrehajthatatlan célcsaládjának
  újraválasztása a korábban azonosított sarok-/hold-problémát javítja.
  A fizikai 5 cm-es ráhagyás és az L12 védelem megmarad.
- Az 5 Hz-es capture a completed L4 tényleges celláit is rögzíti, pontos
  tick/frame/revision/source lineage mellett. A producer legfeljebb 4096 cella
  kompakt skalárpufferét készíti; a JSON és a kibontás az observation consumer
  feladata, a meglévő transporton. A hiányzó térkép explicit unavailable.
- Az offline Atlasz külön kezeli a történeti visszaverődéseket és a tényleges
  helyi akadályrács idővonalát. A lokalizációs figyelmeztetés állapotváltása
  külön döntési eseményként is megmarad a minták között.

## Szoftveres evidence

A release gate **23 PASS**. A capture célzott ellenőrzése **16 PASS**, benne
direct/process azonosság, pontos cellák és lineage, 5 Hz-es packingkorlát,
üres/hiányzó térkép, no-demand/c50 út és canonical evidence compiler/source
ellenőrzés. A scalar snapshot codec **18 PASS**; a warning-váltás megtartása
**1 PASS**. Az L4 régi fal törlésének két regressziója és a friss lokális
mozgás megtartásának ellenőrzése sikeres.

1200 cella packingja a Pi-n medián **0,234 ms**, maximum **0,251 ms**;
a kompakt wire pickle medián 0,018 ms. Ez a publisher mérése, nem a teljes
élő futás időzítési eredménye. Az 5 Hz-es napló nem exact replay scope.

## Élő ellenőrzés és nyitott munka

Az új változat motor nélküli capture-próbája és kétperces Room Cruise futása
még hátravan. Az odometria felhalmozódó eltérésének korrekciója, a történeti
Atlasz ellenőrzött visszaillesztése és a kiindulópontra visszatérés nincs
bizonyítva. M5–M6 teljesítését ezek nélkül nem állítjuk.
