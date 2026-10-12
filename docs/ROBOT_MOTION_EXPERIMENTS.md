# Mérhető robotfejlődés: DIAG, szimuláció és LCE

A fejlesztés célja, hogy egy skill javítása összehasonlítható mérést adjon,
és az LCE meg tudja nevezni a következő szükséges próbát. A V3 réteg- és
async-contract változatlan: a mozgást a canonical command gateway és az
L0–L12 lánc realizálja, a diagnózis és a tanulási napló passzív fogyasztó.

## Megvalósítási terv

1. A meglévő EVI-ből egy `motion_performance` DIAG analyzer számít időben
   összekapcsolt mozgásmutatókat. Minden intervallumhoz ellenőrzi az időt,
   a source tick azonosságát és a lokális pose folytonosságát. Nem interpolál
   át recording-gap, frame-váltás vagy localization-generation váltás fölött.
2. A meglévő motormentes skill-szimulátor differenciálhajtású modellt kap.
   A valódi SDK/socket/skill-worker/Brain út indítja a canonical V3
   parancsokat. Az offline motor sink kimenete hajtja a fizikai modellt;
   a modell nem implementál külön navigation vagy safety algoritmust.
3. Azonos környezetben és seed mellett páros baseline/változat próbák
   készülnek. Több explicit scenario teszteli az általánosíthatóságot.
   A javulás, a regresszió és az elégtelen evidence külön eredmény.
4. Az LCE a konkrét sikertelen checket, a mozgásméréseket és az artifact
   hivatkozásokat kapja vissza. A szimulációs célteljesülés élő validációs
   igényt eredményez; a cél funkcionális fejlődésének bizonyítása élő,
   source-ellenőrzött mozgásevidence-et kíván.
5. Az élő terv megnevezi a skillt/forrásváltozatot, a canonical lépéseket,
   a térbeli/időbeli korlátokat, a mért célfeltételeket és a STOP szabályt.
   Végrehajtás előtt az agent a felhasználó kifejezett engedélyét kéri.
   A tervező tool nem indít runtime-ot vagy motort.
6. A tanulási állapotváltások a meglévő host journal → capture follower →
   MCAP eseményútra kerülnek. A rendszerállapot 10 Hz-es mintavételű;
   a ritkább kezdés, mérési eredmény, összehasonlítás és engedélykérés
   eseményenként megmarad, saját idővel és artifact-hivatkozással.

## A mérések jelentése

| Kérdés | Evidence |
| --- | --- |
| Követi-e a robot az engedélyezett sebességet? | Előző L9 referencia és következő L3 becslés időben párosított hibája, coverage és RMSE/bias. |
| Hol korlátozódik a teljesítmény? | Kért/engedélyezett mozgás, constraint/safety STOP és actuator saturation időtartama. |
| Egyenletes-e a mozgás? | Sebesség- és fordulási változások, gyorsulás, stop/start és irányváltás a megfigyelt mintákból. |
| Halad-e a célhoz? | Folytonos lokális pose útja, elmozdulása és az elérhető mission célhoz tartozó hiba/progress. |
| Skill-logika vagy V3/plant/környezet? | Első hibás check/boundary és parancs–válasz mérés, majd azonos feltételek között egy tényezőt változtató próba. Ez vizsgálati hipotézis, nem automatikus root-cause bizonyítás. |
| Jobb-e az új változat? | Páros, azonos environment/seed/oracle mellett mért eredmények; külön jelzett regresszió és hiányos próba. |
| Érdemes-e módosítani? | A baseline először mérődik; megfelelő eredmény vagy nem bizonyított javulás önmagában nem indokol javítást. |

L3 sebesség és pose becslés. Ezekből számított út és tracking-hiba nem
független chassis-sebesség/úthossz mérés. A szimuláció ground truth-ja
független a V3 pose-becslésétől, de csak a megadott modellhez tartozik.
Modellezési eltérés, talaj, valódi motor és szenzor bizonytalansága élő
próbával vizsgálandó.

## Capture és visszaellenőrzés

A 10 Hz-es behavior capture kis scalar állapotot és tanulási eseményeket
tartalmaz. Nincs benne raw szenzorhalmaz vagy replay-checkpoint, és
nem igazol exact replayt. Az explicit 50 Hz-es sensor-debug capture
teljes lezárt inputot rögzíthet, és a natív Replayer ellenőrizheti.
Mindkét scope EVI compilerrel ellenőrizhető; a compiler teljessége,
a capture integritása, a replay és a robotikai célteljesülés külön állítás.

Új átmeneti artifactok egyedi `/tmp` könyvtárba kerülnek. A teljes
measured artifact és SHA256 elérhető marad; a kis LLM-visszajelzés
explicit omissiont és lekérdezési hivatkozást használ nagy adatnál.
A szimulációs eredmény nem kerül fizikai siker-mintaként az OutcomeLearner
robot-tapasztalati priorjába.

## Validációs sorrend

A célzott tesztek mérik az időbeli összerendelést és gap/frame/sequence
hibákat, a valódi socketes szimulációt és a független cél-oracle-t,
a páros összevetés regresszióit, az engedélyt kérő motormentes toolt,
valamint a 10 Hz-es direct/process MCAP esemény-egyezést és EVI lineage-et.
Ezután a canonical kis gate és a kapcsolódó közös boundary miatt a release
gate fut. A permanent tesztmanifest nem bővül automatikusan.

Meglévő releváns élő capture offline kiértékelhető új motorindítás nélkül.
Új élő mozgás kizárólag az aktuális feladatban kifejezetten engedélyezett,
konkrét próbára indulhat. Váratlan safety/fault/process/timing eredmény
után a próba megszakad; automatikus újraindítás nincs.
