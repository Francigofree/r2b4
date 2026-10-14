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

## Használat

A [single scenario](examples/skill_learning_motion.json) egy 0,4 m-es,
független célt ír elő. A [scenario suite](examples/skill_learning_motion_suite.json)
ugyanezt vizsgálja négy környezetben: névleges hajtás, csökkent tapadás,
lassabb kerékválasz és jobb oldali hajtáseltérés. A seed és a cél azonos;
környezetenként egy plant-tényező változik.

Egy már elmentett, például `relative_goal` nevű skill szimulált tanulása:

```bash
./r learn "Érd el a független 0,4 m-es célt minden megadott környezetben" \
  --skill relative_goal --scenario docs/examples/skill_learning_motion_suite.json \
  --max-iterations 3 --timeout-s 120 --json
```

Az LCE először az eredeti skillt méri. Ha az már megfelelő, nincs javító
LLM-hívás. Sikertelen baseline esetén a javított source ugyanazokat a
scenario-kat kapja; a `learning.compare` csak lezárt, source-ellenőrzött,
replay-egyező párokat fogad el. A sikertelen rövid mozgás ideje nem
hatékonysági baseline egy célt elérő hosszabb mozgáshoz. Az idő és a
tracking-minőség összevetése két sikeres próba között értelmezhető.
Hibával vagy megszakítással végződő worker-futás nem lehet pozitív
változtatási ajánlás vagy élő siker alapja, akkor sem, ha előtte mozgott
vagy elérte a modellbeli célt.
A `learning.json` minden iteráció eredeti source-szövegét és SHA256-át
megőrzi. A `learning.read` `/iterations/0/source` pointere visszaadja a
baseline forrását. A sikertelen vagy regressziós javítás elmentett változata
kézzel felülvizsgálható; a ciklus nem írja automatikusan vissza a baseline-t.

A fizikát használó próba sikerének állapota `LIVE_VALIDATION_REQUIRED`.
A CLI ezt nem tekinti befejezett fizikai tanulásnak, ezért nem nulla exit
code-dal tér vissza. A teljes `learning.json`, `measured.json`, per-trial
50 Hz-es MCAP/replay/EVI és a tanulási folyamat 10 Hz-es `learning.mcap`
az eredményben megadott `/tmp` könyvtárban található.

Az agent toolok szerepe:

* `learning.trial`: explicit motormentes mérés egy scenario-val vagy suite-tal;
* `learning.read`: teljes artifact lekérdezése SHA256-tal és JSON pointerrel;
* `learning.compare`: azonos környezetekben mért baseline és változat;
* `learning.live_plan`: konkrét canonical mérőpróba, source-hash, fizikai
  kalibráció szerinti sebességkorlát, timeout, STOP és engedélykérdés;
* `learning.live_feedback`: eredeti production MCAP, azonosított skill-run
  és source, boot-clock szerint kötött mérési intervallum, DIAG-feltételek.

Az élő terv a benne közölt külön mérősource-ra vonatkozik; a hivatkozott
eredeti skillt nem futtatja. A relatív cél-eltolás nem teljes úthosszkorlát.
Engedély után a felülvizsgált `bounded_execution.source` használandó:
timeout és minden megszakítás után canonical STOP, második próbálkozás nélkül.
Régebbi capture boot-azonosító nélkül továbbra is elemezhető, de a
`learning.live_feedback` nem igazol belőle skill-runra kötött élő sikert.

A szimuláció `learning.mcap` fájljában a több próba mozgásmintái kis,
trial-azonosítóval ellátott események. A gyorsított fizika virtuális mérési
ideje külön mezőben marad a host publikációs idejétől. A per-trial 50 Hz-es
capture tartalmazza az exact replayhez szükséges natív inputot. Élő üzemben
a meglévő 10 Hz-es behavior capture rögzíti a natív scalar állapotot és a
tanulási eseményeket.

Az élő tanulás teljes folyamatához a `full` módú, 10 Hz-es capture session
már az engedélykérés előtt induljon. A mérési MCAP-ot az explicit compiler
előtt le kell zárni; az ezt követő értékelési események rögzítéséhez újabb
aktív 10 Hz-es session szükséges. A session-határokon a megőrzött host
journal, run/source/clock azonosítók és artifact-hivatkozások kötik össze
az eseményeket. A public-runtime journal követője a session kezdetén
az aktuális fájlvéghez áll; a korábban publikált engedélykérési eseményt
nem importálja történeti adatként. A voice/HRI út meglévő, azonos boothoz
kötött 60 másodperces lookbackje korábbi eseményeket is megőrizhet, de a
teljes tanulási folyamat rögzítéséhez az aktív session szükséges. A capture
indítása önmagában nem mozgásengedély; mozgás továbbra is csak a konkrét,
felülvizsgált próbára adott felhasználói engedély után indulhat.

## Fejlesztési bizonyítás, 2026-10-14

A valódi SDK/socket → skill-worker → Brain → canonical gateway → V3
láncon egy 0,1 m-t kérő baseline-t javítottunk 0,4 m-t kérő skillre.
A független cél mindkét környezetben 0,4 m volt, azonos seed mellett;
csak a kerékválasz időállandója változott 0,08-ról 0,10 másodpercre.
A javítást egy determinisztikus, scripted AgentCore modellválasz adta:
ez az integráció és a mérési döntés bizonyítása, nem a valódi LLM
általános javítási képességének minősítése.

| Környezet | Baseline végponti hiba | Javított és újrahasznált skill hibája |
| --- | ---: | ---: |
| Névleges | 0,2931 m | 0,0226 m |
| Lassabb kerékválasz | 0,2890 m | 0,0181 m |

Hat teljes, eredeti source-hoz és eredményhez kötött mérés készült.
Mind a hat 50 Hz-es native replay `VERIFIED MATCH`. A páros összevetés
`IMPROVED`, két eltérő fizikai környezet-fingerprinttel. Egy javító
modellhívás történt; a megfelelő baseline újrahasználata nulla hívást
igényelt. Mindkét teljes learning MCAP 10 Hz-es és veszteségmentes,
kezdés/iteráció/mérés/lezárás eseményekkel; a javító futás az összevetést
is rögzítette. A végső státusz
`LIVE_VALIDATION_REQUIRED`, fizikai robotmozgás nélkül.

A relatív SDK próba külön igazolja, hogy 0,1 m célra a canonical 0,01 m
céltűrés jut el a natív commandba, a fordulás relatív yaw-tűrése is
megmarad, és a kezdeti/cél/végső pose azonos lokális keretben van.
A simulator polling producer-vesztését kis pufferrel, eredeti journal
és MCAP sequence/id/time egyezéssel ellenőriztük. A live feedback
FAILED/CANCELLED/FAULT és hiányos eseményscan esetén sem ad pozitív sikert.

Két meglévő 2026-10-14-i production capture és EVI forrásellenőrzése
`PASS`, 10 Hz-es teljes behavior scope-pal. Skill-terminal,
`run_id`/`source_hash` és lezárt skill-időablak hiányában scope-juk
`CAPTURE_WIDE`; belőlük skillfejlődés vagy fizikai pontosság nem igazolt.

A célzott fejlesztői tesztek mellett a canonical kis gate 14 tesztje és
a közös production boundary-k release gate-je 23 teszttel sikeres volt.
A végső simulator pack mind a 16 célzott esete sikeres; az új live feedback
eseményscan-határ a 9 célzott verdict-esettel együtt igazolt.
Fizikai modellhűség, élő funkcionális javulás és nem tesztelt környezetekben
való általánosíthatóság továbbra is külön mérési feladat.
