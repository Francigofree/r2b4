# Rögzített tapasztalatból mérhető fejlesztés

Az Observer → Capture → MCAP → explicit EVI → DIAG → offline szimuláció
út a meglévő V3 production implementációt használja. A diagnosztika és a
hangolás nem kap command-, safety- vagy motor-authorityt, és nem módosítja
az aktív `conf/vezerles.json` fájlt.

## Felvétel

Új operátori session alapértelmezése folyamatos `full`, 5 Hz-es behavior
capture. A `c alap` továbbra is explicit trigger körüli rövid gyűrű;
a `c nincs` kikapcsolja a felvételt. A már futó resident session beállításai
nem változnak meg egy új kliens csatlakozásakor.

| Adat | Rögzítés |
| --- | --- |
| L1–L12, mission, command, pose, motion, safety, health | 5 Hz-es kompakt állapot |
| Navigációs döntés, cél, terv, elutasítás, constraint, safety változás | Változásra, a mintavételtől függetlenül |
| Brain, behavior, world, skill, RobotInterface esemény | Meglévő upper-runtime eseménynaplóból |
| Agent, voice, ER2 kommunikáció | Meglévő HRI eseményekből |
| CPU frekvencia, CPU-terhelés, load, memória, hőmérséklet, capture CPU/RSS | Capture sidecarban, 1 Hz |

A host telemetria a `/r2b4/event` csatornában `v3.host_telemetry` source
topicon jelenik meg. A nem olvasható kerneladat explicit `unavailable`,
nem nulla érték. Az első CPU-terhelésminta nem tartalmaz intervallumból
számolt kihasználtságot. A monoton measurement-idő, a saját sample sequence
és a külön completion/publication idő megmarad.

A public-runtime journalból a capture indulása utáni események kerülnek be.
Egy korábban indult, azóta változatlan Brain/skill működéshez nincs új
kognitív checkpoint; a HRI út külön, legfeljebb 60 másodperces, boothoz
kötött előzményt követ. A teljes feladatsor rögzítését a feladat indítása
előtt kell megnyitni.

A capture process végzi a kernel-fájlok olvasását, az encodingot és az
MCAP-írást. A control oldal bounded, nem blokkoló, kompakt publicationt
végez. Az alapbeállítás továbbra is kvótázott; túlcsordulás és evidence-loss
explicit integrity hiba. A `full` folyamatos behavior-felvételt jelent,
nem korlátlan lemezhasználatot vagy teljes raw szenzorfolyamot.

Az 5 Hz-es felvétel nem tartalmaz teljes térképet, kamera-/LiDAR-payloadot
vagy minden tick lezárt replay inputját. Exact replayhez explicit `c 50`
sensor-debug felvétel kell. A low-level `McapCaptureConfig` 50 Hz-es
alapértéke ezt a külön fejlesztői API-t szolgálja; az operátori alapérték 5 Hz.

Részletes buffer-, kvóta- és eseményszerződés:
[FLIGHT_RECORDER_P0.md](FLIGHT_RECORDER_P0.md).

## Navigációs hibakeresés

Lezárt felvételből, külön offline lépésként:

```bash
./r evi path/to/run.mcap --output /tmp/run.evidence
./r diag navigation_root_cause /tmp/run.evidence --json
./r diag motion_performance /tmp/run.evidence --json
```

A teljes `./r diag` is tartalmazza a navigációs okelemzést. Az epizódok
source tickhez, missionhöz, measurement-időhöz és EVI message-reference-hez
kötik a megállást, a döntést, a térkép revisionjét, a lokalizációt és a
safety korlátozást. Felvételi rés, missionváltás vagy érvénytelen pose
folytonosság nem hidalható át feltételezett mozgással.

A rögzített döntési mechanizmus bizonyítható: például L6 elutasította az
útvonalakat, vagy L12 STOP-ot adott. Ettől külön kérdés, hogy helyes volt-e
a célpont, valódi volt-e az akadály, és miért lett elavult az adat.
Az ilyen mélyebb ok hipotézis vagy evidence-blocked marad, ha a felvétel
nem bizonyítja. Az L3 mozgásbecslése nem független fizikai ground truth.

Az EVI-integritás, a production run kimenetele, a replay eredménye és az
okelemzés bizonyossága külön mezők és külön állítások.

Az agentnek visszaadott összefoglaló méretkorlátos. Nagy epizód- és
példalisták explicit omission jelzést, hash-t, forráshivatkozást és a
teljes DIAG-hoz vezető pointert kapnak. A capture-integritás, futásazonosság
és replay bizonyítékát ez a projekció nem csonkolja.

## Offline hangolás

A `./r tune auto --help` mutatja a rögzített felvételhez és a megadott
szimulációs környezetekhez kötött keresés opcióit. A keresés a canonical
V3 és a meglévő differenciális hajtású PWM plant segítségével méri össze
a validált, memóriabeli konfigurációváltozatokat.

A [példa környezetsor](examples/navigation_autotune_surrogate.json) egy
szintetikus kiindulópont. Geometriáját és plant paramétereit a vizsgálandó
problémához kell igazítani; a fájl nem állítja, hogy rekonstruálja a szobát.
A `source_capture_sha256` mező az eredeti, lezárt MCAP SHA256 értéke legyen.
Egy keresés legfeljebb 512 változatot és csoportonként 1–4 környezetet enged.
Az alapérték 200 változat, fix seed és 3600 s wall-time budget, amelyet
véges próbák között ellenőriz. A holdout nem lehet pusztán átnevezett vagy
más seedű keresési környezet.

```bash
./r tune auto --capture path/to/run.mcap --evidence /tmp/run.evidence \
  --scenarios /tmp/scenarios.json --variants 200 --seed 7 \
  --budget-s 3600 --output /tmp/r2b4-tuning-new
```

A cél új könyvtár legyen. Az `--evidence` elhagyható; ha megadott,
a teljes EVI-verifikáció és az eredeti capture hash-egyezése szükséges.
A jelentésbe ekkor a rögzített navigációs okelemzés és a mozgásmetrikák is
bekerülnek.

Az eredmény tartalmazza a forrás lineage-ét, az alapkonfigurációt,
a párosított metrikákat, az elutasítások okait és az ígéretes változatok
külön konfiguráció-overlay fájljait. A keresési környezetektől elkülönített
holdout kiértékelés és a kiválasztott szimulációk canonical replay-e
korlátozza az egyetlen próbára optimalizálást. Ha nincs elegendő evidence
vagy nincs biztonságos javulás, nincs automatikusan kijelölt nyertes.

Egy valódi 5 Hz-es capture nem határozza meg a robot környezetének teljes
geometriáját vagy a hajtás fizikáját. Ilyenkor explicit szimulációs
környezet szükséges; annak eredménye surrogate kísérlet. A rögzített
tényeket és a szimulációban mért javulást a jelentés külön tartja.
Egy szimulációs felvétel saját környezete a metadata alapján átmásolható
a keresési csoportba, de ahhoz is külön holdout környezet szükséges.

Az automatikus hangoló launcher útja aktív resident V3 mellett nem indít
CPU-igényes keresést. A program nem telepít konfigurációt és nem indít
élő próbát. Az ígéretes overlay külön átvizsgálható; fizikai teszthez
továbbra is az aktuális feladatban adott felhasználói engedély kell.

## Bizonyítási határ

Az offline tesztek és a canonical replay a szoftveres szemantikát,
integritást és reprodukálhatóságot ellenőrzik. Nem bizonyítanak nulla
CPU-/I/O-terhelést, fizikai robotviselkedés-javulást vagy hosszú élő
futás alatti jittert. Ezekhez külön motor nélküli timing A/B mérés,
majd engedélyezett fizikai acceptance szükséges.
