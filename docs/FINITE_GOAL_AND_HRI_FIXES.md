# A 22:36–22:41 közötti események javítási terve és megvalósítása

A változás a meglévő Brain → RobotInterface → canonical V3 utat javítja.
Új authority, scheduler vagy IPC-rendszer nincs. A motorút, a keréksebesség
fizikai minimuma és a konfigurált 8 cm-es véges céltolerancia változatlan.

## Javítások

| Prioritás | Probléma | Megvalósítás és ellenőrzés |
| --- | --- | --- |
| P0 védelem | Leromlott személydetektor mellett az L6 aktív személykövetési tervet adhatott. A production source a lejárt eredményt és hibás geometriát DEGRADED healthként publikálja. | Az UNKNOWN és DEGRADED health visszavonja a pályát a target prediction és recovery előtt. Friss health újra engedélyezheti a tervezést; FAILED továbbra is missionhez kötött hiba. A canonical camera-demand/generation/replay scenario ellenőrzi. |
| P1 | A közeli célt a teljes rollout végére átlépő pálya negatív heading score-t kapott, ezért elfogyott az összes végrehajtható jelölt. | Közös, bounded helper értékeli a pálya legközelebbi megközelítését. A cél közelében szinguláris bearing büntetése a hátralévő távolsággal súlyozott. Az összes rollout-minta ütközésellenőrzése megmarad. A rögzített végpóz, kis oldalirányú eltérés, direkt/process egyezés és cél mögötti akadály külön tesztelt. |
| P1 | A finite executor hosszú ideig várt mozdulatlan robotra, majd a Brain elvesztette a megszakítás konkrét okát. | 10 s érdemi pózváltozás nélkül NAVIGATION_STALLED, végső canonical STOP és sikertelen goal. Helyben forgás is érdemi változás; teljesülést kizárólag a canonical mission bizonyít. |
| P0 védelem / P1 diagnózis | A finite STOP és runtime shutdown versenye hibának látszhatott. Process-death elfedése ugyanakkor nem megengedett. | Friss PASS/SHUTDOWN_SAFE_LOW terminal report igazolhatja a már lezárt motor-owner megállását. FAULT vagy igazolatlan runtime-eltűnés továbbra is hiba; command identity és eredeti exception/cause megmarad. |
| P1 | A finite action alapértelmezett ALAP beállítása újraindította az aktív FULL felvételt. | A publikus V3 adapter örökli az aktív capture módját és frekvenciáját; explicit felülírás továbbra is lehetséges. A FULL öröklése admissionig tesztelt. Az ALAP továbbra is véges eseményablak, nem hosszú mozgás teljes felvétele. |
| P1 | A terv distance_m és translation_allowed feltételei elutasításra kerültek; a goal leíró mező is hibát okozott. | A Brain validálja a relatív eltolás hosszát, a forgásra korlátozott lépéseket és a mozgás utáni megfigyelési sorrendet. A legacy goal mező kizárólag az eredeti kérés szövegét ismételheti. Az Agent prompt felsorolja a ténylegesen támogatott feltételeket. Ismeretlen védelmi feltétel továbbra sem hagyható figyelmen kívül. |
| P1 | A végrehajtás helyett adott képességhiány-magyarázat teljesített goalként jelent meg. | Új strukturált unfulfilled válasz: megmarad a magyarázat, a Brain REQUEST_UNFULFILLED miatt FAILED, a HRI/CLI action_status sikertelen. A modellválasz értelmezése és a ConversationService út tesztelt. |
| P1 | Korábbi boot HRI eseményei az azonos monotonic tartomány miatt aktuális parancsnak látszottak. Restore friss GOAL_SUBMITTED eseményt is generált. | A journal boot epochot és falióra-rögzítési időt ad. Import csak az aktuális epochból; régi/epoch nélküli történet kimarad, minősítetlen live sor explicit evidence loss. Restore megőrzi a created_ns értéket, és csak restore/interruption eseményt ad. |
| P1 | Nem volt külön LLM-körszám, tényleges modell, provider-attempt vagy vision lineage; a STOP-hiba eredeti szövege elveszett. | LLM STARTED/COMPLETED/FAILED körönként, inference ID, cél, időtartam, configured/actual model, provider-attempt szám. OpenAI esetén elérhető response/request ID és token usage is. Visionnél kompakt mérési lineage; STOP-hibánál exception/cause és command/mission identity. A finite eredményben és interface evidence-ben kért/elért/hátralévő eltolás; frame/generation váltáskor nincs összehasonlítható távolság. Nyers kép és nagy payload nem kerül a control interpreterbe. |
| P1 | A lezárt hiányos capture állapota „még rögzít” volt; az EVI export teljessége összekeverhető a capture/replay teljességével. | FINALIZED_INCOMPLETE és a natív hibák megőrzése; külön sample, raw és replay státusz. Az Evidence Compiler COMPLETE továbbra is az export teljességét jelenti. |
| P1 | A shutdown a még futó publisher mellett várt végleges capture-fájlt, ezért ép felvételnél is figyelmeztetett. | A SIGTERM előtti bounded várás az utóablaknak és a delivery marginnak szól. A natív recorder a publisher lezárása és backlog drain után véglegesít; a host a runtime kilépése után ellenőrzi az MCAP-integritást. Hiányzó vagy hibás végleges artifact ekkor külön figyelmeztetés. |

## Validáció és korlátok

Az opcionális feature/core/deep tesztek mellett a canonical kis gate és a
release gate fut. A szintetikus 1 m-es feladat teljes L0–L12 láncon,
OfflineMotorSinkkel készít 50 Hz-es natív MCAP-ot és hozzá tartozó nyers LiDAR
evidence-t. A natív Replayer eredménye MATCH, az explicit MCAP Evidence Compiler
és verifier eredménye COMPLETE/PASS, karantén nélkül. A szimulált eltolás
0,920976 m: a befejezés a meglévő 0,08 m-es tolerancia szerint történt.

A retained végpózhoz tartozó üres szintetikus scene-ben a baseline kernel
108 jelöltjéből 0 volt végrehajtható; javítás után 54-ből 41. A 100 ismétléses,
váltott sorrendű offline CPU-mérés mediánja 9,415 → 7,290 ms. Távoli célnál
a teljes eredmény azonos, a medián 6,542 → 6,567 ms. Ez rollout-kernel mérés,
nem live L0–L12 időzítés vagy GIL/jitter-bizonyíték.

Ez nem hardveres kalibráció. A korábbi 0,78 m fizikai mérés és 0,8567 m lokális
becslés eltérését ez a refaktor nem méri újra és nem korrigálja találgatott
szorzóval. Ehhez külön engedélyezett, canonical mozgásos mérés kell, több
távolsággal és iránnyal, külső referencia és encoder/lokális/globális pose
együttes összevetésével. A 8 cm-es tolerancia szűkítése szintén külön döntés és
fizikai validáció; a befejezés most sem ígér pontosan 1,000 m-t.

A Brain vision.observe lépése kalibrált képet szolgáltat, beszélt képleírást
nem. A mozgás utáni magyar képleírás ezért továbbra is hiányzó szemantikus
capability. Az Agentnek ezt unfulfilledként kell jeleznie; nem dobhatja el a
feltételt, és a kép elkészülését nem nevezheti a teljes kérés teljesítésének.
A későbbi bővítés helye egy host oldali, bounded megfigyelésértelmező capability:
friss kép → egyszeri LLM-értelmezés → typed szöveg/lineage eredmény → Brain
completion → HRI. Nem periodikus LLM-control, és nem L0–L12 authority.

Fizikai robotmozgás, valódi cloud LLM kérés, commit és push nem történt.
Futó robotfolyamatot ez az offline javítás nem indított újra.
Az eredeti capture-ek és logok változatlanok; a történeti 10 Hz-es, hiányos
felvételekből a refaktor nem készít utólag teljes replay evidence-t.

## Új élő evidence: 2026-10-06 23:47–23:48

A felhasználó által futtatott „menj 1 m-t előre” kéréshez a
`v3_20261006_234748_20434_capture.mcap` teljes mozgásszakaszt tartalmaz.
A canonical navigáció COMPLETE, a Brain MISSION_COMPLETED eredményt publikált.
A megállt lokális póz eltolása kb. 0,935 m, a cél maradéktávolsága 0,06466 m:
a befejezés a konfigurált 0,08 m-es tolerancia szerint történt. A felhasználó
kb. 0,91 m-t mért; ez továbbra sem pontosan 1 m-es fizikai teljesítés, és egy
közelítő mérésből nem állapítható meg kalibrációs szorzó.

A natív container/CRC/digest és sample-integrity PASS, az elő- és utóablak
teljes, required delivery loss nincs. Az explicit offline Evidence Compiler
345 üzenettel COMPLETE, a verifier PASS, 0 karantén. A 10 Hz-es scope nyers
szenzoradatot és checkpointot nem kér, ezért exact replay nem bizonyítható.
Az LLM egyszer, a kérés értelmezésére hívódott: 7,578 s, egy provider- és
transport-attempt; configured Luna, actual Astra. Ez az esemény a capture
8 s-os előablakán kívül van, a conversation/HRI journal bizonyítja.

A tartós host owner (PID 20434) 22:40:42 óta futott, a host source-ok később
változtak. A régi hostmodulok újratöltése nélkül az új interface-diagnosztika
nem validált élőben; a friss V3 process navigációs javítása viszont lefutott.
E vizsgálat nem indított mozgást és nem indította újra a szolgáltatást.
