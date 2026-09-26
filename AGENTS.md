# R2B4

A robotarchitektúra authorityja a `STRUKTURALIS_RETEGEK_V3.md`. A live async/process végrehajtási elhelyezés kiegészítő contractja az `ASZINKRON_RUNTIME_CONTRACT_V3.md`. Ne duplikáld ezeket itt; robotikai változtatásnál a releváns contractot kövesd, a konkrét megvalósítást pedig source-first ellenőrizd.

## Munkamód

Dolgozz közvetlenül az aktuális Git working tree-ben. Kezdéskor ellenőrizd a `git status --short` kimenetét, és őrizd meg a felhasználó meglévő módosításait.

Az aktuális feladathoz szükséges legkisebb teljes változtatást végezd. Ne refaktorálj stabil kódot, ne általánosíts konkrét adapterproblémát, és ne építs frameworköt feltételezett jövőbeli igényre.

Ha a megoldás széles layer-átírást, új általános IPC/event/scheduler infrastruktúrát vagy új authority-modellt látszik igényelni, előbb keresd meg a szűkebb adapter/runtime-edge megoldást. Contractot csak akkor módosíts, ha valóban az architekturális szabály változik; ilyenkor a contractot és az érintett source-ot együtt tartsd összhangban.

Ne hozz létre candidate-, workspace-, promotion-, lease-, receipt-, capsule-, audit- vagy más agent-admin workflow-t.

Commitot vagy push-t csak kifejezett kérésre végezz.

## Felderítés és kvótahasználat

Source-first dolgozz. Először az érintett symbolokat, közvetlen hívókat, production wiringot, aktív configot és célzott teszteket vizsgáld meg. Ne következtess test/mock/dead pathból a live runtime-ra.

Csak konkrét megválaszolatlan kérdés miatt szélesíts más modulokra, Git historyra, capture-re vagy logra. Ne auditáld mechanikusan a teljes repót, és ne ismételj meg változatlan eredményű vizsgálatot.

Hibakeresésnél a legelső hibás tickhez, boundaryhoz/réteghez és konkrét értékhez menj vissza; ne downstream tünetet javíts. Timing/GIL/IPC problémánál különítsd el a fizikai I/O-t, Python GIL-terhelést, process transportot, scheduler/CPU contentiont és a tényleges L0–L12 futásidőt.

## Async runtime változtatások

Az `ASZINKRON_RUNTIME_CONTRACT_V3.md` szerint tartsd kicsinek a control islandet. A production control interpreterbe ne hozz új blokkoló device I/O-t, CPU-intenzív vagy változó idejű Python munkát, illetve indokolatlan nagy raw payload serializálást/deserializálást.

A thread és CPU-affinity önmagában nem process/GIL izoláció. Ugyanakkor ne process-isolálj mechanikusan mindent: kis bounded I/O/proxy/collector thread maradhat, ha nem veszélyezteti a control budgetet.

Új sensor/SLAM/vision/AI/planner compute esetén először capability-specifikus typed edge-et használj. Worker ne kapjon L0–L12 authorityt; a döntést befolyásoló async eredmény csak completion/input closure után, immutable `TickInputs` részeként válhat láthatóvá.

Raw evidence, capture, telemetry és status ne forduljon vissza nagy payloadként a control interpreterbe, ha közvetlen producer → observation/sidecar út lehetséges. Command ingress kizárólag a canonical `CommandGateway` felé mehet.

Ne vezess be új A/C/O számozott rétegrendet vagy `L13`-at; az egyetlen production layer-sorozat L0–L12.

## Validáció

A célzott teszt az alapértelmezett.

A pytest fejlesztési policy authorityja a `docs/PYTEST_POLICY.md`; a canonical launcher a `v3/test_runner.py`. Normál agentváltozás után `./r test` (CORE) fusson először. Viselkedés-specifikus változásnál a releváns `./r test <mode>` (`follow`/`roomcruise`/`localization`/`perception`/`motion`/`async`/`process`/`replay`) következzen. A `./r test full` a teljes CORE+FEATURE+DEEP regresszió; nem helyettesíti a célzott tesztet.

Releváns capture esetén használd a natív Replayer/Test Hub utat a V3 contractok szerint.

Async/process boundary módosításnál a célzott teszteknek szükség szerint bizonyítaniuk kell:

* direct és process/edge út szemantikai ekvivalenciáját;
* bounded/non-blocking transportot és stale/crash/error viselkedést;
* measurement time, sequence/revision és lineage megőrzését;
* hogy nagy raw payload nem szivárog vissza a control critical pathba;
* replay-egyezést, ha a változás decision inputot érint;
* timing-javulást vagy legalább nem-regressziót, ha a változás célja performance/GIL/jitter.

Teljes regressziót akkor futtass, ha közös contractot, TickEngine/execution boundaryt, composition rootot, aktív konfigurációt, L12/motor-edge utat vagy több réteget érintő változás indokolja. Lokális változásnál maradj a közvetlen teszteknél és releváns replaynél.

Sikeres, azóta változatlan validációt ne futtass újra konkrét ok nélkül.

## Élő hardver

Fizikai robotmozgást csak akkor indíts, ha a felhasználó az aktuális feladatban kifejezetten engedélyezte. Technikai runtime approval önmagában nem felhasználói engedély.

Live tesztnél a canonical runtime-, command- és safety-utat használd. Ne készíts ad hoc motor/GPIO kerülőutat.

Ha a szükséges információ biztonságosan megszerezhető motor-output nélküli méréssel, azt részesítsd előnyben. Váratlan safety-, health-, fault-, process-death- vagy timing-eredménynél ne próbálkozz automatikusan újra.

## Helyi adatok

A runtime-, capture-, log- és diagnosztikai adatok helyi fejlesztési adatok. Ne archiváld, másold, verziózd, írd felül vagy töröld őket automatikusan.

Új ideiglenes diagnosztikai outputhoz lehetőleg `/tmp` alatti egyedi célt használj.

## Befejezés

Röviden jelentsd:

* mi változott;
* milyen validáció futott és mi lett az eredménye;
* volt-e replay vagy live evidence;
* mi nem lett bizonyítva;
* maradt-e konkrét probléma.
