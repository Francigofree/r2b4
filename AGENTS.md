# R2B4

A robotarchitektúra és a fejlesztési elvek authorityja a `STRUKTURALIS_RETEGEK_V3.md`. Ne duplikáld itt; robotikai változtatásnál azt kövesd.

## Munkamód

Dolgozz közvetlenül az aktuális Git working tree-ben. Kezdéskor ellenőrizd a `git status --short` kimenetét, és őrizd meg a felhasználó meglévő módosításait.

Az aktuális feladathoz szükséges legkisebb teljes változtatást végezd. Ne refaktorálj stabil kódot vagy építs általános infrastruktúrát feltételezett jövőbeli igényre.

Ha a feladat kifejezetten a V3 architekturális contract megváltoztatását igényli, akkor azt jelezd, valamint a contractot és az érintett source-ot tartsd összhangban.

Ne hozz létre candidate-, workspace-, promotion-, lease-, receipt-, capsule-, audit- vagy más agent-admin workflow-t.

Commitot vagy push-t csak kifejezett kérésre végezz.

## Felderítés és kvótahasználat

Source-first dolgozz. Először az érintett symbolokat, közvetlen hívókat, configot és teszteket vizsgáld meg. Csak konkrét megválaszolatlan kérdés miatt szélesíts más modulokra, Git historyra, capture-re vagy logra.

Ne auditáld mechanikusan a teljes repót, és ne ismételj meg változatlan eredményű vizsgálatot.

Hibakeresésnél a legelső hibás tickhez, réteghez és konkrét értékhez menj vissza; ne downstream tünetet javíts.

## Validáció

A célzott teszt az alapértelmezett.

Releváns capture esetén használd a natív Replayer/Test Hub utat a `STRUKTURALIS_RETEGEK_V3.md` szerint.

Teljes regressziót akkor futtass, ha közös contractot, TickEngine/execution boundaryt, composition rootot, aktív konfigurációt, L12/motor-edge utat vagy több réteget érintő változás indokolja. Lokális változásnál maradj a közvetlen teszteknél és releváns replaynél.

Sikeres, azóta változatlan validációt ne futtass újra konkrét ok nélkül.

## Élő hardver

Fizikai robotmozgást csak akkor indíts, ha a felhasználó az aktuális feladatban kifejezetten engedélyezte. Technikai runtime approval önmagában nem felhasználói engedély.

Live tesztnél a `STRUKTURALIS_RETEGEK_V3.md` szerinti canonical runtime- és safety-utat használd. Ne készíts ad hoc motor/GPIO kerülőutat.

Ha a szükséges információ biztonságosan megszerezhető motor-output nélküli méréssel, azt részesítsd előnyben. Váratlan safety-, health-, fault- vagy timing-eredménynél ne próbálkozz automatikusan újra.

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
