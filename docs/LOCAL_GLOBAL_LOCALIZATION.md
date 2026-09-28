# Lokális–globális lokalizáció: fejlesztési állapot

2026-09-28. Ez megvalósítási terv és validációs leírás; az architekturális
authority továbbra is a három gyökérkönyvtári V3/rendszercontract.

## Kiinduló hibák és megvalósított lépés

| Terület | Konkrét probléma | Megvalósítás |
| --- | --- | --- |
| P0, L3 heading | Friss gyro és 10 Hz-es relatív LiDAR mellett 6,02 s után `LOST`, mert az abszolút yaw kovarianciája nő. | A relatív heading minősége külön becslést kap a regisztráció megfigyelhetőségéből, az időben halmozott yaw-eltérésből és az utolsó érvényes mérés óta növő bizonytalanságból. |
| P0, L3 encoder | A sebességillesztés `BASELINE` eredménye az ép számlálóalapú odometriát is letiltotta; az aszimmetrikus részleges sebesség hamis slipet jelezhetett. | A friss, működő számlálók kumulatív elmozdulása önálló helyi bizonyíték. Csak megbízható sebesség használható sebességkorrekcióhoz, wheel-rate fallbackhez és wheel/gyro slip-vizsgálathoz. |
| P1, L4 memória | A lokális strukturális memória globális `GOOD` állapotot követelt. | Tanulása és felidézése a helyi transzlációtól, headingtől, folytonosságtól és varianciától függ. A globális coverage külön marad. |
| P1, L6 folytonosság | Egy replacement timeout az előző, még érvényes trajectoryt is törölte. | Csak a sikertelen replacement szűnik meg. A régi pálya eredeti lejáratáig folytatható; a timeout nem hosszabbítja meg. |

Az abszolút EKF-kovariancia és a lokális pose nem kap mesterséges korrekciót a
relatív ellenőrzéstől. A heading bizonytalansági modell az eddigi konfigurált
yaw/rate/process zajokat használja; fizikai kalibrációja még szükséges.
Kis, tartós yaw-eltérés előbb `DEGRADED`, majd `LOST` állapothoz vezet.
Hiányzó vagy degenerált relatív regisztráció nem frissíti az érvényességet;
az öregedés measurement time alapján történik. Az új yaw-konzisztenciaállapot
az L3 checkpoint része.

A `BASELINE` felmentéshez explicit kumulatív elmozdulás és két működő számláló
kell. Hibás timing, stale mérés, device-degradáció és hard rejection továbbra
is megszünteti a helyi odometriai authorityt. A bizonytalanság a tényleges
elmozdulással nő akkor is, ha még nincs használható sebességillesztés.
Az L11 kerékenkénti feedback watchdog változatlanul önálló követelmény.

## Acceptance

- 60 s rate-only gyro, késleltetve érkező 10 Hz-es relatív LiDAR és végig
  `BASELINE` encoder, checkpointból azonos folytatással.
- IMU-/encoder-kiesés, stale és hibás timing, leállt számláló, hard rejection,
  hiányzó kumulatív bizonyíték, degenerált/hiányzó relatív regisztráció és
  halmozódó yaw-eltérés; relatív evidence visszatérésekor helyreállás.
- Natív counter backend → encoder adapter → L3 ellenőrzés: a részleges
  sebességillesztés alatt is helyes kumulatív elmozdulás; a hosszú, ellenőrizetlen
  elmozdulás bizonytalansága nem tűnhet el a hiányzó sebesség miatt.
- L4 memória tanulása és felidézése globális fix nélkül, globális minőségváltás,
  helyi minőségromlás, lejárat és checkpoint.
- EXPLORE/FOLLOW_PERSON/relatív NAVIGATE: pending és timeout alatt érvényes
  trajectory, változatlan lejárat, lejáratkor stop, checkpointból azonos kimenet.
- 601 s zárt, 50 Hz-es L1–L12 szimuláció: rate-only gyro, kvantált kumulatív
  encoder, ismétlődő sebességillesztési szünet, 10 Hz-es helyi scan, ritka globális
  korrekció; a végső szakasz natív replaye checkpointból.

## Következő, méréssel megalapozandó lépések

1. **LiDAR latency/freshness:** a helyi scan/safety már a raw scan ágból jön,
   a relatív eredmény publikációja nem követel globális pose-sikert. A relatív
   regisztráció és a globális matcher viszont ugyanazon workerben számol
   (`v3/lidar_matcher_process.py`). A meglévő fizikai capture 36/346 matcher
   timeoutot jelez. Külön kell mérni a scan mérési korát, worker-futásidőt,
   queue/delivery időt és control-closure késést, mielőtt deadline-t vagy
   végrehajtási elhelyezést változtatunk.
2. **Lokalizációt megőrző tervezés:** L6-ban már van `DEGRADED` sebességskálázás,
   uncertainty inflation és bounded reacquire. A következő bővítés az irányfüggő
   megfigyelhetőség szerinti trajectory-scoring; ehhez előbb hiteles geometriai
   evidence és folyosó/fordulás acceptance szükséges. Heading `LOST` mellett a
   vak forgás nem elfogadható helyreállítás.
3. **Fizikai acceptance:** külön engedélyezett mozgással kell igazolni a zajmodell,
   slip-küszöbök és L11 feedback-tolerancia megfelelőségét, valamint a valós
   stop–start gyakoriság és control timing javulását. A szintetikus teszt nem
   bizonyít abszolút térképi pontosságot vagy közös szenzorbias elleni védelmet.
