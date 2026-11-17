# Szenzor- és Nav2-helyreállítás, 2026-11-17

A vizsgálat a production adapterekből és az aktív `conf/vezerles.json`
bekötéséből indult. Az architekturális authority változatlan: a V3 contract
2.2 szakasza és az async runtime contract érvényes. A módosítás nem változtat
motor-, command-, mission- vagy safety-authorityt.

| Terület | Ellenőrzött probléma és javítás |
| --- | --- |
| BNO055, P0 | Két egymást követő átmeneti I²C-hiba lezárta az ownert. Most a supervisor újranyitja a buszt új processzben és új shared transporttal. Az eredeti mérési idő megmarad; a kalibrációt az új minta saját adatai alapján ellenőrzi a source. |
| L12/resident, P0 | Az átmeneti hiba `FAILED`/exception úton terminal FAULT-ot okozott. A bounded recovery most a source-ban minta nélküli `DEGRADED` inputtá zárul. A meglévő L12 STOP megtartja az ACTIVE missiont; exhaustion, hibás contract és valódi failure továbbra is reteszel. |
| Encoder, P1 | A child kiesése továbbterjedt. Most új mailbox/processz készül; az új baseline nem publikálható igazolt sebességként. A számlálógeneráció explicit, a source sequence monoton marad. |
| L3, P1 | Hosszabb encoder-kiesés kimeríthette a prediction historyt; a nulláról újraindult counter hamis elmozdulást okozhatott. Explicit kiesésnél nincs áthidalt count-intervallum; új generációnál új anchor és LOST lokális minőség szükséges. Az újraigazoláshoz a meglévő független relatív mérési út kell. |
| LiDAR, P1 | A driver meglévő reconnectje közben végleges hibává válhatott a köztes állapot. Most minta nélküli várakozás következik, és csak a kiesés után mért scan térhet vissza. A kompakt state mailbox nem vár pipe-frame-re vagy elhagyott writer-lockra. SLAM backend és raw evidence lane nélküli processzkiesésnél új processz/transport, egyedi producer sequence és friss scan kell. |
| Nav2, P1 | Child-kilépéskor a teljes Nav2-csoport végleg leállt. A meglévő supervisor most együtt reapeli és korlátozottan újraindítja a szervereket és a bridge-et. A régi status/socket nem teljesítheti az új PID readinessét. A külső SLAM/map processz változatlanul a saját ownerénél marad. |
| Kamera/személydetektálás | A Room Cruise kritikus eszközhalmazában nem szerepel. A meglévő személyfüggő demand-, generation- és fail-closed viselkedést a release scenario ellenőrzi; ezt a változtatás nem írja át. |

A recovery-keret hibasorozatonként korlátozott. Egy percen át friss, stabil
működés után újra rendelkezésre áll; a generáció és a teljes restartszámláló
ettől nem fordul vissza. Gyors hibasorozatnál exhaustion következik.

## Konkrét nyitott esetek

- Teljes SLAM-csoport kiesése után az új térkép/TF frame azonossága nincs
  igazolva. A `SLAM_BRIDGE_RESTART_REQUIRES_RUNTIME_RESTART` védelem megmarad;
  automatikus mission-resume-hoz explicit frame-folytonosság vagy az owning
  rétegek teljes újrainicializálási útja szükséges.
- Aktív raw/map evidence queue melletti LiDAR-processzhalálnál a félbemaradt
  queue-frame integritása nem igazolt. A recovery itt explicit új capture
  sessiont kér. A driver saját, ugyanabban az ownerben végzett átmeneti
  reconnectje továbbra is helyreállhat.
- A legacy lokális LiDAR-map backend processzének újraindítása külön
  frame-/map-revalidációt igényel. Az aktív production backend SLAM Toolbox.

## Szoftveres evidence és korlát

- `./r test`: 18 sikeres teszt.
- `./r test release`: 28 sikeres teszt, köztük encoder crash/stall,
  személyfüggő capability-loss, Nav2 safety/expiry és natív replay.
- Az új `test_sensor_recovery.py` és `test_nav2_process_recovery.py`: 10 sikeres
  eset; tényleges processzkilövés fake I/O-val, elhagyott publisher-lock,
  reconnect, keretkimerítés/visszaállítás, resident mission-megtartás.
- A kapcsolódó IMU/encoder/LiDAR/Nav2 adaptercsomag végső célzott futása:
  100 sikeres teszt. Egy korábbi opcionális Nav2-teszt régi TTL-számot rögzített;
  most az eredeti source-idők lejárati kapcsolatát ellenőrzi az aktuális policyval.
- Két szintetikus, lezárt recovery-input sorozat natív capture → Replayer
  eredménye MATCH: IMU hold/resume és hosszú encoder-kiesés/generációváltás.

Élő hardver-, ROS- és mozgásteszt nem futott. A fizikai újrakalibrálás,
menetminőség, valós terhelés alatti jitter és hosszú idejű hardveres
helyreállíthatóság nincs bizonyítva e szoftvertesztekkel.
