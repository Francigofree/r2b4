# Production runtime CPU: azonos számítások újrahasznosítása

2026-11-15. A változtatások összehasonlítási alapja a feladat eleji
`f2c854c` source. A méréshez eltett öt eredeti encoder/controller fájl tartalma
ezzel byte-szinten egyezik. A meglévő runtime logok és capture-ök megmaradtak.

A végrehajtott terv: a production wiring és a tényleges konfiguráció
azonosítása; az ismételt munka megszüntetése az érintett adapterekben;
eredményazonosság és CPU mérése; célzott tesztek, release gate, natív
motor-output nélküli SLAM/Nav2 szimuláció, replay és explicit MCAP export.
Nem változott architekturális contract vagy aktív konfiguráció.

## Production út és CPU-felelősségek

| Út | Source és aktív működés | Következtetés / változás |
| --- | --- | --- |
| Control | `v3_process_runtime.py` → `v3_hardware_runtime.py` → `v3_runtime.py`; egy 50 Hz-es TickEngine | Nem költözött encoder, ROS, raw adatfeldolgozás vagy új I/O a control interpreterbe. |
| Fizikai acquisition | Külön encoder- és IMU-processz, mindkettő 20 ms-os mintaperiódussal; LiDAR owner és meglévő bounded IPC | A mintavétel, source idő, sequence, stale/crash kezelés és payload-elhelyezés változatlan. |
| SLAM és ROS | `tools/slam_toolbox.py`, `v3/slam_toolbox_bridge.py`; a LiDAR producer közvetlen scan útja, eredeti mérési idővel | A SLAM bridge és a planner-only bridge node-ja egyszer regisztrálódik az executorhoz. A meglévő 5/2 ms timeout és QoS marad. A teljes navigation bridge már korábban is ezt használta. |
| MPPI | `tools/nav2_planner.py`, `conf/nav2_planner.yaml`, a telepített Nav2 1.3.13; 20 Hz, 400 rollout × 32 lépés, egy iteráció | Nem változott mintaszám, horizont, critic, súly, footprint, CostCritic stride vagy zajpolitika. |
| Végső Nav2 collision proof | `deploy/nav2_wheel_adapter/controller.cpp`, `WheelAwareMPPIController` | Egy mutex-held parancsértékelésen belül közös footprint-alapgeometria, mintánként lusta SAT-tengely cache, és újralátogatott costmap-sorok hard-cell bitmaszkja. |
| Capture | `ProcessMcapCaptureSession` → meglévő bounded queue → `McapCaptureConsumer`; a tárolt felhasználói beállítás 5 Hz/full/ring | Az adatmezők kódolása és DataField tömörítése egyetlen bejárásban készül, behavior és explicit 50 Hz sensor-debug módban is. A validált Nav2/local-costmap JSON gráf nem épül fel másodszor. |
| Python vision fogyasztó | `ProcessVisionPort`, demand-driven stream, kamera és detector külön ownerben | Idle és sikertelen demand conditionre, néma socket adatérkezésre vár. Demand-visszavonás és stop a meglévő socket shutdownnal ébreszt. Nincs periodikus üres polling. |
| Status, host, scheduler | Meglévő passzív status sidecar és host orchestration; az aktív affinity policy kikapcsolt | Nem változott CPU-affinity vagy scheduler policy. A CPU-, wall-, GIL-, fizikai I/O- és scheduler-várakozás nem lett összemosva. |

A telepített `rclpy.spin_once(node)` forrása minden hívásban hozzáadja, majd
eltávolítja a node-ot; mindkét művelet executor guardot triggerel. A közvetlen
executor-hívás ezeket az ismételt műveleteket szünteti meg. Nem állítjuk, hogy
minden guard-trigger kernel context switch-et okozott.

A stock MPPI collision algoritmusa megmaradt. A telepített header és a
[Nav2 1.3.13 CostCritic source](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/critics/cost_critic.cpp)
alapján a footprint-függő küszöbnek már van cache-e; nem került mellé második
approximate rollout/collision modell. A módosítás a saját végső command proofban
azonos geometriát és costmap-információt használ újra.

## Bizonyított szemantika és határok

A collision cache minden új commandra újraépül, és ugyanazon costmap mutex
alatt él, mint a teljes eredeti ellenőrzés és az alternatív sebességek.
Footprint-, pose-, costmap- vagy időbeli állapot nem cache-elődik tickek között.
A sorösszegzés lethal és unknown cellát egyaránt hard cellának tekint; csak
olyan teljes cellasort hagy ki a bboxon belül, amelyben az eredeti loop sem
végezne SAT-ot. A t=0 ellenőrzés, első blokkoló cella sorrendje, ív, horizont,
sampling guard, epszilonok, lépésszám és minden collision evidence-mező azonos.
A rövid söprések közvetlenül a régi cellaciklust használják.

A capture alapértelmezett encoder API-ja változatlan expanded formát ad.
Az összevont út opt-in, friss objektumgráfot épít, nincs identity memo vagy
recordok közötti mutable cache. A meglévő MCAP reader ugyanazt a kanonikus
alakot állítja vissza. A checkpoint, raw-scan referencia, hiba/trigger,
finite validation és integrity/accounting szemantikája megmarad.
Producer-oldali queue, backlog, capture-ütemezés és raw producer → sidecar út
nem változott.

## CPU-mérések

Motor-output nélküli, azonos gépen végzett izolált A/B mérések. A táblázat
az adott művelet CPU-idejének csökkenését mutatja, nem a teljes robotét.
Az előtte/utána sorrend váltakozott; a capture mérések mediánok.

| Mért munka | Előtte CPU | Utána CPU | Csökkenés |
| --- | ---: | ---: | ---: |
| 1600 behavior observation teljes encode → compact → JSON útja | 149.61 ms | 117.83 ms | 21.2% |
| 160 valódi Nav2 JSON observation teljes MCAP payload-kódolása | 652.87 ms | 243.14 ms | 62.8% |
| 200 sensor-debug tick, natív szimulációs MCAP-ból typed értékké visszafejtve | 551.67 ms | 440.44 ms | 20.2% |
| Collision, üres 60×60 map, 7533 parancs | 309.27 ms | 185.53 ms | 40.0% |
| Collision, ritka lethal cellák | 203.36 ms | 183.45 ms | 9.8% |
| Collision, sűrű lethal cellák | 184.40 ms | 173.99 ms | 5.6% |
| Collision, unknown fal | 278.95 ms | 189.05 ms | 32.2% |
| Collision, két fal | 285.22 ms | 186.81 ms | 34.5% |
| ROS executor, üres 3 s-os futások átlaga | 141.80 ms | 102.31 ms | 27.8% |
| ROS executor, 20 Hz pub/sub 3 s-os futások átlaga | 149.02 ms | 122.65 ms | 17.7% |

A capture kimenete minden összehasonlított frame-re byte-azonos.
A C++ összehasonlítás 12 559 random, cellahatár-, álló/forgó-, map-exit-,
unknown-, invalid- és footprint-változási esetben ellenőrizte a teljes
visszaadott parancsot és az összes collision evidence-mezőt.
A meglévő natív integráció 100×100-as mapot és a Python geometriai oracle-t is
használja, beleértve a maszk 64 bites szóhatárán átmenő tartományokat.

A vision collector 1.2 s-os üres/sikertelen demand mérésében 11→0,
néma streamnél 23→0 voluntary context switch történt. Explicit demand-váltás,
failure és stop teszteli az ébresztést; a csend nem újítja meg a mérési időt.
A ROS pub/sub mérésben a fogadott sequence-ek folytonosak; a mérési ablak
végén legfeljebb egy utolsó elküldött üzenet callbackje maradt az ablakon kívül.
A mért callback-késleltetések 0.49 ms alatt voltak. Kernel context switch
csökkenést a ROS-részre ez a kis mérés nem igazolt.

Nyers mérések és reprodukciós programok:
`tmp/cpu_20261115/{capture_bench.py,capture_results.json,collision.cpp,collision_main.cpp,collision_results.txt,ros_spin_bench.py,ros_spin_results.json,vision_wait_bench.py,vision_wait_results.json}`.
A C++ benchmark az eltett eredeti és az új függvényeket ugyanabban a binárisban
futtatja; build környezete a production `_navigation_environment`.

## Validáció és megmaradt bizonytalanság

- `./r test release`: **24 passed**, a kis robot gate-et is beleértve.
- Célzott encoder/local-costmap/SLAM/planner tesztek: **76 passed** az első körben.
- Végleges natív adapter és új capture/vision tesztek: **11 passed**; a későbbi
  full encoder-kiegészítés után az új capture fájl **4 passed**.
- Full raw 50 Hz persisted capture/replay és wheel-feedback fault/EVI evidence:
  **7 passed**. A tömörített matcher-reference külön tesztben is megőrzi a
  fizikai raw scan integritáshoz szükséges revisionöket.
- Vision demand/identity/camera observation: **9 passed**.
- A reprodukálható `deploy/build_nav2_wheel_adapter.sh` build és install sikeres;
  a telepített overlay is az új collision kódot tartalmazza.
- `./r sim simulate --scenario room --duration-s 12 --ros-domain-id 179
  --output tmp/cpu_20261115/sim_room_final`: **PASS**, `physical_output=false`,
  `GOAL_COMPLETE`, 0 ütközés, final STOP, 2.47 cm célhiba, natív replay **MATCH**.
  Ez valódi ROS SLAM/Nav2 és V3, szintetikus szenzorokkal és virtuális motor sinkkel.
- Explicit `r evi compile` a végleges szimuláció MCAP-jára: **COMPLETE**;
  `r evi verify --source`: **PASS**. Az export
  `tmp/cpu_20261115/sim_evidence_final`, a verification
  `tmp/cpu_20261115/sim_evidence_final_verify.json` alatt található. A compiler
  és a verification külön evidence, nem robotikai verdict.

A `test_navigation_map_capture.py` négy opcionális pack-esete a módosítás előtti,
eltett eredeti source-szal is ugyanúgy hibázik. Kettő a régi producer fixture
3 map üzenetét várja, de 0 keletkezik; kettő a retime fixture-ben nem igazítja
a navigation input contextjét az új tickhez. Ezeket a meglévő fixture-hibákat
nem alakítottuk át production javításnak; a release gate zöld.

A végleges szimulációban a controller MPPI-fázisa átlagosan **1.106 ms thread
CPU/parancs**, a végső collision proof **28.5 µs**, a controller összesen
**1.211 ms** volt 88 értékelésből. A teljes szimulátor loop 12 alkalommal
lépte túl a 20 ms-ot, a maximum 48.65 ms volt. Ez a mérés a szintetikus szenzorok
előállítását és az evidence-publicationt is tartalmazza; nem izolált L0–L12
vagy GIL-várakozás. Ebből nem következik, hogy az MPPI a teljes production
CPU-terhelés domináns oka, vagy hogy a teljes loop jittere javult.

A korábbi `cpu2_20261115_211826_summary.json` 15 s-os futásában a control
processz 48.4% egy-mag CPU-t, a main thread 19.0%-ot használt; egy másik thread
326 259 voluntary váltást mutatott. A mérés nem rendeli ezt a threadet source
ownerhez, és a külön ROS supervisorok sem mind a control leszármazottai.
Ezért az adat nem teljes production-runtime CPU baseline, és nem alkalmas
egy GIL/IPC/device-I/O ok megnevezésére.

Fizikai robotmozgás és új élő hardveres futás nem történt. A teljes hardveres
runtime összesített CPU-, hőmérséklet- és jitter-nemregressziója, valamint
hosszú, változatos navigációs helyzetek minősége nincs új live A/B evidence-szel
bizonyítva. A frekvenciák és döntési/safety logika megőrzését source, byte-egyezés,
teljes collision-egyezés, célzott tesztek és a fenti replay támasztja alá.
