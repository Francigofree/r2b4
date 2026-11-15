# Lokális navigációs szimulátor

Belépés: `./r sim`. Headless, a Pi-n telepített ROS-környezetekkel működik;
Gazebo és külső számítógép nélkül. Nem indít fizikai robot-runtime-ot és nem
komponál fizikai motor-, GPIO- vagy szenzoradaptert. Az L12 kimenetének sinkje
egyetlen, memóriában tárolt virtuális parancs. A production konfigurációt olvassa;
a futásbeli módosítások kizárólag az eredménykönyvtárba kerülnek.

## Végrehajtás és tervezési határ

```text
virtuális PWM/kerék/fizika → szintetikus encoder, IMU és raycast LiDAR
                         ↓
canonical CommandGateway → lezárt TickInputs → production V3 L0–L12
                                                    ↓
                                    virtuális motor-sink → fizika

completed L3 odometria + scan → production SLAM transport
                                     ↓
                     SLAM Toolbox + production SLAM bridge
                         ↓                        ↓
                 scan-bound lidar_pose      map, scan, TF, odom
                         ↓                        ↓
                  következő TickInputs      production Nav2/Smac/MPPI
                                                  ↓
                              production Nav2NavigationClient → TickInputs
```

A réteg- és async contract változatlan. A teszteszköz az input source és output
sink körüli offline adapter; nincs új planner, mission-state vagy safety-authority.
Az EXPLORE parancs ugyanúgy a production frontier/coverage és recovery kódhoz jut.
Csak a SLAM Toolbox visszatérő mérési póza korrigálja a globális lokalizációt:
a régi plant truth-alapú `lidar_relative_motion` mintája ezen az úton kimarad.

A ROS domain alapértéke **75**, a live **74** tiltott. A scan-, odometria-,
Nav2-socketek és a gyermeklogok futásonként külön könyvtárban vannak. Ugyanazon
szimulátordomain egyidejű használatát helyi lock zárja ki. A ROS-domain más,
független eszközzel való foglalását az operátornak kerülni kell. A tesztek sorosan
futnak, az OMP/BLAS szálkorlátokat a production környezetaktiválás adja. A saját
gyermekek RSS-limitje 1536 MiB, nice értéke 10; váratlan process exit, memória-
vagy pacing-határ túllépés lezárja a futást. Nincs automatikus hibáspróba-ismétlés.

## Használat

```bash
./r sim doctor
./r sim simulate --scenario room
./r sim simulate --scenario corridor
./r sim simulate --scenario explore
./r sim simulate --scenario docs/examples/local_navigation_fault.json
./r sim fault --scenario docs/examples/local_navigation_fault.json

./r sim analyze capture.mcap
./r sim analyze capture.mcap --summary-only
./r sim replay capture.mcap
./r sim scenario capture.mcap

./r sim autotune --variants 2 --budget-s 600
./r sim autotune --search v3 --variants 2 --suite sajat_suite.json
./r sim regression --baseline regi/run/result.json --scenario room
./r sim watch --baseline regi/run/result.json --max-runs 20
./r sim stress --scenario explore --repeat 10
```

Az `--output` mindig új könyvtár. Alapérték: egyedi `tmp/nav_sim_*` a repóban.
Hibás eredménynél a CLI exit code 2; PASS/MATCH/COMPLETE esetén 0.
`--no-replay` csak a `simulate` gyors fejlesztői futásához használható.
Az autotune és a regressziós összevetés replay MATCH nélkül nem fogad el javulást.

Beépített világok: `room`, `corridor`, `dead_end`, `moving_obstacle`,
`lidar_dropout`, `explore`. Saját JSON szcenárióval 0.1–30 s-os véges próbák
adhatók meg. A `stress` ezeket teljes indítás/leállítással ismétli, így a
lifecycle is terhelődik; ez nem egy megszakítás nélküli többórás mapping session.

## Világ és érzékelők

A `R2B4_LOCAL_NAVIGATION_SCENARIO_V1` az existing `motion_scenario` mezőit
használja: `kind: differential_drive`, `environment_id`, `seed`, `duration_s`,
`goal`, `room`, `obstacles`, `plant`, célpozíció/yaw toleranciák. Kiegészítések:

| Mező | Tartalom |
|---|---|
| `behavior` | `navigate` vagy `roomcruise` |
| `walls` | Legfeljebb 64 `[xmin,ymin,xmax,ymax]` téglalap, méterben |
| `moving_obstacles` | Legfeljebb 8 kör: `x_m,y_m,radius_m,vx_mps,vy_mps` |
| `faults` | Legfeljebb 32 időzített hiba |
| `expect` | `goal_reached`, `minimum_path_m`, `maximum_stationary_s` |
| `provenance` | MCAP-eredet és rekonstrukciós feltételezések |

A robot tényleges tengelytávja, sebesség/PWM görbéi, kerékholtsávja és a
production gyorsuláskorlátozás aktív konfigurációból származik. Az ütközésvizsgálat
a tengelyhez képest eltolt, elforgatott téglalap alakú testet használja; a scan
kezdőpontja az aktív LiDAR extrinsic. A környezet geometriai koordinátáinak
origója a robot induló hajtástengelye. A LiDAR 360 rayt ad 10 Hz-en, a virtuális
encoder/IMU és a V3 50 Hz-en fut. A fizikában elsőrendű kerékválasz, kétoldali
gain, traction és seedelt encoder-zaj állítható. Ezek **modellfeltevések**,
nem mért fizikai kalibráció. Ütközéskor a próba lezárul; kontaktmechanikai,
súrlódási vagy talajdeformációs modell nincs. Beszorulás traction=0-val is
modellezhető; recovery sikerét a virtuális test tényleges haladása mutatja.

## Hibabefecskendezés és lifecycle

Minden faultnak `kind`, `start_s`, `duration_s` mezője van, az aktív szakaszhoz
viszonyítva. A késés/kiesés nem írja át az eredeti measurement időt vagy sequence-et.

| Kind | Hatás |
|---|---|
| `lidar_drop` | Új LiDAR-minták/scanexport kiesnek; a korábbi minta elöregszik |
| `lidar_delay` | Scan és a V3 LiDAR-minták 0.02–2 s késése, `delay_s` mezővel |
| `imu_drop`, `encoder_drop` | A megnevezett szintetikus szenzorminták hiányoznak |
| `command_loss` | A canonical command heartbeat megáll, TTL lejár |
| `nav2_loss` | A Nav2 kliens kimenő és bejövő forgalma kiesik |
| `tf_drop` | Completed L3 odometria exportja kimarad, így odom/TF is elöregszik |
| `process_pause` | Saját gyermek teljes felfüggesztése, majd folytatása |
| `process_exit` | Saját gyermek célzott leállítása, automatikus restart nélkül |
| `cpu_load` | Külön véges CPU-terhelő folyamat |

Process fault esetén `target`: `slam`, `slam_bridge`, `bridge`,
`controller_server`, `planner_server`, `behavior_server`, `bt_navigator` vagy
`lifecycle_manager`. A `process_pause` a teljes folyamatot állítja meg; nem
azonos egyetlen ROS callback célzott blokkolásával. `tf_drop` az odometriát is
érinti, ezért nem tisztán TF-specifikus kísérlet.

Minden próba a valódi lifecycle managerrel indul; az indítás IDLE V3 mellett
scaneket és odometriát szolgáltat. Shutdown előtt canonical STOP és a plant
lecsengésének ellenőrzése fut, majd csak a saját process groupok szűnnek meg.
A faulttal kiváltott production FAULT is FAIL eredmény: az első hibapont és a
nulla végső kimenet külön látható, a faultot a futtató nem minősíti át sikernek.

## MCAP, elemzés és szcenáriógenerálás

Az `analyze` egyetlen MCAP-ból, előfeltételként meglévő EVI/DIAG nélkül fut:
natív deep integrity → streamelt eseményidővonal → explicit Evidence Compiler
(1 worker) → meglévő navigation, navigation_root_cause és motion_performance DIAG.
A `--summary-only` csak a natív idővonalat és a mintavett mozgásmérést készíti.
Az elemzés PASS a feldolgozás eredménye; nem állít sikeres robotfeladatot.

A `replay` közvetlenül a canonical Replayer. Hiányos 5 Hz-es capture nem válik
teljes 50 Hz-es replay-bemenetté: BLOCKED marad. A szimulátor saját capture-je
teljes 50 Hz-es inputot és raw LiDAR-t ment; replaykor ROS nem indul újra, a
rögzített typed completionökkel a V3 determinisztikusságát ellenőrzi.

A `scenario` saját szimulációs MCAP esetén a pontos mentett világot veszi át.
Valódi capture-ből egészséges `v3.navigation_map` megfigyeléshez és legfeljebb
250 ms-mal korábbi globális pózhoz kötött első lokális endpoint-felhőt használ.
0.2 m-es foglalt cellákra közelít, legfeljebb 64 cellával. A külső szobahatár és
a nem megfigyelt tér feltételezés; a provenance ezt és a csonkolást rögzíti.
Ez **közelítő teszteset**, nem az eredeti incidens bizonyított reprodukciója.
Hiányzó map/póz evidence esetén explicit SCENARIO_BLOCKED; nincs kitalált szenzor.
A meglévő 5 Hz-es passive map/decision evidence elegendő lehet ehhez; az MCAP
production scope-ját ez a fejlesztés nem bővíti és nem terheli további raw adattal.

## Hangolás és regresszió

Autotune alapból valódi MPPI critic-súlyokat változtat. A `--search v3` a meglévő
`r2b4_navigation_autotuner.generate_overrides` keresőjét használja. Minden próba
ugyanazt a világot, seedet, időlépést, parancsot és faultsorrendet kapja; a
train/holdout környezet geometriája eltér. Egy saját suite formátuma:
`schema: R2B4_LOCAL_NAVIGATION_SUITE_V1`, `train` és `holdout` szcenáriótömbök.

Saját overlay: `schema: R2B4_LOCAL_NAVIGATION_OVERRIDE_V1`, opcionális
`vezerles`, `nav2`, `slam` objektum. A JSON vezerles overlayt a production
ConfigResolver validálja. ROS-ban meglévő numerikus algoritmusparaméterek
hangolhatók; topic, plugin, frame, footprint, freshness és frekvencia nem.
Az `--overlays` ilyen dokumentumok tömbje. Az aktív `conf/` fájlok nem íródnak.
Érvénytelen interfészt vagy paramétert a futtató elutasít.

A biztonsági evidence, végső STOP és replay MATCH megelőzi a pontozást. A baseline
lehet biztonságosan befejezett, de viselkedésileg sikertelen próba; ajánlható
változat minden szcenárióban csak PASS lehet. A rangsor
célhibát, futásidőt/utat, clearance-t, megállást és lokalizációs hibát hasonlít
össze; RC-ben a független truth coverage számít, a céltalan körözés nem javítja
a pontszámot. A FAIL→PASS javítás külön állítás a közös PASS-próbák numerikus
javulásától. A train alapján rögzített egyetlen győztes egyszer fut holdouton;
sikertelen holdout után nincs másik változat kiválasztása ugyanazon evidence-ből.
A legjobb megfelelő változat `best_override.json`; automatikus éles
alkalmazás nincs. A kézi review és független ismétlés továbbra szükséges.

Minden eredmény a jelenlegi production import-closure és releváns konfiguráció
tartalmi hashét, Git HEAD-et és dirty állapotot tartalmazza. Futás közbeni
forrásváltozás FAIL. Regressziónál a régi `result.json` és az új futás ugyanazon
szcenárióját hasonlítja össze, toleranciákkal; hiányzó evidence INCONCLUSIVE.
Minden új `r sim` hívás az aktuális forrást használja. Az explicit `watch`
indításkor, majd a releváns forrás/config/bináris fájlok megváltozásakor friss
Python-folyamatban hívja a regressziós parancsot. Változatlan fájloknál csak a
stat adatokat figyeli, nem futtat újra tesztet. Az első FAIL-nél megáll;
megszakítása Ctrl-C. Automatikus kódmódosítást nem végez.

## Eredmények és bizonyítási korlát

`result.json`: PASS/FAIL, első boundary/tick, virtuális truth-metrikák, navigation
okok és döntési átmenetek, SLAM/Nav2 végállapot, memória/CPU, külön replay-verdict.
`timeline.ndjson`: tickenként truth, becsült póz, lokalizációs quality, cél,
navigation ok, L12 döntés és virtuális output. A teljes capture a pontosabb
rétegenkénti elemzés authorityja. Ismétlődő cél, hosszú állás vagy STOP oka itt és
a meglévő DIAG-ban vizsgálható; a kimenet nem állít automatikusan kauzális root cause-t.

Mért tick-wall idő tartalmazza a szintetikus szenzorgenerálást és az observation
publikálást. Nem izolált L0–L12 timing és nem GIL/IPC/scheduler-okbizonyítás.
Nav2/SLAM ütemezése nem determinisztikus; azonos seed nem jelent bitazonos ROS
futást. A natív replay csak azonos lezárt V3-inputokra ad MATCH-et. A felsorolt
CLI-módok létezése önmagában nem bizonyítja az összes recovery/lifecycle/
fault-kombinációt, többórás stabilitást vagy a fizikai robot modellpontosságát.
