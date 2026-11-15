# Szobacirkálás: sebesség és mozgásminőség — 2026-11-15

Az előző stabilitási javítás utáni emberi visszajelzés alapján készült folytatás.
A szimulátor a production V3, SLAM Toolbox és Nav2 utat futtatja izolált
ROS-domainen. A fizikai próbát a felhasználó ebben a feladatban engedélyezte.
Az architekturális és async contract változatlan.

## Bizonyított okok és javítások

### Elrejtett tervezői sebességplafon

Az aktív room-cruise mission és az L9 platformlimit már 0,45 m/s volt.
A `tools/nav2_planner.py::_write_geometry` viszont minden indításkor
`min(.30, platform_max_v)` értéket írt a native MPPI konfigurációjába.
A `122326_19333` élő capture pozitív mozgáskéréseinek mediánja 0,29856 m/s.

A generált plafon most közvetlenül a platformlimitből származik; az aktív
`conf/nav2_planner.yaml` `FollowPath.vx_max` értéke 0,30 → **0,45 m/s**.
A mission ennél kisebb saját limitet továbbra is érvényesít.

### Hiányzó megvalósítható kerékcsalád az MPPI átlagolása után

A native `projectMppi` csak a STOP-kerék ágakat ajánlotta fel. Ezek a nyers
parancs egyes kerekeinek nagyságát is felső határnak vették. Emiatt kizárta
azt a szabályos gördülő ívet, amely változatlan testsebesség és kisebb
szögsebesség mellett az alacsony belső kereket a fizikai minimumra emeli.

Konkrét élő evidence: sequence 7, `10026237628658 ns`,
nyers `(v, omega)=(0,1754808426; 0,4100486338)`, régi eredmény `(0; 0)`.
A canonical projekció `(0,1754808426; 0,1432715356)`; a belső kerék 0,15 m/s.
479 megőrzött eseményből 267 nullázás volt; **210 nullázásnál** létezik ilyen
nem nulla steady jelölt. Ez a kinematikai lehetőséget bizonyítja; az adott
210 új ív teljes ütközésmentességét nem.
[Offline bizonyíték](../../tmp/cruise_quality_projection/evidence.json).

A wrapper most a canonical projekciót is felajánlja a STOP-ágak mellett.
A kért testsebesség és szögsebesség nem erősíthető, a kerékcsalád és a
kalibrált minimum/maximum megmarad. A kiválasztott, megváltozott ív teljes
padded-footprint ütközésvizsgálata változatlanul kötelező. Native teszt
ellenőrzi azt is, amikor a nyers ív szabad, de a módosított ív akadályba fut.

### Projekciós kerekítési hiba az L9 strict contract előtt

A javítás előtti alap-szimuláció első hibás tickje **932, L9**.
A kért omega `0x1.a001200000000p-5`, a projekció után
`0x1.a001200000001p-5`: az eltérés `6,938893903907228e-18 rad/s`.
A `ConstrainedMotion` helyesen elutasította az erősítést.
[Hibás futás](../../tmp/cruise_quality_baseline/run/result.json).

A Python és C++ projekció a végső koordináta-visszaalakítás után is pontosan
a kért/platform testhatárokon belül marad. A downstream contract nem kapott
epsilon-kivételt. A célzott teszt a konkrét számokat, a szomszédos ULP-ket,
mind a négy előjelkombinációt és a tényleges L9 állapotát vizsgálja.

Csak e számtani javítás után az alapfutás PASS, 30 s, nulla ütközés,
1834 tick replay MATCH.
[Alapfutás](../../tmp/cruise_quality_precision_baseline/run/result.json).

### Független lineáris és szögsebesség-limit a ROS-edge-en

A native Nav2 SpeedLimit a lineáris aránnyal a szögsebességet is skálázza.
0,45-ös alapplafon és 0,25-ös NAVIGATE mission esetén ez 1,4 → 0,778 rad/s,
ami már a 0,843 rad/s fizikai helybenfordulási minimum alatt lenne.
A [rögzített 1.3.13 source](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/optimizer.cpp#L450)
megerősíti ezt a kapcsolást.

A bridge a már meglévő, goal-dispatch előtt ellenőrzött atomikus
paramétertranzakcióban külön adja át a mission/platform metszet szerinti
`vx_max`, `vx_min`, `wz_max` értékeket. A missionből képzett SpeedLimit
publikáció megszűnt. Nem került új IPC, scheduler vagy control-authority a
rendszerbe; azonos missionértékek nem indítanak ismételt optimizer-resetet.
A normál NAVIGATE 0,25/1,4 és a room-cruise 0,45/1,4 váltás célzottan tesztelt.

### SLAM: a státuszfájl ne blokkolhassa a pózcallbacket

Az első élő próba utolsó SLAM-diagnosztikájában a 278. scan póza
`12157303507455 ns` időpontban már a DDS-ben van, de a callback csak
`12161716097584 ns` időpontban fut le: **4,41259 s** várakozás.
A DDS transport 0,207 ms, a scan-publikálástól a pose küldéséig 31,10 ms.
A lejárt válasz helyesen elutasításra kerül. A 1491. tickben a globális fix
kora átlépi a 2 s-ot; az így létrejövő 156 `NAV2_LOCALIZATION_HOLD` tickben
a lokális transzláció és a heading GOOD, a globális pozíció DEGRADED.
[Első boundary](../../tmp/cruise_quality_post_live/first_localization_hold.json).

A bridge ugyanebben a ROS-loopban szinkron `write_text/chmod/replace`
státuszírást végzett. A korábbi műszeres régiók maximuma 70 ms alatti, de a
státusz-I/O nem volt mérve. **Az eredeti élő 4,41 s-os szünet fizikai I/O-,
CPU/GIL- vagy scheduler-kiváltója nincs bizonyítva.** A szinkron I/O és a
pózfogadás kölcsönös blokkolása forrásból és injekcióval bizonyítható hiba.

A SLAM bridge most a Nav2-nél már létező háttérírót használja: egy folyamatban
lévő írás és legfeljebb egy felülírható pending JSON; a measurement time,
source sequence és lineage nem frissül a writerben. A fájl joga továbbra is
0600. A snapshot kiírási ideje, hibája és superseded számlálója látható.
A close legfeljebb 1 s-ot vár; beragadt I/O mellett a terminal snapshot
kiírása nem garantálható, a shutdown timeout stderr-en megjelenik.

A célzott teszt blokkolt fájlírás mellett 100 további SLAM-snapshotot ad át
100 ms alatt, 99 superseded értékkel; az utolsó mérési idő és póz változatlan.
Ez nem ad új lokalizációs vagy motorjogosultságot.

## Szimulált összehasonlítás

Azonos nyílt szoba, seed 7, 30 s; első sor csak a kerekítési javítást
tartalmazza, a második már a sebességplafon és a hiányzó jelölt javítását is.

| Mérés | Alap | Javított adapter + plafon |
|---|---:|---:|
| Virtuális út | 3,158 m | 3,422 m |
| Meglátogatott 0,25 m-es truth-cellák | 16 | 18 |
| Elindulás utáni állás, kb. 5 Hz-en | 25,5% | 15,4% |
| Lineáris jerk RMS, kb. 5 Hz-en | 1,001 m/s³ | 1,688 m/s³ |
| Ütközés | 0 | 0 |
| Natív replay | 1834 MATCH | 1775 MATCH |

[Alap](../../tmp/cruise_quality_precision_baseline/run/result.json),
[javított](../../tmp/cruise_quality_candidate_open/run/result.json).
**A mozgás simaságának javulása ebből nem bizonyított:** a jerk nőtt.
A szimulátor nem ad truth-alapú relatív LiDAR-odometriát a production L3-nak;
annak DEGRADED állapotában a változatlan L9 limit kb. 0,2896 m/s.
Ezért a tartós 0,45 m/s elérését e futással nem lehet igazolni.
A virtuális jerk nem fizikai gyorsulásmérés, a ROS-ütemezés nem determinisztikus.

## Első engedélyezett élő próba

Parancs: `./r rc 30 c full c 50`.
Capture: `runtime/captures/v3_20261115_125852_25915_capture.mcap`.
Runtime PASS, integritás PASS, nyers LiDAR-sorozat teljes, 2150 tick natív
replay **MATCH**, canonical STOP és runtime shutdown megtörtént.
[Explicit Evidence Compiler és elemzés](../../tmp/cruise_quality_post_live/analysis.json),
[natív replay](../../tmp/cruise_quality_post_replay/replay.json).

A native adapter kumulatív diagnosztikája 78 projekciót, 7 nullázást és egy
collision-recheck elutasítást rögzít 86 eseményből. A korábbi 267/479 és az új
7/86 arány eltérő geometriájú és hosszúságú menetekből származik; a közvetlen
210-es offline ellenpélda adja az oksági kinematikai bizonyítékot.

A kb. 5 Hz-re ritkított L3-becslés szerint a pozitív sebesség p90 0,287 m/s,
az elindulás utáni állás 43,6%, a tolatási arány 12,8%, lineáris jerk RMS
2,848 m/s³. Ezek **nem igazolják a fizikai mozgásminőség javulását**.
Nem azonos kezdeti póz és útvonal, ezért a korábbi 126 s-os menettel nem
állítható fel tiszta sebesség- vagy jerk-rangsor.
Az összes aktív tickből mért maximumok: kérés **0,4462 m/s**, L9 által
engedett 0,3561 m/s, L3 becsült 0,4020 m/s. A tervező 0,30-as plafonjának
megszűnése élő inputon is igazolt; tartós fizikai 0,45 m/s ebből nem következik.
[Célzott számlálók](../../tmp/cruise_quality_post_live/targeted.json).

A 656. tickben már a Nav2 nyers javaslata tolatást kér, miközben a robot
headingje -2,824 rad, célja `(1,995; 2,375)`; az L9 nem hozta létre a
hátramenetet. Az aktív `PathAngleCritic.mode=1` nem preferálja a pálya felé
előre néző headinget. A mode=0 külön kísérlet, nem automatikus javítás.

A teljes 50 Hz-es raw capture mellett a producer control p99 13,8 ms,
maximum 24,3 ms; observer maximum 79,8 ms, periódusmaximum 98,5 ms.
Ezek külön régiók: nem bizonyítanak fizikai I/O-, GIL-, transport- vagy
scheduler-gyökérokot. A velocity TTL és szenzorfrissesség nem lazult.

## Elutasított előremeneti hangolás

Külön overlay: `PathAngleCritic.mode=1 → 0`; a reverse-limit és a critic-súlyok
változatlanok. A [native critic source](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/critics/path_angle_critic.cpp)
szerint ez előre néző headinget preferál a pálya irányába; nem fizikai tiltás.

Mögöttes, 1 m-es NAVIGATE célon mode=1: PASS, 7,34 s, 0,0233 m hiba,
767 tick MATCH; mode=0: **FAIL**, 25 s után 0,1682 m hiba, 1442 tick MATCH,
nulla ütközés. A mode=0 sok tolatást megszüntetett, de a feladatot nem teljesítette.
[Alap](../../tmp/cruise_quality_behind_mode1/run/result.json),
[kísérlet](../../tmp/cruise_quality_behind_mode0/run/result.json).

Bútorozott szobában mindkettő PASS: út 3,425 → 3,424 m, minimum clearance
0,3549 → 0,3562 m, nulla ütközés, 1734/1683 tick MATCH. A ritkított jerk RMS
1,425 → 1,211 m/s³, de az elindulás utáni állás 9,1% → 22,1% lett;
mode=0 mellett 23 ritkított lokalizációs HOLD van, ezért a simaságkülönbség
nem tiszta critic-hatás. **A mode=0 nem került az éles konfigurációba.**
[Alap](../../tmp/cruise_quality_furnished_mode1/run/result.json),
[kísérlet](../../tmp/cruise_quality_furnished_mode0/run/result.json).

## Éles változtatások határa

A 0,45 m/s tervezői plafon és a külön tengelyenkénti mission-paraméterezés
a következő runtime-indításkor érvényes. A fizikai kerékminimum 0,15 m/s,
maximum 0,582 m/s, L9 gyorsuláskorlátok, footprint/padding, L12 safety,
200 ms velocity TTL és 250 ms szenzorfrissesség változatlanok.
Az újratervezés/recovery nem kapott végtelen retryt vagy STOP-simítást.

## Nyitott bizonyítás

A tartós fizikai 0,45 m/s, a természetesebb/simább szobacirkálás, a hosszú
üzem és minden szűk geometriájú manőver sikere még nem bizonyított.
A rejectált hangolási kísérletek és további validáció eredményeit alább kell
az elfogadott éles változtatásoktól elkülönítve értékelni.
