# SLAM: hiányzó LiDAR-visszaverődések hibás illesztése — 2026-11-08

Az új A/B felvételekben a jelentős mozgást a natív LiDAR és a helyi odometria egyaránt kimutatja, de a SLAM már a saját gráfjában erősen összenyomja az elmozdulást. A konkrét hibás határ a hiányzó visszaverődések `0.0` értéke a ROS `LaserScan` bemenetén: az installált Karto matcher ezeket érvényes illesztési pontként kezeli. Az eredeti scanek natív offline újrafeldolgozása az egyetlen `0 → NaN` változtatással jelentős javulást mutatott. Ez még nem élő elfogadási eredmény.

## Felvételek és térképek

Kizárólag az alábbi új pár tartozik ehhez az elemzéshez:

- A: [v3_20261108_173711_3055_capture.mcap](../../runtime/captures/v3_20261108_173711_3055_capture.mcap), [AB_A_STATIONARY.yaml](../../runtime/slam_toolbox/maps/AB_A_STATIONARY.yaml), azonos nevű `.pgm`, `.json`, `.data`, `.posegraph`.
- B: [v3_20261108_173901_3055_capture.mcap](../../runtime/captures/v3_20261108_173901_3055_capture.mcap), [AB_B_ROOMCRUISE.yaml](../../runtime/slam_toolbox/maps/AB_B_ROOMCRUISE.yaml), azonos nevű kísérőfájlok.

A PGM-ek veszteségmentes PNG-másolatai és az összehasonlító ábra ténylegesen meg lettek nézve. A térképek felbontása 0,05 m, origójuk eltér. Az origók alkalmazása után egy robusztus SE(2)-illesztés csak −1,06 cm / +2,00 cm / +0,024° maradékeltolást talált. Ez érzékenységi ellenőrzés, nem külső geometriai ground truth.

| Térképi mérőszám | A, álló | B, Room Cruise |
| --- | ---: | ---: |
| Méret, pixel | 151 × 102 | 183 × 110 |
| Foglalt cellák | 398 | 1981 |
| Ismert cellák | 5933 | 9405 |
| Foglalt / ismert | 6,71% | 21,06% |
| Foglalt belső cellák, az első szomszédsági rétegen túl | 0 | 529 |

B jobb felső szobakontúrján többszörös, eltolt falvonalak látszanak. Az illesztett összehasonlításban B 800 foglalt cellája A ismert területére esik; közülük 427 több mint 10 cm-re van A legközelebbi falcellájától. Az újonnan megfigyelt geometria és a tényleges térképhiba pusztán ebből nem választható teljesen külön.

Evidence: [ábra](../../tmp/amr_review_20261108_jYfj9K/ab_map_comparison.png), [pixelmérések és illesztés](../../tmp/amr_review_20261108_jYfj9K/ab_map_quantitative.json).

## A mozgási és EKF-határ ellenőrzése

B explicit Evidence Compiler eredménye COMPLETE; source verification **PASS**, 1195 üzenet, 0 karantén. A natív replay mind a **482 tickre MATCH**, ismételt futásban is egyező trace-szel. A replay a rögzített döntési bemenetek determinisztikus kezelését igazolja; önmagában nem igazolja a SLAM térképi pontosságát.

A B-felvétel 117 eredeti raw scanjéből 115 mérésidejű helyi pózhoz illeszthető. A 114 egymást követő scanpár mind elfogadható eredményt adott a production relatív matcherrel; a vizsgált időablak 11,132 s. A deskew az eredeti pontonkénti host-receipt időket és a mérésidejű helyi headinget használta.

| Azonos mérésidejű ablak | Úthossz | Nettó elmozdulás |
| --- | ---: | ---: |
| L3 helyi odometria | 2,327 m | 1,919 m |
| Független raw LiDAR-scanlánc | 2,111 m | 1,693 m |
| Deskew-olt LiDAR-scanlánc | 2,126 m | 1,713 m |
| L3 SLAM-mal korrigált globális póz | korrekciók miatt nem összevethető fizikai úthossz | 0,397 m |

Az előremeneti scanpárok medián iránykoszinusza 0,99976, LiDAR/L3 elmozdulásaránya 0,876. A különbség nem indokol automatikus kerékkalibrációt; a ~80%-os globális elmozduláshiányt viszont nem magyarázza. Nincs 180°-os tengelyfordulásra utaló eredmény. Az L3-ban 33/33 SLAM-korrekció elfogadott, NIS-elutasítás nincs; a map→odom eltolásváltozás 1,517 m.

Evidence: [source verify](../../tmp/amr_review_20261108_jYfj9K/B_verify.json), [replay](../../tmp/amr_review_20261108_jYfj9K/B_replay.json), [scanpárok összegzése](../../tmp/amr_review_20261108_jYfj9K/B_motion_pair_summary.json), [páronkénti eredmények](../../tmp/amr_review_20261108_jYfj9K/B_motion_pair_results.json), [EKF-korrekciós határ](../../tmp/amr_review_20261108_jYfj9K/B_global_correction_review.json).

## Bizonyított SLAM-bemeneti hiba

Az installált `libkartoSlamToolbox.so` saját dataset/gráf betöltőjével vizsgált B-gráf 43 scanjának utolsó odometriapóza `(1,865973; 0,447275; 1,860817)`; ez egyezik a helyi L3-pózzal. A nagy elmozdulás tehát helyesen bejutott a SLAM-ba. A mentett korrigált póz ezzel szemben `(0,390966; 0,042170; 1,592087)`, nettó elmozdulása 0,393234 m.

Mindkét datasetben 720 bin/scan, 0,05 m minimumtávolság és −0,005 m szenzoreltolás szerepel. A hiányzó bin értéke `0.0`, nem mért akadálypont:

- A: 48 scan, **12 980** nulla bin.
- B: 43 scan, **11 307** nulla bin, scanenként 240–423.

Az installált Karto `GridIndexLookup` ellenőrzése szerint **minden ilyen nulla érvényes, azaz nem `INVALID_SCAN` lookupot kap**. A szűrt pontlistában ugyanakkor nincsenek szenzoreredeti pontok: a filtered/unfiltered pontút eltérése miatt a hibát nem zárja ki a szűrt geometria helyesnek látszó tartalma. A ROS-határon a hiányzó visszaverődést ezért nem szabad véges, nulla távolságú mérésként továbbadni.

Evidence: [gráf/dataset összegzés és bemeneti SHA256-ok](../../tmp/amr_review_20261108_jYfj9K/posegraph/summary.json), [B bin/lookup mérések](../../tmp/amr_review_20261108_jYfj9K/posegraph/B_range_stats.csv), [A bin/lookup mérések](../../tmp/amr_review_20261108_jYfj9K/posegraph/A_range_stats.csv), [natív betöltő/mérő source](../../tmp/amr_review_20261108_jYfj9K/posegraph/dump_posegraph.cpp).

## Offline ellenpróba és nyitott élő elfogadás

Azonos 43 B-scan és a productionból szerializált paraméterek két natív újrafeldolgozási ága futott. Mindkét ágban loop closing kikapcsolva, solver nélkül. A változatlan nullás baseline mind a 43 mentett korrigált pózt reprodukálta: maximális XY-eltérés 2,24×10⁻¹⁶ m, yaw-eltérés 0.

Kizárólag a 11 307 hiányzó bin `0 → NaN` átírásával a korrigált nettó elmozdulás **0,393234 → 1,729147 m**, az úthossz **0,551296 → 2,109344 m** lett. A végső korrigált–odom XY-eltérés **1,529625 → 0,197111 m**. Ez a konkrét offline illesztési hiba és az adapterhelyi javítás hatásának bizonyítéka; nem a robot fizikai ground truth-ja vagy a későbbi élő futás eredménye.

Evidence: [natív ellenpróba összegzése](../../tmp/amr_review_20261108_jYfj9K/posegraph/B_counterfactual_summary.json), [változatlan baseline](../../tmp/amr_review_20261108_jYfj9K/posegraph/B_zero_reprocess.csv), [NaN-ág](../../tmp/amr_review_20261108_jYfj9K/posegraph/B_nan_reprocess.csv).

A változatlan offline baseline a mentett B-térkép mind a 20 130 pixelét is
pontosan reprodukálta, azonos origóval és 183 × 110 mérettel. A NaN-ágban
helyreálltak a vékony falkontúrok; a foglalt cellák száma 1981-ről 638-ra
változott. A cellaszám önmagában nem minőségmetrika.
[Azonos bemeneti scanek térképei](../../tmp/amr_review_20261108_jYfj9K/posegraph/B_zero_vs_nan_maps.png).

## Javítás és validáció

A `v3/slam_toolbox_bridge.py` kizárólag a ROS `LaserScan.ranges` felépítésekor
alakítja a transport hiányzó `0.0` binjeit `NaN`-ná. A pozitív távolságok,
szögindexek, scan-azonosság, mérési idő, odometria és a producer/wire evidence
változatlan. A munka a külön ROS interpreterben marad; nincs új control-IPC
vagy motor-authority.

- Bridge + transport célzott tesztek: **50 passed / 0,57 s**.
- `./r test` kötelező robot-contract gate: **14 passed / 0,71 s**.
- Érintett source és teszt whitespace-check: PASS.

## Új élő próba és konkrét akadály

Forrás: [v3_20261108_211302_11444_capture.mcap](../../runtime/captures/v3_20261108_211302_11444_capture.mcap),
FULL 50 Hz. A motormentes readiness 40 mintája PASS: lokális translation,
heading és globális pozíció végig GOOD, motor-output tiltott; a bemelegedés
utáni legnagyobb L4 méréskor 173,394 ms, a scan sequence 59-et haladt.

A kért 120 s Room Cruise 0,30 m/s / 1,40 rad/s korláttal elindult, de az
első nem nulla motorparancstól **0,540823 s** után L11-fault következett
(tick1646 → tick1672). A felhasználó megerősítette, hogy a robot megmozdult,
majd leállt. A friss encoder-acquisition mellett bal oldalon nulla, jobb
oldalon egy elfogadott él szerepel. Az egyetlen jobb él 512,793 ms-os
koránál `MISSING_EDGE:left;MISSING_EDGE:right` és canonical L12 FAULT/zero
következett. Az aktív policy 500 ms-os élbizonytalansági és külön 250 ms-os
acquisition-age korlátot tartalmazott. A lokális és globális minőség a
fault tickben még GOOD. Ez nem a SLAM elvesztése miatti leállás.

A `raw_*_pulse_count` itt már dekódolt, előjeles számláló; önmagában nem
független fizikai GPIO-event bizonyíték. A tick-skalárokban bal callback
nincs, jobb oldalon egy A-callback van, későbbi A/B callback vagy
dekóderelutasítás nem látható. A fizikai jelhiány és a GPIO notification /
callback elakadása ebből még nem választható szét; encoder-hardverhibát
nem állapítunk meg, a védelmet nem lazítjuk tovább.

Az explicit compiler COMPLETE, 2852 üzenettel; az eredeti MCAP-ra végzett
source verification PASS. A fault körüli tick1570–1672
natív replayje **MATCH**: 103 kiválasztott, 148 végrehajtott tick, kétszer
azonos trace. A friss mentett SLAM-gráf 115 scanjában **32 952 NaN**, nulla
véges nulla és nulla Inf szerepel; minden NaN natív matcher-lookupja
`INVALID_SCAN`. Ez bizonyítja a javítás tényleges élő ROS-aktiválódását.

A külön Nav2 előindítás map/TF hiányában időtúllépéssel leállt, mert az
IDLE runtime előtt történt; nem ismételtük meg automatikusan. A helyes
indítási sorrendet a `docs/NAV2_PLANNER.md` már rögzíti. Ez a Room Cruise
EXPLORE próba a helyi V3 tervezőt használta, nem Nav2 NAVIGATE missiont.

Minden runtime és ROS sidecar leállt. Automatikus új mozgáspróba nem történt.
**A 120 s elfogadás, a mozgás közbeni új élő térképminőség és a production
helyi akadálytérkép tartós stabilitása továbbra sem bizonyított.** Először
az encoder callback-bizonyíték hiányát kell tisztázni; további EKF/NIS vagy
kerékskála-hangolást a mostani adatok nem indokolnak.

Evidence: [readiness](../../tmp/amr_review_20261108_jYfj9K/live/readiness.json),
[első hibás motor/encoder határ](../../tmp/amr_review_20261108_jYfj9K/live/terminal_encoder_boundary.json),
[incident replay](../../tmp/amr_review_20261108_jYfj9K/live/incident_replay.json),
[source verify](../../tmp/amr_review_20261108_jYfj9K/live/verify.json),
[élő sentinel-bizonyíték](../../tmp/amr_review_20261108_jYfj9K/live/posegraph/sentinel_summary.json),
[mentett térkép](../../runtime/slam_toolbox/maps/SLAM_NAN_ABORT_20261108.yaml).
