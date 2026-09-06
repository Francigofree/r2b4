# R2B4 Replayer V2.1 és V3

## Native V3 teljes tick replay

A `v3.replay` a `R2B4_V3_NATIVE_FLOOR_TICK_CAPTURE_V1` fájlokat offline,
közvetlen typed érték-összehasonlítással játssza vissza. A capture L1 értéke a
production `RawDeviceBatch -> AcquisitionFrame` tiszta másolata, ezért az input
veszteségmentesen visszaállítható. A bounded floor profil per-tick STOP/TELEOP
requestje a capture-kori gateway-contract és az L5 érték alapján áll vissza.
Hardver-, szenzor-, óra-, GPIO- vagy motor-I/O nincs; az L12 writer memóriabeli.

A replay kétszer, friss composition példányokkal futtatja a production L1-L12
láncot, majd minden tick minden rétegét és a final actuationt összeveti a
capture-rel. Az eredmény emellett rögzíti az L10 setpointot, a production L11
speed-map feed-forward/P/I/integrátor értékeit, a ramp-, saturation- és output
limiteket, valamint a bal/jobb mért sebességet és bias-t.

A szerkezetileg teljes, terminális `PASS`, `FAIL` és `FAULT` capture egyaránt
replayelhető. A capture futási eredménye (`execution.capture_status`) külön
authority a replay `MATCH/MISMATCH` eredményétől: egy biztonságosan leállított
`FAIL` futás helyes determinisztikus reprodukciója `MATCH`. `ACTIVE`, `INVALID`
vagy más nem terminális/érvénytelen capture továbbra sem replayelhető. Nulla
L12 `ALLOW` tick esetén is készül eredmény, de a wheel-control klasszifikáció
explicit `0_NO_L12_ALLOW_CONTROL_WINDOW` lesz.

```bash
python3 -m v3.replay inspect /path/to/v3_floor_ticks.json
python3 -m v3.replay replay /path/to/v3_floor_ticks.json \
  --physics-config conf/fizika.json \
  --speed-map-config conf/speed_map.json \
  --capture-source-manifest /path/to/capture_workspace/base_manifest.json \
  --output logs/session_<run-id>/v3_replay_result.json
python3 -m v3.replay verify-result \
  logs/session_<run-id>/v3_replay_result.json
```

`MATCH` csak akkor készül, ha nincs első divergencia, a két replay trace azonos,
és minden tickhez pontosan egy offline L12 write tartozik. A capture-time source
manifest opcionális: ha elérhető, a report külön jelzi a releváns production
forrás és aktív konfiguráció hash-egyezését.

A V3 LiDAR L1 snapshot minden elérhető matcher-revíziónál passzívan megőrzi a
matcher/source-scan identitást, readiness/timeout/degeneráció állapotot és a
legfontosabb minőségi mérőszámokat. Ezek a mezők replay-evidence-ek, nem új
control- vagy safety-authority-k.

## Legacy Replayer V2.1

A V2.1 az új, lezárt fizikai réteghatárokat replayeli:

- `L8_MOTION_CONTROLLER`: physical command/envelope/capabilities -> wheel setpoint;
- `L9_MOTION_EXECUTOR`: wheel setpoint/feedback -> candidate motor output;
- `SERVICE_ACTUATION`: a normál L9 úttól elkülönített service ág.

Az `L10B_SAFETY_GATE_LINEAGE` rögzített, de a teljes raw safety snapshot hiánya
miatt nem replayelhető. PlantAdapter, fizikai szimuláció, teljes planner/matcher
replay és digitális iker nincs a V2.1 scope-ban. A V1 executor-only és V2
semantic-stage capture-ek változatlanul támogatottak.

A megőrzött V1 alap a production `MotionExecutor` determinisztikus, offline regressziós
alaprétege. A production control loopban egy opt-in passzív tap minden ticknél
rögzíti az executor tényleges bemenetét, közvetlen kimenetét, a végső motor-
kimenetet és annak safety-lineage-ét. Az offline futás ugyanazt az importált
`motion_executor.MotionExecutor` osztályt használja; motor driver vagy más
hardveres kimenet nincs az offline útban.

## Hatókör

V1-ben a visszajátszott production komponens a `MotionExecutor`. Az EKF,
szenzorfeldolgozás, resolver, planner és safety supervisor kimenete capture-ölt
bemenet/lineage, nem újraimplementált párhuzamos logika. Ez szándékosan a
legkisebb olyan szelet, amely tick-pontos production regressziót ad. A későbbi
adapterek ugyanerre az immutable capture/result szerződésre épülhetnek.

Capture csak explicit környezeti kapcsolóval indul:

```bash
R2B4_REPLAYER_CAPTURE=1 python3 os.py
```

Opcionálisan előre megadható egy egyedi azonosító:

```bash
R2B4_REPLAYER_CAPTURE=1 \
R2B4_REPLAYER_CAPTURE_ID=capture_room_001 \
python3 os.py
```

A kapcsoló önmagában nem ad mozgásparancsot és nem változtat safety döntést.
Valódi mozgás továbbra is kizárólag a projekt live-motion/Test Hub szabályai
szerint végezhető. A capture író bounded queue-t és külön háttérszálat használ;
queue overflow, íráshiba, cycle gap, hibás idő vagy nem szabályos lezárás esetén
a capture `INVALID`, illetve megszakításkor `ACTIVE` marad. Egyik sem kaphat
`MATCH` replay eredményt.

## Adatelrendezés

Minden futás közbeni adat a forrásfától elkülönített `replayer_data/` alatt van:

```text
replayer_data/
  captures/<capture_id>/
    capture_manifest.json
    source_manifest.json
    frames.jsonl
    config/*.json
  results/<capture_id>/<result_id>/
    replay_manifest.json
    comparisons.jsonl
    diff.json
    diagnosis.json
    evidence.json
    integrity.json
```

Deploymentkor a `replayer_data/` könyvtárat írható runtime-data gyökérként kell
provisionálni és ki kell zárni a read-only managed source védelemből. Ha ez nem
írható, a capture inicializáció fail-closed módon meghiúsul, miközben a robot
runtime capture nélkül tovább működhet; ilyen esetből nem keletkezik lezárt
referencia.

A lezárt capture teljes fájlkészlete hash-elt, a frame-ek egymásra épülő
SHA-256 láncot alkotnak, a manifest saját kanonikus hash-t kap, és a könyvtár
best-effort read-only jogosultsággal zárul. Replay soha nem ír a capture
könyvtárba. Minden replay új `result_id`-t és új könyvtárat kap.

A capture a releváns aktív konfiguráció (`control_mode`, `fizika`, `speed_map`,
`vezerles`) immutable másolatát használja replaykor. A production komponens
forrásfájljainak hash-e szintén rögzül. A jelenlegi forrás/config eltérése az
evidence-ben látható; ez regressziós provenance, miközben a viselkedési `MATCH`
az executor kimenetek egyezését jelenti.

## Offline használat

```bash
python3 -m replayer list
python3 -m replayer inspect <capture_id>
python3 -m replayer verify <capture_id>
python3 -m replayer replay <capture_id>
python3 -m replayer verify-result <capture_id> <result_id>
```

V2.1 capture réteg- és inkluzív monotonic időablak-célzása:

```bash
python3 -m replayer replay <capture_id> --layer L8 \
  --start-monotonic-ns 1060000000 --end-monotonic-ns 1080000000
python3 -m replayer replay <capture_id> --layer L9
```

A `--layer` ismételhető (`L8`, `L9`, `SERVICE`). Részleges replay előtt a
kiválasztott rétegek teljes capture-prefixe warm-upként lefut, így a stateful
controller/executor az ablak kezdetén helyes állapotból indul. Üres ablak vagy
elérhető boundary nélküli rétegválasztás `ERROR`, soha nem `MATCH`.

Az `inspect` gyors, manifest-only összefoglaló, és szándékosan nem ad acceptance
verdictet. A V2.1 `diagnosis.json` az első eltérés monotonic idejét, rétegét,
bemenetét, elvárt/tényleges kimenetét és releváns replay-state-jét tartalmazza;
az integritásmanifest ezt az artefaktumot is védi.

A replay alapértelmezett abszolút PWM toleranciája `1e-9`. `MATCH` csak akkor
lehetséges, ha az immutable capture minden integritási, teljességi és időzítési
kapun átmegy, minden frame lefut a production adapterrel, és a PWM-pár valamint
az `output_reason` minden ticknél egyezik. Egyéb végállapotok: `MISMATCH`,
`INVALID_CAPTURE`, `ERROR`.

Az evidence külön jelzi, hány ticknél módosította a downstream safety/output
guard az executor kimenetét. V1 ezt a végső kimenetet lineage-ként őrzi, nem
próbálja offline újraalkotni a safety rendszert.
