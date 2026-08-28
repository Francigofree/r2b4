# R2B4 Replayer V2.1

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
