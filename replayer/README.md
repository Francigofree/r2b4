# R2B4 Replayer V1

A Replayer V1 a production `MotionExecutor` determinisztikus, offline regressziós
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
python3 -m replayer verify <capture_id>
python3 -m replayer replay <capture_id>
python3 -m replayer verify-result <capture_id> <result_id>
```

A replay alapértelmezett abszolút PWM toleranciája `1e-9`. `MATCH` csak akkor
lehetséges, ha az immutable capture minden integritási, teljességi és időzítési
kapun átmegy, minden frame lefut a production adapterrel, és a PWM-pár valamint
az `output_reason` minden ticknél egyezik. Egyéb végállapotok: `MISMATCH`,
`INVALID_CAPTURE`, `ERROR`.

Az evidence külön jelzi, hány ticknél módosította a downstream safety/output
guard az executor kimenetét. V1 ezt a végső kimenetet lineage-ként őrzi, nem
próbálja offline újraalkotni a safety rendszert.
