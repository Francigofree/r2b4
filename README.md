# R2B4 V3

Az R2B4 Raspberry Pi 5 alapú, beltéri differenciálhajtású robot natív,
determinisztikus V3 vezérlő- és diagnosztikai rendszere.

A production control canonical útja az L1–L12 pipeline. Az L12 az egyetlen normál
motor-write authority; capture, telemetry, GUI, agent és offline evidence tool nem kerülheti meg
a production control- és safety-utat.

## Authority

- Architekturális invariánsok: `STRUKTURALIS_RETEGEK_V3.md`
- Pillanatnyi implementáció: `v3/` production source + aktív config
- Konkrét futás szoftveres evidence-e: finalizált, integritás-ellenőrzött capture +
  canonical Replayer + explicit MCAP Evidence Compiler
- Történeti dokumentáció és nyers log: háttéranyag

Aktív konfigurációk:

- `conf/hardver.json`
- `conf/fizika.json`
- `conf/speed_map.json`
- `conf/vezerles.json`

## CPU-ütemezés

<!-- SCHEDULER_MANAGED_AFFINITY_POLICY_V1 -->
A `conf/vezerles.json` `runtime_affinity` szekciója két üzemmódot támogat.

- `enabled: true`: az R2B4 szerepenként explicit Linux CPU-affinity maszkokat
  alkalmaz és ellenőriz. A `control_cpus` pontosan egyelemű, és egyik másik
  szerep maszkja sem fedheti át.
- `enabled: false`: az R2B4 **nem alkalmaz saját CPU-pinninget**. A runtime,
  matcher, LiDAR, planner, capture, szenzor- és szolgáltatásfolyamatok az
  örökölt host/cgroup CPU-készleten maradnak, elhelyezésüket a Linux scheduler
  végzi.

A jelenlegi production baseline `enabled: false`, tehát scheduler-managed mód.
A konfigurációban megmaradó `*_cpus` mezők ilyenkor nem aktív elhelyezési
parancsok; schema/replay kompatibilitási értékek. A schema ettől még szigorú:
`control_cpus` egyetlen CPU, a többi szerep vele diszjunkt, a CPU-listák nem
lehetnek üresek, duplikáltak, negatívak vagy nem egészek. Ez lehetővé teszi,
hogy a policy később egyetlen `enabled` váltással ismét aktiválható legyen.

`strict: true` csak bekapcsolt affinity-policy mellett kényszeríti az OS/cgroup
maszkok alkalmazását és visszaellenőrzését. `enabled: false` mellett nincs
R2B4-affinity alkalmazás. Ez nem jelent OS-szintű CPU-izolációt: más Linux
folyamatok, kernel threadek és IRQ-k ütemezését az R2B4 nem szabályozza.

Futó rendszer ellenőrzése: `python3 tools/v3_performance_audit.py live`.
Scheduler-managed módban az affinity-rész `NOT_APPLICABLE` /
`SCHEDULER_MANAGED` eredménnyel, sikeres exit kóddal tér vissza; `enabled: true`
mellett a tényleges `/proc` affinity-layoutot auditálja.

## Használat

Az egyetlen ajánlott ember/agent belépő a gyökér `r` launcher. A `r` nem robotikai
authority: a robotparancsokat változtatás nélkül a `v3.interface_cli` felé delegálja,
a host/developer segédek pedig külön launcher-infrastruktúrában maradnak.

```bash
./r help
./r help fp
./r help er2 stream
./r commands
./r commands --json
./r s --json

./r s
./r d
./r rc 30
./r fp 20
./r f 10 0.15
./r x
./r sd
```

A `r` argumentum nélkül csoportosított súgót ad. A `r help PARANCS` és a
`r PARANCS --help` megmutatja a használatot; robotparancsoknál a rövidítések is
működnek, és a mozgássúgó tartalmazza a mértékegységeket, alapértékeket és
capture-választókat. Elírt parancsnál a launcher javítási javaslatot ad, és hibával
kilép; a javasolt parancsot nem hajtja végre.

A robotparancsok `--json` kapcsolója a parancs előtt és után is használható
(`r --json s`, `r s --json`). Sikeres futáskor stdout-on egyetlen JSON-eredmény
jelenik meg; a folyamatjelzések stderr-re kerülnek. Az ER2, Git és nyers pytest
saját opciókezelése megmarad.

A timed mozgásparancsok a szükséges runtime-ot automatikusan elindítják, majd STOP
következik. A parancs által indított runtime leáll, és a capture
finalizálása látható; a már előzőleg futó runtime és annak capture-beállításai
megmaradnak. `0` másodperc folyamatos módot jelent, ilyenkor a runtime futva marad
explicit STOP/shutdown kérésig. A pozicionális argumentumok sorrendje változatlan:
először az idő, utána a sebességértékek.

Capture mintavétel alapértelmezése 10 Hz. Választható: `c 50`, `c 10`, `c 5`,
`c 1`; a capture mód továbbra is `c alap`, `c full` vagy `c nincs`. Példák:

Az 1/5/10 Hz-es capture kompakt rendszerállapot-napló: health, pose, mission,
navigáció, mozgás és safety. Nem készít replay-checkpointot, és nem továbbít
nyers szenzorpayloadot vagy teljes térkép/rollout-geometriát. A `c 50` explicit
sensor-debug: teljes tick-input, checkpoint és közvetlen LiDAR → capture raw
evidence; ez több CPU-t és tárhelyet igényel. A `c full` a rögzítési ablakot
választja, önmagában nem kapcsolja be a raw adatokat. Pontos replay csak teljes
50 Hz-es evidence-ből lehetséges.

```bash
./r rc 30 c 10
./r rc 30 c 50
./r fp 20 c nincs
./r cap status
```

Offline evidence és pytest:

```bash
./r evi runtime/captures/<capture>.mcap
./r evi query <capture>.evidence --tick-id 1505
./r test
./r test release
./r test pack feature
./r test all
```

A permanent pytest authority két kis manifest: `tests/gate/manifest.json` és
`tests/scenarios/manifest.json`. A `tests/packs/` fejlesztői evidence, nem
automatikus release-követelmény. Policy: `docs/PYTEST_POLICY.md`; canonical
runner: `v3/test_runner.py`. Nyers pytest továbbra is elérhető: `./r pytest ...`.

Fejlesztő/host segédek például: `r git`, `r gitre`, `r tools`, `r tool NAME`,
`r cpu`, `r cpu2`, `r disc`, `r mem`, `r temp`, `r ps`, `r net`, `r usb`, `r i2c`,
`r host`, `r version`. A teljes aktuális felület agent-barát JSON formában:
`r commands --json`; az élő RobotInterface capability-k: `r caps`.

A régi gyökér `r2b4` launcher megszűnt. A belső `v3.operator_cli` modul megmarad,
mert a resident runtime-session technikai child-process entrypointja használja; ez
nem második felhasználói launcher.

Közvetlen production entrypoint:

```bash
python3 v3_process_runtime.py --approval native-resident-v3
```

A processz headless. A command és status edge alapértelmezett helye:

- `runtime/v3_command.json`
- `runtime/v3_status.json`

A `runtime/` könyvtár generált, nem verziózott adat.

Közvetlen command kliens:

```bash
python3 -m v3.control_cli --help
```

## Passzív observation és MCAP capture

A production eredményekből passzív, data-blind `ObservationHub` fan-out készül:

```text
V3 completed objects
        |
        v
 ObservationHub
   |        |
   |        +--> telemetry / GUI: LATEST
   |
   +--> capture: RELIABLE, bounded, required
```

A capture consumer nem része a control authoritynak. A required capture delivery
adatvesztése explicit integrity failure; ilyen artifact nem nevezhető teljesnek és
nem használható exact replay `MATCH` bizonyítékként.

A final capture authority MCAP:

```text
runtime/captures/*.mcap
```

A finalizálás atomi: a partial artifact csak sikeres drain/integrity/finalize után
válik canonical capture-ré.

Az RPi5 runtime saját standard-library MCAP writer/readert használ. Az
`mcap` Python csomag **nem runtime dependency**, ezért az MCAP használatához nem
kell külön venv vagy `pip install mcap` a roboton.

Az opcionális `requirements-interop.txt` csak fejlesztői/CI interoperabilitási
ellenőrzéshez tartalmaz független MCAP implementációt; robot-runtime-ra nem kell
telepíteni.

## MCAP Evidence Compiler

A `r evi` explicit offline developer tool: MCAP → teljes, strukturált evidence.
A runtime leállása capture-finalizálással véget ér; nincs automatikus feldolgozás.

```bash
./r evi capture.mcap --output /tmp/capture.evidence --workers auto
./r evi verify /tmp/capture.evidence --source capture.mcap
./r evi query /tmp/capture.evidence --tick-id 1505 --field 'expected.layers.L11.*'
```

A tool minden recoverelhető message-et, teljes raw LiDAR payloadot és minden JSON
leaf pathot megőriz/indexel. Sérült inputból `PARTIAL` bundle is készülhet. Nem
futtat diagnózist, replayt vagy pytestet. Részletek: [MCAP Evidence Compiler](docs/MCAP_EVIDENCE.md).

## Canonical replay

Az MCAP marad az authority artifact. A Replay Bridge csak a szükséges bounded
ablakot materializálja ideiglenes kompatibilitási formába, majd a meglévő canonical
V3 Replayer ugyanazt a production composition/TickEngine utat futtatja offline.

Ha a validation scope replayt kér, csak tényleges `MATCH` teljesíti a replay gate-et.
`MISMATCH`, replay error vagy el nem végzett kért replay nem `PASS`.

Hiányos capture nem software divergence; capture/evidence integrity és replay
verdict külön fogalom.

## Evidence és diagnosztikai állítások

Az MCAP Evidence Compiler nem általános truth authority és nem ad diagnózist.

- Közvetlen capture-tény: evidence a saját capture-scope-jában.
- Artifact-integrity eredmény: ellenőrzött integrity verdict.
- Canonical replay `MATCH`/`MISMATCH`: ellenőrzött replay verdict a vizsgált scope-ban.
- Root-cause rangsor, anomáliaértelmezés és javítási javaslat: derived diagnosis.

Diagnosztikai következtetés csak közvetlen kauzális evidence mellett lehet
`PROVEN`; egyébként `INDICATED`, `NOT_PROVEN` vagy `EVIDENCE_BLOCKED`.

## Offline validáció

```bash
python3 -m v3.import_guard
python3 -m pytest -q
```

Az offline tesztek, MCAP inspection, evidence compilation és replay nem nyitnak GPIO- vagy
motor-capabilityt. Live motorfutás csak explicit approval-lal indítható.
