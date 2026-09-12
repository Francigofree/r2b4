# R2B4 V3

Az R2B4 az Alba Raspberry Pi 5 alapú, beltéri differenciálhajtású robot natív,
determinisztikus V3 vezérlő- és diagnosztikai rendszere.

A production control canonical útja az L1–L12 pipeline. Az L12 az egyetlen normál
motor-write authority; capture, telemetry, GUI, agent és Test Hub nem kerülheti meg
a production control- és safety-utat.

## Authority

- Architekturális invariánsok: `STRUKTURALIS_RETEGEK_V3.md`
- Pillanatnyi implementáció: `v3/` production source + aktív config
- Konkrét futás szoftveres evidence-e: finalizált, integritás-ellenőrzött capture +
  canonical Replayer/Test Hub
- Történeti dokumentáció és nyers log: háttéranyag

Aktív konfigurációk:

- `conf/hardver.json`
- `conf/fizika.json`
- `conf/speed_map.json`
- `conf/vezerles.json`

## Használat

Ajánlott belépő a gyökérben lévő launcher:

```bash
./r2b4 runtime start
./r2b4 runtime status
./r2b4 runtime diag

./r2b4 capture start
./r2b4 capture status
./r2b4 capture stop

./r2b4 forward start [speed_mps] [nocapture]
./r2b4 mozog start <left_mps> <right_mps> [nocapture]
./r2b4 roomcruise start [nocapture]

./r2b4 stop
./r2b4 shutdown
```

A launcher a szükséges runtime-ot automatikusan elindítja. A `nocapture` csak a
launcher által kért mozgás-capture-t hagyja ki; a runtime fail-evidence szabályai
ettől nem változnak.

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

## Test Hub V2

Az új Test Hub ugyanabból a final MCAP authorityból dolgozik CLI, agent és későbbi
GUI számára.

Olcsó integritási összefoglaló:

```bash
python3 -m v3.test_hub_v2 inspect runtime/captures/<capture>.mcap
python3 -m v3.test_hub_v2 inspect runtime/captures/<capture>.mcap --deep
```

Kis, agent-barát diagnosztikai összefoglaló:

```bash
python3 -m v3.test_hub_v2 agent runtime/captures/<capture>.mcap
```

Teljes run-bound diagnosztikai csomag:

```bash
python3 -m v3.test_hub_v2 diagnose runtime/captures/<capture>.mcap \
  --output-dir /tmp/r2b4-diagnosis
```

Célzott, bounded evidence-lekérés:

```bash
python3 -m v3.test_hub_v2 query runtime/captures/<capture>.mcap \
  --ticks 100:120 --layers L2,L3,L6,L9,L12
```

Evidence-index ellenőrzése:

```bash
python3 -m v3.test_hub_v2 verify-evidence /tmp/r2b4-diagnosis/evidence_index.json
```

A Test Hub képes integritás-ellenőrzésre, bounded/indexelt queryre, timeline-ra,
agent briefre, incident triage-ra és szükség esetén canonical replayre.

## Canonical replay

Az MCAP marad az authority artifact. A Replay Bridge csak a szükséges bounded
ablakot materializálja ideiglenes kompatibilitási formába, majd a meglévő canonical
V3 Replayer ugyanazt a production composition/TickEngine utat futtatja offline.

Ha a validation scope replayt kér, csak tényleges `MATCH` teljesíti a replay gate-et.
`MISMATCH`, replay error vagy el nem végzett kért replay nem `PASS`.

Hiányos capture nem software divergence; capture/evidence integrity és replay
verdict külön fogalom.

## Evidence és diagnosztikai állítások

A Test Hub nem általános truth authority.

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

Az offline tesztek, MCAP inspection, Test Hub és replay nem nyitnak GPIO- vagy
motor-capabilityt. Live motorfutás csak explicit approval-lal indítható.
