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
./r2b4 start
./r2b4 status
./r2b4 diag

./r2b4 capture start
./r2b4 capture status
./r2b4 capture stop

./r2b4 forward 0.15
./r2b4 backward 0.15
./r2b4 wheels 0.10 0.20
./r2b4 teleop 0.15 -0.20
./r2b4 roomcruise
./r2b4 proba c full

./r2b4 stop
./r2b4 shutdown
```

A launcher a szükséges runtime-ot automatikusan elindítja. A capture-választó
`c alap` (alapértelmezett, 8+2 másodperces triggered capture), `c full`
(folyamatos felvétel) vagy `c nincs` (capture kikapcsolva); `--capture MODE`
alakban is megadható. A régi `runtime start/status/diag`, `forward start`,
`mozog start` és `roomcruise start` parancsok továbbra is működnek.
A `--no-trigger` és régi `nocapture` csak a launcher által kért ALAP
mozgás-trigger élesítését hagyja ki; a runtime fail-evidence szabályai ettől
nem változnak.

A shell launcher a `v3.operator_cli` adaptert indítja. Az orchestration
Pythonból a `v3.operator_controller.OperatorController` API-n érhető el,
opcionális `OperatorEvent` callbackkel. A mozgásparancsokat és a heartbeatet
továbbra is a canonical `v3.control_cli` kezeli; a robot readiness- és
safety-döntéseit a V3 runtime hozza. A `proba` tíz rögzített TELEOP fázist futtat,
öt másodperces szünetekkel; a fordulás végét a runtime yaw-telemetriája jelzi.

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

## Test Hub

Az alapértelmezett belépő az integrált Test Hub Next. Paraméter nélkül a
legújabb `runtime/captures/*.mcap` fájlt elemzi, a capture profiljának megfelelően,
és 5 Hz-es áttekintést, valamint `agent_view.json` összefoglalót készít:

```bash
python3 -m v3.test_hub
python3 -m v3.test_hub run capture.mcap --output-dir /tmp/egyedi-hub --replay full
python3 -m v3.test_hub view capture.mcap --hz 1 --output /tmp/egyedi-overview.ndjson
python3 -m v3.test_hub compare before.mcap after.mcap
```

A `v3.test_hub_next` közvetlenül is használható. Az alapértelmezett cél
`<capture>.evidence/`; meglévő adatot nem ír felül, ismételt futáshoz
új `--output-dir` szükséges. Az 1/5/10 Hz-es nézet megőrzi a capture-ben rögzített
állapotváltásokat is. A compare különbségeket mutat, automatikus verdict nélkül.
A nézetek származtatott adatok; az MCAP és a canonical replay marad az authority.

A V2 háttérmodulok szükséges függőségek. A runtime utófeldolgozása és az operator
ugyanezt az automatikus profilválasztást használja. A régi `validate`, `replay`,
`inspect`, `verify-result`, `verify-evidence` kompatibilitási parancsok is megmaradnak.

### Capture-alapú feldolgozási profilok

A profilválasztás forrása az MCAP `r2b4.capture.tick_sample_hz` metaadata.
A `--hz` csak az áttekintő nézet sűrűségét állítja; nem módosítja az elemzési profilt.

| Capture | Profil | Vizsgálatok |
|---|---|---|
| 50 Hz | `FORENSIC_LOW_LEVEL` | Változatlan tick-/layer-vizsgálatok, timing, raw szenzorok, canonical replay és sweep |
| ≤10 Hz (jelenleg 1/5/10 Hz) | `BEHAVIORAL_HIGH_LEVEL` | Mission/command életciklus és azonosítók, navigáció, mozgásblokkolás, pose/covariance trend, safety-kimenetel |

Régi, frekvencia-metaadat nélküli MCAP a korábbi 50 Hz-es utat kapja. Hibás vagy
nem támogatott frekvencia explicit hiba; a rendszer nem találgat a tick-távolságból.

A behavioral profil a meglévő capture-beolvasást és mission-epizódokat használja.
A szándékos tick-kihagyás nem adatvesztés. A navigációs stagnálás legalább 5 másodperc
megfigyelt, változatlan progress után jelez, egy aktív autonóm missionon belül.
Mission-/command-váltás vagy három mintaperiódusnál nagyobb rés új ablakot kezd;
TELEOP-ra nincs progress-stagnálási szabály. A covariance-jelzés legalább 5 másodperc
alatti, első és utolsó minta közötti négyszeres növekedést vizsgál.

Az alacsony szintű device-, admission-, layer-fault- és timing-észrevétel legfeljebb
`WARNING`, és nem válik bizonyított root cause-zá. A megfigyelt mission/safety FAULT,
mozgásblokkolás vagy navigációs probléma `FINDING` marad. A sérült, hiányos vagy
nem finalizált capture evidence-hiba (`FAIL`), ezt a ritkább mintavétel nem menti fel.

A behavioral profilban a replay és sweep `NOT_APPLICABLE`, a `--replay` értékétől
függetlenül. Ez nem `MATCH`, és nem teljesít exact-replay követelményt. Ehhez 50 Hz-es
capture szükséges. A mozgás- és lokalizációs quality fájlok mintavételezett trendeket
tartalmaznak; jerk-, control-jitter- és raw-szenzor folytonossági verdictet nem adnak.

A választott profil és korlátai az `agent_view.json`, `diagnosis.json`, agent brief,
GUI manifest és áttekintő nézet részei. A `behavior_summary.json`,
`behavior_episodes.ndjson` és `behavior_timeline.ndjson` tartalmazza a mission- és
command-kapcsolatokat. A számlálók rögzített mintákat számolnak, nem teljes control
tick-számot vagy időarányt; a köztes, nem rögzített állapotok nem rekonstruálhatók.

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
