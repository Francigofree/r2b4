# R2B4 HRI P0 upgrade — 2026-09-23

Cél: a jelenlegi ember→LLM→RobotInterface út lezárása valódi, visszakövethető
ember–robot interakcióvá anélkül, hogy új robot-authority, motorút vagy control-loop
munka keletkezne.

## Source-first alap

A csomag a repo ezen ellenőrzött állapotára készült:

- commit: `04b5cb704b1f532028c49058f4fc416a1c5deb9d`
- `r2b4_voice/action_executor.py` blob: `41bf9d0f420e7a1ef39e1578c56aeab9da5e0d0a`
- `r2b4_voice/voice_service.py` blob: `30ea393905b6270660609dc637d084bc18b0ab72`
- `v3/process_sidecars.py` blob: `2c84fea69c9f151d13ef480fe9f31d04226fabba`
- `v3/mcap_capture.py` blob: `85bc246806e542556f4318975bcfd551e44f5ccf`
- `v3/test_hub_next.py` blob: `32489500690dc2fa836eeb25027fbd7d378eba2f`

Az installer a fenti working-tree blobokat **írás előtt** ellenőrzi. Eltérés esetén
nem módosít semmit.

## P0.1 — action → mission → behavior visszacsatolás

- A `VoiceActionExecution` megőrzi a kanonikus `RobotInterface.execute()` eredményéből
  a `command_id`-t és `mission_id`-t.
- Ha az adapter csak `command_id`-t ad, az aktuális L5 szabály szerint a kért mission
  identity `mission-{command_id}`; a Test Hub ezt utólag a tényleges L5 evidence-dzsel
  ellenőrzi.
- Új `HriBehaviorObserver` figyeli a **read-only `v3.status`** projectiont 10 Hz körül.
  Nem ír commandot, runtime-ot, safety-t vagy motort.
- A watcher naplózza a mission/navigation/safety/actuation állapotváltásokat ugyanazzal
  az `interaction_id / turn_id / command_id / mission_id` lánccal.
- Pozitív action végrehajtásakor az LLM fizikai sikert nem állíthat kész tényként:
  az első válasz determinisztikus `Rendben.`; a követés aktív navigációja után a
  read-only observer adhat `Követlek.` visszajelzést. Már aktív követés invalidálásakor
  a visszajelzés: `Megálltam, jelenleg nem tudlak biztonságosan követni.`

## P0.2 — STOP interrupt lane THINKING/SPEAKING közben

- A normál párbeszéd továbbra is half-duplex.
- Külön daemon STOP-listener ugyanazt a **nem destruktív bounded AudioFramePortot**
  olvassa saját sequence cursorral; nem lesz második mikrofon-owner.
- Csak a meglévő szűk `is_stop_intent()` exact phrase lista indít STOP-ot.
- STOP közvetlenül a kanonikus `v3.command.stop` interface útra kerül, LLM/action
  proposal nélkül.
- THINKING közben kapott STOP latch-eli az aktuális turnt; későn érkező pozitív LLM
  proposal `REJECTED:INTERRUPTED_BY_STOP`, ezért nem indíthat újra mozgást.
- A későn visszatérő LLM szövege helyett a turn `Megálltam.` választ ad.

**Pontos korlát:** a jelenlegi repo STT-je Groq-alapú. Ezért a verbális STOP LLM-független,
de nem hálózatfüggetlen. A repóban nincs jelenlegi helyi Vosk/Whisper/keyword-ASR út;
a csomag nem csempész be új provider/perception alrendszert.

## P0.3 — HRI evidence az MCAP/Test Hub rendszerben

- A voice host egy kicsi, bounded `runtime/hri_events.ndjson` journalba ír.
- Ez **nem evidence authority** és nem kerül a control loopba.
- A már izolált capture sidecar futás végén legfeljebb 512 releváns HRI eseményt
  olvas be, és `v3.hri_event` observationként betolja a meglévő MCAP generikus
  `/r2b4/event` csatornájába.
- Triggerelt capture esetén a finalize-kor érkező HRI event engedélyezett a post-window
  után is; az eredeti monotonic idő az event payloadban marad.
- Test Hub új outputjai:
  - `hri_timeline.ndjson`
  - `hri_summary.json`
- A Test Hub a behavior timeline alapján visszaköti az eseményeket a tényleges
  `command_id / mission_id` lánchoz és számolja többek között:
  - action → behavior start
  - action → motion start
  - interrupt STOP count
  - korrelált HRI event count
- Az MCAP marad az authority; a HRI summary/timeline derived evidence.

## Installer — mi változott a korábbi hibákhoz képest

Az `apply_upgrade.py`:

1. source blob preflightot végez **minden írás előtt**;
2. minden módosítást memóriában stage-el;
3. Python AST parse ellenőrzést fut a stage-elt fájlokra;
4. repo-local tartós backupot készít: `runtime/upgrade_backups/hri_p0_YYYYmmdd_HHMMSS/`;
5. same-directory temp + `fsync` + `os.replace` atomi fájlcserét használ;
6. telepítés után célzott regressziót fut;
7. bármilyen write- vagy pytest-hibánál automatikusan visszaállítja az összes eredeti
   fájlt és törli az újonnan létrehozott fájlokat.

Tehát nincs olyan állapot, mint a korábbi `replace_once` hibánál, amikor részleges
upgrade maradhat a repóban.

## Telepítés

Csomag kibontása után először opcionálisan futtatható teljes no-write preflight:

```bash
cd /home/alba/project_r2b4
python3 /UTVONAL/r2b4_hri_p0_upgrade_20260923/apply_upgrade.py \
  --root /home/alba/project_r2b4 --check-only
```

Ez source blob + payload SHA-256 + minden stage-elt Python AST ellenőrzést elvégez,
de semmit nem ír a repóba.

Telepítés:

```bash
cd /home/alba/project_r2b4
python3 /UTVONAL/r2b4_hri_p0_upgrade_20260923/apply_upgrade.py \
  --root /home/alba/project_r2b4
```

A tesztek alapértelmezetten lefutnak. `--no-tests` csak diagnosztikai/mentési célra
van; normál telepítéshez ne használd.

A sikeres végén az installer kiírja a backup pontos útját.

### Kézi rollback élő teszt után

Ha az installer PASS, de az élő validáció során mégis vissza kell állni:

```bash
python3 /UTVONAL/r2b4_hri_p0_upgrade_20260923/restore_backup.py \
  --root /home/alba/project_r2b4 \
  --backup /home/alba/project_r2b4/runtime/upgrade_backups/hri_p0_YYYYmmdd_HHMMSS
```

## Élő validáció

### 1. Statikus/regressziós gate

```bash
cd /home/alba/project_r2b4
r test voice
r test testhub
r test
```

Elvárt: PASS.

### 2. Voice hardver/provider check

```bash
python3 -m r2b4_voice.voice_service --check
```

Elvárt: mikrofon és speaker PASS, STT/LLM/TTS kulcsok PASS.

### 3. Élő voice service EXECUTE módban

```bash
python3 -m r2b4_voice.voice_service --action-mode execute --capture-mode alap
```

Mondd:

1. `Alba`
2. `Kövess`

Elvárt log-részlet:

```text
voice: action=v3.command.follow_person status=EXECUTED command_id=... mission_id=mission-...
```

A robot először csak `Rendben.` választ mondhat. A `Követlek.` csak a read-only live
observer által látott `navigation.status=ACTIVE` után jöhet.

### 4. STOP THINKING közben — kritikus P0 teszt

Indíts olyan follow/face utasítást, amely után a voice state THINKING, majd rögtön mondd:

```text
Állj!
```

Elvárt:

- log: `voice: STOP interrupt executed via RobotInterface`
- fizikai mozgás STOP
- a későn visszaérő action nem indulhat el;
- HRI journalban legyen `INTERRUPT_STOP_DETECTED`, `STOP_REQUESTED`,
  `INTERRUPT_STOP_EXECUTED`;
- ha az LLM ezután pozitív actiont ad, legyen `ACTION_REJECTED` +
  `INTERRUPTED_BY_STOP`.

Ellenőrzés:

```bash
tail -n 50 runtime/hri_events.ndjson
```

### 5. STOP SPEAKING közben

Amikor Alba TTS-t játszik, mondd:

```text
Stop!
```

Elvárt: a kanonikus STOP végrehajtódik és a mozgás megszűnik. A jelenlegi
`PcmWavePlayer` blokkoló playbackje ettől még befejezheti az éppen játszott mondatot;
a P0 garancia itt a robotmozgás megszakítása, nem az audio-player process azonnali
kilövése.

### 6. Behavior feedback

Új follow futásnál figyeld:

- `Rendben.` — action execution elfogadva;
- `Követlek.` — csak captured/read-only navigation ACTIVE után;
- ha ACTIVE után INVALIDATED lesz: `Megálltam, jelenleg nem tudlak biztonságosan követni.`

Ezzel ellenőrizhető, hogy a robot nem az LLM állításából, hanem a tényleges V3
behavior állapotból beszél.

### 7. MCAP + Test Hub HRI evidence

Állítsd le szabályosan a futást, hogy a capture sidecar finalize és postprocess
lefuthasson. Ezután:

```bash
LATEST="$(cat runtime/.r2b4_capture_path)"
EVID="${LATEST%.mcap}.evidence"
cat "$EVID/hri_summary.json"
sed -n '1,160p' "$EVID/hri_timeline.ndjson"
```

Elvárt:

- `availability: PRESENT`
- `event_count > 0`
- voice motion action után `command_count >= 1`
- `correlated_event_count > 0`
- timeline-ban ugyanaz a `command_id` és tényleges `mission_id`
- `correlation.command_mission_match: true` a korrelálható eventeken
- `latencies` alatt action→behavior/motion idő, ha a behavior elindult.

### 8. Bizonyítsd, hogy az MCAP az authority

```bash
LATEST="$(cat runtime/.r2b4_capture_path)"
python3 - "$LATEST" <<'PY'
import sys
from v3.mcap_reader import McapReader, EVENT_TOPIC
r = McapReader(sys.argv[1])
rows = []
for _message, value in r.iter_json_messages(topics=(EVENT_TOPIC,)):
    if isinstance(value, dict) and value.get("source_topic") == "v3.hri_event":
        rows.append(value)
print("HRI_MCAP_EVENTS", len(rows))
for row in rows[-10:]:
    print(row.get("payload"))
assert rows, "nincs HRI event az MCAP-ban"
PY
```

Elvárt: `HRI_MCAP_EVENTS` nagyobb mint 0.

## Érintett production fájlok

Módosul:

- `r2b4_voice/action_executor.py`
- `r2b4_voice/voice_service.py`
- `v3/process_sidecars.py`
- `v3/mcap_capture.py`
- `v3/test_hub_next.py`

Új:

- `v3/hri_evidence.py`
- `tests/test_voice_hri_p0.py`
- `tests/test_v3_hri_evidence.py`

A control-layer/L0–L12 kód, motor driver, scheduler és hardware edge nem változik.
