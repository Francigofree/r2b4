# AlbaOS R2B4

Az R2B4 egy Raspberry Pi-alapu, differencialhajtasu robot runtime-ja. A projekt kozvetlenul a hardveren fut, forraskodja pedig allando sandboxban, Git workflow nelkul fejlodik.

## Agent munkakezdes

```bash
bash scripts/bootstrap_guard.sh --print
```

A guard ellenorzi a kotelezo agent dokumentumokat, a vedett rendszerazonositokat, a legacy utak tiltasat es a Test Hub alapartefaktokat. Ezutan a root `AGENTS.md` sorrendjet kell kovetni.

Hivatalos projektforrasok:

- Stabil architektura, szerzodesek es dontesek: `STRUKTURALIS_RETEGEK.md`
- Aktualis fejlesztesi allapot: `project_rules/current_state.md`
- Aktualis Git nelkuli valtozasmanifest: `project_rules/current_change.json`
- Validacios rend: `docs/AGENT_RUNTIME.md`
- Agent rendszerutasitas: `project_rules/agent_system_prompt.txt`

## Runtime

- Entrypoint: `os.py`
- Controller eletciklus es vegso control loop: `cont.py`
- GUI/API: `fastgui/`, alapertelmezett port `7860`
- Runtime status: `runtime/status.json`
- Aktiv konfiguracio: `conf/*.json`

Inditas es allapot:

```bash
python3 tools/agent_runtime_manager.py status
python3 tools/agent_runtime_manager.py start --ready-timeout-s 70
python3 tools/agent_runtime_manager.py stop --graceful-timeout-s 8 --hard-timeout-s 4
```

## Vedett fo szerzodesek

- Egyetlen vezerlesi mod: `UNIFIED`.
- Egy resolver, egy vegrehajto: `MotionExecutor`.
- Felso retegek csak fizikai `v`, `omega` vagy bal/jobb kereksebesseg celokat adnak.
- A negyiranyu speed map egyszer, a kozos kerek feed-forward retegben fut.
- Vegso pose SSOT: EKF; odometry mod: `LIDAR_FIRST`.
- Fix globalis frame: `R2B4_BOOT_ROBOT_MAP`, `+X` elore, `+Y` balra, yaw CCW pozitiv.
- Szenzor nem irhat motort; normal PWM csak az executor vegso kimenete.
- Safety-, PASS- es minosegi kuszob nem lazithato egy futas sikeressegeert.

A reszletes, forrasbol ellenorzott leiras a `STRUKTURALIS_RETEGEK.md` fajlban van.

## Teszteles

```bash
python3 tools/r2b4_test_hub.py list
python3 tools/r2b4_test_hub.py run <profile>
python3 tools/r2b4_test_hub.py report
python3 tools/r2b4_test_hub.py archive-logs --max-file-mb 12
```

Artefakt prioritas:

1. `logs/latest/latest_hub_summary.json`
2. `logs/latest/latest_hub_incident.json`
3. `<run_dir>/ownership_manifest.json`
4. celzott stdout/stderr tail
5. nyers session log csak vegso esetben

Az offline regresszio:

```bash
python3 -m pytest -q
```

A `pytest.ini` csak a `tests/` konyvtarat gyujti; hardvert olvaso utilityk nem reszei az altalanos tesztgyujtesnek.

## Git nelkuli valtozaskovetes

Az aktualis feladat csak deklaralt fajlokat hash-el, masolatot nem keszit:

```bash
python3 tools/agent_change_tracker.py begin --task-id <id> --goal "<cel>" --files <fajl> ...
python3 tools/agent_change_tracker.py status
python3 tools/agent_change_tracker.py finish --reason "<ok>" --test "python3 -m pytest -q :: PASS"
```

Az aktualis manifest: `project_rules/current_change.json`. Uj feladat kezdesekor felulirhato, de `ACTIVE` feladatot a tool nem enged csendben elvesziteni.

## Konyvtarak

- `controller/`: command, resolver, navigation, motion es status retegek
- `middleware/`: EKF, LIDAR odometry, encoder/IMU feldolgozas, speed-map feed-forward
- `motion_executor.py`: egyetlen normal kerek/PWM vegrehajto
- `safety/`, `startup/`: safety es eletciklus
- `driver/`, `sensors/`: hardver interface-ek
- `fastgui/`: operatori GUI es API
- `tools/`: validacios, diagnosztikai es agent eszkozok
- `tests/`: offline regresszio
- `logs/session_<timestamp>/`: runtime- es tesztfutasonkenti logok, summaryk,
  incident bundle-ok es mintafajlok
- `logs/latest`: symlink a legutobbi `logs/session_<timestamp>/` sessionre

Torteneti tervek vagy riportok fejlecuk szerint csak hatteranyagok; nem rendszer-SSOT-k.
