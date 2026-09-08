# R2B4 V3

Az R2B4 canonical robot-runtime-ja a natív, determinisztikus V3 L1–L12
pipeline. Az architekturális authority a `STRUKTURALIS_RETEGEK_V3.md`; a
pillanatnyi igazságot a `v3/` production source és a négy aktív konfiguráció
adja:

- `conf/hardver.json`
- `conf/fizika.json`
- `conf/speed_map.json`
- `conf/vezerles.json`

## Production entrypoint

```bash
python3 v3_process_runtime.py --approval native-resident-v3
```

A processz headless. A command és status edge alapértelmezett helye
`runtime/v3_command.json`, illetve `runtime/v3_status.json`; a `runtime/`
könyvtár generált, nem verziózott adat.

Külső command kliens:

```bash
python3 -m v3.control_cli --help
```

## Offline validáció

```bash
python3 -m v3.import_guard
python3 -m pytest -q
python3 -m v3.replay replay <capture.json> --output /tmp/v3-replay.json
python3 -m v3.replay verify-result /tmp/v3-replay.json
```

A `v3.test_hub` ugyanazt a production V3 replay útvonalat használja. Live
motorfutás csak explicit approval-lal indítható; az offline tesztek és replay
nem nyitnak GPIO- vagy motor-capabilityt.
