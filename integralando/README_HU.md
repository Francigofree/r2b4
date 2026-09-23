# R2B4 canonical Action Catalog P0/P1 upgrade — 2026-09-23

Source baseline: `6cfc56b69df84adcfe185f410d7d046f790877f9`.

## Cél

Egyetlen kanonikus, géppel olvasható `v3.command.*` action-katalógus legyen a statikus action contract SSOT.
A live `available/ready/reason` továbbra is az adapterek aktuális állapota, tehát a katalógus nem új runtime authority.

## Architektúra

`v3/action_catalog.py` -> `RobotInterface.capabilities()` -> launcher/GUI/agent/LLM/voice.

- nincs `_ALLOWED`
- nincs `_LIMITS`
- nincs `LLM_ACTION_ALLOWLIST`
- a paraméterek required/min/max/default szerződése egyszer szerepel
- `voice_exposed` fail-closed default (`False`), de a jelenlegi fejlesztési szakaszban mind a 8 kanonikus V3 action explicit `True`
- Gemini/Groq structured output schema turnönként a friss voice-exposed action nézetből épül
- a STOP gyors/determinisztikus voice út nem változik

## Telepítés

Kicsomagolás után:

```bash
cd /home/alba/project_r2b4
python3 /UTVONAL/r2b4_action_catalog_p0_p1_20260923/apply_upgrade.py \
  --root /home/alba/project_r2b4 \
  --check
```

Csak `CHECK OK` után:

```bash
python3 /UTVONAL/r2b4_action_catalog_p0_p1_20260923/apply_upgrade.py \
  --root /home/alba/project_r2b4
```

Az installer:
- a baseline Git blobot ellenőrzi minden módosított meglévő fájlnál;
- source drift esetén írás előtt leáll;
- staging után AST parse-t futtat;
- repo-local backupot készít;
- atomikusan ír;
- targeted pytestet futtat;
- teszthibánál automatikusan visszaállít.

## Ellenőrzés

```bash
python3 - <<'PY'
from v3.robot_interface import RobotInterface
caps = RobotInterface('/home/alba/project_r2b4').capabilities()
print(caps['schema'])
print(caps['action_catalog_schema'])
for name, item in caps['action_catalog']['actions'].items():
    print(name, 'voice=', item['voice_exposed'], 'params=', list(item['parameters']))
PY
```

Elvárt 8 action:
- stop
- forward
- backward
- teleop
- wheels
- explore
- face_person
- follow_person

## Kézi restore

Az installer kiírja a backup pathot. Visszaállítás:

```bash
python3 /UTVONAL/r2b4_action_catalog_p0_p1_20260923/restore_backup.py \
  --root /home/alba/project_r2b4 \
  --backup /home/alba/project_r2b4/runtime/upgrade_backups/action_catalog_p0p1_YYYYmmdd_HHMMSS
```
