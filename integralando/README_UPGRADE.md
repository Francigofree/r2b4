# R2B4 System Behavior Contract — P0 upgrade

Upgrade: `system_behavior_contract_p0_v1`

Authority: `R2B4_SYSTEM_BEHAVIOR_CONTRACT.md`

## Mit változtat

- wake default: `robot`;
- ready válasz default: `figyelek`;
- wake csak a voice interaction sessiont nyitja meg, **nem indítja a V3-at**;
- a voice session saját, V3-tól független lifecycle-t kap;
- session timeout: 10 s folyamatos csend;
- új typed `ExecutionModeSelector`;
- conversation / observation / robot-action külön út;
- V3 csak robot-action esetén indulhat a meglévő `RobotInterface -> OperatorController -> canonical V3` útvonalon;
- a voice action megőrzi a kért capture módot;
- `camera.photo` V3 nélkül használható ugyanazzal a kalibrált `NativePicamera2Camera` implementációval;
- kamera-fotó idején az operator transition lock kizárja a párhuzamos V3 kamera-ownert;
- V3 futása közben `camera.photo` továbbra is fail-closed (`OWNED_BY_V3_RUNTIME`);
- Gemini és Piper defaultok nem változnak;
- exact STOP fast-path / interrupt lane nem kerül lecserélésre.

## Tudatos P0 korlát

A P0 observation út **kalibrált kameraképet készít**, de még nem küldi vissza a képet külön vizuális reasoning partnernek. Emiatt a voice válasz a capture receipt alapján csak azt igazolja, hogy a kamerakép elkészült; nem állít képtartalmat. Az ER2 stream/preview közös routingja P1.

## Telepítés

Nincs venv és nincs full-repo SHA gate.

```bash
cd <kicsomagolt_csomag>
python3 installer.py
```

A default célrepo:

```text
/home/alba/project_r2b4
```

Más repo:

```bash
python3 installer.py --repo /másik/project_r2b4
```

A telepítő:

1. fájlonként ellenőrzi a kiinduló Git blob SHA-kat;
2. backupot készít `.upgrade_backups/system_behavior_contract_p0_v1_<timestamp>/` alá;
3. sorban alkalmazza a módosításokat;
4. hozzáadja a P0 új moduljait és regressziós tesztjét;
5. `py_compile` ellenőrzést futtat;
6. futtatja a célzott P0 tesztet;
7. futtatja a canonical `./r test` CORE és `./r test feature` kaput;
8. hibánál automatikusan restore-ol.

Sikeres telepítés után:

```bash
cd /home/alba/project_r2b4
./r voice restart
./r voice check
```

Szélesebb offline kapu, élő teszt előtt:

```bash
./r test full
```

## Új / módosított diagnosztika

`runtime/wake_status.json` új mezői:

- `session_active`
- `session_remaining_ms`
- `last_execution_mode`

A HRI journal új eseményei között megjelenhet:

- `WAKE_DETECTED`
- `VOICE_SESSION_OPEN`
- `VOICE_SESSION_CLOSED`
- `OBSERVATION_COMPLETED`
- `OBSERVATION_FAILED`

## Rollback

A telepítő kiírja a backup könyvtárát. Ha később kézi rollback kell, az adott backupból a fájlok visszamásolhatók. A telepítő saját install-hibája automatikusan rollbackel.
