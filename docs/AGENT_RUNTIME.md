# R2B4 agent validacios utmutato

Ez routing dokumentum, nem task-state es nem profil-registry. Az aktualis
profilok es contractok SSOT-ja a forras:

```bash
python3 tools/r2b4_test_hub.py list
```

Az aktiv task, a fajlhash-ek, az agentmod es a bizonyitek-referenciak gepi
SSOT-ja:

```bash
python3 tools/agentctl.py capsule
python3 tools/agentctl.py status
```

Az SSOT a writable `runtime/agent_coordination/current_change.json`; nem resze
a vedett canonical forrasnak.

## Offline modositas

1. `bash scripts/bootstrap_guard.sh --brief`
2. `python3 tools/agentctl.py capsule`
3. `agentctl open`, majd modositas kizarolag a kiirt task-workspace-ben;
4. `agentctl workspace` szerinti candidate `cwd`/`PYTHONPATH` alatt celzott teszt;
5. `agentctl audit`;
6. kozos contract/bootstrap/test-infrastruktura eseten `full_pytest` lease es
   teljes `python3 -m pytest -q`;
7. `agentctl close`, amely a candidate hash-eket, auditot es teszteket vedett
   receiptbe zarja, de nem ir canonical forrast.

Canonical promotion kulon emberi kapu. A promotion ujraellenorzi a base-et,
auditot, teszteket es receiptet; teljes vedett snapshotot, fsync-elt recovery
journalt es atomikus fajlcseret hasznal. Megszakitas utan `agentctl recover`,
kesobbi explicit visszaallitasra `agentctl restore` hasznalhato.

Infrastruktura-validacio kedveert elo robotmozgas nem indul.

## Runtime-, motion- vagy szenzormodositas

1. celzott offline regresszio es a blast radius szerinti teljes pytest;
2. runtime status es egyetlen runtime processz;
3. a Test Hub profil sajat preflightja;
4. szukseges friss measurement-truth/M0 vagy fail-closed M0-mini;
5. a legszukebb bizonyito profil;
6. futasazonos summary, FAIL eseten incident es ownership manifest;
7. vegso IDLE es PWM `0/0`.

Elo futas csak explicit felhasznaloi keretben, `live_motion` lease mellett
indulhat. Ad-hoc mozgas helyett Test Hub profil kotelezo.

## Evidence rend

1. Forraskod es aktiv konfiguracio.
2. `<run_dir>/summary.json` vagy scenario summary.
3. FAIL eseten futasazonos incident es ownership manifest.
4. Stabil baseline.
5. Torteneti dokumentum; nyers log csak konkret bizonyitekhianyra, szuk
   szeletben.

`logs/latest/latest_*` csak volatilis pointer. Tartós bizonyíték a
`logs/session_<timestamp>/` alatti futasazonos artefakt. A task lezárásának
tartós bizonyítéka `logs/agent_tasks/<task-id>/receipt.json`.

## Agentmod

A default egyetlen agent. Az `agentctl capsule` csak jogosultsagot jelezhet egy
celzott kiegeszito szerepre; aktivaciohoz `agentctl review` es gepileg
ellenorzott evidence kell.

- `independent_reviewer`: vedett vagy kozos guard contract valtozas független
  ellenorzesere;
- `root_cause_analyst`: legalabb ket kulon futas azonos hibasignature-je es
  futasazonos immutable FAIL incident eseten.

Egyszerre legfeljebb egy kiegeszito agent lehet. Nem irhat kodot, nem indithat
uj agentet, runtime-ot vagy elo profilt. Parhuzamos iro tiltott.

## Lease-ek

```bash
python3 tools/agentctl.py lease acquire full_pytest
python3 tools/agentctl.py lease status canonical_promotion
python3 tools/agentctl.py lease acquire runtime_control
python3 tools/agentctl.py lease acquire live_motion
python3 tools/agentctl.py lease acquire latest_artifact_publish
python3 tools/agentctl.py lease release <resource>
```

A lease gepi runtime-allapot, nem LLM-kontektszoveg. Lejar, taskhoz kotott es
mas task altal nem oldhato fel.

A Test Hub a `latest_*` pointerek publikaciojat automatikusan a
`latest_artifact_publish` lease alatt vegzi. A pointer tovabbra sem tartos
evidence-authority.

## Forrasvedelem

- Normal `CODE_CHANGE` nem claimelhet agent-infrastruktura fajlt.
- `AGENT_INFRA_CHANGE` kulon explicit taskmod es emberi jovahagyas.
- A candidate eldobhato, mikozben canonical hash valtozatlan marad.
- A managed canonical forras root-owned es read-only; a runtime/log/session
  teruletek kulon writable-ak.
- A vedett base-seal, receipt, snapshot es promotion journal
  `/var/lib/r2b4-agent/` alatt van. Azonos user szandekos `sudo` bypassa maradek
  adminisztratori kockazat, nem normal fejlesztoi jogosultsag.

## Valtozatlan vedett szerzodesek

- egyetlen `UNIFIED` vezerlesi ut;
- egy tick = egy intent, egy EKF pose, egy motor output;
- normal motoriras csak `MotionExecutor`;
- pose owner `EKF_POSE_ODOMETRY_SSOT` a `R2B4_BOOT_ROBOT_MAP` frame-ben;
- scan matcher `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1`, kulon `spawn`
  processz, `1/1` latest-only queue es V2 confidence;
- safety-, PASS- es minosegi kapu nem lazithato.

A reszletes ertekeket nem ez a dokumentum duplikalja: azok a
`project_rules/protected_baseline.json`, az aktiv config es az implementacios
forras gepileg ellenorzott ertekei.
