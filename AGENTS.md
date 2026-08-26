# R2B4 minimalis agent munkaszerzodes

A teljes projektfara ervenyes. Nincs hasznalhato Git workflow vagy Git-alapu
helyreallitas.

## Indulas

1. Futtasd: `bash scripts/bootstrap_guard.sh --brief`; hibanál allj meg.
2. Futtasd: `python3 tools/agentctl.py capsule`.
3. Csak a kapszula `source_routes` fajljait, az aktiv konfiguraciot es a
   futasazonos compact evidence-et olvasd. Teljes torteneti dokumentumot vagy
   nyers logot csak konkret bizonyitekhiany eseten nyiss meg.
4. Ujrakezdeskor add vissza az elozo `context_fingerprint` erteket; `UNCHANGED`
   eseten nincs dokumentum-ujraolvasas.

Az aktiv task gepi SSOT-ja `runtime/agent_coordination/current_change.json`. Kezzel
karbantartott Markdown state nem authority es nem kotelezo agentkontextus.

## Munka

- Normal uj task: `python3 tools/agentctl.py open --task-id <id> --goal <cel> --files ...`.
  A parancs izolalt candidate-et hoz letre es kiirja a `workspace_path`-ot.
- Kizarolag a kiirt task-workspace-ben dolgozz. Canonical forrast kozvetlenul,
  `sudo`-val vagy jogosultsag-atallitassal ne irj.
- Uj fajl: `python3 tools/agentctl.py claim --files ...` modositas elott.
- Tesztkornyezet: `python3 tools/agentctl.py workspace`; a teszt `cwd` es
  `PYTHONPATH` erteke a candidate gyokere.
- Gepi audit: `python3 tools/agentctl.py audit`.
- Lezaras: `python3 tools/agentctl.py close --reason <ok> --test '<cmd> :: PASS'`.
  Ez verifikalt candidate-et es receiptet zar le, canonical promotiont nem vegez.
- Promotion csak kulon, explicit emberi kapun, az `agentctl promote` explicit
  task-azonos jovahagyasaval. Agent ezt nem futtathatja automatikusan.
- Megszakitott promotion: `agentctl recover <id>`; explicit source rollback:
  `agentctl restore <id> --approve restore:<id>`.
- Agent-infrastruktura modositas kulon friss sessionben, kizarolag
  `--mode AGENT_INFRA_CHANGE --approve agent-infra:<id>` mellett nyithato.
- A normal mod egyetlen agent. Legfeljebb egy celzott reviewer vagy root-cause
  agent aktivalhato `agentctl review` paranccsal, csak a gepi policy altal
  elfogadott konkret evidence-re. Fix fan-out, rekurziv delegacio es parhuzamos
  kodiro tiltott.
- `workspace_write`, canonical promotion, runtime, live motion, latest-publish es teljes pytest csak
  a sajat gepi lease birtokaban futhat.
- Stabil reteget csak konkret hibabizonyitek es celzott regresszio mellett
  modosits. Bypass-, legacy-, direkt normal PWM- vagy alternativ control utat
  ne hozz vissza; safety-, PASS- es minosegi kaput ne lazits.

## Bizonyitas

- Forras es aktiv config > futasazonos Hub summary/incident/ownership > stabil
  baseline > torteneti dokumentum vagy nyers log.
- Modositas utan celzott teszt kotelezo; kozos contract/bootstrap/test
  infrastruktura eseten teljes `pytest` is.
- Elo mozgas csak explicit felhasznaloi keret, friss preflight, Test Hub profil
  es vegso IDLE/PWM-null ellenorzes mellett indulhat.
- Teszt SSOT: `python3 tools/r2b4_test_hub.py list|run|report`; a `latest_*`
  pointer volatilis, tartos evidence csak a futasazonos session.
