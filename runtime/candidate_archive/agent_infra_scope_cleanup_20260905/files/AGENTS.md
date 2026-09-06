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

- Uj task: `python3 tools/agentctl.py open --task-id <id> --goal <cel> --files ...`.
  A parancs izolalt candidate-et hoz letre es kiirja a `workspace_path`-ot.
- Egyetlen `CHANGE` mod van. Ugyanabban a deklaralt scope-ban source/config es
  agent-infrastruktura fajl is modosithato; az authorityk ettol nem egyesulnek.
- Kizarolag a kiirt task-workspace-ben dolgozz. Canonical forrast kozvetlenul,
  `sudo`-val vagy jogosultsag-atallitassal ne irj.
- Uj fajl: `python3 tools/agentctl.py claim --files ...` modositas elott.
- Tesztkornyezet: `python3 tools/agentctl.py workspace`; a teszt `cwd` es
  `PYTHONPATH` erteke a candidate gyokere.
- Elozetes gepi audit igeny szerint: `python3 tools/agentctl.py audit`; a kotelezo
  fail-closed auditot a `close` automatikusan lefuttatja, ezert nem kell duplan futtatni.
- Lezaras: `python3 tools/agentctl.py close --reason <ok> --test '<cmd> :: PASS'`.
  Ez diffet, fingerprintet, evidence-indexet, auditot es receiptet automatikusan
  eloallit; canonical promotiont nem vegez.
- Replay-first hibakereses: a Replayer V3 `inspect`, celzott replay,
  `verify-result` es diagnosis parancsai futasazonos evidence-et adnak. Az
  `agentctl` ezt nem futtatja es nem robot-validacios authority.
- `supersede` az audit-valid candidate-et resealeli es clone-lineage szamara
  megorzi; torolni csak az explicit `discard` torol. Folytatas:
  `agentctl open ... --clone-from <superseded-task-id>`.
- Promotion csak kulon, explicit emberi kapun, az `agentctl promote` explicit
  task-azonos jovahagyasaval. Agent ezt nem futtathatja automatikusan.
- Megszakitott promotion: `agentctl recover <id>`; explicit source rollback:
  `agentctl restore <id> --approve restore:<id>`.
- Agent-infrastruktura csak task/workspace/lease/promotion/capsule/agent-policy.
  Robotarchitektura, config, logging, Test Hub es Replayer nem agent-infra.
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

- V3 source es aktiv config > `STRUKTURALIS_RETEGEK_V3.md` > Replayer V3
  `inspect`, celzott replay es futasazonos diagnosis/evidence > szukseg eseten
  live hardware > torteneti dokumentum vagy nyers log. Contracttal ellentetes regi regresszios
  teszt explicit non-authority evidence, nem irhatja felul a canonical contractot.
- A celzott teszt az alapertelmezett. A scope/contract/risk alapjan az infra
  automatikusan kerhet teljes `pytest`-et; az agent teljes regressziot ettol
  fuggetlenul is futtathat indokolt diagnosztikai cellal, a gepi lease birtokaban.
- Elo mozgas csak explicit felhasznaloi keret, friss V3 preflight es vegso
  IDLE/PWM-null ellenorzes mellett indulhat.
- Robot-validacios SSOT a Replayer + Test Hub V3; tartos evidence csak a
  futasazonos output, `latest_*` pointer nem authority.
