# R2B4 agent- és V3 validációs útmutató

Ez routing dokumentum, nem task-state vagy tesztprofil-registry. Az aktív task,
a deklarált fájlok, workspace, lease-ek és receipt-ek gépi authorityja az
`agentctl`; a robotarchitektúra authorityja ettől külön a
`STRUKTURALIS_RETEGEK_V3.md`.

## Agent workflow

```bash
bash scripts/bootstrap_guard.sh --brief
python3 tools/agentctl.py capsule
python3 tools/agentctl.py open --task-id <id> --goal <cel> --files <fajlok...>
python3 tools/agentctl.py workspace
```

Egyetlen `CHANGE` mód van. Egy task deklarált scope-ja tartalmazhat egyszerre
production source/config és agent-infrastruktúra fájlokat. Ez csak workflow-
egyszerűsítés: a robot-, capture-, replay- és live authorityk nem egyesülnek.

Módosítani kizárólag a kiírt candidate workspace-ben szabad. Új fájlt írás
előtt `agentctl claim --files ...` vesz scope-ba. A close kötelező determinisztikus
auditot futtat, ellenőrzi a tesztevidenciát, resealeli a candidate-et és immutable
receiptet készít:

```bash
python3 tools/agentctl.py close --reason <ok> --test '<parancs> :: PASS'
```

A lényegi védelmek változatlanok: egy író lease, deklarált hash-scope,
out-of-scope audit failure, canonical drift detection, teljes pytest lease a
megosztott agent-infra változásokhoz, valamint külön, task-azonos emberi
promotion-kapu. Az agent nem promotál automatikusan.

Agent-infrastruktúra kizárólag:

- task és candidate workspace;
- lease és egyíró-policy;
- audit, receipt, promotion/recovery/restore;
- context capsule és agent-policy.

A robotarchitektúra, aktív robotconfig, logging, Test Hub, capture és Replayer
robot-validációs infrastruktúra; attól nem válik agent-infrává, hogy az agent
használja.

## V3 robot-validáció

Az authority- és fejlesztési sorrend:

```text
V3 source + aktív config
→ a STRUKTURALIS_RETEGEK_V3.md szerint
→ Replayer + Test Hub V3
→ csak szükség esetén, explicit keretben live hardware
```

A Replayer V3 a saját CLI-jével fut; az `agentctl` nem importálja és nem indítja
el. Az offline út alapműveletei:

```bash
python3 -m v3.replay inspect <capture.json>
python3 -m v3.replay replay <capture.json> --output <result.json> \
  --start-tick-id 100 --end-tick-id 140 --start-layer L3 --end-layer L10
python3 -m v3.replay verify-result <result.json>
python3 -m v3.test_hub validate <capture.json> \
  --output-dir <run-id-directory>
python3 -m v3.test_hub verify-evidence <run-id-directory/evidence_index.json>
```

A közös, szimulátorhoz is újrahasználható határ
`input source → production V3 → output sink`. A `v3.capture` általános, nem
tesztprofil-specifikus sink; a `v3.replay` automatikusan lefuttatja a kijelölt
első tick előtti state warmupot; a `v3.test_hub` egy explicit run-directoryba
ír `inspect`, replay-result, L1–L12 diagnosis és hash-index evidence-et.
FAULT ticknél a capture a sikeres folytonos L1-prefixet, az explicit
`fault_layer` értéket és a kötelező utolsó L12 outputot őrzi; a hibás és az azt
követő rétegek `not_executed` diagnosztikát kapnak, nem fiktív outputot.

A capture, replay és live külön modul és külön authority. A replay offline
magja nem importál live hardver-authorityt és nem birtokol GPIO- vagy motor-
capabilityt. Tartós bizonyíték kizárólag a futásazonos capture/result/diagnosis;
`logs/latest/latest_*` csak kényelmi pointer, nem authority. A Replayer V2.1 és
a legacy Test Hub V3 fejlesztésben history/compatibility, nem diagnosztikai vagy
evidence-authority.

Live hardver csak explicit felhasználói keret, friss V3 preflight, szükséges
lease és végső IDLE/PWM-null ellenőrzés mellett használható. Infrastruktúra-
validáció kedvéért robotmozgás nem indul.

## Kiegészítő agent és helyreállítás

A default egyetlen agent. Legfeljebb egy evidence-bound reviewer vagy root-cause
szerep aktiválható `agentctl review` paranccsal; nem írhat párhuzamosan és nem
delegálhat rekurzívan.

Megszakadt promotionhoz `agentctl recover <id>`, explicit visszaállításhoz
`agentctl restore <id> --approve restore:<id>` használható. A managed canonical
forrás read-only; a runtime/log/session területek külön írhatók.
