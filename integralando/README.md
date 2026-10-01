# R2B4 DIAG evidence-native refactor

Source-first target:

- repo: `Francigofree/r2b4`
- base branch: `main`
- base HEAD: `dcc4c2c77928bc06b1103c949757024869a9df6b`
- source date: 2026-10-01

## Cél

```text
ObservationHub -> capture -> MCAP -> EVI (.evidence) -> tools/diag -> analyzers
```

A `tools/diag` kizárólag sealed EVI evidence bundle-t olvas. Nem nyit MCAP-ot.
A kimenet diagnosztikai adat ember és LLM számára: tények, mérések,
kapcsolatok, adatminőség és evidence-reference-ek. Nincs recommendation,
suggestion vagy automatikus root-cause verdict.

## CLI

```bash
r diag
r diag --json
r diag full latest
r diag list
r diag admission safety latest
r diag safety latest
r diag navigation latest --json
```

`r diag` alapértelmezése: **full DIAG a legutóbbi `.evidence` bundle-re**.

## Telepítés

A Pi-n:

```bash
cd /tmp/r2b4_diag_refactor_20261001
python3 apply_diag_evidence_refactor.py --root /home/alba/project_r2b4
```

Ha a HEAD azóta változott, az installer alapból megáll. Csak a változások
áttekintése után:

```bash
python3 apply_diag_evidence_refactor.py \
  --root /home/alba/project_r2b4 \
  --allow-head-mismatch
```

A source anchorok `--allow-head-mismatch` mellett is kötelezőek.

Az installer:

1. ellenőrzi a repo alakját és HEAD-et,
2. preflightolja a három launcher integrációs patch-et,
3. backupot készít `.upgrade_backups/diag_evidence_refactor_*` alatt,
4. telepíti `tools/mcap_evidence/reader.py` + `tools/diag/` fájlokat,
5. a `v3/diag` régi MCAP implementationt compatibility entrypointra redukálja,
6. az `r diag` launchert `python -m tools.diag`-ra állítja,
7. eltávolítja a régi MCAP-only feature tesztet,
8. compileall + célzott pytest + két CLI smoke tesztet futtat,
9. validation hiba esetén visszaállítja a backupot.

## Telepítés utáni gyors ellenőrzés

```bash
r diag list
r diag
r diag --json > /tmp/r2b4_diag.json
```

Aktív resident V3 runtime mellett a top-level `r diag` nem indul el; a host
operator lockot a teljes offline elemzés alatt tartja. A runtime állapot alias
`r d` és a `r rt diag` útvonal ettől függetlenül megmarad.
