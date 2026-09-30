# R2B4 Test Hub — kanonikus működési szerződés

Ez a dokumentum a Test Hub lezárt, nem verziózott modulnévhez kötött felületét írja le.
A Test Hub offline diagnosztikai rendszer. Nem része a motor-, safety- vagy control-authority útvonalnak.

## 1. Authority és adatút

A futás elsődleges bizonyítéka a finalizált MCAP capture:

```text
V3 production L1–L12
       |
       v
ObservationHub / capture
       |
       v
runtime/captures/<run>.mcap        <- authority
       |
       +--> Test Hub offline process
                |
                v
       <run>.evidence/              <- derived evidence
```

Alapszabályok:

- Az MCAP az authority; a `.evidence/` könyvtár származtatott, újragenerálható adat.
- A Test Hub csak finalizált capture-t elemez; production control döntést nem hoz.
- A runtime a hardver-ownership lezárása után külön Python processzben indítja a Test Hubot.
- Replay esetén is az MCAP marad authority. A replay bridge szükség esetén csak ideiglenes, bounded JSON ablakot materializál `/tmp` alatt, majd törli.
- A replay/integrity státusz és a robot viselkedési finding külön fogalom. Determinisztikusan replayelhető hibás viselkedés nem válik automatikusan „jó” futássá.

## 2. Egyetlen publikus belépési pont

Minden emberi, agent és belső integráció számára a kanonikus Python entrypoint:

```bash
python3 -m v3.test_hub
```

A publikus parancskészlet:

```bash
# teljes feldolgozás; argumentum nélkül a legújabb MCAP
python3 -m v3.test_hub
python3 -m v3.test_hub run runtime/captures/<capture>.mcap

# olcsó, származtatott nézet
python3 -m v3.test_hub view runtime/captures/<capture>.mcap --hz 10

# több még fel nem dolgozott capture
python3 -m v3.test_hub batch

# before/after, automatikus winner/verdict nélkül
python3 -m v3.test_hub compare before.mcap after.mcap

# Test Hubhoz kapcsolódó pytest profilok
python3 -m v3.test_hub test --scope feature

# pontos backend műveletek ugyanazon publikus entrypointról
python3 -m v3.test_hub inspect runtime/captures/<capture>.mcap --deep
python3 -m v3.test_hub diagnose runtime/captures/<capture>.mcap --output-dir /tmp/evidence
python3 -m v3.test_hub agent runtime/captures/<capture>.mcap
python3 -m v3.test_hub query runtime/captures/<capture>.mcap --ticks 0:10 --layers L1,L2,L12
python3 -m v3.test_hub verify-evidence /tmp/evidence/evidence_index.json
```

A korábbi, migrációs verziónevet hordozó Test Hub modulok nem publikus kompatibilitási felületek. A lezáró refaktor után ezek a külön modulok nem léteznek.

## 3. Belső modulhatárok

A publikus facade: `v3/test_hub.py`.

A belső komponensek funkció szerint kapnak nevet, nem migrációs verziószám szerint:

- `v3/test_hub_app.py`: orchestration, `run/view/batch/compare/test`, portable evidence összeállítása.
- `v3/test_hub_backend.py`: MCAP inspect, canonical diagnosis, bounded query, evidence verification.
- `v3/test_hub_analysis.py`: triage és incident-analízis.
- `v3/test_hub_diagnostic_coverage.py`: schema-drift és generikus mezőlefedettség.
- `v3/test_hub_behavior.py`: általános behavior evidence.
- `v3/test_hub_motion_quality.py`: mozgásminőség evidence.
- `v3/test_hub_localization_quality.py`: lokalizációs minőség evidence.
- `v3/test_hub_motion_tuning.py`: hangolási evidence.
- `v3/test_hub_task_evidence.py`: magas szintű task evidence.
- `v3/test_hub_views.py`: olcsó, agent-barát nézetek.
- `v3/test_hub_portable.py`: portable bundle, replay sweep, pytest integráció.
- `v3/test_hub_runtime.py`: kicsi, stdlib-only runtime → offline Test Hub process handoff.

A `R2B4_TEST_HUB_V2`, `R2B4_EVIDENCE_INDEX_V2` és hasonló stringek **evidence schema-verziók**. Ezeket a modulnevek megszüntetése miatt nem kell átnevezni; régi evidence olvashatóságát őrzik.

## 4. Capture profilok

A profil forrása az MCAP `r2b4.capture.tick_sample_hz` metaadata. A nézet `--hz` értéke nem írja felül az analysis profilt.

- `FORENSIC_LOW_LEVEL`: teljes 50 Hz capture; exact replay, timing és nyers szenzorvizsgálatok alkalmazhatók.
- `BEHAVIORAL_HIGH_LEVEL`: 1/5/10 Hz sampled capture; mission, navigation, motion, pose/covariance és safety trendek vizsgálhatók. Exact replay és alacsony szintű folytonossági verdict nem alkalmazható.

Régi, frekvencia-metaadat nélküli capture olvasása továbbra is támogatott a korábbi 50 Hz alapértelmezéssel. Ez adat-kompatibilitás, nem aktív Test Hub legacy útvonal.

## 5. Anti-drift: hogyan követi a Test Hub a rendszer fejlődését

A Test Hub két szinten követi automatikusan a production változásokat:

1. **Production dataclass contractok.** A `v3/diagnostic_contracts.py` a releváns production dataclassok mezőiből épít diagnosztikai szerződést és schema fingerprintet. Új mező alapból bekerül, hacsak nincs explicit kizárva.
2. **Generikus schema coverage.** A `v3/test_hub_diagnostic_coverage.py` minden capture-ben megjelenő scalar mezőt összesít, az ismeretlen új mezőket is. Jelzi a hiányzó/új mezőket és a schema driftet, de önmagában nem talál ki root cause-t.

Ez biztosítja, hogy például encoder/IMU/LiDAR/localization payload bővítés után a Test Hub legalább generikusan azonnal lássa az új evidence-t. Domain-specifikus következtetéshez továbbra is explicit analyzer-szabály és teszt szükséges.

## 6. Új production feature integrációs szabálya

Ha egy fejlesztés új diagnosztikai adatot hoz létre:

1. A production adat legyen benne a kanonikus capture-ben.
2. Ha dataclass-alapú kritikus diagnosztika, kerüljön be a `registered_diagnostic_contracts()` registrybe, ha még nincs lefedve.
3. A generikus coverage-nek az új mezőt automatikusan látnia kell.
4. Ha domain-specifikus értelmezés kell, azt külön analyzerben kell implementálni; a generic coverage nem gyárt automatikus root cause-t.
5. Új evidence artifact kerüljön a portable manifestbe és agent handoffba, ha távoli elemzéshez szükséges.
6. Legalább `tests/feature/test_*.py` contract teszt fedje le. Mély replay/fault eset a `tests/deep/` rétegbe kerüljön.
7. Production runtime/control modul nem importálhat nehéz Test Hub analyzert. Runtime oldalon csak a process-handoff engedett a hardver ownership lezárása után.

## 7. Evidence bundle minimum

Egy normál Test Hub futás fő artifactjai:

- `inspect.json`
- `triage.json`
- `diagnosis.json`
- `agent_brief.json`
- `agent_view.json`
- `timeline.ndjson`
- `overview_<hz>hz.ndjson`
- `diagnostic_schema_coverage.json`
- `behavior_summary.json` + behavior timeline/episodes
- `task_evidence_summary.json` + task timeline/episodes
- `motion_quality.json` + segments
- `localization_quality.json` + events
- `motion_tuning_summary.json` + segments
- `runtime_performance.json`
- replay artifactok, ha a profil szerint alkalmazhatók
- `portable_manifest.json`

A részletes derived analyzerek hibája legyen explicit `ERROR`/availability adat, és ne írja át indokolatlanul a canonical robot/replay státuszt.

## 8. Lezárási invariánsok

A Test Hub fejlesztési migráció akkor tekinthető lezártnak, ha:

- csak `python3 -m v3.test_hub` a publikus Python entrypoint;
- nincs aktív, migrációs verziónevet hordozó Test Hub modul;
- aktív forrás és dokumentáció nem hivatkozik ezekre a modulnevekre;
- a runtime handoff külön processzben marad;
- MCAP marad authority;
- a replay bridge bounded és ideiglenes marad;
- a sampled/forensic profilkülönbség explicit marad;
- a schema-drift coverage production contractból származik;
- a CORE/FEATURE/DEEP pytest réteg érvényes marad;
- a történeti `runtime/` evidence és `.upgrade_backups/` tartalom nincs átírva.

## 9. Kapcsolódó dokumentációk javasolt kiegészítése

A következő meglévő dokumentumokban elegendő rövid hivatkozás, nem kell megismételni ezt a teljes szerződést:

- `README.md`: Test Hub quick start + link erre a dokumentumra; ne említsen verziózott/„Next” modulnevet.
- `STRUKTURALIS_RETEGEK_V3.md`: az observation/capture után jelölje az offline Test Hubot authority-n kívüli fogyasztóként.
- `docs/PYTEST_POLICY.md`: jelezze, hogy a Test Hub `test --scope core|feature|deep|full` ugyanazt a pattern-alapú pytest registryt használja.
- Replay dokumentáció, ha van: rögzítse, hogy az MCAP az authority, a JSON csak bounded, ideiglenes bridge-formátum.
- Capture/ObservationHub dokumentáció, ha van: rögzítse a `finalized MCAP -> separate-process Test Hub -> derived .evidence` lifecycle-t.

A cél az SSOT: részletes Test Hub szabályok itt legyenek, a többi dokumentum csak a saját határfelületét és erre a fájlra mutató hivatkozást tartalmazza.
