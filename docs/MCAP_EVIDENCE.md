# MCAP Evidence Compiler

Az explicit offline developer tool otthona `tools/mcap_evidence/`. A Test Hub
kivezetésre került. Runtime shutdown → capture finalize → sidecar exit → runtime
exit; ezután nincs automatikus evidence-feldolgozás vagy RobotInterface capability.

```bash
./r evi capture.mcap
./r evi capture.mcap --output /tmp/run.evidence --workers auto
./r evi verify /tmp/run.evidence --source capture.mcap
./r evi query /tmp/run.evidence --tick-id 1505 --layer L11
./r evi query /tmp/run.evidence --field 'expected.layers.L11.*' --limit 10
python3 -m tools.mcap_evidence --help
```

Nincs implicit „latest capture”, batch, diagnosis, triage, tuning, replay vagy
pytest. Ezek külön fogyasztók/feladatok. A compiler-státusz nem robotikai verdict.
Canonical replay továbbra is külön: `python3 -m v3.replay replay ...`.

## Feldolgozás és státusz

1. Egy strukturális forward scan az induláskor megnyitott file `fstat` EOF-jáig.
   A message-ek eredeti byte-jai ideiglenes work-unit fájlokba kerülnek. Futás
   közben növekvő forrásnál a későbbi byte-ok nem kerülnek az adott bundle-be.
2. ProcessPool decode/export/index; work-unitonként saját shard és SQLite index.
   IPC-n csak fájlnevek, unit-azonosítók és számlálók mennek. `auto` esetén a
   workerszám `min(os.cpu_count() or 1, work_unit_count)`; `--workers 1` soros út.
3. Forrássorrendben determinisztikus merge, coverage és file-hash manifest.
4. Kötelező verification, majd atomi publikálás ugyanazon filesystemen.

| Státusz | Jelentés | Exit code |
|---|---|---|
| COMPLETE | Strukturálisan teljes forrás, minden recoverelt message feldolgozva | 0 |
| PARTIAL | Sérült/hiányos forrás, minden biztonságosan recoverelt message elszámolt | 0 |
| FAILED | Compiler/I/O/decoder hiba; új final bundle nincs | 1 |
| INTERRUPTED | Ctrl-C/SIGTERM; új final bundle nincs | 130 |

CLI argumentumhiba: 2. SIGKILL után maradhat `.tmp-*` könyvtár, de félkész final
bundle nem publikálódik. A compiler nem kezeli vagy törli más futások ideiglenes
könyvtárait.

A hiányzó footer/DataEnd/closing magic vagy hibás CRC nem akadályozza a már
biztonságosan keretezett message-ek mentését. Bizonytalan record length után nincs
byte-pattern alapú resync. Félbevágott uncompressed chunkból a teljes nested
message-ek kimenthetők. Unknown recordok eredeti tartalma megmarad.

Az R2B4 uncompressed MCAP út stdlib-only. Külső zstd/lz4 capture-hez a megfelelő
`zstandard`/`lz4` Python decoder szükséges; hiánya explicit FAILED, nem hamis
„minden feldolgozva” állítás. A strukturális scan egy chunk tartalmát tartja RAM-ban;
a fájl teljes tartalmát nem tölti be. A strict `v3/mcap_reader.py` változatlan.

## Bundle

- `manifest.json`: `R2B4_MCAP_EVIDENCE_MANIFEST`, compiler státusz, Git commit,
  dirty jelzés, a compiler forrásfájljainak hash-e, parancs és output inventory.
- `coverage.json`: `R2B4_MCAP_EVIDENCE_COVERAGE`, message-, byte-, leaf- és
  normalized-view elszámolás. `recovered = json_decoded + quarantined = exported`.
- `integrity.json`: snapshot méret/hash, source változás, CRC/framing tények és
  recoverelt source-message stream hash. Ez nem production capture verdict.
- `index.json`, `field_index.json`: `R2B4_MCAP_EVIDENCE_INDEX`, topic/message
  számlálók és minden JSON leaf path census-a. A keresési postings `index.sqlite`-ban.
- `source/records.ndjson`, `channels.ndjson`, `schemas.ndjson`, `metadata.ndjson`,
  `chunk_checks.ndjson`, `corrupt_ranges.ndjson`: strukturális evidence.
- `source/messages/*.ndjson`: `R2B4_MCAP_EVIDENCE_SOURCE` sorok. Stabil message ID,
  source offset, channel, sequence, eredeti ns időpontok, payload méret/hash,
  dekódolt JSON és az eredeti byte-ok base64 alakban is.
- `source/quarantined_messages_*.ndjson`: UTF-8/JSON hibás vagy nem JSON-encodingú
  message-ek teljes eredeti byte-jai. Semmi nem tűnik el dekódolási hiba miatt.
- `normalized/`: ticks, events, runtime, checkpoints, layers, sensors, lidar és
  általános topics nézetek. Minden sor `message_id` + `source_pointer` lineage-et
  hordoz. Nincs domain-értelmezés vagy verdict.

A shard neve unit- és partszámot tartalmaz. `--shard-bytes` alapértéke 32 MiB;
egyetlen nagy sort nem vág ketté. A raw LiDAR pontok, timestamp, health, revision
és geometria teljes egészében megmaradnak, akkor is, ha egy sor nagyobb a célnál.
Tömörített chunknál az eredeti compressed byte-ok külön `source/chunks/` fájlban
maradnak; a message lokátora chunk-offset + uncompressed nested-offset.

A canonical field path JSON pointer: `/expected/layers/L11/foo`. Üres object és
array is leaf. Querynél egyszerű dotted rövidítés és glob (`*`, `?`) használható;
pontot/slasht tartalmazó kulcsokra a JSON pointer pontos, escaped alakja ajánlott.
Arrays minden indexelt eleme szerepel: nincs sampling vagy field whitelist.
Topic-, channel-, sequence-, offset-, message-, tick-, layer-, sensor- és ns
időintervallum-szűrés SQLite indexből történik; MCAP nem nyílik meg.

A verification minden export hash-ét, a source-stream és output message-ek
azonosságát, a field censust, byte accountingot és normalized projection lineage-et
ellenőrzi. `--source` ezen felül az eredeti snapshot byte-jainak hash-ét is újraolvassa.
Enélkül a verification a hordozható bundle belső egyezőségét ellenőrzi.

## Publikálás és védelem

A compiler `<output>.tmp-*` könyvtárba dolgozik. A final path csak sikeres
verification után jelenik meg. A Linux `renameat2` no-replace/exchange művelete
atomi publikálást és atomi felülírást biztosít. Meglévő output alapból hiba.

`--overwrite` kizárólag valódi, nem symlink könyvtáron engedélyezett, amelynek
`manifest.json` schema-ja pontosan `R2B4_MCAP_EVIDENCE_MANIFEST`. Régi Test Hub vagy
ismeretlen könyvtár védett. Hiba/megszakítás esetén a korábbi bundle megmarad.
Inputot tartalmazó célkönyvtár nem választható. A runtime capture-eket és korábbi
lokális diagnosztikai adatokat a compiler nem módosítja.
