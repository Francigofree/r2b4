# Natív MCAP capture és Test Hub

Az architekturális szabályokat a `STRUKTURALIS_RETEGEK_V3.md` határozza meg.
Ez a dokumentum az implementáció használatát és korlátait írja le.

## Runtime

```bash
python3 v3_process_runtime.py --approval native-resident-v3 \
  --capture-path runtime/captures/egyedi-futas.mcap \
  --capture-mode append_only
```

A runtime approval token nem helyettesíti a fizikai mozgás felhasználói engedélyét.
A `triggered` az alapértelmezett mód; SIGUSR1 vagy FAULT aktiválja a pre/post
ablakot. A `--capture-pre-event-ns` alapértéke 5 másodperc, a post ablaké 2.
Az első tick előtti manual trigger függőben marad az első completed tickig.
Tick nélküli, veszteségmentes futás nem készít capture-t.
Az `append_only` az első megfigyeléstől folyamatosan ír, trigger nélkül.

Egyetlen adatmailbox van: a Hub required RELIABLE subscriptionje.
A worker közvetlenül abból fogyaszt. A manual trigger és finish külön kis
Event/lock állapot, nem újabb adatqueue. A hub változatlan objektumreferenciát
ad át, nem serializál és nem végez consumer I/O-t. A globális sequence-ben
filtered topic miatt megengedett kihagyás; loss authority a subscription
`lost_count` és `assert_integrity()` állapota.

Leállítás: runtime/hardware publikáció vége → Hub close → mailbox teljes drain
→ subscription integrity ellenőrzése → MCAP finalize/flush/fsync → atomikus,
felülírást kizáró final publikáció és directory fsync → Test Hub V2.
A triggered ablak lezárulta után a worker tovább drainel a shutdownig;
final artifact csak ezután jelenik meg. I/O-hibánál a partial fájl megmarad.
Meglévő capture-t nem ír felül. Integrity-hibás final artifact FAIL marad.

A Test Hub derived fájljai a capture melletti `.evidence` könyvtárba kerülnek.
Automatikusan incident replay fut; az eredmény pontos scope-ja a
`replay_result.json` fájlban látható. Teljes replay külön kérhető.

## Offline használat

```bash
python3 -m v3.test_hub_v2 inspect capture.mcap --deep
python3 -m v3.test_hub_v2 diagnose capture.mcap --output-dir /tmp/egyedi-evidence --replay full
python3 -m v3.test_hub_v2 verify-evidence /tmp/egyedi-evidence/evidence_index.json
python3 -m v3.replay replay capture.mcap --output /tmp/egyedi-replay.json
```

A korábbi `v3.replay` és `v3.test_hub` belépők MCAP-ra is működnek.
A JSON capture API/importok kompatibilitási célból megmaradnak; minden
production encoder a `v3.capture_encoding` modulból származik.
A bridge csak structural/deep CRC és capture-integrity PASS után választ
checkpointot és készít ideiglenes JSON-t. Alapból legfeljebb 4096 tick és
128 MiB materializálható, beleértve a configot és checkpointot is. Nagyobb
futásból explicit rövid ablakot kell kérni a `ReplayWindow` API-val.
A bridge JSON a replay után hiba esetén is törlődik. Az authority MCAP path és
SHA256 szerepel a replay resultban, és a `verify-evidence` újraellenőrzi a fájlt.
A kért replay ERROR/NOT_RUN/MISMATCH nem PASS; hiányos evidence nem software divergence.

## Memória, idő és korlátok

A ring byte budget az encoded payload mellett az entry/reference overheadet
is számolja. A konfigurált mailbox kapacitás a typed objektumreferenciák száma,
nem byte budget. A 64 MiB ring-limit **nem** a folyamat RAM-limitje.
A `conservative_ram_budget()` külön számol a teljes retained typed gráfokkal,
encoder working gráffal, átmeneti JSON/bytes másolatokkal, writer chunk/index
másolatokkal és session bookkeepinggel. A hívónak a typed gráf és encoder
working gráf méretére felső korlátot kell megadnia; az encoded JSON mérete ezt
nem helyettesíti. A runtime, driver, interpreter és allocator RAM-ja külön tétel.

A session bookkeeping bounded: alapból legfeljebb 100000 observation kerül
feldolgozásra (`--capture-max-session-records`), mindkét módban. Ebbe a duplikált
raw snapshot publikációk is beleszámítanak. A korlát után tovább drainel, de
`SESSION_RECORD_LIMIT` miatt kötelező FAIL; a robotvezérlés változatlanul fut.
Hosszabb append_only futáshoz ezt a korlátot és a RAM-budgetet együtt kell méretezni.
Egy encoded record alaplimitje 16 MiB; egy tick legfeljebb 16 raw revisionre hivatkozhat.

A raw LiDAR limit alapból 4096 pont. Túllépés explicit truncation és incomplete
capture; nem jogosult MATCH-re. A LiDAR measurement ideje és a hub
`published_monotonic_ns` ideje külön marad: MCAP log time a domain-idő,
publish time az observation-idő. A reader uncompressed, indexelt MCAP-ot olvas;
rekordonként 64 MiB olvasási védőkorlátot használ.

A final event tartalmaz mailbox high-water markot, process CPU-t/max RSS-t,
writer add-message időt, tick periódus és tick-kezdettől publicationig mért
latency összesítést. A publication latency a checkpoint/observer előtti munkát
is tartalmazza; nem tisztán a TickEngine futási ideje. A process CPU/RSS mérés
az egész folyamaté, nem kizárólag a capture workeré. Az add-message idő nem
fsync-latency és nem önálló tartós storage benchmark.

## Validáció

```bash
python3 -m v3.import_guard
python3 -m pytest -q
# Kizárólag tesztkörnyezetben; az RPi runtime nem importálja az mcap csomagot:
python3 -m venv --system-site-packages /tmp/r2b4-interop-egyedi
/tmp/r2b4-interop-egyedi/bin/pip install -r requirements-interop.txt
/tmp/r2b4-interop-egyedi/bin/python -m pytest -q tests/test_v3_mcap_interop.py
# Véges álló mérés, saját üres STOP command-könyvtárral, canonical runtime-on:
python3 -m tools.v3_mcap_measurement --duration-s 10
```

A mérés új `/tmp/r2b4-mcap-measurement-*` könyvtárat hoz létre. Nem másolja és
nem módosítja a meglévő runtime adatokat. Fizikai mozgást, terhelt motorutat és
korábbi capture nélküli runtime-hoz viszonyított timing-változást nem bizonyít.
