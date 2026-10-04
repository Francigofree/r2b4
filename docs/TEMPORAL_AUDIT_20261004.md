# R2B4 Temporal Audit — 2026-10-04

Az eredeti audit eredménye **1 P0, 6 P1 és 5 P2 megállapítás**. A P0 a beszélgetési turn lejárata/lezárása után végrehajtható robotképes toolhívás volt. Hardvermentes reprodukció elérte a valódi default tool-regiszter `er2.delegate` határát a service lezárása után; fizikai végrehajtást nem indítottunk.

Az alábbi eredeti leltár, megállapítások és CSV az audit idején vizsgált forrást őrzik. A későbbi P0/P1 javítások állapotát a következő szakasz rögzíti; a történeti CSV nem a módosított source új leltára.

## P0/P1 javítások lezárása — 2026-10-04

A felhasználó kérésére a P0 és mind a hat P1 megállapítás javítása elkészült. A fejlesztés a P0/P1 scope validációjával lezárva; a P2 tételek külön feladatként maradnak. Robotikai authority-contract, L0–L12 layer-kód és aktív JSON-érték nem változott.

| Tétel | Javítás és owner | Regressziós bizonyíték |
|---|---|---|
| P0-01 | `ConversationService` véges turn deadline, queued/running cancellation timeout/close/STOP esetén; `AgentCore` ellenőrzés modell előtt/után és broker-dispatchnél; ROBOTICS delegate továbbadja a cancellationt és fennmaradó időt. A revoked eredmény nem publikálható érvényes javaslatként. | Default robot toolhatár nem érhető el timeout, close, STOP cancellation vagy turn deadline után. |
| P1-01 | A voice continuous-silence határ a meglévő VAD voiced PCM frame-jének `read_monotonic_ns` idejét követi. Az utterance elkészülése nem szükséges a beszédaktivitás figyelembevételéhez; a frame-read a fennmaradó csendablakhoz igazodik. | Határ előtt induló folyamatos beszéd 10 s-nál megtartja a sessiont; utána 10 s valódi csend lezárja. |
| P1-02 | A host shutdown 10 s-os runtime grace-éhez a sidecar saját konstansaiból származtatott, egymás utáni status/capture finish budget adódik. Capture esetén 150 s, nélküle 24 s a külső keret; a belső capture finish 120 s maradt. | Virtuális idővel a saját finish budgetén belül kilépő capture/runtime nem okoz hamis host shutdown-hibát. |
| P1-03 | ConfigResolver elutasítja a TTL-hez túl lassú reader pollt és a kritikus source/snapshot freshnesshez túl lassú multirate cadence-et. Explicit command heartbeat is a TTL-nél rövidebb kell legyen. | Az eredeti rossz reader/encoder variánsok elutasítva; production config változatlanul feloldható. |
| P1-04 | `resident_status.status_is_fresh` a közös host observation-age szabály. Ready, fresh-ready, IDLE, ALLOW és CLI ACTIVE preflight ezt használja, a meglévő élő runtime ellenőrzés mellett. | Régi RUNNING/ready/ALLOW status elutasítva; friss status elfogadva. |
| P1-05 | Teljes ER2 async session timeout connect/media/initial-send előtt; preview teljes request/tool-round budget és késői response eldobás. Media connect/read és a canonical véges navigáció előkészítés/lock/poll/indítás örökli a deadline/cancellationt. A provider failover host-várakozása ugyanazt a turn budgetet használja, clientenként legfeljebb egy későn befejeződő provider requesttel. | Connect, media és első send stall bounded; késői preview robottool eldobva; deadline/cancel az előkészítésben nem jut NAVIGATE-ig; lejárt turn nem indít provider retry/failovert vagy párhuzamos új provider requestet. |
| P1-06 | Resident TTS deadline az első ping előtt indul; nonblocking flock acquisition, ping connect/send/összes recv ugyanabból a startup keretből kap maradék időt. Synthesis request budget továbbra is külön. | Stalloló ping és foglalt file-lock is bounded startup-failure; meglévő TTS roundtrip sikeres. |

Javítási validáció: első CORE gate **26 passed**; provider scope **6 passed**; végső célzott host/config/voice/camera/TTS regresszió **17 passed**; `./r test full` **199 passed, 1 deselected**, 43,65 s. A teljes scope a szintetikus native replay-, process-, async-, perception-, motion-, voice- és provider-teszteket is tartalmazza. A 200-as repo tesztkvótán belül a kapcsolódó új boundary ellenőrzések két összetett host-szcenárióba kerültek, a cadence-negatív esetek a meglévő CORE invariáns-szcenárió részei. Whitespace/diff ellenőrzés is sikeres.

A releváns új ellenőrzések: [host temporal boundary szcenáriók](../tests/feature/test_temporal_host_boundaries.py), [voice continuous silence](../tests/feature/test_voice_orchestration_p0.py), [config cadence invariánsok](../tests/core/test_v3_temporal_config_invariants.py).

Live runtime vagy fizikai robotmozgás nem indult. Meglévő MCAP/capture feldolgozás és explicit Evidence Compiler nem futott. A replay szintetikus, a lassú I/O a regresszióban fake vagy lokális tesztsocket. A már elindult külső SDK/HTTP hívás nem kényszeríthető leállításra: a caller budgeten belül megszakad a várakozás, a késői eredmény nem indíthat robottoolt; a provider client megtartja a bounded request slotot az I/O befejezéséig. Fizikai művelet cancellationjekor a canonical STOP és a worker rendezett kilépése megvárandó, ezért a safety cleanup külön idő a munkavégzési deadline után. A natív Piper fallback compute továbbra is külön, saját hard compute-deadline nélküli út.

Hardveres timing/jitter, tényleges hálózati/SDK késések és capture-writer véglegesítési idő nem bizonyított. Nem maradt ismert, elbukó P0/P1 regresszió; a P2 és a live bizonyítás külön scope. A helyi runtime/capture adatokhoz a javítás nem nyúlt; commit/push parancsot nem futtattunk.

## Hatókör, bizonyítás és leltár

Vizsgált Git HEAD: `49f030f4fa9b2733c6f671742ce804133c327dd1`. A leltár a working tree forrását olvasta. A kezdéskor meglévő `runtime/` módosításokhoz és helyi capture/log adatokhoz az audit nem nyúlt.

Az összes tracked fájl útvonalának bejárása után **365 szöveges forrás/config/deploy/dokumentáció/teszt fájl** került vizsgálatra. **273 fájlban 9086 temporal előfordulás**, valamint **1507 AST-alapú temporal assignment/default** található. Python parse-hiba nem volt. A számok előfordulásokat és jelölteket jelentenek, nem 9086 független timert: ugyanaz a policy többször szerepelhet definícióban, használatban, kommentben és tesztben.

A [teljes, fájl/sor/symbol szintű CSV](TEMPORAL_AUDIT_20261004.csv) tartalmazza a `timeout`, `ttl`, `heartbeat`, `stale`, `freshness`, `deadline`, `sleep`, `poll`, `retry/retries`, `backoff`, `max_*age`, `*_gap` találatokat, továbbá az egységjelölt időértékeket, cadence/period/interval/budget/delay/horizon/window/hold/skew/grace/cooldown/silence/watchdog mezőket, órákat, frame/tick alapú időzítéseket, `wait/join/select/settimeout/communicate` hívásokat és systemd lifecycle értékeket. A CSV mezői: `path`, `line`, `scope`, `owner_family`, `symbol`, `kind`, `temporal_terms`, `source`, `ast_detail`. Az AST-rész a többsoros hívás/default teljes kifejezését is rögzíti. A `source` forrásszöveg, nem runtime adat.

A szótalálatok között lehetnek térbeli vagy más nem időbeli mezők is; például `dtheta` és `retention_confidence`. Ezek nem új időzítési authorityk. Az aktív paraméterek alábbi, értelmezett táblájából a bizonyított ilyen találatokat kivettük. A per-file függelék a nulla találatos fájlokat is felsorolja.

Nem olvastuk be a `runtime/**` helyi adatokat, `models/**` bináris modelleket, rejtett `conf/.*` credential/host cache-eket, a történeti `APPLY_RESULT.json` és `TEST_HUB_FINALIZATION_RESULT.json` diagnosztikai eredményeket, illetve bináris fájlokat. Ezek nem production timer-definíciók. Az audit nem futtatott diagnosztikai/live eszközt csak azért, mert timer szerepel benne. Az `old/` tesztek leltárba kerültek, de nem bizonyítják a live wiringot.

Authority: [felhasználói működés](../R2B4_SYSTEM_BEHAVIOR_CONTRACT.md), [L0–L12 struktúra](../STRUKTURALIS_RETEGEK_V3.md), [async/process runtime](../ASZINKRON_RUNTIME_CONTRACT_V3.md). A [meglévő temporal ownership dokumentum](TEMPORAL_OWNERSHIP.md) kiegészítő szemantikai referencia. A konkrét futási helyet a source döntötte el.

A production út: `r` → launcher/interface/operator → `v3.operator_cli __runtime-session` → `v3_process_runtime.py` → `v3_hardware_runtime.py` → `v3_runtime.py` → composition/resident/native control → TickEngine L1–L12. A deployed voice service `r2b4_voice.voice_service`; a külön indítható `wake_service` nem ennek authorityja.

Bizonyítási szintek:

- **R**: valódi repo-modullal, hardver és hálózat nélkül reprodukált viselkedés; külső mellékhatás stubolt.
- **S**: a production hívási láncban forrásból igazolt feltétel vagy hiány. A tényleges késést/előfordulást nem mértük.
- **T**: meglévő célzott teszt sikeres. Ez nem általános runtime vagy jitter-garancia.

Prioritás: **P0** az elavult felhasználói szándékból később induló robotképes mellékhatás; **P1** működési contract sérülése, hamis readiness vagy hiányos lifecycle/időkeret; **P2** policy-egyértelműség, konfigurálhatóság és megfigyelhetőség javítása. Ezek auditprioritások, nem mért üzemi incidensgyakoriságok.

## P0/P1/P2 lista

### P0-01 — Lejárt vagy lezárt turn továbbra is végrehajthat toolt

**R + S.** `r2b4_voice/conversation_service.py:174–203` `wait_for_turn` timeout esetén csak a várakozót engedi el; `close` lezárja a service-t és bounded join-t végez, de a futó `_process`/Agent turn nem kap cancellationt. Az `AgentCore.run` toolköreinek darabszámkorlátja nem időkeret és nem revocation: `r2b4_orchestration/agent_core.py:126–141`. A production voice wiring valódi brokerrel/default tool-regiszterrel épül: `conversation_interface.py:97–141`.

Az `agent_tools.py:21–54` default `er2.delegate` módja `stream`, alapból engedélyezett toolokkal és 20 s durationnel; az executor a canonical robotút felé lép. A canonical command TTL a később kiadott új parancsot ellenőrzi, az eredeti host turn érvényességét nem állítja helyre.

Reprodukció: egy futó modellválasz blokkolása → a caller 0,01 s várakozása lejár → `service.close(0,01)` → modell felszabadítása. A valódi `ConversationService` + `AgentCore` + `AgentToolBroker` + default regiszter út után az események: `CLOSED`, majd `ER2_TOOL_AFTER_CLOSE`. Csak a külső ER2 végrehajtóhatár volt stubolt; hálózat és motor nem futott.

**Hatás:** lezárt sessionből vagy lejárt callerből új robotképes kérés indulhat. A voice STOP interrupt a visszatérő javaslatokat szűri, de az Agent belső toolhívásának nincs turn-lifetime ellenőrzése. A STOP-verseny és fizikai mozgás nincs reprodukálva.

**Javítás owner/h határa:** T22 host turn lifecycle. Véges deadline és cancellation/generation továbbítása; ellenőrzés minden mellékhatásos tool/delegate előtt, és továbbadás a véges végrehajtó adapterbe. A caller timeout/close érvénytelenítse az érintett turnt. Ehhez nem kell új L-réteg vagy általános IPC/scheduler.

### P1-01 — A voice session nem 10 s folyamatos csend után zár

**R + S.** A rendszercontract 10 s folyamatos csendet ír elő. `voice_service.py:380–404` a session-expiryt az új frame beolvasása előtt ellenőrzi; `:995–1019` a timer resume pillanatához köti a 10 s-ot. A felismert, de még nem lezárt beszéd nem újítja a silence-határt. A VAD a `wake_core.py:190–236` útban már aktív utterance-et gyűjt.

Reprodukció: valódi 20 ms-os voiced PCM frame-ek 9,90 és 9,92 s-nál aktív utterance-et indítottak; 10,00 s-nál a session bezárt, és az új voiced frame olvasatlan maradt. Eredmény: `utterance_active_before_boundary=true`, `session_open_at_boundary=false`, `voiced_frame_left_unread=1`; robotparancs nem keletkezett.

**Hatás:** a határ előtt induló beszéd elvágható. **Owner:** T23. A meglévő VAD beszédaktivitása alapján kell folyamatos csendet számolni; a feldolgozás/TTS alatti timer-pause külön életciklus maradjon.

### P1-02 — A host shutdown 10 s, a capture finalizálás 120 s

**S.** `v3/operator_controller.py:397–434` SIGTERM után 10 s-ig vár a runtime kilépésére. `v3/process_sidecars.py:39–41,456–475` a capture finish számára 120 s process join-t enged, előtte 2 s enqueue és utána 2 s result-wait is lehet. A `v3_process_runtime.py:492–508` `finally` szinkronban finalizál a runtime kilépése előtt.

**Hatás:** a saját alsó időkeretén belül működő finalizálásra a host téves shutdown-hibát jelenthet. Adatvesztést vagy capture-sérülést nem reprodukáltunk. A triggered tail várakozás a SIGTERM előtti külön fázis, nem növeli a későbbi 10 s-os keretet.

**Owner:** T20 külső shutdown, T21 capture finish. Nevezett, összehangolt teljes shutdown budget szükséges, a bounded alfolyamatokkal összhangban. A 120 s önkényes csökkentése mérési bizonyíték nélkül nem megoldás.

### P1-03 — A cadence–freshness invariánsok ellenőrzése hiányos

**R + S.** `v3/config.py:112–138,196–201` több fontos temporal invariánst már validál. Kimarad a command reader poll–TTL viszony és a multirate per-source period–érintett freshness viszony. A `ConfigResolver.from_documents` elfogadta az 1 s reader pollt 250 ms maximum TTL mellett, valamint az 1 s encoder multirate periodot 250 ms snapshot/admission freshness mellett. A production aktív értékei jelenleg 5 ms, illetve 20 ms; ezek a konkrét rossz variánsok nincsenek bekapcsolva.

**Hatás:** konfigurációval előidézhető rendszeres parancslejárás/kimaradás, elavult input és elérhetetlen vagy váltakozó readiness. Unsafe ALLOW-t nem bizonyítottunk; a downstream gate-ek jellemzően fail-closed működnek.

**Owner:** T01 feloldott config, a producer/poll és source closure saját invariánsaival. A kritikus source periodokat a ténylegesen rájuk alkalmazott budgethez kell kötni. A CLI explicit heartbeat override-ját is ugyanazon TTL mellett kell validálni. Nem indokolt minden freshness limitet egyenlővé tenni: például a person 500 ms result-age és az L2 250 ms admission külön policy.

### P1-04 — A host readiness/ALLOW útjai eltérően kezelik a status korát

**R + S.** `operator_controller.py:161–179` `live_runtime_status` és `:1293–1312` `wait_idle` megköveteli a `0 <= age < 500 ms` feltételt. `:1314–1345` `_wait_ready` és `_wait_fresh_ready` nem ellenőrzi ezt; utóbbi a baseline-tól eltérő tartalmat tekinti frissnek. `:1052–1098` `_wait_allow` nem-navigation útja és `v3/control_cli.py:91–94` preflight szintén nyers statusból dolgozik.

Reprodukció: fake élő PID + `RUNNING/ready_for_active=true`, `monotonic_ns=1` régi status; `_wait_ready` azonnal elfogadta. A fake csak a process/status külső határt váltotta ki.

**Hatás:** a host régi állapotot jelenthet friss readinessként/ALLOW-ként. Az L12 önálló runtime safety-gate-jét ez a próba nem kerülte meg. **Owner:** T20/T03. Azonos friss status-helper használata, és a státusz runtime-identitásának ellenőrzése, ahol rendelkezésre áll; host ne gyártson új safety authorityt.

### P1-05 — A host/ER2 külső időkeret nem fedi a teljes műveletet

**S.** `r2b4_er2/streaming.py:75,112–175` létrehozza a duration deadline-t, de a timer task csak a connect, kezdeti media observation és első send után indul. E fázisokat nem védi ugyanaz a duration. A preview `interactions.create` útjának nincs repo-owned teljes deadline-ja; a tool-round korlát 8 darab, nem időkorlát (`preview.py`). Az SDK tényleges default network timeoutját nem állapítottuk meg.

A `v3/finite_navigation.py:95–119` kiszámolja a teljes véges akció deadline-ját, de a `finite_motion_transaction` runtime-start/preemption/IDLE előkészítéséhez nem adja tovább a fennmaradó időt vagy cancellationt. A deadline a későbbi pose/végrehajtási fázist védi. A command producer watchdog saját indulásától védi a mozgási sessiont; nem az előkészítés indulásától.

Az LLM provider 30/20/20 s timeoutjai, provider retry és failover, valamint több Agent tool-round összeadódhat; az outer 90 s caller-wait nem korlátozza őket és nem törli az eredményt (P0-01). Socket timeoutból nem számítható kemény teljes streaming felső korlát.

**Hatás:** egy névleg véges host-művelet az ígért kereten túl blokkolhat, későn fejezhet be. **Owner:** T22/T25 és T20 véges adapter. Egy monotonic abszolút deadline fennmaradó idejét kell továbbadni a saját blocking fázisoknak; explicit `None` nyitott session továbbra is külön, szándékos mód lehet. Physical action watchdog és host turn deadline külön owner marad.

### P1-06 — A TTS 20 s startup timeout nem teljes startup budget

**S.** `r2b4_voice/resident_tts.py:28,174–224`: connect 1 s, request 90 s, startup 20 s. Az első ping, blokkoló `flock(LOCK_EX)` és a második ping a startup deadline létrehozása előtt fut. A startup loop pingje is a 90 s request timeoutot használja, majd újra ellenőrzi a 20 s-os határt.

**Hatás:** a startup névleges 20 s-ja nem korlátozza a teljes műveletet; a locknak nincs repo-owned időkorlátja. Ugyanez a per-stage/total különbség az OAuth refresh locknál is megjelenik, de annak hálózati és credential-élettartama külön tulajdonos. Server stallt vagy lock contentiont nem mértünk.

**Owner:** T24. A startup kezdete előtt felvett deadline, annak maradékával bounded lock/ping/start várakozás; synthesis request budget külön megőrzendő. A lokális in-process Piper fallback számítására jelenleg nincs teljes repo-owned compute timeout.

### P2-01 — Az equality szabály nem egységesen nevezett

**R + S.** `contracts/temporal.py:46–60`: freshnessnél `age > max_age` stale, deadline-nál `now >= deadline` expired. `adapters/resident_command.py:294` viszont csak `observed > expires` esetén járatja le a commandot; `contracts/async_runtime.py:41–44` compute-deadline-ra freshness-semantikát használ. Reprodukció: ugyanazon 200-as határnál `deadline_reached=true`, a gateway még `TELEOP`, és a compute `deadline_missed=false`.

A `MotionValidity` és person prediction validity dokumentáltan inclusive (`contracts/messages.py:314–317,716–746`). Ezeket nem szabad automatikusan megváltoztatni. **Owner:** T00/T03/T09. Explicit döntés és boundary teszt szükséges arról, mely mező freshness/inclusive validity és melyik véges deadline/expiry. Egyetlen equality tick eltérését bizonyítottuk, tartós safety-sértést nem.

### P2-02 — Az alternatív/default temporal knobok nem mind aktív authorityk

**S.** `config.py:235–250` már jelöli: completion-input production módban `release_tick_gap=5`, `release_delay=100 ms` és `trajectory_replan_min_tick_gap=5` inaktív; `follow_person_search_step=400 ms` compatibility-only. A leltárban ezeket a production deadline-tól külön kell olvasni.

További példák: `MultiRateInputConfig` person default 80 ms, aktív JSON 60 ms; vision capability default detection age 400 ms, production person result policy 500 ms. A magasabb constructor default nem írja felül a strict resolved configot. Person 500 ms result-age és általános 250 ms L2 admission esetén az elfogadásban a szigorúbb alkalmazott gate dominál; camera completion-lag 500 ms és measurement age 250 ms eltérő időeredetű policy.

**Owner:** T01 és érintett capability. Effective SSOT/default/inactive megjelölés a diagnosztikában; ne duplikált policyként vagy automatikus equality-invariánsként kezeljük. A `TEMPORAL_OWNERSHIP.md` mátrixból hiányzó explicit L7 objective-lifetime sort a magasabb contract és a source alapján pótolni érdemes.

### P2-03 — Egyes host timer-konfigurációk elfogadják a NaN-t

**R + S.** `VoiceServiceConfig(session_silence_s=nan)` és `OpenAIChatConfig(timeout_s=nan)` sikeresen létrejött: a pozitivitás-ellenőrzés nem `isfinite`-ellenőrzés. Előbbinél később `int(nan)` hibára vezethet, utóbbinál a transportnak adható tovább. A robot JSON ConfigResolver szigorúan elutasítja a nonfinite értékeket; ez host adapter boundary probléma.

**Owner:** T23/T24 és egyéb host CLI/config adapterek. Véges, érvényes tartomány ellenőrzése a belépési ponton. A dokumentált optional `None` és a szándékos nyitott session nem NaN-helyettesítő.

### P2-04 — A capture 50 Hz jelentése nincs a konfigurálható tickhez kötve

**R + S.** `capture_rate.py:5` `CONTROL_CAPTURE_HZ=50`; `mcap_capture.py:80–100,924` 50 Hz kérésnél minden tick sensor-debug capture. ConfigResolver 10 ms tick periodot is elfogadott, így a név szerinti 50 Hz út valójában 100 tick/s lehet. Az aktív config 20 ms, itt nincs eltérés.

A ring `max_tick_count=768` és `pre_event=8 s`, checkpoint context 1 s külön hard bound. 100 Hz-nél már a 8 s-os pre-window is 800 tick; a `:717–723` darabszámcap felülírhatja a kívánt időablakot. Ez bounded memóriahasználat, nem bizonyított fájlsérülés. A performance `over25/over40 ms` metrikák is abszolút diagnosztikai küszöbök, nem tick-derived safety-budgetek.

**Owner:** T21/T01. A teljes-tick capture mód jelentése és a szükséges ring-capacity legyen explicit a tick period mellett; a metadata/ablak ellenőrzésére konfigurációfüggő teszt kell.

### P2-05 — Per-stage retry/recovery budget és cooldown-pillanat

**S.** A planner recovery 3 attempt, 50/100 ms backoff, 1,25 s ready timeout. `adapters/l6_planner_process.py:146–163` ugyanazt a ready értéket külön ready-get, warmup-put/get és collector-ready várakozáshoz használja: ez nem egyetlen 1,25 s teljes recovery budget. A stop/terminate/collector join is több bounded fázis. LiDAR ready/matcher startup és több sidecar close hasonlóan per-stage értékeket tartalmaz. Ezt a teljes lifecycle latencia számításánál össze kell adni; nem nevezhető automatikusan control-jitter hibának, mert az edge/worker végzi.

`r2b4_voice/llm_failover.py:171–185` egyszer veszi fel a `now` pillanatot a provider loop előtt. Egy lassú első provider után az időközben lejárt fallback cooldown még aktívnak látszhat; ez forrásból igazolt döntési feltétel, külön reprodukció nem futott.

**Owner:** T16/T08 lifecycle és T24 provider policy. A per-stage értékek dokumentálása, szükség esetén konkrét adapter-local teljes budget; cooldown ellenőrzésnél aktuális monotonic idő. Nincs szükség új általános retry frameworkre.

## SSOT/ownership mátrix

Az owner itt felelős source-komponenst jelent, nem személyt. T01 a numerikus production paraméterek központi feloldója; a policy szemantikai tulajdonosa az alkalmazó capability/réteg. Azonos számérték nem közös ownership. T00 közös matematikai/temporal szemantika, policy nélkül. A CSV `owner_family` fájlszintű routing-csoport, nem minden hivatkozott mező runtime authorityja: például T01 config sorának fogyasztója a lent jelölt réteg.

| ID | Idő/policy, production SSOT | Tulajdonos / fogyasztó | Óra, érvényesítés és lifecycle | Állapot |
|---|---|---|---|---|
| T00 | `contracts/temporal.py`, typed timing/validity contracts; felső V3 contractok | Policy-free szemantika, minden fogyasztó | Injected monotonic ns; age/skew/deadline/continuity; timestampok nem budgetek | Equality pontosítás P2-01 |
| T01 | `conf/hardver.json`, `fizika.json`, `speed_map.json`, `vezerles.json` → `v3/config.py` immutable snapshot | ConfigResolver; minden robot policy a saját rétegénél | Hardware open előtt strict schema, finite érték, több cross-invariant; capture/replay snapshot | Cadence gap P1-03; defaults P2-02 |
| T02 | Tick period 20 ms; TickContext/runtime scheduling; bounded runtime alternatív config | `v3_runtime.py`, TickEngine/composition; launcher csak indít | Monotonic deadline; missed slot skip, nincs catchup storm; rétegek immutable tick inputot kapnak | Source szerint helyes elhelyezés; jitter nem mért |
| T03 | Ingress TTL 250 ms, future skew 5 ms, reader 5 ms; producer heartbeat default 100 ms, legfeljebb default TTL/2 | Command producer liveness; `adapters/resident_command.py` gateway független expiry-validation | Monotonic observed ingress; external owner PID/max-runtime; expiry után STOP; closed tick/replay parancs immutable | P0 host-turn külön rés; P1-03/04, P2-01 |
| T04 | L1/L2 source readiness/admission 250 ms, future skew 10 ms | L1 validity, L2 admission | Physical measurement age; immutable closed input, stale kritikus source fail-closed | Az async receipt nem friss mérés |
| T05 | Encoder process 20 ms, sample interval 100 ms, estimate window 40–160 ms; GPIO debounce/guard/confirm | Encoder producer/process/typed source; L3/L11 fogyasztó | Device event/measurement monotonic; bounded scalar mailbox; caller stale/crash gate | T async stall/crash; P1-03 cadence |
| T06 | IMU process 20 ms, max measurement age 100 ms, startup 1 s / poll 50 ms | IMU edge/backend/source | I/O process; control nonblocking shared read/cached age; ready 5 s, stop 2 s | Process snapshot és freshness külön |
| T07 | LiDAR read 100 ms, stale 500 ms, startup grace 10 s, reconnect 400 ms, settle 500 ms, stop 2 s | `adapters/rplidar_c1.py` device lifecycle | Monotonic read/no-data/reconnect; serial I/O nem control-island | Driver stale nem L12 clearance age |
| T08 | Matcher ready 8 s/stop 1 s; poll 120 Hz; source/result age 250 ms; matcher compute 55/120 ms; process heartbeat 100 ms/ready 12 s | LiDAR/matcher/localization edge és worker | Measurement, completion, state heartbeat külön; process lock/poll/restart, lineage; pose history collector út | Ready per-stage P2-05; T process/replay |
| T09 | Multirate 20/50 ms, source-specific cadence, publication snapshot age 250 ms, close join 2 s; async completion closure | `multirate_inputs.py`, capability typed edge/closure | Publish visibility <= tick; source measurement age később; global join budget; completion timing nem receipt-age | P1-03; control immutable TickInputs |
| T10 | Camera active 16 fps, completion lag 500 ms, owner grace default 1 s; image freshness 250 ms; media/socket lifecycle | Vision owner/process/camera/media adapter | Demand ON/OFF; producer measurement time; connect/request/close külön; large image közvetlen media út | Constructor default nem config SSOT |
| T11 | Person result max age 500 ms, async cadence 60 ms, photo minimum interval 5 s | Person detector/typed source, photo evidence worker | Detection frame lineage és measured age; photo throttle observation-only | L2 effektív age szigorúbb lehet; P2-02 |
| T12 | L3 dt <=250 ms; stationary hold 50 ms; absolute fix 2 s, relative 1,5 s, consistency memory 30 s | L3 estimation és quality | Tick continuity / measurement time; nincs saját sleep/device clock | Saját estimation policy |
| T13 | L4 track/costmap ages 500/750 ms; scan 1 s, pose 2 s; prediction 350 ms; structural 30 s/confirm 1 s | L4 world model/history/tracking/occupancy | Measurement time, cross-sensor skew; bounded history; future prediction validity | Prediction inclusive validity explicit |
| T14 | L5 mission lifecycle; nincs önálló explicit temporal konstans/találat a layer fájlban | L5 mission identity/state | Command és tick validityből örököl; nem új timer authority | Nulla találat nem hiányzó mission lifecycle |
| T15 | L6 world/costmap 250 ms; replan 100 ms; plan 350 ms; local goal 8 s; rollout 1 s; FOLLOW hold/search/recovery | L6 döntési policy | Tick monotonic; source/context-scoped plan acceptance; finite search windows | Inactive tick-gap P2-02 |
| T16 | Async request 300 ms, transport watchdog 2 s; recovery 3 attempt / 50 ms exponential backoff / stage ready 1,25 s | Planner submit/worker/collector + recovering adapter | Actual worker completion vs submit; transport az eredeti running submitból; 1 running + 1 pending, generation reject | Nem receipt alapú deadline; P2-01/05 |
| T17 | L7 original objective validity; L8 realization horizon 100 ms/control gap 250 ms/world age 250 ms | L7 retained objective, L8 realization | L7 mission/frame/scope/continuity invalidation; pending nem hosszabbít; L8 saját current validation | L7 ownership doc hiány P2-02 |
| T18 | L9 finite velocity-transition budget; L10 mapping; L11 control gap/feedback uncertainty/age 250 ms | L9/L10/L11 actuator intent/control | L9 derived `ceil(2*minimum_speed/(max_acceleration*degraded_scale)*1e9)`; L11 nem önhosszabbító | Mérnöki/dinamikai duration, nem fix TTL |
| T19 | L12 aktuális LiDAR age gate; motor PWM 8 kHz; DRV8871 sleep hold 2 ms | L12 kizárólagos motor authority, GPIO motor edge | Current measurement age és health; hardware LOW hold a STOP/FAULT/close útban; 2 ms fizikai device-sequence | Nem általános scheduler sleep; OS késés nem mért |
| T20 | Operator start/ready/idle/allow/shutdown; finite session watchdog default 30 s, pose max 8 s | OperatorController/FiniteNavigationExecutor/interface | Host monotonic polling; ownership/preemption/STOP canonical gateway; teljes akció és előkészítés külön | P1-02/04/05 |
| T21 | Capture 1/5/10/50 Hz; pre/post 8/2 s; checkpoint 1 s; sidecar/status lifecycle; Replayer | Observation/capture/status producer/sidecar és offline replay | Raw evidence nem nagy payloadként controlba; replay closed TickInputs; writer clock nem decision authority | P1-02, P2-04 |
| T22 | Conversation waiter 20/25/90 s, Agent round count, service close 2 s fázisonként | ConversationService + AgentCore/broker host turn | Waiter timeout jelenleg nem revocation; nincs közös finite turn/cancel authority a mellékhatásig | P0-01, P1-05 |
| T23 | Voice continuous silence 10 s, VAD frame-countok, microphone retry/read/close; deployed voice service | VoiceService/VAD/microphone; STOP interrupt külön lane | Monotonic PCM activity/session lifecycle; 20 ms frame; TTS/processing alatt pause | P1-01; P2-03 |
| T24 | Provider request timeout/retry/cooldown; OAuth expiry/refresh; TTS/socket/audio lifecycle | Provider adapter, FailoverLLMClient, OAuth credential owner, resident TTS/player | Request/cooldown monotonic; JWT/OAuth absolute expiry wall UTC; lock/request/compute külön | P1-06, P2-03/05; nincs teljes turn bound |
| T25 | ER2 heartbeat default 1 s; segment 1,5 s; watchdog 10 s; media 15 s; finite duration vagy explicit nyitott session | Streaming/preview/tool bridge | Model-turn heartbeat nem command heartbeat; producer TTL külön; SDK connect/receive és host deadline külön | P1-05; nincs SDK-default bizonyíték |
| T26 | `deploy/systemd/r2b4-wake.service`: RestartSec 2 s, TimeoutStopSec 5 s | systemd host lifecycle | SIGTERM után külső kill keret; nem orderly provider/TTS completion guarantee | 5 s kisebb a blocking host calloknál; P0 cancellation szükséges |
| T27 | `tools/**`, root `*_diag.py`/`*_probe.py`, `gitre.py` saját timeoutjai/ablakai | Explicit indított diagnosztika/offline compiler | Nincs production L0–L12 authority; saját invocation/resource lifetime | Csak statikus leltár; nem futtattuk automatikusan |
| T28 | `tests/**`, `old/**`, pytest/test runner test-duration/poll/retry értékei | Teszt harness, archived külön scope | Synthetic/fake clocks és process harness; nem live defaults | Aktív célzott tesztek lent; old nincs futtatva |

### Fontos authority- és időeredet-különbségek

- **Physical measurement → computation completion → collector receipt → tick closure** négy külön pillanat. Friss receipt nem teszi frissé a mérést, késői closure pedig nem teszi utólag deadline-missszé az időben befejezett compute-ot.
- **TTL producer ownership:** a producer újítja a parancsot/revisiont; gateway függetlenül lejárat. A gateway ingresskor saját observed monotonic órát használhat, mert a tick timestamp régebbi lehet. A lezárt tickből replaykor nem olvassa újra a mailboxot.
- **Planner transport watchdog:** supersede/pending request nem nullázza a futó worker eredeti watchdogját; a generation/request/context lineage érvényesítés a collector/closure határnál marad.
- **L7 objektívum lifetime:** az eredeti deadline és mission/frame/scope identitás authority. Pending L6 munka nem renewal; L6/L8 current revalidation külön gate.
- **GPIO `sleep(0.002)`:** motor hardware LOW hold. STOP/FAULT tickben tényleges control-budgetet használhat; a nominal 2 ms és a Python/OS oversleep eltérő. Ezt nem szabad puszta statikus `sleep` találat alapján sensor process-isolation hibának minősíteni.
- **Pose history lock:** a production control `ProcessLidarPort.publish_pose_reference` bounded deque-ba tesz; a 50 ms shared pose-lock a collectorban van, a blocking lookup a childban. Nem igazolódott 50 ms-os control lock.
- **UTC vs monotonic:** OAuth/JWT abszolút credential expiryhez wall clock indokolt; duration/deadline/cadence inputhoz monotonic. Capture timestamps és performance mérőórák nem új robotikai döntési authorityk.

## Kódbeli és host/deploy időzítések

Az alábbi táblázat az aktív JSON-on kívüli konstansokat/defaultokat és származtatott/lifecycle időzítéseket foglalja össze. Paraméterezhető constructor/CLI/env default esetén a tényleges invocation felülírhatja az értéket; a gép privát service-envjét nem olvastuk be. Minden további használat, dinamikus változó és tesztérték a CSV-ben szerepel.

| Terület / forrás | Értékek és időeredet | Jelentés / határ |
|---|---|---|
| `v3_runtime.py`, `v3/engine.py` | Config tick 20 ms; `perf_counter/thread_time` futásidőmérés; missed-slot skip | Control cadence és mérés külön; a sleep a runtime edge-en, nem L-layerben |
| `v3/control_cli.py` | Heartbeat default 100 ms, defaultnál `min(100 ms, TTL/2)`; explicit CLI override; owner exit/max-runtime | Command producer liveness; STOP nem újít active akciót |
| `v3/operator_controller.py` | PID startup 40×50 ms; 200 ms settle; korábbi runtime exit 5 s; új fresh-ready 20 s; ready default 8 s; idle 3 s; ALLOW 50×50 ms; shutdown 10 s; capture-tail 70×50 ms | Több egymás utáni fázis, nem egyetlen teljes 20 s-os start-budget; P1-02/04 |
| `v3/finite_navigation.py` | Watchdog default 30 s, tartomány 1–600 s; finite deadline `min(watchdog−0,5 s, explicit timeout)` az akció kezdetétől; pose max 8 s; poll 50 ms | Preparation nem kapja meg a maradék időt; P1-05 |
| `v3/adapters/l6_planner_process.py` | Ready/warmup/collector wait külön 1,25 s aktív; stop-enqueue 200 ms; stop/terminate/collector join 1 s fázisonként | Edge lifecycle, count-bounded recovery; P2-05 |
| `v3/adapters/recovering_l6_planner.py` | 3 attempt, `50 ms * 2**(attempt−1)` backoff | Recovery nem új L6 authority és nem újítja az eredeti requestet |
| `v3/adapters/process_lidar_port.py`, `v3/lidar_matcher_process.py` | Parent/child collector poll 5 ms; matcher queue poll/full-put 50 ms; raw-end 500 ms aktív | Source time/sequence megőrzendő; collector physical I/O-tól külön |
| `v3/adapters/rplidar_c1.py` | No-byte idle wait 1 ms; scan driver poll 1/120 s; driver startup/reconnect/read a JSON-ból | Poll cadence nem measurement freshness |
| `v3/scan_matching.py`, LiDAR estimator | Matcher 55/120 ms budget, elapsed/perf mérés; reacquire 3 konzekutív scan; cooldown 1/2 s | Compute/adaptív munka-budget nem garantált hard process preemption |
| `v3/adapters/camera_ownership.py`, `vision_owner.py` | Last-demand grace default 1 s; idle poll 50 ms; close join 1 s; supervisor wait 500 ms | Demand-driven camera lifecycle; last demand hiányában OFF |
| `v3/adapters/vision_media_socket.py`, `process_vision_port.py` | Media/client startup default 15 s; retry poll 25 ms; ping 500 ms; accept 200 ms; writes 5 s; proxy wake 100 ms/close join 200 ms | Steady connection `settimeout(None)` tudatos nyitott demand session; producer image út a control mellett |
| Camera/photo/image worker adapters | Frame wait 1 s; camera close 2 s; detector poll 200 ms / stop 2 s / wait 1 s; photo lock wait és request külön 5 s; photo poll 10 ms; async-photo poll 50 ms/close 500 ms | Photo 5 s throttle observation policy; image/detection measurement age külön |
| `v3/adapters/picamera2_camera.py` | Constructor 20 fps; production hardware config 16 fps | Default nem live SSOT |
| `v3/adapters/camera_media.py` | Default video 5 fps; request 5 s, tartomány 0,1–15 s; ffmpeg finish 15 s, cleanup 5 s | Host media subprocess életciklus; nem robot motion időkeret |
| `v3/adapters/gpio_motor.py` | `_DRV8871_SLEEP_HOLD_S=0.002`; PWM 8000 Hz config | Motor hardware sequence; nominal idő nem OS hard deadline |
| `v3/capture_rate.py`, `v3/mcap_capture.py`, `v3/execution.py` | Capture default 10 Hz; választék 1/5/10/50 Hz; full-tick/sensor-debug címke 50 Hz; pre 8 s/post 2 s; max 768 tick; checkpoint 1 s | Passive evidence ablak és hard memory cap; P2-04 |
| `v3/process_sidecars.py` | Sidecar ready 10 s; capture finish 120 s; raw-end 2 s; data poll 10 ms; finish queue/result 2 s; terminate join 2 s | Per-stage lifecycle, writer nem control; P1-02 |
| `v3/runtime_performance.py` | Histogram 100 µs bucket, 100 ms cap; 25/40 ms overrun counters | Diagnosztikai fix küszöb, nem új decision deadline |
| HRI evidence observer | Poll 100 ms; max watch 45 s; lookback 60 s | Passive observation, nem command ingress |
| `conversation_service.py`, `conversation_cli.py`, Agent callers | Default wait 20 s; conversation CLI 25 s; Agent/voice wait 90 s; close sentinel enqueue 2 s és join 2 s | Waiter és worker lifetime nem azonos; P0-01 |
| `voice_service.py` | Session silence 10 s; mic retry 2 s; frame wait 1 s; output settle 450 ms; interrupt lane 20/50 ms wait; interrupt thread join 1 s | Deployed voice policy; continuous activity hibája P1-01 |
| `wake_core.py`, `adapters/live_microphone.py` | PCM 48 kHz; frame 20 ms; VAD start 2/end-silence 18/pre-roll 10/max utterance 250/min voiced 4 frame = 40/360/200/5000/80 ms; ring 50 frame = 1 s; process stop 2 s fázisonként | Frame-count derived timer; optional `read_after(None)` tudatos API, production caller bounded |
| Külön `wake_service.py` | Mic retry 2 s, runtime poll 500 ms, failure cooldown 1,5 s, frame wait 1 s, runtime idle 3 s | Külön CLI/compat út, nem a deployed voice loginservice defaultja |
| OpenAI/Gemini/Groq LLM, Groq STT, Gemini TTS | LLM request defaults 30/20/20 s; STT 12 s; Gemini TTS 25 s | Transport call budget, nem teljes Agent-turn határ; env/config felülírható |
| `llm_failover.py` | Max transient attempt 2; retry delay 200 ms; cooldown default unknown 15 s, response/transient/provider 20 s, usage-unavailable 60 s, auth/quota 300 s | Provider request retry; már végrehajtott robottoolt nem ismétel meg |
| `openai_oauth.py` | Early refresh 120 s; network 30 s; login default 300 s/server poll 500 ms; JWT expiry wall UTC | Credential SSOT, nem robot freshness; file-lock külön blocking lifetime |
| `resident_tts.py`, `tts_daemon.py`, `piper_tts.py` | Client connect 1 s/request 90 s/startup 20 s/poll 50 ms; daemon accept 1 s/client 90 s; flock blocking; native fallback compute saját deadline nélkül | Startup/request/compute külön; P1-06 |
| Speaker/voice output | Playback wait `max(10 s, audio duration + 8 s)`; ready asset subprocess 10 s | Audio lifecycle; duration mint számított időkeret |
| `r2b4_er2/config.py`, `streaming.py`, `preview.py` | Heartbeat 1 s (0,9–10); segment 1,5 s (0,1–5); watchdog 10 s, legalább segment+1 s, max 600 s; media 15 s (0,25–30); reconnect budget 5; error backoff `min(2 s,0,25 s*count)`; reconnect 100 ms; preview max 8 tool round | Heartbeat model-turn cadence, nem hard maximum silence/network-health guarantee; P1-05 |
| `r2b4_orchestration/agent_tools.py` | `er2.delegate` stream default duration 20 s, engedett 1–30 s | Delegate finite session; nincs caller-turn cancellation kapcsolat |
| `deploy/systemd/r2b4-wake.service` | Restart on failure, RestartSec 2 s; SIGTERM; TimeoutStopSec 5 s | Külső process lifetime; önmagában nem törli a processben futó turnt a SIGTERM előtt |
| `tools/**`, root diag/probe és tesztek | Minden lexical/AST timeout/poll/window/default a CSV-ben saját fájl/symbol/scope szerint | Invocation-local/offline/teszt policy; nincs production override |

## Az eredeti audit validációja és bizonyítási korlátai

A canonical launcherrel futott ellenőrzések:

| Parancs | Eredmény | Relevancia |
|---|---|---|
| `./r test` | 26 passed | CORE temporal/config/command/motor és egyéb canonical gate-ek |
| `./r test voice` | 7 passed | Voice/Agent/host orchestration meglévő tesztek |
| `./r test providers` | 6 passed | Provider/failover/OAuth transport tesztek |
| `./r test async` | 8 passed | Multirate/async és peripheral stalled/crash isolation |
| `./r test tests/deep/test_v3_localization_process.py tests/deep/test_v3_process_affinity.py` | 3 passed | Process localization/affinity boundary |
| `./r test replay` | 8 passed, 1 deselected | Native szintetikus Replayer/checkpoint regresszió; endurance alapból deselected |

Összesen **58 sikeres tesztfutás**; a gate-ek között lehet átfedő teszt. Sikeres, változatlan tesztet nem futtattunk újra. Full regresszió nem futott: source/config/contract változás nem készült.

Az audit hardvermentes próbái ezen felül igazolták a késői delegate toolhatárt, a voice aktív beszéd közbeni session-expiryt, az equality eltérést, a két cadence-config elfogadását, a stale host readiness elfogadását, a két NaN-konfigurációt és a 10 ms tick / 50 Hz capture elnevezési eltérést. Ezek az audit állításait ellenőrző diagnosztikai próbák, nem hozzáadott regressziós tesztek. A külső robot/network hívások stuboltak voltak; a P0 lifecycle/core/broker/regiszter út valódi repo-kódot futtatott.

**Replay evidence:** szintetikus native replay tesztek. Meglévő helyi MCAP-t nem olvastunk, explicit MCAP Evidence Compiler nem futott. **Live evidence:** nincs; motor-outputtal vagy anélkül live runtime sem indult.

Nem bizonyított a fizikai P0 mellékhatás, STOP-race üzemi gyakorisága, hardware/GIL/CPU/scheduler/IPC jitter, actual source cadence vagy p99 runtime budget, capture finalizálási idő/adatvesztés, hálózati SDK default timeout, service-env override-ok és server/lock contention. A statikus budget nem mért hard realtime garancia. A sikeres meglévő tesztek nem fedik le automatikusan a feltárt lifecycle-réseket.

Első javítási sorrend: P0-01 turn revocation → P1-01 continuous silence → P1-04 host status freshness → P1-03 cadence invariants → P1-02/05/06 konkrét lifecycle adapterek. A P2 policy/default/metadata pontosítások ezután végezhetők. A javításokhoz külön feladat és a megváltozó boundaryt igazoló célzott teszt szükséges; az auditban nem változtattuk meg a viselkedést.

## Aktív konfiguráció temporal paraméterei

Az alábbi értékek a vizsgált JSON-ból származnak. Az inaktív/compat mezők is szerepelnek, mert a konfiguráció részei; alkalmazhatóságukat P2-02 és a ConfigResolver diagnostics jelöli. A revision/scan darabszámok konzekutív observation-alapú lifecycle határok, nem másodpercértékek. A `max_attempts` countot külön felvettük. A sorok a kulcs alapján és a forrásleltárral visszakereshetők.

**110 értelmezett temporal/cadence/lifecycle paraméter.** A numerikus SSOT minden sorban T01; az owner oszlop a szemantikai fogyasztó.

| Configfájl | Kulcs | JSON-érték | Olvasható érték | Owner |
|---|---|---|---|---|
| `conf/hardver.json` | `camera.fps` | `16.0` | 16 frame/s | T10 |
| `conf/hardver.json` | `camera.max_frame_completion_lag_ns` | `500000000` | 500 ms | T10 |
| `conf/hardver.json` | `camera.stop_join_timeout_s` | `2.0` | 2 s | T10 |
| `conf/hardver.json` | `person_detection.maximum_result_age_ns` | `500000000` | 500 ms | T11 |
| `conf/hardver.json` | `person_detection.photo_evidence.minimum_interval_ns` | `5000000000` | 5 s | T11 |
| `conf/vezerles.json` | `lidar_pose.tracking_reacquire_consecutive_scans` | `3` | 3 count | T08 |
| `conf/vezerles.json` | `lidar_pose.matcher_budget_ms` | `55` | 55 ms | T08 |
| `conf/vezerles.json` | `lidar_pose.slow_path_budget_ms` | `120` | 120 ms | T08 |
| `conf/vezerles.json` | `lidar_pose.relocalization_cooldown_s` | `1` | 1 s | T08 |
| `conf/vezerles.json` | `lidar_pose.loop_closure_cooldown_s` | `2` | 2 s | T08 |
| `conf/vezerles.json` | `lidar_pose.relative_max_interval_ns` | `500000000` | 500 ms | T08 |
| `conf/vezerles.json` | `lidar_runtime.matcher_process_ready_timeout_s` | `8` | 8 s | T08 |
| `conf/vezerles.json` | `lidar_runtime.matcher_stop_timeout_s` | `1` | 1 s | T08 |
| `conf/vezerles.json` | `lidar_runtime.matcher_max_input_age_s` | `0.25` | 0.25 s | T08 |
| `conf/vezerles.json` | `lidar_runtime.driver_poll_hz` | `120` | 120 Hz | T08 |
| `conf/vezerles.json` | `lidar_driver.read_timeout_s` | `0.1` | 0.1 s | T07 |
| `conf/vezerles.json` | `lidar_driver.stale_timeout_s` | `0.5` | 0.5 s | T07 |
| `conf/vezerles.json` | `lidar_driver.startup_grace_s` | `10.0` | 10 s | T07 |
| `conf/vezerles.json` | `lidar_driver.reconnect_interval_s` | `0.4` | 0.4 s | T07 |
| `conf/vezerles.json` | `lidar_driver.command_settle_s` | `0.5` | 0.5 s | T07 |
| `conf/vezerles.json` | `lidar_driver.stop_join_timeout_s` | `2.0` | 2 s | T07 |
| `conf/vezerles.json` | `layers.admission.max_sample_age_ns` | `250000000` | 250 ms | T04 |
| `conf/vezerles.json` | `layers.admission.max_future_skew_ns` | `10000000` | 10 ms | T04 |
| `conf/vezerles.json` | `layers.estimation.max_dt_ns` | `250000000` | 250 ms | T12 |
| `conf/vezerles.json` | `layers.estimation.max_measurement_age_ns` | `250000000` | 250 ms | T12 |
| `conf/vezerles.json` | `layers.estimation.process_noise_reference_dt_s` | `0.02` | 0.02 s | T12 |
| `conf/vezerles.json` | `layers.estimation.stationary_prediction_hold_ns` | `50000000` | 50 ms | T12 |
| `conf/vezerles.json` | `layers.estimation.quality.global_fix_max_age_ns` | `2000000000` | 2 s | T12 |
| `conf/vezerles.json` | `layers.estimation.quality.relative_max_age_ns` | `1500000000` | 1.5 s | T12 |
| `conf/vezerles.json` | `layers.estimation.quality.consistency_memory_s` | `30.0` | 30 s | T12 |
| `conf/vezerles.json` | `layers.world_model.max_track_age_ns` | `500000000` | 500 ms | T13 |
| `conf/vezerles.json` | `layers.world_model.local_costmap_max_cell_age_ns` | `750000000` | 750 ms | T13 |
| `conf/vezerles.json` | `layers.world_model.person_lidar_max_skew_ns` | `150000000` | 150 ms | T13 |
| `conf/vezerles.json` | `layers.world_model.person_track_max_age_ns` | `500000000` | 500 ms | T13 |
| `conf/vezerles.json` | `layers.world_model.person_track_reacquire_max_age_ns` | `2500000000` | 2.5 s | T13 |
| `conf/vezerles.json` | `layers.world_model.pose_history_max_age_ns` | `2000000000` | 2 s | T13 |
| `conf/vezerles.json` | `layers.world_model.pose_lookup_max_skew_ns` | `250000000` | 250 ms | T13 |
| `conf/vezerles.json` | `layers.world_model.scan_history_max_age_ns` | `1000000000` | 1 s | T13 |
| `conf/vezerles.json` | `layers.world_model.person_track_prediction_max_age_ns` | `350000000` | 350 ms | T13 |
| `conf/vezerles.json` | `layers.world_model.structural_max_age_ns` | `30000000000` | 30 s | T13 |
| `conf/vezerles.json` | `layers.world_model.structural_confirm_min_span_ns` | `1000000000` | 1 s | T13 |
| `conf/vezerles.json` | `layers.navigation.max_world_freshness_ns` | `250000000` | 250 ms | T15 |
| `conf/vezerles.json` | `layers.navigation.max_costmap_freshness_ns` | `250000000` | 250 ms | T15 |
| `conf/vezerles.json` | `layers.navigation.trajectory_replan_interval_ns` | `100000000` | 100 ms | T15 |
| `conf/vezerles.json` | `layers.navigation.trajectory_replan_min_tick_gap` | `5` | 5 tick (inaktív production módban) | T15 |
| `conf/vezerles.json` | `layers.navigation.local_goal_max_age_ns` | `8000000000` | 8 s | T15 |
| `conf/vezerles.json` | `layers.navigation.rollout_horizon_ns` | `1000000000` | 1 s | T15 |
| `conf/vezerles.json` | `layers.navigation.follow_person_lost_hold_ns` | `400000000` | 400 ms | T15 |
| `conf/vezerles.json` | `layers.navigation.follow_person_search_timeout_ns` | `2000000000` | 2 s | T15 |
| `conf/vezerles.json` | `layers.navigation.follow_person_search_step_ns` | `400000000` | 400 ms | T15 |
| `conf/vezerles.json` | `layers.navigation.follow_person_search_max_duration_ns` | `10000000000` | 10 s | T15 |
| `conf/vezerles.json` | `layers.navigation.localization_recovery_timeout_ns` | `5000000000` | 5 s | T15 |
| `conf/vezerles.json` | `layers.async_l6.request_timeout_ns` | `300000000` | 300 ms | T16 |
| `conf/vezerles.json` | `layers.async_l6.release_tick_gap` | `5` | 5 tick (inaktív production módban) | T16 |
| `conf/vezerles.json` | `layers.async_l6.max_plan_age_ns` | `350000000` | 350 ms | T16 |
| `conf/vezerles.json` | `layers.async_l6.release_delay_ns` | `100000000` | 100 ms | T16 |
| `conf/vezerles.json` | `layers.async_l6.transport_timeout_ns` | `2000000000` | 2 s | T16 |
| `conf/vezerles.json` | `layers.motion_realization.horizon_ns` | `100000000` | 100 ms | T17 |
| `conf/vezerles.json` | `layers.motion_realization.max_control_gap_ns` | `250000000` | 250 ms | T17 |
| `conf/vezerles.json` | `layers.wheel_pi.max_control_gap_ns` | `250000000` | 250 ms | T18 |
| `conf/vezerles.json` | `layers.wheel_pi.max_feedback_uncertainty_ns` | `250000000` | 250 ms | T18 |
| `conf/vezerles.json` | `layers.wheel_pi.max_feedback_age_ns` | `250000000` | 250 ms | T18 |
| `conf/vezerles.json` | `sensor_policy.encoder_maximum_sample_interval_ns` | `100000000` | 100 ms | T05 |
| `conf/vezerles.json` | `sensor_policy.imu_maximum_sample_age_ns` | `100000000` | 100 ms | T06 |
| `conf/vezerles.json` | `sensor_policy.lidar_maximum_result_age_ns` | `250000000` | 250 ms | T08/T19 |
| `conf/vezerles.json` | `sensor_policy.lidar_maximum_future_skew_ns` | `10000000` | 10 ms | T08/T19 |
| `conf/vezerles.json` | `sensor_policy.lidar_maximum_measurement_age_ns` | `250000000` | 250 ms | T08/T19 |
| `conf/vezerles.json` | `sensor_policy.encoder_minimum_estimation_window_ns` | `40000000` | 40 ms | T05 |
| `conf/vezerles.json` | `sensor_policy.encoder_maximum_estimation_window_ns` | `160000000` | 160 ms | T05 |
| `conf/vezerles.json` | `sensor_policy.camera_maximum_frame_age_ns` | `250000000` | 250 ms | T10 |
| `conf/vezerles.json` | `motor.pwm_frequency_hz` | `8000` | 8000 Hz | T19 |
| `conf/vezerles.json` | `encoder.a_debounce_micros` | `150` | 150 µs | T05 |
| `conf/vezerles.json` | `encoder.direction_guard_micros` | `50` | 50 µs | T05 |
| `conf/vezerles.json` | `encoder.direction_change_confirm_window_micros` | `250000` | 250000 µs | T05 |
| `conf/vezerles.json` | `imu.startup_timeout_ns` | `1000000000` | 1 s | T06 |
| `conf/vezerles.json` | `imu.startup_poll_interval_ns` | `50000000` | 50 ms | T06 |
| `conf/vezerles.json` | `runtime.tick_period_ns` | `20000000` | 20 ms | T02 |
| `conf/vezerles.json` | `runtime.max_preflight_age_ns` | `250000000` | 250 ms | T02 |
| `conf/vezerles.json` | `runtime.required_lidar_preflight_revisions` | `3` | 3 count | T02 |
| `conf/vezerles.json` | `runtime.multirate.critical_default_period_ns` | `20000000` | 20 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.auxiliary_default_period_ns` | `50000000` | 50 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.max_snapshot_age_ns` | `250000000` | 250 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.worker_join_timeout_s` | `2.0` | 2 s | T09 |
| `conf/vezerles.json` | `runtime.multirate.source_periods[0].period_ns (WHEEL_ENCODERS)` | `20000000` | 20 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.source_periods[1].period_ns (BNO055_IMU)` | `20000000` | 20 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.source_periods[2].period_ns (RPLIDAR_C1)` | `20000000` | 20 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.source_periods[3].period_ns (CAMERA_FRONT)` | `40000000` | 40 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.source_periods[4].period_ns (PERSON_DETECTOR_FRONT)` | `60000000` | 60 ms | T09 |
| `conf/vezerles.json` | `runtime.multirate.source_periods[5].period_ns (MICROPHONE_FRONT)` | `40000000` | 40 ms | T09 |
| `conf/vezerles.json` | `runtime.planner_recovery.retry_backoff_ns` | `50000000` | 50 ms | T16 |
| `conf/vezerles.json` | `runtime.planner_recovery.ready_timeout_s` | `1.25` | 1.25 s | T16 |
| `conf/vezerles.json` | `runtime.encoder_process.sample_period_ns` | `20000000` | 20 ms | T05 |
| `conf/vezerles.json` | `runtime.encoder_process.ready_timeout_s` | `5.0` | 5 s | T05 |
| `conf/vezerles.json` | `runtime.encoder_process.stop_timeout_s` | `2.0` | 2 s | T05 |
| `conf/vezerles.json` | `runtime.imu_process.sample_period_ns` | `20000000` | 20 ms | T06 |
| `conf/vezerles.json` | `runtime.imu_process.ready_timeout_s` | `5.0` | 5 s | T06 |
| `conf/vezerles.json` | `runtime.imu_process.stop_timeout_s` | `2.0` | 2 s | T06 |
| `conf/vezerles.json` | `runtime.lidar_process.state_heartbeat_ns` | `100000000` | 100 ms | T08 |
| `conf/vezerles.json` | `runtime.lidar_process.ready_timeout_s` | `12.0` | 12 s | T08 |
| `conf/vezerles.json` | `runtime.lidar_process.stop_timeout_s` | `3.0` | 3 s | T08 |
| `conf/vezerles.json` | `runtime.lidar_process.pose_lock_timeout_s` | `0.05` | 0.05 s | T08 |
| `conf/vezerles.json` | `runtime.lidar_process.raw_end_timeout_s` | `0.5` | 0.5 s | T08 |
| `conf/vezerles.json` | `runtime.planner_process.collector_poll_s` | `0.05` | 0.05 s | T16 |
| `conf/vezerles.json` | `runtime.planner_process.stop_enqueue_timeout_s` | `0.2` | 0.2 s | T16 |
| `conf/vezerles.json` | `runtime.planner_process.stop_timeout_s` | `1.0` | 1 s | T16 |
| `conf/vezerles.json` | `runtime.command_ingress.maximum_ttl_ns` | `250000000` | 250 ms | T03 |
| `conf/vezerles.json` | `runtime.command_ingress.maximum_future_skew_ns` | `5000000` | 5 ms | T03 |
| `conf/vezerles.json` | `runtime.command_ingress.reader_poll_s` | `0.005` | 0.005 s | T03 |
| `conf/vezerles.json` | `runtime.command_ingress.reader_stop_timeout_s` | `1.0` | 1 s | T03 |
| `conf/vezerles.json` | `runtime.planner_recovery.max_attempts` | `3` | 3 count | T16 |

## Teljes fájllefedettség

Minden beolvasott tracked szöveges fájl; az oszlop a CSV-ben szereplő temporal sorok száma. A nulla nem azt jelenti, hogy a komponensnek nincs lifetime-ja, hanem hogy nincs a fenti módszerrel talált explicit temporal előfordulás. A teljes forráslistát nem bővítettük a task során keletkező auditfájlokkal.

| Fájl | Temporal sorok |
|---|---:|
| [.gitignore](../.gitignore) | 0 |
| [AGENTS.md](../AGENTS.md) | 2 |
| [ASZINKRON_RUNTIME_CONTRACT_V3.md](../ASZINKRON_RUNTIME_CONTRACT_V3.md) | 15 |
| [R2B4_SYSTEM_BEHAVIOR_CONTRACT.md](../R2B4_SYSTEM_BEHAVIOR_CONTRACT.md) | 6 |
| [README.md](../README.md) | 0 |
| [STRUKTURALIS_RETEGEK_V3.md](../STRUKTURALIS_RETEGEK_V3.md) | 34 |
| [conf/fizika.json](../conf/fizika.json) | 0 |
| [conf/hardver.json](../conf/hardver.json) | 5 |
| [conf/r2b4_agent_system.md](../conf/r2b4_agent_system.md) | 0 |
| [conf/speed_map.json](../conf/speed_map.json) | 0 |
| [conf/vezerles.json](../conf/vezerles.json) | 111 |
| [conf/voice_llm_system.md](../conf/voice_llm_system.md) | 0 |
| [deploy/bash-completion/r](../deploy/bash-completion/r) | 0 |
| [deploy/systemd/r2b4-wake.service](../deploy/systemd/r2b4-wake.service) | 2 |
| [deploy/systemd/wake.env.example](../deploy/systemd/wake.env.example) | 0 |
| [docs/DIAG.md](../docs/DIAG.md) | 1 |
| [docs/DIAG_P0_P1_DESIGN.md](../docs/DIAG_P0_P1_DESIGN.md) | 1 |
| [docs/ER2_NAVIGATION.md](../docs/ER2_NAVIGATION.md) | 10 |
| [docs/LOCAL_GLOBAL_LOCALIZATION.md](../docs/LOCAL_GLOBAL_LOCALIZATION.md) | 19 |
| [docs/MCAP_EVIDENCE.md](../docs/MCAP_EVIDENCE.md) | 1 |
| [docs/MOTION_ANALYSIS_20260930.md](../docs/MOTION_ANALYSIS_20260930.md) | 12 |
| [docs/MOTION_CONTINUITY_PLAN_20260929.md](../docs/MOTION_CONTINUITY_PLAN_20260929.md) | 29 |
| [docs/PYTEST_POLICY.md](../docs/PYTEST_POLICY.md) | 8 |
| [docs/R2B4_AGENT_CORE.md](../docs/R2B4_AGENT_CORE.md) | 0 |
| [docs/ROOMCRUISE_TUNER.md](../docs/ROOMCRUISE_TUNER.md) | 2 |
| [docs/TEMPORAL_OWNERSHIP.md](../docs/TEMPORAL_OWNERSHIP.md) | 29 |
| [gitre.py](../gitre.py) | 0 |
| [old/test_er2_defaults_and_event_logging.py](../old/test_er2_defaults_and_event_logging.py) | 0 |
| [old/test_er2_launcher_stream_cli.py](../old/test_er2_launcher_stream_cli.py) | 1 |
| [old/test_er2_local_tts_wiring.py](../old/test_er2_local_tts_wiring.py) | 0 |
| [old/test_er2_navigation_motion.py](../old/test_er2_navigation_motion.py) | 18 |
| [old/test_er2_p0r2.py](../old/test_er2_p0r2.py) | 35 |
| [old/test_launcher_tab_voice.py](../old/test_launcher_tab_voice.py) | 0 |
| [old/test_launcher_workflows.py](../old/test_launcher_workflows.py) | 7 |
| [old/test_local_tts_provider.py](../old/test_local_tts_provider.py) | 1 |
| [old/test_plain_llm_launcher.py](../old/test_plain_llm_launcher.py) | 2 |
| [old/test_v3_camera_foundation.py](../old/test_v3_camera_foundation.py) | 9 |
| [old/test_v3_camera_geometry.py](../old/test_v3_camera_geometry.py) | 1 |
| [old/test_v3_camera_rectification.py](../old/test_v3_camera_rectification.py) | 1 |
| [old/test_v3_encoder_ab_direction_robustness.py](../old/test_v3_encoder_ab_direction_robustness.py) | 7 |
| [old/test_v3_finite_motion_frame_transaction.py](../old/test_v3_finite_motion_frame_transaction.py) | 13 |
| [old/test_v3_l11_reversal_reacquisition.py](../old/test_v3_l11_reversal_reacquisition.py) | 14 |
| [pytest.ini](../pytest.ini) | 0 |
| [r](../r) | 0 |
| [r2b4_control_rootcause_diag.py](../r2b4_control_rootcause_diag.py) | 181 |
| [r2b4_er2/__init__.py](../r2b4_er2/__init__.py) | 0 |
| [r2b4_er2/__main__.py](../r2b4_er2/__main__.py) | 0 |
| [r2b4_er2/cli.py](../r2b4_er2/cli.py) | 11 |
| [r2b4_er2/config.py](../r2b4_er2/config.py) | 18 |
| [r2b4_er2/evidence.py](../r2b4_er2/evidence.py) | 1 |
| [r2b4_er2/executor.py](../r2b4_er2/executor.py) | 8 |
| [r2b4_er2/media.py](../r2b4_er2/media.py) | 2 |
| [r2b4_er2/preview.py](../r2b4_er2/preview.py) | 0 |
| [r2b4_er2/speech.py](../r2b4_er2/speech.py) | 0 |
| [r2b4_er2/streaming.py](../r2b4_er2/streaming.py) | 38 |
| [r2b4_er2/tool_bridge.py](../r2b4_er2/tool_bridge.py) | 27 |
| [r2b4_imu_control_diag.py](../r2b4_imu_control_diag.py) | 176 |
| [r2b4_lidar_l4_latency_probe.py](../r2b4_lidar_l4_latency_probe.py) | 164 |
| [r2b4_live_control_rootcause_diag.py](../r2b4_live_control_rootcause_diag.py) | 154 |
| [r2b4_orchestration/__init__.py](../r2b4_orchestration/__init__.py) | 0 |
| [r2b4_orchestration/agent_config_tools.py](../r2b4_orchestration/agent_config_tools.py) | 6 |
| [r2b4_orchestration/agent_contracts.py](../r2b4_orchestration/agent_contracts.py) | 0 |
| [r2b4_orchestration/agent_core.py](../r2b4_orchestration/agent_core.py) | 0 |
| [r2b4_orchestration/agent_evidence_tools.py](../r2b4_orchestration/agent_evidence_tools.py) | 4 |
| [r2b4_orchestration/agent_runner.py](../r2b4_orchestration/agent_runner.py) | 3 |
| [r2b4_orchestration/agent_source_tools.py](../r2b4_orchestration/agent_source_tools.py) | 0 |
| [r2b4_orchestration/agent_tools.py](../r2b4_orchestration/agent_tools.py) | 11 |
| [r2b4_orchestration/execution_mode.py](../r2b4_orchestration/execution_mode.py) | 3 |
| [r2b4_orchestration/executor.py](../r2b4_orchestration/executor.py) | 7 |
| [r2b4_periferial_control_diag.py](../r2b4_periferial_control_diag.py) | 222 |
| [r2b4_voice/__init__.py](../r2b4_voice/__init__.py) | 0 |
| [r2b4_voice/action_executor.py](../r2b4_voice/action_executor.py) | 11 |
| [r2b4_voice/action_validation.py](../r2b4_voice/action_validation.py) | 0 |
| [r2b4_voice/conversation_cli.py](../r2b4_voice/conversation_cli.py) | 4 |
| [r2b4_voice/conversation_contracts.py](../r2b4_voice/conversation_contracts.py) | 3 |
| [r2b4_voice/conversation_interface.py](../r2b4_voice/conversation_interface.py) | 1 |
| [r2b4_voice/conversation_journal.py](../r2b4_voice/conversation_journal.py) | 4 |
| [r2b4_voice/conversation_service.py](../r2b4_voice/conversation_service.py) | 17 |
| [r2b4_voice/gemini_llm.py](../r2b4_voice/gemini_llm.py) | 8 |
| [r2b4_voice/gemini_tts.py](../r2b4_voice/gemini_tts.py) | 11 |
| [r2b4_voice/groq_llm.py](../r2b4_voice/groq_llm.py) | 5 |
| [r2b4_voice/groq_stt.py](../r2b4_voice/groq_stt.py) | 26 |
| [r2b4_voice/llm_decision.py](../r2b4_voice/llm_decision.py) | 0 |
| [r2b4_voice/llm_failover.py](../r2b4_voice/llm_failover.py) | 29 |
| [r2b4_voice/llm_provider.py](../r2b4_voice/llm_provider.py) | 0 |
| [r2b4_voice/openai_llm.py](../r2b4_voice/openai_llm.py) | 22 |
| [r2b4_voice/openai_oauth.py](../r2b4_voice/openai_oauth.py) | 47 |
| [r2b4_voice/piper_tts.py](../r2b4_voice/piper_tts.py) | 4 |
| [r2b4_voice/plain_llm.py](../r2b4_voice/plain_llm.py) | 8 |
| [r2b4_voice/prompting.py](../r2b4_voice/prompting.py) | 0 |
| [r2b4_voice/resident_tts.py](../r2b4_voice/resident_tts.py) | 19 |
| [r2b4_voice/robot_context.py](../r2b4_voice/robot_context.py) | 0 |
| [r2b4_voice/runtime_control.py](../r2b4_voice/runtime_control.py) | 13 |
| [r2b4_voice/safety_intents.py](../r2b4_voice/safety_intents.py) | 0 |
| [r2b4_voice/self_knowledge.py](../r2b4_voice/self_knowledge.py) | 15 |
| [r2b4_voice/speaker.py](../r2b4_voice/speaker.py) | 3 |
| [r2b4_voice/tts_daemon.py](../r2b4_voice/tts_daemon.py) | 4 |
| [r2b4_voice/tts_provider.py](../r2b4_voice/tts_provider.py) | 0 |
| [r2b4_voice/voice_output.py](../r2b4_voice/voice_output.py) | 7 |
| [r2b4_voice/voice_service.py](../r2b4_voice/voice_service.py) | 102 |
| [r2b4_voice/wake_core.py](../r2b4_voice/wake_core.py) | 41 |
| [r2b4_voice/wake_service.py](../r2b4_voice/wake_service.py) | 40 |
| [requirements-interop.txt](../requirements-interop.txt) | 0 |
| [requirements.txt](../requirements.txt) | 0 |
| [tests/README.md](../tests/README.md) | 2 |
| [tests/conftest.py](../tests/conftest.py) | 1 |
| [tests/core/test_agent_config_tools.py](../tests/core/test_agent_config_tools.py) | 0 |
| [tests/core/test_agent_prompt_hierarchy.py](../tests/core/test_agent_prompt_hierarchy.py) | 3 |
| [tests/core/test_gemini_structured_transport.py](../tests/core/test_gemini_structured_transport.py) | 5 |
| [tests/core/test_llm_failover.py](../tests/core/test_llm_failover.py) | 6 |
| [tests/core/test_openai_oauth.py](../tests/core/test_openai_oauth.py) | 5 |
| [tests/core/test_openai_primary_provider.py](../tests/core/test_openai_primary_provider.py) | 0 |
| [tests/core/test_openai_responses_transport.py](../tests/core/test_openai_responses_transport.py) | 2 |
| [tests/core/test_operator_sequence_stop.py](../tests/core/test_operator_sequence_stop.py) | 13 |
| [tests/core/test_v3_bounded_runtime_config.py](../tests/core/test_v3_bounded_runtime_config.py) | 1 |
| [tests/core/test_v3_camera_geometry_ssot.py](../tests/core/test_v3_camera_geometry_ssot.py) | 5 |
| [tests/core/test_v3_config_p0_authority.py](../tests/core/test_v3_config_p0_authority.py) | 8 |
| [tests/core/test_v3_gpio_motor.py](../tests/core/test_v3_gpio_motor.py) | 12 |
| [tests/core/test_v3_minimum_continuous_wheel_speed.py](../tests/core/test_v3_minimum_continuous_wheel_speed.py) | 0 |
| [tests/core/test_v3_motor_pwm_contract.py](../tests/core/test_v3_motor_pwm_contract.py) | 0 |
| [tests/core/test_v3_multirate_inputs.py](../tests/core/test_v3_multirate_inputs.py) | 3 |
| [tests/core/test_v3_planner_escape_deadlock.py](../tests/core/test_v3_planner_escape_deadlock.py) | 4 |
| [tests/core/test_v3_route_optimization_config.py](../tests/core/test_v3_route_optimization_config.py) | 10 |
| [tests/core/test_v3_stop_only_composition.py](../tests/core/test_v3_stop_only_composition.py) | 1 |
| [tests/core/test_v3_temporal_config_invariants.py](../tests/core/test_v3_temporal_config_invariants.py) | 16 |
| [tests/core/test_v3_temporal_contract.py](../tests/core/test_v3_temporal_contract.py) | 23 |
| [tests/deep/test_mcap_evidence.py](../tests/deep/test_mcap_evidence.py) | 5 |
| [tests/deep/test_mcap_evidence_sparse_index.py](../tests/deep/test_mcap_evidence_sparse_index.py) | 1 |
| [tests/deep/test_v3_async_peripheral_isolation.py](../tests/deep/test_v3_async_peripheral_isolation.py) | 64 |
| [tests/deep/test_v3_lidar_world_replay.py](../tests/deep/test_v3_lidar_world_replay.py) | 60 |
| [tests/deep/test_v3_localization_process.py](../tests/deep/test_v3_localization_process.py) | 24 |
| [tests/deep/test_v3_process_affinity.py](../tests/deep/test_v3_process_affinity.py) | 10 |
| [tests/deep/test_v3_replay_claims.py](../tests/deep/test_v3_replay_claims.py) | 0 |
| [tests/feature/test_agent_core.py](../tests/feature/test_agent_core.py) | 0 |
| [tests/feature/test_camera_cli.py](../tests/feature/test_camera_cli.py) | 10 |
| [tests/feature/test_camera_observation.py](../tests/feature/test_camera_observation.py) | 5 |
| [tests/feature/test_execution_mode_executor.py](../tests/feature/test_execution_mode_executor.py) | 4 |
| [tests/feature/test_execution_mode_selector.py](../tests/feature/test_execution_mode_selector.py) | 5 |
| [tests/feature/test_resident_tts.py](../tests/feature/test_resident_tts.py) | 7 |
| [tests/feature/test_tools_diag_evidence.py](../tests/feature/test_tools_diag_evidence.py) | 2 |
| [tests/feature/test_tools_diag_p0_p1_quality.py](../tests/feature/test_tools_diag_p0_p1_quality.py) | 6 |
| [tests/feature/test_tools_diag_persistence.py](../tests/feature/test_tools_diag_persistence.py) | 0 |
| [tests/feature/test_v3_bno055_imu_backend.py](../tests/feature/test_v3_bno055_imu_backend.py) | 8 |
| [tests/feature/test_v3_canonical_motion_harmony.py](../tests/feature/test_v3_canonical_motion_harmony.py) | 11 |
| [tests/feature/test_v3_counter_encoder_backend.py](../tests/feature/test_v3_counter_encoder_backend.py) | 45 |
| [tests/feature/test_v3_dual_frame_localization.py](../tests/feature/test_v3_dual_frame_localization.py) | 26 |
| [tests/feature/test_v3_latest_lidar_backend.py](../tests/feature/test_v3_latest_lidar_backend.py) | 53 |
| [tests/feature/test_v3_lidar_world_model.py](../tests/feature/test_v3_lidar_world_model.py) | 56 |
| [tests/feature/test_v3_live_lidar.py](../tests/feature/test_v3_live_lidar.py) | 11 |
| [tests/feature/test_v3_motion_feedback_quality.py](../tests/feature/test_v3_motion_feedback_quality.py) | 12 |
| [tests/feature/test_v3_person_detection.py](../tests/feature/test_v3_person_detection.py) | 10 |
| [tests/feature/test_v3_person_detection_runtime_integration.py](../tests/feature/test_v3_person_detection_runtime_integration.py) | 50 |
| [tests/feature/test_v3_person_geometry_projection.py](../tests/feature/test_v3_person_geometry_projection.py) | 10 |
| [tests/feature/test_v3_rate_only_heading_authority.py](../tests/feature/test_v3_rate_only_heading_authority.py) | 14 |
| [tests/feature/test_v3_roomcruise_localization_motion.py](../tests/feature/test_v3_roomcruise_localization_motion.py) | 42 |
| [tests/feature/test_v3_roomcruise_tuner.py](../tests/feature/test_v3_roomcruise_tuner.py) | 8 |
| [tests/feature/test_v3_rplidar_c1.py](../tests/feature/test_v3_rplidar_c1.py) | 12 |
| [tests/feature/test_v3_stationary_covariance.py](../tests/feature/test_v3_stationary_covariance.py) | 5 |
| [tests/feature/test_v3_stationary_relocalization_repair.py](../tests/feature/test_v3_stationary_relocalization_repair.py) | 4 |
| [tests/feature/test_voice_orchestration_p0.py](../tests/feature/test_voice_orchestration_p0.py) | 19 |
| [tests/rig.py](../tests/rig.py) | 3 |
| [tests/v3_test_fixtures.py](../tests/v3_test_fixtures.py) | 9 |
| [tools/__init__.py](../tools/__init__.py) | 0 |
| [tools/diag/__init__.py](../tools/diag/__init__.py) | 0 |
| [tools/diag/__main__.py](../tools/diag/__main__.py) | 0 |
| [tools/diag/admission.py](../tools/diag/admission.py) | 0 |
| [tools/diag/analyzers/__init__.py](../tools/diag/analyzers/__init__.py) | 0 |
| [tools/diag/analyzers/drive.py](../tools/diag/analyzers/drive.py) | 0 |
| [tools/diag/analyzers/evidence_health.py](../tools/diag/analyzers/evidence_health.py) | 1 |
| [tools/diag/analyzers/execution_chain.py](../tools/diag/analyzers/execution_chain.py) | 2 |
| [tools/diag/analyzers/lifecycle.py](../tools/diag/analyzers/lifecycle.py) | 0 |
| [tools/diag/analyzers/lineage.py](../tools/diag/analyzers/lineage.py) | 0 |
| [tools/diag/analyzers/localization.py](../tools/diag/analyzers/localization.py) | 2 |
| [tools/diag/analyzers/navigation.py](../tools/diag/analyzers/navigation.py) | 0 |
| [tools/diag/analyzers/recovery.py](../tools/diag/analyzers/recovery.py) | 0 |
| [tools/diag/analyzers/safety.py](../tools/diag/analyzers/safety.py) | 0 |
| [tools/diag/analyzers/world_model.py](../tools/diag/analyzers/world_model.py) | 0 |
| [tools/diag/basis.py](../tools/diag/basis.py) | 0 |
| [tools/diag/cli.py](../tools/diag/cli.py) | 0 |
| [tools/diag/context.py](../tools/diag/context.py) | 7 |
| [tools/diag/contracts.py](../tools/diag/contracts.py) | 8 |
| [tools/diag/episodes.py](../tools/diag/episodes.py) | 2 |
| [tools/diag/persistence.py](../tools/diag/persistence.py) | 0 |
| [tools/diag/profiling.py](../tools/diag/profiling.py) | 8 |
| [tools/diag/provenance.py](../tools/diag/provenance.py) | 2 |
| [tools/diag/registry.py](../tools/diag/registry.py) | 0 |
| [tools/mcap_evidence/__init__.py](../tools/mcap_evidence/__init__.py) | 0 |
| [tools/mcap_evidence/__main__.py](../tools/mcap_evidence/__main__.py) | 0 |
| [tools/mcap_evidence/cli.py](../tools/mcap_evidence/cli.py) | 0 |
| [tools/mcap_evidence/compiler.py](../tools/mcap_evidence/compiler.py) | 2 |
| [tools/mcap_evidence/export.py](../tools/mcap_evidence/export.py) | 0 |
| [tools/mcap_evidence/index.py](../tools/mcap_evidence/index.py) | 3 |
| [tools/mcap_evidence/ingest.py](../tools/mcap_evidence/ingest.py) | 17 |
| [tools/mcap_evidence/normalize.py](../tools/mcap_evidence/normalize.py) | 0 |
| [tools/mcap_evidence/performance.py](../tools/mcap_evidence/performance.py) | 29 |
| [tools/mcap_evidence/query.py](../tools/mcap_evidence/query.py) | 3 |
| [tools/mcap_evidence/reader.py](../tools/mcap_evidence/reader.py) | 19 |
| [tools/mcap_evidence/records.py](../tools/mcap_evidence/records.py) | 0 |
| [tools/mcap_evidence/salvage.py](../tools/mcap_evidence/salvage.py) | 2 |
| [tools/mcap_evidence/schemas.py](../tools/mcap_evidence/schemas.py) | 0 |
| [tools/mcap_evidence/verify.py](../tools/mcap_evidence/verify.py) | 4 |
| [tools/r2b4_async_input_soak.py](../tools/r2b4_async_input_soak.py) | 52 |
| [tools/r2b4_capture_ab_gate.py](../tools/r2b4_capture_ab_gate.py) | 10 |
| [tools/r2b4_capture_size_audit.py](../tools/r2b4_capture_size_audit.py) | 0 |
| [tools/r2b4_control_rootcause_diag.py](../tools/r2b4_control_rootcause_diag.py) | 181 |
| [tools/r2b4_cpu2.py](../tools/r2b4_cpu2.py) | 59 |
| [tools/r2b4_imu_control_diag.py](../tools/r2b4_imu_control_diag.py) | 176 |
| [tools/r2b4_lidar_l4_latency_probe.py](../tools/r2b4_lidar_l4_latency_probe.py) | 164 |
| [tools/r2b4_live_control_rootcause_diag.py](../tools/r2b4_live_control_rootcause_diag.py) | 154 |
| [tools/r2b4_periferial_control_diag.py](../tools/r2b4_periferial_control_diag.py) | 222 |
| [tools/r2b4_voice_mic_test.py](../tools/r2b4_voice_mic_test.py) | 14 |
| [tools/tuners/__init__.py](../tools/tuners/__init__.py) | 0 |
| [tools/tuners/r2b4_roomcruise_tuner.py](../tools/tuners/r2b4_roomcruise_tuner.py) | 25 |
| [tools/v3_camera_calibrate.py](../tools/v3_camera_calibrate.py) | 13 |
| [tools/v3_camera_test.py](../tools/v3_camera_test.py) | 26 |
| [tools/v3_encoder_ab_probe.py](../tools/v3_encoder_ab_probe.py) | 13 |
| [tools/v3_mcap_measurement.py](../tools/v3_mcap_measurement.py) | 9 |
| [tools/v3_p0_async_acceptance.py](../tools/v3_p0_async_acceptance.py) | 77 |
| [tools/v3_performance_audit.py](../tools/v3_performance_audit.py) | 16 |
| [tools/v3_person_detection_test.py](../tools/v3_person_detection_test.py) | 9 |
| [tools/v3_phase_timing_report.py](../tools/v3_phase_timing_report.py) | 7 |
| [tools/v3_sensor_measurement.py](../tools/v3_sensor_measurement.py) | 8 |
| [tools/validate_er2_p0.py](../tools/validate_er2_p0.py) | 19 |
| [v3/__init__.py](../v3/__init__.py) | 0 |
| [v3/action_catalog.py](../v3/action_catalog.py) | 28 |
| [v3/adapters/__init__.py](../v3/adapters/__init__.py) | 0 |
| [v3/adapters/async_person_photo_evidence.py](../v3/adapters/async_person_photo_evidence.py) | 3 |
| [v3/adapters/bno055_device.py](../v3/adapters/bno055_device.py) | 38 |
| [v3/adapters/bno055_imu.py](../v3/adapters/bno055_imu.py) | 18 |
| [v3/adapters/bounded_command.py](../v3/adapters/bounded_command.py) | 23 |
| [v3/adapters/camera.py](../v3/adapters/camera.py) | 2 |
| [v3/adapters/camera_geometry.py](../v3/adapters/camera_geometry.py) | 6 |
| [v3/adapters/camera_media.py](../v3/adapters/camera_media.py) | 29 |
| [v3/adapters/camera_ownership.py](../v3/adapters/camera_ownership.py) | 0 |
| [v3/adapters/camera_rectification.py](../v3/adapters/camera_rectification.py) | 14 |
| [v3/adapters/counter_encoder.py](../v3/adapters/counter_encoder.py) | 183 |
| [v3/adapters/fake_edges.py](../v3/adapters/fake_edges.py) | 2 |
| [v3/adapters/gpio_counter.py](../v3/adapters/gpio_counter.py) | 85 |
| [v3/adapters/gpio_encoder.py](../v3/adapters/gpio_encoder.py) | 2 |
| [v3/adapters/gpio_motor.py](../v3/adapters/gpio_motor.py) | 23 |
| [v3/adapters/l6_planner_process.py](../v3/adapters/l6_planner_process.py) | 47 |
| [v3/adapters/latest_lidar.py](../v3/adapters/latest_lidar.py) | 111 |
| [v3/adapters/litert_person_detector.py](../v3/adapters/litert_person_detector.py) | 0 |
| [v3/adapters/live_camera.py](../v3/adapters/live_camera.py) | 35 |
| [v3/adapters/live_encoder.py](../v3/adapters/live_encoder.py) | 69 |
| [v3/adapters/live_imu.py](../v3/adapters/live_imu.py) | 19 |
| [v3/adapters/live_inputs.py](../v3/adapters/live_inputs.py) | 2 |
| [v3/adapters/live_lidar.py](../v3/adapters/live_lidar.py) | 134 |
| [v3/adapters/live_microphone.py](../v3/adapters/live_microphone.py) | 19 |
| [v3/adapters/live_person_detection.py](../v3/adapters/live_person_detection.py) | 33 |
| [v3/adapters/microphone.py](../v3/adapters/microphone.py) | 76 |
| [v3/adapters/motor_pwm.py](../v3/adapters/motor_pwm.py) | 0 |
| [v3/adapters/motor_writer.py](../v3/adapters/motor_writer.py) | 2 |
| [v3/adapters/multirate_inputs.py](../v3/adapters/multirate_inputs.py) | 198 |
| [v3/adapters/native_lidar_port.py](../v3/adapters/native_lidar_port.py) | 117 |
| [v3/adapters/operator.py](../v3/adapters/operator.py) | 7 |
| [v3/adapters/person_detection.py](../v3/adapters/person_detection.py) | 49 |
| [v3/adapters/person_photo_evidence.py](../v3/adapters/person_photo_evidence.py) | 14 |
| [v3/adapters/picamera2_camera.py](../v3/adapters/picamera2_camera.py) | 89 |
| [v3/adapters/process_encoder_backend.py](../v3/adapters/process_encoder_backend.py) | 22 |
| [v3/adapters/process_imu_device.py](../v3/adapters/process_imu_device.py) | 17 |
| [v3/adapters/process_lidar_port.py](../v3/adapters/process_lidar_port.py) | 79 |
| [v3/adapters/process_vision_port.py](../v3/adapters/process_vision_port.py) | 55 |
| [v3/adapters/recovering_l6_planner.py](../v3/adapters/recovering_l6_planner.py) | 61 |
| [v3/adapters/resident_command.py](../v3/adapters/resident_command.py) | 92 |
| [v3/adapters/rplidar_c1.py](../v3/adapters/rplidar_c1.py) | 81 |
| [v3/adapters/system.py](../v3/adapters/system.py) | 4 |
| [v3/adapters/v3_control.py](../v3/adapters/v3_control.py) | 25 |
| [v3/adapters/vision_media_contracts.py](../v3/adapters/vision_media_contracts.py) | 9 |
| [v3/adapters/vision_media_socket.py](../v3/adapters/vision_media_socket.py) | 35 |
| [v3/adapters/vision_owner.py](../v3/adapters/vision_owner.py) | 24 |
| [v3/affinity_diagnostics.py](../v3/affinity_diagnostics.py) | 0 |
| [v3/async_capability.py](../v3/async_capability.py) | 49 |
| [v3/capture.py](../v3/capture.py) | 131 |
| [v3/capture_behavior.py](../v3/capture_behavior.py) | 1 |
| [v3/capture_compaction.py](../v3/capture_compaction.py) | 0 |
| [v3/capture_encoding.py](../v3/capture_encoding.py) | 21 |
| [v3/capture_ipc.py](../v3/capture_ipc.py) | 3 |
| [v3/capture_rate.py](../v3/capture_rate.py) | 10 |
| [v3/composition/__init__.py](../v3/composition/__init__.py) | 0 |
| [v3/composition/bounded_live_control.py](../v3/composition/bounded_live_control.py) | 19 |
| [v3/composition/bounded_physical_control.py](../v3/composition/bounded_physical_control.py) | 0 |
| [v3/composition/full_fake.py](../v3/composition/full_fake.py) | 0 |
| [v3/composition/input_shadow.py](../v3/composition/input_shadow.py) | 2 |
| [v3/composition/live_inputs.py](../v3/composition/live_inputs.py) | 2 |
| [v3/composition/mission_navigation.py](../v3/composition/mission_navigation.py) | 1 |
| [v3/composition/motor_output.py](../v3/composition/motor_output.py) | 0 |
| [v3/composition/native_control.py](../v3/composition/native_control.py) | 38 |
| [v3/composition/native_sensor_inputs.py](../v3/composition/native_sensor_inputs.py) | 3 |
| [v3/composition/resident_live_control.py](../v3/composition/resident_live_control.py) | 42 |
| [v3/composition/resident_physical_control.py](../v3/composition/resident_physical_control.py) | 2 |
| [v3/composition/runtime_config.py](../v3/composition/runtime_config.py) | 28 |
| [v3/composition/stop_only.py](../v3/composition/stop_only.py) | 2 |
| [v3/config.py](../v3/config.py) | 63 |
| [v3/config_hardware.py](../v3/config_hardware.py) | 53 |
| [v3/config_types.py](../v3/config_types.py) | 18 |
| [v3/contracts/__init__.py](../v3/contracts/__init__.py) | 0 |
| [v3/contracts/async_runtime.py](../v3/contracts/async_runtime.py) | 13 |
| [v3/contracts/base.py](../v3/contracts/base.py) | 3 |
| [v3/contracts/lidar.py](../v3/contracts/lidar.py) | 0 |
| [v3/contracts/localization.py](../v3/contracts/localization.py) | 7 |
| [v3/contracts/messages.py](../v3/contracts/messages.py) | 81 |
| [v3/contracts/planner.py](../v3/contracts/planner.py) | 2 |
| [v3/contracts/temporal.py](../v3/contracts/temporal.py) | 49 |
| [v3/control_cli.py](../v3/control_cli.py) | 58 |
| [v3/device_health_policy.py](../v3/device_health_policy.py) | 0 |
| [v3/diag/__init__.py](../v3/diag/__init__.py) | 0 |
| [v3/diag/__main__.py](../v3/diag/__main__.py) | 0 |
| [v3/diagnostic_contracts.py](../v3/diagnostic_contracts.py) | 10 |
| [v3/engine.py](../v3/engine.py) | 20 |
| [v3/execution.py](../v3/execution.py) | 2 |
| [v3/external_gateway.py](../v3/external_gateway.py) | 10 |
| [v3/finite_navigation.py](../v3/finite_navigation.py) | 38 |
| [v3/host_cli.py](../v3/host_cli.py) | 12 |
| [v3/hri_evidence.py](../v3/hri_evidence.py) | 35 |
| [v3/import_guard.py](../v3/import_guard.py) | 0 |
| [v3/interface_adapters.py](../v3/interface_adapters.py) | 0 |
| [v3/interface_cli.py](../v3/interface_cli.py) | 41 |
| [v3/launcher_cli.py](../v3/launcher_cli.py) | 5 |
| [v3/launcher_extras.py](../v3/launcher_extras.py) | 0 |
| [v3/layers/__init__.py](../v3/layers/__init__.py) | 0 |
| [v3/layers/l10_chassis_control.py](../v3/layers/l10_chassis_control.py) | 2 |
| [v3/layers/l11_actuator_control.py](../v3/layers/l11_actuator_control.py) | 140 |
| [v3/layers/l12_safety_final.py](../v3/layers/l12_safety_final.py) | 13 |
| [v3/layers/l1_acquisition.py](../v3/layers/l1_acquisition.py) | 0 |
| [v3/layers/l2_admission.py](../v3/layers/l2_admission.py) | 26 |
| [v3/layers/l3_state_estimation.py](../v3/layers/l3_state_estimation.py) | 210 |
| [v3/layers/l4_structural_memory.py](../v3/layers/l4_structural_memory.py) | 47 |
| [v3/layers/l4_temporal_history.py](../v3/layers/l4_temporal_history.py) | 35 |
| [v3/layers/l4_temporal_occupancy.py](../v3/layers/l4_temporal_occupancy.py) | 31 |
| [v3/layers/l4_temporal_tracking.py](../v3/layers/l4_temporal_tracking.py) | 68 |
| [v3/layers/l4_world_model.py](../v3/layers/l4_world_model.py) | 152 |
| [v3/layers/l5_command_mission.py](../v3/layers/l5_command_mission.py) | 0 |
| [v3/layers/l6_navigation.py](../v3/layers/l6_navigation.py) | 351 |
| [v3/layers/l7_motion_selection.py](../v3/layers/l7_motion_selection.py) | 17 |
| [v3/layers/l8_motion_realization.py](../v3/layers/l8_motion_realization.py) | 23 |
| [v3/layers/l9_operational_constraints.py](../v3/layers/l9_operational_constraints.py) | 26 |
| [v3/lidar_config.py](../v3/lidar_config.py) | 12 |
| [v3/lidar_estimator.py](../v3/lidar_estimator.py) | 86 |
| [v3/lidar_matcher_process.py](../v3/lidar_matcher_process.py) | 66 |
| [v3/lidar_relative_odometry.py](../v3/lidar_relative_odometry.py) | 10 |
| [v3/mcap_capture.py](../v3/mcap_capture.py) | 207 |
| [v3/mcap_inspect.py](../v3/mcap_inspect.py) | 0 |
| [v3/mcap_reader.py](../v3/mcap_reader.py) | 28 |
| [v3/mcap_replay_bridge.py](../v3/mcap_replay_bridge.py) | 29 |
| [v3/mcap_writer.py](../v3/mcap_writer.py) | 22 |
| [v3/observation.py](../v3/observation.py) | 16 |
| [v3/operator_cli.py](../v3/operator_cli.py) | 3 |
| [v3/operator_controller.py](../v3/operator_controller.py) | 162 |
| [v3/ports.py](../v3/ports.py) | 0 |
| [v3/process_sidecars.py](../v3/process_sidecars.py) | 34 |
| [v3/pytest_profiles.py](../v3/pytest_profiles.py) | 1 |
| [v3/replay.py](../v3/replay.py) | 38 |
| [v3/resident_status.py](../v3/resident_status.py) | 4 |
| [v3/robot_interface.py](../v3/robot_interface.py) | 0 |
| [v3/runtime_performance.py](../v3/runtime_performance.py) | 107 |
| [v3/scan_matching.py](../v3/scan_matching.py) | 31 |
| [v3/test_runner.py](../v3/test_runner.py) | 4 |
| [v3/wheel_motion.py](../v3/wheel_motion.py) | 0 |
| [v3_bounded_config.py](../v3_bounded_config.py) | 0 |
| [v3_bounded_runtime.py](../v3_bounded_runtime.py) | 33 |
| [v3_external_gateway.py](../v3_external_gateway.py) | 3 |
| [v3_hardware_runtime.py](../v3_hardware_runtime.py) | 86 |
| [v3_process_runtime.py](../v3_process_runtime.py) | 35 |
| [v3_runtime.py](../v3_runtime.py) | 101 |
