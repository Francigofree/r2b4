# Az r launcher és az ER2 architekturális vizsgálata

2026-09-26. Source-first állapotfelmérés és a jelen változtatás eredménye.

A robotparancsok fő útvonala canonical, de a vizsgálat kezdetén négy P0
hibacsoport sértette a host/session megszakítási vagy hardware-ownership
garanciákat. Ezek javítva vannak. A fennmaradó P1/P2 tételek miatt a teljes
felületre nem állítható korlátlan lifecycle/session megfelelőség.

Az authority a [strukturális contract](../STRUKTURALIS_RETEGEK_V3.md), különösen
a 2., 9., 10. és 12. pont, valamint az
[async contract](../ASZINKRON_RUNTIME_CONTRACT_V3.md). A javítás ezeket nem
módosítja. Nem keletkezett új robotréteg, általános session/IPC framework vagy
motor-authority.

**Vizsgált útvonalak és tulajdonosok**

Az [r](../r) a repo rootot feloldó shell bootstrap; a
[launcher_cli.main](../v3/launcher_cli.py) sorrendje: help → commands → er2 →
host helper → interface CLI. A parserből származtatott 19 robotparancs és 20
alias nem ütközik a host parancsokkal. Az aliasok ugyanazokra a canonical
parancsokra oldódnak fel.

| Felület | Tényleges forrásút | Authority / lényeg |
| --- | --- | --- |
| help, commands | launcher → interface parser / action catalog / test runner | Felfedezés, nincs robotállapot-tulajdon. A szöveges commands kimenetből az ER2 még hiányzik. |
| status, diag, caps | interface CLI → RobotInterface → adapter | Runtime/host observation. A v3.status friss live statust kér; az operator.status nyers korábbi statust is tartalmazhat. |
| forward, backward, teleop, wheels, roomcruise, faceperson, followperson és aliasaik | interface CLI → RobotInterface → V3ControlInterfaceAdapter → OperatorController → control_cli | A kerékcél a gateway előtt chassis v/omega céllá alakul; nincs közvetlen L10/L11/PWM injektálás. |
| stop/x | RobotInterface.stop → V3ControlInterfaceAdapter → OperatorController.stop → control_cli STOP | Producerleállítás és canonical STOP, majd runtime IDLE/inaktív output visszaigazolás. |
| shutdown/sd, panic, runtime/rt | OperatorInterfaceAdapter → OperatorController | Host process lifecycle; a production lifecycle és final safety továbbra is a runtime-é. |
| proba/pr | OperatorInterfaceAdapter → run_proba → canonical control_cli producerek | Többfázisú host művelet. A P0 javítás minden hívó számára visszavonhatóvá teszi. |
| capture/cap | OperatorInterfaceAdapter → capture_start/stop | Observation orchestration; konfigurációváltás/re-arm esetén jelenleg runtime restart is történhet. |
| testhub/th | TestHubInterfaceAdapter → test_hub_next | Passzív, offline evidence/replay. Nincs pozitív actuation-visszaút. |
| camera/cam | CameraInterfaceAdapter → camera_media | Exkluzív diagnosztika: meglévő operator lock + runtime-ownership ellenőrzés. |
| system/sys | SystemInterfaceAdapter | Hostadatok. |
| er2 status | ER2 CLI → RobotInterface read + VisionMediaClient | Provider-konfiguráció és tényleges JPEG-probe; nem indít runtime-ot. A probe HRI eseményt írhat. |
| er2 preview | PreviewClient → Gemini Interactions | Alapesetben observation; --tools engedélyezi a szűk robot tool felületet; --camera szükség esetén runtime-ot indít. |
| er2 stream | StreamingClient → Gemini Live → Er2RobotTools | Host provider-session; physical tool futás kizárólag a gateway/facade útján. |
| --speak | Er2SpeechReporter → GeminiTtsClient → PcmWavePlayer | Host audio, nem robot control. |
| test, pytest/tests | host_cli → test_runner / raw pytest | A curated teszt authority a test_runner; a raw pytest fejlesztői passthrough. |
| git, gitre/gittre, tools, tool/script | host_cli → fejlesztői subprocess | Megbízható fejlesztői felület, nem sandbox és nem külső AI capability-lista. |
| cpu/cpu2, disc/disk, mem/memory, temp/temperature, ps/processes, net/network, usb, i2c, host | host_cli | Host diagnosztika; i2c és a közvetlen hardveres toolok foglaltsági védelemmel. |
| install, where, root, version | host_cli | Launcher telepítés és repo-metaadatok. |

Az ER2 fizikai parancs teljes útja:

```text
Gemini function call
  → Er2RobotTools: provider-specifikus argumentum- és szegmenskorlát
  → ExternalRobotGateway: végrehajtási policy, session owner/watchdog
  → RobotInterface: capability owner feloldása
  → V3ControlInterfaceAdapter / OperatorController: host orchestration
  → v3.control_cli / ResidentCommandClient: egyetlen heartbeat/revision/TTL owner
  → AsyncResidentCommandGateway: trust, revision, TTL, process limitek
  → ResidentLiveControlComposition: immutable TickInputs closure
  → TickEngine L5 → … → L12 → egyetlen normál MotorWriter
```

A [RobotInterface](../v3/robot_interface.py) host facade, az
[ExternalRobotGateway](../v3/external_gateway.py) host policy adapter. Egyik sem
azonos a production [resident CommandGateway](../v3/adapters/resident_command.py)
authorityval. A host oldali dict/JSON önmagában nem production réteghatár-sértés:
a command snapshot a resident edge-en válik typed CommandRequestté, majd
[TickInputs részévé](../v3/composition/resident_live_control.py).

A production wiringot a [v3_process_runtime](../v3_process_runtime.py) alapján
ellenőriztem: AsyncResidentCommandGateway, külön processzes status/capture,
majd a resident composition. A vizsgált production layer/composition/gateway
forrásokban nincs visszaimport a launcher, OperatorController, RobotInterface,
ExternalRobotGateway vagy ER2 implementáció felé.

A [process vision owner](../v3/adapters/process_vision_port.py) indítja a
[VisionMediaServert](../v3/adapters/vision_media_socket.py). JPEG:
vision process → privát Unix socket → ER2 host process. A nagy képpayload nem
megy vissza a control interpreterbe. Az ER2
[HRI evidence](../r2b4_er2/evidence.py) produceroldali, best-effort observation;
nem alakít L12 döntést. A media transport ugyanakkor nem visz frame sequence-et
vagy measurement timestampet: ez a lent jelzett freshness-adósság.

A voice Gemini/Groq ág külön host composition:
[build_voice_interface](../r2b4_voice/conversation_interface.py) →
ConversationService → strukturált javaslat →
[VoiceActionExecutor](../r2b4_voice/action_executor.py) → ugyanaz a
RobotInterface. Ez nem közös ER2 provider-session és nem L0–L12 réteg.
A voice és ER2 session-policy részben párhuzamos, de a command heartbeat
mindkettőnél a canonical control_cli felelőssége.

**Aktív konfiguráció, forrásból ellenőrizve**

- [conf/vezerles.json](../conf/vezerles.json): ingress maximum TTL 250 ms,
  maximum v 0,5 m/s, maximum omega 1,2 rad/s, maximum mailbox 4096 byte,
  reader poll 5 ms. A control_cli alapértelmezett heartbeatje 100 ms,
  a publikált TTL 200 ms; ezek illeszkednek a resident korlátokhoz.
- [conf/hardver.json](../conf/hardver.json): kamera és person detection
  engedélyezett; lores 640×360, 20 fps. Ez konfiguráció, nem live readiness-bizonyíték.
- [Er2Config](../r2b4_er2/config.py): alapértékek 0,2 m/s, 0,6 rad/s,
  1,5 s szegmens, 10 s producer-watchdog, 1 s provider heartbeat,
  1,25 s media socket timeout. A két heartbeat eltérő szerepű.
- requirements és a telepített SDK egyezett: google-genai 2.25.0.
  A GEMINI_API_KEY jelenléte ellenőrizve; értékét nem naplóztam.
  Az ER2 environmentből olvas, a voice ezen felül conf/.wake.env fallbacket is használ.

**P0 hibák és implementált javítások**

P0 itt: STOP utáni újraaktiválás, megszakítás után gazdátlan fizikai munka,
elrejtett STOP-hiba, illetve a robot hardveres ownerének megkerülése.

| ID | Konkrét eredeti hiba / kiváltó esemény | Javítás és forrás |
| --- | --- | --- |
| P0-1 | `r proba`/`pr` két fázisa közötti szünetben STOP: a producer-killer csak `v3.operator_cli proba` argv-t ismert, miközben az új launcher más argv-val fut. A teljes sequence lockja alatt a következő fázis ismét ACTIVE-ot publikálhatott. A phase producernek owner/watchdog sem volt megadva. | [OperatorController](../v3/operator_controller.py): a meglévő sequence marker PID + invocation identityt tartalmaz; STOP visszavonja. Phase start előtt és a monitor loopban ellenőrzés; csak rövid transition tart lockot. Régi sequence cleanup nem állíthat le új tulajdonost. A phase producer megkapja a PID-t és 35 s watchdogot a meglévő control_cli úton. |
| P0-2 | Streaming timer/cancel/kapcsolati cleanup törölte a `to_thread` awaiterét, de a robot_drive thread tovább futott. Startup után késői ACTIVE, reconnectkel átfedő drive vagy késői STOP következhetett. | [StreamingClient._execute_tool](../r2b4_er2/streaming.py) + [Er2RobotTools](../r2b4_er2/tool_bridge.py): cooperative cancellation, azonnali canonical STOP-kérés, shieldelt worker megvárása; a későn visszatérő startup után is final STOP. A cleanup/reconnect nem hagy hátra futó fizikai toolt. |
| P0-3 | A gateway STOP-hibát FAULTED válaszként adott vissza, amit a bridge figyelmen kívül hagyott. A drive COMPLETED, a stream stopped_cleanly eredményt adhatott. A provider és CLI cleanup kivételeket is lenyelt. | `robot_stop` sikertelen válasza Er2SafetyError; Preview és Streaming ezt nem alakítja újabb modellfordulóvá/retryvá. A cleanup nem nyeli el. A drive setup/execute is a final STOP védelme alatt van. |
| P0-4 | `r tool v3_process_runtime --approval native-resident-v3` közvetlen resident belépés volt az operator lifecycle nélkül. A tool resolver útvonalat is elfogadott; több közvetlen perifériadiagnosztika kimaradt a hardware guardból. | [host_cli](../v3/host_cli.py): a resident belépési pont ezen az ágon elutasított, canonical alternatíva `r runtime start`; tool név és feloldott root/tools útvonal ellenőrzés; meglévő guard kiterjesztése IMU/periféria/rootcause/MCAP diagnosztikákra. |

A sequence marker változása meglévő host állapot szűk javítása. Nincs új
authority vagy általános foglalási rendszer. A PID marad az első sorban;
a régi PID-only operator sequence leállítása továbbra is támogatott.

**Kapcsolódó, szűk P1 javítások ugyanebben a változtatásban**

- A telepített `google.genai.live.AsyncSession.receive` forrása szerint egy
  teljes modellforduló végén az iterator lezárul. A korábbi ER2 kód minden ilyen
  lezárás után reconnectelt és elfogyasztotta az ötös reconnect-budgetet.
  Most ugyanazon sessionben új receive-turn következik. A provider dokumentáció
  is külön kezeli a session folytatását és a forduló végét:
  [Google Live session management](https://ai.google.dev/gemini-api/docs/live-api/session-management).
- Normál turn-vég és valódi kapcsolatvesztés megkülönböztetése; task-kivétel
  kiolvasása retry előtt. Érvénytelen resumption update törli a handle-t;
  új `run_async` nem örököl korábbi feladatból handle-t. Elveszett session
  érvényes handle nélkül terminálisan leáll, nem kap feladat nélküli új sessiont.
- Egy provider tool-batchből legfeljebb egy robot_drive kísérlet indul.
  Preview esetén a hibás első próbálkozás is elfogyasztja ezt a lehetőséget.
- `--seconds` NaN/Inf/bool érték elutasított a streaming client határán.
- Eszköz nélküli Preview végén megszűnt a más kliens robotját leállító,
  feltétel nélküli STOP. A saját maga által elindított runtime cleanupja megmaradt.

**Fennmaradó P1 hibák és minimális javítási terv**

| ID | Bizonyított forráshely és következmény | Következő szűk változtatás / elfogadási feltétel |
| --- | --- | --- |
| P1-1 | [ER2 CLI _RuntimeLease](../r2b4_er2/cli.py) és [interface CLI _run_timed_motion](../v3/interface_cli.py): előzetes runtime_running booleanból következtetnek tulajdonra. A read/start nem egy transition; close nem ellenőrzi ugyanazt a runtime-példányt. Preview --tools kamera nélkül a toolban indíthat runtime-ot, amit a CLI nem tart sajátként nyilván. | A meglévő operator start-válaszban actual started/PID információ, ugyanazon transitionből; cleanup a ténylegesen indított processzre. Két kliens start/stop/replacement tesztje. Nem új lease framework. |
| P1-2 | [ensure_runtime és _ensure_fresh_capture_slot](../v3/operator_controller.py): capture mode/Hz eltérés és felhasznált ALAP slot runtime restartot okozhat. Az interface CLI örökli ugyan a módot/Hz-t, de a használt slot újraélesítése még újraindíthatja a preexisting runtime-ot. | Reuse esetben a capture igény ne implicit módon változtassa meg más kliens runtime-ját; explicit lifecycle művelethez kötött re-arm. Voice, CLI, ER2 ugyanazon adapterviselkedését tesztelni. |
| P1-3 | [robot_drive](../r2b4_er2/tool_bridge.py): duration mérése csak az execute/ALLOW után indul; STOP költsége sincs a szegmensidőben. Startup alatt már lehet output, a producer-watchdog külön, alapból 10 s. A deklarált 1,5 s nem bizonyított teljes actuation-időkorlát. | Az existing command producer időkeretéhez kötni a szegmens lejáratát, az időorigint explicit rögzíteni. Startup-késés és host-stall teszt, majd engedélyezett motoros mérés. |
| P1-4 | [StreamingClient.run_async](../r2b4_er2/streaming.py): timer csak connect, initial frame és initial send után indul; a hálózati/turn-wait teljes deadline nincs végigvezetve. Preview tool-round korlát van, teljes request-deadline nincs. | Provider-specifikus connect/send/turn/összművelet timeout; a meglévő STOP + worker-drain cleanup használata. Elakadt connect/media/provider tesztek. |
| P1-5 | [Preview/Streaming tool dispatch](../r2b4_er2/preview.py): function-call ID visszaadása megtörténik, de nincs fizikai művelet-ismétlésvédelem. A Live tool_call_cancellation külön üzenetét a kliens nem dolgozza fel; blocking drive alatt nem olvas következő üzenetet. | Bounded, provider-sessionhez kötött call-ID kezelés, explicit cancellation policy. Elveszett tool-response/resumption teszt és provider-protokoll verifikáció; a tényleges szerveroldali újraküldés ebben a vizsgálatban nem volt megfigyelve. |
| P1-6 | [OperatorController._wait_allow/_wait_ready](../v3/operator_controller.py) és [control_cli._active_preflight](../v3/control_cli.py): egyes gate-ek nyers statust/tick-növekedést olvasnak, nem egységes freshness + command/mission korrelációt. A live_runtime_status ellenőriz kort, de nem bizonyít process identityt. | A meglévő fresh status helper használata, ALLOW visszaigazolás kötése a beküldött commandhoz. Stale és régi runtime-status teszt. L12 truth marad authoritative. |
| P1-7 | [VisionMediaServer](../v3/adapters/vision_media_socket.py): OK+méret+JPEG protokoll, frame identity/measurement-idő nélkül; az alap socket csak UID alapján névterezett. Két checkout vagy régi worker összetéveszthető. | Capability-specifikus kis metadata + root/process azonosítás a media edge-en; raw JPEG továbbra is közvetlen producer → kliens. Frissesség/mismatched-owner teszt. |

Ezek megvalósítási sorrendje: P1-1/2 ownership → P1-3/4 időkorlátok →
P1-5 session/protokoll → P1-6/7 identity/freshness. Minden lépés külön,
adapterre szűkített módosítás; production contract változtatása csak tényleges
authority-változás esetén indokolt.

**P2 technikai adósság**

| Tétel | Forrás / hatás | Kis kockázatú korszerűsítés |
| --- | --- | --- |
| Katalógus és CLI drift | launcher `_commands` plain output kihagyja ER2-t; régi operator_cli follow defaultja 0,15, az action catalogé 0,25; interface CLI docstringje még teljes runtime ownershipöt ígér. | Meglévő action catalog/parser alapértékeinek újrahasználata, help/docstring helyesbítése. |
| Config és provider lifecycle | ER2 csak env, voice env + .wake.env; saját SDK clientek lezárása nincs explicit minden ágon. Provider preflight a runtime indítás után is történhet. | Kis közös host config helper, korai offline validáció, saját client explicit close. Titkok nem kerülhetnek status/evidence kimenetbe. |
| Gateway policy túl általános | ExternalRobotGateway allow_execute az összes facade actionre kiterjed; watchdog-injektálás csak a catalog session_watchdog actionjeire. Az ER2 szűk tool surface-e ezt korlátozza, a generikus JSONL gateway önmagában nem. | Product/remote használat előtt az engedélyezett capability scope pontosítása a meglévő catalogból; STOP továbbra is külön fail-safe művelet. |
| Eltérő diagnosztikai wiring | A tools/v3_mcap_measurement.py és tools/r2b4_control_rootcause_diag.py a továbbra is létező AsyncResidentStatusPublisher/McapCaptureSession utat használja; a production main ProcessResidentStatusPublisher/ProcessMcapCaptureSession utat. Az előbbi mérése nem bizonyítja automatikusan az utóbbi izolációját/timingját. | A mérési scope pontosítása, szükség esetén az adott diagnosztika illesztése a tényleges production wiringhoz. |
| Evidence korlát | HRI append best-effort; request ID, provider call ID és command identity nem minden eseményben kapcsolható össze. | Meglévő eseménymezők pontosítása; nincs szükség új audit/receipt/trace frameworkre. |

**Validáció és korlátok**

- CORE: `./r test` — 29 passed.
- ER2 célzott: `./r pytest -q tests/feature/test_er2_p0r2.py` — 10 passed.
- Offline media/tool/provider adapter smoke: `python3 tools/validate_er2_p0.py` — PASS.
- Teljes curated regresszió: `./r test full` — 97 passed.
- Routing catalog: 19 canonical robotparancs, 20 alias, nincs host/robot névütközés.
- `git diff --check` — tiszta.

A regressziók többek között valódi sequence/phase vezérlést futtatnak fake
hardware edge-ekkel; külön host kliens STOP-ját injektálják a fázisközi
szünetbe; owner-exit/watchdog lejáratot ellenőriznek; későn visszatérő fizikai
tool startupot szakítanak meg cancel és timer útján; STOP-hibánál kizárják a
sikeres provider-választ és retryt; két modellfordulót ugyanazon kapcsolaton
futtatnak. Ezek host/adapter szemantikai bizonyítékok.

Nem indítottam robotmozgást, runtime-ot, live provider-hívást vagy capture-replayt.
Meglévő runtime/capture/log adatot nem írtam át, másoltam, archiváltam vagy
töröltem. A párhuzamos live processzekhez nem nyúltam. Nem bizonyított a fizikai
STOP-latencia, a hardveres inaktivitás, a terhelt runtime jittere, a valódi Gemini
reconnect/cancellation/idempotencia és a hálózati end-to-end határidő.
A production TickInputs és L0–L12 implementáció változatlan; releváns új
decision-input capture hiányában az offline fake provider teszt nem nevezhető
canonical replay MATCH-nek. Commit és push nem készült.
