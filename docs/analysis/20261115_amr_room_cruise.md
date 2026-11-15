# Room Cruise: célfolytonosság és blokkoló napló-I/O

2026-11-15. A szoftveres javulás szimulátorban mérhető. Az élő elfogadás
**nyitott**: az első kért 60 s-os próba LiDAR-startup hibával, még a ready
állapot és a behavior-végrehajtás előtt leállt. Nem történt automatikus újrapróba.

## Terv és megvalósítás

1. A jelenlegi production út és a legutóbbi capture vizsgálata, a különböző
   megállási okok szétválasztásával.
2. A köztes COVERAGE-szektorcélok végső célként kezeléséből eredő
   lassítás/STOP/újraindulás megszüntetése a meglévő Nav2 action útján.
3. A méréssel azonosított szinkron megfigyelési fájlírások kivétele az
   időkritikus végrehajtásból; az érvényességi és safety-boundok megőrzése.
4. Célzott tesztek, release gate, páros szimulált összehasonlítás,
   bútorozott ellenőrzés, natív replay és explicit MCAP Evidence Compiler.
5. Két 60 s-os élő RC-próba. Ez a lépés az első startup hiba miatt nem teljesült.

Az L5 mission, L6 admission, L7 objective, L9 gyorsuláskorlát és L12 final
authority nem változott. A 200 ms-os parancs-, 250 ms-os szenzorfrissességi
határ, a fizikai kerékhatárok, a footprint és a 0,10 m test-clearance megmaradt.

### Korai szektorcélváltás

Forrás: `v3/adapters/room_cruise.py`, `v3/nav2_navigation_bridge.py`.
Korábban minden COVERAGE-szektor külön véges `NavigateToPose` volt. Siker után
`finish_navigation` PENDING-et küldött, majd új action indult. Az MPPI a célhoz
közeledve a 0,15 m/s fizikai kerékminimum alá is lassíthatott; a projekció ilyenkor
helyesen nullázott. A downstream gyorsuláskorlát ezt a hiányzó mozgáskérést nem
tudta folytonossá tenni.

Most csak COVERAGE alatt, friss Nav2 remaining-path feedback és legfeljebb
1 m euklideszi céltávolság mellett választható következő szektor. A feedback
legfeljebb 250 ms-os lehet; a friss globális costmap teljes testre végzett
célvizsgálata megmarad. Az új cél legalább 1,4 m-re van a robottól és 1 m-re
a régi céltól. A kiválasztás bounded, ütemezése a meglévő selectoré.

A bridge a meglévő native Nav2 preemptiont használja, CANCEL és mesterséges
PENDING nélkül. Az elfogadásig a régi action tulajdonosa marad. Elutasításkor
annak valódi eredménye érvényes; elfogadáskor az új handle/result future váltja
fel. A régi késői completion nem fejezheti be az új child-goalt. A pending
elfogadás 2 s-os boundot kapott; CANCEL és heartbeat-lejárat a késői acceptet
is visszavonja. Korai lecserélés nem növeli a fizikailag elért célok számlálóját:
külön `coverage_handoffs` evidence méri.

Csak az eredeti lejáratig maradhat meg egy már elfogadott parancs. A friss nulla
parancsot a váltás sem simítja el. Régebbi ROS-parancs nem írhat felül újabbat;
az új action elfogadása előtti pozitív parancs nem szerezheti meg az új cél
lineage-ét. FRONTIER, normál NAVIGATE és recovery nem kap korai célváltást.

### Két bizonyított fájl-I/O boundary

* A javítás nélküli szimulátor 1627. tickjében a truth timeline fájlírása
  **561,844 ms wall / 0,086 ms thread CPU** volt. A következő tickben a változatlan
  pacing-bound lezárt. Ez konkrét szimulátor-megfigyelési hiba, nem L0–L12 compute.
* A controller `45419` szála a műszerezett menetben, epoch
  `1794780148.329414`-kor kezdett 83 bájtos `write(2, ...)` hívása
  **877,969 ms**-ig tartott. A fd 2 a supervisor által megnyitott
  `controller_server.log`; a környezetben a „Passing new path” INFO-sorok állnak.
  Egy másik logging write 240,135 ms volt. Ez a native controller közvetlen
  naplóútjának blokkolását bizonyítja, nem Python GIL-terhelést.

A véges szimulátor truth-idővonala most a már korlátozott időtartamból és
50 Hz-ből számolt kapacitású memóriában marad; JSON-encode és fájlírás csak
a virtuális STOP és a ROS children lezárása után történik. Overflow explicit
hiba, nincs adateldobás. A cleanup egyetlen reap-timeout után is lezárja a többi
csoportot és folytatja a capture véglegesítését, miközben a hibát megtartja.

A teljes native Nav2 indítás `--disable-stdout-logs --disable-external-lib-logs`
argumentumot kapott. A meglévő `/rosout` → `Nav2Diagnostics` hibagyűjtés és
observation/capture megmarad. Nem került be új logger-processz vagy IPC.
A planner-only mód eredeti naplózása megmarad; ez a végső szűkítés a mért
`full=True` indítás parancslistáját nem változtatta meg.
A [ROS elsődleges dokumentációja](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Logging.rst)
írja le a három külön logcélt és e két kapcsolót.

A külön, telített stderr-csöves próbában az alap ROS warning blokkolt;
a production `_commands` által adott kapcsolókkal a hívás visszatért, és a
`/rosout` publisher jelen volt. Ez a node inicializálása utáni loghívást méri;
a globális RCL startup-warningok nem tartoznak ehhez a garanciához.
Evidence: `tmp/amr_20261115/log_sink_probe.{py,json}`.

## Szimulált eredmény

Production V3 + valódi SLAM Toolbox/Nav2, virtuális motor sink. Azonos nyílt
szoba, seed 7, 60 s aktív mérési ablak. Az alap már tartalmazza a timeline-I/O
javítását, a robotikai célfolytonossági és Nav2-log változtatást még nem.
A ROS-ütemezés nem determinisztikus; ez mért összehasonlítás, nem garantált rangsor
minden környezetre. A truth minőségi mutatók az első mozgástól, 50 Hz-en készülnek;
az állásküszöb `abs(v)<0,02 m/s && abs(omega)<0,1 rad/s`.

| Mutató | Alap | Végleges |
|---|---:|---:|
| Elindulás előtti idő | 3,56 s | 2,50 s |
| Elindulás utáni megállások | 11 | **1** |
| Elindulás utáni állóidő | 3,64 s / 6,45% | **0,20 s / 0,35%** |
| Lineáris jerk RMS | 10,892 m/s³ | **6,033 m/s³** |
| Szögjerk RMS | 25,656 rad/s³ | **10,985 rad/s³** |
| Sebesség szórása | 0,1945 m/s | **0,1455 m/s** |
| Út | 16,065 m | **20,289 m** |
| Truth lefedettség, 0,25 m cellák | 63 | 82 |
| Minimum test-clearance | 0,276 m | 0,479 m |
| Ütközés | 0 | 0 |
| Natív replay | 3205 tick MATCH | **3203 tick MATCH** |

A végleges menetben 10 elfogadott korai célváltás volt, 0 recovery-epizód,
és az első mozgást követő mind a 2875 tick NAV2_ACTIVE. A teljes menet indulási
jerk-maximuma érdemben nem csökkent; a peak-jerk javulását nem állítjuk.

A külön bútorozott, 30 s-os ellenőrzés **PASS**, 1743 tick **MATCH**, nulla
ütközés, 0,362 m minimum clearance, 3 handoff, 0 recovery. Az első mozgástól
18,48 s mérhető haladásban nincs megállás. A kezdeti várakozás **11,5 s** volt;
ez a próba nem igazol gyors hidegindítást és nem páros bútorozott A/B.

Reprodukció és eredmények:

* `tmp/amr_20261115/measure_quality.py` — a teljes truth-idővonalból számol;
  a capture-mód külön L3-becslésként, nem fizikai truth-ként címkéz.
* `tmp/amr_20261115_sim_buffered_baseline/run/result.json` és
  `tmp/amr_20261115/baseline_quality.json`.
* `tmp/amr_20261115_sim_accept/run/result.json` és
  `tmp/amr_20261115/final_sim_quality.json`.
* `tmp/amr_20261115_sim_furnished/run/result.json` és
  `tmp/amr_20261115/furnished_quality.json`.
* `./r sim simulate --scenario explore --duration-s 60 --ros-domain-id 181
  --output tmp/EGYEDI_CEL`.

A simulator source-snapshot immár a külön ROS bridge-eket, a room-cruise
selectort/recoveryt és a frontier adaptert is hash-eli; korábban a control import
closure ezeket nem fedte le. A korábbi alapfutás edge-forrásának authorityja a
feladat eleji tiszta `9bd226d` working tree; az új snapshot-teszt bizonyítja,
hogy e fájlok változása most már érvényteleníti az összehasonlítást.

## Validáció és negatív evidence

* Végleges `./r test release tests/packs/feature/test_room_cruise_coverage.py
  tests/packs/feature/test_local_navigation_simulator.py
  tests/packs/feature/test_nav2_lifecycle.py`: **84 passed**.
* Külön navigation/admission/recovery/coverage célzott kör: **68 passed**.
* A logbeállítás full-módra szűkítése után lifecycle pack: **20 passed**;
  végleges `./r test`: **14 passed**.
* `git diff --check`: PASS. A V3 contractok és az aktív geometria/tuning változatlan.
* A szimulációs MCAP explicit exportja: `tmp/amr_20261115_sim_evidence`.
  **COMPLETE**, 5239 üzenet, source integrity COMPLETE. Külön
  `r evi verify --source`: **PASS**, 0 quarantined, source_checked=true.
  A riportok `tmp/amr_20261115/sim_evidence_{compile,verify}.json` alatt vannak.

A hibás menetek megmaradtak, nem lettek PASS-ként elszámolva:

* `amr_20261115_sim_baseline_serial`: 29,74 aktív s után a fent leírt
  timeline-write pacing FAIL; 0 ütközés, replay MATCH.
* `amr_20261115_sim_continuation`: 40 aktív s, 5 handoff után bridge process-exit,
  `NAV2_LIFECYCLE_STATUS_STALE`. A recovery `spin_request` wall ideje 2,111 s,
  CPU-ja 0,321 ms volt; előtte FollowPath ACK-timeout látszott. Ennek teljes
  kernel/middleware eredete nem bizonyított. 0 ütközés, replay MATCH.
* `amr_20261115_sim_traced2`: 60 s PASS, replay MATCH; ebből származik a
  controller 878 ms-os log-write bizonyítéka. A syscall tracing befolyásolhatja
  a mért időket; a végleges, tracing nélküli menet külön szerepel a táblázatban.
* `amr_20261115_sim_final`: tracinges cleanup/reap timeout, BLOCKED,
  nem véglegesített capture. A saját beragadt diagnosztikai folyamatokat
  leállítottuk; ez indokolta a külön szimulátor-cleanup javítást.

## Élő indítás: nem elfogadott

Canonical parancs: `./r rc 60 c full c 50`.
Runtime-log: `tmp/amr_20261115/live_start.log`.
Hiba: `process-isolated LiDAR startup failed: RuntimeError:native V3 LiDAR owner did not start`.
A Nav2-előkészítés ezután `resident not ready` állapotot jelzett.

Capture: `runtime/captures/v3_20261115_231619_47216_capture.mcap`.
**0 tick, 0 raw LiDAR**, FAULT, incomplete; nincs replayelhető élő mozgásevidence.
A strict inspector ezt FAIL-ként jelzi, lásd
`tmp/amr_20261115/live_start_inspection.json`. A forrásban a generikus hiba
matcher-readiness/worker-exit vagy driver-start elutasítást fedhet; a logból
ezek nem választhatók szét. A jelen levő `/dev/ttyUSB0` nem bizonyítja az eszköz
üzemkészségét. Táp-, serial- vagy matcher-gyökérokot ezért nem állítunk.

Canonical shutdown után a runtime STOPPED. Az eredeti 5 Hz-es capture-beállítást
visszaállítottuk. Az AGENTS.md váratlan process-death/health eredmény utáni
automatikus újrapróba-tilalma miatt második élő futás nem indult.

A fizikai mozgás simasága, encoder/PI követése, hosszú üzem és az AMR-szerű
felhasználói élmény **nincs élőben igazolva**. A következő lépés a LiDAR-startup
okának tisztázása, majd külön engedélyezett 60 s-os RC-elfogadás. A safety gate
vagy a behavior-completion feltétel lazítása nem része a javításnak.
