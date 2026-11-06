# Room Cruise navigáció: 2026-11-03

A kért 120 másodperces élő elfogadás **nem teljesült**. Az első próbát 19,243 s
acknowledged navigáció után public status-hiba, a később külön engedélyezett,
végleges geometriájú próbát 14,680 s után heading-LOST zárta le. Mindkettő FULL
50 Hz-es capture-rel, canonical STOP-pal és `PASS / SHUTDOWN_SAFE_LOW`
runtime-zárással végződött. Mindkét teljes L1–L12 natív replay MATCH, integritás
és replay-ellenőrzés PASS; ezek nem jelentik a kért navigációs elfogadást.

## A futásban jelen levő javítás és tesztjei

Az L6 lokális céljelöltek endpoint-előszűrése korábban a téglalap köré írt körrel
is kizárt olyan folyosót, amelyben a robot téglalap alakú footprintje biztonságosan
elfér. A szűk javítás a teljes pivot/folyosó-bizonyítással rendelkező útvonalon
szükséges endpoint-feltételt használ; az ezt követő téglalapos swept-footprint
ellenőrzés, a 0,05 m ráhagyás, a lokalizációs infláció és az L12 változatlanul
érvényes. Source: [L6 rollout](../../v3/layers/_l6_rollout.py), célzott teszt:
[side clearance](../../tests/packs/feature/test_v3_navigation_side_clearance.py).

A live előtt `./r test`: **14 passed**; a célzott navigációs validáció:
`./r test tests/packs/feature/test_v3_navigation_side_clearance.py tests/packs/feature/test_v3_explore_worker_search.py`:
**22 passed**. A célzott teszt a biztonságos egyenes folyosó elfogadását és az
1 mm-rel a szükséges margin alatti folyosó elutasítását is ellenőrzi.

## Az első élő próba és az első hibás boundary

A canonical `room_cruise` skill `duration_s=120`, `capture_mode=full`,
`capture_hz=50` paraméterrel futott. Előtte friss, ready, disabled és nulla
motor-outputú STOP, GOOD lokális/heading állapot és a három kritikus eszköz OK
healthje volt szükséges. A host a szokásos **500 ms** status-freshness szabályt
követte. Nem indult automatikus újrapróbálás.

- Skill run: `4cd17c8e15894af6b6c94d7cbf769654`;
  behavior: `behavior-1-1318087301655`;
  mission: `mission-operator-roomcruise-1793740333700331412-4493`.
- Acknowledged execution start: **1318974361215 ns**.
- Az utolsó sikeresen olvasott public status a **679. tick**, mérési ideje
  **1337390403568 ns**; a host **455,09 ms** idősnek látta. L6:
  `IDLE / PLANNER_EXECUTION_HOLD`; L12: ALLOW, nulla motor-output.
- A következő public `v3.status` olvasás **1338216985410 ns**-nél
  `NO_LIVE_RUNTIME_STATUS` hibát adott, **19,243 s**-mal az execution start után.
  A capability a `live_runtime_status → status_is_fresh` útvonalon zár;
  [adapter](../../v3/adapters/v3_control.py),
  [operator](../../v3/operator_controller.py),
  [500 ms szabály](../../v3/resident_status.py).
- A natív capture eközben továbbhaladó productiont mutat: a **690., 695. és
  700. tick** már `ACTIVE / LOCAL_GUIDANCE_REVALIDATED`, nem nulla motor-outputtal.
  A 700. tick mindössze **124,17 ms**-mal előzte meg a host hibáját.
  A **705. tick** már canonical `STOP / NOT_ACTIVE`, disabled, nulla output;
  végső runtime-eredmény `PASS / SHUTDOWN_SAFE_LOW`, fault layer nincs.

Ez a public status elérhetőségi/frissességi boundarynél megfigyelt hiba;
nem 500 ms-os production tick-kiesés. A status-egress késésének konkrét
scheduler/IPC/encoding oka nem bizonyított. Az execution start utáni mentett
tickekben az encoder, IMU és LiDAR healthje végig OK. A lokális pozíció/heading
GOOD vagy DEGRADED maradt; a globális lokalizáció később LOST lett.

## Capture, replay és timing evidence

Authority capture:
[v3_20261103_221206_4415_capture.mcap](../../runtime/captures/v3_20261103_221206_4415_capture.mcap).
SHA256: `fee7d14c995a4f1d18a404028c92e697b03340cc5e7ba8abf69534527436f8f0`.
**946 tick**, ID 0–945; monoton idő **1315492492381–1343140636394 ns**.
FULL 50 Hz, sample/raw/replay evidence complete; required integrity-hiány nincs.

Explicit MCAP Evidence Compiler:
[/tmp evidence](/tmp/r2b4-roomcruise-20261103-221206.evidence),
`compiler_status=COMPLETE`, ugyanahhoz a source SHA256-hoz kötve.

Explicit natív replay az egész capture-re: **MATCH**, L1–L12, 946 tick,
mindkét futásban 946 offline motor-write, repeated trace match, első eltérés
nincs. `verify-result`: **PASS**, checksum OK. Eredmények:
[/tmp replay](/tmp/r2b4-roomcruise120-A8gI7T/replay_full_capture.json),
[/tmp ellenőrzés](/tmp/r2b4-roomcruise120-A8gI7T/replay_verify.json).
A capture source-manifest nélküli; historical source-hash azonosságra nincs
állítás. A replay a rögzített konfigurációra és erre a futásra érvényes.

| Natív session timing | Érték |
| --- | ---: |
| Célperiódus | 20 ms |
| Periódus átlag / p95 / p99 / maximum | 29,280 / 59,8 / 79,5 / 120,238 ms |
| Periódus >40 ms | 192 / 944 |
| Control wall p99 / maximum | 62,8 / 114,852 ms |
| L1–L12 pipeline wall / thread-CPU p99 | 62,5 / 24,5 ms |
| L6 wall p99 / maximum | 41,8 / 61,891 ms |
| Work >20 ms | 276 / 945 |

A wall-time nem választja szét a Python CPU-t, GIL-várást, process-transportot
és scheduler-késést. Az adatokból timing-javulás nem bizonyított.
A capture-ben az affinity disabled. A status source minden completed tickből
latest-only mailboxot frissít; a sidecar 10 ms-os idle poll után atomikusan,
tickenként a resolved konfigurációval együtt ír. FIFO-ban elakadt régi elemre
source-bizonyíték nincs. Az affinity bekapcsolásának javító hatása nem mért;
a status ekkor is osztozna a capture/planner számára kijelölt magokon.

Host preflight, terminal, utolsó státusz, compact megfigyelések és végső natív
timing riport: [/tmp futási könyvtár](/tmp/r2b4-roomcruise120-A8gI7T).
Kiegészítő natív tick-összesítés:
[semantic summary](/tmp/r2b4-roomcruise120-A8gI7T/native_semantic_summary.json).

## A live után pontosított fizikai geometria

A felhasználó a próba után **36 × 36 cm** testméretet adott meg; a hajtott
tengely a hátsó éltől **10 cm**, a LiDAR **9,5 cm**, az IMU **14,5 cm** távolságra
van. A tengelyorigóhoz képest a testközép **+8 cm**, a LiDAR **−0,5 cm**, az IMU
**+4,5 cm**. A 0,05 m navigation margin külön érték, nem a fizikai testméret része.

A fenti élő capture még a korábbi **0,46 × 0,38 m**, középre helyezett footprint
konfigurációval keletkezett, nulla footprint-center és LiDAR-offset mellett.
A test/tengely-geometria és LiDAR-extrinsic
helyesbítése **a live után** elkészült: `footprint_length_m=0.36`,
`footprint_width_m=0.36`, `footprint_center_x_m=0.08`,
`lidar_x_offset_m=-0.005`, `lidar_y_offset_m=0`. A téglalapos clearance és
pivot-sugár a tengelyhez képesti eltérést használja, a LiDAR pontok és
lokalizáció pedig a megadott mount-offsetet követik. Az IMU transzlációját a heading/yaw-rate adapter nem használja;
önmagában nem indokol yaw-korrekciót.

Végső motormentes validáció: `./r test release` **23 passed**;
config-célteszt **9 passed**; geometria **35 passed**; LiDAR **19 passed**;
a historical decoder-kiegészítés ellenőrzése **11 passed**.
A teljes historical capture új, végleges source melletti replay-je is **MATCH**,
`verify-result` **PASS**, változatlan 946 tick / L1–L12 scope-pal, eltérés nélkül:
[final-source replay](/tmp/r2b4-roomcruise120-A8gI7T/final_source_replay.json),
[ellenőrzés](/tmp/r2b4-roomcruise120-A8gI7T/final_source_replay_verify.json).
A decoder a historical konfiguráció hiányzó új geometria/extrinsic mezőit nullás
régi szemantikával tölti ki; nem cseréli le a capture méreteit a jelenlegi robotéra.
Ezek nem bizonyítják az új geometria élő helyességét.

## Külön engedélyezett új próba a végleges geometriával

A felhasználó új engedélye, a fenti tesztek és historical replay után a második
`room_cruise` próba már a **0,36 × 0,36 m / center +0,08 m / LiDAR −0,005 m**
konfigurációval futott. Skill: `9ac741c0858941e187a62517301fbf6d`, behavior:
`behavior-1-2043337788637`; acknowledged start **2044229784851 ns**.
Host-hiba **2058910161593 ns**: `LOCALIZATION_UNUSABLE`, **14,680 s** után.
Automatikus újrapróba ezt követően sem indult.

Az első hibás natív boundary **L3, tick 555 / 2058772083876 ns**:
heading `DEGRADED → LOST`, yaw sigma **0,446711 → 0,448534 rad**, így a
variancia **0,199551 → 0,201183 rad²**, átlépve a config **0,2 rad²** limitjét.
A [quality-besorolás](../../v3/layers/l3_state_estimation.py) ezt LOST-ra fordítja;
L6 ugyanebben a tickben `IDLE / LOCALIZATION_HOLD`, mindkét motor-output nulla.
L12 ALLOW mellett sem keletkezett pozitív mozgáskérés.

A folytonos lokális pozíció DEGRADED; a legutolsó elfogadott relative interval
LiDAR sequence **78**, vége **2054944081008 ns**, kora ekkor **3,828 s**.
Az encoder/IMU/LiDAR sample-ek kora **13,09 / 15,10 / 130,53 ms**, mindhárom
health OK. A friss device-sample tehát nem jelent friss, elfogadott relatív
lokalizációs korrekciót. A relatív korrekció kimaradásának konkrét upstream oka
ebből a vizsgálatból nem bizonyított; a quality-limit lazítása nem következik
az evidence-ből. A host utolsó státusza tick 559, yaw sigma **0,455355 rad**,
relative age **3,906 s**, L6 továbbra is zero-output hold.

Authority:
[v3_20261103_222411_6570_capture.mcap](../../runtime/captures/v3_20261103_222411_6570_capture.mcap),
SHA256 `49e3603daee71e958bb72383a8c5a43b54ea4090162c01a6d2343ef0139ba83e`.
**593 tick**, ID 0–592, **2041332301695–2059554617648 ns**;
sample/raw/replay complete. Teljes natív replay **MATCH**, ismételt trace-match,
első eltérés nincs, `verify-result` **PASS**:
[replay](/tmp/r2b4-roomcruise120-final-jh9f_up5/replay_full_capture.json),
[ellenőrzés](/tmp/r2b4-roomcruise120-final-jh9f_up5/replay_full_capture_verify.json),
[boundary evidence](/tmp/r2b4-roomcruise120-final-jh9f_up5/native_semantic_summary.json).

Második session timing: periódus átlag/p95/p99/max **30,825 / 58,3 / 75,6 /
83,358 ms**; control wall p99/max **55,2 / 66,688 ms**; L6 wall p99/max
**36,5 / 52,887 ms**; pipeline wall/thread-CPU p99 **54,8 / 19,8 ms**.
A két külön útvonalú, eltérő idejű próba nem kontrollált performance A/B mérés.

Nem bizonyított a 120 másodperces folyamatos navigáció, az ütközésmentesség
független fizikai méréssel, a sarkokból kiszabadulás, a stabil mozgásminőség vagy
az Atlasz-visszatalálás. Nyitott maradt a public status időleges elavulása és az
elfogadott relatív lokalizációs korrekciók kimaradása miatti heading-bizonytalanság;
a timing-jitter és a globális lokalizáció romlása is megmaradt megfigyelés.
