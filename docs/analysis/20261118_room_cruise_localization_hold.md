# Room Cruise: rövid localization HOLD után felesleges újraindulás

## Eredeti futás

Capture: `runtime/captures/v3_20261118_211922_43703_capture.mcap`.
Strict MCAP/integrity: PASS. Explicit Evidence Compiler: COMPLETE, 4760 message;
bundle- és source-verifikáció: PASS.
Evidence: `tmp/stop_20261118_211922/evidence`.

A navigation-decision eventek a mintavételezett tickfolyamnál pontosabban
megőrizték az állapotváltást. Az idő a 0. control tickhez relatív.

| Tick | Idő | Első releváns tény |
| --- | ---: | --- |
| 1318 | 26,376 s | L3 local translation GOOD, L6 NAV2_ACTIVE, motor engedélyezett |
| 1319 | 26,396 s | WHEEL_ENCODERS DEGRADED / ENCODER_COUNTER_DIAGNOSTIC; L3 local translation LOST; L6 NAV2_LOCALIZATION_HOLD; L12 STOP, nulla output |
| 1320 | 26,416 s | Device health ismét OK; L3 recovery még tart; nulla output |
| 1331 | 26,635 s | L3 local translation ismét GOOD; L6 NAV2_PENDING |
| 1337 | 26,756 s | NAV2_STALE |
| 1342 | 26,856 s | NAV2_PENDING |
| 1596 | 31,935 s | NAV2_ACTIVE, új child goal |

A minőségvesztés kb. 240 ms volt. Utána még 5,300 s kellett az ACTIVE
visszatéréséhez. A bridge observation szerint a korábbi COVERAGE állapot
FRONTIER-re váltott, a coverage számlálók nullázódtak; később újra COVERAGE indult.

Bizonyított source-lánc:

`accept_nav2_guidance` quality HOLD → `request=None` →
`Nav2NavigationClient.submit(None)` CANCEL → bridge `cruise.stop()` és
`recovery.reset()` → ugyanazon L5 mission új kérése új FRONTIER-kezdésként jelenik meg.

Az eredeti rövid encoder-diagnosztika konkrét rejection-kódja nincs meg a
1319. tick evidence-ében. A következő sampled tick BASELINE értéke nem azonos
a kiváltó hibával. A rövid safety STOP indokoltságát ezért nem lazítottuk.
Az ACTIVE ablakok control maximuma 29,913 ms, a period maximuma 40,524 ms:
a több másodperces állás nem több másodperces control-tick volt. Ez nem
azonosít GPIO-, GIL-, scheduler- vagy transport-okot.

Az eredeti felvétel 5 Hz-es system-behavior capture, raw input és checkpoint
nélkül. A canonical Replayer elutasította:
`CAPTURE_INCOMPLETE: replay core is incomplete`. E futásra nincs MATCH-állítás.

## Terv és megvalósítás

Csak `v3/layers/_l6_nav2.py` változott a production source-ban.

1. A frame-/continuity-vesztés továbbra is megszakítja a Nav2-kérést.
2. Már elindított, azonos mission/frame/odom-generation kérés átmeneti
   localization HOLD alatt fennmarad. A friss heartbeatet csak az aktív L5
   mission állítja elő; a bridge továbbra is csupán navigációs javaslatot ad.
3. A HOLD továbbra is nulla motion-output. A hold időpontja velocity-barrier:
   korábban készült ACTIVE nem élesztheti újra a mozgást. A friss guidance
   eredeti source-contextje, measurement-idői és lejárata változatlan marad.
4. Startup, missioncsere, STOP, inaktív mission és generationváltás nem
   örökölhet ilyen kérésfolytonosságot.

Nincs új IPC, worker, raw payload, config vagy authority. A contractok
változtatását ez az L6-on belüli lifecycle-szétválasztás nem igényli.

## Validáció

- `./r test`: 18 PASS.
- `test_nav2_control_integration.py`: 29 célzott eset PASS. A két új native
  láncteszt NAVIGATE és EXPLORE módban valódi L3 inputból idéz elő 420 ms-os
  HOLD-ot, nulla motoroutputot, azonos request identityt és az első jó tickben
  friss guidance-dzal folytatást igazol. Checkpoint restore és persisted
  canonical replay mindkét módban MATCH.
- `test_nav2_navigation.py`: 30 PASS, meglévő bounded transport, cancellation,
  freshness és lineage evidence.
- `git diff --check`: PASS.

## Engedélyezett élő ellenőrzés

Parancs: `./r roomcruise 45 c 50 --json`.
Capture: `runtime/captures/v3_20261118_213439_50624_capture.mcap`.
Evidence: `tmp/stop_20261118_211922/live_evidence`.
Replay: `tmp/stop_20261118_211922/live_replay.json`.

Az élő behavior a kért 45 s előtt FAILED lett. Tick 1697-nél
`ENCODER_RESTARTING` / `CRITICAL_DEVICE_DEGRADED` következett be, nulla final
outputtal. A GPIO-owner logja:

`ENCODER_NOTIFICATION_DELIVERY_STALLED:left`,
`last_callback_ns=10858684449080`, `mismatch_since_ns=10858836517652`.

A végső `current_b=delivered_b=0` nem cáfolja a korábbi elveszett átmenet
evidence-ét: a fizikai jel visszatérhet ugyanarra a szintre callback-progress
nélkül. A watchdog ezt szándékosan megőrzi. Nem módosítottuk és nem
indítottunk automatikus újabb motoros futást.

A valódi sensor-kiesés alatti 1709–1758. tick mind az 50 final outputja nulla.
Az 1750. checkpointban a Nav2 request még ugyanahhoz a missionhöz tartozik,
aktuális source-contexttel; a bridge COVERAGE állapotban maradt. A recovery
előtt a host leállította a behaviort, ezért az élő resume nem bizonyított.

A teljes 1827 tickes natív replay MATCH, repeated trace match, első divergence
nincs; a replay-result verifikáció PASS. Az explicit compiler COMPLETE
(3948 message), bundle- és source-verifikáció PASS; a capture replay-complete
és integrity PASS. A runtime PASS
itt biztonságos shutdown-t jelent; nem írja felül a behavior FAILED eredményét.
A robot/runtime az ellenőrzés végén STOPPED.

Megmaradt probléma: a GPIO notification-delivery elakadás upstream oka nem
bizonyított. A refaktor célzott software/replay evidence-e sikeres; hibamentes
45 s-os élő végrehajtás és élő localization-loss → resume még nincs igazolva.
