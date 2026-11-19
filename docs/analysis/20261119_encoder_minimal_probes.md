# Encoder: minimális production szondázás

A live encoder adatút csak a controlhoz szükséges measurement/lineage és
kisméretű integrity/delivery szondákat továbbítja.

- A production A/B callback-trace tiltott: `diagnostic_event_capacity=0` a
  forrás alapértékében és az aktív `conf/vezerles.json` konfigurációban.
  Tiltott állapotban nem keletkezik `QuadratureAlertEvent` objektum.
  A külön, explicit A/B probe továbbra is kérhet bounded részletes trace-t.
- A `NativeEncoderSource` 79 helyett 39 diagnosztikai DataFieldet épít.
  A 40 elhagyott mező részletes fit-, pin-, history- vagy összegzett
  diagnosztika; L3/L11 nem fogyasztja őket. Nem épülnek újra és nem kerülnek
  ismételt control/capture szállításra minden 20 ms-os mintában.
- Megmarad a fizikai signed count/displacement, sequence, generation,
  measurement-idő, velocity trust, estimation timebase és physical edge-idő,
  irányigazolás, counter-running állapot, hibadelta, callback-receipt és
  callback-latency. Freshness, uncertainty és watchdog policy nem változott.
- A részletes fit-ellenőrzések közvetlenül az encoder backend diagnosztikáját
  vizsgálják; ezek miatt a production sample nem hordoz külön mezőmásolatot.

## Offline evidence

Forrásfelvétel: `runtime/captures/v3_20261118_213439_50624_capture.mcap`.
A korábbi teljes felvétel native replayje MATCH volt; az explicit compiler
és source-verifikáció COMPLETE/PASS eredménye rendelkezésre áll.

A teljes 1827 tick kétszeri canonical production végrehajtása a régi és a
szűkített encoder-inputtal minden L3–L12 outputban és final motorértékben
azonos. L1/L2-ben csak az elhagyott passzív mezők különböznek. Ez offline
counterfactual vizsgálat, nem új élő futás.

A szűkített inputokból készült, 50 tickes mentett offline capture canonical
Replayer eredménye MATCH, repeated trace match; első divergence nincs:
`tmp/capture_load_20261118/compact_inputs_replay.json`.

Az 50 valódi encoder-readingből vett, négyszeres A–B–B–A minta (áganként
5000 hívás) a diagnosztikai DataField-projekció és pickle együttes CPU-idejében
49,11% csökkenést mutatott. E mezők pickle-mérete 47,90%-kal kisebb.
Az 50 teljes tick JSON-payloadja 7,02%-kal kisebb. Ezek izolált eredmények;
nem teljes robot-CPU vagy élő timing-javulás bizonyítékai.

Részletek és reprodukció: `tmp/capture_load_20261118/encoder_minimal_evidence.json`,
`encoder_minimal_evidence.py`. A régi ág a méréskori HEAD pontos encoder
projekcióját használta, az új a módosított source-ot.

## Ellenőrzés és korlát

- Encoder/backend, notification recovery, persisted MCAP/EVI evidence: 43 PASS.
- Külön raw-trace off/on ekvivalencia és compact scalar IPC: 2 PASS.
- `./r test release`: 28 PASS, a kis robot-contract gate-tel együtt.
- `git diff --check`: PASS.

Robot runtime nem indult. A konfiguráció offline feloldható, a kötelező
software gate-ek sikeresek. Az encoder notification-delivery elakadás fizikai,
native-library vagy ütemezési oka továbbra sem bizonyított; az 50 Hz-es capture
terhelési korrelációja önmagában nem oksági bizonyítás. Új élő endurance és
fizikai működéspróba nem történt.
