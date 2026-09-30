# R2B4 Test Hub contract-driven diagnostics upgrade

**Base repository:** `Francigofree/r2b4`  
**Base main HEAD:** `cf947a6c14b3677abca4c7b846051bf64a9040ff`  
**Upgrade id:** `R2B4_TEST_HUB_CONTRACT_DRIVEN_V1`

## Mit valósít meg

1. Új `v3/diagnostic_contracts.py` réteg a production dataclassokból származtatott diagnosztikai contractokkal.
   - encoder `wheel_velocity`
   - IMU `ekf_heading`
   - LiDAR matcher / relative motion / pose
   - L3 `LocalizationQuality`
   - a schema fingerprint automatikusan változik, ha a production dataclass mezői változnak.

2. Új `v3/test_hub_diagnostic_coverage.py` streaming generic analyzer.
   - minden MCAP-ban ténylegesen megjelent scalar mezőt automatikusan felismer;
   - ismeretlen jövőbeli mezőt is elemez TestHub-kód módosítása nélkül;
   - presence/type/change/min/max/mean/state/monotonicity leíró metrikákat készít;
   - külön jelzi a `missing_from_capture` és `new_unregistered_fields` eltéréseket;
   - külön kezeli a generic és a domain-specifikusan értelmezett mezőket;
   - **nem állít automatikus gyökérokot és nem kap robot-authorityt**.

3. A Test Hub minden új evidence bundle-be létrehozza:

   `diagnostic_schema_coverage.json`

   és az `agent_view.json` hivatkozik rá.

4. Device-health triage javítás.
   - ugyanazt a `PRODUCTION_CRITICAL_DEVICE_IDS` policyt használja, mint a production;
   - optional kamera/person-detector non-OK állapot továbbra is mérve van, de nem lesz HIGH actionable incident;
   - azonos, folyamatos critical health állapot egy episode-edge incidentet ad, nem egyet minden tickre.

5. Három célzott pytest védi az új contractot:
   - production dataclass → diagnostic contract automatikus követés;
   - jövőbeli/unknown field generic láthatóság root-cause állítás nélkül;
   - production-critical device policy + incident episode collapse.

## Telepítés

```bash
cd <a kicsomagolt upgrade könyvtára>
python3 apply_upgrade.py /home/alba/project_r2b4 --pytest
```

A script fail-closed:
- ellenőrzi a repo HEAD-et;
- ellenőrzi a két módosított source blob SHA-ját;
- minden textual patch anchornak pontosan egyszer kell illeszkednie;
- backupot készít a `runtime/upgrade_backups/` alatt;
- `py_compile` ellenőrzést futtat;
- `--pytest` esetén lefuttatja az új célzott tesztfájlt is.

Ha a repo HEAD időközben változott, **ne erőltesd automatikusan**. Az upgrade-et érdemes újragenerálni az új HEAD-re. Az `--allow-head-mismatch` csak a HEAD-ellenőrzést lazítja; a source blob ellenőrzés továbbra is megvédi a patch-et.

## Várható új evidence

Egy friss forensic capture után a `.evidence/diagnostic_schema_coverage.json` például ezt képes megmutatni:

```text
sensor:wheel_velocity
  production_field_count: ...
  observed_field_count: ...
  generic_analyzed_field_count: ...
  domain_analyzed_observed_field_count: ...
  missing_from_capture: []
  new_unregistered_fields: []
  schema_drift_detected: false
```

Ha egy encoder upgrade új `edge_delivery_latency_ns` mezőt ad a production contracthoz:

- a diagnostic contract fingerprint automatikusan változik;
- ha a mező bekerül a capture-be, a generic analyzer azonnal látja;
- ha a production dataclassban megvan, de a capture producer nem exportálja, `missing_from_capture` jelzi;
- ha a capture-ben megjelenik egy még nem ismert mező, generic módon akkor is elemezhető;
- domain-specifikus oksági állítás csak külön, explicit analyzer-szabállyal készülhet.

## Érintett fájlok

Módosul:
- `v3/test_hub_analysis.py`
- `v3/test_hub_next.py`

Új:
- `v3/diagnostic_contracts.py`
- `v3/test_hub_diagnostic_coverage.py`
- `tests/feature/test_v3_test_hub_diagnostic_contracts.py`

A capture/replay canonical authority és a motor/safety runtime útvonal nem változik.
