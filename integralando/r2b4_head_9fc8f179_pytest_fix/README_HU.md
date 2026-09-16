# R2B4 pytest synchronization fix — HEAD 9fc8f179

Base commit:

`9fc8f179b5ed5ed5d0bf28eb0c4305fffe8edaea`

## Mit javít

A teljes pytest futás 11 hibáját négy gyökérokra bontja, és a legkisebb source-first javítást alkalmazza.

1. **Kamera/import guard**
   - A Picamera2/libcamera konkrét hardverfüggés kizárólag a két kamera edge-adapterben engedélyezett.
   - `APPROVED_THIRD_PARTY_ROOTS` változatlanul csak `numpy` és `scipy`.
   - Új regressziós teszt ellenőrzi, hogy egy layer továbbra sem importálhat Picamera2-t.

2. **Architektúra import-whitelist tesztek**
   - A tesztek felveszik a már ténylegesen használt `live_inputs`, `device_health_policy`, `live_camera` és `picamera2_camera` V3 modulokat.
   - Production source-ot emiatt nem alakít át.

3. **Production kritikus eszközazonosítók**
   - A resident-runtime és sensor-measurement fixture-k a production azonosítókat használják:
     `WHEEL_ENCODERS`, `BNO055_IMU`, `RPLIDAR_C1`.

4. **Capture/replay determinisztika teszt**
   - Az íves replay fixture ugyanazt a `PRODUCTION_CRITICAL_DEVICE_IDS` policyt és ugyanazt a LiDAR device identityt használja, mint a canonical replay.

## Érintett fájlok

- `v3/import_guard.py`
- `tests/test_v3_architecture_boundaries.py`
- `tests/test_v3_resident_runtime.py`
- `tests/test_v3_differential_drive_arcs.py`
- `tests/test_v3_sensor_measurement_tool.py`

Nincs V3 rétegdoksi-módosítás, nincs motor/runtime viselkedés lazítás, nincs config-módosítás.

## Alkalmazás

```bash
cd /home/alba/project_r2b4
python3 /UTVONAL/apply_fix.py --dry-run .
python3 /UTVONAL/apply_fix.py .
```

A script:
- csak a fenti HEAD-en fut;
- megtagadja az érintett dirty fájlok felülírását;
- minden módosítási anchorból pontosan egyet követel;
- atomikusan ír;
- commitot/push-t nem végez.

## Validáció

Először célzottan:

```bash
python3 -m pytest -q \
  tests/test_v3_architecture_boundaries.py \
  tests/test_v3_resident_runtime.py \
  tests/test_v3_differential_drive_arcs.py \
  tests/test_v3_sensor_measurement_tool.py
```

Majd teljes regresszió:

```bash
python3 -m pytest -q
```

## Fontos

A csomag forráselemzés alapján készült. A Raspberry Pi hardveres környezetét ebben a sandboxban nem lehetett futtatni, ezért a végső PASS-t a fenti Pi-oldali pytest igazolja.
