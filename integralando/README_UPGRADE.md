# R2B4 canonical calibrated camera-frame upgrade

**Source-first base:** `Francigofree/r2b4` `main` @ `3dbef91bd09f25fbe0e1aaff419846387fbf0e0f`.

## Cél és invariáns

A production runtime-ban raw kamerakép nem publikus. A Picamera2 raw buffer csak a kamera owneren belüli átmeneti adat. A publikus `CameraFrameSnapshot.image_bytes` és a person detector a canonical, empirikusan kalibrált `lores` frame-et használja; az ER2/media és person-photo evidence `lores` vagy `main` JPEG-et kérhet, de mindkettő kizárólag kalibrált formában hagyhatja el a vision ownert.

A P0 nem végez folyamatos high-resolution rectificationt. A `main` YUV420 buffer privát marad, és csak konkrét `main` JPEG-kéréskor fut egyszeri YUV→BGR→cached rectification→JPEG pipeline. A direct H264/video továbbra is fail-closed; ez tudatos performance boundary, amíg külön gyorsított calibrated-video pipeline nem készül.

## Mi változik

- új `v3/adapters/camera_rectification.py`: empirikus K/D, cached OpenCV remap map, natív `cv2.remap`;
- `NativePicamera2Camera`: csak empirikus, rectifiable geometry mellett indul; a publikus frame már `CALIBRATED` és hordoz calibration ID/K/timing metaadatot;
- a kalibrált lores payload visszaíródik a még élő Picamera2 requestbe, így a `request.save()` által készített lores JPEG is ugyanabból a calibrated pixelképből készül;
- `main` JPEG demand-driven: a privát YUV420 main buffer csak kéréskor alakul BGR képpé, ugyanazzal a cached empirical map-pel rectifikálódik, majd JPEG-ként kerül ki;
- fixed focus (`fixed_lens_position`) kerül a camera configba, és runtime-ban egyeznie kell a calibration reference lens positionnel;
- process IPC-ben csak calibration metadata megy át, képpayload továbbra sem kerül a control processbe;
- ER2 media protokoll: `R2B4CJPEG1 lores|main`; mindkét válasz kalibrált, a streaming/default továbbra is lores;
- photo evidence megtartja a `main` felbontást, de azt csak triggerkor rectifikáljuk;
- direct `camera.photo` / `camera.video` adapterek fail-closed;
- MCAP/L0 camera metadata: `calibration_state`, `calibration_id`, `rectification_duration_ns`, rectified `fx/fy/cx/cy`;
- új maintenance-only `tools/v3_camera_calibrate.py`, amely raw frame-et csak leállított resident runtime mellett használ empirical K/D meghatározására.

## 0. Előfeltétel

A runtime rectifier és a calibration tool OpenCV-t használ. Raspberry Pi OS-on a javasolt rendszer-csomag:

```bash
sudo apt update
sudo apt install -y python3-opencv
python3 -c 'import cv2; print(cv2.__version__)'
```

## 1. Empirikus kalibráció — az upgrade telepítése ELŐTT

Állítsd le a resident runtime-ot. Használj sík checkerboard táblát. A `--cols` és `--rows` az **inner corner** darabszám, `--square-mm` a négyzet tényleges oldala.

Példa — a számokat a saját tábládhoz és kívánt fix fókuszhoz add meg:

```bash
cd /tmp/r2b4_camera_calibration_upgrade
python3 payload/tools/v3_camera_calibrate.py \
  --lens-position 1.0 \
  --cols 9 \
  --rows 6 \
  --square-mm 25 \
  --samples 24 \
  --output /tmp/r2b4_camera_calibration.json
```

Mozgasd a táblát a teljes látómezőben: közép, szélek, sarkok, különböző távolság és döntés. A tool csak geometriailag eltérő mintákat fogad el. Ha az RMS hiba meghaladja a gate-et, nem ír elfogadott calibrationt.

A calibration output tartalmazza a native-sensor koordinátákra visszatranszformált K-t, D-t, a calibration sensor cropot és a rögzített LensPosition-t.

## 2. Upgrade telepítése

A telepítő csak az ehhez a source-first bázishoz készült állapoton fut alapból, és a módosított file-okról automatikusan backupot készít `.upgrade_backups/` alatt.

```bash
cd /tmp/r2b4_camera_calibration_upgrade
python3 apply_upgrade.py \
  --repo /home/alba/project_r2b4 \
  --calibration /tmp/r2b4_camera_calibration.json
```

Ne használd az `--allow-head-mismatch` opciót addig, amíg a source driftet nem ellenőrizted kézzel.

## 3. Validáció

A repo policy szerint először CORE, utána a releváns perception/process tesztek:

```bash
cd /home/alba/project_r2b4
./r test
./r test perception
./r test process
```

Motor-output nélküli fizikai camera probe:

```bash
python3 tools/v3_camera_test.py status --seconds 5
```

A kimenetben ellenőrizd:

- `calibration_state = CALIBRATED`
- nem üres `calibration_id`
- `rectification_duration_ms`
- stabil observed FPS és frame age

ER2 camera path ellenőrzés:

```bash
./r er2 "Mit látsz?" --camera --json
```

A media kliens csak az új calibrated `R2B4CJPEG1 lores|main` protokollt használja. A normál ER2 streaming továbbra is a canonical lores frame-et kéri; a main csak explicit/on-demand kérésre készül.

## Performance

A map-generálás nem frame-enként történik. A `(crop, width, height, stride)` kulcshoz egyszer készül OpenCV remap LUT, utána frame-enként egy natív `cv2.remap` fut a 640×360 lores képen. A csomag készítésekor x86 környezetben izolált 640×360 `cv2.remap` microbenchmark ~0.38 ms/frame átlagot adott; **ez nem Raspberry Pi 5 mérés**, ezért a live `rectification_duration_ns` telemetry az authority a roboton.

A 1280×720 `main` YUV420 nem rectifikálódik folyamatosan: csak `main` JPEG kéréskor fut YUV→BGR→rectification→JPEG. A H264/video út P0-ban továbbra is tiltott, így nincs raw video fallback.

## Rollback

Az installer kiírja a backup könyvtárat, például:

```text
BACKUP=/home/alba/project_r2b4/.upgrade_backups/camera_calibrated_frame_edge_v1_YYYYMMDD_HHMMSS
```

Rollback előtt állítsd le a runtime-ot, majd a backupból állítsd vissza az érintett file-okat. Újonnan létrehozott file-ok (`camera_rectification.py`, calibration tool/test) külön törlendők, ha teljes visszaállítást akarsz.

## Tudatos P0 korlát

A csomag nem állítja, hogy high-resolution/H264 calibrated video készen van. A direct video képesség ezért unavailable. P1-ben külön accelerated calibrated-video pipeline szükséges, amely calibrated YUV-ot ad a hardware encodernek anélkül, hogy a control/I/O core-ra compute kerülne.
