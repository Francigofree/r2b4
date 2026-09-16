# R2B4 person spatial tracking upgrade — 9ea6c261

Baseline: `9ea6c261e3408194378a6f08352af2b2ebf07546`

Ez a csomag a jelenlegi person-detection + photo-evidence production wiringra épül.
Nem írja felül a frissebb `native_sensor_inputs.py`, `v3_bounded_config.py`,
`conf/hardver.json` vagy `person_photo_evidence.py` fájlokat.

## Mit ad hozzá?

- Az L0 `person_detection` observation legfeljebb 5 ember bounding boxát adja át.
- L4 a kamera vízszintes irányát a meglévő `lidar_local_points` mélységével társítja.
- L4 stabil `person-N` `ObstacleTrack` objektumokat tart fenn `x/y`, `vx/vy`, radius és confidence értékkel.
- Az időben túl távoli kamera/LiDAR mintákat nem fuzionálja.
- A track state checkpoint/restore része, ezért rövid replay determinisztikus marad.
- Nem ad új command-, navigation-, safety- vagy motor-authorityt.
- A meglévő RoomCruise photo-evidence működés megmarad.
- A V3 rétegdoksi nem változik.

## Telepítés

```bash
cd <kicsomagolt-csomag>
python3 upgrade.py /home/alba/project_r2b4
```

Az installer a `9ea6c261...` baseline-ra készült. Ha más Git HEAD-et vagy már módosított
érintett source-ot talál, még a felülírás előtt leáll.

Backup:

```text
/home/alba/project_r2b4_backup_person_spatial_9ea6c261/
```

## Opcionális validáció

```bash
./validate_upgrade.sh /home/alba/project_r2b4
```

Teljes regresszió:

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q
```

## Fizikai beállítás

A `conf/vezerles.json` új `v3_navigation.person_tracking` részt kap.
A Camera Module 3 alapértelmezett vízszintes látószöge 66°-ként van beállítva
(`1.1519173063162575` rad), a kamera yaw offset alapból `0.0`.
Ha a kamera fizikailag néhány fokkal balra/jobbra néz, csak a
`camera_yaw_offset_rad` értéket kell kalibrálni.
