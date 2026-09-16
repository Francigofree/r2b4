# R2B4 FACE_PERSON V1 upgrade

Bázis: `Francigofree/r2b4` production source kompatibilitás a
`3236cada2eb12b7f63944ffa1fc55c47b39ae8be` állapothoz.

Az installer nem követeli meg, hogy a teljes Git HEAD változatlan legyen: a
runtime/capture állományok változhatnak. Viszont minden érintett production
source fájl Git blob SHA-ját ellenőrzi, és eltérés esetén **nem patch-el**.

## R2 kompatibilitás az async-L6 P0 javítással

Ez az R2 csomag a `3236cada2eb12b7f63944ffa1fc55c47b39ae8be` HEAD-re készült.
Az előző FACE_PERSON csomaghoz képest a repo közben módosította a
`v3/layers/l6_navigation.py` fájlt az async planner handoff freshness P0
javítása miatt. Az R2 ezt a javítást **nem írja felül és nem gyengíti**.

Az installer pontosan a javított L6 blobot várja:

```text
fc6465cd83483bc89d656c9e950a68fe3989c07a
```

és külön ellenőrzi a friss-result freshness anchor jelenlétét is. Ha ez nincs
meg, a telepítés patch nélkül leáll.

## Mit valósít meg?

Canonical V3 út:

```text
Camera + LiteRT person detector
          |
          v
L1 -> L2 -> L4 WorldSnapshot
               person-* spatial track
          |
          v
L5 FACE_PERSON mission
          |
          v
L6 target bearing -> VelocityTarget(v=0, omega)
          |
          v
L7 one MotionObjective
          |
          v
L8 existing closed-loop pivot realization
          |
          v
L9 -> L10 -> L11 -> L12 -> MotorWriter
```

Nincs külön motorút és nincs launcher-oldali TELEOP követőloop. A launcher csak
egy `FACE_PERSON` mission heartbeatet küld a resident command mailboxba.

## V1 viselkedés

- csak helyben fordul: az L6 által kért lineáris sebesség mindig `0.0 m/s`;
- érvényes L4 `person-*` track nélkül STOP;
- confidence küszöb: `0.50`;
- közép-deadband: `0.10 rad` (~5.7 fok);
- yaw gain: `2.5`;
- min. kért pivot: `0.40 rad/s`;
- max. kért pivot: `0.60 rad/s`;
- több embernél determinisztikusan a legkisebb abszolút bearing error alapján
  választ, majd távolság/confidence/track-id tie-break következik;
- nincs előre/hátra haladás;
- nincs keresőforgás elveszett cél esetén;
- nincs még tartós, többemberes személy-lock/re-ID.

A meglévő L4 track-élettartam miatt elveszett személy rövid ideig még a world
modelben maradhat; utána target nélkül a behavior STOP-ra vált. A végső motor
engedélyezés továbbra is L12 authority.

## Photo evidence

A meglévő `person_detection.photo_evidence` FACE_PERSON közben is aktív lesz.
A jelenlegi production config szerint stabil detektálási epizód után JPEG kerül:

```text
/home/alba/project_r2b4/pic/person_YYYYMMDD_HHMMSS_mmm_detXXXXXX_frameXXXXXX.jpg
```

A jelenlegi policy: 3 egymást követő pozitív detector result, 5 egymást követő
miss után re-arm, két kép között minimum 5 s.

Ez a fotó a kamera/detektor működését bizonyítja; a teljes L4->L12 behavior
bizonyítéka a FULL MCAP/Test Hub evidence.

## Telepítés

1. Állíts le mindent:

```bash
cd /home/alba/project_r2b4
./r2b4 shutdown
```

2. Másold/csomagold ki ezt az upgrade csomagot valahová, majd először csak
ellenőrizd a forrás-kompatibilitást:

```bash
python3 upgrade.py --root /home/alba/project_r2b4 --check
```

Elvárt:

```text
preflight: PASS (source compatible with 3236cada...)
```

3. Telepítés + célzott validáció:

```bash
python3 upgrade.py --root /home/alba/project_r2b4
```

A script:

- backupot készít `runtime/upgrade_backups/faceperson_<timestamp>/` alá;
- csak az ellenőrzött source blobokat módosítja;
- létrehozza a FACE_PERSON célzott tesztjét;
- futtat `git diff --check`-et;
- futtatja a V3 import guardot;
- futtatja a FACE_PERSON és a kapcsolódó resident/CLI/operator/person/L5-L9/async-L6 teszteket;
- bármely hiba esetén automatikusan visszaállítja a backupot.

Teljes pytest is kérhető:

```bash
python3 upgrade.py --root /home/alba/project_r2b4 --full-tests
```

## Élő teszt

A robot körül legyen szabad hely a helyben forduláshoz.

Indítás:

```bash
cd /home/alba/project_r2b4
./r2b4 faceperson c full
```

Helyes indulás ember nélkül:

```text
faceperson: STARTED
behavior: ARMED; motor may remain STOP until a valid target exists
capture: FULL continuous recording
```

A behavior szándékosan nem követel azonnali `ALLOW`-ot: target nélkül a helyes
állapot STOP, miközben a FACE_PERSON mission heartbeat él.

Próba:

1. ne legyen ember a látótérben -> robot maradjon állva;
2. állj a kamera bal oldalára -> robot csak forduljon feléd;
3. menj a jobb oldalra -> ellenkező irányba forduljon;
4. állj középre -> álljon meg;
5. menj ki a képből -> rövid track-timeout után álljon meg;
6. menj vissza -> újra forduljon rád;
7. ellenőrizd, hogy `pic/` alatt létrejött-e timestampelt person JPEG.

Leállítás és FULL capture finalizálás:

```bash
./r2b4 stop
./r2b4 shutdown
```

Ezután:

```bash
ls -lht pic/person_*.jpg | head
python3 -m v3.test_hub
```

## Acceptance feltételek

Élő teszt akkor tekinthető sikeresnek, ha:

- target nélkül nincs mozgás;
- target bal/jobb helyzetére a pivot iránya megfelelő;
- középre állva STOP;
- target elvesztése után STOP;
- a capture-ben L5 `FACE_PERSON` látszik;
- az L4-ben `person-*` track látszik;
- L6 aktív esetben `FACE_PERSON_TRACK` és `v_mps=0` látszik;
- L7 `selected_source=face_person`;
- L8 kért lineáris sebessége minden FACE_PERSON mozgó tickben 0;
- L12 marad az egyetlen végső motor/safety authority;
- nincs runtime fault;
- a FULL capture finalizálódik és Test Hub feldolgozható.

## Rollback

A legutóbbi installer-backup visszaállítása:

```bash
python3 upgrade.py --root /home/alba/project_r2b4 --rollback
```

## Érintett production fájlok

- `v3/contracts/messages.py`
- `v3/layers/l5_command_mission.py`
- `v3/layers/l6_navigation.py`
- `v3/layers/l7_motion_selection.py`
- `v3/composition/native_control.py`
- `v3/adapters/resident_command.py`
- `v3/control_cli.py`
- `v3/operator_controller.py`
- `v3/operator_cli.py`
- `v3/adapters/person_photo_evidence.py`
- `conf/vezerles.json`
- `README.md`

Tesztoldalon módosul `tests/test_v3_resident_command.py`, és létrejön
`tests/test_v3_face_person.py`.

## Tudatosan nem módosított rétegek

L8, L9, L10, L11 és L12 változatlan. A V3 strukturális rétegdokumentumot sem
módosítja a csomag: a rétegek felelőssége és az authority út változatlan marad.
