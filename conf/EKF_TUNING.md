# EKF Tuning

Az EKF tuning új alapja a session-alapú JSONL naplózás.

## Bemenet

A tuninghoz a legfrissebb session két fájlja kell:

- `logs/session_*/control.jsonl`
- `logs/session_*/sensors.jsonl`

## Ajánlott mérési menet

1. Álló helyzet 10-20 mp
2. Egyenes előre 20-30 mp
3. Helyben fordulás balra és jobbra

## Mit nézz

- `control.jsonl`
  - `ekf_diag`
  - `control_snapshot`
- `sensors.jsonl`
  - `encoder_diag`
  - `imu_diag`

## Megjegyzés

A régi `logs/session_<timestamp>/ekf_full_log.jsonl` és `pid_log.jsonl` már nem része a kanonikus útnak.
