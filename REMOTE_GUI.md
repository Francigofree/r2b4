# Remote GUI

Az új GUI logoldala a session loggerre épül.

## Fontos végpontok

- `GET /api/log/sessions`
- `GET /api/log/stats`
- `GET /api/log/latest?channel=system`
- `GET /api/log/live?channel=audit`
- `POST /api/log/control`
- `POST /api/log/archive/run`

## Megjegyzés

A régi runtime-alapú audit és telemetria fájlok megszűntek.
