# GUI Log Panel Guide

## Oldalrészek

- `Áttekintés`
  - capture állapot
  - queue
  - dropped
  - write error
- `Legfrissebb`
  - a legutóbbi session kiválasztott csatornája
- `Session nézet`
  - konkrét session böngészése
- `Archívum`
  - archivált hónapok
  - kézi archiválás

## Fejlécgombok

- `LOG BE/KI`
  - a capture módot kapcsolja
- `LOG ARCHIVÁLÁS`
  - lezárt sessionök archiválása

## API

- `/api/log/state`
- `/api/log/stats`
- `/api/log/latest`
- `/api/log/session/{session}/channel/{channel}`
- `/api/log/live`
- `/api/log/archive/list`
- `/api/log/archive/run`
