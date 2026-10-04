# R2B4 pytest

A pytest-rendszer szándékosan vékony. A fejlesztést nem a teljes tesztfa állapota,
hanem néhány robot-szintű contract védi.

## Szerkezet

```text
tests/
  gate/manifest.json          kötelező, gyors robot-invariánsok
  scenarios/manifest.json     kevés release scenario
  packs/core/                 opcionális fejlesztői tesztek
  packs/feature/              opcionális fejlesztői tesztek
  packs/deep/                 opcionális fejlesztői tesztek
  endurance/manifest.json     explicit hosszú futás
```

A manifest csak meglévő pack-tesztek node ID-ját nevezi meg. Így ugyanaz a teszt
nem másolódik két helyre, és egy új pytest fájl nem válik automatikusan release
követelménnyé.

## Használat

```bash
./r test                     # kicsi, fail-fast gate
./r test release             # gate + robot scenariók
./r test full                # release alias
./r test pack core
./r test pack feature
./r test pack deep
./r test all                 # összes pack; diagnosztikai
./r test endurance
./r test tests/packs/feature/test_FILE.py::test_NAME
```

Normál fejlesztésnél először `r test`. A módosított subsystem packja csak akkor
fusson, ha az adott implementációhoz hasznos. Release vagy közös canonical
boundary változásnál `r test release`.

A `python -m pytest` továbbra is használható, de a teljes pack-fát futtatja; ez
nem a kötelező regressziós definíció.

## Karbantartási szabály

Permanent teszt értéket/algoritmust ne fagyasszon. Kapcsolatot és roboteredményt
védjen: authority, STOP/FAULT, freshness, fizikai realizálhatóság, determinisztikus
execution/replay. Privát helper, tuning, provider, CLI-szöveg és lezárt migráció
maradjon pack-szintű fejlesztői evidence.

A permanent budget a manifestekben van: gate <= 15, scenarios <= 10.
