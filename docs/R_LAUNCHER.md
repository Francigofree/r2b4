# Az `r` launcher használata és bővítése

Az `r` a meglévő CLI-k vékony belépési pontja. A robotparancsokat a
`v3/interface_cli.py`, a host-segédeket a `v3/host_cli.py` kezeli; a launcher
nem birtokol robotállapotot vagy külön végrehajtási utat.

## Gyors használat

```bash
r skill /home/alba/project_r2b4/robot_skills/terkepesz_r2b4.py
r skill terkepesz_r2b4 --parameters '{"duration_s": 30}' --json
r skill                         # mentett skillek listája
r evi                           # legutolsó lezárt MCAP feldolgozása
r diag                          # legutolsó evidence teljes diagnosztikája
r help skill
r help evi
r help diag
r commands                      # parancsok, aliasok, publikus capability-k
r commands --json               # ugyanez gépi katalógusként
```

A skillparaméterek mindig az adott skill forrása szerinti JSON-objektumot
alkotnak; a példában szereplő paramétert a kiválasztott skillhez kell igazítani.
Név, `NAME.py`, projektgyökérhez viszonyított `robot_skills/NAME.py` vagy abszolút
fájlút adható meg. A cél a mentett `robot_skills` könyvtár közvetlen `.py`
bejegyzése legyen. A launcher a hivatkozást és a JSON-t indítás előtt ellenőrzi,
majd a canonical `skill.run` műveletet kéri. A skill a meglévő LCE által felügyelt
külön interpreterben fut tovább; az indítási eredmény a `run_id`-t tartalmazza.

```bash
r execute skill.status --parameters '{"run_id":"ID"}' --json
r execute skill.stop --parameters '{"run_id":"ID"}' --json
r execute skill.source --parameters '{"name":"terkepesz_r2b4"}' --json
r execute skill.test --parameters '{"name":"terkepesz_r2b4"}' --json
r x                             # canonical STOP
```

Az `evi` a `runtime/captures/*.mcap` valódi, nem symlink fájljai közül a
legnagyobb módosítási idejűt választja; egyezésnél a fájlnév dönt. A production
capture a finalizálásig `.partial` néven marad, így nem kerül a választásba.
A kiválasztott input stderr-en látszik. Az eredmény alapból a mellette lévő
`NAME.evidence` bundle. Meglévő outputnál hiba és külön `--output` célra utaló
üzenet érkezik; nincs automatikus felülírás.

```bash
r evi latest --workers 1
r evi CAPTURE.mcap --output /tmp/new-run.evidence
r evi compile CAPTURE.mcap
r evi verify BUNDLE.evidence --source CAPTURE.mcap
r evi query BUNDLE.evidence --tick-id 1505 --layer L11
```

A `diag` meglévő alapértelmezése `full latest`: a
`runtime/captures/*.evidence` legfrissebb sealed manifestjét választja,
ellenőrzi a bundle-t, és diagnosztikai artifactot ment a `runtime/diag` alá.
Egyedi `evi --output` célhoz explicit `r diag full BUNDLE.evidence` kell.
A `--no-save` ideiglenes futást kér. Hiányzó diagnosztikai evidence esetén a
DIAG exit code-ja `2` lehet; ez nem compilerhiba. Aktív resident runtime mellett
a launcher meglévő offline DIAG-védelme elutasítja a futást.
Az élő részletes runtime-állapot továbbra is `r d` vagy `r rt diag`.

## Parancskészlet

Az aktuális teljes lista: `r commands`; a részletes opciók: `r help PARANCS`.

| Feladat | Belépési pont |
| --- | --- |
| Robotállapot és biztonsági műveletek | `s`, `d`, `caps`, `x`, `sd`, `panic` |
| Mozgás és operator-session | `f`, `b`, `t`, `m`, `rc`, `fa`, `fp`, `pr` |
| Runtime és capture | `rt`, `cap` |
| Publikus interface, LCE | `read`, `execute`, `skill` |
| Offline evidence és diagnosztika | `evi`, `diag`, `tune` |
| Kamera | `cam` / `camera`: `status`, `on`, `off`, `photo`, `video` |
| AI és voice | szöveges kérés, `--` sima LLM-prompt, `route`, `er2`, `chatgpt`, `voice` / `wake` |
| Fejlesztés | `test`, `pytest`, `git`, `gitre`, `tools`, `tool` |
| Host és telepítés | `host`, `sys`, `cpu`, `cpu2`, `disc`, `mem`, `temp`, `ps`, `net`, `usb`, `i2c`, `install`, `where`, `root`, `version` |

Az új LCE-felületek a canonical `read` / `execute` útján is elérhetők:
`skill.list`, `skill.*`, `observe.*`, `media.recorder.*`, `hri.report`,
`source.*`, `docs.*`, `config.*`, `evi.*`, `diag.*`. A katalógus ezeket az owning
capability-definíciókból követi, a mentett skillneveket forrásimport nélkül
fedezi fel. Az élő elérhetőséget külön a `r caps` mutatja.
A régebbi operator/control CLI-k és a privát runtime-worker nem kapnak új
launcher-rövidítést; a Python segédprogramok listája `r tools`.

## Hogyan követi a rendszer fejlődését?

A `v3.launcher_cli.command_entries()` ugyanazt az effektív parancskészletet adja
a súgónak, a gépi katalógusnak, a parancsfelismerésnek és a felső szintű TAB-nak.
A robotparancsok az owning argparse parserből, a host-parancsok a
`host_cli.COMMAND_HELP` definícióból származnak. A launcher saját belépési
pontjainak metadataja egy helyen szerepel.

- Új robotparancs vagy opció az `interface_cli` parserében: automatikus katalógus,
  súgó és TAB; a tényleges működés továbbra is az owning CLI felelőssége.
- Új ER2-, OAuth-, kamera- vagy EVI-opció: a saját parserből követett TAB.
- Új mentett Python-skill: a következő discoveryban megjelenik a neve és a
  `skill.NAME` capability; nem kell launcher-allowlistet módosítani.
- Új software/observer capability: a meglévő owning katalóguson keresztül
  megjelenik a `commands` és az `execute` TAB kínálatában.
- A RobotInterface statikus adapterei saját `capability_names` és
  `read_capability_names` metadatajukból kerülnek a `read` / `execute` kínálatba;
  a launcher a meglévő adapter-compositiont követi, élő statuslekérdezés nélkül.
- Új pytest pack: a packkönyvtárakból követett `r test pack` TAB.
- Új, önálló CLI-belépési pont: egyszer szükséges hozzáadni a launcher
  delegálását és metadataját; ez nem automatikus tetszőleges kódfuttatás.

A Bash completion kizárólag a Pythonból kapott hint/candidate protokollt
rajzolja ki. Telepítés: `r install`; már telepített, a repóra mutató symlink
esetén a source-változások azonnal érvényesülnek. A discovery és a help nem
indít skillt, robot-runtime-ot, auth-folyamatot vagy evidence-feldolgozást.
