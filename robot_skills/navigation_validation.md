# Élő navigációs mérés

A `navigation_validation` kizárólag a befecskendezett aszinkron Robot SDK-t
használja. Importálása és tesztje nem indít hardvert. A futtató előbb az aktuális
canonical runtime-ot készíti elő `full`, 50 Hz-es capture-rel, majd az idle,
friss, jó lokális pozíciójú roboton egyszer indítja a skillt:

```bash
./r skill navigation_validation --parameters '{"cruise_s":20}' --json
```

A fenti parancs fizikai mozgást kér. Egyetlen watchdoggal határolt RoomCruise indul,
az eredeti command producer birtokolja a heartbeatet és a TTL-t. Az elfogadás
után mért húsz másodperces ablak végén a skill explicit, saját canonical
`behavior.cancel` kérést küld. A `TIMED_OBSERVATION_COMPLETE` eredmény az időablak
mérését és a STOP-ot jelzi; a behavior tervezetten `CANCELLED`, nem célba érés.
A default sebességkorlát
0,20 m/s és 1,10 rad/s. A jelenlegi 0,3557 m nyomtáv, 0,15 m/s kalibrált
folytonos minimum és 0,19 m/s target wheel floor mellett a centered-spin
fizikai minimuma 0,8434, target minimuma 1,0683 rad/s; az 1,10-es korlát ezt
lehetővé teszi. A canonical aktuális config és a safety továbbra is érvényesít
minden korlátot. A 0,18 m/s és 0,60 rad/s envelope nem ad teljes manőverképességet.

A skill 10 Hz körüli, bounded SDK-megfigyeléseit külön jelenti a teljes 50 Hz-es
capture-től. A mintavett lokális odometriából számolt radiális távolság 1,25 m-es,
az összegzett út 3 m-es küszöböt kap; valamelyik küszöb elérése canonical
`behavior.cancel` kérést és `BOUND_REACHED` eredményt ad. Ezek becslésre alapozott
leállítási küszöbök: a mintavétel, a parancsút és a fékezés miatt a megállási
távolság nagyobb lehet. A fizikai mozgás független mérését nem helyettesítik. Az ilyen
befejezés nem bizonyít húsz másodperces sikeres navigációt.

Processz-, capture-, behavior-, command-, mission- vagy lokális generation-váltás,
stale/regresszáló státusz, elveszett lokalizáció, kritikus health-hiba és váratlan
safety STOP megszakítja az egyetlen próbát. Aktív, folytonos lokális `DEGRADED`
minőség csak a runtime saját `ALLOW` döntése mellett megfigyelhető. A skill nem
indít automatikus újrapróbálást. A véges behavior után disabled, nulla kimenetű
canonical STOP igazolása szükséges. Hiba vagy cancellation alatt saját aktív
behaviorét canonically visszavonja.

Az opcionális `candidate_places` pontosan egy már létező, aktuálisan qualified
Spatial place-nevet fogad. Ilyenkor a meglévő `strategic_explore` behavior fut egy
véges, legfeljebb `strategic_budget_s` (alapból 30 s) célpróbával. A skill nem ír
Spatial tudást, nem talál ki koordinátát, nem igazít történeti atlaszt a runtime-ra.
Név hiányában a stratégiai út `NOT_EXERCISED`. Expozíciókorlát után új fázis nem indul.

A futtató megvárja a skill terminal eseményét, megőrzi annak `run_id`,
`source_hash`, `started_ns`, `finished_ns` értékeit, és csak utána állítja le a
runtime-ot a canonical `operator.runtime.stop` úton. Ezzel a skill teljes
source-binding eseménye a lezárt MCAP-ba kerül. A skill maga nem finalizál
capture-t és nem indít automatikus evidence-feldolgozást.

Explicit offline feldolgozás a lezárt, konkrét MCAP-hivatkozáson:

```bash
./r evi CAPTURE.mcap --output /tmp/EGYEDI_CEL/capture.evidence
./r evi verify /tmp/EGYEDI_CEL/capture.evidence --source CAPTURE.mcap
./r diag full /tmp/EGYEDI_CEL/capture.evidence
```

A canonical replay külön, explicit offline lépés. Nagy FULL capture esetén a
meglévő checkpointos incident ablakot kell kiválasztani úgy, hogy a teljes
mérési fázist és az igazolt STOP-ot lefedje. Az offline validator a replay
eredményét ellenőrzi, és a natív MCAP, EVI, DIAG, skill terminal binding és fázis command-azonosítók alapján
állít össze forráshoz kötött eredményt. A Python-return, a capture integrity,
a replay `MATCH`, a mért navigation/timing-mutatók és a független fizikai siker
külön verdict. A skill visszatérésekor ezek `PENDING_OFFLINE`, `NOT_RUN`, illetve
`NOT_INDEPENDENTLY_MEASURED` maradnak. Nem gyakorolt no-path, progress recovery,
történeti memória-restart vagy atlasz-index viselkedés nem kaphat élő PASS-t.

A célzott SDK-double teszt a canonical launcherrel futtatható:

```bash
./r test tests/packs/feature/test_navigation_validation_skill.py
```

Ez a teszt a fail-closed orchestrationt és az eredmény szemantikáját bizonyítja;
a natív motormentes EXPLORE-szimuláció, az élő futás és a fizikai pontosság
külön evidence.

A replay ablakának kezdete a legkorábbi `phases[].start_monotonic_ns`, vége
legalább az `observed_end_ns` a skill eredményéből:

```bash
python -m v3.replay replay CAPTURE.mcap --project-root . \
  --start-monotonic-ns START_NS --end-monotonic-ns STOP_NS \
  --output /tmp/EGYEDI_CEL/replay.json
```

A végleges offline elfogadási riport bemenetei az eredeti, változatlan artifactok:

```bash
python -m tools.navigation_live_validation \
  --capture CAPTURE.mcap --evidence /tmp/EGYEDI_CEL/capture.evidence \
  --replay-result /tmp/EGYEDI_CEL/replay.json \
  --skill-result /tmp/EGYEDI_CEL/skill_result.json \
  --diag-results /tmp/EGYEDI_CEL/diag_results.json \
  --output /tmp/EGYEDI_CEL/acceptance.json
```

A `skill_result.json` a teljes terminal `skill.status` válasz; a DIAG JSON az
analyzer-azonosítóval kulcsolt canonical riportokat tartalmazza, köztük a
`navigation` riportot a producer időzítési összesítésével. A validator nem
indít runtime-ot vagy hardvert. A `PARTIAL` eredmény azt jelzi, hogy maradt
nem gyakorolt eset; a hiányzó bizonyíték nem jelent sikert.
