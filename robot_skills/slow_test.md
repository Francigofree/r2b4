# Lassú mozgás encoder elfogadási tesztje

A skill explicit indításakor **élő robotmozgást kér**. Importálás és mentés nem indít mozgást.

Indítás a projekt gyökeréből:

```bash
./r skill robot_skills/slow_test.py --json
```

Kemény padlón, megjelölt futás:

```bash
./r skill robot_skills/slow_test.py --parameters '{"surface":"hard"}' --json
```

Az indítás háttérben futó skillt és `run_id`-t ad. Az eredmény lekérése:

```bash
./r execute skill.status --parameters '{"run_id":"AZ_INDÍTÁSKOR_KAPOTT_ID"}' --json
```

Opcionálisan a `surface` paraméter `hard` vagy `soft` értéke megjelöli a padlót;
a sebességprofilt nem módosítja. Alapértéke `unspecified`.

## Menetsor

| Szakasz | Egyenes előremeneti cél | Tartás az ALLOW admission után |
| --- | --- | --- |
| 1 | 0,15 m/s | 4 s |
| 2 | 0,20 m/s | 2 s |
| 3 | 0,15 m/s | 4 s |

A TELEOP SDK-művelet új missiont indít, ezért szakaszváltáskor a canonical
controller STOP-ot és új indulást végez. A három tartás összesen 10 s;
a runtime indulása, admissionök, szakaszközi megállások, shutdown és offline
elemzés ezen felül időt igényelnek. A sebesség-alapjelek időintegrálja 1,6 m,
de ez nem garantált fizikai úthossz. Mindhárom szakasz előre halad, nincs visszatérés.
A végén a skill STOP-pal leállítja a runtime-ot és véglegesíti a capture-t.

Az eredeti 0→0,20→0 háromszög nem volt alkalmas erre a tesztre: a nulla TELEOP
tiltott, az ismételt TELEOP nem streaming sebességállítás, és a production
tartós kerékminimum 0,15 m/s. Ennél lassabb mozgás az indulási átmenetben,
illetve a fizikai terhelés miatt jelentkezhet; a skill nem kerüli meg ezt a minimumot.

## Bizonyítás és megszakítás

A skill leállított runtime-ból indul, saját FULL 50 Hz capture-t készít.
Az SDK-val ellenőrzi a friss runtime-státuszt, a kritikus encoder/IMU/LiDAR
health-et, az aktív missiont és az ALLOW-t. Váratlan hibánál megszakítja
a menetsort, leállít és nem próbálja újra.

A mozgás, felügyelet és capture-életciklus kizárólag SDK-műveleteket használ.
A health-szűrés a meglévő canonical kritikus eszközlistát fogyasztja.
A részletes encoderadat az SDK-státuszból nem olvasható, ezért a **már
leállított futás** kiértékelése a meglévő offline MCAP Evidence Compiler,
ellenőrzött query és native Replayer API-kat használja. A captured config
és kalibráció az authority; az elemzés nem nyit hardvert és nem ad commandot.
Az evidence bundle, `result.json` és `replay.json` egyedi
`/tmp/r2b4-slow-test-*` könyvtárban keletkezik; útvonala az eredményben szerepel.

- `PASS`: teljes, ellenőrzött raw/replay capture; három megvalósult canonical
  mozgásreferencia (legalább 0,5 s szakaszonként) és
  pozitív signed kerékelmozdulás; helyes count/time sebesség, fizikai fitablak,
  trust és publikált sebesség; mindkét keréken legalább 1 s összefüggő,
  friss accepted élekkel megújuló bizonytalan-fit feed-forward PI-korrekció
  nélkül; canonical replay `MATCH`.
- `INCONCLUSIVE`: nincs bizonyított eltérés, de valamely szükséges evidence
  vagy a tartós bizonytalan-fit helyzet hiányzik. A láthatóan sikeres mozgás
  ilyenkor nem bizonyítja a hiba elleni tartós toleranciát.
- `FAILED`: live megszakítás, bizonyított encoder/control eltérés vagy replay
  mismatch. Az első megnevezett tick/mező vagy live hiba az eredményben szerepel.

A replay a már számított encoder DeviceSample-ből indul: önmagában a MATCH
nem bizonyítja az encoder-adapter algoritmusát. Ezért külön szükséges az
intervallumképlet ellenőrzése. A capture nem tartalmaz minden nyers GPIO-élt;
az összes fizikai pulse és a residual uncertainty teljes rekonstrukcióját
ez a teszt nem állítja. Az encoderből származó sebesség/út nem független mérés:
abszolút sebességpontossághoz padlójelölés vagy független videós mérés szükséges.

Az egyenes TELEOP iránytartása L8-ban korrekciót alkalmazhat: a két kerék
célja eltérhet, a középsebesség pedig a heading-hiba miatt csökkenhet.
A kiértékelés a kért L5 sebességet és az L8 referencia L10 középsebességgel
történő realizálását ellenőrzi, valamint a kalibrált tartós kerékminimumot;
nem követeli a bal/jobb kerékcél pontos egyezését.

## Várható fizikai mozgás

Elvárt: három rövid, egyenes előremenet, indulási gyorsulással, köztes
STOP/indulásokkal és végső megállással. A normál 0,6 m/s² gyorsuláskorláttal
a 0,15 m/s elérése legalább kb. 0,25 s; a tényleges admission és talajhatás
ezt módosíthatja. A STOP nem ígér lassú, háromszögprofil szerinti fékezést.

Kemény padlón várhatóan hamarabb indul, egyenletesebben és az alapjelhez
közelebbi sebességgel halad. Puha padlón későbbi indulás, kisebb sebesség,
kerékenként eltérő lassulás, irányeltérés vagy szakaszos mozgás is előfordulhat.
Ezek feltételes fizikai várakozások. Az ép, friss accepted élek melletti
bizonytalan fit nem okozhat hibás PI-korrekciót; valódi beragadás vagy
élvesztés továbbra is jogos STOP/FAULT-hoz vezethet.

## Élő evidence, 2026-10-12

Az első élő futás capture-je:
`runtime/captures/v3_20261012_210945_6773_capture.mcap`.
A TELEOP-ot a robot elfogadta, de a host SDK-válaszának `MotionHandle`
JSON-kódolási hibája megszakította a skillt. A runtime szabályosan leállt;
a háromszakaszos teszt eredménye `FAILED`, az encoder elfogadási bizonyítás
`INCONCLUSIVE`. A válaszút azóta a meglévő `_jsonable` konvertert használja,
és a valódi socketes regresszió sikeres.

A teljes, integritás-ellenőrzött 50 Hz capture replaye `MATCH`.
A 258–282. tick között 0,480 s ALLOW és encoderrel igazolt kerékforgás volt,
majd a 283. tick explicit `COMMAND_STOP` lett, nulla motorkimenettel.
29 bal és 30 jobb GPIO-fit megfelelt az intervallumképletnek; nem találtunk
encoder/control eltérést. A bizonytalan-fit feed-forward folytonos ideje
0,400/0,419 s volt, ami rövid részbizonyíték, de nem teljesíti az 1 s-os
elfogadási küszöböt. A STOP utáni kifutással együtt a signed kerékút
35,44/43,18 mm; ez nem függetlenül mért robot-testelmozdulás.

Az offline artifactok: `/tmp/r2b4-slow-test-g9yhr56a/`, benne az eredeti
`live_skill.json`, a részleges mozgás ellenőrzése `partial_motion.json`,
az evidence bundle és a `replay.json`. A padló típusa nem lett megadva.

A második, külön jóváhagyott futás:
`runtime/captures/v3_20261012_211945_8169_capture.mcap`.
Az első megszakítás 571. tick, `LIDAR_CLEARANCE_LOW`: az elülső clearance
0,392 m lett a 0,400 m minimum mellett. L12 ekkor nullázta a kimeneteket.
A megszűnő fizikai élek miatt később, 591. ticknél L11 watchdog-FAULT
keletkezett. Ez a futás `FAILED`, replaye `MATCH`; az encoder-fit nem okozott
hibás PI-korrekciót. A tartós helyes bizonytalan-fit feed-forward bal/jobb
1,580/1,341 s volt. Artifact: `/tmp/r2b4-slow-test-8_4ikcz0/`.

A harmadik, külön jóváhagyott **kemény padlós** futás:
`runtime/captures/v3_20261012_212225_8169_capture.mcap`.
Mindhárom szakasz teljesült; a runtime szabályos STOP-pal, FAULT nélkül
állt le. A teljes capture 981 tick, a natív replay `MATCH`.
A javított kiértékelés 822 tickből 480 bal és 480 jobb GPIO-fitnél sem
talált encoder/control eltérést, és mindhárom canonical referencia
realizálása igazolt. A tartós bizonytalan-fit idő csak 0,481/0,321 s,
ezért az immunitási elfogadás továbbra is `INCONCLUSIVE`: ez a kemény padlós
futás nem gyakorolta az 1 s-os tartós bizonytalanságot.

A futó skill az induláskori forrását használta; az utólag javított, iránytartást figyelembe vevő
kiértékelés új motorindítás nélkül, a változatlan verified evidence bundle
és a korábban sikeres natív replay felhasználásával készült.
Artifact: `/tmp/r2b4-slow-test-ie8sp3wv/`, eredeti `live_skill.json`,
javított `corrected_result.json`, `replay.json` és evidence bundle.

A stabil ablakokban (az admission után 0,5 s-tól) az encoder GPIO-fit
sebességmediánjai bal/jobb: 0,159/0,158; 0,207/0,199; 0,159/0,160 m/s.
A teljes futás nulláról induló számlálói 2855/2837 impulzusra jutottak:
1,83985/1,82825 m encoder szerinti kerékút, a szakaszváltások és kifutás
mozgásával együtt. Ezért a 10 s tartáson túli mozgásra is számítani kell.
A 921–980. tickekben mindkét fizikai motorkimenet nulla, az utolsó állapot
`STOP`, `enabled=False`. Független chassis-sebesség vagy úthossz nincs bizonyítva.
