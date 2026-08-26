# M4 parkolt hibak es gyokerok

- Nyilvantartasi allapot: `PARKOLT`
- Minden tetel jelolese: **KÉSŐBB ELLENŐRIZENDŐ**
- Letrehozva: `2026-07-19`
- Hivatkozott elo futasok:
  - `M4_room_cruise_quality_validator`, Hub start `2026-07-18T22:10:02Z`
  - `M4_1_room_cruise_quality_validator`, Hub start `2026-07-18T22:57:06Z`

Ez a dokumentum nem aktiv fejlesztesi terv. A tetelekhez most nem keszul
javitas vagy tovabbi elo validacio. A „velt gyokerok” a strukturalt mintak,
kapuk es az aktualis kodut egybeeso bizonyitekai; ahol a bizonyitek nem zarja
ki az alternativ okot, azt kulon jelezzuk.

## Kotelezo kesobbi triage-szabaly

Uj mozgas-, lokalizacios vagy runtime-anomalianal a hibat ossze kell vetni az
alabbi felismeresi jelekkel. Egyezes eseten a riportban szerepeljen:

`LEHETSÉGES KAPCSOLAT: M4-Dxx – KÉSŐBB ELLENŐRIZENDŐ`

Ez nem automatikus gyokerok-megallapitas. Az aktualis strukturalt artefaktummal
ujra igazolni kell a kapcsolatot. A parkolt tetel csak explicit felhasznaloi
dontessel nyithato ujra.

## M4-D01 – Primitive-handoff es PWM ugrasa

**KÉSŐBB ELLENŐRIZENDŐ**

- Bizonyitott tunet:
  - Az eredeti M4 handoff P95: vegrehajtasi kerekhivatkozas `0.20848 m/s`,
    mert kereklepes `0.16782 m/s`, tenyleges `dv=0.16858 m/s`,
    `domega=0.28663 rad/s`, PWM `0.46013`.
  - Az M4.1 candidate kerekhivatkozas P95-ja `0.10270 m/s`-ra es PWM-je
    `0.35196`-ra csokkent, de a mert kereklepes `0.18284 m/s`, a tenyleges
    `dv=0.18789 m/s` es `domega=0.31160 rad/s` maradt vagy romlott.
  - A legnagyobb esemenyek pivot/iv es iv/pivot hatarokon jelentek meg.
- Logokkal/koddal tamogatott velt gyokerok:
  - **Magas bizonyossag:** a diszkret primitive-intent TRACK
    kerekhivatkozassa alakul, a kozos shaping pedig `v/omega` tengelyen
    korlatoz; nem primitive-fazis-, kerekirany- vagy wheel-jerk-tudatos.
  - **Kozepes bizonyossag:** az iranyvalto kerek nullaatmenete es a
    `0.15 m/s`-tol indulo feed-forward tartomany egyutt nagy PWM
    elojel/amplitudo valtozast hozhat. A korrelacio bizonyitott, a teljes
    oksagi lanc kulon direction-switch merest igenyel.
- Kesobbi felismeresi jel: lathato rantás primitive-valtaskor; handoff
  `target/actual wheel`, `actual v/omega` vagy PWM P95 kapu egyideju FAIL-je.

## M4-D02 – Iranyfuggo kerek- es linearis tracking hiba

**KÉSŐBB ELLENŐRIZENDŐ**

- Bizonyitott tunet:
  - Eredeti M4 linear tracking P90 `0.05425 m/s`; bal/jobb forward P90
    `0.03653/0.03737 m/s`, jobb reverse `0.04477 m/s`.
  - M4.1 candidate linear P90 `0.06151 m/s`; settled M4.1 linear P90
    `0.04230 m/s`, bal/jobb settled wheel P90 `0.03884/0.04963 m/s`.
  - Az M4.1 M3 bontasban bal/jobb forward FAIL (`0.04233/0.05082 m/s`),
    bal/jobb reverse PASS (`0.01995/0.02981 m/s`); wrong-sign ratio minden
    elegendoen mintazott iranyban nulla.
- Logokkal/koddal tamogatott velt gyokerok:
  - **Magas bizonyossag:** nem ownership-, route- vagy elojelhiba; a futas
    UNIFIED/TRACK lanc-, ownership- es forbidden-path kapui PASS-ok.
  - **Kozepes bizonyossag:** irany- es amplitudofuggo aktiv speed-map/PID
    magnitude mismatch, amelyet a sok atmeneti minta tovabb erosithat.
    A konkret map/PID parameterhiba meg nincs iranyonkenti elo
    azonosito meressel bizonyitva.
- Kesobbi felismeresi jel: helyes kerek/PWM elojel mellett novekvo P90
  sebesseghiba, kulonosen forward iranyban vagy ivben.

## M4-D03 – Endpoint heading szenzor/EKF szetcsuszas

**KÉSŐBB ELLENŐRIZENDŐ**

- Bizonyitott tunet:
  - Eredeti M4 max endpoint par-elteres `24.31 deg`; az EKF, encoder es LIDAR
    egymashoz kozel maradt, a legnagyobb elteres az integralt IMU-aghoz kotodott.
  - M4.1: kb. `608 deg` osszfordulasnal EKF/IMU `31.08 deg`, EKF/encoder
    `5.71 deg`, EKF/LIDAR `18.60 deg`; same-snapshot encoder/IMU rate P90
    `0.07193 rad/s`, contradiction sample `0`.
- Logokkal tamogatott velt gyokerok:
  - **Kozepes-magas bizonyossag:** kumulativ gyro-integracios bias es/vagy
    endpoint idobazis-illesztes; a helyes elojel es a jo same-snapshot rate
    nem tamaszt frame-elojel vagy azonnali szenzorellentmondas hibajat ala.
  - **Kozepes bizonyossag:** az aszinkron LIDAR endpoint mintavalasztas is
    novelheti a par-elterest, de nem magyarazza egyedul az IMU maximumot.
- Kesobbi felismeresi jel: hosszu, sok fordulast tartalmazo futasban novekvo
  endpoint drift, mikozben same-snapshot rate es frame/elojel konzisztens.

## M4-D04 – Intermittalo runtime/SD I/O teljesitmeny-anomalia

**KÉSŐBB ELLENŐRIZENDŐ**

- Bizonyitott tunet:
  - Az eredeti M4 control-loop gate hatarertek felett volt:
    `frequency_below_45_ratio=0.103586` a `0.10` limithez kepest.
  - Az M4.1 control-loop timing PASS (`0.01581`, loop budget P95
    `9.871 ms`), mikozben a hardware performance gate `1090.92 ms` SD-write
    peak miatt FAIL; CPU P95 `30.9%`, homerseklet legfeljebb `52.4 C`,
    throttling `0`.
- Logokkal tamogatott velt gyokerok:
  - **Kozepes bizonyossag:** intermittalo host/storage I/O vagy scheduler
    stall, nem tartos CPU- vagy thermal limit. Az M4 es M4.1 eltero timing
    eredmenye nem tamaszt stabil vezerlesi ciklus-koltseg regressziot ala.
- Kesobbi felismeresi jel: ritka nagy status/logger/SD kesleltetes, lassu
  tick burst vagy valtozo `below45` ratio normal CPU/homerseklet mellett.

## M4-D05 – Tul lassu slew es a `0.15 m/s` steady minimum serulese

**KÉSŐBB ELLENŐRIZENDŐ**

- Allapot: az ezt okozo elso candidate elutasitva es visszaallitva; nem aktiv
  runtime-hiba.
- Bizonyitott tunet: M4.1 candidate settled non-pivot minimum `0.12600 m/s`,
  a `0.145 m/s` kapu alatt.
- Logokkal/parametermatematikaval bizonyitott velt gyokerok:
  - **Magas bizonyossag:** a candidate `0.35 m/s2` linearis gyorsitasa
    nullarol legalabb `0.429 s` alatt eri el a `0.15 m/s` munkapontot, mikozben
    a beallt meresi ablak `0.30 s` utan kezdodik.
  - Literalissan folytonos elojelvaltasnal a fizikai jelnek nullan es a
    `0-0.15 m/s` tartomanyon at kell haladnia; a `0.15 m/s` kovetelmeny a
    steady/nevleges munkapontra ertelmezheto.
- Kesobbi felismeresi jel: uj smoothing candidate utan `steady_motion_minimum`
  FAIL, lassu elindulas vagy `0.30 s` utan is `0.145 m/s` alatti nem-pivot cel.

## M4-D06 – Primitive-osztalyon beluli fizikai spike-ok

**KÉSŐBB ELLENŐRIZENDŐ**

- Bizonyitott tunet: M4.1 globalis max mert kereklepes `0.25772 m/s`,
  tenyleges `domega=0.76355 rad/s`, PWM `0.41570`; tobb top esemeny azonos
  `left_arc -> left_arc` vagy `right_arc -> right_arc` osztalyon belul tortent.
- Logokkal tamogatott velt gyokerok:
  - **Magas bizonyossag:** nem minden spike magyarazhato primitive-classifier
    valtassal.
  - **Kozepes bizonyossag:** clearance/reference frissites, kerek-hurok
    magnitude-tracking es a ritkabb statusmintazas egyuttesen emeli a ket
    megfigyelt frame kozotti lepest. Az egyedi komponens hozzajarulasa nincs
    meg szetvalasztva.
- Kesobbi felismeresi jel: nagy actual/PWM lepes valtozatlan `m3_class`
  mellett; clearance- vagy target-frissites idobeli egybeesese.

## M4-D07 – Egyszeri execution-contract anomalia az eredeti M4-ben

**KÉSŐBB ELLENŐRIZENDŐ**

- Bizonyitott tunet: az eredeti M4 strukturalt mintasoraban egy normal TRACK
  pivot mintat `execution_contract_violation` jelolt, mikozben forbidden path,
  owner conflict es route hiba nem volt. Az M4.1 futasban ugyanez a szamlalo
  `0` volt.
- Velt gyokerok:
  - **Alacsony-kozepes bizonyossag:** classifier/status-snapshot idozitesi
    anomalia vagy `local_nav_pivot` contract-jelolo rovid inkonzisztencia.
    Ismetlodes nelkul runtime ownership-hiba nem allithato.
- Bizonyitekmegorzesi korlat: az eredeti reszletes `latest` scenario artefaktot
  a kesobbi M4.1 kompozicio felulirta; a futas Hub metaadata es a korabban
  rogzult strukturalt triage-allapot maradt meg.
- Kesobbi felismeresi jel: uj `execution_contract_violation` ugy, hogy
  `forbidden_path=0`, `owner_conflict=0`, route es TRACK execution helyes.

## M4-D08 – Futasazonos emberi minosegbizonyitek hianya

**KÉSŐBB ELLENŐRIZENDŐ**

- Ez bizonyitasi hiany, nem vezerlesi hiba.
- Bizonyitott tunet: az M4/M4.1 human visual gate `INCONCLUSIVE`, mert nem volt
  az aktualis `evidence_id`-hoz kotott strukturalt teljes-futasos megfigyeles.
- Kesobbi felismeresi jel: kvantitativ PASS mellett a teljes verdict meg mindig
  `INCONCLUSIVE`; csak azonos `evidence_id`, teljes 60 s megfigyeles,
  noticeability `<=1/5` es nulla abrupt event zarhatja le.

## Nem hibakent parkolt, mar bizonyitott tulajdonsagok

- Az M4.1 futas akadalyfuggo sebessegszabalyozasa PASS: open p50 `0.300 m/s`,
  near p50 `0.18812 m/s`, monoton clearance-policy.
- Minimum clearance `0.5442 m`, safety/nonfinite esemeny `0`.
- UNIFIED mode, TRACK route, ownership, forbidden/legacy/direct path es
  execution-contract az M4.1 futasban PASS.
- A futas utan friss IDLE, cel `0/0`, PWM `0/0` igazolt; runtime leallitva.

## Elsoleges bizonyitekforrasok

- `logs/latest/latest_M4_1_room_cruise_quality_validator.json`
- `logs/latest/latest_M4_1_room_cruise_quality_validator_summary.json`
- `logs/latest/latest_M3_room_cruise_unified_validator.json`
- `logs/latest/M4_1_room_cruise_quality_validator_samples.jsonl`
- `logs/session_<timestamp>/summary.json`
- `logs/session_<timestamp>/incident_bundle.json`

Nyers log ujraolvasasa csak akkor indokolt, ha a fenti strukturalt artefaktumok
egy kesobbi anomalia kapcsolatat nem tudjak eldonteni.
