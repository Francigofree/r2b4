# Scan matcher korszerusites — 2026-07-23

- Lezarasi allapot: `FINAL / PROTECTED`
- Stabil contract: `R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1`
- Confidence contract: `R2B4_SCAN_MATCH_CONFIDENCE_V2`
- Ujranyitasi rend: `STRUKTURALIS_RETEGEK.md` `D012`

## Eredmeny

A scan matcher kulon processzbe kerult, bounded latest-only IPC-vel. A raw
LIDAR safety snapshot nem var a matcherre, az 50 Hz-es control loop csak
immutable, mar validalt eredmenyt olvas. Az EKF pose SSOT, a motor-output ut,
a safety-, freshness-, minosegi es timing-kuszobok nem valtoztak.

A matcher lokalizacios alapnak alkalmas: az offline geometriai, confidence-,
lineage-, processz- es no-motion timing validacio PASS, az elo M0 PASS.
A teljes robot magasabb navigacios release-readiness-e azonban meg nem PASS:
a ket elo M1 valtozatlan matcher mellett kereksebesseg-kovetesi minosegi
kapukon bukott. Ez nem matcher-, safety-, timing- vagy lineage-hiba, de
emberkovetes vagy mas magasabb mozgaslogika release-kapuja elott kulon le kell
zarni.

## Rendszerszintu veglegesites

A mukodo megoldas nem csak dokumentalt implementacio:

- a canonical azonosítok egyetlen kod-SSOT-ja
  `middleware/scan_matcher_contract.py`;
- a `LidarService` konstrukcio fail-closed elutasitja a `spawn`, `1/1`
  queue vagy `0.25/0.25 s` freshness szerzodestol eltero explicit configot;
- minden child input/result IPC contract-ID-t visz, es a parent eltero vagy
  hianyzo ID-val nem publikal matcher-resultot;
- a contract-ID es a teljes runtime contract a LIDAR odometry/status feluletre
  is eljut;
- `project_rules/protected_baseline.json` rogziti a konfiguraciot, kodszintu
  konstansokat, kotelezo/tiltott source tokeneket es raw-before-IPC sorrendet;
- a bootstrap guard megallitja a threades matcher worker, egynel nagyobb
  queue, stale/ID gate eltavolitas, confidence V2 csere vagy raw safety
  blokkolas visszahozasat;
- a stabil architekturalis SSOT `D012` dontese csak dokumentalt offline A/B,
  teljes lineage/safety review, friss M0 es ket M1, valamint kulon felhasznaloi
  jovahagyas mellett nyithato ujra.

## Regi algoritmus es bizonyitott gyengesegek

A regi `middleware/scan_matching.py` determinisztikus, hierarchikus
coarse-to-fine scan-to-map keresest vegzett az EKF pose seed kore. A pontok es
a lokalis keyframe-map kozotti legkozelebbi szomszedot SciPy KDTree adta; a
matcher ezutan lokalis x/y/yaw finomitast futtatott. A stateful
`LidarEstimator` legfeljebb 40 keyframe-et es keyframe-enkent legfeljebb
96 pontot tartott, 2.5 m lokalis sugarban.

A driver es a matcher worker thread ugyanabban a CPython runtime processzben
futott. A bemenet ugyan latest-only jellegu volt, de egy mar futó regi scan
eredmenye egy uj raw scan utan is publikalhato volt. A matcher GIL- es
eroforras-koltsege nem volt processz-szinten elvalasztva a control runtime-tol.

A regi confidence lenyegeben `1 / (1 + cost / 0.02)` volt. Nem merte kulon:

- az inlier aranyt es a robust residualt;
- a scan szogszektoros lefedettseget;
- egy kozel azonos alternativ minimumot;
- az x/y/yaw megfigyelhetoseget;
- a degeneralt, ritka, reszleges vagy dinamikus geometriat.

Az aszimmetrikus offline esetekben a regi mag 0–3.4 cm es 0–1 fok hibat adott
kb. 18–23 ms alatt, tehat az alapgeometria hasznalhato volt. Ismetlodo
periodikus geometriaban viszont reprodukalhato volt 0.50 m hibas basin
`0.9994` confidence mellett. Ez a nyers residual-confidence bizonyitott,
kritikus gyengesege.

IDLE allapotban torteno kezi robotelmozdulas utan az EKF es a lokalis
matcher-map a regi frame-et tarthatta. Egy valos preflightban ez
`0.234 < 0.250` confidence miatt fail-closed megallast okozott.

## Dontes es algoritmikai megoldas

Teljes matcher-mag csere helyett a meglevo mag celzott fejlesztese bizonyult
minimalisabb kockazatunak:

- a coarse-to-fine SE(2) kereses es az EKF-seed jo geometriai eredmenyt adott;
- a nearest-neighbour resz `scipy.spatial.cKDTree`, tehat a legdragabb belso
  kereses natív kodban fut;
- a fo hiba a score/confidence es a vegrehajtasi izolacio volt, nem az
  alapveto transzformacios konvencio.

Az uj score a legrosszabb residualok determinisztikus trimmingjet, tavolsag-
clippinget, robust RMSE-t, inlier aranyt es angular-sector lefedettseget
hasznal. A kereses utan alternativ x/y/yaw minimumokat es veges perturbacios
megfigyelhetoseget mer. A `R2B4_SCAN_MATCH_CONFIDENCE_V2` a residual-, inlier-,
coverage-, uniqueness- es observability-komponenseket kulon publikalja.
Degeneracio vagy runtime-budget tullepes explicit okkal, alacsony/zero
confidence-szel zar.

Az aktiv fast/slow matcher budget `45/120 ms`. A min-confidence, EKF gate,
safety-confirm count es mas minosegi kuszob nem lazult.

## Processz- es adatfolyam-szerzodes

`LidarService` inditaskor egy `spawn` matcher processzt hoz letre. A teljes
stateful `LidarEstimator` ebben a processzben el; az input es output queue
egy-egy bounded single-slot IPC.

1. Minden uj, teljes raw scan egy input event.
2. Queue-olvasaskor csak a legfrissebb event marad; regi queued scan eldobodik.
3. A child egy result eventet ad, szinten replace-latest szemantikaval.
4. A parent csak akkor publikal, ha az estimator-generation egyezik, a result
   pontosan az aktualis raw scan ID-hoz tartozik, es az eredmeny friss.
5. Reset vagy estimator-csere generationt novel, uriti mindket queue-t es nem
   var a mar futo regi szamitasra. Annak kesoi eredmenye fail-closed eldobodik.

A canonical `raw_scan_id`, `matcher_result_id/candidate_id`,
`lidar_odometry_measurement_id`, freshness, deduplikacio es fail-closed
lineage megmaradt. A raw safety snapshot a parent driver threadben azonnal
frissul, a matcher processz nem tulajdonosa safety-, EKF- vagy motorallapotnak.

## Manualis robotmozgatasi szerzodes

A robot kezzel mozgathato, amikor nincs aktiv elo tesztszakasz, a runtime
`IDLE`, es a PWM `0/0`. M0/M1 alatt csak a deklaralt 10 masodperces esetkozi
szunetben szabad elmozditani.

M0 es M1 most a preflight elott canonical `reset_pos` reanchort futtat.
Minden 10 masodperces szunet vegen ugyanez tortenik, majd a Hub megvarja az uj,
stabil measurement-ready allapotot. Aktiv primitive alatti kulso elmozdulas
tovabbra is rendellenesseg, es nem kap bypass-t.

Elo bizonyitek:

- M0 preflight reset: effective; a harom szunet mindegyike reset + ready PASS.
- Elso M1: 7/7 szunet/reset PASS.
- Masodik M1: 7/7 szunet/reset PASS.

## Offline geometriai es confidence-validacio

Az alabbi determinisztikus esetek ugyanazzal az uj V2 score-ral futottak:

| Eset | Poziciohiba | Yaw-hiba | Confidence | Degeneralt |
|---|---:|---:|---:|---|
| static | 0.000 m | 0.0000 rad | 0.844 | nem |
| straight | 0.0369 m | 0.0000 rad | 0.260 | nem |
| reverse | 0.0325 m | 0.0000 rad | 0.249 | nem |
| pivot | 0.000 m | 0.0000 rad | 0.784 | nem |
| ARC | 0.0270 m | 0.0025 rad | 0.466 | nem |
| 62% dropout | 0.0355 m | 0.0050 rad | 0.156 | igen, ambiguous |
| 45 dinamikus pont | 0.0313 m | 0.0050 rad | 0.341 | nem |
| ismetlodo geometria | 0.100 m hibas basin | 0.0000 rad | 0.082 | igen, ambiguous |

A dropout-eset geometriailag meg kozel maradt, de helyesen bizonytalannak
minosult. Az ismetlodo eset a regi magas-confidence/nagy-pose-hiba
regresszioja: az uj modell nem fogadta el magabiztos mereskent.

Kalibracios mini-set:

- Brier score: `0.1520`;
- hasznalhato esetek elfogadasa `3/3 = 100%`;
- rossz/degeneralt eset elfogadasa `0/1 = 0%`.

Offline regressziok:

- quality/convention/process: `39 passed`;
- teljes celzott LIDAR-csomag: `80 passed`;
- processz/RSS/queue/lineage/diagnosztika: `22 passed`;
- Hub preflight/reset: `3 passed`;
- manual-reposition validator: `1 passed`;
- change-tracker MD/manifest tranzakcio: `10 passed`;
- vegleges contract/quality/process/lineage celzott regresszio: `86 passed`;
- teljes repository regresszio a vedett contracttal: `1065 passed`;
- bootstrap: `PASS`.

## Eroforras- es control-loop meres

A produkcios Raspberry Pi 5 runtime 10 Hz raw scan mellett:

- matcher aktualis RSS: `53,808–54,096 kB`, azaz `52.5–52.8 MiB`;
- matcher lifetime peak RSS: `250,736 kB`, azaz kb. `244.9 MiB`
  (import/spawn high-water mark);
- 5 s ablak matcher CPU: `0.447` CPU-mag;
- legutobbi CPU/wall matcher futas: kb. `39–43 / 43 ms`;
- teljes IPC latency az idle ablakban: p50 `49.3 ms`, p95 `57.4 ms`;
- az elo sorozat utan: p50 `50.5 ms`, p95 `62.6 ms`, max `171.3 ms`;
- input queue drop `0`, output queue drop `0`, processz-hiba `0`;
- post-run queue-melyseg `0/0`;
- stale result drop `700`: ezek superseded eredmenyek, amelyeket a parent
  szandekosan nem alkalmazott; backlog vagy stale EKF-alkalmazas nem tortent.

A regi threades matcherhez kepest egy kulon kb. 53 MiB current-RSS processz
jelent meg, cserebe a matcher CPU/GIL vegrehajtasa elvalt a control threadtol.
A CPU-affinity tovabbra is control CPU `3`, service/matcher CPU `0–2`.

Vegleges contract-futas:
`hub_M3_motion_runtime_profile_no_motion_live_20260723T200149Z`: PASS.

- watchdog p50 `50.00 Hz`, p10 `49.25 Hz`;
- loop-budget EMA p50 `4.5005 ms`, p95 `5.3943 ms`;
- `4/200` slow tick, maximum scheduler-delay `17.035 ms`;
- LIDAR slow-spike `0`, GC `0`;
- logger drop `0`, write error `0`.

Ez bizonyitja, hogy a matcher nem blokkolja az 50 Hz-es control loopt. A kulon
processz sajat CPU- es memoriaigenye merheto, bounded es queue-backlog nelkuli.

## Elo M0/M1 eredmenyek

### M0

`hub_M0_measurement_trust_live_20260723T185248Z`: PASS.

- 4/4 eset PASS;
- aktiv encoder timing-gap `0`;
- missing timing-contract `0`;
- observation/lineage error `0`;
- safety-stop `0`;
- matcher elfogadas `45/54 = 83.3%`;
- egyedi EKF-meres / egyedi matcher-result `33/37 = 89.2%`;
- case confidence medianok `0.689–0.757`.

### Elso M1

`hub_M1_motion_baseline_live_20260723T185413Z`: FAIL.

Mind a 8 eset lefutott; matcher-lineage, aktiv timing es safety kapuk tisztak.
A matcher elfogadas `128/148 = 86.5%`, az egyedi EKF-meres/matcher-result
`86/100 = 86.0%`.

Kizarolag settled wheel-speed tracking hibak:

- start response: `0.02552 m/s`;
- bal ARC: `0.02254 m/s`;
- jobb ARC: `0.01821 m/s`.

### Masodik M1

`hub_M1_motion_baseline_live_20260723T185633Z`: FAIL.

Ismet 8/8 eset vegrehajtva, 7/7 reset PASS, aktiv timing-gap `0`,
lineage-hiba `0`, safety-stop `0`. Matcher elfogadas
`129/150 = 86.0%`, egyedi EKF-meres/matcher-result `92/101 = 91.1%`.

Kizarolag settled wheel-speed tracking hibak:

- start response: `0.01529 m/s`;
- backward: `0.01745 m/s`;
- jobb ARC: `0.03526 m/s`.

A ket M1 FAIL nem indokol matcher-, EKF-, safety- vagy kuszobmodositast.
A fennmarado blocker a kozos kereksebesseg-kovetesi/motion-quality retege,
amely kulon, bizonyitekvezerelt feladat.

## Vegso alkalmassagi allitas

A matcher alkalmas az EKF, a lokalis terkep, a visszateres es kesobbi
magasabb navigacios logikak lokalizacios alapjanak tovabbfejlesztesere:
non-blocking, latest-only, stale-safe, bounded, geometriailag robusztus es
kalibralhato confidence-t ad.

Az egesz robotot azonban meg nem szabad emberkovetesre vagy magasabb
navigacios release-re alkalmasnak minositeni. A matcher oldala lezart, de a
ket M1-ben ismetlodott settled wheel-speed minosegi hibakat a valtozatlan
kapukkal kulon le kell zarni.
