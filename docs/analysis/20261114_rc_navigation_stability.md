# RC navigáció stabilizálása — 2026-11-14

**Az élő elfogadás sikertelen.** Az új célválasztás, kerékadapter és recovery
szoftveres és motormentes bizonyítékot kapott, de a szobában tartós, önálló
navigációt nem sikerült igazolni. Az engedélyezett legfeljebb három új fizikai
próbából **egy** futott. A bridge váratlan lifecycle-frissességi hibája után az
AGENTS.md élő hardverre vonatkozó szabálya szerint nem indult automatikus újrapróba.
A többi próba külön ROS domainben, szintetikus érzékelőkkel, motor-authority nélkül futott.

A feladat előtti két lezárt MCAP-ban 78/276, illetve 46/258 aktív megfigyelésben
volt jelentős nyers MPPI-javaslat mellett nulla projektált parancs. Ezek
[baseline-összesítései](../../tmp/rc_stability_20261114_65kx8jpc/v3_20261114_221325_22218_capture_baseline.json)
és [a későbbi baseline](../../tmp/rc_stability_20261114_65kx8jpc/v3_20261114_221556_24294_capture_baseline.json)
alátámasztják a jelzett beragadást. Eltérő útvonalak és eltérő élettartamú aktív
ablakok miatt ezekből kontrollált A/B javulási arány nem állítható.

A konkrét változtatások:

- Az MPPI-specifikus kerékprojekció az eredeti kerékirány mellett kerekenként
  STOP-ágat is vizsgál, ellentétes kerékirányoknál tiszta forgást is megenged.
  Test- és kerékamplitúdót nem növel, kereket nem fordít meg. A stabil V3
  projekció alapértelmezett szemantikája megmaradt.
- A test lineáris sebességét büntető VelocityDeadbandCritic helyére natív,
  kerékenkénti `R2B4WheelCritic` került. A fizikai minimum 0,15 m/s; a 0,19 m/s
  célminimum nem új kemény feltétel a tiszta forgásra. A PathAngle irányfüggetlen,
  a PreferForward súlya 5-ről 1-re csökkent. A hátramenet maximuma 0,20 m/s;
  BackUp 0,19 m/s, a forgás a launcher fizikai kerékhatáraihoz igazodik.
- A V3 50 Hz-es frekvenciája, L9 0,6 m/s² és 2,5 rad/s² tényleges gyorsulási
  korlátja megmaradt. Az MPPI javaslati gyorsulása átlépheti a holtsávot;
  a tényleges végrehajtási ramp továbbra is V3-feladat.
- A célválasztó már a teljes aszimmetrikus téglalapot ellenőrzi. A fizikai test
  0,36 × 0,36 m, a tengelyhez képest x-irányban +0,08 m-rel eltolt:
  x=[−0,10; 0,26], y=[−0,18; 0,18] m. Az egyszer alkalmazott padding 0,10 m,
  így az ellenőrzött téglalap x=[−0,20; 0,36], y=[−0,28; 0,28] m.
- A túl széles puha zóna sugara 0,55-ről 0,50 m-re csökkent, lecsengése 6-ról
  8-ra nőtt. Megszűnt a célközéppont `cost<=60` feltétele. A teljes téglalappal
  az OccupancyGrid lethal=100 és unknown=−1 cellái tiltottak; az infláció 1–99
  költsége nem második testmargin. A szerződés szerinti 10 cm-es L12 STOP-határ
  és a mért fizikai méret nem csökkent indokolatlanul.
- Frontier esetén a centroidtól a robot felé legfeljebb 0,60 m-rel beljebb
  keres igazoltan szabad megközelítési célt. Így a teljes test ellenőrzése nem
  tesz automatikusan használhatatlanná minden szabad/ismeretlen határpontot.
  Ismert szobában legfeljebb 32 rangsorolt cél kap teljes testellenőrzést;
  nincs ellenőrizetlen fallback.
- A fix Wait/Spin/BackUp BT-sorozat helyére a meglévő ROS bridge-ben egy
  korlátozott recovery-epizód került. Friss, legfeljebb 0,5 s-os teljes local
  costmap alapján választ hátramenetet, kétirányú forgást vagy kerülőt.
  A local costmap publikálása 1-ről 5 Hz-re nőtt. Kerülőnél valódi Smac-terv
  készül, a teljes interpolált tervet is ellenőrzi a teljes testtel.
- Az előző action terminális eredményét megvárja; a cancel-ACK önmagában nem
  elég. A sikertelen manőver az adott epizódon belül kizárt. Kimerüléskor a cél
  a meglévő célválasztó tiltólistájára kerül. Wait nem bizonyíthat kiszabadulást.
  BackUp/Spin/kerülő sikere után az eredeti navigációban legalább 0,25 m
  odometriai elmozdulás és 0,15 m tervhaladás, vagy az eredeti cél elérése kell.
- Az adapter a végleges parancs egy másodperces körívén a teljes téglalap
  belsejét is ellenőrzi, legfeljebb félcellányi testpont-elmozdulással.
  Az elutasítás/projekció 16 elemű natív eseménytárba kerül; JSON csak 5 Hz-en
  készül. Raw/projected/forwarded parancs, keréksebesség, mért sebesség, pose,
  vizsgált körív, első tiltó cella és ok együtt kerül az MCAP-ba. A célválasztás
  és recovery külön, szintén korlátozott eseménytárat és kumulatív számlálókat ad.
  Nyers térkép és diagnosztikai tömegadat nem került a control interpreterbe.

Az élő parancs `./r rc 60 c full` volt. A capture:
[20261114_224950](../../runtime/captures/v3_20261114_224950_31375_capture.mcap),
SHA-256 `d45e5935f63531e5838eb6c032141c03fcedb1a630cd5f32706b31d0321c9283`.
A teljes mérés, események és a futtatott offline elemző:
[summary.json](../../tmp/rc_stability_20261114_65kx8jpc/live1/summary.json),
[motion_events.json](../../tmp/rc_stability_20261114_65kx8jpc/live1/motion_events.json),
[analyze.py](../../tmp/rc_stability_20261114_65kx8jpc/analyze.py).

| Élő mérőszám | Eredmény |
|---|---:|
| Mission-minták / mintavett időtartam | 343 / 68,40 s, az indulási várakozással |
| Első NAV2_ACTIVE / első NAV2_UNAVAILABLE | +8,38 / +24,80 s a mission első mintájától |
| NAV2_ACTIVE / STALE / UNAVAILABLE mintavett ideje | 14,22 / 2,02 / 43,60 s |
| Odometriából számolt út | 3,96 m |
| Térképi, lokalizált út | 3,52 m |
| Testtel bejárt rasterterület | 1,49 m², 0,10 m-es rasteren |
| Lefedési cél / elért cél | 2 / 1 |
| Leghosszabb transzláció nélküli és teljes megállás | 45,00 s, a kiesés körül és utána |
| Élő recovery-epizód | 0; élő kiszabadulási sikerarány nem mérhető |
| Friss, aktív controller-ablakok mediánja / minimuma | 19,91 / 18,81 Hz |
| Jelentős raw mellett nulla kimenet, aktív 5 Hz-es mintákban | 2 / 73 |
| Adapterprojekció / holtsáv miatti STOP / collision-reject | 9 / 6 / 0 natív esemény |
| Rögzített egyedi adapteresemény / kumulatív esemény | 15 / 15; overflow=0 |
| CPU a működő navigáció időablakában, átlag / maximum | 69,85% / 77,31%, összes CPU kapacitásához viszonyítva |
| V3 target / átlagperiódus / p99 / maximum | 20,00 / 20,026 / 25,50 / 40,38 ms |
| Control p99 / teljes work p99 | 9,40 / 11,50 ms |
| 20 ms feletti work / 40 ms feletti periódus | 3 / 1 |
| Mintavett legkisebb LiDAR-testclearance az aktív ablakban | 0,355 m |
| Nem nulla motor-output NAV2_UNAVAILABLE mintákban | 0 / 219 |
| SLAM és összes Nav2 folyamat CPU-maszkja | 0–3 |

A CPU-maszkot a tényleges `/proc/PID/status` állományokból ellenőriztem:
[élő affinity evidence](../../tmp/rc_stability_20261114_65kx8jpc/live1_affinity.json).
Az időzítési számok a teljes runtime-ból származnak; a hosszú álló/kiesett rész
miatt nem kontrollált teljesítmény-összehasonlítások. Szigorú jittermentesség
nem teljesült. A rasterterület és a kétféle út becslés, nem külső ground truth.

Konkrét adapterpélda az MCAP 7. eseményéből:
raw `(v,w)=(0,24484; 0,73973)`; raw kerekek `(0,11328; 0,37640)` m/s.
A bal kerék fizikai holtsávba esett. Az új projekció
`(0,13156; 0,73973)` parancsot adott, kerekek `(0; 0,26312)` m/s sebességével.
A test és a kerekek amplitúdója nem nőtt; a javaslat nem veszett el nullaként.
Az akkori mért sebesség `(0,12851; 0,21379)` volt; a kisebb mért forgás nem
azonos újabb steady projekcióval: a V3 gyorsulási ramp és a fizikai követés
külön vizsgálandó. A mission 49 mintájában volt L9 ACCELERATION_LIMIT,
MINIMUM_WHEEL_SPEED miatti újabb leállítás nem szerepelt.

A terminal hiba `NAV2_LIFECYCLE_STATUS_STALE`: a poll-köz 1,657 s,
az utolsó readiness-válasz kora 1,816 s lett. A manager utolsó válasza aktív
volt; ez a bridge readiness-frissességének elvesztése, nem bizonyított manager
deaktiválódás. A ROS logban eltűnő klienshez küldött cancel-response timeout
is szerepel. A [Jazzy rclpy válaszhívása](https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/action_server.cpp)
a natív küldés körül nem engedi el a GIL-t. Ez konkrét lehetséges blokkoló
boundary, de az élő 1,657 s pontos hívása ebből a capture-ből nem azonosítható.
A következő méréshez az executor wall/CPU maximumai bekerültek az 5 Hz-es
megfigyelésbe. A lifecycle- és command-freshness határidőket nem lazítottam.

A megosztott public host PID 7078 még 21:16-kor indult, a duration-ellenőrzést
javító forrás 21:20-as módosítása előtt. A host az élő kiesés ellenére
`COMPLETED/REQUESTED_DURATION_REACHED` eredményt adott. Az aktuális szoftveres
contract ezt a helyzetet elutasítja. Ellenőrzött nyugalmi állapotban a hostot
szabályosan újratöltöttem (PID 33251), motor-runtime indítása nélkül:
[reload evidence](../../tmp/rc_stability_20261114_65kx8jpc/host_reload.json).
Ez nem teszi utólag sikeressé az élő navigációt; az új host élő újraigazolása
nem történt meg.

A validáció:

- `./r test`: **14 passed**; `./r test release`: **24 passed**.
- Nav2/geometria/kerék/diagnosztika célzott csomag: **68 passed**; a későbbi
  frontier-inset és acceptance/behavior/geometria/diagnosztika ellenőrzések:
  **69 passed**. Ezek részben átfedő csomagok, nem összeadandó totalszámok.
- A végső natív kerék-/costmap-csomag **4 passed**, benne a pontos első tiltó
  cella és cost bizonyításával. Natív C++ fordítás az izolált production prefixbe sikerült. A kerékprojekció
  régi Python-szemantikájának egyezése, az új ágak 1000 rögzített véletlen
  esete, a változatlan V3 envelope, és valódi natív costmap
  collision/unknown/map-exit ellenőrzése külön futott.
- [Motormentes ismert szobás próba](../../tmp/rc_motorless_stability1/summary.json):
  25 s cruise, három cél, két elért cél, 5,05 m szintetikus út; nem fizikai bizonyíték.
- [Mesterséges drivetrain-elakadás](../../tmp/rc_motorless_recovery2/summary.json):
  egy epizódban három különböző manőver; egy native failure, két native success,
  egy sikertelen progress-ellenőrzés, végül egy igazolt kiszabadulás.
  A sikeres Spin önmagában nem oldotta fel az epizódot.
- [Kényszerített kerülőág](../../tmp/rc_motorless_detour1/summary.json):
  két valódi Smac-terv teljes testellenőrzése, két külön kerülő; az első
  határidőre megszakadt, a második native action sikerült. Az eredeti cél felé
  visszatérő progress az ablak végéig nem igazolódott, ezért nem recovery-siker.
- [Frontier/cancel/heartbeat próba](../../tmp/rc_motorless_frontier1/summary.json):
  pozitív canonical javaslat, a testhez beljebb választott cél,
  cancel utáni nulla mozgás és heartbeat-expiry PASS.
- Az explicit [MCAP Evidence Compiler](../../tmp/rc_stability_20261114_65kx8jpc/live1_evi.log)
  COMPLETE eredményt adott, sértetlen forrással és 3724 üzenettel.
  A [natív Replayer ténylegesen futott](../../tmp/rc_stability_20261114_65kx8jpc/live1_replay.log),
  eredménye `CAPTURE_INCOMPLETE: replay core is incomplete`. Az 5 Hz-es
  behavior-capture nem tartalmaz teljes TickInputs/checkpoint sorozatot.
  Ez nem MATCH és nem MISMATCH.

Fennmaradó konkrét problémák:

1. A lifecycle-frissesség elvesztésének konkrét hosszú hívása nincs élő
   stack-/phase-bizonyítékkal azonosítva és megszüntetve. A tartós autonóm
   cirkálás elfogadása emiatt sikertelen; új fizikai futás nem indult.
2. Az új recovery működése élő sarok-, akadály- vagy beszorulási helyzetben
   nincs bizonyítva. Motormentes siker nem helyettesíti ezt.
3. A nyers MPPI-parancs mindkét fizikai kerékminimum alatt továbbra is STOP-ra
   kényszerülhet. A mintavett sebességingadozás teljes megszűnése nem bizonyított.
4. Upstream MPPI-solver-/native behavior-hibánál a belső első ütköző cellát
   a Nav2 nem mindig közli. Az adapter és a cél/recovery preflight részletes
   rejection-evidence-e működik; az összes upstream belső elutasítás teljes
   cellaszintű megfigyelhetősége még nem teljes.
5. A V3 szigorú 50 Hz-es jitterfeltétele és a diagnosztikai többletterhelés
   kontrollált, folyamatos navigáció melletti nem-regressziója nincs igazolva.
6. Nincs élő exact-replay MATCH, külső pozíció-ground-truth, teljes
   kontaktusmentességi vagy tartós szobabejárási bizonyíték.
