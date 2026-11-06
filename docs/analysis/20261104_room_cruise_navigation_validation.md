# Room Cruise navigációs validáció – 2026-11-04

A végleges, 120 másodperces AMR-szerű elfogadás **még nincs bizonyítva**.
A jelenleg feldolgozott 120 s-célú futás **36,349 s** acknowledged navigáció után
public status-frissességi hibával megszakadt. A robot canonical STOP és
`PASS / SHUTDOWN_SAFE_LOW` runtime-zárás mellett állt le. Az új, tmpfs-re helyezett
ephemeral status/command runtime-edge végső élő eredménye még függőben van.

## M1–M6 helyzetkép

| Mérföldkő | Igazolt rész és fennmaradó határ |
| --- | --- |
| M1 – Térértelmezés | A fizikai modell 0,36 × 0,36 m; testközép a tengelytől +0,08 m, LiDAR −0,005 m. A 0,05 m ráhagyás külön marad; a footprint és pivot geometriája a tengelyorigót követi. Független fizikai akadálytávolság-pontosság nincs mérve. |
| M2 – Útválasztás | A szűk endpoint-javítás a végrehajtható téglalapos folyosót nem zárja ki pusztán körrel. A capture célváltásokat, jelölt-elutasításokat és továbbhaladást mutat; minden nehéz helyzetből helyes döntés nincs igazolva. |
| M3 – Kiszabadulás | A meglévő canonical recovery/manőverút használható és megfigyelhető. A sarkok, folyosóvégek, hátraarc és tolatás teljes fizikai elfogadása nincs lezárva. |
| M4 – Stabil mozgás | A mintavett mozgás és a tickenkénti szemantikai átmenetek elemezhetők. A 120 s futást igazolt host/runtime-edge késés szakította meg; a tartós akadozásmentesség nincs bizonyítva. |
| M5 – Tartós tájékozódás | Qualified Spatial coverage/frontier lekérdezés passzívan megőrizhető. Történeti Atlasz és jelenlegi koordináták igazolt illesztése, illetve visszatérés a kiindulópontra nincs bizonyítva. |
| M6 – AMR-validáció | A teljes 120 s és az ismételt fizikai elfogadás függőben. A capture-integritás vagy a Python skill eredménye önmagában nem AMR-siker. |

Az L12 végső safety authority megmaradt. A változtatások a meglévő adapterekre,
geometriára és observation-élekre korlátozódnak; nincs új command- vagy
motor-authority.

## Az 5 Hz-es bizonyíték tartalma

A FULL 5 Hz capture mintavett L3–L12 állapotokat, command/mission lineage-et,
health-et és mozgásadatokat tartalmaz. Az önálló decision recorder minden
completed tick szemantikai változását vizsgálja: a rövid L9/nulla motor-output
átmenetek két 5 Hz-es minta között is megmaradnak. Egy nem nulla sebességrámpa
önmagában nem vált ki új `motion` eseményt.

Az esemény legfeljebb egy typed `CommandIngressTiming` értéket is megtart:
revision, issue, reader receipt, closure/expiry és az előző publication időadatait.
A puszta heartbeat revision-váltás nem generál új eseményt. Raw szenzorpayload,
teljes térkép/rollout és replay-checkpoint nem kerül ebbe a scope-ba.
**Raw evidence és exact replay: NOT_REQUESTED.**

A flight-recorder és mozgásdiagnosztikai célzott ellenőrzés **40 passed**.
A most feldolgozott futás előtti status-egress teszt **18 passed**, a release gate
**23 passed** volt. Az új [RAM-mailbox helper](../../v3/live_io_paths.py)
biztonsági ellenőrzése **17 passed**; a közvetlen production caller-wiring
(agent config tools, launcher discovery, voice orchestration, camera observation)
ellenőrzése **38 passed**. A végső release gate/live eredmény még hozzáadandó.

## A megszakadt 120 s-célú futás első hibás boundaryja

Authority:
[v3_20261104_163946_3573_capture.mcap](../../runtime/captures/v3_20261104_163946_3573_capture.mcap).
SHA256: `02bd8f403f30e74143246ef789b19811e90ad2dc7e3831fef086312332b1cf50`.
Skill: `9f22e4435530407b9b8f9341a212c731`;
command: `operator-roomcruise-1793806794697224049-3643`.
Acknowledged start **581205196523 ns**, host-hiba **617554094427 ns**.

Az első production STOP **tick 1641 / 617468547553 ns**:
`resident.mailbox.expired.365.1641`, tehát canonical command TTL-lejárat,
**85,55 ms-mal a host public status-hibája előtt**.

| Boundary-adat | Rögzített érték |
| --- | ---: |
| Rev365 issue / reader receipt | 616949691841 / 616959901582 ns |
| Rev365 expiry, 500 ms TTL | 617449691841 ns |
| Rev365 publication return | 616950032642 ns |
| Következő rev366 issue / reader receipt | 618271778995 / 618278618187 ns |
| Producer issue-gap 365 → 366 | 1,322087154 s |
| Rev366 issue → reader receipt | 6,839192 ms |

A késés az új rev366 issue **előtt** van; az új revision reader transportja nem
tartott több mint egy másodpercig. Az előző publication már 0,341 ms-mal az
issue után visszatért. A rev366 pontos preparation-startja nem maradt meg a
következő STOP envelope-ban, ezért a producer preparation/I/O/scheduler fázisa
nem választható tovább szét.

Ezzel egyidejűleg a status sidecar a **1625. tick** file-write műveletében
**617036603086–618271868034 ns**, azaz **1,235250410 s** wall időt töltött.
Az összes running write thread-CPU maximuma **1,944 ms**, a production tickperiódus
maximuma **98,577 ms**. Ez bizonyított status-write/producer publication késés,
nem 500 ms-os L0–L12 tick-kiesés. A wall mínusz thread-CPU önmagában nem különíti
el a fizikai storage-várást és a scheduler/GIL várakozását.

A rev366 issue a status-write visszatérése előtt **89,039 µs**-mal történt.
A régi command a host-abort után röviden újra ACTIVE lett, **tick 1681** mellett
pozitív motor-outputtal, mielőtt a cleanup STOP megérkezett. Ezért mind a status,
mind a command ephemeral mailbox helye érintett a szűk runtime-edge javításban;
a persistent capture és térbeli tudás ettől külön marad.

## Offline mérések és elfogadási korlátok

Az explicit MCAP Evidence Compiler **COMPLETE**, a source-hash ellenőrzése
**PASS**: **2313 message**, quarantine **0**, capture ingress-loss **0**.
A scope **209 mintavett tick + 1039 decision event**, köztük 17 `motion` átmenet.
Qualified mintavett L3-úthossz **11,407 m**, 207 tracking-pár;
lineáris/szögsebesség RMSE **0,05994 m/s / 0,15310 rad/s**. Sampled no-progress
epizód nincs. Ezek becslés-alapú kapcsolatok, nem fizikai ground truth.

Native session timing: control wall p99/max **37,6 / 90,299 ms**;
periódus p99/max **56,5 / 98,577 ms**. A futás végén a lokális pozíció, heading és
globális pozíció GOOD; a legutóbbi elfogadott relative interval kora 119,857 ms.
A megállás első bizonyított oka itt nem a lokalizáció elvesztése.

Ideiglenes offline eredmények:
[összesítés](/tmp/r2b4-roomcruise120-5hz-7pxfaxxg/offline_summary.json),
[ingress boundary](/tmp/r2b4-roomcruise120-5hz-7pxfaxxg/command_ingress_boundary.json),
[EVI ellenőrzés](/tmp/r2b4-roomcruise120-5hz-7pxfaxxg/evi_verify.json).
Az eredeti MCAP a persistent authority; a `/tmp` exportok nem tartósak.

Nem bizonyított még a teljes 120 s önálló navigáció, az ütközésmentesség és emberi
kiszabadítás hiánya független megfigyeléssel, a minden nehéz helyzetből történő
kijutás, a történeti Atlasz illesztése vagy a kiindulópontra visszatérés.
A végső élő futás, annak külön operator-megfigyelése és mérföldkő-értékelése még
hozzáadandó.
