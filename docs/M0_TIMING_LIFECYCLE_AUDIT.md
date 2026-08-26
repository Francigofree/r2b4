# M0 timing es lifecycle audit — 2026-07-22

## Hatokor es invariansok

Az audit ket, egymastol fuggetlen tunetet vizsgal:

1. `trust_forward_pulse:encoder_timing_gap` az aktiv 40 ms-os canonical kapun;
2. egy M0 Hub-riport utan megfigyelt `FORWARD` runtime allapot.

A timing-, safety-, confidence- es mozgasminosegi kuszobok valtozatlanok. A
LIDAR-reteg befagyasztott. Egy idoben pontosan egy live Hub-profil futhat.

## PROVEN — atfedo live profilok

A `hub_M0_measurement_trust_live_20260722T161248Z` futas
`16:12:48Z--16:13:32Z`, a
`hub_M0_measurement_trust_live_20260722T161309Z` futas
`16:13:09Z--16:13:54Z` kozott futott. Az atfedes 23 masodperc. A `FORWARD`
allapotot `16:13:52Z`-kor ellenoriztuk, amikor a masodik profil meg aktiv volt.
Ezert a megfigyeles nem runtime cleanup-hibat bizonyit; egy masodik, aktiv
validator mozgasi parancsa volt.

A forrasban a live `run_profile()` elott nem volt processzek kozotti lock. Ez
lehetove tette, hogy ket Hub-processz ugyanazt a runtime-ot, command buszt es
`latest_*` artefaktumokat egyszerre hasznalja. A javitas egy nem varakozo,
processz-szintu file lock a canonical Hub wrapperben. Foglalt lock eseten a
masodik profil a preflight es a scenario elott
`LIVE_PROFILE_ALREADY_RUNNING` hibaval leall. A lock exception utan is
felszabadul.

## Timing-gap allapot

`PROVEN`: a `160958Z` M0 es a `161248Z` M0 compact incident egyarant aktiv
forward timing-gapet nevezett meg; logger drop/write error mindket futasban
`0`. A `161248Z` run reszben atfedett egy masik profillal, es a kozos latest
scenario artefaktumot a kesobb zarodo futas felulirta, ezert a gap reszletes
fazisrekordja nem tekintheto megorizett run-azonos bizonyiteknak.

`NOT PROVEN`: encoder callback-kimaradas, GIL, lock-varakozas, logger I/O,
LIDAR matcher vagy OS scheduler mint gyokerok. Uj, kizarolagos M0 es
run-azonosan megorzott timing-record szukseges a kauzalis szukiteshez.

## Kizarolagos ujrafutas

A lockkal futtatott, egyetlen
`hub_M0_measurement_trust_live_20260722T163421Z` profil PASS. Mind a negy
esetben encoder motion/idle timing-gap delta `0`, timing-contract hiany `0`,
motion/unowned GC `0`, logger drop/write error `0`, safety-stop `0`. A run utan
egy runtime maradt, IDLE, PWM `0/0`, nulla kert es limitalt twist, motion task
nelkul, safety OK es stop NONE.

Ez bizonyitja az aktualis M0 kapu teljesuleset es cafolja a folyamatos encoder-
vagy validator-definicios hibat. Nem bizonyitja az elozo intermittalo gap
konkret GIL/scheduler/lock/I/O gyokerokat; reprodukalhato, run-azonos timing
record nelkul ezek `NOT PROVEN` maradnak.

## Regresszios szerzodes

- ket live Hub-processz nem juthat egyszerre runtime preflight/scenario agra;
- a masodik azonnal, strukturalt owner PID/profile adattal bukik;
- a lock minden normal es exception kilepesnel felszabadul;
- offline profilokra a live lock nem vonatkozik.
