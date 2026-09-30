# A 2026-09-30-i három mozgásteszt elemzése

Az elemzés a változtatás előtti source, a capture-ben tárolt aktív konfiguráció,
a natív Replayer és a meglévő Test Hub evidence alapján készült. Az eredeti
capture/evidence fájlok változatlanok. Új fizikai mozgás nem indult.

## Futások és közvetlen tények

| Futás | Capture azonosító | Tick | Átlagos control frekvencia | Leghosszabb tick-köz |
|---|---|---:|---:|---:|
| Előre | `v3_20260930_172050_10613_capture` | 490 | 46,40 Hz | 62,57 ms |
| IDLE | `v3_20260930_172408_11003_capture` | 926 | 49,17 Hz | 49,33 ms |
| Room Cruise | `v3_20260930_172628_11426_capture` | 796 | 34,01 Hz | 126,82 ms |

Mindhárom teljes, változtatás előtti natív replay `MATCH`, két egymástól
független offline végrehajtás egyező trace-ével. Az IDLE raw evidence lezárása
`RAW_LIDAR_TRANSPORT_END_MISSING` miatt hiányos; a lezárt control inputok replay
integritása ettől külön teljes. Az IDLE-re csak ebben a scope-ban állítható MATCH.

### Előremenet

- 46–488. tick: folyamatos TELEOP, 0,15 m/s és 0 rad/s kérés, L12 ALLOW.
  Nem a tervező vagy egy megszakadó sebességkérés okozza az oszcillációt.
- A rögzített bal/jobb keréksebesség átlaga 0,1863/0,1837 m/s.
  A mozgás előtti utolsó ticktől a végéig a signed encoder-distance
  1,7528/1,7367 m, átlagosan 1,7448 m. A lokális heading változása −3,16°,
  ami összhangban van a kismértékű jobb drifttel.
- A felhasználó kb. 1,59 m-t mért; az encoderút ennél kb. 9,7%-kal nagyobb.
  Ez mérési/kalibrációs eltérés, nem bizonyított fizikai túlfutás. A globális
  pose-ból számolt 1,804 m út sem külső ground truth.
- 489. tick: `L11 feedback remained uncertain too long`, majd L12 FAULT.
  A végén a jobb kerék fitje 0,105–0,123 m/s-ra, majd a balé
  0,104–0,125 m/s-ra esik. A közös, véges uncertainty-watchdog ekkor leállít.
  A pulse trust közben 1; az ép számláló és a control-grade sebességfit külön tény.
- A bal/jobb quadrature rejection növekmény +197/+1. Ez érzékelési
  aszimmetriát mutat, de önmagában nem bizonyít GPIO-, kábelezési vagy mechanikai hibát.

### IDLE

926/926 tickben STOP, motorengedély nélkül; mindkét encoder elmozdulása nulla.
A lokális heading változása kb. 0,0048°. A globális becslésből számolt
1,48 cm út nem tényleges gurulás bizonyítéka. Ez a futás nem igazolja, hogy
mozgás közben hibátlan az encoder, de nem mutat álló helyzetű pulse-driftet.

### Room Cruise: az első eltérő döntési pontok

Az idők a capture 0. tickjéhez képest értendők.

| Tick | Idő | A mozgást megszakító közvetlen ok |
|---|---:|---|
| 164–166 | 4,886 s-tól | L6 `PLANNER_STALE_HOLD`: a régi köztes célhoz már nem fér el a teljes, újraellenőrzött rollout. |
| 204–205 | 6,373 s-tól | L6 `WORLD_STALE`, majd L12 `CRITICAL_DEVICE_DEGRADED` / LiDAR stale. |
| 232–464 | 7,539–13,281 s | L7 `NO_PROGRESS_VIABLE_TRAJECTORY`, 5,742 s mozgáskérés nélkül. |
| 496–500 | 14,185 s-tól | Canonical ingress STOP: `resident.mailbox.expired.126.496`. |
| 562 | 16,750 s | Újabb L6 `PLANNER_STALE_HOLD`. |
| 610–700 | 18,646–20,821 s | Ingress STOP: `resident.mailbox.expired.171.610`; az első tickben LiDAR-degraded is. |

A 731. ticktől következő STOP a futás vége, nem újabb önkényes planner-megállás.
Az L12 ALLOW önmagában nem jelent tényleges mozgáskérést: a hosszú planner-hold
alatt is ALLOW mellett nulla a kerékcél.

## Gyökérokok és javítás

**L6: a szabadon választható köztes cél túl későn cserélődik.**
A 229. tick requestjének célja a lokális pose-tól kb. 0,229 m-re van,
a lecserélési tolerancia csak 0,15 m. A megromlott lokális minőség mellett
a megengedett cél 0,192684 m/s és 0,24 rad/s. A 0,15 m/s kerékminimumhoz
szükséges helyben fordulás legalább kb. 0,843 rad/s lenne, tehát nem fér a capbe.
A közeli célhoz a megengedett ívek heading/progress pontszáma már elégtelen;
a reverse escape sem ad előrelépést. A 232. tickben elfogadott családban
egyik trajectory sem progress-viable, miközben nincs collision. A régi cél
megmarad, és az újraindulás csak a 8 s-os célkor lejárata utáni completionnel jön.

A javítás L6-ban marad: EXPLORE egy rollout távolsággal korábban választ
új köztes célt, illetve újraválaszt, ha a teljes elfogadott család használhatatlan.
Pending rollout alatt a már L6 által kiválasztott új EXPLORE-célhoz lehet
egyetlen, friss geometrián ellenőrzött lokális folytatást bizonyítani.
A collision, freshness, lejárat és sebességcap ugyanúgy kötelező;
a NAVIGATE elkötelezett célja változatlan szemantikájú.

Capture-alapú ellenpróba: a 227. checkpointból a 228–229. tick canonical
composition útján előállított requestjét ugyanazzal a pure plannerrel számoltuk.
A régi céllal a teljes eredmény pontosan egyezik a 232. tick rögzített
completionjével: **0 használható pálya**. A javított L6 új céljával, ugyanazon
geometrián és limitekkel **13 használható pálya** van. Ez offline
ellenpróba, nem a módosított robot pályájának fizikai visszajátszása.

**L11: a megbízhatósági skálázás csak a P-t és az új integrálfelhalmozást érintette.**
A már megtanult I-korrekció teljes erővel megmaradt a gyenge fit sávjában,
majd a fit elvesztésekor a PI reset egyszerre törölte. Például a 474. checkpoint
bal integrálja −0,047384; `ki=0,6` mellett ez −0,02843 outputkorrekció,
miközben a feedback gain már csak 0,4596. A reset visszaugrasztotta a kimenetet
a magasabb feed-forwardra. Ez bizonyított vezérlési diszkontinuitás és az
oszcilláció egyik lehetséges erősítője; a teljes fizikai oszcilláció kizárólagos
okának nem nevezhető.

A javítás a teljes PI-korrekciót, az anti-windup számításával együtt,
ugyanazzal a feedback gainnel skálázza. A gyenge sávban továbbra sincs új
integrálfelhalmozás. A bounded feed-forward, a közös uncertainty-watchdog,
a stale/hibás feedback kezelése és L12 változatlan marad.

**Command ingress: tényleges liveness-kimaradás, további mérés szükséges.**
A 496. tickben a 126. revision TTL-je 70,08 ms-mal lejárt.
A 610. tickben a 171. revision 29,95 ms-mal lejárt; 171 és 172 kiadási ideje
között 2,399 s telik el. Utóbbi nem magyarázható pusztán egy lassú L6 tickkel:
a külön command producer kiadási folyamatában is rés van. A capture nem
bontja ezt scheduler-, filesystem I/O- vagy más producer-késésre. A meglévő
atomic mailbox már nem fsync-el. A TTL növelése nem gyökérokjavítás;
itt nem változott timeout vagy safety policy.

## Validáció és fennmaradó munka

- CORE, motion, Room Cruise és async célzott tesztek: sikeresek.
  Esetszámok: 47 / 19 / 10 / 6.
- `./r test full`: **175 sikeres teszt**, benne a natív capture/replay és
  checkpoint/restore regressziókkal.
- A meglévő mozgásfolytonossági teszt most ténylegesen áthalad több köztes célon,
  160 ms-mal késleltetett planner-completionnel. A PI-teszt mindkét sebességelőjellel
  ellenőrzi a megtanult korrekció kifutását és a checkpoint/restore egyezést.
- A kibővített két regresszió a régi source-szal hibázik, a javítással sikeres.
  A régi mozgásteszt 104. tickjében 0,0375 m/s-os lépcsőt mér a megengedett
  0,012 helyett; a PI-regresszióban a gyenge fit mellett teljes I-korrekció marad.
- Eredeti capture-ek: módosítás előtti teljes natív replay MATCH
  (490 + 926 + 796 tick). Ez hibareprodukciót bizonyít, nem javított fizikai viselkedést.
- Részletes ideiglenes eredmények: `/tmp/r2b4-motion-20260930.CqQOXl/`.

Hátra van az új, külön engedélyezett fizikai elfogadási futás: az oszcilláció,
az 1,59 m kontra encoder-distance eltérés, a bal oldali rejection-aszimmetria,
a command producer 2,399 s-os kiesése és a LiDAR/control jitter ellenőrzése.
A capture tick-közei a teljes ütemezést jellemzik; nem azonosak az L0–L12 CPU
idejével. A meglévő wall/thread-CPU fázismérés és producer-oldali mérés kell a
GIL, a process transport, a fizikai I/O és a scheduler-terhelés szétválasztásához.
Az offline javításból ezek megszűnése nem következik.
