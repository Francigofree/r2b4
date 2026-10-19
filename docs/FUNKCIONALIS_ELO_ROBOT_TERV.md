# Funkcionális élő robot: parancstól a nyugalmi állapotig

2026-10-19. Fejlesztési terv és mérési jegyzet; nem új architektúracontract.
Az authority továbbra is a `STRUKTURALIS_RETEGEK_V3.md` és az
`ASZINKRON_RUNTIME_CONTRACT_V3.md`. Az alábbi megállapítások az aktív source-ra
és az itt megnevezett konkrét mérésekre vonatkoznak.

## A kívánt használat

Nyugalomban V3 és motor-output nincs. A hangfigyelő fogadhat parancsot;
az `r` launcher mindig használható. Sima beszélgetéshez V3 nem szükséges.
Mozgási feladatnál a host készíti elő a runtime-ot, a skill szemantikus
behavior-kérést ad, a mozgást a canonical V3 és L12 valósítja meg.
A skill végén saját command/observer/media igényei megszűnnek, és saját
indítású V3-a leáll. Kézzel előkészített runtime a kezelő sessionjéhez tartozik.

```text
V3 OFF + parancsfigyelés
  → kérés besorolása
  → beszélgetés V3 nélkül / mozgási runtime előkészítése
  → véges skill és canonical behavior
  → STOP, skill terminal esemény, saját runtime shutdown
  → V3 OFF + parancsfigyelés
```

## E körben kiépített kapcsolatok

| Funkció / határ | Forrásban talált rés | Megvalósítás |
| --- | --- | --- |
| Python-skill → runtime-életciklus | STOP után a saját V3 és szenzorfolyamatok maradhattak | A host konkrét létrehozó controller-generation/PID alapján tartja számon és zárja a saját példányt; véges részfeladatok között megtartja |
| Skill terminal → capture | Az automatikus shutdown lezárhatná a capture-t a terminal source-binding előtt | Terminal journal flush a shutdown előtt; a tényleges runtime cleanup külön host esemény |
| SDK → időzített behavior | `completion_on_duration` az alsó motion adapterbe csúszott | A host a BehaviorSystemnek adja át; az időablakot a meglévő owner méri |
| Hideg indítás → rövid skill | A készülékek startupja elfogyasztotta a rövid behavior admission idejét | A két időzített skill előbb canonical runtime/readiness előkészítést kér |
| Hangfigyelés → passzív státusz | 20 ms PCM-ciklusonként fsync és operator-status olvasás | Változatlan listening állapotban 1 Hz; állapotváltás/hiba/STOP azonnali |
| Felhasználó → mentett mozgási skill | Nem volt könnyen indítható, véges Python-skill e két alapfunkcióhoz | `room_cruise`, `follow_person`, alapból 300 s, legfeljebb 3600 s; nincs LLM a futási ciklusban |

A skill-idő nem folyamatos kerékmozgás bizonyítéka: a natív recoverable HOLD-ok
és célközeli várakozások a feladat részei. Személykövetésnél a jelenlegi élő
V3 tracker választ személyt; ez nem név szerinti felismerés. Régi térbeli vagy
személymemória nem válik friss követési evidence-szé.

## Következő, forrással indokolt integrációk

1. **Brain/voice által választott teljes feladat saját runtime-életciklusa.**
   Az itt megoldott ownership a Python-skillhez tartozik. A közvetlen
   TaskGraph/behavior célok terminal ágait külön kell összekötni a saját
   indítású runtime lezárásával. Ne legyen általános, más sessiont leállító
   idle timeout. Forrás: `brain_core.py`, `robot_runtime.py`, `behavior_system.py`.
2. **Személykeresés: valódi observation fázis.** A `SearchPerson` navigáció
   után JPEG-et kér, de e kérés önmagában nem tart személydetektor-demandet.
   A név szerinti kereséshez bounded detektor-observation és már hiteles
   személynév-kötés kell. A memória helyjavaslat, nem aktuális látásbizonyíték.
   Forrás: `search_person.py`, `vision_media_socket.py`, `v3_hardware_runtime.py`.
3. **Keresési HOLD/resume egyezés.** A room/follow adapter megtartja a missiont
   `WORLD_STALE` és `LOCAL_COSTMAP_STALE` alatt; a keresési programok ezekre
   hamarabb terminal hibát adnak. Ugyanazon két recoverable ok kezelése kis
   adapterjavítás lehet; deadline, hard fault és STOP változatlanul szükséges.
4. **Követési fordulók fizikai envelope-ja.** Az aktív natív alapértelmezett
   follow angular cap 0,30 rad/s, a kalibrált centered-spin minimum kb.
   0,843 rad/s, a target kb. 1,068 rad/s. Az alapértelmezett profil ezért
   nem tud helyben fordulni. A live próba explicit 1,10 rad/s korlátot kér;
   a globális profil átállítása külön, bizonyított tuningdöntés legyen.
5. **Host-journal tartós lemezhasználata.** Az `events.ndjson` append-only,
   nincs retentionkorlát. A historyk és queue-k boundedek, de ez önmagában
   nem garantál bounded lemezhasználatot. Retention csak a capture-follower
   cursor/producer/sequence szemantikájának megtartásával vezethető be.

## Mérés és elfogadás

Egy mozgási skill elfogadása: V3 OFF indulás; saját runtime/readiness;
helyes mission/command azonosság; tényleges L12 ALLOW és encoder-evidence;
időtartam vagy explicit célkimenet; canonical nulla kimenetű STOP; saját V3 OFF;
camera/detector saját demand felszabadult. A kézzel előkészített és közben
lecserélt runtime megőrzése külön socket/worker teszt.

5 perc–1 óra alatt külön mérendő: processzek száma/RSS/CPU, státusz frissessége,
deadline és source health, capture/journal byte-növekedés, célvesztés/HOLD/resume,
STOP késleltetése és a végső erőforrás-felszabadítás. A rövid live próba,
a replay MATCH és a működő timeout nem bizonyít egyórás fizikai megbízhatóságot
vagy mért energiafogyasztást. Teljes FULL 50 Hz capture helyett hosszú normál
üzemben a meglévő bounded ALAP capture indokolt; teljes replayhez külön rövid
FULL 50 Hz mérés szükséges.

Az első live követési próba: `9e3e06faca8842959ce74dcaf842f71c`.
8,100 s-nél lejárt a még STARTING behavior (`command_id=None`), a cold
admission csak 9,742 s-nél tért vissza. Mozgás nem igazolódott. A javítás
a startup különválasztása, nem a safety vagy timeout lazítása.
A saját shutdown sikeres, a FULL 50 Hz capture PASS, 443 tick:
`runtime/captures/v3_20261019_225101_6234_capture.mcap`.
Részletes helyi mérési eredmény: `/tmp/r2b4-functional-live-6fl2rboy/`.

Az aktuális szoftveres és további live eredményeket a feladat zárójelentése
és a konkrét artifactok tartalmazzák; hiányzó evidence nem jelent PASS-t.
