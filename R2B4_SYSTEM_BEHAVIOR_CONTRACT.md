# R2B4 robotrendszer — legfelső szintű működési contract

**Contract:** `R2B4_SYSTEM_BEHAVIOR_CONTRACT_V1`

**Szerep:** az R2B4 teljes robotrendszerének legfelső szintű, felhasználó felől értelmezett működési iránya. A dokumentum azt rögzíti, **minek kell a robotrendszer egészének lennie és hogyan kell viselkednie**, nem azt, hogy ezt pillanatnyilag melyik osztály, processz, thread vagy parancs valósítja meg.

Ez a dokumentum fejlesztési irányt és rendszerszintű invariánsokat ad. **Nem implementációs állapotjelentés:** egy pont attól még normatív cél marad, hogy a jelenlegi source még nem teljesíti.

Nem feladata konkrét algoritmus, processztopológia, systemd-beállítás, CLI-szintaxis, timeout-implementáció, queue, CPU-affinity, tuningérték, ER2 tool-lista, kamerakalibrációs algoritmus vagy tesztterv rögzítése.

---

## 1. Authority és kapcsolat a meglévő dokumentációval

Kérdéstípus szerint:

- **A teljes robotrendszer felhasználói működése, boot utáni rendelkezésre állása, ember–robot interakciója, valamint annak eldöntési elve, hogy mikor szükséges V3:** ez a dokumentum.
- **Production control ownership, L0–L12 réteghatárok, safety, motor-authority és engedélyezett production adatél:** `STRUKTURALIS_RETEGEK_V3.md`.
- **Async/process végrehajtási elhelyezés, control-island és edge szabályok:** `ASZINKRON_RUNTIME_CONTRACT_V3.md`.
- **ER2 navigációs interfész konkrét szemantikája:** `docs/ER2_NAVIGATION.md`.
- **Aktuális használat és launcher-felület:** `README.md` és a canonical source.
- **Pillanatnyi implementáció, capability-k és konfiguráció:** production source + aktív config.

Ez a dokumentum **nem írhatja felül** a production safety-, single-authority- és canonical command-path invariánsokat.

Ha a source rendszerszintű viselkedése eltér ettől a contracttól, az implementációs eltérés. Ha viszont a kérdés alacsonyabb szintű control-, safety-, timing- vagy layer-szemantikára vonatkozik, annak authorityja továbbra is a meglévő V3 contract.

A dokumentációk között részleteket nem kell másolni. Ahol egy alacsonyabb szintű szabály már létezik, erre a dokumentum csak hivatkozik.

---

## 2. A robotrendszer és a V3 fogalma

Az **R2B4 robotrendszer több, mint a V3**.

A **V3** ebben a dokumentumban a teljes production mozgási/control környezetet jelenti: azt a futó rendszert, amely a robot fizikai mozgatásához szükséges perifériákat, world modelt, control- és safety-rétegeket, valamint a canonical végrehajtási útvonalat biztosítja.

A V3 számítás- és energiaigényes komponens. Ezért **nem kell folyamatosan futnia csak azért, hogy a robot megszólítható vagy megfigyelésre képes legyen**.

A robot rendszerszintű rendelkezésre állását legalább három, egymástól külön kezelendő fogalom alkotja:

1. **emberi interakció elérhetősége** — a robot hallja-e és képes-e feldolgozni a felhasználói kérést;
2. **V3 életciklusa** — a production mozgási/control rendszer éppen leállított, induló, IDLE vagy aktív végrehajtási állapotban van-e;
3. **fizikai robottevékenység** — van-e tényleges aktív mission, mozgás vagy más vezérelt robotművelet.

Ezek nem ugyanazok az állapotok, és egyikből sem szabad automatikusan következtetni a másikra.

A robot emberi interakcióra képes lehet úgy, hogy a V3 nem fut. A V3 futhat IDLE állapotban úgy, hogy a robot fizikailag nem mozog. A robotrendszer nem tekinthető „leálltnak” pusztán azért, mert a V3 nincs aktív állapotban.

---

## 3. Boot és alap rendelkezésre állás

Normál üzemben a robot emberi interakciós képességének a felhasználói Linux-bejelentkezéssel együtt rendelkezésre kell állnia.

A boot/bejelentkezés utáni alapállapot célja:

- a robot nem kezd önállóan fizikai mozgásba;
- a mikrofonhoz tartozó canonical voice/wake szolgáltatás működőképes;
- a robot a konfigurált wake phrase-t figyeli;
- a V3 nem fut szükségtelenül;
- a V3 nem előfeltétele annak, hogy a robot megszólítható legyen.

A voice/wake szolgáltatás host-oldali orchestration. Nem kaphat motor-, mission-, navigation- vagy safety-authorityt.

A konkrét Linux service-mechanizmus implementációs kérdés. A jelenlegi rendszerszintű követelmény az, hogy normál felhasználói bejelentkezés után az interakciós szolgáltatás külön kézi V3-indítás nélkül rendelkezésre álljon.

---

## 4. Wake és beszélgetési működés

A wake phrase feladata **az emberi interakció megnyitása**, nem a V3 automatikus elindítása.

A kívánt általános folyamat:

```text
login / rendszer rendelkezésre áll
  ↓
wake figyelés
  ↓
wake felismerve
  ↓
rövid ready-visszajelzés
  ↓
felhasználói kérés
  ↓
végrehajtási mód + szükséges capability meghatározása
  ├─ nem kell V3 → beszélgetés / read-only / observation capability
  └─ kell V3     → V3 indítás → readiness → canonical robot action
```

A wake phrase konfigurálható. **Alapértelmezett értéke: `robot`.**

A wake utáni ready-visszajelzés szövege konfigurálható. **Alapértelmezett értéke: `figyelek`.**

A ready-visszajelzés jelentése: **„hallgatlak / készen állok a kérésedre”**.

Sem a wake felismerése, sem a ready-visszajelzés önmagában nem indokol V3-indítást, pozitív actuationt vagy robotmozgást.

### Beszélgetési session

A wake után megnyíló beszélgetési session aktív marad, amíg:

- új felhasználói megszólalás érkezik, vagy
- **10 másodperc folyamatos csend** nem telik el.

A 10 másodperces csend után a session lezárható, és a rendszer visszatérhet wake-listening állapotba.

A beszélgetési session életciklusa nem azonos a V3 életciklusával. V3-indítás, V3-IDLE, mozgás vége vagy V3-leállás önmagában nem zárhatja le a voice sessiont, amíg annak saját session-szabálya szerint még aktívnak kell maradnia.

---

## 5. Mikor szükséges V3

A V3 **mozgási/control végrehajtási környezet**, nem a teljes robotrendszer alapállapota.

V3 akkor szükséges, amikor egy felsőbb réteg **mozgatási intentet** vagy más olyan robotakciót kér, amely a production control/safety útvonalat igényli.

A kívánt életciklus:

```text
felsőbb réteg mozgatási intentet kér
  ↓
V3 nincs aktív
  ↓
V3 indul / bootol
  ↓
szükséges perifériák és alrendszerek inicializálódnak
  ↓
V3 eléri a végrehajtható IDLE / ready állapotot
  ↓
a már meglévő canonical módon végrehajtja a mozgást
```

A V3 indulása nem tekinthető azonnali műveletnek. A teljes readinesshez több periféria és alrendszer inicializációja szükséges; egyes eszközök, például a LiDAR, több másodperces indulási időt is igényelhetnek.

A felsőbb rétegnek ezért nem közvetlen motorparancsot kell adnia egy leállított V3-nak, hanem **végrehajtási igényt** kell kifejeznie. A rendszer feladata, hogy a V3 readiness állapotát biztosítsa, és csak utána engedje a canonical végrehajtást.

Nem szabad V3-at indítani pusztán azért, mert egy kérés:

- beszélgetési vagy információs;
- nem kér actuationt;
- read-only vagy observation jellegű;
- olyan host/edge capabilityből kiszolgálható, amelynek szemantikailag nincs szüksége a production mozgási/control környezetre.

A döntő kérdés nem az, hogy „a jelenlegi kódban melyik processz birtokolja ezt”, hanem az, hogy **a capability helyes rendszerszemantikája szerint szükséges-e hozzá V3**.

A mozgás kívánt működése folyamatos, sima haladás, menet közbeni sebesség-, irány-
és pályakorrekcióval. A replanning, globális lokalizáció és háttérmunka önmagában
nem tördelheti a fizikai mozgást. Friss robot-relative geometria és használható
lokális odometria/heading mellett a globális pozíció bizonytalansága óvatosabb,
legalább 0,15 m/s-os realizálható tartós haladást vagy háttérkorrekciót indokol.
Valós lokális evidence-vesztés, akadály, célhoz érkezés, explicit STOP és safety
korlát továbbra is indokolhat megállást. A 0–0,15 m/s közötti indulás/fékezés
véges átmenet; a kis sebességű encoder-fit nem megbízható sebesség-authority.

A jelenlegi implementációs coupling nem válhat automatikusan rendszerszintű követelménnyé.

### 5.1. Állapot-provenance és state/action konzisztencia

A magas szintű Agent szándékot és célt választ; nem feltételezheti, hogy egy
korábban megfigyelt fizikai állapot változatlanul érvényes. Minden fizikai
végrehajtásnak bizonyítható kapcsolatban kell maradnia azzal az állapottal,
amelyből a cél készült.

Relatív finite motion esetén kötelező sorrend:

`runtime/capture stabilizálás → STOP/IDLE → friss pose → target → admission`

A friss pose és az admission között runtime restart, capture re-arm vagy más
local-frame epoch váltás nem megengedett. A hostoldali operator transition ezt
sorosan védi; a resident runtime-session identitás változása fail-closed hiba.

A kutatás/fejlesztési evidence-nek legalább a requestet, start pose-t, target
pose-t, final pose-t, command/mission identitást, completion/failure reasont és
a frame/runtime provenance-t meg kell őriznie. A robot nem állíthatja egy
műveletről, hogy sikeres, ha a completion identity vagy a provenance nem
bizonyítható.

---

## 6. V3-független capability-k fejlesztési iránya

A robot fejlődési iránya az, hogy a **nem actuation-jellegű képességek a lehető legáltalánosabban ne legyenek szükségtelenül a V3 életciklusához kötve**.

Minimum követelmény:

- **mikrofon / voice input** V3 nélkül is működjön;
- **kamera / vizuális megfigyelés** V3 nélkül is elérhető legyen.

Általános elvként ugyanez alkalmazandó minden olyan későbbi capabilityre, amely:

- read-only vagy observation jellegű;
- nem igényel robotmozgást;
- nem igényel production control-authorityt;
- külön, canonical ownership mellett biztonságosan szolgáltatható.

Példák:

- általános beszélgetés;
- külső információ lekérése;
- a robot saját állapotának megfelelő read-only lekérdezése;
- vizuális megfigyelés;
- később más szenzoros read-only capability, ha az architekturálisan tisztán leválasztható.

A „mit látsz most magad előtt?” típusú kérés rendszerszintű jelentése **megfigyelés**, nem mozgás. Ennek kiszolgálásához nem szabad csak azért V3-at indítani, mert a kamera aktuális implementációja történetileg ahhoz van kötve.

Ez nem engedély kamera-, mikrofon- vagy sensor-ownership megkerülésére. Az SSOT, kalibráció, kizárólagos fizikai ownership, freshness, health és async-edge szabályok továbbra is kötelezők.

A helyes fejlesztési irány nem második kamera- vagy mikrofonút létrehozása, hanem a capability olyan ownershipje és publikációja, amely a magasabb szintű rendszerszemantikával és a meglévő V3 contractokkal egyszerre konzisztens.

A kamera alapállapota OFF. Csak aktív vision igény kapcsolhatja be; sem V3,
Agent, ER2 vagy voice indulása önmagában nem igény. A vision-függő V3 mission
LLM nélkül is létrehozza és a végén elengedi saját igényét. Az önálló
`vision.observe` és CLI kameraút V3 nélkül kalibrált képet ad; több fogyasztó
ugyanazt a fizikai ownert használja. Az utolsó fogyasztó után rövid grace
lezárhatja a kamerát. Más aktív fogyasztó mellett V3 stop vagy manuális camera
OFF nem szakíthatja meg a megfigyelést. Kamerahiba csak a vision capabilityt
teszi elérhetetlenné; vision-függő mozgás ilyenkor biztonságosan megáll.

---

## 7. A végrehajtási mód kiválasztása

A felhasználói vagy agent-kérés után a rendszernek egy **végrehajtási mód választó** felelősséget kell alkalmaznia.

Feladata annak eldöntése, hogy az adott intent:

- egyszerű beszélgetési / információs válasz;
- read-only host vagy observation capability;
- ER2-t igénylő magasabb szintű robotikai reasoning;
- közvetlen canonical robot action;
- vagy más, később definiált végrehajtási mód

segítségével teljesítendő-e.

### Alapértelmezett beszélgető partner

Normál beszélgetési kérésnél az **alapértelmezett beszélgető partner a ChatGPT/OpenAI provider**.

Ez nem kizárólagos backend. A **végrehajtási mód választó** az intent és a szükséges capability alapján választhat más végrehajtási módot is, többek között:

- **Gemini fallback**;
- **ER2 stream**;
- **ER2 preview**;
- később bármely más, a publikus capability boundary mögé szabályosan integrált beszélgetési, reasoning- vagy robotikai végrehajtási módot.

A ChatGPT/OpenAI provider, a Gemini, az ER2 stream és az ER2 preview nem külön robotikai authority-k. A kiválasztás azt határozza meg, hogy melyik magasabb szintű végrehajtó/partner dolgozza fel a kérést; robotmozgás esetén a tényleges actuation ettől függetlenül a canonical V3 command- és safety-úton történik.


### LLM hitelesítés és automatikus fallback

A ChatGPT/OpenAI elsődleges hitelesítési útja a **Sign in with ChatGPT OAuth**. A mentett OAuth access token automatikusan használható, lejárat előtt vagy lejáratkor a hozzá tartozó rotating refresh tokennel megújítandó. Az `OPENAI_API_KEY` opcionális fallback, nem kötelező előfeltétel.

Az LLM provider-lánc normál sorrendje: **ChatGPT/OpenAI OAuth → OpenAI API key → Gemini → Groq**, kizárólag azokból az ágakból, amelyekhez érvényes helyi hitelesítés van. Kvóta-, rate-limit-, provider-, transport- vagy érvénytelen LLM-válasz hiba esetén csak minimális, indokolt újrapróbálkozás engedett; ezután a következő konfigurált LLM-re kell váltani. Kvóta/rate-limit és szemantikai/strukturált válaszhiba nem indokol ismételt azonos-provider próbálkozásokat.

A provider-váltás kizárólag a magas szintű LLM-válasz előállítását érinti. Nem ismételhet meg már végrehajtott toolt vagy robotakciót, nem hoz létre új robotikai authorityt, és nem kerülheti meg a canonical RobotInterface/V3 command- és safety-utat.

### Hallható válasz

Ha a rendszer a választ hallhatóan adja vissza, az **alapértelmezett beszédszintézis a lokális TTS**.

A beszélgető/reasoning partner és a beszédszintézis két külön felelősség. Attól, hogy egy választ ChatGPT/OpenAI, Gemini, ER2 stream, ER2 preview vagy más későbbi végrehajtó állít elő, a hallható kimenet alapértelmezett útja továbbra is a lokális TTS.

Más TTS backend később opcionálisan támogatott lehet, de nem válhat szükséges előfeltételévé a normál hallható válasznak.

A végrehajtási mód választó **nem robotikai authority**.

Nem írhat motort, nem módosíthat safety-state-et, és nem kerülheti meg a canonical production command pathot.

Feladata a szükséges végrehajtási út, partner és capability kiválasztása; a tényleges robotmozgás továbbra is kizárólag a canonical V3 útvonalon hajtható végre.

---

## 8. ER2, voice, CLI és más külső döntéshozók

ER2, voice/LLM, CLI, GUI vagy más külső komponens **nem külön robotikai authority**.

Mindegyik külső kérés ugyanahhoz az elvhez igazodik:

```text
felhasználó / agent / ER2
        ↓
publikus robot-interface / capability boundary
        ↓
végrehajtási mód választó
        ↓
szükséges capability / partner
        ├─ ChatGPT/OpenAI / conversation
        ├─ Gemini / fallback conversation
        ├─ ER2 stream / ER2 preview
        ├─ read / host / observation
        └─ robot action
               ↓
        ha kell: V3 readiness
               ↓
        canonical command ingress
               ↓
        production V3 control + safety
```

Az ER2 feladata lehet magas szintű döntés, reasoning vagy robotfeladat kezdeményezése, de a tényleges robotmozgást ugyanazon canonical control/safety úton kell kérnie, mint bármely más külső kliensnek.

LLM vagy ER2 nem kaphat közvetlen motor-, GPIO-, layer-state-, safety- vagy belső V3-authority handle-t.

---

## 9. A mikrofon és a V3 egymástól független életciklusa

Ha a voice szolgáltatás engedélyezett:

- a mikrofon működése nem függhet attól, hogy a V3 fut-e;
- V3 indulása nem ad új mikrofon-authorityt;
- V3 leállása nem kapcsolhatja ki a mikrofont;
- egy mission vagy mozgás befejezése nem szüntetheti meg automatikusan az emberi interakciós képességet;
- a robotnak futó fizikai feladat közben is fenn kell tartania a biztonságosan támogatott emberi beavatkozási lehetőséget;
- a voice session saját, 10 másodperces csend-alapú lifecycle szerint működik.

A már meglévő fail-safe STOP szabályokat ez a dokumentum nem módosítja.

A voice szolgáltatás explicit ki- és bekapcsolhatósága operátori funkció. Ennek canonical felülete a gyökér `r` launcher lehet, de maga a launcher nem válik robotikai authorityvá.

A voice szolgáltatás kikapcsolása az emberi hangbemenetet kapcsolja ki; nem hozhat létre alternatív control-state-et és nem módosíthatja a production safety szabályait.

---

## 10. `r` launcher rendszerszintű szerepe

A gyökér `r` az ajánlott közös ember/agent belépő és operátori vezérlőfelület.

Rendszerszintű szerepe:

- egységes hozzáférés a robot publikus capability-jeihez és lifecycle műveleteihez;
- voice szolgáltatás operátori ki-/bekapcsolása;
- diagnosztikai és fejlesztői elérés;
- ER2 és production robotparancsok elérése a meglévő canonical interfészeken keresztül.

Az `r` nem második control rendszer, nem saját robot-state owner, és nem adhat kerülőutat a `RobotInterface`, a canonical command ingress vagy a V3 safety körül.

---

## 11. Fejlesztési invariánsok

A jövőbeli fejlesztések akkor illeszkednek ehhez a rendszerszintű irányhoz, ha az alábbiak megmaradnak:

1. **A robot megszólíthatósága nem azonos a V3 futásával.**
2. **Wake felismerés önmagában nem indít robotmozgást és nem indítja el a V3-at.**
3. **A felhasználói kérés után a végrehajtási mód választó dönti el, milyen végrehajtó/partner és capability szükséges.**
4. **Mozgatási intent esetén a V3 szükség szerint elindul, teljes readinessig bootol, majd IDLE/ready állapotból hajtja végre a mozgást.**
5. **Nem actuation-jellegű capability nem köthető szükségtelenül a V3-hoz.**
6. **Minimum a mikrofon és a kamera V3 nélkül is használható capability.**
7. **A V3-függetlenségi szabály lehetőség szerint általánosan alkalmazandó más read-only/observation capabilitykre is.**
8. **Mozgás csak canonical V3 command- és safety-úton történhet.**
9. **Voice, ChatGPT/OpenAI, Gemini, ER2 és más agent/végrehajtó csak kliens/orchestrator lehet, nem motor-authority.**
10. **A voice/microphone lifecycle és a V3 lifecycle külön kezelendő.**
11. **A wake phrase konfigurálható; alapértéke `robot`.**
12. **A wake utáni ready-visszajelzés konfigurálható; alapértéke `figyelek`.**
13. **A beszélgetési session 10 másodperc folyamatos csend után lezárható.**
14. **Az alapértelmezett beszélgető partner ChatGPT/OpenAI; LLM-hibánál a hitelesített fallback-lánc automatikusan válthat OpenAI API key, Gemini vagy Groq providerre; ER2 stream/preview továbbra is külön végrehajtási mód.**
15. **A hallható válaszok alapértelmezett beszédszintézise a lokális TTS.**
16. **A beszélgető/reasoning partner kiválasztása és a TTS-kimenet külön felelősség.**
17. **A V3 technikai implementációja változhat anélkül, hogy a felhasználó felől látható rendszerszemantika megváltozna.**
18. **Egy jelenlegi source-coupling nem tekinthető automatikusan kívánt architektúrának.**
19. **Új kerülőút helyett a meglévő canonical ownershipet kell úgy alakítani, hogy a kívánt rendszerszintű működést helyesen szolgálja ki.**

---

## 12. Amit ez a dokumentum szándékosan nem rögzít

Ez a contract nem mondja meg:

- hány processz vagy thread fusson;
- mely CPU-magon fusson egy capability;
- pontosan mely service induljon mely systemd unitból;
- a 10 másodperces session timeout pontosan mely komponensben és milyen timerrel legyen megvalósítva;
- mozgás után a V3 technikailag azonnal leálljon-e vagy meghatározott ideig IDLE-ben resident maradjon;
- pontosan mely komponens implementálja a végrehajtási mód választót;
- a ChatGPT/OpenAI, Gemini, ER2 stream, ER2 preview vagy későbbi végrehajtók konkrét routing algoritmusát;
- hogyan legyen a kamera vagy más sensor V3-függetlenítve;
- mely periféria milyen sorrendben bootoljon;
- pontosan mennyi legyen a LiDAR vagy más periféria readiness timeoutja;
- milyen konkrét ER2 tool készlet legyen;
- milyen timeout, retry, queue vagy watchdog érték legyen;
- milyen tesztlépéssel kell egy adott módosítást validálni.

Ezek source-first implementációs vagy külön contract-kérdések.

A fenti részletek megváltozhatnak. A rendszerszintű cél közben változatlan marad:

> **A robot legyen folyamatosan megszólítható; normál beszélgetésnél alapértelmezetten ChatGPT/OpenAI OAuth-pal dolgozzon, LLM-hibánál automatikusan válthasson a konfigurált fallback providerre, és szükség esetén a végrehajtási mód választó külön ER2 streamre vagy ER2 preview-ra válthasson; a hallható válasz alapértelmezetten lokális TTS-en szólaljon meg; és a V3 csak akkor induljon el, amikor tényleges robotmozgás vagy más production controlt igénylő feladat szükséges.**

---

## 13. További dokumentációs integráció

Ennek a contractnak a Codex és más fejlesztő agentek számára is látható, legfelső szintű authoritynak kell lennie a teljes robotrendszer felhasználói viselkedését érintő fejlesztéseknél.

A root `AGENTS.md` ezért hivatkozzon erre a dokumentumra, de **ne másolja bele** a szabályait.

Ajánlott authority-sorrend:

1. `R2B4_SYSTEM_BEHAVIOR_CONTRACT.md` — teljes rendszer kívánt viselkedése és fejlesztési iránya;
2. `STRUKTURALIS_RETEGEK_V3.md` — production control architektúra és safety;
3. `ASZINKRON_RUNTIME_CONTRACT_V3.md` — async/process elhelyezési szabályok;
4. domain-specifikus dokumentációk;
5. aktuális source + config az implementáció tényleges állapotához.
