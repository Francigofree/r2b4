# Room Cruise indítási hiba — 2026-10-16 20:43

A `r rt start c 10 c full`, majd `r rc continuous` próbánál a robot azért nem mozdult, mert az EXPLORE action **a host konfigurációvalidációján elbukott**. Nem jutott el ACTIVE command az L5-höz. A bizonyított hiba a régi host-kód és az új konfiguráció együttélése; ez az incident nem bizonyít mozgásgain-, navigációs vagy motorvezérlési regressziót.

## Authority és ellenőrzés

- Capture: `runtime/captures/v3_20261016_204318_22870_capture.mcap`.
- Meglévő explicit MCAP Evidence Compiler bundle: `runtime/captures/v3_20261016_204318_22870_capture.evidence`.
- MCAP SHA256: `f6a7144ffc6abccacb4c167e159348d070ff79913c404da5c0ade090bc0ff73c`.
- Az EVI `verify(..., source=...)` közvetlen offline API-val végzett teljes ellenőrzése: **PASS**, 1153 üzenet, quarantined 0, `source_checked=true`, compiler `COMPLETE`. A bundle és a capture nem változott; nem futott új hosszú teljes DIAG.
- Scope: **304 mentett tick**, production tick **0–1509**, monoton idő **5764062345842–5794273249132 ns**, időtartam **30.210903 s**. A capture 10 Hz-en mintázta az 50 Hz célütemű runtime-ot.

Nincs checkpoint és raw LiDAR a compact, mintavételezett capture-ben. Ebből **nincs teljes, pontos L1–L12 replay evidence**; az elemzés az integritás-ellenőrzött mentett tickekre és eseményekre vonatkozik. Új live mozgás nem indult.

## Első hibás boundary

Az időpontok a capture első tickjéhez viszonyított monoton idők.

1. **+16.318848 s**, `msg-000000000617`: `ACTION_REQUESTED`, `v3.command.explore`.
2. **+16.321260 s**, `msg-000000000618`: `ACTION_ERROR`, `ValueError`, pontos hiba: **`behavior: missing=[], unknown=['followperson']`**. Command- és mission-azonosító még nincs.
3. `msg-000000000619`: `BEHAVIOR_ACTION_FAILED`.
4. **+17.654311 s**, `msg-000000000669`: `BEHAVIOR_FAILED`, reason **`ACTION_START_FAILED:ValueError:behavior: missing=[], unknown=['followperson']`**.

A hiba a host action/config boundaryn történik, **a canonical mozgásparancs kiadása előtt**. A behavior failure esemény observation/publication ideje külön fogalom; a pontok a mentett események log-time-ját adják.

## Production következmény

Mind a 304 mentett tickben:

- Command input: `STOP`; L5: `STOP/IDLE`, `COMMAND_STOP`.
- L6: `IDLE`, `COMMAND_STOP`; nincs elfogadott EXPLORE mission.
- L8 requested v/omega, L9 allowed v/omega, L10 kerékcél, L11 és L12 output: **0**.
- L12 a kezdeti 13 mintában `STOP/CRITICAL_DEVICE_DEGRADED`, utána 291 mintában `STOP/NOT_ACTIVE`. Nincs FAULT.

A kezdeti sensor-readiness időszak `tick 66 / +1.317439 s` körül már lezárult. A későbbi teljes mozdulatlanság nem ennek a startup safety-várakozásnak a folytatása: a host később sem ad ki ACTIVE commandot.

Kiegészítő runtime riport: `/tmp/r2b4-runtime.kh6jrem0.log`, normál operator shutdown, `fault_layer=null`, 1510 tick. A runtime normál lezárása nem jelenti az EXPLORE action sikeres indítását.

## Mi bizonyítja a kód/config eltérést?

- A capture frissen indított control processének resolved konfigurációja már tartalmazza a **`followperson`** mezőt és a `FollowPersonConfig(max_v_mps=0.25, max_omega_rad_s=0.3)` értéket.
- A vizsgált aktuális source `v3/config.py::ConfigResolver.from_documents` a `behavior` szekcióban már `{roomcruise, followperson}` mezőket fogad el. Az aktuális source szerint a rögzített konfiguráció ezen a ponton érvényes.
- A tényleges host action mégis **ismeretlen `followperson`** mezőt jelent. Ez a mező az előző konfigurációs változtatással került a fájlba.
- Az aktív public robot host **PID 2645**, a `ps` szerint **2026-10-16 19:11:36** óta fut (`python3 -m r2b4_orchestration.robot_runtime`). A hiba behavior eseményében a `session_owner_pid` szintén **2645**. Ez ugyanaz a régi host, amely a korábbi capture-ökben is szerepelt, és a módosítások előtt töltötte be a Python-modulokat.

A configfájl azonnal újraolvasható, a már futó interpreter importált kódja ettől nem frissül. Így a friss control child és a régi public host eltérő schema-val dolgozik. A minimális üzemeltetési teendő a **deployed public host biztonságos frissítése/újraindítása**, majd az aktuális code/config egyezés ellenőrzése. Az indítási hiba CLI-visszajelzését is a tényleges terminal behavior eredményhez kell kötni, hogy sikertelen action ne jelenjen meg ACTIVE-ként.

## Validációs határok

Az incident bizonyítottan az ACTIVE command előtt akadt el; ebben a futásban nem gyakoroltuk az EXPLORE navigációt vagy a korábbi mozgás-/hold-javításokat.

## Elvégzett helyreállítás és javítás

A V3 resident leállított állapotának ellenőrzése után a régi public host canonical
`preempt(HOST_SOURCE_REFRESH)` kérést kapott. Ezt követte a meglévő,
SO_PEERCRED user/PID és project/module/socket cmdline ellenőrzésére korlátozott
termination út. A friss hostot kizárólag `robot.state` olvasás indította.

- Régi public host PID **2645**, új public host PID **25076**.
- Új behavior állapot **IDLE**, health **AVAILABLE**, hiba nélkül.
- A resident V3 runtime végig leállítva maradt; új mozgáskérés nem történt.
- Helyreállítási artifact: `/tmp/r2b4-host-refresh-lm8irwtp/result.json`.

Az első frissítő olvasás a még megszűnő régi socketre csatlakozott és
`ConnectionResetError`-ral leállt. A régi PID teljes megszűnésének külön
ellenőrzése után a read-only hostindítás sikerült. Nem ismételtünk
mozgáskérést, és nem lazult runtime/safety feltétel.

A CLI a behavior indításának visszatérő `FAILED` / `CANCELLED` állapotát
most hibának tekinti, az eredeti reasonnel és nem nulla exit code-dal.
Sikertelen indítás után nem lép a timed várakozásba és nem ír `command ACTIVE`
sikerüzenetet. Folyamatos kérésnél a valódi `STARTING` visszaigazolást is
`STARTING`-ként mutatja. A felhasználó már futó resident sessionjét nem állítja
le a CLI hibatakarítása.

- Célzott public interface/CLI regresszió: **22 passed**, Room Cruise és Follow,
  folyamatos és időzített indítás, eredeti hiba továbbadása.
- Kötelező kis `./r test` robot-contract gate: **14 passed**.
- `git diff --check`: tiszta.

Az `r sd` / `r rt start` a V3 resident lifecycle-ja, nem tölti újra a
független public host importált Python-moduljait. Host-kód és konfigurációs
schema együttes frissítése után a public hostot is frissíteni kell;
általános automatikus hotreload vagy sikertelen mozgás újrapróbálása nem került be.

A konkrét indítási blokk elhárult, de a javítás utáni fizikai Room Cruise
mozgást nem indítottuk el és nem bizonyítottuk. Új capture vagy replay nem
készült; a felhasznált 10 Hz-es eredeti capture exact replayre továbbra sem alkalmas.
