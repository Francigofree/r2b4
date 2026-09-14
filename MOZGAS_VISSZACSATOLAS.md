# Mozgásszabályozás közelítő speed mappel

A speed map az induló PWM-becslés. A menet közbeni keréksebesség-hibát L11,
a becsült pálya és heading hibáját L8 korrigálja. A változás a
`STRUKTURALIS_RETEGEK_V3.md` meglévő adat-élein és ownershipjén belül marad.

## Kiinduló evidence

A 2026-09-14-i két utolsó padlóteszt final MCAP-ja integritásellenőrzésen
és a módosítás előtti teljes natív L1–L12 replayen is átment: MATCH.

| Capture | Vizsgált aktív tickek | Átlagos becsült v | Heading változása | Keresztirányú eltérés |
| --- | --- | --- | --- | --- |
| `v3_20260914_205748_7450_capture.mcap` | 159–748 | 0,1399 m/s | −10,34° | −10,15 cm |
| `v3_20260914_212515_10869_capture.mcap` | 505–912 | 0,1426 m/s | −9,48° | −6,85 cm |

Mindkét szakasz parancsa v=0,15 m/s, omega=0. A keresztirányú eltérés a
vizsgált szakasz első becsült pose-ához és headingjéhez viszonyított érték;
nem külső helymérés. A második capture már mozgás közben kezdődik.

A legkorábbi konkrét control-problémák:

- Az első futás 159. tickjében az L10 cél 0,012002 m/s, de L11 a map
  0,15 m/s-os pontjának teljes 0,19566 / 0,19210 PWM-jét adta.
- L8 a teljes egyenes szakaszon nulla omegát kért a változó heading mellett.
- L11 az integrál előjelével ellentétes, 0,006 m/s-nál nagyobb hibánál
  lenullázta az addig tanult kompenzációt. A zajos sebességhiba ezért újra
  és újra megszakíthatta a tartós terhelés kompenzálását.
- A második futásban már az első rögzített, 505. tick L5/L8 constraintje
  `max_omega_rad_s=1e-6`. A `r2b4 mozog` konverziója az egyforma kerékcélból
  ezt számolta, ami L9-ben a headingkorrekciót is megakadályozta volna.

A caster, motor, csúszás és padló egyedi fizikai hozzájárulása ebből nem
választható szét bizonyítottan; a control-hiányok a forrásból és capture-ből
azonosíthatók.

## Megvalósítás

1. **L11 elővezérlés:** az első map-pont alatt lineáris átmenet a közelítő
   fenntartási PWM-től az első mérési pontig. A nulla cél továbbra is nulla
   kimenet. A feedback csökkentheti a PWM-et a fenntartási érték alá is.
2. **L11 kompenzáció:** a PI megtartja az integrált a sebességhiba
   előjelváltásakor. Feltételes integrálás védi a felső PWM-határt és a
   parancsolt forgásirány nulla határát; a telítésből kifelé integrálhat.
   A P-tag az első tickben is működik. STOP, kerékirányváltás, kimaradt
   tick és 250 ms feletti időköz törli az adott kompenzációt; elavult
   visszajelzés alatt nincs integrálás. A speed map fájlját nem tanulja át.
3. **Aktív alapérték:** Kp=0,25, Ki=0,60, integrálkorlát=0,75 m,
   tehát az integrális PWM-korrekció legfeljebb ±0,45. A végső PWM-et és
   annak előjelét továbbra is L11 határolja. Ezek offline ellenőrzött
   kiinduló értékek, fizikai hangolásuk még nincs igazolva.
4. **L8 pályakövetés:** állandó sebességcélhoz egyenest vagy körívet rögzít
   az aktuális becsült pose-ból. A pálya legközelebbi pontjának oldalhibája,
   a tangenshez viszonyított headinghiba és a szögsebesség hibája adja a
   legfeljebb ±0,6 rad/s korrekciót. Hátramenetnél az oldalhiba iránya
   megfordul. A helyreferencia nem szalad előre időalapon, ha L9 lassít.
   A jelenlegi L6 állandó twistű rolloutjai ugyanígy követhetők a mintáikból.
   VELOCITY pivotnál korlátos headingreferencia követi a kért fordulást.
5. **Referenciahatár:** STOP, nulla twist, megváltozott velocity target,
   koordinátarendszer-váltás vagy tickrés új referenciát indít. A referencia
   és a kerékirányok bekerülnek a natív checkpointba, így a slice replay
   ugyanazt a szabályozási állapotot állítja vissza.
6. **Parancs:** a `mozog` a névleges omega mellé 0,6 rad/s korrekciós
   tartalékot ad, legfeljebb a resident 1,2 rad/s-os határáig. A kért
   névleges sebesség változatlan. Az explicit kisebb mission-limitet L9
   továbbra is betartja; a szabályozó nem kerülheti meg.

## Validáció és bizonyítási határ

A célzott tesztek közelítő map mellett előre/hátra haladást, kis sebességet,
motorválasz- és terhelésváltozást, 40–80 ms kerékjel-késést, zajt, telítést,
irányváltást és STOP-ot vizsgálnak. A teljes natív L1–L12 tesztben
menet közben felcserélődik a motorok relatív erőssége, változik a súrlódás,
6 cm oldal- és 0,08 rad headingzavar, majd tartós −0,02 rad/s chassis-zavar
jelentkezik. Az egyenesek és kétirányú ívek végső 2 másodpercének kapuja:
2,5 cm pályahiba, 0,06 rad headinghiba, 0,012 m/s sebességhiba.

A tesztbeli motorreakció egyszerű, a valós motorhoz nem illesztett modell.
A hibátlan teszt és az új MCAP teljes/checkpointos MATCH replayje a
megvalósítás offline viselkedését és determinizmusát bizonyítja. A régi
capture új kóddal eltérő control-outputja MISMATCH, nem új fizikai pálya
és nem teljesített régi replay gate.

A módosított kódon mindkét régi capture L1–L7 eredménye változatlan;
az első eltérés rendre a 159. és 505. tick `L8.requested_omega_rad_s`
mezője. Az ismételt replay mindkét esetben determinisztikus.

A célzott mozgás-, navigation-, composition- és replay-tesztek teljesültek;
az import guard és a launcher szintaktikai ellenőrzése is PASS.
A teljes regressziós körből két korábban is fennálló L3-teszthiba maradt:
`test_native_predict_covariance_tracks_elapsed_time_across_tick_rates`
mindkét 40 ms-os paraméterezése. A változatlan L3 és teszt izolált futtatása
ugyanezt adja: egy kapcsolt kovarianciaelem 1,216e-5 a 20 ms-os referencia
1,274e-5 értékével szemben, kívül a teszt 4%-os relatív határán. Ezt a
szabályozásfejlesztés nem javította, a teszthatár nem lett lazítva.

Új élő teszt nem történt. A tényleges padlón elérhető pontosság, a megváltozott
gain-ek tranziens viselkedése és a tapadási határok csak külön engedélyezett
canonical runtime-teszttel igazolhatók. Ehhez először rövid, 0,15 m/s-os
egyenes szükséges megfelelő headingkorrekciós tartalékkal; váratlan safety-,
health- vagy timing-eredménynél nincs automatikus ismétlés. Telítésen túl,
megbízható pose/encoder nélkül vagy tapadásvesztéskor ez a változás sem
ígér pályatartást.
